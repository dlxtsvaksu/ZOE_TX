/**
 * @file Crsf.c
 * @brief CRSF 协议支持(手台 -> ELRS 模块), 250Hz 遥控帧
 *
 * @note 分工(关键): 中断只做"节拍"和"启动 DMA", 组帧/解析都在调度器任务里
 *  - TIM1 更新中断(4ms, CRSF_TxPulse): 把已组好的帧交给 UART TX DMA, 只做赋值 + 启动 DMA
 *  - CRSF_TxTask(1ms): 组 0x16 遥控帧(16 通道 x 11bit) + CRC8-DVB-S2
 *  - CRSF_RxTask(1ms): 从 RX DMA 环形缓冲取字节, 解析 0x14 链路统计 / 回 0x28 ping
 *  - HAL_UART_TxCpltCallback: 只清忙标志
 *  中断里绝不能组帧/算 CRC/调用阻塞式发送
 *
 * @note 硬件: USART2(PA2=TX, PA3=RX), 400000 8N1
 *       TX = DMA1_Channel7(NORMAL), RX = DMA1_Channel6(CIRCULAR)
 *       (见 Elrs_tx.ioc: Dma.USART2_TX.1 / Dma.USART2_RX.4;
 *        DMA1_Channel4/5 是 OLED 的 I2C2_TX/RX, 不是本文件用的)
 */
#include "Crsf.h"
#include "usart.h"
#include "tim.h"
#include "Func.h"
#include <string.h>

/* ========================== 可调宏 ========================== */

/* CRC 覆盖范围: 1 = 覆盖 addr+len+type+payload; 0 = 只覆盖 type+payload.
   CRSF/ELRS/EdgeTX 的实际约定是 0(从 type 字节算起), 所以默认 0;
   自动探测仍会在收到模块下行帧后按实测校正(见 CRSF_RxTask) */
#define CRSF_CRC_WITH_ADDR_LEN 0
#if CRSF_CRC_WITH_ADDR_LEN
#define CRSF_CRC_START 0U
#else
#define CRSF_CRC_START 2U
#endif

#define CRSF_FRAME_MAX   64U    /* 单帧最大长度(含 addr/len/crc) */
#define CRSF_TX_SLOTS    2U     /* 发送乒乓缓冲 */
#define CRSF_TX_NONE     0xFFU
#define CRSF_RX_RING     128U   /* RX 环形缓冲(2 的幂, DMA 循环写) */

/* ========================== 协议约定(与模块一致, 已按官方实现锁定) ==========================
   官方 ELRS 模块的 CRSFHandset::handleInput() 对手台帧只有两种合法首字节:
     alignBufferToSync() 把 0xEE(模块地址)或 0xC8(CRSF_SYNC_BYTE)当同步头, 其它字节一律丢弃并计入 BadPkts;
     收到完整帧后 UART 看门狗判"手台还在不在": Bad >= Good 或 !controllerConnected 就断开 ->
     重扫波特率(noCrossfire), 期间模块完全不发射 RF(现象: 接收机永远连不上, LED 慢闪等 TX,
     模块本身也从"回遥测"变成完全闭嘴).
   所以本机发出去的帧首字节只能是 0xEE/0xC8, CRC 必须从 type 起算(模块自己就是
   crsf_crc.calc(&buf[2], totalLen-3)). 这两点写死, 不再做"A/B 试探":
   试探用的非法帧会把模块看门狗喂炸, 模块静默, 结果是"越试越连不上".
   诊断页第1页 A/C/M 用来确认: 应为 A:C8 C:2 M:2(2 = 模块回帧的 CRC 从 type 起算) */
#define CRSF_USE_FIXED_CONVENTION 1     /* 1 = 用下面两个宏写死的约定(正式用法); 0 = 保留自动探测, 仅排查时用 */
#define CRSF_FIXED_ADDR           0xC8U /* 发送帧首字节: 0xC8 = CRSF_SYNC_BYTE(EdgeTX 手台用的就是它) */
#define CRSF_FIXED_CRC_FROM       2U    /* CRC 起始偏移: 2 = 只覆盖 type+payload(CRSF 实际约定) */

/* 自动探测的节奏(仅在 CRSF_USE_FIXED_CONVENTION == 0 时生效) */
#define CRSF_PROBE_STEP_MS    1000U    /* 每种组合试 1 秒 */
#define CRSF_PROBE_RETRY_MS  30000U    /* 一轮试完仍不成功: 30 秒后再来一轮(模块可能晚于本机上电) */
#define CRSF_PROBE_COMBOS        6U    /* 组合总数(地址 3 种 x CRC 覆盖 2 种) */

/* ========================== 发送缓冲与状态 ========================== */
static uint8_t  tx_frame[CRSF_TX_SLOTS][CRSF_FRAME_MAX];
static uint8_t  tx_len[CRSF_TX_SLOTS];
static volatile uint8_t tx_pending = CRSF_TX_NONE; /* 已组好待发(ISR 消费) */
static volatile uint8_t tx_sending = CRSF_TX_NONE; /* 正在 DMA 发送的槽 */
static volatile uint8_t tx_last    = CRSF_TX_NONE; /* 最近启动过的槽(节拍兜底时重发) */
static volatile uint8_t tx_busy    = 0U;           /* 1 = DMA 发送中 */
static volatile uint8_t tx_tick    = 0U;           /* TIM1 节拍到(通知任务组帧) */

volatile uint16_t crsf_tx_cnt       = 0;
volatile uint16_t crsf_tx_err       = 0;
volatile uint16_t crsf_rx_cnt       = 0;
volatile uint16_t crsf_rx_crc_err   = 0;
volatile uint16_t crsf_rx_other_cnt = 0;
volatile uint16_t crsf_tim_tick_cnt = 0;    /* TIM1 中断次数: 不涨 = 节拍没进来 */
volatile uint16_t crsf_tx_skip_cnt = 0;     /* 因上一帧未发完而跳过的节拍数 */
volatile uint16_t crsf_tx_rate     = 0;     /* 实测: 帧/秒 */
volatile uint16_t crsf_tick_rate   = 0;     /* 实测: 节拍/秒 */
volatile uint8_t  crsf_nvic_ok      = 0;    /* NVIC 自检结果 */

/* 每秒速率统计用的静态量 */
static uint16_t rate_tx_last   = 0U;
static uint16_t rate_tick_last = 0U;
static uint32_t rate_stamp     = 0U;
volatile CRSF_LinkStats_t crsf_link_stats;

/* 链路新鲜度: 每收到一帧有效 0x14 就刷新时刻, CRSF_RxTask(1ms) 里判超时;
   主界面据此实时显示"连接中/已连接", 断流时把 valid 与 LQ 打回 0(不锁住旧值) */
volatile uint32_t crsf_link_last_ms = 0U;
volatile uint8_t  crsf_link_ok      = 0U;

/* 接收机电池电压(遥测 0x08): 和链路新鲜度同一套做法 —— 每收到一帧就刷新时刻,
   RxTask(1ms) 判超时后把 ok 清 0, 主界面就显示回 "--"(不锁住旧电压).
   原始计数的单位见 Crsf.h 的 CRSF_RXBAT_RAW_PER_V(0.01V 一档: 410 = 4.10V) */
volatile uint16_t crsf_rx_bat_raw     = 0U;
volatile uint32_t crsf_rx_bat_last_ms = 0U;
volatile uint8_t  crsf_rx_bat_ok      = 0U;

/* ===== 协议约定(CRSF_USE_FIXED_CONVENTION = 1 时为固定值, 不再被探测改写) ===== */
volatile uint8_t crsf_cur_addr     = CRSF_ADDR_HANDSET;  /* 发送帧首字节(固定约定下 = 0xC8) */
volatile uint8_t crsf_cur_crc_from = CRSF_CRC_START;
volatile uint8_t crsf_probe_locked  = 0U;
volatile uint8_t crsf_probe_stopped = 0U;
volatile uint8_t crsf_rx_crc_from   = 0xFFU;  /* 模块回帧用哪种 CRC 约定通过(0/2), 0xFF = 还没成功过 */
volatile uint8_t crsf_rx_addr       = 0U;     /* 最近一条合法下行帧的首字节(诊断用: 应为 0xC8/0xEE) */

#if !CRSF_USE_FIXED_CONVENTION
static const uint8_t probe_addr_tab[3] = { CRSF_ADDR_HANDSET, CRSF_ADDR_RADIO_TX, CRSF_ADDR_TX_MODULE };
static const uint8_t probe_crc_tab[2]  = { 0U, 2U };
static uint8_t  probe_idx   = 0U;
static uint8_t  probe_wait  = 0U;    /* 1 = 处于"轮次间隔"等待 */
static uint32_t probe_stamp = 0U;
#endif

/* ========================== 接收缓冲与状态机 ========================== */
static uint8_t  rx_ring[CRSF_RX_RING];
static uint16_t rx_rd    = 0U;             /* 软件读指针(任务独占) */
static uint8_t  rx_state = 0U;             /* 0=等地址 1=等长度 2=收内容 */
static uint8_t  rx_buf[CRSF_FRAME_MAX];
static uint8_t  rx_len   = 0U, rx_pos = 0U;

/* ========================== CRC8-DVB-S2 ========================== */

/**
 * @brief CRC8/DVB-S2 单字节计算(poly 0xD5, 初值 0, 不反射)
 */
static uint8_t CRSF_Crc8(uint8_t crc, uint8_t data)
{
  crc ^= data;
  for (uint8_t i = 0; i < 8U; i++)
    crc = (uint8_t)((crc & 0x80U) ? ((crc << 1) ^ 0xD5U) : (crc << 1));
  return crc;
}

/**
 * @brief 计算 [from, len) 区间的 CRC
 */
static uint8_t CRSF_CrcRange(const uint8_t *buf, uint8_t len, uint8_t from)
{
  uint8_t crc = 0U;
  for (uint8_t i = from; i < len; i++)
    crc = CRSF_Crc8(crc, buf[i]);
  return crc;
}

/**
 * @brief CRC8/Betaflight "额外 CRC" 单字节计算(poly 0xBA, 初值 0, 不反射)
 * @note 只用在命令帧(0x32)的内层 CRC 上: EdgeTX 的 crc8_BA() / Betaflight 的 extra CRC
 *       就是这个算法, 这里用逐位实现, 与 EdgeTX 的 crc8tab_BA[] 表完全等价
 */
static uint8_t CRSF_Crc8Ba(uint8_t crc, uint8_t data)
{
  crc ^= data;
  for (uint8_t i = 0; i < 8U; i++)
    crc = (uint8_t)((crc & 0x80U) ? ((crc << 1) ^ 0xBAU) : (crc << 1));
  return crc;
}

/* ========================== 组帧 ========================== */

/**
 * @brief 组 0x16 遥控帧(共 26 字节): 16 通道 x 11bit, 通道值域 172~1811
 * @param slot 目标发送槽(由 CRSF_AcquireSlot 取得, 保证不在发送/待发中)
 */
static void CRSF_BuildRcFrame(uint8_t slot)
{
  uint16_t ch[16];
  uint8_t *f = tx_frame[slot];
  uint32_t bits = 0U;
  uint8_t  nbits = 0U, *p = &f[3];

  for (uint8_t i = 0U; i < 16U; i++)
    ch[i] = ch_out[i];         /* 最终值: 基础值 + 混控, 已限幅 */

  f[0] = crsf_cur_addr;              /* 探测时会被改成 0xEA / 0xEE 试 */
  f[1] = 24U;                  /* len = type + payload + crc */
  f[2] = CRSF_TYPE_RC_CHANNELS;

  for (uint8_t i = 0U; i < 16U; i++)
  {
    bits |= (uint32_t)(ch[i] & 0x7FFU) << nbits;   /* 11bit, LSB first */
    nbits += 11U;
    while (nbits >= 8U)
    {
      *p++ = (uint8_t)(bits & 0xFFU);
      bits >>= 8;
      nbits -= 8U;
    }
  }

  f[25] = CRSF_CrcRange(f, 25U, crsf_cur_crc_from);
  tx_len[slot] = 26U;
}

/**
 * @brief 取一个空闲发送槽(既不在发送也不在待发), 无空闲返回 CRSF_TX_NONE
 * @note 临界区只有几条指令, 防止与 TIM1 中断抢同一个槽
 */
static uint8_t CRSF_AcquireSlot(void)
{
  uint8_t  slot = CRSF_TX_NONE;
  uint32_t pm = __get_PRIMASK();

  __disable_irq();
  if (tx_sending != 0U && tx_pending != 0U)
    slot = 0U;
  else if (tx_sending != 1U && tx_pending != 1U)
    slot = 1U;
  __set_PRIMASK(pm);

  return slot;
}

/**
 * @brief 把组好的槽发布为"待发", 由 TIM1 中断在下一个节拍发出
 */
static void CRSF_PublishSlot(uint8_t slot)
{
  uint32_t pm = __get_PRIMASK();

  __disable_irq();
  tx_pending = slot;                 /* 单字节发布, 保证中断看到的是完整值 */
  __set_PRIMASK(pm);
}

/**
 * @brief TIM1 更新中断(4ms)调用: 启动一帧 DMA, 保证 250Hz 节拍
 * @note 中断里不组帧、不算 CRC; 若任务尚未组出新帧, 则重发上一帧, 保证节拍不丢
 */
void CRSF_TxPulse(void)
{
  uint8_t slot;

  crsf_tim_tick_cnt++;                /* 节拍自检: 中断没进来时这个数不涨 */
  tx_tick = 1U;                       /* 通知任务: 该组新帧了 */

  if (tx_busy)                        /* 上一帧还没发完 */
  {
    /* 自愈: 若 HAL 那边已经空闲(gState=READY)而我们的标志还卡着, 说明完成回调丢了 */
    if (huart2.gState == HAL_UART_STATE_READY)
    {
      tx_busy    = 0U;
      tx_sending = CRSF_TX_NONE;
    }
    else
    {
      crsf_tx_skip_cnt++;             /* 真正因为上一帧还在发而跳过 */
      return;
    }
  }

  slot = tx_pending;
  if (slot != CRSF_TX_NONE)
  {
    tx_pending = CRSF_TX_NONE;        /* 取走新帧 */
  }
  else
  {
    slot = tx_last;                   /* 没有新帧(任务被拖延): 重发上一帧 */
    if (slot == CRSF_TX_NONE)
      return;                         /* 还没有任何帧(理论上不会发生) */
  }

  tx_sending = slot;
  tx_busy = 1U;

  if (HAL_UART_Transmit_DMA(&huart2, tx_frame[slot], tx_len[slot]) != HAL_OK)
  {
    tx_busy = 0U;
    tx_sending = CRSF_TX_NONE;
    crsf_tx_err++;
    return;
  }

  tx_last = slot;
  crsf_tx_cnt++;
}

/**
 * @brief 发送完成(USART2 的 TC 中断 -> HAL_UART_TxCpltCallback)
 */
void CRSF_TxCplt(void)
{
  tx_sending = CRSF_TX_NONE;
  tx_busy = 0U;
}

/**
 * @brief 协议约定探测(每秒推进一次): 依次试"地址 x CRC 覆盖范围"的 6 种组合,
 *        直到摸到模块接受的组合
 * @note 收到模块的合法帧 -> 锁定; 收到过字节但 CRC 不过 -> 说明接收通路正常, 停止探测
 */
static void CRSF_Probe(void)
{
#if CRSF_USE_FIXED_CONVENTION
  return;                                 /* 固定约定模式: 不做任何探测 */
#else
  uint32_t period;

  if (crsf_probe_locked || crsf_probe_stopped)
    return;

  period = probe_wait ? CRSF_PROBE_RETRY_MS : CRSF_PROBE_STEP_MS;
  if ((uint32_t)(HAL_GetTick() - probe_stamp) < period)
    return;
  probe_stamp = HAL_GetTick();
  probe_wait  = 0U;

  if (crsf_rx_cnt != 0U)                  /* 模块回帧了: 锁定当前组合 */
  {
    crsf_probe_locked = 1U;
    return;
  }
  if (crsf_rx_crc_err != 0U)              /* 收到过字节说明接收通路正常: 不再乱试 */
  {
    crsf_probe_stopped = 1U;
    return;
  }

  if (probe_idx >= CRSF_PROBE_COMBOS)     /* 一轮试完: 等 30 秒再试一轮 */
  {
    probe_idx    = 0U;
    probe_wait   = 1U;
    probe_stamp  = HAL_GetTick();
    return;
  }

  crsf_cur_addr     = probe_addr_tab[probe_idx % 3U];
  crsf_cur_crc_from = probe_crc_tab[(probe_idx / 3U) % 2U];
  probe_idx++;
  tx_tick = 1U;                           /* 立刻用新组合重发一帧 */
#endif
}

/**
 * @brief 每秒统计一次实测频率: 一眼看出"是节拍慢(被中断淹没)"还是"发送被跳过"
 */
static void CRSF_UpdateRates(void)
{
  if ((uint32_t)(HAL_GetTick() - rate_stamp) < 1000U)
    return;
  rate_stamp     = HAL_GetTick();
  crsf_tx_rate   = (uint16_t)(crsf_tx_cnt       - rate_tx_last);
  crsf_tick_rate = (uint16_t)(crsf_tim_tick_cnt - rate_tick_last);
  rate_tx_last   = crsf_tx_cnt;
  rate_tick_last = crsf_tim_tick_cnt;
}

/**
 * @brief CRSF 发送任务(1ms): 每个 TIM1 节拍组一帧新数据
 */
void CRSF_TxTask(void)
{
  uint8_t slot;

  CRSF_UpdateRates();                     /* 每秒更新一次速率显示值 */
  CRSF_Probe();                           /* 每秒推进一次协议约定探测 */

  if (!tx_tick)
    return;
  tx_tick = 0U;

  slot = CRSF_AcquireSlot();
  if (slot == CRSF_TX_NONE)
    return;                           /* 两个槽都在用: 本周期跳过, 节拍由中断重发兜底 */

  CRSF_BuildRcFrame(slot);            /* 组帧 + CRC(不关中断, 只写自己占用的槽) */

  CRSF_PublishSlot(slot);
}

static void CRSF_HandleFrame(void);
static void CRSF_ParamHandleFrame(void);   /* 参数协议下行帧处理(定义在文件末尾) */
static void CRSF_SendFrameEx(uint8_t addr, uint8_t type, const uint8_t *payload, uint8_t len);
static void CRSF_OnDevicePing(void);
static void CRSF_ParseParamEntry(void);   /* 0x2B 元数据/值解析(定义在文件末尾) */
static void CRSF_ParamReq(uint8_t type, uint8_t field, uint8_t chunk);   /* 0x2B/0x2C 参数请求 */
static void CRSF_SendDeviceInfo(void);

/**
 * @brief 地址字节是否属于合法的 CRSF 帧首(用于字节流同步)
 */
static uint8_t CRSF_IsValidAddr(uint8_t b)
{
  return (uint8_t)((b == CRSF_ADDR_BROADCAST) || (b == CRSF_ADDR_HANDSET) ||
                   (b == CRSF_ADDR_TX_MODULE) || (b == CRSF_ADDR_RADIO_TX) ||
                   (b == CRSF_ADDR_RX));
}

/**
 * @brief 逐字节状态机: 地址 -> 长度 -> 内容 -> 处理
 */
static void CRSF_ParseByte(uint8_t b)
{
  switch (rx_state)
  {
    case 0U:                                /* 等地址 */
      if (CRSF_IsValidAddr(b))
      {
        rx_buf[0] = b;
        rx_state = 1U;
      }
      break;

    case 1U:                                /* 长度: 合法范围 2~62 */
      if (b >= 2U && b <= 62U)
      {
        rx_buf[1] = b;
        rx_len = b;
        rx_pos = 2U;
        rx_state = 2U;
      }
      else
      {
        rx_state = 0U;                      /* 长度非法: 重新找帧头 */
      }
      break;

    case 2U:                                /* 帧内容(含 CRC) */
      rx_buf[rx_pos++] = b;
      if (rx_pos >= (uint8_t)(rx_len + 2U))
      {
        CRSF_HandleFrame();
        rx_state = 0U;
      }
      break;

    default:
      rx_state = 0U;
      break;
  }
}

/**
 * @brief CRSF 接收任务(1ms): 用 DMA 剩余计数器算读指针, 把新字节喂给状态机
 * @note 不用串口中断逐字节收: RX DMA 循环搬运 + 任务侧解析, 中断零负担
 */
void CRSF_RxTask(void)
{
  uint16_t pos = (uint16_t)(CRSF_RX_RING - __HAL_DMA_GET_COUNTER(huart2.hdmarx));

  if (pos >= CRSF_RX_RING)
    pos = 0U;                           /* 防御: DMA 异常/刚好写满整环(CNDTR=0)时不要把指针算飞 */

  while (rx_rd != pos)
  {
    uint8_t b = rx_ring[rx_rd];
    rx_rd = (uint16_t)((rx_rd + 1U) & (CRSF_RX_RING - 1U));
    CRSF_ParseByte(b);
  }

  /* 链路新鲜度(1ms 判一次): 0x14 断流(模块没电/还没对上接收机/接插件松)就把状态打回"连接中".
     valid 清 0 -> 主界面切回 CONNECTING; LQ 归零 -> 不会停在断流前的最后一帧(以前看着像 LQ 被锁住).
     RSSI/SNR 保留最后一次的值, 配合诊断页的 V 位能看出"这是断流前的数据" */
  if ((crsf_link_ok != 0U) && ((HAL_GetTick() - crsf_link_last_ms) >= CRSF_LINK_TIMEOUT_MS))
  {
    crsf_link_ok                = 0U;
    crsf_link_stats.valid       = 0U;
    crsf_link_stats.uplink_lq   = 0U;
    crsf_link_stats.downlink_lq = 0U;
  }

  /* 接收机电池(遥测 0x08)新鲜度: 用更长的超时(遥测里 RxBt 本来就发得慢),
     超时只清 ok 不清电压 —— 诊断页还能看到最后一次回传值 */
  if ((crsf_rx_bat_ok != 0U) && ((HAL_GetTick() - crsf_rx_bat_last_ms) >= CRSF_RXBAT_TIMEOUT_MS))
    crsf_rx_bat_ok = 0U;
}

/**
 * @brief 校验一条完整帧并分派(0x14 链路统计 / 0x28 设备 ping ...)
 */
static void CRSF_HandleFrame(void)
{
  uint8_t total    = (uint8_t)(rx_len + 2U);          /* addr + len + ... */
  uint8_t payload_len = (uint8_t)(rx_len - 2U);       /* 去掉 type 和 crc */
  uint8_t crc_recv = rx_buf[total - 1U];
  uint8_t crc_from = CRSF_CRC_START;
  uint8_t crc_calc = CRSF_CrcRange(rx_buf, (uint8_t)(total - 1U), CRSF_CRC_START);

  if (crc_calc != crc_recv)
  {
    /* 兼容另一种 CRC 约定(只覆盖 type + payload), 首次对接时自动适配 */
    crc_from = 2U;
    crc_calc = CRSF_CrcRange(rx_buf, (uint8_t)(total - 1U), 2U);
    if (crc_calc != crc_recv)
    {
      crsf_rx_crc_err++;
      return;
    }
  }
  crsf_rx_crc_from = crc_from;
  crsf_rx_addr     = rx_buf[0];     /* 模块下行帧的首字节(诊断页显示): 0xC8/0xEE 才说明模块在按 CRSF 说话 */

#if !CRSF_USE_FIXED_CONVENTION
  /* 仅排查用: 跟模块的 CRC 约定保持一致.
     正式固件(CRSF_USE_FIXED_CONVENTION = 1)里这段必须关掉 —— 模块的 UART 看门狗会统计我们的坏包,
     一旦把约定改成模块不认的写法, 就变成"每帧都是坏包" -> 模块断开重扫波特率(noCrossfire), 完全不发射 */
  if (crsf_cur_crc_from != crc_from)
  {
    crsf_cur_crc_from  = crc_from;
    crsf_probe_stopped = 1U;          /* 已由实测确定, 不再让探测改它 */
  }
#endif

  crsf_rx_cnt++;

  switch (rx_buf[2])
  {
    case CRSF_TYPE_LINK_STATS:          /* 0x14 链路统计 */
      if (payload_len >= 7U)
      {
        crsf_link_stats.uplink_rssi_dbm = (int8_t)rx_buf[3];   /* payload[0] */
        crsf_link_stats.uplink_lq       = rx_buf[5];           /* payload[2] */
        crsf_link_stats.uplink_snr      = (int8_t)rx_buf[6];   /* payload[3] */
        crsf_link_stats.uplink_tx_power = rx_buf[9];           /* payload[6] */
        if (payload_len >= 11U)
        {
          crsf_link_stats.downlink_rssi_dbm = (int8_t)rx_buf[10]; /* payload[7] */
          crsf_link_stats.downlink_lq       = rx_buf[12];         /* payload[9] */
          crsf_link_stats.downlink_snr      = (int8_t)rx_buf[13]; /* payload[10] */
        }
        crsf_link_stats.valid = 1U;
        crsf_link_last_ms     = HAL_GetTick();   /* 刷新新鲜度: 主界面据此显示"已连接" */
        crsf_link_ok          = 1U;
      }
      break;

    case CRSF_TYPE_BATTERY:             /* 0x08 接收机电池传感器: 主界面中间显示 "RX xx.xV" */
      if (payload_len >= 2U)
      {
        /* 电压 = payload[0..1] 大端, 单位 0.01V 一档(见 Crsf.h 的 CRSF_RXBAT_RAW_PER_V):
           4.10V -> 0x019A = 410; 乘 10 得 4100mV -> 主界面 "4.1V".
           后面的电流/容量/剩余量本机暂时不用 */
        crsf_rx_bat_raw     = (uint16_t)(((uint16_t)rx_buf[3] << 8) | (uint16_t)rx_buf[4]);
        crsf_rx_bat_last_ms = HAL_GetTick();
        crsf_rx_bat_ok      = 1U;
      }
      CRSF_ParamHandleFrame();          /* 仍存进原始缓冲: 参数页/诊断页照旧能看到这帧 */
      break;

    case CRSF_TYPE_DEVICE_PING:         /* 0x28 模块 ping: 需要尽早回 0x29 */
      CRSF_OnDevicePing();            /* 抓模板 + 回复设备信息 */
      break;

    default:
    case CRSF_TYPE_DEVICE_INFO:
    case CRSF_TYPE_PARAM_SETTINGS_ENTRY:
    case CRSF_TYPE_PARAM_READ:
    case CRSF_TYPE_PARAM_WRITE:
      CRSF_ParamHandleFrame();
      break;

      crsf_rx_other_cnt++;
      break;
  }
}

/**
 * @brief 回 0x29 设备信息(模块 ping 0x28 时回复), 12 字节 payload
 * @note Phase3 做参数协议时把 hw/fw/字段数填成真实值, 现在填 0 = "无参数字段"
 */
static void CRSF_SendDeviceInfo(void)
{
  uint8_t slot = CRSF_AcquireSlot();
  uint8_t dest = rx_buf[0];           /* 回到发 ping 的那个地址 */
  uint8_t *f;

  if (slot == CRSF_TX_NONE)
    return;                           /* 没有空闲槽, 下一帧 ping 再回(模块会重发) */

  f = tx_frame[slot];
  f[0]  = crsf_cur_addr;
  f[1]  = 14U;                        /* len = type + 12 字节 payload + crc */
  f[2]  = CRSF_TYPE_DEVICE_INFO;
  f[3]  = dest;                       /* 目标地址 */
  f[4]  = crsf_cur_addr;              /* 源地址 */
  f[5]  = 0x01U;                      /* 序列号(4 字节, 可自定义) */
  f[6]  = 0x00U;
  f[7]  = 0x00U;
  f[8]  = 0x00U;
  f[9]  = 0x00U;                      /* 硬件 ID */
  f[10] = 0x00U;                      /* 固件 ID */
  f[11] = 0x00U;                      /* 固件版本 */
  f[12] = 0x00U;                      /* 字段数: 0 = 无参数字段 */
  f[13] = 0x00U;                      /* 参数版本 */
  f[14] = 0x00U;                      /* 参数个数 */
  f[15] = CRSF_CrcRange(f, 15U, crsf_cur_crc_from);
  tx_len[slot] = 16U;

  CRSF_PublishSlot(slot);
}

/**
 * @brief 读 NVIC 使能位: 判断某个中断是否真的开起来了(自检用, 免得分不清"代码没写"和"写了没生效")
 */
static uint8_t CRSF_IrqEnabled(IRQn_Type irqn)
{
  uint32_t n = (uint32_t)irqn;
  return (uint8_t)((NVIC->ISER[n >> 5U] & (1UL << (n & 0x1FU))) != 0UL);
}

/**
 * @brief CRSF 初始化: 准备发送缓冲(先备一帧中位帧) + 启动 RX 循环 DMA
 */
void CRSF_Init(void)
{
  rx_state   = 0U;
  rx_rd      = 0U;
  tx_pending = CRSF_TX_NONE;
  tx_sending = CRSF_TX_NONE;
  tx_busy    = 0U;
  tx_tick    = 0U;
  crsf_link_stats.valid = 0U;
  crsf_link_ok          = 0U;      /* 链路状态从"连接中"开始 */
  crsf_link_last_ms     = 0U;
  crsf_rx_bat_ok        = 0U;      /* 接收机电池: 从"没有数据(--)"开始 */
  crsf_rx_bat_raw       = 0U;
  crsf_rx_bat_last_ms   = 0U;

  /* 协议约定(固定值, 见文件头的宏说明; 不做 A/B 试探) */
  crsf_rx_addr       = 0U;   /* 还没收到过模块的合法帧 */
#if CRSF_USE_FIXED_CONVENTION
  crsf_cur_addr      = CRSF_FIXED_ADDR;
  crsf_cur_crc_from  = CRSF_FIXED_CRC_FROM;
  crsf_probe_locked  = 1U;              /* 视为已锁定 */
  crsf_probe_stopped = 0U;
  crsf_rx_crc_from   = 0xFFU;
#else
  crsf_cur_addr      = CRSF_ADDR_HANDSET;
  crsf_cur_crc_from  = CRSF_CRC_START;
  crsf_probe_locked  = 0U;
  crsf_probe_stopped = 0U;
  crsf_rx_crc_from   = 0xFFU;
  probe_idx          = 0U;
  probe_wait         = 0U;
  probe_stamp        = HAL_GetTick();
#endif

  /* 速率统计复位 */
  crsf_tx_skip_cnt = 0U;
  crsf_tx_rate     = 0U;
  crsf_tick_rate   = 0U;
  rate_tx_last     = crsf_tx_cnt;
  rate_tick_last   = crsf_tim_tick_cnt;
  rate_stamp       = HAL_GetTick();

  /* NVIC 自检: "TX 一直是 0" 最常见的原因就是中断没使能, 这里把它记下来供诊断页显示
     位序(对应诊断页 "I:" 后 4 个数字的顺序, 见 Schedule.c 第1页): bit0=TIM1_UP(4ms 节拍)
       bit1=USART2  bit2=DMA1_Channel7(UART TX)  bit3=DMA1_Channel6(UART RX) —— 通道号照 Elrs_tx.ioc */
  crsf_nvic_ok  = 0U;
  crsf_nvic_ok |= (uint8_t)(CRSF_IrqEnabled(TIM1_UP_IRQn)       ? 0x01U : 0U);
  crsf_nvic_ok |= (uint8_t)(CRSF_IrqEnabled(USART2_IRQn)        ? 0x02U : 0U);
  crsf_nvic_ok |= (uint8_t)(CRSF_IrqEnabled(DMA1_Channel7_IRQn) ? 0x04U : 0U);
  crsf_nvic_ok |= (uint8_t)(CRSF_IrqEnabled(DMA1_Channel6_IRQn) ? 0x08U : 0U);

  /* 先备一帧(中位/当前摇杆值), 让 TIM1 的第一个节拍就有东西可发(节拍兜底用 tx_last) */
  CRSF_BuildRcFrame(0U);
  tx_last = 0U;

  /* 启动接收: 128 字节环形缓冲 + 循环 DMA(DMA1_Channel6, 见 ioc Dma.USART2_RX.4) */
  HAL_UART_Receive_DMA(&huart2, rx_ring, CRSF_RX_RING);

  /* 命令帧自检: 组出的对频帧与金标字节比对, 结果上诊断页 "S:"(1 = 一致) */
  CRSF_CmdSelfTest();
}

/* ========================== HAL 回调(本文件实现, 覆盖 HAL 的 weak 定义) ========================== */

/**
 * @brief 串口发送完成(DMA 完成后由 USART2 的 TC 中断触发): 只清忙标志
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
    CRSF_TxCplt();
}

/**
 * @brief 串口错误(帧错/噪声/溢出/校验): HAL 出错时会终止接收, 这里把它重新挂上
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    if (huart->RxState != HAL_UART_STATE_BUSY_RX)
      HAL_UART_Receive_DMA(&huart2, rx_ring, CRSF_RX_RING);
  }
}

/**
 * @brief 定时器更新中断: TIM1(4ms = 250Hz) 提供 CRSF 发送节拍
 * @note 回调体只有几个赋值 + 一次非阻塞 DMA 启动, 不会拉长中断
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
    CRSF_TxPulse();
}




/* ========================== 一次性命令帧(0x32) ==========================
   子命令字节集中在这里: 若模块对某条命令没反应, 只改这几行即可, 其它代码不用动 */
#define CRSF_SUB_RX_1       0x10U   /* CRSF_COMMAND_SUBCMD_RX */
#define CRSF_SUB_RX_BIND_2  0x01U   /* ..._RX_BIND: 对频 */
#define CRSF_SUB_GEN_1      0x0AU   /* CRSF_COMMAND_SUBCMD_GENERAL */
#define CRSF_SUB_BLE_2      0x01U   /* 蓝牙模拟器摇杆(待实测校正) */
#define CRSF_SUB_POWER_2    0x03U   /* 设置发射功率(待实测校正) */

/* 命令帧的地址约定(与 EdgeTX/Betaflight 实测一致, 帧格式说明见 Crsf.h):
   sync = 0xC8 (EdgeTX 命令帧首字节 = UART_SYNC, 模块不校验)
   dest = 0xEE (模块地址; ELRS 路由只把 0x32 帧投递给 0xEE 或 0x00 的端点)
   orig = 0xEA (手台地址) */
#define CRSF_CMD_SYNC       0xC8U
#define CRSF_CMD_DEST       0xEEU
#define CRSF_CMD_ORIG       0xEAU

volatile uint8_t crsf_cmd_two_crc = 1U;   /* 1 = 官方双 CRC 写法(默认), 0 = 只带外层 CRC */
volatile uint8_t crsf_cmd_ok      = 0U;   /* 开机自检: 1 = 组出的对频帧与金标字节一致 */

/**
 * @brief 组一条命令帧(0x32, 带扩展头):
 *        [sync][len][0x32][dest][orig][sub1][sub2][data...][crcBA?][crc]
 * @param f        输出缓冲(至少 16 字节)
 * @param sync     帧首字节(0xC8)
 * @param dest     目的地址(0xEE 模块 / 0x00 广播)
 * @param data/len 可选附加数据(最多 4 字节, 可为 NULL)
 * @param two_crc  1 = 内层 poly 0xBA 的 CRC 也算进 payload(EdgeTX 官方写法)
 * @return 整帧字节数(含 sync/len/crc)
 * @note CRC 一律从 type 字节(下标 2)开始算 —— CRSF 只有这一个约定, 不受自动探测影响.
 *       外层 CRC(poly 0xD5)覆盖到内层 CRC 为止, 是模块唯一校验的那一字节
 */
static uint8_t CRSF_MakeCommandFrame(uint8_t *f, uint8_t sync, uint8_t dest,
                                     uint8_t sub1, uint8_t sub2,
                                     const uint8_t *data, uint8_t len, uint8_t two_crc)
{
  uint8_t n = 7U;                  /* 0=sync 1=len 2=type 3=dest 4=orig 5=sub1 6=sub2 */
  uint8_t crc_ba = 0U;
  uint8_t i;

  if (len > 4U)
    len = 4U;

  f[0] = sync;
  f[2] = CRSF_TYPE_COMMAND;
  f[3] = dest;
  f[4] = CRSF_CMD_ORIG;
  f[5] = sub1;
  f[6] = sub2;
  for (i = 0U; i < len; i++)
    f[n++] = data[i];

  for (i = 2U; i < n; i++)                 /* 内层 CRC 覆盖 type..最后一个 payload 字节 */
    crc_ba = CRSF_Crc8Ba(crc_ba, f[i]);
  if (two_crc != 0U)
    f[n++] = crc_ba;                       /* 官方写法: 内层 CRC 作为 payload 的一部分 */

  f[n] = CRSF_CrcRange(f, n, 2U);          /* 外层 CRC(模块唯一校验的一字节) */
  f[1] = (uint8_t)(n - 1U);                /* len = type..末 CRC 的字节数 */
  return (uint8_t)(n + 1U);
}

/**
 * @brief 组好一条命令帧并交给发送槽(下一个 4ms 节拍由 TIM1 中断发出)
 */
static void CRSF_SendCommandFrame(uint8_t dest, uint8_t sub1, uint8_t sub2,
                                  const uint8_t *data, uint8_t len)
{
  uint8_t slot = CRSF_AcquireSlot();

  if (slot == CRSF_TX_NONE)
    return;                                /* 没空槽就丢这一条, UI 会重发 */

  tx_len[slot] = CRSF_MakeCommandFrame(tx_frame[slot], CRSF_CMD_SYNC, dest,
                                       sub1, sub2, data, len, crsf_cmd_two_crc);
  CRSF_PublishSlot(slot);
}

/**
 * @brief 切换命令帧的 CRC 形式(1 = 双 CRC 官方写法, 0 = 单 CRC), 供 UI 做 A/B 测试
 */
void CRSF_SetCmdCrcForm(uint8_t two_crc)
{
  crsf_cmd_two_crc = (two_crc != 0U) ? 1U : 0U;
}

/**
 * @brief 命令帧自检: 与金标字节逐字节比对, 结果放 crsf_cmd_ok(诊断页 "S:")
 * @note 金标 = C8 07 32 EE EA 10 01 14 EB(按 EdgeTX 双 CRC 写法算出的对频帧)
 */
void CRSF_CmdSelfTest(void)
{
  static const uint8_t gold[9] = { 0xC8U, 0x07U, 0x32U, 0xEEU, 0xEAU, 0x10U, 0x01U, 0x14U, 0xEBU };
  uint8_t b[16];
  uint8_t n;
  uint8_t i;

  n = CRSF_MakeCommandFrame(b, CRSF_CMD_SYNC, CRSF_CMD_DEST,
                            CRSF_SUB_RX_1, CRSF_SUB_RX_BIND_2, NULL, 0U, 1U);
  crsf_cmd_ok = 0U;
  if (n == (uint8_t)sizeof(gold))
  {
    crsf_cmd_ok = 1U;
    for (i = 0U; i < n; i++)
    {
      if (b[i] != gold[i])
      {
        crsf_cmd_ok = 0U;
        break;
      }
    }
  }
}

/**
 * @brief 发一条命令帧(0x32): 目的地址用模块地址 0xEE
 * @param sub1 子命令类别, sub2 子命令, data/len 可选附加数据(最多 4 字节)
 * @note 与 RC 帧共用发送槽, 由 UI 每个界面周期调用一次实现重发
 */
void CRSF_SendCommand(uint8_t sub1, uint8_t sub2, const uint8_t *data, uint8_t len)
{
  CRSF_SendCommandFrame(CRSF_CMD_DEST, sub1, sub2, data, len);
}





/**
 * @brief 设置发射功率档位(0~5, 对应 25/50/100/250/500/1000mW; 子命令待实测校正)
 */

/**
 * @brief 发一条命令帧(0x32, 可指定目的地址): 0xEE = 模块, 0x00 = 广播
 * @param dest 目的地址(命令帧扩展头里真正决定投递的字段; 帧首字节固定 0xC8)
 * @note 与 RC 帧共用发送槽, 由 UI 逐个变体试发
 */
void CRSF_SendCommandTo(uint8_t dest, uint8_t sub1, uint8_t sub2, const uint8_t *data, uint8_t len)
{
  CRSF_SendCommandFrame(dest, sub1, sub2, data, len);
}
/* ========================== CRSF 参数协议(0x28/0x29/0x2B/0x2C/0x2D) ==========================
   用途: 枚举/读写模块参数(ELRS 的 Bind / Max Power / BLE 摇杆等都是模块参数表里的字段).
   各版本实现细节略有差异, 所以这里把收到的原始帧按十六进制存下来(参数页可见),
   对着实测数据校正解析, 而不是靠猜 */
#define CRSF_PT_BUF 48U

volatile uint8_t  crsf_pt_type        = 0U;    /* 最近收到的参数协议帧类型(0=无) */
volatile uint8_t  crsf_pt_len         = 0U;    /* 该帧总字节数(截断到缓冲大小) */
volatile uint16_t crsf_pt_cnt         = 0U;    /* 参数协议帧计数 */
volatile uint8_t  crsf_pt_buf[CRSF_PT_BUF];
volatile uint8_t  crsf_dev_online     = 0U;    /* 收到过 0x29 */
volatile uint8_t  crsf_dev_field_cnt  = 0U;    /* 0x29: 模块报的参数字段数 */
volatile uint8_t  crsf_dev_param_ver  = 0U;    /* 0x29: 参数版本 */

/* 注: 原来的 CRSF_SendFrame 已由 CRSF_SendFrameEx(可指定地址)取代 */

/**
 * @brief 0x28 设备 ping(广播): 让模块回 0x29 设备信息(含参数字段数)
 */
void CRSF_ReqPing(void)
{
  static const uint8_t p[2] = { 0x00U, 0x00U };   /* dest=广播, origin=广播 */
  CRSF_SendFrameEx(crsf_param_addr != 0U ? crsf_param_addr : crsf_cur_addr, CRSF_TYPE_DEVICE_PING, p, 2U);
}

/**
 * @brief 0x2B 请求字段元数据(名字/类型/选项): field=字段号, chunk=分片号
 */
void CRSF_ReqParamEntry(uint8_t field, uint8_t chunk)
{
  CRSF_ParamReq(CRSF_TYPE_PARAM_SETTINGS_ENTRY, field, chunk);
}

/**
 * @brief 0x2C 读字段当前值
 */
void CRSF_ReqParamRead(uint8_t field, uint8_t chunk)
{
  CRSF_ParamReq(CRSF_TYPE_PARAM_READ, field, chunk);
}

/**
 * @brief 0x2D 写字段(值按模块字段类型给出)
 */
void CRSF_ParamWrite(uint8_t field, const uint8_t *val, uint8_t len)
{
  uint8_t p[8];

  if (len > 5U)
    len = 5U;
  /* 参数帧是 CRSF 扩展帧, 必须带 dest/origin:
       [addr][len][0x2D][dest=0xEE][origin=0xEA][字段号][值...][crc]
     少这两个字节模块会当路由头解析 -> 目的地址变成"字段号", 整帧被丢弃(毫无反应).
     ELRS 的 CRSFEndpoint::parameterUpdateReq() 里 payload[0] 才是字段号, 值从 payload[1] 起 */
  p[0] = CRSF_CMD_DEST;
  p[1] = CRSF_CMD_ORIG;
  p[2] = field;
  for (uint8_t i = 0U; i < len; i++)
    p[3U + i] = val[i];

  CRSF_SendFrameEx(CRSF_CMD_SYNC, CRSF_TYPE_PARAM_WRITE, p, (uint8_t)(3U + len));
}

/**
 * @brief 参数协议下行帧处理: 存原始字节 + 解析 0x29 设备信息
 * @note 原始字节供参数页/诊断页十六进制显示
 */
static void CRSF_ParamHandleFrame(void)
{
  uint8_t total = (uint8_t)(rx_len + 2U);
  uint8_t n = (total > CRSF_PT_BUF) ? CRSF_PT_BUF : total;

  for (uint8_t i = 0U; i < n; i++)
    crsf_pt_buf[i] = rx_buf[i];
  crsf_pt_type = rx_buf[2];
  crsf_pt_len  = n;
  crsf_pt_cnt++;

  if (rx_buf[2] == CRSF_TYPE_DEVICE_INFO)
  {
    /* [0..4] 帧头, [5..] 设备名\0 + serial(4) + 硬件版本(1) + 固件版本(4) + 参数字段数(1) + 参数版本(1)
       (见 CRSFEndpoint::sendDeviceInformationPacket) —— 设备名长度可变, 所以字段数位置得先扫到名字的 NUL */
    uint8_t n = 5U;

    while ((n < (uint8_t)(total - 1U)) && (rx_buf[n] != 0U))
      n++;
    n = (uint8_t)(n + 10U);                 /* 跳过名字的 NUL + serial(4) + hw(1) + fw(4) */
    if ((uint8_t)(n + 1U) < (uint8_t)(total - 1U))
    {
      crsf_dev_field_cnt = rx_buf[n];
      crsf_dev_param_ver = rx_buf[n + 1U];
      crsf_dev_online    = 1U;              /* 帧结构对上了才算"模块上线" */
    }
  }
  else if (rx_buf[2] == CRSF_TYPE_PARAM_SETTINGS_ENTRY)
  {
    CRSF_ParseParamEntry();          /* 字段元数据: 名字/类型/选项/当前值 */
  }
}
/* ========================== 0x28 ping 变体扫描 + 参数帧地址 ==========================
   不同实现对"发给模块的设备 ping / 参数帧"在地址字节与 payload 目标上略有差异:
   这里逐个变体试(调用者每 2s 调一次 CRSF_PingTick), 一旦收到 0x29 就锁定,
   并把当时生效的地址记入 crsf_param_addr, 之后 0x2B/0x2C/0x2D 都用它 */
#define CRSF_PING_VARIANTS 5U

volatile uint8_t crsf_param_addr  = 0U;     /* 0 = 用 crsf_cur_addr; 否则用扫描到的模块地址 */
volatile uint8_t crsf_ping_idx    = 0U;
volatile uint8_t crsf_ping_locked = 0U;

static const uint8_t ping_addr_tab[CRSF_PING_VARIANTS] = { 0xEEU, 0xEEU, 0xEAU, 0xEAU, 0xC8U };
static const uint8_t ping_dest_tab[CRSF_PING_VARIANTS] = { 0x00U, 0xEEU, 0x00U, 0xEAU, 0x00U };

/**
 * @brief 发一帧指定地址/类型的帧(参数协议请求用)
 */
static void CRSF_SendFrameEx(uint8_t addr, uint8_t type, const uint8_t *payload, uint8_t len)
{
  uint8_t slot = CRSF_AcquireSlot();
  uint8_t *f;
  uint8_t n = 3U;

  if (slot == CRSF_TX_NONE)
    return;
  if (len > 16U)
    len = 16U;

  f = tx_frame[slot];
  f[0] = addr;
  f[2] = type;
  for (uint8_t i = 0U; i < len; i++)
    f[n++] = payload[i];

  f[1] = (uint8_t)(n - 1U);
  f[n] = CRSF_CrcRange(f, n, crsf_cur_crc_from);
  tx_len[slot] = (uint8_t)(n + 1U);

  CRSF_PublishSlot(slot);
}

/**
 * @brief 设备 ping 扫描: 每调用一次发一个变体(建议每 2s 调一次)
 * @note 收到 0x29 后自动停止切换, 并保留生效的地址供参数请求使用
 */
void CRSF_PingTick(void)
{
  uint8_t p[2];
  uint8_t i = crsf_ping_idx;

  if (crsf_dev_online)                     /* 模块已应答: 锁定 */
  {
    crsf_ping_locked = 1U;
    return;
  }
  if (crsf_ping_locked)
    return;

  crsf_param_addr = ping_addr_tab[i];      /* 试这个地址(成功后保留) */
  p[0] = ping_dest_tab[i];                 /* payload 目标字段 */
  p[1] = 0xC8U;                            /* origin: 本机地址 */

  CRSF_SendFrameEx(crsf_param_addr, CRSF_TYPE_DEVICE_PING, p, 2U);

  crsf_ping_idx = (uint8_t)((i + 1U) % CRSF_PING_VARIANTS);
}
/* ========================== 模块自己发的 0x28 ping: 抓下来当模板 ==========================
   模块周期性发 0x28 给我们, 它的字节布局就是"最可靠的参照":
   参数页会显示 RX28: 前 5 字节(地址/长度/类型/payload), 便于对齐我们自己的 ping */
volatile uint8_t  crsf_ping_buf[24];
volatile uint8_t  crsf_ping_len = 0U;
volatile uint16_t crsf_ping_cnt = 0U;

static void CRSF_OnDevicePing(void)
{
  uint8_t total = (uint8_t)(rx_len + 2U);
  uint8_t n = (total > 24U) ? 24U : total;

  for (uint8_t i = 0U; i < n; i++)
    crsf_ping_buf[i] = rx_buf[i];
  crsf_ping_len = n;
  crsf_ping_cnt++;

  CRSF_SendDeviceInfo();            /* 仍然要回复 0x29 */
}
/**
 * @brief 强制设置发送帧的 CRC 覆盖约定(0=含 addr+len, 2=只含 type+payload)
 * @note 供 UI 做 A/B 测试; 同时停掉自动探测, 避免被改回去
 */
void CRSF_SetCrcFrom(uint8_t from)
{
  crsf_cur_crc_from  = (from == 0U) ? 0U : 2U;
  crsf_probe_stopped = 1U;
}
/**
 * @brief 强制设置 TX 帧的地址与 CRC 约定(排查 A/B 用): 同时停掉自动探测
 * @note 地址/CRC 组合共 6 种, UI 里按一下换一种, 边看接收机边试
 */
void CRSF_SetTxConvention(uint8_t addr, uint8_t crc_from)
{
  crsf_cur_addr      = addr;
  crsf_cur_crc_from  = (crc_from == 0U) ? 0U : 2U;
  crsf_probe_stopped = 1U;
}
/* ========================== 模块参数表(枚举/浏览/读写) ==========================
   0x2B = 字段元数据(名字/类型/选项/当前值), 0x2C = 读值, 0x2D = 写值
   ELRS 对 0x2C 也是用 0x2B 那种帧回(CRSFEndpoint::parameterUpdateReq 的 READ 分支),
   所以下面 0x2B 的解析同时负责取"当前值".
   帧布局(第 0 片, 见 CRSFEndpoint::sendParameter):
     [3]dest [4]origin [5]字段号 [6]剩余分片 [7]父级 [8]类型 [9]名字\0 [数据段]
     类型 9(SELECTION): 选项串\0 值 最小 最大 默认 单位\0
     类型 0/1(UINT8/INT8): 值 最小 最大 0 单位\0     类型 2/3: 值2 最小2 最大2 0 单位\0
     类型 13(COMMAND): step 200 状态串\0            其它(STRING/INFO/FOLDER): 只有名字 */
#define CRSF_PARAM_MAX   40U   /* 字段表槽位数(按字段号索引): ELRS 发射模块的参数能到 30 多个,
                                  留 40 保证整张表都扫得到(每槽约 81B RAM) */
#define CRSF_PARAM_NAME  18U
#define CRSF_PARAM_VAL    8U
#define CRSF_PARAM_META  48U   /* 选项串(以 ';' 分隔)的存储上限: ELRS 2.4G 的 Max Power
                                  选项串 "10;25;50;100;250;500;1000;2000;MatchTX " 有 39 字符
                                  (+结尾 NUL = 40), 24 装不下; 双频(LR1121/2.4G+900M)机型的
                                  选项串会长到 64 字符, 这里会被截断 —— 但只影响"后几档的文字",
                                  档位个数由 vmax(读自数据区, 与选项串无关)决定, 选档写入用的
                                  是下标, 所以截断不会让功率切不动 */

typedef struct {
  uint8_t id;
  uint8_t type;
  uint8_t valid;
  uint8_t has_value;
  uint8_t name_len;
  uint8_t meta_len;
  uint8_t vmin;                 /* 值下界(SELECTION/UINT8 里紧跟名字后的第 2 个值字节) */
  uint8_t vmax;                 /* 值上界(SELECTION 里 = 最后一个选项的下标, 用来回卷) */
  uint8_t name[CRSF_PARAM_NAME];
  uint8_t meta[CRSF_PARAM_META];
  uint8_t value[CRSF_PARAM_VAL];
  uint8_t value_len;
} CRSF_Param_t;

/* 字段类型(CRSF 规范/ExpressLRS crsf_value_type_e: 0..1 单字节整数, 2..3 双字节整数,
   9 选项, 10 字符串, 11 文件夹, 12 信息, 13 命令) */
#define CRSF_PTYPE_UINT8      0U
#define CRSF_PTYPE_INT8       1U
#define CRSF_PTYPE_UINT16     2U
#define CRSF_PTYPE_INT16      3U
#define CRSF_PTYPE_SELECTION  9U
#define CRSF_PTYPE_STRING    10U
#define CRSF_PTYPE_FOLDER    11U   /* 文件夹: 只有名字没有值 —— 按名字找字段时必须跳过它 */
#define CRSF_PTYPE_INFO      12U
#define CRSF_PTYPE_COMMAND   13U

static CRSF_Param_t param_tab[CRSF_PARAM_MAX];
volatile uint8_t crsf_param_sel   = 0U;    /* UI 当前浏览的字段号 */
volatile uint8_t crsf_param_scan  = 0U;    /* 1 = 正在扫描 */
volatile uint8_t crsf_param_ready = 0U;    /* 1 = 已解析到至少一个字段 */

static uint8_t  scan_id = 0U;
static uint32_t scan_at = 0U;

/* ========================== 字段数据分片拼装 ==========================
   模块把一条字段的"线性数据"(源码里的 chunkBuffer)按 chunkMax 切开再逐片发,
   chunkMax = getConnectorMaxPacketSize(手台) - 6 - 2
   (见 ExpressLRS CRSFEndpoint::sendParameter):
     chunkBuffer: [0][1] 留给片头(字段号/剩余片数), [2]=父级, [3]=类型, [4..]=名字\0 数据区...
     第 0 片: [5]字段 [6]剩余片数 [7]父级 [8]类型 [9..]名字\0 数据   <- 数据区从 [7] 起
     第 k 片: [5]字段 [6]剩余片数 [7..]数据区续段                   <- 片头会盖掉上一片最后
                                                                     2 字节(已发过, 无损)
   也就是: 数据区(上面 [7..] 那段) 第 0 片从偏移 0 起, 第 k 片从偏移 k*chunkMax 起.
   数据区布局(见 textSelectionParameterToArray):
     父级 类型 名字\0 [选项串\0 值 最小 最大 默认 单位\0]
   一条字段有多长? 以 2.4G 的 "Max Power" 为例(见 TXModuleParameters.cpp 的 luaPower):
     2(父级/类型) + 10("Max Power"\0) + 31(选项串 "10;25;50;100;250;500;1000;2000"\0)
     + 4(值/最小/最大/默认) + 3(单位 "mW"\0) = 50 字节.
   chunkMax 是模块按"手台报给它的最大包长"算的: 64-8=56 时这条正好 1 片; 但只要它缩到 49 以下
   (手台慢/收发窗变窄时 ELRS 会自己调小 maxPacketBytes), 同一条就会被切成 2 片; 带 "MatchTX "
   的机型(选项串 40 字节 -> 数据区 59 字节)必然 2 片.
   以前只认第 0 片(而且选项串扫描上界还写错成 24), 于是 值/最小/最大 全从选项串中段取 ->
   写出的档位号模块不认, 功率永远切不动(表现就是"没有功率表").
   续片由 CRSF_ParamTick 用 0x2C 主动要回来; 主机端回归测试见 host/sim/(test_param.c) */
#define CRSF_PARAM_SEG 96U     /* 一条字段线性数据的拼装上限(超了截断, 前面的选项/值仍可用) */

static uint8_t  pseg[CRSF_PARAM_SEG];
static uint8_t  pseg_len    = 0U;    /* 已拼字节数 */
static uint8_t  pseg_field  = 0xFFU; /* 正在拼的字段号 */
static uint8_t  pseg_chunks = 0U;    /* 该字段共几片(第 0 片里读出来) */
static uint8_t  pseg_next   = 0U;    /* 下一片要请求的片号(0 = 没有待续片) */
static uint32_t pseg_at     = 0U;    /* 上一次收到片段/发出续片请求的时刻(超时放弃用) */
#define CRSF_PARAM_SEG_MS 500U       /* 续片 500ms 还不来就丢掉这条, 别把扫描卡死 */

static void CRSF_ParseParamSeg(void);   /* 拼齐后解析(实现在下面) */

/**
 * @brief 0x2B/0x2C 字段帧: 把本片数据段拼进 pseg, 末片拼齐后再解析
 */
static void CRSF_ParseParamEntry(void)
{
  uint8_t total = (uint8_t)(rx_len + 2U);
  uint8_t field = rx_buf[5];
  uint8_t start = 5U;
  uint8_t n, k;

  if (total < 9U)
    return;                               /* 最短合法字段帧 = 8 字节头尾 + 1 字节数据 */
  if (field >= CRSF_PARAM_MAX)
    return;                               /* 字段号越界: 丢掉, 别写坏表 */

  /* 续片判定: 就是我们要回来的那一片(字段号与剩余片数都对得上) */
  if ((pseg_next != 0U) && (field == pseg_field) &&
      (rx_buf[6] == (uint8_t)(pseg_chunks - 1U - pseg_next)))
  {
    start = 7U;                           /* 续片: 开头 2 字节是片头, 数据从 [7] 起 */
  }
  else
  {
    pseg_field  = field;                  /* 新的一段: 第 0 片(含片头/父级/类型/名字) */
    pseg_len    = 0U;
    pseg_chunks = (uint8_t)(rx_buf[6] + 1U);
    pseg_next   = 0U;
  }

  n = (uint8_t)((total - 1U) - start);    /* 本片数据段长度(不含 CRC) */
  for (k = 0U; k < n; k++)
  {
    if (pseg_len < CRSF_PARAM_SEG)
      pseg[pseg_len++] = rx_buf[start + k];
  }

  if (rx_buf[6] != 0U)
  {
    pseg_next = (uint8_t)((start == 7U) ? (pseg_next + 1U) : 1U);   /* 还有续片: 下一拍去要 */
    pseg_at   = HAL_GetTick();
    return;
  }
  CRSF_ParseParamSeg();                   /* 末片: 拼齐了, 解析 */
}

/**
 * @brief 解析拼好的线性流: [0]字段 [1]剩余 [2]父级 [3]类型 [4..]名字\0 数据...
 */
static void CRSF_ParseParamSeg(void)
{
  uint8_t nstart = 4U;                    /* 名字在拼装流里的起点 */
  uint8_t nlen   = 0U;
  uint8_t pid, ptype, i;
  CRSF_Param_t *p;

  pseg_next = 0U;                         /* 末片已到(本函数只由末片调用): 续片请求就此结束.
                                             不清的话下面会当成"还有续片", 每解析完一条还要白等 500ms 超时 */
  if (pseg_len < 6U)
  {
    pseg_len = 0U;
    return;
  }
  pid   = pseg[0];
  ptype = (uint8_t)(pseg[3] & 0x7FU);     /* 高 bit = 隐藏字段 */
  if (pid >= CRSF_PARAM_MAX)
  {
    pseg_len = 0U;
    return;                               /* 字段号越界: 丢掉 */
  }
  while (((uint8_t)(nstart + nlen) < pseg_len) && (nlen < (CRSF_PARAM_NAME - 1U)) &&
         (pseg[nstart + nlen] >= 0x20U) && (pseg[nstart + nlen] < 0x7FU))
    nlen++;
  if (nlen == 0U)
  {
    pseg_len = 0U;
    return;                               /* 名字列不是可打印串(不是 0x2B 字段帧): 丢掉 */
  }

  p = &param_tab[pid];
  p->id        = pid;
  p->type      = ptype;
  p->valid     = 1U;
  p->has_value = 0U;
  p->vmin      = 0U;
  p->vmax      = 0U;
  p->meta_len  = 0U;
  for (i = 0U; i < nlen; i++)
    p->name[i] = pseg[nstart + i];
  p->name[nlen] = 0U;
  p->name_len   = nlen;

  i = (uint8_t)(nstart + nlen + 1U);      /* 名字的 NUL 之后 = 该类型自己的数据段 */

  if (ptype == CRSF_PTYPE_SELECTION)
  {
    /* 选项串在前("10;25;50;100;250;500;1000;2000;MatchTX "), 后面才是 值/最小/最大.
       扫描上界必须用整段真实长度 pseg_len, 不能用存储上限(以前写成 CRSF_PARAM_META-1 = 23):
       功率串 41 字符会被"扫描"截断, 而下面的 值/最小/最大 又从这个截断点接着取 ->
       取到串中段的 ASCII 码当档位号写出去, 模块不认, 功率永远切不动 */
    uint8_t mlen = 0U;
    uint8_t keep;
    while (((uint8_t)(i + mlen) < pseg_len) && (pseg[i + mlen] >= 0x20U) && (pseg[i + mlen] < 0x7FU))
      mlen++;
    keep = (mlen < (CRSF_PARAM_META - 1U)) ? mlen : (CRSF_PARAM_META - 1U);
    for (uint8_t k = 0U; k < keep; k++)
      p->meta[k] = pseg[i + k];
    p->meta[keep] = 0U;
    p->meta_len   = keep;
    i = (uint8_t)(i + mlen + 1U);         /* 跳过选项串的 NUL(用真实长度, 不是存储长度) */
  }

  if ((ptype == CRSF_PTYPE_SELECTION) || (ptype <= CRSF_PTYPE_INT8))
  {
    /* 单字节三段: [当前值][最小][最大] (ELRS 的 int8Parameter.properties = {value,min,max}) */
    if ((uint8_t)(i + 2U) < pseg_len)
    {
      p->value[0]  = pseg[i];
      p->value_len = 1U;
      p->vmin      = pseg[i + 1U];
      p->vmax      = pseg[i + 2U];        /* SELECTION: 选项下标上界 = 选项个数 - 1 */
      p->has_value = 1U;
    }
  }
  else if ((ptype == CRSF_PTYPE_UINT16) || (ptype == CRSF_PTYPE_INT16))
  {
    /* 双字节大端: [当前值][最小][最大] 各 2 字节, 原样存(写回时直接用) */
    if ((uint8_t)(i + 5U) < pseg_len)
    {
      p->value[0]  = pseg[i];
      p->value[1]  = pseg[i + 1U];
      p->value_len = 2U;
      p->vmax      = pseg[i + 5U];        /* 只取低 8 位(功率类字段都远小于 256) */
      p->has_value = 1U;
    }
  }

  crsf_param_ready = 1U;
  pseg_len   = 0U;                        /* 这一条解析完: 拼装状态整个清掉 */
  pseg_field = 0xFFU;
}

/* 注: 不再单独解析 0x2C 短帧 —— ELRS 对 0x2C(读) 也是用 0x2B 那种字段帧回的
   (CRSFEndpoint::parameterUpdateReq 的 READ 分支), 值已经在 CRSF_ParseParamEntry 里取出来了.
   0x2C 帧的原始字节仍会存进 crsf_pt_buf, 参数页照旧能看 */

uint8_t CRSF_ParamType(uint8_t id)
{
  return (id < CRSF_PARAM_MAX) ? param_tab[id].type : 0U;
}

uint8_t CRSF_ParamName(uint8_t id, char *dst, uint8_t max)
{
  uint8_t n = 0U;

  if (max == 0U)
    return 0U;
  dst[0] = 0;
  if (id >= CRSF_PARAM_MAX)
    return 0U;
  while ((n < param_tab[id].name_len) && ((uint8_t)(n + 1U) < max))
  {
    dst[n] = (char)param_tab[id].name[n];
    n++;
  }
  dst[n] = 0;
  return n;
}

uint8_t CRSF_ParamValueLen(uint8_t id)
{
  return (id < CRSF_PARAM_MAX) ? param_tab[id].value_len : 0U;
}

uint8_t CRSF_ParamValue(uint8_t id, uint8_t idx)
{
  if ((id >= CRSF_PARAM_MAX) || (idx >= param_tab[id].value_len))
    return 0U;
  return param_tab[id].value[idx];
}

/**
 * @brief 按名字关键字找字段号(不区分大小写), 找不到返回 0xFF
 * @note 查找串要写具体: ELRS 的参数表里 "TX Power" 是文件夹(类型 11, 只有名字没有值),
 *       "Max Power" 才是真正的功率档位字段. 用 "Power" 会先命中文件夹, CRSF_ParamStep
 *       对文件夹返回 0xFF(没有值可写) -> 功率永远不动(表现就是"按 Power 没反应").
 *       所以 UI 里传的是 "Max Power" */
uint8_t CRSF_ParamFind(const char *key)
{
  uint8_t klen = 0U;

  while (key[klen] != (char)0)
    klen++;
  if (klen == 0U)
    return 0xFFU;

  for (uint8_t i = 0U; i < CRSF_PARAM_MAX; i++)
  {
    const CRSF_Param_t *p = &param_tab[i];

    if (!p->valid || (p->name_len < klen))
      continue;
    for (uint8_t s = 0U; (uint8_t)(s + klen) <= p->name_len; s++)
    {
      uint8_t k = 0U;

      while (k < klen)
      {
        char a = (char)p->name[s + k];
        char b = key[k];

        if ((a >= (char)0x41) && (a <= (char)0x5A)) a = (char)(a + 32);
        if ((b >= (char)0x41) && (b <= (char)0x5A)) b = (char)(b + 32);
        if (a != b)
          break;
        k++;
      }
      if (k == klen)
        return p->id;
    }
  }
  return 0xFFU;
}

/**
 * @brief 开始扫描整张参数表
 */
void CRSF_ParamScanStart(void)
{
  for (uint8_t i = 0U; i < CRSF_PARAM_MAX; i++)
  {
    param_tab[i].id = i;
    param_tab[i].valid = 0U;
    param_tab[i].has_value = 0U;
    param_tab[i].name_len = 0U;
    param_tab[i].name[0] = 0U;
    param_tab[i].meta_len = 0U;
    param_tab[i].meta[0] = 0U;
    param_tab[i].value_len = 0U;
    param_tab[i].vmin = 0U;
    param_tab[i].vmax = 0U;
  }
  scan_id = 0U;
  scan_at = 0U;
  pseg_len   = 0U;                       /* 分片拼装状态一起清掉: 免得残留半条流影响解析 */
  pseg_field = 0xFFU;
  pseg_next  = 0U;
  crsf_pt_layout  = 0U;                  /* 请求布局已按规范固定(带 dest/origin), 这个旧变量只留给参数页显示 */
  crsf_param_scan  = 1U;
  crsf_param_ready = 0U;
}

/**
 * @brief 扫描推进(UI 每 5ms 调一次, 内部限制到 60ms/字段)
 * @note 0x2C(读) 就够拿整条字段(ExpressLRS 的 parameterUpdateReq READ 分支会回 0x2B 那种字段帧);
 *       0x2B 当请求发过去模块不认(走 default 分支), 所以这里只发 0x2C, 省一半请求帧
 */
void CRSF_ParamTick(void)
{
  uint8_t fcnt = crsf_dev_field_cnt;

  if ((uint32_t)(HAL_GetTick() - scan_at) < 60U)
    return;
  scan_at = HAL_GetTick();

  /* 有字段正在拼分片: 先把它的下一片要回来(与是否在扫描无关, 优先).
     拼装期间不再发别的请求, 免得两路应答交叉把拼装流搅乱; 500ms 没等到就放弃这条 */
  if (pseg_next != 0U)
  {
    if ((uint32_t)(HAL_GetTick() - pseg_at) < CRSF_PARAM_SEG_MS)
    {
      CRSF_ReqParamRead(pseg_field, pseg_next);
      return;
    }
    pseg_next = 0U;                        /* 续片不来(模块掉线/丢帧): 丢掉半条, 继续扫描 */
    pseg_len  = 0U;
    pseg_field = 0xFFU;
  }
  if (!crsf_param_scan)
    return;
  if (fcnt == 0U)
    fcnt = 16U;                            /* 还没拿到 0x29(字段数未知): 先按 16 个扫 */
  if (fcnt > CRSF_PARAM_MAX)
    fcnt = CRSF_PARAM_MAX;                 /* 表只有这么多槽, 多出来的字段号没地方放 */

  if (scan_id >= fcnt)
  {
    crsf_param_scan = 0U;                  /* 一轮扫完就收工: 模块不支持参数时也不会一直占发送槽 */
    return;
  }

  CRSF_ReqParamRead(scan_id, 0U);
  scan_id++;
}

/**
 * @brief 触发 COMMAND 类字段(写一个 0 即触发; 例如 Bind / BLE 摇杆)
 */
void CRSF_ParamTrigger(uint8_t id)
{
  static const uint8_t zero = 0U;
  CRSF_ParamWrite(id, &zero, 1U);
}

/**
 * @brief 把字段值加上 delta 并写回(0x2D), 返回新值(0xFF = 该字段没有可写的值)
 * @note SELECTION/UINT8/INT8: 值到上界(0x2B 帧里的 max 字节)就从 vmin 重新开始,
 *       这样连按能把所有档位循环一遍; 双字节字段按大端原样回写
 */
uint8_t CRSF_ParamStep(uint8_t id, uint8_t delta)
{
  CRSF_Param_t *p;
  uint8_t v[2];
  uint16_t nv;

  if (id >= CRSF_PARAM_MAX)
    return 0xFFU;
  p = &param_tab[id];
  if ((p->valid == 0U) || (p->has_value == 0U) || (p->vmax == 0U))
    return 0xFFU;                          /* 没扫到/没有值(COMMAND 等)/没有下一档: 都不写 */

  if ((p->type == CRSF_PTYPE_UINT16) || (p->type == CRSF_PTYPE_INT16))
  {
    uint16_t cur = (uint16_t)(((uint16_t)p->value[0] << 8) | (uint16_t)p->value[1]);
    nv = (uint16_t)(cur + delta);
    if ((uint8_t)(nv & 0xFFU) > p->vmax)
      nv = p->vmin;
    v[0] = (uint8_t)(nv >> 8);
    v[1] = (uint8_t)(nv & 0xFFU);
    CRSF_ParamWrite(id, v, 2U);
  }
  else
  {
    nv = (uint16_t)((uint16_t)p->value[0] + delta);
    if ((uint8_t)nv > p->vmax)
      nv = p->vmin;
    v[0] = (uint8_t)nv;
    CRSF_ParamWrite(id, v, 1U);
  }
  return (uint8_t)nv;
}
volatile uint8_t crsf_pt_layout = 0U;    /* 旧 A/B 探测遗留(现恒为 0), 参数页仍显示它 */

/* ========================== 参数请求(0x2B/0x2C) ==========================
   参数帧是 CRSF 扩展帧, 必须带 dest/origin:
     [addr=0xC8][len][type][dest=0xEE][origin=0xEA][字段号][chunk][crc]
   ELRS 的 CRSFEndpoint::parameterUpdateReq() 直接把 payload[0] 当字段号、payload[1] 当值,
   所以这两个字节不能省(少了它, 模块把字段号当目的地址, 整帧路由丢弃 -> 毫无反应). */
static void CRSF_ParamReq(uint8_t type, uint8_t field, uint8_t chunk)
{
  uint8_t p[4];

  p[0] = CRSF_CMD_DEST;
  p[1] = CRSF_CMD_ORIG;
  p[2] = field;
  p[3] = chunk;

  CRSF_SendFrameEx(CRSF_CMD_SYNC, type, p, 4U);
}


/* ========================== 上行遥测帧(手台 -> 模块 -> 接收机/飞控) ========================== */



/**
 * @brief 0x1E 姿态帧: 3 个 int16(大端), 单位 rad*10000
 * @note 目前没有 IMU, 预留接口(接上传感器后每秒调用即可)
 */
void CRSF_SendAttitude(int16_t pitch, int16_t roll, int16_t yaw)
{
  uint8_t p[6];

  p[0] = (uint8_t)((uint16_t)pitch >> 8);
  p[1] = (uint8_t)((uint16_t)pitch & 0xFFU);
  p[2] = (uint8_t)((uint16_t)roll >> 8);
  p[3] = (uint8_t)((uint16_t)roll & 0xFFU);
  p[4] = (uint8_t)((uint16_t)yaw >> 8);
  p[5] = (uint8_t)((uint16_t)yaw & 0xFFU);

  CRSF_SendFrameEx((crsf_param_addr != 0U) ? crsf_param_addr : crsf_cur_addr, CRSF_TYPE_ATTITUDE, p, 6U);
}

