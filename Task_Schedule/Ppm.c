/**
 * @file Ppm.c
 * @brief 航模 PPM 编码输出(TIM4_CH1 = PB6): 输出比较翻转 + 每个沿在中断里排下一次比较
 *
 * @note 为什么 16 位定时器就够(核心):
 *  - TIM4 时基 1us(PSC=71 = 72MHz/72), ARR=0xFFFF 自由运行;
 *  - 通道用 TIM_OCMODE_TOGGLE, PB6 的电平翻转由硬件比较器完成(没有软件抖动);
 *  - 中断里不做翻转, 只做一件事: CCR1 += "下一条沿到本沿的间隔". 存的是**相对间隔**而不是
 *    绝对时间戳, 且每个间隔恒 < 65536, 而 CNT/CCR 是同一个 16 位模, 所以 CCR1 跨过
 *    65535->0 回绕时依然落在正确的未来时刻 -> 不丢沿; 帧长也不受 16 位限制, 它是表里
 *    多条间隔拼出来的. 单次间隔上限 65535us = 65.535ms, 而 PPM 里最长的一段是帧同步
 *    低电平(约 20ms 以内), 余量充足.
 *
 * @note 帧结构(正逻辑: 脉宽固定, 间隔携带信息):
 *    每通道 [高 PPM_PULSE_US] + [低 (通道脉宽 - PPM_PULSE_US)];
 *    最后一通道的低电平再并上"帧同步低电平" = PPM_FRAME_US - Σ(通道路时间).
 *    一帧固定 2*PPM_CH 个沿, 所以间隔表长度是固定的, 通道值变化只改数值不改长度.
 *
 * @note 分工(与 Crsf.c 一致): 中断只"排下一个沿"(帧首重算一次表, 几十条指令);
 *       ch_out[] -> 脉宽的换算放在 PPM_Task(5ms 调度槽). 中断里不做换算/打印/阻塞调用.
 *
 * @note 配置在哪(本文件不做任何定时器配置):
 *  - 时基/模式/引脚都在 ZOE_TX_V1.2.ioc 里, 由 CubeMX 生成到 tim.c 的 MX_TIM4_Init():
 *    CH1 = Output Compare Toggle CH1(CCMR1.OC1M = 3), PSC = 71, ARR = 65535, PB6 = AF 推挽;
 *  - TIM4 的 NVIC(NVIC.TIM4_IRQn, 优先级 1)和中断入口 TIM4_IRQHandler 也在 .ioc 里,
 *    由 CubeMX 生成到 stm32f1xx_it.c, 本文件不再手写任何中断入口;
 *  - PPM 第一个沿的时点(CCR1 = PPM_PULSE_US)在 tim.c 的 TIM4_Init 2 USER CODE 段;
 *  - OC 模式不会开 CCR 预装载(OC1PE = 0), 所以中断里写 CCR1 立即生效 —— 唯一依赖的硬件细节;
 *  - 本文件只有三件事: 沿表(PPM_BuildFrame) + 中断排下一个沿(PPM_Isr) + 启动(PPM_Init).
 *
 * @note 硬件: TIM4 在 APB1 上, 靠 APB1 分频器的 x2 补偿才得到 72MHz(见 SystemClock_Config:
 *       APB1CLKDivider = HCLK/2). 以后改时钟树/主频, PSC=71 就不再是 1us, 要在 .ioc 里改.
 */
#include "Ppm.h"
#include "tim.h"
#include "Func.h"     /* ch_out[] / crsf_min / crsf_mid / crsf_max */
#include "main.h"     /* Error_Handler() */



/* 下发给硬件的通道脉宽(us): PPM_Task 写, 中断帧首读.
   16 位对齐读写在 Cortex-M3 上就是一条指令(天然原子); 就算某帧取到"半新半旧"的值,
   对 PPM 也无害 —— 接收机下一帧就会拿到完整的新值.
   静态初值 = 中位: PPM_Init 之后到 PPM_Task 第一次跑(≤5ms)之间中断已经在出帧了,
   有初值就不会让接收机看到"整帧挤在下限 1000us"的第一帧 */
volatile uint16_t ppm_ch_us[PPM_CH] = {[0 ... PPM_CH - 1] = PPM_MID_US};

/* 沿间隔表(us): [PULSE, 间隔0, PULSE, 间隔1, ..., PULSE, 间隔N-1 + 帧同步低电平],
   只在中断里读写(帧首重算), 所以不加 volatile */
static uint16_t ppm_delta[2U * PPM_CH];
static uint8_t  ppm_edge_i;         /* 下一个要排的沿(0 = 帧首, 该沿是第 0 通道的上升沿) */

/* 诊断计数(在中断/5ms 槽里更新, 目前没有读者; TIM4 的 NVIC 由 .ioc 的 NVIC.TIM4_IRQn 生成) */
volatile uint16_t ppm_edge_cnt;     /* 沿总数 */
volatile uint16_t ppm_frame_cnt;    /* 帧数 */
volatile uint16_t ppm_frame_rate;   /* 实测帧率(帧/秒) */
static uint16_t   ppm_rate_last;    /* 上一次算帧率时的 ppm_frame_cnt */
static uint32_t   ppm_rate_stamp;   /* 上一次算帧率的时刻(HAL_GetTick) */

/* ========================== 工具 ========================== */

/**
 * @brief CRSF 通道值(172~1811, 中位 992) -> PPM 脉宽(1000~2000us, 中位 1500us)
 * @note 用 32 位中间量: (1811-172)*1000 = 1639000 不会溢出
 */
static uint16_t PPM_CrsfToUs(uint16_t ch)
{
  int32_t v = (int32_t)ch - (int32_t)crsf_min;
  if (v < 0)
    v = 0;
  else if (v > (int32_t)(crsf_max - crsf_min))
    v = (int32_t)(crsf_max - crsf_min);

  return (uint16_t)(PPM_MIN_US +
                    (uint32_t)v * (PPM_MAX_US - PPM_MIN_US) / (crsf_max - crsf_min));
}

/* ========================== 沿表 ========================== */

/**
 * @brief 按当前 ppm_ch_us[] 重算整帧的沿间隔(在中断的帧首调用)
 * @note 只做加法和赋值, 几十条指令, 远小于第一个脉冲宽度(PPM_PULSE_US), 不会拖长中断
 */
static void PPM_BuildFrame(void)
{
  uint32_t sum = 0U;

  for (uint8_t i = 0U; i < PPM_CH; i++)
  {
    uint16_t us = ppm_ch_us[i];
    if (us < PPM_MIN_US) us = PPM_MIN_US;
    if (us > PPM_MAX_US) us = PPM_MAX_US;

    ppm_delta[2U * i]        = PPM_PULSE_US;                  /* 上升沿 -> 下降沿: 固定脉宽 */
    ppm_delta[2U * i + 1U]   = (uint16_t)(us - PPM_PULSE_US); /* 下降沿 -> 下一通道上升沿 */
    sum += us;
  }

  /* 帧同步低电平 = 帧周期 - 所有通道路时间. 通道整体偏大时相减可能不够, 就钳到下限
     (此时帧会比 PPM_FRAME_US 略长, 接收机对帧长是宽容的), 但绝不允许出现负间隔 */
  uint32_t sync = (uint32_t)PPM_FRAME_US - sum;
  if (sync < PPM_SYNC_MIN_US)
    sync = PPM_SYNC_MIN_US;
  ppm_delta[2U * PPM_CH - 1U] = (uint16_t)(ppm_delta[2U * PPM_CH - 1U] + sync);
}

/**
 * @brief 每个沿一次(TIM4 CC1 中断): 帧首重算沿表, 然后排下一个沿的时刻
 * @note 中断体 = 一次加法 + 一次 16 位寄存器写 + 索引维护, 约 1us; 最短间隔 600us, 余量巨大
 */
static void PPM_Isr(void)
{
  if (ppm_edge_i == 0U)      /* 本沿是帧首(第 0 通道的上升沿): 取最新通道值重算整帧 */
  {
    PPM_BuildFrame();
    ppm_frame_cnt++;
  }

  /* 下一个沿的时刻 = 当前 CCR1 + 间隔. 这里是"相对累加": 16 位寄存器在 65536 处回绕时,
     结果依然落在正确的未来时刻(CNT 与 CCR 同模循环), 所以帧长可以远超 65.5ms.
     写 CCR1 必须"立即生效": CH1 是 OC Toggle 模式, CubeMX 生成的配置没有开 CCR 预装载
     (CCMR1.OC1PE = 0), 所以写进去立刻参与硬件比较 */
  htim4.Instance->CCR1 += ppm_delta[ppm_edge_i];
  ppm_edge_cnt++;

  if (++ppm_edge_i >= (2U * PPM_CH))
    ppm_edge_i = 0U;
}

/* ========================== 对外接口 ========================== */

/**
 * @brief 直接指定某通道的脉宽(us): 越界自动限幅; 超出 PPM_CH 的通道号忽略
 */
void PPM_SetChannel(uint8_t ch, uint16_t us)
{
  if (ch >= PPM_CH)
    return;
  if (us < PPM_MIN_US) us = PPM_MIN_US;
  if (us > PPM_MAX_US) us = PPM_MAX_US;
  ppm_ch_us[ch] = us;
}

/**
 * @brief 5ms 调度槽: 把混控后的最终通道值(ch_out[])换算成 PPM 脉宽, 并统计实测帧率
 */
void PPM_Task(void)
{
  for (uint8_t i = 0U; i < PPM_CH; i++)
    ppm_ch_us[i] = PPM_CrsfToUs(ch_out[i]);

  /* 帧率统计(诊断用): 16 位计数相减天然处理回绕 */
  uint32_t now = HAL_GetTick();
  if ((uint32_t)(now - ppm_rate_stamp) >= 1000U)
  {
    ppm_frame_rate = (uint16_t)(ppm_frame_cnt - ppm_rate_last);
    ppm_rate_last  = ppm_frame_cnt;
    ppm_rate_stamp = now;
  }
}

/**
 * @brief 启动 PPM 输出 —— 本函数只有一句: 启动 TIM4 的 CH1
 * @note 必须在 MX_TIM4_Init()(main.c)之后调用. 时基(1us)/OC Toggle 模式/PB6 复用/
 *       第一个沿的时点(CCR1 = PPM_PULSE_US)全部由 CubeMX 生成的代码配好(见文件头"配置在哪"),
 *       所以这里既不写任何配置寄存器, 也不碰 CNT/CCR1: HAL 里没有"启动定时器"的生成项,
 *       这一句只能留在用户代码里.
 */
void PPM_Init(void)
{
  /* 使能 CC1 中断 + 通道输出(CC1E, PB6 交给定时器驱动) + 计数器.
     之后每个 CC1 事件会同时回调 HAL_TIM_OC_DelayElapsedCallback 和
     HAL_TIM_PWM_PulseFinishedCallback(两个都是 weak 函数), 只在其中一个里排下一个沿:
     两个都实现会让 CCR1 一次加两遍, 波形全错 */
  if (HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1) != HAL_OK)
    Error_Handler();
}

/* ========================== HAL 回调(覆盖 HAL weak 定义) ========================== */

/**
 * @brief TIM4 比较匹配中断(每个沿一次): 排下一个沿的时刻
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if ((htim->Instance == TIM4) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
    PPM_Isr();
}
