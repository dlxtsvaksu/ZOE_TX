#ifndef __CRSF_H__
#define __CRSF_H__

#include <stdint.h>

/* ========================== CRSF 地址(帧首字节) ========================== */
#define CRSF_ADDR_BROADCAST  0x00
#define CRSF_ADDR_HANDSET    0xC8   /* 手台(本机) -> 模块 */
#define CRSF_ADDR_RADIO_TX   0xEA   /* 手台(模块视角) */
#define CRSF_ADDR_RX         0xEC   /* 接收机 */
#define CRSF_ADDR_TX_MODULE  0xEE   /* 发射模块 */

/* ========================== CRSF 帧类型 ========================== */
#define CRSF_TYPE_GPS         0x02
#define CRSF_TYPE_BATTERY     0x08   /* 电池传感器(接收机回传, 大端, EdgeTX/ELRS 里叫 "RxBt").
                                        电压字段单位见 CRSF_RXBAT_RAW_PER_V(规范注释是 mV*100) */
#define CRSF_TYPE_LINK_STATS  0x14
#define CRSF_TYPE_RC_CHANNELS 0x16
#define CRSF_TYPE_ATTITUDE    0x1E
#define CRSF_TYPE_DEVICE_PING 0x28
#define CRSF_TYPE_DEVICE_INFO 0x29
#define CRSF_TYPE_COMMAND     0x32   /* 一次性命令帧(对频/功率/蓝牙等) */
#define CRSF_TYPE_PARAM_SETTINGS_ENTRY 0x2B   /* 参数枚举/元数据 */
#define CRSF_TYPE_PARAM_READ           0x2C   /* 读参数 */
#define CRSF_TYPE_PARAM_WRITE          0x2D   /* 写参数 */

/* ========================== 链路统计(0x14 解析结果) ========================== */
typedef struct {
    int8_t   uplink_rssi_dbm;    /* 上行 RSSI(dBm, 负值) */
    uint8_t  uplink_lq;          /* 上行链路质量 % */
    int8_t   uplink_snr;         /* 上行 SNR(dB) */
    uint8_t  uplink_tx_power;    /* 上行发射功率索引 */
    int8_t   downlink_rssi_dbm;  /* 下行 RSSI(dBm, 负值) */
    uint8_t  downlink_lq;        /* 下行链路质量 % */
    int8_t   downlink_snr;       /* 下行 SNR(dB) */
    uint8_t  valid;              /* 1 = 已收到过有效 0x14 */
} CRSF_LinkStats_t;

/* ========================== 运行时统计(可上屏观察) ========================== */
extern volatile CRSF_LinkStats_t crsf_link_stats;
extern volatile uint16_t crsf_tx_cnt;       /* 已发出的遥控帧数(250Hz 时每秒 +250) */
extern volatile uint16_t crsf_tx_err;       /* 发送失败/节拍错过次数 */
extern volatile uint16_t crsf_rx_cnt;       /* 解析成功的模块下行帧数 */
extern volatile uint16_t crsf_rx_crc_err;   /* CRC 校验失败帧数 */
extern volatile uint16_t crsf_rx_other_cnt; /* 其他类型帧数 */
extern volatile uint16_t crsf_tim_tick_cnt; /* TIM1 更新中断次数(250Hz 节拍自检) */
extern volatile uint16_t crsf_tx_skip_cnt;  /* 因"上一帧未发完"而跳过的节拍数 */
extern volatile uint16_t crsf_tx_rate;      /* 实测发送频率(帧/秒) */
extern volatile uint16_t crsf_tick_rate;    /* 实测节拍频率(次/秒) */
extern volatile uint8_t  crsf_nvic_ok;      /* NVIC 自检: bit0=TIM1_UP, bit1=USART2, bit2=DMA1_Ch7(UART TX), bit3=DMA1_Ch6(UART RX) */

/* ===== 链路连接状态(主界面实时显示"是否连上"用) ===== */
#define CRSF_LINK_TIMEOUT_MS 1500U           /* 0x14 断流超过这个时间 = 未连接(主界面回到"连接中").
                                                模块 0x14 一般 1~10Hz, 留够余量免得正常时就闪 */
extern volatile uint32_t crsf_link_last_ms;  /* 最近一次收到有效 0x14 的时刻(HAL_GetTick) */
extern volatile uint8_t  crsf_link_ok;       /* 1 = 链路统计新鲜(超时时间内有 0x14); 0 = 连接中/断流 */

/* ===== 接收机电池电压(遥测回传 0x08 电池传感器, 主界面 "RX" 显示用) ===== */
/* 0x08 电压字段的单位 = 每 1V 的原始计数. CRSF/TBS 结构注释写的是 mV*100, 也就是 0.01V 一档:
   4.10V -> 410. OpenTX/EdgeTX 的处理也能反推: 它把该字段 /10 变成 0.1V 单位再按 1 位小数显示.
   若换成按 0.1V 发的模块(4.1V -> 41), 只把这里改成 10U 即可, 其余代码不用动 */
#define CRSF_RXBAT_RAW_PER_V  100U
#define CRSF_RXBAT_TIMEOUT_MS 5000U          /* 超过这个时间没再收到 0x08 = 没有该传感器/断流, 界面回到 "--".
                                                遥测里 RxBt 发得慢(常见 0.5~2s 一次, 取决于遥测比例),
                                                这个值别调太小, 否则显示会一闪一闪 */
extern volatile uint16_t crsf_rx_bat_raw;     /* 最近回传的接收机电压原始计数(0.01V 一档: 410 = 4.10V) */
extern volatile uint32_t crsf_rx_bat_last_ms; /* 最近一次收到的时刻(HAL_GetTick) */
extern volatile uint8_t  crsf_rx_bat_ok;      /* 1 = 超时时间内收到过 0x08 */

/* ===== 协议约定自动探测(排查"模块不认本机帧"用) ===== */
extern volatile uint8_t crsf_cur_addr;      /* 当前发送用地址(0xC8 / 0xEA / 0xEE) */
extern volatile uint8_t crsf_cur_crc_from;  /* 当前 CRC 起始偏移(0=含addr+len, 2=只含type+payload) */
extern volatile uint8_t crsf_probe_locked;  /* 1 = 已找到并锁定模块接受的组合 */
extern volatile uint8_t crsf_probe_stopped; /* 1 = 检测到接收通路正常, 停止探测 */
extern volatile uint8_t crsf_rx_crc_from;   /* 模块帧用哪种约定通过(0/2), 0xFF = 还没成功过 */
extern volatile uint8_t crsf_rx_addr;       /* 最近一条合法下行帧的首字节(诊断: 应为 0xC8/0xEE) */

/* ========================== 接口 ========================== */
void CRSF_Init(void);      /* 初始化缓冲并启动 RX DMA(main 里调用) */
void CRSF_TxTask(void);    /* 1ms 任务: 按 TIM1 节拍组 0x16 遥控帧 */
void CRSF_RxTask(void);    /* 1ms 任务: 解析模块下行帧(0x14/0x28...) */
void CRSF_TxPulse(void);   /* TIM1 更新中断里调用: 只负责启动 DMA */
void CRSF_TxCplt(void);    /* HAL_UART_TxCpltCallback 里调用: 只清忙标志 */

/* ========================== 一次性命令(对频/功率/蓝牙摇杆) ==========================
   命令帧(0x32)的线上格式(必须和官方 ELRS 模块的实现一致):
     [0] 首字节 0xC8  CRSF_SYNC_BYTE —— 模块 CRSFHandset::alignBufferToSync 只认 0xEE/0xC8,
                      其它字节一律丢弃并计入 BadPkts(看门狗 Bad >= Good 就断开重扫波特率)
     [1] len          = 其后字节数(type..末 CRC)
     [2] type   0x32  CRSF_FRAMETYPE_COMMAND
     [3] dest   0xEE  目的地址 = 模块自己(模块把 0x32 交给 processInternalCrsfPackage,
                      那里要求 dest == 0xEE 且 orig == 0xEA, 否则整条丢掉)
     [4] orig   0xEA  手台地址
     [5] sub1        0x10 = CRSF_COMMAND_SUBCMD_RX
     [6] sub2        0x01 = ..._RX_BIND(对频), 0x05 = ..._MODEL_SELECT_ID(Model Match)
     [7] crc8_BA     内层 CRC(poly 0xBA), 官方写法才有; 模块把它当普通 payload
     [8] crc8        外层 CRC(poly 0xD5)覆盖 [2..7](模块自己就是 crsf_crc.calc(&buf[2], len-3)),
                     模块唯一校验的一字节
   对频帧(9 字节): C8 07 32 EE EA 10 01 14 EB
   注意: 只有这一种写法 —— 之前那些变体(广播目的地址 0x00 / 子命令 0x0A / 单 CRC)都会变成
   模块眼里的坏包, 越试越连不上, 已全部删除 */
void CRSF_SendCommand(uint8_t sub1, uint8_t sub2, const uint8_t *data, uint8_t len);
void CRSF_SendCommandTo(uint8_t dest, uint8_t sub1, uint8_t sub2, const uint8_t *data, uint8_t len);
extern volatile uint8_t crsf_cmd_two_crc;  /* 1 = 双 CRC 官方写法(默认), 0 = 单 CRC */
extern volatile uint8_t crsf_cmd_ok;       /* 开机自检: 1 = 对频帧字节与金标一致(诊断页 "S:") */
void CRSF_SetCmdCrcForm(uint8_t two_crc);  /* 切换命令帧 CRC 形式(供 A/B 测试) */
void CRSF_CmdSelfTest(void);               /* 命令帧自检(CRSF_Init 里调用) */

/* ========================== 参数协议(0x28/0x29/0x2B/0x2C/0x2D) ========================== */
extern volatile uint8_t  crsf_pt_type;       /* 最近收到的参数协议帧类型 */
extern volatile uint8_t  crsf_pt_len;        /* 该帧字节数(截断到 48) */
extern volatile uint16_t crsf_pt_cnt;        /* 参数协议帧计数 */
extern volatile uint8_t  crsf_pt_buf[48];    /* 原始字节(十六进制显示用) */
extern volatile uint8_t  crsf_dev_online;    /* 1 = 收到过 0x29 设备信息 */
extern volatile uint8_t  crsf_dev_field_cnt; /* 0x29: 参数字段数 */
extern volatile uint8_t  crsf_dev_param_ver; /* 0x29: 参数版本 */
extern volatile uint8_t  crsf_param_addr;  /* 参数帧使用的地址(0=用 crsf_cur_addr) */
extern volatile uint8_t  crsf_ping_idx;    /* ping 变体序号 */
extern volatile uint8_t  crsf_ping_locked; /* 1 = 已锁定可用的 ping 变体 */
extern volatile uint8_t  crsf_ping_buf[24]; /* 模块发来的 0x28 ping 原始字节(模板) */
extern volatile uint8_t  crsf_ping_len;    /* 该帧字节数 */
extern volatile uint16_t crsf_ping_cnt;    /* 收到模块 ping 的次数 */
void CRSF_ReqPing(void);
void CRSF_PingTick(void);   /* 每 2s 调一次: 扫描 ping 变体直到模块应答 */
void CRSF_SetCrcFrom(uint8_t from);  /* 强制 CRC 覆盖约定(0/2), 供 A/B 测试 */
void CRSF_SetTxConvention(uint8_t addr, uint8_t crc_from);  /* 同时强制地址与 CRC 约定 */
void CRSF_ReqParamEntry(uint8_t field, uint8_t chunk);
void CRSF_ReqParamRead(uint8_t field, uint8_t chunk);
void CRSF_ParamWrite(uint8_t field, const uint8_t *val, uint8_t len);

/* ===== 参数表浏览/枚举(0x2B/0x2C) ===== */
extern volatile uint8_t crsf_param_sel;    /* UI 当前浏览的字段号 */
extern volatile uint8_t crsf_param_scan;   /* 1 = 正在扫描 */
extern volatile uint8_t crsf_param_ready;  /* 1 = 已解析到字段 */
extern volatile uint8_t crsf_pt_layout;    /* 参数请求 payload 布局(0/1/2) */
void    CRSF_ParamScanStart(void);         /* 开始扫描整张参数表 */

/* ===== 上行遥测帧(手台->模块) ===== */
void CRSF_SendAttitude(int16_t pitch, int16_t roll, int16_t yaw);
void    CRSF_ParamTick(void);              /* 扫描推进(UI 每次调用) */
uint8_t CRSF_ParamType(uint8_t id);
uint8_t CRSF_ParamName(uint8_t id, char *dst, uint8_t max);
uint8_t CRSF_ParamValueLen(uint8_t id);
uint8_t CRSF_ParamValue(uint8_t id, uint8_t idx);
uint8_t CRSF_ParamFind(const char *key);   /* 按名字找字段号, 0xFF = 没找到 */
void    CRSF_ParamTrigger(uint8_t id);     /* 触发 COMMAND 类字段 */
uint8_t CRSF_ParamStep(uint8_t id, uint8_t delta);   /* 值 +delta 并写回, 返回新值(0xFF = 该字段没有值可写) */

#endif /* __CRSF_H__ */
