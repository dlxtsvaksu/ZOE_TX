#include "Schedule.h"
#include "Func.h"
#include "oled.h"
#include "font.h"
#include "Crsf.h"
#include "usart.h"
#include "Ui.h"
#include "Ppm.h"
#include "UsbHid.h"
#include "main.h"    /* 引脚宏: PWR_DETECT / PWR_EN / LED */

static void func1(void) {
    ADC_Calibrate();
}
static void func2(void) {
    Adc_convert_Crsf();
    Mix_Apply();          /* 叠加混控并限幅, 得到最终 16 通道 */
}
static void func3(void) {
    Switch_Read();
    Key_Read();
}
volatile uint8_t flag = 0;

/* ========================= 电源键: 长按开机使能 / 长按软关机 =========================
   硬件版本: 见 BoardCfg.h 的 BOARD_PWR_VERSION(用户自己填), 本文件按版本分叉:
     [长按开关机版] 下面这一整套时序完整生效(读电源键 -> 1.2s 锁存 PWR_EN -> 再长按 3s 软关机);
     [自锁开关版]   机械自锁开关直接决定供电: 上电即 pwr_latched = 1, 不读电源键,
                    不写 PWR_EN, 也不软关机(断电只能拨开关) -- 见本段末尾的 #else 分支.
   下面这段说明描述的是"长按开关机版":
   硬件: 电源键按下 -> PB13(PWR_DETECT) 读高; 固件把 PB14(PWR_EN) 拉高, 让自锁电路维持供电.
   时序(与开机进度条同步):
     按住不足 UI_BOOT_MS(1.2s) 松手 -> 进度清零, PWR_EN 保持低(可重新按住, 从头再走)
     按住到 UI_BOOT_MS             -> 锁存: PWR_EN 拉高, 松手不断电(LED 亮 = 电源好)
     锁存后再按住 PWR_OFF_MS(3s)   -> 软关机: 清锁存 + PWR_EN 拉低(松手即断电);
                                      计时从"锁存那一刻"起算(不是从按下起算), 否则按住 1.2s
                                      锁存后不撒手会被立刻判成关机; 松手前压住 off_lock,
                                      必须松手再按才能重新开机
     软关机完成                     -> pwr_off_done = 1: Ui.c 在未锁存且 pwr_hold_ms == 0 时
                                      画静态"关机/长按开机"页, 不再画空进度开机界面
   进度条与使能必须同源: 这里只维护 pwr_hold_ms(真实毫秒). 用 HAL_GetTick() 时间戳而不用
   "调用次数"计数 —— 1ms 槽会被 OLED 推页阻塞 ~3ms 而丢拍, 计数会与进度条/真实时间脱节.
   Ui.c 只负责把 pwr_hold_ms 画成进度条, 所以"条填满那一刻"恒等于"PWR_EN 拉高的那一刻". */
#define PWR_DEBOUNCE_MS 3U       /* 松手去抖: 低电平连续这么久才算松开(机械键会弹跳) */

volatile uint32_t pwr_hold_ms = 0U;   /* 两版都有: Ui.c 拿它画开机进度条 + 判断是否已开机 */
volatile uint32_t pwr_off_ms  = 0U;   /* 锁存后本次按压已持续(ms), 0 = 没在按: Ui.c 画关机倒计时 */
volatile uint8_t  pwr_off_done = 0U;  /* 1 = 软关机已完成: Ui.c 显示静态"关机"页, 不再画空进度开机界面 */

#if BOARD_PWR_HAS_LATCH
/* 电源键来源(见 BoardCfg.h): PB13/PWR_DETECT 按下读高是本板设计; PA15/EXIT 是早期误读的脚 */
#if (BOARD_PWR_KEY_SRC == BOARD_PWR_KEY_SRC_DETECT)
  #define PWR_KEY_PORT   PWR_DETECT_GPIO_Port
  #define PWR_KEY_PIN    PWR_DETECT_Pin
  #define PWR_KEY_DOWN   GPIO_PIN_SET        /* 按下 = 高 */
#else
  #define PWR_KEY_PORT   EXIT_GPIO_Port
  #define PWR_KEY_PIN    EXIT_Pin
  #define PWR_KEY_DOWN   GPIO_PIN_RESET      /* 按下 = 低 */
#endif

volatile uint8_t  pwr_latched = 0U;   /* 长按版: 锁存供电后才为 1(软关机后回到 0) */

void power_detect(void) {
    static uint32_t hold_at  = 0U;   /* 本次按下的起始时刻 */
    static uint8_t  hold_act = 0U;   /* 1 = 本次按下已在计时 */
    static uint8_t  rel_ms   = 0U;   /* 松开去抖计数 */
    static uint8_t  off_lock = 0U;   /* 软关机后压住: 松手前不再重新计时/锁存 */
    uint32_t held;
    uint32_t now = HAL_GetTick();

    if (HAL_GPIO_ReadPin(PWR_KEY_PORT, PWR_KEY_PIN) == PWR_KEY_DOWN) {   /* PB13 按下读高 */
        rel_ms = 0U;

        if (off_lock != 0U) {                /* 软关机后仍按着: 不计时, 等松手 */
            pwr_hold_ms = 0U;
            pwr_off_ms  = 0U;
            return;
        }

        if (hold_act == 0U) {                /* 按下沿: 从这一刻开始计时 */
            hold_act = 1U;
            hold_at  = now;
        }
        held = now - hold_at;

        if (pwr_latched == 0U) {
            pwr_hold_ms = held;              /* 进度条数据源 */
            if (held >= UI_BOOT_MS) {        /* 进度条刚好填满的同一条件 -> 锁存供电 */
                pwr_latched = 1U;
                pwr_off_done = 0U;           /* 已重新锁存开机: 退出"软关机完成"状态 */
                pwr_hold_ms = UI_BOOT_MS;    /* 进度条钉在满格 */
                pwr_off_ms  = 0U;            /* 关机长按从这一刻开始计时 */
                hold_at     = now;           /* 重定基准: 否则"按住 1.2s 后一直拿着"会被立刻判成关机 */
                HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);   /* LED 亮 = 电源好 */
            }
        } else {
            pwr_hold_ms = UI_BOOT_MS;        /* 已锁存: 进度条钉在满格 */
            pwr_off_ms  = held;              /* 锁存后已按住多久 -> Ui.c 画关机倒计时 */
            if (held >= PWR_OFF_MS) {        /* 再长按 3s -> 软关机 */
                pwr_off_done = 1U;           /* 先置完成标志, 避免 Ui.c 看到 pwr_latched=0 的瞬间画开机页 */
                pwr_latched = 0U;
                pwr_hold_ms = 0U;
                pwr_off_ms  = 0U;
                hold_act    = 0U;
                off_lock    = 1U;
                HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
            }
        }
    } else {                                 /* 按键松开 */
        off_lock = 0U;                       /* 松手后允许重新长按开机 */
        if (rel_ms < PWR_DEBOUNCE_MS) rel_ms++;

        if (rel_ms >= PWR_DEBOUNCE_MS) {
            hold_act   = 0U;                 /* 本次计时结束(锁存与否都要清, 否则下次按会误判) */
            pwr_off_ms = 0U;                 /* 松手 = 取消软关机长按计时 */
            if (pwr_latched == 0U) {         /* 没锁存就松手: 进度清零, 可重按 */
                pwr_hold_ms = 0U;
                HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
            }
            /* 已锁存: 松手不断电, LED/PWR_EN 保持; pwr_hold_ms 也保持满格 */
        }
    }
}

#else  /* ==================== 自锁开关版: 开关直接供电, 固件不碰电源 ==================== */

/* 上电即视为已开机(机械开关已经合上): 不需要也不应该去拉 PWR_EN */
volatile uint8_t  pwr_latched = 1U;

void power_detect(void) {
    pwr_off_done = 0U;           /* 自锁版没有软关机状态 */
    pwr_hold_ms = UI_BOOT_MS;    /* 进度条数据源钉满格 -> Ui.c 直接跳过开机门禁进主界面 */
    pwr_off_ms  = 0U;            /* 恒 0 -> Ui.c 的关机动画分支永不进入(长按无作用) */
}

#endif /* BOARD_PWR_HAS_LATCH */

/* ================================================================
 * 屏幕内容: 1 = CRSF 诊断页(排查"接收机全 172 / 链路断在哪"用)
 *           0 = 摇杆指示界面
 * ================================================================ */
#define OLED_DIAG_SCREEN 1

#if OLED_DIAG_SCREEN
/* 十进制右对齐打印(width = 字符宽度, 左补空格; afont8x6 每字符 6x8) */
static void DiagNum(uint8_t x, uint8_t y, int32_t v, uint8_t width)
{
    char s[14];
    uint8_t n = 0;
    uint32_t u;

    if (v < 0) { u = (uint32_t)(-v); } else { u = (uint32_t)v; }
    do {
        s[n++] = (char)('0' + (u % 10u));
        u /= 10u;
    } while ((u != 0u) && (n < 12u));
    if (v < 0) s[n++] = '-';
    while (n < width) s[n++] = ' ';
    for (uint8_t i = 0; i < (n / 2u); i++) {
        char t = s[i]; s[i] = s[n - 1u - i]; s[n - 1u - i] = t;
    }
    s[n] = '\0';
    OLED_PrintASCIIString(x, y, s, &afont8x6, OLED_COLOR_NORMAL);
}

/* CRSF 诊断页(按 UP 键切页):
 *  第1页 发送侧: TX/s、T/s = 实测帧率/节拍率        TX、SK = 发送总数/被跳过的节拍数
 *                ERR、G、EC = 发送错误数 / huart2.gState / huart2.ErrorCode
 *                A、C、L = 发送地址 / CRC起始 / 探测是否锁定   I = NVIC 自检(T1,U2,D7,D6)
 *                RX、E、O、M = 收到合法帧 / CRC错 / 其他类型 / 模块帧CRC约定
 *  第2页 链路+摇杆: LQ、R、S = 上行链路质量/RSSI/SNR    DN、DR、DS = 下行
 *                S1、S2 = 我们实际发出去的四个摇杆通道值   V、L、ST、K = 链路统计/锁定/停止/0x14新鲜
 *  两页都显示:  CLK = 主频(MHz, 读 HCLK; 现在 72 = HSE 8MHz 晶振 x PLL9, 由 .ioc 时钟树决定)
 *                IE  = OLED I2C 失败次数(400kHz 下若持续增长 = I2C 速率偏高, 降到 200k)
 */
static uint8_t diag_page = 0;              /* 0 = 发送侧诊断, 1 = 链路与摇杆 */
/* 第 8 行: 模块到底有没有在说话.
   最危险的情况是"以前在说话, 现在不说了" = MOD SILENT 3s:
   手台帧的 CRC 约定/地址一旦不合模块的规矩, 模块的 UART 看门狗(Bad >= Good)就断开重扫波特率
   (noCrossfire), 期间完全不发射 RF -> 接收机永远连不上. 后面跟的是模块最后回帧的首字节
   (正常应为 0xEA/0xC8/0xEE; 一直是 0x00 = 从来没收到过模块的合法帧) */
#define MOD_SILENT_MS 3000U
static void DiagModLine(void)
{
    static uint16_t last_cnt = 0U;
    static uint32_t stamp    = 0U;
    const char *hex = "0123456789ABCDEF";
    char s[24];
    uint8_t n = 0U;
    const char *pre;

    if (crsf_rx_cnt != last_cnt) { last_cnt = crsf_rx_cnt; stamp = HAL_GetTick(); }

    if ((crsf_rx_cnt == 0U) && (crsf_rx_crc_err == 0U))
    {
        OLED_PrintASCIIString(1, 55, "MOD SILENT (no byte)", &afont8x6, OLED_COLOR_NORMAL);
        return;
    }

    if ((uint32_t)(HAL_GetTick() - stamp) >= MOD_SILENT_MS) pre = "MOD SILENT 3s";
    else if (crsf_probe_locked)                            pre = "MOD TALK LOCK";
    else if (crsf_probe_stopped)                           pre = "MOD TALK STOP";
    else                                                   pre = "MOD TALKING";

    while (pre[n] != 0) { s[n] = pre[n]; n++; }
    s[n++] = ' '; s[n++] = '0'; s[n++] = 'x';
    s[n++] = hex[(crsf_rx_addr >> 4) & 0x0FU];
    s[n++] = hex[crsf_rx_addr & 0x0FU];
    s[n]   = 0;
    OLED_PrintASCIIString(1, 55, s, &afont8x6, OLED_COLOR_NORMAL);
}

/* 第 1 页: 发送侧(排查"TX 不涨 / 涨得慢") */
static void DiagPageTx(void)
{
    OLED_PrintASCIIString(1, 1, "TX/s:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(31, 1, crsf_tx_rate, 3);
    OLED_PrintASCIIString(49, 1, " T/s:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(79, 1, crsf_tick_rate, 3);

    OLED_PrintASCIIString(1, 8, "TX:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(19, 8, crsf_tx_cnt, 5);
    OLED_PrintASCIIString(49, 8, " SK:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(73, 8, crsf_tx_skip_cnt, 5);

    OLED_PrintASCIIString(1, 16, "ERR:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(25, 16, crsf_tx_err, 4);
    OLED_PrintASCIIString(49, 16, " G:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(67, 16, huart2.gState, 2);
    OLED_PrintASCIIString(79, 16, " EC:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(103, 16, huart2.ErrorCode, 3);

    OLED_PrintASCIIString(1, 24, "A:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(13, 24, crsf_cur_addr, 3);
    OLED_PrintASCIIString(31, 24, " C:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(49, 24, crsf_cur_crc_from, 1);
    OLED_PrintASCIIString(55, 24, " L:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(73, 24, crsf_probe_locked, 1);
    OLED_PrintASCIIString(79, 24, "I:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(91, 24, (crsf_nvic_ok >> 0) & 1u, 1);
    DiagNum(97, 24, (crsf_nvic_ok >> 1) & 1u, 1);
    DiagNum(103, 24, (crsf_nvic_ok >> 2) & 1u, 1);
    DiagNum(109, 24, (crsf_nvic_ok >> 3) & 1u, 1);

    OLED_PrintASCIIString(1, 32, "RX:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(19, 32, crsf_rx_cnt, 5);
    OLED_PrintASCIIString(49, 32, " E:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(67, 32, crsf_rx_crc_err, 5);

    OLED_PrintASCIIString(1, 40, "O:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(13, 40, crsf_rx_other_cnt, 5);
    OLED_PrintASCIIString(43, 40, " M:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(61, 40, crsf_rx_crc_from, 3);
    /* 命令帧自检: 1 = 组出的对频帧与金标 C8 07 32 EE EA 10 01 14 EB 字节一致 */
    OLED_PrintASCIIString(79, 40, " S:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(97, 40, crsf_cmd_ok, 1);

    OLED_PrintASCIIString(1, 48, "UP = page2", &afont8x6, OLED_COLOR_NORMAL);
    /* OLED I2C 失败次数: 400kHz 下应保持 0(或极小), 持续增长说明速率偏高/上拉不足 */
    OLED_PrintASCIIString(67, 48, "IE:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(85, 48, oled_i2c_err, 3);
    DiagModLine();
}

/* 第 2 页: 链路统计 + 摇杆值 */
static void DiagPageRx(void)
{
    OLED_PrintASCIIString(1, 1, "LQ:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(19, 1, crsf_link_stats.uplink_lq, 3);
    OLED_PrintASCIIString(37, 1, " R:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(55, 1, crsf_link_stats.uplink_rssi_dbm, 5);
    OLED_PrintASCIIString(85, 1, " S:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(103, 1, crsf_link_stats.uplink_snr, 4);

    OLED_PrintASCIIString(1, 8, "DN:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(19, 8, crsf_link_stats.downlink_lq, 3);
    OLED_PrintASCIIString(37, 8, " DR:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(55, 8, crsf_link_stats.downlink_rssi_dbm, 5);
    OLED_PrintASCIIString(85, 8, " DS:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(103, 8, crsf_link_stats.downlink_snr, 4);

    OLED_PrintASCIIString(1, 16, "S1:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(19, 16, aetr[0], 5);
    DiagNum(55, 16, aetr[1], 5);

    OLED_PrintASCIIString(1, 24, "S2:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(19, 24, aetr[2], 5);
    DiagNum(55, 24, aetr[3], 5);

    OLED_PrintASCIIString(1, 32, "V:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(13, 32, crsf_link_stats.valid, 1);
    OLED_PrintASCIIString(19, 32, " L:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(37, 32, crsf_probe_locked, 1);
    OLED_PrintASCIIString(43, 32, " ST:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(61, 32, crsf_probe_stopped, 1);
    /* K: 1 = 超时时间内收到过 0x14(链路统计新鲜, 主界面据此显示"已连接") */
    OLED_PrintASCIIString(67, 32, " K:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(85, 32, crsf_link_ok, 1);
    /* B: 最近回传的接收机电池电压(这里按 0.1V 一档显示, 041 = 4.1V; 由 0x08 电池传感器帧更新).
       帧里是 0.01V 一档的原始计数(见 Crsf.h 的 CRSF_RXBAT_RAW_PER_V), 所以这里除 10 变成 0.1V.
       一直 000 且主界面显示 "RX --" -> 模块/接收机没在回传该传感器 */
    OLED_PrintASCIIString(91, 32, " B:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(109, 32, (uint16_t)(crsf_rx_bat_raw / 10U), 3);

    OLED_PrintASCIIString(1, 40, "TX/s:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(31, 40, crsf_tx_rate, 3);
    OLED_PrintASCIIString(49, 40, " T/s:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(79, 40, crsf_tick_rate, 3);

    OLED_PrintASCIIString(1, 48, "UP = page1", &afont8x6, OLED_COLOR_NORMAL);
    /* P: 模块在 0x14 里回报的发射功率档位(1=10mW 2=25mW 3=100mW 4=500mW 5=1W 6=2W 7=250mW 8=50mW)
          F: 参数表是否已扫到(1 = 模块支持参数协议; UI 只在对频/诊断用得到它) */
    OLED_PrintASCIIString(61, 48, "P:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(73, 48, crsf_link_stats.uplink_tx_power, 2);
    OLED_PrintASCIIString(85, 48, " F:", &afont8x6, OLED_COLOR_NORMAL);
    DiagNum(103, 48, crsf_param_ready, 1);
    DiagModLine();
}

void DiagShowPage(void)
{
    if (Key_State_Queue[UP] != 0U)          /* UP 键切页 */
    {
        Key_State_Queue[UP] = 0U;
        diag_page = (uint8_t)(diag_page ^ 1U);
    }

    OLED_NewFrame();
    if (diag_page == 0U)
        DiagPageTx();
    else
        DiagPageRx();
}
#endif

static void func4(void) {
    /* OLED 非阻塞推送: 一帧推完才重画, 否则只继续推进未发完的页 */
    if (!OLED_FrameBusy())
        UI_Task();
    OLED_ShowFrame();
}
static Task_t tasks[] = {
    { power_detect, 1, 0 },   /* 1ms: 长按版 = 电源键 1.2s 锁存 PWR_EN(与开机进度条同源) + 再长按 3s 软关机;
                                 自锁开关版 = 空实现(只把 pwr_hold_ms 钉满格) */
    { func1, 5, 0 },
    { Adc_FilterTask, 1, 0 },
    { Batt_Task,      1, 0 },   /* 1ms: 电池电压平均(ADC_IN6) */
    { func2,  5, 0 },
    { PPM_Task, 5, 0 },   /* 5ms: ch_out[] -> PPM 脉宽表(混控结果已算好, 见 Ppm.c) */
    { func3, 10, 0 },
    { USB_HID_Task,  5, 0 },   /* USB HID: 5ms 上报一次摇杆/开关 */
    { CRSF_TxTask,   1, 0 },   /* CRSF: 每个 4ms 节拍组一帧 0x16 */
    { CRSF_RxTask,   1, 0 },   /* CRSF: 解析模块下行帧(0x14/0x28...) */
    { func4,  5, 0 },   /* 5ms: 一页 3.0ms(400kHz) > 预算 3ms -> 每次只推 1 页, 一帧 8 页 x 5ms ≈ 40ms(~25fps).
                           想再快就把 oled.c 的 OLED_FRAME_BUDGET_MS 提到 7(每次 2 页 -> ~20ms/50fps),
                           代价是每次阻塞 ~6ms, 1ms 任务的间歇被拉到 ~7ms */
};

void Schedule_Run(void) {
    uint32_t now = HAL_GetTick();
    uint32_t n  = sizeof(tasks) / sizeof(tasks[0]); 

    for (uint32_t i = 0; i < n; i++) {
        if (tasks[i].interval_ms != 0 &&
            now - tasks[i].last_run >= tasks[i].interval_ms) {
            if (tasks[i].func != NULL) tasks[i].func();
            tasks[i].last_run = now;
        }
    }
}
