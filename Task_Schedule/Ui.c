/**
 * @file Ui.c
 * @brief 界面: 第0页=摇杆, 第1页=菜单(系统选项 / 对频), 第2页=诊断
 *
 * @note 按键: UP/DOWN 移动焦点(到顶/到底自动循环), 长按加速连发(见 UI_FocusStep);
 *       ENTER 进入或执行, EXIT 返回
 * @note 摇杆校准: 直接驱动 Func.c 里已有的状态机(state + ADC_Calibrate),
 *       界面上显示当前阶段和倒计时, 完成后提示校准结果
 * @note 每个页面都由 UI_DrawPage() 统一补一圈整屏方框(见 UI_ScreenFrame): 方角;
 *       只有开机界面(UI_BootAnim)和软关机动画(UI_PwrOffPage)的整屏大框保留四角倒角
 */
#include "Ui.h"
#include "Schedule.h"
#include "Func.h"
#include "Crsf.h"
#include "oled.h"
#include "font.h"
#include "usart.h"
#include "Store.h"
#include "main.h"
#include <string.h>

/* Schedule.c 里实现的诊断页(菜单里的 Diag 项) */
void DiagShowPage(void);

/* ========================== 页面与菜单动作 ========================== */
#define UI_PAGE_STICK 0U
#define UI_PAGE_MENU  1U
#define UI_PAGE_DIAG  3U
#define UI_PAGE_CHAN  2U
#define UI_PAGE_CHBAR 7U
#define UI_PAGE_INPUT 8U
#define UI_PAGE_MODEL 10U
#define UI_PAGE_MIX   11U
#define UI_PAGE_MIXED 12U
#define UI_PAGE_VER   13U
#define UI_PAGE_TRIMSET 14U

#define UI_A_SYS    0U
#define UI_A_CAL    2U
#define UI_A_DIAG   3U
#define UI_A_BIND   4U
#define UI_A_BACK   7U
#define UI_A_CHAN  10U

#define UI_A_INPUT 12U
#define UI_A_MODEL 14U
#define UI_A_MIX   15U
#define UI_A_VER   16U
#define UI_A_TRIM  17U

typedef struct {
    const char *label;
    uint8_t     action;
} UI_MenuItem_t;

static const UI_MenuItem_t ui_items_root[] = {
    { "混控",    UI_A_MIX   },
    { "模型",  UI_A_MODEL },
    { "系统", UI_A_SYS  },
    { "对频",   UI_A_BIND },
};
static const UI_MenuItem_t ui_items_sys[] = {
    { "校准", UI_A_CAL  },
    { "输入映射",     UI_A_INPUT },
    { "微调分配",     UI_A_TRIM  },
    { "版本",   UI_A_VER  },

    { "返回",      UI_A_BACK },
};

static uint8_t  ui_page    = UI_PAGE_STICK;
static uint8_t  ui_menu    = 0U;      /* 0=根 1=System */
static uint8_t  ui_cursor  = 0U;
static uint8_t  ui_cal     = 0U;      /* 1 = 正在校准 */
static uint8_t  ui_cal_ok  = 0U;      /* 校准完成提示 */
static uint32_t ui_stamp   = 0U;      /* 校准计时 */
static uint32_t ui_boot    = 0U;      /* 锁存完成时刻(用于启动提示 / READY 延时) */
static uint8_t  ui_bind_popup = 0U;   /* 1 = 正在显示对频状态小弹窗 */
static uint32_t ui_bind_stamp = 0U;   /* 对频弹窗出现时刻(用于 6 秒自动退出) */

/* 输入分配页(第8页)状态 */
static uint8_t  ui_in_focus = 0U;      /* 0~7: 偶数 = SRC, 奇数 = DIR */
static uint8_t  ui_in_edit  = 0U;      /* 1 = 正在编辑焦点格 */
/* 微调分配页(第14页)状态 */
static uint8_t  ui_tr_focus = 0U;      /* 0~7: 偶数 = 微调格, 奇数 = 方向格 */
static uint8_t  ui_tr_edit  = 0U;      /* 1 = 正在编辑焦点行 */
/* 混控页状态 */
static uint8_t  ui_mix_ch    = 0U;      /* 选中的通道 0~15 */
static uint8_t  ui_mix_focus = 0U;      /* 编辑器里的有效格编号 */
static uint8_t  ui_mix_edit  = 0U;      /* 1 = 编辑态 */
static uint32_t ui_mix_press = 0U;      /* ENTER 按下时刻(长按删除) */
static uint32_t ui_rep_t     = 0U;      /* 连发起始时刻 */
static uint32_t ui_rep_last  = 0U;      /* 上次连发时刻 */
static uint8_t  ui_rep_key   = 0xFFU;   /* 正在连发的按键(0xFF = 无) */
static uint8_t  ui_rep_step  = 0U;      /* 已连发次数(用于加速) */

#define UI_MSG_MS 1500U


/* ========================== 小工具 ========================== */
/* 摇杆 / 通道数值 / 通道进度条 三页循环: dir=0 上一页, dir=1 下一页 */
static uint8_t UI_RingStep(uint8_t page, uint8_t dir)
{
    if (dir != 0U)
        return (page == UI_PAGE_STICK) ? UI_PAGE_CHAN : ((page == UI_PAGE_CHAN) ? UI_PAGE_CHBAR : UI_PAGE_STICK);
    return (page == UI_PAGE_STICK) ? UI_PAGE_CHBAR : ((page == UI_PAGE_CHAN) ? UI_PAGE_STICK : UI_PAGE_CHAN);
}

/* 按键事件的有效期: UI 每 5ms 跑一拍, 正常情况下事件当拍就被消费掉.
   超过这个时间还没被消费的一律丢弃 —— 否则那条"陈旧事件"会被后来的页面当成刚按下的键.
   典型现象: 电源键与 EXIT 同脚(PA15), 上电要长按 1.2s 才锁存 -> 队列里留了一条 EXIT;
   开机/READY/关机动画这三个阶段 UI 直接 return 不消费按键, 于是这条 EXIT 一直躺到
   第一次按 ENTER 进菜单时被菜单分支当"返回"吃掉: 菜单闪一下退回主界面(只有第一次) */
#define UI_KEY_TTL_MS 100U

static uint8_t UI_Key(uint8_t id)
{
    if (Key_State_Queue[id] != 0U)
    {
        uint8_t fresh = ((uint32_t)(HAL_GetTick() - Key_State_Time[id]) < UI_KEY_TTL_MS) ? 1U : 0U;

        Key_State_Queue[id] = 0U;
        return fresh;
    }
    return 0U;
}

/* 队列里的事件全部作废(开机/READY/关机动画阶段用: 这几段 UI 不认按键) */
static void UI_KeyFlush(void)
{
    for (uint8_t i = 0U; i < 4U; i++)
        Key_State_Queue[i] = 0U;
}

/* 打印整数(右对齐, 左补空格) */
static void UI_Num(uint8_t x, uint8_t y, int32_t v, uint8_t width)
{
    char s[14];
    uint8_t n = 0U;
    uint32_t u;

    if (v < 0) { u = (uint32_t)(-v); } else { u = (uint32_t)v; }
    do { s[n++] = (char)((char)0x30 + (u % 10u)); u /= 10u; } while ((u != 0u) && (n < 12u));
    if (v < 0) s[n++] = (char)0x2D;
    while (n < width) s[n++] = (char)0x20;
    for (uint8_t i = 0U; i < (n / 2u); i++) { char t = s[i]; s[i] = s[n - 1u - i]; s[n - 1u - i] = t; }
    s[n] = (char)0;
    OLED_PrintASCIIString(x, y, s, &afont8x6, OLED_COLOR_NORMAL);
}
/* 顶行反色用: 带颜色参数的打印(白底黑字 = OLED_COLOR_REVERSED) */
static void UI_NumC(uint8_t x, uint8_t y, int32_t v, uint8_t width, OLED_ColorMode color)
{
    char s[14];
    uint8_t n = 0U;
    uint32_t u;

    if (v < 0) { u = (uint32_t)(-v); } else { u = (uint32_t)v; }
    do { s[n++] = (char)((char)0x30 + (u % 10u)); u /= 10u; } while ((u != 0u) && (n < 12u));
    if (v < 0) s[n++] = (char)0x2D;
    while (n < width) s[n++] = (char)0x20;
    for (uint8_t k = 0U; k < (n / 2u); k++) { char t = s[k]; s[k] = s[n - 1u - k]; s[n - 1u - k] = t; }
    s[n] = (char)0;
    OLED_PrintASCIIString(x, y, s, &afont8x6, color);
}
static void UI_NumD(uint8_t x, uint8_t y, int32_t v, uint8_t width, OLED_ColorMode color)
{
    char s[14];
    uint8_t n = 0U;
    uint32_t u;

    if (v < 0) { u = (uint32_t)(-v); } else { u = (uint32_t)v; }
    do { s[n++] = (char)((char)0x30 + (u % 10u)); u /= 10u; } while ((u != 0u) && (n < 12u));
    if (v < 0) s[n++] = (char)0x2D;
    while (n < width) s[n++] = (char)0x20;
    for (uint8_t k = 0U; k < (n / 2u); k++) { char t = s[k]; s[k] = s[n - 1u - k]; s[n - 1u - k] = t; }
    s[n] = (char)0;
    OLED_PrintASCIIString(x, y, s, &afont12x6, color);
}

static void UI_SecC(uint8_t x, uint8_t y, uint32_t ms_left, OLED_ColorMode color)
{
    char s[6];

    s[0] = (char)((char)0x30 + (uint8_t)((ms_left / 1000U) % 10U));
    s[1] = (char)0x2E;
    s[2] = (char)((char)0x30 + (uint8_t)((ms_left % 1000U) / 100U));
    s[3] = (char)0x73;
    s[4] = (char)0;
    OLED_PrintASCIIString(x, y, s, &afont8x6, color);
}
/* ========================== 第0页: 摇杆 ========================== */
/* 开关位置字符: < = 172(低), - = 992(中), > = 1811(高) */
/* 长按 UP/DOWN 步进: 按下边沿立即 1 步, 按住后按下面两档参数连发(见 UI_KeyRepeatEx):
   UI_KeyRepeat 快档(菜单/数值编辑) / UI_KeyRepeatSlow 慢档(通道/模型这类列表选择).
   返回本次应步进的次数(0 = 不动作) */
/* 连发间隔: 从 iv0 起每连发一步减 accel, 最快到 ivmin(iv0 == ivmin 时恒为 ivmin) */
static uint32_t UI_RepInterval(uint8_t step, uint32_t iv0, uint32_t iv_min, uint32_t accel)
{
    uint32_t dec = (uint32_t)step * accel;

    return (dec >= (iv0 - iv_min)) ? iv_min : (iv0 - dec);
}

static uint8_t UI_KeyRepeatEx(uint8_t id, uint32_t delay_ms, uint32_t iv0, uint32_t iv_min, uint32_t accel)
{
    uint32_t now = HAL_GetTick();
    uint32_t held, iv;
    uint8_t  steps = 0U;

    if (UI_Key(id))                        /* 按下边沿: 立刻一步 */
    {
        ui_rep_key  = id;
        ui_rep_t    = now;
        ui_rep_last = now;
        ui_rep_step = 0U;
        return 1U;
    }
    if ((ui_rep_key != id) || (Key_State_Now[id] == 0U))
        return 0U;                         /* 这个键不在按住状态 */

    held = now - ui_rep_t;
    if (held < delay_ms)
        return 0U;                         /* 先等 delay_ms */

    iv = UI_RepInterval(ui_rep_step, iv0, iv_min, accel);
    while ((now - ui_rep_last) >= iv)      /* 补步: 上限 20 步/次 */
    {
        ui_rep_last += iv;
        if (ui_rep_step < 200U) ui_rep_step++;
        if (steps < 20U) steps++;
        iv = UI_RepInterval(ui_rep_step, iv0, iv_min, accel);
    }
    return steps;
}

/* 快档连发(菜单移动/数值编辑用): 按住 300ms 后开始, 间隔 60ms 每步减 5ms 直到 15ms */
static uint8_t UI_KeyRepeat(uint8_t id)
{
    return UI_KeyRepeatEx(id, 300U, 60U, 15U, 5U);
}

/* 慢档连发(通道/模型这类列表选择用): 按住 600ms 后开始, 间隔固定 150ms 不加速,
   长按不会一下飞过好几个候选 */
static uint8_t UI_KeyRepeatSlow(uint8_t id)
{
    return UI_KeyRepeatEx(id, 600U, 150U, 150U, 150U);
}

/* 焦点循环步进(菜单/列表/多档选项统一用这个):
   UP/DOWN 在 0 ~ num-1 里循环(到顶再按回到末尾), 长按按 UI_KeyRepeatEx 连发;
   slow != 0 时用慢档(UI_KeyRepeatSlow), 否则用快档(UI_KeyRepeat).
   dir = 0 对应 UP(上一格), dir = 1 对应 DOWN(下一格); 返回步进后的焦点值.
   一帧内 UP/DOWN 各调一次即可(两个键同时按住也都能补步) */
static uint8_t UI_FocusStepEx(uint8_t focus, uint8_t num, uint8_t dir, uint8_t slow)
{
    uint8_t n;

    if (num == 0U)
        return focus;
    if (dir == 0U)
    {
        n = (slow != 0U) ? UI_KeyRepeatSlow(UP) : UI_KeyRepeat(UP);
        while (n-- != 0U)
            focus = (uint8_t)((focus + num - 1U) % num);
    }
    else
    {
        n = (slow != 0U) ? UI_KeyRepeatSlow(DOWN) : UI_KeyRepeat(DOWN);
        while (n-- != 0U)
            focus = (uint8_t)((focus + 1U) % num);
    }
    return focus;
}



/* 慢档焦点步进(通道/模型这种长列表选择用): 长按时的步进慢, 不会一下飞过好几个候选 */
static uint8_t UI_FocusStepSlow(uint8_t focus, uint8_t num, uint8_t dir)
{
    return UI_FocusStepEx(focus, num, dir, 1U);
}

/* 快档焦点步进(默认): 菜单/数值编辑/短列表用 */
static uint8_t UI_FocusStep(uint8_t focus, uint8_t num, uint8_t dir)
{
    return UI_FocusStepEx(focus, num, dir, 0U);
}

static char UI_SwChar(uint16_t v)
{
    if (v <= crsf_min)
        return (char)0x3C;          /* < */
    if (v >= crsf_max)
        return (char)0x3E;          /* > */
    return (char)0x2D;              /* - */
}

/* 打印一个开关状态, 例如 A:< (3 字符 = 18px) */
/* 打印电压(传 mV): 固定 width 个字符宽, 右对齐到 x+width*6, 例如 12.6V /   3.9V
   width 必须 <= 7(char s[9] 的余量); 手台电压用 4(见 UI_TxBat), 接收机电池按位数用 4/5/6(见 UI_RxBat).
   font 决定字高: 开机页用 afont8x6(8px), 主界面顶栏/底栏和中文 12px 字对齐时用 afont12x6.
   两种字体的字符宽度都是 6px, 所以 width 的含义不变 */
static void UI_Volt(uint8_t x, uint8_t y, uint16_t mv, uint8_t width, OLED_ColorMode color,
                    const ASCIIFont *font)
{
    char s[9];
    uint8_t n;
    uint8_t p;
    uint8_t need = 4U;                             /* "X.YV" 至少 4 个字符 */
    uint32_t d = ((uint32_t)mv + 50U) / 100U;      /* 0.1V 单位, 四舍五入 */
    uint32_t t;

    /* 整数部分(伏)每多一位, 字符串多 1 个字符; need 上限 8 = s[] 能放下的最大长度 */
    for (t = d / 10U; (t >= 10U) && (need < 8U); t /= 10U)
        need++;
    n = (width > need) ? width : need;             /* 至少要放得下这个数值 */
    /* 从右往左填: 'V' 在最右, 然后 0.1V 位、'.'、整数位(高位在左), 剩下的左边补空格.
       这样得到的才是 "12.6V", 不要"先正着写再整体反转"(那会把 V 转到最前面) */
    p = n;
    s[--p] = (char)0x56;                           /* "V" */
    s[--p] = (char)(0x30 + (char)(d % 10U));       /* 0.1V 位 */
    d /= 10U;
    s[--p] = (char)0x2E;                           /* "." */
    do { s[--p] = (char)(0x30 + (char)(d % 10U)); d /= 10U; } while ((d != 0U) && (p > 0U));
    while (p > 0U) s[--p] = (char)0x20;            /* 左侧补空格 -> 右对齐到 x+n*6 */
    s[n] = (char)0;
    OLED_PrintASCIIString(x, y, s, font, color);
}

/* 主界面状态行统一用 12px 高(中文 font12x12 + 数字 afont12x6, 底边对齐):
   顶栏 y=1..12 放链路状态和接收机电池, 底栏 y=51..62 放模型名和手台电池.
   开关标签在 y=16/32/48, 底栏只占中间 x=23..104, 两侧都留出空位, 不会压到 A~F 标签. */
#define UI_STATUS_Y 1U
#define UI_BOTTOM_Y 51U

/* 主界面顶栏右侧: 模块转发上来的接收机电池电压(0x08 电池传感器, 就是 EdgeTX/ELRS 里的 "RxBt"),
   例如 "回传 4.2V" / "回传 12.6V"; 没有该传感器/断流超时 -> "回传 --".
   参数 raw 是 0x08 帧里的原始电压计数(单位见 Crsf.h 的 CRSF_RXBAT_RAW_PER_V: 0.01V 一档, 410 = 4.10V).
   排版: 标签 "回传" 固定在 x=64(64~87); 电压右对齐到 125 列, 4 字符占 101~124,
   5 字符占 95~124, 6 字符占 89~124. 左侧 LQ 组占 2~43, 中间留 20px 左右, 不挤.
   标签和电压都用 12px 高字体(中文 font12x12 / 数字 afont12x6), 上下底边一致. */
#define UI_RXBAT_LABEL_X 64U
#define UI_RXBAT_X_END 125U     /* 电压右边缘(不含): 4 字符 x=101, 墨迹到 124 */
#define UI_RXBAT_X_MIN 89U      /* 6 字符左端: 标签墨迹到 87, 留 1px */
static void UI_RxBat(uint16_t raw, uint8_t ok, OLED_ColorMode color)
{
    uint32_t mv;
    uint16_t v;
    uint8_t  nd;
    uint8_t  x;

    OLED_PrintString(UI_RXBAT_LABEL_X, UI_STATUS_Y, "回传", &font12x12, color);
    if (ok == 0U)
    {
        /* "--" 占位符和数值一样右对齐: 2 字符 = 12px, 右边缘同样落在 125 */
        OLED_PrintASCIIString((uint8_t)(UI_RXBAT_X_END - 12U), UI_STATUS_Y, "--", &afont12x6, color);
        return;
    }

    mv = (uint32_t)raw * (1000U / CRSF_RXBAT_RAW_PER_V);   /* 原始计数 -> mV(0.01V 档时 = raw*10) */
    if (mv > 65535U)
        mv = 65535U;                                       /* 异常帧别把 mV 撑爆 */
    v  = (uint16_t)((mv + 50U) / 100U);                    /* 0.1V 单位, 只用来定显示位数 */
    nd = (uint8_t)((v >= 1000U) ? 6U : ((v >= 100U) ? 5U : 4U));   /* 电压字符数(含末尾 V) */
    x  = (uint8_t)(UI_RXBAT_X_END - (uint8_t)(nd * 6U));
    if (x < UI_RXBAT_X_MIN)
        x = UI_RXBAT_X_MIN;                                /* 位数多时保底: 不许贴标签 */
    UI_Volt(x, UI_STATUS_Y, (uint16_t)mv, nd, color, &afont12x6);
}

/* 主界面底栏右侧: 手台(遥控器)自己的电池电压, 显示成 "手台 6.6V" ——
   和上面的接收机电池 "回传 4.2V" 一眼就能分开.
   排版: 标签 "手台" 固定在 x=57(57~80), 电压固定 4 字符占 81~104;
   左边 x=23~52 是模型名 "模型n", 中间留 4px, 整组 x=23~104 居中(中心 63.5).
   手台电池是 10k+10k 分压后测的(见 Func.c 的 BATT_DIV_X1000), ADC 满量程也只有 6.6V ->
   电压永远是 4 个字符("X.YV"), 所以这里按固定 4 字符右对齐就够 */
#define UI_TXBAT_LABEL_X 57U
#define UI_TXBAT_VOLT_X  81U
static void UI_TxBat(OLED_ColorMode color)
{
    OLED_PrintString(UI_TXBAT_LABEL_X, UI_BOTTOM_Y, "手台", &font12x12, color);
    UI_Volt(UI_TXBAT_VOLT_X, UI_BOTTOM_Y, batt_mv, 4U, color, &afont12x6);
}
/* ========================== 开机动画 ========================== */
/* 上电阶段: 标题 + 电池电量条 + 四段进度框; 四格内容 = 电源键已按住时长(见 UI_BootAnim),
   第四格填满的同时 Schedule.c 把 PWR_EN 锁存 -> 松手不断电; 中途松手则进度清零 */
/* 倒角边框: 四角各切 cut 像素, 用 45 度斜线连接(比直角更圆润) */
static void UI_Border(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t cut, OLED_ColorMode c)
{
    uint8_t i;

    OLED_DrawLine((uint8_t)(x + cut), y, (uint8_t)(x + w - cut), y, c);                    /* 上 */
    OLED_DrawLine((uint8_t)(x + cut), (uint8_t)(y + h), (uint8_t)(x + w - cut), (uint8_t)(y + h), c);  /* 下 */
    OLED_DrawLine(x, (uint8_t)(y + cut), x, (uint8_t)(y + h - cut), c);                    /* 左 */
    OLED_DrawLine((uint8_t)(x + w), (uint8_t)(y + cut), (uint8_t)(x + w), (uint8_t)(y + h - cut), c);  /* 右 */
    for (i = 0U; i <= cut; i++)                                                            /* 四个角 */
    {
        OLED_SetPixel((uint8_t)(x + i),     (uint8_t)(y + cut - i),     c);
        OLED_SetPixel((uint8_t)(x + w - i), (uint8_t)(y + cut - i),     c);
        OLED_SetPixel((uint8_t)(x + i),     (uint8_t)(y + h - cut + i), c);
        OLED_SetPixel((uint8_t)(x + w - i), (uint8_t)(y + h - cut + i), c);
    }
}

/* 整屏方框: 1px 边框贴住屏幕最外圈(0,0)-(127,63), 把这一个页面圈起来.
   功能页面统一方角(只有开机界面和关机动画保留倒角, 见 UI_BootAnim / UI_PwrOffPage).
   注意: 必须在页面内容画完之后再画 —— 文字(反色模式)会把途经的像素擦黑,
         先画方框的话边框会被文字打出一串缺口. */
static void UI_ScreenFrame(void)
{
    OLED_DrawRectangle(0U, 0U, 127U, 63U, OLED_COLOR_NORMAL);
}

/* 倒角实心条(进度条填充): 两端各切 cut 像素 */
static void UI_FillChamfer(uint8_t x, uint8_t y, uint8_t len, uint8_t h, uint8_t cut, OLED_ColorMode c)
{
    uint8_t i;

    if (len == 0U) return;
    if (len < (uint8_t)(cut * 2U)) { OLED_DrawFilledRectangle(x, y, len, h, c); return; }
    for (i = 0U; i < len; i++)
    {
        uint8_t d = 0U;
        if (i < cut)                        d = (uint8_t)(cut - i);
        else if (i >= (uint8_t)(len - cut)) d = (uint8_t)(cut - (len - 1U - i));
        OLED_DrawLine((uint8_t)(x + i), (uint8_t)(y + d), (uint8_t)(x + i), (uint8_t)(y + h - 1U - d), c);
    }
}

#define UI_FRAME_CUT 4U                    /* 开机/关机整屏方框的倒角切角像素(0 = 直角) */
#define UI_READY_MS  150U                  /* 锁存后第四格填满 + READY 再多亮一会儿(ms) */

/* 四段开机进度框: 每段 = UI_BOOT_MS / 4, 按顺序填充; 每格是正方形.
   方框宽/高是绘图库的 w/h 参数(实际像素 = w+1 / h+1):
   W = H = SIDE-1 -> 空框实际 SIDE x SIDE; 填充传 H+1 -> 实心也是 SIDE x SIDE.
   15x15 正方形: 4*15 + 3*4 = 72px, 自动居中 -> 左右各留 28px, 框间距 4px */
#define UI_BOOT_SEG_N    4U
#define UI_BOOT_BOX_SIDE 15U
#define UI_BOOT_BOX_W    (UI_BOOT_BOX_SIDE - 1U)
#define UI_BOOT_BOX_H    (UI_BOOT_BOX_SIDE - 1U)
#define UI_BOOT_BOX_GAP  4U
#define UI_BOOT_BOX_X0   ((128U - (UI_BOOT_SEG_N * UI_BOOT_BOX_SIDE + (UI_BOOT_SEG_N - 1U) * UI_BOOT_BOX_GAP)) / 2U)
#define UI_BOOT_BOX_Y    48U

/* 长条 = 电池电量条: 1S Li-ion 3.3V(空) ~ 4.2V(满).
   本板 ADC 经 10k+10k 分压后满量程只有 6.6V; 若用 2S 电池, 请按实际
   电池节数改 UI_BATT_MAX_MV(注意 2S 高压段会被 ADC 削顶) */
#define UI_BATT_MIN_MV   3300U
#define UI_BATT_MAX_MV   4200U
#define UI_BATT_BAR_MAX  96U               /* 填充最大长度(与 UI_Border(13,34,101,12) 内圈匹配) */

/* 电池电压 -> 长条填充长度(0~96 px) */
static uint8_t UI_BattBarLen(void)
{
    uint16_t mv = batt_mv;

    if (mv <= UI_BATT_MIN_MV) return 0U;
    if (mv >= UI_BATT_MAX_MV) return (uint8_t)UI_BATT_BAR_MAX;
    return (uint8_t)(((uint32_t)(mv - UI_BATT_MIN_MV) * UI_BATT_BAR_MAX) /
                     (uint32_t)(UI_BATT_MAX_MV - UI_BATT_MIN_MV));
}

/* 开机动画: held = 电源键已按住时长(ms) = Schedule.c 的 pwr_hold_ms.
   四段进度框: 每段 UI_BOOT_MS/4, 按顺序填满; "第四格填满"与"PWR_EN 使能"同一时刻.
   长条: 不再显示按住进度, 改为电池电压电量条(右上角同时显示实际电压).
   held == 0 = 没按住(或中途松手已清零) -> 四个框全空. */
static void UI_BootAnim(uint32_t held)
{
    uint8_t seg;
    uint8_t i;

    if (held > UI_BOOT_MS) held = UI_BOOT_MS;
    seg = (uint8_t)((held * UI_BOOT_SEG_N) / UI_BOOT_MS);   /* 已完成 0~4 格 */

    OLED_NewFrame();

    OLED_PrintASCIIString(40, 2, "ZOE TX", &afont16x8, OLED_COLOR_NORMAL);
    if (HAL_GPIO_ReadPin(CHARGE_GPIO_Port, CHARGE_Pin) == GPIO_PIN_SET)
        OLED_PrintString(46, 20, "未充电", &font12x12, OLED_COLOR_NORMAL);
    else
        OLED_PrintString(46, 20, "充电中", &font12x12, OLED_COLOR_NORMAL);
    UI_Volt(102, 22, batt_mv, 4U, OLED_COLOR_NORMAL, &afont8x6);   /* 右上角: 实际电池电压, 右对齐到 125 */

    UI_Border(13, 34, 101, 12, 2, OLED_COLOR_NORMAL);      /* 电池电量条外框(倒角), 居中 */
    UI_FillChamfer(15, 36, UI_BattBarLen(), 9, 2, OLED_COLOR_NORMAL);

    /* 四段进度: 未到的段只画空框, 已完成的段填实心.
       空框实际 (W+1)x(H+1), 填充实际 (W+1)xH; 填充时传 H+1 -> 两者都是 SIDE x SIDE 正方形 */
    for (i = 0U; i < UI_BOOT_SEG_N; i++)
    {
        uint8_t x = (uint8_t)(UI_BOOT_BOX_X0 + (i * (UI_BOOT_BOX_W + 1U + UI_BOOT_BOX_GAP)));

        if (i < seg)
            OLED_DrawFilledRectangle(x, UI_BOOT_BOX_Y, UI_BOOT_BOX_W, UI_BOOT_BOX_H + 1U, OLED_COLOR_NORMAL);
        else
            OLED_DrawRectangle(x, UI_BOOT_BOX_Y, UI_BOOT_BOX_W, UI_BOOT_BOX_H, OLED_COLOR_NORMAL);
    }

    UI_Border(0U, 0U, 127U, 63U, UI_FRAME_CUT, OLED_COLOR_NORMAL);   /* 整屏大框: 开机界面保留倒角 */
}

/* 打印一个开关状态, 例如 A:< (3 字符 = 18px)
   mirror = 1: 只给右列(B/D/F)用 -> 把 < 和 > 对调, 让左右两列镜像对称(中间的 - 不动) */
static void UI_SwLine(uint8_t x, uint8_t y, char name, uint16_t v, uint8_t mirror)
{
    char s[4];
    char c = UI_SwChar(v);

    if (mirror != 0U)
    {
        if (c == (char)0x3C)      c = (char)0x3E;
        else if (c == (char)0x3E) c = (char)0x3C;
    }

    s[0] = name;
    s[1] = (char)0x3A;              /* : */
    s[2] = c;
    s[3] = (char)0;
    OLED_PrintASCIIString(x, y, s, &afont8x6, OLED_COLOR_NORMAL);
}

static void UI_StickPage(void)
{
    OLED_NewFrame();
    /* 顶行反色状态条: 先铺白底, 再用反色画字.
       整屏方框在最外圈(0/127, 0/63), 这里 x=2/y=1 起铺 124x12 -> 左右各留 1px 黑边;
       12px 高正好放 font12x12 中文和 afont12x6 数字, 下沿在 y=12, 离 y=16 的开关标签还有 3px.
       校准提示条也用同一块 (2,1,124,12), 覆盖时不会在左右留下白边 */
    OLED_DrawFilledRectangle(2, 1, 124, 12, OLED_COLOR_NORMAL);
    /* Mode2: 左摇杆 x=ch4(Rud) y=ch3(Thr), 右摇杆 x=ch1(Ail) y=ch2(Ele) */
    OLED_DrawJoystickEx(aetr[3], aetr[2], aetr[0], aetr[1], crsf_min, crsf_mid, crsf_max);

    /* 开关状态: 左列 A/C/E, 右列 B/D/F(左右对称)  < =172  - =992  > =1811
       右侧标签 x=108(3 字符宽 18px -> 墨迹到 125, 右留 1px 黑边)
       右列传 mirror=1: < 与 > 反过来显示(同一位置时两侧箭头呈镜像), 中间的 - 不变 */
    UI_SwLine(1, 16, (char)0x41, Switch_State[0], 0U);      /* A */
    UI_SwLine(1, 32, (char)0x43, Switch_State[2], 0U);      /* C */
    UI_SwLine(1, 48, (char)0x45, Switch_State[4], 0U);      /* E */
    UI_SwLine(108, 16, (char)0x42, Switch_State[1], 1U);    /* B 镜像 */
    UI_SwLine(108, 32, (char)0x44, Switch_State[3], 1U);    /* D 镜像 */
    UI_SwLine(108, 48, (char)0x46, Switch_State[5], 1U);    /* F 镜像 */

    if (ui_cal)
    {
        uint32_t el = HAL_GetTick() - ui_stamp;      /* 从开始校准算起 */
        uint32_t left;

        if (state != 2U)                             /* 阶段1: 采集摇杆中位(state 0/1) */
        {
            left = (el < 2000U) ? (2000U - el) : 0U;
            OLED_DrawFilledRectangle(2, 1, 124, 12, OLED_COLOR_NORMAL);
            OLED_PrintString(2, 1, "中心校准", &font12x12, OLED_COLOR_REVERSED);
            UI_SecC(68, 3, left, OLED_COLOR_REVERSED);
            OLED_DrawFilledRectangle(38, 51, 52, 12, OLED_COLOR_REVERSED);
            OLED_PrintString(40, 51, "摇杆回中", &font12x12, OLED_COLOR_NORMAL);
        }
        else                                         /* 阶段2: 采集行程 min/max */
        {
            left = (el < 8000U) ? (8000U - el) : 0U;
            OLED_DrawFilledRectangle(2, 1, 124, 12, OLED_COLOR_NORMAL);
            OLED_PrintString(2, 1, "行程校准", &font12x12, OLED_COLOR_REVERSED);
            UI_SecC(68, 3, left, OLED_COLOR_REVERSED);
            OLED_DrawFilledRectangle(22, 51, 84, 12, OLED_COLOR_REVERSED);
            OLED_PrintString(28, 51, "摇杆上下左右", &font12x12, OLED_COLOR_NORMAL);
        }
    }
    else if (ui_cal_ok && ((HAL_GetTick() - ui_stamp) < UI_MSG_MS))
    {
        OLED_DrawFilledRectangle(2, 1, 124, 12, OLED_COLOR_NORMAL);
        OLED_PrintString(2, 1, "校准完成保存", &font12x12, OLED_COLOR_REVERSED);
        UI_NumC(80, 2, store_sv_ok, 1, OLED_COLOR_REVERSED);
        if (store_sv_ok == 0U)                 /* 保存失败: 显示 HAL 错误码 */
        {
            OLED_PrintASCIIString(92, 2, "E:", &afont8x6, OLED_COLOR_REVERSED);
            UI_NumC(104, 2, store_err, 3, OLED_COLOR_REVERSED);
        }
        else if (adc_cal_bad)
            OLED_PrintString(90, 1, "行程低", &font12x12, OLED_COLOR_REVERSED);
    }
    else if ((HAL_GetTick() - ui_boot) < 3000U)         /* 上电 3 秒内: 显示是否读到校准 */
    {
        OLED_DrawFilledRectangle(2, 1, 124, 12, OLED_COLOR_NORMAL);
        OLED_PrintString(2, 1, "读取校准", &font12x12, OLED_COLOR_REVERSED);
    }
    /* 只要模块在回传(超时内有 0x14)就显示 LQ, 即使 LQ = 0 也照常显示, 末尾加 '*' 表示
       "链路通着, 但还没跟接收机对上". 以前 LQ = 0 就把整行换成 CONNECTING, 于是对频中/
       失联时连模块已经在回的接收机电池都看不见, 看着像卡死.
       排版: "LQ:" + 3 位数值 + '*' 占 x=2~43(12px 字高, 与右边 "回传" 底边对齐),
       右边 x=64~124 是接收机电池 "回传 4.2V"; 手台电池移到底栏, 顶栏不再挤三组 */
    else if (crsf_link_ok)
    {
        OLED_PrintASCIIString(2, UI_STATUS_Y, "LQ:", &afont12x6, OLED_COLOR_REVERSED);
        UI_NumD(20, UI_STATUS_Y, (int32_t)crsf_link_stats.uplink_lq, 3, OLED_COLOR_REVERSED);
        if (crsf_link_stats.uplink_lq == 0U)
            OLED_PrintASCIIString(38, UI_STATUS_Y, "*", &afont12x6, OLED_COLOR_REVERSED);
        UI_RxBat(crsf_rx_bat_raw, crsf_rx_bat_ok, OLED_COLOR_REVERSED);   /* 右侧: 接收机电池 */
    }
    else
    {
        /* 连接中: 上电还没收到 0x14 / 断流超时(模块没在说话).
           末尾 3 个点每 250ms 涨一个 -> "正在连接"一眼能看出是活的; 一收到 0x14 就换成上面的 LQ 显示.
           排版: "连接中" 3 个 12px 汉字 = 36px(2~37), 点从 x=38 起(38~55);
           右侧仍显示 "回传 --"(模块没回传时没有接收机电压), 手台电压在底栏 */
        char    dot[4];
        uint8_t n = (uint8_t)((HAL_GetTick() / 250U) % 4U);

        OLED_PrintString(2, UI_STATUS_Y, "连接中", &font12x12, OLED_COLOR_REVERSED);
        dot[0] = (n > 0U) ? (char)0x2E : (char)0x20;
        dot[1] = (n > 1U) ? (char)0x2E : (char)0x20;
        dot[2] = (n > 2U) ? (char)0x2E : (char)0x20;
        dot[3] = (char)0;
        OLED_PrintASCIIString(38, UI_STATUS_Y, dot, &afont12x6, OLED_COLOR_REVERSED);
        UI_RxBat(0U, 0U, OLED_COLOR_REVERSED);          /* 右侧: 回传 -- */
    }

    /* ---------------------- 底部行(y=51, 12px 高) ----------------------
       摇杆图形只画到 y=50, 这一行正好放 12px 中文; 左右两侧 x<=18 / x>=108 是
       E/F 开关标签(y=48~55), 所以内容只占中间的 x=23~104, 不会压字.
       左边: 模型名 "模型n"; 低压时每 500ms 交替成 "低压";
       右边: 手台电池 "手台 6.6V". 校准中不画: 校准提示条摆在 y=51, 会盖住这一行 */
    if (ui_cal == 0U)
    {
        if ((batt_mv < BATT_LOW_MV) && (((HAL_GetTick() / 500U) & 1U) != 0U))
        {
            /* "低压" 2 个 12px 汉字 = 24px, 占模型名的左半格(x=23~46) */
            OLED_PrintString(23, UI_BOTTOM_Y, "低压", &font12x12, OLED_COLOR_NORMAL);
        }
        else
        {
            /* 当前模型名(与模型页同一套叫法, 1~MODEL_MAX 现 8): "模型" 24px + 1 位数字 6px */
            OLED_PrintString(23, UI_BOTTOM_Y, "模型", &font12x12, OLED_COLOR_NORMAL);
            UI_NumD(47, UI_BOTTOM_Y, (int32_t)(model_cur + 1U), 1, OLED_COLOR_NORMAL);
        }
        UI_TxBat(OLED_COLOR_NORMAL);                    /* 右侧: 手台电池 "手台 6.6V" */
    }
}
/* ========================== 对频命令帧(只有官方一种写法) ==========================
   手台->模块的 0x32 命令帧必须和官方模块的实现一致(格式见 Crsf.h):
     C8 07 32 EE EA 10 01 14 EB
     (0xC8 = CRSF_SYNC_BYTE, dest = 0xEE 模块, orig = 0xEA 手台, 0x10 = SUBCMD_RX, 0x01 = RX_BIND)
   只有这一种: 广播目的地址(0x00)/GENERAL 子命令(0x0A)/单 CRC 这些变体都是猜出来的,
   发出去就是模块眼里的坏包 -> 模块 UART 看门狗(Bad >= Good)断开重扫波特率(noCrossfire),
   模块彻底静默且完全不发射 RF(现象: 接收机永远连不上, 一直卡在等 TX).
   所以变体表整体删除, 只留下面三个常量 */
#define UI_BIND_DEST 0xEEU   /* 目的地址: 模块自己(必须是 0xEE, 广播不会被 processInternalCrsfPackage 处理) */
#define UI_BIND_SUB1 0x10U   /* CRSF_COMMAND_SUBCMD_RX */
#define UI_BIND_SUB2 0x01U   /* ..._RX_BIND(对频) */
#define UI_BIND_POPUP_MS 6000U   /* 对频状态弹窗显示时长: 超过 6 秒自动关闭 */

/* 后台参数表扫描状态(参数页已摘掉, 这里只维持扫描/ping, 供诊断页看 F 标志) */
static uint32_t ui_param_ping_at = 0U;   /* 上次 ping / 参数表重扫时间 */
static uint8_t  ui_param_tries   = 0U;   /* 后台自动重扫参数表的次数(避免模块不支持时一直重发) */

/* ========================== 第1页: 菜单 ========================== */
static const UI_MenuItem_t *UI_MenuItems(uint8_t *count)
{
    if (ui_menu == 1U) { *count = 5U; return ui_items_sys;  }   /* 校准/输入映射/微调分配/版本/返回 */
    *count = 4U;
    return ui_items_root;
}

static void UI_MenuPage(void)
{
    uint8_t count = 0U;
    const UI_MenuItem_t *items = UI_MenuItems(&count);

    OLED_NewFrame();

    if (ui_menu == 0U)
        OLED_PrintString(1, 1, "菜单", &font12x12, OLED_COLOR_NORMAL);
    else
        OLED_PrintString(1, 1, "系统", &font12x12, OLED_COLOR_NORMAL);

    {
        uint8_t first = 0U;
        uint8_t shown = count;
        uint8_t r;
        /* 屏幕 64px 高: 标题占 1~12, 正文只放 4 行(y=12/24/36/48 -> 墨迹到 59, 不压底边框).
           原来的 6 行会让第 5/6 行被屏幕下沿裁掉. 超过 4 条时以光标为中心的滚动窗口 */
        if (count > 4U)
        {
            if (ui_cursor > 1U)
                first = (uint8_t)(ui_cursor - 1U);
            if ((uint8_t)(first + 4U) > count)
                first = (uint8_t)(count - 4U);
            shown = 4U;
        }
        for (r = 0U; r < shown; r++)
        {
            uint8_t idx = (uint8_t)(first + r);
            uint8_t y   = (uint8_t)(12U + (r * 12U));

            if (idx == ui_cursor)                                   /* 当前项: 整行反色 */
            {
                OLED_DrawFilledRectangle(2, y, 124, 12, OLED_COLOR_NORMAL);
                OLED_PrintASCIIString(2, y, ">", &afont12x6, OLED_COLOR_REVERSED);
                OLED_PrintString(10, y, (char *)items[idx].label, &font12x12, OLED_COLOR_REVERSED);
            }
            else
            {
                OLED_PrintString(10, y, (char *)items[idx].label, &font12x12, OLED_COLOR_NORMAL);
            }
        }
    }

}

/* ========================== 菜单动作 ========================== */
static void UI_DoAction(uint8_t action)
{
    switch (action)
    {
        case UI_A_SYS:
            ui_menu = 1U;
            ui_cursor = 0U;
            break;

        case UI_A_BACK:
            if (ui_menu != 0U) { ui_menu = 0U; ui_cursor = 0U; }
            else ui_page = UI_PAGE_STICK;
            break;

        case UI_A_CAL:                               /* 启动用户自己的校准状态机 */
            state     = 0U;
            ui_cal    = 1U;
            ui_cal_ok = 0U;
            ui_stamp  = HAL_GetTick();
            ui_page   = UI_PAGE_STICK;
            break;

        case UI_A_MIX:                               /* 混控通道列表 */
            ui_mix_ch = 0U; ui_mix_focus = 0U; ui_mix_edit = 0U; ui_mix_press = 0U;
            ui_page = UI_PAGE_MIX;
            break;

        case UI_A_MODEL:                             /* 模型切换页 */
            ui_page = UI_PAGE_MODEL;
            break;


        case UI_A_INPUT:                             /* 通道方向(±1)编辑页 */
            ui_page   = UI_PAGE_INPUT;
            ui_in_focus = 0U;
            ui_in_edit  = 0U;
            ui_cursor = 0U;
            break;

        case UI_A_TRIM:                              /* 微调分配页: trim1~4 -> Ail/Ele/Thr/Rud */
            ui_tr_focus = 0U;
            ui_tr_edit  = 0U;
            ui_page     = UI_PAGE_TRIMSET;
            break;

        case UI_A_VER:                               /* 版本页: 纯静态, 只认 EXIT */
            ui_page = UI_PAGE_VER;
            break;

        case UI_A_CHAN:
            ui_page = UI_PAGE_CHAN;
            break;

        case UI_A_DIAG:
            ui_page = UI_PAGE_DIAG;
            break;

        case UI_A_BIND:                              /* 对频: 发官方命令, 同时弹出 6 秒状态提示 */
            ui_bind_popup = 1U;
            ui_bind_stamp = HAL_GetTick();
            CRSF_SetCmdCrcForm(1U);                  /* 只发官方写法: C8 07 32 EE EA 10 01 14 EB */
            CRSF_SendCommandTo(UI_BIND_DEST, UI_BIND_SUB1, UI_BIND_SUB2, NULL, 0U);
            break;

        default:
            break;
    }
}

/* ========================== 第5页: 16 通道监视 ========================== */
/* 通道最终值(已含混控, 已限幅): 由 Mix_Apply() 每 5ms 更新 */
static uint16_t UI_ChVal(uint8_t n)
{
    return (n < 16U) ? (uint16_t)ch_out[n] : (uint16_t)crsf_mid;
}
/* ========================== 通道进度条页(不显示数值) ========================== */
/* 16 个通道各占 4px 高(实绘 3px), 条长 = 通道值在 172~1811 区间内的比例(满格 128px) */
static void UI_ChBarPage(void)
{
    uint8_t i;

    OLED_NewFrame();
    for (i = 0U; i < 16U; i++)
    {
        uint16_t v   = UI_ChVal(i);
        uint8_t  col = (uint8_t)(i / 8U);                    /* 0=左栏 ch1~8, 1=右栏 ch9~16 */
        uint8_t  row = (uint8_t)(i % 8U);
        /* 左栏 x=1, 右栏 x=64(右栏条形 90..125, 右留 1px 黑边) */
        uint8_t  lx  = (col == 0U) ? 1U : 64U;               /* 单元起点 */
        uint8_t  bx  = (uint8_t)(lx + 26U);                  /* 条形起点 */
        /* 首行 y=1(不压上边框), 末行 y=55(不压下边框), 中间行距不变 */
        uint8_t  y   = (row == 0U) ? 1U : ((row == 7U) ? 55U : (uint8_t)(row * 8U));
        uint32_t len;
        uint8_t  x;

        if (v < crsf_min) v = crsf_min;
        if (v > crsf_max) v = crsf_max;
        len = ((uint32_t)(v - crsf_min) * 36U) / (uint32_t)(crsf_max - crsf_min);   /* 满格 36px */
        if (len < 2U) len = 2U;

        /* 通道号: ch + 两位数字(右对齐) */
        OLED_PrintASCIIString(lx, y, "ch", &afont8x6, OLED_COLOR_NORMAL);
        UI_Num((uint8_t)(lx + 12U), y, (int32_t)(i + 1U), 2);

        /* 实心条(5px) + 剩余刻度的点线 */
        OLED_DrawFilledRectangle(bx, (uint8_t)(y + 1U), (uint8_t)len, 5, OLED_COLOR_NORMAL);
        for (x = (uint8_t)(bx + len); x < (uint8_t)(bx + 36U); x = (uint8_t)(x + 2U))
            OLED_DrawLine(x, (uint8_t)(y + 6U), x, (uint8_t)(y + 6U), OLED_COLOR_NORMAL);
    }
}

/* ========================== 第8页: 输入方向(摇杆正反/分配) ========================== */
/* aetr_input[0..3] 对应前 4 个 CRSF 通道: +1 正向 / -1 反向(围绕中位镜像).
   作用点: Adc_convert_Crsf() 里 ADC 校准换算之后; 后期混控也以这四个 input 为源.
   UP/DN 选择, ENTER 翻转, EXIT = 保存到 Flash 并返回菜单 */
/* ========================== 第8页: 输入分配(源 + 正反) ========================== */
/* 左列 Ail/Ele/Thr/Rud 四个输出; 每行右侧两格, 共 8 个可编辑项:
     SRC: 该输出取哪一路 ADC 输入(IN1~IN4)   DIR: +1 正向 / -1 反向(围绕中位镜像)
   焦点态: UP/DN 循环移动焦点, ENTER 进编辑, EXIT 保存并返回菜单
   编辑态: UP/DN 改值,      ENTER 退出编辑(保存), EXIT 保存并返回菜单 */

static void UI_InputPage(void)
{
    static const char *nm[4] = { "Ail", "Ele", "Thr", "Rud" };
    uint8_t i;

    OLED_NewFrame();
    OLED_PrintString(1, 1, "输出  输入  方向", &font12x12, OLED_COLOR_NORMAL);

    for (i = 0U; i < 4U; i++)
    {
        uint8_t y  = (uint8_t)(12U + (i * 8U));
        uint8_t sc = (uint8_t)(i * 2U);                 /* SRC 格编号 */
        uint8_t dc = (uint8_t)(sc + 1U);                /* DIR 格编号 */
        char    s[8];

        s[0] = nm[i][0]; s[1] = nm[i][1]; s[2] = nm[i][2]; s[3] = (char)0;
        OLED_PrintASCIIString(1, y, s, &afont8x6, OLED_COLOR_NORMAL);

        s[0] = (char)0x49; s[1] = (char)0x4E;           /* "IN" */
        s[2] = (char)(0x31 + aetr_src[i]);
        s[3] = (char)0;
        if (ui_in_focus == sc)
            OLED_DrawFilledRectangle(40, y, 18, 8, OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(40, y, s, &afont8x6, (ui_in_focus == sc) ? OLED_COLOR_REVERSED : OLED_COLOR_NORMAL);

        s[0] = (aetr_input[i] < 0) ? (char)0x2D : (char)0x2B;
        s[1] = (char)0x31; s[2] = (char)0;
        if (ui_in_focus == dc)
            OLED_DrawFilledRectangle(84, y, 12, 8, OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(84, y, s, &afont8x6, (ui_in_focus == dc) ? OLED_COLOR_REVERSED : OLED_COLOR_NORMAL);
    }

    OLED_PrintString(1, 51, "确认/退出 保存" , &font12x12, OLED_COLOR_NORMAL);
}



/* ========================== 第10页: 模型选择 ========================== */
/* UP/DN 切换模型: 先把当前模型的混控整条存回 Flash, 再按新模型从 Flash 过滤到 RAM;
   EXIT 保存并返回菜单。右上角显示 模型号/总数, 正文用 "模型1"~"模型8", 上下键慢速连发.
   所有行统一从 x=2 起; 右上角和 "混控:" 的数字都用 afont12x6(12px 高),
   和左边 font12x12 中文底边对齐, 不再出现 8px 数字顶着标题上沿的情况 */
static void UI_ModelPage(void)
{
    OLED_NewFrame();
    OLED_PrintString(2, 1, "模型", &font12x12, OLED_COLOR_NORMAL);

    /* 右上角: 当前模型号 / 模型总数 (MODEL_MAX <= 9 时各 1 位, 共 3 字符 = 18px,
       右对齐到 x=124; 12px 高, 与标题底边对齐) */
    UI_NumD(107, 1, (int32_t)(model_cur + 1U), 1, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(113, 1, "/", &afont12x6, OLED_COLOR_NORMAL);
    UI_NumD(119, 1, (int32_t)MODEL_MAX, 1, OLED_COLOR_NORMAL);

    /* 正文: 上下键选的模型名 "模型1" ~ "模型8", 反色条 = 当前选择 */
    OLED_DrawFilledRectangle(2, 16, 62, 12, OLED_COLOR_NORMAL);
    OLED_PrintString(2, 16, "模型", &font12x12, OLED_COLOR_REVERSED);
    UI_NumD(38, 16, (int32_t)(model_cur + 1U), 1, OLED_COLOR_REVERSED);

    /* "混控" 24px + 冒号 6px + 3 位计数 18px, 全部 12px 高.
       冒号单独用 afont12x6, 避免 font12x12 的 ASCII 缺省字体(16x8)把冒号拉高 4px */
    OLED_PrintString(2, 32, "混控", &font12x12, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(26, 32, ":", &afont12x6, OLED_COLOR_NORMAL);
    UI_NumD(36, 32, (int32_t)mix_ram_cnt, 3, OLED_COLOR_NORMAL);

    /* 底部提示分两段打印, 中间留 8px, 避免空格走 16x8 ASCII 缺省字体 */
    OLED_PrintString(2, 51, "退出", &font12x12, OLED_COLOR_NORMAL);
    OLED_PrintString(34, 51, "保存", &font12x12, OLED_COLOR_NORMAL);
}

/* ========================== 第11/12页: 混控 ========================== */
/* 列表页: 16 个通道(":" 后 IN 的数字 = 该通道已分配的输入源个数), UP/DN 慢速移动焦点,
           ENTER 进该通道的混控编辑, EXIT 返回菜单
   编辑页: 每行 = 一个输入源(左格选源 IN1~IN4 / A~F, 右格权重 -100~+100 步进 5),
           未满 4 个时下方居中显示 "+" 添加; 焦点态 UP/DN 移动, ENTER 进编辑,
           在行上"长按 ENTER"删除(后面的补到前面); 编辑态 UP/DN 改值, ENTER 退出保存;
           标题行 = MIX CH n INPUT m/4 (m = 本通道输入源个数, 上限 4) */
static const char *UI_MixSrcName(uint8_t src)
{
    static const char *nm[MIX_SRC_NUM] = { "Ail", "Ele", "Thr", "Rud", "SA", "SB", "SC", "SD", "SE", "SF" };   /* 0~3 = aetr 四通道, 4~9 = 开关 A~F */
    return (src < MIX_SRC_NUM) ? nm[src] : nm[0];
}

/* 编辑器有效格数: 每个源 2 格(源/权重), 未满 MIX_PER_CH 时再加 1 个 "+" */
static uint8_t UI_MixCellNum(uint8_t ch)
{
    uint8_t c = Mix_ChCount(ch);
    uint8_t n = (uint8_t)(c * 2U);
    if (c < MIX_PER_CH) n++;
    return (n == 0U) ? 1U : n;
}

static void UI_MixPage(void)
{
    uint8_t i;

    OLED_NewFrame();
    for (i = 0U; i < 8U; i++)
    {
        /* 首行 y=1(不压上边框), 末行 y=55(不压下边框), 中间行距不变 */
        uint8_t y  = (i == 0U) ? 1U : ((i == 7U) ? 55U : (uint8_t)(i * 8U));
        uint8_t b  = (uint8_t)(i + 8U);
        uint8_t fa = (i == ui_mix_ch) ? 1U : 0U;
        uint8_t fb = (b == ui_mix_ch) ? 1U : 0U;

        if (fa) OLED_DrawFilledRectangle(2, y, 58, 8, OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(2, y, "CH", &afont8x6, fa ? OLED_COLOR_REVERSED : OLED_COLOR_NORMAL);
        UI_NumC(14, y, (int32_t)(i + 1U), 2, fa ? OLED_COLOR_REVERSED : OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(26, y, ":", &afont8x6, fa ? OLED_COLOR_REVERSED : OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(38, y, "IN", &afont8x6, fa ? OLED_COLOR_REVERSED : OLED_COLOR_NORMAL);
        UI_NumC(50, y, (int32_t)Mix_ChCount((uint8_t)(i + 1U)), 1, fa ? OLED_COLOR_REVERSED : OLED_COLOR_NORMAL);

        if (fb) OLED_DrawFilledRectangle(64, y, 62, 8, OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(64, y, "CH", &afont8x6, fb ? OLED_COLOR_REVERSED : OLED_COLOR_NORMAL);
        UI_NumC(76, y, (int32_t)(b + 1U), 2, fb ? OLED_COLOR_REVERSED : OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(88, y, ":", &afont8x6, fb ? OLED_COLOR_REVERSED : OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(100, y, "IN", &afont8x6, fb ? OLED_COLOR_REVERSED : OLED_COLOR_NORMAL);
        UI_NumC(112, y, (int32_t)Mix_ChCount((uint8_t)(b + 1U)), 1, fb ? OLED_COLOR_REVERSED : OLED_COLOR_NORMAL);
    }
}

static void UI_MixEditPage(void)
{
    uint8_t ch = (uint8_t)(ui_mix_ch + 1U);
    uint8_t c  = Mix_ChCount(ch);
    uint8_t i;

    OLED_NewFrame();

    OLED_PrintString(1, 1, "输入源", &font12x12, OLED_COLOR_NORMAL);
    OLED_PrintString(40, 1, "权重", &font12x12, OLED_COLOR_NORMAL);
    UI_Num(106, 1, (int32_t)c, 1);
    OLED_PrintASCIIString(114, 1, "/4", &afont12x6, OLED_COLOR_NORMAL);

    for (i = 0U; i < c; i++)
    {
        uint8_t y  = (uint8_t)(16U + (i * 8U));
        uint8_t f0 = (uint8_t)(i * 2U);
        uint8_t f1 = (uint8_t)(f0 + 1U);
        Mix_t  *m  = Mix_ChAt(ch, i);
        const char *nm;
        char s[8];

        if (m == 0) break;
        nm = UI_MixSrcName(m->src);

        UI_Num(1, y, (int32_t)(i + 1U), 1);

        s[0] = (nm[0] != 0) ? nm[0] : (char)0x20;
        s[1] = (nm[1] != 0) ? nm[1] : (char)0x20;
        s[2] = (nm[2] != 0) ? nm[2] : (char)0x20;
        s[3] = (char)0;
        if (ui_mix_focus == f0) OLED_DrawFilledRectangle(12, y, 24, 8, OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(12, y, s, &afont8x6, (ui_mix_focus == f0) ? OLED_COLOR_REVERSED : OLED_COLOR_NORMAL);

        s[0] = (m->weight < 0) ? (char)0x2D : (char)0x2B;
        s[1] = (char)0;
        if (ui_mix_focus == f1) OLED_DrawFilledRectangle(40, y, 24, 8, OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(40, y, s, &afont8x6, (ui_mix_focus == f1) ? OLED_COLOR_REVERSED : OLED_COLOR_NORMAL);
        UI_NumC(46, y, (int32_t)((m->weight < 0) ? -m->weight : m->weight), 3, (ui_mix_focus == f1) ? OLED_COLOR_REVERSED : OLED_COLOR_NORMAL);
    }

    if (c < MIX_PER_CH)                    /* 下一行正中: "+" 添加 */
    {
        uint8_t y  = (uint8_t)(16U + (c * 8U));
        uint8_t fi = (uint8_t)(c * 2U);
        if (ui_mix_focus == fi) OLED_DrawFilledRectangle(58, y, 12, 8, OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(61, y, "+", &afont8x6, (ui_mix_focus == fi) ? OLED_COLOR_REVERSED : OLED_COLOR_NORMAL);
    }

    if (ui_mix_edit)
        OLED_PrintString(1, 51, "上下修改 确认保存", &font12x12, OLED_COLOR_NORMAL);
    else
        OLED_PrintString(1, 51, "长按删除  短按编辑", &font12x12, OLED_COLOR_NORMAL);
}

static void UI_ChanPage(void)
{
    uint8_t i;

    OLED_NewFrame();

    /* 装饰: 中缝长虚线, 左右两栏分隔 */
    for (i = 0U; i < 64U; i = (uint8_t)(i + 16U))
        OLED_DrawLine(61, i, 61, (uint8_t)(i + 11U), OLED_COLOR_NORMAL);

    for (i = 0U; i < 8U; i++)
    {
        /* 首行 y=1(不压上边框), 末行 y=55(不压下边框), 中间行距不变 */
        uint8_t y = (i == 0U) ? 1U : ((i == 7U) ? 55U : (uint8_t)(i * 8U));
        uint8_t b = (uint8_t)(i + 8U);

        /* 左栏 ch1~ch8 */
        OLED_PrintASCIIString(1, y, "CH", &afont8x6, OLED_COLOR_NORMAL);
        UI_Num(13, y, (int32_t)(i + 1U), 1);
        OLED_PrintASCIIString(19, y, ":", &afont8x6, OLED_COLOR_NORMAL);
        UI_Num(25, y, (int32_t)UI_ChVal(i), 4);

        /* 右栏 ch9~ch16 (通道号右对齐, 冒号对齐; 数值 4 位宽 24px, 从 102 起 -> 墨迹到 125) */
        OLED_PrintASCIIString(75, y, "CH", &afont8x6, OLED_COLOR_NORMAL);
        UI_Num(87, y, (int32_t)(b + 1U), 2);
        OLED_PrintASCIIString(99, y, ":", &afont8x6, OLED_COLOR_NORMAL);
        UI_Num(102, y, (int32_t)UI_ChVal(b), 4);
    }
}
/* ========================== 版本页 ========================== */
/* 中文走 font.c 里已取模的 12x12 字库(font12x12, 版本信息/交/流/群 都有);
   QQ 和号码用 afont12x6(每字 6px、12px 高), 与中文同高同基线:
   "QQ"(12px) + "交流群"(36px) + 10 位号码(60px) = 108px,
   居中 x=(128-108)/2=10 -> x: 10~21 / 22~57 / 58~117, 两边各留 10px.
   注意不能把 "QQ" / 号码混进 font12x12 的 OLED_PrintString: 它的 ASCII 缺省字体是
   afont16x8(每个 ASCII 8px 宽、16px 高), 会又宽又高, 破坏对齐.
   右上角 y=5 印硬件版本标识(HW:LATCH / HW:SW, 见 BoardCfg.h), 8px 高,
   底边落在 y=12, 与左边 12px 的标题底边对齐, 只占顶行右侧, 不干扰下面布局 */
static void UI_VerPage(void)
{
    OLED_NewFrame();

    OLED_PrintString(2, 1, "版本信息", &font12x12, OLED_COLOR_NORMAL);

    /* 右上角: 本固件对应的硬件版本(见 BoardCfg.h 的 BOARD_HW_TAG)
       "HW:LATCH" = 长按开关机版, "HW:SW" = 自锁开关版. 纯防呆用: 两版固件
       不通用(自锁版固件没人拉 PWR_EN, 长按版固件会一直等按键), 烧完扫一眼就知道对不对.
       右对齐: x = 126 - 6*字符数 -> 8 字符(HW:LATCH) x=78, 5 字符(HW:SW) x=96,
       最后一个字的墨迹都停在 124, 不压 127 的右边框, 也碰不到左边标题(墨迹到 49) */
    {
        uint8_t x = (uint8_t)(126U - (6U * (uint8_t)(sizeof(BOARD_HW_TAG) - 1U)));
        OLED_PrintASCIIString(x, 5, BOARD_HW_TAG, &afont8x6, OLED_COLOR_NORMAL);
    }

    /* 第1行: 版本号. afont16x8 每字 8px -> 10 字 80px, 居中: x = (128-80)/2 = 24 */
    OLED_PrintASCIIString(24, 20, "ZOE_TX_1.0", &afont16x8, OLED_COLOR_NORMAL);

    /* 第2行: 交流群号码. 三段都是 12px 高, 统一 y=44, 底边自然对齐
       (x: 10~21 / 22~57 / 58~117, 见上面注释) */
    OLED_PrintASCIIString(10, 44, "QQ",         &afont12x6, OLED_COLOR_NORMAL);
    OLED_PrintString(22, 44, "交流群",          &font12x12, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(58, 44, "1125415933", &afont12x6, OLED_COLOR_NORMAL);

}

/* ========================== 关机动画 ========================== */
/* 长按电源键 = 软关机: 锁存后再按住 PWR_OFF_MS(3s) 才真断电(见 Schedule.c 的 power_detect),
   期间整屏切成黑底"关机动画": 四段正方形进度框 + 标题/返回提示, 风格与开机界面一致.
   画面在 pwr_off_ms >= UI_PWROFF_SHOW_MS 才出现, 所以四格从"刚出现"开始由空到满:
   每格约 (PWR_OFF_MS - UI_PWROFF_SHOW_MS) / 4 = 500ms, 用向上取整让第 4 格在
   2.5s 左右点亮, 留约 0.5s 满格提示(否则 3s 到点直接断电, 第 4 格永远来不及亮).
   松手 -> pwr_off_ms = 0 -> 立即退回原页面 = 取消.
   3s 到点真正软关机后 pwr_latched = 0: 若插着 USB(板子仍供电), UI_Task 的未锁存分支
   会切回开机界面(四个空框), 不会停在关机界面; 没 USB 时电池断电, 屏幕自然熄灭.
   注意: 本页是整屏自绘, 所以调用点必须在 UI_DrawPage/UI_ScreenFrame 之前(见 UI_Task),
         期间直接 return, 不叠加普通页面方框. 黑底由 OLED_NewFrame() 清屏得到,
         文字/方框/进度都用 OLED_COLOR_NORMAL(黑底白字).
   绘图库坐标: 空框 OLED_DrawRectangle(x,y,W,H) 实际 (W+1)x(H+1); 填充用
   OLED_DrawFilledRectangle(x,y,W,H+1) 也是 (W+1)x(H+1) = 正方形. */
#define UI_PWROFF_SHOW_MS 1000U                 /* 长按超过这么久才切关机画面(ms) */

/* 四段关机进度框: 20x20 正方形, 间距 4px -> 4*20+3*4=92px, 自动居中 x=18.
   y=24~43; 标题 y=6~17, 底部 "返回" y=48~59, 与整屏倒角框(0~63)留出边距. */
#define UI_PWROFF_SEG_N    4U
#define UI_PWROFF_BOX_SIDE 20U
#define UI_PWROFF_BOX_W    (UI_PWROFF_BOX_SIDE - 1U)
#define UI_PWROFF_BOX_H    (UI_PWROFF_BOX_SIDE - 1U)
#define UI_PWROFF_BOX_GAP  4U
#define UI_PWROFF_BOX_X0   ((128U - (UI_PWROFF_SEG_N * UI_PWROFF_BOX_SIDE + (UI_PWROFF_SEG_N - 1U) * UI_PWROFF_BOX_GAP)) / 2U)
#define UI_PWROFF_BOX_Y    24U

static void UI_PwrOffPage(void)
{
    uint32_t off  = pwr_off_ms;
    uint32_t span = PWR_OFF_MS - UI_PWROFF_SHOW_MS;
    uint8_t  seg;
    uint8_t  i;

    if (off < UI_PWROFF_SHOW_MS) off = UI_PWROFF_SHOW_MS;
    if (off > PWR_OFF_MS)        off = PWR_OFF_MS;

    /* 关机画面可见时长 -> 0~4 格; 向上取整, 保证第 4 格在断电前亮起 */
    seg = (uint8_t)(((off - UI_PWROFF_SHOW_MS) * UI_PWROFF_SEG_N + span - 1U) / span);

    OLED_NewFrame();                                                        /* 黑底 */

    /* 4 个 12px 汉字 = 48px -> x=(128-48)/2=40 */
    OLED_PrintString(40, 6, "长按关机", &font12x12, OLED_COLOR_NORMAL);

    /* 四段进度: 未到的段只画空框, 已完成的段填实心; 两者实际都是 20x20 正方形 */
    for (i = 0U; i < UI_PWROFF_SEG_N; i++)
    {
        uint8_t x = (uint8_t)(UI_PWROFF_BOX_X0 + (i * (UI_PWROFF_BOX_W + 1U + UI_PWROFF_BOX_GAP)));

        if (i < seg)
            OLED_DrawFilledRectangle(x, UI_PWROFF_BOX_Y, UI_PWROFF_BOX_W, UI_PWROFF_BOX_H + 1U, OLED_COLOR_NORMAL);
        else
            OLED_DrawRectangle(x, UI_PWROFF_BOX_Y, UI_PWROFF_BOX_W, UI_PWROFF_BOX_H, OLED_COLOR_NORMAL);
    }

    OLED_PrintString(52, 48, "返回", &font12x12, OLED_COLOR_NORMAL);        /* 2 字 24px -> 居中 */

    UI_Border(0U, 0U, 127U, 63U, UI_FRAME_CUT, OLED_COLOR_NORMAL);          /* 整屏倒角框(与开机界面一致) */
}

/* ========================== 对频状态弹窗 ========================== */
/* 菜单里按 "对频" 后覆盖当前页 6 秒, 显示标题 + 倒计时 + 当前链路状态.
   状态用现有回传数据推断(不新增 CRSF 解析):
     - 模块没回 0x14          -> "未连接"
     - 模块在回但 LQ = 0      -> "对频中"(模块已通, 还没和接收机对上)
     - LQ > 0                 -> "对频完成"
   弹窗只用 zh12x12 已有的字模(对/频/未/连/接/中/完/成), 不依赖补字库.
   绘制顺序: 先在页面内容上盖白底, 再套一圈黑框(反色), 文字也用反色 -> 白底黑字. */
static void UI_BindPopup(void)
{
    const char *state;
    uint32_t elapsed = HAL_GetTick() - ui_bind_stamp;
    uint32_t left = (elapsed < UI_BIND_POPUP_MS) ? ((UI_BIND_POPUP_MS - elapsed + 999U) / 1000U) : 1U;
    char sec[3];

    /* 白底 x8~119 / y16~44, 向内 1px 画黑框 -> 外圈留 1px 白边 */
    OLED_DrawFilledRectangle(8, 16, 111, 29, OLED_COLOR_NORMAL);
    OLED_DrawRectangle(9, 17, 109, 26, OLED_COLOR_REVERSED);

    OLED_PrintString(14, 18, "对频", &font12x12, OLED_COLOR_REVERSED);
    sec[0] = (char)((char)0x30 + (char)left);   /* left = 1~6 */
    sec[1] = (char)0x73;                         /* 's' */
    sec[2] = (char)0;
    OLED_PrintASCIIString(100, 18, sec, &afont12x6, OLED_COLOR_REVERSED);

    if (crsf_link_ok == 0U)
        state = "未连接";
    else if (crsf_link_stats.uplink_lq == 0U)
        state = "对频中";
    else
        state = "对频完成";
    OLED_PrintString(14, 31, (char *)state, &font12x12, OLED_COLOR_REVERSED);

    if (crsf_link_ok != 0U)
    {
        OLED_PrintASCIIString(76, 31, "LQ:", &afont12x6, OLED_COLOR_REVERSED);
        UI_NumD(94, 31, (int32_t)crsf_link_stats.uplink_lq, 3, OLED_COLOR_REVERSED);
    }
    else
    {
        OLED_PrintASCIIString(76, 31, "LQ: --", &afont12x6, OLED_COLOR_REVERSED);
    }
}

/* USB_DETECT(PA13) 高电平 = 检测到 USB.
   注意: PA13 同时是 SWDIO, 插着 ST-Link 时可能被调试器拉高;
   测 USB 插入时请拔掉调试器, 否则会误判. */
static uint8_t UI_UsbDetectHigh(void)
{
    return (HAL_GPIO_ReadPin(USB_DETECT_GPIO_Port, USB_DETECT_Pin) == GPIO_PIN_SET) ? 1U : 0U;
}

/* 正常页面上的 USB 检测弹窗: 白底黑字, 居中盖在当前页面上.
   只在 USB_DETECT 为高时由 UI_DrawPage 调用, 不做超时: 拔掉 USB 下一帧就消失. */
static void UI_UsbPopup(void)
{
    /* 100x24 白底, 向内 1px 画黑框 -> 外圈留 1px 白边;
       "USB HIGH" = 8 字符 x 8px = 64px, 居中后左右各留 17px */
    OLED_DrawFilledRectangle(14, 20, 99, 24, OLED_COLOR_NORMAL);
    OLED_DrawRectangle(15, 21, 97, 21, OLED_COLOR_REVERSED);
    OLED_PrintASCIIString(32, 24, "USB HIGH", &afont16x8, OLED_COLOR_REVERSED);
}

/* 软关机完成页: 显示静态"关机 / 长按开机".
   为什么需要这个页面:
     - 软关机到点后 pwr_latched=0, 但电源键往往还按着; off_lock 把 pwr_hold_ms 固定为 0,
       如果这里继续画 UI_BootAnim(0), 就会看到一个进度条永远不走的"假开机界面";
     - 没 USB 时松手即断电, 有 USB 时板子继续由 USB 供电, 此时提示用户重新长按开机;
     - 冷启动 / 重新按下电源键(pwr_hold_ms>0)仍由 UI_Task 画开机进度条. */
static void UI_PowerOffIdle(void)
{
    OLED_NewFrame();

    OLED_PrintString(52, 18, "关机", &font12x12, OLED_COLOR_NORMAL);       /* 2 字 24px -> 居中 */
    OLED_PrintString(40, 38, "长按开机", &font12x12, OLED_COLOR_NORMAL);   /* 4 字 48px -> 居中 */

    if (UI_UsbDetectHigh())
        OLED_PrintASCIIString(55, 52, "USB", &afont8x6, OLED_COLOR_NORMAL); /* 3 字符 18px -> 居中 */

    UI_Border(0U, 0U, 127U, 63U, UI_FRAME_CUT, OLED_COLOR_NORMAL);          /* 与关机页/开机页一致的倒角框 */
}

/* ========================== 第14页: 微调分配 + 微调弹窗 ========================== */
/* 微调量 -> "+20.0%" / "-0.5%" / "+0.0%": 固定 6 字符(36px).
   单位是 0.1% 满行程: trim_val = 200 就是 20.0%, 整数部分最多两位(a/10 = 0~20) */
static void UI_TrimPct(uint8_t x, uint8_t y, int16_t v, OLED_ColorMode color)
{
    char     s[7];
    uint16_t a = (uint16_t)((v < 0) ? -v : v);                   /* 0 ~ TRIM_LIMIT(200) */
    uint8_t  t = (uint8_t)((a / 10U) / 10U);                     /* 整数的十位: 0~2 */

    s[0] = (v < 0) ? (char)0x2D : (char)0x2B;                    /* - / + */
    s[1] = (t != 0U) ? (char)(0x30 + t) : (char)0x20;            /* 十位为 0 时填空格: 不出现 +01.0% */
    s[2] = (char)(0x30 + (uint8_t)((a / 10U) % 10U));            /* 整数的个位 */
    s[3] = (char)0x2E;                                           /* . */
    s[4] = (char)(0x30 + (uint8_t)(a % 10U));                    /* 0.1% 位 */
    s[5] = (char)0x25;                                           /* % */
    s[6] = (char)0;
    OLED_PrintASCIIString(x, y, s, &afont8x6, color);
}

/* 微调模式弹窗(白底黑字 + 内缩 1px 黑框, 盖在当前页面上):
   标题 微调, 右上角 PWR 提示"再短按一次电源键退出"; 每行 = 输出字母 / 挂的微调号 / 当前偏移量.
   数值直接读 trim_val, Trim_Task 每 10ms 更新 -> 按住微调键时实时变化 */
static void UI_TrimPopup(void)
{
    static const char ch[4] = { 'A', 'E', 'T', 'R' };   /* Ail / Ele / Thr / Rud */
    uint8_t i;

    OLED_DrawFilledRectangle(10, 8, 106, 48, OLED_COLOR_NORMAL);   /* 白底 10..116 x 8..55 */
    OLED_DrawRectangle(11, 9, 104, 45, OLED_COLOR_REVERSED);        /* 内缩 1px 黑框 11..115 x 9..54 */

    OLED_PrintString(16, 10, "微调", &font12x12, OLED_COLOR_REVERSED);
    OLED_PrintASCIIString(96, 10, "PWR", &afont12x6, OLED_COLOR_REVERSED);

    for (i = 0U; i < 4U; i++)
    {
        uint8_t y = (uint8_t)(22U + (i * 8U));
        uint8_t t = aetr_trim[i];
        char    s[3];

        s[0] = ch[i]; s[1] = (char)0;
        OLED_PrintASCIIString(16, y, s, &afont8x6, OLED_COLOR_REVERSED);

        if ((t >= 1U) && (t <= TRIM_NUM)) { s[0] = (char)0x54; s[1] = (char)(0x30 + t); }   /* T1~T4 */
        else                              { s[0] = (char)0x2D; s[1] = (char)0x2D; }          /* -- */
        s[2] = (char)0;
        OLED_PrintASCIIString(26, y, s, &afont8x6, OLED_COLOR_REVERSED);

        if ((t >= 1U) && (t <= TRIM_NUM))
            UI_TrimPct(78, y, trim_val[t - 1U], OLED_COLOR_REVERSED);
    }
}

/* 第14页(系统菜单 -> 微调分配): 与"输入映射"页同一套格式(三列 + 每行两个焦点格):
     输出名(x=1) / 微调格(x=40, 焦点框 40..57) / 方向格(x=84, 焦点框 84..95).
   微调格 = --/T1~T4(该输出挂哪一路微调); 方向格 = +1 正 / -1 反(微调在该输出上的作用方向);
   中间 x=60 是实体键提示(T6/7 = TRIM6 减 / TRIM7 加), 不参与焦点.
   焦点态: UP/DN 循环 8 格(长按加速), ENTER 进编辑, EXIT 保存并返回菜单
   编辑态: 微调格 UP/DN 循环 --/T1/T2/T3/T4; 方向格 UP=+1 正, DOWN=-1 反;
           ENTER 保存退出编辑, EXIT 保存并返回菜单
   "--" = 该输出不挂微调; 同一个微调可以同时挂多个输出(不去重, 与常见遥控一致) */
static void UI_TrimSetPage(void)
{
    static const char *nm[4] = { "Ail", "Ele", "Thr", "Rud" };
    static const char *kb[TRIM_NUM] = { "T6/7", "T5/8", "T2/3", "T4/1" };
    uint8_t i;

    OLED_NewFrame();
    OLED_PrintString(1, 1, "输出  微调  方向", &font12x12, OLED_COLOR_NORMAL);

    for (i = 0U; i < 4U; i++)
    {
        uint8_t y  = (uint8_t)(12U + (i * 8U));
        uint8_t tc = (uint8_t)(i * 2U);                 /* 微调格编号 */
        uint8_t dc = (uint8_t)(tc + 1U);                /* 方向格编号 */
        uint8_t t  = aetr_trim[i];
        char    s[5];

        s[0] = nm[i][0]; s[1] = nm[i][1]; s[2] = nm[i][2]; s[3] = (char)0;
        OLED_PrintASCIIString(1, y, s, &afont8x6, OLED_COLOR_NORMAL);

        if ((t >= 1U) && (t <= TRIM_NUM)) { s[0] = (char)0x54; s[1] = (char)(0x30 + t); }   /* T1~T4 */
        else                              { s[0] = (char)0x2D; s[1] = (char)0x2D; }          /* -- */
        s[2] = (char)0;
        if (ui_tr_focus == tc)
            OLED_DrawFilledRectangle(40, y, 18, 8, OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(40, y, s, &afont8x6, (ui_tr_focus == tc) ? OLED_COLOR_REVERSED : OLED_COLOR_NORMAL);

        if ((t >= 1U) && (t <= TRIM_NUM))               /* 实体键提示: 塞在中间空档, 不参与焦点 */
            OLED_PrintASCIIString(60, y, (char *)kb[t - 1U], &afont8x6, OLED_COLOR_NORMAL);

        s[0] = (aetr_trim_rev[i] != 0U) ? (char)0x2D : (char)0x2B;   /* -1 反 / +1 正 */
        s[1] = (char)0x31;
        s[2] = (char)0;
        if (ui_tr_focus == dc)
            OLED_DrawFilledRectangle(84, y, 12, 8, OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(84, y, s, &afont8x6, (ui_tr_focus == dc) ? OLED_COLOR_REVERSED : OLED_COLOR_NORMAL);
    }

    OLED_PrintString(1, 51, "确认/退出 保存", &font12x12, OLED_COLOR_NORMAL);
}

/* ========================== 页面绘制 ========================== */
/* 画当前页, 然后统一补一圈整屏方框(UI_ScreenFrame).
   各页面的坐标约定(与方框留出 1px 黑边, 避免和边框粘在一起):
     文字/数值: 左起 x=1(墨迹从 2 开始), 右侧最外墨迹 <= 125;
     反色实心条: 铺满方框内圈, 左边 x=2, 宽度 = 124(即到 x=125);
     行: 首行 y=1(不压上边框), 末行 y=55(不压下边框), 中间行保持 8px 行距;
     只有摇杆页顶行是反色状态条(白底黑字), 该条 x=2/y=2 起铺(左/上各留 1px).
   方框必须最后画: 文字(反色模式)会把途经的像素擦黑, 先画的话边框会被打出缺口.
   功能页面方框统一方角(倒角只在开机界面保留) */
static void UI_DrawPage(void)
{
    if (ui_page == UI_PAGE_MENU)       UI_MenuPage();
    else if (ui_page == UI_PAGE_DIAG)  DiagShowPage();
    else if (ui_page == UI_PAGE_CHBAR) UI_ChBarPage();
    else if (ui_page == UI_PAGE_INPUT) UI_InputPage();
    else if (ui_page == UI_PAGE_MODEL) UI_ModelPage();
    else if (ui_page == UI_PAGE_MIX)   UI_MixPage();
    else if (ui_page == UI_PAGE_MIXED) UI_MixEditPage();
    else if (ui_page == UI_PAGE_CHAN)  UI_ChanPage();
    else if (ui_page == UI_PAGE_VER)   UI_VerPage();
    else if (ui_page == UI_PAGE_TRIMSET) UI_TrimSetPage();
    else                               UI_StickPage();

    /* 对频状态弹窗盖在当前页面内容上, 6 秒后由 UI_Task 清标志 */
    if (ui_bind_popup)
        UI_BindPopup();

    /* USB 诊断弹窗: 直接读 USB_DETECT(PA13) 电平, 高电平就盖在当前正常页面上.
       放在对频弹窗之后 -> 两者同时出现时 USB 状态仍然可见;
       必须放在 UI_ScreenFrame 之前, 否则弹窗会压掉整屏方框. */
    if (UI_UsbDetectHigh())
        UI_UsbPopup();

    /* 微调模式弹窗: 盖在所有页面/弹窗之上, 但必须在 UI_ScreenFrame 之前(先画的话方框会被弹窗压掉) */
    if (trim_mode)
        UI_TrimPopup();

    /* 软关机界面(UI_PwrOffPage)和关机完成页(UI_PowerOffIdle)是整屏自绘的, 不在这里叠加 —— 见 UI_Task 开头分支 */
    UI_ScreenFrame();
}

/* ========================== 任务入口(由 Schedule 的 func4 调用) ========================== */
void UI_Task(void)
{
    /* 未锁存(电源键还没按够 UI_BOOT_MS)时:
       - 冷启动 / 重新按下电源键: 画开机进度条(pwr_hold_ms 会随时间增长);
       - 软关机完成后仍按着, 或松手后靠 USB 供电: pwr_off_done=1 且 pwr_hold_ms=0,
         画静态"关机/长按开机"页. 此时 off_lock 会压住重新计时, 不能再画空进度开机界面,
         否则看起来像"开机界面失效、进度条不走".
       没 USB 时松手就断电熄屏; 有 USB 时停在关机页, 松开再长按才会重新进开机进度条.
       自锁开关版(BoardCfg.h): power_detect 让 pwr_latched 上电即 1 -> 本段整段不会进入. */
    if (pwr_latched == 0U)
    {
        ui_boot = 0U;                    /* 松手/软关机后清掉上次的上电时刻 */
        ui_bind_popup = 0U;              /* 关机/重新上电时不要带出上次的对频弹窗 */
        trim_mode = 0U;                  /* 也不带微调模式: 下次上电按 Flash 里的分配重新开始 */
        (void)Trim_ModeEvent();          /* 丢掉门禁期间攒下的切换请求(锁存前的电源键短按无效) */
        UI_KeyFlush();                   /* 这段 UI 不认按键: 门禁期间按的键(UP/DN/EN/EXIT 四键矩阵)都不算数 */
        if ((pwr_off_done != 0U) && (pwr_hold_ms == 0U))
            UI_PowerOffIdle();           /* 软关机完成: 静态关机页, 等待重新长按开机 */
        else
            UI_BootAnim(pwr_hold_ms);    /* 冷启动 / 正在重新长按开机 */
        return;
    }

    if (ui_boot == 0U)
        ui_boot = HAL_GetTick();         /* 锁存那一刻 = 上电完成(启动提示也按它计时) */

    /* 锁存后让四格填满 + READY 再亮 UI_READY_MS, 否则这一帧一闪而过看不见 */
    if ((HAL_GetTick() - ui_boot) < UI_READY_MS)
    {
        UI_BootAnim(UI_BOOT_MS);
        UI_KeyFlush();                   /* 开机提示期间按的键不算数(电源键 PB13 不在 4 键矩阵里, 见 BoardCfg.h) */
        (void)Trim_ModeEvent();          /* READY 提示期间的短按也丢掉, 免得一结束就莫名其妙进微调模式 */
        return;
    }

    /* 软关机长按进行中: 整屏切成黑底, 显示 "长按关机" + 四格进度(见 UI_PwrOffPage).
       必须放在 UI_DrawPage/UI_ScreenFrame 之前整屏接管(不能只叠一块), 期间直接 return:
       按键也不响应, 避免关到一半把页面切走.
       pwr_off_ms 只有长按版会累加, 自锁开关版恒 0 -> 这一段永不进入 */
    if (pwr_off_ms >= UI_PWROFF_SHOW_MS)
    {
        ui_bind_popup = 0U;              /* 关机画面优先, 取消后也不要再冒出对频弹窗 */
        UI_PwrOffPage();
        UI_KeyFlush();                   /* 关机动画期间按的键不算数(否则下次开机第一下就跳) */
        return;
    }

    /* 参数协议后台推进(和当前页面无关): 上电后把模块参数表扫出来, 诊断页的
       F 标志(crsf_param_ready)就靠它看模块支不支持参数协议.
       CRSF_ParamTick 自带 60ms 节流(每拍 2 帧), 不会挤爆 4ms 的发送节拍 */
    {
        CRSF_ParamTick();
        if ((HAL_GetTick() - ui_param_ping_at) >= 2000U)
        {
            ui_param_ping_at = HAL_GetTick();
            if (crsf_dev_online == 0U)
                CRSF_PingTick();                     /* 模块还没上线: 每 2s 换一个 ping 变体 */
            if ((crsf_param_scan == 0U) && (crsf_param_ready == 0U) && (ui_param_tries < 3U))
            {
                ui_param_tries++;
                CRSF_ParamScanStart();               /* 枚举参数表(模块不支持时最多重试 3 次) */
            }
        }
    }

    /* 电源键短按 = 切换微调模式(Schedule.c 的 power_detect 判定 40~700ms, 见 PWR_SHORT_MIN/MAX_MS).
       只有摇杆(0)/16 通道(2)/通道条(7)三页、且不在校准时才允许进入; 退出时立刻落盘.
       本段在"软关机画面"之后: 微调模式下长按电源键 3s 照样能软关机(上面已 return) */
    if (Trim_ModeEvent())
    {
        if (trim_mode != 0U)
        {
            trim_mode = 0U;
            Store_SaveCalib();               /* 退出即保存微调量(与校准/混控共用同一条 Flash 记录) */
        }
        else if ((ui_cal == 0U) &&
                 ((ui_page == UI_PAGE_STICK) || (ui_page == UI_PAGE_CHAN) || (ui_page == UI_PAGE_CHBAR)))
        {
            trim_mode     = 1U;
            ui_bind_popup = 0U;              /* 微调弹窗优先, 别和对频弹窗叠在一起 */
        }
        ui_rep_key = 0xFFU;                  /* 清连发: 手上还按着 UP/DN 也不带进/带出微调模式 */
        UI_KeyFlush();                       /* 这次按压期间的导航键事件全部作废 */
    }

    if (trim_mode != 0U)                     /* 微调模式: 四个导航键(=TRIM1~4)只做微调, 菜单/翻页/校准全失效 */
    {
        UI_KeyFlush();                       /* Trim_Task(10ms) 直接读引脚, 不走 Key_State_* 队列, 这里清干净 */
        UI_DrawPage();                       /* 页面 + 微调弹窗 + 整屏方框 */
        return;
    }

    /* 校准: 状态机在 Func.c 里跑, 界面锁在第0页并显示阶段提示 */
    if (ui_cal)
    {
        ui_page = UI_PAGE_STICK;
        if (state == 3U) { ui_cal = 0U; ui_cal_ok = 1U; ui_stamp = HAL_GetTick(); }
    }

    if (ui_cal)                                      /* 校准中: 只允许 EXIT 提前结束 */
    {
        if (UI_Key(EXIT))
        {
            state     = 3U;
            ui_cal    = 0U;
            ui_cal_ok = 0U;
        }
        UI_DrawPage();
        return;
    }

    /* 对频状态弹窗: 6 秒后自动关闭; 显示期间只认 EXIT(提前关掉), 其它按键不穿透到菜单 */
    if (ui_bind_popup)
    {
        if ((HAL_GetTick() - ui_bind_stamp) >= UI_BIND_POPUP_MS)
            ui_bind_popup = 0U;
        else
        {
            if (UI_Key(EXIT))
                ui_bind_popup = 0U;
            UI_DrawPage();
            return;
        }
    }

    /* 按键导航 */
    if (ui_page == UI_PAGE_MENU)
    {
        uint8_t count = 0U;
        const UI_MenuItem_t *items = UI_MenuItems(&count);

        if (ui_cursor >= count)               /* 菜单切换后纠正光标(防御) */
            ui_cursor = (uint8_t)(count - 1U);

        ui_cursor = UI_FocusStep(ui_cursor, count, 0U);   /* UP: 循环焦点, 长按加速 */
        ui_cursor = UI_FocusStep(ui_cursor, count, 1U);   /* DOWN: 循环焦点, 长按加速 */
        if (UI_Key(ENTER)) UI_DoAction(items[ui_cursor].action);
        if (UI_Key(EXIT))  UI_DoAction(UI_A_BACK);
    }
    else if (ui_page == UI_PAGE_DIAG)
    {
        if (UI_Key(EXIT)) ui_page = UI_PAGE_STICK;
    }
    else if (ui_page == UI_PAGE_VER)
    {
        if (UI_Key(EXIT)) ui_page = UI_PAGE_MENU;   /* 版本页只有 EXIT 有反应 */
    }
    else if (ui_page == UI_PAGE_STICK)
    {
        if (UI_Key(UP))                        /* UP: 上一页 */
            ui_page = UI_RingStep(ui_page, 0U);
        if (UI_Key(DOWN))                      /* DOWN: 下一页 */
            ui_page = UI_RingStep(ui_page, 1U);
        if (UI_Key(ENTER))                     /* ENTER: 进菜单 */
        {
            ui_page   = UI_PAGE_MENU;
            ui_menu   = 0U;
            ui_cursor = 0U;
        }
    }

    if (ui_page == UI_PAGE_MIX)                 /* 混控列表: 选通道 */
    {
        ui_mix_ch = UI_FocusStepSlow(ui_mix_ch, 16U, 0U);   /* UP: ch1~ch16 循环, 慢速连发 */
        ui_mix_ch = UI_FocusStepSlow(ui_mix_ch, 16U, 1U);   /* DOWN */
        if (UI_Key(ENTER))
        {
            ui_mix_focus = 0U; ui_mix_edit = 0U; ui_mix_press = 0U;
            ui_page = UI_PAGE_MIXED;
        }
        if (UI_Key(EXIT)) { Store_SaveCalib(); ui_page = UI_PAGE_MENU; }
    }

    if (ui_page == UI_PAGE_MIXED)               /* 混控编辑: 焦点/编辑/长按删除 */
    {
        uint8_t ch  = (uint8_t)(ui_mix_ch + 1U);
        uint8_t c   = Mix_ChCount(ch);
        uint8_t num = UI_MixCellNum(ch);
        uint8_t row;

        if (ui_mix_focus >= num) ui_mix_focus = (uint8_t)(num - 1U);
        row = (uint8_t)(ui_mix_focus / 2U);

        if (ui_mix_edit)                        /* 编辑态: UP/DN 改值 */
        {
            Mix_t *m = Mix_ChAt(ch, row);
            if (m != 0)
            {
                if (ui_mix_focus & 1U)          /* 权重: 步进 1, 长按加速, 限幅 ±100 */
                {
                    uint8_t n = UI_KeyRepeat(UP);                  /* 步进 1, 长按加速 */
                    while (n--) { int16_t w = (int16_t)m->weight + 1; if (w >  100) w =  100; m->weight = (int8_t)w; }
                    n = UI_KeyRepeat(DOWN);
                    while (n--) { int16_t w = (int16_t)m->weight - 1; if (w < -100) w = -100; m->weight = (int8_t)w; }
                }
                else                            /* 源: 0~7 循环, 长按加速 */
                {
                    m->src = UI_FocusStep(m->src, MIX_SRC_NUM, 0U);
                    m->src = UI_FocusStep(m->src, MIX_SRC_NUM, 1U);
                }
            }
            /* 退出编辑态时清掉连发状态: 手还按着 UP/DN 也不会马上在焦点态继续跑 */
            if (UI_Key(ENTER)) { ui_mix_edit = 0U; ui_rep_key = 0xFFU; Store_SaveCalib(); }
            if (UI_Key(EXIT))  { ui_mix_edit = 0U; ui_rep_key = 0xFFU; Store_SaveCalib(); ui_page = UI_PAGE_MIX; }
        }
        else                                    /* 焦点态: UP/DN 循环移动(长按加速) */
        {
            ui_mix_focus = UI_FocusStep(ui_mix_focus, num, 0U);
            ui_mix_focus = UI_FocusStep(ui_mix_focus, num, 1U);
            row = (uint8_t)(ui_mix_focus / 2U);               /* 焦点可能刚被长按推走 */

            if (ui_mix_focus >= (uint8_t)(c * 2U))            /* "+": 添加一个输入源 */
            {
                if (UI_Key(ENTER) && Mix_Add(ch))
                {
                    ui_mix_focus = (uint8_t)(Mix_ChCount(ch) * 2U - 2U);
                    Store_SaveCalib();
                }
            }
            else if (UI_Key(ENTER))                            /* 行上: 记下按下时刻 */
            {
                ui_mix_press = HAL_GetTick();
            }

            if (ui_mix_press != 0U)                            /* 短按=进编辑, 长按=删除 */
            {
                uint32_t held = HAL_GetTick() - ui_mix_press;
                if (Key_State_Now[ENTER] == 0U)
                {
                    if (held < 900U) { ui_mix_edit = 1U; ui_rep_key = 0xFFU; }   /* 进编辑态: 清连发 */
                    ui_mix_press = 0U;
                }
                else if (held >= 900U)
                {
                    Mix_Del(ch, row);                          /* 删除: 后面的补到前面 */
                    Store_SaveCalib();
                    ui_mix_focus = 0U; ui_mix_press = 0U;
                }
            }

            if (UI_Key(EXIT)) { Store_SaveCalib(); ui_page = UI_PAGE_MIX; }
        }
    }

    if (ui_page == UI_PAGE_MODEL)               /* 模型页: UP/DN 切换, EXIT 保存返回 */
    {
        uint8_t m = (uint8_t)model_cur;

        m = UI_FocusStepSlow(m, MODEL_MAX, 0U);   /* UP: 模型循环切换, 慢速连发 */
        m = UI_FocusStepSlow(m, MODEL_MAX, 1U);   /* DOWN */
        if (m != model_cur)
        {
            Store_SaveCalib();                  /* 整条记录(校准+方向+源+模型+混控)一起写 */
            model_cur = m;
            Mix_LoadForModel((uint8_t)model_cur);
        }
        if (UI_Key(EXIT)) { Store_SaveCalib(); ui_page = UI_PAGE_MENU; }
    }

    if (ui_page == UI_PAGE_INPUT)               /* 输入分配页: 焦点/编辑两态 */
    {
        uint8_t i = (uint8_t)(ui_in_focus / 2U);

        if (ui_in_edit)                         /* 编辑态: UP/DN 改值, ENTER/EXIT 退出保存 */
        {
            if (ui_in_focus & 1U)               /* DIR: UP=+1, DOWN=-1 */
            {
                if (UI_Key(UP))   aetr_input[i] = (int8_t)1;
                if (UI_Key(DOWN)) aetr_input[i] = (int8_t)-1;
            }
            else                                /* SRC: 上下循环 IN1~IN4 */
            {
                if (UI_Key(UP))   aetr_src[i] = (uint8_t)((aetr_src[i] + 1U) & 3U);
                if (UI_Key(DOWN)) aetr_src[i] = (uint8_t)((aetr_src[i] + 3U) & 3U);
            }
            if (UI_Key(ENTER)) { ui_in_edit = 0U; Store_SaveInput(); }
            if (UI_Key(EXIT))  { ui_in_edit = 0U; Store_SaveInput(); ui_page = UI_PAGE_MENU; }
        }
        else                                    /* 焦点态: UP/DN 循环移动, ENTER 进编辑 */
        {
            ui_in_focus = UI_FocusStep(ui_in_focus, 8U, 0U);   /* UP: 循环焦点(长按加速) */
            ui_in_focus = UI_FocusStep(ui_in_focus, 8U, 1U);   /* DOWN */
            if (UI_Key(ENTER)) ui_in_edit  = 1U;
            if (UI_Key(EXIT))  { Store_SaveInput(); ui_page = UI_PAGE_MENU; }
        }
    }

    if (ui_page == UI_PAGE_TRIMSET)             /* 微调分配页: 焦点/编辑两态(与输入映射页同构) */
    {
        uint8_t i = (uint8_t)(ui_tr_focus / 2U);   /* 偶数 = 微调格, 奇数 = 方向格 */

        if (ui_tr_edit)                         /* 编辑态: UP/DN 改值, ENTER/EXIT 退出保存 */
        {
            if (ui_tr_focus & 1U)               /* 方向格: UP = +1 正, DOWN = -1 反 */
            {
                if (UI_Key(UP))   aetr_trim_rev[i] = 0U;
                if (UI_Key(DOWN)) aetr_trim_rev[i] = 1U;
            }
            else                                /* 微调格: UP/DN 循环 --/T1~T4 */
            {
                uint8_t v = aetr_trim[i];

                if (UI_Key(UP))   v = (uint8_t)((v + 1U) % (TRIM_NUM + 1U));
                if (UI_Key(DOWN)) v = (uint8_t)((v + TRIM_NUM) % (TRIM_NUM + 1U));
                aetr_trim[i] = v;
            }
            if (UI_Key(ENTER)) { ui_tr_edit = 0U; Store_SaveCalib(); }
            if (UI_Key(EXIT))  { ui_tr_edit = 0U; Store_SaveCalib(); ui_page = UI_PAGE_MENU; }
        }
        else                                    /* 焦点态: UP/DN 循环 8 格, ENTER 进编辑 */
        {
            ui_tr_focus = UI_FocusStep(ui_tr_focus, 8U, 0U);   /* UP: 循环焦点(长按加速) */
            ui_tr_focus = UI_FocusStep(ui_tr_focus, 8U, 1U);   /* DOWN */
            if (UI_Key(ENTER)) ui_tr_edit = 1U;
            if (UI_Key(EXIT))  { Store_SaveCalib(); ui_page = UI_PAGE_MENU; }
        }
    }

    if ((ui_page == UI_PAGE_CHAN) || (ui_page == UI_PAGE_CHBAR))   /* 两个通道页: UP/DOWN 翻页, ENTER 进菜单 */
    {
        if (UI_Key(UP))                          /* 上一页 */
            ui_page = UI_RingStep(ui_page, 0U);
        if (UI_Key(DOWN))                        /* 下一页 */
            ui_page = UI_RingStep(ui_page, 1U);
        if (UI_Key(ENTER))                       /* 进菜单 */
        {
            ui_page   = UI_PAGE_MENU;
            ui_menu   = 0U;
            ui_cursor = 0U;
        }
    }

    /* 页面切换后清掉连发状态: 防止"按住 UP/DN 的同时按 ENTER 换页"导致
       新页面的焦点沿用旧节拍立刻继续飞(要重新按一下才会连发) */
    {
        static uint8_t last_page = 0xFFU;

        if (last_page != ui_page)
        {
            last_page = ui_page;
            ui_rep_key = 0xFFU;
        }
    }

    /* 绘制当前页 + 整屏方框(推屏由调用者 func4 负责) */
    UI_DrawPage();
}
