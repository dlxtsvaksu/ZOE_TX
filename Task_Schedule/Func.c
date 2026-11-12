#include "Func.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include <stdint.h>
#include "main.h"
#include "Store.h"
#include "BoardCfg.h"   /* BOARD_PWR_HAS_LATCH: 自锁开关版没有电源键, 微调模式改用板上的 TRIM_MODE 键 */
volatile ADC_CALI_t user_adc_calibration[4]={
    {0,2048,4096},
    {0,2048,4096},
    {0,2048,4096},
    {0,2048,4096},
};
volatile uint8_t state=3;
volatile uint32_t time=0;
/*============================摇杆=============================*/
volatile uint16_t aetr[4];
/* 前 4 通道方向(输入分配): +1 = 正向, -1 = 反向(围绕中位镜像).
   作用点在 Adc_convert_Crsf() 里"ADC 校准换算之后"; 掉电保存在 Store, 上电由 Store_LoadCalib() 恢复.
   后期遥控器混控也以这四个 input 为源 */
volatile int8_t aetr_input[4] = {-1, -1, -1, -1};
volatile uint8_t aetr_src[4]  = {0, 1, 2, 3};      /* 前 4 通道的输入源(0~3): 默认输出 i 用第 i 路 ADC */
extern volatile uint16_t ADC_Values[5];
/* 中位死区(counts): 回中时抑制噪声抖动; ADC 平均: 每 1ms 累加, 满 8 次取平均 */
#define ADC_DEADBAND 20   /* 中位死区(counts, 约0.5%): 覆盖电位器回中迟滞与残余噪声 */
#define ADC_AVG_N    8
/* 标定表合理性检查: 每侧行程小于 ADC_SPAN_MIN 视为"没推到极限/信号被钳位",
   则用 ADC_SPAN_DEF 兜底, 避免"行程≈中位"时一推就打到最大/最小值 */
#define ADC_SPAN_MIN 200U
#define ADC_SPAN_DEF 1200U
volatile uint16_t adc_filt[4] = {0, 0, 0, 0};
void Adc_FilterTask(void){
    static uint32_t acc[4] = {0, 0, 0, 0};
    static uint8_t  n = 0;
    for(uint8_t i=0;i<4;i++) acc[i] += ADC_Values[i];
    if(++n >= ADC_AVG_N){
        for(uint8_t i=0;i<4;i++){
            uint16_t mid = user_adc_calibration[i].ADC_MID;
            uint16_t dev;
            adc_filt[i] = (uint16_t)(acc[i] / ADC_AVG_N);
            acc[i] = 0;
            /* 诊断: 记录相对中位的最大偏差(界面可看, 可复位) */
            dev = (adc_filt[i] > mid) ? (uint16_t)(adc_filt[i] - mid) : (uint16_t)(mid - adc_filt[i]);
            if(dev > adc_dev_max[i]) adc_dev_max[i] = dev;
        }
        n = 0;
    }
}
/*============================电池=============================*/
/* 电池电压来自 ADC_IN6(PA6), 经 10k+10k 电阻分压(比例 2.0) */
#define BATT_DIV_X1000  2000U      /* 分压比 x1000: 2.000 */
#define BATT_AVG_N      8          /* 与摇杆同节奏: 8 次平均 */
volatile uint16_t batt_mv = 0;     /* 电池电压(mV) */
void Batt_Task(void){
    static uint32_t acc = 0;
    static uint8_t  n = 0;
    acc += ADC_Values[4];
    if(++n >= BATT_AVG_N){
        uint32_t raw = acc / BATT_AVG_N;
        uint32_t mv  = (raw * 3300U + 2048U) / 4096U;      /* ADC 引脚电压(mV) */
        mv = (mv * BATT_DIV_X1000 + 500U) / 1000U;         /* x2 -> 电池电压(mV) */
        batt_mv = (uint16_t)mv;
        acc = 0; n = 0;
    }
}
void Adc_convert_Crsf(void){
    if(state!=3){        //如果正在校准，摇杆值进入保护状态
        aetr[0]=crsf_mid;
        aetr[1]=crsf_mid;
        aetr[2]=crsf_min;
        aetr[3]=crsf_mid;
    }
    else{
        for(uint8_t i=0;i<4;i++){
            /* 输入分配: 输出 i 的源由 aetr_src[i] 决定(0~3), 越界则退回第 i 路 */
            uint8_t  s   = (aetr_src[i] < 4U) ? aetr_src[i] : (uint8_t)i;
            uint16_t mid = user_adc_calibration[s].ADC_MID;
            uint16_t raw = adc_filt[s];        // 该源平均后的值

            /* 中位死区 */
            if((raw > mid) && ((raw - mid) < ADC_DEADBAND))
                raw = mid;
            else if((raw < mid) && ((mid - raw) < ADC_DEADBAND))
                raw = mid;

            if(raw>=mid)
                aetr[i]=(raw-mid)*(crsf_max-crsf_mid)/(user_adc_calibration[s].ADC_MAX-mid)+crsf_mid;
            else
                aetr[i]=crsf_mid-(mid-raw)*(crsf_mid-crsf_min)/(mid-user_adc_calibration[s].ADC_MIN);

            /* 微调: 叠加到通道值上(放在方向镜像之前 -> 微调跟随通道正反, 与常规遥控一致).
               aetr_trim[i] = 0 表示该输出不挂微调; aetr_trim_rev[i] = 1 表示该输出的微调取反;
               1000 = 满行程 */
            {
                uint8_t ti = aetr_trim[i];

                if ((ti >= 1U) && (ti <= TRIM_NUM))
                {
                    int16_t off = Trim_Counts((uint8_t)(ti - 1U));
                    int32_t v;

                    if (aetr_trim_rev[i] != 0U)
                        off = (int16_t)(-off);              /* 反: 微调在该输出上取反 */
                    v = (int32_t)aetr[i] + (int32_t)off;

                    if (v > (int32_t)crsf_max) v = (int32_t)crsf_max;
                    if (v < (int32_t)crsf_min) v = (int32_t)crsf_min;
                    aetr[i] = (uint16_t)v;
                }
            }

            /* 通道方向: aetr_input = -1 则围绕中位镜像(反向) */
            if(aetr_input[i] < 0)
                aetr[i] = (uint16_t)((crsf_mid + crsf_mid) - aetr[i]);

            /* 输出限幅 */
            if(aetr[i] > crsf_max) aetr[i] = crsf_max;
            if(aetr[i] < crsf_min) aetr[i] = crsf_min;
        }
    }
}

/* 中位采集累加器: case1 的 2s 窗口内持续累加(每 5ms 一次), 结束取平均 */
static uint32_t adc_mid_acc[4] = {0, 0, 0, 0};
static uint16_t adc_mid_n   = 0;
volatile uint16_t adc_dev_max[4] = {0, 0, 0, 0};   /* 相对中位的最大偏差(诊断用) */
volatile uint8_t  adc_cal_bad = 0U;     /* 1 = 标定行程过小/标定表异常, 已用默认行程兜底 */

/* 标定表检查: 行程过小(没推到极限)或数值越界时用安全默认行程兜底, 并置 adc_cal_bad.
   上电从 Flash 恢复后调用一次, 校准结束时也会调用 */
void Adc_CalibCheck(void){
    adc_cal_bad = 0U;
    for(uint8_t i=0;i<4;i++){
        uint16_t mid = user_adc_calibration[i].ADC_MID;
        uint16_t lo  = user_adc_calibration[i].ADC_MIN;
        uint16_t hi  = user_adc_calibration[i].ADC_MAX;
        int32_t  dlo, dhi;

        if(mid > 4095U){ mid = 2048U; user_adc_calibration[i].ADC_MID = mid; adc_cal_bad = 1U; }
        if(lo  > 4095U){ lo  = 0U;    user_adc_calibration[i].ADC_MIN = lo;  adc_cal_bad = 1U; }
        if(hi  > 4095U){ hi  = 4095U; user_adc_calibration[i].ADC_MAX = hi;  adc_cal_bad = 1U; }

        dlo = (int32_t)mid - (int32_t)lo;      /* 有符号差: 越界/颠倒也能捕捉 */
        dhi = (int32_t)hi  - (int32_t)mid;
        if(dlo < (int32_t)ADC_SPAN_MIN){
            user_adc_calibration[i].ADC_MIN = (mid > ADC_SPAN_DEF) ? (uint16_t)(mid - ADC_SPAN_DEF) : 0U;
            adc_cal_bad = 1U;
        }
        if(dhi < (int32_t)ADC_SPAN_MIN){
            user_adc_calibration[i].ADC_MAX = (mid < (4095U - ADC_SPAN_DEF)) ? (uint16_t)(mid + ADC_SPAN_DEF) : 4095U;
            adc_cal_bad = 1U;
        }
    }
}

void Adc_DevReset(void){
    for(uint8_t i=0;i<4;i++) adc_dev_max[i] = 0;
}

static void Adc_CenterAcc(void){
    for(uint8_t i=0;i<4;i++) adc_mid_acc[i] += adc_filt[i];   /* 用平均后的值累加 */
    if(adc_mid_n < 60000U) adc_mid_n++;
}

static void Adc_Center_Get(void){
    for(uint8_t i=0;i<4;i++){
        uint16_t m = (adc_mid_n != 0U) ? (uint16_t)(adc_mid_acc[i] / adc_mid_n) : adc_filt[i];
        user_adc_calibration[i].ADC_MID = m;                                    /* 中位 = 窗口均值(几百个样本) */
        user_adc_calibration[i].ADC_MIN = (m > 10U) ? (uint16_t)(m - 10U) : 0U;  /* 先给接近中位的初值, 便于行程比较 */
        user_adc_calibration[i].ADC_MAX = (uint16_t)(m + 10U);
        adc_mid_acc[i] = 0;
    }
    adc_mid_n = 0;
}

static void Adc_Range_Get(void){
    for(uint8_t i=0;i<4;i++){
        if(user_adc_calibration[i].ADC_MIN > adc_filt[i])   /* 用平均后的值, 避免单点噪声把行程带偏 */
            user_adc_calibration[i].ADC_MIN = adc_filt[i];
        if(user_adc_calibration[i].ADC_MAX < adc_filt[i])
            user_adc_calibration[i].ADC_MAX = adc_filt[i];
    }
}

/* 行程采集结束: 两端各留约 1.5% 余量, 保证推到极限时能打满 172/1811 */
static void Adc_Range_Final(void){
    Adc_CalibCheck();      /* 先做合理性兜底: 行程过小则改用默认行程 */
    for(uint8_t i=0;i<4;i++){
        uint16_t mid = user_adc_calibration[i].ADC_MID;
        uint16_t lo  = user_adc_calibration[i].ADC_MIN;
        uint16_t hi  = user_adc_calibration[i].ADC_MAX;
        uint16_t dlo = (uint16_t)((mid - lo) / 64U);
        uint16_t dhi = (uint16_t)((hi - mid) / 64U);

        if((mid > lo) && ((uint16_t)(mid - lo) > (uint16_t)(dlo + 20U)))
            user_adc_calibration[i].ADC_MIN = (uint16_t)(lo + dlo);
        if((hi > mid) && ((uint16_t)(hi - mid) > (uint16_t)(dhi + 20U)))
            user_adc_calibration[i].ADC_MAX = (uint16_t)(hi - dhi);
    }
}
void ADC_Calibrate(void){
    switch(state){
        case 0:
            time =HAL_GetTick();
            state=1;
            break;
        case 1:
            Adc_CenterAcc();       // 2s 窗口内持续累加, 结束后取平均作为中位
        if (HAL_GetTick() - time >= 2000){
            Adc_Center_Get();
            state=2;
        }
            break;
        case 2:
        if (HAL_GetTick() - time<=8000){
            Adc_Range_Get();
        }else{
            Adc_Range_Final();     // 两端留余量, 保证能打满
            state=4;              // 先进入"保存"状态(见 case 4)
        }
            break;
        case 4:                      // 校准完成: 保存到 Flash, 下次上电自动恢复
            Store_SaveCalib();
            state=3;                 // 回到"完成"态
            break;

        case 3:
        default:
            break;
    }
}
/*============================开关=============================*/
/*============================混控=============================*/
volatile uint8_t model_cur   = 0U;         /* 当前模型序号(掉电保存) */
Mix_t            mix_ram[MIX_MAX];         /* 只保存"当前模型"的混控: 上电从 Flash 过滤 */
uint8_t          mix_ram_cnt = 0U;         /* 当前模型的混控条数 */

/* 最终 16 通道值(基础值 + 混控, 已限幅): 由 Mix_Apply() 每 5ms 更新 */
volatile uint16_t ch_out[16];

/* 最终 16 通道值: 基础值 = 中位(没有混控的通道就输出中位), 然后叠加各条混控, 最后限幅.
   混控运算: input 的 CRSF 值先折算成相对中位的偏移(±819 counts), 乘以权重(%)后相加,
   累加时保留 x100 的精度(内部单位 = counts x %), 最后统一 /100 再限幅 172~1811.
   例: ch1 只有 "Ail x 100%" 一条 -> 输出就是 Ail 的完整行程(172~1811);
       再叠 "SA x 5%" -> SA 打到 1811 时 ch1 再偏 +41 counts. */
void Mix_Apply(void){
    int32_t acc[16];                        /* 单位: counts x % */
    uint8_t i;

    for(i=0;i<16;i++) acc[i] = (int32_t)crsf_mid * 100;   /* 基础值: 中位 */

    for(i=0;i<mix_ram_cnt;i++){             /* 叠加混控 */
        uint8_t chn = mix_ram[i].ch;
        uint8_t s   = mix_ram[i].src;
        int32_t sv;

        if(chn < 1U || chn > 16U) continue;
        if(s < 4U)       sv = (int32_t)aetr[s];              /* 0~3 = Ail/Ele/Thr/Rud */
        else if(s < 10U) sv = (int32_t)Switch_State[s - 4U]; /* 4~9 = SA/SB/SC/SD/SE/SF */
        else             continue;

        acc[chn-1] += (sv - (int32_t)crsf_mid) * (int32_t)mix_ram[i].weight;
    }

    for(i=0;i<16;i++){                      /* /100 后限幅到标准 CRSF 范围 */
        int32_t v = acc[i] / 100;
        if(v > (int32_t)crsf_max) v = (int32_t)crsf_max;
        if(v < (int32_t)crsf_min) v = (int32_t)crsf_min;
        ch_out[i] = (uint16_t)v;
    }
}

uint8_t Mix_ChCount(uint8_t ch){
    uint8_t n = 0U;
    for(uint8_t i=0;i<mix_ram_cnt;i++)
        if(mix_ram[i].ch == ch) n++;
    return n;
}

Mix_t *Mix_ChAt(uint8_t ch, uint8_t n){
    uint8_t k = 0U;
    for(uint8_t i=0;i<mix_ram_cnt;i++){
        if(mix_ram[i].ch != ch) continue;
        if(k == n) return &mix_ram[i];
        k++;
    }
    return 0;
}

uint8_t Mix_Add(uint8_t ch){
    if(mix_ram_cnt >= MIX_MAX)      return 0U;
    if(Mix_ChCount(ch) >= MIX_PER_CH) return 0U;
    mix_ram[mix_ram_cnt].model  = (uint8_t)model_cur;
    mix_ram[mix_ram_cnt].ch     = ch;
    mix_ram[mix_ram_cnt].src    = 0U;       /* 默认 IN1 */
    mix_ram[mix_ram_cnt].weight = 100;      /* 默认 +100% */
    mix_ram_cnt++;
    return 1U;
}

uint8_t Mix_Del(uint8_t ch, uint8_t n){
    uint8_t k = 0U;
    for(uint8_t i=0;i<mix_ram_cnt;i++){
        if(mix_ram[i].ch != ch) continue;
        if(k == n){                         /* 命中: 后面的整体往前补, 并更新条数 */
            for(uint8_t j=i;j+1U<mix_ram_cnt;j++) mix_ram[j] = mix_ram[j+1U];
            mix_ram_cnt--;
            return 1U;
        }
        k++;
    }
    return 0U;
}

volatile uint16_t Switch_State[6];
void Switch_Read(void){
    if(HAL_GPIO_ReadPin(SA_GPIO_Port,SA_Pin)==GPIO_PIN_RESET)
    {Switch_State[0]=crsf_max;}
    else {Switch_State[0]=crsf_min;}

    if(HAL_GPIO_ReadPin(SB_GPIO_Port,SB_Pin)==GPIO_PIN_RESET)
    {Switch_State[1]=crsf_max;}
    else {Switch_State[1]=crsf_min;}

    if(HAL_GPIO_ReadPin(SC1_GPIO_Port,SC1_Pin)==GPIO_PIN_RESET&&HAL_GPIO_ReadPin(SC2_GPIO_Port,SC2_Pin)==GPIO_PIN_SET)
    {Switch_State[2]=crsf_min;}
    else if(HAL_GPIO_ReadPin(SC1_GPIO_Port,SC1_Pin)==GPIO_PIN_SET&&HAL_GPIO_ReadPin(SC2_GPIO_Port,SC2_Pin)==GPIO_PIN_RESET)
    {Switch_State[2]=crsf_max;}
    else {Switch_State[2]=crsf_mid;}

    if(HAL_GPIO_ReadPin(SD1_GPIO_Port,SD1_Pin)==GPIO_PIN_RESET&&HAL_GPIO_ReadPin(SD2_GPIO_Port,SD2_Pin)==GPIO_PIN_SET)
    {Switch_State[3]=crsf_min;}
    else if(HAL_GPIO_ReadPin(SD1_GPIO_Port,SD1_Pin)==GPIO_PIN_SET&&HAL_GPIO_ReadPin(SD2_GPIO_Port,SD2_Pin)==GPIO_PIN_RESET)
    {Switch_State[3]=crsf_max;}
    else {Switch_State[3]=crsf_mid;}

    /* SE/SF: 单脚两位开关(上拉输入): 接地 = 打到 max, 断开 = min, 与 SA/SB 相同 */
    if(HAL_GPIO_ReadPin(SE_GPIO_Port,SE_Pin)==GPIO_PIN_RESET)
    {Switch_State[4]=crsf_max;}
    else {Switch_State[4]=crsf_min;}

    if(HAL_GPIO_ReadPin(SF_GPIO_Port,SF_Pin)==GPIO_PIN_RESET)
    {Switch_State[5]=crsf_max;}
    else {Switch_State[5]=crsf_min;}
}
/*============================微调(trim)=============================
   4 个微调 trim1~trim4, 每个由一对实体按键控制(前一个减, 后一个加):
     trim1 = TRIM6 / TRIM7      trim2 = TRIM5 / TRIM8
     trim3 = TRIM2 / TRIM3(与导航键 EXIT / ENTER 同脚)
     trim4 = TRIM4 / TRIM1(与导航键 DOWN / UP    同脚)
   所以只有 trim_mode = 1(短按电源键进入, 见 Ui.c 的 UI_Task)时才让这 8 个键产生步进,
   平时它们仍旧只当导航键用(两套逻辑互不干扰: 这里直接读引脚, 不碰 Key_State_* 队列).
   微调量单位 0.1% 满行程: ±TRIM_LIMIT(200) = ±20%, 单击 TRIM_STEP(5) = 0.5% */
volatile int16_t trim_val[TRIM_NUM] = {0, 0, 0, 0};
volatile uint8_t aetr_trim[4]       = {1, 2, 3, 4};   /* 默认 trim1->Ail, trim2->Ele, trim3->Thr, trim4->Rud */
volatile uint8_t aetr_trim_rev[4]   = {0, 0, 0, 0};   /* 输出 i 的微调方向: 0 = 正, 1 = 反 */
volatile uint8_t trim_mode          = 0U;             /* 1 = 微调模式(UI 负责开关并落盘) */

#define TRIM_DEB_N       2U     /* 连续 2 拍(20ms)电平一致才认 */
#define TRIM_REP_DELAY   400U   /* 按住多久起连发(ms) */
#define TRIM_REP_IV0     90U    /* 连发起始间隔(ms) */
#define TRIM_REP_IV_MIN  25U    /* 连发最快间隔(ms) */
#define TRIM_REP_ACCEL   8U     /* 每连发一步缩短的间隔(ms) */
#define TRIM_REP_MAX     200U   /* 加速计数上限(到顶后保持最快间隔) */

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       idx;      /* 微调号 0~3 */
    int8_t        dir;      /* -1 = 减, +1 = 加 */
} Trim_Btn_t;

/* 8 个实体微调键 -> (微调号, 增减): 前一个键减, 后一个键加 */
static const Trim_Btn_t trim_btn[8] = {
    { TRIM6_GPIO_Port, TRIM6_Pin, 0U, -1 }, { TRIM7_GPIO_Port, TRIM7_Pin, 0U, +1 },
    { TRIM5_GPIO_Port, TRIM5_Pin, 1U, -1 }, { TRIM8_GPIO_Port, TRIM8_Pin, 1U, +1 },
    { TRIM2_GPIO_Port, TRIM2_Pin, 2U, -1 }, { TRIM3_GPIO_Port, TRIM3_Pin, 2U, +1 },
    { TRIM4_GPIO_Port, TRIM4_Pin, 3U, -1 }, { TRIM1_GPIO_Port, TRIM1_Pin, 3U, +1 },
};

static volatile uint8_t trim_req = 0U;   /* 1 = 有一次"切换微调模式"请求(电源键短按 -> Trim_ModeToggleReq) */
static uint8_t  trim_lvl[8];             /* 去抖后的按住状态 */
static uint8_t  trim_raw[8];             /* 上次原始电平 */
static uint8_t  trim_cnt[8];             /* 连续一致次数 */
static uint32_t trim_t0[8];              /* 按下时刻 */
static uint32_t trim_tl[8];              /* 上次步进时刻 */
static uint16_t trim_n[8];               /* 已连发步数(加速用) */

/* 微调量 -> 通道 counts: 1000 = 满行程(crsf_max - crsf_min) */
int16_t Trim_Counts(uint8_t n)
{
    if (n >= TRIM_NUM)
        return 0;
    return (int16_t)(((int32_t)trim_val[n] * (int32_t)(crsf_max - crsf_min)) / 1000);
}

/* 请求切换微调模式(由 Schedule.c 的电源键短按调用: 1ms 槽, 只置标志不阻塞) */
void Trim_ModeToggleReq(void)
{
    trim_req = 1U;
}

/* UI 消费一次切换请求: 1 = 有一次 */
uint8_t Trim_ModeEvent(void)
{
    if (trim_req == 0U)
        return 0U;
    trim_req = 0U;
    return 1U;
}

/* 连发间隔: 从 iv0 起每步减 accel, 最快到 iv_min */
static uint32_t Trim_Iv(uint16_t n)
{
    uint32_t dec = (uint32_t)n * TRIM_REP_ACCEL;

    return (dec >= (TRIM_REP_IV0 - TRIM_REP_IV_MIN)) ? (uint32_t)TRIM_REP_IV_MIN
                                                    : (uint32_t)(TRIM_REP_IV0 - dec);
}

/* 一步微调(只有微调模式才动) */
static void Trim_Step(uint8_t k)
{
    int16_t v;

    if (trim_mode == 0U)
        return;
    v = (int16_t)((int16_t)trim_val[trim_btn[k].idx] + (int16_t)(trim_btn[k].dir * TRIM_STEP));
    if (v >  TRIM_LIMIT) v =  TRIM_LIMIT;
    if (v < -TRIM_LIMIT) v = -TRIM_LIMIT;
    trim_val[trim_btn[k].idx] = v;
}

/* 10ms 槽: 8 个微调键去抖 + 长按加速(独立于 UI 的连发逻辑) */
void Trim_Task(void)
{
    uint32_t now = HAL_GetTick();

    for (uint8_t k = 0U; k < 8U; k++)
    {
        uint8_t raw = (HAL_GPIO_ReadPin(trim_btn[k].port, trim_btn[k].pin) == GPIO_PIN_RESET) ? 1U : 0U;

        if (raw == trim_raw[k])
        {
            if (trim_cnt[k] < TRIM_DEB_N) trim_cnt[k]++;
        }
        else
        {
            trim_raw[k] = raw;
            trim_cnt[k] = 0U;
        }
        if (trim_cnt[k] < TRIM_DEB_N)
            continue;                       /* 还在抖动 */

        if (trim_lvl[k] != raw)             /* 电平变化(按下/松开) */
        {
            trim_lvl[k] = raw;
            if (raw != 0U)
            {
                Trim_Step(k);               /* 按下边沿: 立刻一步 */
                trim_t0[k] = now;
                trim_tl[k] = now;
                trim_n[k]  = 0U;
            }
        }
        else if ((raw != 0U) && (trim_mode != 0U))
        {
            uint32_t iv;

            if ((now - trim_t0[k]) < TRIM_REP_DELAY)
                continue;                   /* 还没到连发时刻 */
            iv = Trim_Iv(trim_n[k]);
            while ((now - trim_tl[k]) >= iv)   /* 补步(now 固定 -> 循环必然收敛) */
            {
                trim_tl[k] += iv;
                Trim_Step(k);
                if (trim_n[k] < TRIM_REP_MAX) trim_n[k]++;
                iv = Trim_Iv(trim_n[k]);
            }
        }
    }

#if !BOARD_PWR_HAS_LATCH
    /* 自锁开关版: 没有电源键可用, 用板上的 TRIM_MODE 键(PA14, 上拉, 按下读低)当切换键 */
    {
        static uint8_t tm_raw = 0U, tm_cnt = 0U, tm_lvl = 0U;
        uint8_t raw = (HAL_GPIO_ReadPin(TRIM_MODE_GPIO_Port, TRIM_MODE_Pin) == GPIO_PIN_RESET) ? 1U : 0U;

        if (raw == tm_raw) { if (tm_cnt < TRIM_DEB_N) tm_cnt++; }
        else { tm_raw = raw; tm_cnt = 0U; }
        if ((tm_cnt >= TRIM_DEB_N) && (tm_lvl != raw))
        {
            tm_lvl = raw;
            if (raw != 0U) trim_req = 1U;
        }
    }
#endif
}

/*============================按键=============================*/
volatile uint8_t Key_State_Queue[4]={0};
volatile uint8_t Key_State_Now[4]={0};
volatile uint8_t Key_State_Last[4]={0};
volatile uint32_t Key_State_Time[4]={0};   /* 该事件的产生时刻(HAL_GetTick): UI 用它丢弃过期事件 */
static void Key_State_Queue_Update(void){
    for(uint8_t i=0;i<4;i++){
        /* 锁存(不是累加): 按过一次这种事件只算一次, 免得同一次按压的抖动/双沿
           被 UI 消费两回(例如第一次进菜单后同一次按压又被当成菜单项动作) */
        if(Key_State_Now[i]==1&&Key_State_Last[i]==0){
            Key_State_Queue[i]=1;
            Key_State_Time[i]=HAL_GetTick();
        }
    }
}
static uint8_t Btn_Read(uint8_t id){
    switch(id){
        case 0: return (HAL_GPIO_ReadPin(UP_GPIO_Port, UP_Pin) == GPIO_PIN_RESET);
        case 1: return (HAL_GPIO_ReadPin(DOWN_GPIO_Port, DOWN_Pin) == GPIO_PIN_RESET);
        case 2: return (HAL_GPIO_ReadPin(ENTER_GPIO_Port, ENTER_Pin) == GPIO_PIN_RESET);
        case 3: return (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin) == GPIO_PIN_RESET);
        default: return 0;
    }
}
 void Key_Read(void){
    for(uint8_t i=0;i<4;i++){
        Key_State_Now[i]=Btn_Read(i);
    }
    Key_State_Queue_Update();
    for(uint8_t i=0;i<4;i++){
        Key_State_Last[i]=Key_State_Now[i];
    }
 }
