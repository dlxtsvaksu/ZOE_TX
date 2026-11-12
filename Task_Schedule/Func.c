#include "Func.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include <stdint.h>
#include "main.h"
#include "Store.h"
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
