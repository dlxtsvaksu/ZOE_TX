#ifndef __FUNC_H__
#define __FUNC_H__

#include "stm32f103xb.h"
#include "stm32f1xx_hal.h"

typedef struct{
    uint16_t ADC_MIN;
    uint16_t ADC_MID;
    uint16_t ADC_MAX;
} ADC_CALI_t;
typedef enum{
    Ail=0,
    Ele,
    Thr,
    Rud,
} ADC_Channel_TypeDef;
typedef enum{
    UP=0,
    DOWN,
    ENTER,
    EXIT,
} Key_TypeDef;
typedef enum{
    SA=0,
    SB,
    SC,
    SD,
    SE,
    SF,
} Switch_TypeDef;

extern volatile ADC_CALI_t user_adc_calibration[4];   /* 摇杆校准表(可由 Flash 恢复) */
extern volatile uint8_t state;
extern volatile uint16_t aetr[4];
extern volatile uint16_t adc_filt[4];   /* 平均滤波后的 ADC 值 */
void Adc_FilterTask(void);              /* 1ms 调用: ADC 平均滤波 */
extern volatile uint16_t adc_dev_max[4];  /* 相对中位的最大偏差(诊断) */
extern volatile int8_t   aetr_input[4];   /* 前 4 通道方向: +1 正 / -1 反 (掉电保存) */
extern volatile uint8_t  aetr_src[4];     /* 前 4 通道输入源(0~3, 掉电保存) */
/*============================混控=============================*/
#define MODEL_MAX   8U     /* 模型数量(0 ~ MODEL_MAX-1) */
#define MIX_MAX     64U    /* 混控条目上限(存 Flash 记录里, 所有模型共用) */
#define MIX_PER_CH  4U     /* 每个通道最多 4 个输入源 */
#define MIX_SRC_NUM 10U    /* 输入源种类: 0~3 = AETR(IN1~IN4), 4~9 = 开关 A~F */

typedef struct {
    uint8_t model;      /* 模型号 0 ~ MODEL_MAX-1 */
    uint8_t ch;         /* 目标通道 1~16 */
    uint8_t src;        /* 输入源 0~7 */
    int8_t  weight;     /* 权重 -100 ~ +100 (%) */
} Mix_t;

extern volatile uint8_t model_cur;          /* 当前模型序号(掉电保存) */
extern Mix_t            mix_ram[MIX_MAX];   /* 当前模型的混控(上电从 Flash 过滤出来) */
extern uint8_t          mix_ram_cnt;        /* 当前模型的混控条数 */

uint8_t Mix_LoadForModel(uint8_t model);    /* 从 Flash 过滤该模型的混控; 1 = 记录有效 (实现在 Store.c) */
uint8_t Mix_ChCount(uint8_t ch);            /* 该通道当前有几个输入源 */
Mix_t  *Mix_ChAt(uint8_t ch, uint8_t n);    /* 该通道第 n 个输入源(0 起, 按数组顺序) */
uint8_t Mix_Add(uint8_t ch);                /* 添加一个输入源(默认 IN1 / +100%) */
uint8_t Mix_Del(uint8_t ch, uint8_t n);     /* 删除该通道第 n 个, 后面的补到前面 */
extern volatile uint16_t ch_out[16];      /* 最终通道值(混控后, 已限幅) */
void    Mix_Apply(void);                     /* 每 5ms: 基础值 + 混控 + 限幅 */
extern volatile uint16_t batt_mv;          /* 电池电压(mV): ADC_IN6 经 10k+10k 分压 */
#define BATT_LOW_MV 3400U                  /* 低压告警阈值(mV), 按电池容量改 */
void Batt_Task(void);                      /* 1ms 槽: 电池电压平均 */
extern volatile uint8_t  adc_cal_bad;    /* 1 = 标定行程过小, 已用默认行程兜底 */
void Adc_CalibCheck(void);                /* 标定表合理性检查/兜底 */
void Adc_DevReset(void);                 /* 偏差最大值复位 */
extern volatile uint8_t Key_State_Queue[4];
extern volatile uint8_t Key_State_Now[4];    /* 按键当前电平: 1 = 按住 (长按判定用) */
extern volatile uint32_t Key_State_Time[4];  /* 队列里那条事件的产生时刻(HAL_GetTick): UI 用来丢过期事件 */
extern volatile uint16_t Switch_State[6];   /* 0~5 = SA/SB/SC/SD/SE/SF, 值域见下 */

#define crsf_min 172 
#define crsf_mid 992
#define crsf_max 1811

void Adc_convert_Crsf(void);
void ADC_Calibrate(void);
void Switch_Read(void);
void Key_Read(void);
#endif