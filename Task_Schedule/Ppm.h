#ifndef __PPM_H__
#define __PPM_H__

#include <stdint.h>

/* ========================== PPM 规格(只改这几个宏即可) ==========================
   帧结构(正逻辑): 每通道 [高 PPM_PULSE_US] + [低 (通道脉宽 - PPM_PULSE_US)];
   最后一通道的低电平再并上"帧同步低电平" = PPM_FRAME_US - Σ(通道路时间).
   约定: 信息在"相邻上升沿之间的间隔"(1000~2000us), 脉宽固定 400us —— 标准 PPM;
        接收机靠帧尾的长同步(>6ms)识别帧头, 再数中间有几个脉冲 = 几路.
   当前: 8 路 + 22.5ms 帧(44.4Hz). 8 路全打 2000us 时帧尾同步 = 22.5-16 = 6.5ms
        (常见识别门限 2.5~4ms, 仍有余量); 全中位时接收机量到的沿间隔约 12ms.
   改路数: 只改 PPM_CH(1~16, 沿表与中断逻辑自适应); 路数明显变多时要同步加大
        PPM_FRAME_US, 否则同步会被 PPM_SYNC_MIN_US 钳位(帧变长, 不标准但仍能解码) */
#define PPM_CH           8U        /* 输出通道数(1~16): 取 ch_out[0 .. PPM_CH-1] = 通道 1~8 */
#define PPM_PULSE_US     400U      /* 每通道脉冲宽度(固定, us): 常见 300~500 */
#define PPM_FRAME_US     22500U    /* 帧周期(us): 22500 = 44.4Hz / 20000 = 50Hz */
#define PPM_SYNC_MIN_US  4000U     /* 帧同步低电平下限(us): 通道全部打满时的保护 */
#define PPM_MIN_US       1000U     /* 通道脉宽下限(us): 摇杆一端 */
#define PPM_MAX_US       2000U     /* 通道脉宽上限(us): 摇杆另一端 */
#define PPM_MID_US       ((PPM_MIN_US + PPM_MAX_US) / 2U)   /* 中位脉宽(1500us): 通道表静态初值用 */

/* 通道脉宽表(1000~2000us; 静态初值 = 中位):
   PPM_Task 每 5ms 从 ch_out[] 换算写入, 中断在帧首取走 */
extern volatile uint16_t ppm_ch_us[PPM_CH];

/* 诊断计数: 在中断/5ms 槽里更新, 目前没有消费者(要挂诊断页时直接读) */
extern volatile uint16_t ppm_edge_cnt;    /* 已产生的沿数(正常约 2*PPM_CH*44/s) */
extern volatile uint16_t ppm_frame_cnt;   /* 已输出的帧数(正常约 44/s) */
extern volatile uint16_t ppm_frame_rate;  /* 实测帧率(帧/秒), 正常约 44 */

void PPM_Init(void);                            /* 上电启动: 只有 HAL_TIM_OC_Start_IT, TIM4 配置全在 Elrs_tx.ioc/tim.c */
void PPM_Task(void);                            /* 5ms 槽: ch_out[] -> ppm_ch_us[] + 帧率统计 */
void PPM_SetChannel(uint8_t ch, uint16_t us);   /* 直接指定某通道脉宽(us, 自动限幅) */

#endif
