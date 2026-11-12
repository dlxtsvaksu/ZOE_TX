#ifndef __SCHEDULE_H__
#define __SCHEDULE_H__

#include "stm32f1xx_hal.h"
#include "Func.h"
#include "BoardCfg.h"    /* 硬件版本开关(自锁开关版 / 长按开关机版), 用户填 BOARD_PWR_VERSION */

typedef void (*TaskFunc_t)(void);

typedef struct {
    TaskFunc_t func;
    uint32_t   interval_ms;
    uint32_t   last_run;
} Task_t;

void Schedule_Run(void); //放在主循环里

/* ========================== 电源键(长按开机 / 软关机) ==========================
   硬件版本见 BoardCfg.h(BOARD_PWR_VERSION), 两个版本下面这些变量都存在(值不同):
     自锁开关版  : 上电即 pwr_latched = 1, pwr_hold_ms 钉满格, pwr_off_ms 恒 0, pwr_off_done 恒 0
                   -> 没有开机门禁, 也没有软关机(断电只靠拨开关)
     长按开关机版: Schedule.c 的 power_detect() 维护真实毫秒(下面的说明按这版写);
                   Ui.c 的开机进度条直接显示 pwr_hold_ms, 所以"进度条填满"与"PWR_EN 使能"
                   永远落在同一时刻(见 Schedule.c 说明); 软关机完成后 pwr_off_done = 1,
                   Ui.c 用它在未锁存且未重新按键时显示静态"关机"页 */
#define PWR_OFF_MS 3000U                /* 锁存后再长按电源键多久 = 软关机(ms), Ui.c 画倒计时也要用(仅长按版) */
extern volatile uint32_t pwr_hold_ms;   /* 本次电源键已按住时长(ms): 0 = 没按住 / 已松开(自锁版恒为满格) */
extern volatile uint8_t  pwr_latched;   /* 1 = 已开机(自锁版上电即 1) / 已锁存(长按版锁存后才 1) */
extern volatile uint32_t pwr_off_ms;    /* 锁存后本次按压已持续时间(ms): 0 = 没在按(自锁版恒 0) */
extern volatile uint8_t  pwr_off_done;  /* 1 = 软关机已完成, Ui.c 显示静态"关机"页(自锁版恒 0) */
void power_detect(void);                /* 1ms 槽: 长按 1.2s 锁存供电 / 再长按 3s 软关机(自锁版为空实现) */

#endif
