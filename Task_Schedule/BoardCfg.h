#ifndef __BOARD_CFG_H__
#define __BOARD_CFG_H__

#define BOARD_PWR_LATCH_SWITCH  1U      /* 自锁开关版 */
#define BOARD_PWR_LONG_PRESS    2U      /* 长按开关机版 */

#define BOARD_PWR_VERSION   BOARD_PWR_LONG_PRESS    /* <== 用户自己填: 1U 或 2U */

/* 填错立刻在编译期报错, 而不是静静编出一个"怪固件" */
#if (BOARD_PWR_VERSION != BOARD_PWR_LATCH_SWITCH) && (BOARD_PWR_VERSION != BOARD_PWR_LONG_PRESS)
#error "BOARD_PWR_VERSION 只能填 BOARD_PWR_LATCH_SWITCH(1U) 或 BOARD_PWR_LONG_PRESS(2U), 见 BoardCfg.h"
#endif

/* 派生宏: 1 = 需要电源锁存 / 开机门禁 / 软关机 */
#if (BOARD_PWR_VERSION == BOARD_PWR_LONG_PRESS)
  #define BOARD_PWR_HAS_LATCH  1
#else
  #define BOARD_PWR_HAS_LATCH  0
#endif

/* ------------------------- 长按版: 电源键接在哪个脚 -------------------------
   Schedule.c 的 power_detect() 按这里选源:
     BOARD_PWR_KEY_SRC_DETECT : PB13(PWR_DETECT), 按下读高 -- 本板设计, 默认;
     BOARD_PWR_KEY_SRC_EXIT   : PA15(EXIT),       按下读低 -- 早期代码误读的脚(注释写的是
                                PB13 却读了 EXIT), 留着这个开关只是为了将来换板方便. */
#define BOARD_PWR_KEY_SRC_DETECT  2U
#define BOARD_PWR_KEY_SRC_EXIT    1U
#define BOARD_PWR_KEY_SRC   BOARD_PWR_KEY_SRC_DETECT   /* <== 用户自己填(换板才需要改) */

/* ------------------- 版本页(VERSION 行)右上角的硬件标识 ------------------- */
#if (BOARD_PWR_VERSION == BOARD_PWR_LONG_PRESS)
  #define BOARD_HW_TAG  "HW:LATCH"
#else
  #define BOARD_HW_TAG  "HW:SW"
#endif

#endif /* __BOARD_CFG_H__ */
