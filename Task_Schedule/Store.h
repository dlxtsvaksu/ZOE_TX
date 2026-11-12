#ifndef __STORE_H__
#define __STORE_H__

#include <stdint.h>

/* 校准参数持久化: 存放在内部 Flash 最后一页
   - 上电调用 Store_LoadCalib(): 读到有效数据才覆盖默认校准表
   - 校准完成调用 Store_SaveCalib(): 擦页+写入+回读校验 */
uint8_t Store_LoadCalib(void);   /* 1 = 读到有效数据, 0 = 保持默认值 */
uint8_t Store_SaveCalib(void);   /* 1 = 保存并回读校验通过 */
uint8_t Store_SaveInput(void);   /* 保存前 4 通道方向(与校准同一条记录) */

/* 存储状态(供界面显示): 1 = 成功 */
extern volatile uint8_t store_ld_ok;   /* 上电读取是否成功 */
extern volatile uint8_t store_sv_ok;   /* 最近一次保存是否成功 */
extern volatile uint8_t store_err;     /* 失败时的 HAL 错误码(0xEE = 回读校验失败) */

#endif /* __STORE_H__ */