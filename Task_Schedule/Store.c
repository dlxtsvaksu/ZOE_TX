/**
 * @file Store.c
 * @brief 校准参数持久化: 存放在内部 Flash 的最后一页(64KB 器件 = 0x0800FC00)
 * @note STM32F1 擦写内部 Flash 会停顿内核约 20~40ms, 本函数只在校准完成时调用一次
 */
#include "Store.h"
#include "Func.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stddef.h>

#define STORE_CALIB_ADDR   0x0800FC00U      /* 64KB 器件最后一页(1KB) */
#define STORE_CALIB_MAGIC  0xCA11B2A7U

typedef struct {
    uint32_t   magic;
    ADC_CALI_t cal[4];
    int8_t     aetr_input[4];    /* 前 4 通道方向(±1), 与校准同一条记录 */
    uint8_t    aetr_src[4];      /* 前 4 通道输入源(0~3), 与校准同一条记录 */
    uint16_t   sum;                         /* 校验和(只覆盖到本字段偏移, 不含本字段) */
    /* ---- 以下不参与校验和: 保证已存的校准记录仍然有效, 读取时逐项做范围校验 ---- */
    uint8_t    model;                       /* 当前模型序号 */
    uint8_t    mix_cnt;                     /* 混控总条数(所有模型) */
    Mix_t      mix[MIX_MAX];                /* 混控数组 */
    /* ---- 微调(同样不参与校验和: 老记录这些字节是 0xFF, 读取时逐项按范围/步长兜底) ---- */
    uint8_t    aetr_trim[4];                /* 输出 i 挂哪个微调: 0 = 不挂, 1~4 = trim1~trim4 */
    int16_t    trim_val[TRIM_NUM];          /* 微调量(0.1% 满行程, ±TRIM_LIMIT) */
    uint8_t    aetr_trim_rev[4];            /* 输出 i 的微调方向: 0 = 正, 1 = 反 */
} Store_Calib_t;

/* 存储状态(供界面显示): 1 = 成功 */
volatile uint8_t store_ld_ok = 0U;   /* 上电读取是否成功 */
volatile uint8_t store_sv_ok = 0U;   /* 最近一次保存是否成功 */
volatile uint8_t store_err   = 0U;   /* 失败时的 HAL 错误码(0xEE = 回读校验失败) */

/**
 * @brief 校验和(magic + cal, 不含 sum 本身)
 */
static uint16_t Store_Sum(const Store_Calib_t *d)
{
    const uint8_t *p = (const uint8_t *)d;
    uint16_t s = 0U;
    uint32_t n = (uint32_t)offsetof(Store_Calib_t, sum);   /* 只算 magic+cal, 不含 sum 与尾部填充 */

    for (uint32_t i = 0U; i < n; i++)
        s = (uint16_t)(s + p[i]);

    return s;
}

/**
 * @brief 上电读取校准参数
 */
uint8_t Store_LoadCalib(void)
{
    const Store_Calib_t *d = (const Store_Calib_t *)STORE_CALIB_ADDR;
    store_ld_ok = 0U;
    if ((d->magic != STORE_CALIB_MAGIC) || (d->sum != Store_Sum(d)))
        return 0U;                          /* 未校准/数据无效: 保持默认值 */
    for (uint8_t i = 0U; i < 4U; i++)
    {
        user_adc_calibration[i].ADC_MIN = d->cal[i].ADC_MIN;   /* 恢复校准表(之前被我误删了) */
        user_adc_calibration[i].ADC_MID = d->cal[i].ADC_MID;
        user_adc_calibration[i].ADC_MAX = d->cal[i].ADC_MAX;
        aetr_input[i] = (d->aetr_input[i] < 0) ? (int8_t)-1 : (int8_t)1;
        aetr_src[i]   = (d->aetr_src[i] < 4U) ? d->aetr_src[i] : (uint8_t)i;
        /* 微调: 老记录(没有这两个字段)读出来是 0xFF, 按范围/步长兜底到默认值 */
        aetr_trim[i]  = (d->aetr_trim[i] <= TRIM_NUM) ? d->aetr_trim[i] : (uint8_t)(i + 1U);
        {
            int16_t tv = d->trim_val[i];

            if ((tv > TRIM_LIMIT) || (tv < -TRIM_LIMIT) || ((tv % TRIM_STEP) != 0))
                tv = 0;                         /* 越界或不是 TRIM_STEP 的整数倍(老记录 = -1) -> 归零 */
            trim_val[i] = tv;
        }
        /* 方向: 老记录 0xFF -> 0(正) */
        aetr_trim_rev[i] = (d->aetr_trim_rev[i] <= 1U) ? d->aetr_trim_rev[i] : 0U;
    }
    model_cur = (d->model < MODEL_MAX) ? d->model : 0U;   /* 模型号越界则回 0 */
    store_ld_ok = 1U;
    return 1U;
}

/**
 * @brief 保存校准参数到 Flash
 */
/**
 * @brief 保存通道方向(±1): 与校准共用同一条 Flash 记录, 整条重写
 */
/**
 * @brief 从 Flash 读取全部混控, 只把 model 匹配的搬进 RAM(其余直接丢掉)
 * @note  上电初始化时调用; 切换模型时也调用一次
 */
uint8_t Mix_LoadForModel(uint8_t model)
{
    const Store_Calib_t *d = (const Store_Calib_t *)STORE_CALIB_ADDR;
    uint8_t n = 0U;

    mix_ram_cnt = 0U;
    if ((d->magic != STORE_CALIB_MAGIC) || (d->sum != Store_Sum(d)))
        return 0U;                                   /* 记录无效: 当前模型没有混控 */
    if (model >= MODEL_MAX) model = 0U;
    n = (d->mix_cnt > MIX_MAX) ? (uint8_t)MIX_MAX : d->mix_cnt;   /* 全部读一遍 */

    for (uint8_t i = 0U; i < n; i++)
    {
        if (d->mix[i].model != model)                  continue;   /* 不是这个模型: 丢掉 */
        if (d->mix[i].ch < 1U || d->mix[i].ch > 16U)   continue;   /* 范围校验 */
        if (d->mix[i].src >= MIX_SRC_NUM)              continue;
        if (mix_ram_cnt >= MIX_MAX) break;
        mix_ram[mix_ram_cnt] = d->mix[i];
        if (mix_ram[mix_ram_cnt].weight >  100) mix_ram[mix_ram_cnt].weight =  100;
        if (mix_ram[mix_ram_cnt].weight < -100) mix_ram[mix_ram_cnt].weight = -100;
        mix_ram_cnt++;
    }
    return 1U;
}

uint8_t Store_SaveInput(void)
{
    return Store_SaveCalib();
}

uint8_t Store_SaveCalib(void)
{
    Store_Calib_t ram;
    FLASH_EraseInitTypeDef er = {0};
    uint32_t page_err = 0U;
    const uint16_t *src;
    uint32_t addr;
    uint8_t  ok = 1U;
    store_sv_ok = 0U;
    store_err   = 0U;
    memset(&ram, 0, sizeof(ram));
    ram.magic = STORE_CALIB_MAGIC;
    for (uint8_t i = 0U; i < 4U; i++)
    {
        ram.cal[i].ADC_MIN = user_adc_calibration[i].ADC_MIN;
        ram.cal[i].ADC_MID = user_adc_calibration[i].ADC_MID;
        ram.cal[i].ADC_MAX = user_adc_calibration[i].ADC_MAX;
        ram.aetr_input[i]  = (aetr_input[i] < 0) ? (int8_t)-1 : (int8_t)1;   /* ±1 归一 */
        ram.aetr_src[i]    = (aetr_src[i] < 4U) ? aetr_src[i] : (uint8_t)i;
        ram.aetr_trim[i]   = (aetr_trim[i] <= TRIM_NUM) ? aetr_trim[i] : (uint8_t)(i + 1U);
        {
            int16_t tv = trim_val[i];

            if (tv >  TRIM_LIMIT) tv =  TRIM_LIMIT;
            if (tv < -TRIM_LIMIT) tv = -TRIM_LIMIT;
            ram.trim_val[i] = tv;
        }
        ram.aetr_trim_rev[i] = (aetr_trim_rev[i] != 0U) ? 1U : 0U;
    }
    ram.model   = (uint8_t)model_cur;
    ram.mix_cnt = 0U;
    {   /* 先保留 Flash 里其它模型的混控(避免切模型丢数据) */
        const Store_Calib_t *old = (const Store_Calib_t *)STORE_CALIB_ADDR;
        if ((old->magic == STORE_CALIB_MAGIC) && (old->sum == Store_Sum(old)))
        {
            uint8_t n = (old->mix_cnt > MIX_MAX) ? (uint8_t)MIX_MAX : old->mix_cnt;
            for (uint8_t i = 0U; (i < n) && (ram.mix_cnt < MIX_MAX); i++)
                if (old->mix[i].model != (uint8_t)model_cur)
                    ram.mix[ram.mix_cnt++] = old->mix[i];
        }
    }
    for (uint8_t i = 0U; (i < mix_ram_cnt) && (ram.mix_cnt < MIX_MAX); i++)   /* 再写当前模型 */
    {
        Mix_t *m = &ram.mix[ram.mix_cnt];
        *m = mix_ram[i];
        m->model = (uint8_t)model_cur;
        if (m->ch  < 1U || m->ch > 16U)      m->ch  = 1U;
        if (m->src >= MIX_SRC_NUM)           m->src = 0U;
        if (m->weight >  100)                m->weight =  100;
        if (m->weight < -100)                m->weight = -100;
        ram.mix_cnt++;
    }
    ram.sum = Store_Sum(&ram);
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        store_err = (uint8_t)HAL_FLASH_GetError();
        return 0U;
    }
    er.TypeErase   = FLASH_TYPEERASE_PAGES;
    er.PageAddress = STORE_CALIB_ADDR;
    er.NbPages     = 1U;
    if (HAL_FLASHEx_Erase(&er, &page_err) != HAL_OK)
    {
        store_err = (uint8_t)HAL_FLASH_GetError();
        HAL_FLASH_Lock();
        return 0U;
    }
    src  = (const uint16_t *)&ram;
    addr = STORE_CALIB_ADDR;
    for (uint32_t i = 0U; i < (sizeof(Store_Calib_t) / 2U); i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, (uint64_t)src[i]) != HAL_OK)
        {
            ok = 0U;
            break;
        }
        addr += 2U;
    }
    HAL_FLASH_Lock();
    if (ok == 0U)
    {
        store_err = (uint8_t)HAL_FLASH_GetError();
        return 0U;
    }
    {
        const Store_Calib_t *d = (const Store_Calib_t *)STORE_CALIB_ADDR;
        if ((d->magic != STORE_CALIB_MAGIC) || (d->sum != Store_Sum(d)))
        {
            store_err = 0xEEU;              /* 回读校验失败 */
            return 0U;
        }
    }
    store_sv_ok = 1U;
    return 1U;
}