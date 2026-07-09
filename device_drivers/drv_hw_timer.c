/**
 * @file    drv_hw_timer.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-07-1
 * @brief   TIM6 硬件定时器驱动实现（用于 ADC 触发等周期性任务）
 * @attention
 *
 * Copyright (c) 2026 E1_PRO 项目组
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_hw_timer.h"

#include "tim.h"

/* Private constants ---------------------------------------------------------*/

#define DRV_HW_TIMER_CH_NUM ((uint32_t)DRV_HW_TIMER_NUM)

/* Private variables ---------------------------------------------------------*/

/** @brief 通道 → TIM 句柄映射 */
static TIM_HandleTypeDef* const s_ch_to_htim[DRV_HW_TIMER_CH_NUM] = {
    [DRV_HW_TIMER_ADC] = &htim6,
};

/** @brief 各通道注册的回调 */
static drv_hw_timer_callback_t s_callbacks[DRV_HW_TIMER_CH_NUM] = { NULL };

/** @brief 各通道初始化标志 */
static bool s_initialized[DRV_HW_TIMER_CH_NUM] = { false };

/** @brief 各通道运行标志 */
static volatile bool s_running[DRV_HW_TIMER_CH_NUM] = { false };

/* Private functions ---------------------------------------------------------*/

/**
 * @brief TIM 句柄 → 通道号转换
 */
static drv_hw_timer_ch_t drv_hw_timer_htim_to_ch(TIM_HandleTypeDef* htim)
{
    if (htim == &htim6)
        return DRV_HW_TIMER_ADC;
    return (drv_hw_timer_ch_t)-1;
}

/* Exported functions --------------------------------------------------------*/

void drv_hw_timer_init(void)
{
    for (uint32_t ch = 0; ch < DRV_HW_TIMER_CH_NUM; ch++) {
        s_initialized[ch] = true;
        s_running[ch] = false;
    }
}

void drv_hw_timer_deinit_all(void)
{
    for (uint32_t ch = 0; ch < DRV_HW_TIMER_CH_NUM; ch++) {
        if (s_running[ch]) {
            drv_hw_timer_stop((drv_hw_timer_ch_t)ch);
        }
        s_initialized[ch] = false;
        s_callbacks[ch] = NULL;
    }
}

bool drv_hw_timer_is_initialized(drv_hw_timer_ch_t ch)
{
    if (ch >= DRV_HW_TIMER_CH_NUM)
        return false;
    return s_initialized[ch];
}

/* --- 启动 / 停止 --- */

drv_hw_timer_error_t drv_hw_timer_start(drv_hw_timer_ch_t ch)
{
    if (ch >= DRV_HW_TIMER_CH_NUM) {
        return DRV_HW_TIMER_ERROR_INVALID_PARAM;
    }
    if (!s_initialized[ch]) {
        return DRV_HW_TIMER_ERROR_UNINITIALIZED;
    }
    if (s_running[ch]) {
        return DRV_HW_TIMER_OK;
    }

    if (HAL_TIM_Base_Start_IT(s_ch_to_htim[ch]) != HAL_OK) {
        return DRV_HW_TIMER_ERROR_UNINITIALIZED;
    }

    s_running[ch] = true;
    return DRV_HW_TIMER_OK;
}

drv_hw_timer_error_t drv_hw_timer_stop(drv_hw_timer_ch_t ch)
{
    if (ch >= DRV_HW_TIMER_CH_NUM) {
        return DRV_HW_TIMER_ERROR_INVALID_PARAM;
    }
    if (!s_running[ch]) {
        return DRV_HW_TIMER_OK;
    }

    HAL_TIM_Base_Stop_IT(s_ch_to_htim[ch]);

    s_running[ch] = false;
    return DRV_HW_TIMER_OK;
}

bool drv_hw_timer_is_running(drv_hw_timer_ch_t ch)
{
    if (ch >= DRV_HW_TIMER_CH_NUM)
        return false;
    return s_running[ch];
}

/* --- 回调注册 --- */

drv_hw_timer_error_t drv_hw_timer_register_callback(drv_hw_timer_ch_t ch,
    drv_hw_timer_callback_t callback)
{
    if (ch >= DRV_HW_TIMER_CH_NUM) {
        return DRV_HW_TIMER_ERROR_INVALID_PARAM;
    }
    if (!s_initialized[ch]) {
        return DRV_HW_TIMER_ERROR_UNINITIALIZED;
    }

    s_callbacks[ch] = callback;
    return DRV_HW_TIMER_OK;
}

/* ===== HAL 回调 ===== */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    const drv_hw_timer_ch_t ch = drv_hw_timer_htim_to_ch(htim);
    if (ch == (drv_hw_timer_ch_t)-1) {
        return;
    }

    if (s_callbacks[ch]) {
        s_callbacks[ch](ch);
    }
}
