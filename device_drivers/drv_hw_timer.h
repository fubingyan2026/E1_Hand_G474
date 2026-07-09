/**
 * @file    drv_hw_timer.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-07-1
 * @brief   TIM6 硬件定时器驱动（用于 ADC 触发等周期性任务）
 * @attention
 *
 * Copyright (c) 2026 E1_PRO 项目组
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 *
 */

#ifndef __DRV_HW_TIMER_H
#define __DRV_HW_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 硬件定时器通道枚举
 */
typedef enum {
    DRV_HW_TIMER_ADC = 0,       /**< TIM6 - ADC 触发定时器 (5 kHz) */
    DRV_HW_TIMER_NUM,           /**< 定时器总数 */
} drv_hw_timer_ch_t;

/**
 * @brief 驱动错误码枚举
 */
typedef enum {
    DRV_HW_TIMER_OK = 0,                /**< 操作成功 */
    DRV_HW_TIMER_ERROR_UNINITIALIZED,   /**< 未初始化 */
    DRV_HW_TIMER_ERROR_INVALID_PARAM,   /**< 无效参数 */
} drv_hw_timer_error_t;

/**
 * @brief 定时器周期中断回调函数类型
 * @param ch  触发中断的定时器通道
 */
typedef void (*drv_hw_timer_callback_t)(drv_hw_timer_ch_t ch);

/* Exported functions prototypes ---------------------------------------------*/

/** @brief 初始化全部硬件定时器通道（内部句柄表，无需传参） */
void drv_hw_timer_init(void);

/** @brief 反初始化全部硬件定时器通道 */
void drv_hw_timer_deinit_all(void);

bool drv_hw_timer_is_initialized(drv_hw_timer_ch_t ch);

drv_hw_timer_error_t drv_hw_timer_start(drv_hw_timer_ch_t ch);
drv_hw_timer_error_t drv_hw_timer_stop(drv_hw_timer_ch_t ch);
bool drv_hw_timer_is_running(drv_hw_timer_ch_t ch);

/**
 * @brief 注册定时器周期中断回调
 * @param ch       定时器通道
 * @param callback 函数指针，在中断上下文中执行
 */
drv_hw_timer_error_t drv_hw_timer_register_callback(drv_hw_timer_ch_t ch,
    drv_hw_timer_callback_t callback);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_HW_TIMER_H */
