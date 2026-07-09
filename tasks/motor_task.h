/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    motor_task.h
 * @brief   电机实时通信任务 — 主循环 poll srv_motor_step
 *
 * 无定时器，srv_motor_step 由主循环全速调用。
 * FSM 内部通过 millis() 控制 3ms 广播周期，drv_uart_is_tx_busy() 控制发送间隔。
 */

#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 电机分组定义 */
#define MOTOR_GROUP_A_NUM (5U) /**< USART2 电机数量 */
#define MOTOR_GROUP_B_NUM (4U) /**< USART3 电机数量 */
#define MOTOR_TOTAL (9U) /**< 电机总数 */

void motor_task_init(void);
void motor_task_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_TASK_H */
