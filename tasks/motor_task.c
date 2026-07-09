/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    motor_task.c
 * @brief   电机实时通信任务 — 主循环 poll srv_motor_step
 *
 * 无固定定时器，motor_task_poll 在主循环全速调用。
 * 发送频率由 srv_motor 内部 FSM 的 millis() / drv_uart_is_tx_busy() 控制。
 */

#include "motor_task.h"

#include "srv_motor.h"

/* Exported functions --------------------------------------------------------*/

void motor_task_init(void)
{
    srv_motor_init();
}

void motor_task_poll(void)
{
    srv_motor_step();
}
