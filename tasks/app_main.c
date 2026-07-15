/*
 * Copyright (c) 2022 HPMicro
 * Copyright (c) 2026 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    app_main.c
 * @brief   G1_Hand 灵巧手主入口
 *
 * 初始化硬件后进入主循环，运行 sw_timer 协作式调度。
 */

#include "behavior_task.h"
#include "can_task.h"
#include "daemon_task.h"
#include "drv_systick.h"
#include "drv_uart.h"
#include "led_task.h"
#include "log_task.h"
#include "srv_motor.h"
#include "sw_timer.h"

int app_main(void)
{
    /* 系统节拍（延时/时间戳） */
    delay_init();

    /* UART 驱动公共初始化（USART1/2/3） */
    drv_uart_init();

    /* 日志输出（UART DMA） */
    log_task_init();

    /* CAN 通信 */
    can_task_init();

    /* LED 状态指示 */
    led_task_init();

    /* 电机行为控制（10ms FSM + 故障监控） */
    behavior_task_init();

    /* 守护进程监控（9 电机反馈超时看门狗，依赖 behavior_task 已注册电机句柄） */
    daemon_task_init();

    /* 主循环：sw_timer 驱动日志/LED/CAN/FB，motor 全速 poll */
    for (;;) {
        drv_uart_rx_restart(DRV_UART_CH_1);
        drv_uart_rx_restart(DRV_UART_CH_2);
        sw_timer_tick(millis());
        sw_timer_task();
        srv_motor_step();
    }

    return 0;
}
