/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    can_task.c
 * @brief   CAN 通信任务 — sw_timer 驱动 srv_can 处理 + 反馈上报
 */

#include "can_task.h"

#include "drv_can.h"
#include "srv_can.h"
#include "sw_timer.h"

/* Private constants ---------------------------------------------------------*/

#define TASK_PERIOD_MS 10U
#define FB_INTERVAL_MS 100U
#define STATUS_INTERVAL_MS 500U

/* Private variables ---------------------------------------------------------*/

static sw_timer_t s_timer;
static uint8_t s_fb_tick;
static uint8_t s_status_tick;

/* Private function prototypes -----------------------------------------------*/

static void can_timer_cb(void* user_data);
static void can_rx_callback(drv_can_channel_t ch, const drv_can_msg_t* msg);

/* Exported functions --------------------------------------------------------*/

void can_task_init(void)
{
    drv_can_init();
    srv_can_init();

    drv_can_register_rx_callback(DRV_CAN_CH_1, can_rx_callback);

    const sw_timer_config_t cfg = {
        .priority = SW_TIMER_PRIO_NORMAL,
        .callback = can_timer_cb,
    };
    sw_timer_init(&s_timer, &cfg);
    sw_timer_start(&s_timer, TASK_PERIOD_MS, 0);
}

/* Private functions ---------------------------------------------------------*/

static void can_timer_cb(void* user_data)
{
    (void)user_data;

    srv_can_process();

    s_fb_tick++;
    s_status_tick++;

    if (s_fb_tick >= (FB_INTERVAL_MS / TASK_PERIOD_MS)) {
        s_fb_tick = 0;
        srv_can_send_feedback();
    }
    if (s_status_tick >= (STATUS_INTERVAL_MS / TASK_PERIOD_MS)) {
        s_status_tick = 0;
        srv_can_send_status();
    }
}

static void can_rx_callback(drv_can_channel_t ch, const drv_can_msg_t* msg)
{
    (void)ch;
    srv_can_on_rx(msg);
}
