/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    can_task.c
 * @brief   CAN 通信任务 — 主机上报 + 从板控制 + RX 接收
 */

#include "can_task.h"

#include "drv_can.h"
#include "sw_timer.h"

#include <string.h>

/* Private constants ---------------------------------------------------------*/

#define TASK_PERIOD_MS (10U)
#define REPORT_INTERVAL_MS (100U)

/* Private variables ---------------------------------------------------------*/

static sw_timer_t s_timer;
static uint16_t s_report_ms;

/* Private function prototypes -----------------------------------------------*/

static void can_timer_cb(void* user_data);

static bool can_send_frame(uint16_t can_id, const uint8_t* data, uint8_t len);
static void can_rx_callback(drv_can_channel_t ch, const drv_can_msg_t* msg);

/* Exported functions --------------------------------------------------------*/

void can_task_init(void)
{
    drv_can_init();
  

    /* 注册 CAN 接收回调 */
    drv_can_register_rx_callback(DRV_CAN_CH_1, can_rx_callback);

    s_report_ms = 0;

    const sw_timer_config_t timer_cfg = {
        .priority = SW_TIMER_PRIO_NORMAL,
        .callback = can_timer_cb,
        .user_data = NULL,
    };
    sw_timer_init(&s_timer, &timer_cfg);
    sw_timer_start(&s_timer, TASK_PERIOD_MS, 0);
}

void can_task_tick(void)
{
  
}

/* Private functions ---------------------------------------------------------*/

static void can_timer_cb(void* user_data)
{
  
}

static bool can_send_frame(uint16_t can_id, const uint8_t* data, uint8_t len)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_1)) {
        return false;
    }

    drv_can_msg_t msg;
    msg.id = can_id;
    msg.is_extended = false;
    msg.dlc = len;
    memcpy(msg.data, data, len);

    return drv_can_send(DRV_CAN_CH_1, &msg) == DRV_CAN_OK;
}


/* --- CAN RX 回调 --- */

static void can_rx_callback(drv_can_channel_t ch, const drv_can_msg_t* msg)
{
    (void)ch;

    if (!msg)
        return;

}
