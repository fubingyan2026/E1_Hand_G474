/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    behavior_task.c
 * @brief   电机行为任务 — sw_timer 驱动 srv_motor_behavior_step (10ms)
 */

#include "behavior_task.h"

#include "srv_motor_behavior.h"
#include "sw_timer.h"

#define BHV_PERIOD_MS 1U

static sw_timer_t s_timer;

static void behavior_timer_cb(void* user_data)
{
    (void)user_data;
    srv_motor_behavior_step();
    // srv_motor_step();
}

void behavior_task_init(void)
{
    srv_motor_init();
    srv_motor_behavior_init();

    sw_timer_init(&s_timer, &(sw_timer_config_t) { .priority = SW_TIMER_PRIO_NORMAL, .callback = behavior_timer_cb });
    sw_timer_start(&s_timer, BHV_PERIOD_MS, 0);
}
