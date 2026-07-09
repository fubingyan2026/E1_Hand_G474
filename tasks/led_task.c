/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    led_task.c
 * @brief   LED 状态指示任务 — 蓝色+红色双 LED 呼吸模式 (TIM1 PWM)
 */

#include "led_task.h"
#include "drv_systick.h"
#include "srv_led.h"
#include "sw_timer.h"

/** @brief LED 刷新周期 (ms) */
#define LED_TASK_REFRESH_PERIOD_MS (10U)

/** @brief 蓝色 LED 呼吸参数 */
#define LED_BLUE_BREATH_CYCLE_MS  (2000U)
#define LED_BLUE_BREATH_MIN_DUTY  (0U)
#define LED_BLUE_BREATH_MAX_DUTY  (1023U)

/** @brief 红色 LED 呼吸参数 */
#define LED_RED_BREATH_CYCLE_MS   (3000U)
#define LED_RED_BREATH_MIN_DUTY   (0U)
#define LED_RED_BREATH_MAX_DUTY   (1023U)

static srv_led_handle_t s_led_blue;
static srv_led_handle_t s_led_red;
static sw_timer_t s_led_timer;

/* ── 蓝色 LED write_pin 回调 ── */

static void led_blue_write_pin(uint16_t value)
{
}

/* ── 红色 LED write_pin 回调 ── */

static void led_red_write_pin(uint16_t value)
{
}

/* ── sw_timer 回调 ── */

static void led_timer_cb(void* user_data)
{
    (void)user_data;
    srv_led_task_refresh();
}

/* Exported functions --------------------------------------------------------*/

void led_task_init(void)
{
    srv_led_init(millis);

    /* ── 蓝色 LED: 呼吸模式 ── */
    srv_led_config_t cfg_blue = {
        .name             = "blue",
        .init_state       = SRV_LED_STATE_BREATHING,
        .write_pin        = led_blue_write_pin,
        .breath_cycle_ms  = LED_BLUE_BREATH_CYCLE_MS,
        .breath_min_duty  = LED_BLUE_BREATH_MIN_DUTY,
        .breath_max_duty  = LED_BLUE_BREATH_MAX_DUTY,
    };
    srv_led_register_static(&cfg_blue, &s_led_blue);

    /* ── 红色 LED: 呼吸模式 ── */
    srv_led_config_t cfg_red = {
        .name             = "red",
        .init_state       = SRV_LED_STATE_BREATHING,
        .write_pin        = led_red_write_pin,
        .breath_cycle_ms  = LED_RED_BREATH_CYCLE_MS,
        .breath_min_duty  = LED_RED_BREATH_MIN_DUTY,
        .breath_max_duty  = LED_RED_BREATH_MAX_DUTY,
    };
    srv_led_register_static(&cfg_red, &s_led_red);

    /* 启动 sw_timer 驱动 LED FSM */
    const sw_timer_config_t timer_cfg = {
        .priority  = SW_TIMER_PRIO_NORMAL,
        .callback  = led_timer_cb,
        .user_data = NULL,
    };
    sw_timer_init(&s_led_timer, &timer_cfg);
    sw_timer_start(&s_led_timer, LED_TASK_REFRESH_PERIOD_MS, 0);
}
