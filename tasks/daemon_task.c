/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    daemon_task.c
 * @brief   守护进程监控任务 — 为 9 个电机注册 daemon 看门狗
 *
 * 喂狗策略（轮询式，零侵入 service 层）：
 * 10ms 周期检查每个电机反馈时间戳 (time_fb / time_status) 是否推进，
 * 推进则 daemon_reload() 喂狗；随后 daemon_task() 做超时判定。
 * 掉线/恢复仅打日志告警，不干预控制。
 *
 * 依赖：behavior_task_init() 已完成 srv_motor_init()（9 电机句柄已注册）。
 */

/* Includes ------------------------------------------------------------------*/
#include "daemon_task.h"

#include "daemon.h"
#include "drv_systick.h"
#include "log.h"
#include "srv_motor.h"
#include "sw_timer.h"

/* Private constants ---------------------------------------------------------*/

/** @brief 检查/派发周期 */
#define DAEMON_TASK_PERIOD_MS (1U)

/** @brief 反馈超时阈值（组内 5 电机分时轮询间隔 ~15ms，留足余量） */
#define MOTOR_FEED_TIMEOUT_MS (200U)

/** @brief 上电宁静期：覆盖 ENABLE + ~1s 编码器校准 */
#define MOTOR_INIT_WAIT_MS (3000U)

/* Private types -------------------------------------------------------------*/

/**
 * @brief 单电机监控项
 */
typedef struct {
    daemon_context_t ctx; /**< daemon 静态句柄 */
    uint32_t last_fb_stamp; /**< 上次反馈时间戳快照 (time_fb + time_status) */
    uint8_t motor_index; /**< srv_motor_get_handle 索引 (0-8) */
} motor_monitor_t;

/* Private variables ---------------------------------------------------------*/

/** @brief 电机守护进程名称（daemon 浅拷贝 config，字符串必须持久） */
static const char* const s_names[SRV_MOTOR_TOTAL] = {
    "motorA1", "motorA2", "motorA3", "motorA4", "motorA5",
    "motorB1", "motorB2", "motorB3", "motorB4",
};

/** @brief 9 电机监控项 */
motor_monitor_t s_mon[SRV_MOTOR_TOTAL];

/** @brief 任务定时器 */
static sw_timer_t s_timer;

/* Private function prototypes ------------------------------------------------*/

static void motor_offline_cb(void* owner_ptr);
static void daemon_timer_cb(void* user_data);

/* Exported functions --------------------------------------------------------*/

void daemon_task_init(void)
{
    /* daemon 中间件首个使用者，负责初始化（ms 时基 = millis） */
    daemon_error_t err = daemon_init(millis);
    if (DAEMON_IS_ERR(err)) {
        LOG_E("daemon", "daemon_init failed: %d", (int)err);
        return;
    }

    /* 为 9 个电机逐一注册监控 */
    for (uint32_t i = 0; i < SRV_MOTOR_TOTAL; i++) {
        motor_monitor_t* mon = &s_mon[i];
        mon->motor_index = (uint8_t)i;
        mon->last_fb_stamp = 0;

        daemon_config_t cfg = {
            .name = s_names[i],
            .owner_ptr = mon,
            .offline_cb = motor_offline_cb,
            .reload_timeout_ms = MOTOR_FEED_TIMEOUT_MS,
            .init_wait_time_ms = MOTOR_INIT_WAIT_MS,
        };

        err = daemon_register_static(&cfg, &mon->ctx);
        if (DAEMON_IS_ERR(err)) {
            LOG_E("daemon", "register %s failed: %d", s_names[i], (int)err);
        }
    }

    sw_timer_init(&s_timer, &(sw_timer_config_t) {
        .priority = SW_TIMER_PRIO_NORMAL,
        .callback = daemon_timer_cb,
    });
    sw_timer_start(&s_timer, DAEMON_TASK_PERIOD_MS, 0);

    LOG_I("daemon", "daemon task init ok, %u motors monitored", (unsigned)daemon_get_count());
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 电机在线状态跳变回调（掉线与恢复共用，方向由 daemon_is_online 区分）
 */
static void motor_offline_cb(void* owner_ptr)
{
    const motor_monitor_t* mon = (const motor_monitor_t*)owner_ptr;
    if (!mon) {
        return;
    }

    if (daemon_is_online(&mon->ctx)) {
        LOG_I("daemon", "%s back online", daemon_get_name(&mon->ctx));
    } else {
        LOG_W("daemon", "%s OFFLINE (fb timeout > %ums)",
            daemon_get_name(&mon->ctx), (unsigned)MOTOR_FEED_TIMEOUT_MS);
    }
}

/**
 * @brief 定时器回调 — 反馈时间戳推进则喂狗，再派发超时检测
 */
static void daemon_timer_cb(void* user_data)
{
    (void)user_data;

    for (uint32_t i = 0; i < SRV_MOTOR_TOTAL; i++) {
        motor_monitor_t* mon = &s_mon[i];
        const srv_motor_handle_t* handle = srv_motor_get_handle(mon->motor_index);
        const srv_motor_feedback_t* fb = srv_motor_get_feedback(handle);
        if (!fb) {
            continue;
        }

        uint32_t stamp = fb->time_fb + fb->time_status;
        if (stamp != mon->last_fb_stamp) {
            daemon_reload(&mon->ctx);
            mon->last_fb_stamp = stamp;
        }
    }

    daemon_task();
}
