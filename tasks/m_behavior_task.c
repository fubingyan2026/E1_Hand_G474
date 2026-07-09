/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    m_behavior_task.c
 * @brief   电机行为控制 — 单 fsm_t 管理 9 电机整体行为
 *
 * s_fsm 状态 = 组整体状态（INIT→OFFLINE→IDLE→ENABLING→RUNNING⇄FAULT）。
 * 任一电机满足条件即触发状态迁移。
 * 使能/控制通过广播帧统一下发。
 */

#include "m_behavior_task.h"

#include "fsm.h"
#include "motor_task.h"
#include "srv_motor.h"
#include "sw_timer.h"

#include <string.h>

/* Private constants ---------------------------------------------------------*/

#define BHV_PERIOD_MS 10U
#define BHV_FSM_COUNT (M_BHV_STATE_FAULT + 1U)

/* Private variables ---------------------------------------------------------*/

static srv_motor_handle_t s_motors[MOTOR_TOTAL];
static fsm_t s_fsm;
static sw_timer_t s_timer;
static bool s_initialized;

static fsm_handler_t s_handlers[BHV_FSM_COUNT];
static fsm_guard_t s_transitions[BHV_FSM_COUNT * BHV_FSM_COUNT];

/* Private function prototypes -----------------------------------------------*/

static void m_behavior_timer_cb(void* user_data);

static fsm_state_t h_init(fsm_t* ctx);
static fsm_state_t h_offline(fsm_t* ctx);
static fsm_state_t h_idle(fsm_t* ctx);
static fsm_state_t h_enabling(fsm_t* ctx);
static fsm_state_t h_running(fsm_t* ctx);
static fsm_state_t h_fault(fsm_t* ctx);
static void on_entry(fsm_t* ctx, fsm_state_t state);

/* 辅助 */
static bool any_online(void);
static bool any_error(void);
static void disable_all(void);

/* Exported functions --------------------------------------------------------*/

void m_behavior_task_init(void)
{
    if (s_initialized)
        return;

    for (uint32_t i = 0; i < MOTOR_GROUP_A_NUM; i++)
        srv_motor_register(&s_motors[i], i + 1, 2);
    for (uint32_t i = 0; i < MOTOR_GROUP_B_NUM; i++)
        srv_motor_register(&s_motors[MOTOR_GROUP_A_NUM + i], i + 1, 3);

    memset(s_handlers, 0, sizeof(s_handlers));
    memset(s_transitions, 0, sizeof(s_transitions));

    s_handlers[M_BHV_STATE_INIT] = h_init;
    s_handlers[M_BHV_STATE_OFFLINE] = h_offline;
    s_handlers[M_BHV_STATE_IDLE] = h_idle;
    s_handlers[M_BHV_STATE_ENABLING] = h_enabling;
    s_handlers[M_BHV_STATE_RUNNING] = h_running;
    s_handlers[M_BHV_STATE_FAULT] = h_fault;

    fsm_fill(&(fsm_config_t) {
                 .handlers = s_handlers,
                 .transitions = s_transitions,
                 .state_count = BHV_FSM_COUNT,
                 .entry_cb = on_entry,
             },
        fsm_always_true);

    fsm_init(&s_fsm, M_BHV_STATE_INIT, &(fsm_config_t) {
                                           .handlers = s_handlers,
                                           .transitions = s_transitions,
                                           .state_count = BHV_FSM_COUNT,
                                           .entry_cb = on_entry,
                                       });

    s_initialized = true;

    sw_timer_init(&s_timer, &(sw_timer_config_t) { .priority = SW_TIMER_PRIO_NORMAL, .callback = m_behavior_timer_cb });
    sw_timer_start(&s_timer, BHV_PERIOD_MS, 0);
}

m_behavior_error_t m_behavior_enable(uint32_t index, bool en)
{
    if (index >= MOTOR_TOTAL || !s_initialized)
        return M_BHV_ERROR_INVALID_PARAM;

    if (en) {
        if (fsm_current_state(&s_fsm) != M_BHV_STATE_IDLE)
            return M_BHV_ERROR_INVALID_STATE;
        srv_motor_enable(&s_motors[index], true);

    } else {
        /* 用户去使能 → 清故障，回 IDLE */
        srv_motor_enable(&s_motors[index], false);
        if (fsm_current_state(&s_fsm) == M_BHV_STATE_FAULT)
            fsm_goto(&s_fsm, M_BHV_STATE_IDLE);
    }

    return M_BHV_OK;
}

m_behavior_error_t m_behavior_set_setpoint(uint32_t index,
    int16_t pos_ref, int16_t spd_ref, int16_t cur_ref)
{
    if (index >= MOTOR_TOTAL || !s_initialized)
        return M_BHV_ERROR_INVALID_PARAM;

    srv_motor_set_setpoint(&s_motors[index], pos_ref, spd_ref, cur_ref);
    /* 电流限制变化时标记待发送 */
    srv_motor_set_current(&s_motors[index], cur_ref);
    return M_BHV_OK;
}

m_behavior_error_t m_behavior_set_current(uint32_t index, int16_t cur_ref)
{
    if (index >= MOTOR_TOTAL || !s_initialized)
        return M_BHV_ERROR_INVALID_PARAM;
    srv_motor_set_current(&s_motors[index], cur_ref);
    return M_BHV_OK;
}

void m_behavior_enable_all(bool en)
{
    for (uint32_t i = 0; i < MOTOR_TOTAL; i++)
        m_behavior_enable(i, en);
}

void m_behavior_set_all_setpoint(int16_t pos_ref, int16_t spd_ref, int16_t cur_ref)
{
    for (uint32_t i = 0; i < MOTOR_TOTAL; i++)
        srv_motor_set_setpoint(&s_motors[i], pos_ref, spd_ref, cur_ref);
}

m_behavior_state_t m_behavior_get_state(uint32_t index)
{
    (void)index;
    return (m_behavior_state_t)fsm_current_state(&s_fsm);
}

bool m_behavior_is_running(uint32_t index)
{
    (void)index;
    return fsm_current_state(&s_fsm) == M_BHV_STATE_RUNNING;
}

const srv_motor_feedback_t* m_behavior_get_fb(uint32_t index)
{
    return index < MOTOR_TOTAL ? srv_motor_get_feedback(&s_motors[index]) : NULL;
}

/* Private functions ---------------------------------------------------------*/

static void m_behavior_timer_cb(void* user_data)
{
    (void)user_data;
    fsm_step(&s_fsm);
}

/* ====================================================================
 * FSM handlers — 任一电机满足条件即触发状态迁移
 * ==================================================================== */

static fsm_state_t h_init(fsm_t* ctx)
{
    (void)ctx;
    return M_BHV_STATE_OFFLINE;
}

/** OFFLINE → IDLE（任一在线且无故障）/ FAULT（任一故障） */
static fsm_state_t h_offline(fsm_t* ctx)
{
    (void)ctx;
    if (!any_online())
        return M_BHV_STATE_OFFLINE;
    return any_error() ? M_BHV_STATE_FAULT : M_BHV_STATE_IDLE;
}

/** IDLE — 等用户使能；检测离线/故障 */
static fsm_state_t h_idle(fsm_t* ctx)
{
    (void)ctx;
    if (!any_online())
        return M_BHV_STATE_OFFLINE;
    if (any_error())
        return M_BHV_STATE_FAULT;
    return M_BHV_STATE_IDLE; /* 等待外部 fsm_goto(ENABLING) */
}

/** ENABLING → RUNNING（任一到达 mcRun）/ FAULT / OFFLINE */
static fsm_state_t h_enabling(fsm_t* ctx)
{
    (void)ctx;
    if (!any_online())
        return M_BHV_STATE_OFFLINE;
    if (any_error())
        return M_BHV_STATE_FAULT;

    for (uint32_t i = 0; i < MOTOR_TOTAL; i++) {
        const srv_motor_feedback_t* fb = srv_motor_get_feedback(&s_motors[i]);
        if (fb && fb->fsm_state == SRV_MOTOR_FSM_RUN)
            return M_BHV_STATE_RUNNING;
    }
    return M_BHV_STATE_ENABLING;
}

/** RUNNING — 持续下发控制值；检测故障/离线 */
static fsm_state_t h_running(fsm_t* ctx)
{
    (void)ctx;
    if (!any_online())
        return M_BHV_STATE_OFFLINE;
    if (any_error())
        return M_BHV_STATE_FAULT;

    /* 持续写入 setpoint，由 motor_task 广播自动发送 */
    for (uint32_t i = 0; i < MOTOR_TOTAL; i++) {
        const srv_motor_feedback_t* fb = srv_motor_get_feedback(&s_motors[i]);
        if (fb && fb->fsm_state == SRV_MOTOR_FSM_RUN)
            srv_motor_set_setpoint(&s_motors[i],
                s_motors[i].pos_ref, s_motors[i].spd_ref, s_motors[i].cur_ref);
    }
    return M_BHV_STATE_RUNNING;
}

/** FAULT — 等待用户 m_behavior_enable_all(false) 清除 */
static fsm_state_t h_fault(fsm_t* ctx)
{
    (void)ctx;
    return M_BHV_STATE_FAULT;
}

/** 进入 ENABLING → 使能全部电机；进入 FAULT → 去使能全部 */
static void on_entry(fsm_t* ctx, fsm_state_t state)
{
    (void)ctx;

    if (state == M_BHV_STATE_ENABLING) {
        for (uint32_t i = 0; i < MOTOR_TOTAL; i++)
            srv_motor_enable(&s_motors[i], true);
    }

    if (state == M_BHV_STATE_FAULT) {
        disable_all();
    }
}

/* ---- 辅助 ---- */

static bool any_online(void)
{
    for (uint32_t i = 0; i < MOTOR_TOTAL; i++) {
        const srv_motor_feedback_t* fb = srv_motor_get_feedback(&s_motors[i]);
        if (fb && (fb->time_fb | fb->time_status))
            return true;
    }
    return false;
}

static bool any_error(void)
{
    for (uint32_t i = 0; i < MOTOR_TOTAL; i++) {
        const srv_motor_feedback_t* fb = srv_motor_get_feedback(&s_motors[i]);
        if (fb && fb->err_code != SRV_MOTOR_ERR_NONE)
            return true;
    }
    return false;
}

static void disable_all(void)
{
    for (uint32_t i = 0; i < MOTOR_TOTAL; i++)
        srv_motor_enable(&s_motors[i], false);
}
