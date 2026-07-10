/**
 * @file    srv_motor_behavior.c
 * @brief   电机行为协调服务 — 单 fsm_t 管理 9 电机整体行为
 *
 * 依赖 srv_motor 反馈/控制接口，不持有电机句柄。
 * s_fsm 状态 = 组整体状态，任一电机满足条件即触发迁移。
 */

#include "srv_motor_behavior.h"

#include "fsm.h"
#include "motor_task.h"

#include <string.h>

/* Private constants ---------------------------------------------------------*/

#define BHV_FSM_COUNT (SRV_MOTOR_BHV_FAULT + 1U)

/* Private variables ---------------------------------------------------------*/

static fsm_t s_fsm;
static bool s_initialized;

static fsm_handler_t s_handlers[BHV_FSM_COUNT];
static fsm_guard_t   s_transitions[BHV_FSM_COUNT * BHV_FSM_COUNT];

/* Private function prototypes -----------------------------------------------*/

static fsm_state_t h_init(fsm_t* ctx);
static fsm_state_t h_offline(fsm_t* ctx);
static fsm_state_t h_idle(fsm_t* ctx);
static fsm_state_t h_enabling(fsm_t* ctx);
static fsm_state_t h_running(fsm_t* ctx);
static fsm_state_t h_fault(fsm_t* ctx);
static void on_entry(fsm_t* ctx, fsm_state_t state);

static bool any_online(void);
static bool any_error(void);
static void disable_all(void);

/* Exported functions --------------------------------------------------------*/

void srv_motor_behavior_init(void)
{
    if (s_initialized) return;

    memset(s_handlers, 0, sizeof(s_handlers));
    memset(s_transitions, 0, sizeof(s_transitions));

    s_handlers[SRV_MOTOR_BHV_INIT]     = h_init;
    s_handlers[SRV_MOTOR_BHV_OFFLINE]  = h_offline;
    s_handlers[SRV_MOTOR_BHV_IDLE]     = h_idle;
    s_handlers[SRV_MOTOR_BHV_ENABLING] = h_enabling;
    s_handlers[SRV_MOTOR_BHV_RUNNING]  = h_running;
    s_handlers[SRV_MOTOR_BHV_FAULT]    = h_fault;

    fsm_fill(&(fsm_config_t){
        .handlers = s_handlers, .transitions = s_transitions,
        .state_count = BHV_FSM_COUNT, .entry_cb = on_entry,
    }, fsm_always_true);

    fsm_init(&s_fsm, SRV_MOTOR_BHV_INIT, &(fsm_config_t){
        .handlers = s_handlers, .transitions = s_transitions,
        .state_count = BHV_FSM_COUNT, .entry_cb = on_entry,
    });

    s_initialized = true;
}

void srv_motor_behavior_step(void)
{
    if (!s_initialized) return;
    fsm_step(&s_fsm);
}

srv_motor_behavior_error_t srv_motor_behavior_enable(uint32_t index, bool en)
{
    srv_motor_handle_t* m = srv_motor_get_handle(index);
    if (!m || !s_initialized)
        return SRV_MOTOR_BHV_ERROR_INVALID_PARAM;

    if (en) {
        if (fsm_current_state(&s_fsm) != SRV_MOTOR_BHV_IDLE)
            return SRV_MOTOR_BHV_ERROR_INVALID_STATE;
        srv_motor_enable(m, true);
    } else {
        srv_motor_enable(m, false);
        if (fsm_current_state(&s_fsm) == SRV_MOTOR_BHV_FAULT)
            fsm_goto(&s_fsm, SRV_MOTOR_BHV_IDLE);
    }
    return SRV_MOTOR_BHV_OK;
}

srv_motor_behavior_error_t srv_motor_behavior_set_setpoint(uint32_t index,
    int16_t pos_ref, int16_t spd_ref, int16_t cur_ref)
{
    srv_motor_handle_t* m = srv_motor_get_handle(index);
    if (!m || !s_initialized)
        return SRV_MOTOR_BHV_ERROR_INVALID_PARAM;

    srv_motor_set_setpoint(m, pos_ref, spd_ref, cur_ref);
    srv_motor_set_current(m, cur_ref);
    return SRV_MOTOR_BHV_OK;
}

void srv_motor_behavior_enable_all(bool en)
{
    for (uint32_t i = 0; i < MOTOR_TOTAL; i++)
        srv_motor_behavior_enable(i, en);
}

void srv_motor_behavior_set_all_setpoint(int16_t pos_ref, int16_t spd_ref, int16_t cur_ref)
{
    for (uint32_t i = 0; i < MOTOR_TOTAL; i++) {
        srv_motor_handle_t* m = srv_motor_get_handle(i);
        if (m) srv_motor_set_setpoint(m, pos_ref, spd_ref, cur_ref);
    }
}

srv_motor_behavior_state_t srv_motor_behavior_get_state(void)
{
    return (srv_motor_behavior_state_t)fsm_current_state(&s_fsm);
}

bool srv_motor_behavior_is_running(void)
{
    return fsm_current_state(&s_fsm) == SRV_MOTOR_BHV_RUNNING;
}

const srv_motor_feedback_t* srv_motor_behavior_get_fb(uint32_t index)
{
    srv_motor_handle_t* m = srv_motor_get_handle(index);
    return m ? srv_motor_get_feedback(m) : NULL;
}

/* ====================================================================
 * FSM handlers — 任一电机满足条件即触发状态迁移
 * ==================================================================== */

static fsm_state_t h_init(fsm_t* ctx)
{
    (void)ctx;
    return SRV_MOTOR_BHV_OFFLINE;
}

static fsm_state_t h_offline(fsm_t* ctx)
{
    (void)ctx;
    if (!any_online()) return SRV_MOTOR_BHV_OFFLINE;
    return any_error() ? SRV_MOTOR_BHV_FAULT : SRV_MOTOR_BHV_IDLE;
}

static fsm_state_t h_idle(fsm_t* ctx)
{
    (void)ctx;
    if (!any_online()) return SRV_MOTOR_BHV_OFFLINE;
    if (any_error())    return SRV_MOTOR_BHV_FAULT;
    return SRV_MOTOR_BHV_IDLE;
}

static fsm_state_t h_enabling(fsm_t* ctx)
{
    (void)ctx;
    if (!any_online()) return SRV_MOTOR_BHV_OFFLINE;
    if (any_error())    return SRV_MOTOR_BHV_FAULT;

    for (uint32_t i = 0; i < MOTOR_TOTAL; i++) {
        const srv_motor_feedback_t* fb = srv_motor_behavior_get_fb(i);
        if (fb && fb->fsm_state == SRV_MOTOR_FSM_RUN)
            return SRV_MOTOR_BHV_RUNNING;
    }
    return SRV_MOTOR_BHV_ENABLING;
}

static fsm_state_t h_running(fsm_t* ctx)
{
    (void)ctx;
    if (!any_online()) return SRV_MOTOR_BHV_OFFLINE;
    if (any_error())    return SRV_MOTOR_BHV_FAULT;

    for (uint32_t i = 0; i < MOTOR_TOTAL; i++) {
        srv_motor_handle_t* m = srv_motor_get_handle(i);
        const srv_motor_feedback_t* fb = m ? srv_motor_get_feedback(m) : NULL;
        if (fb && fb->fsm_state == SRV_MOTOR_FSM_RUN && m)
            srv_motor_set_setpoint(m, m->pos_ref, m->spd_ref, m->cur_ref);
    }
    return SRV_MOTOR_BHV_RUNNING;
}

static fsm_state_t h_fault(fsm_t* ctx)
{
    (void)ctx;
    return SRV_MOTOR_BHV_FAULT;
}

static void on_entry(fsm_t* ctx, fsm_state_t state)
{
    (void)ctx;
    if (state == SRV_MOTOR_BHV_ENABLING) {
        for (uint32_t i = 0; i < MOTOR_TOTAL; i++) {
            srv_motor_handle_t* m = srv_motor_get_handle(i);
            if (m) srv_motor_enable(m, true);
        }
    }
    if (state == SRV_MOTOR_BHV_FAULT)
        disable_all();
}

static bool any_online(void)
{
    for (uint32_t i = 0; i < MOTOR_TOTAL; i++) {
        const srv_motor_feedback_t* fb = srv_motor_behavior_get_fb(i);
        if (fb && (fb->time_fb | fb->time_status)) return true;
    }
    return false;
}

static bool any_error(void)
{
    for (uint32_t i = 0; i < MOTOR_TOTAL; i++) {
        const srv_motor_feedback_t* fb = srv_motor_behavior_get_fb(i);
        if (fb && fb->err_code != SRV_MOTOR_ERR_NONE) return true;
    }
    return false;
}

static void disable_all(void)
{
    for (uint32_t i = 0; i < MOTOR_TOTAL; i++) {
        srv_motor_handle_t* m = srv_motor_get_handle(i);
        if (m) srv_motor_enable(m, false);
    }
}
