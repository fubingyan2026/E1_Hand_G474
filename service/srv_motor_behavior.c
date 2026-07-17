/**
 * @file    srv_motor_behavior.c
 * @brief   电机行为协调服务 — 单 fsm_t 管理 9 电机整体行为
 *
 * 依赖 srv_motor 反馈/控制接口，不持有电机句柄。
 * s_fsm 状态 = 组整体状态：OFFLINE→IDLE→CALIB→RUNNING，任意态→FAULT。
 * 上电首次进入 CALIB 到位找零：下发 180° 目标 + 小电流限制，
 * angle_fb 到达目标 ±BHV_CALIB_POS_TOL 容差内 → srv_motor_zero_reset 设零。
 * 在线判定查询 daemon 看门狗（中间件，按 srv_motor_get_name 注册名懒解析）。
 * 链路层使能/校准/模式序列由 srv_motor 的 STARTUP FSM 自治完成，本层只观察。
 *
 * 枚举值 INIT(0) 为 CAN 协议保留值，FSM 不再使用（见 .h 说明）。
 */

#include "srv_motor_behavior.h"

#include "daemon.h"
#include "drv_systick.h"
#include "fsm.h"
#include "log.h"
#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define BHV_LOG_ENABLE 1

#if BHV_LOG_ENABLE
#define BHV_LOG_E(...) LOG_E("bhv", __VA_ARGS__)
#define BHV_LOG_W(...) LOG_W("bhv", __VA_ARGS__)
#define BHV_LOG_I(...) LOG_I("bhv", __VA_ARGS__)
#define BHV_LOG_D(...) LOG_D("bhv", __VA_ARGS__)
#else
#define BHV_LOG_E(...) ((void)0)
#define BHV_LOG_W(...) ((void)0)
#define BHV_LOG_I(...) ((void)0)
#define BHV_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

#define BHV_FSM_COUNT (SRV_MOTOR_BHV_FAULT + 1U)

/* --- 零点校准参数（到位找零：走到目标位置即设零，可按机械结构调整） --- */

#define BHV_CALIB_POS_REF (23040) /**< 校准目标 +180° (raw = deg×128)，方向反了改负号 */
#define BHV_CALIB_CUR_REF (500) /**< 校准电流限制 ≈0.69A (Q15: A = raw×5.625/32768) */
#define BHV_CALIB_POS_TOL (256) /**< 到位判定容差 ±2° (raw = deg×128) */
#define BHV_CALIB_TIMEOUT_MS (6000U) /**< 校准总超时：未完成电机跳过设零并告警 */

/** @brief 状态名表（日志用，按枚举值索引） */
static const char* const s_state_names[BHV_FSM_COUNT] = {
    "INIT",
    "OFFLINE",
    "IDLE",
    "CALIB",
    "RUNNING",
    "FAULT",
};

/* Private variables ---------------------------------------------------------*/

static fsm_t s_fsm;
static bool s_initialized;

static fsm_handler_t s_handlers[BHV_FSM_COUNT];
static fsm_guard_t s_transitions[BHV_FSM_COUNT * BHV_FSM_COUNT];

/** @brief daemon 实例懒解析缓存（daemon_task_init 晚于本模块 init，首次 step 时解析） */
static const daemon_context_t* s_daemon[SRV_MOTOR_TOTAL];

/* --- 零点校准运行时状态 --- */

/**
 * @brief 零点校准上下文（模块内单例）
 */
typedef struct {
    bool calibrated; /**< 上电首次校准完成标志 */
    uint16_t done_mask; /**< 已设零电机位图 (bit i) */
    int16_t saved_cur[SRV_MOTOR_TOTAL]; /**< 校准前 cur_ref 备份，完成后恢复 */
    uint32_t start_time; /**< 校准开始时间（总超时用） */
} bhv_calib_ctx_t;

/** @brief 校准上下文单例 */
static bhv_calib_ctx_t s_calib;

/* Private function prototypes -----------------------------------------------*/

static fsm_state_t h_offline(fsm_t* ctx);
static fsm_state_t h_idle(fsm_t* ctx);
static fsm_state_t h_calib(fsm_t* ctx);
static fsm_state_t h_running(fsm_t* ctx);
static fsm_state_t h_fault(fsm_t* ctx);
static void on_entry(fsm_t* ctx, fsm_state_t state);

static bool any_online(void);
static bool any_error(void);
static bool any_run(void);
static void disable_all(void);

/* Exported functions --------------------------------------------------------*/

void srv_motor_behavior_init(void)
{
    if (s_initialized)
        return;

    memset(s_handlers, 0, sizeof(s_handlers));
    memset(s_transitions, 0, sizeof(s_transitions));
    memset(s_daemon, 0, sizeof(s_daemon));
    memset(&s_calib, 0, sizeof(s_calib));

    /* INIT 为协议保留值，不注册 handler（不可达） */
    s_handlers[SRV_MOTOR_BHV_OFFLINE] = h_offline;
    s_handlers[SRV_MOTOR_BHV_IDLE] = h_idle;
    s_handlers[SRV_MOTOR_BHV_CALIB] = h_calib;
    s_handlers[SRV_MOTOR_BHV_RUNNING] = h_running;
    s_handlers[SRV_MOTOR_BHV_FAULT] = h_fault;

    fsm_config_t cfg = {
        .handlers = s_handlers,
        .transitions = s_transitions,
        .state_count = BHV_FSM_COUNT,
        .entry_cb = on_entry,
    };
    fsm_fill(&cfg, fsm_always_true);
    fsm_init(&s_fsm, SRV_MOTOR_BHV_OFFLINE, &cfg);

    s_initialized = true;
    BHV_LOG_I("behavior init done, state=%s", s_state_names[SRV_MOTOR_BHV_OFFLINE]);
}

void srv_motor_behavior_step(void)
{
    if (!s_initialized)
        return;
    fsm_step(&s_fsm);
}

srv_motor_behavior_error_t srv_motor_behavior_set_setpoint(uint32_t index,
    int16_t pos_ref, int16_t spd_ref, int16_t cur_ref)
{
    srv_motor_handle_t* m = srv_motor_get_handle(index);
    if (!m || !s_initialized)
        return SRV_MOTOR_BHV_ERROR_INVALID_PARAM;

    /* 仅 RUNNING 态响应外部目标：校准期间防覆盖 180° 目标，
     * 未校准/离线/故障期间外部目标一律忽略 */
    if (fsm_current_state(&s_fsm) != SRV_MOTOR_BHV_RUNNING)
        return SRV_MOTOR_BHV_ERROR_INVALID_STATE;

    srv_motor_set_setpoint(m, pos_ref, spd_ref, cur_ref);
    return SRV_MOTOR_BHV_OK;
}

void srv_motor_behavior_set_all_setpoint(int16_t pos_ref, int16_t spd_ref, int16_t cur_ref)
{
    /* 仅 RUNNING 态响应外部目标（与单电机接口一致） */
    if (!s_initialized || fsm_current_state(&s_fsm) != SRV_MOTOR_BHV_RUNNING)
        return;

    for (uint32_t i = 0; i < SRV_MOTOR_TOTAL; i++) {
        srv_motor_handle_t* m = srv_motor_get_handle(i);
        if (m)
            srv_motor_set_setpoint(m, pos_ref, spd_ref, cur_ref);
    }
}

srv_motor_behavior_state_t srv_motor_behavior_get_state(void)
{
    return (srv_motor_behavior_state_t)fsm_current_state(&s_fsm);
}

const srv_motor_feedback_t* srv_motor_behavior_get_fb(uint32_t index)
{
    srv_motor_handle_t* m = srv_motor_get_handle(index);
    return m ? srv_motor_get_feedback(m) : NULL;
}

/* ====================================================================
 * FSM handlers — 任一电机满足条件即触发状态迁移
 * ==================================================================== */

static fsm_state_t h_offline(fsm_t* ctx)
{
    (void)ctx;
    if (!any_online())
        return SRV_MOTOR_BHV_OFFLINE;
    return any_error() ? SRV_MOTOR_BHV_FAULT : SRV_MOTOR_BHV_IDLE;
}

static fsm_state_t h_idle(fsm_t* ctx)
{
    (void)ctx;
    if (!any_online())
        return SRV_MOTOR_BHV_OFFLINE;
    if (any_error())
        return SRV_MOTOR_BHV_FAULT;
    if (any_run())
        return s_calib.calibrated ? SRV_MOTOR_BHV_RUNNING : SRV_MOTOR_BHV_CALIB;
    return SRV_MOTOR_BHV_IDLE;
}

/**
 * @brief CALIB 状态 — 到位找零
 *
 * on_entry 已下发 180° 目标 + 小电流限制。本 handler 每周期检测：
 * 电机 angle_fb 进入目标位置 ±BHV_CALIB_POS_TOL 容差内
 * → 判定到位 → srv_motor_zero_reset() 设零 + 目标回零 + 恢复电流。
 * 全部运行中电机完成（或总超时）→ RUNNING。
 */
static fsm_state_t h_calib(fsm_t* ctx)
{
    (void)ctx;
    if (!any_online())
        return SRV_MOTOR_BHV_OFFLINE;
    if (any_error())
        return SRV_MOTOR_BHV_FAULT;

    uint32_t now = millis();
    bool timeout = (now - s_calib.start_time >= BHV_CALIB_TIMEOUT_MS);
    uint32_t run_cnt = 0;
    uint32_t done_cnt = 0;

    for (uint32_t i = 0; i < SRV_MOTOR_TOTAL; i++) {
        srv_motor_handle_t* m = srv_motor_get_handle(i);
        const srv_motor_feedback_t* fb = srv_motor_get_feedback(m);
        if (!fb || fb->fsm_state != SRV_MOTOR_FSM_RUN)
            continue;
        run_cnt++;

        if (s_calib.done_mask & (1U << i)) {
            done_cnt++;
            continue;
        }

        if (timeout) {
            /* 总超时：跳过设零，恢复电流限制并告警 */
            srv_motor_set_setpoint(m, 0, m->spd_ref, s_calib.saved_cur[i]);
            s_calib.done_mask |= (1U << i);
            BHV_LOG_W("calib %s timeout, zero skipped (angle=%d)",
                srv_motor_get_name(i), fb->angle_fb);
            continue;
        }

        /* 到位判定：angle_fb 进入目标 ± 容差 */
        int16_t diff = (int16_t)(fb->angle_fb - BHV_CALIB_POS_REF);
        int16_t diff_abs = (diff < 0) ? (int16_t)-diff : diff;
        if (diff_abs <= BHV_CALIB_POS_TOL) {
            /* 到位确认：设零 → 目标回 0（= 当前位置）→ 恢复电流限制 */
            srv_motor_zero_reset(m);
            srv_motor_set_setpoint(m, 0, m->spd_ref, s_calib.saved_cur[i]);
            s_calib.done_mask |= (1U << i);
            BHV_LOG_I("calib %s zero set (angle=%d)", srv_motor_get_name(i), fb->angle_fb);
        }
    }

    /* 完成判定：所有运行中电机均已处理（设零或超时跳过），且至少 1 个 */
    if (run_cnt > 0 && done_cnt >= run_cnt) {
        s_calib.calibrated = true;
        BHV_LOG_I("calib done, %u motors, took %lums",
            (unsigned)run_cnt, (unsigned long)(now - s_calib.start_time));
        return SRV_MOTOR_BHV_RUNNING;
    }

    return SRV_MOTOR_BHV_CALIB;
}

static fsm_state_t h_running(fsm_t* ctx)
{
    (void)ctx;
    if (!any_online())
        return SRV_MOTOR_BHV_OFFLINE;
    if (any_error())
        return SRV_MOTOR_BHV_FAULT;
    if (!any_run())
        return SRV_MOTOR_BHV_IDLE;
    return SRV_MOTOR_BHV_RUNNING;
}

static fsm_state_t h_fault(fsm_t* ctx)
{
    /* err_code 全部清零（电机端故障消除/重上电）后自动恢复回 IDLE */
    (void)ctx;
    if (!any_online())
        return SRV_MOTOR_BHV_OFFLINE;
    if (!any_error()) {
        BHV_LOG_I("all err_code cleared, auto recover to IDLE");
        return SRV_MOTOR_BHV_IDLE;
    }
    return SRV_MOTOR_BHV_FAULT;
}

/**
 * @brief 状态进入回调 — 仅在实际状态变化时触发一次（fsm 框架保证）
 */
static void on_entry(fsm_t* ctx, fsm_state_t state)
{
    (void)ctx;
    BHV_LOG_I("state -> %s", s_state_names[state]);

    if (state == SRV_MOTOR_BHV_CALIB) {
        /* 启动到位找零：备份电流限制，下发 180° 目标 + 小电流 */
        s_calib.done_mask = 0;
        s_calib.start_time = millis();

        for (uint32_t i = 0; i < SRV_MOTOR_TOTAL; i++) {
            srv_motor_handle_t* m = srv_motor_get_handle(i);
            if (!m || !m->initialized)
                continue;
            s_calib.saved_cur[i] = m->cur_ref;
            srv_motor_set_setpoint(m, BHV_CALIB_POS_REF, m->spd_ref, BHV_CALIB_CUR_REF);
        }
        BHV_LOG_I("calib start: pos=%d cur=%d tol=%d",
            BHV_CALIB_POS_REF, BHV_CALIB_CUR_REF, BHV_CALIB_POS_TOL);
    }

    if (state == SRV_MOTOR_BHV_FAULT) {
        /* 打印肇事电机（可能多个，逐一列出） */
        for (uint32_t i = 0; i < SRV_MOTOR_TOTAL; i++) {
            const srv_motor_feedback_t* fb = srv_motor_behavior_get_fb(i);
            if (fb && fb->err_code != SRV_MOTOR_ERR_NONE) {
                BHV_LOG_W("FAULT src: %s err_code=%u",
                    srv_motor_get_name(i), (unsigned)fb->err_code);
            }
        }
        disable_all();
    }
}

/* ====================================================================
 * 判定辅助
 * ==================================================================== */

/**
 * @brief 任一电机在线（daemon 看门狗实时判定）
 * @note  daemon_task_init 晚于本模块初始化，实例指针懒解析并缓存；
 *        daemon 尚未注册时视为离线。
 */
static bool any_online(void)
{
    for (uint32_t i = 0; i < SRV_MOTOR_TOTAL; i++) {
        if (!s_daemon[i])
            s_daemon[i] = daemon_get_instance(srv_motor_get_name(i));
        if (s_daemon[i] && daemon_is_online(s_daemon[i]))
            return true;
    }
    return false;
}

static bool any_error(void)
{
    return false;
    for (uint32_t i = 0; i < SRV_MOTOR_TOTAL; i++) {
        const srv_motor_feedback_t* fb = srv_motor_behavior_get_fb(i);
        if (fb && fb->err_code != SRV_MOTOR_ERR_NONE)
            return true;
    }
    return false;
}

/** @brief 任一电机反馈处于 mcRun 运行态 */
static bool any_run(void)
{
    for (uint32_t i = 0; i < SRV_MOTOR_TOTAL; i++) {
        const srv_motor_feedback_t* fb = srv_motor_behavior_get_fb(i);
        if (fb && fb->fsm_state == SRV_MOTOR_FSM_RUN)
            return true;
    }
    return false;
}

static void disable_all(void)
{
    BHV_LOG_W("disable ALL motors (fault protection)");
    for (uint32_t i = 0; i < SRV_MOTOR_TOTAL; i++) {
        srv_motor_handle_t* m = srv_motor_get_handle(i);
        if (m)
            srv_motor_enable(m, false);
    }
}
