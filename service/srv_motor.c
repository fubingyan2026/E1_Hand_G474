/**
 * @file    srv_motor.c
 * @author  maximillian
 * @version V1.2.0
 * @date    2026-07-09
 * @brief   关节电机串口通信服务 — 基于 fsm_t 的分时发送 + 广播控制 + 分时轮询
 *
 * 由 motor_task_poll() 在主循环全速调用 srv_motor_step()。
 * TX 节拍由 drv_uart_is_tx_busy() 控制，广播周期由 millis() 控制。
 * 每路 UART 一个 fsm_t 实例，状态迁移：
 *   STARTUP ─► BCAST_EN ─► BCAST_POS ─► POLL ─► BCAST_CUR ─► IDLE
 *   (步骤0-2:              (模式/使能脏则发,  (位置广播    (查询反馈   (电流+限速
 *    ENABLE→等1s           无脏跳过)         AB55/AC55)   1电机/周期) 1电机/周期)
 *    →SET_MODE→IDLE)
 *
 *  广播 300 Hz，反馈 ~133/167 Hz/motor。
 *  每个 handler 在 DMA 空闲时执行发送 → fsm_step 自动迁移到下一状态。
 */

#include "srv_motor.h"

#include "crc.h"
#include "drv_systick.h"
#include "drv_uart.h"
#include "fsm.h"
#include "log.h"
#include "msg_fifo.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define MOTOR_LOG_ENABLE 0

#if MOTOR_LOG_ENABLE
#define MOTOR_LOG_E(...) LOG_E("motor", __VA_ARGS__)
#define MOTOR_LOG_W(...) LOG_W("motor", __VA_ARGS__)
#define MOTOR_LOG_I(...) LOG_I("motor", __VA_ARGS__)
#define MOTOR_LOG_D(...) LOG_D("motor", __VA_ARGS__)
#else
#define MOTOR_LOG_E(...) ((void)0)
#define MOTOR_LOG_W(...) ((void)0)
#define MOTOR_LOG_I(...) ((void)0)
#define MOTOR_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

/** @brief 每组最大电机数（广播帧 8 槽位） */
#define MOTOR_PER_GROUP 5U

/**
 * @brief 各组串口通道配置（可通过编译宏覆盖）
 * 例: cmake -DSRV_MOTOR_GRPA_UART=DRV_UART_CH_2 -DSRV_MOTOR_GRPB_UART=DRV_UART_CH_1 ...
 */
#define SRV_MOTOR_GRPA_UART DRV_UART_CH_1
#define SRV_MOTOR_GRPB_UART DRV_UART_CH_2

/** @brief 每 N 次 INFO_02_R 后插入一次 INFO_01_R ，必须大于1 ！*/
const uint16_t POLL_01_INTERVAL = 4U;

const uint16_t MOTOR_ONE_FRAME_TIME_US = 400U;

/**
 * @brief 帧间间隔 (µs)，可通过编译宏覆盖
 * 默认 0 = DMA 完成后立即发下帧
 * 例: -DMOTOR_POST_DLY_US=500 → 每帧后等 500µs
 */
const uint16_t MOTOR_POST_DLY_US = 400U;

const float MAX_FREQUENTLY = 1000.0f / ((MOTOR_ONE_FRAME_TIME_US + MOTOR_POST_DLY_US) * 0.001f * 4);

/** @brief 广播控制周期 (ms)，可通过编译宏覆盖
 *  默认 3ms → ~333 Hz。例: -DMOTOR_BCAST_PERIOD_MS=5 → 200 Hz */
const uint16_t MOTOR_BCAST_PERIOD_MS = (1000 / MAX_FREQUENTLY + 1) * 1;

/**
 * @brief 分时发送 FSM 状态
 *
 * 每路 UART 独立运行该状态机，handler 在 DMA 空闲时发送一帧并迁移。
 * STARTUP → BCAST_EN → BCAST_POS → POLL → BCAST_CUR → IDLE
 * 位置广播每周期一次，轮询和电流各服务一个已注册电机。
 */
typedef enum {
    MOTOR_ST_STARTUP = 0, /**< 步骤0-2: ENABLE→等校准→SET_MODE */
    MOTOR_ST_BCAST_EN, /**< 模式/使能广播（条件进入） */
    MOTOR_ST_BCAST_POS, /**< 发送位置广播帧 AB55/AC55 交替 */
    MOTOR_ST_POLL, /**< 反馈查询（1 个已注册电机/周期） */
    MOTOR_ST_BCAST_CUR, /**< 配置 MAX_CUR（1 电机/周期） */
    MOTOR_ST_BCAST_LIM, /**< 配置 SPD_LIMIT（同电机，紧跟 CUR 后） */
    MOTOR_ST_IDLE, /**< 等待广播周期启动 */
    MOTOR_STATE_COUNT /**< 状态总数 */
} motor_step_state_t;

/* 电机组结构体 --------------------------------------------------------------*/

/**
 * @brief UART 电机组（每路串口一个实例）
 *
 * 绑定一个 fsm_t 实例管理分时发送状态，motors[] 按 dev_id-1 索引。
 */
typedef struct {
    fsm_t fsm; /**< 分时发送状态机 */
    srv_motor_handle_t* motors[MOTOR_PER_GROUP]; /**< 电机指针数组 */
    uint8_t count; /**< 已注册电机数 */
    drv_uart_channel_t uart_ch; /**< UART 通道 (DRV_UART_CH_1/CH_2) */
    uint8_t poll_cursor; /**< 轮询游标 */
    uint32_t poll_02_cnt; /**< INFO_02_R 计数器 */
    uint32_t cycle_start; /**< 当前广播周期起始时间 (millis) */
    uint8_t cur_cursor; /**< 电流限制轮询游标 */
    bool en_dirty; /**< 使能状态脏标志 */
    bool mode_dirty; /**< 控制模式脏标志（需发送 SET_MODE） */
    uint8_t startup_step; /**< 启动步骤: 0=ENABLE,1=校准等待,2=SET_MODE */
    uint8_t startup_motor; /**< 当前已发到第几个电机 (0..count-1) */
    uint32_t enable_time; /**< ENABLE 发送时间 (millis)，用于校准计时 */
    uint32_t frame_tick; /**< 上一帧发送时间 (micros)，帧间隔控制 */
    uint8_t en_states[16]; /**< 各 ID 使能命令缓存 */
    msg_fifo_t rx_queue; /**< 接收帧队列（ISR → 主循环） */
    uint8_t rx_qbuf[SRV_MOTOR_FRAME_SIZE * 32]; /**< 接收队列缓冲（kfifo 向下取整为 512B ≈ 25 帧，
                                                      需容纳单个 256B DMA 缓冲的 12 帧突发） */
} srv_motor_group_t;

/* 模块全局变量 --------------------------------------------------------------*/

/** @brief 两组电机实例 */
static srv_motor_group_t s_grp_a;  /**< 组A — USART1 (DRV_UART_CH_1) */
static srv_motor_group_t s_grp_b;  /**< 组B — USART2 (DRV_UART_CH_2) */

/** @brief 根据 uart_ch 获取组指针（自动匹配各实例的 uart_ch 配置） */
static srv_motor_group_t* s_grp_from_ch(drv_uart_channel_t ch)
{
    if (s_grp_a.uart_ch == ch) return &s_grp_a;
    if (s_grp_b.uart_ch == ch) return &s_grp_b;
    return NULL;
}

/** @brief 由组指针反查编号（0=A, 1=B，用于日志） */
static uint32_t s_grp_idx(const srv_motor_group_t* g)
{
    if (g == &s_grp_b) return 1U;
    return 0U;
}


/** @brief 电机句柄静态存储 + 扁平索引表 */
static srv_motor_handle_t s_handle[SRV_MOTOR_TOTAL];

/** @brief 初始化配置表（每组电机绑定到对应组的串口通道） */
static const struct { uint8_t dev_id; drv_uart_channel_t uart; } s_cfg[SRV_MOTOR_TOTAL] = {
    {1,SRV_MOTOR_GRPA_UART},{2,SRV_MOTOR_GRPA_UART},{3,SRV_MOTOR_GRPA_UART},{4,SRV_MOTOR_GRPA_UART},{5,SRV_MOTOR_GRPA_UART},
    {1,SRV_MOTOR_GRPB_UART}, {2,SRV_MOTOR_GRPB_UART}, {3,SRV_MOTOR_GRPB_UART},//
     {4,SRV_MOTOR_GRPB_UART},
};

/** @brief 模块初始化标志 */
static bool s_initialized;

/* FSM 静态资源（两组共用同一套 handler/transition 表） ----------------------*/

static fsm_handler_t s_handlers[MOTOR_STATE_COUNT];
static fsm_guard_t s_transitions[MOTOR_STATE_COUNT * MOTOR_STATE_COUNT];

/* 函数原型 ------------------------------------------------------------------*/

/* FSM handler（按执行时序排列） */
static fsm_state_t fsm_startup(fsm_t* ctx);
static fsm_state_t fsm_bcast_en(fsm_t* ctx);
static fsm_state_t fsm_bcast_pos(fsm_t* ctx);
static fsm_state_t fsm_poll(fsm_t* ctx);
static fsm_state_t fsm_bcast_cur(fsm_t* ctx);
static fsm_state_t fsm_bcast_lim(fsm_t* ctx);
static fsm_state_t fsm_idle(fsm_t* ctx);

/* 帧构建 */
static void build_std_frame(uint8_t buf[SRV_MOTOR_FRAME_SIZE],
    uint32_t can_id, const uint8_t data[8]);
static void build_bcast_frame(uint8_t buf[SRV_MOTOR_FRAME_SIZE],
    uint16_t head2, const int16_t values[8]);
static void build_enable_frame(uint8_t buf[SRV_MOTOR_FRAME_SIZE],
    const uint8_t en_states[16]);
static void build_poll_frame(uint8_t buf[SRV_MOTOR_FRAME_SIZE],
    uint8_t dev_id, uint8_t cfg_index);

/* 辅助 */
static void motor_rx_cb(drv_uart_channel_t ch, const uint8_t* data, uint32_t len);
static void motor_poll_one(srv_motor_group_t* g, drv_uart_channel_t ch);
static void motor_drain_rx(srv_motor_group_t* g, drv_uart_channel_t ch);
static void motor_parse_reply(const uint8_t frame[SRV_MOTOR_FRAME_SIZE],
    srv_motor_group_t* g);
static srv_motor_group_t* motor_grp_from_fsm(fsm_t* ctx);

/* Exported functions --------------------------------------------------------*/

/* --- 生命周期 --- */

srv_motor_error_t srv_motor_init(void)
{
    if (s_initialized)
        srv_motor_deinit();

    /* 构建 FSM 资源配置（两组共用） */
    memset(s_handlers, 0, sizeof(s_handlers));
    memset(s_transitions, 0, sizeof(s_transitions));

    s_handlers[MOTOR_ST_STARTUP]   = fsm_startup;
    s_handlers[MOTOR_ST_BCAST_EN]  = fsm_bcast_en;
    s_handlers[MOTOR_ST_BCAST_POS] = fsm_bcast_pos;
    s_handlers[MOTOR_ST_POLL]      = fsm_poll;
    s_handlers[MOTOR_ST_BCAST_CUR] = fsm_bcast_cur;
    s_handlers[MOTOR_ST_BCAST_LIM] = fsm_bcast_lim;
    s_handlers[MOTOR_ST_IDLE]      = fsm_idle;

    fsm_config_t fsm_cfg = {
        .handlers = s_handlers,
        .transitions = s_transitions,
        .state_count = MOTOR_STATE_COUNT,
    };
    fsm_fill(&fsm_cfg, fsm_always_true);

    /* 初始化两组电机 */
    memset(&s_grp_a, 0, sizeof(s_grp_a));
    s_grp_a.uart_ch = SRV_MOTOR_GRPA_UART;
    fsm_init(&s_grp_a.fsm, MOTOR_ST_STARTUP, &fsm_cfg);
    msg_fifo_init(&s_grp_a.rx_queue, s_grp_a.rx_qbuf, sizeof(s_grp_a.rx_qbuf), SRV_MOTOR_FRAME_SIZE);
    MOTOR_LOG_I("grpA init ok, ch=%u", (unsigned)s_grp_a.uart_ch);

    memset(&s_grp_b, 0, sizeof(s_grp_b));
    s_grp_b.uart_ch = SRV_MOTOR_GRPB_UART;
    fsm_init(&s_grp_b.fsm, MOTOR_ST_STARTUP, &fsm_cfg);
    msg_fifo_init(&s_grp_b.rx_queue, s_grp_b.rx_qbuf, sizeof(s_grp_b.rx_qbuf), SRV_MOTOR_FRAME_SIZE);
    MOTOR_LOG_I("grpB init ok, ch=%u", (unsigned)s_grp_b.uart_ch);

    s_initialized = true;

    /* 注册 UART 接收回调 — ISR 中入队，主循环解析 */
    drv_uart_register_rx_callback(s_grp_a.uart_ch, motor_rx_cb);
    drv_uart_register_rx_callback(s_grp_b.uart_ch, motor_rx_cb);

    /* 根据配置表注册全部电机 */
    for (uint32_t i = 0; i < SRV_MOTOR_TOTAL; i++)
        srv_motor_register(&s_handle[i], s_cfg[i].dev_id, s_cfg[i].uart);

    /* 初始参考值：pos 广播目标，spd/cur 用作 SPD_LIMIT / MAX_CUR 配置值 */
    for (uint32_t i = 0; i < MOTOR_PER_GROUP; i++) {
        if (s_grp_a.motors[i]) {
            s_grp_a.motors[i]->pos_ref = 0;          /* 停原点 */
            s_grp_a.motors[i]->spd_ref = 6000;       /* SPD_LIMIT ~15% */
            s_grp_a.motors[i]->cur_ref = 1000;       /* MAX_CUR  ~1.37A */
        }
        if (s_grp_b.motors[i]) {
            s_grp_b.motors[i]->pos_ref = 0;
            s_grp_b.motors[i]->spd_ref = 6000;
            s_grp_b.motors[i]->cur_ref = 1000;
        }
    }

    MOTOR_LOG_I("srv_motor init done, %u motors total", SRV_MOTOR_TOTAL);
    return SRV_MOTOR_OK;
}

void srv_motor_deinit(void)
{
    if (!s_initialized)
        return;
    for (uint32_t j = 0; j < MOTOR_PER_GROUP; j++) {
        if (s_grp_a.motors[j])
            s_grp_a.motors[j]->initialized = false;
    }
    memset(&s_grp_a, 0, sizeof(s_grp_a));
    MOTOR_LOG_I("grpA deinit ok");

    for (uint32_t j = 0; j < MOTOR_PER_GROUP; j++) {
        if (s_grp_b.motors[j])
            s_grp_b.motors[j]->initialized = false;
    }
    memset(&s_grp_b, 0, sizeof(s_grp_b));
    MOTOR_LOG_I("grpB deinit ok");

    s_initialized = false;
    MOTOR_LOG_I("srv_motor deinit all done");
}

/* --- 电机注册 --- */

srv_motor_error_t srv_motor_register(srv_motor_handle_t* inst,
    uint8_t dev_id, uint8_t uart_idx)
{
    if (!inst || !s_initialized)
        return SRV_MOTOR_ERROR_NULL_PTR;
    if (dev_id < 1 || dev_id > MOTOR_PER_GROUP || uart_idx >= 2)
        return SRV_MOTOR_ERROR_INVALID_PARAM;

    srv_motor_group_t* g = s_grp_from_ch((drv_uart_channel_t)uart_idx);
    if (g->motors[dev_id - 1])
        return SRV_MOTOR_ERROR_ALREADY_EXIST;

    memset(inst, 0, sizeof(*inst));
    inst->dev_id = dev_id;
    inst->uart_ch = (drv_uart_channel_t)uart_idx;
    inst->initialized = true;

    g->motors[dev_id - 1] = inst;
    g->count++;
    MOTOR_LOG_I("reg motor dev=%u ch=%u grp%u total=%u",
        dev_id, (unsigned)uart_idx, (unsigned)uart_idx, (unsigned)g->count);
    return SRV_MOTOR_OK;
}

/* --- 控制接口 --- */

void srv_motor_set_setpoint(srv_motor_handle_t* inst,
    int16_t pos_ref, int16_t spd_ref, int16_t cur_ref)
{
    if (!inst || !inst->initialized)
        return;
    inst->pos_ref = pos_ref;
    inst->spd_ref = spd_ref;
    inst->cur_ref = cur_ref;
    MOTOR_LOG_D("setpoint dev=%u pos=%d spd=%d cur=%d",
        inst->dev_id, pos_ref, spd_ref, cur_ref);
}

void srv_motor_enable(srv_motor_handle_t* inst, bool en)
{
    if (!inst || !inst->initialized)
        return;

    inst->en_cmd = en ? SRV_MOTOR_CMD_ENABLE : SRV_MOTOR_CMD_DISABLE;
    MOTOR_LOG_I("motor dev=%u %s", inst->dev_id, en ? "ENABLE" : "DISABLE");

    srv_motor_group_t* g = s_grp_from_ch(inst->uart_ch);
    uint8_t slot = inst->dev_id - 1;
    g->en_states[slot] = inst->en_cmd;
    g->en_dirty = true;
}

void srv_motor_set_current(srv_motor_handle_t* inst, int16_t cur_ref)
{
    if (!inst || !inst->initialized) return;
    if (inst->cur_ref == cur_ref) return;  /* 未变化，跳过 */
    inst->cur_ref = cur_ref;
    inst->cur_pending = true;
    MOTOR_LOG_D("cur limit dev=%u cur=%d", inst->dev_id, cur_ref);
}

srv_motor_handle_t* srv_motor_get_handle(uint32_t index)
{
    return (index < SRV_MOTOR_TOTAL) ? &s_handle[index] : NULL;
}

/* --- 模式切换 --- */

void srv_motor_set_mode(srv_motor_handle_t* inst, srv_motor_ctrl_mode_t mode)
{
    if (!inst || !inst->initialized)
        return;
    if (inst->ctrl_mode == mode)
        return; /* 未变化 */

    inst->ctrl_mode = mode;
    srv_motor_group_t* g = s_grp_from_ch(inst->uart_ch);
    g->mode_dirty = true;
    MOTOR_LOG_I("dev=%u mode switch to %u", inst->dev_id, mode);
}

/* --- 零点校准 --- */

void srv_motor_zero_reset(srv_motor_handle_t* inst)
{
    if (!inst || !inst->initialized)
        return;

    srv_motor_group_t* g = s_grp_from_ch(inst->uart_ch);

    /* 等待 DMA 空闲再发（通常等 <1ms，零点是低频操作不阻塞业务） */
    while (drv_uart_is_tx_busy(g->uart_ch)) { /* spin */ }

    uint8_t data[8] = { inst->dev_id, SRV_MOTOR_INDEX_UPDATE_FIN };
    uint8_t frame[SRV_MOTOR_FRAME_SIZE];
    build_std_frame(frame, SRV_MOTOR_CAN_ID_CONFIG, data);
    drv_uart_send(g->uart_ch, frame, SRV_MOTOR_FRAME_SIZE);

    MOTOR_LOG_I("dev=%u zero reset sent", inst->dev_id);
}

void srv_motor_zero_reset_all(drv_uart_channel_t ch)
{
    srv_motor_group_t* g = s_grp_from_ch(ch);
    if (!g) return;

    for (uint32_t i = 0; i < MOTOR_PER_GROUP; i++) {
        if (g->motors[i]) {
            while (drv_uart_is_tx_busy(ch)) { /* spin */ }
            uint8_t data[8] = { g->motors[i]->dev_id, SRV_MOTOR_INDEX_UPDATE_FIN };
            uint8_t frame[SRV_MOTOR_FRAME_SIZE];
            build_std_frame(frame, SRV_MOTOR_CAN_ID_CONFIG, data);
            drv_uart_send(ch, frame, SRV_MOTOR_FRAME_SIZE);
        }
    }

    MOTOR_LOG_I("zero reset ALL on ch=%u", (unsigned)ch);
}

/* --- 反馈接口 --- */

const srv_motor_feedback_t* srv_motor_get_feedback(const srv_motor_handle_t* inst)
{
    return (inst && inst->initialized) ? &inst->fb : NULL;
}

bool srv_motor_is_online(const srv_motor_handle_t* inst)
{
    return inst && inst->initialized && (inst->fb.time_fb > 0);
}

/* --- 核心步进 (每 1ms 由 motor_task 调用) --- */

void srv_motor_step(void)
{
    if (!s_initialized)
        return;

    /* ── 组A ── */
    drv_uart_rx_restart(s_grp_a.uart_ch);
    motor_drain_rx(&s_grp_a, s_grp_a.uart_ch);
    if (s_grp_a.count) {
        uint32_t now = micros();
        if (now - s_grp_a.frame_tick >= (MOTOR_POST_DLY_US + MOTOR_ONE_FRAME_TIME_US)) {
            fsm_step(&s_grp_a.fsm);
            s_grp_a.frame_tick = micros();
        }
    }

    // /* ── 组B ── */
    // motor_drain_rx(&s_grp_b, s_grp_b.uart_ch);
    // if (s_grp_b.count) {
    //     uint32_t now = micros();
    //     if (now - s_grp_b.frame_tick >= (MOTOR_POST_DLY_US + MOTOR_ONE_FRAME_TIME_US)) {
    //         fsm_step(&s_grp_b.fsm);
    //         s_grp_b.frame_tick = micros();
    //     }
    // }

}

/* ====================================================================
 * FSM handler — 每个状态的处理函数
 *
 * handler 返回下个目标状态：
 *   条件满足 → 执行动作 + 返回下个状态，fsm_step 自动迁移
 *   条件不满足 → 返回当前状态（保持），下次 step 重试
 * ==================================================================== */

/**
 * @brief STARTUP 状态 — 启动序列：ENABLE→等校准→SET_MODE→IDLE
 *
 * 每个步骤在 DMA 空闲时发送一帧，自动推进。
 * 校准等待期间不阻塞主循环。
 */
static fsm_state_t fsm_startup(fsm_t* ctx)
{
    srv_motor_group_t* g = motor_grp_from_fsm(ctx);

    switch (g->startup_step) {
    case 0: /* ENABLE 所有电机 */
        if (!drv_uart_is_tx_busy(g->uart_ch)) {
            /* 跳过空槽位 */
            while (g->startup_motor < MOTOR_PER_GROUP && !g->motors[g->startup_motor])
                g->startup_motor++;

            if (g->startup_motor >= MOTOR_PER_GROUP) {
                /* 全部发完 → 进入等待校准 */
                g->startup_step = 1;
                break;
            }

            uint8_t data[8] = { 0,0, 0,0, 0,0, SRV_MOTOR_CMD_ENABLE, 0 };
            uint8_t f[20];
            build_std_frame(f, g->motors[g->startup_motor]->dev_id, data);
            drv_uart_send(g->uart_ch, f, 20);
            g->enable_time = millis();
            MOTOR_LOG_I("grp%u ENABLE dev=%u (%u/%u)",
                (unsigned)s_grp_idx(g), g->motors[g->startup_motor]->dev_id,
                (unsigned)(g->startup_motor + 1), (unsigned)g->count);
            g->startup_motor++;
        }
        return MOTOR_ST_STARTUP;

    case 1: /* 等待编码器校准 ~1.5s */
        if (millis() - g->enable_time >= 1500) {
            g->startup_motor = 0;
            g->startup_step = 2;
        }
        return MOTOR_ST_STARTUP;

    case 2: /* SET_MODE 所有电机 */
        if (!drv_uart_is_tx_busy(g->uart_ch)) {
            while (g->startup_motor < MOTOR_PER_GROUP && !g->motors[g->startup_motor])
                g->startup_motor++;

            if (g->startup_motor >= MOTOR_PER_GROUP) {
                MOTOR_LOG_I("grp%u startup done, entering run",
                    (unsigned)s_grp_idx(g));
                g->cycle_start = millis();
                return MOTOR_ST_IDLE;
            }

            uint8_t data[8] = { g->motors[g->startup_motor]->dev_id,
                SRV_MOTOR_INDEX_CTRL_MODE, 0, 0,
                SRV_MOTOR_MODE_POSITION, 0, 0, 0 };
            uint8_t f[20];
            build_std_frame(f, SRV_MOTOR_CAN_ID_CONFIG, data);
            drv_uart_send(g->uart_ch, f, 20);
            MOTOR_LOG_I("grp%u SET_MODE dev=%u (%u/%u)",
                (unsigned)s_grp_idx(g), g->motors[g->startup_motor]->dev_id,
                (unsigned)(g->startup_motor + 1), (unsigned)g->count);
            g->startup_motor++;
        }
        return MOTOR_ST_STARTUP;

    default:
        break;
    }
    return MOTOR_ST_STARTUP; /* step 0 全部发完 → 无 break 继续下一轮 */
}

/**
 * @brief BCAST_EN 状态 — 模式切换 + 使能广播（条件进入）
 */
static fsm_state_t fsm_bcast_en(fsm_t* ctx)
{
    srv_motor_group_t* g = motor_grp_from_fsm(ctx);

    /* 控制模式切换（每轮最多处理一项） */
    if (g->mode_dirty) {
        if (!drv_uart_is_tx_busy(g->uart_ch)) {
            for (uint32_t i = 0; i < MOTOR_PER_GROUP; i++) {
                if (g->motors[i]) {
                    uint8_t data[8] = { g->motors[i]->dev_id,
                        SRV_MOTOR_INDEX_CTRL_MODE, 0, 0,
                        g->motors[i]->ctrl_mode, 0, 0, 0 };
                    uint8_t f[20];
                    build_std_frame(f, SRV_MOTOR_CAN_ID_CONFIG, data);
                    drv_uart_send(g->uart_ch, f, 20);
                }
            }
            g->mode_dirty = false;
            MOTOR_LOG_D("grp%u mode cfg sent", (unsigned)s_grp_idx(g));
        }
        return MOTOR_ST_BCAST_EN; /* 暂不退：下轮取 en_dirty */
    }

    /* 使能广播 */
    if (!g->en_dirty)
        return MOTOR_ST_BCAST_POS; /* 无变化，直接跳过 */

    if (!drv_uart_is_tx_busy(g->uart_ch)) {
        uint8_t frame[SRV_MOTOR_FRAME_SIZE];
        build_enable_frame(frame, g->en_states);
        drv_uart_send(g->uart_ch, frame, SRV_MOTOR_FRAME_SIZE);
        g->en_dirty = false;
        MOTOR_LOG_D("grp%u en bcast sent", (unsigned)s_grp_idx(g));
        return MOTOR_ST_BCAST_POS;
    }
    return MOTOR_ST_BCAST_EN; /* DMA 忙，等待 */
}

/**
 * @brief BCAST_POS 状态 — 发送位置广播帧 AB55/AC55 交替
 */
static fsm_state_t fsm_bcast_pos(fsm_t* ctx)
{
    srv_motor_group_t* g = motor_grp_from_fsm(ctx);

    if (!drv_uart_is_tx_busy(g->uart_ch)) {
        int16_t values[8];
        for (uint32_t i = 0; i < MOTOR_PER_GROUP; i++)
            values[i] = g->motors[i] ? g->motors[i]->pos_ref : 0;

        uint8_t frame[SRV_MOTOR_FRAME_SIZE];
        build_bcast_frame(frame, SRV_MOTOR_BC_POS_ID18, values);
        drv_uart_send(g->uart_ch, frame, SRV_MOTOR_FRAME_SIZE);

        return MOTOR_ST_POLL;
    }
    return MOTOR_ST_BCAST_POS;
}

/**
 * @brief POLL 状态 — 反馈查询，每周期 1 个已注册电机
 */
static fsm_state_t fsm_poll(fsm_t* ctx)
{
    srv_motor_group_t* g = motor_grp_from_fsm(ctx);

    if (!drv_uart_is_tx_busy(g->uart_ch)) {
        motor_poll_one(g, g->uart_ch);
        MOTOR_LOG_D("grp%u poll ok", (unsigned)s_grp_idx(g));
        return MOTOR_ST_BCAST_CUR;
    }
    return MOTOR_ST_POLL;
}

/**
 * @brief IDLE 状态 — 等待广播周期间隔
 */
static fsm_state_t fsm_idle(fsm_t* ctx)
{
    srv_motor_group_t* g = motor_grp_from_fsm(ctx);
    if (millis() - g->cycle_start >= MOTOR_BCAST_PERIOD_MS) {
        g->cycle_start = millis();
        MOTOR_LOG_D("grp%u tick, start bcast seq", (unsigned)s_grp_idx(g));
        return MOTOR_ST_BCAST_EN; /* 优先检查使能广播 */
    }
    return MOTOR_ST_IDLE;
}

/**
 * @brief BCAST_CUR — 发送 MAX_CUR 配置，同周期紧接 BCAST_LIM
 */
static fsm_state_t fsm_bcast_cur(fsm_t* ctx)
{
    srv_motor_group_t* g = motor_grp_from_fsm(ctx);
    if (!g->count) return MOTOR_ST_IDLE;

    uint8_t slot = 0;
    for (uint8_t n = 0; slot < MOTOR_PER_GROUP && n <= g->cur_cursor; slot++)
        if (g->motors[slot]) n++;
    slot--;
    if (slot >= MOTOR_PER_GROUP || !g->motors[slot]) return MOTOR_ST_IDLE;
    srv_motor_handle_t* m = g->motors[slot];

    if (!drv_uart_is_tx_busy(g->uart_ch)) {
        uint8_t data[8] = { m->dev_id, SRV_MOTOR_INDEX_MAX_CUR, 0, 0,
            (uint8_t)m->cur_ref, (uint8_t)(m->cur_ref >> 8), 0, 0 };
        uint8_t frame[SRV_MOTOR_FRAME_SIZE];
        build_std_frame(frame, SRV_MOTOR_CAN_ID_CONFIG, data);
        drv_uart_send(g->uart_ch, frame, SRV_MOTOR_FRAME_SIZE);
        MOTOR_LOG_I("cfg MAX_CUR dev=%u val=%d", m->dev_id, m->cur_ref);
        return MOTOR_ST_BCAST_LIM;
    }
    return MOTOR_ST_BCAST_CUR;
}

/**
 * @brief BCAST_LIM — 发送 SPD_LIMIT 配置，然后推进游标
 */
static fsm_state_t fsm_bcast_lim(fsm_t* ctx)
{
    srv_motor_group_t* g = motor_grp_from_fsm(ctx);

    uint8_t slot = 0;
    for (uint8_t n = 0; slot < MOTOR_PER_GROUP && n <= g->cur_cursor; slot++)
        if (g->motors[slot]) n++;
    slot--;
    if (slot >= MOTOR_PER_GROUP || !g->motors[slot]) return MOTOR_ST_IDLE;
    srv_motor_handle_t* m = g->motors[slot];

    if (!drv_uart_is_tx_busy(g->uart_ch)) {
        uint8_t data[8] = { m->dev_id, SRV_MOTOR_INDEX_SPD_LIMIT, 0, 0,
            (uint8_t)m->spd_ref, (uint8_t)(m->spd_ref >> 8), 0, 0 };
        uint8_t frame[SRV_MOTOR_FRAME_SIZE];
        build_std_frame(frame, SRV_MOTOR_CAN_ID_CONFIG, data);
        drv_uart_send(g->uart_ch, frame, SRV_MOTOR_FRAME_SIZE);
        MOTOR_LOG_I("cfg SPD_LIMIT dev=%u val=%d", m->dev_id, m->spd_ref);
        g->cur_cursor = (uint8_t)((g->cur_cursor + 1) % g->count);
        return MOTOR_ST_IDLE;
    }
    return MOTOR_ST_BCAST_LIM;
}

/* ====================================================================
 * 帧构建函数
 * ==================================================================== */

/**
 * @brief 构建 20 字节标准帧 (head + can_id + can_data + crc)
 * @note  CRC 覆盖 can_id + can_data（12 字节），帧头不计入
 */
static void build_std_frame(uint8_t buf[SRV_MOTOR_FRAME_SIZE],
    uint32_t can_id, const uint8_t data[8])
{
    memset(buf, 0, SRV_MOTOR_FRAME_SIZE);
    buf[0] = 0x55;
    buf[1] = 0xAA;
    buf[2] = 0x00;
    buf[3] = 0x14; /* 帧头小端 */
    buf[4] = (uint8_t)can_id;
    buf[5] = (uint8_t)(can_id >> 8);
    buf[6] = (uint8_t)(can_id >> 16);
    buf[7] = (uint8_t)(can_id >> 24);
    memcpy(&buf[8], data, 8);

    uint16_t crc16 = get_CRC16_CCITT_FALSE(&buf[4], 12);
    buf[16] = (uint8_t)crc16;
    buf[17] = (uint8_t)(crc16 >> 8);
    /* buf[18-19] 保持 0，CRC 字段高 2 字节补零 */
}

/**
 * @brief 构建 20 字节广播帧 (head2 + value[8] + crc16)
 * @note  CRC 仅覆盖 value（16 字节），不含 head2
 */
static void build_bcast_frame(uint8_t buf[SRV_MOTOR_FRAME_SIZE],
    uint16_t head2, const int16_t values[8])
{
    memset(buf, 0, SRV_MOTOR_FRAME_SIZE);
    buf[0] = (uint8_t)head2;
    buf[1] = (uint8_t)(head2 >> 8);

    for (uint32_t i = 0; i < 8; i++) {
        buf[2 + i * 2] = (uint8_t)values[i];
        buf[2 + i * 2 + 1] = (uint8_t)(values[i] >> 8);
    }

    uint16_t crc16 = get_CRC16_CCITT_FALSE(&buf[2], 16);
    buf[18] = (uint8_t)crc16;
    buf[19] = (uint8_t)(crc16 >> 8);
}

/**
 * @brief 构建使能广播帧 (0xB155 + en_states[16] + crc16)
 */
static void build_enable_frame(uint8_t buf[SRV_MOTOR_FRAME_SIZE],
    const uint8_t en_states[16])
{
    memset(buf, 0, SRV_MOTOR_FRAME_SIZE);
    buf[0] = (uint8_t)SRV_MOTOR_BC_ENABLE;
    buf[1] = (uint8_t)(SRV_MOTOR_BC_ENABLE >> 8);
    memcpy(&buf[2], en_states, 16);

    uint16_t crc16 = get_CRC16_CCITT_FALSE(buf, 18);
    buf[18] = (uint8_t)crc16;
    buf[19] = (uint8_t)(crc16 >> 8);
}

/**
 * @brief 构建配置查询帧 (can_id=0xA0, data={dev_id, index, 0...})
 */
static void build_poll_frame(uint8_t buf[SRV_MOTOR_FRAME_SIZE],
    uint8_t dev_id, uint8_t cfg_index)
{
    uint8_t data[8] = { dev_id, cfg_index };
    build_std_frame(buf, SRV_MOTOR_CAN_ID_CONFIG, data);
}

/* ====================================================================
 * 辅助函数
 * ==================================================================== */

/**
 * @brief 轮询下一个电机的反馈
 *
 * Round-robin 游标遍历 motors[] 数组，跳过未注册的槽位。
 * 每 POLL_01_INTERVAL 次 INFO_02_R 插入一次 INFO_01_R 查询。
 *
 * 调用前已通过 drv_uart_is_tx_busy() 确保 DMA 空闲。
 */
static void motor_poll_one(srv_motor_group_t* g, drv_uart_channel_t ch)
{
    if (!g->count) return;

    /* 找到第 poll_cursor 个已注册电机（0-based compact → 实际 slot） */
    uint8_t slot = 0;
    for (uint8_t n = 0; slot < MOTOR_PER_GROUP && n <= g->poll_cursor; slot++)
        if (g->motors[slot]) n++;
    slot--; /* 回退到最后匹配的 slot */
    if (slot >= MOTOR_PER_GROUP || !g->motors[slot]) return;

    srv_motor_handle_t* tgt = g->motors[slot];

    /* 选择查询索引：每 POLL_01_INTERVAL 次插一次 INFO_01_R */
    uint8_t idx = (++g->poll_02_cnt >= POLL_01_INTERVAL)
        ? (g->poll_02_cnt = 0, SRV_MOTOR_INDEX_INFO_01_R)
        : SRV_MOTOR_INDEX_INFO_02_R;

    uint8_t frame[SRV_MOTOR_FRAME_SIZE];
    build_poll_frame(frame, tgt->dev_id, idx);
    drv_uart_send(ch, frame, SRV_MOTOR_FRAME_SIZE);

    MOTOR_LOG_D("poll dev=%u idx=%u sent", tgt->dev_id, idx);
    tgt->query_index = idx;
    g->poll_cursor = (uint8_t)((g->poll_cursor + 1) % g->count);
}

/**
 * @brief UART RX 中断回调 — DMA 收到数据后入队（ISR 安全）
 */
static void motor_rx_cb(drv_uart_channel_t ch, const uint8_t* data, uint32_t len)
{
    srv_motor_group_t* g = (ch == s_grp_a.uart_ch) ? &s_grp_a : &s_grp_b;
    for (uint32_t off = 0; off + SRV_MOTOR_FRAME_SIZE <= len; off += SRV_MOTOR_FRAME_SIZE)
        msg_fifo_push(&g->rx_queue, data + off);
}

/**
 * @brief 从 msg_fifo 取出所有待解析帧（主循环调用）
 */
static void motor_drain_rx(srv_motor_group_t* g, drv_uart_channel_t ch)
{
    (void)ch;
    uint32_t count = 0;
    uint8_t buf[SRV_MOTOR_FRAME_SIZE];
    while (msg_fifo_pop(&g->rx_queue, buf)) {
        motor_parse_reply(buf, g);
        count++;
    }
    // if (count > 0)
    //     MOTOR_LOG_D("grp%u parsed %lu frames", (unsigned)s_grp_idx(g), (unsigned long)count);
}

/**
 * @brief 解析一条回复帧，更新对应电机的反馈数据
 *
 * 帧头 0x1400AA55 (小端: 55 AA 00 14)，CRC 校验 can_id+can_data 共 12 字节。
 * can_id 结构: byte[0]=0, byte[1]=dev_id, byte[2]=index。
 *
 * INFO_02_R (index=2): D_cur + Q_cur + speed_fb + angle_fb (各 2 字节 LE)
 * INFO_01_R (index=1): fsm + err_code + soft_ver + temp + angle_fb + vbus
 */
#define LE16(p, off) ((int16_t)((uint16_t)(p)[off] | ((uint16_t)(p)[(off) + 1] << 8)))

static void motor_parse_reply(const uint8_t frame[SRV_MOTOR_FRAME_SIZE],
    srv_motor_group_t* g)
{
    /* 验证帧头: 0x55 0xAA 0x00 0x14 */
    if (frame[0] != 0x55 || frame[1] != 0xAA || frame[2] != 0x00 || frame[3] != 0x14)
        return;

    /* 验证 CRC: can_id(4B) + can_data(8B) */
    uint16_t crc = (uint16_t)frame[16] | ((uint16_t)frame[17] << 8);
    if (get_CRC16_CCITT_FALSE((uint8_t*)&frame[4], 12) != crc)
        return;

    uint8_t dev_id = frame[5]; /* can_id byte[1] */
    uint8_t cfg_index = frame[6]; /* can_id byte[2] */

    /* O(1) 查找：dev_id-1 直接索引 motors[] */
    srv_motor_handle_t* inst = (dev_id >= 1 && dev_id <= MOTOR_PER_GROUP)
        ? g->motors[dev_id - 1]
        : NULL;
    if (!inst)
        return;

    if (cfg_index == SRV_MOTOR_INDEX_INFO_02_R) {
        inst->fb.d_cur = LE16(frame, 8);
        inst->fb.q_cur = LE16(frame, 10);
        inst->fb.speed_fb = LE16(frame, 12);
        inst->fb.angle_fb = LE16(frame, 14);
        inst->fb.time_fb = millis();
        MOTOR_LOG_D("fb dev=%u angle=%d spd=%d q_cur=%d",
            dev_id, inst->fb.angle_fb, inst->fb.speed_fb, inst->fb.q_cur);

    } else if (cfg_index == SRV_MOTOR_INDEX_INFO_01_R) {
        inst->fb.fsm_state = frame[8];
        inst->fb.err_code = frame[9];
        inst->fb.temp = (int8_t)frame[11];
        inst->fb.angle_fb = LE16(frame, 12);
        inst->fb.vbus = LE16(frame, 14);
        inst->fb.time_status = millis();
        MOTOR_LOG_D("status dev=%u fsm=%u err=%u temp=%d vbus=%d",
            dev_id, (unsigned)inst->fb.fsm_state, (unsigned)inst->fb.err_code,
            inst->fb.temp, inst->fb.vbus);
    }
}

/**
 * @brief 从 fsm_t 指针反查所属电机组
 * @param ctx FSM 上下文指针
 * @return 所属电机组指针
 */
static srv_motor_group_t* motor_grp_from_fsm(fsm_t* ctx)
{
    if (&s_grp_a.fsm == ctx)
        return &s_grp_a;
    if (&s_grp_b.fsm == ctx)
        return &s_grp_b;
    return NULL;
}
