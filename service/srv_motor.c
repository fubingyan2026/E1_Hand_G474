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
 *   IDLE ─(3ms)─► BCAST_EN ─► BCAST_POS ─► BCAST_SPD ─► POLL_1 ─► POLL_2 ─► BCAST_CUR ─► RX ─► IDLE
 *                   (有脏则发,                        (轮询cur_pending,
 *                    无脏跳过)                          无则跳过)
 *
 *  广播 300 Hz，反馈 ~133/167 Hz/motor。
 *  每个 handler 在 DMA 空闲时执行发送 → fsm_step 自动迁移到下一状态。
 */

#include "srv_motor.h"

#include "crc.h"
#include "drv_systick.h"
#include "drv_uart.h"
#include "fsm.h"

#include <string.h>

/* Private constants ---------------------------------------------------------*/

/** @brief 每组最大电机数（广播帧 8 槽位） */
#define MOTOR_PER_GROUP 8U

/** @brief 每 N 次 INFO_02_R 后插入一次 INFO_01_R */
#define POLL_01_INTERVAL 10U

/** @brief 广播控制周期 (ms) */
#define BCAST_PERIOD_MS 3U

/**
 * @brief 分时发送 FSM 状态
 *
 * 每路 UART 独立运行该状态机，handler 在 DMA 空闲时发送一帧并迁移。
 * IDLE → BCAST_POS → BCAST_SPD → POLL_1 → POLL_2 → BCAST_EN(条件) → RX → IDLE
 */
typedef enum {
    MOTOR_ST_IDLE = 0, /**< 等待 3ms 周期启动 */
    MOTOR_ST_BCAST_POS, /**< 发送位置广播帧 */
    MOTOR_ST_BCAST_SPD, /**< 发送速度广播帧 */
    MOTOR_ST_POLL_1, /**< 发送第 1 路反馈查询 */
    MOTOR_ST_POLL_2, /**< 发送第 2 路反馈查询 */
    MOTOR_ST_BCAST_EN, /**< 发送使能广播帧（条件进入） */
    MOTOR_ST_BCAST_CUR, /**< 发送电流限制帧（轮询每个电机） */
    MOTOR_ST_RX, /**< 等待下一个 3ms 周期 */
    MOTOR_STATE_COUNT /**< 状态总数（必须保持在最后） */
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
    uint8_t uart_idx; /**< UART 索引 (2=USART2, 3=USART3) */
    uint8_t poll_cursor; /**< 轮询游标 */
    uint32_t poll_02_cnt; /**< INFO_02_R 计数器 */
    uint32_t cycle_start; /**< 当前广播周期起始时间 (millis) */
    uint8_t cur_cursor; /**< 电流限制轮询游标 */
    bool en_dirty; /**< 使能状态脏标志 */
    uint8_t en_states[16]; /**< 各 ID 使能命令缓存 */
} srv_motor_group_t;

/* 模块全局变量 --------------------------------------------------------------*/

/** @brief 两组电机实例（索引 0=USART2, 1=USART3） */
static srv_motor_group_t s_grp[2];

/** @brief 模块初始化标志 */
static bool s_initialized;

/* FSM 静态资源（两组共用同一套 handler/transition 表） ----------------------*/

static fsm_handler_t s_handlers[MOTOR_STATE_COUNT];
static fsm_guard_t s_transitions[MOTOR_STATE_COUNT * MOTOR_STATE_COUNT];

/* 函数原型 ------------------------------------------------------------------*/

/* FSM handler */
static fsm_state_t fsm_idle(fsm_t* ctx);
static fsm_state_t fsm_bcast_pos(fsm_t* ctx);
static fsm_state_t fsm_bcast_spd(fsm_t* ctx);
static fsm_state_t fsm_poll_1(fsm_t* ctx);
static fsm_state_t fsm_poll_2(fsm_t* ctx);
static fsm_state_t fsm_bcast_en(fsm_t* ctx);
static fsm_state_t fsm_bcast_cur(fsm_t* ctx);
static fsm_state_t fsm_rx(fsm_t* ctx);

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
static void motor_send_broadcast(srv_motor_group_t* g, uint16_t head2);
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

    s_handlers[MOTOR_ST_IDLE] = fsm_idle;
    s_handlers[MOTOR_ST_BCAST_POS] = fsm_bcast_pos;
    s_handlers[MOTOR_ST_BCAST_SPD] = fsm_bcast_spd;
    s_handlers[MOTOR_ST_POLL_1] = fsm_poll_1;
    s_handlers[MOTOR_ST_POLL_2] = fsm_poll_2;
    s_handlers[MOTOR_ST_BCAST_EN]  = fsm_bcast_en;
    s_handlers[MOTOR_ST_BCAST_CUR] = fsm_bcast_cur;
    s_handlers[MOTOR_ST_RX]        = fsm_rx;

    fsm_config_t fsm_cfg = {
        .handlers = s_handlers,
        .transitions = s_transitions,
        .state_count = MOTOR_STATE_COUNT,
    };
    fsm_fill(&fsm_cfg, fsm_always_true);

    /* 初始化两组电机 */
    for (uint32_t i = 0; i < 2; i++) {
        srv_motor_group_t* g = &s_grp[i];
        memset(g, 0, sizeof(*g));
        g->uart_idx = (uint8_t)(i + 2); /* 2=USART2, 3=USART3 */
        fsm_init(&g->fsm, MOTOR_ST_IDLE, &fsm_cfg);
    }

    s_initialized = true;
    return SRV_MOTOR_OK;
}

void srv_motor_deinit(void)
{
    if (!s_initialized)
        return;
    for (uint32_t i = 0; i < 2; i++) {
        srv_motor_group_t* g = &s_grp[i];
        for (uint32_t j = 0; j < MOTOR_PER_GROUP; j++)
            if (g->motors[j])
                g->motors[j]->initialized = false;
        memset(g, 0, sizeof(*g));
    }
    s_initialized = false;
}

/* --- 电机注册 --- */

srv_motor_error_t srv_motor_register(srv_motor_handle_t* inst,
    uint8_t dev_id, uint8_t uart_idx)
{
    if (!inst || !s_initialized)
        return SRV_MOTOR_ERROR_NULL_PTR;
    if (dev_id < 1 || dev_id > MOTOR_PER_GROUP)
        return SRV_MOTOR_ERROR_INVALID_PARAM;
    if (uart_idx != 2 && uart_idx != 3)
        return SRV_MOTOR_ERROR_INVALID_PARAM;

    srv_motor_group_t* g = &s_grp[uart_idx - 2];
    if (g->motors[dev_id - 1])
        return SRV_MOTOR_ERROR_ALREADY_EXIST;

    memset(inst, 0, sizeof(*inst));
    inst->dev_id = dev_id;
    inst->uart_idx = uart_idx;
    inst->initialized = true;

    g->motors[dev_id - 1] = inst;
    g->count++;
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
}

void srv_motor_enable(srv_motor_handle_t* inst, bool en)
{
    if (!inst || !inst->initialized)
        return;

    inst->en_cmd = en ? SRV_MOTOR_CMD_ENABLE : SRV_MOTOR_CMD_DISABLE;

    srv_motor_group_t* g = &s_grp[inst->uart_idx - 2];
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

    for (uint32_t i = 0; i < 2; i++) {
        srv_motor_group_t* g = &s_grp[i];
        if (!g->count)
            continue;

        drv_uart_channel_t ch = (drv_uart_channel_t)(DRV_UART_CH_1 + i);

        /* RX 解析始终运行，不与 TX 状态耦合 */
        motor_drain_rx(g, ch);

        /* 驱动 FSM：每步执行当前状态 handler，条件满足时自动迁移 */
        fsm_step(&g->fsm);
    }
}

/* ====================================================================
 * FSM handler — 每个状态的处理函数
 *
 * handler 返回下个目标状态：
 *   条件满足 → 执行动作 + 返回下个状态，fsm_step 自动迁移
 *   条件不满足 → 返回当前状态（保持），下次 step 重试
 * ==================================================================== */

/**
 * @brief IDLE 状态 — 等待 3ms 广播周期间隔
 */
static fsm_state_t fsm_idle(fsm_t* ctx)
{
    srv_motor_group_t* g = motor_grp_from_fsm(ctx);
    if (millis() - g->cycle_start >= BCAST_PERIOD_MS) {
        g->cycle_start = millis();
        return MOTOR_ST_BCAST_EN; /* 优先检查使能广播 */
    }
    return MOTOR_ST_IDLE;
}

/**
 * @brief BCAST_POS 状态 — 发送位置广播帧
 */
static fsm_state_t fsm_bcast_pos(fsm_t* ctx)
{
    srv_motor_group_t* g = motor_grp_from_fsm(ctx);
    drv_uart_channel_t ch = (drv_uart_channel_t)(DRV_UART_CH_1 + (g->uart_idx - 2));

    if (!drv_uart_is_tx_busy(ch)) {
        motor_send_broadcast(g, SRV_MOTOR_BC_POS_ID18);
        return MOTOR_ST_BCAST_SPD;
    }
    return MOTOR_ST_BCAST_POS; /* DMA 忙，等候 */
}

/**
 * @brief BCAST_SPD 状态 — 发送速度广播帧
 */
static fsm_state_t fsm_bcast_spd(fsm_t* ctx)
{
    srv_motor_group_t* g = motor_grp_from_fsm(ctx);
    drv_uart_channel_t ch = (drv_uart_channel_t)(DRV_UART_CH_1 + (g->uart_idx - 2));

    if (!drv_uart_is_tx_busy(ch)) {
        motor_send_broadcast(g, SRV_MOTOR_BC_SPD_ID18);
        return MOTOR_ST_POLL_1;
    }
    return MOTOR_ST_BCAST_SPD;
}

/**
 * @brief POLL_1 状态 — 发送第 1 路反馈查询
 */
static fsm_state_t fsm_poll_1(fsm_t* ctx)
{
    srv_motor_group_t* g = motor_grp_from_fsm(ctx);
    drv_uart_channel_t ch = (drv_uart_channel_t)(DRV_UART_CH_1 + (g->uart_idx - 2));

    if (!drv_uart_is_tx_busy(ch)) {
        motor_poll_one(g, ch);
        return MOTOR_ST_POLL_2;
    }
    return MOTOR_ST_POLL_1;
}

/**
 * @brief POLL_2 状态 — 发送第 2 路反馈查询
 */
static fsm_state_t fsm_poll_2(fsm_t* ctx)
{
    srv_motor_group_t* g = motor_grp_from_fsm(ctx);
    drv_uart_channel_t ch = (drv_uart_channel_t)(DRV_UART_CH_1 + (g->uart_idx - 2));

    if (!drv_uart_is_tx_busy(ch)) {
        motor_poll_one(g, ch);
        return MOTOR_ST_BCAST_CUR;
    }
    return MOTOR_ST_POLL_2;
}

/**
 * @brief RX 状态 — 回到 IDLE 等待下个 3ms 周期
 */
static fsm_state_t fsm_rx(fsm_t* ctx)
{
    (void)ctx;
    return MOTOR_ST_IDLE;
}

/**
 * @brief BCAST_EN 状态 — 若使能有变化则发送广播，否则直接跳过到 BCAST_POS
 */
static fsm_state_t fsm_bcast_en(fsm_t* ctx)
{
    srv_motor_group_t* g = motor_grp_from_fsm(ctx);

    if (!g->en_dirty)
        return MOTOR_ST_BCAST_POS; /* 无变化，直接跳过 */

    drv_uart_channel_t ch = (drv_uart_channel_t)(DRV_UART_CH_1 + (g->uart_idx - 2));
    if (!drv_uart_is_tx_busy(ch)) {
        uint8_t frame[SRV_MOTOR_FRAME_SIZE];
        build_enable_frame(frame, g->en_states);
        drv_uart_send(ch, frame, SRV_MOTOR_FRAME_SIZE);
        g->en_dirty = false;
        return MOTOR_ST_BCAST_POS;
    }
    return MOTOR_ST_BCAST_EN; /* DMA 忙，等待 */
}

/**
 * @brief BCAST_CUR 状态 — 轮询发送电流限制帧
 */
/**
 * @brief BCAST_CUR 状态 — 每周期最多发送 1 个电机的电流限制
 *
 * 轮询找到下一个 cur_pending 的电机，DMA 空闲则发送标准帧。
 * DMA 忙时不阻塞，直接跳到 RX 下周期再试。
 */
static fsm_state_t fsm_bcast_cur(fsm_t* ctx)
{
    srv_motor_group_t* g = motor_grp_from_fsm(ctx);
    drv_uart_channel_t ch = (drv_uart_channel_t)(DRV_UART_CH_1 + (g->uart_idx - 2));

    for (uint32_t n = 0; n < MOTOR_PER_GROUP; n++) {
        srv_motor_handle_t* m = g->motors[g->cur_cursor];
        g->cur_cursor = (g->cur_cursor + 1) % MOTOR_PER_GROUP;

        if (m && m->cur_pending && !drv_uart_is_tx_busy(ch)) {
            uint8_t data[8] = {0};
            data[4] = (uint8_t)m->cur_ref;
            data[5] = (uint8_t)(m->cur_ref >> 8);
            data[6] = SRV_MOTOR_CMD_SET_REF;

            uint8_t frame[SRV_MOTOR_FRAME_SIZE];
            build_std_frame(frame, m->dev_id, data);
            drv_uart_send(ch, frame, SRV_MOTOR_FRAME_SIZE);

            m->cur_pending = false;
            break;  /* 本周期只发一个 */
        }
    }

    return MOTOR_ST_RX;  /* 无论是否发送，继续下个状态 */
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
 * @note  CRC 覆盖 head2 + value（18 字节）
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

    uint16_t crc16 = get_CRC16_CCITT_FALSE(buf, 18);
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
 * @brief 发送广播帧（位置或速度）
 *
 * 遍历 motors[] 数组，将各电机 pos_ref 或 spd_ref 填入广播槽位。
 * 调用前已通过 drv_uart_is_tx_busy() 确保 DMA 空闲。
 *
 * @param g     电机组指针
 * @param head2 广播帧类型 (SRV_MOTOR_BC_POS_ID18 / SRV_MOTOR_BC_SPD_ID18)
 */
static void motor_send_broadcast(srv_motor_group_t* g, uint16_t head2)
{
    int16_t values[8] = { 0 };

    for (uint32_t i = 0; i < MOTOR_PER_GROUP; i++) {
        srv_motor_handle_t* m = g->motors[i];
        if (m) {
            values[i] = (head2 == SRV_MOTOR_BC_POS_ID18) ? m->pos_ref : m->spd_ref;
        }
    }

    uint8_t frame[SRV_MOTOR_FRAME_SIZE];
    build_bcast_frame(frame, head2, values);

    drv_uart_channel_t ch = (drv_uart_channel_t)(DRV_UART_CH_1 + (g->uart_idx - 2));
    drv_uart_send(ch, frame, SRV_MOTOR_FRAME_SIZE);
}

/**
 * @brief 轮询下一个电机的反馈
 *
 * Round-robin 游标遍历 motors[] 数组，跳过未注册的槽位。
 * 每 POLL_01_INTERVAL 次 INFO_02_R 插入一次 INFO_01_R 查询。
 * 若上一查询未收到回复（query_pending），标记超时并跳过本轮。
 *
 * 调用前已通过 drv_uart_is_tx_busy() 确保 DMA 空闲。
 */
static void motor_poll_one(srv_motor_group_t* g, drv_uart_channel_t ch)
{
    /* 跳过未注册的槽位，找到下一个有效电机 */
    while (!g->motors[g->poll_cursor])
        g->poll_cursor = (g->poll_cursor + 1) % MOTOR_PER_GROUP;

    srv_motor_handle_t* tgt = g->motors[g->poll_cursor];

    /* 上轮查询未收到回复则标记超时，跳过本轮 */
    if (tgt->query_pending) {
        tgt->query_pending = false;
        g->poll_cursor = (g->poll_cursor + 1) % MOTOR_PER_GROUP;
        return;
    }

    /* 选择查询索引：每 POLL_01_INTERVAL 次插一次 INFO_01_R */
    uint8_t idx = (++g->poll_02_cnt >= POLL_01_INTERVAL)
        ? (g->poll_02_cnt = 0, SRV_MOTOR_INDEX_INFO_01_R)
        : SRV_MOTOR_INDEX_INFO_02_R;

    uint8_t frame[SRV_MOTOR_FRAME_SIZE];
    build_poll_frame(frame, tgt->dev_id, idx);
    drv_uart_send(ch, frame, SRV_MOTOR_FRAME_SIZE);

    tgt->query_pending = true;
    tgt->query_index = idx;
    g->poll_cursor = (g->poll_cursor + 1) % MOTOR_PER_GROUP;
}

/**
 * @brief 从 UART 接收 FIFO 中取出并解析所有 20 字节帧
 */
static void motor_drain_rx(srv_motor_group_t* g, drv_uart_channel_t ch)
{
    uint8_t rx_buf[SRV_MOTOR_FRAME_SIZE];
    while (drv_uart_rx_available(ch) >= SRV_MOTOR_FRAME_SIZE) {
        if (drv_uart_rx_read(ch, rx_buf, SRV_MOTOR_FRAME_SIZE) == SRV_MOTOR_FRAME_SIZE)
            motor_parse_reply(rx_buf, g);
    }
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

    inst->query_pending = false;

    if (cfg_index == SRV_MOTOR_INDEX_INFO_02_R) {
        inst->fb.d_cur = LE16(frame, 8);
        inst->fb.q_cur = LE16(frame, 10);
        inst->fb.speed_fb = LE16(frame, 12);
        inst->fb.angle_fb = LE16(frame, 14);
        inst->fb.time_fb = millis();

    } else if (cfg_index == SRV_MOTOR_INDEX_INFO_01_R) {
        inst->fb.fsm_state = frame[8];
        inst->fb.err_code = frame[9];
        inst->fb.temp = (int8_t)frame[11];
        inst->fb.angle_fb = LE16(frame, 12);
        inst->fb.vbus = LE16(frame, 14);
        inst->fb.time_status = millis();
    }
}

/**
 * @brief 从 fsm_t 指针反查所属电机组
 * @param ctx FSM 上下文指针
 * @return 所属电机组指针
 */
static srv_motor_group_t* motor_grp_from_fsm(fsm_t* ctx)
{
    for (uint32_t i = 0; i < 2; i++)
        if (&s_grp[i].fsm == ctx)
            return &s_grp[i];
    return NULL;
}
