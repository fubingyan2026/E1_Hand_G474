/**
 * @file    srv_motor.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-07-09
 * @brief   关节电机串口通信服务 — 广播控制 + 分时轮询反馈
 * @attention
 *
 * 通过 2 路 USART 控制 9 个关节模组：
 *   Group A — USART2 (DRV_UART_CH_2), 5 motors, ID 1-5
 *   Group B — USART3 (DRV_UART_CH_3), 4 motors, ID 1-4
 *
 * 控制：位置+速度广播帧，每控制周期每路 2 帧，目标 300 Hz。
 * 反馈：分时轮询 CONFIG_INFO_02_R（角度/速度/DQ电流）优先，CONFIG_INFO_01_R 低频。
 *
 * 帧格式参考 docs/uart_protocol.md，驱动层依赖 drv_uart。
 */

#ifndef __SRV_MOTOR_H
#define __SRV_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "drv_uart.h"

/* Exported constants --------------------------------------------------------*/

/** @brief 电机分组定义 */
#define SRV_MOTOR_GROUP_A_NUM (5U) /**< Group A (USART2) 电机数量 */
#define SRV_MOTOR_GROUP_B_NUM (4U) /**< Group B (USART3) 电机数量 */
#define SRV_MOTOR_TOTAL (SRV_MOTOR_GROUP_A_NUM + SRV_MOTOR_GROUP_B_NUM)

/** @brief 帧常量 */
#define SRV_MOTOR_FRAME_SIZE (20U) /**< 协议帧固定 20 字节 */
#define SRV_MOTOR_FRAME_HEAD (0x1400AA55UL) /**< 标准帧帧头 */

/**
 * @brief 广播帧类型
 */
typedef enum {
    SRV_MOTOR_BC_POS_ID18 = 0xAB55, /**< 位置广播 ID1-8 */
    SRV_MOTOR_BC_SPD_ID18 = 0xAD55, /**< 速度广播 ID1-8 */
    SRV_MOTOR_BC_ENABLE = 0xB155, /**< 使能广播 ID1-16 */
} srv_motor_bc_type_t;

/**
 * @brief 功能 CAN ID
 */
typedef enum {
    SRV_MOTOR_CAN_ID_CONFIG = 0xA0, /**< 配置读写 */
    SRV_MOTOR_CAN_ID_JOG = 0xF0, /**< JOG 点动 */
    SRV_MOTOR_CAN_ID_BCAST = 0xFF, /**< 广播运动控制 */
} srv_motor_can_id_t;

/**
 * @brief 配置项索引（can_id=0xA0 的参数 index，遵循 V2.0.8 协议）
 */
typedef enum {
    SRV_MOTOR_INDEX_INFO_01_R    = 1,  /**< 读：状态/故障/温度/电压 */
    SRV_MOTOR_INDEX_INFO_02_R    = 2,  /**< 读：角度/速度/DQ电流 */
    SRV_MOTOR_INDEX_UPDATE_FIN   = 3,  /**< 写：重置末端位置为零点 */
    SRV_MOTOR_INDEX_SET_ID       = 4,  /**< 写：在线设置节点 ID (value=id<<7) */
    SRV_MOTOR_INDEX_ENC_ZERO     = 5,  /**< 写：磁编码器零点标定 */
    SRV_MOTOR_INDEX_COMM_SILENT  = 6,  /**< 写：通讯静默 */
    SRV_MOTOR_INDEX_CTRL_MODE    = 7,  /**< 写：控制模式切换 (1=速度,2=位置,3=电流) */
    SRV_MOTOR_INDEX_POS_KP       = 8,  /**< 写：位置环 KP (Q12) */
    SRV_MOTOR_INDEX_POS_KD       = 9,  /**< 写：位置环 KD (Q12) */
    SRV_MOTOR_INDEX_SPD_KP       = 10, /**< 写：速度环 KP (Q12) */
    SRV_MOTOR_INDEX_SPD_KI       = 11, /**< 写：速度环 KI (Q15) */
    SRV_MOTOR_INDEX_INFO_03_R    = 12, /**< 读：速度PI/位置PD参数 */
    SRV_MOTOR_INDEX_SAVE_FLASH   = 13, /**< 写：参数保存到 FLASH (KEY=0xA5) */
    SRV_MOTOR_INDEX_READ_FLASH   = 14, /**< 写：参数从 FLASH 读取 */
    SRV_MOTOR_INDEX_BUAD_RATE    = 15, /**< 写：波特率设置 (0=500k,1=230400…) */
    SRV_MOTOR_INDEX_POS_LPF_LV   = 16, /**< 写：位置平滑力度 0~5 */
    SRV_MOTOR_INDEX_INFO_04_R    = 17, /**< 读：平滑力度/波特率 */
    SRV_MOTOR_INDEX_MAX_CUR      = 18, /**< 写：最大输出电流 (Q15) */
    SRV_MOTOR_INDEX_PROTECT_CUR  = 19, /**< 写：保护限制电流 (Q15) */
    SRV_MOTOR_INDEX_PROTECT_LV1  = 20, /**< 写：电流保护等级1阈值 (Q15) */
    SRV_MOTOR_INDEX_CUR_LV1_DELAY = 21,/**< 写：保护等级1持续时间 (ms) */
    SRV_MOTOR_INDEX_RECV_CUR     = 22, /**< 写：电流恢复阈值 (Q15) */
    SRV_MOTOR_INDEX_CUR_RECV_WIN = 23, /**< 写：恢复判定时间窗口 (ms) */
    SRV_MOTOR_INDEX_INFO_05_R    = 24, /**< 读：保护等级1/恢复时间 */
    SRV_MOTOR_INDEX_INFO_06_R    = 25, /**< 读：瞬爆冷却/瞬爆时长 */
    SRV_MOTOR_INDEX_COLD_TIME    = 26, /**< 写：瞬间爆发冷却时长 (×25ms) */
    SRV_MOTOR_INDEX_PLUS_TIME    = 27, /**< 写：瞬间爆发时长 */
    SRV_MOTOR_INDEX_DRV_OT      = 28,  /**< 写：驱动器温度保护阈值 (°C) */
    SRV_MOTOR_INDEX_SPD_LIMIT    = 30, /**< 写：电机转子最大速度限制 (Q15) */
    SRV_MOTOR_INDEX_SN           = 31, /**< 写：SN 码 */
    SRV_MOTOR_INDEX_INFO_07_R    = 32, /**< 读：SN 码 */
} srv_motor_config_index_t;

/**
 * @brief 使能命令
 */
typedef enum {
    SRV_MOTOR_CMD_NONE = 0, /**< 无操作 */
    SRV_MOTOR_CMD_ENABLE = 1, /**< 使能上电 */
    SRV_MOTOR_CMD_DISABLE = 2, /**< 去使能释放 */
    SRV_MOTOR_CMD_SET_REF = 3, /**< 仅下发参考值 */
} srv_motor_cmd_t;

/**
 * @brief 控制模式（CONFIG_CTRL_MODE_SW, index=7）
 */
typedef enum {
    SRV_MOTOR_MODE_NOOP    = 0, /**< 无操作 (NOP) */
    SRV_MOTOR_MODE_SPEED   = 1, /**< 速度模式 */
    SRV_MOTOR_MODE_POSITION = 2, /**< 位置模式 */
    SRV_MOTOR_MODE_CURRENT = 3, /**< 电流模式 */
} srv_motor_ctrl_mode_t;

/** @brief 轮询超时 */
#define SRV_MOTOR_POLL_TIMEOUT_MS (10U) /**< 等待电机回复超时 (ms) */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 电机服务错误码
 */
typedef enum {
    SRV_MOTOR_OK = 0, /**< 操作成功 */
    SRV_MOTOR_ERROR_NULL_PTR = -1, /**< 空指针 */
    SRV_MOTOR_ERROR_UNINITIALIZED = -2, /**< 未初始化 */
    SRV_MOTOR_ERROR_INVALID_PARAM = -3, /**< 无效参数 */
    SRV_MOTOR_ERROR_NOT_FOUND = -4, /**< 未找到实例 */
    SRV_MOTOR_ERROR_ALREADY_EXIST = -5, /**< 同名实例已存在 */
    SRV_MOTOR_ERROR_TX_BUSY = -6, /**< 发送队列满 */
} srv_motor_error_t;

/**
 * @brief 电机运行状态（对应电机端状态反馈 fsm）
 */
typedef enum {
    SRV_MOTOR_FSM_READY = 0, /**< mcReady — 就绪 */
    SRV_MOTOR_FSM_ALIGN = 5, /**< mcAlign — 预定位中 */
    SRV_MOTOR_FSM_RUN = 7, /**< mcRun   — 运行中 */
    SRV_MOTOR_FSM_FAULT = 9, /**< mcFault — 故障 */
} srv_motor_fsm_t;

/**
 * @brief 电机故障码
 */
typedef enum {
    SRV_MOTOR_ERR_NONE = 0, /**< 无故障 */
    SRV_MOTOR_ERR_HW_OVERCURRENT = 1, /**< 硬件过流 */
    SRV_MOTOR_ERR_SW_OVERCURRENT = 2, /**< 软件过流 */
    SRV_MOTOR_ERR_HW_OVERVOLTAGE = 3, /**< 硬件过压 */
    SRV_MOTOR_ERR_SW_OVERVOLTAGE = 4, /**< 软件过压 */
    SRV_MOTOR_ERR_HW_UNDERVOLTAGE = 5, /**< 硬件欠压 */
    SRV_MOTOR_ERR_SW_UNDERVOLTAGE = 6, /**< 软件欠压 */
    SRV_MOTOR_ERR_PHASE_LOSS = 7, /**< 缺相 */
    SRV_MOTOR_ERR_STALL = 8, /**< 电机堵转 */
    SRV_MOTOR_ERR_OVERTEMP = 9, /**< 过温保护 */
    SRV_MOTOR_ERR_COMM_LOST = 10, /**< 串口通信丢失 */
    SRV_MOTOR_ERR_INTERNAL = 11, /**< 内部自检故障 */
} srv_motor_err_t;

/**
 * @brief 电机反馈数据
 *
 * 由 srv_motor_step 内部在解析回复时更新，外部通过 srv_motor_get_feedback() 只读获取。
 */
typedef struct {
    /* — CONFIG_INFO_02_R（高频：角度/速度/DQ电流） — */
    int16_t angle_fb; /**< Q7 末端关节角度 (-255°~+255°) */
    int16_t speed_fb; /**< Q15 速度反馈 (0~32767 → 0~50 krpm) */
    int16_t q_cur; /**< Q15 Q轴电流 (0~32767 → 0~5.625 A) */
    int16_t d_cur; /**< Q15 D轴电流 (0~32767 → 0~5.625 A) */
    uint32_t time_fb; /**< 反馈更新时间戳 (millis) */

    /* — CONFIG_INFO_01_R（低频：状态/故障/温度/电压） — */
    srv_motor_fsm_t fsm_state; /**< 电机状态机 (mcReady/mcAlign/mcRun/mcFault) */
    srv_motor_err_t err_code; /**< 故障码 */
    int8_t temp; /**< MCU 温度 (物理°C = temp - 50) */
    int16_t vbus; /**< 母线电压 (Vbus / 128 = 电压 V) */
    uint32_t time_status; /**< 状态更新时间戳 (millis) */
} srv_motor_feedback_t;

/**
 * @brief 电机实例句柄前向声明
 */
typedef struct srv_motor_handle srv_motor_handle_t;

/**
 * @brief 电机实例句柄
 *
 * 外部静态分配，通过 srv_motor_register() 注册到服务中。
 * 所有字段在注册时初始化，外部仅通过 API 接口操作。
 */
struct srv_motor_handle {
    uint8_t dev_id; /**< 设备 ID (1-based) */
    drv_uart_channel_t uart_ch; /**< UART 通道 (DRV_UART_CH_1 / CH_2) */

    /* 控制设定 （外部写入，step 读取下发） */
    int16_t pos_ref; /**< Q7 目标角度 */
    int16_t spd_ref; /**< Q15 目标速度 / 位置模式限速 */
    int16_t cur_ref; /**< Q15 电流限制 */
    uint8_t en_cmd; /**< 使能命令 (CMD_NONE / ENABLE / DISABLE) */

    /* 反馈缓存 */
    srv_motor_feedback_t fb; /**< 最新反馈数据 */

    /* 轮询状态（内部使用） */
    uint8_t query_index; /**< 当前查询的 index */
    bool cur_pending; /**< 电流限制待发送 */
    srv_motor_ctrl_mode_t ctrl_mode; /**< 控制模式 (SRV_MOTOR_MODE_xxx) */
    bool initialized; /**< 初始化标志 */
};

/* Exported functions prototypes ---------------------------------------------*/

/* --- 生命周期 --- */

/**
 * @brief 初始化电机服务
 * @return 操作结果
 */
srv_motor_error_t srv_motor_init(void);

/**
 * @brief 反初始化电机服务
 */
void srv_motor_deinit(void);

/* --- 电机注册 --- */

/**
 * @brief 注册一个电机实例
 * @param inst   电机句柄指针（调用者静态分配）
 * @param dev_id 设备 ID (1-based)
 * @param uart_ch UART 通道 (DRV_UART_CH_2 或 DRV_UART_CH_3)
 * @return 操作结果
 */
srv_motor_error_t srv_motor_register(srv_motor_handle_t* inst,
    uint8_t dev_id, uint8_t uart_idx);

/* --- 控制接口 --- */

/**
 * @brief 设置电机控制目标（缓存，下个广播周期自动下发）
 * @param inst    电机句柄
 * @param pos_ref Q7 目标角度
 * @param spd_ref Q15 目标速度 / 位置模式限速
 * @param cur_ref Q15 电流限制
 */
void srv_motor_set_setpoint(srv_motor_handle_t* inst,
    int16_t pos_ref, int16_t spd_ref, int16_t cur_ref);

/**
 * @brief 使能或去使能电机
 * @param inst 电机句柄
 * @param en   true=使能, false=去使能
 */
void srv_motor_enable(srv_motor_handle_t* inst, bool en);

/**
 * @brief 设置电流限制（低频，变化时通过标准帧下发）
 * @param inst    电机句柄
 * @param cur_ref Q15 电流限制 (0~32767 → 0~5.625A)
 */
void srv_motor_set_current(srv_motor_handle_t* inst, int16_t cur_ref);

/** @brief 按全局索引获取电机句柄 (0-8, 0-4=USART2, 5-8=USART3) */
srv_motor_handle_t* srv_motor_get_handle(uint32_t index);

/* --- 模式切换 --- */

/**
 * @brief 设置电机控制模式（配置 index=7，CONFIG_CTRL_MODE_SW）
 * @param inst 电机句柄
 * @param mode 控制模式 (SRV_MOTOR_MODE_xxx)
 * @note  下个 FSM 周期 BCAST_EN 自动发送 SET_MODE 配置帧
 */
void srv_motor_set_mode(srv_motor_handle_t* inst, srv_motor_ctrl_mode_t mode);

/* --- 零点校准 --- */

/**
 * @brief 重置当前末端位置为零点（CONFIG_UPDATE_FIN_ZERO, index=3）
 * @param inst 电机句柄
 * @note  立即发送配置帧，电机无回复。执行后当前物理位置被记为 0°
 */
void srv_motor_zero_reset(srv_motor_handle_t* inst);

/**
 * @brief 批量归零 — 同一 UART 通道上的所有电机（dev_id=0xFF 广播）
 * @param ch  UART 通道
 * @note  等待 DMA 空闲后发送，仅影响该串口上的电机
 */
void srv_motor_zero_reset_all(drv_uart_channel_t ch);

/* --- 反馈接口 --- */

/**
 * @brief 获取电机反馈只读指针
 * @param  inst 电机句柄
 * @return 反馈数据指针（NULL=无效）
 */
const srv_motor_feedback_t* srv_motor_get_feedback(const srv_motor_handle_t* inst);

/**
 * @brief 检查电机是否在线
 * @param  inst 电机句柄
 * @return true=最近有有效反馈
 */
bool srv_motor_is_online(const srv_motor_handle_t* inst);

/* --- 周期驱动（由 motor_task 的 sw_timer 调用） --- */

/**
 * @brief 核心步进函数
 *
 * 需以 ~3 ms 周期调用。每次执行：
 * 1. 为每组构建并发送位置+速度广播帧
 * 2. 轮询下个电机的反馈
 * 3. 解析接收到的回复数据
 */
void srv_motor_step(void);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_MOTOR_H */
