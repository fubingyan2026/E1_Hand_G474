/**
 * @file    srv_motor_behavior.h
 * @brief   电机行为协调服务 — 单 fsm_t 管理 9 电机整体行为
 *
 * 依赖 srv_motor 的反馈和控制接口，管理电机组生命周期。
 * s_fsm 状态 = 组整体状态：OFFLINE→IDLE→CALIB→RUNNING，任意态→FAULT（自动去使能）。
 * 上电首次进入 CALIB 到位找零（180° 小电流走到目标位置 → 设零），之后 IDLE⇄RUNNING。
 * 在线判定来自 daemon 看门狗（按 srv_motor_get_name 注册名查询）。
 * 链路层使能/校准/模式序列由 srv_motor 的 STARTUP FSM 自治完成。
 */

#ifndef __SRV_MOTOR_BEHAVIOR_H
#define __SRV_MOTOR_BEHAVIOR_H

#include <stdbool.h>
#include <stdint.h>

#include "srv_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 行为状态枚举 --------------------------------------------------------------*/

/**
 * @brief 行为状态 — 数值为 CAN 协议契约 (can_protocol.md §4.2)，禁止改动
 * @note  INIT 为保留值（协议兼容），FSM 不再使用/上报
 */
typedef enum {
    SRV_MOTOR_BHV_INIT = 0, /**< 保留值（协议兼容），FSM 不再使用 */
    SRV_MOTOR_BHV_OFFLINE = 1, /**< 离线，全部电机通信超时（daemon 判定） */
    SRV_MOTOR_BHV_IDLE = 2, /**< 在线，已就绪 (mcReady) */
    SRV_MOTOR_BHV_CALIB = 3, /**< 零点校准中（到位找零，上电首次自动执行） */
    SRV_MOTOR_BHV_RUNNING = 4, /**< 正常运行（任一电机 mcRun） */
    SRV_MOTOR_BHV_FAULT = 5, /**< 故障，自动去使能 */
} srv_motor_behavior_state_t;

typedef enum {
    SRV_MOTOR_BHV_OK = 0,
    SRV_MOTOR_BHV_ERROR_INVALID_STATE = -1,
    SRV_MOTOR_BHV_ERROR_INVALID_PARAM = -2,
} srv_motor_behavior_error_t;

/* API -----------------------------------------------------------------------*/

/** @brief 初始化行为服务（注册电机 + 构建 FSM） */
void srv_motor_behavior_init(void);

/** @brief 核心步进（由 behavior_task 的 sw_timer 周期调用） */
void srv_motor_behavior_step(void);

/** @brief 设置控制目标 — 仅 RUNNING 态响应，其余状态返回 INVALID_STATE */
srv_motor_behavior_error_t srv_motor_behavior_set_setpoint(uint32_t index,
    int16_t pos_ref, int16_t spd_ref, int16_t cur_ref);

/** @brief 批量设置控制目标 — 仅 RUNNING 态响应 */
void srv_motor_behavior_set_all_setpoint(int16_t pos_ref, int16_t spd_ref, int16_t cur_ref);

/* 查询 */
srv_motor_behavior_state_t srv_motor_behavior_get_state(void);
const srv_motor_feedback_t* srv_motor_behavior_get_fb(uint32_t index);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_MOTOR_BEHAVIOR_H */
