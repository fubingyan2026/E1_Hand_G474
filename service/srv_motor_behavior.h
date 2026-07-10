/**
 * @file    srv_motor_behavior.h
 * @brief   电机行为协调服务 — 单 fsm_t 管理 9 电机整体行为
 *
 * 依赖 srv_motor 的反馈和控制接口，管理电机组生命周期。
 * s_fsm 状态 = 组整体状态：INIT→OFFLINE→IDLE→ENABLING→RUNNING⇄FAULT
 * 任一电机满足条件即触发状态迁移，使能/控制通过广播帧统一下发。
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

typedef enum {
    SRV_MOTOR_BHV_INIT     = 0, /**< 初始，等待首次反馈 */
    SRV_MOTOR_BHV_OFFLINE  = 1, /**< 离线，通信超时 */
    SRV_MOTOR_BHV_IDLE     = 2, /**< 在线，已就绪 (mcReady) */
    SRV_MOTOR_BHV_ENABLING = 3, /**< 使能中，等待校准完成 */
    SRV_MOTOR_BHV_RUNNING  = 4, /**< 正常运行 (mcRun) */
    SRV_MOTOR_BHV_FAULT    = 5, /**< 故障，自动去使能 */
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

/** @brief 使能/去使能（按 group，在 IDLE 态可使能） */
srv_motor_behavior_error_t srv_motor_behavior_enable(uint32_t index, bool en);

/** @brief 设置控制目标 + 电流限制 */
srv_motor_behavior_error_t srv_motor_behavior_set_setpoint(uint32_t index,
    int16_t pos_ref, int16_t spd_ref, int16_t cur_ref);

/* 批量操作 */
void srv_motor_behavior_enable_all(bool en);
void srv_motor_behavior_set_all_setpoint(int16_t pos_ref, int16_t spd_ref, int16_t cur_ref);

/* 查询 */
srv_motor_behavior_state_t srv_motor_behavior_get_state(void);
bool srv_motor_behavior_is_running(void);
const srv_motor_feedback_t* srv_motor_behavior_get_fb(uint32_t index);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_MOTOR_BEHAVIOR_H */
