/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    m_behavior_task.h
 * @brief   电机行为控制任务 — 电机状态 FSM + 故障监控 + 动作控制
 *
 * 职责：管理每个电机的生命周期（INIT→OFFLINE→IDLE→ENABLING→RUNNING→FAULT），
 *       基于电机反馈驱动 FSM 状态迁移，仅在 RUNNING 状态下允许下发控制命令。
 *       底层实时通信由 motor_task / srv_motor 处理。
 */

#ifndef M_BEHAVIOR_TASK_H
#define M_BEHAVIOR_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "srv_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 行为 FSM 状态
 */
typedef enum {
    M_BHV_STATE_INIT = 0, /**< 初始，等待首次反馈 */
    M_BHV_STATE_OFFLINE = 1, /**< 离线，通信超时 */
    M_BHV_STATE_IDLE = 2, /**< 在线，已就绪 (mcReady) */
    M_BHV_STATE_ENABLING = 3, /**< 使能中，等待校准完成 */
    M_BHV_STATE_RUNNING = 4, /**< 正常运行 (mcRun) */
    M_BHV_STATE_FAULT = 5, /**< 故障，自动去使能 */
} m_behavior_state_t;

/**
 * @brief 行为控制错误码
 */
typedef enum {
    M_BHV_OK = 0,
    M_BHV_ERROR_INVALID_STATE = -1, /**< 当前状态下不允许操作 */
    M_BHV_ERROR_INVALID_PARAM = -2, /**< 参数无效 */
    M_BHV_ERROR_FAULT = -3, /**< 电机故障 */
} m_behavior_error_t;

/* Exported functions prototypes ---------------------------------------------*/

/** @brief 初始化行为控制任务（10ms 周期） */
void m_behavior_task_init(void);

/**
 * @brief 使能/去使能指定电机
 * @param index 电机索引 (0-8)
 * @param en    true=使能, false=去使能
 * @return M_BHV_OK 或错误码
 * @note 使能仅在 IDLE 状态有效；去使能在 RUNNING/FAULT 状态有效
 */
m_behavior_error_t m_behavior_enable(uint32_t index, bool en);

/**
 * @brief 设置指定电机控制目标
 * @param index   电机索引 (0-8)
 * @param pos_ref Q7 目标角度
 * @param spd_ref Q15 目标速度 / 位置模式限速
 * @param cur_ref Q15 电流限制
 * @return M_BHV_OK 或错误码（非 RUNNING 状态返回 INVALID_STATE）
 */
m_behavior_error_t m_behavior_set_setpoint(uint32_t index,
    int16_t pos_ref, int16_t spd_ref, int16_t cur_ref);

/* --- 电流限制（低频配置，变化时通过标准帧轮询下发） --- */

m_behavior_error_t m_behavior_set_current(uint32_t index, int16_t cur_ref);

/* --- 批量操作 --- */

void m_behavior_enable_all(bool en);
void m_behavior_set_all_setpoint(int16_t pos_ref, int16_t spd_ref, int16_t cur_ref);

/* --- 查询接口 --- */

/** @brief 获取电机行为状态 */
m_behavior_state_t m_behavior_get_state(uint32_t index);

/** @brief 检查电机是否正常运行中 */
bool m_behavior_is_running(uint32_t index);

/** @brief 获取电机反馈只读指针 */
const srv_motor_feedback_t* m_behavior_get_fb(uint32_t index);

#ifdef __cplusplus
}
#endif

#endif /* M_BEHAVIOR_TASK_H */
