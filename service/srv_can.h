/**
 * @file    srv_can.h
 * @brief   CAN FD 电机控制协议服务 — 控制帧解析 + 反馈帧打包
 *
 * 控制帧 (Host→Device, CAN ID 0x100):
 *   [0]      ctrl: 0x01=enable, 0x02=disable
 *   [1-18]   pos_ref[9]  (int16_t LE, Q7)
 *   [19-36]  spd_ref[9]  (int16_t LE, Q15)
 *   [37-54]  cur_ref[9]  (int16_t LE, Q15)
 *   [55-63]  reserved
 *
 * 反馈帧 (Device→Host, CAN ID 0x101, 100ms):
 *   [0]      global_state
 *   [1-9]    fsm_state[9]
 *   [10-27]  angle_fb[9] (int16_t LE, Q7)
 *   [28-45]  speed_fb[9] (int16_t LE, Q15)
 *   [46-63]  q_cur[9]    (int16_t LE, Q15)
 *
 * 状态帧 (Device→Host, CAN ID 0x102, 500ms):
 *   [0]      global_state
 *   [1-9]    fsm_state[9]
 *   [10-18]  err_code[9]
 *   [19-27]  temp[9]      (int8_t, °C = raw - 50)
 *   [28-45]  vbus[9]      (int16_t LE, V = raw / 128)
 *   [46-47]  reserved
 */

#ifndef __SRV_CAN_H
#define __SRV_CAN_H

#include <stdbool.h>
#include <stdint.h>

#include "drv_can.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief CAN ID 定义 */
#define SRV_CAN_ID_CTRL      0x100  /**< 控制帧 (Host→Device) */
#define SRV_CAN_ID_FEEDBACK  0x101  /**< 高频反馈 (角度/速度/Q电流) */
#define SRV_CAN_ID_STATUS    0x102  /**< 低频状态 (错误/温度/电压) */

/** @brief 帧长度（CAN FD DLC 编码: 0-8,12,16,20,24,32,48,64） */
#define SRV_CAN_CTRL_LEN     64U   /**< 55B 数据 + 9B 保留 */
#define SRV_CAN_FB_LEN       64U   /**< 64B 恰好填满 */
#define SRV_CAN_STATUS_LEN   48U   /**< 46B 数据 + 2B 保留 */

/* API -----------------------------------------------------------------------*/

/** @brief 初始化 CAN 协议服务 */
void srv_can_init(void);

/**
 * @brief 处理收到的 CAN 帧（由 ISR 回调调用）
 * @param msg CAN 报文指针
 * @note  仅缓存数据，不直接调用电机 API（ISR 安全）
 */
void srv_can_on_rx(const drv_can_msg_t* msg);

/**
 * @brief 处理缓存的控制帧并下发到电机行为层
 * @note  由 can_task 的 sw_timer 在主循环调用
 */
void srv_can_process(void);

/**
 * @brief 构建并发送反馈帧
 * @note  由 can_task 的 sw_timer 周期调用
 */
void srv_can_send_feedback(void);

/**
 * @brief 构建并发送低频状态帧 (fsm_state/err_code/temp/vbus)
 * @note  由 can_task 低频周期调用
 */
void srv_can_send_status(void);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_CAN_H */
