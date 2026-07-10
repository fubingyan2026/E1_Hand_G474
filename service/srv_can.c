/**
 * @file    srv_can.c
 * @brief   CAN FD 电机控制协议服务实现
 *
 * 控制帧解析 → srv_motor_behavior API。
 * 反馈帧打包 → drv_can_send (CAN FD, 64B max)。
 */

#include "srv_can.h"

#include "srv_motor_behavior.h"

#include <string.h>

/* Private variables ---------------------------------------------------------*/

/** @brief ISR → 主循环控制数据暂存 */
static struct {
    bool    pending;
    uint8_t ctrl;
    int16_t pos[9];
    int16_t spd[9];
    int16_t cur[9];
} s_rx;

/* Exported functions --------------------------------------------------------*/

void srv_can_init(void)
{
    memset(&s_rx, 0, sizeof(s_rx));
}

/**
 * @brief ISR 回调：快速复制数据，置标志位
 */
void srv_can_on_rx(const drv_can_msg_t* msg)
{
    if (!msg || msg->id != SRV_CAN_ID_CTRL || msg->dlc < SRV_CAN_CTRL_LEN)
        return;

    s_rx.ctrl = msg->data[0];
    memcpy(s_rx.pos, &msg->data[1],  18);
    memcpy(s_rx.spd, &msg->data[19], 18);
    memcpy(s_rx.cur, &msg->data[37], 18);
    s_rx.pending = true;
}

/**
 * @brief 主循环处理：将缓存的控制数据下发到电机行为层
 */
void srv_can_process(void)
{
    if (!s_rx.pending) return;
    s_rx.pending = false;

    if (s_rx.ctrl & 0x01)
        srv_motor_behavior_enable_all(true);
    else if (s_rx.ctrl & 0x02)
        srv_motor_behavior_enable_all(false);

    for (uint32_t i = 0; i < SRV_MOTOR_TOTAL; i++)
        srv_motor_behavior_set_setpoint(i, s_rx.pos[i], s_rx.spd[i], s_rx.cur[i]);
}

/**
 * @brief 打包并发送 9 电机反馈数据 (CAN FD)
 */
void srv_can_send_feedback(void)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_1)) return;

    uint8_t buf[SRV_CAN_FB_LEN];
    memset(buf, 0, sizeof(buf));

    buf[0] = (uint8_t)srv_motor_behavior_get_state();

    for (uint32_t i = 0; i < SRV_MOTOR_TOTAL; i++) {
        const srv_motor_feedback_t* fb = srv_motor_behavior_get_fb(i);

        buf[1 + i] = fb ? (uint8_t)fb->fsm_state : 0;

        if (fb) {
            buf[10 + i * 2]     = (uint8_t)fb->angle_fb;
            buf[10 + i * 2 + 1] = (uint8_t)(fb->angle_fb >> 8);
            buf[28 + i * 2]     = (uint8_t)fb->speed_fb;
            buf[28 + i * 2 + 1] = (uint8_t)(fb->speed_fb >> 8);
            buf[46 + i * 2]     = (uint8_t)fb->q_cur;
            buf[46 + i * 2 + 1] = (uint8_t)(fb->q_cur >> 8);
        }
    }

    drv_can_msg_t tx = {
        .id = SRV_CAN_ID_FEEDBACK, .is_extended = false,
        .is_fd = true, .dlc = SRV_CAN_FB_LEN,
    };
    memcpy(tx.data, buf, SRV_CAN_FB_LEN);
    drv_can_send(DRV_CAN_CH_1, &tx);
}

/**
 * @brief 打包并发送 9 电机低频状态数据 (CAN FD, 500ms)
 */
void srv_can_send_status(void)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_1)) return;

    uint8_t buf[SRV_CAN_STATUS_LEN];
    memset(buf, 0, sizeof(buf));

    buf[0] = (uint8_t)srv_motor_behavior_get_state();

    for (uint32_t i = 0; i < SRV_MOTOR_TOTAL; i++) {
        const srv_motor_feedback_t* fb = srv_motor_behavior_get_fb(i);

        if (fb) {
            buf[1  + i]      = (uint8_t)fb->fsm_state;
            buf[10 + i]      = (uint8_t)fb->err_code;
            buf[19 + i]      = (uint8_t)fb->temp;
            buf[28 + i * 2]     = (uint8_t)fb->vbus;
            buf[28 + i * 2 + 1] = (uint8_t)(fb->vbus >> 8);
        }
    }

    drv_can_msg_t tx = {
        .id = SRV_CAN_ID_STATUS, .is_extended = false,
        .is_fd = true, .dlc = SRV_CAN_STATUS_LEN,
    };
    memcpy(tx.data, buf, SRV_CAN_STATUS_LEN);
    drv_can_send(DRV_CAN_CH_1, &tx);
}
