/**
 * @file    drv_can.h
 * @author  maximillian
 * @version V2.2.0
 * @date    2026-07-09
 * @brief   CAN 设备驱动（STM32G474 FDCAN，支持经典 CAN 和 CAN FD）
 * @attention
 *
 * CubeMX 配置 FDCAN1 (PA11 RX / PA12 TX)。支持经典 CAN 和 CAN FD 帧格式。
 * drv_can_msg_t.dlc 始终为实际字节数（0-64），driver 内部自动与 FDCAN DLC 编码互转。
 */

#ifndef __DRV_CAN_H
#define __DRV_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief CAN 通道枚举
 */
typedef enum {
    DRV_CAN_CH_1 = 0, /**< FDCAN1 — PA11(RX) / PA12(TX) */
    DRV_CAN_CH_NUM,   /**< 通道总数 */
} drv_can_channel_t;

/**
 * @brief 驱动错误码
 */
typedef enum {
    DRV_CAN_OK = 0,
    DRV_CAN_ERROR_NULL_PTR,
    DRV_CAN_ERROR_UNINITIALIZED,
    DRV_CAN_ERROR_TX_BUSY,
    DRV_CAN_ERROR_INVALID_PARAM,
} drv_can_error_t;

/**
 * @brief CAN 报文结构
 *
 * 同时支持经典 CAN 和 CAN FD。
 * dlc 始终为实际字节数（0~64），非 FDCAN DLC 编码值。
 */
typedef struct {
    uint32_t id;          /**< CAN ID（标准 11-bit 或扩展 29-bit） */
    bool     is_extended; /**< true=扩展帧 */
    bool     is_fd;       /**< true=CAN FD 帧 */
    uint8_t  dlc;         /**< 数据长度（字节数，0-64） */
    uint8_t  data[64];    /**< 数据负载 */
} drv_can_msg_t;

/** @brief CAN 接收回调函数类型（中断上下文执行） */
typedef void (*drv_can_rx_callback_t)(drv_can_channel_t ch, const drv_can_msg_t* msg);

/* Exported functions prototypes ---------------------------------------------*/

/* --- 初始化 / 生命周期 --- */

/** @brief 初始化全部 CAN 通道（内部句柄表，无需传参） */
drv_can_error_t drv_can_init(void);

/** @brief 反初始化全部 CAN 通道 */
void drv_can_deinit_all(void);

bool drv_can_is_initialized(drv_can_channel_t ch);

/* --- 发送 --- */

/**
 * @brief 发送 CAN 报文（非阻塞）
 * @return DRV_CAN_ERROR_TX_BUSY 表示 Tx FIFO 满
 */
drv_can_error_t drv_can_send(drv_can_channel_t ch, const drv_can_msg_t* msg);

/**
 * @brief 查询 Tx FIFO 是否有空闲位置
 */
bool drv_can_tx_ready(drv_can_channel_t ch);

/* --- 状态监控 --- */

/**
 * @brief 轮询总线协议状态（主循环/任务周期调用，建议 ≥10ms 一次）
 * @param ch 通道号
 * @note  Bus-Off 时打印告警并自动触发恢复序列；
 *        Error-Passive 状态跳变时打印告警（无 ACK / 位错误累积的早期信号）
 */
void drv_can_poll_status(drv_can_channel_t ch);

/* --- 接收回调 --- */

/**
 * @brief 注册接收回调（每通道独立注册）
 * @param ch       通道号
 * @param callback 回调函数（NULL=取消）
 * @note  回调在中断上下文中执行，应尽量简短
 */
drv_can_error_t drv_can_register_rx_callback(drv_can_channel_t ch,
    drv_can_rx_callback_t callback);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_CAN_H */
