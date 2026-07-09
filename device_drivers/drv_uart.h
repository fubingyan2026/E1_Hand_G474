/**
 * @file    drv_uart.h
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-07-09
 * @brief   通用串口设备驱动（USART1/2/3，DMA 输出 + DMA circular + IDLE 中断接收）
 * @attention
 *
 * 多实例驱动：内部句柄表，drv_uart_init() 无需传参。
 * TX：DMA normal，gState + s_tx_busy 双状态检忙。
 * RX：DMA circular + IDLE 中断 → kfifo，无逐字节拷贝。
 *
 * HAL 回调 (HAL_UART_TxCpltCallback / HAL_UARTEx_RxEventCallback) 集中在此驱动中。
 */

#ifndef __DRV_UART_H
#define __DRV_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief UART 通道枚举
 */
typedef enum {
    DRV_UART_CH_1 = 0, /**< USART1 — PC4(TX) / PC5(RX), 115200 baud（日志串口） */
    DRV_UART_CH_2,     /**< USART2 — PA2(TX) / PA3(RX), 500000 baud */
    DRV_UART_CH_3,     /**< USART3 — PB10(TX) / PB11(RX), 500000 baud */
    DRV_UART_CH_NUM,   /**< 通道总数 */
} drv_uart_channel_t;

/**
 * @brief UART 驱动错误码
 */
typedef enum {
    DRV_UART_OK = 0,                /**< 操作成功 */
    DRV_UART_ERROR_NULL_PTR,        /**< 空指针错误 */
    DRV_UART_ERROR_UNINITIALIZED,   /**< 未初始化 */
    DRV_UART_ERROR_TX_BUSY,         /**< TX DMA 忙 */
    DRV_UART_ERROR_INVALID_PARAM,   /**< 无效参数 */
} drv_uart_error_t;

/* Exported functions prototypes ---------------------------------------------*/

/* --- 初始化 / 生命周期 --- */

/** @brief 初始化所有 UART 通道（USART1/2/3） */
drv_uart_error_t drv_uart_init(void);

/** @brief 反初始化所有 UART 通道 */
void drv_uart_deinit_all(void);

/** @brief 检查指定通道是否已初始化 */
bool drv_uart_is_initialized(drv_uart_channel_t ch);

/* --- TX --- */

/**
 * @brief 非阻塞 DMA 发送
 * @param ch   通道号
 * @param data 数据指针
 * @param len  数据长度
 * @return 操作结果错误码
 */
drv_uart_error_t drv_uart_send(drv_uart_channel_t ch, const uint8_t* data, uint32_t len);

/** @brief 查询 TX DMA 是否忙碌 */
bool drv_uart_is_tx_busy(drv_uart_channel_t ch);

/* --- RX（DMA circular + IDLE 中断 → kfifo） --- */

/**
 * @brief 从接收 FIFO 读取数据
 * @param ch      通道号
 * @param buf     目标缓冲区
 * @param max_len 最大读取字节数
 * @return 实际读取的字节数
 */
uint32_t drv_uart_rx_read(drv_uart_channel_t ch, uint8_t* buf, uint32_t max_len);

/** @brief 查询接收 FIFO 中可读字节数 */
uint32_t drv_uart_rx_available(drv_uart_channel_t ch);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_UART_H */
