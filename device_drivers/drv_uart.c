/**
 * @file    drv_uart.c
 * @author  maximillian
 * @version V2.1.0
 * @date    2026-07-14
 * @brief   通用串口设备驱动实现（USART1/2/3，DMA 输出 + DMA 正常模式接收）
 * @attention
 *
 * 多实例驱动：内部句柄 + DMA 配置表，drv_uart_init() 无需传参。
 * TX：DMA normal，gState + s_tx_busy 双状态检忙。
 * RX：DMA normal 模式，单次接收 rx_dma_buf_size 字节，完成后自动重启。
 *
 * HAL 回调 (HAL_UART_TxCpltCallback / HAL_UART_RxCpltCallback) 集中在此文件。
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_uart.h"

#include "usart.h"

#include <string.h>

/* Private constants ---------------------------------------------------------*/

/** @brief RX DMA 单次接收字节数（256 = ~11ms，减少重启频次） */
#define DRV_UART_RX_BUF_SIZE (256U)

/** @brief TX 最大单次发送量 */
#define DRV_UART_TX_BUF_SIZE (256U)

/* Private types -------------------------------------------------------------*/

typedef struct {
    UART_HandleTypeDef* huart;          /**< HAL UART 句柄 */
    uint8_t             rx_buf[DRV_UART_RX_BUF_SIZE]; /**< RX DMA 接收缓冲 */
    uint8_t             tx_buf[DRV_UART_TX_BUF_SIZE]; /**< TX DMA 发送缓冲 */
    bool                tx_busy;        /**< TX DMA 传输中 */
    bool                rx_need_restart; /**< IDLE 后主循环重启 RX DMA */
    bool                initialized;    /**< 初始化标志 */
    drv_uart_rx_callback_t rx_callback; /**< 接收回调（中断中执行） */
} drv_uart_inst_t;

/* Private variables ---------------------------------------------------------*/

static drv_uart_inst_t s_inst[DRV_UART_CH_NUM] = {
    [DRV_UART_CH_1] = { .huart = &huart1 },
    [DRV_UART_CH_2] = { .huart = &huart2 },
    [DRV_UART_CH_3] = { .huart = &huart3 },
};

/* Exported functions --------------------------------------------------------*/

/* --- 初始化 / 生命周期 --- */

drv_uart_error_t drv_uart_init(void)
{
    for (uint32_t ch = 0; ch < DRV_UART_CH_NUM; ch++) {
        drv_uart_inst_t* inst = &s_inst[ch];

        inst->tx_busy = false;
        inst->initialized = false;

        /* 启动 DMA normal 模式接收 */
        if (HAL_UART_Receive_DMA(inst->huart, inst->rx_buf, sizeof(inst->rx_buf)) != HAL_OK) {
            return DRV_UART_ERROR_UNINITIALIZED;
        }

        inst->initialized = true;
    }

    return DRV_UART_OK;
}

void drv_uart_deinit_all(void)
{
    for (uint32_t ch = 0; ch < DRV_UART_CH_NUM; ch++) {
        drv_uart_inst_t* inst = &s_inst[ch];
        if (!inst->initialized) {
            continue;
        }

        HAL_UART_DMAStop(inst->huart);

        inst->tx_busy = false;
        inst->initialized = false;
    }
}

bool drv_uart_is_initialized(drv_uart_channel_t ch)
{
    if (ch >= DRV_UART_CH_NUM) {
        return false;
    }
    return s_inst[ch].initialized;
}

/* --- TX --- */

drv_uart_error_t drv_uart_send(drv_uart_channel_t ch, const uint8_t* data, uint32_t len)
{
    if (ch >= DRV_UART_CH_NUM || !data) {
        return DRV_UART_ERROR_NULL_PTR;
    }
    if (!s_inst[ch].initialized) {
        return DRV_UART_ERROR_UNINITIALIZED;
    }
    if (len == 0 || len > UINT16_MAX) {
        return DRV_UART_OK;
    }

    drv_uart_inst_t* inst = &s_inst[ch];

    if (inst->tx_busy || inst->huart->gState != HAL_UART_STATE_READY) {
        return DRV_UART_ERROR_TX_BUSY;
    }

    if (len > DRV_UART_TX_BUF_SIZE) {
        return DRV_UART_ERROR_INVALID_PARAM;
    }

    /* 拷贝到内部持久缓冲区再启动 DMA，防止调用者栈回收后 DMA 读脏数据 */
    memcpy(inst->tx_buf, data, len);

    if (HAL_UART_Transmit_DMA(inst->huart, inst->tx_buf, (uint16_t)len) != HAL_OK) {
        return DRV_UART_ERROR_TX_BUSY;
    }

    inst->tx_busy = true;

    return DRV_UART_OK;
}

bool drv_uart_is_tx_busy(drv_uart_channel_t ch)
{
    if (ch >= DRV_UART_CH_NUM || !s_inst[ch].initialized) {
        return false;
    }

    return s_inst[ch].tx_busy || s_inst[ch].huart->gState != HAL_UART_STATE_READY;
}

/* --- RX --- */

uint32_t drv_uart_rx_read(drv_uart_channel_t ch, uint8_t* buf, uint32_t max_len)
{
    /* kfifo 已移除，RX 数据通过 drv_uart_register_rx_callback 直接接收 */
    (void)ch; (void)buf; (void)max_len;
    return 0;
}

uint32_t drv_uart_rx_available(drv_uart_channel_t ch)
{
    (void)ch;
    return 0;
}

void drv_uart_sync_rx(drv_uart_channel_t ch)
{
    (void)ch;
}

void drv_uart_register_rx_callback(drv_uart_channel_t ch,
    drv_uart_rx_callback_t callback)
{
    if (ch >= DRV_UART_CH_NUM) return;
    s_inst[ch].rx_callback = callback;
}

/* ===== HAL 回调（统一管理所有 UART 实例） ===== */

/**
 * @brief UART TX DMA 完成回调 — 清除 tx_busy
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    for (uint32_t ch = 0; ch < DRV_UART_CH_NUM; ch++) {
        if (s_inst[ch].initialized && s_inst[ch].huart == huart) {
            s_inst[ch].tx_busy = false;
            return;
        }
    }
}

/**
 * @brief UART RX DMA 完成回调 — 通知上层，标记待重启
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    for (uint32_t ch = 0; ch < DRV_UART_CH_NUM; ch++) {
        if (s_inst[ch].initialized && s_inst[ch].huart == huart) {
            drv_uart_inst_t* inst = &s_inst[ch];
            if (inst->rx_callback) {
                inst->rx_callback((drv_uart_channel_t)ch, inst->rx_buf, sizeof(inst->rx_buf));
            }
            inst->rx_need_restart = true;
            return;
        }
    }
}

/**
 * @brief UART 错误回调 — 清标志
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    __HAL_UART_CLEAR_FLAG(huart,
        UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
}

/** @brief 主循环调用：重启 Normal DMA 接收 */
void drv_uart_rx_restart(drv_uart_channel_t ch)
{
    if (ch >= DRV_UART_CH_NUM || !s_inst[ch].initialized) return;
    drv_uart_inst_t* inst = &s_inst[ch];
    if (inst->rx_need_restart) {
        HAL_UART_Receive_DMA(inst->huart, inst->rx_buf, sizeof(inst->rx_buf));
        inst->rx_need_restart = false;
    }
}

/* Private functions ---------------------------------------------------------*/

