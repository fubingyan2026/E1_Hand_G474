/**
 * @file    drv_uart.c
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-07-09
 * @brief   通用串口设备驱动实现（USART1/2/3，DMA 输出 + DMA circular + IDLE 中断接收）
 * @attention
 *
 * 多实例驱动：内部句柄 + DMA 配置表，drv_uart_init() 无需传参。
 * TX：DMA normal，gState + s_tx_busy 双状态检忙。
 * RX：DMA circular + IDLE 中断 → kfifo_move_in 零拷贝同步，无逐字节中断。
 *
 * HAL 回调 (HAL_UART_TxCpltCallback / HAL_UARTEx_RxEventCallback) 集中在此文件。
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_uart.h"

#include "kfifo.h"
#include "usart.h"

#include <string.h>

/* Private types -------------------------------------------------------------*/

/**
 * @brief UART 通道实例
 */
typedef struct {
    UART_HandleTypeDef* huart;          /**< HAL UART 句柄 */
    uint8_t*            rx_dma_buf;     /**< RX DMA circular 缓冲区 */
    uint32_t            rx_dma_buf_size;/**< 缓冲区大小（2 的幂） */
    kfifo_t             rx_fifo;        /**< 接收 FIFO（建在 rx_dma_buf 上，零拷贝） */
    bool                tx_busy;        /**< TX DMA 传输中标志 */
    bool                initialized;    /**< 初始化标志 */
} drv_uart_inst_t;

/* Private constants ---------------------------------------------------------*/

/** @brief USART1 RX DMA circular 缓冲区大小（日志串口，较大） */
#define DRV_UART1_RX_BUF_SIZE (1024U)

/** @brief USART2 RX DMA circular 缓冲区大小 */
#define DRV_UART2_RX_BUF_SIZE (256U)

/** @brief USART3 RX DMA circular 缓冲区大小 */
#define DRV_UART3_RX_BUF_SIZE (256U)

/* Private variables ---------------------------------------------------------*/

/** @brief USART1 RX DMA circular 缓冲区 */
static uint8_t s_rx_buf_1[DRV_UART1_RX_BUF_SIZE];

/** @brief USART2 RX DMA circular 缓冲区 */
static uint8_t s_rx_buf_2[DRV_UART2_RX_BUF_SIZE];

/** @brief USART3 RX DMA circular 缓冲区 */
static uint8_t s_rx_buf_3[DRV_UART3_RX_BUF_SIZE];

/**
 * @brief 通道实例表（下标与 drv_uart_channel_t 对应）
 */
static drv_uart_inst_t s_inst[DRV_UART_CH_NUM] = {
    [DRV_UART_CH_1] = { .huart = &huart1, .rx_dma_buf = s_rx_buf_1, .rx_dma_buf_size = DRV_UART1_RX_BUF_SIZE },
    [DRV_UART_CH_2] = { .huart = &huart2, .rx_dma_buf = s_rx_buf_2, .rx_dma_buf_size = DRV_UART2_RX_BUF_SIZE },
    [DRV_UART_CH_3] = { .huart = &huart3, .rx_dma_buf = s_rx_buf_3, .rx_dma_buf_size = DRV_UART3_RX_BUF_SIZE },
};

/* Private function prototypes -----------------------------------------------*/

static void drv_uart_sync_rx_dma(drv_uart_channel_t ch);

/* Exported functions --------------------------------------------------------*/

/* --- 初始化 / 生命周期 --- */

drv_uart_error_t drv_uart_init(void)
{
    for (uint32_t ch = 0; ch < DRV_UART_CH_NUM; ch++) {
        drv_uart_inst_t* inst = &s_inst[ch];

        inst->tx_busy = false;
        inst->initialized = false;

        kfifo_init(&inst->rx_fifo, inst->rx_dma_buf, inst->rx_dma_buf_size, NULL);

        /* 启动 DMA circular + IDLE 中断接收 */
        if (HAL_UARTEx_ReceiveToIdle_DMA(inst->huart, inst->rx_dma_buf, inst->rx_dma_buf_size) != HAL_OK) {
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

        __HAL_UART_DISABLE_IT(inst->huart, UART_IT_IDLE);
        HAL_UART_DMAStop(inst->huart);
        kfifo_reset(&inst->rx_fifo);

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

    if (HAL_UART_Transmit_DMA(inst->huart, (uint8_t*)data, (uint16_t)len) != HAL_OK) {
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
    if (ch >= DRV_UART_CH_NUM || !buf || max_len == 0) {
        return 0;
    }
    if (!s_inst[ch].initialized) {
        return 0;
    }

    return kfifo_get(&s_inst[ch].rx_fifo, buf, max_len);
}

uint32_t drv_uart_rx_available(drv_uart_channel_t ch)
{
    if (ch >= DRV_UART_CH_NUM || !s_inst[ch].initialized) {
        return 0;
    }

    return kfifo_len(&s_inst[ch].rx_fifo);
}

/* ===== HAL 回调（统一管理所有 UART 实例） ===== */

/**
 * @brief UART TX DMA 完成回调
 *
 * 遍历所有实例匹配句柄，清除 tx_busy 标志。
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
 * @brief UART RX IDLE 事件回调
 *
 * 遍历所有实例匹配句柄，同步 DMA 位置到 kfifo。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    (void)Size;

    for (uint32_t ch = 0; ch < DRV_UART_CH_NUM; ch++) {
        if (s_inst[ch].initialized && s_inst[ch].huart == huart) {
            drv_uart_sync_rx_dma((drv_uart_channel_t)ch);
            return;
        }
    }
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 将指定通道的 RX circular DMA 写入位置同步到接收 FIFO
 * @param ch 通道号
 */
static void drv_uart_sync_rx_dma(drv_uart_channel_t ch)
{
    if (ch >= DRV_UART_CH_NUM || !s_inst[ch].initialized) {
        return;
    }

    drv_uart_inst_t* inst = &s_inst[ch];
    if (!inst->huart->hdmarx) {
        return;
    }

    const uint32_t remaining = __HAL_DMA_GET_COUNTER(inst->huart->hdmarx);
    const uint32_t dma_hw_index = inst->rx_dma_buf_size - remaining;

    kfifo_move_in(&inst->rx_fifo, dma_hw_index);
}
