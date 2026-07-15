/**
 * @file    drv_uart.c
 * @author  maximillian
 * @version V2.2.0
 * @date    2026-07-15
 * @brief   通用串口设备驱动实现（USART1/2/3，DMA 输出 + DMA 正常模式 ping-pong 双缓冲接收）
 * @attention
 *
 * 多实例驱动：内部句柄 + DMA 配置表，drv_uart_init() 无需传参。
 * TX：DMA normal，gState + s_tx_busy 双状态检忙。
 * RX：DMA normal 模式 ping-pong 双缓冲 —— 收满中断内先切换缓冲并立即重启
 *     DMA（微秒级窗口，抑制 ORE），再解析已收满的旧缓冲；主循环
 *     drv_uart_rx_restart() 仅作重启失败时的兜底。
 *
 * HAL 回调 (HAL_UART_TxCpltCallback / HAL_UART_RxCpltCallback) 集中在此文件。
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_uart.h"

#include "log.h"
#include "usart.h"

#include <string.h>

/* Private constants ---------------------------------------------------------*/

/** @brief RX DMA 单次接收字节数（256 = ~11ms，减少重启频次） */
#define DRV_UART_RX_BUF_SIZE (20U*1)

/** @brief TX 最大单次发送量 */
#define DRV_UART_TX_BUF_SIZE (256U)

/* Private types -------------------------------------------------------------*/

typedef struct {
    UART_HandleTypeDef* huart;          /**< HAL UART 句柄 */
    uint8_t             rx_buf[2][DRV_UART_RX_BUF_SIZE]; /**< RX DMA ping-pong 双缓冲 */
    volatile uint8_t    rx_buf_idx;     /**< DMA 当前正在填充的缓冲索引 (0/1) */
    uint8_t             tx_buf[DRV_UART_TX_BUF_SIZE]; /**< TX DMA 发送缓冲 */
    bool                tx_busy;        /**< TX DMA 传输中 */
    volatile bool       rx_need_restart; /**< ISR 内重启失败时主循环兜底重启 */
    bool                initialized;    /**< 初始化标志 */
    drv_uart_rx_callback_t rx_callback; /**< 接收回调（中断中执行） */
} drv_uart_inst_t;

/* Private variables ---------------------------------------------------------*/

static drv_uart_inst_t s_inst[DRV_UART_CH_NUM] = {
    [DRV_UART_CH_1] = { .huart = &huart1 },
    [DRV_UART_CH_2] = { .huart = &huart2 },
    [DRV_UART_CH_3] = { .huart = &huart3 },
};

/* Private functions prototypes ----------------------------------------------*/

/**
 * @brief 启动 RX DMA 前清除残留错误标志并冲刷 RDR
 * @note  MX_USARTx_Init 后接收器已使能，启动 DMA 前线上到达的字节会
 *        滞留 RDR 并置位 ORE；HAL_UART_Receive_DMA 使能 EIE 后残留
 *        ORE 立刻触发错误中断并中止接收。必须先清再启动。
 */
static void drv_uart_rx_flush_errors(UART_HandleTypeDef* huart)
{
    __HAL_UART_CLEAR_FLAG(huart,
        UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
    __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
}

/**
 * @brief 启动/重启 RX DMA 到当前活动缓冲（ISR 与主循环共用）
 * @return true=启动成功；false=HAL 句柄忙等，需主循环兜底重试
 */
static bool drv_uart_rx_start(drv_uart_inst_t* inst)
{
    drv_uart_rx_flush_errors(inst->huart);
    return HAL_UART_Receive_DMA(inst->huart,
               inst->rx_buf[inst->rx_buf_idx], DRV_UART_RX_BUF_SIZE) == HAL_OK;
}

/* Exported functions --------------------------------------------------------*/

/* --- 初始化 / 生命周期 --- */

drv_uart_error_t drv_uart_init(void)
{
    for (uint32_t ch = 0; ch < DRV_UART_CH_NUM; ch++) {
        drv_uart_inst_t* inst = &s_inst[ch];

        inst->tx_busy = false;
        inst->initialized = false;
        inst->rx_buf_idx = 0;
        inst->rx_need_restart = false;

        /* 清残留 ORE/RDR 后启动 DMA normal 模式接收（缓冲 0） */
        if (!drv_uart_rx_start(inst)) {
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
 * @brief UART RX DMA 完成回调 — ping-pong 切换缓冲后立即重启，再解析旧缓冲
 * @note  先重启后解析：把接收停止窗口从「ISR 解析 256B + 主循环延迟」
 *        压缩到微秒级，500 kbps 下需 2 字节 (40µs) 才会 ORE，窗口足够安全。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    for (uint32_t ch = 0; ch < DRV_UART_CH_NUM; ch++) {
        if (s_inst[ch].initialized && s_inst[ch].huart == huart) {
            drv_uart_inst_t* inst = &s_inst[ch];

            /* 1. 切换到另一块缓冲并立即重启 DMA */
            uint8_t* done_buf = inst->rx_buf[inst->rx_buf_idx];
            inst->rx_buf_idx ^= 1U;
            if (!drv_uart_rx_start(inst)) {
                inst->rx_need_restart = true; /* 句柄忙（主循环正在启动 TX 等），主循环兜底 */
            }

            /* 2. 解析已收满的旧缓冲（此期间 DMA 已在填充新缓冲） */
            if (inst->rx_callback) {
                inst->rx_callback((drv_uart_channel_t)ch, done_buf, DRV_UART_RX_BUF_SIZE);
            }
            return;
        }
    }
}

/**
 * @brief UART 错误回调 — 打印错误原因，立即重启接收
 * @note  DMA 接收模式下 HAL 将 ORE/FE/NE 等按阻塞错误处理并中止 RX DMA，
 *        必须立即重启否则接收永久停止；活动缓冲数据可能已被污染，丢弃。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    uint32_t err = HAL_UART_GetError(huart);

    for (uint32_t ch = 0; ch < DRV_UART_CH_NUM; ch++) {
        drv_uart_inst_t* inst = &s_inst[ch];
        if (!inst->initialized || inst->huart != huart) {
            continue;
        }

        LOG_E("drv_uart", "ch%u err=0x%02lX%s%s%s%s%s%s",
            (unsigned)ch + 1U, (unsigned long)err,
            (err & HAL_UART_ERROR_PE) ? " PE(parity)" : "",
            (err & HAL_UART_ERROR_NE) ? " NE(noise)" : "",
            (err & HAL_UART_ERROR_FE) ? " FE(frame)" : "",
            (err & HAL_UART_ERROR_ORE) ? " ORE(overrun)" : "",
            (err & HAL_UART_ERROR_DMA) ? " DMA(transfer)" : "",
            (err & HAL_UART_ERROR_RTO) ? " RTO(timeout)" : "");

        /* RX DMA 已被 HAL 中止，原地立即重启（rx_start 内部先清错误标志） */
        if (!drv_uart_rx_start(inst)) {
            inst->rx_need_restart = true;
        }

        /* TX DMA 错误同样会中止发送，防止 tx_busy 永久卡死 */
        if (huart->gState == HAL_UART_STATE_READY) {
            inst->tx_busy = false;
        }
        return;
    }

    /* 未匹配到实例（初始化早期），仅清标志防止错误中断风暴 */
    drv_uart_rx_flush_errors(huart);
}

/** @brief 主循环调用：ISR 内重启失败时的兜底重启 */
void drv_uart_rx_restart(drv_uart_channel_t ch)
{
    if (ch >= DRV_UART_CH_NUM || !s_inst[ch].initialized) return;
    drv_uart_inst_t* inst = &s_inst[ch];
    if (inst->rx_need_restart) {
        if (drv_uart_rx_start(inst)) {
            inst->rx_need_restart = false;
        }
    }
}

/* Private functions ---------------------------------------------------------*/

