/**
 * @file    drv_can.c
 * @author  maximillian
 * @version V2.2.0
 * @date    2026-07-09
 * @brief   CAN 设备驱动（STM32G474 FDCAN，经典 CAN + CAN FD）
 * @attention
 *
 * CubeMX 配置 FDCAN1 (PA11 RX / PA12 TX)，FDCAN_FRAME_FD_NO_BRS 模式。
 * 句柄表内置在模块中，drv_can_init() 无需传参。
 * drv_can_msg_t.dlc 始终为实际字节数，driver 内部与 FDCAN DLC 编码互转。
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_can.h"

#include "fdcan.h"
#include "main.h"

#include <string.h>

/* Private types -------------------------------------------------------------*/

typedef struct {
    drv_can_rx_callback_t rx_callback;
    bool                  initialized;
} drv_can_ctx_t;

/* Private constants ---------------------------------------------------------*/

/** @brief 通道 → HAL 句柄映射（基于 CubeMX fdcan.c，仅 FDCAN1） */
static FDCAN_HandleTypeDef* const s_hfdcan[DRV_CAN_CH_NUM] = {
    [DRV_CAN_CH_1] = &hfdcan1,
};

/**
 * @brief FDCAN DLC 编码 → 实际字节数 转换表
 *
 * FDCAN_TxHeaderTypeDef / RxHeaderTypeDef 的 DataLength 字段使用 DLC 编码（0-15），
 * 与实际字节数并非线性对应（如 DLC=9 → 12 字节）。此表做一次查表转换。
 */
static const uint8_t s_dlc_to_bytes[16] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8,      /*  0-8 */
    12, 16, 20, 24, 32, 48, 64,      /* 9-15 */
};

/* Private variables ---------------------------------------------------------*/

static drv_can_ctx_t s_ctx[DRV_CAN_CH_NUM];

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief 实际字节数 → FDCAN DLC 编码
 * @param byte_count 实际字节数（0-64）
 * @return FDCAN DLC 编码值（0-15），可直接赋给 .DataLength
 */
static uint32_t bytes_to_fdcan_dlc(uint16_t byte_count);

/* Exported functions --------------------------------------------------------*/

/* --- 初始化 / 生命周期 --- */

drv_can_error_t drv_can_init(void)
{
    /* 拉低 CAN 收发器 STB 引脚（正常模式，低电平有效） */
    HAL_GPIO_WritePin(FD_CAN1_STB_PA10_GPIO_Port, FD_CAN1_STB_PA10_Pin, GPIO_PIN_RESET);

    for (uint32_t ch = 0; ch < DRV_CAN_CH_NUM; ch++) {
        memset(&s_ctx[ch], 0, sizeof(s_ctx[ch]));

        if (HAL_FDCAN_Start(s_hfdcan[ch]) != HAL_OK) {
            return DRV_CAN_ERROR_UNINITIALIZED;
        }

        /* 配置全局 filter：放行所有非匹配的标准帧和扩展帧，拒绝 remote 帧 */
        if (HAL_FDCAN_ConfigGlobalFilter(s_hfdcan[ch],
                FDCAN_ACCEPT_IN_RX_FIFO0,
                FDCAN_ACCEPT_IN_RX_FIFO0,
                FDCAN_REJECT_REMOTE,
                FDCAN_REJECT_REMOTE) != HAL_OK) {
            HAL_FDCAN_Stop(s_hfdcan[ch]);
            return DRV_CAN_ERROR_UNINITIALIZED;
        }

        /* 使能 RX FIFO 0 新消息中断（映射到中断线 0） */
        if (HAL_FDCAN_ActivateNotification(s_hfdcan[ch],
                FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
            HAL_FDCAN_Stop(s_hfdcan[ch]);
            return DRV_CAN_ERROR_UNINITIALIZED;
        }

        s_ctx[ch].initialized = true;
    }

    return DRV_CAN_OK;
}

void drv_can_deinit_all(void)
{
    for (uint32_t ch = 0; ch < DRV_CAN_CH_NUM; ch++) {
        if (!s_ctx[ch].initialized) {
            continue;
        }
        HAL_FDCAN_DeactivateNotification(s_hfdcan[ch], FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
        HAL_FDCAN_Stop(s_hfdcan[ch]);
        memset(&s_ctx[ch], 0, sizeof(s_ctx[ch]));
    }

    /* 拉高 CAN 收发器 STB 引脚（待机模式） */
    HAL_GPIO_WritePin(FD_CAN1_STB_PA10_GPIO_Port, FD_CAN1_STB_PA10_Pin, GPIO_PIN_SET);
}

bool drv_can_is_initialized(drv_can_channel_t ch)
{
    if (ch >= DRV_CAN_CH_NUM) {
        return false;
    }
    return s_ctx[ch].initialized;
}

/* --- 发送 --- */

drv_can_error_t drv_can_send(drv_can_channel_t ch, const drv_can_msg_t* msg)
{
    if (ch >= DRV_CAN_CH_NUM || !msg) {
        return DRV_CAN_ERROR_INVALID_PARAM;
    }
    if (!s_ctx[ch].initialized) {
        return DRV_CAN_ERROR_UNINITIALIZED;
    }
    if (msg->dlc > 64) {
        return DRV_CAN_ERROR_INVALID_PARAM;
    }

    FDCAN_TxHeaderTypeDef tx = {
        .Identifier           = msg->id,
        .IdType               = msg->is_extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID,
        .TxFrameType          = FDCAN_DATA_FRAME,
        .DataLength           = bytes_to_fdcan_dlc(msg->dlc),
        .ErrorStateIndicator  = FDCAN_ESI_ACTIVE,
        .BitRateSwitch        = msg->is_fd ? FDCAN_BRS_ON : FDCAN_BRS_OFF,
        .FDFormat             = msg->is_fd ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN,
        .TxEventFifoControl   = FDCAN_NO_TX_EVENTS,
        .MessageMarker        = 0,
    };

    if (HAL_FDCAN_AddMessageToTxFifoQ(s_hfdcan[ch], &tx, msg->data) != HAL_OK) {
        return DRV_CAN_ERROR_TX_BUSY;
    }

    return DRV_CAN_OK;
}

bool drv_can_tx_ready(drv_can_channel_t ch)
{
    if (ch >= DRV_CAN_CH_NUM || !s_ctx[ch].initialized) {
        return false;
    }
    return HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan[ch]) > 0;
}

/* ===== HAL 回调 ===== */

/**
 * @brief FDCAN Rx FIFO 0 新消息回调
 *
 * 由 HAL_FDCAN_IRQHandler 内部触发。
 * 读取报文后调用用户注册的接收回调，dlc 已转换为实际字节数。
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;

    /* 查找通道 */
    drv_can_channel_t ch;
    for (ch = 0; ch < DRV_CAN_CH_NUM; ch++) {
        if (s_ctx[ch].initialized && s_hfdcan[ch] == hfdcan) {
            break;
        }
    }
    if (ch >= DRV_CAN_CH_NUM) {
        return;
    }

    FDCAN_RxHeaderTypeDef rx;
    drv_can_msg_t msg;

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx, msg.data) != HAL_OK) {
        return;
    }

    msg.id          = rx.Identifier;
    msg.is_extended = (rx.IdType == FDCAN_EXTENDED_ID);
    msg.is_fd       = (rx.FDFormat == FDCAN_FD_CAN);

    /* FDCAN DataLength 是 DLC 编码值（0-15），查表转换为实际字节数 */
    {
        uint32_t dlc_val = rx.DataLength;
        msg.dlc = (dlc_val < 16) ? s_dlc_to_bytes[dlc_val] : 0;
    }

    if (s_ctx[ch].rx_callback) {
        s_ctx[ch].rx_callback(ch, &msg);
    }
}

/* --- 接收回调注册 --- */

drv_can_error_t drv_can_register_rx_callback(drv_can_channel_t ch,
    drv_can_rx_callback_t callback)
{
    if (ch >= DRV_CAN_CH_NUM) {
        return DRV_CAN_ERROR_INVALID_PARAM;
    }
    if (!s_ctx[ch].initialized) {
        return DRV_CAN_ERROR_UNINITIALIZED;
    }

    s_ctx[ch].rx_callback = callback;
    return DRV_CAN_OK;
}

/* Private functions ---------------------------------------------------------*/

static uint32_t bytes_to_fdcan_dlc(uint16_t byte_count)
{
    if (byte_count <= 8) {
        return byte_count;
    }
    if (byte_count <= 12) {
        return 9;
    }
    if (byte_count <= 16) {
        return 10;
    }
    if (byte_count <= 20) {
        return 11;
    }
    if (byte_count <= 24) {
        return 12;
    }
    if (byte_count <= 32) {
        return 13;
    }
    if (byte_count <= 48) {
        return 14;
    }
    return 15; /* up to 64 */
}
