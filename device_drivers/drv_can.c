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
#include "log.h"
#include "main.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define CAN_LOG_ENABLE 1

#if CAN_LOG_ENABLE
#define CAN_LOG_E(...) LOG_E("drv_can", __VA_ARGS__)
#define CAN_LOG_W(...) LOG_W("drv_can", __VA_ARGS__)
#define CAN_LOG_I(...) LOG_I("drv_can", __VA_ARGS__)
#define CAN_LOG_D(...) LOG_D("drv_can", __VA_ARGS__)
#else
#define CAN_LOG_E(...) ((void)0)
#define CAN_LOG_W(...) ((void)0)
#define CAN_LOG_I(...) ((void)0)
#define CAN_LOG_D(...) ((void)0)
#endif

/* Private constants -----------------------------------------------------------*/

/** @brief 发送失败日志聚合窗口 (ms) */
#define DRV_CAN_ERR_LOG_PERIOD_MS (1000U)

/* Private types -------------------------------------------------------------*/

typedef struct {
    drv_can_rx_callback_t rx_callback;
    bool                  initialized;
    bool                  bus_off;      /**< 上次轮询的 Bus-Off 状态（边沿检测） */
    bool                  err_passive;  /**< 上次轮询的 Error-Passive 状态 */
    uint32_t              tx_fail_cnt;  /**< 日志聚合窗口内发送失败次数 */
    uint32_t              tx_fail_log;  /**< 上次发送失败日志时间戳 (ms) */
    uint32_t              diag_log;     /**< 上次诊断日志时间戳 (ms) */
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

        /* 1. 全局 filter 必须在 Start 之前配置（HAL 要求 READY 状态）：
         *    放行所有非匹配的标准帧和扩展帧到 RX FIFO0，拒绝 remote 帧 */
        if (HAL_FDCAN_ConfigGlobalFilter(s_hfdcan[ch],
                FDCAN_ACCEPT_IN_RX_FIFO0,
                FDCAN_ACCEPT_IN_RX_FIFO0,
                FDCAN_REJECT_REMOTE,
                FDCAN_REJECT_REMOTE) != HAL_OK) {
            CAN_LOG_E("ch%u ConfigGlobalFilter failed (state=%d)",
                (unsigned)ch + 1U, (int)HAL_FDCAN_GetState(s_hfdcan[ch]));
            return DRV_CAN_ERROR_UNINITIALIZED;
        }

        /* 2. 使能 RX FIFO 0 新消息中断（映射到中断线 0） */
        if (HAL_FDCAN_ActivateNotification(s_hfdcan[ch],
                FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
            CAN_LOG_E("ch%u ActivateNotification failed", (unsigned)ch + 1U);
            return DRV_CAN_ERROR_UNINITIALIZED;
        }

        /* 3. 最后启动外设（进入 BUSY，开始参与总线） */
        if (HAL_FDCAN_Start(s_hfdcan[ch]) != HAL_OK) {
            CAN_LOG_E("ch%u FDCAN Start failed", (unsigned)ch + 1U);
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
        .BitRateSwitch        = FDCAN_BRS_OFF,
        .FDFormat             = msg->is_fd ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN,
        .TxEventFifoControl   = FDCAN_NO_TX_EVENTS,
        .MessageMarker        = 0,
    };

    if (HAL_FDCAN_AddMessageToTxFifoQ(s_hfdcan[ch], &tx, msg->data) != HAL_OK) {
        /* 发送失败限频日志（TX FIFO 满通常意味着无 ACK / 总线异常） */
        drv_can_ctx_t* ctx = &s_ctx[ch];
        ctx->tx_fail_cnt++;
        uint32_t now = HAL_GetTick();
        if (now - ctx->tx_fail_log >= DRV_CAN_ERR_LOG_PERIOD_MS) {
            CAN_LOG_E("ch%u tx fail x%lu (fifo free=%lu)",
                (unsigned)ch + 1U, (unsigned long)ctx->tx_fail_cnt,
                (unsigned long)HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan[ch]));
            ctx->tx_fail_log = now;
            ctx->tx_fail_cnt = 0;
        }
        return DRV_CAN_ERROR_TX_BUSY;
    }

    return DRV_CAN_OK;
}

/* --- 状态监控 --- */

void drv_can_poll_status(drv_can_channel_t ch)
{
    if (ch >= DRV_CAN_CH_NUM || !s_ctx[ch].initialized) {
        return;
    }

    FDCAN_ProtocolStatusTypeDef psr;
    if (HAL_FDCAN_GetProtocolStatus(s_hfdcan[ch], &psr) != HAL_OK) {
        return;
    }

    drv_can_ctx_t* ctx = &s_ctx[ch];

    /* Bus-Off：打日志 + 清 CCCR.INIT 触发硬件恢复序列（129×11 个隐性位后重新入网） */
    if (psr.BusOff) {
        if (!ctx->bus_off) {
            ctx->bus_off = true;
            CAN_LOG_E("ch%u BUS-OFF! check bitrate/sample-point/termination, recovering...",
                (unsigned)ch + 1U);
        }
        CLEAR_BIT(s_hfdcan[ch]->Instance->CCCR, FDCAN_CCCR_INIT);
    } else if (ctx->bus_off) {
        ctx->bus_off = false;
        CAN_LOG_I("ch%u bus recovered", (unsigned)ch + 1U);
    }

    /* Error-Passive 边沿：无 ACK / 位错误累积的早期信号 */
    if (psr.ErrorPassive != ctx->err_passive) {
        ctx->err_passive = psr.ErrorPassive;
        if (psr.ErrorPassive) {
            CAN_LOG_W("ch%u ERROR-PASSIVE (tx err accumulating, no ACK?)",
                (unsigned)ch + 1U);
        } else {
            CAN_LOG_I("ch%u back to error-active", (unsigned)ch + 1U);
        }
    }

    /* TX FIFO 卡满时输出限频诊断（定位 FIFO 不排空的原因）：
     * act: 0x00=SYNC(等待总线空闲/同步) 0x08=IDLE 0x10=RX 0x18=TX
     *      —— 长期 SYNC 说明 RX 脚被拉在显性电平（收发器故障/未上电/总线短路）
     * lec: 0=无 1=stuff 2=form 3=ACK 4=bit1 5=bit0 6=CRC 7=无变化
     * tec/rec: 发送/接收错误计数器 */
    if (HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan[ch]) == 0U) {
        uint32_t now = HAL_GetTick();
        if (now - ctx->diag_log >= DRV_CAN_ERR_LOG_PERIOD_MS) {
            ctx->diag_log = now;
            FDCAN_ErrorCountersTypeDef ec = { 0 };
            (void)HAL_FDCAN_GetErrorCounters(s_hfdcan[ch], &ec);
            CAN_LOG_W("ch%u txfifo FULL act=0x%02lX lec=%lu tec=%lu rec=%lu%s%s",
                (unsigned)ch + 1U,
                (unsigned long)psr.Activity,
                (unsigned long)psr.LastErrorCode,
                (unsigned long)ec.TxErrorCnt,
                (unsigned long)ec.RxErrorCnt,
                psr.ErrorPassive ? " EP" : "",
                psr.BusOff ? " BUSOFF" : "");
        }
    }
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
