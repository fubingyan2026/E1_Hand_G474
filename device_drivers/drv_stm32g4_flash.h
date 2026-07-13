//
// Created by maximillian on 2026-07-06.
//

/**
 * @file    drv_stm32g4_flash.h
 * @brief   STM32G474 Flash 底层驱动 — 64-bit 双字编程，支持单/双 Bank
 *
 * 提供 Flash 操作原语（擦除/写入/读取/缓存管理），供 EasyFlash 移植层
 * (ef_port.c) 调用。本驱动属于 device_drivers 层，是唯一接触 HAL Flash 的模块。
 */

#ifndef __DRV_STM32G4_FLASH_H
#define __DRV_STM32G4_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

typedef enum {
    DRV_FLASH_OK = 0,       /**< 操作成功 */
    DRV_FLASH_ERASE_ERR,    /**< 擦除失败 */
    DRV_FLASH_READ_ERR,     /**< 读取失败 */
    DRV_FLASH_WRITE_ERR,    /**< 写入失败 */
} drv_flash_error_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief   Flash 驱动初始化 — 检测单/双 Bank 模式，缓存页大小
 * @note    由 ef_port.c 内部调用，应用层无需直接调用
 */
void drv_flash_init(void);

/**
 * @brief   从 Flash 读取数据（字节拷贝）
 * @param addr  Flash 物理地址
 * @param buf   接收缓冲区
 * @param size  读取字节数
 * @return  DRV_FLASH_OK=成功
 */
drv_flash_error_t drv_flash_read(uint32_t addr, uint32_t* buf, size_t size);

/**
 * @brief   擦除 Flash 页（支持跨 Bank 区域自动拆分）
 * @param addr  Flash 物理地址（需页对齐）
 * @param size  擦除字节数（需页对齐）
 */
drv_flash_error_t drv_flash_erase(uint32_t addr, size_t size);

/**
 * @brief   写 Flash（64-bit 双字编程 + 读回校验）
 */
drv_flash_error_t drv_flash_write(uint32_t addr, const uint32_t* buf, size_t size);

/**
 * @brief   使 Flash Data Cache 失效（擦/写后调用，防止读到缓存旧值）
 */
void drv_flash_cache_invalidate(void);

/**
 * @brief   中断安全的嵌套锁 — 锁定
 */
void drv_flash_env_lock(void);

/**
 * @brief   中断安全的嵌套锁 — 解锁
 */
void drv_flash_env_unlock(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_STM32G4_FLASH_H */
