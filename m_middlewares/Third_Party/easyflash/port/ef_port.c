/**
 * @file    ef_port.c
 * @brief   EasyFlash Flash 操作移植层 — 代理到 drv_stm32g4_flash 驱动
 * @note    所有 Flash 读写/擦除/锁定操作均委派给 device_drivers/drv_stm32g4_flash.c，
 *          本文件仅做 EasyFlash 接口适配和默认环境变量表管理。
 */

#include <easyflash.h>
#include "drv_stm32g4_flash.h"
#include "log.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/

/** @brief 默认环境变量表 — 无预定义变量 */
static const ef_env default_env_set[] = { { NULL, NULL } };

/* Exported functions --------------------------------------------------------*/

/**
 * @brief   EasyFlash 端口初始化
 * @param[out] default_env      默认环境变量表指针输出
 * @param[out] default_env_size 默认环境变量数量输出
 * @retval   EF_NO_ERR           初始化成功
 *
 * 调用 drv_flash_init() 完成硬件配置（Bank 检测/页大小缓存），
 * 然后返回空的环境变量表。
 */
EfErrCode ef_port_init(ef_env const** default_env, size_t* default_env_size)
{
    drv_flash_init();

    *default_env = default_env_set;
    *default_env_size = 0;

    return EF_NO_ERR;
}

/**
 * @brief   从 Flash 读取数据
 * @param[in]  addr  Flash 物理地址
 * @param[out] buf   接收缓冲区
 * @param[in]  size  读取字节数
 * @retval   EF_NO_ERR   读取成功
 * @retval   EF_READ_ERR 读取失败
 */
EfErrCode ef_port_read(uint32_t addr, uint32_t* buf, size_t size)
{
    return (drv_flash_read(addr, buf, size) == DRV_FLASH_OK)
        ? EF_NO_ERR : EF_READ_ERR;
}

/**
 * @brief   擦除 Flash 页
 * @param[in] addr  Flash 物理地址（需页对齐）
 * @param[in] size  擦除字节数（需页对齐）
 * @retval   EF_NO_ERR    擦除成功
 * @retval   EF_ERASE_ERR 擦除失败
 */
EfErrCode ef_port_erase(uint32_t addr, size_t size)
{
    return (drv_flash_erase(addr, size) == DRV_FLASH_OK)
        ? EF_NO_ERR : EF_ERASE_ERR;
}

/**
 * @brief   向 Flash 写入数据（64-bit 双字编程 + 读回校验）
 * @param[in] addr Flash 物理地址
 * @param[in] buf  要写入的数据缓冲区
 * @param[in] size 要写入的字节数
 * @retval   EF_NO_ERR   写入成功
 * @retval   EF_WRITE_ERR 写入失败
 */
EfErrCode ef_port_write(uint32_t addr, const uint32_t* buf, size_t size)
{
    return (drv_flash_write(addr, buf, size) == DRV_FLASH_OK)
        ? EF_NO_ERR : EF_WRITE_ERR;
}

/**
 * @brief   锁定环境变量缓存（中断安全的嵌套锁）
 * @note    代理到 drv_flash_env_lock()
 */
void ef_port_env_lock(void)
{
    drv_flash_env_lock();
}

/**
 * @brief   解锁环境变量缓存（中断安全的嵌套锁）
 * @note    代理到 drv_flash_env_unlock()
 */
void ef_port_env_unlock(void)
{
    drv_flash_env_unlock();
}

/* Logging stubs (EasyFlash expects these for debug/info output) --------------*/

/**
 * @brief EasyFlash 调试日志 — 映射到 LOG_D（含文件/行号）
 */
void ef_log_debug(const char* file, long line, const char* format, ...)
{
    char buf[LOG_DEFAULT_FORMAT_BUFFER_SIZE];
    va_list args;

    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    log_log(LOG_LEVEL_DEBUG, "easyflash", "%s:%ld %s", file, line, buf);
}

/**
 * @brief EasyFlash 信息日志 — 映射到 LOG_I
 */
void ef_log_info(const char* format, ...)
{
    char buf[LOG_DEFAULT_FORMAT_BUFFER_SIZE];
    va_list args;

    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    log_log(LOG_LEVEL_INFO, "easyflash", "%s", buf);
}

/**
 * @brief EasyFlash 原始输出 — 映射到 LOG_I
 */
void ef_print(const char* format, ...)
{
    char buf[LOG_DEFAULT_FORMAT_BUFFER_SIZE];
    va_list args;

    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    log_log(LOG_LEVEL_INFO, "easyflash", "%s", buf);
}
