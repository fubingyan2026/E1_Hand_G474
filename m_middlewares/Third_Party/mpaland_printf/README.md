# printf/sprintf 嵌入式微型实现

[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](https://raw.githubusercontent.com/mpaland/printf/master/LICENSE)

原作者：Marco Paland (info@paland.com), PALANDesign Hannover, Germany, 2014-2019  
**本项目适配版本**：已针对 STM32G474 (Cortex-M4 FPv4-SP) 裸机裁剪和 Bug 修复

---

## 概述

这是一个**短小精悍**的 printf、sprintf、(v)snprintf 实现，专为资源受限的嵌入式系统设计。

在裸机 STM32 项目中，标准 newlib 的 `printf` 会连带拉入大量库代码（堆管理、文件 I/O 等），
导致固件体积膨胀 **20KB+**。本实现 **零外部依赖**，自身包含整数转换（`ntoa`），
**无 malloc，线程安全**，完美契合本项目的"全程静态分配"原则。

**原始代码量仅约 600 行**，两个文件即插即用。

### 本项目所做的改动

| 改动 | 说明 |
|------|------|
| 🎯 **全部 `double` → `float`** | STM32G474 FPv4-SP 只有硬件单精度，`double` 靠软件仿真会额外增加 8~15 KB 固件。改为纯 `float` 运算，全部使用硬件指令 |
| 🗑️ **移除 `long long` 支持** | STM32G474 是 32 位 ARM，`long`/`size_t`/`ptrdiff_t`/`uintptr_t` 均为 4 字节，无需 64 位整数格式化。移除了 `_ntoa_long_long`、`FLAGS_LONG_LONG`、`%ll` 分支及相关条件判断 |
| 🔧 **Bug 修复** | 修复 `INT_MIN` 有符号整数溢出（UB）、`%s` 传 NULL 指针导致 HardFault、符号字符被 padding 填满时静默丢弃 三个缺陷 |
| 🔌 **CMake 集成** | 已接入项目 CMake 构建系统，通过 `#include "public.h"` 一行导入 |

---

## 使用方法

### 1. 加入工程

本项目已集成完毕，**无需手动添加文件**。任何源文件 `#include "public.h"` 即可使用。

### 2. 实现底层字符输出

若使用 `printf_()` 输出到终端/串口，需实现 `_putchar`（本项目日志模块已提供）：

```c
void _putchar(char character)
{
    // 将 character 发送到串口、RTT 或其它输出通道
    // 例如：SEGGER_RTT_WriteChar(0, character);
}
```

若仅使用 `sprintf_()` / `snprintf_()` 格式化字符串到缓冲区，**则无需实现 `_putchar`**。

### 3. API 一览

> **注意**：为避免与标准库冲突，本库函数名带有 `_` 后缀，并通过宏映射为标准名称。

| 函数 | 原型 | 说明 |
|------|------|------|
| `printf_` | `int printf_(const char* format, ...)` | 格式化输出到 `_putchar()` |
| `sprintf_` | `int sprintf_(char* buffer, const char* format, ...)` | 格式化到缓冲区（不限长度，慎用） |
| `snprintf_` | `int snprintf_(char* buffer, size_t count, const char* format, ...)` | 格式化到缓冲区（限长，推荐） |
| `vsnprintf_` | `int vsnprintf_(char* buffer, size_t count, const char* format, va_list va)` | va_list 版本 |
| `vprintf_` | `int vprintf_(const char* format, va_list va)` | va_list 版本 |
| `fctprintf` | `int fctprintf(void (*out)(char, void*), void* arg, const char* format, ...)` | 自定义输出回调 |

用法 1:1 兼容标准 `stdio.h`：

```c
#include "public.h"

void _putchar(char c) { /* 串口发送一个字节 */ }

int main(void)
{
    printf_("Hello World! %d %s", 42, "test");

    char buf[64];
    snprintf_(buf, sizeof(buf), "%d 号电机: %ld rpm", 3, 3200L);

    // fctprintf 可实现流式处理（如分段发送长字符串）
    fctprintf(&my_output_fn, NULL, "Large data: %s", huge_string);
    return 0;
}
```

> **安全提醒**：`sprintf_` 无缓冲区长度限制，容易导致缓冲区溢出。
> **强烈建议**改用 `snprintf_` 并传入正确的缓冲区大小。

---

## 格式说明符

格式原型：`%[flags][width][.precision][length]type`

### 支持的类型

| 类型 | 输出 |
|------|------|
| `d` 或 `i` | 有符号十进制整数 |
| `u` | 无符号十进制整数 |
| `b` | 无符号二进制 |
| `o` | 无符号八进制 |
| `x` | 无符号十六进制（小写） |
| `X` | 无符号十六进制（大写） |
| `c` | 单个字符 |
| `s` | 字符串 |
| `p` | 指针地址 |
| `%` | 输出一个 `%` |

> **本构建未启用**：`%f`/`%F`/`%e`/`%E`/`%g`/`%G`——因 STM32G474 无 `double` 硬件加速，已在编译时禁用。
> 如需浮点输出，请移除 CMakeLists.txt 中的 `PRINTF_DISABLE_SUPPORT_FLOAT` 和
> `PRINTF_DISABLE_SUPPORT_EXPONENTIAL`。

### 支持的标志（Flags）

| 标志 | 说明 |
|------|------|
| `-` | 左对齐（默认右对齐） |
| `+` | 强制显示正负号（`+`/`-`） |
| ` `（空格）| 正数前加一个空格 |
| `#` | 对 `o`/`b`/`x`/`X` 加前缀（`0`/`0b`/`0x`/`0X`） |
| `0` | 用 `0` 填充（而非空格） |

### 支持的宽度

| 宽度 | 说明 |
|------|------|
| `(number)` | 最小输出字符数，不足补空格；超出不截断 |
| `*` | 宽度由参数指定 |

### 支持的精度

| 精度 | 说明 |
|------|------|
| `.(number)` | 整数：最小数字位数（补前导零）；`s`：最大字符数 |
| `.*` | 精度由参数指定 |

### 支持的长度修饰符

| 长度 | `d`/`i` | `u`/`o`/`x`/`X` |
|------|----------|------------------|
| (无) | `int` | `unsigned int` |
| `hh` | `char` | `unsigned char` |
| `h` | `short int` | `unsigned short int` |
| `l` | `long int` | `unsigned long int` |
| `j` | `intmax_t` | `uintmax_t` |
| `z` | `size_t` | `size_t` |
| `t` | `ptrdiff_t`¹ | `ptrdiff_t`¹ |

> ¹ 需启用 `PRINTF_SUPPORT_PTRDIFF_T`（默认启用）
>
> **注意**：本构建已移除 `%ll`（long long）支持。STM32G474 为 32 位平台，
> `long` 已是 32 位，绝大多数场景无需 64 位整数格式化。
> 对于 `%j`（`intmax_t`）等理论上需要 64 位的类型，本构建以 32 位精度处理。

---

## 编译开关（功能裁剪）

编译开关已在 `CMakeLists.txt` 的 `target_compile_definitions` 中配置，也可在 `printf_config.h`
（需定义 `PRINTF_INCLUDE_CONFIG_H` 以启用）中覆盖。

### 本项目当前配置

```cmake
target_compile_definitions(E1_Hand_G474 PRIVATE
    PRINTF_DISABLE_SUPPORT_FLOAT          # 禁用 %f（避免链接软件 double 库）
    PRINTF_DISABLE_SUPPORT_EXPONENTIAL    # 禁用 %e/%g（同上）
)
```

### 完整配置选项

| 宏 | 默认值 | 说明 |
|----|--------|------|
| `PRINTF_INCLUDE_CONFIG_H` | 未定义 | 定义后从 `printf_config.h` 读取配置 |
| `PRINTF_NTOA_BUFFER_SIZE` | 32 | 整数转换栈缓冲区大小（字节） |
| `PRINTF_FTOA_BUFFER_SIZE` | 32 | 浮点转换栈缓冲区大小（字节） |
| `PRINTF_DEFAULT_FLOAT_PRECISION` | 6 | 浮点数默认小数位数（本构建未启用） |
| `PRINTF_MAX_FLOAT` | 1e9f | `%f` 的最大可打印值（本构建未启用） |
| `PRINTF_DISABLE_SUPPORT_FLOAT` | 未定义 | **本构建已启用**，禁用 `%f` |
| `PRINTF_DISABLE_SUPPORT_EXPONENTIAL` | 未定义 | **本构建已启用**，禁用 `%e`/`%g` |
| `PRINTF_DISABLE_SUPPORT_PTRDIFF_T` | 未定义 | 定义此宏**禁用** `%t` |
| `PRINTF_SUPPORT_LONG_LONG` | 已从代码中移除 | 本项目无需 64 位整数格式化 |

---

## 返回值

| 情况 | 返回值 |
|------|--------|
| 成功 | 写入的字符数（**不含**结尾 `\0`） |
| 截断 (`snprintf_`/`vsnprintf_`) | **本可以**写入的字符数（≥`count` 表示被截断） |
| 错误 | `-1` |
| `buffer = NULL` | 仅返回格式化长度，不写入任何内容 |

```c
int len = snprintf_(NULL, 0, "Hello, world");  // len = 12
```

---

## 在本项目中的集成

`mpaland_printf` 位于 `m_middlewares/Third_Party/mpaland_printf/`，通过以下方式使用：

- **`#include "public.h"`** 一行引入所有中间件，其中包含本库
- **日志模块**（`log/log.h`）的 `log_printf()` 底层使用此库的格式化引擎
- **SEGGER RTT 输出**（`log_task` 默认通道）通过实现 `_putchar` 重定向到 `SEGGER_RTT_WriteString()`
- **协议调试**：`protocol_packer`/`protocol_parser` 调试输出依赖此库

### CMake 集成（已配置）

```cmake
# CMakeLists.txt
aux_source_directory(m_middlewares/Third_Party/mpaland_printf MID_printf)
# ... 添加到 target_sources 和 target_include_directories
```

---

## Bug 修复清单（相对上游）

| Bug | 文件位置 | 现象 | 修复 |
|-----|----------|------|------|
| `INT_MIN` 有符号溢出 | [printf.c:739](m_middlewares/Third_Party/mpaland_printf/printf.c#L739) | `printf_("%d", INT_MIN)` 触发 UB | 改为 `-(unsigned)value` 无符号取负 |
| `%s` NULL 崩溃 | [printf.c:802](m_middlewares/Third_Party/mpaland_printf/printf.c#L802) | `printf_("%s", NULL)` 触发 HardFault | 添加 `if (!p) p = "(null)"` |
| 符号字符丢弃 | [printf.c:224-227](m_middlewares/Third_Party/mpaland_printf/printf.c#L224-L227) | 高精度 + 符号时 `+`/`-`/空格被 padding 吞掉 | 引入 `buf_limit` 为符号预留 1 字节 |

---

## 许可

本库基于 MIT 协议开源，版权所有 (c) 2014-2019 Marco Paland, PALANDesign Hannover, Germany。
