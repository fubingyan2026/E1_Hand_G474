# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**E1_Hand_G474** — 基于 STM32G474 的灵巧手嵌入式固件，裸机（无 RTOS），C11，CMake 构建。

## 代码生成黄金规则

### 优先复用 `m_middlewares/` 下已有的通用模块，禁止重复造轮子

在新增任何功能之前，**必须优先检查 `m_middlewares/` 中是否已有可复用的模块**。包括：

- **框架原语**：定时器（`sw_timer`）、状态机（`fsm`）、事件标志（`event`）、消息队列（`msg_fifo`）、看门狗（`daemon`）
- **数据结构**：无锁 FIFO（`kfifo`）、侵入式链表（`clist`）
- **算法**：PID/云台 PID、MIT 控制器、CRC、PT1 滤波、PLL 锁相环、数学工具（三角函数/插值/限幅）
- **协议**：协议打包器（`protocol_packer`）、协议解析器（`protocol_parser`）
- **日志**：带时间戳的 log 模块（`kfifo` 缓冲）
- **输入**：按键扫描（`key_base`）

> `m_middlewares/public.h` 一行 `#include` 即可引入所有中间件。

具体每个模块的能力见下方「核心中间件速查表」。

### Build & Development

### Prerequisites
- ARM GCC toolchain (`arm-none-eabi-gcc`)
- CMake >= 3.22

### Build commands

优先使用 `build.bat` 一键构建（自动检测 ARM GCC 工具链、CMake、Ninja）：

```bat
# Debug 构建（默认）
build.bat

# Release 构建
build.bat -t Release

# 从其他目录指定项目路径
build.bat D:\path\to\project -t Debug
```

手动构建（需自行配置工具链环境变量）：

```bash
# Debug build
cmake -B build/Debug -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -GNinja
ninja -C build/Debug

# Release build
cmake -B build/Release -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -GNinja
ninja -C build/Release
```

构建产物（位于 `build/{Debug|Release}/`）：
- `E1_Hand_G474.elf` — ELF 可执行
- `E1_Hand_G474.hex` — Intel HEX
- `E1_Hand_G474.bin` — 纯二进制

### MCU 信息
- STM32G474 (Cortex-M4, FPv4-SP)
- 编译宏：`STM32G474xx`, `USE_HAL_DRIVER`
- 启动文件：`startup_stm32g474xx.s`

## 架构：五层分层结构

> **详细分层规范请参见 [agent_standards/ARCHITECTURE_GUIDE.md](agent_standards/ARCHITECTURE_GUIDE.md)**

```
tasks/           应用编排层    xxx_task_init() / sw_timer       ← 拥有 sw_timer、注入回调
service/         业务逻辑层    srv_*  FSM / 协议 / 算法          ← 硬件无关或绑定单一 driver
m_middlewares/   通用中间件    sw_timer/fsm/msg_fifo/滤波…      ← 平台无关、可复用、静态分配
device_drivers/  设备驱动层    drv_*  HAL 薄封装 + 引脚自包含    ← 唯一接触 main.h/CubeMX 句柄
Core/ (CubeMX)   HAL 生成层    main.h / hfdcan1 / htimX / …    ← 只在 USER CODE 块内改
```

**依赖规则**：只能向下，禁止向上或跨层反向。

| 层 | 可 include | 禁止 include |
|----|-----------|-------------|
| tasks | service, device_drivers, middleware, Core | — |
| service | middleware、其绑定的 device_drivers、Core 类型 | 其他 service（除非明确数据依赖）、tasks |
| middleware | 其他 middleware、C 标准库 | device_drivers, service, tasks, Core/HAL |
| device_drivers | Core（main.h/HAL/外设句柄）、middleware | service, tasks |

### 关键原则
- **middleware 必须平台无关**（不含 `stm32g4xx_hal.h`）
- **device_drivers 是唯一允许引用 `main.h` 引脚宏和 CubeMX 句柄的层**
- **全程静态分配**（无 malloc），调用者提供句柄内存
- **`init` 一律无参**（driver 层内部包含 HAL 句柄表或宏）

## 目录结构

```
Core/                   CubeMX 生成代码（Inc + Src）
  Inc/main.h            引脚宏定义、外设句柄全局声明
  Inc/fdcan.h / tim.h / usart.h / gpio.h / dma.h
  Src/main.c            MX_xxx_Init() → app_main()
  Src/stm32g4xx_it.c    中断入口
device_drivers/         设备驱动层（drv_ 前缀）
  drv_can.c/h           CAN (FDCAN1, PA11/PA12)
  drv_uart.c/h          UART (USART1/2/3 — PC4/PC5, PA2/PA3, PB10/PB11, DMA + IDLE 中断)
  drv_systick.c/h       SysTick 延时/时间戳 (delay_us/millis/micros)
  drv_hw_timer.c/h      硬件定时器
m_middlewares/          通用中间件（平台无关）
  framework/            sw_timer, fsm, event, msg_fifo, daemon
  utils/                kfifo（无锁 2的幂 FIFO）, clist（侵入式双向链表）
  algorithm/            算法
    controller/         PID, gimbal_pid, MIT (力矩控制)
    filter/             滤波 (PT1)
    math/               maths, utils_math, utils
    pll/                锁相环
    crc.c/h             CRC 计算
  log/log.c/h           日志模块（kfifo 缓冲 + 时间戳回调）
  protocol_tools/       协议打包/解析器（protocol_packer, protocol_parser）
  key_base/             按键基础模块
  public.h              统一导出所有 middleware 头文件
service/                业务逻辑层（srv_ 前缀）
  srv_led.c/h           LED 控制（ON/OFF/闪烁/呼吸灯 FSM, clist 多实例管理）
tasks/                  应用编排层
  app_main.c/h          主入口：init 顺序 → for(;;) { sw_timer_tick(); sw_timer_task(); }
  can_task.c/h          CAN 通信任务（10ms 周期）
  led_task.c/h          LED 状态指示任务（蓝色+红色双 LED, 10ms 刷新）
  log_task.c/h          日志输出任务（5ms 周期, UART DMA TX + RX）
```

## 核心中间件速查表

**新增代码前先查下表**——看 `m_middlewares/` 是否有可直接使用的模块：

### 框架原语 (`m_middlewares/framework/`)

| 模块 | Include | API 前缀 | 能力 |
|------|---------|----------|------|
| `sw_timer` | `"sw_timer.h"` | `sw_timer_` | 软件定时器，3 优先级（HIGH ISR / NORMAL LOW 主循环），静态分配 |
| `fsm` | `"fsm.h"` | `fsm_` | 扁平转移矩阵状态机（state² guard 表）+ entry/exit 回调 + user_data |
| `event` | `"event.h"` | `event_` | ISR 安全的 32-bit 事件标志，ISR↔主循环生产者消费者 |
| `msg_fifo` | `"msg_fifo.h"` | `msg_fifo_` | 基于 kfifo 的定长消息队列，单生产者单消费者 |
| `daemon` | `"daemon.h"` | `daemon_` | 任务看门狗：注册名 + 超时 + 掉线回调 |

### 数据结构 (`m_middlewares/utils/`)

| 模块 | Include | API 前缀 | 能力 |
|------|---------|----------|------|
| `kfifo` | `"kfifo.h"` | `kfifo_` | 无锁 2 的幂字节 FIFO（所有 FIFO/队列的底层依赖） |
| `clist` | `"clist.h"` | `clist_` | 侵入式循环双向链表（多实例管理） |

### 算法 (`m_middlewares/algorithm/`)

| 模块 | 路径 | 能力 |
|------|------|------|
| PID | `controller/pid.h` | 增量式 PID |
| 云台 PID | `controller/gimbal_pid.h` | 云台专用 PID |
| MIT 控制器 | `controller/mit.h` | 力矩控制 |
| CRC | `crc.h` | CRC 计算 |
| PT1 滤波 | `filter/filter.h` | 一阶低通滤波 |
| 数学工具 | `math/maths.h`, `utils_math.h`, `utils.h` | 三角函数、插值、限幅、常用数学 |
| PLL | `pll/pll.h` | 锁相环 |

### 其他 (`m_middlewares/`)

| 模块 | Include | 能力 |
|------|---------|------|
| 日志 | `"log.h"` | kfifo 缓冲 + 时间戳回调，`log_printf()`/`log_hexdump()` |
| 协议打包 | `"protocol_packer.h"` | 结构化协议打包器 |
| 协议解析 | `"protocol_parser.h"` | 结构化协议解析器 |
| 按键 | `"key_base.h"` | 按键扫描/消抖/连击识别 |

## 初始化顺序

在 `app_main()` 中按依赖顺序调用：
1. `delay_init()` → SysTick 时基
2. `log_task_init()` → UART DMA 日志
3. `can_task_init()` → CAN（须早于 power_task）
4. `led_task_init()` → LED

## 编码规范

### 所有新代码必须严格遵循 [agent_standards/MODULE_CODING_GUIDE.md](agent_standards/MODULE_CODING_GUIDE.md)

核心要点速查：

| 规范 | 要求 |
|------|------|
| **代码风格** | WebKit（4 空格缩进，函数 Allman 大括号，控制语句 K&R） + MISRA C:2012 |
| **命名前缀** | driver=`drv_`，service=`srv_`，task=`xxx_task_`，middleware=模块名 |
| **类型后缀** | `_t` 结构体/枚举，`_cb_t` 回调 |
| **枚举值** | 大写蛇形 + 模块前缀（`MODULE_NAME_OK = 0`, 负数为错误） |
| **注释** | 公共 API + 复杂逻辑用中文 Doxygen（`@brief/@param/@return`） |
| **文件结构** | 头注释 → Includes → 私有常量 → 私有变量 → 私有原型 → 导出函数 → 私有函数 |
| **Config-in-context** | `xxx_config_t`（含回调）嵌入 `xxx_context_t`/handle，运行时状态分离 |
| **参数校验** | 公共函数首行校验 NULL / 未初始化 / 越界 |
| **内存** | 全程静态分配（无 malloc），调用者提供句柄内存 |
| **多实例** | 每个实例独立的缓冲区/队列放入句柄结构体，禁止模块级 static 单例 |
| **服务设计** | 风格 A（回调注入，共享总线/异构）或风格 B（直连 drv_，1:1 绑定）|

### 硬件 / 引脚参考

引脚功能汇总在 [agent_standards/HARDWARE_GUIDE.md](agent_standards/HARDWARE_GUIDE.md)，硬件配置以 `Core/Inc/main.h` 中的 CubeMX 宏为准。
