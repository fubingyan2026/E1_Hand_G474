# m_middlewares — 平台无关中间件层

本目录包含所有与硬件解耦的可复用库，遵循五层架构中的 **Middleware** 层。所有模块均采用**静态分配**模式，不依赖 HAL/OS，可跨 STM32 平台移植。

## 目录结构

```
m_middlewares/
├── public.h              ← 统一入口头文件，聚合所有模块
├── algorithm/            ← 控制算法与数字信号处理
├── framework/            ← 基础运行时框架（定时器/状态机/看门狗/消息队列）
├── utils/                ← 数据结构基础设施（链表/环形缓冲）
├── log/                  ← 日志系统
├── key_base/             ← 按键检测（单击/双击/长按）
├── protocol_tools/       ← 通用协议组包/解包
└── Third_Party/          ← 第三方开源库
```

## 模块速查表

### framework/ — 运行时框架

| 模块 | 文件 | 依赖 | 说明 |
|------|------|------|------|
| **sw_timer** | `framework/sw_timer.h` | `utils/clist.h` | 三级优先级软件定时器，H/NORMAL/LOW。HIGH 在 SysTick ISR 中触发，其余在主循环分发。支持单次/周期/精确次数。 |
| **fsm** | `framework/fsm.h` | 无 | 表驱动状态机。`state_count × state_count` 扁平转移矩阵，guard 条件函数，entry/exit 回调。**未纳入 public.h**。 |
| **daemon** | `framework/daemon.h` | `utils/clist.h` | 任务看门狗。静态注册任务名 + 超时 + 离线回调，`daemon_reload()` 喂狗。含启动初始等待时间、滞回去抖。 |
| **event** | `framework/event.h` | 无 | ISR 安全 32 位事件标志。`event_send()` 可在中断中调用，主循环 `event_recv()` 轮询。 |
| **msg_fifo** | `framework/msg_fifo.h` | `utils/kfifo.h` | 类型化消息队列。封装 kfifo 为逐元素 push/pop，单生产者单消费者无锁。 |

### algorithm/ — 算法库

| 模块 | 文件 | 说明 |
|------|------|------|
| **pid** | `algorithm/controller/pid.h` | 通用 PID，支持位置式/增量式双模式。config/context 分离模式。 |
| **gimbal_pid** | `algorithm/controller/gimbal_pid.h` | 云台串级 PID。角速度由陀螺仪硬件注入 derivative，反馈更新与计算分离。 |
| **mit** | `algorithm/controller/mit.h` | MIT 阻抗控制：`τ = Kp·Δpos + Kd·Δvel + ff`。**未纳入 public.h**。 |
| **pll** | `algorithm/pll/pll.h` | 锁相环，从正交信号（Sin/Cos 编码器）估算角度/角速度。内部使用 PT1 滤波。依赖 `filter.h`。 |
| **filter** | `algorithm/filter/filter.h` | PT1 低通、双二阶 IIR（LPF/Notch/BPF）、Slew Rate 限幅、延迟移动平均。源自 Betaflight。 |
| **crc** | `algorithm/crc.h` | CRC8/CRC16 计算/校验/追加三合一。源自 DJI RoboMaster。 |
| **maths** | `algorithm/math/maths.h` | 通用数学：3D 向量/旋转矩阵、中值滤波、标准差、快速三角函数近似、Q12 定点。源自 Betaflight。 |
| **utils** | `algorithm/math/utils.h` | 预处理宏：`container_of`、`ARRAYLEN`、`STATIC_ASSERT`、`cmp16/32`（饱和无符号→有符号比较）。 |
| **utils_math** | `algorithm/math/utils_math.h` | VESC 数学工具：角度插值、FFT 特定 bin、锂电池容量估算、快速 LP 宏、矢量饱和。 |

### utils/ — 基础数据结构

| 模块 | 文件 | 说明 |
|------|------|------|
| **clist** | `utils/clist.h` | Linux 内核风格侵入式双向循环链表。嵌入用户结构体，零动态分配。被 daemon、sw_timer、key_base 共用。 |
| **kfifo** | `utils/kfifo.h` | 无锁环形缓冲区（2 的幂大小）。支持 DMA 同步（`kfifo_move_in`）、临界区保护。被 log、protocol_parser、msg_fifo 共用。 |

### 其他模块

| 模块 | 文件 | 依赖 | 说明 |
|------|------|------|------|
| **log** | `log/log.h` | `utils/kfifo.h` | ESP32 风格多级日志（E/W/I/D/T）。环形缓冲输出管线，支持 ANSI 彩色、时间戳、hexdump。通过 kfifo 缓冲后由应用层取出。 |
| **key_base** | `key_base/key_base.h` | `utils/clist.h` | 多实例按键检测。支持 DOWN/CLICK/DOUBLE/TRIPLE/REPEAT/LONG_HOLD/LONG_RELEASE。可配置去抖和时序窗口。 |
| **protocol_packer** | `protocol_tools/protocol_packer.h` | 无 | 组包器：`[HEADER][PAYLOAD][CHECKSUM][FOOTER]`。checksum 和长度字段通过回调注入。 |
| **protocol_parser** | `protocol_tools/protocol_parser.h` | `utils/kfifo.h` | 解包器：header/footer 匹配 + checksum 校验 + 空闲超时。内部使用 kfifo 缓冲。 |

### Third_Party/ — 第三方库

| 库 | 路径 | 用途 |
|----|------|------|
| **CmBacktrace** | `Third_Party/CmBacktrace/` | ARM Cortex-M 故障回溯。HardFault 时自动打印调用栈、寄存器、故障寄存器。输出通过 `log.h` 的 `LOG_I` 输出。 |
| **EasyFlash** | `Third_Party/easyflash/` | 嵌入式 KV 数据库，存 MCU 内部 Flash。通过 `ef_port.c` 移植，依赖 `drv_stm32f4_flash` 驱动。 |
| **LwMEM** | `Third_Party/lwmem/` | 轻量级动态内存管理，支持多 region。启用全功能模式（free/realloc）。 |
| **SEGGER RTT** | `Third_Party/SEGGER_RTT/` | J-Link 实时双向通信，6KB 上行缓冲。 |

## 核心依赖关系

```
public.h ── 统一入口，聚合以下全部（除标注"未纳入"的模块）

clist.h  ←── daemon, sw_timer, key_base
kfifo.h  ←── log, protocol_parser, msg_fifo
filter.h ←── pll.h
```

`clist` 和 `kfifo` 是整个中间件层的两块基石 — 几乎所有需要实例管理或数据缓冲的模块都依赖它们。

## 设计约定

- **config/context 分离**：`xxx_config_t` 在 init 时设定（只读），`xxx_context_t` 持有运行时状态。init 后通过 `ctx->config.field` 访问配置。
- **_t 后缀命名**：枚举和结构体统一以 `_t` 结尾，回调函数指针以 `_cb_t` 结尾。
- **错误码规范**：`MODULE_OK = 0`；错误为负数。
- **多实例安全**：所有状态字段存在 context 结构体中，`.c` 文件内不使用 `static` 全局变量。
- **静态分配**：调用方在任务层 `.c` 中以 `static` 声明所有 context 句柄。

## 使用方式

```c
// tasks 层直接包含统一入口即可：
#include "public.h"

// 如需使用未纳入 public.h 的模块：
#include "framework/fsm.h"          // 状态机
#include "algorithm/controller/mit.h" // MIT 控制器
```
