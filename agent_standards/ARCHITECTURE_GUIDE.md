# ARCHITECTURE_GUIDE.md — 分层架构规范

> 本文档描述 **E1_Master_Power_Manage** 项目的分层软件架构。供后续 AI / 开发者据此实现新功能，
> 确保新增代码与既有架构一致。**代码风格与命名细节见 [MODULE_CODING_GUIDE.md](MODULE_CODING_GUIDE.md)，本文档只讲分层职责与协作模式。**

---

## 1. 总览

STM32F407 裸机固件，无 RTOS，`sw_timer` 协作式单线程调度：

```
main() → app_main() → while(1) { sw_timer_task(); }
```

五层自顶向下，**依赖只能向下，禁止向上或跨层反向依赖**：

```
┌─────────────────────────────────────────────────────────────┐
│  tasks/           应用编排层    xxx_task_init() / sw_timer     │  ← 拥有 sw_timer、注入回调
├─────────────────────────────────────────────────────────────┤
│  service/         业务逻辑层    srv_*  FSM / 协议 / 算法        │  ← 硬件无关或绑定单一 driver
├─────────────────────────────────────────────────────────────┤
│  m_middlewares/   通用中间件    sw_timer/fsm/msg_fifo/滤波…    │  ← 平台无关、可复用、静态分配
├─────────────────────────────────────────────────────────────┤
│  device_drivers/  设备驱动层    drv_*  HAL 薄封装 + 引脚自包含  │  ← 唯一接触 main.h/CubeMX 句柄
├─────────────────────────────────────────────────────────────┤
│  Core/ (CubeMX)   HAL 生成层    main.h / hadcX / htimX / …     │  ← 只在 USER CODE 块内改
└─────────────────────────────────────────────────────────────┘
```

**依赖规则速查：**

| 层 | 可以 include | 禁止 include |
|----|-------------|-------------|
| tasks | service、device_drivers、middleware、Core | — |
| service | middleware、**其绑定的** device_drivers、Core 类型 | 其他 service（除非明确的数据依赖，如 srv_can_mst 内嵌 srv_can_dual 类型）、tasks |
| middleware | 其他 middleware、C 标准库 | device_drivers、service、tasks、Core/HAL |
| device_drivers | Core（main.h/hal/外设句柄）、middleware（如 kfifo） | service、tasks |

> 关键：**middleware 必须平台无关**（不含 `stm32f4xx_hal.h`）。**device_drivers 是唯一允许引用 `main.h` 引脚宏和 CubeMX 句柄（`hadc1`/`htim1`/`hcan1`…）的层。**

---

## 2. Core / HAL 层（CubeMX 生成）

- 由 STM32CubeMX 从 `.ioc` 生成，`MX_xxx_Init()` 在 `main.c` 中调用，随后调用 `app_main()`。
- **只在 `/* USER CODE BEGIN */ … /* USER CODE END */` 之间添加代码**，其余会被重新生成覆盖。
- 引脚宏定义在 `Core/Inc/main.h`（如 `LED_B_PB13_Pin` / `LED_B_PB13_GPIO_Port`）。
- 外设句柄为全局：`hadc1/2/3`、`htim1/6/10/11/12`、`hcan1`、`huart1`、`hspi1/3`。
- 中断入口在 `Core/Src/stm32f4xx_it.c`，HAL 回调（`HAL_xxx_Callback`）由 driver 层重写（弱符号覆盖）。

---

## 3. device_drivers 层（`drv_` 前缀）

**职责**：把 HAL 外设包装成语义化、面向逻辑通道的接口。**是唯一持有硬件配置（引脚表、句柄表、通道路由）的层。**

### 3.1 核心原则：硬件配置自包含

引脚/句柄配置**内置在 driver 的 `.c` 文件中**，`drv_xxx_init(void)` **一律无参**——**不接受**上层注入 HAL 句柄或引脚。driver 直接 `#include "main.h"` / `"tim.h"` / `"adc.h"` / `"can.h"` … 引用 CubeMX 宏和句柄。

**句柄内置有两种写法（按实例数量选择）：**

**(a) 单实例 → 宏定义收敛句柄。** 同一句柄被多处引用时，用一个宏收敛，换外设只改一行：

```c
/* drv_buzzer.c */
#define BUZZER_HTIM (&htim12)
#define BUZZER_CH   (TIM_CHANNEL_2)

/* drv_log_uart.c */
#define LOG_HUART (&huart1)   /* 全文只用 LOG_HUART / LOG_HUART->gState，杜绝散落的 &huart1 */
```

**(b) 多实例 → 内部句柄/配置表 `s_xxx[]`，`init(void)` 遍历初始化全部：**

```c
/* drv_fan.c —— 每通道 PWM + FG 硬件 */
static const drv_fan_hw_t s_hw[] = {
    { &htim10, TIM_CHANNEL_1, FAN0_FG_IO_Pin,  2 },   /* 通道0 */
    { &htim11, TIM_CHANNEL_1, FAN0_FG_IOE1_Pin, 2 },  /* 通道1 */
};
#define FAN_COUNT (sizeof(s_hw) / sizeof(s_hw[0]))

/* drv_can.c / drv_adc.c —— 纯句柄表 */
static CAN_HandleTypeDef*  const s_hcan[DRV_CAN_CH_NUM]  = { [DRV_CAN_CH_1] = &hcan1 };
static ADC_HandleTypeDef*  const s_adc_handles[DRV_ADC_INST_NUM] = { &hadc1, &hadc2, &hadc3 };

void drv_xxx_init(void)          /* 无参：遍历表启动全部外设 */
{
    for (uint32_t i = 0; i < XXX_COUNT; i++) { /* 启动外设 */ }
}
```

> **逐通道操作仍带 `ch`/`id`/`inst` 参数**（`drv_xxx_set_duty(id, …)`、`drv_can_send(ch, …)`、`drv_adc_read_raw(ch)`），但 **`init` 一律无参**。
> 无 init 的驱动例外：`drv_revision` 直接读 `main.h` 宏，无需初始化。

**全体驱动均为此形态**：`drv_adc`、`drv_fan`、`drv_can`、`drv_power`、`drv_status`、`drv_led`、`drv_buzzer`、`drv_rgb`、`drv_log_uart`、`drv_hw_timer` 都是 `drv_xxx_init(void)`。

### 3.2 逻辑通道枚举 + 路由表

driver 对外暴露**逻辑通道枚举**，内部路由到物理 (实例, 索引)。上层只认逻辑名，不关心物理分组：

```c
typedef enum { DRV_ADC_CH_VIN = 0, DRV_ADC_CH_NTC1, /* … */ DRV_ADC_CH_MAX } drv_adc_channel_t;
uint32_t drv_adc_read_raw(drv_adc_channel_t ch);   /* 内部查路由表 → dma_buf[idx] */
```

### 3.3 HAL 回调归属

HAL 弱回调（`HAL_ADC_ConvCpltCallback`、`HAL_GPIO_EXTI_Callback`、`HAL_CAN_RxFifo0MsgPendingCallback`…）**在对应 driver 的 `.c` 中重写**，通过句柄/引脚反查逻辑通道，再调用注册的回调或更新内部状态。中断上下文只做最小工作。

### 3.4 生命周期与错误码

```c
drv_xxx_error_t drv_xxx_init(void);        /* 无参；DRV_XXX_OK=0, 负数为错误 */
void            drv_xxx_deinit_all(void);   /* 多实例：一次性反初始化全部 */
bool            drv_xxx_is_initialized(drv_xxx_ch_t ch);
```

- `init` 无参、幂等；`deinit_all` 停止全部实例（单实例驱动可命名 `drv_xxx_deinit(void)`）。
- HAL 句柄若被多处引用，收敛为一个宏（见 §3.1(a)），**禁止在 `.c` 中散落 `&hxxx`**。

---

## 4. m_middlewares 层（无前缀，模块名即命名空间）

**职责**：平台无关的可复用库。**绝不引用 HAL / 引脚 / 具体外设。** 通过回调注入获取外部能力（如时间戳）。

### 4.1 现有原语

| 模块 | 作用 |
|------|------|
| `sw_timer` | 软件定时器，3 优先级（HIGH 在 SysTick ISR、NORMAL/LOW 在主循环派发），静态分配 |
| `fsm` | 扁平转移矩阵状态机（`state_count²` guard 表）+ entry/exit 回调 + user_data |
| `daemon` | 任务看门狗：注册名+超时+掉线回调，周期 `daemon_reload()` 喂狗 |
| `event` | ISR 安全的 32-bit 事件标志，ISR↔主循环生产者消费者 |
| `msg_fifo` | 建立在 `kfifo` 上的定长消息队列，单生产者单消费者 |
| `kfifo` | 无锁 2 的幂字节 FIFO |
| `clist` | 侵入式循环双向链表（实例管理） |
| `algorithm/` | PID、PLL、滤波（pt1）、CRC、math |

- 通过 [m_middlewares/public.h](m_middlewares/public.h) 统一导出。
- 时间等外部依赖用回调注入：`srv_led_init(led_get_time_cb_t)`、`log_init({.get_timestamp_cb=millis})`。

---

## 5. service 层（`srv_` 前缀）

**职责**：业务逻辑——协议打包/解析、FSM 时序、控制算法、状态聚合。**不拥有 sw_timer**，由 task 层周期驱动。

### 5.1 两种硬件解耦风格（按耦合度选择）

**风格 A：回调注入（硬件无关）** — 当服务管理的资源是**共享总线**或**多个异构实例**时使用。服务只持有函数指针，不 include 任何 driver。

```c
typedef bool (*srv_can_mst_send_cb_t)(uint16_t id, const uint8_t* d, uint8_t len);
typedef struct {
    srv_can_mst_read_cb_t read_data;    /* 数据来源回调 */
    srv_can_mst_send_cb_t send_frame;   /* 硬件输出回调 */
} srv_can_mst_config_t;
srv_can_mst_error_t srv_can_mst_init(const srv_can_mst_config_t* config);
```
适用：`srv_can_mst`、`srv_can_slv`、`srv_can_dual`（共享 CAN 总线）、`srv_led`（write_pin 回调，一个服务管多路 LED）。

**风格 B：直连驱动（1:1 绑定）** — 当服务与某个 driver 一一对应时，直接调用 `drv_`：

```c
#include "drv_power.h"
void srv_pwr_ctrl_step(uint16_t elapsed_ms) { /* … */ drv_power_set(DRV_POWER_RAIL_AUX_EN, true); }
```
适用：`srv_pwr_ctrl`↔`drv_power`、`srv_pwr_det`↔`drv_status`、`srv_fan_ctrl`↔`drv_fan`、`srv_adc`↔`drv_adc`。

> **选择规则**：资源被多方复用 / 服务需可移植可测试 → 风格 A；服务是某外设的专属业务包装 → 风格 B。

### 5.2 步进式驱动（Step-based ticking）

服务暴露 `srv_xxx_step(uint16_t elapsed_ms)` 或 `srv_xxx_task(void)`，由 task 层 sw_timer 回调周期调用。**服务内部不创建 sw_timer。**

### 5.3 多实例服务

一个服务管理多个实例时（如 `srv_led`），采用：**调用者静态分配句柄 + 每实例自带资源 + clist 链表串联**。

```c
struct led_handle {
    srv_led_config_t config;        /* 配置副本 */
    clist_head_t     node;          /* 链表节点 */
    fsm_t            fsm;           /* 每实例 FSM */
    msg_fifo_t       cmd_fifo;      /* ★每实例独立队列（禁止用全局共享！） */
    uint8_t          cmd_buffer[SRV_LED_CMD_BUF_SIZE];
    /* …运行时状态… */
};
srv_led_error_t srv_led_register_static(const srv_led_config_t* cfg, srv_led_handle_t* inst);
```

> **多实例陷阱**：per-instance 的缓冲区/队列必须放进句柄结构体，**绝不能用模块级 `static` 单例**——否则多实例互相踩踏。（本项目曾因 `static msg_fifo_t s_cmd_fifo` 被所有 LED 共享而出错。）

### 5.4 单例服务

只有一个逻辑实例的服务（`srv_pwr_ctrl`、`srv_pwr_det`、`srv_fan_ctrl`、`srv_adc`）用**模块级 static 状态**，`srv_xxx_init()` 无需句柄参数。

---

## 6. tasks 层（`xxx_task` 命名）

**职责**：应用编排。把 service 接到硬件、注册 sw_timer 周期、提供桥接回调。**薄封装，不含业务逻辑。**

### 6.1 标准结构

```c
/* xxx_task.c */
#include "xxx_task.h"
#include "srv_xxx.h"        /* 业务服务 */
#include "drv_yyy.h"        /* 若需桥接回调 */
#include "sw_timer.h"

#define TASK_PERIOD_MS (10U)
static sw_timer_t s_timer;

static void xxx_timer_cb(void* user_data)   /* sw_timer 回调 → 驱动 service 步进 */
{
    (void)user_data;
    srv_xxx_step(TASK_PERIOD_MS);
}

void xxx_task_init(void)
{
    drv_yyy_init();                          /* 1. 初始化依赖的驱动 */
    srv_xxx_init(/* 注入回调或无参 */);       /* 2. 初始化服务 */

    const sw_timer_config_t cfg = {          /* 3. 注册周期定时器 */
        .priority = SW_TIMER_PRIO_NORMAL,
        .callback = xxx_timer_cb,
        .user_data = NULL,
    };
    sw_timer_init(&s_timer, &cfg);
    sw_timer_start(&s_timer, TASK_PERIOD_MS, 0);
}
```

### 6.2 桥接回调（连接风格 A 服务 ↔ 驱动）

task 提供 `static` 桥接函数，把服务的抽象回调映射到具体 driver：

```c
static bool can_send_frame(uint16_t id, const uint8_t* d, uint8_t len)
{
    drv_can_msg_t m = { .id = id, .dlc = len };
    memcpy(m.data, d, len);
    return drv_can_send(DRV_CAN_CH_1, &m) == DRV_CAN_OK;
}
/* init: srv_can_mst_init(&(srv_can_mst_config_t){ .send_frame = can_send_frame, … }); */

static void led_blue_write_pin(uint16_t duty) { drv_led_set_duty(DRV_LED_CH_BLUE, duty); }
/* init: srv_led_register_static(&(srv_led_config_t){ .write_pin = led_blue_write_pin, … }); */
```

### 6.3 初始化顺序（关键）

在 [app_main.c](tasks/app_main.c) 中按依赖顺序调用，**顺序错误会导致回调未注册时被触发**：

```
delay_init()        → SysTick 时基
log_task_init()     → UART DMA 日志
can_task_init()     → CAN（须早于 power_task，注册 read_data 回调）
fan_task_init()     → 风扇
led_task_init()     → LED
sample_task_init()  → ADC 采样
power_task_init()   → 电源时序
```

---

## 7. 跨层通用约定

| 主题 | 规则 |
|------|------|
| **命名前缀** | driver=`drv_`，service=`srv_`，task=`xxx_task_`，middleware=模块名。类型 `_t`、回调 `_cb_t`、枚举值 `MODULE_UPPER`。Include guard `__MODULE_H`。 |
| **静态分配** | 全程静态分配，调用者提供句柄内存，模块内部不 malloc。 |
| **Config-in-context** | 每模块 `xxx_config_t`（init 时定，含回调）嵌入 `xxx_context_t`/handle（运行时态）。访问 `ctx->config.field`。 |
| **错误码** | `MODULE_OK = 0` 或正数，错误为负；提供 `MODULE_IS_OK/ERR` 宏。 |
| **参数校验** | 公共函数首行校验 NULL / 未初始化 / 越界。 |
| **数据快照** | ISR 更新的数据用快照读接口（如 `srv_can_dual_get_snapshot()`）供主循环无锁读取。 |
| **中文注释** | 公共 API + 复杂逻辑用中文 Doxygen（`@brief/@param/@return`）。 |

---

## 8. 新增功能决策树

> **“我要加一个 XXX 功能，代码放哪层？”**

1. **要读写具体引脚/外设寄存器/HAL 句柄？** → 写在 `device_drivers/drv_xxx`。引脚配置内置，暴露逻辑通道 API。
2. **是纯算法/数据结构，与硬件无关且可复用？** → 放 `m_middlewares/`。
3. **是业务逻辑（协议、时序 FSM、状态聚合、控制律）？** → 写在 `service/srv_xxx`：
   - 资源共享/需可移植 → 风格 A（回调注入）
   - 专属某 driver → 风格 B（直连 drv_）
   - 暴露 `srv_xxx_step()/_task()`，不自建 sw_timer。
4. **是把上述接起来 + 定周期跑？** → 写在 `tasks/xxx_task`：注册 sw_timer、提供桥接回调、在 app_main 按序 init。

### 端到端示例：新增“湿度传感器上报”

```
device_drivers/drv_humid.c   ── drv_humid_init(void); uint16_t drv_humid_read_raw();  (读 ADC/I2C，句柄用宏或内部表)
service/srv_humid.c          ── srv_humid_init(); srv_humid_step(ms); srv_humid_get(%);  (原始值→%RH 换算+滤波，风格B直连 drv_humid)
tasks/humid_task.c           ── humid_task_init(): sw_timer 周期调 srv_humid_step()（drv 由 srv 内部初始化）
tasks/app_main.c             ── 在合适顺序加入 humid_task_init();
（若需 CAN 上报：在 can_task 的 read_status 回调里调 srv_humid_get() 填入上报结构体）
```

---

## 9. 文件模板骨架

**driver（自包含型）**：`drv_xxx.h`（逻辑通道枚举 + **无参** `drv_xxx_init(void)` + 读写 API）/ `drv_xxx.c`（单实例用 `#define XXX_HTIM (&htimN)` 宏 / 多实例用 `s_hw[]`/`s_hcan[]` 表 + HAL 回调重写）。

**service（风格 A）**：`srv_xxx.h`（`_config_t` 含回调 + 数据类型 + init/step/process API）/ `srv_xxx.c`（回调注入，不含 HAL）。

**service（风格 B）**：`srv_xxx.h`（数据类型 + init/step/get API）/ `srv_xxx.c`（`#include "drv_xxx.h"` 直连）。

**task**：`xxx_task.h`（仅 `void xxx_task_init(void);`）/ `xxx_task.c`（sw_timer + 桥接回调，见 §6.1）。

> 具体代码格式（缩进、大括号、注释、init/deinit 写法）**严格遵循 [MODULE_CODING_GUIDE.md](MODULE_CODING_GUIDE.md)**。
