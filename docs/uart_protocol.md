以下是为您优化后的关节模组通讯协议说明。此版本已去除了更新记录、上位机UI操作指南等非协议内容，修正了所有排版与乱码问题，仅保留**协议内容、指令格式、配置参数定义、具体交互示例**及**参考代码实现**。

---

# 关节模组通讯协议 (V2.0.5)

## 1. 通讯物理层定义

* **通讯接口**：异步串口 (UART)
* **波特率**：**`230400 bps`** (可配置)
* **数据格式**：数据位 8，停止位 1，无奇偶校验 (8N1)
* **字节序**：多字节整型数据采用 **Intel 格式** (小端模式，低字节在前)

---

## 2. 协议数据包结构

协议数据包采用定长 **20 字节** 封包。

```c
typedef struct {
    uint32_t head;        // 协议头，固定为 0x1400AA55
    uint32_t can_id;      // 功能 ID
    uint8_t  can_data[8]; // 数据段
    uint32_t crc;         // CRC16_CCITT_FALSE 校验值 (高2字节补零)
} UART_Frame_t;
```

### 字段说明

| 字段 | 大小 (Bytes) | 说明 |
| :--- | :---: | :--- |
| **`head`** | 4 | 帧头。固定为 `0x1400AA55`（对应字节流：`55 AA 00 14`） |
| **`can_id`** | 4 | 功能 ID（详见下方列表） |
| **`can_data`**| 8 | 数据载荷 |
| **`crc`** | 4 | 校验值。基于 `can_id` 与 `can_data` 计算。高 2 字节固定补 `0` |

### 功能 ID 列表 (`can_id`)

| `can_id` (Hex) | 方向 | 功能定义 | `can_data` 载荷 |
| :--- | :---: | :--- | :--- |
| **`0x00000000` ~ `0x0000009F`** | TX | 单节点运动控制 | 目标位置、目标速度、控制命令 |
| **`0x000000FF`** | TX | 广播运动控制 (全节点响应) | 同上 |
| **`0x000000A0`** | TX | 配置参数写入 / 查询请求 | 设备 ID、配置项索引 (`index`)、配置值 |
| **`0x000000F0`** | TX | JOG 点动控制 (重置当前点为零位) | 设备 ID、点动方向 |
| **`0x00010000` ~ `0x009FFF00`** | RX | 节点状态 / 配置查询回复 | 状态信息或参数回复载荷 |

---

## 3. 指令格式与载荷定义

### 3.1 运动控制指令 (`can_id`: `0x00000000` ~ `0x0000009F` / `0x000000FF`)

* **数据段映射关系**：

| `data[7]` | `data[6]` | `data[5]` | `data[4]` | `data[3]` | `data[2]` | `data[1]` | `data[0]` |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| RSVD (0) | **`EN`** (使能) | **`cur_ref` (高字节)** | **`cur_ref` (低字节)** | **`spd_ref` (高字节)** | **`spd_ref` (低字节)** | **`pos_ref` (高字节)** | **`pos_ref` (低字节)** |

* **控制模式与有效载荷对应表**：

| 控制模式 | `cur_ref` | `spd_ref` | `pos_ref` |
| :--- | :---: | :---: | :---: |
| **位置模式** | 无效 | 有效 (限制速度) | 有效 |
| **速度模式** | 无效 | 有效 | 无效 |
| **电流模式** | 有效 | 有效 (限制转速) | 无效 |

* **参数定义**：
  * **`pos_ref`**: 目标角度，Q7 格式。数值为 $\text{角度(度)} \times 128$。不溢出范围为 $-255^\circ \sim +255^\circ$。
  * **`spd_ref`**: 速度设置，Q15 格式。$0 \sim 32767$ 对应相对于基准速度（$50000\text{ rpm}$，未经减速比）的 $0\% \sim 100\%$。
  * **`cur_ref`**: 电流设置，Q15 格式。$0 \sim 32767$ 对应电流 $0 \sim 5.625\text{ A}$。
  * **`EN` (使能状态)**: 
    * `1`: `CMD_ENABLE` (使能并上电锁定电机)
    * `2`: `CMD_DISABLE` (去使能并释放电机)
    * `0` / `3`: `CMD_SET_REF` (仅下发目标值)
    * 其他值：不执行操作

> **注意**：每次上电后第一次执行使能 (`EN = 1`) 时，电机需要约 1 秒进行编码器定位校准，此时请务必保持关节处于无负载悬空状态。

---

### 3.2 JOG 点动指令 (`can_id`: `0x000000F0`)

* **数据段映射关系**：

| `data[7]` | `data[6]` | `data[5]` | `data[4]` | `data[3]` | `data[2]` | `data[1]` | `data[0]` |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| RSVD1 (0) | RSVD1 (0) | RSVD1 (0) | RSVD1 (0) | RSVD1 (0) | **`dir`** (点动) | RSVD0 (0) | **`dev_id`** (设备ID) |

* **点动方向定义 (`dir`)**：
  * `0`: 无动作
  * `1`: 顺时针 (CW)，位置 $+5^\circ$
  * `2`: 逆时针 (CCW)，位置 $-3^\circ$
  *(执行点动后，会将动作终止位置重置为该电机的逻辑零位)*

#### 点动指令发送示例 (以节点 ID 为 2 为例)

* **节点 2 顺时针点动：**
  `55 AA 00 14 F0 00 00 00 02 00 01 00 00 00 00 00 EE 60 00 00`
* **节点 2 逆时针点动：**
  `55 AA 00 14 F0 00 00 00 02 00 02 00 00 00 00 00 0E AE 00 00`

---

### 3.3 参数读写及状态查询指令 (`can_id`: `0x000000A0`)

### 3.3.1 请求帧数据段映射关系 (`can_data[8]`)

| `data[7]` | `data[6]` | `data[5]` | `data[4]` | `data[3]` | `data[2]` | `data[1]` | `data[0]` |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| `value` (Byte 3) | `value` (Byte 2) | `value` (Byte 1 / `Value_H`) | `value` (Byte 0 / `Value_L`) | RSVD (0) | RSVD (0) | **`index`** <br>(配置索引) | **`dev_id`** <br>(目标设备 ID) |

* **配置写入值 (`value`) 字节序**：采用 **Intel 格式** (小端模式，低字节在前)，即低字节占用 `data[4]`，高字节占用 `data[5]` 往后延伸。
*(若执行的是查询/读取操作，`value` 填充为 0)*

#### 3.3.2 应答帧格式 (从机回复 RX)

当节点做出配置或状态响应时，其回复的 `can_id`（由4字节组成，低字节在前）的结构定义如下：

| `byte[3]` (高) | `byte[2]` | `byte[1]` | `byte[0]` (低) |
| :---: | :---: | :---: | :---: |
| `0` | **`index`** (配置项索引) | **`dev_id`** (回复节点ID) | `0` |

#### 3.3.3 典型配置项 (`index`) 及载荷定义

##### ① `index = 1` : 读取运行状态与故障码 (`CONFIG_INFO_01_R`)

* **应答载荷数据段 (`data[8]`) 结构**：

| `data[7]` | `data[6]` | `data[5]` | `data[4]` | `data[3]` | `data[2]` | `data[1]` | `data[0]` |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **`Vbus` (高)** | **`Vbus` (低)** | **`angle_fb` (高)** | **`angle_fb` (低)** | **`temp`** | **`soft_ver`** | **`err_code`** | **`fsm`** |

* **状态机定义 (`fsm`)**：
  `0`: `mcReady` (就绪) | `5`: `mcAlign` (预定位中) | `7`: `mcRun` (运行中) | `9`: `mcFault` (发生故障)
* **故障码定义 (`err_code`)**：
  `0`: 无故障 | `1`: 硬件过流 | `2`: 软件过流 | `3`: 硬件过压 | `4`: 软件过压 | `5`: 硬件欠压 | `6`: 软件欠压 | `7`: 缺相 | `8`: 电机堵转 | `9`: 过温保护 | `10`: 串口通信丢失 | `11`: 内部自检故障
* **参数还原方法**：
  * **MCU 温度 (`temp`)**: 物理温度 ($^\circ\text{C}$) $= temp - 50$
  * **手指角度 (`angle_fb`)**: 物理角度 ($^\circ$) $= \text{Q7还原} = (data[5] \ll 8 \mid data[4]) \times 0.0078125$
  * **母线电压 (`Vbus`)**: 电压 ($\text{V}$) $= (data[7] \ll 8 \mid data[6]) \times 0.0078125$

* **状态读取示例 (读取节点 2，`index = 1`)**：
  * **主机发送请求：**
    `55 AA 00 14 A0 00 00 00 02 01 00 00 00 00 00 00 3F 77 00 00`
  * **从机应答数据：**
    `55 AA 00 14 00 01 02 00 07 00 67 02 AC 1F B8 0B AC 95 00 00`
    *解析*：应答帧中的 `can_id` 为 `0x00010200`（即 `index = 1`, `dev_id = 2`），`fsm = 7` (运行)，`err_code = 0` (无故障)。

---

##### ② `index = 2` : 读取运行指标 (`CONFIG_INFO_02_R`)

* **应答载荷数据段 (`data[8]`) 结构**：

| `data[7]` | `data[6]` | `data[5]` | `data[4]` | `data[3]` | `data[2]` | `data[1]` | `data[0]` |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **`angle_fb` (高)** | **`angle_fb` (低)** | **`speed` (高)** | **`speed` (低)** | **`Q_cur` (高)** | **`Q_cur` (低)** | **`D_cur` (高)** | **`D_cur` (低)** |

* **参数定义**：
  * `D_cur`, `Q_cur`: 电极直轴、交轴工作电流。Q15 格式（$0 \sim 32767$ 对应 $0 \sim 5.625\text{A}$）。
  * `speed`: 速度反馈。Q15 格式（$0 \sim 32767$ 对应转速 $0 \sim 50\text{ krpm}$）。
  * `angle_fb`: 末端关节绝对反馈角度。Q7 格式。

---

##### ③ `index = 4` : 在线设置节点 ID (`CONFIG_SET_ID`)

* **发送请求载荷结构**：

| `data[7]` | `data[6]` | `data[5]` | `data[4]` | `data[3]` | `data[2]` | `data[1]` | `data[0]` |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| RSVD | RSVD | **`Value_H`** | **`Value_L`** | RSVD | RSVD | **`0x04`** | **`dev_id`** (原ID) |

* **参数定义**：
  将新 ID 转化为 Q7 格式赋于 `Value`。例如设置新节点 ID 为 `10`：
  $\text{值} = 10 \ll 7 = 1280 \Rightarrow \text{Value\_H} = \text{0x05}, \text{Value\_L} = \text{0x00}$。
* **应答载荷格式**：
  `data[0]` 返回修改后的 `res` 状态（`1`: 成功；`0`: 失败）。该修改立即写入 FLASH 且断电不丢失。

---

##### ④ `index = 8 ~ 11` : 设置环路 PID 增益

* **参数控制表**：

| 配置指令索引 | `index` | 数值格式 | 数值范围 | 对应实际物理增益 |
| :--- | :---: | :---: | :---: | :--- |
| **位置环比例系数 (`KP`)** | `8` | Q12 | $0 \sim 32767$ | $0.000 \sim 8.000$ |
| **位置环微分系数 (`KD`)** | `9` | Q12 | $0 \sim 32767$ | $0.000 \sim 8.000$ |
| **速度环比例系数 (`KP`)** | `10`| Q12 | $0 \sim 32767$ | $0.000 \sim 8.000$ |
| **速度环积分系数 (`KI`)** | `11`| Q15 | $0 \sim 32767$ | $0.000 \sim 1.000$ |

---

##### ⑤ `index = 13` : 保存参数到 FLASH (`CONFIG_WRITE_FLASH`)

修改完除 ID 外的其他运行参数（如 PID、平滑系数等）后，必须发送该指令以进行永久固化，否则断电复位后参数将重置为旧配置。

* **发送请求载荷结构**：

| `data[7]` | `data[6]` | `data[5]` | `data[4]` | `data[3]` | `data[2]` | `data[1]` | `data[0]` |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| RSVD (0) | RSVD (0) | RSVD (0) | **`0xA5`** (KEY) | RSVD (0) | RSVD (0) | **`13`** | **`dev_id`** |

---

## 4. 特殊同步广播控制协议

特殊广播帧设计用于通过单次传输同时、快速地调整多路节点。
广播帧不包含 `can_id`，而是通过两个特殊定义的 **2 字节标识头** 在帧前进行区分，整体包长同样保持为 20 字节。

```c
typedef struct {
    uint16_t head2;     // 广播类型标志头
    int16_t  value[8];  // 8路节点的数据载荷区 (小端格式)
    uint16_t crc16;     // CRC16_CCITT_FALSE 校验值
} UART_Frame2_t;
```

### 广播类型标志头与载荷功能定义

| `head2` (Hex) | 负载数据属性 (`value[8]`) | 控制节点物理范围 | 说明 |
| :--- | :--- | :--- | :--- |
| **`0xAB55`** | 位置增量设定值 (Q7格式) | 节点 `ID1` ~ `ID8` | 1. 角度范围：$-255^\circ \sim +255^\circ$<br>2. 每次重新广播使能，电机都会产生自校准动作。 |
| **`0xAC55`** | 位置增量设定值 (Q7格式) | 节点 `ID9` ~ `ID16` | 同上 |
| **`0xAD55`** | 目标速度设定值 (Q15格式) | 节点 `ID1` ~ `ID8` | $0 \sim 32767$ 对应转速 $0\% \sim 100\%$ |
| **`0xAE55`** | 目标速度设定值 (Q15格式) | 节点 `ID9` ~ `ID16` | 同上 |
| **`0xB155`** | 节点状态使能配置块 | 节点 `ID1` ~ `ID16` | 此时数据段强制转换为 $16$ 字节 `uint8_t value[16]`，对应：<br>- `1`: `CMD_ENABLE` (启动并锁定)<br>- `2`: `CMD_DISABLE` (释放电机)<br>- 其他: 保持原状态 |

#### 广播指令发送示例

* **设置 ID1 ~ ID8 位置为 90度：**
  `55 AB 00 2D 00 2D 00 2D 00 2D 00 2D 00 2D 00 2D 00 2D B6 67`
  *解析*：目标 Q7 设定值 $= 90 \times 128 = 11520$ (对应 Hex 为 `0x2D00`)。

---

## 5. 参考驱动代码示例 (C 语言)

以下提供了完整的通讯包组装、使能切换、角度控制下发以及 `CRC16_CCITT_FALSE` 校验计算的软件实现：

```c
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#define TARGET_DEVICE_ID          (0x01)         // 目标执行节点 ID
#define JOG_ID                    (0xF0)         // 点动指令 ID
#define CONFIG_FUNC_ID            (0xA0)         // 参数配置与查询功能 ID
#define UART_PACKGE_HEAD          (0x1400AA55)   // 定长帧头

#define CMD_NONE                  (0)
#define CMD_ENABLE                (1)            // 使能上电
#define CMD_DISABLE               (2)            // 释放去使能
#define CMD_SET_REF               (3)            // 下发控制值

#pragma pack(push, 1) // 保证编译器执行 1 字节对齐
typedef struct {
    uint32_t head;        // 帧头
    uint32_t can_id;      // 功能 ID
    uint8_t  can_data[8]; // 数据载荷段
    uint32_t crc;         // CRC校验 (高2字节置0)
} UART_Frame_t;
#pragma pack(pop)

// 外部硬件串口发送接口
extern void UART_Send(uint8_t* pBuf, uint16_t size);

/**
 * @brief CRC16_CCITT_FALSE 算法实现
 */
uint16_t CRC16_CCITT_FALSE(uint8_t data[], int offset, int length)
{
    const uint16_t polynomial = 0x1021;
    uint16_t crc = 0xFFFF;
    for (int i = offset; i < offset + length; i++)
    {
        crc ^= (uint16_t)(data[i] << 8);
        for (int j = 0; j < 8; j++)
        {
            if ((crc & 0x8000) != 0)
                crc = (uint16_t)((crc << 1) ^ polynomial);
            else
                crc <<= 1;
        }
    }
    return crc;
}

/**
 * @brief 节点驱动使能与释放控制
 * @param p 帧结构体指针
 * @param en true: 使能并锁定; false: 解锁脱机
 */
void set_enable(UART_Frame_t* p, bool en)
{
    memset(p, 0, sizeof(UART_Frame_t));
    p->head = UART_PACKGE_HEAD;
    p->can_id = TARGET_DEVICE_ID;
    
    // 配置使能/关闭标志
    p->can_data[6] = en ? CMD_ENABLE : CMD_DISABLE;

    // 校验范围包含 can_id 与 8 字节 data 载荷段
    uint16_t cal_crc = CRC16_CCITT_FALSE((uint8_t*)&p->can_id, 12);
    p->crc = (uint32_t)cal_crc; // 隐式完成高2字节补0动作

    UART_Send((uint8_t*)p, sizeof(UART_Frame_t));
}

/**
 * @brief 角度定位驱动控制 (位置控制模式)
 * @param p 帧结构体指针
 * @param pos_deg 目标执行角度 (支持负角，范围-255.0 ~ 255.0)
 */
void set_pos(UART_Frame_t* p, float pos_deg)
{
    memset(p, 0, sizeof(UART_Frame_t));
    p->head = UART_PACKGE_HEAD;
    p->can_id = TARGET_DEVICE_ID;

    // 1. 目标角度转换 (Q7 格式)
    int16_t pos_ref = (int16_t)(pos_deg * 128.0f);
    p->can_data[0] = (uint8_t)pos_ref;
    p->can_data[1] = (uint8_t)(pos_ref >> 8);

    // 2. 目标运行限制速度设置 (默认给Q15半速值：16384)
    uint16_t spd_ref = 16384;
    p->can_data[2] = (uint8_t)spd_ref;
    p->can_data[3] = (uint8_t)(spd_ref >> 8);

    // 3. 电流限制值 (不限制，设置为 0)
    p->can_data[4] = 0;
    p->can_data[5] = 0;

    // 4. 下发目标运动模式
    p->can_data[6] = CMD_SET_REF;

    // 5. 生成校验并发送
    uint16_t cal_crc = CRC16_CCITT_FALSE((uint8_t*)&p->can_id, 12);
    p->crc = (uint32_t)cal_crc;

    UART_Send((uint8_t*)p, sizeof(UART_Frame_t));
}
```