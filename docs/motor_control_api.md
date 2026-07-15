# E1_Hand 电机控制 API 参考手册

| 项目 | 信息 |
|------|------|
| **文档版本** | V1.0.0 |
| **作者** | maximillian |
| **创建日期** | 2026-07-14 |
| **最后更新** | 2026-07-14 |
| **目标 MCU** | STM32G474 (Cortex-M4 FPv4-SP, 512KB Flash / 128KB RAM) |
| **传输层** | UART 230400 bps 8N1 + DMA |
| **依赖协议** | [关节模组通讯协议 V2.0.5](uart_protocol.md) |

---

## 1. 架构概述

`srv_motor` 模块通过 2 路 UART 控制最多 9 个关节电机，每路 UART 一个 FSM 实例管理分时发送。

```
┌────────────┐    UART1     ┌──────────┐
│  srv_motor │ ──────────►  │ 组A (5电机)│  dev_id 1-5
│   (FSM)    │    UART2     │          │
│            │ ──────────►  │ 组B (4电机)│  dev_id 1-4
└────────────┘              └──────────┘
```

**设计原则表**：

| 原则 | 说明 |
|------|------|
| FSM 驱动 | 每路 UART 一个状态机，DMA 空闲时自动推进，无需外部定时 |
| 分时复用 | 广播帧 + 轮询帧 + 电流帧按固定顺序穿插，每条 UART 串行 |
| 校准等待 | ENABLE 后等待 ~1s 编码器自校准，期间不阻塞主循环 |
| 成对广播 | 速度广播需 0xAD55 + 0xAE55 成对发送才被电机接受 |
| CRC 区间 | 标准帧覆盖 can_id+data(12B)；广播帧仅覆盖 values(16B)，不含 head2(2B) |

---

## 2. FSM 状态机

### 2.1 状态转移路径

```
STARTUP ─► BCAST_EN ─► BCAST_POS ─► BCAST_SPD ─► POLL_1 ─► POLL_2 ─► BCAST_CUR ─► IDLE
    ↑                                                                                    │
    └──────────────────── 1 个周期完成后回到 IDLE 等待 ─────────────────────────────────┘
```

### 2.2 状态转移矩阵

| 从 → 到 | STARTUP | BCAST_EN | BCAST_POS | BCAST_SPD | POLL_1 | POLL_2 | BCAST_CUR | IDLE |
|---------|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|
| **STARTUP** | — | — | — | — | — | — | — | 校准完成 |
| **BCAST_EN** | — | DMA忙滞留 | 无脏标志 | — | — | — | — | — |
| **BCAST_POS** | — | — | — | 无条件 | — | — | — | — |
| **BCAST_SPD** | — | — | — | DMA忙滞留 | 发送完成 | — | — | — |
| **POLL_1** | — | — | — | — | DMA忙滞留 | 发送完成 | — | — |
| **POLL_2** | — | — | — | — | — | DMA忙滞留 | 发送完成 | — |
| **BCAST_CUR** | — | — | — | — | — | — | DMA忙滞留 | 无条件 |
| **IDLE** | — | 周期到 | — | — | — | — | — | 等待中 |

### 2.3 状态行为表

| 状态 | 执行条件 | 处理 | 下一状态 |
|------|---------|------|---------|
| `STARTUP` | step 0: DMA 空闲 | 发送 ENABLE 标准帧，记录时间 | STARTUP |
| `STARTUP` | step 1: 始终 | 等待 `millis() - enable_time >= 1000` | STARTUP / step 2 |
| `STARTUP` | step 2: DMA 空闲 | 发送 SET_MODE (index=7, value=3) | IDLE |
| `BCAST_EN` | mode_dirty + DMA 空闲 | 发送 SET_MODE 标准帧 | BCAST_EN (下轮取 en_dirty) |
| `BCAST_EN` | 无脏标志 | 跳过 | BCAST_POS |
| `BCAST_EN` | en_dirty + DMA 空闲 | 发送 0xB155 使能广播 | BCAST_POS |
| `BCAST_POS` | 始终 | 跳过（电流模式不广播位置） | BCAST_SPD |
| `BCAST_SPD` | 始终 | 跳过（电流模式不广播速度） | POLL_1 |
| `POLL_1/2` | DMA 空闲 | 发送 INFO_01_R 或 INFO_02_R 查询帧 | 下一个 |
| `BCAST_CUR` | 游标指向有效电机 + DMA 空闲 | 发送 CMD_SET_REF (spd+cur) 标准帧 | IDLE |
| `BCAST_CUR` | 游标指向空槽位 | 推进游标，不发帧 | IDLE |
| `BCAST_CUR` | DMA 忙 | **不推进游标**，下周期重试 | IDLE |
| `IDLE` | `millis()-cycle_start >= MOTOR_BCAST_PERIOD_MS` | 启动新周期 | BCAST_EN |

---

## 3. 控制模式

### 3.1 模式列表

| value | 模式名称 | 有效控制量 | `spd_ref` 含义 | `cur_ref` 含义 | `pos_ref` 含义 |
|:-----:|---------|:--:|------|------|------|
| `1` | 速度模式 | `spd_ref` | **目标转速** | 忽略 | 忽略 |
| `3` | 电流模式 | `cur_ref` | 限速（最大转速） | **目标电流** | 忽略 |

> **注意**: `value=2` 不是有效模式，电机不响应。value=0 可能为位置模式（未验证）。

### 3.2 模式切换 API

```c
// 切到速度模式
srv_motor_set_mode(srv_motor_get_handle(0), 1);

// 切到电流模式（默认）
srv_motor_set_mode(srv_motor_get_handle(0), 3);
```

> 调用后下个 FSM 周期自动发送 SET_MODE 配置帧（`can_id=0xA0`, `index=7`），与使能广播不冲突（分两轮发送）。

### 3.3 电流模式使能流程

```
ENABLE (标准帧) ───── 等待 1000ms 校准 ───── SET_MODE value=3 (标准帧) ─────► IDLE ─► BCAST_CUR 持续发送
```

### 3.4 模式切换帧格式

| Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 | Byte 5 | Byte 6 | Byte 7 |
|--------|--------|--------|--------|--------|--------|--------|--------|
| 0x55 | 0xAA | 0x00 | 0x14 | 0xA0 | 0x00 | 0x00 | 0x00 |
| `dev_id` | `0x07` | `0x00` | `0x00` | `value` | `0x00` | `0x00` | `0x00` |

| 字段 | 偏移 | 长度 | 说明 |
|------|------|------|------|
| `head` | 0 | 4 | 帧头 `0x1400AA55` (LE: `55 AA 00 14`) |
| `can_id` | 4 | 4 | `0x000000A0` (配置读写, LE) |
| `dev_id` | 8 | 1 | 目标设备 ID (1-based) |
| `index` | 9 | 1 | `0x07` = 控制模式 |
| `value` | 12 | 1 | 模式值: `1`=速度, `3`=电流 |
| `crc` | 16 | 2 | CRC16-CCITT-FALSE(can_id[4]+data[8], 12B), 低字节在前 |

---

## 4. 控制指令

### 4.1 命令字一览

| 命令字 | 宏定义 | 方向 | 说明 |
|--------|--------|------|------|
| `0x01` | `SRV_MOTOR_CMD_ENABLE` | H→M | 使能上电，锁定电机 |
| `0x02` | `SRV_MOTOR_CMD_DISABLE` | H→M | 去使能，释放电机 |
| `0x03` | `SRV_MOTOR_CMD_SET_REF` | H→M | 仅下发参考值（不改变使能状态） |

### 4.2 使能/去使能帧

| Byte 8 | Byte 9 | Byte 10 | Byte 11 | Byte 12 | Byte 13 | Byte 14 | Byte 15 |
|--------|--------|---------|---------|---------|---------|---------|---------|
| `pos_L` (0) | `pos_H` (0) | `spd_L` (0) | `spd_H` (0) | `cur_L` (0) | `cur_H` (0) | `EN` | RSVD (0) |

- `can_id = dev_id`（单节点控制）
- `EN = 1` 使能，`EN = 2` 去使能
- 其他字段填 0

### 4.3 电流控制帧 (CMD_SET_REF)

| Byte 8 | Byte 9 | Byte 10 | Byte 11 | Byte 12 | Byte 13 | Byte 14 | Byte 15 |
|--------|--------|---------|---------|---------|---------|---------|---------|
| 0 | 0 | `spd_L` | `spd_H` | `cur_L` | `cur_H` | `0x03` | RSVD (0) |

- `can_id = dev_id`（单节点控制）
- `spd_ref`：电流模式下限速值，Q15 格式
- `cur_ref`：目标电流，Q15 格式
- 每 FSM 周期持续发送，**不依赖 `cur_pending` 标志**

### 4.4 设置电流 API

```c
// 设置 cur_ref，BCAST_CUR 每周期自动下发
srv_motor_set_current(srv_motor_get_handle(0), cur_ref);
```

| `cur_ref` 值 | 实际电流 (Q15) | 适用场景 |
|:-----------:|------|------|
| 1000 | ~0.17A | 极轻负载 |
| 3000 | ~0.51A | 一般测试 |
| 8000 | ~1.37A | 中等扭矩 |
| 30000 | ~5.15A | 满扭矩 |

---

## 5. 启动序列

### 5.1 完整启动时序

```
主机                                    电机
 │                                       │
 │── ENABLE (CMD=1, can_id=dev_id) ───► │  mcReady → mcAlign
 │                                       │  (编码器校准 ~1s)
 │                                       │
 │·············· 等待 1000ms ············│
 │                                       │
 │── SET_MODE (index=7, value=3) ──────► │  设定电流模式
 │                                       │
 │── BCAST_CUR (CMD=3, spd+cur) ──────► │  mcRun → 执行电流
 │── BCAST_CUR (CMD=3, spd+cur) ──────► │  (每周期持续)
 │── POLL (index=1/2) ────────────────► │
 │◄─ 反馈 ───────────────────────────── │  angle/spd/cur
 │                                       │
```

### 5.2 启动参数配置

| 常量 | 默认值 | 定义位置 | 说明 |
|------|--------|----------|------|
| `MOTOR_BCAST_PERIOD_MS` | `100U` | `srv_motor.c` | 广播控制周期 (ms) |
| `SRV_MOTOR_GRPA_UART` | `DRV_UART_CH_1` | 编译宏 | 组A 串口通道 |
| `SRV_MOTOR_GRPB_UART` | `DRV_UART_CH_2` | 编译宏 | 组B 串口通道 |
| `POLL_01_INTERVAL` | `10U` | `srv_motor.c` | 每 N 次 INFO_02_R 插入 INFO_01_R |

---

## 6. 反馈读取

### 6.1 轮询查询帧

| Byte 8 | Byte 9 | Byte 10..15 |
|--------|--------|-------------|
| `dev_id` | `index` | 0x00 × 6 |

- `can_id = 0xA0` (CONFIG)
- `index = 1`: INFO_01_R (状态/故障/温度/电压)
- `index = 2`: INFO_02_R (角度/速度/DQ电流)
- 每 `POLL_01_INTERVAL` 次 INFO_02_R 插入一次 INFO_01_R

### 6.2 反馈解析

#### INFO_02_R (index=2)

| Byte 8 | Byte 9 | Byte 10 | Byte 11 | Byte 12 | Byte 13 | Byte 14 | Byte 15 |
|--------|--------|---------|---------|---------|---------|---------|---------|
| `D_cur_L` | `D_cur_H` | `Q_cur_L` | `Q_cur_H` | `speed_L` | `speed_H` | `angle_L` | `angle_H` |

- `D_cur`, `Q_cur`: Q15, 0~32767 → 0~5.625A
- `speed`: Q15, 0~32767 → 0~50 krpm
- `angle_fb`: Q7

#### INFO_01_R (index=1)

| Byte 8 | Byte 9 | Byte 10 | Byte 11 | Byte 12 | Byte 13 | Byte 14 | Byte 15 |
|--------|--------|---------|---------|---------|---------|---------|---------|
| `fsm` | `err_code` | `soft_ver` | `temp` | `angle_L` | `angle_H` | `vbus_L` | `vbus_H` |

- `fsm`: 0=mcReady, 5=mcAlign, 7=mcRun, 9=mcFault
- `temp`: 物理°C = raw - 50
- `vbus`: Q7, V = raw / 128

### 6.3 读取 API

```c
const srv_motor_feedback_t* fb = srv_motor_get_feedback(srv_motor_get_handle(0));
if (fb && fb->time_fb > 0) {
    // fb->angle_fb, fb->speed_fb, fb->q_cur, fb->d_cur (高频)
    // fb->fsm_state, fb->err_code, fb->temp, fb->vbus (低频)
}
bool online = srv_motor_is_online(srv_motor_get_handle(0));
```

---

## 7. 广播帧 CRC 说明

### 7.1 CRC 校验范围

| 帧类型 | 覆盖数据 | 字节数 | CRC 算法 |
|--------|---------|:---:|------|
| 标准帧 (`0x1400AA55`) | `can_id`(4B) + `can_data`(8B) | 12 | CRC16-CCITT-FALSE |
| 广播帧 (`0xAB55/AD55/AE55/B155`) | `value`(16B) **不含 head2** | 16 | CRC16-CCITT-FALSE |

> **验证方法**: `0xAD55` 和 `0xAE55` 在 value 相同时 CRC 应相等。

### 7.2 CRC 参数

| 参数 | 值 |
|------|-----|
| 多项式 | `0x1021` |
| 初始值 | `0xFFFF` |
| 反射输入 | 否 |
| 反射输出 | 否 |
| 异或输出 | `0x0000` |

---

## 8. 广播帧速查

| head2 | 宏 | 控制范围 | value 格式 | 说明 |
|-------|----|:---:|------|------|
| `0xAB55` | `SRV_MOTOR_BC_POS_ID18` | ID1~8 | Q7 × 8 | 位置增量广播 |
| `0xAD55` | `SRV_MOTOR_BC_SPD_ID18` | ID1~8 | Q15 × 8 | 速度广播 |
| `0xAE55` | — | ID9~16 | Q15 × 8 | 速度广播 (需与 AD55 成对) |
| `0xB155` | `SRV_MOTOR_BC_ENABLE` | ID1~16 | uint8 × 16 | 使能广播 |

---

## 9. API 速查

```c
/* 生命周期 */
srv_motor_init();      // 初始化：注册电机 + 设定初始值（FSM 由 srv_motor_step 驱动）
srv_motor_deinit();    // 反初始化

/* 控制 */
srv_motor_set_setpoint(inst, pos_ref, spd_ref, cur_ref);  // 设定位置/速度/电流
srv_motor_enable(inst, true/false);                         // 使能/去使能
srv_motor_set_current(inst, cur_ref);                       // 设定电流（触发 cur_pending）
srv_motor_set_mode(inst, mode);                             // 切换控制模式 (1=速度, 3=电流)

/* 查询 */
srv_motor_get_handle(index);            // index: 0~8 获取句柄
srv_motor_get_feedback(inst);           // 获取反馈数据只读指针
srv_motor_is_online(inst);              // 检查电机是否在线

/* 驱动 */
srv_motor_step();   // 主循环调用，驱动 FSM
```

---

## 10. 示例

### 10.1 初始化并运行电流模式

```c
srv_motor_init();  // 自动注册 9 电机, 设 cur=30000, spd=0

// FSM STARTUP 自动执行:
//   ENABLE → 等 1s → SET_MODE(value=3) → BCAST_CUR 持续
// srv_motor_step() 在 app_main 主循环调用
```

### 10.2 动态调整电流

```c
srv_motor_handle_t* m = srv_motor_get_handle(0);
srv_motor_set_current(m, 5000);   // 降到 ~0.86A
srv_motor_set_current(m, 15000);  // 升到 ~2.58A
```

### 10.3 切换到速度模式

```c
srv_motor_set_mode(srv_motor_get_handle(0), 1);
// 需要恢复 BCAST_SPD 发送速度广播帧才生效
```

---

## 附录 A. 帧传输示例

| 帧类型 | 原始字节 (hex) |
|--------|---------------|
| ENABLE dev=1 | `55 AA 00 14 01 00 00 00 00 00 00 00 00 00 01 00 BD B4 00 00` |
| SET_MODE 电流 | `55 AA 00 14 A0 00 00 00 01 07 00 00 03 00 00 00 B3 85 00 00` |
| CMD_SET_REF cur=8000 spd=3000 | `55 AA 00 14 01 00 00 00 00 00 B8 0B 40 1F 03 00 B4 F1 00 00` |
| 查询 INFO_02_R dev=1 | `55 AA 00 14 A0 00 00 00 01 02 00 00 00 00 00 00 C8 67 00 00` |
| 查询 INFO_01_R dev=1 | `55 AA 00 14 A0 00 00 00 01 01 00 00 00 00 00 00 4A BF 00 00` |
| 广播速度 0xAD55 (5990) | `55 AD 66 17 66 17 66 17 66 17 66 17 66 17 66 17 66 17 41 C0` |

## 附录 B. 关键常数表

| 常量 | 值 | 定义位置 |
|------|-----|----------|
| `SRV_MOTOR_TOTAL` | `9` | `service/srv_motor.h` |
| `SRV_MOTOR_FRAME_SIZE` | `20` | `service/srv_motor.h` |
| `MOTOR_PER_GROUP` | `8` | `service/srv_motor.c` |
| `MOTOR_BCAST_PERIOD_MS` | `100` (默认) | `service/srv_motor.c` |
| `POLL_01_INTERVAL` | `10` | `service/srv_motor.c` |
| `SRV_MOTOR_CMD_ENABLE` | `1` | `service/srv_motor.h` |
| `SRV_MOTOR_CMD_DISABLE` | `2` | `service/srv_motor.h` |
| `SRV_MOTOR_CMD_SET_REF` | `3` | `service/srv_motor.h` |
| `SRV_MOTOR_CAN_ID_CONFIG` | `0xA0` | `service/srv_motor.h` |
