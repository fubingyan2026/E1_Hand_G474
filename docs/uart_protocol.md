好的，我将为您输出这份《关节模组通讯协议V2.0.8》的完整Markdown文档，包含所有章节和细节。

---

# 关节模组通讯协议 V2.0.8

## 更新记录

| 版本 | 修改内容 | 日期 | 作者 |
| :--- | :--- | :--- | :--- |
| v1.0 | 首版拟定 | 2025.4.27 | |
| v1.1 | 数据包从28减小到24个字节 | 2025.5.6 | |
| v1.2 | 串口波特率由19200提高到115200 | 2025.5.9 | |
| v1.3 | 指令增加速度设置支持 | 2025.5.19 | |
| v1.4 | 增加电流信息读取 | 2025.5.20 | |
| v1.5 | 指令增加电流设置支持 | 2025.5.28 | |
| v1.6 | 1. 波特率由115200提高到256000<br>2. 去掉TAIL字段，数据包从24字节降到20个字节；更改标志头；增加6个广播报文<br>3. 校验改成CRC16_CCITT_FALSE | 2025.6.7 | |
| v1.7 | 1. 增加PID、通讯波特率、控制模式设置<br>2. 增加反馈信息包3读取 | 2025.6.18 | |
| v1.8 | 1. 增加电流保护限制参数配置<br>2. 默认波特率由256000改成230400 | 2025.6.26 | |
| v1.9 | 1. 速度基准值由40krpm改成50krpm（自固件v124版本起）<br>2. 指令中电流字段改成保留，该字段值不起作用，无法再通过该字段动态设置电流（自固件版本v120起） | 2025.7.7 | |
| v2.0 | 控制指令更改：使能EN增加CMD_SET_REF | 2025.7.28 | |
| v2.0.1 | 1. 更新伪代码示例<br>2. 增加一些电流保护限制配置项文字说明<br>3. 增加Gaia Tools上位机使用说明 | 2025.8.1 | |
| v2.0.2 | 增加瞬爆配置项说明 | 2025.8.11 | |
| v2.0.3 | 增加电流控制模式说明 | 2025.9.17 | |
| v2.0.4 | 增加配置指令CONFIG_UPDATE_FIN_ZERO(3)描述 | 2025.12.22 | |
| v2.0.5 | 1. 更正CONFIG_INFO_02_R(2)描述错位<br>2. 增加参数配置项CONFIG_DRV_OT(28)和CONFIG_SPD_LIMIT(30)描述说明 | 2026.03.27 | |
| v2.0.6 | 增加GAIAHAND手主控的"一键回零"和"一键收纳"功能说明 | 2026.04.10 | |
| v2.0.7 | 1. 增加SN码读写说明<br>2. 增加INFO_06信息字段说明 | 2026.04.23 | |
| v2.0.8 | 优化状态信息01(CONFIG_INFO_01_R)中故障err_code和运行状态fsm字段描述说明 | 2026.04.30 | |

---

## 1. 功能列表

| 序号 | 功能描述 |
| :---: | :--- |
| 1 | 位置、速度控制模式 |
| 2 | 点动控制 |
| 3 | 读取状态信息 |
| 4 | 配置参数 |

---

## 2. 协议内容

### 通讯速度计算示例

串口数据传输，波特率230400，停止位1，数据位8，奇偶校验无。

包速率推算：

```
传输时间 = 总数据位数 / 波特率 = N × (D + S) / B
```

总数据位数 = N × (D + S) = 20 × (8 + 1) = 20 × 9 = 180 位

传输时间 = 180 / 230400 ≈ 0.00078125 秒 ≈ **0.781 毫秒**

### 数据包结构

使用二进制数据通信，数据包结构如下，**多字节为Intel格式（小端）**：

```c
typedef struct {
    uint32_t head;      // 协议头
    uint32_t can_id;    // 功能ID
    uint8_t can_data[8]; // 数据段
    uint32_t crc;       // CRC16_CCITT_FALSE
} UART_Frame_t;
```

**字段说明：**

| 字段 | 内容 | 备注 |
| :--- | :--- | :--- |
| head | 协议标志头 | 固定 `0x1400AA55`，对应字节序为 byte[3]=14, byte[2]=00, byte[1]=AA, byte[0]=55 |
| can_id | 功能ID | 见功能ID字段说明 |
| can_data[8] | 数据字段 | 含义见功能ID说明 |
| crc | CRC校验值 | 校验字段 `can_id` + `can_data[8]`，CRC16_CCITT_FALSE标准，占4个字节，高2字节为零 |

### CAN_ID 功能字段列表

| can_id | 功能 | data内容 | 方向 | 备注 |
| :--- | :--- | :--- | :--- | :--- |
| 0x00000000 ~ 0x0000009F, 0x000000FF | 角度和速度指令 | 角度，速度，电流 | TX | |
| 0x000000A0 | 配置/状态读取 | 设备ID，数据索引 | TX | 详见2.3节 |
| 0x000000F0 | 点动（零点重置） | 设备ID，点动方向 | TX | |
| 0x00010000 ~ 0x009FFF00 | 读取返回信息 | 数据 | RX | |

---

### 2.1 控制指令（0x00 ~ 0x9F, 0xFF）

一包控制指令数据中包含位置、速度、电流3个给定值和使能控制4个字段：

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| dev_id | RSVD | EN | colspan="2" | cur_ref | colspan="2" | spd_ref | colspan="2" | pos_ref |

各模式下的有效参数如下：

| 模式 | cur_ref | spd_ref | pos_ref | 备注 |
| :--- | :---: | :---: | :---: | :--- |
| 位置模式 | | | ✓ | |
| 速度模式 | | ✓ | | |
| 电流模式 | ✓ | | | |

**字段说明：**

| 字段 | 内容 | 备注 |
| :--- | :--- | :--- |
| **pos_ref** | 目标角度，Q7格式 | 要设定的值（度）× 128，支持负角度。<br>示例（Intel小端）：<br>`int16_t ref_q7 = -10.123(度) × 128`<br>`data[0] = (uint8_t)ref_q7;`<br>`data[1] = (uint8_t)(ref_q7 >> 8);`<br>**不溢出的范围是 -255度 ~ 255度** |
| **spd_ref** | 电机速度设置，Q15格式 | 0~32767 表示 0% ~ 100% 基准速度。<br>**基准速度为 50000 rpm**（未经减速比）<br>最大速度 20 krpm，对应 Q15 值为 (32767 × 20 / 50) = 13106<br>减速比为 568.81 |
| **cur_ref** | 电流设置，Q15格式 | 0~32767 表示电流 0~5.625A<br>参考值：<br>0.100A = 582<br>0.125A = 728<br>0.200A = 1165<br>0.250A = 1456<br>0.300A = 1747<br>0.500A = 2912<br>**⚠️ 自v1.9起该字段保留，不再生效** |
| **EN** | 使能控制 | `1`: CMD_ENABLE 开启控制，电机上电锁定<br>`2`: CMD_DISABLE 关闭控制，释放电机<br>`0, 3`: CMD_SET_REF 设置给定值<br>其他：无操作<br>**每次上电后，第一次开启控制，编码器需要定位（约1~5秒不等，取决于电机负载状态）** |
| **RSVD** | 保留 | 请置0 |
| **dev_id** | 设备ID | 范围 `0x00000001 ~ 0x0000009F`<br>`0x000000FF` 为广播指令，所有设备同时响应 |

---

### 2.2 JOG点动指令（0xF0）

**需要先使能控制（见控制指令）以后，点动才会响应。** 点动会同时重置零点到点动后的位置。

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0x000000F0 | colspan="4" | RSVD1 | dir | RSVD0 | dev_id |

**字段说明：**

| 字段 | 内容 | 备注 |
| :--- | :--- | :--- |
| dev_id | 设备ID | 支持0xFF广播ID |
| RSVD0 | 保留 | |
| dir | 方向 | `0`: 无动作<br>`1`: CW顺时针，+5度<br>`2`: CCW逆时针，-3度<br>**实际点动手指角度变化与安装方式有关，请结合实际使用。<br>可以通过前进后退组合出任意位置。** |
| RSVD1 | 保留 | |

**指令示例：**

ID2点动正转：
```
55 AA 00 14 F0 00 00 00 02 00 01 00 00 00 00 00 EE 60 00 00
```

ID2点动反转：
```
55 AA 00 14 F0 00 00 00 02 00 02 00 00 00 00 00 0E AE 00 00
```

> 注：`55 AA 00 14` 为标志头，`0E AE 00 00` 为CRC16，下文示例中将不再重复说明，重点关注 `can_id` 和 `data` 字段。

---

### 2.3 状态读取 & 参数配置（0xA0）

#### 2.3.1 发送指令格式

**示例：读取设备2的状态信息（index=1）**

```
55 AA 00 14 A0 00 00 00 02 01 00 00 00 00 00 00 3F 77 00 00
```

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0x000000A0 | colspan="3" | value | colspan="2" | RSVD | index | dev_id |

**字段说明：**

| 字段 | 内容 | 备注 |
| :--- | :--- | :--- |
| dev_id | 设备ID | 使用ID255可以批量广播配置（但无法批量读取） |
| index | 要读取或配置的项索引 | 取值与含义见索引列表<br>具体是读取还是配置由索引含义决定 |
| RSVD | 保留 | 请置0 |
| value | 要写入配置项的值 | 格式见对应索引描述<br>对于读取功能索引无需该值，置0 |

**配置索引列表：**

| index | 配置指令名 | 内容/含义 | 备注 |
| :---: | :--- | :--- | :--- |
| 1 | CONFIG_INFO_01_R | 读取数据包1（运行状态、版本信息） | |
| 2 | CONFIG_INFO_02_R | 读取数据包2（角度位置，电流值） | |
| 3 | CONFIG_UPDATE_FIN_ZERO | 重置当前末端位置为零点 | |
| 4 | CONFIG_SET_ID | 设置ID | 有回复（成功/失败） |
| 5 | CONFIG_UPDATE_ENC_ZERO | 电机磁编码器零点标定 | 谨慎使用，有回复（成功/失败） |
| 6 | CONFIG_COMM_SILENT | 保留 | |
| 7 | CONFIG_CTRL_MODE_SW | 控制模式切换（速度/位置模式） | 无回复（如无特殊说明，下同） |
| 8 | CONFIG_SET_POS_KP | 位置环KP设置 | |
| 9 | CONFIG_SET_POS_KD | 位置环KD设置 | |
| 10 | CONFIG_SET_SPD_KP | 速度环KP设置 | |
| 11 | CONFIG_SET_SPD_KI | 速度环KI设置 | |
| 12 | CONFIG_INFO_03_R | 读取信息3（速度和位置PID参数） | |
| 13 | CONFIG_WRITE_FLASH | PID参数写入FLASH | 有回复（成功/失败） |
| 15 | CONFIG_BUAD_RATE | 5档通讯波特率选择 | 有回复（成功/失败） |
| 16 | CONFIG_POS_LPF_LV | 位置平滑力度等级设置 | |
| 17 | CONFIG_INFO_04_R | 读取配置信息4（位置平滑力度、波特率） | |
| 18 | CONFIG_MAX_CUR | 最大输出电流 | |
| 19 | CONFIG_PROTECT_CUR_LIMIT | 保护限制后的最大电流 | |
| 20 | CONFIG_PROTECT_CUR_LV1 | 电流保护等级1阈值设定 | |
| 21 | CONFIG_CUR_LV1_DELAY | 电流保护等级1持续时间设定 | |
| 22 | CONFIG_RECV_CUR | 电流恢复阈值 | |
| 23 | CONFIG_CUR_RECV_WIN | 恢复判定时间窗口ms | |
| 24 | CONFIG_INFO_05_R | 读取配置信息5（电流保护等级1、恢复时间） | |
| 25 | CONFIG_INFO_06_R | 读取配置信息6（瞬爆冷却，瞬爆时长） | |
| 26 | CONFIG_COLD_TIME | 瞬间爆发冷却时长 | |
| 27 | CONFIG_PLUS_TIME | 瞬间爆发时长 | |
| 28 | CONFIG_DRV_OT | 驱动器温度保护阈值 | |
| 29 | CONFIG_MT_OT | 保留 | 保留 |
| 30 | CONFIG_SPD_LIMIT | 电机转子最大速度限制 | |
| 31 | CONFIG_SN | 写SN码 | 谨慎使用 |
| 32 | CONFIG_INFO_07_R | 读取配置信息7（SN码） | |

**回复数据格式示例：**

读取设备2的状态信息（索引1，CONFIG_INFO_01_R）：
```
55 AA 00 14 00 02 01 00 00 00 75 4F 00 00 FA 05 AC 95 00 00
```

**回复的 can_id 结构：**

| | byte[3]（高位） | byte[2] | byte[1] | byte[0]（低位） |
| :--- | :--- | :--- | :--- | :--- |
| can_id | 0 | index | dev_id | 0 |
| data[7] ~ data[0] | colspan="4" | DATAS | |

**字段含义：**

| 字段 | 含义 | 备注 |
| :--- | :--- | :--- |
| dev_id | 回复信息的设备ID | |
| index | 配置索引 | 与指令索引一致 |
| DATAS | 数据字段 | 内容详见各个指令描述 |

---

#### 2.3.2 读取/返回数据1（CONFIG_INFO_01_R）

包含运行状态、故障码、固件版本信息。

**读取发送格式：**

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0x000000A0 | | | | | | | 1 | dev_id |

**返回数据：**

设备上电时会自动发送一次该数据包，方便用于识别在线设备以及通信线路检查。

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| sta_id | Vbus | angle_fb | temp | soft_ver | err_code | fsm |

其中 `sta_id` 结构：

| | byte[3] | byte[2] | byte[1] | byte[0] |
| :--- | :--- | :--- | :--- | :--- |
| sta_id | 0 | index=1 | dev_id | 0 |

**字段说明：**

| 字段 | 内容 | 备注 |
| :--- | :--- | :--- |
| **fsm** | 运行状态 | `0`: 就绪<br>`1`: 初始化<br>`2`: 预充<br>`3`: RSVD0<br>`4`: RSVD1<br>`5`: 预定位<br>`6`: 启动<br>`7`: 运行<br>`8`: 停止<br>`9`: 故障<br>`10`: 刹车 |
| **err_code** | 实时故障/历史故障 | 实时故障 = err_code & 0x0F<br>历史故障 = (err_code >> 4) & 0x0F<br><br>故障码：<br>`0`: 无<br>`1`: 硬件过流<br>`2`: 软件过流<br>`3`: 硬件过压<br>`4`: 软件过压<br>`5`: 硬件欠压<br>`6`: 软件欠压<br>`7`: 缺相<br>`8`: 堵转<br>`9`: 软件过温<br>`10`: RSVD<br>`11`: RSVD |
| **soft_ver** | 软件版本 | 如 103 解释为 v1.0.3 |
| **temp** | MCU温度 | 单位摄氏度，偏移量50<br>如收到 temp=110，表示温度是 110-50=60°C |
| **angle_fb** | 手指角度 | Q7格式，Intel小端<br>`angle_fb = (data[5] << 8 | data[4]) × 0.0078125 (1/128)`，单位度 |
| **Vbus** | 母线电压 | Q7格式，单位V |
| **sta_id** | 设备ID + index | dev_id: 设备ID<br>index=1 (CONFIG_INFO_01_R) |

---

#### 2.3.3 读取/返回数据2（CONFIG_INFO_02_R）

包含电流、母线电压、角度信息。

**读取发送格式：**

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0x000000A0 | | | | | | | 2 | dev_id |

**返回数据：**

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| sta_id | angle_fb | speed | Q_cur | D_cur |

`sta_id` 结构：

| | byte[3] | byte[2] | byte[1] | byte[0] |
| :--- | :--- | :--- | :--- | :--- |
| sta_id | 0 | index=2 | dev_id | 0 |

**字段说明：**

| 字段 | 内容 | 备注 |
| :--- | :--- | :--- |
| **D_cur** | D轴电流 | Q15格式，0~32767 表示电流 0~5.625A<br>Intel小端示例：<br>`Dcur = (data[1] << 8 | data[0])`<br>相电流幅值：`Is = sqrtf(D×D + Q×Q)` |
| **Q_cur** | Q轴电流 | Q15格式，同上 |
| **speed** | 电机速度 | Q15格式，单位rpm |
| **angle_fb** | 手指角度 | Q7格式 |
| **sta_id** | 设备ID + index | |

---

#### 2.3.4 重置当前末端位置为零点（CONFIG_UPDATE_FIN_ZERO）

**发送格式：**

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0x000000A0 | | | | | | | index=3 | dev_id |

**无回复。**

---

#### 2.3.5 设置ID（CONFIG_SET_ID）

**发送格式：**

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0x000000A0 | Value_H | Value_L | | | | | index=4 | dev_id |

**示例：设置ID为10**

```c
uint16_t id_set = 10 << 7;  // to 07
uint8_t Value_L = id_set;
uint8_t Value_H = id_set >> 8;
```

**回复内容：**

| | byte[3]（高位） | byte[2] | byte[1] | byte[0]（低位） |
| :--- | :--- | :--- | :--- | :--- |
| can_id | 0 | index=4 | dev_id | 0 |
| data[7]~data[1] | colspan="3" | | res |
| data[0] | res | | | |

| 字段 | 内容 |
| :--- | :--- |
| res | `0`: 失败<br>`1`: 成功，新设备ID |

> **参数立刻写入FLASH，无需再次发送 CONFIG_WRITE_FLASH 指令。**

---

#### 2.3.6 切换控制模式（CONFIG_CTRL_MODE_SW）

**发送格式：**

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0x000000A0 | | | | | | mode | index=7 | dev_id |

**字段说明：**

| 字段 | 内容 | 备注 |
| :--- | :--- | :--- |
| dev_id | 目标设备ID | |
| index | CONFIG_CTRL_MODE_SW(7) | |
| mode | 控制模式 | `0`: 无操作<br>`1`: 速度模式<br>`2`: 位置模式<br>`3`: 电流模式<br>**电流模式下依然会受到电流保护作用** |

---

#### 2.3.7 设置位置/速度PID（CONFIG_SET_POS_Kx）

**发送格式：**

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0x000000A0 | Value_H | Value_L | | | | | index | dev_id |

| 配置名 | index | value格式 |
| :--- | :---: | :--- |
| CONFIG_SET_POS_KP | 8 | Q12，范围0~32767（对应0~8.000） |
| CONFIG_SET_POS_KD | 9 | Q12，范围0~32767（对应0~8.000） |
| CONFIG_SET_SPD_KP | 10 | Q12，范围0~32767（对应0~8.000） |
| CONFIG_SET_SPD_KI | 11 | Q15，范围0~32767（对应0~1.000） |

> **更新之后立刻生效，无回复。**
>
> **仅改变内存数据，下电丢失，如需永久保存请调试好后使用 CONFIG_WRITE_FLASH 指令。**

---

#### 2.3.8 读取/返回数据3（CONFIG_INFO_03_R）

数据包3的内容是位置的KP、KD，速度的KP、KI。

**发送格式：**

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0x000000A0 | | | | | | | 12 | dev_id |

**返回数据示例：**

POS_KP(Q12) = 0xFF | (0x03 << 8) = 0x03FF = 1023 ( /4096 = 0.245 )

---

#### 2.3.9 参数保存（CONFIG_WRITE_FLASH）

**发送格式：**

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0x000000A0 | | | | | KEY | | index=13 | dev_id |

| 字段 | 内容 | 备注 |
| :--- | :--- | :--- |
| dev_id | 目标设备ID | |
| index | CONFIG_WRITE_FLASH(13) | |
| KEY | 0xA5 | |

**回复内容：**

| | byte[3]（高位） | byte[2] | byte[1] | byte[0]（低位） |
| :--- | :--- | :--- | :--- | :--- |
| can_id | 0 | index=13 | dev_id | 0 |
| data[7]~data[1] | colspan="3" | | res |
| data[0] | res | | | |

| 字段 | 内容 |
| :--- | :--- |
| res | `0`: 失败<br>`1`: 成功 |

---

#### 2.3.10 设置通讯波特率（CONFIG_BUAD_RATE）

**发送格式：**

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0x000000A0 | | | | | | level | index=15 | dev_id |

| 字段 | 内容 | 备注 |
| :--- | :--- | :--- |
| dev_id | 目标设备ID | |
| index | CONFIG_BUAD_RATE(15) | |
| level | 波特率档位 | `0`: 500000<br>`1`: 230400（默认）<br>`2`: 115200<br>`3`: 38400<br>`4`: 19200<br>**多机控制时不建议降低波特率** |

**回复内容：**

| | byte[3]（高位） | byte[2] | byte[1] | byte[0]（低位） |
| :--- | :--- | :--- | :--- | :--- |
| can_id | 0 | index=15 | dev_id | 0 |
| data[7]~data[1] | colspan="3" | | res |
| data[0] | res | | | |

| 字段 | 内容 |
| :--- | :--- |
| res | `0`: 失败<br>`1`: 成功 |

---

#### 2.3.11 位置平滑力度等级（CONFIG_POS_LPF_LV）

**发送格式：**

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0x000000A0 | | | | | | level | index=16 | dev_id |

---

#### 2.3.12 读取配置信息4（CONFIG_INFO_04_R）

**发送格式：**

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0x000000A0 | | | | | | | index=17 | dev_id |

**返回数据：**

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| sta_id | protect_limit | max_current | reserved | param_ver | baud_level | lpf_level |

`sta_id` 结构：

| | byte[3] | byte[2] | byte[1] | byte[0] |
| :--- | :--- | :--- | :--- | :--- |
| sta_id | 0 | index=17 | dev_id | 0 |

**字段说明：**

| 字段 | 内容 | 备注 |
| :--- | :--- | :--- |
| lpf_level | 位置平滑等级 | 0~5，数值越大平滑效果越大 |
| baud_level | 当前波特率等级 | 0~4 |
| param_ver | 参数版本号 | |
| max_current | 最大输出电流 | 0~32767，Q15格式，电流基准值5.625A |
| protect_limit | 保护限制电流 | 0~32767，Q15格式 |

---

#### 2.3.13 最大输出电流设定（CONFIG_MAX_CUR）

**发送格式：**

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0x000000A0 | | | valueQ | | | | index=18 | dev_id |

| 字段 | 内容 | 备注 |
| :--- | :--- | :--- |
| index | CONFIG_MAX_CUR(18) | |
| valueQ | 设定的电流值 | 0~32767，Q15格式，基准值5.625A<br>例如设定0.5A：`valueQ = 0.5/5.625 × 32767` |

---

#### 2.3.14 保护限制电流设定（CONFIG_PROTECT_CUR_LIMIT）

触发保护阈值后限制输出不超过该电流。

**发送格式：**

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0x000000A0 | | | valueQ | | | | index=19 | dev_id |

| 字段 | 内容 | 备注 |
| :--- | :--- | :--- |
| index | CONFIG_PROTECT_CUR_LIMIT(19) | |
| valueQ | 设定的电流值 | 0~32767，Q15格式 |

---

#### 2.3.15 保护电流阈值设定（CONFIG_PROTECT_CUR_LV1）

当电流超过该值并持续一定时间后，触发限制电流保护。

**发送格式：**

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0x000000A0 | | | valueQ | | | | index=20 | dev_id |

---

#### 2.3.16 保护电流允许时间设定（CONFIG_CUR_LV1_DELAY）

允许电流超过保护阈值后的持续时间，超过该时间后触发电流限制保护。

**发送格式：**

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0x000000A0 | | | value | | | | index=21 | dev_id |

| 字段 | 内容 | 备注 |
| :--- | :--- | :--- |
| index | CONFIG_CUR_LV1_DELAY(21) | |
| value | 设定的时间 | 0~65000 毫秒 |

---

#### 2.3.17 保护恢复阈值设定（CONFIG_RECV_CUR）

当电流小于该值且持续一定时间（恢复时间窗），自动退出电流保护限制。

> 注：退出保护还有一种方式，给定新的指令后立即恢复，冷却时间为 CONFIG_COLD_TIME。

**发送格式：**

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0x000000A0 | | | valueQ | | | | index=22 | dev_id |

| 字段 | 内容 | 备注 |
| :--- | :--- | :--- |
| index | CONFIG_RECV_CUR(22) | |
| valueQ | 设定的电流值 | 0~32767，Q15格式 |

---

#### 2.3.18 保护恢复窗口时间设定（CONFIG_CUR_RECV_WIN）

当电流小于恢复值且持续该时间，自动退出电流保护限制。

**发送格式：**

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0x000000A0 | | | value | | | | index=23 | dev_id |

| 字段 | 内容 | 备注 |
| :--- | :--- | :--- |
| index | CONFIG_CUR_RECV_WIN(23) | |
| value | 设定的时间 | 0~65000 毫秒 |

---

#### 2.3.19 读取配置信息5（CONFIG_INFO_05_R）

保护阈值相关：电流保护等级1、恢复时间。

**发送格式：**

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0x000000A0 | | | | | | | index=24 | dev_id |

**返回数据：**

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| sta_id | recover_delay | recv_cur | protect_delay_lv1 | protect_cur_lv1 |

`sta_id` 结构：

| | byte[3] | byte[2] | byte[1] | byte[0] |
| :--- | :--- | :--- | :--- | :--- |
| sta_id | 0 | index=24 | dev_id | 0 |

---

#### 2.3.20 读取配置信息6（CONFIG_INFO_06_R）

读取瞬间爆发相关参数：瞬爆冷却时长、瞬爆时长。

> 瞬间爆发设计用于堵转之后的尝试恢复，处于堵转状态下在指令改变时触发爆发电流。

**发送格式：**

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0x000000A0 | | | | | | | index=25 | dev_id |

**返回数据：**

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| sta_id | spd_limit | drv_over_temp | enc_offset | plus_t | cold_dt |

`sta_id` 结构：

| | byte[3] | byte[2] | byte[1] | byte[0] |
| :--- | :--- | :--- | :--- | :--- |
| sta_id | 0 | index=25 | dev_id | 0 |

**字段说明：**

| 字段 | 内容 | 备注 |
| :--- | :--- | :--- |
| cold_dt | 瞬间爆发冷却时长 | 单位 25ms |
| plus_t | 瞬间爆发时长 | |
| enc_offset | 编码器偏移 | |
| drv_over_temp | 驱动器过温阈值 | |
| spd_limit | 速度限制 | |

---

#### 2.3.21 爆发冷却时间设定（CONFIG_COLD_TIME）

瞬间爆发冷却时间设置。

**发送格式：**

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0x000000A0 | | | value | | | | index=26 | dev_id |

| 字段 | 内容 | 备注 |
| :--- | :--- | :--- |
| index | CONFIG_COLD_TIME(26) | |
| value | 设定的时间 | 0~255，单位25ms<br>如80为2秒（80×25=2000ms） |

---

#### 2.3.22 爆发时长设定（CONFIG_PLUS_TIME）

每次触发电流爆发的时长。

**发送格式：**

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0x000000A0 | | | value | | | | index=27 | dev_id |

---

#### 2.3.23 驱动器温度保护阈值（CONFIG_DRV_OT）

设置驱动器温度保护阈值（自动恢复阈值为该值-10，不可单独设置）。

**发送格式：**

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0x000000A0 | | | value | | | | index=28 | dev_id |

| 字段 | 内容 | 备注 |
| :--- | :--- | :--- |
| index | CONFIG_DRV_OT(28) | |
| value | 设定的温度 | uint8_t，0~255，单位摄氏度 |

> 注：要读当前阈值见"配置信息6"。

---

#### 2.3.24 电机最大速度限制（CONFIG_SPD_LIMIT）

**发送格式：**

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0x000000A0 | | | valueH | valueL | | | index=30 | dev_id |

| 字段 | 内容 | 备注 |
| :--- | :--- | :--- |
| index | CONFIG_SPD_LIMIT(30) | |
| value | 设定的速度 | 0~32767，Q15格式<br>Base = 50krpm<br>例如设置为20krpm：<br>`Value = 20/50 × 32767` |

> 注：要读当前阈值见"配置信息6"。

---

#### 2.3.25 写SN码（CONFIG_SN）

保留

---

#### 2.3.26 读取配置信息7（CONFIG_INFO_07_R）

**发送格式：**

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0x000000A0 | | | | | | | index=32 | dev_id |

**返回数据：**

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| sta_id | SN | | | | | | |

`sta_id` 结构：

| | byte[3] | byte[2] | byte[1] | byte[0] |
| :--- | :--- | :--- | :--- | :--- |
| sta_id | 0 | index=32 | dev_id | 0 |

| 字段 | 内容 | 备注 |
| :--- | :--- | :--- |
| SN | SN码 | |

---

### 2.4 特殊广播

特殊广播设计用于快速同步控制 ID1~ID16 的设备。

```c
typedef struct {
    uint16_t head2;       // 标志头
    int16_t value[8];     // 数据段
    uint16_t crc16;       // CRC16_CCITT_FALSE
} UART_Frame2_t;
```

**字段说明：**

| 字段 | 内容 | 备注 |
| :--- | :--- | :--- |
| **head2** | 标志头 | `BOARDCASE_POS01_08_HEAD1 = 0xAB55` // 角度<br>`BOARDCASE_POS09_15_HEAD2 = 0xAC55` // 角度<br>`BOARDCASE_SPD01_08_HEAD3 = 0xAD55` // 速度<br>`BOARDCASE_SPD09_16_HEAD4 = 0xAE55` // 速度<br>`BOARDCASE_EN01_16_HEAD7 = 0xB155` // 使能 |
| **value[x]** | 指令数据 | 根据标志头区分，x=[0,7] 对应设备ID 1~8 或 9~16<br><br>**角度设置**：Q7格式，设定值（度）×128，支持负角度，范围 -255度~255度<br>**速度设置**：Q15格式，0~32767 表示 0%~100% 基准速度<br>基准速度参见控制指令章节 |
| **value[y]** | 使能广播 | 对于使能广播（0xB155），value 解释为 `uint8_t value[16]`，y=[0,15] 对应设备ID 1~16<br>`value[y]=1`: 使能控制，电机上电锁定<br>`value[y]=2`: 关闭控制，释放电机<br>`value[y]=其他`: 无操作<br>**每次上电后，第一次开启控制，编码器需要定位（约2秒），请保持空载状态** |
| **crc16** | CRC校验 | CRC16_CCITT_FALSE |

**数据示例：**

广播设置 ID1~ID8 位置 90 度：
```
55 AB 00 2D 00 2D 00 2D 00 2D 00 2D 00 2D 00 2D 00 B6 67
```
（90×128 = 11520 = 0x2D00）

广播设置 ID9~ID16 位置 -90 度：
```
55 AC 00 D3 00 D3 00 D3 00 D3 00 D3 00 D3 00 D3 00 D3 00 B0 76
```
（-90×128 = -11520 = 0x3D300）

广播使能：
```
55 B1 00 14 B1 00 00 00 FF 01 00 00 00 00 0F 35 80 00 0
```

---

## 3. 上位机说明与指令样例

> 软件工具：`Gaiatools_v151.zip` / `Gaiatools_v152.zip`

### 3.1 UI介绍

![UI界面](image)

**分区说明：**

| 区 | 功能 | 说明 |
| :---: | :--- | :--- |
| **1区** | 串口连接 | 默认波特率230400。读取按钮用于确认设备连接状态，返回的数据显示在3区中。 |
| **2区** | 原始数据 | 上方为接收，下方为发送（若串口未打开时显示为红字）。可以作为样例参考。 |
| **3区** | 状态信息显示 | 手动点击"读取"可以获取最新信息，或者勾选自动刷新，以3Hz的频率自动读取（对应的设备ID为1区中的ID）。 |
| **4区** | Play试玩 | |
| **5区** | 参数配置区 | 对应的设备ID为1区中的ID。 |
| **6区** | 程序升级 | 选择好HEX文件后，确认ID无误，点击"在线下载"。 |

![参数配置界面](image)

### 3.2 指令样例

可以通过操作上位机生成指令，无需打开串口也可以生成指令。上位机的每一次操作都会有输出，2区的LOG窗口上方为接收，下方为发送，方便参考。

```
55 AA 00 14 00 01 01 00 00 05 80 24 00 00 00 00 C0 1F 00 00
55 AA 00 14 00 01 0C 00 FF 0F FF 0F FF 03 51 00 53 BB 00 00
55 AA 00 14 00 01 11 00 01 01 03 00 10 11 69 03 14 78 00 00
55 AA 00 14 00 01 18 00 60 0B 58 02 46 02 90 01 00 52 00 00
```

> **注：每包数据均为20个字节，如果收发字节统计不是20的整数倍，说明通讯有受到过干扰。**

### 3.3 基本控制

**步骤1：确定在线设备ID和通讯状态**

连接好串口后，上位机先打开串口，然后再上电。设备在上电后会发出一帧数据，如上图黄框中的 `0x01` 即是设备ID。ID框选择对应ID之后，点"读取"按钮，如果设备有回复，说明通讯正常。

> 注：如果上电后没有报文数据，请检查串口线和供电是否正常。
>
> 如果同时连接多个设备，请确保ID不重复，否则需要先单独改ID再连接到一起。

**步骤2：控制操作**

勾选需要控制的ID，点击"使能控制"，之后拖动滑块即可控制，也可手动输入角度，点击发送。

> 上电默认位置控制，勾选"速度模式"可以切换到速度控制模式，模式不保存，下电恢复。

**点动**：位置模式下，JOG- 和 JOG+，微调位置同时使零点更新到当前位置。

### 3.4 参数配置 - ID更改

对应的设备ID为1区中的ID。

- **"写入"字样按钮**：点击后立即写入Flash，永久保存。
- **"更新"字样按钮**：点击后参数立即生效，下电丢失。如需永久保存，请在设置完所有参数后点击"写入Flash"按钮。

![参数配置界面](image)

---

## 4. 主控特殊功能

主控的ID为 **254**，使用ID254发送的内容即可被主控识别。

### 4.1 "一键回零"

给主控发送位置为0的指令：

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 254 | | | | | | | 0 | 30 | 00 |

指令发送后，主控会开环执行3轮手势动作。如果执行过程中再次发送则会打断当前的进度，之后可再次发送指令重新开始。

### 4.2 "一键收纳"

给主控发送"使能"指令：

| can_id | data[7] | data[6] | data[5] | data[4] | data[3] | data[2] | data[1] | data[0] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 254 | | | | | | | 0 | 10 | 00 |

该指令会在"收手/开手"之间循环。

---

## 5. 示例代码片段

> 例程伪代码，多字节为Intel格式

```c
#define TARGET_DEVICE_ID      (0x01)   // 设备ID（也即指令功能ID）
#define DEVICE_BOARDCASE_ID   (0xFF)   // 广播ID（所有设备对该ID均响应）
#define JOG_ID                (0xFF)   // 点动功能ID
#define CONFIG_FUNC_ID        (0xA0)   // 信息查询功能ID

/*
 * BYTE[6] CMD列表
 * 配置与状态读取功能ID（0xA0）支持的配置项
 */
#define CONFIG_REBOOT            (0)   // 复位请求
#define CONFIG_INFO_01_R         (1)   // 读取状态信息1
#define CONFIG_INFO_02_R         (2)   // 读取状态信息2（位置、电流、母线电压）
#define CONFIG_UPDATE_FIN_ZERO   (3)   // 末端角度编码器零点
#define CONFIG_SET_ID            (4)   // 设置设备ID
#define CONFIG_UPDATE_ENC_ZERO   (5)   // 转子角度编码器零点
#define CONFIG_COMM_SILENT       (6)   // 通讯静默（除本指令外不响应任何指令）
#define CONFIG_CTRL_MODE_SW      (7)   // 控制模式切换（2位置/1速度）
#define CONFIG_SET_POS_KP        (8)   // 位置环KP
#define CONFIG_SET_POS_KD        (9)   // 位置环KD
#define CONFIG_SET_SPD_KP        (10)  // 速度环KP
#define CONFIG_SET_SPD_KI        (11)  // 速度环KI
#define CONFIG_INFO_03_R         (12)  // 读取配置信息3（速度PI，位置PD参数）
#define CONFIG_WRITE_FLASH       (13)  // 参数写入FLASH（参数存储）
#define CONFIG_READ_FLASH        (14)  // 参数读取FLASH（参数存储）
#define CONFIG_BUAD_RATE         (15)  // 波特率设置 (0:500000,1:230400,2:115200,3:38400,4:19200)
#define CONFIG_POS_LPF_LV        (16)  // 位置指令平滑力度0~5
#define CONFIG_INFO_04_R         (17)  // 读取配置信息4（位置平滑力度、波特率）
#define CONFIG_MAX_CUR           (18)  // 最大输出电流
#define CONFIG_PROTECT_CUR_LIMIT (19)  // 保护限制后的最大电流
#define CONFIG_PROTECT_CUR_LV1   (20)  // 电流保护等级1阈值设定
#define CONFIG_CUR_LV1_DELAY     (21)  // 电流保护等级1持续时间设定
#define CONFIG_RECV_CUR          (22)  // 电流恢复阈值
#define CONFIG_CUR_RECV_WIN      (23)  // 恢复判定时间窗口ms
#define CONFIG_INFO_05_R         (24)  // 读取配置信息5（电流保护等级1、恢复时间）
#define CONFIG_INFO_06_R         (25)  // 读取配置信息6（瞬爆冷却，瞬爆时长）
#define CONFIG_COLD_TIME         (26)  // 瞬爆冷却
#define CONFIG_PLUS_TIME         (27)  // 瞬爆时长

// JOG_dir
#define JOG_CW   (1)   // 顺时针
#define JOG_CCW  (2)   // 逆时针

/*
 * 协议标志头尾（注意大小端）
 */
#define UART_PACKGE_HEAD (0x1400AA55)

typedef struct {
    uint32_t head;       // 固定0x1400AA55 (byte[0]=0x55)
    uint32_t can_id;     // 功能ID
    uint8_t can_data[8]; // 数据段
    uint32_t crc;        // CRC16_CCITT_FALSE
} UART_Frame_t;

UART_Frame_t tx_frame = {
    .head = UART_PACKGE_HEAD,
};

UART_Frame_t rx_frame;

void set_enable(UART_Frame_t* p, bool en)
{
    int16_t pos_ref = 0;
    uint16_t spd_ref = 0;
    uint16_t cur_limit = 0;   // 保留
    uint8_t cmd = en ? CMD_ENABLE : CMD_DISABLE;

    p->can_id = TARGET_DEVICE_ID;
    p->can_data[0] = 0;
    p->can_data[1] = 0;
    p->can_data[2] = 0;
    p->can_data[3] = 0;
    p->can_data[4] = 0;
    p->can_data[5] = 0;
    p->can_data[6] = cmd;
    p->crc = CRC16_CCITT_FALSE((uint8_t*)&p->can_id,
               sizeof(UART_Frame_t) - sizeof(p->head) - sizeof(p->crc));
    UART_Send((uint8_t*)p, sizeof(UART_Frame_t));
}

/*
 * 设置角度
 */
void set_pos(UART_Frame_t* p, float pos_deg)
{
    int16_t pos_ref;
    uint16_t spd_ref = 16384;   // Q15, 0~32767对应0%~100%基准速度(50000rpm)
    uint16_t cur_limit = 0;     // 保留
    uint8_t cmd = CMD_SET_REF;

    pos_ref = pos_deg * 128;
    p->can_id = TARGET_DEVICE_ID;
    p->can_data[0] = (uint8_t)pos_ref;
    p->can_data[1] = (uint8_t)(pos_ref >> 8);
    p->can_data[2] = (uint8_t)spd_ref;
    p->can_data[3] = (uint8_t)(spd_ref >> 8);
    p->can_data[4] = (uint8_t)cur_limit;
    p->can_data[5] = (uint8_t)(cur_limit >> 8);
    p->can_data[6] = cmd;
    p->crc = CRC16_CCITT_FALSE((uint8_t*)&p->can_id,
               sizeof(UART_Frame_t) - sizeof(p->head) - sizeof(p->crc));
    UART_Send((uint8_t*)p, sizeof(UART_Frame_t));
}

/*
 * 读取状态信息
 */
void get_status(UART_Frame_t* p)
{
    p->can_id = CONFIG_FUNC_ID;
    p->can_data[0] = TARGET_DEVICE_ID;
    p->can_data[1] = CONFIG_INFO_01_R;   // index=1
    p->crc = CRC16_CCITT_FALSE((uint8_t*)&p->can_id,
               sizeof(UART_Frame_t) - sizeof(p->head) - sizeof(p->crc));
    UART_Send((uint8_t*)p, sizeof(UART_Frame_t));

    // 串口接收数据
    UART_Recv((uint8_t*)&rx_frame, sizeof(UART_Frame_t));
    int angle = (rx_frame->can_data[5] << 8 | rx_frame->can_data[4]) >> 7;
    printf("DEV:%2d, FSM:%3d, ERROR_CODE:%2d, ANGLE:%5.2d\n",
           rx_frame->can_id,
           rx_frame->can_data[0],
           rx_frame->can_data[1],
           angle);
}

/*
 * 点动
 */
void do_jog(UART_Frame_t* p, bool dir)
{
    p->can_id = JOG_ID;
    p->can_data[0] = TARGET_DEVICE_ID;
    p->can_data[2] = dir ? JOG_CW : JOG_CCW;
    p->crc = CRC16_CCITT_FALSE((uint8_t*)&p->can_id,
               sizeof(UART_Frame_t) - sizeof(p->head) - sizeof(p->crc));
    UART_Send((uint8_t*)p, sizeof(UART_Frame_t));
}

/*
 * CRC计算 - CRC16_CCITT_FALSE
 */
uint16_t CRC16_CCITT_FALSE(uint8_t data[], int offset, int length)
{
    const uint16_t polynomial = 0x1021;
    uint16_t crc = 0xFFFF;
    for (int i = offset; i < offset + length; i++) {
        crc ^= (uint16_t)(data[i] << 8);
        for (int j = 0; j < 8; j++) {
            if ((crc & 0x8000) != 0)
                crc = (uint16_t)((crc << 1) ^ polynomial);
            else
                crc <<= 1;
        }
    }
    return crc;
}

void task_test()
{
    // get_status(&tx_frame);
    set_enable(&tx_frame, 1);
    sleep(3000);
    for (;;) {
        set_pos(&tx_frame, 90.0f);
        sleep(3000);
        set_pos(&tx_frame, -90.0f);
        sleep(3000);
        // do_jog(&tx_frame, 1);
    }
}
```

---

## 6. END

---

以上即为《关节模组通讯协议V2.0.8》的完整内容。如需进一步分析特定功能或计算总线负载，请随时告知。