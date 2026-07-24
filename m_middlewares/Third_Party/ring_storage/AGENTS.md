# ring_storage/

环形缓冲区 Flash 参数存储模块。专为嵌入式 MCU 参数持久化设计，替代 EasyFlash NG 用于参数存储场景。

## 结构

```
ring_storage/
├── ring_storage.h           # 公共 API（类型、错误码、函数声明）
├── ring_storage_port.h      # 平台抽象接口（用户实现 read/write/erase/lock/unlock）
├── ring_storage.c           # 核心实现（帧打包/解析/扫描/GC）
├── ring_storage_port.c      # STM32G4 平台移植实现
├── rs_crc32.h               # CRC32 模块头（多项式 0xEDB88320，与 zlib/PNG 兼容）
├── rs_crc32.c               # CRC32 查表法实现（8 路循环展开）
├── README.md                # 用户文档
└── AGENTS.md                # 本文档（AI 辅助开发参考）
```

## 核心设计

### 帧格式

```
偏移    字段             大小    说明
----------------------------------------------
 0      magic            4B     0x52535446 ("RSTF")
 4      version          4B     单调递增版本号
 8      frame_len        4B     帧总逻辑大小（含头尾）
12      kv_count         4B     KV 条目数
16      header_crc32     4B     帧头 CRC32（偏移 0~15）
20      KV 数据区        N B     [klen(1)][key][vlen(2)][val]...
20+N   data_crc32       4B     KV 数据区 CRC32
24+N   commit_magic     4B     0x434F4D54 ("COMT") 原子提交点
----------------------------------------------
固定开销：28B/帧（20B 帧头 + 8B 帧尾）
```

Flash 物理布局中，帧体写入后可能有 0xFF 对齐填充，帧尾固定位于对齐后区域的末尾 8 字节。扫描时通过 `frame_len`（逻辑大小）和 `RS_FRAME_FLASH_SIZE`（对齐后大小）分别定位帧边界和帧尾地址。

### 断电保护（三重校验）

| 校验层 | 保护对象 | 失败行为 |
|--------|---------|---------|
| header_crc32 | 帧头 magic/version/frame_len/kv_count | 跳过该帧，前进一个对齐单位 |
| commit_magic | 帧尾 0x434F4D54 | 跳过该帧（视为未完成写入） |
| data_crc32 | KV 数据区 | 跳过该帧（视为数据损坏） |

commit_magic 是**最后写入**的字段，作为原子提交点。在 STM32G4 双字编程下，data_crc32(4B) + commit_magic(4B) = 8B 恰为一个双字，一次写入原子完成。

### 磨损均衡（顺序轮转 GC）

GC 策略为 **Round-Robin 顺序轮转**：每次 GC 选择 `(active_sector_index + 1) % N` 号扇区作为目标，所有 N 个扇区均匀参与磨损。

```
GC 流程：
1. 计算 next_index = (active_sector_index + 1) % sector_num
2. 若目标扇区非空则先擦除（处理 init 后首次 GC 的残留数据）
3. 分块复制最新帧到目标扇区起始（每块对齐到 write_gran）
4. 擦除原活动扇区
5. active_sector_index = next_index，active_sector_addr 更新
```

断电安全保证：步骤 3 失败不会执行步骤 4，原扇区数据完好。步骤 4 失败时两个扇区各有一份有效帧，下次 init 扫描时取 version 最大的。

### 写入颗粒度自适应

`rs_align_write()` 处理 8/32/64/128/256 bit 写入颗粒度。对齐部分直接写入，末尾不足一个对齐单元时用栈上 `RS_MAX_ALIGN_BYTES`(32B) 缓冲区填充 0xFF 后写入。

### 扇区空检测

`rs_is_sector_empty()` 采样扇区**头部/中部/尾部**各 16 字节，而非仅检查头部。防止前 16 字节为 0xFF 但扇区深层有残留数据的漏判。

## 锁策略

| 函数 | 持锁范围 | 说明 |
|------|---------|------|
| `ring_storage_save` | 整个 save 流程 | goto cleanup 统一释放 |
| `rs_load_frame` | 仅 3 段 Flash I/O | lock→read→unlock，CRC/解析在锁外 |
| `rs_gc_collect` | 由 save 持有 | 不单独加锁 |

`rs_load_frame` 缩小持锁范围的意图：CRC32 计算和 `rs_parse_and_load_kv`（含 strlen/strncmp/memcpy）在锁外执行，避免长时间屏蔽中断导致 CAN 帧丢失。

## 错误码

```c
typedef enum {
    RING_STORAGE_OK = 0,                // 操作成功
    RING_STORAGE_ERROR_NULL_PTR,        // 空指针
    RING_STORAGE_ERROR_INVALID_PARAM,   // 无效参数
    RING_STORAGE_ERROR_UNINITIALIZED,   // 未初始化
    RING_STORAGE_ERROR_BUFFER_TOO_SMALL,// 缓冲区不足
    RING_STORAGE_ERROR_KV_TABLE_FULL,   // KV 注册表已满
    RING_STORAGE_ERROR_KV_DUPLICATE,    // KV key 重复
    RING_STORAGE_ERROR_KEY_TOO_LONG,    // key 超长
    RING_STORAGE_ERROR_FLASH_READ,      // Flash 读取失败
    RING_STORAGE_ERROR_FLASH_WRITE,     // Flash 写入失败
    RING_STORAGE_ERROR_FLASH_ERASE,     // Flash 擦除失败
    RING_STORAGE_ERROR_NO_VALID_FRAME,  // 无有效帧（首次使用）
    RING_STORAGE_ERROR_CRC,             // CRC 校验失败
    RING_STORAGE_ERROR_CORRUPT,         // 帧数据损坏（KV 解析边界异常）
    RING_STORAGE_ERROR_GC_FAILED,       // GC 失败
} ring_storage_error_t;
```

`CORRUPT` 区别于 `CRC`：前者表示 KV 解析时数据边界越界（帧可能被截断），后者表示 CRC32 计算值不匹配。

## 关键约束

- **value 指针生命周期**：`ring_storage_register()` 传入的 value 指针必须指向静态/全局内存，save 时直接解引用，不可为栈上临时变量。
- **KV 注册时机**：所有 `ring_storage_register()` 必须在 `ring_storage_load()` / `ring_storage_save()` 之前完成，运行中不可增减 KV。
- **最少扇区数**：`area_size >= 2 × sector_size`，init 时校验。
- **重烧录保护**：若使用 STM32CubeIDE 调试，需在 Debug Configuration → Debugger → Flash download 中选择 "Erase sectors" 而非 "Erase all"，否则 ring_storage 数据区会被全片擦除清除。

## 使用方法

```c
#include "ring_storage.h"

/* 1. 创建实例 */
static uint8_t s_frame_buf[1024];
static ring_storage_context_t s_storage;

/* 2. 配置（STM32G4 为例） */
const ring_storage_config_t cfg = {
    .start_addr        = 0x08078000,
    .area_size         = 8192,
    .sector_size       = RING_STORAGE_SECTOR_2K,
    .write_gran        = 64,
    .frame_buffer      = s_frame_buf,
    .frame_buffer_size = sizeof(s_frame_buf),
};

/* 3. 注册 KV（value 必须为静态/全局变量） */
ring_storage_init(&s_storage, &cfg);
ring_storage_register(&s_storage, "motor_poles", &g_poles, sizeof(g_poles));
ring_storage_register(&s_storage, "pid_kp",      &g_kp,      sizeof(g_kp));

/* 4. 启动时加载 */
ring_storage_load(&s_storage);

/* 5. 修改参数后持久化 */
ring_storage_save(&s_storage);
```

## 与 EasyFlash 对比

| 特性          | EasyFlash NG      | ring_storage      |
|---------------|-------------------|-------------------|
| 模型          | 单 KV 独立存储    | 整包快照存储      |
| 固定开销      | 48B/ENV（64bit）  | 28B/帧            |
| 查找          | 全扫描/缓存       | O(1) 内存注册表   |
| GC            | 逐个搬迁 ENV      | 整帧复制          |
| 磨损均衡      | 扇区轮转          | Round-Robin 轮转   |
| 断电保护      | PRE_WRITE+WRITE   | commit_magic 单点 |
| 适用场景      | KV 多、增删频繁   | 参数集小、批量保存|

## 依赖

- `rs_crc32.h/rs_crc32.c`：CRC32 校验（8 路展开查表法，多项式 0xEDB88320）
- `ring_storage_port.h`：平台抽象层（read/write/erase/lock/unlock，5 个接口）
- `log.h`：日志输出（通过 `RING_STORAGE_LOG_ENABLE` 宏开关控制）
