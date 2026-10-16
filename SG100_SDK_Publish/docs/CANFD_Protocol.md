# SG100 SDK CANFD 通讯协议说明

## 概述

SG100 SDK 支持通过 CANFD 总线与 SG100 灵巧手掌板通信。CANFD 适配器运行于**被动模式**，SDK 仅负责协议帧的封包/解包，物理层收发由外部 CAN 驱动（如 SocketCAN）负责。

---

## 一、通信架构

```
┌─────────────────────────────────────────────┐
│  外部 CAN 驱动 (SocketCAN / 自定义)          │
│  - 从 CAN 总线接收帧 → 注入 SDK             │
│  - 从 SDK 取待发帧 → 写入 CAN 总线           │
└──────────────┬──────────────────────────────┘
               │ InjectRxFrame() / GetTxFrame()
┌──────────────▼──────────────────────────────┐
│  CANFDAdapter (被动模式)                     │
│  - 发送: RS485 帧体 → 64B 分片 → TX 队列    │
│  - 接收: CANFD 原始帧 → 流重组 → 协议体      │
└──────────────┬──────────────────────────────┘
               │ rx_cb_ / Send()
┌──────────────▼──────────────────────────────┐
│  ProtocolHandler → SG100Device              │
│  - 协议解析 / 状态缓存 / 命令分发            │
└─────────────────────────────────────────────┘
```

---

## 二、CAN ID 方案

| 参数 | 默认值 | 说明 |
|------|--------|------|
| 帧类型 | 11-bit 标准帧 | 可通过配置切换为 29-bit 扩展帧 |
| TX 起始 ID | `0x200` (512) | 发送分片的起始 CAN ID |
| RX 过滤范围 | `0x200 ~ 0x2FF` (512~767) | 接收过滤的最小/最大 ID |
| 顺序 TX ID | 默认启用 | 同一消息的分片 ID 从 `tx_start_id` 依次递增 |

### ID 配置结构

```cpp
typedef struct {
    uint32_t tx_start_id;           // 发送分片起始 ID
    uint32_t rx_filter_min_id;      // 接收过滤最小 ID (含)
    uint32_t rx_filter_max_id;      // 接收过滤最大 ID (含)
    uint8_t  use_extended_id;       // 0=标准帧(11bit), 1=扩展帧(29bit)
    uint8_t  enable_sequential_tx_id; // 0=固定 tx_start_id, 1=分片递增
    uint8_t  enable_rx_filter;      // 0=不过滤, 1=按 min/max 过滤
} CANFD_Config_t;
```

### 默认配置

```cpp
tx_start_id = 0x200;              // 对齐掌板固件
rx_filter_min_id = 0x200;
rx_filter_max_id = 0x2FF;
use_extended_id = 0;              // 标准帧
enable_sequential_tx_id = 1;       // 分片时 ID+1
enable_rx_filter = 1;             // 启用 RX 过滤
```

---

## 三、帧分片与重组

### 3.1 发送分片 (Send)

`CANFDAdapter::Send()` 将 RS485 协议帧按 64 字节分片：

```
RS485 帧体 (7+Len+4 字节)
┌───────────────────────────────────────┐
│ [0x48][0x4D][LenLo][LenHi][Dst][Src][Cmd][Data...][CRC32-LE] │
└───────────────────────────────────────┘
          │ 按 64 字节切分
          ▼
┌──────────┬──────────┬──────────┐
│ Frame #0 │ Frame #1 │ Frame #2 │  ← 各 64 字节 (尾帧可能不足)
│ CAN ID=0x200│ ID=0x201 │ ID=0x202 │  ← ID 递增
└──────────┴──────────┴──────────┘
```

- **CRC32 追加**：发送前 SDK 计算 CRC-32/BZIP2 并追加到帧尾
- **ID 递增**：`can_id = tx_start_id + fragment_index`
- **队列溢出**：TX 队列满时丢弃最旧帧（FIFO 策略，最大深度 32）

### 3.2 接收重组 (InjectRxFrame)

`CANFDAdapter::InjectRxFrame()` 将收到的 CANFD 载荷流式拼接，按帧头 `0x48 0x4D` 定位帧边界：

```
收到帧 #0: 0x48 0x4D 9A FD FE 05 ...
收到帧 #1: ...继续...
收到帧 #2: ...尾部... CRC32

         ↓ 流式拼接
  rx_stream_[512]
         ↓ 搜索 0x48 0x4D
         ↓ 按 LenLo/LenHi 组合为 uint16_t Len，计算总长 (7+Len+4)
         ↓ 检查 CRC32
         ↓ 完整帧 → 回调投递 (去除 CRC32，仅投递 7+Len 字节协议体)
```

**重组规则**：

- 连续追加到 512 字节流缓冲区
- 搜索帧头 `0x48 0x4D`，定位帧起始
- 从第 2-3 字节读取 `LenLo`/`LenHi`，组合为 uint16_t 小端序 `Len = LenLo | (LenHi << 8)`
- 计算帧总长 = `7 + Len + 4`
- 等待缓冲区累积足够字节后提取完整帧
- CRC-32/BZIP2 校验，不匹配则丢弃
- 合法帧 → 调用 `rx_cb_` 投递协议体（已剥离 CRC，仅 `[0x48][0x4D][LenLo][LenHi][Dst][Src][Cmd][Data...]`）

---

## 四、外部驱动集成接口

### 4.1 设备层 API

```cpp
// 初始化设备为 CANFD 被动模式
SG100Device* dev = mgr.AddCANFDDevice("HAND_CANFD");

// 设置 CANFD 配置
CANFD_Config_t cfg{};
cfg.tx_start_id = 0x200;
cfg.rx_filter_min_id = 0x200;
cfg.rx_filter_max_id = 0x2FF;
dev->SetCANFDConfig(cfg);

// 启动设备 (不启动后台线程，仅供协议处理)
mgr.StartAll();

// 发送 (通过 Send() 内部入 TX 队列)
dev->SetHandControl(ctrl);  // SDK 内部调用 Send()

// 取待发帧 (外部驱动调用)
CANFD_Frame_t tx_frame;
while (dev->CANFDGetTxFrame(tx_frame)) {
    // 写入 CAN 总线
    write(can_fd, &tx_frame, ...);
}

// 注入收到的帧 (外部驱动调用)
dev->CANFDProcessRxFrame(can_id, data, data_len);
```

### 4.2 帧结构

```cpp
typedef struct {
    uint32_t can_id;                    // CAN ID (不含平台标志位)
    uint8_t  data[CANFD_MAX_PAYLOAD];   // 载荷 (64 字节)
    uint8_t  data_len;                  // 有效字节数 (1~64)
} CANFD_Frame_t;
```

### 4.3 被动模式要点

- SDK 不启动后台线程（`DRIVE_MODE_PASSIVE`）
- **发送**：外部周期性调用 `CANFDGetTxFrame()` + 写入总线
- **接收**：外部将收到的帧通过 `CANFDProcessRxFrame()` 注入
- `SendAndAwait`（同步阻塞）不可用，返回 false

---

## 五、配置参数详解

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `tx_start_id` | uint32 | 0x200 | 发送分片起始 CAN ID |
| `rx_filter_min_id` | uint32 | 0x200 | RX 过滤下限（含） |
| `rx_filter_max_id` | uint32 | 0x2FF | RX 过滤上限（含） |
| `use_extended_id` | uint8 | 0 | 0=11-bit 标准帧，1=29-bit 扩展帧 |
| `enable_sequential_tx_id` | uint8 | 1 | 0=所有分片固定 `tx_start_id`，1=每分片 +1 |
| `enable_rx_filter` | uint8 | 1 | 0=接收所有 ID 的帧，1=只接收 `[min, max]` 范围 |

---

## 六、协议帧格式

### CANFD 物理帧载荷

```
Byte 0-N: RS485 帧体 (含 CRC-32)
┌──────┬──────┬───────┬───────┬─────┬─────┬─────┬──────────┬──────────┐
│ 0x48 │ 0x4D │ LenLo │ LenHi │ Dst │ Src │ Cmd │ Data...  │ CRC32-LE │
│  H   │  M   │       │       │     │     │     │ 0~65535B │   4B     │
└──────┴──────┴───────┴───────┴─────┴─────┴─────┴──────────┴──────────┘
 └─────────────── 7 字节帧头 ──────────────────┘ └─ Data ─┘ └─ CRC ─┘
```

- **帧头**：`0x48 0x4D`（"HM"），固定 2 字节
- **Len**：Data 段字节数，uint16_t 小端序（Lo 先 Hi 后），2 字节，取值范围 0~65535
- **Dst/Src/Cmd**：各 1 字节，共 3 字节
- **Data**：负载数据，Len 字节
- **CRC32**：CRC-32/BZIP2 (Poly=0x04C11DB7)，小端序，4 字节

**帧总长** = 7 + Len + 4 字节，超出 64 字节时分片发送。

### CRC-32 算法

- 多项式：`0x04C11DB7` (CRC-32/BZIP2)
- 初始值：`0xFFFFFFFF`
- 异或输出：`0xFFFFFFFF`
- 输入/输出反转：均不反转
- 计算范围：帧头 + LenLo + LenHi + Dst + Src + Cmd + Data（即 7+Len 字节）
- 对齐掌板串口协议

---

## 七、SocketCAN 集成示例

```cpp
// 设备初始化
auto& mgr = SG100DeviceManager::Instance();
mgr.AddCANFDDevice("HAND_CANFD");
SG100Device* dev = mgr.GetDevice("HAND_CANFD");

CANFD_Config_t cfg{};
cfg.tx_start_id = 0x200;
cfg.rx_filter_min_id = 0x200;
cfg.rx_filter_max_id = 0x2FF;
dev->SetCANFDConfig(cfg);
mgr.StartAll();

// 打开 SocketCAN
int can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
int enable_canfd = 1;
setsockopt(can_fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd));

// TX 线程: 从 SDK 取帧发送
std::thread tx([&]() {
    CANFD_Frame_t frame;
    while (running) {
        while (dev->CANFDGetTxFrame(frame)) {
            struct canfd_frame cfd{};
            cfd.can_id = frame.can_id & CAN_SFF_MASK;
            cfd.len = frame.data_len;
            cfd.flags |= CANFD_BRS;  // 启用比特率切换
            memcpy(cfd.data, frame.data, frame.data_len);
            write(can_fd, &cfd, sizeof(cfd));
        }
        usleep(2000);
    }
});

// RX 线程: 接收帧注入 SDK
std::thread rx([&]() {
    struct canfd_frame cfd{};
    while (running) {
        ssize_t n = read(can_fd, &cfd, sizeof(cfd));
        if (n > 0) dev->CANFDProcessRxFrame(cfd.can_id, cfd.data, cfd.len);
    }
});
```

---

## 八、与掌板固件的对应关系

| SDK (主机侧) | Plam 固件 (STM32) | 说明 |
|-------------|-------------------|------|
| `tx_start_id = 0x200` | `FRAME_START_ID = 0x200` | TX 起始 ID |
| `rx_filter = 0x200~0x2FF` | `FilterID1=0x200, FilterID2=0x2FF` | RX 过滤范围 |
| 11-bit 标准帧 | `FDCAN_STANDARD_ID` | 帧格式 |
| 分片 64B/pkt | `CAN_FRAME_PAYLOAD_BYTES = 64` | 单帧最大载荷 |
| CRC-32/BZIP2 | `Serial_CalculateCRC32()` | 校验算法 |
| 串行 TX 队列 (深度32) | TX FIFO 32 元素 | 发送队列 |
| 流式 RX 重组 (512B 缓冲) | `CAN_Assemble_t` (255B+bitmap) | 接收重组 |
| BRS 开启 | `FDCAN_FRAME_FD_BRS` | 比特率切换 |
| 被动模式 | FDCAN 主动模式 | 驱动模式（角色不同） |

> **注意**：`sg100_types.h:811` 中 `CANFD_FRAME_HEADER_SIZE = 6` 与 `sg100_protocol.h:44` 中 `HEADER_SIZE = 7` 不一致。协议层权威定义在 `sg100_protocol.h`，Len 字段为 uint16_t 小端序，帧头共 7 字节。`CANFD_FRAME_HEADER_SIZE` 的 6 字节定义为历史遗留，CANFD 适配器内部以 `SG100Protocol::HEADER_SIZE = 7` 为准。

## 九、CANFD 适配器限制

| 限制项 | 值 | 说明 |
|--------|-----|------|
| 单帧最大载荷 | 64 字节 | CANFD 标准 |
| RX 流缓冲区 | 512 字节 | 足够容纳最大协议帧 (7+255+4=266B) |
| TX 队列深度 | 32 帧 | 溢出时丢弃最旧 |
| 同步阻塞调用 | 不支持 | `SendAndAwait` 返回 false |
| 物理层 CRC | SDK 软件 CRC | CANFD 硬件 CRC 不替代软件校验 |
