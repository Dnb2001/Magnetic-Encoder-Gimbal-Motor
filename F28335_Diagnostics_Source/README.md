# F28335 UDS / ISO-TP 学习版

这套代码把诊断链路拆成四层，目的是让你能顺着一次请求看清前后逻辑，而不是把 PCI 位域、CAN 邮箱和 UDS 服务混在一个 `UDS.h` 里。

## 建议阅读顺序

1. `Diagnostic.c`：先看每 1 ms 整体做什么。
2. `UDS.c`：看完整 PDU 如何按 SID 查服务表、检查 Session、生成正负响应。
3. `IsoTp.c`：看 PDU 如何变成 SF/FF/CF，以及 FC、BS、STmin、超时如何驱动状态机。
4. `CANIF.c`：最后看 CAN 帧如何进入 MBOX3/MBOX4 和 ISR 环形队列。

主要调用链：

```text
MBOX4 -> eCAN ISR -> CANIF RX Queue -> IsoTp_Task1ms
      -> 完整 UDS PDU -> UDS_ProcessRequest
      -> IsoTp_Send -> SF 或 FF/CF -> CANIF_Send -> MBOX3
```

## 文件用途

| 文件 | 用途 |
| --- | --- |
| `Diagnostic_Config.h` | CAN ID、邮箱、缓存、BS、STmin 和超时配置 |
| `Diagnostic_Types.h` | F28335/桌面测试共用类型和 `CAN_Frame` |
| `CANIF.h/.c` | eCAN MBOX3/MBOX4、ISR 队列和发送邮箱 |
| `CANIF_ByteCodec.h/.c` | 根据当前 `CANMC.DBO` 映射 MDL/MDH 字节 |
| `IsoTp.h/.c` | ISO-TP 双向状态机 |
| `UDS.h/.c` | UDS Session、服务表、0x10 和 0x27 骨架 |
| `Diagnostic.h/.c` | 诊断初始化和唯一 1 ms 顶层入口 |
| `Integration_Example.c` | 主工程初始化、周期任务和 ISR 接入位置 |
| `tests/` | 桌面 GCC 自动测试，不加入 CCS 目标工程 |

## 放入 CCS 工程

生产工程加入这些源文件：

```text
CANIF.c
CANIF_ByteCodec.c
IsoTp.c
UDS.c
Diagnostic.c
```

并加入同目录下所有 `.h`。`Integration_Example.c` 可以只阅读，也可以把其中三个调用函数合并进你的应用代码。`tests/` 不加入 F28335 工程。

前提条件：

- 你的工程已经完成 eCAN-A 时钟、GPIO 和 CAN 位速率初始化；
- 在基础 eCAN 初始化完成后调用 `Diagnostic_Init()`；
- 每 1 ms 调用一次 `Diagnostic_Task1ms()`；
- `ECAN0INTA_ISR` 调用 `CANIF_RxISRHandler()`；
- 继续由最外层 ISR 写 `PIEACK_GROUP9`；
- 不要在 10 kHz FOC/ADC ISR 中执行 UDS 状态机。

本模块不会写 `CANBTC`，因为位速率取决于你的 SYSCLK、BRP、TSEG1、TSEG2 和采样点；也不会强制修改全局 `CANMC.DBO`。发送和接收时会读取当前 DBO，自动选择正确的 MDL/MDH 字节次序，因此不会为了诊断邮箱改变 MBOX0~2 的数据布局。

## 默认邮箱和 CAN ID

| 方向 | 邮箱 | 标准 ID |
| --- | ---: | ---: |
| 诊断仪 -> ECU | MBOX4（RX） | `0x7E0` |
| ECU -> 诊断仪 | MBOX3（TX） | `0x7E8` |

MBOX0、MBOX1、MBOX2 不会被初始化、读取或清标志。更换 ID 时只改 `Diagnostic_Config.h`。这一版寄存器字段固定使用 MBOX3/MBOX4；若以后改邮箱号，除了宏，还要同步改 `CANIF.c` 中的寄存器位字段，因为 TI 头文件的 `ME3/MD3/TRS3` 不是可用变量索引的数组。

## 用 CANoe / TSMaster 先测这几组

所有请求发到 `0x7E0`，监听 `0x7E8`。

### 1. 进入 Extended Session

```text
TX: 02 10 03 AA AA AA AA AA
RX: 06 50 03 00 32 01 F4 AA
```

其中：

- `02` 是 ISO-TP SF_DL，不属于 UDS；
- `10 03` 才是交给 UDS 的 PDU；
- `50` 是 `0x10 + 0x40` 的正响应 SID；
- `00 32` 表示 P2ServerMax=50 ms；
- `01 F4` 的单位是 10 ms，表示 P2*ServerMax=5000 ms。

### 2. 未支持的 SID

```text
TX: 01 99 AA AA AA AA AA AA
RX: 03 7F 99 11 AA AA AA AA
```

### 3. Default Session 请求 SecurityAccess

```text
TX: 02 27 01 AA AA AA AA AA
RX: 03 7F 27 7F AA AA AA AA
```

先发 `10 03` 进入 Extended Session 后再请求 seed：

```text
TX: 02 27 01 AA AA AA AA AA
RX: 06 67 01 12 34 56 78 AA
```

`12 34 56 78` 是明确标注的教学 seed。当前没有项目 Key 算法，所以 `27 02` 的任何 Key 都返回 NRC `0x35 InvalidKey`，安全等级不会被错误解锁。

### 4. 验证 ECU 接收多帧

先发送总长度 10 的 FF：

```text
TX: 10 0A 22 F1 90 01 02 03
RX: 30 08 00 AA AA AA AA AA
```

收到 FC CTS 后再发送：

```text
TX: 21 04 05 06 07 08 09 0A
```

拼成的 UDS PDU 为 10 字节；由于示例 PDU 对 `0x22` 的长度和 DID 没有实际 handler，最终会收到相应负响应。这里主要观察 FF 后是否立即回复 `30 08 00`、CF SN 是否从 1 开始。

## CCS Watch 建议变量/接口

状态结构体在各 `.c` 内是 `static`，打开调试符号时仍可在对应编译单元观察。更通用的做法是调用公开查询接口：

```text
IsoTp_GetTxState()
IsoTp_GetRxState()
IsoTp_GetError()
IsoTp_GetRxLength()
UDS_GetCurrentSession()
UDS_GetSecurityLevel()
CANIF_GetRxOverflowCount()
```

常见状态：

- `ISOTP_TX_WAIT_FC`：FF 已发出，正在等诊断仪 FC；
- `ISOTP_TX_WAIT_STMIN`：两个 CF 之间等待 STmin；
- `ISOTP_RX_WAIT_CF`：已收 FF，正在等后续 CF；
- `ISOTP_ERROR_WRONG_SN`：收到的 CF 序号不是期望值；
- `ISOTP_ERROR_N_BS_TIMEOUT`：发送 FF 后等 FC 超时；
- `ISOTP_ERROR_N_CR_TIMEOUT`：接收 FF 后等 CF 超时。

## 文字编码

新文件使用 UTF-8 中文注释。若 CCS 显示乱码，把 Workspace 或 Project 的 Text file encoding 设为 UTF-8。原来的 `UDS.h` 很可能是文件编码与 CCS 解码方式不一致，并不代表中文注释本身不能使用。

## 当前边界

- 经典 CAN、8 字节、11 位标准帧；
- Normal Addressing、单物理诊断连接；
- 不支持 CAN FD、扩展寻址、功能寻址和并发请求；
- `0x10` 可运行；`0x27` 只有安全的教学状态骨架；
- 尚未加入 `0x3E/0x22/0x31/0x34/0x36/0x37`；
- 尚未实现耗时服务需要的 NRC `0x78 ResponsePending`、P2/P2* 调度、Flash 擦写和 Bootloader 跳转。

先把 CAN TX/RX、SF 和两方向多帧传输在 CANoe/TSMaster 跑稳，再添加具体 UDS 服务，会比一次把 Bootloader 全部堆进去更容易定位问题。

