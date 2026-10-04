# 基于 CAN 总线的 STM32 Bootloader 固件升级系统

> 通过 CAN 总线对 STM32F103 进行远程固件升级（IAP）的完整实现。
> PC 端用 Python + PCAN-USB 发送 `.bin` 固件，STM32 端 Bootloader 接收、
> 写入 Flash、CRC32 校验、跳转执行。支持异常注入测试与掉电保护。

---

## 1. 项目概述

### 1.1 要解决的问题

嵌入式设备安装在现场后，如果需要更新固件：

- 拆机、接调试器（ST-Link/J-Link）成本高，有些场景根本进不去
- 设备可能只有一根通信线可用（CAN / RS485 / UART）

**Bootloader（IAP，In-Application Programming）就是解决方案**：
设备自己带一段"引导程序"，上电时检查应用是否有效，有效就跳过去执行，
需要升级时通过既有通信接口接收新固件写进 Flash。

### 1.2 本项目的实现

```
┌──────────┐   USB    ┌───────────┐   CAN 总线   ┌──────────────┐
│   PC     │─────────▶│ PCAN-USB  │─────────────▶│ CAN 收发器    │
│ Python   │          │ (XCAN)    │◀─────────────│ 3.3V         │
└──────────┘          └───────────┘              └──────┬───────┘
                                                        │ CANH/CANL
                                                        │
                                                 ┌──────▼───────┐
                                                 │ STM32F103C8T6│
                                                 │  Bootloader  │
                                                 └──────────────┘
```

**完整功能闭环：**

```
PC 选择 .bin 文件
    ↓
CAN 分包发送（871 帧 / 5226 字节，约 1.5 秒）
    ↓
STM32 擦除 APP 区（107 页）
    ↓
逐帧写入 Flash + 逐半字读回校验
    ↓
CMD_FINISH 携带 CRC32 → STM32 从 Flash 重算并比对
    ↓
校验通过 → 写入元数据页 → 跳转 APP
    ↓
APP 开始运行
```

### 1.3 实测数据

| 指标 | 数值 |
|---|---|
| 固件大小 | 5226 字节（5224 + 2 字节对齐填充） |
| 分包数 | 871 帧（每帧 6 字节） |
| 传输耗时 | **1.5 秒**（约 580 帧/秒） |
| 擦除耗时 | 约 3 秒（107 页 × ~28ms） |
| Bootloader 占用 | 约 12 KB Flash / 1.3 KB RAM |
| CAN 波特率 | 500 kbps（采样点 83.3%） |

---

## 2. 硬件环境

### 2.1 物料清单

| 器件 | 型号 / 说明 |
|---|---|
| 主控 | STM32F103C8T6 最小系统板（Blue Pill） |
| CAN 收发器 | 3.3V CAN 收发器模块（TJA1050 类，4 脚：TX/RX/GND/3V3 + CANH/CANL） |
| USB-CAN | XCAN-USB（PCAN-USB 兼容，Geehy APM32F072） |
| 调试器 | ST-Link V2 |
| 串口 | USB-TTL（CH340），用于 printf 调试输出 |
| 其它 | 面包板、杜邦线 |

> **注意**：芯片实测 Flash 容量为 **128 KB**（`0x1FFFF7E0` 寄存器返回 128），
> 而官方 STM32F103C8T6 标称只有 64 KB。这是国内常见的"超配"C8T6
> （通常是 F103CB 的 die）。本项目按 128 KB 规划分区。

### 2.2 接线

**CAN 部分（核心）**

| STM32 | 收发器 | 说明 |
|---|---|---|
| PA12 | TX | CAN_TX，GPIO 配置为 `AF_PP` |
| PA11 | RX | CAN_RX，GPIO 配置为 `Input / No pull` |
| 3V3 | 3.3V | 收发器供电 |
| GND | GND | 共地 |
| — | CANH | 接 XCAN-USB 的 CANH |
| — | CANL | 接 XCAN-USB 的 CANL |

**串口调试**

| STM32 | USB-TTL |
|---|---|
| PA9（TX） | RXD |
| PA10（RX） | TXD |
| GND | GND |

> USB-TTL 的 VCC **不接**（Blue Pill 由自己的 USB 供电），否则会双电源冲突。

**终端电阻**

- XCAN-USB 侧：红色拨码开关置 **ON**（内置 120Ω）
- 收发器侧：模块自带或外接 120Ω
- 总线上**两端各一个** 120Ω，总阻值约 60Ω（可用万用表测量确认）

### 2.3 时钟与波特率计算

```
HSE 8 MHz → PLL × 9 → SYSCLK 72 MHz
                        ├─ AHB  = 72 MHz
                        └─ APB1 = 36 MHz  ← CAN 挂在这里

CAN 位时间（36 MHz 时钟）：
   Prescaler = 4          → TQ = 4 / 36MHz = 111.1 ns
   BS1       = 14 TQ
   BS2       = 3 TQ
   SJW       = 1 TQ
   ────────────────────────────────
   总位时间  = 1 + 14 + 3 = 18 TQ = 2 μs  →  500 kbps
   采样点    = (1 + 14) / 18 = 83.3%      ← 业界推荐 75~87.5%
```

---

## 3. Flash 分区设计

### 3.1 布局

```
0x08000000 ┌──────────────────────────┐
           │  Bootloader              │  20 KB   （64 页，永不自我擦除）
0x08005000 ├──────────────────────────┤
           │  Application (APP)       │ 107 KB   （107 页，升级目标）
0x0801FC00 ├──────────────────────────┤
           │  Metadata                │   1 KB   （1 页）
0x08020000 └──────────────────────────┘
```

### 3.2 为什么这样分

| 约束 | 设计决定 | 原因 |
|---|---|---|
| F103 的 Flash 按 **1 KB 页**擦除 | 所有分区按 1 KB 对齐 | 只能整页擦，不能字节擦 |
| Bootloader 必须绝对可靠 | 放在最低地址，**升级流程永不擦除自己** | 擦除范围严格限制在 APP 页（第 20~126 页） |
| 需要判断"APP 是否完整" | 单独留 1 页存**元数据** | 单靠 SP/PC 无法发现"写了一半" |
| APP 大小未知 | 元数据里存 `fw_size` | 校验时才知道 CRC 该算多少字节 |
| Flash 可能位翻转 | 元数据里存 `fw_crc` | 上电校验能发现静默损坏 |
| 芯片有 128 KB | APP 给足 107 KB | 留出充足空间，也便于以后做双区升级 |

### 3.3 元数据结构

占用元数据页的前 16 字节：

```c
typedef struct {
    uint32_t magic;      // 0xA5A5A5A5 表示"元数据有效"
    uint32_t fw_size;    // 固件字节数
    uint32_t fw_crc;     // 固件 CRC32（MPEG-2 变体）
    uint32_t reserved;   // 预留（版本号 / 时间戳等）
} AppMeta_t;
```

**为什么用 magic 而不是"全 FF 判断"**：Flash 擦除后是 `0xFFFFFFFF`，
如果只判断"非全 FF"，一个被写坏的页也可能被误认为有效。
显式的 magic 值让判断更明确。

---

## 4. CAN 通信协议

### 4.1 帧定义

| CAN ID | 方向 | 名称 | 数据格式 |
|---|---|---|---|
| `0x100` | PC → STM32 | 命令帧 | `data[0]` = 命令码<br>（CMD_FINISH 时 `data[1..4]` = CRC32 小端） |
| `0x101` | PC → STM32 | 数据帧 | `data[0..1]` = 序号（小端）<br>`data[2..7]` = 6 字节固件数据 |
| `0x102` | STM32 → PC | 应答帧 | `data[0]` = 状态码<br>`data[1..2]` = 期望的下一个序号（小端） |

### 4.2 命令码

| 值 | 名称 | 行为 |
|---|---|---|
| `0x01` | CMD_START | 进入升级模式，**立即回 ACK**（不擦除，握手要快） |
| `0x02` | CMD_ERASE | 先清元数据页，再擦除 APP 区（约 3 秒），完成后回 ACK |
| `0x03` | CMD_FINISH | 校验 CRC32 → 写元数据 → 回 ACK → 请求跳转 |

### 4.3 状态码

| 值 | 名称 | 含义 |
|---|---|---|
| `0x00` | ST_OK | 成功 |
| `0x01` | ST_ERR_SEQ | 序号错误（乱序或重复），ACK 里带上期望序号 |
| `0x02` | ST_ERR_CMD | 未知命令 |
| `0x03` | ST_ERR_FLASH | Flash 擦除/写入失败 |
| `0x04` | ST_ERR_CRC | CRC 校验失败 |
| `0x05` | ST_ERR_SIZE | 地址越界 |

### 4.4 协议设计要点

**① 停等式流控（Stop-and-Wait）**

每发一帧必须等 ACK 才能发下一帧。

- 优点：实现简单，序号空间只需 16 位
- 缺点：带宽利用率低（但 500 kbps 下实测仍有 580 帧/秒，够用）
- 为什么不用滑动窗口：升级是低频操作，1.5 秒 vs 1.2 秒没有意义，简单可靠更重要

**② 序号严格匹配 + 缓冲区冲刷**

PC 端每次等 ACK 前先 `flush()` 清空接收缓冲，并且只接受**序号匹配**的 ACK。

> **这是踩过的坑**：握手阶段会连发很多次 `CMD_START`，每次 STM32 都回一个 ACK。
> 这些 ACK 堆积在 PC 的接收缓冲里，导致后面等"第 0 帧 ACK"时读到了"握手时的旧 ACK"，
> 序号对不上 → 误判失败。
>
> **修法**：`wait_ack(expect_seq)` 只在序号匹配时返回；每帧发送前 `flush()`。

**③ CRC 放在 CMD_FINISH 而不是单独命令**

少一次往返，且"校验"和"结束"语义上本来就是一件事。

**④ CMD_START 与 CMD_ERASE 分离**

最初 CMD_START 直接擦除，结果是"要等 3 秒才回 ACK"，PC 端握手超时。
分离后握手变成毫秒级，擦除单独一步给足超时时间。

---

## 5. 升级流程（时序）

```
 PC (Python)                        STM32 (Bootloader)
     │                                    │
     │  上电 / 复位                        │
     │                                    ├─ app_is_valid() ?
     │                                    │    读元数据页
     │                                    │    magic == 0xA5A5A5A5 ?
     │                                    │    重算 Flash CRC 比对 ?
     │                                    │
     │                                    ├─[有效] 开 3 秒升级窗口
     │  ──── CMD_START ──────────────────▶ │
     │  ◀─── ACK(OK, 0) ────────────────── │   （窗口内收到命令 → 留在 bootloader）
     │                                    │
     │  ──── CMD_ERASE ──────────────────▶ │
     │                                    ├─ 清元数据页（先让 APP 变"无效"）
     │                                    ├─ 擦除 107 页（约 3 秒）
     │  ◀─── ACK(OK, 0) ────────────────── │
     │                                    │
     │  ──── DATA seq=0, 6 bytes ────────▶ │
     │                                    ├─ 序号检查
     │                                    ├─ 写 3 个半字 + 逐个读回
     │  ◀─── ACK(OK, 1) ────────────────── │
     │           ⋮  重复 871 次  ⋮          │
     │  ──── DATA seq=870 ───────────────▶ │
     │  ◀─── ACK(OK, 871) ──────────────── │
     │                                    │
     │  ──── CMD_FINISH + CRC32 ─────────▶ │
     │                                    ├─ 从 Flash 重算 CRC
     │                                    ├─ 比对
     │                                    ├─ 一致 → 写元数据页
     │  ◀─── ACK(OK, 871) ──────────────── │
     │                                    │
     │                                    ├─ JumpToApp()
     │                                    │
     │                                    ▼  APP 运行
```

---

## 6. 关键技术点

### 6.1 printf 重定向 + MicroLIB

```c
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}
```

Keil 必须勾选 **Use MicroLIB**（Options for Target → Target），否则 `printf` 不工作。

**限制**：不要在中断里调用 `printf`。
本项目里 CAN 接收中断只做"写 Flash + 回 ACK"，所有打印都在主循环或非关键路径。

### 6.2 Flash 写入：半字 + 读回校验

F103 只能按 **16 位半字**编程，且目标必须是 `0xFFFF`（已擦除）。

```c
uint16_t half = data[i] | (data[i+1] << 8);
if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, half) != HAL_OK) → 错误
if (*(volatile uint16_t *)addr != half) → 错误     // 读回校验
```

**为什么要读回**：Flash 写入失败不一定返回错误（比如写到了未擦除的位置）。
读回能立刻发现，避免"以为写成功了"。

### 6.3 CRC32 变体选择

本项目用 **CRC-32/MPEG-2**：

```
多项式   0x04C11DB7
初始值   0xFFFFFFFF
输入反射 否
输出反射 否
结果异或 无
```

**为什么选这个变体**：它和 STM32 内置硬件 CRC 单元（`CRC->DR`）**完全一致**。
如果以后要把软件 CRC 换成硬件 CRC 加速，结果不用改，PC 端工具也不用改。

验证向量：`CRC32("123456789") = 0x0376E6E7` ✅

**⚠️ 注意**：Python 的 `zlib.crc32()` 是**反射变体**，结果不同，不能混用。

### 6.4 JumpToApp —— 最讲究细节的一步

```c
static void JumpToApp(void)
{
    uint32_t  app_sp = *(volatile uint32_t *)APP_ADDRESS;       // 向量表[0] = 初始 MSP
    uint32_t  app_pc = *(volatile uint32_t *)(APP_ADDRESS + 4); // 向量表[1] = 复位入口

    __disable_irq();                 // 1. 关中断
    HAL_CAN_DeInit(&hcan);           // 2. 关外设
    HAL_UART_DeInit(&huart1);
    HAL_DeInit();                    // 3. 复位 HAL + SysTick
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;
    for (i = 0; i < 8; i++) {        // 4. 清所有 NVIC 使能位和挂起位
        NVIC->ICER[i] = 0xFFFFFFFFUL;
        NVIC->ICPR[i] = 0xFFFFFFFFUL;
    }

    __set_MSP(app_sp);               // 5. 换栈指针
    SCB->VTOR = APP_ADDRESS;         // 6. 搬中断向量表
    __enable_irq();                  // 7. 开中断

    ((pFunction)app_pc)();           // 8. 跳过去（不返回）
}
```

**每一步为什么必要：**

| 步骤 | 不做会怎样 |
|---|---|
| 关中断 | bootloader 的中断在 APP 里触发，跳进野向量 |
| 关外设 | CAN/UART 寄存器残留状态干扰 APP 初始化 |
| 清 NVIC | 挂起的中断会在 APP 打开中断的瞬间立刻触发 |
| `__set_MSP` | 新固件有自己的栈，必须换 |
| `SCB->VTOR` | **最关键**：不搬向量表，APP 里任何一个中断都会跳到 bootloader 的向量表 |
| 用函数指针跳转 | 这是唯一能"不通过复位就改变 PC"的手段 |

### 6.5 APP 侧必须做的一件事

APP 的 `main()` 第一行：

```c
SCB->VTOR = 0x08005000UL;    // 把向量表指向自己
```

**漏了会怎样**：APP 运行正常，但一进中断就跳到 bootloader 的向量表里去 → 随机死机。
（本项目 APP 里有 `HAL_Delay`，依赖 SysTick 中断，漏了必死。）

APP 工程的 Keil 设置：`IROM1: Start 0x08005000, Size 0x1B000`

---

## 7. 异常处理与测试

### 7.1 已实现的保护

| 场景 | 保护机制 | 结果 |
|---|---|---|
| 数据帧乱序 | 序号 ≠ 期望值 | 回 `ST_ERR_SEQ` + 期望序号，PC 端重传 |
| 数据帧重复 | 同上（序号已推进） | 识别为重复，**不重复写入** |
| 数据帧丢失 | 停等式流控 + PC 端超时重传（最多 10 次） | 自动恢复 |
| 地址越界 | `offset + len > APP_MAX_SIZE` | 回 `ST_ERR_SIZE`，拒绝写 |
| 固件损坏 | 逐帧读回校验 | 立即返回 `ST_ERR_FLASH` |
| 发错固件 | CRC32 整片比对 | 回 `ST_ERR_CRC`，**拒绝跳转** |
| **升级中途断电** | **元数据先清空，最后才写** | **拒绝跳转，等待重新升级（不变砖）** |
| Flash 位翻转 | 上电时重算 CRC | 发现不一致 → 拒绝跳转 |
| 空片首次上电 | 元数据 magic 不符 | 留在 bootloader 等待升级 |

### 7.2 异常注入测试结果

上位机带 `--fault` 模式，自动跑 5 项测试：

```
C:\> python test_pcan.py --fault

[0/4] handshake ...
      bootloader responded (attempt 13)
      flushed 9 stale messages
[1/4] CMD_ERASE ...
      ACK: status=0x00 expect=0

===== TEST 1: out-of-order frame =====
  send seq=5 (STM32 expects 0)
  ACK: status=0x01 expect=0
  PASS

===== TEST 2: duplicate frame =====
  send seq=0, then send seq=0 again
  1st ACK: status=0x00 expect=1
  2nd ACK: status=0x01 expect=1
  PASS

===== TEST 3: send remaining 870 frames =====
  all sent, 1.4 sec
  PASS

===== TEST 4: wrong CRC =====
  send bad CRC = 0xACEBD88E (correct 0xACEBD88F)
  ACK: status=0x04 expect=871
  PASS

===== TEST 5: correct CRC =====
  ACK: status=0x00 expect=871
  PASS

========== SUMMARY ==========
  [PASS] out-of-order rejected
  [PASS] duplicate detected
  [PASS] remaining frames
  [PASS] wrong CRC rejected
  [PASS] correct CRC accepted
```

**掉电测试（手动）**：升级到 50% 时拔电源 → 重新上电：

```
[BOOT] meta: magic=0xFFFFFFFF size=4294967295 crc=0xFFFFFFFF
[BOOT] bad magic -> invalid
[BOOT] *** NO VALID APP *** stay in bootloader
```

设备没有变砖，重新升级即可恢复。

---

## 8. 上位机工具（Python + python-can）

### 8.1 环境

```
Python 3.x
pip install python-can
```

XCAN-USB 需要安装 PEAK-System 的 PCAN 驱动，设备会以
`interface='pcan', channel='PCAN_USBBUS1'` 的形式出现。

> **同一个 PCAN 设备同时只能被一个程序占用。** 跑 Python 前必须关掉 PCAN-View。

### 8.2 用法

```
python test_pcan.py <firmware.bin>     # 正常升级
python test_pcan.py --fault            # 异常注入测试
```

### 8.3 关键实现

**握手（处理"设备可能正在跑 APP"的情况）**

```python
for attempt in range(40):
    send_cmd(bus, CMD_START)
    st, sq = wait_ack(bus, None, timeout=0.5)
    if st == 0x00:
        break
    # 没响应 → 提示用户按复位键，继续尝试
```

因为 bootloader 的升级窗口只有 3 秒，PC 端必须**持续重试**直到设备进入窗口。

**CRC 实现（和 STM32 端逐位一致）**

```python
def crc32_mpeg(data):
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= (b << 24)
        for _ in range(8):
            if crc & 0x80000000:
                crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF
            else:
                crc = (crc << 1) & 0xFFFFFFFF
    return crc
```

**抗缓冲区污染**

```python
def wait_ack(bus, expect_seq, timeout=2.0):
    ...
    if st != 0x00:            return st, sq   # 错误直接返回
    if expect_seq is None or sq == expect_seq:
        return st, sq                         # 序号匹配才接受
    # 否则是过期 ACK，继续等
```

---

## 9. 复现步骤

### 9.1 Bootloader 工程

1. STM32CubeMX 新建工程，选择 STM32F103C8T6
2. 配置：
   - RCC：HSE = Crystal/Ceramic Resonator
   - SYS：Debug = **Serial Wire**（必须，否则 SWD 锁死）
   - 时钟树：72 MHz，APB1 = 36 MHz
   - PC13：GPIO_Output（板载 LED）
   - USART1：Asynchronous，115200-8-N-1
   - CAN1：Master，Prescaler=4，BS1=14TQ，BS2=3TQ，SJW=1TQ，Normal 模式
   - NVIC：使能 **CAN RX0 interrupt**
3. 生成 MDK-ARM 工程
4. Keil 设置：
   - Options → Target → 勾选 **Use MicroLIB**
   - Options → Target → IROM1: `0x08000000` / `Size 0x10000`
   - Options → Debug → ST-Link Debugger
5. 用本文档配套的 `main.c` 替换生成的 `Core/Src/main.c`
6. 编译烧录

### 9.2 APP 工程

复制 Bootloader 工程目录，修改：

- Keil → Options → Target → IROM1: **Start `0x08005000`, Size `0x1B000`**
- `main()` 第一行加 `SCB->VTOR = 0x08005000UL;`
- 删掉 bootloader 的逻辑，换成自己的应用代码
- Keil → Options → User → After Build/Rebuild：
  ```
  "D:\keli\core\ARM\ARMCC\Bin\fromelf.exe" --bin -o "..\App.bin" "#L"
  ```
  这样每次编译自动生成 `.bin`

### 9.3 升级

```
python test_pcan.py D:\Stm32cubemax\App2\App.bin
```

---

## 10. 已知限制与可改进方向

| # | 限制 | 改进方向 |
|---|---|---|
| 1 | 元数据只有一份，写入过程中断电会丢失 | 双份元数据（A/B）+ 序号，取最新的有效那份 |
| 2 | 升级时 APP 区被整片擦除，升级失败期间设备不可用 | **双区升级（A/B 分区）**：写到备用区，校验通过再切换 |
| 3 | 只有 Bootloader 支持 CAN，APP 里没有 CAN | APP 加一个"请求升级"命令 → 写 magic 到 RAM + 复位 |
| 4 | 单帧 6 字节，传输效率不高 | 支持 CAN-FD，或改成"多帧不等 ACK + 批量确认" |
| 5 | 没有加密/签名 | 加 AES 或 ECDSA 签名，防止固件被替换 |
| 6 | Bootloader 无看门狗 | 加 IWDG，防止升级流程中死等 |
| 7 | CRC 结果不与元数据交叉验证固件版本 | 元数据加 version / build time |

---

## 11. 面试要点

### 11.1 一分钟介绍（背下来）

> 我做了一个基于 CAN 总线的 STM32 Bootloader 固件升级系统。
> PC 端用 Python 和 PCAN-USB，通过 CAN 总线把 `.bin` 固件发给 STM32F103，
> Bootloader 负责擦除、写入 Flash、CRC32 校验，最后跳转到新固件执行。
> 5226 字节的固件大约 1.5 秒传完。
> 我设计了 `/0x100` 命令帧、`0x101` 数据帧、`0x102` 应答帧这样一套简单协议，
> 用停等式流控保证可靠性；还做了元数据页 + CRC 上电校验，保证升级中途断电也不会变砖。
> 最后我写了异常注入测试，验证了乱序、重复、CRC 错误、掉电这几种情况的处理。

### 11.2 常见追问

**Q：为什么用停等式流控，不用滑动窗口？**
> 升级是低频操作，1.5 秒和 1.2 秒的差别对用户体验没有影响。
> 停等式实现简单、状态少、不容易出 bug，对 Bootloader 这种"出错就很难救"的代码
> 来说，简单可靠比带宽利用率更重要。如果以后传几 MB 的固件，再换成滑动窗口。

**Q：怎么保证"发了一半断电"不会变砖？**
> 两个设计配合：
> 一是**擦除时先清掉元数据页**（magic 变 0xFFFFFFFF），这样只要开始升级，
> 设备就立刻变成"APP 无效"状态；
> 二是**元数据只有在整片 CRC 校验通过后才写入**。
> 所以无论在哪一步断电，上电后读到的元数据要么是空的、要么是旧的但 CRC 对不上的，
> bootloader 都会拒绝跳转，停在升级模式等重新升级。

**Q：为什么 APP 也要设 VTOR？Bootloader 不是已经设过了吗？**
> Bootloader 设 VTOR 是为了让自己跳转后 APP 的中断能正确响应。
> 但 APP 自己重新初始化时会调 `HAL_Init()`，某些情况下会重置 SCB 相关配置；
> 更重要的是，如果 APP 是通过 ST-Link 单独烧录的（不经过 Bootloader），
> 那 VTOR 就没人设了。所以 APP 必须自己保证 VTOR 正确，这是它的责任。

**Q：跳转前为什么要关中断、清 NVIC？**
> 因为 bootloader 里可能有中断处于"已使能但未处理"的状态。
> 如果不清，APP 打开中断的瞬间这些中断会立刻触发，
> 而此时向量表刚切换、外设刚重新初始化，很容易跑飞。
> 清 ICER（使能位）和 ICPR（挂起位）是保证"干净交接"的必要步骤。

**Q：CRC 为什么用 MPEG-2 变体而不是常见的 zlib CRC32？**
> 因为 STM32F1 内置硬件 CRC 单元用的就是这个变体（poly 0x04C11DB7、
> 初值全 1、不反射、不异或）。选它就意味着以后如果 CRC 计算成为瓶颈，
> 可以直接换成硬件外设而不用改协议、也不用改上位机。
> 我实测验证过：`CRC32("123456789") = 0x0376E6E7`，和标准测试向量一致。

**Q：调试过程中遇到最有意思的问题是什么？**
> 是"过期 ACK 污染"。握手阶段 PC 会连发很多次 CMD_START，
> STM32 每次都回一个 ACK，这些 ACK 堆积在 PC 的接收缓冲区里。
> 后面 PC 等"第 0 帧的 ACK"时，读到的却是握手时留下的旧 ACK，序号对不上，
> 于是误判失败、反复重传。这个问题很隐蔽，因为它是概率性的 ——
> 握手一次成功就没事，握手多次就出问题。
> 我的修法是两点：**等待 ACK 时必须校验序号，序号不匹配就丢弃继续等**；
> **每帧发送前先清空接收缓冲**。
> 这个经历让我理解了为什么串口/CAN 协议里序号校验是必须的，而不是可选的。

---

## 12. 仓库结构

```
stm32-can-bootloader/
├── README.md                      本文档
├── LICENSE                        MIT
├── .gitignore                     排除 Keil 构建产物和用不到的 CMSIS 大文件
│
├── bootloader/                    ★ Bootloader 工程（Keil MDK，链接在 0x08000000）
│   ├── Core/
│   │   ├── Inc/                   main.h / stm32f1xx_hal_conf.h / stm32f1xx_it.h
│   │   └── Src/
│   │       ├── main.c             ★ Bootloader 主体
│   │       │                        · CAN 协议（命令 / 数据 / ACK）
│   │       │                        · Flash 擦除 / 半字写入 / 读回校验
│   │       │                        · CRC32（MPEG-2 变体）
│   │       │                        · 元数据页读写
│   │       │                        · APP 有效性校验
│   │       │                        · JumpToApp
│   │       ├── stm32f1xx_it.c     中断入口
│   │       ├── stm32f1xx_hal_msp.c  外设时钟/GPIO 底层初始化
│   │       └── system_stm32f1xx.c
│   ├── Drivers/                   STM32F1 HAL + CMSIS
│   ├── MDK-ARM/                   Keil 工程文件
│   └── Bootloader2.ioc            CubeMX 配置文件（可重新生成 Drivers）
│
├── app/                           APP 示例工程（链接在 0x08005000）
│   ├── Core/Src/main.c            APP 主体（含 SCB->VTOR 设置）
│   ├── Drivers/
│   ├── MDK-ARM/
│   └── App2.ioc
│
└── tools/
    └── test_pcan.py               PC 上位机
                                     · 正常升级模式
                                     · --fault 异常注入测试模式
```

> **两个工程的区别**：`bootloader/` 的 IROM1 是 `0x08000000`，
> `app/` 的 IROM1 是 `0x08005000`。两者除了链接地址和 main.c 内容，
> CubeMX 配置（时钟 / CAN / 串口）完全相同。

---

## 附：踩过的坑（备忘）

| 现象 | 原因 | 解决 |
|---|---|---|
| `Undefined symbol hcan` | CubeMX 生成的句柄叫 `hcan`，代码里写成 `hcan1` | 全局替换 |
| 串口输出乱码 / `00` | USB-TTL 的 VCC 也接了，双电源冲突 | 只接 TXD/RXD/GND |
| `HAL_CAN_Start` 失败 | PA11（CAN_RX）没接，F103 的 CAN_RX 是浮空输入 | 接上收发器的 RXD |
| CAN 发不出去，邮箱堵死 | 总线上没有其他节点 ACK | 至少两个节点，或先用 Loopback 模式 |
| 数据帧第 0 帧就 FAILED | 握手产生的旧 ACK 污染缓冲区 | 序号严格匹配 + 每帧前 flush |
| 加 after-build 后没生成 .bin | Keil 只在真正链接时才执行 User 命令 | 用 Rebuild all，或手动 fromelf |
| 改完文件 Keil 里没变化 | Keil 缓冲区把外部修改覆盖回去了 | **改文件前先关 Keil** |
| `error: #18: expected a ")"` | 输入法把 `!=` 变成了全角的 `! =` | 改文件用 copy 覆盖，不要手打 |
