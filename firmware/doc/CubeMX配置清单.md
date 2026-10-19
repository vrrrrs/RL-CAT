# CubeMX 配置清单与核对表

> **文档编号**：FW-CFG-001
> **版本**：v1.0
> **日期**：2026-10-09
> **工程路径**：`firmware/stm32/stm32.ioc`（相对项目根目录 `RL-CAT/`）
> **配套文档**：[固件设计书.md](固件设计书.md)、[伺服总线模块设计.md](伺服总线模块设计.md)

---

## 1. 当前配置状态总览

**MCU**：STM32F103C8Tx（LQFP48）
**CubeMX**：6.4.0，固件包 FW_F1 V1.8.7
**工具链**：MDK-ARM V5.32

| 类别 | 状态 |
|---|---|
| ✅ 已正确 | 时钟、UART 中断使能、**三路波特率**（USART1/2 = 1 Mbps，USART3 = **115200**）、ADC、DMA、GPIO、SWD |
| ⚠️ 需修正 | **中断优先级**（三个 UART 抢占级都是 0，需区分） |
| ❌ 待补充 | 1 kHz 时基定时器、Flash 存储区、UART DMA、JTAG 重映射 |

---

## 2. ✅ 已确认正确的配置

### 2.1 时钟树（⚠️ 最关键，已修好）

```
HSE = 8 MHz 外部晶振（PD0-OSC_IN / PD1-OSC_OUT）
  ↓
PLL × 9  →  72 MHz
  ↓
SYSCLK = 72 MHz
AHB   = 72 MHz（DIV1）
APB1  = 36 MHz（DIV2）   ← USART2、USART3 挂这里
APB2  = 72 MHz（DIV1）   ← USART1、ADC 挂这里
ADC   = 12 MHz（APB2/6）
```

**⚠️⚠️ 上电前必须确认：板子上的晶振真的是 8 MHz。**

```
如果晶振是 12 MHz → PLL×9 = 108 MHz → 超频，可能不稳定
如果晶振是 8 MHz  → 72 MHz ✅ 正确
```

**验证方法：**

```c
// 在 main 里临时加
volatile uint32_t sysclk = HAL_RCC_GetSysClockFreq();   // 应该是 72000000
volatile uint32_t pclk1  = HAL_RCC_GetPCLK1Freq();      // 应该是 36000000
volatile uint32_t pclk2  = HAL_RCC_GetPCLK2Freq();      // 应该是 72000000

// 用 MCO 输出时钟到 PA8 用示波器量（更可靠）
// 或用 SysTick 翻转 LED，量周期
```

**⚠️ 如果 `HAL_RCC_OscConfig()` 返回失败，会进 `Error_Handler()` 死循环。** 建议在 `Error_Handler` 里加 LED 快闪，方便判断是"启动失败"还是"程序没跑"。

### 2.2 UART 配置

| 外设 | 引脚 | 波特率 | 用途 | 状态 |
|---|---|---|---|---|
| **USART1** | PA9 (TX) / PA10 (RX) | **1 000 000** | 树莓派 | ✅ |
| **USART2** | PA2 (TX) / PA3 (RX) | **1 000 000** | 舵机总线 | ✅ |
| **USART3** | PB10 (TX) / PB11 (RX) | 115200 | ESP32-C3 | ⚠️ 见 §3.2 |

**⚠️ 波特率误差核算（72 MHz，APB1 = 36 MHz，APB2 = 72 MHz）：**

```
USART1 (APB2 = 72 MHz):   72000000 / 1000000 = 72      ✅ 整数，误差 0%
USART2 (APB1 = 36 MHz):   36000000 / 1000000 = 36      ✅ 整数，误差 0%
```

**⚠️ 这是把时钟修到 72 MHz 的额外收益 —— 1 Mbps 完美整除。如果还是 8 MHz，1 Mbps 会有明显误差。**

### 2.3 中断使能（已开启）

```
NVIC.USART1_IRQn = true
NVIC.USART2_IRQn = true
NVIC.USART3_IRQn = true
NVIC.DMA1_Channel1_IRQn = true
```

**`stm32f1xx_it.c` 里已生成 5 个处理函数：**

```c
void DMA1_Channel1_IRQHandler(void)   // ADC DMA
void ADC1_2_IRQHandler(void)
void USART1_IRQHandler(void)          // 树莓派
void USART2_IRQHandler(void)          // 舵机总线 ⭐
void USART3_IRQHandler(void)          // ESP32
```

**⚠️ 官方舵机库的 `uart.c` 也定义了 `USART1_IRQHandler` / `USART2_IRQHandler` —— 会重复定义。所以必须替换掉官方 `uart.c`（见《伺服总线模块设计》§7）。**

### 2.4 ADC + DMA（电池监测）

```
ADC1.Channel-0         = ADC_CHANNEL_0      → PA0
ADC1.SamplingTime      = 239.5 cycles
ADC1.ContinuousConvMode= ENABLE             ✅ 连续转换
ADC1.NbrOfConversion   = 1
DMA1_Channel1         = ADC1, 循环模式, 半字, 内存递增
ADC 时钟               = APB2/6 = 12 MHz
```

**✅ 配置正确。连续转换 + DMA 循环 → 固件直接读 DMA 缓冲区即可，不占 CPU。**

**转换时间核算：**

```
(239.5 + 12.5) 个 ADC 周期 / 12 MHz = 21 µs
采样率约 47.6 kHz  →  远超电池监测需求（1 Hz 就够）✅
```

**⚠️ 需要在代码里调用一次 `HAL_ADC_Start_DMA()` 启动。**

### 2.5 GPIO

| 引脚 | 标签 | 模式 | 用途 | 状态 |
|---|---|---|---|---|
| **PC13** | `LED` | 推挽输出，低速 | **状态 LED（板载，低电平点亮）** | ✅ |
| **PA8** | `HERATBEAT` | 推挽输出，低速 | **心跳** | ⚠️ 标签拼写错误，见 §4.5 |
| **PB8** | `SCL` | **开漏**，高速，无上下拉 | 软件 I2C → JY901P | ✅ |
| **PB9** | `SDA` | **开漏**，高速，无上下拉 | 软件 I2C → JY901P | ✅ |
| **PA13** | — | Serial Wire (SWDIO) | 调试 | ✅ |
| **PA14** | — | Serial Wire (SWCLK) | 调试 | ✅ |

**⚠️ 引脚分工（统一口径，勿混）：**

| 引脚 | 用途 |
|---|---|
| **PA8** | **心跳**（HEARTBEAT）—— 指示固件主循环在跑 |
| **PC13** | **状态 LED** —— 指示业务状态（正常/警告/急停）。⚠️ 低电平点亮 |

**⚠️ 注意 PA8 与 MCO 冲突**：§2.1 提到"用 MCO 输出时钟到 PA8 量频"是个**临时调试手段** —— 测完必须恢复成 GPIO 输出，否则心跳功能失效。

**⚠️ PB8/PB9 是开漏 + 无内部上拉 —— 这是正确的，因为需要外部 4.7 kΩ 上拉到 3.3 V。请确认硬件上装了。**

**⚠️ PA13/PA14 保持 SWD 不动。** 绝对不要在代码里调 `GPIO_Remap_SWJ_Disable` —— 那会永久失去调试能力（只能靠 BOOT0 恢复）。

---

## 3. ⚠️ 需要修正的配置

### 3.1 🔴 中断优先级（必须改）

**当前状态：三个 UART 的抢占优先级都是 0。**

```
NVIC.USART1_IRQn = true:0:0:...     ← 抢占 0
NVIC.USART2_IRQn = true:0:0:...     ← 抢占 0（相同！）
NVIC.USART3_IRQn = true:0:0:...     ← 抢占 0（相同！）
```

**⚠️ 问题：**

```
① 抢占优先级相同 → 三者【不能互相打断】
   → 如果 USART1（树莓派）的处理函数正在跑，USART2（舵机）的字节会丢
② 优先级 0 是最高的，和 DMA(5) 的层级关系虽然对，但三者之间没有区分
```

**👉 目标配置（`NVIC_PRIORITYGROUP_4` = 4 位抢占，0 位子优先级）：**

| 中断 | 抢占优先级 | 理由 |
|---|---|---|
| **USART2**（舵机总线） | **6** ⭐ 最高 | 1 Mbps，丢字节 = 丢整帧，重发代价高 |
| **USART1**（树莓派） | **7** | 可以丢帧（有 SEQ + CRC），允许重发 |
| **USART3**（ESP32） | **8** | 非关键，可丢 |
| **DMA1_Channel1**（ADC） | **5** | 保持现状（最高） |
| **TIM2**（1 kHz 节拍，待加） | **9** | 调度用，最低 |
| **SysTick** | **15** | 保持（HAL 时基） |

**⚠️ 优先级数字越小 = 越高。所以 5 > 6 > 7 > 8 > 9 > 15。**

**修改方式（CubeMX）：**

```
Pinout & Configuration → System Core → NVIC
  → 找到 USART2 global interrupt，把 Preemption Priority 改成 6
  → USART1 改成 7
  → USART3 改成 8
```

**⚠️ 改完必须重新生成代码。**

### 3.2 ✅ USART3 波特率 = 115200（已实现）

**✅ 工程代码里就是 115200**（`main.c` 的 `huart3.Init.BaudRate = 115200`）。

| 用途 | 波特率 | 状态 |
|---|---|---|
| **ESP32-C3 无线** | **115200** | ✅ 已配（2026-10-10 由 921600 下调）|

**⚠️ 为什么下调（务必连同一起理解）：**

这一路不是控制通路，而是**低频调试/观察通道**，可靠性优先于带宽。
但下调波特率会**放大阻塞发送的耗时**，所以必须同时满足两条硬约束：

```
① STM32 侧发送必须【非阻塞】（中断/DMA）
   阻塞发送 103 B 的耗时 = 103 × 10 / 波特率：
       921600 → 1.12 ms    （实测把 100 Hz 控制环拖到 ~92 Hz）
       115200 → 8.93 ms    （会砸到 ~53 Hz，控制环崩溃）
   而 RL 的 decimation=2 要求控制环【严格 100 Hz】。

② 镜像必须【降频】（TEL_FAST 100 Hz → 10 Hz）
   115200 线速率约 11.5 kB/s：
       100 Hz 全量 = 10.3 kB/s = 90%  → 必然丢帧
        10 Hz      = 1.5 KB/s  = 13%  → 余量充足
```

理由与实测数据见 [`docs/32-esp32-bridge.md`](../../docs/32-esp32-bridge.md) §1
与 [`docs/36-esp32-plan.md`](../../docs/36-esp32-plan.md) §2.2。
实现位置：`firmware/stm32/Core/Src/esp_fwd.c`。

**⚠️ 澄清（⚠️ 这里曾有一个概念错误）：**

```
❌ 旧说法："921600（或 115200）是 JY901P 需要的"
   错误原因：JY901P 走【软件 I2C】，与 USART3 无关，也没有波特率概念。

✅ 实际：这个数字是【ESP32-C3 无线链路】的波特率。
        JY901P 走 PB8/PB9 的软件 I2C，要配的是 I2C 时钟频率与输出速率，
        见《固件设计书》§5.4。
```

**误差核算（USART3 挂 APB1 = 36 MHz）：**

```
115200：USARTDIV = 36000000 / (16 × 115200) = 19.53125
        → 曼蒂萨 19、小数 8（8/16 = 0.5）
        → 实际波特率 = 115385，误差 0.16%

（921600 的误差同样恰好是 0.16%，两者在这一档上等价。）
→ ✅ 可接受（UART 容忍 ±2~3%）
```

**⚠️ 改完记得同步 ESP32 侧**：它的 UART1 也要配成 **115200 8N1**。

---
## 4. ❌ 待补充的配置

### 4.1 Timer —— 1 kHz 系统节拍（建议加）

**当前没有定时器。** 需要一个 1 kHz 时基用于：

```
· 主循环调度（100 Hz 任务）
· 软件超时（IMU 超时、链路超时、总线超时）
· 心率灯
```

**⚠️ 为什么不用 SysTick：** SysTick 已被 HAL 占用（做 `HAL_GetTick()`，1 ms 中断）。

**两个选择：**

| 方案 | 做法 | 评价 |
|---|---|---|
| **A. 直接用 `HAL_GetTick()`** | 不用加定时器 | ⭐ **最简单，推荐** |
| B. 加 TIM2 做独立 1 kHz | CubeMX 里配 TIM2 | 更精确，但多占一个外设 |

**👉 建议先用方案 A。** `HAL_GetTick()` 的 1 ms 精度对 100 Hz 调度完全够用。

**⚠️ 但如果后续需要微秒级计时（舵机总线超时），需要额外方案：**

```c
// 用 DWT 周期计数器（Cortex-M3 支持，零开销）
#define DWT_CYCCNT  (*(volatile uint32_t*)0xE0001004)
#define DWT_CONTROL (*(volatile uint32_t*)0xE0001000)
#define DEMCR       (*(volatile uint32_t*)0xE000EDFC)

static inline void dwt_init(void) {
    DEMCR |= (1 << 24);          // TRCENA
    DWT_CYCCNT = 0;
    DWT_CONTROL |= 1;            // CYCCNTENA
}
static inline uint32_t micros(void) { return DWT_CYCCNT / 72; }   // 72 MHz
```

**⚠️ 这样就有微秒级计时，且不占任何外设。**

### 4.2 Flash —— 标定参数存储（要加）

**STM32F103C8 的 Flash：64 KB，页大小 1 KB，共 64 页。**

```
固件占用：0x08000000 ~ 0x0800F7FF（约 62 KB，取决于优化）
标定数据：0x0800FC00 ~ 0x0800FFFF（最后 1 页）  ⭐
```

**⚠️ 不需要在 CubeMX 里配置 —— 直接在 `bsp_flash.c` 里写地址。**

**⚠️ 关键注意：**

| # | 注意 |
|---|---|
| 1 | **Flash 写之前必须先擦除整页** |
| 2 | **擦写期间 CPU 会暂停（F103 没有双 bank）**，所以要关中断 |
| 3 | **不要在 100 Hz 循环里写 Flash** —— 放到 IDLE 状态或专门的命令处理里 |
| 4 | **写入次数寿命约 10000 次** —— 不要频繁写 |

### 4.3 USART2 的 DMA（可选，但推荐）

**为了彻底避免 1 Mbps 下丢字节，建议给 USART2 加 DMA 接收。**

| 项 | 配置 |
|---|---|
| **DMA1_Channel6** | USART2_RX，循环模式，字节对齐，内存递增 |
| **DMA1_Channel7** | USART2_TX，正常模式（可选） |

**⚠️ CubeMX 里配：**

```
Connectivity → USART2 → DMA Settings → Add
  → USART2_RX → DMA1 Channel 6 → Circular → Byte
```

**⚠️ 加完之后，`stm32f1xx_it.c` 里会多出 `DMA1_Channel6_IRQHandler`。**

**⚠️ 还要在代码里手动使能 IDLE 中断（CubeMX 不生成）：**

```c
__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
```

**详见《伺服总线模块设计》§5.3。**

### 4.4 JTAG 引脚重映射（如果要用 PA15/PB3/PB4）

**当前 `stm32.ioc` 里没有这几个引脚，说明还没用到。**

**⚠️ F103 默认开启完整 JTAG，占用 PA13/PA14/PA15/PB3/PB4。所以这几个脚默认不能当 GPIO。**

**如果将来要用（比如 `方案.md` §5.5 里预留的 PB12~PB15 足底开关不冲突，但 PA15/PB3/PB4 有冲突）：**

```c
// 在 MX_GPIO_Init() 之前调用，释放 PA15/PB3/PB4，保留 SWD
__HAL_RCC_AFIO_CLK_ENABLE();
__HAL_AFIO_REMAP_SWJ_JTAGDISABLE();   // ⚠️ 只能用这个，不能用 SWJ_Disable
```

**⚠️⚠️ 绝对不要用 `__HAL_AFIO_REMAP_SWJ_DISABLE()`** —— 那会同时关掉 SWD，导致无法再调试（只能靠 BOOT0 进 bootloader 擦除）。

### 4.5 GPIO 标签拼写错误

```
PA8.GPIO_Label = HERATBEAT     →  应为 HEARTBEAT
```

**⚠️ 改标签名会导致生成的宏名变化（`HERATBEAT_Pin` → `HEARTBEAT_Pin`），如果代码里已经引用了旧名会编译失败。** 现在还早，建议改掉。

### 4.6 清理未用的 CMSIS 库

**`Drivers/CMSIS/` 下有 `DSP/` 和 `NN/` 两个目录，几百个文件，占大量磁盘空间，但完全没用到。**

**⚠️ 它们不会被编译（不在 Keil 工程的源文件列表里），所以不影响 Flash 占用和编译时间。但会让工程目录很乱、版本控制很痛苦。**

**👉 建议删掉 `Drivers/CMSIS/DSP` 和 `Drivers/CMSIS/NN`。**

---

## 5. 引脚分配总表（与 `方案.md` §5.5 对照）

| 引脚 | 配置 | 用途 | 状态 |
|---|---|---|---|
| **PA0** | ADC1_IN0 | 电池分压 | ✅ |
| PA1 | — | 空闲 | — |
| **PA2** | USART2_TX | 舵机控制板 RX | ✅ |
| **PA3** | USART2_RX | 舵机控制板 TX | ✅ |
| PA4 | — | 预留 | — |
| PA5 | — | 预留 | — |
| PA6 | — | 空闲（预留 SPI IMU） | — |
| PA7 | — | 空闲 | — |
| **PA8** | GPIO 输出 | 心跳 | ✅ |
| **PA9** | USART1_TX | 树莓派 RX | ✅ |
| **PA10** | USART1_RX | 树莓派 TX | ✅ |
| PA11 | — | 空闲（USB DM） | — |
| PA12 | — | 空闲（USB DP） | — |
| **PA13** | SWDIO | 调试 | ✅ 不动 |
| **PA14** | SWCLK | 调试 | ✅ 不动 |
| PA15 | — | **⚠️ 默认被 JTAG 占用** | 见 §4.4 |
| **PC13** | GPIO 输出 | 状态 LED（低电平点亮）| ✅ |
| PB1 | — | 备用 | — |
| **PB2** | — | **⚠️ BOOT1，必须悬空** | 不配 |
| PB3 | — | **⚠️ 默认被 JTAG 占用** | 见 §4.4 |
| PB4 | — | **⚠️ 默认被 JTAG 占用** | 见 §4.4 |
| PB5 | — | 空闲 | — |
| **PB8** | GPIO 开漏 | 软件 I2C SCL → JY901P | ✅ |
| **PB9** | GPIO 开漏 | 软件 I2C SDA → JY901P | ✅ |
| **PB10** | USART3_TX | ESP32-C3 | ✅ |
| **PB11** | USART3_RX | ESP32-C3 | ✅ |
| PB12–PB15 | — | 预留足底开关（需 5 脚连接器 + 可选 10k 上拉） | 未配 |

**⚠️ PB2 (BOOT1) 绝对不要配成任何功能，必须悬空或下拉。**

---

## 6. 代码生成设置

**当前 CubeMX 配置：**

```
ProjectManager.MainLocation      = Core/Src        ✅
ProjectManager.KeepUserCode      = true            ✅ 保护 USER CODE 区
ProjectManager.DeletePrevious    = true            ⚠️ 会删除旧的生成文件
ProjectManager.HeapSize          = 0x200 (512 B)   ⚠️ 见下
ProjectManager.StackSize         = 0x400 (1 KB)    ⚠️ 见下
ProjectManager.CompilerOptimize  = 6 (Level 3?)    ⚠️ 建议 -Os
ProjectManager.CoupleFile        = false           ✅ 每外设独立 .c
ProjectManager.NoMain            = false           ✅
```

**⚠️ 三个建议：**

| # | 项 | 建议 | 理由 |
|---|---|---|---|
| 1 | **HeapSize** | **改成 0x0** | 我们的代码【不用 malloc】；官方舵机库的 malloc 也要改掉。省 512 B RAM |
| 2 | **StackSize** | 保持 0x400，但**监控使用量** | 20 KB RAM 紧张，栈溢出很难查 |
| 3 | **优化等级** | **-Os（体积优先）** | 64 KB Flash 紧张 |

**⚠️ 栈使用量监控方法（调试用）：**

```c
// 启动时把栈区域填成 0xAA，运行一段时间后看有多少被改写
extern uint32_t _estack;
void stack_paint(void) {
    uint32_t *p = &_estack - 256;      // 栈顶往下 1 KB
    for (int i = 0; i < 256; i++) p[i] = 0xAAAAAAAA;
}
uint32_t stack_used(void) {
    uint32_t *p = &_estack - 256;
    uint32_t n = 0;
    while (n < 256 && p[n] == 0xAAAAAAAA) n++;
    return (256 - n) * 4;
}
```

---

## 7. 修改清单（按优先级）

### 🔴 必须改（不改会出问题）

```
□ 1. NVIC 中断优先级：USART2=6, USART1=7, USART3=8    ← §3.1
□ 2. 确认 HSE 晶振真的是 8 MHz                        ← §2.1
□ 3. HeapSize 改成 0x0                                 ← §6
```

### 🟡 建议改

```
□ 4. USART3 波特率 = 115200（✅ 2026-10-10 已完成，原建议 921600）← §3.2
□ 5. 优化等级改成 -Os                                  ← §6
□ 6. 删除 Drivers/CMSIS/DSP 和 /NN                     ← §4.6
□ 7. PA8 标签拼写改成 HEARTBEAT                        ← §4.5
```

### 🟢 按需加

```
□ 8. USART2 加 DMA 接收（DMA1_Channel6）              ← §4.3
□ 9. 如果要释放 PA15/PB3/PB4，加 JTAG 重映射           ← §4.4
□ 10. 如果要 1 kHz 定时器，加 TIM2                     ← §4.1
```

---

## 8. 上电自检清单

**第一次烧录后，按顺序验证：**

```
□ 1. LED 能亮/灭（GPIO 正常）
□ 2. 用调试器读 HAL_RCC_GetSysClockFreq() == 72000000
□ 3. 三个 UART 都能自发自收（USB-TTL 短接 TX-RX 测试）
□ 4. ADC 读数随电压变化（拧电位器或接电池）
□ 5. 软件 I2C 能读到 JY901P 的设备地址（示波器看 SCL/SDA 波形）
□ 6. 能 Ping 到舵机（先只接 1 个）
□ 7. 能 Ping 到全部 12 个舵机
```

**⚠️ 第 2 步最重要 —— 时钟不对，后面全部白干。**

**⚠️ 第 5 步如果没有示波器，可以用"读到的值是否合理"判断（JY901P 的加速度应该有 1g 左右）。**

---

**文档结束。修改完 CubeMX 配置后，请重新生成代码并跑一遍 §8 的自检清单。**
