# 固件联调与部署手册（Bring-up & Deployment）

> **文档编号**：DOC-FW-DEPLOY-001 ｜ **版本**：v1.0 ｜ **日期**：2026-10-10
> **面向对象**：执行上机联调与部署的人 + 树莓派侧开发者
>
> **配套文档**
> - [`34-firmware-acceptance.md`](34-firmware-acceptance.md) —— 逐项验收单（**本手册的执行清单**）
> - [`33-firmware-plan.md`](33-firmware-plan.md) —— 施工方案与取舍理由
> - [`30-firmware.md`](30-firmware.md) —— 五张关键表（关节/限位/引脚/观测/参数）
>
> **⚠️ 本手册只讲"怎么把已经写好的固件跑起来、联调、部署"**，
> 不讲实现细节（那些在上面三份里）。

---

## 0. 一页速查：三条命令与六个串口命令

### PC 侧（不需要硬件）

```powershell
# 构建固件（armclang，0 error 0 warning 才通过，并报告 Flash/RAM 占用）
powershell -ExecutionPolicy Bypass -File firmware/tools/build.ps1

# C 侧单元测试（6 个套件）
powershell -ExecutionPolicy Bypass -File firmware/tools/test-host.ps1

# Python 侧（协议自检 + 模拟器回环，不需要 pyserial）
powershell -ExecutionPolicy Bypass -File firmware/tools/test-python.ps1

# CubeMX 重新生成后：核对 Keil 工程源文件列表没丢文件
powershell -ExecutionPolicy Bypass -File firmware/tools/check-uvprojx.ps1
```

### 串口命令（USART1，1 Mbps 8N1，ASCII）

| 命令 | 用途 |
|---|---|
| `h` | 帮助 |
| `s` | 状态：时钟 / DWT / 传感器 / 三路 UART 计数 / Flash |
| `i` | I2C 自检 + 探测 JY901P |
| `A` | 舵机总线逐个 PING 1~12 |
| `T <0\|1>` | 全部舵机力矩 卸/使能 |
| `P <j> <step>` | 单关节直接控位（调试） |
| `F <step> [stride]` | 12 个关节全部到某位置 |
| `B` | 舵机总线统计 + 12 个关节反馈一览 |
| `M` | IMU 详情（**含 gravity 自检行**） |
| `V` | 电池详情（含标定辅助） |
| `N <mode>` | 请求模式 idle/stand/active/damp/estop |
| `R` | **模式 + 安全 + 标定报告**（排障第一站） |
| `C <sub>` | 标定：`start` / `cancel` / `zero` / `erase` |
| `L` | 链路统计（`link_pi`） |
| `g <u\|k>` / `G <n>` | 单发 TEL_STATUS/心跳 / 连发 n 帧 TEL_FAST（误码率测试） |
| `p [n]` | USART1 回显 n 字节（链路自检） |
| `w <type> <len> <hex>` | 手工组一个协议帧发出去 |

**⚠️ shell 与协议帧可共存于同一条 USART1 字节流**：协议帧以 `0xAA` 开头，
shell 只吃可打印 ASCII，所以 `0xAA` 永远进不了 shell 的命令缓冲。

---

## 1. 硬件接线速查

```
┌─ USART1 ↔ 树莓派 ────────────────────── 【交叉】⚠️
│   STM32 PA9 (TX) ──→ Pi RX
│   STM32 PA10(RX) ←── Pi TX
│   GND ────────────── GND
│
├─ USART2 ↔ 舵机控制板 ────────────────── 【直连】⚠️ 与上面不同！
│   STM32 PA2 (TX) ──→ 控制板 TX 端子
│   STM32 PA3 (RX) ←── 控制板 RX 端子
│   ⚠️ 控制板跳线必须在【A 位】
│   ⚠️ 端子丝印按 MCU 侧标注，照丝印一对一接就是直连
│
├─ USART3 ↔ ESP32-C3 ──────────────────── 【交叉】
│   STM32 PB10(TX) ──→ ESP32 RX
│   STM32 PB11(RX) ←── ESP32 TX
│
├─ IMU JY901P（软件 I2C）──────────────────
│   PB8 = SCL，PB9 = SDA
│   ⚠️ 外部 4.7 kΩ 上拉到 3.3 V（必须装）
│
├─ 电池电压 ─────────────────────────────
│   电池+ → 100kΩ → PA0 → 20kΩ → GND，PA0 对 GND 加 100 nF
│
├─ 调试与指示 ───────────────────────────
│   PA13/PA14 = SWD（不要禁用！）
│   PA8 = 心跳（1 Hz 翻转）
│   PC13 = 板载状态灯（灭=正常，慢闪=I2C 异常，快闪=启动失败/周期超时）
│
└─ 舵机动力 ─────────────────────────────
    电池+ ──16 AWG 粗线──→ 12 个舵机 V（全部并联）⭐ 绕过控制板
    电池− ──16 AWG 粗线──→ 12 个舵机 G（并与控制板/STM32/Pi 共地）
```

**⚠️ 三条最容易接错的：**

| # | 易错点 | 后果 |
|---|---|---|
| 1 | USART1 该交叉却直连（或反之） | 双向都不通，但看起来像"固件没发数据" |
| 2 | USART2 该直连却交叉 | PING 全部无响应 |
| 3 | 控制板跳线在 B 位 | 控制板走 USB 通道，UART 完全不通 |

---

## 2. ⭐ 给树莓派侧开发者的接口契约

**这一节是 Pi 侧实现 `docs/31-pi-runtime.md` 时的唯一依据。**

### 2.1 波特率与帧格式

```
USART1，1 000 000 bps，8N1，无流控，3.3V TTL
帧： 0xAA 0x55 | LEN | SEQ | TYPE | PAYLOAD | CRC16(小端)
     LEN   = PAYLOAD 长度（≤128）
     CRC16 = CRC-16/CCITT-FALSE，覆盖 [LEN, SEQ, TYPE, PAYLOAD]（LEN+3 字节）
     帧总长 = 7 + LEN
```

### 2.2 固件承担与不承担的（职责边界）

| 职责 | 谁做 | 备注 |
|---|---|---|
| 策略推理 | **Pi** | 固件完全不碰 |
| 组装 47 维观测 | **Pi** | 固件只提供原始物理量 |
| **步态相位 sin/cos** | **STM32** ⭐ | 经 TEL_FAST 下发，Pi 不要自己算 |
| 关节目标角 → 舵机步数 | **STM32** | Pi 只发 rad |
| NaN/限幅/限速 | **STM32** ⭐ | Pi 发错也不会炸（最后一道防线） |
| 安全保护 | **STM32** ⭐ | 不依赖 Pi；Pi 崩了会自己趴下 |
| 日志记录 | **Pi** | 固件只发 TEL_LOG |

### 2.3 ⚠️ Pi 侧必须遵守的四条

```
① 不要自己产生步态相位。用 TEL_FAST 的 gait_sin/gait_cos，
   否则与固件的相位会漂移，策略输入就错了（协议 §4.4.1）。

② last_action 用【策略原始输出】（tanh 之后、×action_scale 之前），
   不是缩放后的关节角（31-pi-runtime.md §3.2）。
   观测第 33~44 维是 last_action × 4.0。

③ 观测第 9~20 维是 (q − q_default)。固件下发的 q 已经减过 q_default
   （而 q_default = 0，所以数值相同；但语义上别再减一次）。

④ 若 status 里没有 STAT_IMU_OK，不要进 ACTIVE。
   固件会拒绝，但 Pi 侧主动检查更早暴露问题。
```

### 2.4 固件上报的 status 位（协议 §4.5）

```
BIT0 READY        时钟与外设自检通过
BIT1 CALIBRATED   已完成关节标定（Flash 里有有效标定数据）
BIT2 LINK_OK      近期（≤30 ms）收到过合法帧  ← ESP32 指令仲裁的依据
BIT3 IMU_OK       IMU 数据新鲜
BIT4 BATT_WARN    ≤10.5 V
BIT5 BATT_STOP    ≤9.9 V（动作已限幅）
BIT6 BATT_FORCE   ≤9.3 V（正在趴下）
BIT7 ESTOP        处于急停状态
```

### 2.5 ACTIVE 准入的五个条件（`R` 命令的 `active_block`）

```
0 = 通过              Pi 可以放心在 ACTIVE 下工作
1 = 未标定            先做 C start / C zero
2 = 链路不活          Pi 必须每 100 Hz 发 CMD_JOINT
3 = IMU 不可用        检查 IMU 接线与 JY901P 的上位机设置
4 = 安全层禁止        看 safety_reasons（可能电压/温度/失联）
5 = 舵机反馈不新鲜     总线读不全 12 帧，检查 P3 各项
```

### 2.6 固件对错误指令的宽容度（Pi 侧不必自己防）

| Pi 发来的东西 | 固件的处理 |
|---|---|
| `q_target` 里有 NaN / Inf | 该关节回落到**下限**（安全侧） |
| `q_target` 超软限位 | 夹到限位 |
| 单周期跳变 > 0.15 rad | **限速**到 0.15 rad/周期 |
| 非法 mode（>4） | 回落到 IDLE |
| 未知 TYPE | 静默丢弃 + 计数（不复位、不刷屏） |
| LEN > 128 | 丢弃并复位接收状态机 |
| 半帧后 5 ms 无后续 | 状态机复位（能自愈） |
| **完全停止发指令** | **30 ms 降级 → 100 ms 缓慢趴下** |

---

## 3. 调试顺序（不要跳步）

**⚠️ 每一步都有对应的验收项，在 [`34-firmware-acceptance.md`](34-firmware-acceptance.md)。**

```
第 0 步  PC 侧三闸全绿                      → 不需要硬件
         build / test-host / test-python

第 1 步  LED 与心跳                          → P2-1
         PC13 灭（板载 LED，低电平点亮）、PA8 稳定 1 Hz
         ⚠️ 快闪 = 启动失败；不亮 = 没跑

第 2 步  时钟 72 MHz                         → P2-2
         ⚠️ 不通过就不要往下做，后面全是假的

第 3 步  shell 能收发                         → P2-4
         i 自检 + I2C 总线电平                → P2-6
         p 10000 回显无丢失                   → P2-5

第 4 步  舵机总线：PING 12 个                 → P3-1
         ⚠️ 先确认舵机 ID 已逐个改好
         T 1 使能力矩 → P 单关节控位          → P3-2/3
         B 看反馈 valid 全 1                  → P3-5

第 5 步  IMU：i 探测 → M 看 gravity          → P5-1/2
         ⚠️ gravity 必须是 (0,0,-1)

第 6 步  电池：V + 万用表对比（<50 mV）       → P5-4
         分级阈值逐点验证                     → P5-5

第 7 步  模式状态机：N 各模式 + R 看解释       → P6-1

第 8 步  ⭐ 标定：C zero（手动）→ C start      → P6-2
         ⚠️ 手工核对 dir 与物理方向一致

第 9 步  ⭐⭐ 安全：停掉 Pi → 100 ms 后趴下    → P6-3
         逐项验证电压/温度/IMU

第 10 步 ⭐ 缓慢趴下（录像看有没有中间姿态）    → P6-4

第 11 步 ⭐⭐ 拔串口线端到端自检               → P6-5

第 12 步 与树莓派联调                          → P7-1
         python pi/sim_esp32.py → 看 P99 回复耗时

第 13 步 真机部署（见 §5）                     → P7-2
```

---

## 4. 故障决策树

### 4.1 "什么都不通"

```
心跳灯 PA8 有没有 1 Hz？
├─ 没有，且 PC13 快闪（10 Hz） → 进了 Error_Handler
│     → 量 3.3V；查 HSE 8 MHz 晶振是否起振
│     → 敲 s 看 sysclk_hz（若串口也不通则用调试器读）
├─ 没有，且 PC13 不亮          → 程序没跑
│     → BOOT0 = 0？供电？烧写成功了吗？
└─ 有心跳 → 固件在跑，问题在具体外设，按下面的分支
```

### 4.2 "串口收不到 / 乱码"

| 现象 | 原因（按概率） |
|---|---|
| 完全无数据 | TX/RX 接反（USART1 是交叉）；波特率不是 1000000 |
| 乱码 | 波特率不匹配；USB-TTL 是 5V 电平 |
| 能收不能发 | PA10(RX) 没接对 |
| 1 Mbps 下丢字节 | USB-TTL 模块（CH340/CP2102）不支持；换 FT232 |

### 4.3 "舵机 PING 全部无响应"

```
① 控制板跳线在 A 位吗？           ← 最常见
② 接线是【直连】吗？（TX→TX）      ← 第二常见
③ 控制板有自己的供电吗？（20~50 mA 细线）
④ 舵机有 12V 动力吗？（16 AWG 粗线）
⑤ 共地了吗？
⑥ 敲 B 看 uart_overflow / uart_errors：
     不为 0 → 波特率或电平问题
```

### 4.4 "只 PING 到 1 号舵机"

**→ 舵机 ID 没改。** 出厂默认全是 1，同时接上会冲突。
用 Feetech 上位机**逐个**改：改一个 → 立刻拔掉 → 再接下一个。

### 4.5 "标定失败"

```
敲 R，看 calib_err：
├─ 1（读位置失败） → 敲 B 看 ok_reads 是否在涨；不涨就是总线问题（回 4.3）
├─ 2（写位置失败） → 总线发送有问题；看 uart 计数
├─ 3（舵机没动）   → 力矩没使能（查 12V 动力、T 1）；或机械卡死
├─ 4（位移太小）   → 该关节阻力大/被限位挡住；确认它自由
└─ 5（结果不合法） → 把 R 的输出贴回来
```

**⚠️ `C start` 直接返回 FAILED 的两个原因**（shell 会打出来）：
```
a) 舵机总线正忙 → 等 20 ms 再试
b) 还没有新鲜的 12 帧反馈 → 敲 B 确认 ok_reads 在涨
```

### 4.6 "进不了 ACTIVE"

**敲 `R`，看 `active_block`** —— 它直接给出原因（0~5，见 §2.5）。
这是唯一需要看的字段，不需要猜。

### 4.7 "机器人自己趴下了"

**敲 `R`，看 `safety_reasons` 位域：**

| 位 | 名称 | 含义 |
|---|---|---|
| 0x01 | VOLT_FORCE | 电压 ≤9.3 V |
| 0x02 | VOLT_STOP | 电压 ≤9.9 V |
| 0x04 | VOLT_WARN | 电压 ≤10.5 V |
| 0x08 | LINK_LOST | 链路失联（>30 ms 降级 / >100 ms 急停） |
| 0x10 | OVERTEMP | 舵机 >70 °C |
| 0x20 | SERVO_ERR | 舵机错误码非零 |
| 0x40 | IMU_STALE | IMU 数据失效 |
| 0x80 | OPERATOR | 操作者急停（本次未实现硬急停，恒 0） |

### 4.8 "周期被拖长 / 心跳变慢"

```
可能原因（按概率）：
① IMU 的 I2C 太慢（SCL 频率过低）→ 用示波器量 PB8，目标 100~400 kHz
   调整：bsp_softi2c.h 的 BSP_I2C_HALF_US
② 舵机总线有超时（整帧 3 ms）→ 敲 B 看 timeouts 是否在涨
③ 在 Flash 擦写（标定落盘）→ 只发生一次，属正常
④ USB-TTL 工具在 1 Mbps 下丢字节导致重传
```

---

## 5. 部署顺序（真机，逐步放开）

**⭐ 原则：一次只放开一个自由度。任何一步异常就退回上一步。**

```
① 吊起来（腿悬空）
   · N stand，确认 12 个关节保持在站姿、不抖、不发热
   · 手推腿感受阻力是否一致（某个特别软 = 该舵机力矩没使能）

② 跑策略但不上电动力（USB 供电 STM32 + 控制板）
   · python pi/sim_esp32.py --port COM5 --mode active --seconds 30
   · 看关节跟着假策略的小幅正弦动
   · ⚠️ 这一步验证"策略→固件→舵机"整条链，但不承重

③ 吊起来 + 真策略（ONNX）
   · 看动作是否合理（不是抽风、不是不动）
   · 记录 TEL_FAST 的 q/qd/tau 是否有异常尖峰

④ 落地但吊带不摘（承重 50%）
   · 低速（cmd 0.1 m/s）短时
   · 观察是否稳定；异常立刻 N damp

⑤ 摘吊带，站姿
   · N stand，空载站立 60 s，看温度与电流

⑥ 极低速走（0.1 m/s）
   · 逐步 0.2 → 0.3 → 0.4（训练上限 0.5）

⑦ 系统辨识数据采集
   · 记录 q/qd/tau 与 IMU，用于反哺仿真（docs/50-deployment.md）
```

**⚠️ 每一阶段都要确认：**

```
□ safety_reasons = 0
□ B 的 timeouts / crc_err 不增长
□ M 的 n_err / n_spike 不增长
□ 舵机温度 < 60 °C
□ 电压 > 10.5 V（否则换电池）
```

---

## 6. 构建与烧写

### 6.1 命令行构建（推荐，无需 Keil GUI）

```powershell
powershell -ExecutionPolicy Bypass -File firmware/tools/build.ps1
# 产物：firmware/stm32/build/stm32.hex  /  stm32.bin
```

脚本用 **Arm Compiler 6.22（armclang）** 直接编译，参数与 Keil 工程一致：
`-mcpu=cortex-m3 -std=gnu11 -Os -fshort-enums`，链接脚本
`firmware/tools/rlcat_f103.sct`（IROM 64K / IRAM 20K）。

**脚本内置资源门禁**：FLASH > 64 KB 或 RAM > 18 KB **直接判构建失败**。

### 6.1b 🚨 Flash 余量告急（2026-10-10 实测）

**两条构建路径的占用差别很大，务必分清：**

```
命令行构建（build.ps1，-Os）     : FLASH 43776 / 65536 = 66.8%   余 21760 B
Keil GUI 构建（工程设置）         : FLASH 64712 / 65536 = 98.7%   余   824 B  ⚠️
```

**⚠️⚠️ GUI 构建只剩 824 字节。** 原因是两边的优化等级不同：
命令行用 `-Os`，而 Keil 工程的优化等级更弱，**同样的代码大了 21 KB**。
（历次追加功能都在吃这 824 B：ESP32 转发从 +432 B 又涨到 +792 B。）

**再往 GUI 构建里加功能会立刻报 `L6406E: No space in execution regions`。**
按性价比排的减压顺序：

| 顺序 | 手段 | 省下 | 代价 |
|---|---|---|---|
| 1 | `RLCA_SHELL_EN = 0` | 最多（shell 字符串常量很占地方） | 失去 USART1 上的所有调试命令 |
| 2 | Keil 里关 `BrowseInformation` | 中 | 失去"跳转到定义/浏览" |
| 3 | `RLCA_SHELL_LINK_CMDS_EN = 0` | 少 | shell 里少几个链路命令 |
| 4 | 平时只用命令行构建，仅在**烧录**时开 GUI | — | 需接受两条路径的产物不完全一致 |

**⚠️ 两条路径的产物大小不同（43 KB vs 64 KB），但功能等价** ——
所以"命令行构建验证功能、GUI 构建烧录"这个组合是可行的，
只是要记住 GUI 那条更容易撞墙。

### 6.2 ⚠️ Keil GUI 需要手动做一次设置

**编译器选择不在 `.uvprojx`/`.uvoptx` 里，脚本改不动。** 需手动：

```
uVision 打开 firmware/stm32/MDK-ARM/stm32.uvprojx
  → Project → Options for Target 'stm32'
  → Target 页签 → ARM Compiler 下拉框 → "Use default compiler version 6"
  → 确定 → 保存
```

切好之后 `UV4 -b stm32.uvprojx` 即可用，与命令行构建互为验证。

### 6.3 ⚠️ CubeMX 重新生成之后必做

```powershell
powershell -ExecutionPolicy Bypass -File firmware/tools/check-uvprojx.ps1
```

它会比对 `.uvprojx` 里的源文件列表与磁盘实际文件，报出：
- `MISSING`：列了但文件不存在（会链接失败）
- `UNTRACKED`：文件在但没被编译（**静默不生效，最危险**）

**所有固件源文件都放在 `Core/Inc` 与 `Core/Src` 下** ——
CubeMX 自己的目录，重新生成不会删外部文件。

### 6.4 烧写

```
ST-Link / SWD：PA13(SWDIO) / PA14(SWCLK) / GND / 3.3V
⚠️ BOOT0 = 0（从 Flash 启动）
⚠️ PB2 (BOOT1) 悬空或下拉
```

---

## 7. 关键参数速查（改参数只改 `rlcat_config.h`）

| 参数 | 值 | 在哪改 |
|---|---|---|
| 控制频率 | 100 Hz | `RLCA_CTRL_HZ` |
| 时序模型 | A（等指令 4 ms） | `RLCA_LINK_MODE_B` 改 1 切 B |
| 失联阈值 | 30 ms 降级 / 100 ms 急停 | `RLCA_LINK_LOST_MS` / `RLCA_LINK_ESTOP_MS` |
| 舵机整帧超时 | 3 ms | `RLCA_SERVO_FRAME_TIMEOUT_US` |
| 单周期限速 | 0.15 rad | `RLCA_MAX_DELTA_PER_CYCLE_RAD` |
| 电池阈值 | 10.5 / 9.9 / 9.3 V | `RLCA_BATT_*_MV` |
| ADC 参考电压 | 3300 mV（**必须实测校准**） | `RLCA_ADC_VREF_MV` |
| 分压比 | 120 / 20 | `RLCA_BATT_DIV_NUM/DEN` |
| 舵机 ID | 索引 i → ID i+1 | `RLCA_SERVO_ID(i)` |
| 关节限位 | 见 `docs/30-firmware.md` §3.2 | `RLCA_{HIP,THIGH,CALF}_{MIN,MAX}_RAD` |
| 步态相位频率 | 3.0 Hz（**必须与训练侧一致**） | `RLCA_GAIT_FREQ_HZ` |
| IMU 寄存器映射 | ✅ **已核实 + 上机通过 P5-2**（见 `rlcat_config.h` §3b）| `RLCA_IMU_REG_*` |
| 缓慢趴下斜坡 | 0.5 rad/s | `rlca_safety.c` 的 `SAFE_RAMP_RATE_*` |
| 趴下超时兜底 | 4 s | `RLCA_SAFE_SITDOWN_TIMEOUT_MS` |
| shell 开关 | 开 | `RLCA_SHELL_EN`（**Flash 紧张时先关它**） |

---

## 8. 交付物清单

| 类别 | 位置 |
|---|---|
| 固件源码 | `firmware/stm32/Core/{Inc,Src}/` |
| 固件产物 | `firmware/stm32/build/stm32.hex` / `.bin` |
| 构建脚本 | `firmware/tools/build.ps1` + `rlcat_f103.sct` |
| C 单元测试 | `firmware/tests/host/` + `firmware/tools/test-host.ps1` |
| Python 协议与模拟器 | `pi/rlcat_proto.py` / `sim_esp32.py` / `selfcheck.py` |
| Python 测试 | `firmware/tools/test-python.ps1` |
| 工程一致性校验 | `firmware/tools/check-uvprojx.ps1` |
| 验收单 | [`34-firmware-acceptance.md`](34-firmware-acceptance.md) |
| 施工方案 | [`33-firmware-plan.md`](33-firmware-plan.md) |

---

**文档结束。遇到本手册没覆盖的问题，把 `R` 与 `B` 的完整输出贴回来。**
