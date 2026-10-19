# STM32 调试命令手册

> **文档编号**：DOC-STM-006
> **版本**：v1.0
> **日期**：2026-10-11
> **面向对象**：上机调试的人
> **配套文档**：[`30-firmware.md`](30-firmware.md)、[`34-firmware-acceptance.md`](34-firmware-acceptance.md)、[`35-firmware-deployment.md`](35-firmware-deployment.md)、[`39-firmware-handover.md`](39-firmware-handover.md)

---

## 0. ⭐ 30 秒速查

```
连上 → 敲 h 看全部命令
出问题 → 敲 s（状态）→ t（周期耗时）→ I（IMU 裸寄存器）
接舵机前 → 先敲 A（只读 PING），别急着敲 T / P / F
看安全状态 → 敲 K（位置写许可 / 力矩 / 实测步数 / 零位与限位）
标定验证 → T 1（开手动力矩覆盖）→ W <j> <off> → K → T 0
乱码 → 敲 E 0 关掉遥测镜像，UART3 变成纯文本
```

**⚠️ 三条铁律：**

| # | 铁律 | 原因 |
|---|---|---|
| 1 | **首次上电只敲只读命令**（`s` `t` `i` `I` `M` `B` `A` `K`） | 写命令会让舵机动 |
| 2 | **`T` / `W` / `P` / `F` 前先确认机器人姿态** | `q_default = 0` 是弯腿站姿，趴着的机器人被拉起来会撞 |
| 3 | **UART1 与 UART3 的分工不要混** | 协议帧 + shell 共用一条线，看二进制帧是正常现象 |

**⚠️ 补充铁律（2026-10-16 加）：**

| # | 铁律 | 原因 |
|---|---|---|
| 4 | **`W` 之前必须 `T 1`，调完必须 `T 0`** | 不 `T 1` 的话 ⑥b 每 200 ms 掐断位置帧，`W` 要反复连发才走到位 |
| 5 | **物理验证用 `W`，不要用 `P`** | `P` 过标定表换算（未标定时 `offset=0`，判断不出物理值） |


---

## 1. 两条物理通道

```
USART1 (PA9=TX / PA10=RX)  ← 交叉接线
   · 树莓派（正式运行时）
   · USB-TTL（桌面调试时）
   · 波特率 500 kbps

USART3 (PB10=TX / PB11=RX) ← 交叉接线
   · ESP32-C3 无线桥
   · 波特率 115200
   · ⚠️ 同时承载【遥测镜像】+【shell】两条逻辑通道
```

### 1.1 ⭐ 无线调试（USART1 手伸不进去时）

`USART1` 在整机装好后往往够不着。ESP32 提供了无线透传：

```
笔记本 ──WiFi──→ ESP32-C3 ──UART3──→ STM32
```

| 项 | 值 |
|---|---|
| SSID | `RL-CAT-E959` |
| 密码 | `rlcat1234` |
| IP | `192.168.4.1` |
| **TCP 3333** | **纯字节透传** —— 连它就能无线敲 STM32 的 shell ⭐ |
| TCP 3334 | ESP32 自己的控制台（**不是** STM32 的 shell）|

**用法**：用任意 TCP 客户端连 `192.168.4.1:3333`，之后就和接 USB-TTL 一样敲命令。
**shell 已支持双端口**（2026-10-11）：命令从哪条线进来，回显就从哪条线回去。

**⚠️ 无线调试前先 `E 0`**（理由见 §2.4）：
```
E 0        ← 关掉遥测镜像，UART3 变成纯文本
...敲你的调试命令...
E 1        ← 调完恢复镜像，否则 ESP32 收不到遥测
```

---

## 2. 命令详解

### 2.1 总览（`h` 的输出）

```
s               status
t               cycle timing (100 Hz budget check)
l <0|1>         LED off/on
m <off|on|slow|fast|estop>  LED mode
i               I2C bus + probe 0x50
A               servo bus: PING all 12 ids
T <0|1>         servo bus: torque off/on (1 also suspends the 5 Hz re-assert)
P <j> <step>    move joint j (0..11) to step (via calib table)
W <j> <raw>     RAW write: pin joint j to encoder step 0..4095
                (no calib table involved; 'T 1' first)
F <step> [stride]  move ALL joints to step (+/-stride)
B               servo bus stats
K               boot safety: write-gate / torque / raw pos / ZERO & LIMITS
M               IMU detail (+ gravity check)
I               IMU raw registers (byte-order check)
N <mode>        request mode (idle/stand/active/damp/estop)
R               mode + safety + calibration report
C <sub>         calibration: start/cancel/zero/erase
V               battery detail (+ calibration aid)
E [0|1]         ESP32 mirror on/off (0 = clean text on UART3)
e               erase calib flash page
r               reset counters
h / ?           help
```

部分构建下还有（`RLCA_SHELL_LINK_CMDS_EN`）：
```
L               link_pi stats
g <u|k>         send one TEL_STATUS / HEARTBEAT
G <n>           stream n TEL_FAST frames (BER test)
p [n]           echo n bytes on USART1
w <type> <len> <hex...>  send an RLCA frame
```

---

### 2.2 🟢 只读命令（**首次上电只用这些**）

#### `s` —— 状态总览

```
sysclk_hz / pclk1_hz / pclk2_hz   ← 时钟，应为 72000000 / 36000000 / 72000000
dwt_ok                            ← 微秒计时可用？必须是 1
uptime_ms                         ← 运行时长
batt_*                            ← 电池电压与分级
pi_total / esp_total / servo_*    ← 三条 UART 的收发计数
```

**怎么用**：`sysclk_hz != 72000000` 或 `dwt_ok = 0` → 后面全白干，先查时钟。

#### `t` ⭐ 周期耗时（**判断 100 Hz 是否达标**）

```
cycles / last_us / max_us / avg_us / overruns
wait_timeouts
OK / WARN
```

**判据**：`max_us` 必须 **≤ 10000**（10 ms）。
`wait_timeouts` 持续增长 = 每周期都在白等模型 A 的 4 ms 窗口。

**⚠️ 实测基线**（2026-10-11，桌面无 Pi）：
```
cycles=3219  last_us=3381  max_us=4539  avg_us=3424
overruns=0   wait_timeouts=0
```

**为什么重要**：RL 训练用 `decimation=2`，要求控制环**严格 100 Hz**。
超时会让实机与训练的时序契约不一致。

#### `i` —— I2C 总线探测

```
bus_released_high = 1        ← 总线空闲时 SDA/SCL 都被上拉为高
probe 0x50 -> ACK            ← JY901P 在位
probe 0x51 / 0x68 / ... -> no ack
```

**怎么用**：
- `bus_released_high = 0` → **上拉电阻没装或接错**（4.7 kΩ）
- `0x50` 无 ACK → 接线错，或模块没供电

#### `I` ⭐⭐ IMU 原始寄存器 dump（**排查姿态问题的终极手段**）

打印原始字节，并**同时给出大端与小端两种解释**：

```
VERSION   (0x2E) = 05 C1 |BE: 1473   |LE: -16123
IICADDR   (0x1A) = 50 00 |BE: 20480  |LE: 80
RRATE     (0x03) = 09 00 |BE: 2304   |LE: 9
BANDWIDTH (0x1F) = 02 00 |BE: 512    |LE: 2
ORIENT    (0x23) = 00 00 |BE: 0      |LE: 0
AXIS6     (0x24) = 00 00 |BE: 0      |LE: 0
ACC  0x34~36 = 62 03 40 01 43 07 |BE: ... |LE: 866 320 1859
GYRO 0x37~39 = ...
ANGLE 0x3D~3F = ... |BE: ... |LE: 2369 -3002 -16748
QUAT 0x51~54 = ...
```

**⚠️ 为什么需要它**：姿态不对有三种**完全不同的**病因，只看换算后的数值**区分不了**：

| 现象 | 结论 |
|---|---|
| `BE` 与 `LE` 差别巨大，且 `LE` 合理 | 字节序问题（**实测是小端**）|
| 原始值量级离谱（如 ACC 满量程） | 量程假设错，或寄存器地址错 |
| 原始值合理但换算结果不对 | 换算系数错 |

**期望值**：
```
IICADDR  LE = 80 (0x50)   ← 从机地址
RRATE    LE = 9           ← 100 Hz
BANDWIDTH LE = 2          ← 98 Hz
ORIENT   = 0              ← 水平安装
AXIS6    = 0              ← 九轴算法
ACC 静止时 |LE 三轴| ≈ 32768（= 1 g @ ±16g 量程）
```

#### `M` —— IMU 姿态详情（**P5-2 的关键判据**）

```
valid = 1
version = ...
n_ok / n_err / n_spike / age_ms
quat   :  w x y z
gyro   :  x y z (rad/s)
accel  :  x y z (g)
euler  :  roll pitch yaw (rad)
gravity(body): gx gy gz    ← ⭐ 水平静止时应 ≈ 0.000 0.000 -1.000
```

**最后一行是重点** —— 它一次性验证四件事：从机地址、寄存器映射、字节序、欧拉角顺序与坐标约定。

**⭐ 实测基线**（板子略倾斜放置）：
```
accel:  0.424 0.158 0.897  (g)      ← 模 1.014 g ✅
euler:  roll 0.227  pitch -0.290  yaw -1.606  (rad)   = +13.0° / -16.6°
gravity(body): -0.286 -0.216 -0.934
```

**⚠️ 判读注意**：
- `yaw` 无磁参考，**是任意值**，不要当绝对航向
- `gravity` 不是 (0,0,-1) 时**先确认板子真的水平**（倾斜放置给出 -0.934 是**正确的**）
- `n_spike` 持续增长 → I2C 时序不稳，考虑降 SCL 频率

#### `B` —— 舵机总线统计

打印同步读写轮次统计（`rx_frames` / `ok` / `failed` / `timeout` / `idle` / `crc_err` / `short`）
以及 12 个关节的反馈一览（位置、速度、负载、温度、错误码）。

**这是 P3-5/P3-8 的判据。**

#### `R` —— 模式 + 安全 + 标定报告

```
mode / active_mode / active_block / torque_on
safety reasons / severity
校准表是否已加载
```

**怎么用**：`active_block` 不为 0 时看 `RLCA_ACTIVE_BLOCK_*` 枚举值定位原因
（`NOT_CALIB` / `SAFETY` / `NO_LINK` / `NO_IMU` / `NO_FB`）。

#### `V` —— 电池详情

```
mv / mv_raw / adc_raw / level / valid / n_samples / n_timeouts
```

**P5-4 电池标定用**：先量实际电压，再对照 `mv` 填校准系数。

#### `L` —— 树莓派链路统计

`link_pi` 的收发帧、CRC 错误、等待超时、失联时长。

---

### 2.3 🟡 会动舵机的命令（**接舵机前先读 §0 铁律 2**）

#### `A` —— PING 全部 12 个 ID

**只读操作，不会让舵机动。** 依次 PING ID 1~12，打印每个的应答情况。

**P3-1 的判据。** 期望：12 个 ID 全部应答。
**⚠️ 若没接舵机**：全部超时是**正常的**，不能据此判断 ID 是否配好。

#### `T <0|1>` —— 全部关节力矩 关 / 开（⭐ 兼"手动力矩覆盖"开关）

```
T 0    卸力矩 + 恢复 ⑥b 的周期性重申（默认状态）
T 1    使能力矩 + 【暂停】⑥b 的周期性重申
```

**⚠️⭐ `T 1` 同时打开"手动力矩覆盖"，这一点很重要：**

**主循环 ⑥b 默认每 200 ms 重申一次卸力。而那次调用的内部会 `bus_abort_now()`，**
**它会**掐断刚发出的位置帧**并关掉力矩。表现就是：**

> **用 `W` / `P` 调试时，必须反复连发才能让关节走到目标，中途会停。**

**⇒ 敲 `T 1` 之后重申被暂停，一发位置指令就能走完全程。**

**⚠️ 安全边界（不削弱任何保护）：**

| 项 | 说明 |
|---|---|
| 谁能打开 | **只有人在 shell 上敲 `T 1`** |
| 何时自动清除 | **收到明确链路指令（`CMD_JOINT`）时** —— 管控交回主循环 |
| 是否影响位置写规则 | **不影响**。`pos_write_allowed` 仍要求"已标定 + 已 arm + 闸门已开" |
| 是否影响安全层 | **不影响**。severity / 急停照常计算 |
| 怎么确认状态 | `K` 里的 `manual_override`（1 = 覆盖生效中） |

**⚠️ 调完一定要敲 `T 0` 恢复重申。**

**⚠️ `T 1` 会让舵机动**：IDLE 模式下 `q_ref = 0`（弯腿站姿）。
如果机器人趴着，`T 1` 会让它试图站起来。

**安全的首次测试顺序**：
```
① 机器人悬空 / 站在支架上
② A            ← 确认通信
③ B            ← 看反馈
④ T 0          ← 确认卸力矩后能用手扳动（证明力矩控制生效）
⑤ 再考虑 T 1
```

#### `W <j> <raw_step>` —— ⭐⭐ 按【原始步数】写单个关节（标定验证专用）

```
T 1             ← 先打开手动力矩覆盖（否则 200 ms 后会被掐断）
W 4 1536        ← 让关节 4 去物理编码器步数 1536
K               ← 看 raw pos_step 那行是否停在 1536
```

**⚠️⚠️ `W` 与 `P` 的本质区别（做标定验证必须用 `W`）：**

| 命令 | 参数语义 | 是否过标定表 |
|---|---|---|
| `P <j> <step>` | "**【当前标定表下】**的步数目标" | ✅ 会经 `rlca_q_to_step` 换算 |
| **`W <j> <raw>`** | ⭐ "**物理编码器步数**" | ❌ **完全不碰标定表，零换算** |

**⇒ 未标定时标定表里 `offset=0`，用 `P` 根本无法判断"物理读数"与"固件认为的步数"是否一致。必须用 `W`。**

**`W` 只把步数夹到编码器物理范围 `[0, 4095]`，不走软限位、不走限速、不改任何状态机、不影响 `s_armed`。**

**⚠️ 典型用途 —— 验证 `offset` 与 `dir`（见 `30-firmware.md` §3.3.1）：**

```
① T 1
② 逐个 W <j> <K 表里该关节的 off>
③ 全部送完之后，机器人应该正好摆出【弯腿站姿】
   ⇒ 若姿态正确，说明 dir 与 offset 都对
④ T 0
```

#### `P <j> <step>` —— 单个关节到指定位（经标定表换算）

```
P 3 2048       把关节 3（索引 0..11）按【当前标定表】换算后下发
```

**⚠️ `step` 在这里是【标定表坐标系下的步数目标】，不是物理编码器值。**
**要做物理验证请用 `W`。**

**⚠️ 换算链仍会经过 `rlca_q_clamp`（软限位）。**

- 用它验证"方向对不对"（P3-3 的判据）最方便
- **别在装配好的整机上乱试大值**

#### `F <step> [stride]` —— 全部关节到 step（+i×stride）

```
F 2048         12 个关节全部到 2048
F 2048 100     关节 i 到 2048 + i×100
```

**⚠️ 这是 12 个舵机同时动的命令，风险最高。** 只建议在机器人悬空时用。

---

### 2.4 🔴 会改状态的命令

#### `K` —— ⭐ 上电安全 / 零位与限位 总览（一次看全）

**只读，不动舵机。输出分 6 段：**

| 段 | 内容 |
|---|---|
| `-- torque re-assert (⑥b) --` | `reassert_count` / `burst_left` / `want_torque` / **`manual_override`** |
| `-- position-write gate (⑦) --` | `boot_calib_present` / **`pos_write_allowed`** / `pos_write_blocked` / `q_seeded` |
| （闸门期镜像，已废弃） | `goal_seeded` / `feedback_fresh` / `fb_valid_count` |
| `-- raw pos_step --` | 12 个关节的**实测原始编码器步数** + `moving` + **`load`** |
| `-- bus stats --` | `bus_rounds` / `rx_frames` / `timeout` / `crc_err` / `state` |
| ⭐ `-- ZERO / LIMITS --` | 每个关节的 `dir` / **`off`** / `q_cal_deg` / **`enc@qcal`** / **`enc@zero`** / 关节角限位 |

**⭐ 关键判据：**

| 看什么 | 含义 |
|---|---|
| `pos_write_allowed = 0` | ✅ 未标定 / 未收到明确指令 → **不会写位置**（安全） |
| `fb_valid_count = 12` | 12 个舵机全部读到反馈 |
| `rx_frames` 在涨、`timeout` 不涨 | 总线健康 |
| `load = 0`（12 个全 0） | 舵机**没有力矩输出** |
| **`raw pos_step` ≈ `enc@zero`** | ⭐ 机器人处于【弯腿站姿】时应该如此 |
| **`raw pos_step` ≈ `enc@qcal` ≈ 2048** | ⭐ 机器人处于【腿伸直】时应该如此 |

**⚠️ 关于 ZERO/LIMITS 段里 `q_cal_deg`**：它是**幅值**（全为正），
符号由 `dir` 表达 —— 详见 `30-firmware.md` §3.3.1。

**⚠️ 输出走 UART1 文本通道，段数多时可能丢字符**（与遥测帧共用一条线）。
若发现行被截断，多发几次 `K`，或先 `g` 暂停遥测。

#### `E [0|1]` ⭐ ESP32 遥测镜像开关（**解决乱码**）

```
E        查看当前状态
E 0      关掉镜像 → UART3 变成【纯文本通道】
E 1      恢复镜像 → ESP32 才能收到遥测
```

**⚠️ 为什么 UART3 会乱码**：UART3 物理上只有一条线，它同时承载：
- **遥测镜像**（二进制协议帧，10 Hz）
- **shell 的文本输入输出**

两者混在一起，文本终端里就是"ASCII 夹杂二进制"。
**这与 UART1 上一直存在的现象同源**（协议帧 + shell 共用一条线）。

**⚠️ 关掉镜像的后果**：ESP32 收不到遥测，其控制口的 `s`/`z`/`c` 会失去门控依据。
**所以这是调试期临时手段，调完记得 `E 1`。**

**⭐ 三种避免乱码的办法**（按推荐度）：

| 办法 | 做法 | 代价 |
|---|---|---|
| **① 关镜像**（最干净）| `E 0` | ESP32 暂时收不到遥测 |
| **② 用十六进制显示** | 终端勾"HEX 显示" | 仍会混，但能看出帧结构 |
| **③ 只看回显** | 发命令后忽略乱码，只读自己的输出 | 需要眼神好 |

**⚠️ 不要试图"调波特率"解决** —— 1 Mbps / 115200 都已验证正确，
乱码是**协议帧与文本共享一条线**的必然结果，不是波特率问题。

#### `N <mode>` —— 请求模式切换

```
N idle      空闲（力矩关）
N stand     站立（力矩开，保持默认站姿）
N active    策略控制（需过准入门：已标定 + 链路活 + IMU 好 + 有反馈）
N damp      阻尼（卸力矩，靠自重落地）
N estop     急停
```

**⚠️ `N active` 过不了准入会【降级到 `stand`】**，而不是掉回 IDLE。

#### `C <sub>` —— 标定流程

```
C start     自动标定（卸力矩 → 等沉稳 → 采样 → 逐关节测方向 → 落盘）
C cancel    取消
C zero      把当前位置记为零点（⚠️ 先把机器人摆到装配站姿）
C erase     擦除 Flash 标定页
```

**⚠️ `C start` 会逐个关节动舵机测方向** —— 必须在悬空或支架上做。
**⚠️ 标定落盘会擦写 Flash，CPU 停 20~40 ms**，放在非实时路径里（已实现）。

#### `e` —— 擦除 Flash 标定页

**⚠️ 擦掉后标定表回到默认值（`dir=1`、`offset=0`），关节角全错。**

#### `r` —— 清零计数器

清零三条 UART 的 `rx_overflow` / `rx_errors`，以及总线统计。
**排查"是否有新增错误"时先 `r` 再观察。**

#### `l <0|1>` / `m <...>` —— LED 控制

```
l 1              板载 LED 亮（PC13，低电平点亮）
m off/on/slow/fast/estop    闪烁模式
```

**怎么用**：验证 PC13 极性。`m estop` 应看到"双闪停"图案。

#### `p [n]` —— 在 USART1 回显 n 字节

**⚠️ 刻意写死 USART1**（用于验证有线链路本身），不受当前端口影响。

#### `w <type> <len> <hex...>` —— 组一个 RLCA 协议帧发出

**⚠️ 也刻意写死 USART1** —— 它的用途是**测试树莓派链路**。

---

## 3. 典型调试场景

### 3.1 上电后不知道正不正常

```
s        → sysclk_hz 是否 72000000、dwt_ok 是否 1
t        → max_us 是否 ≤ 10000
i        → bus_released_high=1、0x50 ACK
M        → gravity 是否接近 (0,0,-1)
```

### 3.2 PC13 闪个不停

| 图案 | 含义 | 处理 |
|---|---|---|
| **均匀 1 Hz** | 自检发现 I2C 总线异常 | 查上拉电阻、`i` 命令 |
| **均匀 5 Hz** | 100 Hz 周期超时 | 敲 `t` 看 `max_us`/`wait_timeouts` |
| **双闪停** | 进了 `Error_Handler`（初始化失败）| **看 PA8 心跳**：心跳停了就是它 |
| **完全不亮也不闪** | 程序没跑 | 查 BOOT0、供电、晶振 |

**⭐ 判别 `Error_Handler` 的铁证是 PA8**：`bsp_fatal_error()` 是死循环，
**一旦进去主循环就永不运行，PA8 心跳会停。**

### 3.3 姿态数据不对

```
I        → 看 BE / LE 两栏，哪栏合理
M        → 看 gravity 是否接近 (0,0,-1)
```

**⚠️ 先确认板子真的水平** —— 倾斜放置时 `gravity` 的 Z 分量小于 1 是**正确的**。

### 3.4 100 Hz 不达标

```
t        → 看 max_us 与 wait_timeouts
```

`wait_timeouts` 持续增长 = 每周期都在白等模型 A 的 4 ms 窗口
（**没接树莓派时必然发生**，已修复：链路不活时不进入等待）。

### 3.5 接舵机前后

```
A        → 12 个 ID 是否都应答（⚠️ 没接舵机时全部超时是正常的）
B        → 看错误码与温度
T 0      → 卸力矩，用手确认变软
P 0 100  → 极小幅试探单个关节，确认方向
```

---

## 4. ⚠️ 已知的坑（都踩过）

| # | 坑 | 教训 |
|---|---|---|
| 1 | **UART3 波特率被 CubeMX 改回 921600** | `.ioc` 是唯一真值来源；只改 `main.c` 下次生成就丢。**改配置必须改 `.ioc`** |
| 2 | **`.uvprojx` 丢 `TargetArmAds`** | 重生成后必须跑 `check-uvprojx.ps1` |
| 3 | **`Optim` 是枚举不能猜** | 实测矩阵：2✅ 3❌ 4❌ 5❌ 6✅ |
| 4 | **IMU 字节序** | 手册没写就必须**实测**，不能靠"同类模块通常一样"推断 |
| 5 | **阻塞发送拖垮控制环** | 103 B @115200 = 8.93 ms，必须用中断发送 |
| 6 | **测试的输入生成器不能和被测代码共享假设** | 否则两边同时错，单测恒定通过 |
| 7 | **`bsp_fatal_error` 翻转之间必须有延时** | 否则以 CPU 主频翻转，肉眼只看到常亮 |
| 8 | **LED 在 PC13 且低电平点亮** | 早期错配在 PB0 且按高电平写 → 状态灯永远不亮 |

---

## 5. 看不见的通道（别踩）

| 通道 | 内容 | 注意 |
|---|---|---|
| **UART3** | 遥测镜像（10 Hz）+ shell | 用 `E 0` 分开 |
| **UART1** | 协议帧（100 Hz）+ shell | 同样的乱码问题，**无开关** |
| **UART2** | 舵机总线 | **别接 USB-TTL 上去**，会与 STM32 抢总线 |

**⚠️ 关于 UART1 的乱码**：固件上电就在往 USART1 发 `TEL_FAST`（103 B @100 Hz）。
用文本终端看必然是乱码 —— **这不是故障**。勾"HEX 显示"能看到规律的 `AA 55 60` 帧头。

---

## 6. 相关文档

| 文档 | 内容 |
|---|---|
| [`30-firmware.md`](30-firmware.md) | 固件总体设计与参数表 |
| [`34-firmware-acceptance.md`](34-firmware-acceptance.md) | 逐项上机验收单（P2~P6） |
| [`35-firmware-deployment.md`](35-firmware-deployment.md) | 联调与部署手册、故障决策树 |
| [`39-firmware-handover.md`](39-firmware-handover.md) | 交付总结、24+3 个缺陷记录、经验教训 |
| [`32-esp32-bridge.md`](32-esp32-bridge.md) | ESP32 无线桥设计（无线调试的来源）|
| [`38-esp32-handover.md`](38-esp32-handover.md) | ESP32 侧交付与操作手册 |
## 7. ⭐ 标准排查流程：收不到总线数据时

> **这一节的结论来自一次真实的 8 轮排查（UART2 舵机总线）。**
> **核心教训：先读硬件寄存器，不要在软件里加计数器。**

### 7.1 为什么不能靠软件计数器

`rx_frames` / `bytes` / `dma_restarts` 这些指标**全部经过固件自己的代码路径**
（`drain_dma` → 环形缓冲 → 解析器 → 状态机）。所以"收不到"这个现象
**同时兼容两种完全不同的病因**：

```
① 总线上真的没字节        -> 接线 / 适配器方向 / 舵机 / DMA 没武装
② 有字节但被固件丢了      -> 搬运或解析的 bug
```

**这两者要修的东西毫无关系。** 只看软件计数器会在两者之间来回猜。

### 7.2 正确顺序

```
第 1 步：敲 D   —— 读硬件寄存器 + 纯硬件字节计数
```

`D` 会做两件事：
- **主动发 20 次 PING 并采样 2 秒**，报告 `dma_hw_bytes_2s`
  （直接读 DMA 的 CNDTR，不经过任何软件缓冲）
- 打印 `dma_en` / `dma_cndtr` / `dma_wr_pos` / `dma_last` / `hal_dma_state`

**判读（这一步就能把问题切一半）**：

| 现象 | 结论 |
|---|---|
| `dma_hw_bytes_2s > 0` | 线上**有**字节 → **是固件软件的问题** |
| `dma_hw_bytes_2s == 0` 且 `dma_en = 1` | 发了请求但线上**没**字节 → 接线/适配器/舵机 |
| `dma_en = 0` | DMA 通道被关了 → 敲 `X` 重新武装 |
| `dma_wr_pos != dma_last` | **有字节躺在 DMA 里没被搬走** → 搬运路径的 bug |

```
第 2 步：对比 dma_hw_bytes 与 ring_rx_total
```

两个数**数量级应当接近**。若差很多（实测曾差 60 倍），
说明**字节在搬运途中被丢**，且 `uart_overflow = 0` 可进一步区分
"缓冲满丢的"与"被代码主动丢的"。

```
第 3 步：敲 Y <id>  —— 看单个舵机的原始应答字节
```

`Y` 会**同时**给出：
- `recv bytes` + `raw:` 原始字节（硬件收到了什么）
- `servo_bus_ping ok` （固件判据的结论）

**两者对照**：
| raw | `ping ok` | 结论 |
|---|---|---|
| 有 | 1 | 全链路正常 |
| 有 | 0 | 收到了但判据拒绝（解析/超时）|
| 无 | 0 | 舵机/接线问题 |

```
第 4 步：敲 B  —— 看丢弃痕迹
```

`B` 末尾会打印 `parse_fail` / `unknown_id` / `drop_reason` / `drop_raw`：
```
drop_reason: 1=不在帧头  2=陌生ID  3=校验/LEN错
drop_raw:    被丢弃那一刻的原始 24 字节
```

**若 `parse_fail = 0` 且 `unknown_id = 0` 且 `drop_len = 0`**
→ **没有任何字节被拒绝过** → 那"缺一个舵机"的原因**不在解析器**，
而是那个应答根本没到达（回到第 1 步看硬件计数）。

### 7.3 ⚠️ 三个必须避开的坑

```
① 不要相信"某个诊断命令成功了" —— 确认它走的是【同一条路径】
   实测踩到：`Y` 自己的等待循环里显式搬了 DMA，与 servo_bus_ping
   走【不同通路】。所以 `Y` 能成功完全不能证明 `servo_bus_ping` 能成功 ——
   而它的"成功"让我误以为链路是通的，掩盖了真正的根因。

② 不要只看 HAL 的状态
   实测抓到 dma_ccr.EN = 0（硬件已停）而 hal_dma_state = READY
   （HAL 以为在跑）—— 软件状态与硬件状态会脱节。

③ 诊断代码本身要写对
   实测踩到：trace 的 record_drop() 放在了循环开头而不是丢弃分支，
   于是它记录的是"最后一次经过的位置"（一个【成功解析】的帧），
   完全误导。诊断工具错了比没有更糟。
```

