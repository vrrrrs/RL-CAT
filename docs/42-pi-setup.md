# 树莓派侧开发环境与要求（Pi Setup & Handover）

> **文档编号**：DOC-PI-SETUP-001
> **版本**：v1.0
> **日期**：2026-10-11
> **面向对象**：**负责树莓派侧开发的人 / 另一个对话的 Agent** + 项目负责人
>
> **本文档分三部分：**
> | 部分 | 读者 | 内容 |
> |---|---|---|
> | **A** | ⭐ **接手开发者** | 现状、要做什么、怎么做 |
> | **B** | ⭐ **项目负责人（你）** | **需要你亲手做的事**（硬件/网络/系统） |
> | **C** | 双方 | 环境细节、验证方法、风险 |

**⚠️ 本文档只讲"环境与分工"。技术设计见 [`31-pi-runtime.md`](31-pi-runtime.md)。**

---

# A. 接手开发者：现状与任务

> **⚠️ 2026-10-12 更新：A 部分原先是"任务书"，现在【已执行完毕】。**
> **实现状态、勘误、下一步见 [§D](#d--实现状态与勘误2026-10-12-更新)。**
> **A 部分保留作为设计意图的记录。**

## A0. 一句话背景

**RL-CAT 是一只 12 自由度四足机器人。三端软件【都已交付】，仿真侧已训出可行走策略。当前的工作是【真机打通】。**

```
仿真（已训出策略）      →  ✅ 完成
STM32 固件              →  ✅ 已交付（1028 C 单测 + 161 Python 全绿）
ESP32 无线桥            →  ✅ 已交付并上真机验证
树莓派侧（基础层）      →  ✅ 已交付（链路/遥测/标定/点动/姿态）
                          （本文档 A 部分即它的任务书）
真机打通                →  🟡 【当前卡在这里】见 §D4
```

**⚠️ 本阶段的边界：只做【基础层】—— 串口通讯 → 舵机标定 → 控制舵机运动 → （可选）传统步态。**
**明确不含：强化学习策略、ONNX/numpy 推理、`Sim/` 仿真与训练。**

---

## A1. ⭐ 好消息：协议层已经写好了，而且零依赖

**`pi/` 下已有 4 个文件（由固件交付方一并交付）：**

| 文件 | 行数 | 作用 | 依赖 |
|---|---|---|---|
| **`rlcat_proto.py`** | 469 | ⭐ **协议实现（两端共用）** | **零外部依赖**（纯标准库） |
| `sim_esp32.py` | 334 | 树莓派侧模拟器（PC 上跑，与 STM32 对打） | **`pyserial`** |
| `selfcheck.py` | 384 | 协议自检（141 项，不需要硬件） | 无 |
| `test_loopback.py` | 365 | 模拟器回环测试（20 项，不需要串口） | 无 |

### A1.1 `rlcat_proto.py` 已经提供了什么（**别重复造**）

**⚠️ 下面这些**全部已经存在**，直接 `import rlcat_proto` 用即可：**

**① 协议常量（与固件 C 实现同源）**

```python
TYPE_CMD_JOINT = 0x01 ;  LEN_CMD_JOINT = 31
TYPE_TEL_FAST  = 0x02 ;  LEN_TEL_FAST  = 96        # ⚠️ 96，不是 90
TYPE_CMD_MODE  = 0x03 ;  TYPE_TEL_STATUS = 0x04
TYPE_HEARTBEAT = 0x7F
N_JOINTS = 12
MODE_IDLE / MODE_STAND / MODE_ACTIVE / MODE_DAMP / MODE_ESTOP
ST_READY / ST_CALIBRATED / ST_LINK_OK / ST_IMU_OK / ST_BATT_* / ST_ESTOP
ERRF_VOLTAGE / ERRF_OVERHEAT / ERRF_OVERCURRENT / ... 
Q_UNIT = 0.0001 ;  QD_UNIT = 0.0001 ;  TAU_UNIT = 0.001
OBS_DIM = 47 ;  ACTION_SCALE = 0.25 ;  GAIT_FREQ_HZ = 3.0
OBS_SCALE_ANG_VEL = 0.25 ;  OBS_SCALE_DOF_VEL = 0.1 ;  OBS_SCALE_ACTION = 4.0
```

**② 编解码（含 CRC16）**

```python
crc16(data) -> int
pack(msg_type, seq, payload) -> bytes
unpack(buf) -> Optional[Frame]
class FrameDecoder            # 流式解码状态机（处理粘包/残包）
make_cmd_joint(q_target, mode, flags, seq) -> bytes
make_cmd_mode(mode, param, seq) -> bytes
make_cmd_calib(action, seq) -> bytes
make_heartbeat(seq) -> bytes
parse_tel_fast(payload) -> TelFast      # dataclass
parse_tel_status(payload) -> TelStatus  # dataclass
```

**③ ⭐ 最容易写错的观测组装 —— 也已经有了**

```python
quat_to_gravity(quat) -> List[float]     # 四元数 → 机身系重力（3 维）
build_observation(tel, cmd_vx, cmd_vy, cmd_wz, last_action, q_default=None) -> List[float]
action_to_q_target(action, q_default=None) -> List[float]
```

**⚠️ `build_observation()` 的实现与训练侧 `rlcat_env.py` 逐维对齐，并且已经处理了 `last_action` 的语义**（策略原始输出，tanh 之后、缩放之前）。

### A1.2 结论：**你要写的其实不多**

```
✅ 已有：协议 / CRC / 帧状态机 / 观测组装 / 动作换算 / 模拟器 / 自检
❌ 要写：主循环、策略加载、真实串口层、指令源、日志、标定脚本、逆运动学
```

---

## A2. 需要实现的东西

**⚠️ 完整技术设计见 [`31-pi-runtime.md`](31-pi-runtime.md)。这里只列清单与要点。**

### 阶段 1（🔴 最高优先）：串口通讯打通

| 要做的 | 说明 |
|---|---|
| **`pi/link.py`** | 真实串口层：打开 `/dev/ttyAMA0`，500 kbps，**raw 模式 + 非阻塞读** |
| **`pi/pi_hello.py`** | 最小可运行脚本：连上 → 收 TEL_FAST → 打印关节角/电压/相位 |

**验收判据：**

```
□ 能连续收到 TEL_FAST，CRC 零错
□ cycle_cnt 连续递增（用 rlcat_proto 的连续性判断）
□ 用手掰某条腿，打印出的 q 跟着变
□ 把机身倾斜，gravity 跟着变（应指向"下"）
```

**⚠️ 这一步做完，就证明了【硬件链路 + 协议 + 坐标约定】全对。**

### 阶段 2：读遥测、看状态

| 要做的 | 说明 |
|---|---|
| 解析 `TEL_STATUS` | 电压 / 12 个舵机温度 / 总线错误计数 / uptime / mode |
| 打印状态面板 | 一个简洁的实时面板或周期性日志 |

**验收判据：**

```
□ 电压读数与万用表对比，误差 < 50 mV
□ 12 个舵机温度都能读到且合理
□ mode / status 位能正确反映固件状态
```

### 阶段 3：舵机标定

| 要做的 | 说明 |
|---|---|
| **`pi/calib.py`** | 发 `CMD_CALIB`，走完固件的标定流程，等待完成并校验结果 |

**⚠️ 标定流程在 STM32 侧已实现**（`rlca_calib_proc.c`：卸力矩 → 等沉稳 → 采样 → **逐关节测方向** → 回位 → 读零点 → 校验）。

**验收判据：**

```
□ 标定能跑完，无超时
□ 标定后 q=0 时机器人呈【站立姿态】
□ offset 存入 Flash，断电重启后仍有效
□ 失败时能给出可读的错误原因（不是只报"失败"）
```

**⚠️ 标定会让机器人**动**——必须先把机器人【吊起或垫高】，腿悬空。**

### 阶段 4：控制单个关节运动

| 要做的 | 说明 |
|---|---|
| **`pi/jog.py`** | 交互式单关节点动：选关节 → 给角度增量 → 下发 → 观察 |

**验收判据：**

```
□ 方向正确（正角度让关节往预期方向动）
□ 幅度正确（给 10° 就动约 10°）
□ 限位生效（超限被拒绝或截断）
□ 12 个关节逐个都能动
```

**⚠️ 必须【吊起】做这一步。** 这是第一次真正驱动舵机。

### 阶段 5：控制全部 12 个关节

| 要做的 | 说明 |
|---|---|
| **`pi/pose.py`** | 一次性下发 12 个关节角，让机器人摆到指定姿态 |

**验收判据：**

```
□ 能摆出站立姿态并保持
□ 能缓慢改变姿态（如缓慢下蹲/起身）而不失稳
```

### 阶段 6（可选但很有价值）：传统步态

| 要做的 | 说明 |
|---|---|
| **`pi/gait.py`** | 用相位 + 逆运动学生成 12 个关节角，下发 |

**⭐ 为什么建议做（价值比看上去大）：**

| # | 价值 |
|---|---|
| 1 | **验证"从关节角到实际运动"整条链** —— RL 策略输出也是关节角，走同一条路 |
| 2 | **在没有策略时让机器人走起来** —— 独立于 RL 验证机械/电子/固件 |
| 3 | **提供 RL 的对比基线** |
| 4 | **是 sim-to-real 的降级方案** |
| 5 | **调试标定的最好工具** —— 走一圈就能看出哪个关节标定有问题 |

**⚠️ 关键设计决定：逆运动学放在【树莓派】上算，不放在 STM32 上。**

```
理由：STM32 的 Keil GUI 构建 Flash 只剩 824 B（98.7%）—— 放不下
好处：改参数不用重烧固件；Python 写起来快
额外好处：传统步态与 RL 策略【走完全相同的接口】（都是 CMD_JOINT），
         可以在两者之间无缝切换
```

**⚠️ 相位由 STM32 提供**（TEL_FAST 的 `gait_sin`/`gait_cos`，3.0 Hz），不要自己算 —— 避免两边不一致。

### 阶段 7（更远）：接入 ONNX 策略

**⚠️ 这一步【不在本阶段范围】，但可以提前准备。**

**前置：需要给 `13_export_policy.py` 补 ONNX 导出**（该脚本在 `Sim/` 下，见 B 部分第 6 条）。

---

## A3. 开发方式（⭐ 已决策）

### A3.1 结论：**远程开发 + 树莓派上运行**

```
本机 Windows ──(一根网线直连)── 树莓派
     │
     └── VS Code + Remote-SSH ──→ 直接在树莓派上编辑与运行
```

**⚠️ 不推荐"本机写完再 scp"：**

```
串口调试是【高频迭代】的 —— 反复改一行、跑一次。
scp 每次几秒，一天几百次就是几十分钟浪费。
而且硬件的怪异行为只在 Pi 上出现，来回搬会打断思路。
```

### A3.2 具体配置

**Web 侧（接手开发者）：**

```
① 装 VS Code + Remote-SSH 扩展
② 连接：ssh <用户名>@192.168.50.2
③ 打开树莓派上的 RL-CAT 仓库目录
④ 直接在远程编辑 pi/ 下的文件，改完立刻在集成终端跑
```

**⚠️ 也可以在 Web 侧用两个终端：一个跑编辑器里的脚本，一个跑串口监视。**

### A3.3 网络拓扑（⚠️ 见 B 部分，需要负责人先配好）

```
┌────────────────┐  WLAN（保持连互联网）
│   Windows PC   │◄────────────────────────► 路由器/互联网
│                │
│                │  以太网（点对点直连，一根网线）
│                │◄────────────────────────► 树莓派
└────────────────┘
   PC 侧: 192.168.50.1/24
   Pi 侧: 192.168.50.2/24
```

---

# B. ⭐ 需要项目负责人（你）亲手做的事

**⚠️ 下面这些接手开发者做不了 —— 都涉及硬件、系统配置或扳手活。**

**按优先级排序。**

---

## B1. 🔴 固定树莓派的网络（所有工作的前提）

**⚠️ 这是阻塞项 —— 没配好，接手开发者连不上树莓派。**

### B1.1 物理连接

```
一根网线：Windows 的以太网口 ── 树莓派的 eth0
⚠️ 不需要路由器，直连即可
⚠️ WLAN 不要动 —— 那是 AI 后端的连接
```

### B1.2 Windows 侧（管理员 PowerShell）

```powershell
# 查看网卡名（应该是"以太网"）
Get-NetAdapter

# 设静态 IP
New-NetIPAddress -InterfaceAlias "以太网" -IPAddress 192.168.50.1 -PrefixLength 24

# 验证
Get-NetIPAddress -InterfaceAlias "以太网"
```

### B1.3 树莓派侧

**开启 SSH：**

```bash
sudo raspi-config
# → Interface Options → SSH → Enable
```

**设静态 IP（NetworkManager 方式，Pi OS Bookworm）：**

```bash
sudo nmcli con add type ethernet ifname eth0 con-name eth0-static \
     ip4 192.168.50.2/24
sudo nmcli con up eth0-static
```

**⚠️ 如果是老的 dhcpcd 方式，改 `/etc/dhcpcd.conf`：**

```
interface eth0
static ip_address=192.168.50.2/24
```

**⚠️ 不要给树莓派设默认网关** —— 否则它会尝试通过这条直连链路访问互联网，导致失败。

### B1.4 验证

```powershell
# Windows 侧
ping 192.168.50.2
ssh <用户名>@192.168.50.2
```

**⚠️ 树莓派的用户名/密码请告诉接手开发者。**

---

## B2. 🔴 关掉串口控制台 + 启用 UART0（Pi 5 专用）

**⚠️ 树的 UART 是最容易配错的地方。**

### B2.1 关掉串口登录控制台

```bash
sudo raspi-config
# → Interface Options → Serial Port
#     "Would you like a login shell to be accessible over serial?"  → 【No】  ⚠️
#     "Would you like the serial port hardware to be enabled?"     → 【Yes】
```

**⚠️ 第一个问题必须选 No —— 否则内核会占用这个串口。**

### B2.2 Pi 5 的 UART 要额外处理

**⚠️ Pi 5 与 Pi 4 不同：默认情况下 GPIO14/15 上的 UART 可能不可用或被蓝牙占用。而且如果用默认的 mini-UART（`ttyS0`），它依赖核心时钟分频 —— 在高波特率（含 500 kbps）下会漂移导致误码。**

**在 `/boot/firmware/config.txt` 末尾加：**

```
# 启用 GPIO14/15 上的专用 UART0（PL011）—— Pi 5 专用
dtoverlay=uart0-pi5
```

**⚠️ 相关背景：[Raspberry Pi 官方文档的 UART 章节对 Pi 5 有已知的更新滞后](https://github.com/raspberrypi/documentation/issues/3239)，别只照文档配。**

### B2.3 ⭐ 验证设备节点（**关键**）

**重启后确认：**

```bash
ls -l /dev/serial* /dev/ttyAMA* /dev/ttyS0
```

**✅ 期望：看到一个**专用 UART**（`ttyAMA0` 之类）**

**❌ 如果只看到 `ttyS0`（mini-UART）——** 说明 overlay 没生效，**500 kbps 下同样不可靠，不要往下走。**

**⚠️ 实际设备名可能是 `ttyAMA0` 或 `ttyAMA10`（Pi 5 的编号有变化）。请把 `ls` 的输出记下来，告诉接手开发者。**

**⚠️ 备选方案：** 如果 UART0 怎么都配不好，**可以用 USB-TTL 转接模块**（几十块）—— 绕开所有 GPIO/overlay 问题，`/dev/ttyUSB0` 行为确定。**这是可靠的退路。**

---

## B3. 🔴 接线（树莓派 ↔ STM32）

### B3.1 接线表

| 树莓派 | 物理引脚 | 方向 | STM32 | 说明 |
|---|---|---|---|---|
| **GPIO14 (TXD0)** | 引脚 8 | → | **PA10 (USART1_RX)** | ⚠️ **交叉** |
| **GPIO15 (RXD0)** | 引脚 10 | ← | **PA9 (USART1_TX)** | ⚠️ **交叉** |
| **GND** | 引脚 6/9/14/… | — | **GND** | ⚠️ **必须共地** |

**⚠️⚠️ 三个必须注意的点：**

| # | 注意 |
|---|---|
| 1 | **这是【交叉】接线**（Pi TX → STM32 RX）。**不要和舵机总线搞混** —— 那条是直连 |
| 2 | **必须是 3.3 V 电平** —— 树莓派 GPIO 是 3.3 V，STM32 也是 3.3 V，**直连即可，不要接 5 V** |
| 3 | **必须共地** —— 不共地会出现随机误码 |

### B3.2 STM32 侧的对应配置（已实现，供核对）

```
USART1 : PA9 (TX) / PA10 (RX)
波特率 : 500 000 (500 kbps)
格式   : 8N1，无流控
```

**⚠️ 如果 STM32 固件里 USART1 的波特率被改过，需要同步。**

---

## B4. 🔴 修 STM32 的 IMU 自检（⚠️ 会导致验收失败）

**⚠️ 这是文档核对 JY901P 手册后发现的【确凿错误】，会让 IMU 自检 100% 失败。**

### B4.1 问题

**固件 `firmware/stm32/Core/Inc/rlcat_config.h` 第 154 行：**

```c
#define RLCA_IMU_REG_CHIP_ID          0x00u   /* 只读；WIT 模块通常回 0x50 */
#define RLCA_IMU_CHIP_ID_EXPECT       0x50u
```

**但 `reference/JY901P_I2C协议.md` 手册里，`0x00` 是：**

```
| 00 | 00 | SAVE | 保存/重启/恢复出厂 | R/W |
```

**⚠️ `0x00` 是【写触发寄存器】，不是芯片 ID。手册的寄存器表里根本没有 CHIP_ID 这一项。**

**后果：`imu_selftest()` 读 `0x00` 拿不到 `0x50` → 返回 false → 自检失败。**

**⚠️ 更危险：万一误【写】`0x00`，会触发"恢复出厂"或"保存"。**

### B4.2 修复建议（**待你确认后由合适的人执行**）

**改成读 `0x2E`（VERSION，版本号，手册确认存在）：**

```c
#define RLCA_IMU_REG_VERSION       0x2Eu   /* VERSION（原 CHIP_ID 0x00 是错的） */
/* 自检判据：能读到 + 值不是 0x00 也不是 0xFF */
```

**⚠️ 改动范围：`rlcat_config.h` §3b + `imu.c` 的 `imu_selftest()`（一处）。** 交付时特意做过隔离，只动这一节即可。

---

## B5. 🟡 确认是否改读原生四元数

**手册确认 JY901P **有原生四元数寄存器**：**

```
| 51 | 81 | q0 | 四元数0 |
| 52 | 82 | q1 | 四元数1 |
| 53 | 83 | q2 | 四元数2 |
| 54 | 84 | q3 | 四元数3 |
```

**而固件现在读的是 `0x3D`（欧拉角），然后自己转换成四元数。**

| 方案 | 优点 | 缺点 |
|---|---|---|
| **A. 改读 `0x51`（原生四元数）** ⭐ | 避免万向锁；省 CPU；精度更好 | 需确认 `RSW` 里的 `QUATER` 位已开 |
| B. 保持现状（欧拉角 + 转换） | 不用改 | 万向锁风险；多一次转换 |

**👉 建议改 A。** 但**需要你确认**，因为涉及已交付的固件代码。

**⚠️ 前置检查：** 手册的 `RSW`（寄存器 `0x02`）里有 `QUATER` 位 —— 必须确认它是**打开的**，否则四元数不更新。

---

## B6. 🟡 舵机 ID 逐个设置（12 个）

```
⚠️ 舵机【出厂全是 ID=1】。
   12 个同时接上会一起应答（总线冲突），必须先逐个改 ID。

工具：Feetech 官方上位机
方法：改一个 → 拔掉 → 再接下一个
```

**建议的 ID 分配（见 [`30-firmware.md`](30-firmware.md) §2.3）：**

| ID | 关节 | ID | 关节 |
|---|---|---|---|
| 1 | `lf_hip` | 7 | `rf_thigh` |
| 2 | `lb_hip` | 8 | `rb_thigh` |
| 3 | `rf_hip` | 9 | `lf_calf` |
| 4 | `rb_hip` | 10 | `lb_calf` |
| 5 | `lf_thigh` | 11 | `rf_calf` |
| 6 | `lb_thigh` | 12 | `rb_calf` |

**⚠️ 如果实际 ID 与此不同也可以** —— 只需同步改固件的 `rlcat_config.h` 里的映射表，**并且告诉接手开发者用的是哪套映射。**

---

## B7. 🟡 树莓派上装依赖

**⚠️ 结论：只需要一个包。**

| 包 | 用途 | 必要性 |
|---|---|---|
| **`pyserial`** | `sim_esp32.py` 与真实串口层用 | 🔴 **必须** |

**安装：**

```bash
# Pi OS Bookworm 之后有 externally-managed 限制，用 venv 或 --break-system-packages
sudo apt install python3-serial          # 方式一：系统包（推荐）
# 或
pip3 install pyserial
```

**⚠️⚠️ 注意：`rlcat_proto.py`（协议与观测组装）【零依赖】，纯标准库。**
**所以即使 `pyserial` 出问题，协议层的自检（`selfcheck.py`）仍然能跑。**

**其余可选的（以后需要时再装）：**

```
onnxruntime    ← 阶段 7 接入策略时才需要
numpy          ← 暂时不需要
git            ← 版本管理（建议装）
```

**⚠️ 请把树莓派的 Python 版本告诉接手开发者**（`python3 --version`）。代码用了 `dataclasses` 和 `from __future__ import annotations`，**需要 Python ≥ 3.7**。

---

## B8. 🟢 实时性卫生（可后做）

**⚠️ 阶段 1~5 不需要，但阶段 6（传统步态）与以后的策略运行需要。**

```bash
# CPU governor 设成 performance（避免降频导致周期抖动）
sudo apt install cpufrequtils
sudo cpufreq-set -g performance
```

**⚠️ 也可以先跑一下，看默认 governor 是什么：**

```bash
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
```

---

## B9. 🟢 清理与收尾

| # | 事项 |
|---|---|
| 1 | **确认 `pi/` 里的 `__pycache__` 不要提交**（后续建 git 时加 `.gitignore`） |
| 2 | **根目录有个 `.build-probe/main.o`** —— 游离的构建残留，可删 |

---

## B10. ⚠️ 一个隐藏的前置依赖

**接手开发者的阶段 1（串口通讯）需要 STM32 已经在真机上跑起来。**

**STM32 交付的验收顺序：**

```
第 0 步  PC 侧四道门禁              ← 不需要硬件
第 1~3 步 LED/心跳 → 时钟 72 MHz → shell   ← 不需要 Pi
第 4 步  舵机总线 PING 12 个         ← 需要先做 B6（舵机 ID）
第 5~6 步 IMU 自检 → 电池标定        ← ⚠️ IMU 自检会踩 B4 的坑
第 7~8 步 模式状态机 → 标定（会动）
第 12 步 与树莓派联调                ← 到这里才需要 Pi
```

**👉 所以实际顺序：**

```
① 你：先做 B1~B7，并在 STM32 上跑通第 1~8 步验收
② 接手开发者：同时准备树莓派侧的基础程序
   ⚠️ 可以在【PC 上】用 sim_esp32.py + test_loopback.py 离线验证，不占硬件
③ 第 12 步汇合
```

**⚠️ 接手开发者可以【先写代码】—— 阶段 1~5 的程序在 PC 上就能开发与部分验证，等 B1 完成后直接上树莓派。**

---

# C. 双方：环境细节与验证

## C1. 参数速查（⚠️ 双方必须一致）

| 项 | 值 | 出处 |
|---|---|---|
| **Pi 串口** | `/dev/ttyAMA0`（⚠️ 以 B2.3 实测为准） | 本项目决策：用 UART0 |
| **Pi 波特率** | **500 000**（500 kbps） | 与 STM32 USART1 一致 |
| **Pi 数据格式** | **8N1**，无流控 | |
| **STM32 侧引脚** | PA9 = TX，PA10 = RX | |
| **接线** | ⚠️ **交叉** + 共线 | |
| **控制频率** | 100 Hz | |
| **观测** | 47 维 | `rlcat_proto.OBS_DIM` |
| **动作** | 12 维，`q_target = 0.25 × action`（`q_default = 0`） | `rlcat_proto.ACTION_SCALE` |
| **TEL_FAST 负载** | **96 字节**（⚠️ 不是 90） | `rlcat_proto.LEN_TEL_FAST` |
| **CMD_JOINT 负载** | **31 字节** | `rlcat_proto.LEN_CMD_JOINT` |
| **模式枚举** | `MODE_IDLE=0 / STAND=1 / ACTIVE=2 / DAMP=3 / ESTOP=4` | `rlcat_proto` |
| **步态相位频率** | 3.0 Hz（⚠️ 由 STM32 产生，Pi 只读） | `rlcat_proto.GAIT_FREQ_HZ` |

---

## C2. ⭐ 上树莓派之前，先在这台 PC 上验证

**⚠️ `rlcat_proto.py` 零依赖，`selfcheck.py` 与 `test_loopback.py` 不需要任何硬件。**

**接手开发者应该先跑这两个，确认协议层是对的：**

```bash
cd pi
python selfcheck.py          # 141 项，验证 Python 实现与固件 C 实现是同一套
python test_loopback.py      # 20 项，验证模拟器逻辑（不需要串口）
```

**⚠️ 这两条在任何 Python ≥ 3.7 上都能跑（Windows 也行）。**

**然后再用 `sim_esp32.py` 与真 STM32 对打（需要 `pyserial` + 串口）：**

```bash
python sim_esp32.py --port COM<x> --baud 500000
```

**⚠️ 这一步在 PC 上就能验证 STM32 的链路，不需要树莓派。建议先做。**

---

## C3. 阶段 1 的验证清单（接手开发者用）

```
□ 设备节点是专用 UART（不是 ttyS0）        ← 依赖 B2
□ 能打开串口，500 kbps
□ 连续收 TEL_FAST，CRC 零错
□ cycle_cnt 连续
□ 掰腿 → q 变化
□ 倾斜机身 → gravity 指向"下"
□ 电压读数与万用表误差 < 50 mV
□ 记录【周期耗时直方图】：均值 / P95 / P99 / 最大
   ⚠️ P99 必须 < 4 ms（STM32 侧的等待窗口是 4 ms）
```

**⚠️ 最后一条特别重要：** 没有周期抖动数字，就分不清"是策略问题还是时序问题"。

---

## C4. ⚠️ 三个必须避免的坑

| # | 坑 | 后果 |
|---|---|---|
| **1** | **用了 mini-UART（`ttyS0`）** | 高波特率下时钟漂移 → 随机误码，极难排查 |
| **2** | **用 `input()` 或阻塞读做键盘指令** | 会卡死 100 Hz 主循环 |
| **3** | **退出时不发 `MODE_IDLE` 就断开** | STM32 会在 30 ms 后降级、100 ms 后强制趴下 → 机器人突然瘫倒 |

**⚠️ 第 3 条的正确做法：**

```python
finally:
    link.send_mode(MODE_IDLE)     # ① 先卸力
    time.sleep(0.2)               # ② 等它收到
    link.close()
```

---

## C5. 参考文档

| 文档 | 内容 |
|---|---|
| [`31-pi-runtime.md`](31-pi-runtime.md) | ⭐ **树莓派侧程序设计（主参考）** |
| [`30-firmware.md`](30-firmware.md) | 五张关键表：关节索引/限位/引脚/观测/参数 |
| [`99-glossary.md`](99-glossary.md) | 术语表（⚠️ `thigh` 在代码里指"髋俯仰关节"） |
| [`34-firmware-acceptance.md`](34-firmware-acceptance.md) | STM32 上机验收单（37 项） |
| [`35-firmware-deployment.md`](35-firmware-deployment.md) | 联调部署手册（13 步 + 故障决策树） |
| [`39-firmware-handover.md`](39-firmware-handover.md) | STM32 交付总结 |
| [`../firmware/doc/协议规范_树莓派通信.md`](../firmware/doc/协议规范_树莓派通信.md) | 协议精确定义 |
| [`../reference/JY901P_I2C协议.md`](../reference/JY901P_I2C协议.md) | JY901P 寄存器手册 |

---

## C6. 一句话总结分工

```
┌─────────────────────────────────────────────────────────────┐
│  负责人（你）                                                │
│   · 配网络（B1）→ 配 UART（B2）→ 接线（B3）                   │
│   · 修 IMU 自检（B4）                                        │
│   · 设舵机 ID（B6）                                          │
│   · 装 pyserial（B7）                                        │
│   · 跑 STM32 验收第 1~8 步                                   │
├─────────────────────────────────────────────────────────────┤
│  接手开发者                                                 │
│   · 先在 PC 上跑 selfcheck.py / test_loopback.py 验证协议层   │
│   · 写 link.py / pi_hello.py（阶段 1）                       │
│   · 阶段 2~5：遥测 / 标定 / 单关节 / 全身                     │
│   · 阶段 6：传统步态（逆运动学放 Pi 上算）                     │
├─────────────────────────────────────────────────────────────┤
│  ⚠️ 两人可以并行 —— 开发者在 PC 上先写，不阻塞               │
└─────────────────────────────────────────────────────────────┘
```

---

# D. ⭐ 实现状态与勘误（2026-10-12 更新）

**树莓派侧的"基础"部分已实现并离线验证完毕**（① 链路 ② 遥测 ③ 标定 ④ 点动 ⑤ 姿态）。
**不含**强化学习策略、ONNX/numpy 推理、仿真训练。

代码在 `pi/`；**要在机器人旁边操作请看 [`../pi/使用说明.md`](../pi/使用说明.md)**。

## D1. 交付物

| 阶段 | 文件 | 说明 |
|---|---|---|
| 链路层 | `pi/link.py` | raw + `select` 非阻塞、残帧、CRC、`cycle_cnt` 丢帧、统计 |
| 语义层 | `pi/telemetry.py` | 位域解码、电池等级、**生效模式推断**、温度单位 |
| 1 | `pi/pi_hello.py` | 链路观察 + 自动判据 |
| 2 | `pi/status_panel.py` | 遥测面板（panel / log / once） |
| 3 | `pi/calib.py` | 标定（`--show` / `--start` / `--check`） |
| 4 | `pi/jog.py` | 单关节点动（交互式 + 脚本式） |
| 5 | `pi/pose.py` | 全身姿态（smoothstep 过渡 + 限速） |
| 验收 | `pi/acceptance.py` | 阶段 1~5 一键验收（默认只读，`--move` 才动舵机） |
| 工具 | `pi/tools/fake_stm32.py` | **假 STM32**（PTY 测试替身），没有板子也能全链路验证 |
| 工具 | `pi/tools/uart_loopback.py` | 物理层误码实测（短接 GPIO14↔GPIO15） |
| 测试 | `pi/tests/test_basic_offline.py` | 端到端 **117 项** |

**离线验证**：`pi/run_checks.sh` 全绿 = 自检 3 套 + 交付方 141 + 20 + 本层 117。
**物理层实测**：早期在 **1 Mbps** 下回环 90 万字节，零误码、零丢字节（999 kbit/s）—— 证明 PL011 可用。
**⚠️ 但 2026-10-18 已把 Pi 链路降到 500 kbps**（原因见 `docs/49` §4-C4：500 kbps 下单字节 20 µs，ISR 延迟易触发 ORE）。
当前配置用 `pi/linkprobe.py` 验证：30 s 内 **0 丢帧**。

## D2. ⚠️ 本文档需要更正的地方（均已实测 / 读码确认）

| # | 本文档原文 | 实际情况 | 影响 |
|---|---|---|---|
| 1 | §B2.3「实际设备名可能是 `ttyAMA0` 或 `ttyAMA10`」 | 这**不是同一个东西的两种编号**：`ttyAMA10` 是 Pi 5 的**专用调试串口**（SoC PL011 @`0x107d001000`，内核 console 和登录 getty 都在它上面）；`ttyAMA0` 是 **GPIO14/15**，加 `dtoverlay=uart0-pi5` 后才出现（`pl011-axi 1f00030000.serial`） | 接错那个就通不了 |
| 2 | §B1 要求 PC 侧设静态 IP `192.168.50.1`（需管理员） | **实测不需要管理员**：PC 以太网口本来就有 `169.254.x`（link-local），路由表里 `169.254.0.0/16` 已是 on-link；只要给 Pi 的 eth0 加 `169.254.100.2/16`，两边立刻能通 | 省掉一步需要管理员的操作 |
| 3 | §B2.1 要求「关掉串口登录控制台」 | 实测**不必动**：console 和 getty 都在 `ttyAMA10`（调试口），加 `uart0-pi5` 后 `/dev/serial0` 仍指向 `ttyAMA10`，`ttyAMA0` 是干净的 | 少改一处，降低风险 |
| 4 | §A2 阶段 3 要「等待完成并校验结果 / 失败时给出可读原因」 | 协议里**没有进度和错误码通道**（`CMD_CALIB` 的 ACK 恒 `result=0`；`action=5` MANUAL_ZERO 被解码器拒收；`TEL_LOG` 固件从不发送）。唯一可靠信号是 `STAT_CALIBRATED` 0→1 | 阶段 3 只能判"成功 / 没成功"，**原因要看 shell** |
| 5 | 协议规范说「shell 与协议帧可共存于同一条 USART1」 | **不成立**：`app_poll()` 里 `link_pi_poll()` 先跑并把 USART1 环形缓冲**抽干**（`for(;;)` 循环），之后 `bsp_shell_poll()` 拿不到字节 | 树莓派**不能**靠往 USART1 打字调 shell；标定进度/错误码/手动零点要走 ESP32 无线口（TCP `192.168.4.1:3333`） |
| 6 | §C3 阶段 1 判据「电压读数与万用表比对」 | 必须从 **`TEL_FAST`** 取。`TEL_STATUS.voltage_mv` 固件写死为 0（`app_main.c:800`） | 用错字段会看到恒 0 |
| 7 | 协议规范 §4.8 说 `TEL_STATUS` 温度单位是 `0.1 °C` | 固件实际写的是**原始 °C**（`app_main.c:809-815`） | 差 10 倍 |
| 8 | （本文档未提及）⚠️⚠️ **`MODE_STAND` 没有任何准入门** | 固件只给 `ACTIVE` 加了准入（`app_mode.c`）。默认标定表是 `dir=+1/offset=0`（`rlca_calib_load_defaults` 里 `memset` 出来的），`q_target=0`（正是站姿）按 `rlca_q_to_step` 换算正好是 **舵机 step 0** → 12 个舵机全部冲向绝对位置 0。固件作者在 `app_main.c:695-709` 记录过这个撞机事故。**也就是说「未标定 → ACTIVE 降级 STAND」这个保护，对未标定的机器人本身就是危险路径** | **安全**：树莓派侧必须保证未标定时不发 STAND/ACTIVE（本层已实现 + 回归测试 K2b/K2c/K3b） |

## D3. §B4 的 IMU 自检 bug —— ✅ **已修复（2026-10-11 上机核实）**

**⚠️ 本节原先是"上机阻塞项"，现已关闭。留档备查。**

### 原问题（§B4 提出）

`rlcat_config.h` 里 IMU 器件 ID 读的是 `0x00`，而 `0x00` 是 JY901P 的
「保存 / 重启 / 恢复出厂」寄存器（**可写！**），不是芯片 ID。
→ `imu_selftest()` 必然失败；万一误写还会触发恢复出厂。

### ✅ 实际修复了什么（比原先发现的更多）

**拿到 `reference/JY901P_I2C协议.md` 手册后上机核对，共发现并修复 3 处：**

| # | 问题 | 修复 |
|---|---|---|
| **1** | **器件 ID 寄存器错取 `0x00`**（那是 SAVE，可写） | 改读 **`0x2E`（VERSION）**；判据改为"**能读到**"而非等于某个魔法数（手册明说不同产品版本号不同） |
| **2** | **字节序按【大端】解析**（从"WIT 串口协议用大端"外推的 —— 错的） | 实测是**小端**。证据：静止时小端 \|a\|=1.003 g，大端 \|a\|=15.6 g；VERSION = `05 C1` → 小端 `0xC105` |
| **3** | **欧拉角换算按 0.01°/LSB** | 手册是 `raw/32768×180°` = **0.0055°/LSB**，原值**错了 1.82 倍** |

**⭐ 并且已改用原生四元数**（原 §B5 的建议已采纳）：

```c
/* 四元数：q0@0x51 / q1@0x52 / q2@0x53 / q3@0x54，连读 8 字节
 * ⭐ 模块内部已融合好，直接给四元数 ——
 *    比"读欧拉角再自己转四元数"更好：避免万向锁、省 CPU、精度更高 */
```

### 实测通过的证据（`docs/34-firmware-acceptance.md` P5-2）

```
IICADDR   (0x1A) = 50 00  -> LE 0x50      ✅ 从机地址
RRATE     (0x03) = 09 00  -> LE 9 = 100Hz ✅
BANDWIDTH (0x1F) = 02 00  -> LE 2 = 98Hz  ✅
ORIENT    (0x23) = 00 00  -> 水平安装      ✅
```

**⚠️ 教训（已写进 `rlcat_config.h`）：这类寄存器表【必须查手册】，不能靠"同类模块通常一样"推断。**

**⚠️ 另注：`0x2E` 的 VERSION 是【只读】的，读它比读可写的 `0x00` 安全得多。**

## D4. 上机之前还需要做的事

**✅ 已完成（2026-10-11/12）：**

```
✅ 修 D3 的 IMU 自检（0x00 → 0x2E；大端 → 小端；角度换算 1.82× 错误；改用原生四元数）
✅ IMU 上位机参数：RRATE@0x03 = 9（100 Hz）、BANDWIDTH@0x1F = 2
✅ 树莓派网络与 UART0 配置（实测 ttyAMA0 可用）
✅ 物理层验证：回环 90 万字节零误码（1 Mbps 下实测）
```

**⬜ 仍需做（三个阻塞项）：**

```
□ 12 个舵机逐个设 ID（出厂全是 1，直接接 12 个会总线冲突）
□ 电池 ADC 标定（万用表比对，目标误差 < 50 mV）
□ 拔掉 GPIO14↔GPIO15 的短接线，按【交叉 + 共地】接到 STM32
□ 建议：改掉树莓派的 sudo 密码（早期操作中它曾被误写进 config.txt，已清理干净）
```

然后按顺序跑：

```bash
cd ~/RL-CAT/pi
python3 pi_hello.py --tilt           # 掰腿 / 倾斜 / 万用表比对
python3 status_panel.py              # 12 路温度、状态位
python3 acceptance.py                # 一键验收（只读，安全）
python3 acceptance.py --move --yes   # ⚠️ 吊起后跑完整验收（含 12 关节点动）
```

---

**文档结束。**

**⚠️ 接手开发者请注意：动手前先读 [`31-pi-runtime.md`](31-pi-runtime.md)（技术设计）与 [`30-firmware.md`](30-firmware.md) §5（观测/动作定义）。**
