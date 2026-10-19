# 树莓派侧：基础层（阶段 1 起）

> **📖 要在机器人旁边操作，请看 [`使用说明.md`](使用说明.md)** —— 那份是操作手册
> （怎么跑、看到什么算对、出问题查哪里）。
>
> **本文档是开发记录**：设计取舍、踩过的坑、固件行为清单、测试覆盖。

> **范围**：串口链路 → 遥测 → 标定 → 单关节点动 → 全身姿态。
> **明确不含**：强化学习策略、ONNX/numpy 推理、`Sim/` 仿真与训练。

---

## 1. 目录与职责

| 文件 | 职责 | 依赖 |
|---|---|---|
| `rlcat_proto.py` | ⚠️ **固件交付方给的**，两端共用的协议实现（帧/CRC/状态机/常量） | 零依赖 |
| `config.py` | 所有参数集中：串口、关节名、限位、软限位、阈值、模式枚举 | `rlcat_proto` |
| `stats.py` | 有界统计：分位耗时 / 速率 / 计数器（100 Hz 循环里不能无界增长） | 标准库 |
| `link.py` | ⭐ **串口链路层**：raw + 非阻塞收发、解析、统计、丢帧检测 | `pyserial` |
| `telemetry.py` | 遥测**语义层**：位域→名字、电池等级、**生效模式推断**、温度单位 | `config` |
| `pi_hello.py` | 阶段 1 最小可运行：连上 → 读遥测 → 给判据 | 上面几个 |
| `status_panel.py` | 阶段 2：遥测/状态面板（panel / log / once 三种输出） | 上面几个 |
| `calib.py` | 阶段 3：标定（只用协议；进度/错误码的局限见 §6） | 上面几个 |
| `jog.py` | 阶段 4：单关节点动（交互式 + 脚本式） | 上面几个 |
| `pose.py` | 阶段 5：全身姿态（smoothstep 过渡 + 限速） | 上面几个 |
| `acceptance.py` | ⭐ 阶段 1~5 一键验收（默认只读；`--move` 才动舵机，可出 JSON 报告） | 上面几个 |
| `tools/fake_stm32.py` | ⭐ **假 STM32**（PTY 测试替身）：没有真板子也能把基础程序跑通 | 标准库 |
| `tools/uart_loopback.py` | ⭐ **1 Mbps 物理层误码实测**（短接 GPIO14↔GPIO15） | `pyserial` |
| `tests/test_basic_offline.py` | 离线端到端测试：链路/遥测/模式/标定/故障注入 | 上面几个 |
| `run_checks.sh` | 一键跑完所有**不需要硬件**的检查 | — |
| `selfcheck.py` / `test_loopback.py` / `sim_esp32.py` | ⚠️ 固件交付方给的，勿改 | — |

**设计原则**：`rlcat_proto.py` 是协议层（编解码），**不动它**；
参数与语义放在 `config.py`；I/O 与状态放在 `link.py`。

---

## 2. 不需要硬件就能做的验证

```bash
cd ~/RL-CAT/pi
./run_checks.sh                 # 一次跑完下面全部
```

单独跑：

| 命令 | 验证什么 | 期望 |
|---|---|---|
| `python3 config.py` | 关节表 / 限位 / 软限位 / NaN 夹紧 | 自检通过 |
| `python3 stats.py` | 有界分位统计 / 速率计 | 自检通过 |
| `python3 link.py` | PTY 内存链路：收发 / 残帧 / 坏 CRC / 丢帧 / 非阻塞（8 组） | 自检通过 |
| `python3 selfcheck.py` | 协议层与固件 C 实现同源（141 项） | PASS 141 |
| `python3 test_loopback.py` | 模拟器回环（20 项） | PASS 20 |
| `python3 tests/test_basic_offline.py` | ⭐ 端到端 117 项（见下） | PASS 117 |

`tests/test_basic_offline.py` 覆盖：

- **A 链路与遥测**：帧率 / cycle_cnt 连续 / CRC 零错 / 电压 / 相位推进 /
  TEL_STATUS 的两个固件怪癖 / 垃圾字节后重新同步
- **B 模式与运动**：未标定时 ACTIVE 静默降级 / 限速 / 跟随 / 力矩 /
  限幅 / NaN 处理 / IDLE 卸力矩
- **C 标定**：ACK、`result` 恒为 0、STAT_CALIBRATED 置位、
  `action=5` 被拒收、START 静默失败、失败时无任何信号
- **D 故障注入**：CRC 错、丢帧、失联急停、心跳骗过失联判定
- **E 耗时**：请求-应答 P99 < 4 ms
- **F 上电与失联**：上电即 ESTOP、周期发帧后解除、30~100 ms 的降级带、
  **上电 1000 ms 门控期内关节不动**
- **G 遥测语义（阶段 2）**：生效模式推断的 6 种情形 / 电池 4 个阈值边界 /
  温度单位换算与等级 / 位域解码 / 注入超温舵机与 latch 错误后面板能读到并告警
- **H 标定会话（阶段 3）**：成功（0→1 跳变）/ 失败注入 / **不保活会被 ESTOP 打断**
  / 起始已标定时的不可识别性 / `--show` 退出码 / 标定后自检能发现零点偏了
- **I 单关节点动（阶段 4）**：命令解析 7 条 / **方向与幅度（给 10° 就动约 10°）**
  / 限位截断 / **12 个关节逐个都能动** / 未标定时拒绝 / 点动期间链路保持活跃
- **J 全身姿态（阶段 5）**：姿态文件解析 / **限速真的生效（耗时 ≥ Δmax/rate）**
  / 收敛判据 / 超限夹紧并报告 / NaN 不发散 / 过程中 ESTOP 立刻中止 / 度弧度打印不混
- **K 一键验收与安全回归（阶段 1~5）**：`acceptance.py` 已标定 + 动作项全通过 /
  ⭐ **未标定时"从未发出 STAND 或 ACTIVE"**（把 §8 第 19 条那条安全规则钉死）/
  未标定时动作项被跳过而不是硬跑 / 报告结构完整

---

## 3. 用假 STM32 手工联调（不需要真板子）

一个终端跑假的：

```bash
python3 tools/fake_stm32.py --pty-file /tmp/fake.pty --calibrated -v
# 它会打印 /dev/pts/N
```

另一个终端：

```bash
python3 pi_hello.py --port "$(cat /tmp/fake.pty)" --seconds 5 --keepalive --tilt
```

常用注入开关：`--drop-rate 0.1`（丢帧）、`--crc-rate 0.1`（CRC 错）、
`--calib-fail`（标定失败）、`--calib-start-fail`（START 静默失败）、
`--battery-invalid`、`--startup-gate-ms`。

---

## 4. 上真机的顺序（阶段 1）

### 4.1 先验物理层（⚠️ 别跳过）

把 **物理引脚 8 (GPIO14)** 和 **引脚 10 (GPIO15)** 用一根杜邦线短接：

```bash
python3 tools/uart_loopback.py /dev/ttyAMA0 1000000
```

判据：**零误码、零丢字节**。本机实测结果：90 万字节 / 0 误码 / 999 kbit/s。

不做这一步，后面链路出问题你分不清是 UART、是协议还是接线。

### 4.2 再接 STM32

| 树莓派 | 物理引脚 | 方向 | STM32 |
|---|---|---|---|
| GPIO14 (TXD0) | 8 | → | PA10 (USART1_RX) |
| GPIO15 (RXD0) | 10 | ← | PA9 (USART1_TX) |
| GND | 6/9/14 | — | GND |

⚠️ 交叉、3.3 V 直连（不要 5 V）、**必须共地**。

### 4.3 跑观察

```bash
python3 pi_hello.py --seconds 30 --tilt
```

通过判据（脚本自动判前 5 条）：

```
□ 持续收到 TEL_FAST，CRC 零错
□ cycle_cnt 连续（不丢周期）
□ 帧率 ~100 Hz，处理耗时 P99 < 4 ms
□ TEL_FAST 电压读数非 0
□ 手工：掰腿 → q 跟着变；倾斜 → 重力指向地面；万用表比对电压（< 50 mV）
```

⚠️ `pi_hello.py` 默认**不发** CMD_JOINT。固件的失联判定基于"距上次收到**任意**帧"，
所以不发帧它会在 100 ms 后真的开始趴下。只是观察时请加 `--keepalive`。

---

## 5. 阶段 2：遥测面板

```bash
python3 status_panel.py                            # 原地刷新的实时面板（终端里）
python3 status_panel.py --mode log --interval 1     # 一行一条，适合重定向到文件
python3 status_panel.py --mode once                 # 打一次快照就退出；有告警退出码 1
python3 status_panel.py --temp-unit deci            # 按协议文档的 0.1°C 解释温度
```

面板内容：链路健康 / **请求模式 vs 生效模式** / 状态位 / 错误位 / 电池等级 /
12 路舵机温度 / 总线计数 / 12 关节角 / 步态相位，底部自动汇总告警。

面板长这样：

```
┌─ RL-CAT 遥测面板 ────────────────────────────────────────────────────────┐
│ 链路  /dev/ttyAMA0 @ 1000000       帧率   99.8 Hz   运行 00:01:23
│       收 5030 帧   CRC 错 0   半帧超时 0   丢 0 周期   发出 0 帧
│ 模式  请求 STAND    生效 STAND
│ 状态  READY|CALIBRATED|LINK_OK|IMU_OK
│ 错误  无
│ 电池  ✅ 11750 mV  正常
│ 温度  (原始 °C（固件实际行为）)
│       hip    [ 32.0   34.0   75.0!  38.0 ]
│       ...
├──────────────────────────────────────────────────────────────────────────┤
│ ⚠️  舵机 [2] 超温（>70°C）—— 固件会强制趴下
└──────────────────────────────────────────────────────────────────────────┘
```

**验收判据**（`docs/42-pi-setup.md` §A2 阶段 2）：

```
□ 电压读数与万用表对比，误差 < 50 mV   ← ⚠️ 读 TEL_FAST，不是 TEL_STATUS
□ 12 个舵机温度都能读到且合理          ← 用体温捂一个舵机：原始°C 会变几度，0.1°C 会变几十
□ mode / status 位能正确反映固件状态    ← 面板会推算"生效模式"并标出降级原因
```

⚠️ **为什么必须推算"生效模式"**：固件 `TEL_STATUS.mode` 给的是**请求的模式**，
而 ACTIVE 有一道准入闸（标定/链路/IMU/舵机反馈/安全层），过不了就**静默降级成
STAND**。不推的话，你会以为机器人在执行指令，其实它只是站着。

---

## 6. 阶段 3：标定 —— 协议里到底能观察到多少

⚠️ **先读这张表，否则一定会误判。**

| 想知道的 | 协议里 (USART1) | shell 里 |
|---|---|---|
| 能不能启动 | `CMD_CALIB(0)` 的 ACK | `C start` 的打印 |
| 进度百分比 | ❌ 没有 | ✅ `R` → `calib_progress` |
| 失败原因 | ❌ 没有（ACK 恒 `result=0`） | ✅ `R` → `calib_err` |
| 是否成功 | ✅ 只有 `STAT_CALIBRATED` 0→1 | ✅ `R` → `calib_state` |
| 逐关节 `dir`/`offset` | ❌ 没有 | ✅ `R` → `j0 id=.. off=.. dir=..` |
| 手动零点 (`C zero`) | ❌ 动作 5 被解码器**拒收** | ✅ `C zero` |

**而 shell 这条路在树莓派上走不通**（见 §7 清单第 17 条：`link_pi_poll()` 把
USART1 抽干了）。所以本层的 `calib.py` 只用协议：**能启动、能判定"从未标定变
成已标定"**；要看进度/错误码/逐关节表，必须用 ESP32 无线口（TCP 192.168.4.1:3333）
或台式 USB-TTL 直连 STM32（此时树莓派要先断开）。

```bash
python3 calib.py --show                 # 只读：当前标定状态（安全）
python3 calib.py --start --yes          # 启动全自动标定并等待（约 8.4 s）
python3 calib.py --start --yes --timeout 25
python3 calib.py --check                # 标定后自检
```

⚠️⚠️ **标定会让 12 个关节都动约 0.15 rad，必须先把机器人吊起或垫高。**

⚠️ **等待期间必须持续发帧**（脚本已做，20 Hz 的 `MODE_IDLE`）。固件的失联判定
基于"距上次收到任意帧"，标定要 8~10 s，中途一旦超过 100 ms 没收到帧，安全层
就会开始趴下斜坡，把整次标定毁掉。这个用测试用例 H3 固化下来了。

**验收判据**（`docs/42-pi-setup.md` §A2 阶段 3）：

```
□ 标定能跑完，无超时
□ 标定后 q=0 时机器人呈【站立姿态】      ← calib.py --check
□ offset 存入 Flash，断电重启后仍有效      ← 重启后再 --show，仍应是 ✅
□ 失败时能给出可读的错误原因              ← ⚠️ 协议做不到，需走无线 shell
```

---

## 7. 阶段 4 / 5：单关节点动与全身姿态

```bash
python3 jog.py --list                                    # 关节表（含软限位）
python3 jog.py                                           # 交互式（吊起后）
python3 jog.py --sequence "4:+10;4:+10;4:-20;all:0" --dwell 1.0 --yes
python3 pose.py --list
python3 pose.py --pose zero --yes                         # 摆回站姿
python3 pose.py --pose-file poses/example.json --yes
python3 pose.py --pose zero --from "0,0,0,0,20,20,20,20,0,0,0,0" \
                --cycles 3 --ramp 15 --yes
```

⚠️⚠️ **必须吊起/垫高**。这两个脚本会真的驱动舵机。

**验收判据**（`docs/42-pi-setup.md` §A2 阶段 4/5）：

```
□ 方向正确（正角度让关节往预期方向动）       ← jog 会同时打印目标角与实际角
□ 幅度正确（给 10° 就动约 10°）              ← 同上
□ 限位生效（超限被拒绝或截断）               ← 会打印"已截断到 xx°"并计数
□ 12 个关节逐个都能动                       ← --sequence 把 0..11 走一遍
□ 能摆出站立姿态并保持                       ← pose.py --pose zero
□ 能缓慢改变姿态（如缓慢下蹲/起身）而不失稳     ← --from/--cycles + --ramp
```

**设计说明（每一条都是踩过坑才写下来的）：**

| 做法 | 为什么 |
|---|---|
| 键盘读在**独立线程**，主循环只从队列取 | 100 Hz 主循环绝不能被键盘阻塞；而"等按键"期间**也必须继续发帧**，否则固件 100 ms 后就趴下 |
| 姿态过渡用 **smoothstep** + `--ramp` 限速 | 阶跃会给舵机冲击；限速让"缓慢改变姿态"可控、可复现 |
| 过渡起点用**实测角**，不是目标角 | 用目标角当起点，第一帧就会产生跳变 |
| 只提供 `zero`（=站姿）一个预设 | 下蹲/抬腿需要真实运动学与限位配合，脚本**不发明**姿态；用 `--pose` / `--pose-file` 自己给（[poses/example.json](poses/example.json) 是模板，数值未验证） |
| 未标定直接拒绝 | 固件会把 ACTIVE 静默降级成 STAND，关节不动——不拦住的话你会以为"发了指令没反应" |
| 只信**本次操作开始之后**收到的帧 | 上电那段的 ESTOP 帧会被误判成"过程中进了 ESTOP"，姿态一发出就中止（`calib.py` 和 `pose.py` 都踩过） |

### ⚠️ 软限位绝不能把站姿排除在外

`hip` 的硬件限位是 `0° ~ +40°`（**不对称，不做内摆**），而 `q_default = 0°`
正好落在这个端点上。如果软限位按"统一留 5° 余量"算成 `[5°, 35°]`，钳位函数就会
把所有髋关节的目标从 `0°` 抬到 `5°` —— **机器人永远站不正**（四个髋关节恒定偏 5°）。

所以 `config.py` 里软限位两端各取"与 `q_default` 更靠内"的那个值，并且：
- `config.py` 自检里有一条硬性断言：`Q_SOFT_MIN[i] <= q_default <= Q_SOFT_MAX[i]`
- 端到端测试 `B5c` 钉住了它

另外一个容易搞混的点：**固件夹的是「硬件限位」，不是 Pi 的「软限位」**。
假 STM32 必须用同一套（`config.clamp_q_hard`），否则两边用同一张表，
"Pi 侧过度钳位"这类问题就测不出来了。

---

## 8. ⚠️ 固件行为清单（写 Pi 程序必须知道，全部已实测/读码确认）

| # | 行为 | 出处 / 后果 |
|---|---|---|
| 1 | **上电后、收到第一帧之前就进 ESTOP** | 失联判定用"距上次收到任意帧"，从没收到过 = 无穷大。所以 Pi 一上电就该发帧，**并且要主动重新请求模式**（模式不会自动回到 ACTIVE） |
| 2 | **`TEL_STATUS.voltage_mv` 恒为 0** | `app_main.c:800`。真电压只在 `TEL_FAST` 里 |
| 3 | **`TEL_STATUS` 温度写的是原始 °C** | 协议单位写的是 0.1 °C，实际差 10 倍。上真机用体温捂一下舵机即可分辨 |
| 4 | **`TEL_STATUS.mode` 是"请求的模式"** | 不是生效的模式。生效模式只能从 `STAT_ESTOP` 和实际运动推断 |
| 5 | **`CMD_CALIB` 的 ACK 恒为 `result=0`** | 没有成功/失败信息；START 失败也回 0（静默失败） |
| 6 | **`CMD_CALIB action=5`(MANUAL_ZERO) 被解码器拒收** | `link_proto.c` 只接受 0..4，连 ACK 都没有。Pi **发不出去** |
| 7 | **标定进度/错误码在协议里读不到** | 只在 ASCII 调试 shell（`bsp_shell.c`）。Pi 唯一可靠信号是 `STAT_CALIBRATED` 0→1 |
| 8 | **标定成功后自动落盘** | 不需要发 `action=3` |
| 9 | **`err_flags` 是 latch，只有重启才清** | 且只要非 0，安全层就把动作幅度压到 0.75（`rlca_safety.c:94-101`）→ 关节只走 75% |
| 10 | **`FLAG_ENABLE / FLAG_RESET_ERR / FLAG_REQ_CALIB` 都没实现** | 只有 `FLAG_REQ_STATUS(0x04)` 有效；`kp_scale`、`led` 字段被解码但从不使用 |
| 11 | **失联判定基于"任意帧"** | 只发 HEARTBEAT 就能让它以为链路正常、继续跟踪**冻结的旧目标**。心跳不能替代控制指令 |
| 12 | **上电 1000 ms 内不写舵机位置** | 且要 12 个关节反馈**全部**有效才开始动。发了指令但不动，先想这条 |
| 13 | **限速 0.15 rad/周期** | Pi 侧动作幅度若超过它会被硬削（≈860 °/s） |
| 14 | **USART1 上有 ASCII banner 和调试 shell** | 二进制流里会混入文本字节。`FrameDecoder` 能重新同步（已测） |
| 15 | **固件不发 HEARTBEAT / TEL_LOG** | `TEL_LOG` 在固件里根本没实现 |
| 16 | **固件默认使用模型 A**：发完 TEL_FAST 后最多等 4 ms 拿指令 | 等不到就用缓存的上一次目标，不会报错 |
| 17 | ⚠️ **USART1 上的 shell 实际上用不了** | `app_poll()` 里 `link_pi_poll()` 先跑并把 USART1 环形缓冲**抽干**（`link_pi.c` 是 `for(;;)` 循环），之后 `bsp_shell_poll()` 什么都拿不到。文档说"shell 与协议帧可共存于同一条 USART1"在代码上不成立。**后果：树莓派不能靠往 USART1 打字来调 shell**；标定进度/错误码只能走 ESP32 无线口（TCP 3333）或台式 USB-TTL 直连 STM32 |
| 18 | ⚠️ **失败的重标定无法用协议识别** | 标定失败时固件不会清 `STAT_CALIBRATED`（旧表还在），位一直是 1，没有 0→1 跳变。所以"起始就已标定"的机器人重标定失败，协议里看不出来 |
| 19 | ⚠️⚠️ **`MODE_STAND` 没有任何准入门（安全）** | 固件只给 `ACTIVE` 加了准入（`app_mode.c`）。而默认标定表是 `dir=+1/offset=0`（`rlca_calib_load_defaults` 里 memset 出来的），`q_target=0`（正是站姿）按 `rlca_q_to_step` 换算正好是 **舵机 step 0** → 12 个舵机全部冲向绝对位置 0（机械行程一端）。固件作者在 `app_main.c:695-709` 记录过这个撞机事故。**也就是说"未标定 → ACTIVE 降级 STAND"这个保护，对未标定的机器人本身就是危险路径。** 本层所有工具遵守"未标定不发 STAND/ACTIVE"，并有回归测试（K2b/K2c/K3b） |

---

## 9. 硬件环境（本机已配好，重启自动恢复）

```
Pi 5 / Debian 13 (trixie) / 内核 6.18.39+rpt-rpi-2712
串口  : /dev/ttyAMA0   ← dtoverlay=uart0-pi5 启用（真 PL011 AXI）
        ⚠️ /dev/ttyAMA10 是【专用调试串口】，console 和 getty 都在它上面，
           不要用它接 STM32
网络  : eth0 静态 169.254.100.2/16 + 192.168.50.2/24，无默认网关
        （直连 PC，不依赖 WiFi，重启持久）
CPU   : governor 仍为 ondemand，跑长循环前建议改 performance
```

---

## 附录 A. 与 RL 契约相关的已知不一致（**本轮范围外，仅存档**）

⚠️ 本层明确**不含**强化学习。但通读训练代码时发现几处"文档 / 已交付代码"与
"实际训出策略的那份训练代码"不一致。**这里只记录、不处理**，以免这些发现丢掉。
真要上策略时，**以 `Sim/legged_gym/legged_gym/envs/rlcat/` 的代码为唯一真源**。

| # | 项 | 文档 / 已交付代码怎么说 | 训练代码实际是什么 | 出处 |
|---|---|---|---|---|
| 1 | **速度指令缩放** | 协议文档写 ×1.0；`rlcat_proto.build_observation` 原样写入 | `commands[:, :3] * commands_scale`，而 `commands_scale = [obs_scales.lin_vel, lin_vel, ang_vel] = [2.0, 2.0, 0.25]` → 实际是 `[2·vx, 2·vy, 0.25·ωz]` | `legged_robot.py:515`、`rlcat_env.py:111`、`rlcat_config.py:110,112` |
| 2 | **上一帧动作缩放** | `docs/30-firmware.md:226`、`docs/31-pi-runtime.md:168` 写 ×4.0；`rlcat_proto.OBS_SCALE_ACTION = 4.0` | `self.actions` 裸值，**×1.0**（`legged_robot.py:86` 的 `self.actions = clip(策略原始输出)`） | `rlcat_env.py:114` |
| 3 | **策略输出幅值** | `docs/30-firmware.md:287` 说"action 经 tanh 后落在 [-1,1]，单步最大 0.25 rad" | 网络是 4×Linear + ELU，**没有 tanh 输出层**（`13_export_policy.py:43-44`：最后一层不加激活）。实测：站立附近 `max|a|≈1.66`；随机观测下可达 ±27 → `q_target` 可达 ±6.9 rad | `policy_weights.npz` 只有 net_0/2/4/6；`rlcat_config.py:143` `activation='elu'` |
| 4 | **关节顺序** | 协议/固件全程用逻辑索引 A | 策略的 `dof_pos/dof_vel/action` 是 Isaac 顺序 C = `[lb,lf,rb,rf]×[hip,thigh,calf]`；映射 `A2C=[3,0,9,6,4,1,10,7,5,2,11,8]`。**`rlcat_proto` 里没有任何置换** | `docs/30-firmware.md:79`、`Sim/RL/RL-CAT-IsaacGym训练流程方案.md §4.2` |
| 5 | **`TEL_STATUS.voltage_mv`** | 协议说是电池电压 | 固件写死为 0；真电压只在 `TEL_FAST` 里 | `app_main.c:800` |
| 6 | **`TEL_STATUS` 温度单位** | 协议规范 §4.8 写 0.1 °C | 固件写的是原始 °C（差 10 倍） | `app_main.c:809-815` |

**第 1~4 条如果照文档实现，策略在真机上会表现完全错误**（差 2~4 倍、关节错位）。
本层的 `config.py` / `telemetry.py` 只处理"基础"部分，**没有**实现观测组装，
所以这些不一致目前不影响本层；但它们必须在做策略部署之前解决。

---

**文档结束。** 要在机器人旁边操作，请用 [`使用说明.md`](使用说明.md)。
