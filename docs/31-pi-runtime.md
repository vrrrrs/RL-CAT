# 树莓派侧程序设计（Pi Runtime）

> **文档编号**：DOC-PI-001
> **版本**：v1.0
> **日期**：2026-10-09
> **面向对象**：树莓派侧程序开发者 / Agent
> **配套文档**：[`30-firmware.md`](30-firmware.md)、[`firmware/doc/协议规范_树莓派通信.md`](../firmware/doc/协议规范_树莓派通信.md)

---

## 0. 职责边界

**树莓派只做两件事：跑策略、发指令。其余全部交给 STM32。**

| 职责 | 树莓派 | STM32 |
|---|---|---|
| 策略推理 | ✅ | ❌ |
| **组装 47 维观测** | ✅ | ❌ |
| 生成关节目标角 | ✅ | ❌ |
| 舵机总线时序 | ❌ | ✅ |
| 传感器采集 | ❌ | ✅ |
| **步态相位生成** | ❌ | ✅ ⭐ |
| **安全保护** | ❌ | ✅ ⭐ |
| 日志记录 | ✅ | ❌ |

**⚠️ 两条硬规则：**

```
① 树莓派【不做】任何安全相关的兜底 —— 安全保护在 STM32 上独立完成。
   树莓派崩了，STM32 必须能自己停下来。
② 树莓派【不产生】步态相位 —— 用 STM32 下发的（理由见 30-firmware.md §5.1.1）。
```

---

## 1. 已确定的决策

| # | 决策 | 值 |
|---|---|---|
| 1 | **策略加载** | **ONNX**（`onnxruntime`）⭐ 便于开源与跨平台 |
| 2 | **串口** | **UART0**（GPIO14/15，专用 PL011）⭐ 不用 mini-UART |
| 3 | **线程模型** | **单线程主循环** + 异步日志 |
| 4 | **步态相位** | **STM32 产生**，经 TEL_FAST 下发 |
| 5 | **急停** | 策略侧不负责；由 ESP32 硬线 + STM32 保护处理 |
| 6 | **指令源** | 键盘（开发）+ 脚本注入（评估） |

---

## 2. 主循环

**100 Hz（10 ms 周期）。**

```
┌─────────────────────────────────────────────────────────┐
│  每个周期：                                              │
│                                                         │
│  ① 收 TEL_FAST（阻塞至多 5 ms，带超时）                  │
│      ├─ 超时 → 记一次丢帧，用上一帧数据继续（或跳过）     │
│      └─ CRC 错 → 丢弃，继续收                            │
│                                                         │
│  ② 组装 47 维观测                                        │
│      · 从 TEL_FAST 取：四元数/陀螺/关节/相位             │
│      · 本地取：速度指令、上一帧动作                       │
│                                                         │
│  ③ 策略推理（ONNX）→ 12 维动作                           │
│      ⚠️ NaN/Inf 检查                                     │
│                                                         │
│  ④ 组 CMD_JOINT（38 字节）→ 发送                          │
│                                                         │
│  ⑤ 记录：观测 / 动作 / 周期耗时 → 内存环形缓冲            │
│                                                         │
│  ⑥ 低频任务（每 N 个周期）：刷盘、读键盘、更新指令         │
└─────────────────────────────────────────────────────────┘
```

### 2.1 ⚠️ 时序模型

**采用「请求-应答」模型（协议文档 §7.3 的模型 A）：**

```
STM32 发起周期 → 发 TEL_FAST → 等 Pi 的 CMD_JOINT（超时 4 ms）
```

**⚠️ 这意味着 Pi 必须在 4 ms 内完成"解包→组装→推理→打包→发送"。**

**时间预算（Pi 5）：**

| 步骤 | 预计耗时 |
|---|---|
| 收 TEL_FAST（103 B @ 500 kbps） | 2.06 ms（硬件时间，不占 CPU） |
| 解包 + 组装 47 维观测 | ~0.1 ms |
| ONNX 推理（MLP 256-128-64） | ~0.2~0.5 ms |
| 组包 + 发送（38 B） | 0.4 ms（硬件时间） |
| **合计** | **≈ 1.5~2.0 ms** |

**✅ 余量约 2 ms。但要实测确认，不要把估算当结论。**

**⚠️ 如果实测发现 4 ms 不够，退路是切到模型 B**（STM32 不等指令，用上一帧）—— 代价是端到端延迟从 ~3 ms 增到 ~13 ms，但周期抖动归零。

---

## 3. 程序结构

```
pi/
├── main.py              100 Hz 主循环 + 命令行入口
├── protocol.py          帧编解码 + CRC16（与固件共用测试向量）
├── observation.py       ⭐ 47 维观测组装（最容易错的一环）
├── policy.py            ONNX 加载与前向
├── command.py           指令源（键盘 / 脚本 / 手柄）
├── link.py              串口收发 + 超时 + 重连
├── recorder.py          内存环形缓冲 + 异步落盘
├── safety.py            树莓派侧的软保护（不等同于 STM32 的硬保护）
└── config.py            所有参数集中
```

### 3.1 `link.py` —— 串口层

```python
class PiLink:
    """与 STM32 的串口链路"""

    def __init__(self, port, baud=1_000_000, timeout=0.005):
        # ⚠️ 必须做的串口配置：
        #   · 关掉 canonical 模式（否则会等换行符）
        #   · 关掉流控
        #   · 设 raw 模式
        #   · VMIN=0, VTIME=0 → 非阻塞读
        ...

    def read_frame(self) -> dict | None:
        """收一个完整帧。返回 None = 超时或校验失败"""

    def send_cmd_joint(self, q_target, mode, flags) -> bool:
        """发 CMD_JOINT"""

    def stats(self) -> dict:
        """返回：rx_frames / crc_errors / timeouts / cycle_jitter"""
```

**⚠️ 三个注意点：**

| # | 注意 |
|---|---|
| 1 | **非阻塞读 + `select`** —— 不要用阻塞 `read()`，它会卡住整个循环 |
| 2 | **接收缓冲要能容纳整帧**（103 + 7 = 110 B），且要处理跨周期残包 |
| 3 | **记录 `rx_overflow` / `crc_errors` / `timeouts`** —— 这三个是判断链路健康的指标 |

**⚠️ 协议接收状态机必须和固件侧一致**（见协议文档 §6），包括：
- `WAIT_HEAD1` 收到 `0xAA` 时留在本状态
- LEN ≤ 128 的上限检查
- 整帧超时

### 3.2 `observation.py` —— ⭐ 最容易错的一环

**参照实现（协议文档 §9 有完整代码）：**

```python
def build_observation(tel, cmd, last_action, q_default=None):
    if q_default is None:
        q_default = [0.0] * 12

    obs  = quat_to_gravity(tel['quat'])                      #  0- 2
    obs += [v * 0.25 for v in tel['gyro']]                   #  3- 5
    obs += [cmd['vx'], cmd['vy'], cmd['wz']]                 #  6- 8
    obs += [tel['q'][i] - q_default[i] for i in range(12)]   #  9-20
    obs += [v * 0.1 for v in tel['qd']]                      # 21-32
    obs += [v * 4.0 for v in last_action]                    # 33-44
    obs += [tel['gait_sin'], tel['gait_cos']]                # 45-46
    assert len(obs) == 47
    return obs
```

**⚠️⚠️ `last_action` 的语义必须和训练侧一致：**

```
训练侧（legged_gym）：last_action = 上一步【策略输出】（tanh 之后、缩放之前）
                       观测里乘 4.0 = 1/action_scale

⚠️ 不是「缩放后的关节角」，也不是「clamp 之后的值」。
   必须是策略原始输出的 12 维向量。
```

**⚠️ 必须写单元测试：** 用 Sim 里录的一组已知观测值验证组装结果逐维一致。**这是唯一能保证位对齐的方法。**

### 3.3 `policy.py` —— ONNX 推理

```python
import onnxruntime as ort
import numpy as np

class Policy:
    def __init__(self, onnx_path):
        # ⚠️ 单线程 + 关掉内存竞技场，减少抖动
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 1
        opts.inter_op_num_threads = 1
        opts.enable_cpu_mem_arena = False

        self.sess = ort.InferenceSession(onnx_path, opts,
                                         providers=['CPUExecutionProvider'])
        self.input_name = self.sess.get_inputs()[0].name
        # ⚠️ 预分配输入缓冲，避免每周期分配
        self._buf = np.zeros((1, 47), dtype=np.float32)

    def __call__(self, obs) -> np.ndarray:
        self._buf[0, :] = obs                      # 原地填充，不重新分配
        out = self.sess.run(None, {self.input_name: self._buf})[0]
        return out[0]                              # (12,)  ← tanh 后的动作
```

**⚠️ 四个必须注意的点：**

| # | 注意 |
|---|---|
| 1 | **`intra_op_num_threads = 1`** —— 多线程在这个小网络上反而增加抖动 |
| 2 | **预分配输入缓冲** —— 每周期 `np.array(...)` 会产生 GC 压力 |
| 3 | **输出已含 tanh** —— ONNX 图应该是完整的前向（含激活），不要重复施加 |
| 4 | **首次推理会慢**（图优化），要在启动时用假数据预热一次 |

**⚠️ ONNX 导出时要确认图中包含 tanh 输出层。** 如果导出的是"裸 MLP 输出"（未过 tanh），Pi 侧要补 `np.tanh`。

**⚠️ 必须做的验证：** 同一组输入下 ONNX vs PyTorch 最大偏差 < 1e-5。

### 3.4 `command.py` —— 指令源

```python
class CommandSource:
    """速度指令源。线程安全（写方是键盘线程，读方是主循环）"""

    def get(self) -> dict:
        """返回 {'vx':..., 'vy':..., 'wz':...}"""

    def set_mode(self, mode: int):
        """切换运行模式，见协议文档 §4.2"""
```

**三种实现：**

| 实现 | 用途 | 说明 |
|---|---|---|
| `KeyboardSource` | 开发调试 | ⚠️ **必须非阻塞** |
| `ScriptSource` | 评估 / 自动化 | 按预设序列给指令 |
| `GamepadSource` | 以后加 | 需要 `evdev` 或 `pygame` |

**⚠️⚠️ 键盘输入的陷阱：**

```
❌ 错误做法：
   vx = float(input("vx> "))     # input() 会阻塞，整个 100 Hz 循环卡死

✅ 正确做法：
   · 单独一个线程读键盘（termios + select 非阻塞）
   · 只往共享变量写；主循环只读
   · 或者用 curses / tty.setcbreak() + sys.stdin.read(1)
```

**⚠️ 主循环读共享变量时要加锁或保证原子性** —— Python 里读一个 dict 不是原子的。

### 3.5 `recorder.py` —— 日志

**⚠️ 核心要求：落盘不能阻塞主循环。**

```python
class Recorder:
    """内存环形缓冲 + 异步落盘"""

    def __init__(self, capacity_sec=60):
        # 预分配 numpy 数组，不动态增长
        self._buf = np.zeros((capacity_sec * 100, RECORD_SIZE), dtype=np.float32)
        self._idx = 0

    def push(self, obs, action, tel, timing):
        """主循环调用 —— 必须极快（只做一次内存写入）"""
        self._buf[self._idx] = ...
        self._idx = (self._idx + 1) % len(self._buf)

    def flush_async(self):
        """低频调用（例如每 100 个周期）—— 可以慢"""
```

**⚠️ 记录什么：**

| 项 | 用途 |
|---|---|
| 47 维观测 | 复盘"策略看到了什么" |
| 12 维动作 | 复盘"策略输出了什么" |
| STM32 遥测（电压/状态/错误码） | 判断硬件异常 |
| **周期耗时** | ⭐ 判断实时性 |
| **丢帧计数（cycle_cnt 跳变）** | ⭐ 判断链路健康 |

---

## 4. ⚠️ 实时性卫生（必做）

**Linux 不是实时系统。不做下面这些，"100 Hz"就只是名义上的。**

| # | 措施 | 命令 / 做法 |
|---|---|---|
| **1** | **CPU governor 设成 `performance`** | `sudo cpupower frequency-set -g performance` |
| **2** | **预分配所有缓冲区** | 见 §3.3 / §3.5 |
| **3** | **串口非阻塞读** | `select` + `VMIN=0` |
| **4** | **关掉串口 canonical 模式** | `termios` 设 raw |
| **5** | **可考虑绑核** | `os.sched_setaffinity(0, {3})` |
| **6** | ⭐ **记录周期抖动直方图** | 必须做，见下 |

### 4.1 ⭐ 周期抖动是判断可行性的唯一指标

```python
# 在每个周期开头与结尾打时间戳
t0 = time.perf_counter()
... 周期工作 ...
dt = time.perf_counter() - t0

# 统计：均值 / P50 / P95 / P99 / 最大值
# 判据：P99 < 4 ms（否则会踩 STM32 的等待超时）
```

**⚠️ 没有这个数字，你就分不清"是策略问题还是时序问题"。**

**建议输出（每 10 秒打一次）：**

```
[周期] 均值 1.82 ms  P50 1.79  P95 2.15  P99 2.63  最大 4.10
[链路] rx 1000  crc_err 0  timeout 0  lost 0
```

### 4.2 ⚠️ Python 特有的坑

| # | 坑 | 对策 |
|---|---|---|
| 1 | **GC 停顿** | 预分配 + 避免每周期创建对象；必要时 `gc.freeze()` |
| 2 | **GIL** | 单线程主循环规避；只有日志/键盘线程，且它们不做重活 |
| 3 | **浮点精度** | 观测用 `float32`（和训练一致），不要混用 float64 |
| 4 | **`time.sleep()` 精度** | 不要用它做周期节拍（Linux 下精度差）—— 用 STM32 的帧作为节拍 |

**⚠️ 第 4 条很重要：** 主循环的节拍应该**由 STM32 的 TEL_FAST 到达驱动**，而不是 Pi 自己 `sleep(0.01)`。这样 Pi 自动跟上 STM32 的精确时钟。

---

## 5. 启动与关闭流程

### 5.1 启动

```
① 解析命令行参数（串口设备、ONNX 路径、指令源、模式）
② 打开串口
③ 加载 ONNX（并预热一次推理）
④ 连续读几帧 TEL_STATUS 做链路自检：
     □ 电压正常（>10.5 V）
     □ status 的 STAT_READY 位已置
     □ STAT_CALIBRATED 位已置（标定数据有效）
     □ err_flags == 0
   ⚠️ 任一不满足 → 报警并退出，不要贸然进入 ACTIVE
⑤ 进入主循环（先 MODE_IDLE，等操作员确认后再给 MODE_ACTIVE）
```

**⚠️ 第 ④ 步不能省** —— 带着未标定的舵机跑策略会直接摔。

### 5.2 关闭（⚠️ 必须优雅）

```python
try:
    run_main_loop()
except KeyboardInterrupt:
    pass
finally:
    # ⚠️ 三件事必须做，顺序不能错
    link.send_mode(MODE_IDLE)      # ① 先让机器人卸力
    time.sleep(0.2)                # ② 等它真的收到
    link.close()                   # ③ 再断开
    recorder.flush()               # ④ 落盘
```

**⚠️ 如果不发 `MODE_IDLE` 就断开** —— STM32 会等 30 ms 后进入降级，再过 100 ms 强制趴下。**结果是机器人突然瘫倒，而不是缓慢卸力。**

---

## 6. ⚠️ 树莓派侧的软保护（补充，不替代 STM32 的硬保护）

| 机制 | 触发条件 | 动作 |
|---|---|---|
| **NaN/Inf 检查** | 策略输出出现 NaN/Inf | **立即停发指令** + 报错退出 |
| **推理超时** | 单次推理 > 5 ms | 记一次异常；连续 3 次 → 停发 |
| **链路失联** | 50 ms 收不到 TEL_FAST | 停止发指令（让 STM32 自己保护） |
| **观测越界** | 任一维超出合理范围 | 记警告；连续出现 → 停发 |
| **CRC 错误率过高** | 连续 10 帧 CRC 错 | 报警（链路有问题） |

**⚠️ 最重要的两条：**

```
① NaN/Inf 检查 —— 这是"策略崩了"最常见的表现。
   不加检查会把 NaN 直接发成关节目标角，舵机可能剧烈乱动。

② 失联时【停止发指令】而不是继续发 —— 
   继续发旧指令会让 STM32 以为链路还活着，失去失联保护的意义。
```

---

## 7. 配置文件（建议）

**所有参数集中，对齐 [`30-firmware.md`](30-firmware.md) §6。**

```python
# config.py
class Config:
    # ── 链路 ──
    SERIAL_PORT   = "/dev/ttyAMA0"    # ⚠️ UART0（PL011），不是 ttyS0
    SERIAL_BAUD   = 1_000_000

    # ── 控制 ──
    CONTROL_HZ    = 100
    ACTION_SCALE  = 0.25
    NUM_JOINTS    = 12
    OBS_DIM       = 47

    # ── 观测缩放（⚠️ 必须与训练侧一致）──
    SCALE_GYRO    = 0.25
    SCALE_QD      = 0.1
    SCALE_ACTION  = 4.0               # = 1/ACTION_SCALE

    # ── 步态 ──
    GAIT_FREQ_HZ  = 3.0               # ⚠️ 与 rlcat_config.py 一致（但相位由 STM32 产生）

    # ── 超时 ──
    FRAME_TIMEOUT_MS   = 5
    INFER_TIMEOUT_MS   = 5
    LINK_LOST_MS       = 50

    # ── 策略 ──
    POLICY_ONNX   = "policy.onnx"

    # ── 安全 ──
    Q_DEFAULT     = [0.0] * 12        # ⚠️ 全 0，已实测确认
```

---

## 8. 开发与验证顺序

```
① protocol.py
   验收：CRC 标准测试向量 0x29B1 通过；空帧/满帧/截断帧/垃圾流 都不崩
   ↓
② link.py
   验收：PC 端用 loopback 或 STM32 回显，连续收 10 万帧零误码
   ↓
③ observation.py
   验收：⭐ 用 Sim 录制的已知观测值逐维比对通过
   ↓
④ policy.py
   验收：ONNX vs PyTorch 最大偏差 < 1e-5
   ↓
⑤ 联调（无舵机，只跑链路 + 推理）
   验收：周期抖动 P99 < 4 ms，连续跑 10 分钟无异常
   ↓
⑥ 接真机（吊起）
   验收：关节运动方向正确、幅度合理
```

**⚠️ 第 ③ 步是"能不能上真机"的关键门槛 —— 观测组装错一维，策略就会输出错误的动作。**

**⚠️ 第 ⑤ 步之前不要接舵机。** 先把纯软件链路跑稳。

---

## 9. 待确认

```
□ 是否保留 TorchScript 作为备用格式（ONNX 出问题时的退路）
□ 日志格式与回放工具（真机数据怎么分析）
□ 是否需要在 Pi 上做实时内核（PREEMPT_RT）—— 取决于实测抖动
□ 手柄支持是否要现在做
□ 观测是否需要对 47 维做 clip（防止异常值进入策略）
```

---

## 10. ⭐ 实现状态（2026-10-12 更新）

**基础部分已实现并离线验证完毕**（链路 / 遥测 / 标定 / 点动 / 姿态）。
**`observation.py` 与 `policy.py` 不在本轮范围**（属于强化学习，明确排除）。
操作手册见 [`../pi/使用说明.md`](../pi/使用说明.md)；开发记录见 [`../pi/README.md`](../pi/README.md)。

### 10.1 与 §3「程序结构」的对照

§3 给的是一套"策略运行时"的模块划分。实际实现按**能力分层**，并且把
RL 相关的两块（`observation.py` / `policy.py`）整块留空：

| §3 设计的文件 | 实际对应 | 说明 |
|---|---|---|
| `protocol.py` | `pi/rlcat_proto.py`（固件交付方提供） | 未改，直接复用 |
| `link.py` | ⭐ `pi/link.py` | 按 §3.1 的三条注意点实现（非阻塞 + `select`、残帧、三个健康指标），并加了 `cycle_cnt` 丢帧检测与诊断报告 |
| `config.py` | ⭐ `pi/config.py` | 参数集中：关节表（逻辑索引 A）、硬件限位 + Pi 软限位、电池/温度阈值 |
| `command.py` | 部分：`pi/jog.py` 的交互键盘 + `--sequence`；`pi/pose.py` 的 `--pose/--from/--cycles` | ⚠️ **手柄（GamepadSource）未实现** |
| `recorder.py` | ⚠️ **未实现** | 目前只有 `pi/status_panel.py --mode log` 的行日志。**真机长跑后的数据复盘会需要它**（环形缓冲 + 异步落盘） |
| `safety.py` | ⚠️ 部分实现 | NaN/越界钳位在 `config.clamp_q*`；未标定拒动在各工具入口；失联保护靠固件。**没有独立的安全模块** |
| `observation.py` | ❌ 未实现 | **本轮范围外**（RL） |
| `policy.py` | ❌ 未实现 | **本轮范围外**（RL） |
| `main.py` | 换成了 `pi/acceptance.py` + 各阶段 CLI | 基础阶段没有"常驻主循环"的需求；验收脚本更适合 |

新增（§3 未提及）：`pi/telemetry.py`（遥测语义层）、`pi/tools/fake_stm32.py`（假 STM32）、
`pi/tools/uart_loopback.py`（物理层误码实测）、`pi/tests/test_basic_offline.py`（端到端 117 项）。

### 10.2 §9「待确认」的收敛

```
☑ 是否保留 TorchScript 备用              → 属 RL 范围，未决（numpy 权重已在 Sim/RL/exported/）
☑ 日志格式与回放工具                     → ⚠️ 未做，见上表 recorder.py
☑ 是否要实时内核（PREEMPT_RT）           → ⏳ 待实测抖动；先按 §4 做实时性卫生
☑ 手柄支持是否现在做                     → ❌ 未做
☑ 观测是否要对 47 维做 clip              → 属 RL 范围，未决
```

### 10.3 ⚠️ 本文档需要更正/补充的地方

| # | 位置 | 说明 |
|---|---|---|
| 1 | §3.1 提到的设备节点 | 实际是 **`/dev/ttyAMA0`**（由 `dtoverlay=uart0-pi5` 启用，真 PL011 AXI）。⚠️ `/dev/ttyAMA10` 是 Pi 5 的**专用调试串口**，不是同一个东西 |
| 2 | §3.2 / §6 里与观测契约相关的数值 | ⚠️ 与训练代码存在若干不一致（速度指令缩放、上一帧动作缩放、无 tanh 输出层、逻辑索引 A vs Isaac 顺序 C）。**已单独存档在 [`../pi/README.md`](../pi/README.md) 附录 A**，本轮（基础部分）不处理 |
| 3 | §5.1 第 ④ 步「电压正常（>10.5 V）」 | 电压必须从 **`TEL_FAST`** 取；`TEL_STATUS.voltage_mv` 固件写死为 0 |
| 4 | §5.1 第 ④ 步「STAT_CALIBRATED 位已置（标定数据有效）」 | 这一条**非常关键**，而且不止是"标定有效"——见下条 |
| 5 | （本文档未提及）⚠️⚠️ | **未标定时绝不能发 `MODE_STAND` / `MODE_ACTIVE`**：固件的 `STAND` 没有准入门，默认标定表（`dir=+1/offset=0`）下 `q_target=0` 会换算成舵机 **step 0**，把 12 个关节驱到机械行程一端。详见 [`../pi/README.md`](../pi/README.md) §8 第 19 条 |
| 6 | §6 的软保护表 | "推理超时"属于 RL 范围，未实现；其余项（NaN 检查、链路失联、观测越界）在基础层按各自工具实现，但**没有独立的 `safety.py`** |

---

**文档结束。开发前请先读 [`30-firmware.md`](30-firmware.md) §5（观测/动作定义）。**
