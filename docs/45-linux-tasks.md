# 45 · 必须在 Linux 上完成的任务

> **本文档是给操作者的交接单。**
> 起因：开发机是 Windows，**没有安装 torch**（实测只有 `numpy 2.5.1`），
> 而策略导出、Isaac Gym 回放、树莓派侧 PTY 测试都只能在 Linux 上做。

---

## 0. 为什么必须是 Linux

| 任务 | 为什么 Windows 做不了 |
|---|---|
| **策略导出**（`.pt` → `.npz`） | 需要 `torch`，Windows 侧未安装（装一份也可以，但 Anaconda 在 D 盘、环境混杂，容易踩坑） |
| **仿真回放** | Isaac Gym 只支持 Linux + NVIDIA 驱动 |
| **树莓派侧 PTY 测试** | `pty` / `termios` 是 POSIX 专有（`fake_stm32.py`、`link.py` 自测、`tests/test_basic_offline.py`） |
| **实时性调优** | `PREEMPT_RT`、`cpufreq governor` 都在 Linux |

**⚠️ "Linux 机器"可以是：**
1. **训练策略那台机器**（已装 Isaac Gym + torch）← **首选**
2. 任意装了 torch 的 Linux（WSL 也可以，导出不需要 GPU）
3. 树莓派本身（`pip install torch` 在 aarch64 可用，但要几百 MB，**只作为最后手段**）

---

## 1. 任务总览

| 编号 | 任务 | 产物 | 阻塞什么 | 预计 |
|---|---|---|---|---|
| **L1** | **导出策略**（onnx + npz） | `pi/models/policy_rlcat_flat.{onnx,npz}` | **整个 sim2real** | 30 分钟 |
| **L2** | 树莓派依赖 + 实时性 | 一份环境报告 | 推理能否稳定 100 Hz | 30 分钟 |
| **L3** | 仿真回放策略（建议） | 10 s 的 obs/action 轨迹 | 排除"策略本身就不行" | 1 小时 |
| **L4** | 树莓派 Linux-only 测试 | 测试输出 | 没有（已通过，只需复核） | 10 分钟 |

**L1 是硬阻塞项。L2、L3 可以并行。**

**⚠️ L1 的额外依赖（都很小）：**

```bash
pip3 install --index-url https://download.pytorch.org/whl/cpu torch   # ~200 MB
pip3 install onnxruntime                                             # ~40 MB（可选，用于三方验证）
```

---

## 2. L1 · 导出策略（⭐ 最高优先级）

### 2.1 背景

训练好的检查点在这里：

```
Sim/legged_gym/logs/rlcat_flat/Oct06_02-58-05_/model_950.pt   ... model_1125 附近
Sim/legged_gym/logs/rlcat_flat/Oct06_03-15-34_/model_100.pt   ... model_1125
Sim/legged_gym/logs/rlcat_flat/Oct06_03-32-41_/model_500.pt
```

#### ⚠️⚠️ 用哪个检查点 —— 本文档曾给错过，已更正

**曾经的错误建议：**"`Oct06_03-15-34_` 是后起的训练，通常是更优的那次。"
**⇒ 这条假设是错的。** `Oct06_03-15-34_` 正是**第 8 轮失败实验**。

**实测对比（2026-10-18，Linux 侧交付时核对）：**

| run | 峰值平均回报 | 末期 episode 长度 | 末期 tracking | 判定 |
|---|---|---|---|---|
| `Oct06_02-58-05_`（第 7 轮） | 40.67 | 1909 | 0.801 | 🟡 可用，中高速欠响应 |
| `Oct06_03-15-34_`（第 8 轮） | **72.11 ← 全表最高** | **891 ← 会摔** | 0.547 | ❌ **失败** |
| **`Oct06_03-32-41_`（第 9 轮）** | 31.82 | **2002 ← 跑满** | 0.784 | ✅ **验收全达标** |

**⭐ 用 `Oct06_03-32-41_/model_500.pt`。**

> **📌 通用教训：`Mean reward` 在不同 episode 长度下**不可比**。**
> 第 8 轮把 `tracking_lin_vel` 权重提到 6.0，训练后期开始摔 —— episode 长度从
> 2002 掉到 891，惩罚累积变少，**平均回报反而被推高**。
> **判断模型好坏必须同时看 `Mean episode length`：长度掉了就是训练在变差，
> 此时 reward 数字再高也没有意义。**

**第 9 轮的固定指令验收（32 环境 × 15 s，零摔倒）：**

| v_cmd (m/s) | −0.20 | 0.00 | +0.20 | +0.30 | +0.50 |
|---|---|---|---|---|---|
| 跟踪误差 | 0.064 | 0.060 | 0.066 | 0.081 | 0.099 |

**详见 [`docs/47-policy-export-report.md`](47-policy-export-report.md)。**

---

### 2.2 ⭐ 为什么 Pi 上跑 numpy，而 ONNX 只做发布格式

**这是个被认真比较过的决策，不是偏好。实测数据：**

```
参数总数       = 54220
numpy 前向     = 32.3 µs/次（Windows x86）  ⇒ Pi 5 约 65~97 µs
10 ms 预算占用 = 0.3~1.0%
```

**⇒ 这个网络小到"推理速度"根本不是决策依据。**

| 维度 | **numpy（`.npz`）** | **onnxruntime** |
|---|---|---|
| 依赖体积 | ✅ 只需 numpy | ⚠️ wheel **30~50 MB** |
| 单次耗时 | **32 µs（纯计算）** | 推理类似 + **API 固定开销 10~50 µs** |
| 冷启动 | ✅ 174 ms | ⚠️ import ORT **0.3~1 s** |
| **⚠️ 线程池** | ✅ 无 | ⚠️ **默认按核数开线程**，Pi 5 只有 4 核，会和 100 Hz 控制循环抢 CPU ⇒ **延迟毛刺** |
| 架构可变 | ❌ 只支持 MLP | ✅ 任意结构 |
| **发布/共享** | ⚠️ 需配 `rl_policy.py` | ✅ **标准格式**，Netron 可看，C++/网页/MATLAB 通用 |
| 调试透明 | ✅ 每层可打印 | ⚠️ 看中间层要额外配置 |

**结论：**

```
.pt  ──┬──► .onnx   ← 【模型发布格式】进仓库，给别人验证/复用
       │
       └──► .npz    ← 【Pi 运行时格式】零依赖、32 µs、可逐层调试
```

**⚠️ 如果将来真要改用 ORT，这三个参数必须配（否则真机会出毛刺）：**

```python
so = ort.SessionOptions()
so.intra_op_num_threads = 1
so.inter_op_num_threads = 1
so.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
# 且启动时预热 ≥10 次（第一次调用会分配内存、触发图优化）
```

**⚠️ 还要注意：ORT 的图优化会**轻微改变浮点结果**（~1e-6），
所以对比测试不能用"逐位相等"，必须放宽到相对 1e-5。**

---

### 2.3 跑什么

**先装 onnxruntime（CPU 版，很小；不做三方验证的话可以跳过）：**

```bash
pip3 install onnxruntime
```

```bash
cd <仓库根>

python3 tools/export_policy.py \
    Sim/legged_gym/logs/rlcat_flat/Oct06_03-15-34_/model_1125.pt \
    -o pi/models/policy_rlcat_flat
```

**⚠️ `-o` 给的是**基名**（不带扩展名），脚本会同时产出：**

```
pi/models/policy_rlcat_flat.onnx     ← 发布格式
pi/models/policy_rlcat_flat.npz      ← Pi 运行时
```

**期望输出：**

```
 actor 结构：
   0: 47    → 256    ELU                  W(256, 47) b(256,)
   1: 256   → 128    ELU                  W(128, 256) b(128,)
   2: 128   → 64     ELU                  W(64, 128) b(64,)
   3: 64    → 12     （输出层，无激活）   W(12, 64) b(12,)
   参数总数 = 54220

   ONNX 导出成功（opset 17）
   大小 约 210 KB
   ✅ 已保存 .../policy_rlcat_flat.npz（约 210 KB）

 ---- 三方一致性验证（torch / onnx / numpy）----
   #0  torch[0:3]=[...]  numpy[0:3]=[...]  rel=0.00e+00  onnx_rel=4.77e-07
   ...
   torch vs numpy : 最大绝对 0.000e+00  最大相对 0.000e+00
   torch vs onnx  : 最大绝对 1.192e-07  最大相对 4.768e-07
   ✅ 一致（容差 1e-05）

 ---- 可选：用 rsl_rl 真实 ActorCritic 复核 ----
   与 rsl_rl 真实 actor 的最大相对误差 = 0.000e+00
   ✅ 层顺序假设正确
```

#### 三种"只看一部分"的用法

```bash
# 没装 onnxruntime、只要 npz
python3 tools/export_policy.py <ckpt> -o <base> --no-onnx

# 只要 onnx（给别人验证用）
python3 tools/export_policy.py <ckpt> -o <base> --no-npz

# 跳过验证（⚠️ 只用于快速试错，上真机前必须补跑）
python3 tools/export_policy.py <ckpt> -o <base> --no-verify
```

---

### 2.4 判据

| 检查 | 期望 | 不符怎么办 |
|---|---|---|
| 第一层 `47 → 256` | 47 入、256 出 | 选错了检查点（拿到 `rough_a1` 的 A1 策略了） |
| 最后一层 `→ 12` | 12 出 | 同上 |
| **参数总数 = 54220** | 完全一致 | 架构或检查点不对 |
| `torch vs numpy` 相对误差 | **= 0**（同一批权重、同一算法） | **激活函数或层顺序写错了** |
| `torch vs onnx` 相对误差 | **< 1e-6**（图优化导致，正常） | opset 或导出有问题 |
| **`rsl_rl 真实 actor` 相对误差** | **< 1e-5** | ⚠️ **层顺序假设错了** —— 这条是唯一**不循环**的检查 |

**⚠️ 为什么最后一条最重要：**

**前两个对比（torch/numpy、torch/onnx）都建立在"我猜的层顺序"上 —— 如果猜错了，它们会**一致地错**。**
**只有用 `rsl_rl` 真实的 `ActorCritic` 重建再比较，才能证明"层顺序假设"本身是对的。**

**⚠️ 如果 `rsl_rl` 没装，这条会自动跳过 —— 那就**没有**独立验证，务必把完整输出发我人工核对。**

---

### 2.5 产物怎么传回来

**两个文件都要：**

| 文件 | 去哪 |
|---|---|
| `*.npz`（~210 KB） | **传树莓派** `pi/models/` |
| `*.onnx`（~210 KB） | **提交进仓库**（发布格式） |

```bash
# ① 传到树莓派
ssh <pi> 'mkdir -p ~/RL-CAT/pi/models'
scp -i ~/.ssh/rlcat_pi_ed25519 \
    pi/models/policy_rlcat_flat.npz \
    pi@169.254.100.2:~/RL-CAT/pi/models/

# ② 把 onnx 也拷回 Windows 开发机的仓库里（或直接在仓库里生成）
```

**⚠️ 两个文件都只有 ~210 KB，都**应该进版本控制**：**
**别人不用重跑训练（也不需要 Isaac Gym）就能复现策略。**

---

## 3. L2 · 树莓派依赖与实时性

### 3.1 依赖核查

```bash
ssh <pi>
cd ~/RL-CAT/pi

# numpy 必须有（推理层只用 numpy，不需要 torch/onnxruntime）
python3 -c "import numpy; print('numpy', numpy.__version__)"
```

| 结果 | 处理 |
|---|---|
| 有 numpy（任意版本 ≥ 1.20） | ✅ 够用，什么都不用装 |
| 没有 | `sudo apt install python3-numpy` |

**⚠️ 刻意不装 `torch`** —— 策略只有 54220 个参数，numpy 前向是微秒级；
torch 在树莓派上要几百 MB、启动几秒，纯属负担。

### 3.2 CPU governor（影响 100 Hz 稳定性）

```bash
# 看当前
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# 设成 performance（否则降频会导致周期抖动）
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

**⚠️ 这是临时的，重启失效。** 要持久化需要写 systemd unit 或改
`/etc/rc.local`。**请把"你用的持久化方式"记下来，后面要写进文档。**

### 3.3 环境报告（交付物）

**请把以下命令的输出整理成一份文本发我：**

```bash
uname -a
python3 -V
python3 -c "import numpy; print(numpy.__version__)"
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
cat /proc/device-tree/model
free -m
vcgencmd measure_temp
```

**用途：写进 `docs/44-reproduce.md`，作为"已验证环境"的基线。**

---

## 4. L3 · 仿真回放策略（建议做，能省掉大量真机试错）

### 4.1 为什么

**在把策略放到真机之前，先确认它在仿真里确实能走。** 否则真机摔了也分不清是
"策略不行"还是"部署错了"。

### 4.2 怎么做（两种，任选）

**方式 A：用 legged_gym 自带的 play 脚本**

```bash
cd Sim/legged_gym
python3 legged_gym/scripts/play.py \
    --task=rlcat_flat --resume --load_run=<run_dir> --checkpoint=<N> --num_envs=1
```

**观察：** 机器人能不能稳定小跑？速度指令跟随如何？

**方式 B：录制一份 obs/action 轨迹（⭐ 对 sim2real 更有用）**

**在 `play.py` 里插一段，把每步的 `obs` 和 `actions` 存成 `.npz`
（或写个独立脚本）。** 这份数据在真机上可以**逐位对比**：
- 真机的 obs 构造有没有错
- 策略输出有没有错

**⇒ 这是排查 sim2real 最快的工具，强烈建议做。**

### 4.3 产物

```
Sim/RL/replay/rlcat_flat_obs_action_trace.npz
    包含: obs (T, 47), actions (T, 12), dt=0.01
```

---

## 5. L4 · 树莓派侧只能在 Linux 跑的测试

```bash
ssh <pi>
cd ~/RL-CAT/pi

./run_checks.sh                     # 期望: PASS 117 + 全部通过
python3 tests/test_basic_offline.py # 期望: PASS 117 FAIL 0
python3 selfcheck.py                # 期望: PASS 141 FAIL 0
python3 link.py                     # PTY 自测，8 组全绿
```

**⚠️ 已知会自动通过的项，无需贴给我；只有**失败项**需要报。**

---

## 6. 交付清单（做完请回我这些）

| # | 交付物 | 形式 |
|---|---|---|
| **1** | `pi/models/policy_rlcat_flat.npz` | 文件（Pi 运行时） |
| **2** | `pi/models/policy_rlcat_flat.onnx` | 文件（发布格式，进仓库） |
| **3** | 导出脚本的**完整输出** | 文本（**含三方验证 + rsl_rl 复核那两段**） |
| **4** | L2 环境报告 | 文本 |
| **5** | L3 的结论：策略在仿真里能走吗？ | 一句话 + 可选短视频 |
| **6** | （可选）`replay/*.npz` 轨迹 | 文件 |

---

## 7. 附：如果 Linux 机器上没有 torch

### 7.1 WSL（最省事）

```powershell
# Windows 侧（管理员 PowerShell）
wsl --install -d Ubuntu-22.04
```

```bash
# Ubuntu 里
sudo apt update && sudo apt install -y python3-pip
pip3 install --index-url https://download.pytorch.org/whl/cpu torch
pip3 install onnxruntime numpy
```

**⚠️ CPU 版就够**（导出不需要 GPU），torch 约 200 MB + onnxruntime 约 40 MB。

### 7.2 只装 CPU 版的关键

**⚠️ 不要装默认的 CUDA 版**（2 GB+）。上面的 `--index-url` 指定了 CPU 轮子。

**⚠️ `onnxruntime` 只在"三方验证"时需要**（导出 onnx 本身只靠 torch）。
**没装也能导出，只是验证会退化成 torch/numpy 两项。**

### 7.3 装完验证

```bash
python3 -c "import torch, numpy; print(torch.__version__, numpy.__version__)"
```

**然后回到 §2.2 跑导出。**

---

## 8. ⚠️ 三个容易踩的坑（提前说明）

| 坑 | 现象 | 处理 |
|---|---|---|
| **选错检查点** | 导出时第一层不是 47 入 | `logs/` 下还有 `rough_a1`（A1 机器人），别拿那个 |
| **`torch.load` 报错** | `WeightsUnpickler ... unsupported` | 脚本已处理（先试 `weights_only=False`）；若仍失败，把完整报错发我 |
| **层顺序/激活猜错** | `torch vs numpy` 或 `rsl_rl 复核` 报不一致 | **绝对不要上真机**，把三段输出都发我 |
| **`onnxruntime` 没装** | 三方验证退化成两项 | 想完整验证就 `pip3 install onnxruntime`；否则把输出发我人工核对 |
| **`--help` 报缺 torch** | 不该发生（已修：torch 延迟导入） | 如果你看到这个，说明脚本是旧版 |

---

## 9. 与主线的关系

**L1 完成 = sim2real 的"策略"这个零件就备齐了。**

**但真机能不能用，还取决于一个**尚未验证的前置条件**：**
**机器人必须在**地面上**稳定站住。**（至今所有测试都是吊起的。）

**⇒ 主线方案见 `docs/46-mainline-plan.md`。**
