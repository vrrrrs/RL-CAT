# 参与贡献

本文档说明本项目的开发流程、提交规范与必须通过的检查。

---

## 1 · 必读文档

| 文档 | 用途 |
|---|---|
| [`docs/49-progress-review.md`](docs/49-progress-review.md) | 25 项已完成分析的工程问题及其根因。提问前请先查阅。 |
| [`docs/51-opensource-plan.md`](docs/51-opensource-plan.md) | 项目现状与开源路线 |
| [`docs/48-sim2real-gaps.md`](docs/48-sim2real-gaps.md) | 已知的 sim-to-real 差距（G1 限位、G2 驱动器延迟） |

---

## 2 · 开发环境

### 仅运行软件测试（无需硬件，无需 Isaac Gym）

```bash
cd pi
./run_checks.sh
```

依赖 Python 3.9 或以上、`numpy`、`pyserial`。CI 执行的是同一套测试。

### 固件主机单测（无需硬件）

```powershell
cd firmware
tools\test-host.ps1
```

七套用例，共 1370 项断言。需要 `gcc`（Windows 下可用 MinGW 或 MSYS2，或 WSL）。

### 编译固件

- **STM32**：使用 Keil MDK 打开 `firmware/stm32/MDK-ARM/stm32.uvprojx`
- **ESP32**：在 `firmware/esp32/` 下使用 PlatformIO

### 训练

需自行向 NVIDIA 获取 Isaac Gym（专有许可，本仓库不包含）：

```bash
export RLCAT_ROOT=$PWD/Sim/RL
export RLCAT_CONDA=~/miniconda
source Sim/RL/scripts/env.sh
bash Sim/RL/scripts/run_train.sh rlcat --num_envs 2048
```

---

## 3 · 提交前必须通过的检查

```bash
# ① 上位机测试
cd pi && ./run_checks.sh

# ② 若修改了固件
cd firmware && tools/test-host.ps1

# ③ 若修改了观测或动作相关代码
cd pi && python3 rl_policy.py --selftest     # 必须 max|diff| = 0
```

第 ③ 项为强制要求。47 维观测布局或关节顺序一旦被改坏，动作将被下发到错误的关节，
导致机器人上电即失控，而其余单元测试无法发现该类问题。

---

## 4 · 代码与提交规范

### 提交信息应说明动机

```
推荐：修正 q_range 重复应用 dir 的问题 —— dir = -1 的六个关节限位被镜像
不推荐：fix bug
```

本项目的多数问题都属于"为何如此"一类（参见 `docs/49`）。仅描述"改了什么"的提交信息，
在数月后将失去参考价值。

### 一项逻辑改动对应一次提交

这对应项目的一条既有原则：单次只改变一个变量，否则问题无法定位。

### 注释说明反直觉之处

```c
/* 此处必须应用 dir，否则…… —— 2026-10-18 实测：
   现象为 xxx，根因为 yyy，因此 zzz 不可修改。 */
```

不要求注释解释代码"做什么"（代码本身应当清晰），要求注释解释"为何不能那样写"。

---

## 5 · 禁止事项

| 禁止 | 原因 |
|---|---|
| 提交含个人绝对路径的文件（`/home/<user>/`、`C:\Users\<user>\`） | 会使其他使用者无法直接运行项目 |
| 提交超过 5 MB 的文件 | 应作为 Release 附件发布 |
| 提交私钥、口令或 `reference/` 下的厂商资料 | 安全与再分发许可 |
| 在 50 Hz 及以上的实时循环中使用 `print`、文件 I/O 或网络 I/O | 阻塞式输出会导致不发帧、链路失效、退出 `ACTIVE`。参见 `docs/49` §4-A2 |
| 在缺少量化数据的情况下宣称某项功能已实现 | 本项目的结论均以实测数据表述 |
| 跨 episode 长度比较平均奖励 | 已有反例：某一轮奖励最高，但 episode 长度仅 891，实际处于摔倒状态 |

---

## 6 · 硬件相关贡献

机械与电路改动难以通过 CI 验证，请在 PR 中提供：

1. 修改内容与动机（在装配或使用中遇到了什么问题）
2. 照片
3. 若修改了 `URDF`，请同步检查 `docs/48-sim2real-gaps.md` 中的 G1（thigh 限位 90° 与 30°）
4. 若修改了关节限位或 `dir`，**必须**使用 `pi/pose.py` 逐关节验证，并更新
   `docs/30-firmware.md` §3.2

---

## 7 · 提交 Issue 前

- 先查阅 `docs/49` 中的 25 项工程问题
- 先检索现有 issue
- 提供完整命令与完整报错信息，而非仅最后一行的错误摘要

---

## 8 · 许可

提交 PR 即表示同意你的贡献按本项目的三层许可发布
（代码 Apache-2.0 / 硬件 CERN-OHL-W-2.0 / 文档 CC-BY-4.0），详见 `NOTICE.md`。
