# RL-CAT

> **一只 12 自由度四足机器人**，目标是用强化学习训出可 sim-to-real 的行走策略。
>
> ⚠️ **项目进行中** —— 当前状态见下方「进展」。

---

## 进展

**最后更新：2026-10-12**

| 模块 | 状态 | 说明 |
|---|---|---|
| **机械设计** | ✅ v1 完成 | 3D 打印结构，12 个总线舵机 |
| **电子设计** | ✅ 完成 | 主控 + 电源 + Pi 5 背板 |
| **URDF** | ✅ 完成 | 质量/惯量按 SolidWorks 实测修正 |
| **仿真环境** | ✅ 完成 | Isaac Gym + legged_gym（`rlcat` task） |
| **RL 训练** | ✅ **已训出可行走策略** | 对角小跑、速度跟踪误差 < 0.1 m/s |
| **STM32 固件** | ✅ **已交付** | 7966 行 / 7 层 / 1028 C 单测 + 161 Python 全绿；**真机验收进行中** |
| **舵机标定** | ✅ **已完成（2026-10-16）** | 12 关节 offset/dir 全部实测确认；零位与限位语义见 [`docs/30-firmware.md`](docs/30-firmware.md) §3.3.1 |
| **ESP32 无线桥** | ✅ **已交付并上真机验证** | 130 主机测试全绿；TX 链路 **300/300 帧零 CRC 错**；A4~A23 待执行 |
| **树莓派侧** | ✅ **基础层已交付** | 链路/遥测/标定/点动/姿态五阶段；离线 117 项 + 物理层 90 万字节零误码 |
| **真机部署** | 🟡 **进行中** | 见下方「当前卡在哪」 |

### ⚠️ 当前卡在哪

**舵机标定已过关。当前阻塞项：**

| # | 阻塞项 | 位置 |
|---|---|---|
| **1** | ⚠️ **IMU 读不到** —— `safety_reasons = 0x40`（`RLCA_SAFE_R_IMU_STALE`）。寄存器配置全部正确（`IICADDR=0x50`、`RRATE=9`、量程匹配），加速度反算 \|a\|≈1g 也对；怀疑 `imu_poll()` 的陀螺离群值保护在反复丢帧。**它挡住 ACTIVE 准入门第 ④ 条。** | `K` / `I` / `M` |
| **2** | **12 个舵机逐个设 ID** —— 出厂全是 1，直接接 12 个会总线冲突 | [`docs/42-pi-setup.md`](docs/42-pi-setup.md) §D4 |
| **3** | **电池 ADC 标定** —— 万用表比对，目标误差 < 50 mV | 同上 |
| **4** | **舵机侧软限位** —— 在上位机设 `Min/Max Angle Limit`（留 10~15% 余量，勿动 `OFS`）。**这是独立于固件的最后一道保险** | 上位机工具 |


**然后按顺序跑：**

```bash
cd ~/RL-CAT/pi
python3 pi_hello.py --tilt           # 掰腿 / 倾斜 / 万用表比对
python3 status_panel.py              # 12 路温度、状态位
python3 acceptance.py                # 一键验收（只读，安全）
python3 acceptance.py --move --yes   # ⚠️ 吊起后跑完整验收（含 12 关节点动）
```

---

## 我是谁 → 该读什么

| 你的角色 | 入口 | 说明 |
|---|---|---|
| **STM32 固件** | [`docs/30-firmware.md`](docs/30-firmware.md) | ⭐ 五张关键表 + 4 份实现规格 |
| **固件调试命令** | [`docs/41-firmware-debug-commands.md`](docs/41-firmware-debug-commands.md) | ⭐ `K` / `W` / `T` 的用法与铁律 |
| **ESP32 固件** | [`docs/36-esp32-plan.md`](docs/36-esp32-plan.md) → [`firmware/esp32/README.md`](firmware/esp32/README.md) | ⭐ 无线桥：实施方案 + 环境坑 + USART3 契约 |
| **树莓派侧** | [`docs/42-pi-setup.md`](docs/42-pi-setup.md) → [`pi/使用说明.md`](pi/使用说明.md) | ⭐ 环境配置 + **操作手册** |
| **机械** | [`docs/10-mechanical.md`](docs/10-mechanical.md) → [`hardware/mechanical/`](hardware/mechanical/) | |
| **电子** | [`docs/20-electronics.md`](docs/20-electronics.md) → [`hardware/electronics/`](hardware/electronics/) | |
| **仿真 / RL** | [`docs/40-simulation.md`](docs/40-simulation.md) → [`Sim/`](Sim/) | |
| **第一次了解项目** | [`docs/00-overview.md`](docs/00-overview.md) | 项目全貌 + 路线图 |
| **想知道当前进度** | 本页「进展」+ [`docs/42-pi-setup.md`](docs/42-pi-setup.md) §D | ⭐ 最新状态与勘误 |
| **想理解"为什么这样设计"** | [`docs/90-decisions.md`](docs/90-decisions.md) | 设计决策记录 |
| **遇到看不懂的名词** | [`docs/99-glossary.md`](docs/99-glossary.md) | ⭐ 术语表（含命名歧义澄清） |

---

## 目录结构

```
RL-CAT/
├── docs/                    项目文档
│   ├── 00-overview.md         项目总览与路线图
│   ├── 10-mechanical.md       机械设计
│   ├── 20-electronics.md      电子设计
│   ├── 30-firmware.md         ⭐ 固件索引（五张关键表）
│   ├── 40-simulation.md       仿真与训练
│   ├── 50-deployment.md       部署与 sim-to-real
│   ├── 90-decisions.md        设计决策记录
│   └── 99-glossary.md         术语表
│
├── hardware/
│   ├── mechanical/           机械 CAD（SolidWorks）
│   └── electronics/          电子设计（嘉立创 EDA）
│
├── firmware/                 STM32 固件 + ESP32 无线桥
│   ├── stm32/                  CubeMX + Keil 工程
│   ├── esp32/                  ESP32-C3 无线桥（ESP-IDF / PlatformIO）
│   ├── tools/                  构建与主机侧测试脚本（pio.ps1 / test_esp32_host.ps1）
│   └── doc/                    固件实现规格（4 份 + README）
│
├── pi/                       树莓派侧（基础层：链路/遥测/标定/点动/姿态）
│   ├── rlcat_proto.py          ⚠️ 固件交付方给的，两端共用协议实现（勿改）
│   ├── link.py / telemetry.py  串口链路层 + 遥测语义层
│   ├── pi_hello.py             阶段 1：链路观察
│   ├── status_panel.py         阶段 2：遥测面板
│   ├── calib.py                阶段 3：标定
│   ├── jog.py                  阶段 4：单关节点动
│   ├── pose.py                 阶段 5：全身姿态
│   ├── acceptance.py           ⭐ 一键验收（默认只读）
│   ├── 使用说明.md              ⭐ **操作手册**（在机器人旁边看这份）
│   ├── README.md               开发记录（设计取舍、坑、测试覆盖）
│   ├── tools/                  假 STM32（PTY 替身）、UART 误码实测
│   └── tests/                  离线端到端测试 117 项
│
├── Sim/                      仿真与 RL 训练
│   ├── RL/                    训练脚本、URDF、日志、视频、策略导出
│   ├── legged_gym/            legged_gym（含 rlcat task）
│   └── rsl_rl/                rsl_rl v1.0.2
│
├── reference/                第三方参考资料
│   ├── servo/                  舵机规格书 + 官方库
│   ├── stm32/                  STM32 参考手册
│   ├── JY901P_I2C协议.md        IMU 寄存器手册
│   └── dimension-drawings/     尺寸图
│
└── archive/                  ⚠️ 已退役文档（含已知错误，勿作设计依据）
```

---

## 关键特性

| 项 | 值 |
|---|---|
| **自由度** | 12（每腿 3：髋外摆 + 髋俯仰 + 膝） |
| **整机质量** | 1.386 kg（CAD 实测，含 12 舵机 + Pi 5 + 电池 + PCB） |
| **机身尺寸** | 200 × 72 × 58 mm |
| **站立高度** | 约 133 mm（仿真实测） |
| **执行器** | 12 × Feetech **ST-3215-C018**（12V 版）总线舵机 |
| **算力** | 树莓派 5 (4GB) 跑策略 + STM32F103C8 做 100 Hz 实时控制 |
| **通信** | Pi ↔ STM32：1 Mbps 自定义帧协议；STM32 ↔ 舵机：1 Mbps 总线 |
| **控制频率** | 100 Hz |
| **观测 / 动作** | 47 维 / 12 维 |

---

## 快速开始

**⚠️ 四个子系统的上手方式不同，按需选择：**

### 树莓派侧（⭐ 当前进度在这里）

```
1. 读 docs/42-pi-setup.md（环境配置 + 负责人待办 + §D 实现状态与勘误）
2. 按 pi/使用说明.md 操作（在机器人旁边看那份）
3. 先跑不需要硬件的检查：
     cd pi && ./run_checks.sh
4. 再用假 STM32 手工联调（不需要真板子）：
     python3 tools/fake_stm32.py --pty-file /tmp/fake.pty --calibrated -v
     python3 pi_hello.py --port "$(cat /tmp/fake.pty)" --seconds 5 --keepalive
```

### STM32 固件

```
1. 读 docs/30-firmware.md（五张关键表）
2. 读 firmware/doc/CubeMX配置清单.md §7（待改配置）
3. 用 CubeMX 打开 firmware/stm32/stm32.ioc
4. 用 Keil 打开 firmware/stm32/MDK-ARM/stm32.uvprojx
5. 上机调试看 docs/41-firmware-debug-commands.md
```

### ESP32 无线桥

```
1. 读 docs/36-esp32-plan.md → firmware/esp32/README.md
2. 构建：firmware/tools/pio.ps1
3. 上机验收：docs/37-esp32-acceptance.md
```

### 仿真 / RL

```
1. 读 docs/40-simulation.md
2. 环境搭建见 Sim/RL/RL-CAT-训练实施报告.md §2
3. 训练入口：Sim/RL/scripts/run_train.sh
4. 观察效果：Sim/RL/scripts/12_play.py
```

**⚠️ 训练代码（`legged_gym/legged_gym/envs/rlcat/`）在 `Sim/legged_gym/` 下。**

### 机械 / 电子

```
机械：hardware/mechanical/（SolidWorks 装配体 + 零件）
电子：hardware/electronics/RL-CAT.eprj2（嘉立创 EDA）
```

**⚠️ 打开 SolidWorks 装配体前，确保整个 `hardware/mechanical/` 目录结构完整** —— 装配体用相对路径引用零件。

---

## 仿真成果

**已训出可稳定行走的策略**（Isaac Gym，RTX 4060 Laptop）：

| 验收项 | 结果 |
|---|---|
| 步态结构 | ✅ 标准对角小跑（非蹦跳/跛行） |
| 左右对称 | ✅ 占空比差 0.08~0.14 |
| 速度跟踪误差 | ✅ **0.060 ~ 0.099 m/s**（−0.3 ~ +0.5 m/s 全段） |
| 稳定性 | ✅ 32 环境 × 15 s 零摔倒 |

**演示：** [`Sim/RL/videos/`](Sim/RL/videos/)（4 段，含后退）
**报告：** [`Sim/RL/RL-CAT-训练实施报告.md`](Sim/RL/RL-CAT-训练实施报告.md)（含 9 轮调参的完整记录与失败分析）

---

## ⚠️ 重要提醒

| # | 提醒 |
|---|---|
| 1 | **`archive/` 里的文档含已被证伪的结论**，不要作为设计依据。详见 [`archive/README.md`](archive/README.md) |
| 2 | **`thigh` 在代码里指"髋俯仰关节"，在机械里指"大腿零件"** —— 容易混，见 [`docs/99-glossary.md`](docs/99-glossary.md) §0 |
| 3 | **关节限位有两个 URDF 不一致**（thigh: 30° vs 90°），固件用 90°，见 [`docs/30-firmware.md`](docs/30-firmware.md) §3.1 |
| 4 | ⚠️⚠️ **`MODE_STAND` 没有任何准入门** —— 未标定时发 STAND 会让 12 个舵机全部冲向绝对位置 0（**撞机**）。树莓派侧已做保护（未标定不发 STAND/ACTIVE），见 [`docs/42-pi-setup.md`](docs/42-pi-setup.md) §D2 第 8 条 |
| 5 | ⚠️ **固件行为与文档有 19 处偏差**（`TEL_STATUS` 电压恒 0、温度单位差 10 倍、USART1 shell 被抽干等）—— 逐条清单见 [`pi/README.md`](pi/README.md) §8 |
| 6 | ⚠️ **`TEL_STATUS.voltage_mv` 固件写死为 0** —— 真电压只在 `TEL_FAST` 里 |
| 7 | **`Sim/` 目录暂未整理**，路径可能变动 |

---

## 文档状态

**最后更新：2026-10-12**

| 文档 | 状态 |
|---|---|
| `30-firmware.md` | ✅ 已填实（五张关键表） |
| `31-pi-runtime.md` | ✅ 已填实 |
| `32-esp32-bridge.md` | ✅ 已填实 |
| `33-firmware-plan.md` | ✅ 已填实 |
| `34-firmware-acceptance.md` | 🟡 验收进行中（P5-1/P5-2 已通过） |
| `35-firmware-deployment.md` | ✅ 已填实 |
| `36-esp32-plan.md` | ✅ 已填实 |
| `37-esp32-acceptance.md` | 🟡 A1~A3 通过，A4~A23 待执行 |
| `38-esp32-handover.md` | ✅ 已填实 |
| `39-firmware-handover.md` | ✅ 已填实 |
| `41-firmware-debug-commands.md` | ✅ 已填实 |
| `42-pi-setup.md` | ✅ 已填实 + ⭐ §D 实现状态与勘误 |
| `99-glossary.md` | ✅ 已填实 |
| `00-overview.md` | 🟡 骨架 + 部分内容 |
| `90-decisions.md` | 🟡 骨架 + 部分内容 |
| `10-mechanical.md` | ⬜ 待重写（旧内容已失效） |
| `20-electronics.md` | 🟡 部分内容 |
| `40-simulation.md` | 🟡 部分内容 |
| `50-deployment.md` | ⬜ 骨架 |

---

## 许可

**⚠️ 尚未确定。** 项目计划开源，协议待定（见 [`docs/90-decisions.md`](docs/90-decisions.md)）。

**注意：`hardware/mechanical/零件/` 中含第三方 CAD 模型**（Waveshare 控制板、Raspberry Pi 5、Feetech ST3215），开源前需确认授权或替换。
