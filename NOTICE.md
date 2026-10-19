# NOTICE · 第三方组件与许可声明

> **本文件是 Apache-2.0 §4(d) 所要求的 NOTICE 文件。**
> 分发本项目的衍生作品时，必须一并包含本文件（或其内容）。

**RL-CAT** — 12 自由度四足机器人
Copyright 2026 vrrrrs
Licensed under the Apache License, Version 2.0（见 `LICENSE`）

---

## 1 · 三层许可

| 范围 | 许可 | 文件 |
|---|---|---|
| **软件**（固件、树莓派侧、训练脚本、工具） | **Apache-2.0** | `LICENSE` |
| **硬件**（机械 STEP/STL、电路、URDF） | **CERN-OHL-W-2.0** | `LICENSE-hardware` |
| **文档**（`docs/`、`*.md` 说明） | **CC-BY-4.0** | `LICENSE-docs` |

---

## 2 · 本项目包含的第三方代码

### 2.1 legged_gym（`Sim/legged_gym/`）

- **来源**：https://github.com/leggedrobotics/legged_gym
- **许可**：见 `Sim/legged_gym/LICENSE`（BSD-3-Clause 系）
- **状态**：**本仓库是它的衍生作品**（新增了 `legged_gym/envs/rlcat/` 下的机器人配置与环境）
- **义务**：保留原 `LICENSE` 与所有源文件的版权头；标注哪些文件被修改（见其 `git` 历史与 `docs/40-simulation.md`）

### 2.2 rsl_rl（`Sim/rsl_rl/`）

- **来源**：https://github.com/leggedrobotics/rsl_rl
- **许可**：见 `Sim/rsl_rl/` 内的许可声明
- **状态**：同样为衍生作品（本项目对 PPO 的实现无实质修改）

### 2.3 第三方资产许可（已附带原文）

- `Sim/legged_gym/licenses/assets/*.txt` —— ANYmal B/C、a1、Cassie 的资产许可
- `Sim/legged_gym/licenses/dependencies/*.txt` —— matplotlib 等依赖许可
- `Sim/rsl_rl/licenses/dependencies/*.txt` —— numpy、torch 许可

> ⚠️ **本仓库不包含**上述资产中的机器人网格文件（`resources/robots/{a1,anymal_b,anymal_c,cassie}` 已在 `.gitignore` 中排除）。
> 需要它们时请自行从上游获取，并遵守各自的许可。

---

## 3 · ⚠️ 本项目**不包含**、需要你自行获取的组件

| 组件 | 为什么不在仓库里 | 怎么获取 |
|---|---|---|
| **NVIDIA Isaac Gym** | 🔴 **专有许可（EULA），禁止再分发**。本项目只在代码里 `import isaacgym` | 向 NVIDIA 申请：https://developer.nvidia.com/isaac-gym |
| **ST STM32 HAL / CMSIS** | 体积大（65 MB），可由工具重新生成 | CubeMX 生成，或 ST 官方 GitHub |
| **WitMotion IMU SDK** | 🔴 厂商资料，不可再分发 | 厂商官网 |
| **Feetech 舵机资料** | 厂商资料 | 厂商官网 |

**⚠️ 因此：想复现训练，你必须自己申请 Isaac Gym。这是本项目的硬性前置条件，不是我们的疏漏。**

---

## 4 · 硬件组件的第三方权利

| 组件 | 说明 |
|---|---|
| **Feetech ST-3215-C018 舵机** | 仅作为采购件使用，其固件与协议文档版权归厂商 |
| **树莓派 5 / ESP32-C3** | 各自厂商 |
| **机械 CAD**（`hardware/mechanical/step/`、`stl/`） | **本项目原创**，按 CERN-OHL-W-2.0 授权 |
| **URDF**（`hardware/mechanical/URDF/robot.urdf`） | **本项目原创**，按 CERN-OHL-W-2.0 授权 |
| **零件外形 STL**（`Sim/**/meshes/`） | 本项目原创，按 CERN-OHL-W-2.0 授权 |
| ⚠️ **未发布**：SolidWorks 原生文件、厂商件 3D 模型、切片文件 | 见 `hardware/README.md` §2 |

---

## 5 · 商标

"RL-CAT" 是本项目的名称，**不附带商标授权**。Apache-2.0 §6 不授予使用本项目名称、标识的许可（描述性引用除外）。

---

## 6 · 免责声明（⚠️ 请注意）

本项目是**研究与教学用途**的机器人平台，涉及：

- 锂电池（可能起火）
- 12 个舵机（堵转时电流很大，可烫伤）
- 一个能自行移动的 1.5 kg 物体

**⇒ 按 Apache-2.0 §7/§8 提供，不附带任何担保。组装、上电、调试的风险由使用者承担。**
**⇒ 上电前请务必阅读 `docs/42-pi-setup.md` 与 `docs/46-mainline-plan.md` 里的安全说明。**
