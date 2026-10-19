# 文档索引（docs/）

> **项目文档总目录。** 项目级入口见 [根 README](../README.md)。

---

## 文档清单

| 文档 | 内容 | 状态 |
|---|---|---|
| [`00-overview.md`](00-overview.md) | 项目总览、系统架构、技术路线、进展 | 🟡 部分内容 |
| [`10-mechanical.md`](10-mechanical.md) | 机械设计：设计约束、坐标系、关节定义、设计理由 | ⬜ 骨架（待机械定稿） |
| [`20-electronics.md`](20-electronics.md) | 电子设计：电源架构、通信、IMU、PCB | 🟡 部分内容 |
| [`30-firmware.md`](30-firmware.md) | ⭐ **固件索引 + 五张关键表** | ✅ **已填实** |
| [`31-pi-runtime.md`](31-pi-runtime.md) | ⭐ 树莓派侧程序：ONNX 推理、观测组装、实时性、安全 + **§10 实现状态** | 🟡 **基础层已交付**（链路/遥测/标定/点动/姿态）；RL 部分未做 |
| [`32-esp32-bridge.md`](32-esp32-bridge.md) | ⭐ ESP32-C3 固件：无线调试、指令仲裁、硬急停 | ✅ **已填实** |
| [`33-firmware-plan.md`](33-firmware-plan.md) | ⭐ **STM32 固件施工方案**：取舍、时序、验证策略、分阶段 | ✅ **P0~P6 已实施** |
| [`34-firmware-acceptance.md`](34-firmware-acceptance.md) | ⭐ **固件上机验收单**（P2~P6 共 42 项，含现象对照表）| 🟡 **已通过 2 项**（P5-1/P5-2）|
| [`35-firmware-deployment.md`](35-firmware-deployment.md) | ⭐ **联调与部署手册**：接口契约、调试顺序、故障决策树 | ✅ **已填实** |
| [`36-esp32-plan.md`](36-esp32-plan.md) | ⭐ **ESP32 固件实施方案**：阶段、环境坑、USART3 接口契约与现状核对 | ✅ **已填实** |
| [`37-esp32-acceptance.md`](37-esp32-acceptance.md) | ⭐ **ESP32 上机验收单**（A1~A23 + 现象对照表） | 🟡 **A1~A3 通过**；A4~A23 待执行 |
| [`38-esp32-handover.md`](38-esp32-handover.md) | ⭐⭐ **ESP32 固件交付总结**：「做了什么/验证到什么程度/还差什么」+ 12 条排查清单 | ✅ **已填实** |
| [`39-firmware-handover.md`](39-firmware-handover.md) | ⭐⭐ **STM32 固件交付总结**：「做了什么/做到什么程度/还差什么」+ 24 个缺陷清单 | ✅ **已填实** |
| [`41-firmware-debug-commands.md`](41-firmware-debug-commands.md) | ⭐ **STM32 调试命令手册**：每条命令的用法与判据、无线调试（ESP32 透传）、8 个已知的坑 | ✅ **已填实** |
| [`42-pi-setup.md`](42-pi-setup.md) | ⭐⭐ **树莓派开发环境与要求**：网络/UART/接线配置、分阶段任务、**负责人待办清单** | ✅ **已填实** + ⭐ **§D 实现状态与勘误**（2026-10-12） |
| [`40-simulation.md`](40-simulation.md) | 仿真与 RL：环境、资产改造、训练历程、脚本 | 🟡 部分内容 |
| [`45-linux-tasks.md`](45-linux-tasks.md) | 必须在 Linux 上完成的任务（策略导出/验证） | ✅ **已填实** |
| [`46-mainline-plan.md`](46-mainline-plan.md) | 主线方案：M1 地面站立 / M2 脚本步态 / M3 sim2real | ✅ **已填实** |
| [`47-policy-export-report.md`](47-policy-export-report.md) | 策略导出交付报告（检查点选择、两个 bug、仿真验证） | ✅ **已填实** |
| [`48-sim2real-gaps.md`](48-sim2real-gaps.md) | 🔴 **sim2real 缺口清单**：关节限位、驱动器延迟、伺服 PD | ✅ **已填实** |
| [`49-progress-review.md`](49-progress-review.md) | ⭐⭐⭐ **进度复盘与踩坑总账**：时间线、完成情况、**25 个坑的根因与教训**、方法论、下一步 | ✅ **已填实**（2026-10-18） |

**⚠️ 树莓派侧另有两份文档不在 `docs/` 下：**

| 文档 | 位置 | 内容 |
|---|---|---|
| ⭐⭐ 操作手册 | [`../pi/使用说明.md`](../pi/使用说明.md) | **在机器人旁边看这份** —— 怎么跑、看到什么算对、出问题查哪里 |
| ⭐ 开发记录 | [`../pi/README.md`](../pi/README.md) | 设计取舍、踩过的坑、**固件行为清单（19 条）**、测试覆盖 |
| [`50-deployment.md`](50-deployment.md) | 部署与 sim-to-real：链路、安全、系统辨识 | ⬜ 骨架 |
| [`90-decisions.md`](90-decisions.md) | 设计决策记录（15 条 + 待补） | 🟡 部分内容 |
| [`99-glossary.md`](99-glossary.md) | ⭐ **术语表 + 命名歧义澄清** | ✅ **已填实** |

---

## 按角色的阅读路径

### 固件开发者

```
1. docs/30-firmware.md              ← ⭐ 五张关键表 + 4 份实现规格的索引
2. docs/99-glossary.md §0           ← 关节命名歧义（thigh/hip/calf）
3. firmware/doc/CubeMX配置清单.md §7 ← 开工第一步
4. firmware/doc/固件设计书.md        ← 架构
5. firmware/doc/伺服总线模块设计.md   ← 最难的部分
6. firmware/doc/协议规范_树莓派通信.md ← 通信
```

### 树莓派开发者

```
1. pi/使用说明.md                    ← ⭐⭐ 操作手册（怎么跑、什么算对、出问题查哪里）
2. docs/42-pi-setup.md              ← 环境/接线/分工；⚠️ 直接看 §D「实现状态与勘误」
3. docs/31-pi-runtime.md            ← 技术设计；⚠️ §10「实现状态」列出了做了 / 没做的
4. docs/30-firmware.md §5           ← 观测/动作定义
                                      ⚠️ 与训练代码有若干不一致 → pi/README.md 附录 A
5. pi/README.md                     ← 开发记录：19 条固件行为清单、测试覆盖
```

### 仿真 / RL

```
1. docs/40-simulation.md
2. Sim/RL/RL-CAT-训练实施报告.md      ← ⭐ 最权威的技术记录
3. docs/30-firmware.md §5            ← 观测/动作布局（与固件的接口）
```

### 机械

```
1. docs/10-mechanical.md             ← 设计约束与坐标系（⚠️ 具体尺寸见 参数.xlsx）
2. hardware/mechanical/参数.xlsx      ← ⭐ 单一数据源
```

### 电子

```
1. docs/20-electronics.md
2. hardware/electronics/RL-CAT.eprj2
```

### 想理解"为什么这样设计"

```
1. docs/90-decisions.md              ← 15 条决策记录
2. archive/方案_v1.md                ← ⚠️ 历史推理过程，含过时内容
```

---

## ⚠️ 三个必读提醒

| # | 提醒 | 详见 |
|---|---|---|
| **1** | **`archive/` 里的文档含已被证伪的结论**，不要作为设计依据 | [`archive/README.md`](../archive/README.md) |
| **2** | **`thigh` 在代码里指"髋俯仰关节"，在机械里指"大腿零件"** | [`99-glossary.md`](99-glossary.md) §0 |
| **3** | **关节限位有两个 URDF 不一致**（thigh: 30° vs 90°），固件用 90° | [`30-firmware.md`](30-firmware.md) §3.1 |

---

## 文档维护约定

### 写法

| 原则 | 说明 |
|---|---|
| **只写不变量** | 写设计意图/约束/约定；**不写会变的数值**（尺寸指向 `参数.xlsx`） |
| **写清"为什么"** | 决策的理由进 `90-decisions.md`，正文只引用 |
| **过时就归档** | 不删，移到 `archive/` 并注明退役原因 + 替代文档 |
| **术语统一** | 用 `99-glossary.md` 的"统一用"列 |

### 路径引用

**⚠️ 文档里的相对路径一律以【项目根目录 `RL-CAT/`】为基准描述**，`docs/` 内部互相引用用相对路径。

例：

```
✅ 位置：`firmware/stm32/stm32.ioc`          （从项目根描述）
✅ 链接：[固件设计书](../firmware/doc/固件设计书.md) （Markdown 相对链接）
```

### 状态标记

| 标记 | 含义 |
|---|---|
| ✅ 已填实 | 内容完整可用 |
| 🟡 部分内容 | 骨架 + 关键内容，待补 |
| ⬜ 骨架 | 只有结构，内容待写 |

---

**文档索引结束。**
