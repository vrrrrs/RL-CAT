# RL-CAT 下位机固件文档

> **给接手固件开发的 Agent 的入口文档**
> 版本：v1.0 ｜ 日期：2026-10-09

---

## 文档清单

| # | 文档 | 内容 | 什么时候读 |
|---|---|---|---|
| **01** | [固件设计书.md](固件设计书.md) | 架构、模块划分、时序、标定、安全、开发阶段 | **先读这个** |
| **02** | [协议规范_树莓派通信.md](协议规范_树莓派通信.md) | 帧格式、消息定义、CRC、接收状态机、时序 | 实现 `link_pi` 前必读 |
| **03** | [伺服总线模块设计.md](伺服总线模块设计.md) | STS 协议、官方库缺陷、DMA+IDLE 方案、验收测试 | 实现 `servo_bus` 前必读 |
| **04** | [CubeMX配置清单.md](CubeMX配置清单.md) | 配置核对、待改项、上电自检 | **开工第一步** |

---

## 建议的推进顺序

```
第 0 步  读文档 01（建立整体认识）
   ↓
第 1 步  按文档 04 修 CubeMX 配置 + 跑通上电自检
   ↓
第 2 步  按文档 03 实现 servo_bus（最难，留足时间）
   ↓
第 3 步  按文档 02 实现 link_pi（可以用 PC 端 Python 模拟树莓派）
   ↓
第 4 步  读文档 01 §6 实现关节标定
   ↓
第 5 步  实现 app_mode + app_safety
   ↓
第 6 步  与树莓派联调
```

**⚠️ 第 1 步不做完，后面每一步都会踩坑（时钟不对 → 时序全错；中断没配对 → 丢数据难查）。**

---

## 开工前必须确认的 4 件事

| # | 事项 | 在哪里 |
|---|---|---|
| 1 | **舵机 ID 分配方案**（建议 1~12，见设计书 §3.3） | ⚠️ 需人确认 |
| 2 | **HSE 晶振真的是 8 MHz** | 看板子丝印或示波器量 |
| 3 | **舵机控制板：跳线在 A 位；接线是【直连】RX→RX、TX→TX** | 看硬件，见设计书 §2.3 |
| 4 | **PB8/PB9 的外部 4.7 kΩ 上拉已装** | 看 PCB |

---

## 三个最重要的技术要点（容易搞错）

### 1. ⚠️ URDF 关节顺序 ≠ 固件逻辑索引

```
固件用【逻辑索引】（髋外摆×4 → 髋俯仰×4 → 膝×4）
URDF 用【按腿分组】（lf_hip, lf_thigh, lf_calf, lb_hip, ...）
```

**⚠️ 协议里传输的 12 维向量一律用逻辑索引，见设计书 §3.2。**

### 2. ⚠️ `SC.c` 和 `SCSerail.c` 是两个不同的文件

```
SC.c        = 协议层（组帧、校验、syncReadBegin）
SCSerail.c  = 串口底层（readSC、IOTimeOut、SCDelay）  ← 拼写错误，实为 SCSerial
```

**搜超时相关代码去 `SCSerail.c`。**

### 3. ⚠️ 官方 `uart.c` 必须整体替换

```
原因①：基于 StdPeriph 库，与 CubeMX 的 HAL 冲突
原因②：自带 USART1_IRQHandler / USART2_IRQHandler，与 stm32f1xx_it.c 重复定义
```

**确切签名见伺服总线模块设计 §7.2。**

---

## 相关文档（本项目其他部分）

**⚠️ 路径以项目根目录 `RL-CAT/` 为基准。**

| 文档 | 位置 | 内容 |
|---|---|---|
| **固件索引** | [`docs/30-firmware.md`](../../docs/30-firmware.md) | ⭐ **关节表/限位/引脚/观测/参数，五张最常查的表** |
| 电子设计（电源/通信） | [`docs/20-electronics.md`](../../docs/20-electronics.md) | 电源架构、通信架构、引脚分配 |
| 项目总览 | [`docs/00-overview.md`](../../docs/00-overview.md) | 项目全貌、路线图 |
| **URDF** | [`hardware/mechanical/URDF/robot.urdf`](../../hardware/mechanical/URDF/robot.urdf) | 关节定义、限位、质量属性（权威值） |
| 官方舵机库 | [`reference/servo/STServoBCL_keil_f103/`](../../reference/servo/STServoBCL_keil_f103/) | 待移植的原始库 |
| 舵机规格书 | [`reference/servo/ST-3215-C018.pdf`](../../reference/servo/ST-3215-C018.pdf) | 电气参数 |
| STM32 参考手册 | [`reference/stm32/STM32f103.pdf`](../../reference/stm32/STM32f103.pdf) | 寄存器细节 |
| 仿真与训练 | [`docs/40-simulation.md`](../../docs/40-simulation.md) | Isaac Gym、RL 流程 |

**⚠️ 已归档的旧文档（`archive/`）不要作为设计依据。** [`archive/README.md`](../../archive/README.md) 列出了每份的退役原因与已知错误。

---

## 工程现状

```
firmware/stm32/
├── Core/
│   ├── Inc/          main.h, stm32f1xx_hal_conf.h, stm32f1xx_it.h
│   └── Src/          main.c（CubeMX 骨架，while(1) 是空的）
├── Drivers/
│   ├── CMSIS/        ⚠️ 含未使用的 DSP/NN 目录（约 46 MB），建议删
│   └── STM32F1xx_HAL_Driver/
├── MDK-ARM/          Keil 工程（纯相对路径引用，可安全移动）
└── stm32.ioc         CubeMX 配置
```

**⚠️ 目前只有 CubeMX 生成的骨架，业务代码要全部自己写。** 详见设计书 §4.2 的目录结构建议。

**✅ 配置已修正**（时钟 72 MHz、三路 UART 中断、1 Mbps 波特率），待补项见 [CubeMX配置清单.md](CubeMX配置清单.md) §7。

---

## 联系与协作

**遇到设计层面的疑问（协议要不要改、架构怎么定），先不要自己决定 —— 回传问题。**

**可以实现层面自行决定的事项：**

```
✅ 变量命名、函数内部实现、日志格式
✅ 缓冲区大小的微调（在预算内）
✅ 调试辅助代码（用 #ifdef 包起来）
✅ 单元测试的写法
```

**需要先确认再动的事项：**

```
❌ 协议帧格式的任何改动
❌ 关节索引顺序的改动
❌ 关节限位数值的改动
❌ 引脚分配的改动
❌ 舵机 ID 分配方案
```

**⚠️ 关于关节限位（重要澄清）：**

**旧交接文档曾说「`*_thigh` 的 +30° 上限小于站立所需的 +31.19°，站立姿态超出限位」—— 那个结论已被仿真实测证伪。**

```
实际：q = 0 就是装配好的站立姿态，站立时 thigh 关节角 = 0，离 +30° 还有 30° 余量。
      仿真中机器人以原始 URDF 成功站立并行走。
      `*_thigh` 放宽到 90° 是为了给摆腿留空间，不是修复"站立够不到"。

详见：archive/README.md 勘误 1
```

**👉 固件侧的关节限位以 [docs/30-firmware.md](../../docs/30-firmware.md) §2 为准。**

---

**文档结束。开工前请先读文档 01 和 04。**
