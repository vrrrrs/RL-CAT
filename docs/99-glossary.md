# 术语表（Glossary）

> **文档编号**：DOC-GLOSSARY-001
> **版本**：v1.0
> **日期**：2026-10-09
>
> **本文件的目的：消除项目内的命名歧义。** 写代码、写文档、讨论时，**使用本表的"统一用"列**。

---

## 0. ⚠️ 三个最容易歧义的词

**这三个词在"机械"和"代码/URDF"里含义不同，是最常见的混乱来源。**

| 词 | 在**代码 / URDF** 里指 | 在**机械**里指 | 推荐做法 |
|---|---|---|---|
| **`hip`** | **髋外摆关节**（abduction） | 髋部零件/髋部结构 | 代码里用 `hip`，机械讨论时说"髋外摆关节" |
| **`thigh`** | **髋俯仰关节**（hip pitch） | **大腿零件** | ⚠️ **代码里继续用 `thigh`**（URDF 已锁定），但文档里必须写清是"关节"还是"零件" |
| **`calf`** | **膝关节** | **小腿零件** | 同上 |

**👉 最保险的写法：**

```
✅ "thigh 关节"（指关节）    /    "大腿零件"（指零件）
❌ "thigh"（单独出现，读者不知道指哪个）
```

---

## 1. 关节命名

### 1.1 腿的编号

| 统一用 | 全称 | 含义 | 不用 |
|---|---|---|---|
| **`lf`** | left front | 左前腿 | LF、左前、FL |
| **`lb`** | left back | 左后腿 | LB、左后、RL |
| **`rf`** | right front | 右前腿 | RF、右前、FR |
| **`rb`** | right back | 右后腿 | RB、右后、RR |

**⚠️ 不用 `FL`/`FR`/`RL`/`RR`** —— 因为本项目的 URDF 用的是 `lf`/`lb`/`rf`/`rb`，混用会造成映射错误。

### 1.2 关节类型

| 统一用 | 中文 | 英文全称 | 转轴（机身系） | 不用 |
|---|---|---|---|---|
| **`hip`** | 髋外摆 | hip abduction / hip roll | 沿机身 **X**（前后） | abduction、roll、外摆关节 |
| **`thigh`** | 髋俯仰 | hip pitch | 沿机身 **Y**（左右） | hip pitch、俯仰关节 |
| **`calf`** | 膝 | knee | 沿机身 **Y**（左右） | knee、膝关节 |

**⚠️ 关节全名写法：`{腿}_{关节}`，例如 `lf_hip`、`rb_thigh`、`rf_calf`。**

---

## 2. 关节索引体系

**⚠️ 项目里有三套"顺序"，见 [docs/30-firmware.md](30-firmware.md) §2.2。**

| 名称 | 定义 | 用在哪 |
|---|---|---|
| **逻辑索引**（A） | 髋外摆×4 → 髋俯仰×4 → 膝×4 | **固件数组、协议、观测/动作向量** |
| **URDF 声明顺序**（B） | 按腿分组（lf→lb→rf→rb） | URDF 文件 |
| **训练侧顺序**（C） | 按名字查表 | legged_gym（自动处理） |

**👉 除非特别说明，"索引 0–11"一律指【逻辑索引】。**

---

## 3. 仿真与 RL 术语

| 术语 | 本项目的含义 | 备注 |
|---|---|---|
| **`q`** | 关节角向量（12 维，rad） | |
| **`q_default`** | 默认关节角 = **全 0** | ✅ 已实测：`q=0` 就是站立姿态 |
| **`qd`** | 关节角速度向量（12 维） | |
| **`tau`** | 关节力矩向量（12 维） | |
| **`action`** | 策略输出（12 维，经 tanh 落在 [-1,1]） | |
| **`action_scale`** | **0.25** rad/单位 | `q_target = q_default + 0.25 × action` |
| **`decimation`** | **2** | sim dt=1/200 → 控制 100 Hz |
| **观测 / obs** | **47 维**向量 | ⚠️ 不是 50 维，见 30-firmware.md §5.1 |
| **方案 C** | 47 维观测方案 | 50 维设计的**前 47 维** |
| **步态相位** | `sin/cos` 对（2 维，索引 45–46） | 固件内部时钟产生，真机无需传感器 |
| **域随机化** | 训练时随机化物理参数 | 提升 sim-to-real 鲁棒性 |
| **sim-to-real** | 仿真到真机的迁移 | |

---

## 4. 硬件与协议术语

| 术语 | 含义 |
|---|---|
| **STS** | Feetech 的串行总线舵机协议（本项目用这个） |
| **SCS** | Feetech 的另一套协议（⚠️ 与本项目不同，官方库同时支持两者，注意别用错） |
| **SyncWrite** | 一条指令同时写多个舵机（`INST_SYNC_WRITE = 0x83`） |
| **SYNC_READ** | 一条请求读多个舵机（`INST_SYNC_READ = 0x82`） |
| **Present Position** | 舵机当前位置反馈（地址 56） |
| **Present Load** | 舵机当前负载反馈（地址 60） |
| **HDSEL** | STM32 USART 的单线半双工模式 ⚠️ **本项目不需要开** |
| **IDLE 中断** | USART 总线空闲检测，用于判断一帧结束 |
| **turnaround** | 半双工总线的收发方向切换时间 |
| **EPROM / SRAM 区** | 舵机寄存器的两个区域；改 EPROM 前要先写 `LOCK = 0` |
| **joint offset** | 关节角与舵机内部角之间的标定偏置 |

---

## 5. 项目代号与文件命名

| 代号 | 含义 |
|---|---|
| **RL-CAT** | 项目名（Reinforcement Learning quadruped，CAT 为代号） |
| **CAT_defeature** | URDF / SolidWorks 里的 robot name（历史遗留，来自建模时的名字） |
| **rlcat** | legged_gym 里的 task 名（`rlcat_flat`、`rlcat_stand`） |

**⚠️ `CAT_defeature` 这个 robot name 是历史遗留，出现在 URDF 里，不要改（改了会牵动多处引用）。**

---

## 6. 目录与路径约定

**⚠️ 项目根目录 = `RL-CAT/`。文档里的相对路径一律以此为准。**

| 路径 | 内容 |
|---|---|
| `docs/` | 项目文档 |
| `hardware/mechanical/` | 机械 CAD（SolidWorks） |
| `hardware/electronics/` | 电子设计（嘉立创 EDA） |
| `firmware/` | STM32 固件工程 + 固件文档 |
| `Sim/` | 仿真与 RL 训练（⚠️ 暂未整理） |
| `reference/` | 第三方参考资料（规格书、官方库、尺寸图） |
| `archive/` | ⚠️ **已退役文档，不要作为设计依据** |

---

## 7. 单位与符号约定

| 量 | 单位 | 说明 |
|---|---|---|
| 长度（URDF） | **米 (m)** | URDF 标准单位 |
| 长度（CAD / 文档） | **毫米 (mm)** | SolidWorks 用 mm，注意换算 |
| 角度（URDF limit） | **弧度 (rad)** | |
| 角度（文档表述） | **度 (°)** | 文档里给度，代码里用弧度 |
| 质量 | **千克 (kg)** | |
| 惯性张量 | **kg·m²**（URDF） / **kg·mm²**（SolidWorks） | ⚠️ 换算 ×1e-6 |
| 舵机位置 | **步 (step)**，0–4095 | 4096 步 = 360° |
| 力矩 | **N·m** | |
| 转速 | **rad/s**（代码） / **RPM**（规格书） | 45 RPM = 4.71 rad/s |

**⚠️ 常见换算陷阱：**

```
SolidWorks 惯性张量是 kg·mm²  →  URDF 要 kg·m²   →  × 1e-6
SolidWorks 长度是 mm          →  URDF 要 m        →  × 0.001
舵机步数 4096 步/圈           →  弧度            →  × (2π/4096)
```

---

## 8. 常用缩写

| 缩写 | 全称 | 含义 |
|---|---|---|
| **DOF** | Degree of Freedom | 自由度（本项目 12 个） |
| **CoM** | Center of Mass | 质心 |
| **IMU** | Inertial Measurement Unit | 惯性测量单元（本项目用 JY901P） |
| **PPO** | Proximal Policy Optimization | RL 算法 |
| **MLP** | Multi-Layer Perceptron | 多层感知机 |
| **BSP** | Board Support Package | 板级支持包（固件分层） |
| **CRC** | Cyclic Redundancy Check | 循环冗余校验 |
| **ADC** | Analog-to-Digital Converter | 模数转换器 |
| **DMA** | Direct Memory Access | 直接内存访问 |
| **TTL** | Transistor-Transistor Logic | 电平标准（本项目 3.3V TTL） |

---

## 9. 待补充

**⚠️ 本节待项目推进后补充：**

```
□ 机械专业术语（构型、连杆、自由度配置）
□ 电子专业术语（电源域、地平面、去耦）
□ 更多 RL 术语（奖励项名称、课程学习阶段名）
```

---

**文档结束。发现新的歧义词请追加到本表。**
