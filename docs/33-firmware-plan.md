# STM32 固件实施方案（Firmware Implementation Plan）

> **文档编号**：DOC-FW-PLAN-001 ｜ **版本**：v1.1 ｜ **日期**：2026-10-10
> **面向对象**：接手 STM32 固件开发的 Agent / 开发者 + 项目负责人
> **状态**：✅ **P0~P6 已实施并离线验证；P7 代码就绪，待硬件联调**
>
> **本文档是「怎么做」的施工方案**，不替代实现规格：
> 规格见 [`../firmware/doc/固件设计书.md`](../firmware/doc/固件设计书.md)、[`协议规范`](../firmware/doc/协议规范_树莓派通信.md)、
> [`伺服总线模块设计`](../firmware/doc/伺服总线模块设计.md)；接口表见 [`30-firmware.md`](30-firmware.md)。
>
> **⚠️ 本文档 §9 是"方案 vs 实际"的对照**，记录了实施中偏离方案的地方与原因。
> 上机执行请看 [`34-firmware-acceptance.md`](34-firmware-acceptance.md) 与
> [`35-firmware-deployment.md`](35-firmware-deployment.md)。

---

## 0. 本次接手的边界（已与负责人确认）

| 项 | 结论 |
|---|---|
| 范围 | **仅 STM32 固件**。Pi 侧程序、ESP32 固件不由本方案负责 |
| 舵机 ID 分配 | **由固件方决定**（见 §3） |
| HSE 晶振 | ✅ 已确认 8 MHz（`.ioc` 也确认：`VCOOutput2Freq_Value=8000000`） |
| 舵机控制板跳线 A 位 / 接线直连 | ✅ 已确认 |
| PB8/PB9 外部 4.7 kΩ 上拉 | ✅ 已确认已装 |
| **硬急停输入** | ⛔ **本次不做**（仅保留 `#if` 代码位，默认关闭） |
| USART3 第二负载（U6/U9） | ✅ 仅是引出排针、未使用 → 无冲突 |
| ESP32 供电能力 | ✅ 假定充足 |
| CubeMX | ✅ 负责人手上有 → **`.ioc` 的改动由负责人执行**，固件方不碰 `.ioc` |
| 编译链 | 固件方负责把 Keil 工程切到本机可用的 **AC6.22**（见 §5） |

---

## 1. 现状实测（不是照抄文档）

| 项 | 实测结论 |
|---|---|
| `.ioc` 的 NVIC 优先级 | ✅ **已经是目标值**：DMA1_Ch1=5、USART2=6、USART1=7、USART3=8、SysTick=15、`NVIC_PRIORITYGROUP_4`，且**已生成进 `stm32f1xx_hal_msp.c`** |
| 时钟 | ✅ HSE 8 MHz × PLL9 = 72 MHz，APB1=36 M（1 Mbps 整除，误差 0%）、APB2=72 M、ADC=12 M |
| UART 波特率 | ✅ USART1/2 = 1 Mbps；**USART3 = 115200**（2026-10-10 由 921600 下调，低频调试通道）|
| ADC + DMA | ✅ 连续转换 + DMA1_Ch1 循环，代码里缺一次 `HAL_ADC_Start_DMA()`（由 BSP 补） |
| GPIO | ✅ **PC13=LED（板载，低电平点亮）**、PA8=`HEARTBEAT`、PB8/PB9 开漏+高速、PA13/14=SWD |
| 中断函数 | `stm32f1xx_it.c` 已生成 5 个；`USART2_IRQHandler` 目前只调 `HAL_UART_IRQHandler` |
| 业务代码 | ❌ **0 行**，`main.c` 的 `while(1)` 是空的 |
| CubeMX 未设项 | `ProjectManager.HeapSize = 0x200`（未改 0）；`Dma.RequestsNb = 1`（只有 ADC，无 USART2 DMA） |
| **编译状态** | ❌ **编不过**：`.uvprojx` 选 AC5，本机 MDK 5.40 只装 AC6.22。已实测 `armclang` 能成功编译 `main.c`（exit 0） |
| 官方舵机库 | ✅ 4 个缺陷全部复现：`SC.c:337` `malloc`（且不查 NULL）、`SC.c:344` `free`、`uart.c` 自带 `USART1/2_IRQHandler`（必然重定义）、`SCSerail.c` 逐字节超时 |
| 舵机 `dir` 依据 | 硬件 URDF：右腿三关节 `<axis> = 0 0 -1`，左腿 `0 0 +1` → 右腿 `dir = -1`（有物理依据） |
| 训练侧对齐 | `rlcat_config.py`：47 维、`action_scale=0.25`、`dof_vel`×0.1、`gait_freq=3.0`、`decimation=2` → 与固件文档一致 |
| 可用的离线工具 | MinGW `gcc`（`D:\Software\Work\mingw64\bin`）、Python 3.14 + numpy → PC 端单元测试与 Pi 模拟器可做 |

---

## 2. 已确认的设计决策（含本次负责人拍板）

| # | 决策 | 值 | 来源 |
|---|---|---|---|
| 1 | **时序模型** | **模型 A（请求-应答）**，`link_pi_wait_cmd(4 ms)`；**保留模型 B 编译开关** `RLCA_LINK_MODE_B` | 负责人确认 |
| 2 | **硬急停** | 本次不实现，仅保留 `RLCA_ESTOP_INPUT_EN`（默认 0）与文档位 | 负责人确认 |
| 3 | **USART3 转发 TEL_FAST** | ⚠️ **已实现**（2026-10-10）：115200 + **中断发送（非阻塞）** + TEL_FAST 降频 10 Hz。见 §9.7 | 负责人 2026-10-10 决定 |
| 4 | **舵机 ID** | 逻辑索引 0–11 → ID 1–12（见 §3） | 固件方 |
| 5 | **步态相位** | **STM32 产生**，经 `TEL_FAST[86..89]` 下发；`PHASE_STEP = 3.0/100 = 0.03` | 文档已定 |
| 6 | **关节限位** | hip `0~+40°`、thigh `−35°~+90°`、calf `±60°`（弧度见 §4） | `30-firmware.md` §3.2 |
| 7 | **`q_default`** | 全 0 | 仿真实测 |
| 8 | **毫秒节拍** | `HAL_GetTick()`（**不需要 TIM2**） | 设计书 §4.3 |
| 9 | **微秒计时** | DWT `CYCCNT`（零外设占用），用于舵机总线整帧 3 ms 超时 | CubeMX 清单 §4.1 |
| 10 | **内存** | 全部静态分配，**不用 `malloc`**，堆将为 0 | 设计书 §9 风险 1 |

---

## 3. ⭐ 舵机 ID 分配（固件方决定）

**方案：逻辑索引 = ID−1**（与文档建议一致，好处是上电自检可直接按顺序扫描）

| 逻辑索引 | 关节名 | 舵机 ID | | 逻辑索引 | 关节名 | 舵机 ID |
|---|---|---|---|---|---|---|
| 0 | `lf_hip` | **1** | | 6 | `rf_thigh` | **7** |
| 1 | `lb_hip` | **2** | | 7 | `rb_thigh` | **8** |
| 2 | `rf_hip` | **3** | | 8 | `lf_calf` | **9** |
| 3 | `rb_hip` | **4** | | 9 | `lb_calf` | **10** |
| 4 | `lf_thigh` | **5** | | 10 | `rf_calf` | **11** |
| 5 | `lb_thigh` | **6** | | 11 | `rb_calf` | **12** |

**实现约定：**

```c
// rlcat_config.h —— ID 只在这一处出现，逻辑层不出现 ID
#define RLCA_SERVO_IDS  { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 }
```

- **上电自检**按 1→12 逐个 `PING`，**ID 缺失/错位 → 置 `STAT_READY=0` 并拒绝进入 ACTIVE**。
- **未知 ID 策略**：扫描时若发现 **ID > 12 或 ID = 0xFE** 的响应，记为 `BUS_ERR_UNEXPECTED_ID` 计数上报（不致命）。原因：出厂默认 ID 都是 1，**12 个舵机同时接上会 ID 冲突**（多个舵机同时应答同一 ID）。因此上位机改 ID 时必须**逐个接线、改完立刻拔掉、再接下一个**（[`30-firmware.md`](30-firmware.md) §2.3）。
- 若实际 ID 与此表不同：**只改这一个宏**，其余代码逻辑不动。

**⚠️ 这条需要你在上机前用 Feetech 上位机把 12 个舵机逐个改成上表的 ID。** 这是唯一需要你做的"舵机侧"配置。

---

## 4. 模块架构与关键取舍

```
firmware/stm32/
├── Core/Inc|Src/                 ← 全部新文件放这里（CubeMX 目录内，重生成风险最小）
│   ├── rlcat_config.h            ⭐ 全参数集中：频率/超时/阈值/限位/ID/换算/开关，代码里无魔数
│   ├── app_main.c                主循环 + 100 Hz 周期调度（唯一节拍源）
│   ├── app_mode.c/.h             状态机 INIT/IDLE/STAND/ACTIVE/DAMP/ESTOP/CALIB
│   ├── app_safety.c/.h           ⭐ L1/L2/L3 低压、超温、失联、伺服 ERR、IMU 失效 → 缓慢趴下
│   ├── link_pi.c/.h              AA55 帧 / CRC16-CCITT / 接收状态机 / TEL_FAST / TEL_STATUS
│   ├── servo_bus.c/.h            ⭐ 事务状态机（SyncWrite 阻塞 450 µs；SyncRead 异步）
│   ├── servo_proto.c/.h          STS 组帧/解帧/ERR（可离线单测，不碰 HAL）
│   ├── servo_math.c/.h           纯数学：步↔rad、负载→N·m、速度→rad/s（可离线单测）
│   ├── joint.c/.h                逻辑索引 A / servo_id / dir / offset / 限位
│   ├── imu.c/.h                  JY901P：软件 I2C 读四元数 + 陀螺
│   ├── battery.c/.h              ADC(DMA) → mV → 分级
│   ├── crc16.c/.h                查表 CRC16/CCITT-FALSE（测试向量 0x29B1）
│   ├── pi_frame.c/.h             帧编解码（不依赖 HAL，PC 上可单测）
│   └── bsp_{uart,tick,dwt,softi2c,flash,gpio}.c/.h
├── Middlewares/SCServo/          官方库副本（保留作协议参考 + 地址宏）+ uart_port.c（HAL 版）
└── MDK-ARM/                      stm32.uvprojx（切 AC6 + 新增源文件分组）
```

**5 条关键取舍（会记进 `90-decisions.md`）：**

| # | 取舍 | 理由 |
|---|---|---|
| 1 | **`servo_proto` 自实现 SyncWrite/SyncRead**，不改官方 `SC.c` | 官方 `SC.c` 是 global 指针 + `malloc` 的状态机，改造要动其核心结构；自写仅约 150 行，且**可完全离线单元测试**（本项目最看重的性质）。官方库仍保留作参考。这偏离《伺服总线模块设计》§7.3 的"建议 A"，属**实现方式**而非协议改动 |
| 2 | **USART2 接收先"中断+512 B 环缓+溢出计数+整帧 3 ms 超时"，跑通 1000 轮零错误后再上 DMA+IDLE** | 文档 §11 也是这个顺序。一上来就 DMA 会让"丢字节"和"逻辑错"难以区分 |
| 3 | **软件 I2C 不用硬件 I2C** | PB8/PB9 **不是** F103 硬件 I2C1 的复用脚（I2C1 在 PB6/PB7），且两脚已被 PCB 固定为开漏，所以只能软件 I2C。SCL 目标 100–400 kHz，用 DWT 计量实际周期 |
| 4 | **相位用 257 项正弦表 + 整数索引**，不用 `sinf()` | `arm_sin_f32` 属 CMSIS-DSP（本项目明确不引入）；257 项 `const float` 表 = 1 KB Flash，无浮点三角函数调用，确定性更好 |
| 5 | **CRC 用查表法**（512 B 表） | Flash 有余量，`TEL_FAST` 每周期要算 96 B，查表法比位运算快 8 倍 |

**分层原则**：BSP 只碰 HAL/寄存器；Service 一个模块一个概念，提供 `_init()/_poll()/_selftest()`；App 只做调度与决策。

---

## 5. 编译与工程（固件方负责）

**问题**：`.uvprojx` 的 `<ToolsetNumber>0x4`（AC5）+ `StupSel=8`，本机无 AC5 → UV4 报 *"ARM-Compiler 'Default Compiler Version 5' which is not available"*，一行都编不过。

**处理**：把 Keil 工程切到 **AC6.22（armclang）**。

**⚠️ 关键发现（2026-10-10 实测）：编译器选择【不存在于 `.uvprojx` / `.uvoptx` 文件里】。**

```
试过但无效：
  · 改 .uvprojx 的 <ToolsetNumber>0x4 → 0x6                  → UV4 -b 仍报 "Compiler Version 5 not available"
  · 改 .uvoptx 的 <ToolsetNumber>                            → 无效
  · 在 .uvprojx 里加 <pArmCC>6220000::V6.22::ARMCLANG</pArmCC> → 无效
结论：AC5/AC6 的选择由 Keil 通过 GUI 写入，脚本改不动。
```

**👉 需要你手动做一次（一次性，约 20 秒）：**

```
uVision 打开 firmware/stm32/MDK-ARM/stm32.uvprojx
  → Project → Options for Target 'stm32'（工具栏魔术棒）
  → Target 页签 → ARM Compiler 下拉框 → 选 "Use default compiler version 6"
  → 确定 → 保存
```

切好之后 `UV4 -b stm32.uvprojx` 即可用，与下面的命令行构建互为交叉验证。

**在 GUI 未切换之前，编译验证由命令行 armclang 构建承担：**

```
powershell -ExecutionPolicy Bypass -File firmware/tools/build.ps1
```

同一套编译器、同一目标、同一宏与包含路径：

```
armclang    -mcpu=cortex-m3 -std=gnu11 -Os -g -fshort-enums -fno-common -Wall -Wextra
启动文件    -masm=auto（⚠️ CubeMX 的 startup 是 Arm 传统汇编语法 AREA/SPACE/DCD，不是 GNU as）
链接        firmware/tools/rlcat_f103.sct
            （IROM 0x08000000 64K / IRAM 0x20000000 20K，与 Keil 默认布局一致）
校验        -Werror 语义：0 error 0 warning 才算通过；输出 .axf/.hex/.bin + 占用报告
```

**新增源文件分组**：`Application/RL-CAT` 组（5 个 `rlca_*.c`）。

**⚠️ CubeMX 重生成会重写 `.uvprojx`**，我方新增的文件条目可能丢失。**双保险：**

1. **所有新文件放在 `Core/Inc`、`Core/Src` 下**（CubeMX 自己的目录，重生成不会删外部文件）；
2. `firmware/tools/check-uvprojx.ps1`：比对 `.uvprojx` 的源文件列表与磁盘实际文件，
   报告 `MISSING`（列了但不存在）/ `UNTRACKED`（存在但没被编译）。**每次重新生成后跑一次。**

**清理**：已删除 `Drivers/CMSIS/DSP`（535 文件 / 11.9 MB）+ `Drivers/CMSIS/NN`（76 文件 / 0.8 MB）
= **省 12.7 MB / 611 文件**。删除后构建与测试仍全绿。

**⚠️ 仍未清理（都不在编译列表里，占约 22 MB，待你确认后再删）：**
`Drivers/CMSIS/Core_A`（Cortex-A 核，本项目是 M3）、`RTOS` / `RTOS2`（本项目不用 RTOS）、
`Lib`（原 DSP 的预编译库，随 DSP 删除已失去意义）。

---

## 6. 100 Hz 周期设计（模型 A）

```
t=0.00 ms  ① 解析上一轮 SyncRead 结果 → 更新关节状态（q/qd/tau/温度/ERR）
           ② 读 IMU（软件 I2C，实测后填真实耗时）
           ③ 取 ADC DMA 缓冲 → 电池电压
t=0.40 ms  ④ 组 TEL_FAST（96 B）→ 发送（103 B ≈ 1.03 ms 硬件时间；用 IT/DMA 发送不占 CPU）
t=1.45 ms  ⑤ ‖ model A：link_pi_wait_cmd(4000 µs)
t=1.85 ms  ⑥ 收齐 CMD_JOINT（38 B）→ CRC → mode 白名单 → NaN/Inf → 逐关节限幅 → 限速 0.15 rad/周期
t=2.00 ms  ⑦ SyncWrite 12 个目标位置（45 B，450 µs，唯一允许阻塞的阶段）
t=2.45 ms  ⑧ turnaround ≥ 20 µs → 发 SyncRead 请求（21 B，210 µs）
t=2.70 ms  ⑨ 12 舵机回复（读 9 B/个 ≈ 180 B / 1.80 ms）→ 中断收进环缓，本周期不解析
t=4.50 ms  ⑩ 周期结束，回主循环；相位 s_phase += 0.03
t=10.0 ms  下一周期（上轮数据已齐 → ① 解析）
────────────────────────────────────────────
CPU 占用 ≈ 2.7 ms（27%）；余量 ≈ 6 ms（60%）
```

**⚠️ 三处必须实测确认（P4 用 DWT 打点统计）：**
1. `⑤` 的等待窗口是否真能在 4 ms 内收到（Pi 侧 P99 要求 < 4 ms，见 `31-pi-runtime.md` §4.1）；
2. `⑨` 的 12 帧回复是否在下一周期 `①` 之前齐；
3. 周期抖动：**P99 必须 < 10 ms**，否则降级到模型 B。

**模型 B 开关**：`RLCA_LINK_MODE_B = 1` 时跳过 ⑤，直接用上一帧指令，周期内不做等待（端到端延迟 +10 ms，抖动归零）。

**RAM / Flash 预算（全部静态）：**

```
UART1 RX 512 + TX 256 ；UART2 RX 512 ；UART3 RX 256         = 1536 B
TEL_FAST 组帧 128 + 接收帧缓冲 128 + 协议临时 256             =  512 B
servo_feedback[12] + bus_stats + joint 表 + calib + 相位表     = ~1650 B
栈 1024 + 其它静态/全局                                        ≈ 2048 B
─────────────────────────────────────────────────
小计 ≈ 5.7 KB / 20 KB（余 ≈ 14 KB）✅

Flash：HAL ≈ 22~26 KB + 固件 ≈ 14~20 KB ≈ 45 KB / 64 KB ✅（AC6 -Os）
```

---

## 7. CubeMX 侧待改（**由负责人执行**，共 3 项）

### 🟢 必须改（1 项）

| # | 位置 | 现值 | 改为 | 理由 |
|---|---|---|---|---|
| **1** | `Project Settings → Project → Minimum Heap Size` | `0x200` | **`0x0`** | 我方全部静态分配、不用 `malloc`，省 512 B RAM。`ProjectManager.HeapSize=0x200` |

**⚠️ 第 1 项要与「不使用含 `malloc` 的官方 `SC.c`」配套**（见 §4 取舍 1）。若你希望保留回退到官方库的可能，则保持 `0x200` 也可以 —— 只差 512 B RAM。

### 🟡 P3 阶段改（DMA 接收方案定型后）

| # | 位置 | 改为 | 理由 |
|---|---|---|---|
| **2** | `Connectivity → USART2 → DMA Settings → Add` | `USART2_RX → DMA1 Channel 6 → Circular → Byte`，`MemInc=Enable`，`Priority=High` | 1 Mbps 下彻底消除丢字节。**⚠️ 加完 `stm32f1xx_it.c` 会多出 `DMA1_Channel6_IRQHandler`**；IDLE 中断仍需代码手动开：`__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE)` |
| **3** | `System Core → NVIC` | `DMA1_Channel6_IRQn` 抢占优先级 = **6**（与 USART2 同级） | 与总线优先级一致，避免被 USART1 打断 |

### ⛔ 本次不改

| 项 | 原因 |
|---|---|
| PB12–PB15 急停输入 | 负责人确认本次不做；代码里保留 `RLCA_ESTOP_INPUT_EN`（默认 0） |
| TIM2 1 kHz 节拍 | 用 `HAL_GetTick()` + DWT，不需要 |
| JTAG 重映射（PA15/PB3/PB4） | 本次不用这几个脚 |
| USART3 相关 | 第二负载已确认只是引出排针，无需改动 |

---

## 8. 离线验证策略（无硬件时如何保证质量）

| 手段 | 判据 |
|---|---|
| **① 编译门禁**（每轮必过） | `UV4 -b stm32.uvprojx` → **0 error / 0 warning**，并报告 FLASH / RAM 占用 |
| **② PC 纯逻辑单元测试**（MinGW gcc，可自动跑） | `crc16("123456789")==0x29B1`；帧状态机：`AA AA 55` 同步 / 垃圾前缀 / `LEN=255` 拒绝 / 截断帧 / 1 MB 随机流不崩；**相位序列与 `rlcat_env.py` 逐值比对**；`joint↔servo` 往返误差 < 1 步；负载/速度/电压换算；限幅+限速+NaN 处理。**覆盖协议 §10 全部 10 项** |
| **③ Pi 侧模拟器（Python）** | 与固件对打：100 Hz 收发、`cycle_cnt` 连续性、丢帧/错帧/半帧恢复、断链 30 ms 降级 / 100 ms 急停 |
| **④ 舵机总线模拟器** | 假舵机（PC 串口脚本）：正常回帧 / 故意不回 / 回错 CRC → 验证 3 ms 整帧超时、12 帧 `valid` 全 1、周期不被拉长 |
| **⑤ 上机验收单（你执行）** | 每条给：动作、预期现象、示波器测点、判据、失败时的排查分支 |

**②③④ 全部不需要硬件，在 P1/P3/P4 完成。**

---

## 9. 实施阶段与交付物

| 阶段 | 内容 | 交付 / 验收 | 依赖硬件 |
|---|---|---|---|
| **P0** | 切 AC6、建工程分组、`rlcat_config.h` 参数表、CRC/帧/数学纯逻辑模块骨架 | UV4 命令行构建 **0 error 0 warning** + 占用报告 | ❌ |
| **P1** | 纯逻辑层完成 + PC 单元测试全套 | 测试程序全绿（协议 §10 的 10 项 + 相位比对） | ❌ |
| **P2** | BSP：UART 环缓+溢出计数、DWT µs、软件 I2C、Flash、GPIO/LED/心跳、上电自检 | 验收单 1–3 项（LED 1 Hz 心跳、三路 UART 自发自收 1 万字节零丢失） | ✅ 需你上机 |
| **P3** | `servo_bus` + `servo_proto`：Ping 1 个 → 扫描 12 个 → SyncWrite → SyncRead | 验收单 4–7 项；连续 1000 轮零错误（`timeouts==0 && crc_errors==0 && rx_overflow==0`） | ✅ |
| **P4** | `link_pi`（模型 A + B 开关）+ Pi 模拟器联调 | 100 Hz 稳定收发；丢帧/错帧恢复；断链 30 ms 降级、100 ms 急停；周期 P99 < 10 ms | ✅ |
| **P5** | `imu` + `battery` | IMU 100 Hz 无丢帧；电压与万用表差 < 50 mV | ✅ |
| **P6** | 关节标定（读位→offset→Flash 存/校验/失败回默认+CALIB）+ `app_mode` + `app_safety` | 标定后 q=0 站立；L1/L2/L3、超温、失联阈值逐项触发正确 | ✅ |
| **P7** | 与 Pi 联调 + 写《部署联调手册》+ 回填 `docs/30-firmware.md`、`50-deployment.md` | 100 Hz 闭环连续 30 min 无异常 | ✅ |

**并行性**：P3 与 P4 可并行（一个调舵机、一个调通信）。**P0/P1 现在即可开工，不需要任何硬件。**

---

## 10. 需要你配合的动作清单

| 时机 | 动作 | 说明 |
|---|---|---|
| **开工前** | 用 Feetech 上位机把 12 个舵机 ID 设为 1–12（§3 表） | ⚠️ 逐个接线、改完立刻拔掉，避免 ID 冲突 |
| **开工前** | CubeMX 改 1 项：`HeapSize 0x200 → 0x0`（§7 必须改项） | 若想保留官方库回退余地，此项可不改 |
| **P0** | 确认是否同意删除 `Drivers/CMSIS/DSP` + `NN`（省 12.7 MB） | 不影响功能 |
| **P2 起每阶段** | 按《上机验收单》执行并回传结果 | 每条都很短（分钟级） |
| **P3** | CubeMX 加 USART2 DMA 接收（§7 的 2、3 项） | DMA 方案定型后 |
| **每次 CubeMX 重生成后** | 跑一次 `firmware/tools/` 的 `.uvprojx` 校验脚本 | 防止新增源文件条目丢失 |

---

## 11. 需要回填的文档问题（施工中顺手修，只改文档不改数值）

| # | 位置 | 问题 |
|---|---|---|
| 1 | `固件设计书` §4.4 | 时序表仍写 **"TEL_FAST 帧（85 B）"**、`SyncWrite 1.02 ms` → 应为 103 B / 450 µs |
| 2 | `固件设计书` §9 风险 6 | "`*_thigh` 限位不够 → 腿伸不直"与其 §5.3 已修正的结论矛盾 |
| 3 | `协议规范` §7.1 | "组装观测向量 **(50 维)**" → 应为 47 |
| 4 | `协议规范` §11 末尾 | "现有的最大帧是 90 字节" → TEL_FAST 已 103 |
| 5 | `协议规范` §7.3 / §12 | 时序模型 A/B 与 `31-pi-runtime.md` 冲突；应统一为 **A（本次已确认）** |
| 6 | `CubeMX配置清单` §7 | "建议改 USART3 波特率"（其 §3.2 已标 ✅ 已实现）→ 清理 |
| 7 | `CubeMX配置清单` §5 | PB12–PB15 描述需注明"急停输入本次不实现" |
| 8 | `docs/30-firmware.md` §4 | 急停引脚由"待定"改为"本次不实现，预留" |

---

## 12. 风险登记（本次新增）

| # | 风险 | 后果 | 对策 |
|---|---|---|---|
| 1 | 模型 A 的 4 ms 窗口装不下 Pi 的 P99 耗时 | 每周期丢指令 → 降级 | P4 实测 Pi 侧 P99；不达标切 `RLCA_LINK_MODE_B` |
| 2 | CubeMX 重生成冲掉 `.uvprojx` 的自定义分组 | 新增文件不参与编译 → 链接失败 | 全部新文件放 `Core/`；`tools/` 校验脚本每次重生成后跑 |
| 3 | 舵机 ID 冲突（出厂都是 1） | 总线多个同时应答 → 帧错乱 | 上电自检逐 ID PING；发现重复 ID 报警拒绝进 ACTIVE |
| 4 | 软件 I2C 时序不准 | IMU 读失败 | 用 DWT 量 SCL 实际周期；必要时降速而非加 `__NOP()` 猜 |
| 5 | 12 个舵机同时回复时环缓溢出 | 丢帧 | 512 B 环缓 + 溢出计数上报；不达标则上 DMA+IDLE |
| 6 | 位置多圈回绕（0↔4095） | 关节角跳变 | 相邻差 > 2048 判为回绕，做连续性修正 |

---

## 9. ✅ 方案 vs 实际（实施对照）

### 9.1 总体

```
P0 编译基线 + 参数表          ✅  命令行 armclang 构建，0 error 0 warning
P1 纯逻辑层 + PC 单测         ✅  288 项
P2 BSP 层 + 上电自检          ✅  验收单 P2 节（8 项）
P3 servo_bus                  ✅  143 项假总线单测；验收单 P3 节（9 项）
P4 link_pi + Pi 模拟器        ✅  267 项 C 单测 + 161 项 Python 测试；验收单 P4 节（9 项）
P5 imu + battery              ✅  196 项；验收单 P5 节（7 项）
P6 标定 + app_mode + app_safety ✅  115 项；验收单 P6 节（6 项）
P7 联调与文档回填             🟡  代码/手册/模拟器就绪，实测待硬件

C 侧单测合计 1009 项；Python 侧 161 项；FLASH 65.6% / RAM 30.8%
```

### 9.2 交付物（比 §4 规划多出的部分）

| 新增 | 为什么方案里没有、实施中加了 |
|---|---|
| `rlca_safety.{h,c}` | 把"什么时候该趴下"抽成**纯逻辑**（可单测），执行放在 `app_safety.c`。规划时只写了 `app_safety`，实施中发现"判据"与"执行"必须分开才能离线验证 |
| `rlca_calib_proc.{h,c}` | 标定流程独立成状态机，同样为了可单测 |
| `{servo,link,imu,batt}erry_port.c` / `link_port.c` / `sensor_ports.h` | **端口依赖注入层**：把"碰 HAL 的那一小块"隔离出来，业务逻辑全部可在 PC 上测。这是本项目最关键的工程手法 |
| `bsp_shell.{h,c}` | 上机自检命令行。规划里没有，但 P2 之后每阶段都靠它验收 |
| `bsp_softi2c.{h,c}` | 软件 I2C 从 `imu.c` 里独立出来（可被 shell 的 I2C 自检复用） |
| `tools/{build,test-host,test-python,check-uvprojx}.ps1` | 四个可复现的验证脚本 |
| `pi/{rlcat_proto,sim_esp32,selfcheck,test_loopback}.py` | 树莓派侧协议实现 + 模拟器 + 自测 |

### 9.3 实施中偏离方案的地方（附原因）

| # | 方案原计划 | 实际做法 | 原因 |
|---|---|---|---|
| 1 | 改 `.uvprojx` 切 AC6 | **改不动**，改用命令行 armclang | 编译器选择不在 `.uvprojx`/`.uvoptx` 里（三个位置都试过）。Keil GUI 需手动切一次，见部署手册 §6.2 |
| 2 | `servo_proto` 自实现（方案 §4 取舍 1） | ✅ 照做 | 换来"完全离线可单测"，143 项测试证明了它的价值 |
| 3 | 相位用 257 项正弦表 | **改用十六分区 + 泰勒多项式** | 表要 4112 B Flash；多项式约 20 行代码、误差 3.58e-6（≈0.1 LSB） |
| 4 | 相位按 π/2 归约 | **改用 π/4**（十六分区） | 按 π/2 归约时"补角"会达到 3π/4，跑出泰勒展开的精度区间 → 实测误差 2.6e-4 |
| 5 | 标定期间跳过整个控制环 | **标定期间继续跑只读事务** | 否则 SyncRead 不执行、`servo_bus_feedback()` 永不刷新，标定的 `get_pos()` 必然失败 |
| 6 | 未规划 | **`app_calib_start()` 要求先有新鲜反馈** | 上层可能还没跑过一轮就读位置，会拿 0 值当零点 |
| 7 | 未规划 | **`link_pi_wait_cmd` 加自旋上限** | 微秒时间源不前进时会死循环，锁死 100 Hz 主循环 |
| 8 | 未规划 | **`servo_bus` 用"短暂静默"判回复结束** | 只等"收满 12 帧"时，掉一个舵机就永远等不到、白等满 3 ms |

### 9.4 实施中发现并修正的**文档错误**（已回填）

| 文档 | 原文 | 正确值 |
|---|---|---|
| 伺服总线模块设计 §6.1 | SyncWrite 12 舵机 45 字节 / 450 µs | **44 字节 / 440 µs**（原文按 `LEN+5` 算，与自己的 LEN 定义不自洽） |
| 伺服总线模块设计 §6.2 | SyncRead 请求 21 字节 | **20 字节** |
| 伺服总线模块设计 §6.2 | 反馈帧总长算式用 `+5` | **`+6`**（反馈帧比指令帧多一个 ERR 字节），读 9 字节 = **15 字节** |
| 伺服总线模块设计 §2.2 | 未说明 `0x83`/`0x82` 的同一字段语义不同 | 补：写 ≤4、读按需（本项目 9） |
| 整体 | 未说明 JY901P 的角度寄存器给的是**欧拉角**而非四元数 | 已写入 `imu.h`；固件做 ZYX 欧拉角→四元数 |

### 9.5 实施中发现并修正的**代码缺陷**（每条都被单测抓到）

```
1  offset 范围检查写成 `offset > 4096`          → 对负值恒真，负的超范围 offset 漏检
2  STS_MAX_PAYLOAD=32 装不下 12 舵机 SyncWrite   → 返回参数错，且调用方 n-1 下溢成 65535 → 死循环
3  STS_FB_FRAME_LEN 漏算 CHK（写成 14）         → 12 帧反馈被全部丢弃
4  标定：进方向测试时没给第一个关节下指令        → 必然 NO_MOTION，标定 100% 失败
5  标定：采样阶段卸了力矩，方向测试前忘了重新使能 → 同上
6  标定：最后一个关节从没被回位                  → 它停在 base+98，offset 错掉
7  标定：方向测试后立刻读 offset                 → 回位指令还没走完，offset 偏 40 步
8  battery 用 s_last_poll_ms != 0 当初始化哨兵   → 上电 t=0 时节流失效，连采好几次
9  rlca_safety 降级（link_level==1）没置原因位   → Pi 只能从 severity 反推
10 calib_validate_result 先做整体检查            → 笼统的 LIMIT 盖掉了"方向不明"这个具体原因
```

### 9.6 未完成 / 需要人工确认的部分

| 项 | 状态 | 说明 |
|---|---|---|
| **实际硬件联调** | ⬜ 待做 | 我无法接触硬件。验收单 P2~P6 共 37 项由操作者执行 |
| **JY901P 寄存器映射** | ✅ **已核实 + 上机通过 P5-2**（2026-10-11）| 手册到手后查出并修正 **3 个真 bug**：① 器件 ID 错取 0x00（那是 SAVE，可写！只读版本号在 0x2E）② 角度换算按 0.01°/LSB（手册是 raw/32768×180°）③ ⭐ **字节序：实测是小端，旧固件按大端解析导致姿态全错**。详见 §9.8 |
| **Keil GUI 切 AC6** | ⬜ 待做 | 一次性操作，见部署手册 §6.2 |
| **模型 A 的 4 ms 窗口** | ⬜ 待实测 | 模拟器给出 P50/P95/P99；不达标改 `RLCA_LINK_MODE_B=1` |
| **硬急停输入** | ⛔ 本次不做 | `RLCA_ESTOP_INPUT_EN = 0`，且**该宏无任何代码引用**（刻意：留无人用的开关，胜过留没人测过的代码） |
| **USART3 转发 TEL_FAST** | ✅ **已实现**（2026-10-10） | 见下方 §9.7。**⚠️ 这一项从"本次不做"变成了"已做"，且与 §0 的原定范围不符** |
| **`0xAA 0x55` 与 shell 共存** | ✅ 已论证 | shell 只吃可打印 ASCII，`0xAA` 永远进不了命令缓冲 |
| **P7 联调与系统辨识** | ⬜ 待做 | 见部署手册 §5 的 7 步放开顺序 |

---

## 9.7 ⚠️ USART3 转发 TEL_FAST：从"不做"到"已做"

### 起因

§0/§2 原定"**USART3 转发 TEL_FAST 暂不处理**"。2026-10-10 ESP32 侧
完成 UART 桥后要求 STM32 补齐 TX 方向，于是**超出原定范围**实现了这一项。

### ⚠️ 第一次实现是错的，且被真机抓到

第一版实现（阻塞发送 + 100 Hz 全量镜像）踩了两个禁区：

| # | 错误 | 后果 |
|---|---|---|
| 1 | 用 `bsp_uart_write()`（阻塞式 `HAL_UART_Transmit`） | 103 B @921600 = 1.12 ms，**把 100 Hz 控制环拖到 ~10.8 ms ≈ 92 Hz** |
| 2 | 100 Hz 全量镜像 | 10.3 kB/s 占满 921600 的 11%，但**降到 115200 后会占 90%**（必然丢帧） |

**这是 ESP32 侧先观测到的**（`rx ≈ 9.5 kB/s` 而非 10.3 kB/s），
不是我这边发现的 —— 说明**离线测试覆盖不到时序影响**。

**⚠️ 92 Hz 不是"稍微慢一点"，而是破坏了 RL 的时序契约**：
训练侧 `decimation = 2` 要求控制环**严格 100 Hz**
（策略 50 Hz × 2 = 控制 100 Hz）。控制环掉到 92 Hz 会让策略
看到的"每步时间"与训练时不一致，属于**会静默劣化策略**的故障。

### ✅ 正确实现（三件事必须同时成立）

| # | 项 | 值 | 位置 |
|---|---|---|---|
| 1 | 波特率 | **115200**（原 921600） | `main.c` 的 `huart3.Init.BaudRate` |
| 2 | 发送方式 | **中断 `HAL_UART_Transmit_IT`**（非阻塞） | `esp_fwd.c` + `HAL_UART_TxCpltCallback` |
| 3 | 镜像频率 | TEL_FAST **100 Hz → 10 Hz**（`RLCA_ESP_FWD_DECIM = 10`） | `esp_fwd.c` |

**镜像的帧类型**（方案 A）：

```
TEL_FAST   103 B  降频到 10 Hz    ← 关节角/姿态，ESP32 的 s 命令要用
TEL_STATUS  47 B  本就 10 Hz      ← status 仲裁位 + 关节温度
HEARTBEAT    7 B  本就  1 Hz      ← ESP32 判断 STM32 是否存活
TEL_ACK      9 B  ❌ 不镜像       ← 控制应答，ESP32 不需要
```

合计约 **1.5 kB/s = 115200 的 13%**，余量充足。

### 关键设计点（都有理由，别随意改）

| 点 | 做法 | 为什么 |
|---|---|---|
| **镜像 = 同一份字节** | 在 `link_pi.c` 的 `send_frame()` 里**组一次帧、发两路** | 若各自动组帧，SEQ 会不一致，两条链路看到的就不是同一份数据 |
| **镜像走端口注入** | `rlca_link_port_t` 新增 `write_mirror` 成员 | 若让 `link_pi.c` 直接依赖 `esp_fwd.c`，它会拖着 HAL，**PC 上就链不上、无法单测**（第一版就是这样，6 个套件全炸） |
| **先发 Pi 再发 ESP32** | `send_frame()` 里的顺序 | ESP32 侧可能因 TCP 卡住而阻塞/失败，若排在前面会推迟 Pi 的遥测 |
| **单缓冲 + busy，不排队** | 上一帧没发完就丢新帧 | 排队会让遥测延迟**无界增长**（ESP32 慢一点 → 队列越积越长 → 看到的越来越旧）；遥测本就可丢 |
| **100 ms 卡死看门狗** | 中断若永远完不成则强制复位 busy | 否则 TC 中断不来时链路会**永久死掉**，再无任何帧发出 |
| **补 `tx_errors` 计数** | `bsp_uart_t` 新增发送侧计数 | 此前**接收侧有 `rx_errors`、发送侧什么都没有** → ESP32 不读数据会**静默丢帧**，上机查不出来 |

### 验证覆盖

| 层 | 手段 |
|---|---|
| **镜像策略**（哪些帧该发） | ✅ PC 单测：`test_link_pi.c` 的 `test_esp_mirror_policy`（断言逐字节相同 + TEL_ACK 不镜像） |
| **非阻塞性**（不拖控制环） | ⬜ **只能上机实测**：看 `s` 的 `esp_tx_errors` 与周期耗时；离线测不了 |
| **降频生效** | ⬜ 上机：ESP32 侧应收 10 Hz 而非 100 Hz |

**⚠️ 这也是教训**：离线单测能覆盖"逻辑对不对"，
**覆盖不了"时序有没有被破坏"** —— 后者必须靠真机或周期耗时统计。

---

**文档结束。**
**下一步：按 [`35-firmware-deployment.md`](35-firmware-deployment.md) §3 的顺序逐项执行
[`34-firmware-acceptance.md`](34-firmware-acceptance.md) 的验收单。**
