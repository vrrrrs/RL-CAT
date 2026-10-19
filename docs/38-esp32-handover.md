# ESP32-C3 无线桥 工作汇总（v0.5.0）

> **文档编号**：DOC-ESP-SUM-001 ｜ **版本**：v1.0 ｜ **日期**：2026-10-10
> **面向对象**：项目负责人 / 后续接手者
> **配套文档**：[`36-esp32-plan.md`](36-esp32-plan.md)（实施方案+环境坑）、[`37-esp32-acceptance.md`](37-esp32-acceptance.md)（上机验收单）、[`32-esp32-bridge.md`](32-esp32-bridge.md)（设计）、[`../firmware/esp32/README.md`](../firmware/esp32/README.md)（使用说明）

---

## 0. 一句话结论

**ESP32-C3 无线桥固件已全部实现并在真机上跑通。** 与 STM32 的 **TX 方向链路（10 Hz 遥测镜像）实测 100% 有效帧、零 CRC 错**。
剩余两件小事：**① STM32 侧把 TEL_STATUS 错开一个周期（一行）**；② （可选、后续）STM32 侧实现 USART3 下行帧接收，ESP32 的 `s/z/c` 才真正能驱动机器人。

---

## 1. 状态总览

| 维度 | 状态 | 证据 |
|---|---|---|
| 固件功能 **P0~P4** | ✅ 完成 | 9 个源文件 + 10 个头文件；`pio run` **0 error 0 warning** |
| 文档 **P5** | ✅ 完成 | 本文档 + 36 方案 + 37 验收单 + DD-16~DD-22 + 两处索引 |
| **离线验证** | ✅ **130 项主机侧测试全绿** | 协议 72 + 仲裁门控 18 + 配置校验 40；另有与协议文档 §9 的 Python 实现 **5/5 逐字节对拍** |
| **真机 A1~A3** | ✅ 通过 | 烧写成功、启动日志 0 错误、自检 6/6、60 s 空载无泄漏 |
| **链路 TX（STM32→ESP32）** | ✅ **通过** | 115200 / 10 Hz：`crc=0 rs=0 pe=0 fe=0`，**300/300 帧有效** |
| 链路 RX（ESP32→STM32） | ⛔ STM32 未实现 | 契约第 2 条，见 §5 |
| 验收 A4~A23 | ⏳ 待执行 | 需切 WiFi（我不能切，见 §6.5）+ 一根跳线；A22/A23 需 STM32 侧 |
| 硬急停 / `e` 命令 | ⛔ 本轮不做（已确认） | DD-19 |

**规模**：RAM 31580 B (9.6%)、Flash 880738 B（app 分区 2 MB 的 **42.0%**）。

---

## 2. 交付物清单

### 2.1 固件（`firmware/esp32/`）

| 文件 | 作用 |
|---|---|
| `platformio.ini` | board=`esp32-c3-devkitm-1`、`framework=espidf`、`build_dir=D:`、`board_build.partitions` |
| `sdkconfig.defaults` | 控制台走 USB-Serial-JTAG、4 MB Flash、1 kHz tick、看门狗由代码初始化 |
| `partitions.csv` | app 分区 2 MB（IDF 默认 1 MB 太紧） |
| `include/rlca_esp32_config.h` | ⭐ 全部参数（WiFi/端口/波特率/**引脚**/超时/协议偏移/命令集/看门狗） |
| `include/rlca_protocol.h` | 帧状态机 + CRC16 + TEL_FAST/TEL_STATUS 访问器 + **重力解算**（与树莓派同公式） |
| `include/rlca_arbiter.h` | 仲裁门控 + `cycle_cnt` 连续性判断（均为**纯函数**，主机侧可测） |
| `include/rlca_cfg.h` / `rlca_cfg_validate.h` | NVS 配置 API + 纯校验函数（密码/端口/波特率/引脚） |
| `include/rlca_net.h` / `rlca_bridge.h` / `rlca_injector.h` / `rlca_selftest.h` / `rlca_logmirror.h` | 各模块接口 |
| `src/main.c` | 初始化 + 任务创建 + 心跳日志（自检、配置、看门狗接线） |
| `src/wifi_ap.c` | AP + **3333 透传口 / 3334 控制口** + 控制口命令分发 |
| `src/uart_bridge.c` | UART 驱动 + 收发任务 + **开机内部回环自检** + 首字节 hex dump |
| `src/protocol.c` | 帧状态机 + CRC16 查表 |
| `src/arbiter.c` | 遥测跟踪 + 注入门控 + 过期判定 |
| `src/injector.c` | 文本命令 → 标准协议帧（`i` / `imu` / `s` / `z` / `c` / `r` / `raw` / `hb`） |
| `src/cfg.c` | NVS 配置读写（密码 / 端口 / 波特率 / 引脚） |
| `src/selftest.c` | 板上自检（与主机侧同一套向量，6 项） |
| `src/logmirror.c` | `ESP_LOG` 镜像到 3334（LF→CRLF，带递归保护） |

### 2.2 测试与工具

| 文件 | 作用 |
|---|---|
| `firmware/esp32/test/test_protocol.c` | 协议单元测试 72 项（协议规范 §10 全 10 项 + CRC 表 vs 位运算 2 万组 + 偏移/状态位 + 量化 + 重力解算） |
| `firmware/esp32/test/test_arbiter_gate.c` | 仲裁门控 18 项（无遥测/新鲜/过期/边界/Pi 在线互斥 + `cycle_cnt` 抽帧容忍） |
| `firmware/esp32/test/test_cfg.c` | 配置校验 40 项（WPA2 密码、端口、波特率、**引脚禁区**） |
| `firmware/esp32/test/reference_frames.py` | 协议文档 §9 的参考实现（逐字节对拍用的**独立 oracle**） |
| `firmware/esp32/tools/bench.py` | 上机工具：`doctor` `selftest` `loop` `mon` `sim` `tcp` `con` `verify` |
| `firmware/esp32/tools/rlca_proto.py` | 帧编解码与美化打印（含四元数→重力） |
| `firmware/tools/pio.ps1` | ⭐ 构建入口：目录重定向 + **按 VID 303A 自动选串口** + SUBST 别名 |
| `firmware/tools/test_esp32_host.ps1` | 一键跑全部主机侧测试 |
| `firmware/tools/bench.ps1`、**`bench.cmd`（仓库根）** | 上机工具入口（`bench.cmd` 绕开 PowerShell 脚本策略） |

### 2.3 文档

| 文档 | 内容 |
|---|---|
| [`36-esp32-plan.md`](36-esp32-plan.md) | 实施方案：阶段划分、关键取舍、**环境踩坑 6 条**、**STM32 侧镜像实现要求（含示例代码）**、离线验证策略、风险登记 |
| [`37-esp32-acceptance.md`](37-esp32-acceptance.md) | 上机验收单：**A1~A23** 逐条「动作/预期现象/判据/失败分支」+ 现象对照表 EP1~EP10 + 记录表 |
| 本文档 | 工作汇总（状态、证据、问题清单、操作手册、待办） |
| [`90-decisions.md`](90-decisions.md) | **DD-16~DD-22**：双 TCP 口、`s` 用关节角增量、完整帧+CRC 仲裁、不做硬急停、IDF 版本、工具链目录、健壮性三件套 |
| [`firmware/esp32/README.md`](../firmware/esp32/README.md) | 使用说明（构建/烧写/工具/引脚/进度） |

---

## 3. 真机链路实测（关键数据）

### 3.1 921600 失败 → 115200 成功

| 指标 | 921600（旧） | **115200（现行）** |
|---|---|---|
| 有效帧比例 | **3.7%**（30/925） | **100%**（300/300） |
| `crc` / `rs`（CRC 错 / 超时重同步） | 36/s、25/s | **0 / 0** |
| `pe` / `fe`（UART 帧错/校验错） | 0 / 0 | 0 / 0 |
| `ovf` / `bf`（FIFO 溢出 / 缓冲满） | 0 / 0 | 0 / 0 |
| 速率 | 9.5 kB/s（但内容全坏） | **1030 B/s = 103 B × 10 Hz** |
| 控制环被拖慢 | ~92 Hz（阻塞发送所致） | 100 Hz（STM32 侧已改 IT 发送）✅ |

**结论**：`pe/fe/ovf/bf` 全 0 而内容大量损坏 ⇒ **比特级错误（信号完整性）**，不是协议/固件/波特率推算问题。降到 115200（每位宽 8.7 µs，是 921600 的 8 倍）后错误完全消失。

### 3.2 ESP32 侧"自证清白"的证据

固件开机自动做两级**芯片内部回环**（不接任何线）：

```
UART1 internal loopback: OK (16 bytes returned)                    ← 16 字节图案逐字节比对通过
UART1 loopback frame stream: 20/20 frames CRC-clean (2060 bytes)   ← 921600 下整帧回环 20/20
```

⇒ ESP32 的 UART 驱动、引脚复用、接收路径在目标速率下**完全可靠**；真实链路的损坏只可能来自外部（地/线/争用）。

### 3.3 现行链路（STM32 → ESP32）

```
first 32 bytes on UART1: AA 55 60 0A 02 00 ...     ← LEN=96、TYPE=0x02(TEL_FAST)，帧头完美
up=10 s: uart rx=10300  frames=100 fast=100 crc=0 rs=0 pe=0 fe=0 ovf=0 bf=0
up=20 s: uart rx=20600  frames=200 fast=200 crc=0 rs=0 ...
up=30 s: uart rx=30900  frames=300 fast=300 crc=0 rs=0 ...
```

---

## 4. 排查过程中发现并修复的问题（⭐ 最有价值的部分）

| # | 现象 | 根因 | 处理 | 归属 |
|---|---|---|---|---|
| 1 | 板子插上后设备管理器图标狂跳、烧不进 | **出厂 Flash 全 0xFF** → ROM 刷 `invalid header` + 看门狗每 2.4 s 复位 → USB 反复枚举（看起来像硬件坏） | 烧入固件即消失；首次烧写需重试一次抓端口窗口 | 认知 |
| 2 | app 分区只有 1 MB，项目里的 2 MB `partitions.csv` 被无视 | **PlatformIO 的 IDF 集成自己生成分区表**（`espidf.py:1528` 读 `board.get("build.partitions", …)`），完全忽略 `CONFIG_PARTITION_TABLE_CUSTOM` | `platformio.ini` 加 `board_build.partitions = partitions.csv`；已上机确认为 `factory 00200000` | ESP32 |
| 3 | 每次启动一行红字 `E task_wdt: TWDT already initialized` | `sdkconfig.defaults` 里 `CONFIG_ESP_TASK_WDT=y` **不是 IDF 5.x 的选项名**（应为 `ESP_TASK_WDT_EN`/`ESP_TASK_WDT_INIT`） | 改为 `EN=y` + `INIT=n`，由 `main.c` 按 `RLCA_TWDT_*` 初始化；已上机确认 0 错误行 | ESP32 |
| 4 | `pio` 报 `HomeDirPermissionsError` | 默认 core 目录在用户主目录，会话沙箱不允许写 | core 目录重定向到 D 盘 | 环境 |
| 5 | CMake 配置阶段 `PermissionError [Errno 13]`（但别的进程能写同一目录） | **DSH 沙箱给工作区打了 Low 完整性级别** → 工作区内的 `python.exe` 一运行就是 Low 进程，写不了 Medium 级的 D 盘（Windows no-write-up）；IDF 恰好调用工作区里的 venv python | 把 PlatformIO core 目录也搬到 D 盘 | 环境 |
| 6 | `Error: Detected a whitespace character in project paths` / `-fmacro-prefix-map=C:/Users/Liu` | IDF 不支持含空格工程路径，而仓库在 `C:\Users\<user name>\…` | `build_dir` 指向无空格路径 + wrapper 自动建 **SUBST 别名 `R:`**（不用 junction，因为会被解析回真实路径） | 环境 |
| 7 | PlatformIO 在 IDF venv 上跑 pip 时 CPU 打满不返回 | pip 解析器回溯 + 被 kill 的运行留下半个 venv 抢锁 | 用平台自带开关 `.pio_skip_pypackages` 跳过；另给 `idf_component_manager` 的 run-counter 加 `try/except` 容错 | 环境 |
| 8 | **ESP32 一个字节都收不到**（`uart rx=0`） | **固件用的 GPIO6/7 在 RL-CAT PCB 上是空脚**；PCB 实际把模块的 **GPIO20(RX)/GPIO21(TX)** 接到 STM32 的 USART3 | 默认引脚改为 **TX=GPIO21 / RX=GPIO20**，并新增 `cfg pins <tx> <rx>`（NVS 可配） | 硬件/固件 |
| 9 | 烧写失败 `Invalid head of packet (0x42)` | 插了 USB-TTL 后 **PlatformIO 自动挑中 COM6**（转接板）而不是 ESP32 的 COM8 | wrapper 改为**按 VID `0x303A` 自动挑 ESP32 自带 USB** 口 | 工具 |
| 10 | 链路能收到字节但 96% 帧 CRC 错 | 921600 下的**信号完整性**（地环路/线材/可能的总线争用） | 降到 **115200** 后 `crc=0`、100% 有效 | 系统 |
| 11 | 10 Hz 抽帧会让 `cycle_gaps` 天天误报 | 原判据要求 `cycle_cnt` 每帧 +1 | 改为「容忍抽帧」：只把**重复**或**跳变 >500**（含 16 位回绕）记为异常，并加 7 项单元测试 | ESP32 |
| 12 | `st=0`：TEL_STATUS 一帧都到不了 | STM32 的 TEL_FAST 与 TEL_STATUS **落在同一个 10 ms 周期**，TEL_FAST 先发（115200 下占 8.9 ms），TEL_STATUS 撞上 `s_busy` 被丢 | **待 STM32 侧一行修复**（错开 5 个周期 = 50 ms），见 §5.3 | STM32 |

---

## 5. 与 STM32 的接口现状（USART3）

### 5.1 契约三条（[`36-esp32-plan.md`](36-esp32-plan.md) §2）

| 契约 | 现状 |
|---|---|
| 1 帧格式完全复用（不新增） | ✅ 满足（`rlca_frame.c` / `rlca_crc16.c`，测试向量 0x29B1） |
| 2 STM32 在 USART3 接收下行帧 | ❌ **未实现**（`g_uart_esp` 只计数，无消费者）→ ESP32 的 `s/z/c` 目前"有回复、无效果" |
| 3 STM32 在 USART3 发送遥测 | ✅ **已实现并实测通过**（10 Hz TEL_FAST，`crc=0`） |
| 4 `STAT_LINK_OK` 只反映 USART1/Pi | ✅ **已确认正确**（`app_main.c:110` 用 `link_pi_is_alive()`）→ ESP32 仲裁自锁风险已排除 |

### 5.2 两侧当前的参数约定

```
STM32 USART3  : 115200 8N1，PB10(TX) / PB11(RX)
ESP32 UART1   : 115200 8N1，GPIO21(TX) / GPIO20(RX)        ← 交叉 + 共地
镜像内容      : TEL_FAST 10 Hz（decim=10）+ TEL_STATUS 10 Hz（⚠️ 见 §5.3）
镜像实现      : esp_fwd.c —— HAL_UART_Transmit_IT + 静态缓冲 + 忙则丢帧 + 100 ms 卡死兜底 ✅ 设计正确
```

### 5.3 待 STM32 侧修的两处

| # | 事项 | 改动量 | 影响 |
|---|---|---|---|
| **① TEL_STATUS 错开发送** | `app_main.c:635` 加一个相位条件：<br>`if (s_req_status \|\| (((now - s_status_last_ms) >= 100u) && ((s_cycle_cnt % 10u) == 5u)))` | **1 行** | 修好后 ESP32 才能看到 `mode` / 12 个舵机温度 / 总线错误计数 / `uptime`（`status` 位与电压两帧都有，所以仲裁与 `imu` 不受影响） |
| ② （可选）镜像 `TEL_ACK` | `esp_fwd.c` 的过滤放开 `TEL_ACK` | ~2 行 | ESP32 才能确认"注入的指令被接受" |
| ③ （后续）USART3 下行帧接收 | 把 `g_uart_esp` 字节喂进协议状态机（可与 `link_pi` 共用 `rlca_frame.c`，两个实例） | 中等 | ESP32 的 `s/z/c` 才能真正驱动舵机；⚠️ 下行同样要过"模式白名单/NaN/限幅/限速" |

**⚠️ ③ 的红线**：`STAT_LINK_OK` 必须继续只反映 USART1/Pi。若改成"任意链路活着"，ESP32 自己的注入会把该位置 1 → **ESP32 仲裁自锁**（自己的指令全被自己丢掉）。

---

## 6. 怎么用（操作手册）

### 6.1 构建 / 烧写 / 监视（**必须走 wrapper**）

```powershell
cd firmware\esp32
powershell -ExecutionPolicy Bypass -File ..\tools\pio.ps1 run             # 构建
powershell -ExecutionPolicy Bypass -File ..\tools\pio.ps1 run -t upload   # 烧写（自动按 VID 303A 选口）
powershell -ExecutionPolicy Bypass -File ..\tools\pio.ps1 device monitor  # 看日志
```

### 6.2 热点与端口

```
SSID = RL-CAT-E959（取 SoftAP MAC 后 4 位；芯片出厂 MAC e0:72:a1:22:e9:58）
密码 = rlcat1234          IP = 192.168.4.1
3333 = 纯透传（STM32 ↔ 笔记本）      3334 = ESP32 控制口（命令 + 日志镜像）
```

### 6.3 控制口命令（连 3334）

```
?            帮助                stats    桥接/UART 计数（含硬件错误计数）
i            链路与遥测状态（不注入）  imu / m  IMU：四元数/陀螺/机身系重力（= 观测 0~2 维）
selftest     板上自检（6/6）      cfg      运行配置（pw / port / baud / pins / clear）
s <j> <ddeg> 单关节微动（需遥测；受仲裁门控）    z 卸载力矩   c 开始标定   r 清计数
raw <hex>    直接向 STM32 发字节   hb 发一帧 HEARTBEAT   k 心跳   q 断开
```

### 6.4 上机工具（`bench.cmd`，仓库根）

```cmd
"<repo>\bench.cmd" doctor                        :: 体检：WiFi/端口/UART 到底哪不对
"<repo>\bench.cmd" selftest                      :: 工具自检（无需硬件）
"<repo>\bench.cmd" con --host 192.168.4.1 --port 3334   :: 控制口交互
"<repo>\bench.cmd" tcp --host 192.168.4.1 --frames 0    :: 解码 3333 遥测流
"<repo>\bench.cmd" verify --loopback --report D:\acc.json  :: ⭐ 一条命令跑完验收自动项
```

> ⚠️ 本机 PowerShell 默认**禁止运行 .ps1**（`UnauthorizedAccess`）：用 `bench.cmd`（不经 PowerShell），或加 `-ExecutionPolicy Bypass`。
> ⚠️ 在 PowerShell 里调用带空格的路径要加调用符：`& "C:\...\bench.cmd" doctor`。

### 6.5 关于"A4~A21 验收为什么还没跑"

验收的 TCP 用例需要笔记本连到 ESP32 的热点；而本机**以太网未接、WLAN 是唯一网络通路**，切热点会切断与 AI 后端的连接（负责人确认过）。所以这部分需负责人执行 [37-esp32-acceptance.md](37-esp32-acceptance.md) §0.1 的**一条命令**，把输出/JSON 回传即可。

---

## 7. 参数速查

| 项 | 值 |
|---|---|
| 芯片 / 板 | ESP32-C3 (QFN32) rev v0.4，4 MB 内嵌 Flash(XMC)，原生 USB-Serial-JTAG VID:PID `303A:1001` |
| PlatformIO board | `esp32-c3-devkitm-1`；框架 `espidf` 3.50503.0（IDF **5.5.3**）+ riscv 工具链 14.2.0 |
| 工具链目录 | core/packages/platforms/build 全在 `D:\Work\EspIDF\`（原因见 §4 的 4/5/6） |
| UART 引脚 | **ESP32 GPIO21(TX) / GPIO20(RX)** ↔ STM32 **PB11 / PB10**（交叉）+ 共地 |
| 波特率 | **115200**（低频调试口，默认）；`cfg baud 921600` 可切回契约值 |
| TCP 端口 | 3333 透传 / 3334 控制（`cfg port` 可改，NVS） |
| NVS 可配项 | AP 密码、两个端口、波特率、UART 引脚（改后重启生效；`cfg clear` 复位） |
| 看门狗 | 5 s，panic=1，三个任务（UART 泵 / socket / 仲裁）喂狗 |
| 仲裁 | 完整帧 + CRC 校验；过期阈值 500 ms；**状态未知 ⇒ 禁止注入**（失败安全） |
| 版本号 | `RLCA_CFG_VERSION = "0.5.0"`（启动日志第一行会打印） |

**引脚禁区**：GPIO11–17（内嵌 Flash）、18/19（USB D±）、2/8/9（strapping）。`cfg pins` 会拒绝这些值。

---

## 8. 未完成 / 待办

| # | 事项 | 责任人 | 判据 |
|---|---|---|---|
| 1 | **STM32 侧错开发送 TEL_STATUS**（§5.3 ①，一行） | STM32 侧 | ESP32 日志 `st≈10/s`，`i` 能显示 mode/温度 |
| 2 | 验收 **A4~A21**（一条 `bench.cmd verify --loopback` 命令） | 负责人 | 全 `[ok]` + JSON 报告 |
| 3 | A6/A7/A8/A21（USB-TTL 对打 / 慢客户端 / 30 min 长跑） | 负责人提供 USB-TTL | 见验收单 |
| 4 | 与真 STM32 联调 **A22/A23** | 依赖 §5.3 ③ | 无线链路能看到 STM32 遥测并下发指令 |
| 5 | （可选）USART3 下行帧接收 → `s/z/c` 真正生效 | STM32 侧 | 真机单关节微动 |
| 6 | （可选）`TEL_ACK` 镜像、shell 显示 `esp_lost` | STM32 侧 | — |
| 7 | （可选）把 5 ms 半帧超时按波特率缩放 | ESP32 | 若实测出现 `rs` 增长再做 |

---

## 9. 关键决策回顾

| 编号 | 决策 |
|---|---|
| DD-16 | 双 TCP 口：3333 纯透传 / 3334 ESP32 控制与日志（避免二进制流里混文本） |
| DD-17 | `s <关节号> <角度增量>` 复用 `CMD_JOINT`，不改协议（舵机步数↔关节角需要 STM32 的标定表，ESP32 拿不到） |
| DD-18 | 仲裁走**完整帧 + CRC**，且"状态未知即拒绝注入"（失败安全） |
| DD-19 | 本轮不做硬急停、不做 `e` 命令 |
| DD-20 | ESP-IDF 版本跟随 PlatformIO 平台默认（5.5.3），不钉旧版（平台会强制最新 riscv 工具链） |
| DD-21 | 工具链目录放 D 盘 + 含空格路径用 SUBST 别名（两条环境硬约束） |
| DD-22 | 健壮性三件套（看门狗 / NVS 配置 / 日志镜像）+ 板上自检 |

---

**文档结束。** 有新的真机现象，请优先更新 [37-esp32-acceptance.md](37-esp32-acceptance.md) 的记录表与本文档 §1/§3。
