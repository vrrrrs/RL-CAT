# RL-CAT ESP32-C3 无线桥固件

> **设计文档**：[`docs/32-esp32-bridge.md`](../../docs/32-esp32-bridge.md)（ESP32 的定位、决策、命令集、仲裁规则）
> **实施方案**：[`docs/36-esp32-plan.md`](../../docs/36-esp32-plan.md)（阶段划分、环境坑、验收单）
> **接口协议**：[`firmware/doc/协议规范_树莓派通信.md`](../doc/协议规范_树莓派通信.md)（与 STM32 完全共用，不新增帧格式）
> **参数表**：[`include/rlca_esp32_config.h`](include/rlca_esp32_config.h)（所有可调项集中于此，代码里无魔数）

---

## 1. 它在系统里干什么

```
阶段一（Pi 5 还没上）
   笔记本 ──WiFi(TCP)──→ ESP32 ──UART1(921600)──→ STM32 ──→ 12 个舵机
   ⭐ 整个 servo_bus 与标定流程可以在没有 Pi 5 的情况下调通

阶段二（Pi 5 接上后）
   Pi 5 ──UART1──→ STM32        唯一策略指令源
   ESP32 ──UART3──→ STM32       只读遥测 + 调试通道；Pi 在线时禁止注入关节指令
```

**明确不做**（见设计文档 §7）：不跑策略、不做完整遥测落盘、不做 BLE、**本轮不做硬急停**。

---

## 2. 两个 TCP 口

| 端口 | 用途 | 内容 |
|---|---|---|
| **3333** | **纯透传** | STM32 ↔ 笔记本 原始字节流（二进制协议帧），双向 |
| **3334** | **ESP32 控制/日志口** | 文本命令、ESP32 日志、链路统计（`stats`） |

**控制口命令**（P1 已有，P2/P3 补齐其余）：

```
?            帮助
k            心跳（回 ok）
stats        链路/桥接计数器
i            链路/遥测状态（不注入）
imu / m      ⭐ IMU 详情：四元数 / 陀螺 / 机身系重力向量（= 观测第 0~2 维）
selftest     板上自检（协议向量，6/6）
cfg          运行配置（pw / port / baud / clear）
raw <hex>    直接向 STM32 发任意字节（对打 PC 模拟器/示波器很有用）
hb           发一帧 HEARTBEAT（协议冒烟测试）
q            断开本连接
s <关节号> <角度增量>   z / c / r   ← 仲裁门控下的注入类命令
```

> ⚠️ 两处与设计文档 §4.1/§4.2 的差异（分口、`s` 用关节角增量而非舵机步数）理由见 [实施方案 §3.4/§3.5](../../docs/36-esp32-plan.md)。

---

## 3. 构建 / 烧写 / 监视

**⚠️ 必须通过 [`../tools/pio.ps1`](../tools/pio.ps1) 调用 pio**，它负责三件事：

| 动作 | 原因 |
|---|---|
| `PLATFORMIO_CORE_DIR = D:\Work\EspIDF\core` | ① 默认 `%USERPROFILE%\.platformio` 在沙箱下不可写；② **IDF 的 venv python 必须放在工作区之外**——工作区被 Windows 打上 **Low 完整性级别**，其中的 `python.exe` 一运行就是 Low 进程，**写不了 Medium 级别的 D 盘**（no-write-up），IDF 会以 `PermissionError [Errno 13]` 失败 |
| `PLATFORMIO_PACKAGES_DIR` / `PLATFORMS_DIR = D:\Work\EspIDF\…` | 框架 + 工具链约 1 GB，不放 C 盘也不放工程目录 |
| 仓库路径含空格时自动建 **SUBST 盘符别名**并在别名下构建 | ESP-IDF 明确拒绝含空格的工程路径（`-fmacro-prefix-map=C:/Users/<user name>` 会在用户名里的空格处把参数切碎）。别名 `R:` → 仓库根 |

```powershell
cd firmware/esp32
powershell -ExecutionPolicy Bypass -File ..\tools\pio.ps1 run            # 构建
powershell -ExecutionPolicy Bypass -File ..\tools\pio.ps1 run -t upload  # 烧写
powershell -ExecutionPolicy Bypass -File ..\tools\pio.ps1 device monitor # 看日志
```

**协议层与仲裁门控可以完全离线测**（不需要板子，也不需要 STM32）：

```powershell
# 51 项协议测试 + 11 项仲裁门控测试 + 与协议文档 §9 的 Python 实现逐字节对拍
powershell -ExecutionPolicy Bypass -File ..\tools\test_esp32_host.ps1
```

**上机验收工具**（[`tools/bench.py`](tools/bench.py)，两种入口任选）：

```powershell
# ⭐ 推荐：仓库根目录的 bench.cmd —— 不经 PowerShell，不受"禁止运行脚本"策略影响
"C:\...\RL-CAT\bench.cmd" doctor                                    # 体检：WiFi/端口/UART 到底哪不对
"C:\...\RL-CAT\bench.cmd" selftest                                  # 工具自检（无需任何硬件）
"C:\...\RL-CAT\bench.cmd" con --host 192.168.4.1 --port 3334        # ESP32 控制口（交互）
"C:\...\RL-CAT\bench.cmd" con --host 192.168.4.1 --port 3333        # STM32 shell（交互，会混二进制遥测）
"C:\...\RL-CAT\bench.cmd" verify --loopback --report D:\acc.json    # ⭐ 一条命令跑完验收自动项

# 等价写法（若坚持用 ps1，必须带 -ExecutionPolicy Bypass）
powershell -ExecutionPolicy Bypass -File ..\tools\bench.ps1 doctor
```

**⭐ `verify`：一条命令跑完验收单的自动项。** 只需一根 GPIO6↔GPIO7 跳线，不需要 STM32、不需要 USB-TTL：

```
原理：跳线让 ESP32 发出的字节回到它自己的接收端，
      于是控制口的 `raw <hex>` 能直接喂给它一帧 TEL_FAST（= STM32 会发的字节），
      遥测 / 仲裁门控 / 指令注入就都能在真机上验证。
覆盖：A4 A5 A10 A11 A12 A13 A14 A15 A16 A18 A19
```

**控制台走芯片自带的 USB-Serial-JTAG**（见 `sdkconfig.defaults`），一根 USB 线即可烧写 + 看日志；`UART0/UART1` 都空出来给 STM32 链路。

**逐条验收步骤见 [`docs/37-esp32-acceptance.md`](../../docs/37-esp32-acceptance.md)。**

---

## 4. 本机实测的板子信息

| 项 | 值 |
|---|---|
| 芯片 | ESP32-C3 (QFN32) **rev v0.4** |
| Flash | **4 MB 内嵌（XMC）**，晶振 40 MHz |
| USB | **原生 USB-Serial-JTAG**（VID:PID `303A:1001`），非 CH343 |
| MAC | `e0:72:a1:22:e9:58` |
| PlatformIO board | `esp32-c3-devkitm-1`（ESP32-C3-MINI-1 一类模块） |
| 出厂状态 | Flash 全 `0xFF` → ROM 刷屏 `invalid header` + 看门狗复位（每约 2.4 s 重新枚举 USB）。**不是硬件故障，烧固件即可**（已烧写，验收 A1 通过） |
| **AP SSID** | **`RL-CAT-E959`** —— 取的是 **SoftAP MAC**（= 出厂 MAC 末字节 +1），不是 esptool 读到的 `...e9:58` |

**⚠️ 引脚约束（C3 + 内嵌 Flash）：**

| GPIO | 状态 |
|---|---|
| 11–17 | ⛔ 被内嵌 Flash 占用（模块不引出） |
| 18 / 19 | ⛔ USB D- / D+ |
| 2 / 8 / 9 | ⚠️ strapping（9 = BOOT），不要用作普通 IO |
| **6 / 7** | ✅ 本项目用作 UART1：TX=GPIO6 → STM32 PB11，RX=GPIO7 ← STM32 PB10 |
| 0–5, 10, 20, 21 | ✅ 可用（20/21 是 UART0 默认脚，本项目留空） |

接线：**交叉**（ESP32 TX → PB11，ESP32 RX ← PB10），共地；与 USART1↔Pi 相同，只有 USART2↔舵机控制板是直连。

---

## 5. 目录结构

```
firmware/esp32/
├── platformio.ini               board=esp32-c3-devkitm-1 / framework=espidf / build_dir=D:
├── sdkconfig.defaults           控制台=USB-Serial-JTAG、4 MB Flash、1 kHz tick
├── partitions.csv               2 MB factory app（默认 1 MB 太紧）
├── include/
│   ├── rlca_esp32_config.h      ⭐ 全部参数（WiFi/端口/波特率/超时/协议偏移/命令集/看门狗）
│   ├── rlca_protocol.h          帧状态机 + CRC16 + TEL_FAST/TEL_STATUS 访问器
│   ├── rlca_arbiter.h           仲裁门控（纯函数，主机侧可测）
│   ├── rlca_cfg.h               NVS 配置 API + rlca_cfg_validate.h（纯校验，主机侧可测）
│   ├── rlca_net.h               AP + 两个 TCP 服务
│   ├── rlca_bridge.h            UART1 ↔ TCP 字节泵
│   ├── rlca_injector.h          文本命令 → 标准帧
│   ├── rlca_selftest.h          板上自检（跑主机侧同一套向量）
│   └── rlca_logmirror.h         ESP_LOG → 3334 控制口
├── src/                         对应实现 + main.c
├── tools/
│   ├── bench.py                 上机验收工具（selftest/loop/mon/sim/tcp/con）
│   └── rlca_proto.py            帧编解码与美化打印（bench 用）
└── test/
    ├── test_protocol.c          协议单元测试（协议规范 §10 全 10 项 + 扩展，51 项）
    ├── test_arbiter_gate.c      仲裁门控单元测试（11 项）
    ├── test_cfg.c               配置校验单元测试（18 项）
    └── reference_frames.py      协议文档 §9 的参考实现（逐字节对拍用）
```

---

## 6. 进度

| 阶段 | 内容 | 状态 |
|---|---|---|
| **P0** | 工具链 + 工程骨架 + 最小固件 | ✅ **上机通过**（烧写成功；自检 6/6；启动日志 0 错误；USB 稳定不跳） |
| **P1** | `wifi_ap`（3333/3334）+ `uart_bridge` 双向透传 | ✅ 编译通过；⏳ 待上机验收 |
| **P2** | `protocol`（帧/CRC）+ `injector`（`i/s/z/c/r` 全命令集） | ✅ 代码完成，51 项协议测试 + Python 对拍全绿；⏳ 待上机 |
| **P3** | `arbiter`（`status`/`cycle_cnt`/电压 + 注入门控） | ✅ 代码完成，11 项门控测试全绿；⏳ 待上机 |
| **P4** | 看门狗 + NVS 配置（`cfg`）+ 日志镜像 + 开机自检（`selftest`） | ✅ 代码完成，18 项配置校验测试全绿；⏳ 待上机 |
| P5 | 文档回填 + 上机验收单 | ✅ 文档完成（[`36-esp32-plan.md`](../../docs/36-esp32-plan.md)、[`37-esp32-acceptance.md`](../../docs/37-esp32-acceptance.md)、DD-16…DD-22）；⏳ 验收单待执行 |
| P4 | 控制口完善、看门狗、NVS 配置、健壮性 | ⬜ |
| P5 | 文档回填 + 上机验收单 | 🟡 进行中 |

**构建产物**（P1）：Flash 831 KB、RAM 30.3 KB。 ⚠️ PlatformIO 的 `Flash: xx%` 仍按 IDF 默认的 1 MB 算（显示 79%），**实际 app 分区是 2 MB**（`partitions.csv`）。
