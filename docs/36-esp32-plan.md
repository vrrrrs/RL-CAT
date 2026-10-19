# ESP32-C3 固件实施方案（Wireless Bridge Implementation Plan）

> **文档编号**：DOC-ESP-PLAN-001 ｜ **版本**：v1.0 ｜ **日期**：2026-10-10
> **面向对象**：接手 ESP32 固件开发的 Agent / 开发者 + 项目负责人
> **状态**：🟡 **P0 进行中**（方案已由负责人确认）
>
> **本文档是「怎么做」的施工方案**，不替代设计规格：
> 规格见 [`32-esp32-bridge.md`](32-esp32-bridge.md)（定位/决策/命令集/仲裁）、
> [`30-firmware.md`](30-firmware.md)（接口表）、
> [`../firmware/doc/协议规范_树莓派通信.md`](../firmware/doc/协议规范_树莓派通信.md)（帧格式，ESP32 与 STM32 共用）。

---

## 0. 本次接手的边界（已与负责人确认）

| 项 | 结论 |
|---|---|
| 范围 | **仅 ESP32 固件** + PC 侧测试工具。Pi 侧程序、STM32 固件不由本方案负责 |
| 固件框架 | **ESP-IDF**（自写透传，不用 ESP-AT，也不用 Arduino） |
| 调试口 | **3333 纯透传 + 3334 ESP32 控制/日志口**（偏离设计文档 §4.1 的单口方案，见 §3.4） |
| 单舵机命令 | **`s <关节号> <角度增量>`**，复用 `CMD_JOINT`，**不改协议**（偏离 §4.2 的 `s <id> <pos>`，见 §3.5） |
| **硬急停** | ⛔ **本轮完全不做**：不做 `safety.c`、不做急停 GPIO、命令集去掉 `e` |
| STM32 侧 USART3 支持 | ⚠️ **不在本次范围**，由本方案给出契约（§2），负责人转交 STM32 侧执行 |
| 舵机 ID / 标定 / 伺服总线 | 属 STM32 侧，不在本次范围 |

---

## 1. 现状实测（不是照抄文档）

| 项 | 实测结论 |
|---|---|
| **板子** | ESP32-C3（QFN32）**rev v0.4**，**4 MB 内嵌 Flash（XMC，device 0x4016）**，晶振 40 MHz |
| **USB** | **原生 USB-Serial-JTAG**（VID:PID `303A:1001`），非 CH343，无需额外驱动 |
| **MAC** | `e0:72:a1:22:e9:58`（用于 AP SSID 后缀） |
| **出厂状态** | Flash 全 `0xFF` → ROM 刷屏 `invalid header` → `TG0WDT_SYS_RST` 每约 2.4 s 复位 → USB 反复枚举（设备管理器图标"跳"）。**芯片与 Flash 均正常，烧固件即可** |
| 板型判定 | 内嵌 4 MB + 原生 USB + 40 MHz ⇒ ESP32-C3-MINI-1 一类模块 ⇒ `board = esp32-c3-devkitm-1` |
| **引脚约束** | GPIO11–17 被内嵌 Flash 占用、18/19 = USB D±、2/8/9 = strapping；本固件用 **GPIO6 = UART1 TX、GPIO7 = UART1 RX** |
| 控制台 | 走 **USB-Serial-JTAG**（`sdkconfig.defaults`），UART0/UART1 全部留给 STM32 链路 |
| 工具链 | PlatformIO **6.1.19** + `platform-espressif32@6.13.0` + **`framework-espidf` 3.50503.0（IDF 5.5.3）** + `toolchain-riscv32-esp 14.2.0` |
| **PIO 目录** | 一律走 `firmware/tools/pio.ps1`：core=`D:\Work\EspIDF\core`、packages/platforms=`D:\Work\EspIDF\`、build=`D:\Work\EspIDF\build`（原因见 §4.1，**这四条都是踩出来的**） |
| **构建结果** | P1：**0 error / 0 warning**，Flash 831 KB、RAM 30.3 KB，app 分区 2 MB（`partitions.csv`） |
| 协议层验证 | 主机侧单元测试 **51 项全绿** + 与协议文档 §9 的 Python 实现**逐字节对拍一致**（`firmware/tools/test_esp32_host.ps1`，不需要板子） |
| STM32 侧现状 | 业务代码仍为 0 行；[`33-firmware-plan.md`](33-firmware-plan.md) §2 明确 **"USART3 转发 TEL_FAST：暂不处理，P4 再定"** |

---

## 2. ⭐ ESP32 ↔ STM32 的 USART3 接口契约（草案 v0.1，**待 STM32 侧确认**）

**⚠️ 这段可直接转交给 STM32 侧。ESP32 侧的所有验收都依赖它。**

```
【ESP32 ↔ STM32 USART3 接口契约 v0.1】

1. 帧格式：完全复用《协议规范_树莓派通信.md》，不新增、不修改任何字段
   （AA 55 / LEN / SEQ / TYPE / PAYLOAD / CRC16-CCITT-FALSE，LEN≤128，CRC 覆盖 LEN..PAYLOAD）

2. STM32 在 USART3 上【接收】与 USART1 同类型的下行帧：
   CMD_JOINT(0x01)、CMD_MODE(0x03)、CMD_CALIB(0x05)、HEARTBEAT(0x7F)
   —— 与 USART1 可共用同一套解析器（两个独立实例即可），下行同样过
      模式白名单 / NaN / 限幅 / 限速（协议规范 §8.3）。

3. STM32 在 USART3 上【发送】：
   · TEL_FAST(0x02) 100 Hz 镜像（编译开关 RLCA_ESP32_MIRROR_EN，阶段一默认开）
   · 最低要求：TEL_STATUS(0x04) ≥10 Hz（ESP32 的仲裁依赖它）
   · TEL_ACK(0x07) / TEL_LOG(0x06) 直通
   —— 带宽核算（2026-10-10 修订为 115200 + 10 Hz）：103 B × 10 Hz = 1.03 kB/s，加上 TEL_STATUS 47 B × 10 Hz，合计约 1.5 kB/s ≈ 115200 的 13%。

4. ⚠️ STAT_LINK_OK(BIT2) 只表示【USART1 = Pi】近期有帧。
   不得被 USART3（ESP32）自己的帧影响。
   否则：ESP32 注入 → LINK_OK 置位 → ESP32 按仲裁规则丢弃自己的注入 ⇒ 自锁。

5. ESP32 侧静默/断连不改变 STM32 的任何状态（ESP32 断连 ≠ Pi 离线）。
   反向同理：WiFi 断开时 ESP32 也不接管控制。

6. USART3 配置：**115200**（2026-10-10 已改）、NVIC 抢占 8、RX 环缓 ≥256 B。
```

**ESP32 侧对契约的依赖点：**

| ESP32 功能 | 依赖契约的哪一条 |
|---|---|
| 阶段一无线调试（TCP 收发的帧直达 STM32） | 2 |
| 笔记本能看到 STM32 遥测（验收②） | 3 |
| 指令仲裁（Pi 在线时禁止注入） | 3 + **4** |
| `s` 命令填充其余 11 关节的当前 `q` | 3（需要 `q` 遥测） |

### 2.1 ⭐ STM32 侧镜像的实现要求（负责人 2026-10-10 定：低频调试口，115200）

**背景**：ESP32 链路定位为**低频调试/观察通道**，波特率定为 **115200**（跳线可靠优先于带宽）。
**⚠️ 由此产生两条硬约束：**

| # | 约束 | 原因 |
|---|---|---|
| **① 必须用中断/DMA 发送，不能阻塞** | `bsp_uart_write()` 现在是阻塞式 `HAL_UART_Transmit`。103 B 在 115200 下要 **8.9 ms**，在 921600 下也要 1.12 ms —— 写在 100 Hz 周期里会把控制环拖慢（上机实测：921600 阻塞镜像已把周期拉到 ~10.8 ms ≈ **92 Hz**） | 需改成 `HAL_UART_Transmit_IT` / `HAL_UART_Transmit_DMA`，或用独立缓冲 + 空闲时发送 |
| **② 必须降频**（建议 10 Hz） | 115200 的线速率约 11.5 kB/s；若 100 Hz 全量镜像 TEL_FAST（10.3 kB/s）会占 **90%** 且必然丢帧。10 Hz（TEL_FAST 103 B + TEL_STATUS 47 B = 1.5 kB/s）只占 **13%**，余量充足 | ESP32 的仲裁过期阈值是 500 ms，10 Hz（100 ms）完全够用 |

**改动位置（一处，最小 diff）**：`firmware/stm32/Core/Src/link_pi.c` 的 `send_frame()` —— 组好帧后把**同一份字节**再送一份给 ESP32：

```c
static bool send_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    if (s_port == NULL) { return false; }
    const uint16_t n = rlca_frame_pack(s_tx_buf, sizeof(s_tx_buf), type, s_tx_seq, payload, len);
    if (n == 0u) { return false; }
    s_tx_seq++;

    const bool ok = s_port->write(s_tx_buf, n);      /* USART1 -> 树莓派 */

#if RLCA_ESP32_MIRROR_EN                                 /* 建议加一个编译开关 */
    link_esp_mirror(s_tx_buf, n, type);                  /* USART3 -> ESP32，非阻塞 */
#endif
    return ok;
}
```

**推荐做法（把降频和非阻塞放在一起）**：

```c
/* link_esp.c —— 只镜像观察类帧，且限频 + 中断发送 */
static uint8_t s_esp_tx[128];
static volatile bool s_esp_busy;

void link_esp_mirror(const uint8_t *frame, uint16_t n, uint8_t type)
{
    if (s_esp_busy) { return; }                 /* 上一帧没发完就丢，绝不等待 */
    if (type != RLCA_TYPE_TEL_FAST && type != RLCA_TYPE_TEL_STATUS) { return; }

    static uint8_t decim;
    if (type == RLCA_TYPE_TEL_FAST && (++decim % 10u) != 0u) { return; }  /* 100 Hz -> 10 Hz */

    memcpy(s_esp_tx, frame, n);
    s_esp_busy = true;
    (void)HAL_UART_Transmit_IT(&huart3, s_esp_tx, n);                     /* 不阻塞 */
}
/* USART3 的 TxCpltCallback 里把 s_esp_busy 清 0 */
```

**⚠️ 不要改的**：`STAT_LINK_OK` 必须继续只反映 USART1/Pi（现在 `app_main.c:110` 用 `link_pi_is_alive()` 是正确的）。若改成"任意链路活着"，ESP32 自己的注入会把该位置 1 → **ESP32 仲裁自锁**。

**接线**：ESP32 的 **GPIO21(TX) / GPIO20(RX)** ↔ STM32 的 **PB11(USART3_RX) / PB10(USART3_TX)**（交叉）+ 共地（按 PCB 走线，不是 GPIO6/7）。

**速率与带宽预算（115200）**

```
TEL_FAST   103 B @ 10 Hz = 1030 B/s
TEL_STATUS  47 B @ 10 Hz =  470 B/s
──────────────────────────────────
合计 1500 B/s ≈ 115200 线速率的 13%   ✅ 余量充足
（若真需要 100 Hz：请改用 DMA 并把波特率提到 460800 以上）
```

**⚠️ 另注：`bsp_uart_write` 是阻塞式**（`bsp_uart.c` 里就是 `HAL_UART_Transmit`），既有代码也在用；只要镜像走 IT/DMA，就不必动它。

### 2.2 契约现状核对（2026-10-10 直接读 STM32 源码 + 真机实测）

| 契约条目 | STM32 侧现状 | 结论 |
|---|---|---|
| 1 帧格式一致 | ✅ 完全一致（`rlca_frame.c` / `rlca_crc16.c`，测试向量 0x29B1） | 满足 |
| 2 USART3 接收下行帧 | ❌ **只收不解析**：`g_uart_esp` 在 `bsp_uart.c` 里正确装了中断 + 环形缓冲 + 溢出计数，但**全工程没有任何消费者**（只有 shell 打印 `esp_total/esp_overflow/esp_errors` 三个计数） | ⛔ **未实现**，需 STM32 侧补 |
| 3 USART3 发送遥测 | ❌ **从不发送**：STM32 只在 USART1（`g_uart_pi`）上发 TEL_FAST/TEL_STATUS，shell 也挂在 USART1 | ⛔ **未实现**，需 STM32 侧补 |
| 4 `STAT_LINK_OK` 只表示 Pi 在线 | ✅ **已经正确**：`app_main.c:110` 用 `link_pi_is_alive(now)`（USART1/Pi 链路模块）置位，与 USART3 无关 | 满足（**自锁风险已排除**） |
| 5 ESP32 静默不改变 STM32 状态 | ✅ 现状天然满足（USART3 数据根本不进任何状态机）；⚠️ 实现第 2 条时要保持 | 待保持 |
| 6 USART3 物理配置 | ✅ **115200**（2026-10-10 已改）+ 中断收 + 环缓；发送走 `HAL_UART_Transmit_IT`（`esp_fwd.c`）| 满足 |

**⚠️ 由此得到的结论：**
**ESP32 的"阶段一无线调试"目前【打不通】** —— 字节能送到 STM32 的 USART3 环形缓冲里，但 STM32 不会处理，也不会回任何东西。

**📌 2026-10-10 更新（已在真机上打通 TX 方向）**：STM32 侧已实现镜像（`TX3/RX3 ↔ ESP32 GPIO20/21`）。ESP32 侧实测：
`first 32 bytes on UART1: AA 55 60 6C 02 …`（正确的 TEL_FAST 帧）→ **字节确实到了**；
但 3.7% 帧干净、CRC 大量错、`pe/fe/ovf/bf` 全 0 → **比特级损坏，属电气信号完整性**（ESP32 芯片内部整帧回环 20/20 全对，已排除 ESP32 侧）。
且 `rx≈9.5 kB/s` 而非 10.3 kB/s → 阻塞镜像把 100 Hz 环拖到 ~92 Hz。
⇒ 决定改为**低频调试通道：115200 + 10 Hz + 非阻塞发送**，实现要求见 §2.1。
因此：
- **P1/P2 的验收只能对着 PC 端 STM32 模拟器做**（或用跳线做 UART loopback 自测），不能对着真 STM32；
- 契约的第 2、3 条必须由 STM32 侧补齐后，P3 的仲裁才有真实输入。
- 另注：STM32 侧已经有一个**挂在 USART1 上的调试 shell**（`bsp_shell.c`，含 `C start/C zero` 标定命令）。若把 shell 复用到 USART3，阶段一的无线调试价值会立刻兑现 —— 这是可选的低成本路径。

---

## 3. 模块架构与关键取舍

```
firmware/esp32/
├── platformio.ini               esp32-c3-devkitm-1 + framework = espidf + build_dir=D:
├── sdkconfig.defaults           控制台走 USB-Serial-JTAG、4 MB Flash、1 kHz tick
├── partitions.csv               app 分区 2 MB（默认 1 MB 太紧）
├── include/rlca_esp32_config.h  ⭐ 全部参数集中（WiFi/端口/波特率/超时/偏移/命令集）
├── include/{rlca_protocol,rlca_net,rlca_bridge,rlca_injector}.h
├── src/
│   ├── main.c                   初始化 + 任务创建 + 心跳日志
│   ├── wifi_ap.c                AP + 两个 TCP 口 + 3334 命令解析（含 ?/k/stats/q）
│   ├── uart_bridge.c            UART1 ↔ TCP 双向泵（环缓 + 溢出计数 + 字节 tap）
│   ├── protocol.c               帧状态机 + CRC16/CCITT-FALSE（可离线单测）
│   └── injector.c               文本命令 → 标准帧（P1: raw/hb；P2: i/s/z/c/r）
└── test/
    ├── test_protocol.c          主机侧单元测试（协议规范 §10 全 10 项 + 扩展）
    └── reference_frames.py      协议文档 §9 的参考实现（逐字节对拍）
```

**⚠️ 与初版计划的差异**：`console.c` 未单独成文件，控制口逻辑合并进 `wifi_ap.c`（socket 表与命令处理必须共享同一把锁/同一张客户端表）；`arbiter.c` 在 P3 落地时新增，届时通过 `rlca_bridge_set_tap()` 挂在字节流上，不做第二次读取。

**5 条关键取舍（会记进 [`90-decisions.md`](90-decisions.md)）：**

| # | 取舍 | 理由 |
|---|---|---|
| 1 | **仲裁按完整帧解析 + CRC 校验**，不是设计文档 §4.3 的"只读偏移 94 一个字节" | `status` 在**帧偏移 99**（=5+94）；不做 CRC 时一个位翻转就可能误判"Pi 在线"而吞掉调试指令。96 B × 100 Hz 对 C3 是零压力 |
| 2 | **失败安全：Pi 状态未知 ⇒ 禁止注入**（`RLCA_ARB_BLOCK_ON_UNKNOWN`） | 宁可拒绝调试指令，也不要"以为 Pi 离线"而抢控制权（设计文档 §6 第一条事故场景） |
| 3 | **遥测整形（镜像哪些帧）由 STM32 侧决定**，ESP32 只做透传 | ESP32 保持"哑管道 + 状态解析"的中间形态，不引入额外协议 |
| 4 | **3333/3334 双口**（见 §3.4） | 3333 永远是干净二进制管道，客户端不必解帧就能区分 ESP32 文本 |
| 5 | **`s` 用关节角增量**（见 §3.5） | 舵机步数↔关节角要用 STM32 的标定表，ESP32 拿不到；增量表达在未标定时也安全可用，且不改协议 |

### 3.4 为什么拆两个 TCP 口

设计文档 §4.1 的方案是"单口 3333，非 `0xAA55` 开头当本地命令"。问题：ESP32 自己的日志/状态也会混进这条流，客户端必须**解帧**才能区分二进制帧与文本。拆成 3333（纯透传）+ 3334（控制/日志）后，两边都变简单。**代价**：调试时需要开两个 TCP 连接（或一个 TCP 工具 + 一个 netcat）。

### 3.5 为什么 `s` 是关节角增量而不是舵机步数

设计文档 §4.2 写的是 `s <id> <pos>`（`2048` = 舵机中位）。但 `joint_to_servo()` 需要 `dir` / `offset`（STM32 的标定表），ESP32 拿不到；要么改协议加一条 `CMD_SERVO_RAW(0x0A)`（负责人已否决改协议），要么换表达方式：

```
s <关节号 0..11> <角度增量 deg>       例：s 4 +10   → 4 号关节（lf_thigh）在当前基础上 +10°
实现：读最近一帧 TEL_FAST 的 q[12] → 目标 = q，仅覆盖 idx 分量 → 组 CMD_JOINT(mode=ACTIVE, flags=ENABLE)
```

**优点**：不需要新增协议消息；其余 11 个关节保持当前姿态（不会因为填 0 而乱动）；`dir`/`offset` 的误差在增量里自然抵消，**未标定也能用**。
**代价**：不能指定绝对舵机步数（那属于 STM32 侧调试命令的范畴）。

---

## 4. 工具链与构建

**⚠️ 一律通过 `firmware/tools/pio.ps1` 调用 pio**（它设置下面几个变量，并处理盘符别名）：

| 变量 / 动作 | 值 | 为什么 |
|---|---|---|
| `PLATFORMIO_CORE_DIR` | `D:\Work\EspIDF\core` | 见 §4.1 ②：**IDF 的 venv python 必须在工作区之外** |
| `PLATFORMIO_PACKAGES_DIR` | `D:\Work\EspIDF\packages` | 框架 + 工具链约 1 GB，不占 C 盘/工程目录 |
| `PLATFORMIO_PLATFORMS_DIR` | `D:\Work\EspIDF\platforms` | 同上 |
| `build_dir`（platformio.ini） | `D:/Work/EspIDF/build` | 见 §4.1 ③：**构建目录也不能含空格** |
| 自动建 **SUBST 盘符**（仓库路径含空格时） | `R:` → 仓库根 | 见 §4.1 ③ |

```powershell
cd firmware/esp32
powershell -ExecutionPolicy Bypass -File ..\tools\pio.ps1 run
powershell -ExecutionPolicy Bypass -File ..\tools\pio.ps1 run -t upload
powershell -ExecutionPolicy Bypass -File ..\tools\pio.ps1 device monitor
```

### 4.1 ⭐ 环境踩坑记录（4 条，都花了时间，写下来免得重踩）

| # | 现象 | 根因 | 处理 |
|---|---|---|---|
| **①** | `pio` 直接报 `HomeDirPermissionsError`，`~/.platformio` 不可写 | 默认 core 目录在用户主目录，会话沙箱不允许写 | core 目录重定向到 D 盘 |
| **②** | CMake 配置阶段 `PermissionError: [Errno 13]`，写 `component_manager_run_counter.*` 失败；但同目录下**别的进程能写**、`os.access` 还报 W_OK | **DSH 沙箱给工作区打了 Low 完整性级别**。工作区内的 `python.exe` 一运行就是 **Low IL 进程**，而 `D:\` 是 Medium IL —— Windows 的 no-write-up 策略直接 EACCES。ESP-IDF 的 CMake 恰好调用工作区里那份 **venv python** | 把 `PLATFORMIO_CORE_DIR` 也搬到 D 盘（venv python 变为 Medium IL）。见 [README §3](../firmware/esp32/README.md) |
| **③** | `Error: Detected a whitespace character in project paths.`；随后 `cc1.exe: error: invalid argument 'C:/Users/Liu' to '-fmacro-prefix-map'` | ESP-IDF 明确不支持含空格的工程路径，而仓库在 `C:\Users\<user name>\...` | `build_dir` 指向无空格路径 + wrapper 自动建 **SUBST 别名 `R:`** 并在别名下构建（用 SUBST 而非目录 junction，因为 junction 会被解析回真实路径，空格又回来了） |
| **④** | PlatformIO 在 IDF venv 上跑 `pip install -U` 时 CPU 打满、长时间不返回（还会因残留 venv 出现两个 pip 抢锁） | pip 解析器回溯 + 被 kill 的运行留下半个 venv | 平台自带开关：框架目录下建 `.pio_skip_pypackages` 跳过该步骤；venv 已装好依赖并写入标记 `pio-idf-venv.json`。**另**：`idf_component_manager` 的 run-counter 在本机必失败，已给该文件加 `try/except OSError` 容错补丁（该文件本是可选计数器） |
| **⑤** | 芯片里 app 分区只有 **1 MB**（`00100000`），项目里的 2 MB `partitions.csv` 被无视 | PlatformIO 的 IDF 集成**自己生成分区表**（[`espidf.py:1528`] `board.get("build.partitions", "partitions_singleapp.csv")`），完全忽略 `CONFIG_PARTITION_TABLE_CUSTOM` | `platformio.ini` 加 `board_build.partitions = partitions.csv`，并删掉构建目录里的旧 `partitions.bin`。**已修复并上机确认（`factory 00200000`）** |
| **⑥** | 每次启动都有一行红字 `E task_wdt: esp_task_wdt_init(517): TWDT already initialized` | `sdkconfig.defaults` 里写的 `CONFIG_ESP_TASK_WDT=y` **不是 IDF 5.x 的选项名**（正确是 `ESP_TASK_WDT_EN` / `ESP_TASK_WDT_INIT`），于是 IDF 自己先初始化了看门狗，代码里的 init 撞车 | 改成 `CONFIG_ESP_TASK_WDT_EN=y` + `CONFIG_ESP_TASK_WDT_INIT=n`（由 `main.c` 自己按 `RLCA_TWDT_*` 初始化）。**已修复并上机确认（0 错误行）** |

**⚠️ 上述 ②④ 两条是对工具链（D 盘上的 IDF venv）的本地适配，不在工程版本控制里**；换机器/重装框架后需重做。重建步骤写在 [`firmware/esp32/README.md`](../firmware/esp32/README.md) §3。

**⚠️ 一旦这些适配被破坏，症状是"构建突然失败但代码没改"** —— 先按 §4.1 逐条对照。

### 4.2 构建产物与规模

| 指标 | P0 最小固件 | P1（AP + 双口 + 透传） |
|---|---|---|
| Flash | 155 KB | **831 KB** |
| RAM | 9.6 KB | **30.3 KB** |
| app 分区 | — | **2 MB**（`partitions.csv`，默认 1 MB 太紧） |

**⚠️ PlatformIO 打印的 `Flash: xx%` 仍按 IDF 默认的 1 MB 计算**（故显示 79%），实际可用 2 MB。

---

## 5. 阶段与交付物

| 阶段 | 内容 | 交付 / 验收 | 依赖硬件 |
|---|---|---|---|
| **P0** | 工具链重定向、工程骨架、参数表、最小固件（板卡信息 + 心跳日志） | 构建通过；烧写后 **USB 不再反复枚举**、`device monitor` 看到日志 | ✅ 烧写 |
| **P1** | `wifi_ap`（AP + 3333/3334 双口）+ `uart_bridge`（双向、环形缓冲、计数、非阻塞广播） | 笔记本连 AP：TCP↔UART 任意字节可达；UART 侧 1 MB 随机流零丢失；loopback 自测 | ✅ |
| **P2** | `protocol`（帧状态机 + CRC，测试向量 `0x29B1`）+ `injector`（`? i s z c r k q`，限速） | PC 侧**逐字节对拍**（与协议文档 §9 的 `pack()` 一致）；上机 `s 4 10` 单关节微动 | ✅ |
| **P3** | `arbiter`：解析 TEL_FAST/TEL_STATUS（`status`/`cycle_cnt`/电压/温度）→ 门控注入 | 模拟器置 `LINK_OK=1` ⇒ 注入被丢弃并计数；置 0 ⇒ 放行；状态过期 ⇒ 保守拒绝 | ✅ |
| **P4** | 控制口完善（统计/客户端列表/在线状态）、看门狗、拥塞丢弃策略、NVS 配置 | 100 Hz 双向连续 30 min 无溢出；WiFi 断开重连不扰动 STM32 | ✅ |
| **P5** | 文档回填 + 上机验收单 + 契约转交 | 文档与代码一致，验收单可逐条执行 | ❌ |

**并行性**：P1 与 P2 的 `protocol.c` 可并行（一个调网络、一个做纯逻辑单测）。**P0 必须先做完。**

### 5.1 当前进度（截至 2026-10-10）

| 阶段 | 状态 | 证据 |
|---|---|---|
| **P0** | ✅ **上机通过（A1/A2/A3）** | 烧写 `Hash of data verified.`；启动日志 `cfg 0.4.0-p4`、**`selftest: 6/6`**、0 错误/告警行；60 s 空载 heap 稳定、计数全 0；端口不再反复枚举 |
| **P1** | 🟡 编译 ✅ / 上机 ⏳ | `wifi_ap` + `uart_bridge` 写完并编译通过（A4/A5 待执行：需切到 `RL-CAT-E959` 与一根 GPIO6↔GPIO7 跳线） |
| **P2** | 🟡 代码 ✅ / 上机 ⏳ | `protocol.c` + `injector.c`（`i/s/z/c/r` 全部落地）；**51 项协议测试 + 与协议文档 §9 的 Python 实现逐字节对拍全绿** |
| **P3** | 🟡 代码 ✅ / 上机 ⏳ | `arbiter.c`（完整帧 + CRC + `cycle_cnt` 连续性 + 状态过期）；**门控策略 11 项主机侧测试全绿** |
| **P4** | 🟡 代码 ✅ / 上机 ⏳ | 任务看门狗（3 个任务喂狗，超时 5 s）、NVS 配置（`cfg` 改密码/端口并持久化）、日志镜像到 3334、开机自检（`selftest` 6/6）；**配置校验 18 项主机侧测试全绿** |

**交付物现状**：

| 类别 | 位置 |
|---|---|
| 固件 | `firmware/esp32/{platformio.ini, sdkconfig.defaults, partitions.csv, include/, src/}`（8 个源文件 + 8 个头文件） |
| 主机侧测试 | `firmware/esp32/test/{test_protocol.c, test_arbiter_gate.c, test_cfg.c, reference_frames.py}` + `firmware/tools/test_esp32_host.ps1`（**80 项全绿**） |
| 上机工具 | `firmware/esp32/tools/{bench.py, rlca_proto.py}` + `firmware/tools/bench.ps1`（`selftest` 已通过，见 §6） |
| 构建入口 | `firmware/tools/pio.ps1` |
| 文档 | 本文档 + [`37-esp32-acceptance.md`](37-esp32-acceptance.md) + [`firmware/esp32/README.md`](../firmware/esp32/README.md) + [`90-decisions.md`](90-decisions.md) DD-16…DD-22 |

---

## 6. 离线验证策略（无 STM32 时如何保证质量）

| 手段 | 判据 |
|---|---|
| **① 构建门禁** | `pio run` **0 error 0 warning** + Flash/RAM 占用报告 |
| **② PC 纯逻辑单测**（MinGW gcc） | `test_protocol.c`：**51 项全绿**（协议规范 §10 全 10 项 + CRC 表 vs 位运算 2 万组随机 + TEL_FAST 偏移/`status` 在帧偏移 99 + `CMD_JOINT` 组帧）；`test_arbiter_gate.c`：**11 项全绿**（无遥测/新鲜/过期/边界/Pi 在线互斥/其它状态位不干扰）；`test_cfg.c`：**18 项全绿**（WPA2 密码 8..63 可打印字符、端口 1024..65534） |
| **③ 与协议文档 §9 的 Python 实现逐字节对拍** | HEARTBEAT / MAXPAYLOAD / CMD_JOINT / CMD_MODE / CMD_CALIB **5/5 一致**（`firmware/tools/test_esp32_host.ps1` 全自动） |
| **④ PC 端 STM32 模拟器**（`bench.py sim`，Python + pyserial） | 100 Hz 发 TEL_FAST（`--link-ok` 可控、电压/`cycle_cnt` 可控）；解码 ESP32 注入的 CMD 帧；`--serve` 可回 HEARTBEAT / REQ_STATUS。**不需要 STM32、只需要一个 USB-TTL** |
| **⑤ UART loopback 自测**（`bench.py loop`） | GPIO6↔GPIO7 短接：TCP 发 N 字节原样回环，验证 **TCP→UART→TCP 全链路**；判据零丢失、零错序。**不需要 STM32、不需要 USB-TTL** |
| **⑥ 工具自身自检**（`bench.py selftest`） | CRC 向量 / 状态机鲁棒性 / TEL_FAST 往返 / 与 oracle 对拍 / 本地 echo 服务器跑通 loop 逻辑 —— **已全绿** |
| **⑦ 一条命令跑完自动项**（`bench.py verify --loopback`） | 只要一根 GPIO6↔GPIO7 跳线：用控制口 `raw` 把 `TEL_FAST` 帧喂回 ESP32 自己的接收端，于是**遥测/仲裁/注入都能在真机上验证**。覆盖 A4 A5 A10~A16 A18 A19，并输出 JSON 报告 |
| **⑧ 上机验收单**（负责人执行） | 见 [`37-esp32-acceptance.md`](37-esp32-acceptance.md)：A1~A23 逐条「动作 / 预期现象 / 判据 / 失败分支」+ 现象对照表；**A1~A3 已由 ESP32 侧执行并通过** |

**⚠️ ②③④⑥ 全部不需要 STM32；④⑤⑥ 连板子都不需要。**

---

## 7. 需要负责人配合的动作

| 时机 | 动作 |
|---|---|
| P0 | 板子用数据线接电脑（已确认）；烧写后确认设备管理器不再闪 |
| P1 | 笔记本连 `RL-CAT-XXXX` 热点，用 TCP 工具连 3333/3334 |
| P1 起 | 把 ESP32 的 **GPIO6 → STM32 PB11、GPIO7 ← STM32 PB10、GND 共地**接好（交叉） |
| P2 起 | 若 STM32 侧尚未实现 USART3 收发，先用 PC 模拟器对打 |
| 任意 | 转交 §2 的契约给 STM32 侧，并把结论回传 |

---

## 8. 需要回填的文档（施工中顺手修）

| # | 位置 | 问题 |
|---|---|---|
| 1 | [`32-esp32-bridge.md`](32-esp32-bridge.md) §3 | 去掉 `safety.c`；补 `arbiter.c` / `console.c` |
| 2 | 同上 §4.1 | 单口方案 → 双口方案（3333/3334） |
| 3 | 同上 §4.2 | `s <id> <pos>` → `s <关节号> <角度增量>`；去掉 `e` |
| 4 | 同上 §4.3 | "只读偏移 94" → 完整帧 + CRC，`status` 在帧偏移 99 |
| 5 | 同上 §5 / §8 | 硬急停本轮不做（明确标注为"后续"） |
| 6 | 同上 §8 | 待确认项按 §1 实测结果逐条结清（供电能力仍建议上机实测） |
| 7 | [`30-firmware.md`](30-firmware.md) §4 | 急停输入引脚由"待定"改为"本轮不实现，预留" |
| 8 | [`20-electronics.md`](20-electronics.md) §4.1/§4.2 | 两个"阻塞级待确认"按 [`33-firmware-plan.md`](33-firmware-plan.md) §0 的结论结清（U6/U9 = 引出排针；供电仍待实测） |
| 9 | [`90-decisions.md`](90-decisions.md) | 追加 §3 的 5 条取舍为 DD-16… |
| 10 | 根 [`README.md`](../README.md) / [`docs/README.md`](README.md) | 进展表加 ESP32 行；索引加本文档 |

---

## 9. 风险登记

| # | 风险 | 后果 | 对策 |
|---|---|---|---|
| 1 | STM32 侧不实现 USART3 收发 | ESP32 只能对着模拟器跑，验收②③无法完成 | §2 契约尽早转交；P1/P2 用模拟器并行推进 |
| 2 | `STAT_LINK_OK` 语义实现错（含 ESP32 自己的帧） | **ESP32 仲裁自锁**，调试指令全被丢弃 | 契约第 4 条明确；P3 用模拟器构造该场景做回归 |
| 3 | 115200 下 TCP 拥塞 | UART→WiFi 方向积压、丢数据 | 每客户端有界发送队列 + 丢弃计数（P1），不用阻塞式 send |
| 4 | WiFi 发射尖峰拉垮 3.3 V（→ STM32 复位） | 整机重启 | 硬件侧：100 µF + 100 nF 已列"建议焊上"；上机时按 [`32-esp32-bridge.md`](32-esp32-bridge.md) §2.2 实测 3.3 V 跌落 < 100 mV |
| 5 | `PLATFORMIO_*` 环境变量未生效（直接调 pio） | 又报 `HomeDirPermissionsError` 或把 1 GB 装进工程目录 | 构建一律走 `firmware/tools/pio.ps1` |
| 6 | 引脚选错（踩到 strapping / Flash / USB 脚） | 板子不启动或 USB 掉线 | §1 的引脚约束表；GPIO6/7 避开全部禁区 |
| 7 | **工具链适配丢失**（core 目录被搬回工作区、SUBST 别名失效、venv 被重建） | 构建"代码没改却突然失败"：`PermissionError` 或空格路径报错 | 症状对照 §4.1；**一律走 `firmware/tools/pio.ps1`**，不要绕过它直接调 pio |
| 8 | 部署到别的机器时上述适配不存在 | 新机器上构建失败 | §4.1 的四条就是移植清单；②④ 两条是本地对工具链的修改，重建后需重做 |

---

**文档结束。P0 通过后按 P1 → P5 推进。**
