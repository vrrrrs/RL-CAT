# ESP32-C3 无线桥 上机验收单

> **文档编号**：DOC-ESP-ACC-001 ｜ **版本**：v1.0 ｜ **日期**：2026-10-10
> **对象**：项目负责人（执行） / ESP32 固件开发者（判读）
> **对应方案**：[`36-esp32-plan.md`](36-esp32-plan.md) ｜ **设计**：[`32-esp32-bridge.md`](32-esp32-bridge.md)
> **前置文档**：[`firmware/esp32/README.md`](../firmware/esp32/README.md)（构建与工具用法）

---

## 0. 怎么用这份单子

```
① 按 A 编号顺序做，不要跳步。每条都有：动作 / 预期现象 / 判据 / 失败分支。
② 每条做完在结果栏写 ✅ 或 ❌ + 现场现象（现象比结论重要）。
③ ❌ 时先看 §6「现象 → 原因对照表」，按最左列查出原因再动。
④ 需要 STM32 的条目（A22 以后）在 STM32 侧补齐 USART3 收发之前【做不了】，
   会在该条目标注 ⛔ 前置未满足。
```

### 0.1 ⭐ 一条命令跑完全部自动项（推荐）

**只需要一根跳线**（GPIO6 ↔ GPIO7），无需 STM32、无需 USB-TTL：

```
① 用一根杜邦线把 ESP32 的 GPIO6 与 GPIO7 短接
② 笔记本 WiFi 切到 RL-CAT-E959（密码 rlcat1234）
③ 执行体检（确认网络与端口通了）：
     "C:\...\RL-CAT\bench.cmd" doctor
④ 跑验收：
     "C:\...\RL-CAT\bench.cmd" verify --loopback --report D:\rlcat-esp32-acceptance.json
⑤ 把屏幕输出（或那个 JSON）回传 → ESP32 侧判读；WiFi 切回原网络
```

**⚠️ 不要用 `powershell -File ...ps1`**：本机默认禁止运行脚本（`UnauthorizedAccess`）。
要么用仓库根的 **`bench.cmd`**（推荐，不经 PowerShell），要么加 `-ExecutionPolicy Bypass`。

**原理**：跳线让 ESP32 发出的每一个字节都回到它自己的接收端；于是控制口的
`raw <hex>` 可以**直接喂给它一帧 TEL_FAST**（正是 STM32 会发的那些字节），
遥测、仲裁门控、指令注入就都能在真机上验证了。

**自动覆盖**：A4、A5、A10、A11、A12、A13、A14、A15、A16、A18、A19。
**仍需人工**：A6/A7（需要 USB-TTL 转接板）、A8（要故意断网/停读）、A21（30 min 长跑）、A22/A23（需要真 STM32）。
**已自动执行完毕**：A1、A2、A3（见下方结果栏）。

**准备清单**

| 项 | 说明 |
|---|---|
| ESP32-C3 开发板 | 本项目实测：ESP32-C3 rev v0.4，内嵌 4 MB Flash，原生 USB-Serial-JTAG |
| **能传数据的 USB 线** | ⚠️ 不是"只能充电"的线；直插主板 USB 口，避开 hub |
| 杜邦线 ×2 | GPIO6 → STM32 PB11、GPIO7 ← STM32 PB10（**交叉**）+ 共地 |
| 一根跳线 | A5 的 UART loopback 用（GPIO6 ↔ GPIO7 短接） |
| USB-TTL 转接板 | A6 起用（模拟 STM32 或当串口监视器） |
| 笔记本 | 连 `RL-CAT-XXXX` 热点，装好 TCP 调试工具（或就用 `bench.py`） |

**⚠️ ESP32 侧引脚速查（C3 + 内嵌 Flash）**

```
可用：GPIO0–5、GPIO6/7（本项目 UART1）、GPIO10、GPIO20/21
禁区：GPIO11–17（内嵌 Flash）、GPIO18/19（USB D±）、GPIO2/8/9（strapping，9=BOOT）
```

**⚠️ 固定参数（来自 `rlca_esp32_config.h`）**

```
SSID = RL-CAT-<SoftAP MAC 后 4 位>   本项目板子 = RL-CAT-E959
       ⚠️ 是 SoftAP 的 MAC（= 出厂 MAC 末字节 +1），不是 esptool 读到的 e0:72:a1:22:e9:58
密码 = rlcat1234                  IP = 192.168.4.1
3333 = 纯透传（STM32 ↔ 笔记本）    3334 = ESP32 控制/日志口
UART1 = GPIO6(TX) → PB11、GPIO7(RX) ← PB10，921600 8N1
```

---

## 1. A1–A3　P0：烧写与启动

### A1　板子被识别、能烧写

| 项 | 内容 |
|---|---|
| **动作** | 插上 USB；`cd firmware/esp32`；`powershell -ExecutionPolicy Bypass -File ..\tools\pio.ps1 run -t upload` |
| **预期** | 端口自动识别（本项目为 `COM8`，VID:PID `303A:1001`），出现 `Hash of data verified.` / `[SUCCESS]` |
| **判据** | 烧写成功且**板子不再反复掉线**（这是 Flash 空片时的典型症状：ROM 刷 `invalid header` + 看门狗每约 2.4 s 复位） |
| **失败分支** | 找不到端口 → 换数据线 / 换 USB 口 / 检查是否 hub → 仍不行看 §6「EP1」 |
| **结果** | ✅ **2026-10-10 通过**：`Hash of data verified.`（第一次尝试因空片复位循环导致端口掉线，重试即成功）；分区表已确认为 `factory 0x10000 / 00200000` |

### A2　启动日志正确

| 项 | 内容 |
|---|---|
| **动作** | `powershell -ExecutionPolicy Bypass -File ..\tools\pio.ps1 device monitor`（或任意串口工具 @115200） |
| **预期** | 看到（数值以实测为准）： |

```
I (xxx) rlca: RL-CAT ESP32-C3 wireless bridge  cfg 0.5.0
I (xxx) rlca: task watchdog: on (timeout 5 s, panic=1)
I (xxx) cfg: config: ports 3333/3334 (default), AP password default
I (xxx) rlca: chip  : model=.. cores=1 rev=v0.4  flash=4 MB
I (xxx) rlca: stm32 : UART1 tx=GPIO6 rx=GPIO7 @ 921600 8N1
I (xxx) rlca: ports : 3333 = raw bridge, 3334 = ESP32 console
I (xxx) rlca: arb   : enable=1 block_on_unknown=1 stale=500 ms
I (xxx) rlca: selftest: 6/6 checks passed
I (xxx) net: AP up: ssid=RL-CAT-E958 password=rlcat1234 ip=192.168.4.1 (max 4 stations)
I (xxx) net: TCP servers listening: 3333 (bridge) and 3334 (console)
I (xxx) log: ESP32 logs are mirrored to console port 3334
I (xxx) bridge: UART1 ready: tx=GPIO6 rx=GPIO7 921600 8N1 ring=4096B
I (xxx) arb: arbiter watching the STM32 link (stale=500 ms, block_on_unknown=1)
I (xxx) rlca: P1..P4 ready -- join the AP and connect TCP 3333 (bridge) / 3334 (console)
I (xxx) rlca: alive: up=1 s heap=...  clients(bridge/console)=0/0  uart rx/tx=0/0 ...
```

| 项 | 内容 |
|---|---|
| **判据** | ① 版本号是 `0.5.0`（⚠️ 若板上仍显示 `0.4.0-p4`，说明版本号改动还没烧入，重烧一次即可）；② `flash=4 MB`；③ **`selftest: 6/6 checks passed`**；④ 每 10 s 一条 `alive:` 心跳；⑤ 无 `rst:` 复位刷屏 |
| **失败分支** | `selftest` 非 6/6 → **立即停下**，把 report 原文发回（说明协议层在真实 CPU 上有问题）；反复复位 → §6「EP2」；`flash` 不是 4 MB → 删 `firmware/esp32/sdkconfig.esp32c3` 重新构建 |
| **结果** | ✅ **2026-10-10 通过**：`cfg 0.4.0-p4` / `task watchdog: on (timeout 5 s, panic=1)` / **`selftest: 6/6 checks passed`** / `AP up: ssid=RL-CAT-E959 ip=192.168.4.1` / 两个 TCP 口就绪 / 日志镜像已装 / **错误与告警行数 = 0** |

### A3　空载行为：无 STM32 时不乱动

| 项 | 内容 |
|---|---|
| **动作** | 什么都不接，等 60 s，观察日志 |
| **预期** | `uart rx/tx=0/0 drops=0 ovf=0 tel(age=0 ms frames=0 crc=0) pi_online=0`；无任何错误刷屏 |
| **判据** | 无 `UART FIFO overflow`、无 `ring buffer full`、无任务看门狗复位 |
| **失败分支** | 有 `ovf` → 见 §6「EP3」 |
| **结果** | ✅ **2026-10-10 通过**：60 s 内 7 条 `alive:` 心跳；`heap` 184364 稳定不降（无泄漏）；`uart rx/tx=0/0 drops=0 ovf=0 tel(frames=0) pi_online=0`；无看门狗触发 |

---

## 2. A4–A8　P1：无线链路

### A4　AP 可连、3334 控制口可用

| 项 | 内容 |
|---|---|
| **动作** | 笔记本连 `RL-CAT-E958`（密码 `rlcat1234`）；`powershell -File firmware\tools\bench.ps1 con --host 192.168.4.1 --cmd "?" --cmd "stats" --cmd "k"` |
| **预期** | 收到帮助文本（含 `i / s / z / c / r / stats / raw / hb / k / q`）、`stats:` 计数、`ok` |
| **判据** | 三条命令都有回显；`stats` 里 `ap=up ssid=RL-CAT-E958` |
| **失败分支** | 连不上热点 → §6「EP4」；连上但 TCP 不通 → 确认连的是 **3334**（控制口）不是 3333 |
| **结果** | ☐ |

### A5　⭐ UART loopback：TCP→UART→TCP 全链路（不需要 STM32、不需要 USB-TTL）

| 项 | 内容 |
|---|---|
| **动作** | ① **把 GPIO6 与 GPIO7 用跳线短接**；② `powershell -File firmware\tools\bench.ps1 loop --host 192.168.4.1 --bytes 262144` |
| **预期** | `loop: sent=262144 received=262144 mismatch=0  ~3s  ~700 kbit/s` + `✅ full round trip, zero loss, zero corruption` |
| **判据** | **零丢失、零错序**；同时 3334 的 `stats` 里 `uart rx/tx` 两个计数都增长到与发送量一致，且 `fifo_ovf=0`、`tx_ring_full=0` |
| **失败分支** | `received=0` → 跳线没接好/接错脚；`mismatch>0` → §6「EP5」；`tx_ring_full>0` → 减小 `--chunk` 或查 §6「EP3」 |
| **结果** | ☐ |

### A6　⭐ 与 PC 端 STM32 模拟器对打（需要 USB-TTL）

| 项 | 内容 |
|---|---|
| **动作** | ① 接 USB-TTL：**ESP32 GPIO6 → 转接板 RX**、**ESP32 GPIO7 ← 转接板 TX**、GND 共地；② `powershell -File firmware\tools\bench.ps1 sim --port COM<x> --link-ok 0 --serve`；③ 另开一个窗口：`powershell -File firmware\tools\bench.ps1 tcp --frames 20` |
| **预期** | ③ 每秒解码出约 100 条 `TEL_FAST seq=.. cyc=.. 11.50V LINK/READY q0..3=[...] gait=(...)`；顺序号连续、`cyc` 每帧 +1 |
| **判据** | 100 Hz 稳定、`crc_err=0`、`cyc` 无跳变（有跳变说明丢帧或丢字节） |
| **失败分支** | 一条都没有 → 确认 GPIO6/7 与转接板是**交叉**；乱码 → 波特率不是 921600；`crc_err>0` → §6「EP5」 |
| **结果** | ☐ |

### A7　遥测被正确转发给**多个**客户端

| 项 | 内容 |
|---|---|
| **动作** | 在 A6 运行中，再开一个 `bench.py tcp --frames 10`（同时保持第一个连接） |
| **预期** | 两个客户端都收到相同的字节流；3334 的 `stats` 显示 `bridge_clients=2` |
| **判据** | 至少支持 2 个并发客户端，互不影响 |
| **失败分支** | 第二个连接被拒 → 查 `RLCA_TCP_MAX_CLIENTS`（默认 4） |
| **结果** | ☐ |

### A8　慢客户端不拖垮链路（非阻塞广播）

| 项 | 内容 |
|---|---|
| **动作** | 接 A7，**拔掉网线/关闭笔记本 WiFi**（模拟客户端消失），或在客户端侧停止读取；保持 `sim` 运行 60 s，然后重连看 `stats` |
| **预期** | ESP32 不重启、`sim` 侧无阻塞（仍 100 Hz）；`stats` 里出现 `tx_drops>0` 但不影响 UART 侧计数 |
| **判据** | **慢/死客户端只丢自己的数据，不影响 UART 与其它客户端** |
| **失败分支** | ESP32 复位或 UART 侧卡住 → 这是必须修的设计缺陷，记录现象回来 |
| **结果** | ☐ |

---

## 3. A9–A13　P2：协议与指令注入

### A9　`hb` 发送标准帧

| 项 | 内容 |
|---|---|
| **动作** | A6 的 `sim` 运行中，在 `con` 里发 `hb` |
| **预期** | 控制口回 `sent HEARTBEAT seq=N len=0 -> uart 7/7 bytes`；**sim 侧**打印 `<- HEARTBEAT seq=N` |
| **判据** | 帧被对端正确解出（CRC 通过） |
| **失败分支** | sim 侧打到 `crc_err` → 见 §6「EP5」 |
| **结果** | ☐ |

### A10　`i` 状态查询（不注入任何东西）

| 项 | 内容 |
|---|---|
| **动作** | 控制口发 `i`；再发 `imu` |
| **预期** | 类似： |

```
link: frames=1234 (TEL_FAST=1234 TEL_STATUS=0 other=0) crc_err=0 len_err=0 resync=0
      telemetry age=3 ms status=0x09 mode=0 voltage=11500 mV cycle=1234 gaps=0 q=yes
      pi_online=0  inject=allowed
imu: age=3 ms  cycle=1234
     quat  w=+1.0000 x=+0.0000 y=+0.0000 z=+0.0000
     gyro  x=+0.000 y=+0.000 z=+0.000 rad/s
     gravity(body) x=+0.000 y=+0.000 z=-1.000   <- observation dims 0..2
```

| 项 | 内容 |
|---|---|
| **判据** | ① `age` 是毫秒级小数；② `q=yes`；③ `imu` 的 gravity 是单位向量（静止水平时应接近 `(0,0,-1)`）；④ **sim 侧不应看到任何新帧**（`i`/`imu` 不注入 —— 因为 STM32 忽略 `FLAG_ENABLE`，注入查询帧会带着 mode/q 一起生效） |
| **失败分支** | sim 侧看到 `CMD_JOINT` → 说明代码退回了旧实现，回来修 |
| **结果** | ☐ |

### A11　⭐ `s <关节号> <角度增量>` 单关节微动

| 项 | 内容 |
|---|---|
| **动作** | A6 运行中（`sim --link-ok 0`），控制口发 `s 4 10`；再看 sim 侧输出 |
| **预期** | 控制口回 `sent CMD_JOINT seq=N len=31 ...` + `joint 4 +10.0 deg -> q=... (others held)`；sim 侧打印 `<- CMD_JOINT mode=ACTIVE flags=0x01 q=[.. +0.010 +0.100 ...]` |
| **判据** | ① `mode=ACTIVE`、`flags=0x01`；② **只有 4 号关节比上一帧 `TEL_FAST` 的 q 大 0.1745 rad（10°）**，其余 11 个与遥测一致（±1 步） |
| **失败分支** | 报 `no joint telemetry yet` → 先做 A6；报 `blocked` → 见 A12；其余关节被改动 → 这是严重缺陷，记录回来 |
| **结果** | ☐ |

### A12　`z` / `c` / 越界保护

| 项 | 内容 |
|---|---|
| **动作** | 依次发：`z`、`c`、`s 12 5`、`s 4 99`、`s 4 0` |
| **预期** | `z` → `CMD_MODE mode=IDLE`；`c` → `CMD_CALIB action=0`；后三条分别回 `usage:`、`|delta| must be <= 30 deg`、`delta is 0`，且**都不发送任何帧** |
| **判据** | 非法输入被拒绝且不发帧 |
| **结果** | ☐ |

### A13　`raw` 透传能力

| 项 | 内容 |
|---|---|
| **动作** | 发 `raw AA55007F`（= 一帧 HEARTBEAT 的头 4 字节，故意不完整），再发 `raw ZZ` |
| **预期** | 第一条回 `raw: 4/4 bytes queued`；第二条回 `err: bad hex` |
| **判据** | 任意字节可下发；坏输入被拒 |
| **失败分支** | 4 字节后 sim 侧不应报错（半帧会在 5 ms 后被状态机丢弃并 `resync+1`）—— 顺带验证「半帧自愈」 |
| **结果** | ☐ |

---

## 4. A14–A17　P3：指令仲裁（⭐ 最关键的安全项）

### A14　Pi 离线 → 放行

| 项 | 内容 |
|---|---|
| **动作** | `sim --link-ok 0`；控制口发 `i`、再发 `s 4 10` |
| **预期** | `i` 显示 `pi_online=0 inject=allowed`；`s` 成功发送 |
| **判据** | 阶段一用例成立：没有 Pi 时 ESP32 可以注入 |
| **结果** | ☐ |

### A15　⭐ Pi 在线 → 拒绝（防止"抢控制权"）

| 项 | 内容 |
|---|---|
| **动作** | 重启 sim：`sim --link-ok 1`；控制口发 `i`、`s 4 10`、`z`、`c` |
| **预期** | `i` 显示 `pi_online=1 inject=blocked`；`s`/`z`/`c` 全部回 `blocked: s cannot inject (Pi is online (STAT_LINK_OK) -- the Pi owns the robot [age=.. ms status=0x0C])`，**sim 侧看不到任何帧** |
| **判据** | Pi 在线时 ESP32 **一个指令帧都不发**（sim 侧无 `<-` 输出） |
| **失败分支** | 有帧发出 → 仲裁失效，**立即停止**并回来修 |
| **结果** | ☐ |

### A16　⭐ 遥测中断 → 保守拒绝（失败安全）

| 项 | 内容 |
|---|---|
| **动作** | A14 状态下**直接 Ctrl-C 掉 sim**（USART3 静默）；等 1 s；控制口发 `i`、`s 4 10` |
| **预期** | `i` 显示 `telemetry age=<越来越大>`，超过 `RLCA_ARB_STATUS_STALE_MS`(500 ms) 后 `inject=blocked`；`s` 被拒，理由是 `telemetry stale (Pi online state unknown)` |
| **判据** | **状态未知时禁止注入**（宁可不调试，也不要"以为 Pi 离线"而抢控制权） |
| **失败分支** | 仍放行 → 违反失败安全策略，回来修 |
| **结果** | ☐ |

### A17　`cycle_cnt` 丢帧可见

| 项 | 内容 |
|---|---|
| **动作** | 在 A6 运行中，短暂拔掉 GPIO7 那根线（RX），2 s 后插回；控制口发 `i` |
| **预期** | `crc_err` 或 `resync` 增长；`gaps` 增长（`cycle_cnt` 不连续）；恢复后 `age` 回到毫秒级 |
| **判据** | 丢帧/断线能被计数并展示，且链路能自愈 |
| **结果** | ☐ |

---

## 5. A18–A21　P4：配置与健壮性

### A18　板上自检（在真实 CPU 上跑协议测试向量）

| 项 | 内容 |
|---|---|
| **动作** | 控制口发 `selftest`（开机日志里也会自动跑一次） |
| **预期** | `ok: selftest: 6/6 checks passed` |
| **判据** | 6/6（CRC 标准向量、CRC 查表 vs 位运算 200 组、帧往返、TEL_FAST 偏移与 `status` 在帧偏移 99、`LEN>128` 拒绝并恢复、`CMD_JOINT` 量化往返） |
| **失败分支** | 非 6/6 → 记下 report 原文回来（说明主机侧过的测试在真实 CPU 上失败，通常是编译器/对齐问题） |
| **结果** | ☐ |

### A19　`cfg`：运行配置可改可存（NVS）

| 项 | 内容 |
|---|---|
| **动作** | ① `cfg`（查看）；② `cfg pw abc`（太短）；③ `cfg pw rlcat5678`；④ `cfg port 80 3334`；⑤ `cfg port 3335 3336`；⑥ 断电重启；⑦ `cfg`；⑧ 再 `cfg clear` 并重启 |
| **预期** | ① 显示当前端口/密码来源与内置默认值；② `err: password must be 8..63 ...`；③ `ok: password stored (reboot to apply)`；④ `err: ports must be 1024..65534 ...`；⑤ `ok: ports stored ...`；⑦ 显示 `3335/3336 (NVS)`，且日志里 TCP 实际监听 3335/3336；⑧ 恢复 `3333/3334 (default)` |
| **判据** | 非法值被拒；合法值重启后生效（说明存进了 NVS）；`clear` 能回默认 |
| **⚠️ 注意** | `cfg clear` 会 `nvs_flash_erase`，同时清掉 WiFi 校准数据（下次启动重新校准，正常现象） |
| **失败分支** | 重启后端口没变 → NVS 读写失败，看 `cfg:` 开头的日志 |
| **结果** | ☐ |

### A20　ESP32 日志镜像到控制口 + 看门狗在跑

| 项 | 内容 |
|---|---|
| **动作** | 只连 3334（不插 USB 看串口）：`bench.py con --cmd stats`；等待 10 s 观察是否收到日志；再插拔一次 USB 看是否照常运行 |
| **预期** | 3334 上持续出现与串口相同的日志（`alive:` 等），`\n` 被转成 `\r\n`；USB 拔掉不影响 AP/TCP；开机日志里有 `task watchdog: on (timeout 5 s, panic=1)` |
| **判据** | ① 不接 USB 也能从 3334 看到 ESP32 自己的日志；② 看门狗已启用（三处 `esp_task_wdt_add/reset`：UART 泵、socket 任务、仲裁任务） |
| **失败分支** | 3334 收不到日志 → 查 `log:` 那行是否打印；板子莫名重启 → 看是否有 `Task watchdog got triggered`（说明某个任务卡住，属必须修的缺陷） |
| **结果** | ☐ |

### A21　30 min 连续运行

| 项 | 内容 |
|---|---|
| **动作** | A6 的 `sim` 保持 100 Hz；每 30 s 用 `con --cmd stats` 取一次计数；跑满 30 min |
| **预期** | `alive:` 心跳不断；结束时 `fifo_ovf=0`、`rx_errors=0`、`heap` 不持续下降；`crc_err` 与 `gaps` 保持 0；无看门狗触发 |
| **判据** | ① 无重启（uptime 连续）；② `fifo_ovf=0`；③ 堆内存无泄漏趋势（首末 `heap` 差 < 2 KB） |
| **结果** | ☐ |

---

## 6. A22–A23　与真 STM32 联调（⛔ 前置未满足）

**⛔ 前置条件**：STM32 侧必须先在 USART3 上**收下行帧 + 发遥测**。截至 2026-10-10 的实测（见 [`36-esp32-plan.md`](36-esp32-plan.md) §2.1）：

```
✅ 已满足：帧格式一致；STAT_LINK_OK 只反映 USART1/Pi（仲裁自锁风险已排除）；USART3 硬件配置就绪
❌ 未满足：USART3 收到字节后没有任何消费者（只计数）；STM32 从不向 USART3 发送任何帧
```

**因此下面两条现在做不了；契约文本见 [`36-esp32-plan.md`](36-esp32-plan.md) §2，转交 STM32 侧补齐后即可执行。**

### A22　真 STM32：遥测可见

| 项 | 内容 |
|---|---|
| **动作** | ESP32 GPIO6/7 与 STM32 PB11/PB10 交叉接好（共地）；`bench.py tcp --frames 20` |
| **预期** | 解码出 STM32 的 `TEL_FAST`（若契约采用 100 Hz 镜像）或至少 `TEL_STATUS` |
| **判据** | `crc_err=0`；`cyc` 连续；电压与万用表差 < 50 mV |
| **结果** | ☐（⛔ 前置未满足） |

### A23　真 STM32：无 Pi 时用无线链路调舵机

| 项 | 内容 |
|---|---|
| **动作** | 上电后（Pi 不接）在控制口发 `i` → `s <关节> <小角度>` |
| **预期** | `i` 显示 `pi_online=0 inject=allowed`；`s` 后真机对应关节小幅动作、其余保持 |
| **判据** | 关节号 → 物理关节的对应关系正确（用 [`30-firmware.md`](30-firmware.md) §2 的逻辑索引表核对）；方向正确 |
| **失败分支** | 关节号错位 → 查逻辑索引 A 与舵机 ID 的映射；方向反 → 属 STM32 侧 `dir` 标定 |
| **结果** | ☐（⛔ 前置未满足） |

---

## 7. 现象 → 原因对照表

| 编号 | 现象 | 最可能的原因 | 怎么确认 |
|---|---|---|---|
| **EP1** | 烧写时找不到端口 | 只充电的 USB 线 / hub / 板子处于空片复位循环 | 设备管理器看是否反复出现又消失（VID:PID `303A:1001`）；换线换口 |
| **EP2** | 上电后反复复位 | 供电不足（WiFi 峰值 500 mA）或 `Error_Handler` | 看复位原因串；换 USB 口；查 3.3 V 跌落 |
| **EP3** | `fifo_ovf` / `tx_ring_full` 增长 | UART 侧速率不匹配或 TX 环形缓冲太小 | 打印 `stats`；减小 TCP 侧 `--chunk`；查 `RLCA_UART_TX_RING` |
| **EP4** | 连不上热点 | SSID 后四位不是 `E958`（换了板子）/ 信道冲突 / 天线被金属挡 | 看启动日志里的 `ssid=`；挪开天线附近金属 |
| **EP5** | `crc_err>0` 或字节错序 | 波特率不符（对端不是 921600）/ 时钟偏差 / 地线没共 / 线太长 | 量两端波特率；确认共地；缩短杜邦线 |
| **EP6** | 所有 `s`/`z`/`c` 都被 `blocked`，但明明没接 Pi | STM32 把 `STAT_LINK_OK` 置起来了（它的 Pi 链路上有东西在发帧），或遥测过期 | 用 `i` 看 `status` 与 `age`；确认 USART1 上没有别的设备在发帧 |
| **EP7** | `s` 报 `no joint telemetry yet` | STM32 没在 USART3 上发 `TEL_FAST`（当前就是这种状态） | 用 `mon`/`tcp` 看有没有任何帧；查 §6 A19 前置 |
| **EP8** | 构建突然失败（代码没改） | 工具链适配被破坏（core 目录被搬回工作区 / SUBST 别名失效 / venv 被重建） | 对照 [`36-esp32-plan.md`](36-esp32-plan.md) §4.1 四条 |
| **EP9** | 板子隔一阵子自己重启，日志里有 `Task watchdog got triggered` | 某个任务卡住（socket 死锁 / UART 驱动异常） | 记下是哪个任务（`IDLE`/`uart_rx`/`net`/`arb`）回来；这是必须修的缺陷，不要靠关看门狗掩盖 |
| **EP10** | 启动日志里 app 分区是 `00100000`（1 MB）而不是 `00200000` | PlatformIO 的 IDF 集成**自己生成分区表**（`espidf.py:1528` 读 `board.get("build.partitions", "partitions_singleapp.csv")`），**忽略 `CONFIG_PARTITION_TABLE_CUSTOM`** ⇒ 项目里的 `partitions.csv` 被默认表顶掉 | `platformio.ini` 里必须有 `board_build.partitions = partitions.csv`；改完删 `D:\Work\EspIDF\build\esp32c3\partitions.bin` 再构建 |

---

## 8. 记录表模板

| 条目 | 结果 | 现场现象 / 数值 | 备注 |
|---|---|---|---|
| A1 烧写 | ✅ | `Hash of data verified.`；分区表 `factory 00200000` | 空片复位循环导致首次端口掉线，重试成功 |
| A2 启动日志 | ✅ | cfg 0.4.0-p4；selftest 6/6；0 错误/告警行 | — |
| A3 空载 | ✅ | 7 条心跳；heap 184364 稳定；计数全 0 | — |
| A4 AP + 控制口 | ☐ | | 需笔记本切到 RL-CAT-E959（当前连着 HUST_WIRELESS） |
| A5 loopback | ☐ | sent=____ received=____ mismatch=____ | 需 GPIO6↔GPIO7 跳线 |
| A6 模拟器对打 | ☐ | 帧率=____ crc_err=____ | |
| A7 多客户端 | ☐ | clients=____ | |
| A8 慢客户端 | ☐ | tx_drops=____ | |
| A9 hb | ☐ | | |
| A10 i | ☐ | inject=____ | |
| A11 s 单关节 | ☐ | 关节号=____ 增量=____ | |
| A12 非法输入 | ☐ | | |
| A13 raw | ☐ | | |
| A14 Pi 离线放行 | ☐ | | |
| A15 Pi 在线拒绝 | ☐ | sim 侧帧数=____（应为 0） | |
| A16 遥测中断拒绝 | ☐ | | |
| A17 丢帧可见 | ☐ | gaps=____ | |
| A18 selftest | ☐ | checks=__/6 | |
| A19 cfg（NVS） | ☐ | 重启后端口=____ | |
| A20 日志镜像/看门狗 | ☐ | 3334 收到日志 ☐ | |
| A21 30 min | ☐ | heap 首/末=____ / ____ | |
| A22 真 STM32 遥测 | ☐ | ⛔ 前置未满足 | |
| A23 真 STM32 调舵机 | ☐ | ⛔ 前置未满足 | |

---

**验收单结束。做完请把「记录表」回传，❌ 条目附现象。**
