/**
  ******************************************************************************
  * @file    rlcat_config.h
  * @brief   RL-CAT STM32 固件 —— 全部可调参数的【单一来源】。
  *
  *          代码里不允许出现魔数：频率、超时、阈值、限位、舵机 ID、
  *          单位换算系数一律在这里定义。
  *
  * 约定：
  *   - 本文件只放【编译期常量】，不放变量。
  *   - 单位一律写在名字或注释里（_ms / _us / _rad / _mv / _steps）。
  *   - 数值来源标注在注释里，便于回溯（文档编号 + 章节）。
  *
  * 文档依据：
  *   docs/30-firmware.md                       （五张关键表）
  *   firmware/doc/固件设计书.md                 （架构与参数）
  *   firmware/doc/协议规范_树莓派通信.md         （帧与超时）
  *   firmware/doc/伺服总线模块设计.md            （总线与舵机）
  *   firmware/doc/CubeMX配置清单.md             （时钟与外设）
  ******************************************************************************
  */

#ifndef RLCA_CONFIG_H
#define RLCA_CONFIG_H

#include <stdint.h>

/*==============================================================================
 * 0. 构建期开关
 *============================================================================*/

/** 时序模型：
 *    0 = 模型 A（请求-应答）—— STM32 发 TEL_FAST 后等 Pi 的 CMD_JOINT，
 *        超时 RLCA_LINK_CMD_WAIT_US。端到端 ~3 ms，抖动受 Pi 影响。
 *    1 = 模型 B（不等指令）  —— 直接用上一帧指令，周期抖动为零，
 *        端到端 ~13 ms。
 *  依据： docs/31-pi-runtime.md §2.1 选模型 A；协议规范 §7.3 给出 B 作为退路。
 *  切换条件：P4 实测 Pi 侧 P99 若 > 4 ms，改为 1。
 *
 * ⭐⭐⭐ 2026-10-17 已切换到 1（模型 B）—— 切换条件确实被触发了。
 *
 *  ⚠️⚠️ 现场实测的两条证据：
 *
 *  ① **舵机总线被饿死 → bus_timeout**
 *     `link_pi_wait_cmd()` 的自旋循环里【只调 link_pi_poll()】，
 *     既不调 `bsp_uart_poll_dma()` 也不调 `servo_bus_poll()`。
 *     ⇒ 那最多 4 ms 里：
 *        · USART2 的 DMA 字节搬不进舵机接收环（avail() 一直是 0）
 *        · 舵机总线状态机完全停摆
 *     ⇒ 任何在飞的读事务都必然撞上 3 ms 整帧超时 ⇒ BUS_TIMEOUT
 *     ⇒ err_flags 在 0/0x40 跳 ⇒ safety_reasons 在 0/0x20 跳 ⇒ 幅度 100%/75% 跳
 *     实测：bus_timeout 约 1.1 次/秒（≈1%/轮），而操作者几乎没敲过 shell 命令
 *           ⇒ 不是 shell 造成的，是这个 4 ms 自旋。
 *
 *  ② **100 Hz 周期被拉长到 14 ms**
 *     10 ms 时隙里白等 4 ms ⇒ 周期抖动大 ⇒ 也直接妨碍 RL 的 decimation 契约。
 *
 *  ⭐ 模型 B 的代价与取舍：
 *     · 端到端延迟 3 ms → ~13 ms（多一帧，因为用的是"上一次收到的指令"）
 *     · 但对 100 Hz 控制 + 舵机本身 45 RPM 的响应速度，13 ms 完全可接受
 *     · 换来的是：周期严格 10 ms、舵机总线不再被饿死、链路判定稳定
 *
 *  ⚠️ 若日后 Pi 侧换成 PREEMPT_RT 且实测 P99 < 1 ms，可以再切回 0 拿回那 10 ms。 */
#define RLCA_LINK_MODE_B              1

/** 状态灯（LED）极性 —— ⚠️ 这里踩过坑，别想当然。
 *
 *  1 = 低电平点亮（LED 引脚拉低才亮）
 *  0 = 高电平点亮
 *
 *  ⚠️⚠️ STM32 Blue Pill 的板载 LED 在【PC13】，且是【低电平点亮】，
 *      所以本工程必须取 1。
 *
 *  早期版本的两个错误（都已修）：
 *    ① 引脚配错：LED 放在 PB0，而 PB0 在 Blue Pill 上根本没接 LED
 *       → 板载灯永远不亮，所有状态指示（含"启动失败"提示）全部静默。
 *    ② 极性写反：按"高电平点亮"写，即使换成 PC13 也会亮灭颠倒。
 *
 *  引脚本身由 CubeMX 决定（`main.h` 的 `LED_Pin` / `LED_GPIO_Port`），
 *  本宏只管极性。两者必须一致，见 docs/39-firmware-handover.md。 */
#define RLCA_LED_ACTIVE_LOW           1

/** ⭐ 上电后额外重发 torque-off 的周期数（100 Hz 循环里）。
 *
 *  ⚠️⚠️ 存在的理由是一次实测的撞机事故：
 *     控制板（Waveshare Bus Servo Adapter）与 STM32 常从同一路电源上电，
 *     而适配器自身也要启动 —— 上电头几百毫秒内它不理 UART。
 *     app_init 里那条 torque-off 会石沉大海，而舵机仍处于力矩使能状态，
 *     目标位置寄存器里又存着默认标定表算出的值 → 12 个关节冲到底。
 *
 *     ⚠️ 更糟的是 servo_bus_set_torque() 无条件返回 true，
 *        固件【以为】力矩已关，实际没关 —— 静默失败的安全指令等于没有。
 *
 *  实测判据：断开 UART2 只给控制板上电 → 舵机不动；
 *            接上 UART2 一起上电   → 12 个关节冲到底。
 *            => 动的指令来自 STM32，不是舵机自己的上电行为。
 *
 *  取 5000 个周期 = 50000 ms（2026-10-16 由 300 上调）。
 *  ⚠️ 这个值现在是【上限】，实际行为见下面第 2 条。
 *
 *  ⚠️⚠️ 2026-10-16 第二次修正 —— 从"限时补发"改成"持续重申"：
 *
 *     原设计是"上电后 N 个周期内每 10 ms 补发一次"，寄希望于
 *     "适配器总会在这段时间内就绪"。但现场证明这个假设不成立：
 *     上电后舵机【仍然】在一秒内开始持续运动，说明那些 torque-off
 *     一条都没生效 —— 要么适配器启动比窗口长，要么窗口刚好错过。
 *
 *     而 `servo_bus_set_torque()` 【无条件返回 true】（见上面的警告），
 *     所以固件无从知道"到底关掉了没有"，只能盲目重试。
 *
 *     ⇒ 改成：只要 `want_torque == false`，就【周期性持续重申泄力】，
 *        间隔 RLCA_TORQUE_REASSERT_MS（默认 200 ms = 5 Hz）。
 *
 *     为什么这样安全（幂等性论证）：
 *       · 舵机已在泄力 → 再写一次 0 是【幂等】的，无副作用
 *       · 舵机仍使能   → 这正是我们要纠正的状态，写 0 就是纠正它
 *       · 目标位置不受影响（写的是【力矩寄存器 40】，不是位置寄存器 42）
 *         ⇒ 所以它【不会】干扰闭环控制，也不会让机器人动
 *       · 只在 want_torque == false 时发
 *         ⇒ 绝不覆盖操作者/策略的主动使能
 *
 *     代价：5 Hz × 12 帧 = 60 帧/s。在 1 Mbps 上约占 0.6%，
 *           且每帧只有 8 字节。完全可接受。
 *
 *  ⚠️ RLCA_TORQUE_OFF_BURST 现在只用于"上电头一段更密集的重申"
 *     （见 app_main.c 的 ⑥b），超过这个周期数后降到 REASSERT 频率。 */
#define RLCA_TORQUE_OFF_BURST         300u    /* 前 3 s 每周期重申（更密集） */

/** ⭐ 持续重申泄力的间隔（ms）。见上面 RLCA_TORQUE_OFF_BURST 的说明。
 *  ⚠️ 只在 want_torque == false 时生效；写的是力矩寄存器，不影响目标位置。 */
#define RLCA_TORQUE_REASSERT_MS       200u

/** ⭐⭐⭐ 上电【安全保持】时长（控制周期数）。
 *
 *  在这段时间内，下发给舵机的目标【直接用实测原始步数】，
 *  完全绕开角度域（标定表 / 限幅 / 换算）。
 *
 *  ⚠️ 为什么必须绕开：未标定时角度域的含义是错的 ——
 *     q_rad 经 rlca_q_clamp（按错限位夹）再经 rlca_q_to_step（按错 offset 换算）
 *     会得到一个固定的错步数，12 个舵机同向同速奔向它。
 *     现场判据："没接齐 12 个舵机就不动"（播种要求 n_valid==12）。
 *
 *  取 300 = 3 s：给操作者一个"上电后不会乱动"的窗口，
 *  同时覆盖适配器启动 + 前几轮反馈建立的时间。
 *
 *  ⚠️ 之后回到正常闭环（角度域）。若要"永远不闭环"，
 *     应通过模式（IDLE）而不是拉长这个窗口。 */
#define RLCA_BOOT_HOLD_CYCLES         300u

/** ⭐ 上电后【只读不发位置】的时长（ms）。
 *
 *  ⚠️⚠️ 撞机事故的第二层机制，也是本项存在的理由：
 *     舵机把"目标位置"存在自己的非易失存储里。若它上次停在 step 2000，
 *     而固件第一条同步写让它去 step 0，它会【全速走 176°】。
 *     `link_limits_apply` 的 0.15 rad/周期限速【拦不住】这一跳 ——
 *     限速只约束"指令之间的变化"，不约束"舵机当前位置与首个指令的落差"。
 *
 *     更糟的是这条指令可能比适配器就绪还早，于是：
 *     位置写进去了、力矩却没关掉 → 舵机冲向那个陈旧目标。
 *
 *  取 1000 ms：覆盖适配器启动 + 让同步读先采到真实位置。
 *  ⚠️ 这段时间内电机【不会动】（只读事务 + 力矩已关），
 *     代价只是上电后 1 秒才开始闭环 —— 对调试完全可接受。 */
#define RLCA_SERVO_STARTUP_GATE_MS    1000u

/** 硬急停输入：
 *    0 = 不编译急停输入代码（本次范围，负责人已确认）
 *    1 = 启用 PB13 上拉输入 + EXTI13 下降沿 → 立即 ESTOP
 *  依据： docs/32-esp32-bridge.md §5.3；docs/33-firmware-plan.md §2③。 */
#define RLCA_ESTOP_INPUT_EN           0

/** 调试辅助（量周期耗时、打印总线统计等）。量产构建置 0。 */
#define RLCA_DEBUG_PERF               1

/** 上机自检命令行（走 USART1，ASCII 文本）。
 *  P2~P3 阶段用它验证 UART / I2C / Flash / LED 是否正常。
 *  ⚠️ link_pi 落地后必须置 0 —— 否则会与 0xAA55 二进制协议抢 USART1。 */
#define RLCA_SHELL_EN                 1

/** ⭐⭐ USART1 是否【兼作】shell 人机口（2026-10-18 加，默认 **0 = 不兼作**）
 *
 *  ⚠️⚠️ 上面那句"link_pi 落地后必须置 0"说的就是这个 —— 但那条 TODO 一直没做，
 *     于是 `bsp_shell_poll()` 里的 `sh_feed(&g_uart_pi, 0)` 持续从 **USART1 的
 *     接收环**里取字节，而 `link_pi_poll()` 读的是**同一个环**（同一个
 *     `bsp_uart_t.buf`，见 bsp_uart_getc）。两个消费者抢一条字节流，
 *     帧被切碎 ⇒ 解码器反复半帧超时。
 *
 *  ⭐ 现场证据（500 kbps，探针 30 s 只发了 2928 帧）：
 *        link_rx_timeouts = 5134   ← **比发出的帧还多**（在假的残帧上超时）
 *        link_crc_errors  = 788
 *        link_len_errors  = 426
 *        link_unknown_type = 0     ← 从没拼出完整的外来帧
 *     而**下行**（32→Pi）完美：`cycle_lost = 0`、CRC 0、半帧 0。
 *     ⇒ 若是 EMI 会**双向**同时坏；只有上行坏 ⇒ 不是 EMI，是上行被撕碎。
 *
 *  ⚠️ USART1 的定位是"Pi 协议链路"（二进制 0xAA55 帧），**不是人机口**。
 *     shell 走 USART3（ESP32 无线）就够用，也是你实际在用的那条。
 *  ⚠️ 若哪天要用 USB-TTL 有线调试 shell，把这里置 1 —— 但那时**不要接树莓派**，
 *     两者不能共存。 */
#define RLCA_SHELL_ON_PI_PORT         0

/** 上电自检：三路 UART 发一行提示 + LED 快闪 + 版本身份。
 *  置 0 可让上电更安静（P7 联调时可关）。 */
#define RLCA_BOOT_BANNER_EN           1

/** 空闲时是否维持舵机总线轮询（P3~P5 阶段：让上电后就能用 shell 调舵机）。
 *  ⚠️ P6 起由 app_mode 状态机决定是否下发力矩，本开关只控制"要不要轮询总线"。 */
#define RLCA_SERVO_IDLE_POLL_EN       1

/** 与树莓派联调用的 shell 命令（发送测试帧 / 统计）。
 *  ⚠️ shell 与协议帧【可以共存】于同一条 USART1 字节流：
 *     协议帧以 0xAA 开头，而 shell 只吃可打印 ASCII，
 *     所以 0xAA 永远进不了 shell 的命令缓冲。
 *     调完之后可用本开关单独关掉 link 命令。
 *
 *  ⚠️⚠️ 2026-10-11 临时置 0：Flash 告急。
 *     Keil GUI 构建（Optim=6）原本只剩 856 B，再加 DMA 诊断命令
 *     就直接报 L6406E「No space in execution regions」。
 *     关掉 L / g / G 三条【只用于树莓派联调】的命令腾出空间。
 *
 *     ⚠️ 恢复条件：等舵机总线的接收问题查清、把临时诊断命令
 *        （I/U/X/Y/Z/D/E）清理掉之后，再改回 1。
 *     ⚠️ 这只影响 shell 命令，不影响固件功能与协议帧收发。 */
#define RLCA_SHELL_LINK_CMDS_EN       0

/*==============================================================================
 * 1. 时序与频率
 *   docs/30-firmware.md §6.1
 *============================================================================*/

#define RLCA_CTRL_HZ                  100u          /* 控制环频率                 */
#define RLCA_CTRL_PERIOD_MS           (1000u / RLCA_CTRL_HZ)   /* = 10 ms       */

#define RLCA_SYS_CLK_HZ               72000000u     /* HSE 8 MHz × PLL9           */
#define RLCA_APB1_HZ                  36000000u
#define RLCA_APB2_HZ                  72000000u

#define RLCA_UART_PI_BAUD             500000u       /* USART1 ↔ 树莓派（2026-10-18 从 1M 降频，见 main.c） */
#define RLCA_UART_SERVO_BAUD          1000000u      /* USART2 ↔ 舵机控制板        */
#define RLCA_UART_ESP_BAUD            115200u       /* USART3 ↔ ESP32-C3（低频调试口）*/

/*==============================================================================
 * 2. 超时与阈值
 *   docs/30-firmware.md §6.2 / 协议规范 §7.2
 *============================================================================*/

#define RLCA_SERVO_FRAME_TIMEOUT_US   3000u   /* 舵机总线【整帧】超时 3 ms      */
#define RLCA_SERVO_TURNAROUND_US      20u     /* 半双工收发切换等待             */
#define RLCA_IMU_READ_TIMEOUT_US      5000u   /* IMU 单次读取超时 5 ms          */
#define RLCA_IMU_STALE_MS             400u    /* IMU 超过 400 ms 无新数据 = 失效 */

/** ⚠️⚠️ 为什么从 100 提到 400（2026-10-17 实测定位）
 *
 *  原值 100 ms 是按"IMU 应该 100 Hz 刷新"推的，但它没考虑一个事实：
 *  **shell 命令是阻塞的**（bsp_uart_write 同步发完才返回），期间控制循环完全停摆。
 *
 *  实测代价（ESP32 无线口 115200 baud）：
 *      'R' 输出约 1400 字符 ≈ 122 ms
 *      'K' 输出约 3150 字符 ≈ 274 ms
 *
 *  ⇒ 每按一次 R/K，下一轮 imu_poll() 都会看到 age > 100 ms
 *  ⇒ 清掉 valid ⇒ RLCA_SAFE_R_IMU_STALE ⇒ severity ≥ 2、block_active
 *  ⇒ 与 link_lost 的误判叠加，表现为"按一次 R 抖一次"
 *
 *  ⚠️ 取 400 而不是无限大：400 ms 对"策略的姿态输入"已经明显不可接受，
 *     但足够覆盖最长的那条命令（K ≈ 274 ms，留约 1.4 倍余量）。
 *
 *  ⚠️ 根治不在这里 —— 真正的修法是让 shell 输出不阻塞控制循环
 *     （UART3 加 TX 环形缓冲，或把 UART3 波特率从 115200 提到 921600，
 *      后者需要 ESP32 侧同步改）。本条只是把误伤窗口挡掉。 */

#define RLCA_LINK_CMD_WAIT_US         4000u   /* 模型 A：等 CMD_JOINT 4 ms      */

/** ⭐⭐ 链路失联阈值 —— 2026-10-17 按实测放大（原 30 / 100）
 *
 *  ⚠️⚠️ 为什么必须放大：**树莓派侧是普通 Linux + Python，不是实时系统。**
 *
 *  实测证据（16:18 版、Pi 保活 100 Hz、术语见 R 输出）：
 *      · Pi 侧 `发出 2235 帧 / 22 s ≈ 101 Hz` —— 发送周期名义上没问题
 *      · 而 32 侧 `link_lost_ms = 36`、`link_alive = 0` —— **空档已超过 30 ms**
 *      · `seq_lost` 约 8% —— 部分帧没被解析到
 *      · 面板上 ESTOP 约每秒跳一次
 *
 *  ⇒ 30 ms 阈值只有"3 个 10 ms 周期"的余量，而 Python 的 GIL（默认 5 ms
 *     切换间隔）、GC、往 SSH 管道 print、非 RT 内核调度，随便一个都能吃掉
 *     几十毫秒 ⇒ 阈值被常态化越过 ⇒ 降级/急停乱跳。
 *
 *  ⭐ 新值的依据：
 *     · LOST 80 ms = 8 个名义周期，能吸收常见的 30~60 ms 抖动
 *     · ESTOP 300 ms = 0.3 s。对四足来说仍足够快（人的反应是 200~300 ms）
 *     · 而且现在急停是【只卸力矩】（RLCA_SAFE_ESTOP_SITDOWN=0），
 *       不会主动折腿 —— 所以"慢一点才停"不会更危险
 *
 *  ⚠️ 这不是"放宽安全"，而是把阈值改成与**实际软件栈**匹配。
 *     真正断线（拔线、Pi 掉电）时 300 ms 内一样会停。
 *  ⚠️ 若日后 Pi 换成 PREEMPT_RT 并实测 P99 < 5 ms，可以再收回到 30/100。 */
/* ⚠️⚠️ 这两个值现在是【开发期放宽版】（2026-10-18），不是最终安全值。
 *
 *  原因：现场出现"约 6~16% 的周期进 ESTOP"，但已用数据证明
 *        · 上行零丢帧（rx_bytes = 帧数 × 38，seq_lost = 0）
 *        · 主循环最长只停 20 ms（poll_gap_max_us）
 *        · 电压 11.7 V、温度 31℃、servo_err = 0、imu_ok = 1
 *    却在 12 次进急停时 `link_level = 2` —— 与上面每一条都矛盾。
 *    这个矛盾必须先解开，否则 M2/M3 无法推进（STAND 会忽略上位机目标）。
 *
 *  改动：
 *    LOST  80 → 150 ms ：避免 20 ms 级周期抖动被误判成"降级"（ACTIVE↔STAND 抖动）
 *    ESTOP 300 → 1200 ms：真正的"Pi 挂了"是秒级事件，1.2 s 足够识别；
 *                        而 87 ms 级瞬态不该把 12 个关节的力矩全卸掉。
 *
 *  ⚠️ 安全边界【没有放宽】的东西：
 *    · 电池 ≤9300 mV → 卸力矩（VOLT_FORCE）         ← 保护电池，不动
 *    · 舵机 >70℃ → 卸力矩（OVERTEMP）              ← 保护舵机，不动
 *    · ESTOP 本身仍然存在，只是阈值更宽容            ← 保护"人"的那一层还在
 *
 *  ⚠️ 做完 M2/M3 的基本验证后，应当把这两个值改回 80 / 300 再做一次安全回归。
 *     改回方法：只改这两行，不用动别处。 */
#define RLCA_LINK_LOST_MS             150u    /* 连续 150 ms 无指令 → 降级      */
#define RLCA_LINK_ESTOP_MS            1200u   /* 连续 1200 ms 无指令 → 急停     */
#define RLCA_FRAME_RX_TIMEOUT_MS      5u      /* 收半个帧后 5 ms 复位状态机     */

/*==============================================================================
 * 3. 电池电压分级
 *   docs/30-firmware.md §6.4
 *   硬件：PA0 ← R1=100k / R2=20k 分压，C1=100nF
 *============================================================================*/

/** ⚠️⚠️ 分压比 —— 实测硬件是 100k / 20k，**不是** 100k / 22k（2026-10-18 更正）
 *
 *  分压比 = (R1 + R2) / R2
 *     100k + 20k  ⇒  120 / 20 = **6.000**   ← 实际硬件
 *     100k + 22k  ⇒  122 / 22 =   5.545     ← 原以为是这个（错）
 *
 *  ⚠️ 用错的比值会把读数压低：
 *     上报值 = 真实值 × (5.545 / 6.000) = 真实值 × 0.9242   ⇒ **低 7.6%**
 *     实测：上报 9948 mV 时真实约 **10764 mV**。
 *
 *  ⚠️ 后果（正是这轮排查被误导的原因之一）：
 *     电池其实还有 10.8 V，却因为读数偏低而：
 *       · 一直亮 BATT_WARN（≤10500）⇒ 面板一直告警
 *       · 逼近 BATT_STOP（≤9900）  ⇒ 触发幅度限幅 ⇒ 目标角被缩放
 *         ⇒ 表现为"关节到不了目标角 / 看起来像下垂"
 *     ⇒ 排查时很容易把它误判成"力矩不足"或"链路降级"。
 *
 *  ⚠️ 若日后换了电阻，务必同时改这两个数，并用万用表复核。 */
#define RLCA_BATT_DIV_NUM             120.0f  /* (100 + 20)                     */
#define RLCA_BATT_DIV_DEN             20.0f   /* 分压比 = 6.000                 */

/** ⭐⭐ 电压【限幅】总开关（2026-10-18 加）
 *
 *  ⚠️ 背景：分压比错的那段时间，电池读数偏低 7.6%，于是长期处于
 *     `VOLT_STOP` 区间 ⇒ `amplitude_scale` 被压 ⇒ 目标角被缩放。
 *     这会让"姿态跟踪"和"步态幅度"全部失真，干扰主线的调试。
 *
 *  ⭐ 语义（刻意只关"限幅"，不关"急停"）：
 *     · 0 = `VOLT_STOP`（≤9900 mV）**只上报，不限幅**
 *           ⇒ 电压低不再偷偷缩减动作幅度
 *     · 1 = 恢复限幅（原始行为）
 *
 *  ⚠️⚠️ **`VOLT_FORCE`（≤9300 mV → 卸力矩急停）不受本开关影响**，永远生效。
 *     理由：限幅是"性能调节"，急停是"安全底线"，两者不该共用一个开关。
 *     真没电时仍然会停，只是不再因为"电压偏低"而偷偷变小动作。
 *
 *  ⚠️ 分压比已改正，理论上不该再误触发；本开关留作调试期的保险。 */
#define RLCA_BATT_LIMIT_EN            0

/** ⚠️ 两步标定（必须做，见 固件设计书 §5.5）：
 *    ① 万用表量 3.3 V 引脚真实值 → 填这里（标称 3300，实际 3260~3340）
 *    ② 对比固件读数与万用表量到的电池电压 → 微调分压比，目标误差 < 50 mV */
#define RLCA_ADC_VREF_MV              3300.0f

#define RLCA_BATT_WARN_MV             10500u  /* L1：LED 慢闪 + 上报            */
#define RLCA_BATT_STOP_MV             9900u   /* L2：上报 + 限制动作幅度         */
#define RLCA_BATT_FORCE_MV            9300u   /* L3：卸载力矩 → 趴下            */

/** 采样与滤波 */
#define RLCA_ADC_FULL_SCALE           4095u   /* 12 位 ADC                      */
#define RLCA_BATT_FILTER_LEN          4u      /* 滑动平均长度（2 的幂，用移位） */
#define RLCA_BATT_SAMPLE_PERIOD_MS    100u    /* 10 Hz 采样足够（原设计 1 Hz 也行）*/
#define RLCA_BATT_STALE_MS            1000u   /* 超过 1 s 无有效采样 → 失效      */

/*==============================================================================
 * 3b. IMU（JY901P，软件 I2C）
 *   ⭐ 依据：`reference/JY901P_I2C协议.md`（WIT 官方 I2C 协议手册）
 *
 *  ⚠️ 本节的映射【已按手册核实】（2026-10-11）。
 *     早期版本是"按 WIT 标准协议的经验值猜的"，实测发现两处错：
 *       · 器件 ID 寄存器错取 0x00 —— 那是 SAVE（写它=重启/恢复出厂！），
 *         只读的 VERSION 在 0x2E。
 *       · 角度换算按 0.01°/LSB —— 手册是 raw/32768×180°，即 0.0055°/LSB，
 *         错了 1.82 倍。
 *     教训：这类寄存器表【必须查手册】，不能靠"同类模块通常一样"推断。
 *
 *  ⚠️ 手册要点（第 7~9 行）：每个地址内是 16 位数据、占 2 字节；
 *     连续地址 = 连续寄存器，所以多字节连读是合法的。
 *     I2C 最高 400 kHz；从机地址 7 位、默认 0x50。
 *============================================================================*/

#define RLCA_IMU_I2C_ADDR             0x50u   /* 7 位从机地址（手册 §IICADDR 默认 0x50）*/

/** 器件/版本寄存器。
 *  ⚠️⚠️ 不要用 0x00！手册里 0x00 是 **SAVE**（保存/重启/恢复出厂，R/W）——
 *     读它至少是无意义，写它 0xFF 会让模块重启、0x01 会恢复出厂设置。
 *     只读的版本号在 **0x2E**。 */
#define RLCA_IMU_REG_VERSION          0x2Eu   /* VERSION，只读，16 位            */

/** 加速度：AX@0x34 / AY@0x35 / AZ@0x36，各 16 位，连读 6 字节 */
#define RLCA_IMU_REG_ACC              0x34u

/** 角速度：GX@0x37 / GY@0x38 / GZ@0x39，各 16 位，连读 6 字节 */
#define RLCA_IMU_REG_GYRO             0x37u

/** 欧拉角：Roll@0x3D / Pitch@0x3E / Yaw@0x3F，各 16 位，连读 6 字节。
 *  ⚠️ 早期版本按 8 字节读（多读了 TEMP 与 D0Status），虽无害但没必要。 */
#define RLCA_IMU_REG_ANGLE            0x3Du

/** 四元数：q0@0x51 / q1@0x52 / q2@0x53 / q3@0x54，连读 8 字节。
 *  ⭐ 模块内部已融合好，直接给四元数 —— 比"读欧拉角再自己转四元数"
 *     少一次三角运算、也少一个出错点。
 *  ⚠️ 目前固件走欧拉角路径（已修好换算），四元数路径留作备用：
 *     若实测发现欧拉角的坐标约定与 Isaac Gym 不一致，改读这里即可。 */
#define RLCA_IMU_REG_QUAT             0x51u

/** RSW 输出内容寄存器（0x02）。默认 0x001E = TIME|ACC|GYRO|ANGLE。
 *  ⚠️ 我们还要四元数，所以需要把 QUATER 位（bit9）也打开。
 *     这是在 JY901P 上位机里设的，固件侧只记录要求。 */
#define RLCA_IMU_RSW_WANTED           0x021Eu /* bit9 QUATER | 默认 0x1E        */

/** RRATE 输出速率寄存器（0x03）。
 *  ⚠️⚠️ 手册默认 **0x06 = 10 Hz**，而控制环是 100 Hz ——
 *     不改成 0x09 就会把同一份数据读 10 次，姿态观测呈阶梯状、策略会抖。
 *     必须在上位机里设成 100 Hz。 */
#define RLCA_IMU_RRATE_100HZ          0x09u

/** 满量程（决定 LSB 的物理量）。
 *  ⚠️ 手册默认 GYRORANGE/ACCRANGE 与下面的取值一致（±2000°/s、±16 g）；
 *     若在上位机里改量程，这里必须同步改，否则整体换算系数就错了。 */
#define RLCA_IMU_ACC_FS_G             16.0f   /* ±16 g                          */
#define RLCA_IMU_GYRO_FS_DPS          2000.0f /* ±2000 °/s                      */
#define RLCA_IMU_INT16_FS             32768.0f

/** 欧拉角量程：手册是 raw/32768 × 180°（即 ±180° 对应 int16 满量程）。
 *  ⚠️ 不是 0.01°/LSB！0.01° 那套是【串口协议】的格式，I2C 寄存器不是。 */
#define RLCA_IMU_ANGLE_FS_DEG         180.0f

/** ⚠️「输出速率 ≥ 100 Hz」是在 JY901P 上位机里设的（I2C 没有波特率概念）。
 *  固件侧设不了，只能记录要求；上机时若姿态观测呈阶梯状，就是这一项没设够。
 *  对应寄存器 RRATE = 0x09。 */
#define RLCA_IMU_REQUIRED_RATE_HZ     100u

/*==============================================================================
 * 4. 关节：索引 / 舵机 ID / 方向 / 限位 / 零位
 *   docs/30-firmware.md §2、§3
 *============================================================================*/

#define RLCA_N_JOINTS                 12u

/** 逻辑索引 A（固件、协议、观测/动作向量统一顺序）：
 *    0..3   = 髋外摆 hip    ：lf, lb, rf, rb
 *    4..7   = 髋俯仰 thigh  ：lf, lb, rf, rb
 *    8..11  = 膝   calf     ：lf, lb, rf, rb
 *   ⚠️ 与 URDF 的声明顺序（按腿分组）不同，见 docs/30-firmware.md §2.2。 */
typedef enum {
    RLCA_J_LF_HIP = 0, RLCA_J_LB_HIP, RLCA_J_RF_HIP, RLCA_J_RB_HIP,
    RLCA_J_LF_THIGH,   RLCA_J_LB_THIGH, RLCA_J_RF_THIGH, RLCA_J_RB_THIGH,
    RLCA_J_LF_CALF,    RLCA_J_LB_CALF,  RLCA_J_RF_CALF,  RLCA_J_RB_CALF,
    RLCA_J_COUNT
} rlca_joint_t;

/** ⭐ 舵机 ID 分配（索引 i → ID i+1）。
 *  本项目由固件方决定，依据 docs/33-firmware-plan.md §3。
 *  ⚠️ 出厂默认 ID 都是 1，必须用 Feetech 上位机【逐个】改成下表的值
 *     （改完立刻拔掉、再接下一个），否则 12 个舵机会同时应答同一 ID。 */
#define RLCA_SERVO_ID_MIN             1u
#define RLCA_SERVO_ID_MAX             12u
#define RLCA_SERVO_ID(i)              ((uint8_t)((i) + 1u))

/* ⚠️ 关节方向表的定义见下方「标定姿态 = 装配姿态」一节：
 *    RLCA_JOINT_DIR_INIT —— 由项目负责人实测给出，不再通过微动测量。
 *    这里不再重复定义（原先"全部暂定 +1"的占位已删除）。 */

/** 关节限位（rad）—— 采用【训练 URDF】的值，见 docs/30-firmware.md §3.1/§3.2
 *    hip   ：  0     ~ +40°      （不对称，不做内摆）
 *    thigh ： −35°   ~ +90°      ⚠️ 训练用 90°，硬件 URDF 只到 30°；
 *                                   为给摆腿留空间，固件与训练保持一致
 *    calf  ： −60°   ~ +60°                                        */
#define RLCA_HIP_MIN_RAD              ( 0.0f)
#define RLCA_HIP_MAX_RAD              ( 0.6981317f)   /* +40°                  */
#define RLCA_THIGH_MIN_RAD            (-0.61086524f)   /* −35°                  */
#define RLCA_THIGH_MAX_RAD            ( 1.5707963f)   /* +90°                  */
#define RLCA_CALF_MIN_RAD             (-1.0471975f)   /* −60°                  */
#define RLCA_CALF_MAX_RAD             ( 1.0471975f)   /* +60°                  */

/** 默认关节角 q_default（rad）—— 全 0。
 *  仿真实测：q = 0 即机器人装配好的【弯腿站立姿态】。
 *  依据：Sim/RL/RL-CAT-训练实施报告.md §4.3；rlcat_config.py。 */
#define RLCA_Q_DEFAULT_RAD            0.0f

/*----------------------------------------------------------------------------
 * ⭐⭐⭐ 三个"零"必须分清（2026-10-16 定案，符号已按实测校正）
 *
 *  ┌────────────┬──────────────────────────┬──────────────┬─────────────┐
 *  │ 名字        │ 姿态                      │ 关节角        │ 舵机读数      │
 *  ├────────────┼──────────────────────────┼──────────────┼─────────────┤
 *  │ URDF 零位   │ 【弯腿站姿】（仿真 q_def=0）│ q = 0        │ = offset ⭐  │
 *  │ 标定姿态    │ 腿伸直、垂直向下、髋水平    │ q = ±q_calib │ = 2048（上位机设）│
 *  │ 编码器零    │ 舵机自己的 0 位            │ 无关          │ 0           │
 *  └────────────┴──────────────────────────┴──────────────┴─────────────┘
 *
 *  ⚠️⚠️ 转换公式：
 *        已知 step(q_calib) = 2048
 *        step(q) = dir × q/0.0001 × (4096/2π) + offset
 *      ⇒ offset = 2048 − dir × q_calib × (4096/2π)
 *
 *  ⚠️⚠️⚠️ q_calib 这里是【幅值，全部取正】（2026-10-16 校正）：
 *
 *     早期写成"带符号的关节角（左 − / 右 +）"，那会让 dir 与 q_calib 的符号
 *     互相抵消 —— 因为 dir 本身就编码了"关节角增大 → 编码器增大还是减小"。
 *     两个符号相乘等于把镜像信息用了两遍，于是左右两侧的 offset 跑到同一侧。
 *
 *     实测验证（项目负责人用 W 命令把关节送到 offset 看姿态）：
 *       j4 (lf_thigh, dir=+1, 44°)：off = 2048 − 512 = 1536  < 2048  ✅
 *       j6 (rf_thigh, dir=−1, 45°)：off = 2048 + 512 = 2560  > 2048  ✅
 *     取正值后左右分居 2048 两侧，与实测一致；取负值则都在同侧（错）。
 *
 *  ⚠️ 所以本表列的是【幅值】，符号完全交给 dir 表达。
 *----------------------------------------------------------------------------*/

#define RLCA_CALIB_POSE_THIGH_LF_DEG   (45.0f)   /* 幅值 */
#define RLCA_CALIB_POSE_THIGH_LB_DEG   (40.0f)
#define RLCA_CALIB_POSE_THIGH_RF_DEG   (45.0f)
#define RLCA_CALIB_POSE_THIGH_RB_DEG   (40.0f)
#define RLCA_CALIB_POSE_CALF_LF_DEG    (90.0f)
#define RLCA_CALIB_POSE_CALF_LB_DEG    (85.0f)
#define RLCA_CALIB_POSE_CALF_RF_DEG    (90.0f)
#define RLCA_CALIB_POSE_CALF_RB_DEG    (85.0f)

/** 标定姿态的 12 维关节角【幅值】（逻辑索引 A：hip×4, thigh×4, calf×4）。
 *  ⚠️ 全部非负；符号由 RLCA_JOINT_DIR_INIT 表达。 */
#define RLCA_CALIB_POSE_Q \
    { 0.0f, 0.0f, 0.0f, 0.0f,                                                   \
      (RLCA_CALIB_POSE_THIGH_LF_DEG * 0.0174532925f),                           \
      (RLCA_CALIB_POSE_THIGH_LB_DEG * 0.0174532925f),                           \
      (RLCA_CALIB_POSE_THIGH_RF_DEG * 0.0174532925f),                           \
      (RLCA_CALIB_POSE_THIGH_RB_DEG * 0.0174532925f),                           \
      (RLCA_CALIB_POSE_CALF_LF_DEG  * 0.0174532925f),                           \
      (RLCA_CALIB_POSE_CALF_LB_DEG  * 0.0174532925f),                           \
      (RLCA_CALIB_POSE_CALF_RF_DEG  * 0.0174532925f),                           \
      (RLCA_CALIB_POSE_CALF_RB_DEG  * 0.0174532925f) }

/** ⭐ 标定姿态下舵机的目标读数（项目负责人在上位机里设的）。
 *  固件用它反推 offset：offset = 本值 − dir × q_calib × SPR。 */
#define RLCA_CALIB_POSE_STEP           2048

/** ⭐⭐ 几何标定表开关（2026-10-18）
 *
 *  1 = 启动时用 `rlca_calib_build_geometric()` 由常量推导整张表，
 *      **完全不读 flash、不需要标定流程、不会判"未标定"**
 *  0 = 回到"从 flash 读，读不到就退化成全零默认表"的旧行为
 *
 *  ⚠️ 前提：标定姿态（腿伸直）下 12 个舵机编码器都读到 2048。
 *     项目负责人已在机械上逐台确认。
 *
 *  ⭐ 为什么默认 1：现场出现过"重启后有概率变成未标定"，根因是
 *     `s_calib_ok` 被捆在 flash 落盘成功与否上（落盘失败 ⇒ 明明有合法表
 *     却判未标定），而且失败的落盘是静默的。
 *     改成常量推导后，这条故障路径**从结构上消失**。
 *
 *  ⚠️ 代价：不再吸收逐台装配公差（本表与实测差约 0.2~11°，最大在 calf）。
 *     若日后需要逐台精修，把这个开关置 0 并用 `C start` 标定即可。 */
#define RLCA_CALIB_GEOMETRIC_EN        1

/** ⭐ 标定流程锁（2026-10-18）
 *
 *  1 = **禁止** `C start` / `C zero` / `RESTORE_DEFAULT`（只读的 R/K 不受影响）
 *  0 = 允许（原始行为）
 *
 *  ⚠️ 目的：防止误触把已经正确的表覆盖掉。用了几何表之后，
 *     标定流程本来就不需要了；留着入口只为"换舵机时"用 ——
 *     那种情况把本开关临时置 0、标定完再置回 1。
 *
 *  ⚠️ 编译期开关，**不能**在运行期随手关 —— 那会削弱这层保护的意义。 */
#define RLCA_CALIB_LOCK                1

/*----------------------------------------------------------------------------
 * ⭐⭐ 关节方向表 —— 由项目负责人实测给出（2026-10-16）
 *
 *  ⚠️⚠️ 方向不再通过"微动 +98 步看位移"测量（那个测试已删除）。
 *
 *  dir[i] 的物理含义（见 rlca_q_to_step）：
 *      关节角 q 增加时，舵机编码器读数是【增加(+1)】还是【减少(-1)】。
 *
 *  项目负责人给出的实测结论（按舵机 ID）：
 *    ┌─────────────────┬──────────────────────────────────────────┐
 *    │ 1, 4            │ 外摆时编码器【增加】        → dir = +1   │
 *    │ 2, 3            │ 外摆时编码器【减小】        → dir = -1   │
 *    │ 5, 6, 9, 10     │ 向前摆时编码器【增加】      → dir = +1   │
 *    │ 7, 8, 11, 12    │ 向后摆时编码器【减小】      → dir = -1   │
 *    └─────────────────┴──────────────────────────────────────────┘
 *
 *  映射到逻辑索引 A（索引 i → 舵机 ID i+1）：
 *    i=0  ID=1  hip   lf  → +1      i=6  ID=7  thigh rb → -1
 *    i=1  ID=2  hip   lb  → -1      i=7  ID=8  thigh ?  → -1
 *    i=2  ID=3  hip   rf  → -1      i=8  ID=9  calf  lf → +1
 *    i=3  ID=4  hip   rb  → +1      i=9  ID=10 calf  lb → +1
 *    i=4  ID=5  thigh lf  → +1      i=10 ID=11 calf  rf → -1
 *    i=5  ID=6  thigh lb  → +1      i=11 ID=12 calf  rb → -1
 *
 *  ⚠️ 按【ID 序号】分组可直接验证：
 *     1↔+1, 2↔-1, 3↔-1, 4↔+1, 5↔+1, 6↔+1, 7↔-1, 8↔-1,
 *     9↔-1, 10↔-1, 11↔+1, 12↔+1
 *
 *  ⚠️⚠️ 2026-10-16 第二次校正：calf（ID 9~12）的方向全部翻转。
 *     项目负责人用 W 命令逐个验证后给出：
 *       "这次发现 j8-j11 的 dir 全都反了，需要每个都颠倒"
 *     即 ID 9..12 的 dir 由 +1,+1,-1,-1 改为 -1,-1,+1,+1。
 *----------------------------------------------------------------------------*/

#define RLCA_JOINT_DIR_INIT \
    { +1, -1, -1, +1,   +1, +1, -1, -1,   -1, -1, +1, +1 }


/** 动作缩放：q_target = q_default + ACTION_SCALE × action
 *  依据：rlcat_config.py 的 control.action_scale = 0.25。 */
#define RLCA_ACTION_SCALE             0.25f

/** 单周期最大目标角变化（限速，最后一道防线）
 *  协议规范 §8.3：建议 0.15 rad ≈ 8.6°/周期 ≈ 860°/s */
#define RLCA_MAX_DELTA_PER_CYCLE_RAD  0.15f

/*==============================================================================
 * 5. 舵机（Feetech ST-3215-C018）
 *   docs/30-firmware.md §6.5
 *============================================================================*/

#define RLCA_SERVO_STEPS_PER_REV      4096.0f
#define RLCA_SERVO_STEPS_PER_RAD      (RLCA_SERVO_STEPS_PER_REV / 6.28318530718f)
#define RLCA_SERVO_RAD_PER_STEP       (6.28318530718f / RLCA_SERVO_STEPS_PER_REV)

/** 堵转扭矩 2.942 N·m @12 V；Present Load 的 0~1000 对应 0~100% 扭矩。
 *  ⇒ 每 LSB ≈ 2.942 / 1000 = 2.942e-3 N·m
 *  协议要求以 0.001 N·m 为单位上报，故换算系数 = 2.942（LSB → 0.001 N·m 单位）。 */
#define RLCA_SERVO_TORQUE_NM_PER_LSB  0.002942f
#define RLCA_TAU_UNIT_NM              0.001f

/** Present Speed 单位 = 步/秒 → rad/s：× 2π/4096 */
#define RLCA_SERVO_SPEED_RAD_PER_LSB  RLCA_SERVO_RAD_PER_STEP

#define RLCA_SERVO_OVERTEMP_C         70u     /* >70 °C → 降级 + 上报           */
#define RLCA_SERVO_DEFAULT_ACC        0u      /* 加速度：0 = 用舵机内部最大值    */
#define RLCA_SERVO_GOAL_SPEED         0u      /* 运行速度：0 = 用最大速度        */

/** 位置回绕判定阈值：相邻两次读数差超过半圈即认为回绕（4096/2 = 2048）
 *  依据：伺服总线模块设计 §9 */
#define RLCA_SERVO_POS_WRAP_HALF      2048

/** 温度上报单位 0.1 °C（协议 TEL_STATUS） */
#define RLCA_TEMP_UNIT_C              0.1f

/*==============================================================================
 * 6. 协议：帧格式 / 消息类型 / 缩放
 *   firmware/doc/协议规范_树莓派通信.md §2~§4
 *============================================================================*/

#define RLCA_FRAME_HEAD0              0xAAu
#define RLCA_FRAME_HEAD1              0x55u
#define RLCA_FRAME_MAX_PAYLOAD        128u   /* LEN 上限，超出即丢弃            */
#define RLCA_FRAME_OVERHEAD           7u     /* 头2 + LEN + SEQ + TYPE + CRC2   */

/* 消息类型 */
#define RLCA_TYPE_CMD_JOINT           0x01u
#define RLCA_TYPE_TEL_FAST            0x02u
#define RLCA_TYPE_CMD_MODE            0x03u
#define RLCA_TYPE_TEL_STATUS          0x04u
#define RLCA_TYPE_CMD_CALIB           0x05u
#define RLCA_TYPE_TEL_LOG             0x06u
#define RLCA_TYPE_TEL_ACK             0x07u
#define RLCA_TYPE_HEARTBEAT           0x7Fu

/* 各消息 PAYLOAD 长度（协议 §3 / §4） */
#define RLCA_LEN_CMD_JOINT            31u
#define RLCA_LEN_TEL_FAST             96u    /* ⭐ 含步态相位 + cycle_cnt      */
#define RLCA_LEN_CMD_MODE             2u
#define RLCA_LEN_TEL_STATUS           40u
#define RLCA_LEN_CMD_CALIB            1u
#define RLCA_LEN_TEL_ACK              2u

/* TEL_FAST 字段偏移（协议 §4.4） */
#define RLCA_TF_OFF_Q                 0u     /* int16[12]，0.0001 rad          */
#define RLCA_TF_OFF_QD                24u    /* int16[12]，0.0001 rad/s        */
#define RLCA_TF_OFF_TAU               48u    /* int16[12]，0.001 N·m           */
#define RLCA_TF_OFF_QUAT              72u    /* int16[4] w,x,y,z，0.0001       */
#define RLCA_TF_OFF_GYRO              80u    /* int16[3]，0.001 rad/s          */
#define RLCA_TF_OFF_GAIT_SIN          86u    /* int16，1/32767                 */
#define RLCA_TF_OFF_GAIT_COS          88u    /* int16，1/32767                 */
#define RLCA_TF_OFF_CYCLE_CNT         90u    /* uint16，100 Hz 周期计数         */
#define RLCA_TF_OFF_VOLTAGE           92u    /* uint16，mV                     */
#define RLCA_TF_OFF_STATUS            94u    /* uint8，位域                    */
#define RLCA_TF_OFF_ERR_FLAGS         95u    /* uint8，位域                    */

/* CMD_JOINT 字段偏移（协议 §4.1） */
#define RLCA_CJ_OFF_MODE              0u
#define RLCA_CJ_OFF_FLAGS             1u
#define RLCA_CJ_OFF_Q_TARGET          2u     /* int16[12]，0.0001 rad          */
#define RLCA_CJ_OFF_KP_SCALE          27u    /* int16，0.01                    */

/* 各字段的量化单位（协议 §4.1 / §4.4） */
#define RLCA_Q_UNIT_RAD               0.0001f
#define RLCA_QD_UNIT_RAD_S            0.0001f
#define RLCA_QUAT_UNIT                0.0001f
#define RLCA_GYRO_UNIT_RAD_S          0.001f
#define RLCA_GAIT_UNIT                (1.0f / 32767.0f)

/* 观测缩放（docs/30-firmware.md §5.1，⚠️ 与训练侧一致，勿改）
 *   注意：这几个缩放是【Pi 侧组装观测时用】的，固件只负责按上面的
 *   物理单位原样上报，不在固件里做观测缩放。这里保留常量供自检脚本比对。 */
#define RLCA_OBS_SCALE_GRAVITY        1.0f
#define RLCA_OBS_SCALE_ANG_VEL        0.25f
#define RLCA_OBS_SCALE_DOF_POS        1.0f
#define RLCA_OBS_SCALE_DOF_VEL        0.1f   /* ⚠️ upstream 是 0.05，本项目 0.1 */
#define RLCA_OBS_SCALE_ACTION         4.0f   /* = 1 / ACTION_SCALE             */

/* mode 枚举（协议 §4.2） */
typedef enum {
    RLCA_MODE_IDLE = 0,
    RLCA_MODE_STAND,
    RLCA_MODE_ACTIVE,
    RLCA_MODE_DAMP,
    RLCA_MODE_ESTOP
} rlca_mode_t;

/* CMD_JOINT.flags 位域（协议 §4.3） */
#define RLCA_FLAG_ENABLE              0x01u
#define RLCA_FLAG_RESET_ERR           0x02u
#define RLCA_FLAG_REQ_STATUS          0x04u
#define RLCA_FLAG_REQ_CALIB           0x08u

/* TEL_FAST.status / TEL_STATUS.status 位域（协议 §4.5） */
#define RLCA_ST_READY                 0x01u
#define RLCA_ST_CALIBRATED            0x02u
#define RLCA_ST_LINK_OK               0x04u
#define RLCA_ST_IMU_OK                0x08u
#define RLCA_ST_BATT_WARN             0x10u
#define RLCA_ST_BATT_STOP             0x20u
#define RLCA_ST_BATT_FORCE            0x40u
#define RLCA_ST_ESTOP                 0x80u

/* err_flags 位域（协议 §4.6） */
#define RLCA_ERRF_VOLTAGE             0x01u
#define RLCA_ERRF_ANGLE               0x02u
#define RLCA_ERRF_OVERHEAT            0x04u
#define RLCA_ERRF_OVERCURRENT         0x08u
#define RLCA_ERRF_ANGLE_LIMIT         0x10u
#define RLCA_ERRF_OVERLOAD            0x20u
#define RLCA_ERRF_BUS_TIMEOUT         0x40u
#define RLCA_ERRF_CALIB_INVALID       0x80u

/** ⭐⭐ 舵机总线错误位的【自动解除】时间（ms）。
 *
 *  ⚠️ 为什么需要（2026-10-16 修）：
 *     `s_err_flags` 原来只在 servo_bus_init()（= 上电）清零，
 *     运行期【永不清除】—— 一次瞬时的总线抖动就会把 BUS_TIMEOUT 永久钉住。
 *
 *     而 app_safety 里 `servo_err_flags != 0` ⇒ RLCA_SAFE_R_SERVO_ERR
 *     ⇒ amplitude_scale 压到 0.75。于是机器人整场跑都在 75% 幅度下，
 *     调试步态时非常误导（"为什么走不到位"）。
 *
 *     现场数据：bus_err 185 / bus_timeout 158 / uptime 202 s
 *     ⇒ 约 0.8% 的轮次偶发失败，属于正常的低速抖动，
 *       但它把整场的动作幅度都锁死在 75%。
 *
 *  ⭐ 新语义：任一错误位被置起后，若连续 N 毫秒【没有新的错误】，
 *     就全部清除。真正的持续故障会不断重新置位，所以不会漏报。
 *     ⇒ 从"latch"变成"最近是否还在出错"，这才是这个字段该有的含义。 */
#define RLCA_ERR_AUTOCLEAR_MS         2000u

/** ⭐⭐ 连续失败多少轮才置 BUS_TIMEOUT 错误位（2026-10-17 加）
 *
 *  ⚠️ 为什么需要：实测舵机总线约 **1%/轮** 会整轮收不到回复（偶发抖动）。
 *     旧实现"一次失败就置位"，于是：
 *        err_flags      在 0 / 0x40 之间跳
 *        safety_reasons 在 0 / 0x20(SERVO_ERR) 之间跳
 *        amp_scale      在 100% / 75% 之间跳
 *     ⇒ 面板上"错误"和"动作幅度"不停闪，而总线其实基本是好的。
 *
 *  ⭐ 语义拆分：
 *     us_timeout 计数   —— 每次失败都加，一个不漏（诊断用）
 *     BUS_TIMEOUT 错误位   —— 只有连续 N 轮失败才置（安全用）
 *
 *  ⚠️ 取 5：一轮 10 ms，5 轮 = 50 ms 收不到任何回复。
 *     那是"真的断了/接错/没上电"，不是抖动。
 *     真正断线时 50 ms 内就会报，不会漏。 */
#define RLCA_BUS_FAIL_STREAK_ERR      5u

/** ⭐⭐⭐ 急停（ESTOP）时怎么处置关节 —— 2026-10-17 项目负责人决定
 *
 *  0 = **只卸力矩**（当前默认）
 *  1 = 主动推进"趴下斜坡"（旧行为）
 *
 *  ⚠️ 为什么改成 0：
 *     · 舵机断电后自身有阻尼 + 减速比（ST-3215 空载 45 RPM，背驱阻力大），
 *       机器人会【缓慢塌下】，不会砸 —— 这本身就是安全的。
 *     · 而旧行为是"用剩余力矩把 12 个关节主动折向趴下姿态"（sit_down 斜坡）。
 *       那是一条**没人要求的运动**：链路一抖就折腿，调试时非常危险，
 *       现场表现为"按一下 shell 的 R/K 机器人就抖一下"。
 *     · 所以：急停 = 停止输出，而不是"换个姿态继续输出"。
 *
 *  ⚠️ 代价（必须知道）：
 *     站着/走着时若链路真的断了，机器人会**塌下来**而不是"缓缓蹲下"。
 *     对小四足 + 大减速比舵机，这是可接受的；若日后发现落地冲击大，
 *     把本宏改回 1 即可恢复旧行为（不需要改代码）。
 *
 *  ⚠️ 与"力矩锁"的关系：即使本项为 0，`s_armed == false` 时力矩本来就是关的，
 *     所以调试早期无论怎么进 ESTOP 都不会动。 */
#define RLCA_SAFE_ESTOP_SITDOWN       0

/* CMD_CALIB.action（协议 §4.9） */
#define RLCA_CALIB_START              0u
#define RLCA_CALIB_CANCEL             1u
#define RLCA_CALIB_QUERY              2u
#define RLCA_CALIB_SAVE               3u
#define RLCA_CALIB_RESTORE_DEFAULT    4u
/** ⭐ 固件扩展动作（协议 §4.9 只定义了上面 5 个）：
 *  操作者把机器人摆到【装配站姿】后发送本动作，固件立即采样当前舵机位置作为零点。
 *  ⚠️ 比全自动流程（卸力矩→靠自重沉→测方向）更准，因为姿态是人为摆正的。
 *  协议文档需要补这一条（见 docs/34-firmware-acceptance.md P6-2）。 */
#define RLCA_CALIB_MANUAL_ZERO        5u

/*==============================================================================
 * 7. 步态相位（⭐ 必须与训练侧严格一致）
 *   docs/30-firmware.md §5.1.1 / 协议规范 §4.4.1
 *   训练侧：Sim/legged_gym/.../rlcat_env.py:104-106
 *============================================================================*/

/** gait_freq = 3.0 Hz（rlcat_config.py 的 control.gait_freq）
 *  每【控制周期】(100 Hz) 相位推进 3.0/100 = 0.03
 *  一个完整周期 = 1/0.03 ≈ 33.3 控制周期 = 0.333 s → 3.0 Hz ✅ */
#define RLCA_GAIT_FREQ_HZ             3.0f
#define RLCA_PHASE_STEP               (RLCA_GAIT_FREQ_HZ / (float)RLCA_CTRL_HZ)

/** 正弦表项数：用 256 段 + 1（首尾重合），整数索引查表，避免浮点三角函数。
 *  精度：相位量化到 1/256 圈 ≈ 0.0122 rad；sin 值误差 < 3e-4，
 *  编码到 int16（÷32767）后误差 < 10 LSB —— 远小于策略敏感度。 */
#define RLCA_SIN_TABLE_BITS           8u
#define RLCA_SIN_TABLE_SIZE           (1u << RLCA_SIN_TABLE_BITS)   /* 256 */
#define RLCA_SIN_TABLE_MASK           (RLCA_SIN_TABLE_SIZE - 1u)

/*==============================================================================
 * 8. 缓冲区尺寸（⚠️ 20 KB RAM 最紧张，改动前先算总量）
 *   固件设计书 §2.1 的预算表
 *============================================================================*/

#define RLCA_UART1_RX_BUF             512u   /* 树莓派 → STM32（TEL/CMD 双向）  */
#define RLCA_UART1_TX_BUF             256u
#define RLCA_UART2_RX_BUF             512u   /* 舵机总线反馈（12 帧 ≈ 240 B）   */
#define RLCA_UART2_TX_BUF             256u
#define RLCA_UART3_RX_BUF             256u   /* ESP32 → STM32（调试注入）        */

/*==============================================================================
 * 8b. UART3（ESP32）转发遥测
 *   docs/32-esp32-bridge.md §0/§3.2/§4.3；docs/36-esp32-plan.md §2.2
 *
 *  ESP32 的定位是【纯字节桥】：UART3 ↔ TCP 逐字节原样搬，不解析整帧。
 *  所以 STM32 这边做的是"把发给树莓派的同一帧【原样镜像】一份到 UART3"，
 *  而不是另发明一种格式。
 *
 *  ⚠️⚠️ 为什么是 115200 + 10 Hz + 中断发送（三个必须同时成立）：
 *     这条链路定位是【低频调试/观察通道】，不是控制通路。
 *     若用阻塞发送，103 B 的耗时 = 103 × 10 / 波特率：
 *         921600 → 1.12 ms     （实测把 100 Hz 环拖到 ~92 Hz）
 *         115200 → 8.93 ms     （会砸到 ~53 Hz，控制环崩溃）
 *     而 RL 的 decimation=2 要求控制环【严格 100 Hz】，
 *     所以发送必须非阻塞（中断），且必须降频到 10 Hz 省带宽。
 *============================================================================*/

/** 转发总开关。 */
#define RLCA_ESP_FWD_EN               1

/** 镜像降频：控制周期数。10 = 100 Hz → 10 Hz。
 *
 *  ⚠️ 115200 的线速率约 11.5 KB/s：
 *       100 Hz 全量 = 10.3 KB/s = 90%  → 必然丢帧
 *        10 Hz      = 1.5 KB/s  = 13%  → 余量充足
 *  ⚠️ 10 Hz 完全够用：ESP32 的仲裁过期阈值是 500 ms，
 *     100 ms 一帧有 5 倍余量（32-esp32-bridge.md §4.3）。 */
#define RLCA_ESP_FWD_DECIM            10u

/** 发送卡死看门狗（ms）。
 *
 *  ⚠️ 中断发送本身不阻塞，所以这里【不是】发送超时，而是防"上一个中断
 *     永远完不成"的兜底：若 ESP32 侧停止读取、或线路异常导致 TC 中断
 *     不来，busy 标志会一直占着，从此再也发不出任何帧。
 *     超过这个时间就强制复位 busy（丢一帧），让链路能自愈。
 *     取 100 ms = 10 个镜像周期，绝不会误伤正常发送。 */
#define RLCA_ESP_TX_STALL_MS          100u

/** 帧长上限（103 = 96 载荷 + 7 开销），用于静态发送缓冲 */
#define RLCA_ESP_TX_MAX_FRAME         (RLCA_FRAME_OVERHEAD + RLCA_LEN_TEL_FAST)

#define RLCA_SERVO_DMA_RX_BUF         512u   /* USART2_RX DMA 循环缓冲          */

/*==============================================================================
 * 9. Flash 标定参数存储
 *   固件设计书 §6.3；F103C8 Flash 页大小 1 KB
 *============================================================================*/

#define RLCA_CALIB_FLASH_ADDR         0x0800FC00u   /* 最后一页                */
#define RLCA_CALIB_FLASH_PAGE_SIZE    1024u
#define RLCA_CALIB_MAGIC              0x524C4341u   /* "RLCA"                  */
#define RLCA_CALIB_VERSION            1u

/*==============================================================================
 * 10. 故障注入（仅测试用；量产置 0）
 *============================================================================*/

#define RLCA_TEST_FAULT_INJECT        0

#endif /* RLCA_CONFIG_H */
