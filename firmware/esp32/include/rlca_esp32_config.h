/**
 * RL-CAT ESP32-C3 wireless bridge -- central parameter table.
 *
 * Every tunable of this firmware lives here; no magic numbers elsewhere.
 * Sources (do not invent values):
 *   docs/32-esp32-bridge.md                    ESP32 design + decisions
 *   docs/30-firmware.md §6                     link speeds, timeouts, thresholds
 *   docs/20-electronics.md §2.1/§4             USART3 wiring, 921600
 *   firmware/doc/协议规范_树莓派通信.md          frame layout, message types, offsets
 *   docs/33-firmware-plan.md §3                joint index <-> servo ID (index = ID-1)
 *
 * @note The STM32 <-> ESP32 link reuses the *same* frame protocol as
 *       Pi <-> STM32; nothing is added or changed here.
 */
#pragma once

/* ------------------------------------------------------------------ */
/* identity                                                            */
/* ------------------------------------------------------------------ */
#define RLCA_CFG_VERSION        "0.5.0"

/* ------------------------------------------------------------------ */
/* WiFi AP + TCP server            (32-esp32-bridge.md §3.3)           */
/* ------------------------------------------------------------------ */
/* SSID = prefix + last 4 hex digits of the station MAC (multi-robot safe). */
#define RLCA_AP_SSID_PREFIX     "RL-CAT-"

/* ⚠️⚠️ 【开源前修改】默认 WPA2 口令 —— 请务必改掉再上机！
 *
 *  ⚠️ 这是**公开在仓库里的默认值**。所有 clone 本项目的机器人出厂都是同一个口令，
 *     局域网内任何人都能连上你的控制口、直接向 STM32 下发关节指令。
 *
 *  ⭐ 改法（三选一）：
 *     ① 直接改下面的字符串（最简单，但会把自己的口令提交进仓库）
 *     ② 编译期覆盖（推荐）：
 *          pio run -e esp32-c3 -D RLCA_AP_PASSWORD='"你的口令"'
 *     ③ 运行时通过串口命令设置（若实现了持久化，见 cfg.c 的 ap_pw 键）
 *
 *  ⚠️ WPA2 要求 8~63 字符。
 *  ⚠️ 下面的 `#ifndef` 保护是为了让 ② 生效 —— 不要删掉它。 */
#ifndef RLCA_AP_PASSWORD
#define RLCA_AP_PASSWORD        "rlcat1234"   /* ⚠️ 默认值，请改！WPA2, >= 8 chars */
#endif
#define RLCA_AP_CHANNEL         0             /* 0 = auto scan */
#define RLCA_AP_MAX_STA         4
#define RLCA_AP_IP              "192.168.4.1"

/* Port 3333: pure byte pipe between the laptop and the STM32.
 * Port 3334: ESP32 text commands + ESP32 logs + live stats.
 * Rationale for splitting (deviation from 32-esp32-bridge.md §4.1, recorded in
 * docs/90-decisions.md): it keeps 3333 a clean binary pipe, so a client never
 * has to parse frames to tell STM32 traffic from ESP32 log text. */
#define RLCA_TCP_PORT_BRIDGE    3333
#define RLCA_TCP_PORT_CONSOLE   3334
#define RLCA_TCP_MAX_CLIENTS    4             /* per port */
#define RLCA_TCP_TX_TIMEOUT_MS  50            /* send() gives up, frame dropped+counted */
#define RLCA_TCP_RX_CHUNK       512

/* ------------------------------------------------------------------ */
/* STM32 link: USART3, cross-wired, 921600                (20-electronics) */
/* ------------------------------------------------------------------ */
/* ESP32 TX -> STM32 PB11 (USART3_RX);  ESP32 RX <- STM32 PB10 (USART3_TX).
 * UART1 is used (not UART0) so the USB-Serial-JTAG console stays available.
 *
 * ⚠️ Pin choice follows the actual RL-CAT PCB: it routes the ESP32-C3 module's
 *    default serial pins (RXD = GPIO20, TXD = GPIO21) to the STM32's USART3.
 *    GPIO6/GPIO7 are NOT routed on that board (verified against the schematic),
 *    which is why a firmware using them receives nothing at all.
 *    Runtime override: console command `cfg pins <tx> <rx>` (stored in NVS).
 * NOTE: GPIO11..17 are taken by the embedded flash, 18/19 are USB D+/D-,
 *       2/8/9 are strapping pins (9 = BOOT) -- do not route them here. */
#define RLCA_UART_PORT          1
#define RLCA_UART_TX_GPIO       21
#define RLCA_UART_RX_GPIO       20
/* 115200 by default: the ESP32 link is a *low rate* debug/observation channel
 * (decided with the project owner), chosen for robustness over jumper wires.
 * The USART3 contract value 921600 is still available at runtime with
 * `cfg baud 921600` (NVS) -- but see docs/36-esp32-plan.md §2.2: at 115200 a
 * 103 B TEL_FAST takes 8.9 ms, so the STM32 must mirror with interrupt/DMA
 * transmit and at a low rate, never with a blocking write inside the 100 Hz loop. */
#define RLCA_UART_BAUD          115200

#define RLCA_UART_RX_RING       4096          /* >= 1 full 103 B frame, comfortably */
#define RLCA_UART_TX_RING       4096
#define RLCA_UART_READ_CHUNK    256
#define RLCA_UART_RX_TIMEOUT_MS 20            /* driver read timeout */

/* ------------------------------------------------------------------ */
/* frame protocol (协议规范_树莓派通信.md §2) -- header only, no changes */
/* ------------------------------------------------------------------ */
#define RLCA_FRAME_HEAD0        0xAA
#define RLCA_FRAME_HEAD1        0x55
#define RLCA_FRAME_MAX_PAYLOAD  128           /* LEN > 128 => reject, resync */
#define RLCA_FRAME_OVERHEAD     7             /* 2 head + LEN + SEQ + TYPE + 2 CRC */
#define RLCA_FRAME_MAX_TOTAL    (RLCA_FRAME_OVERHEAD + RLCA_FRAME_MAX_PAYLOAD)

/* message types (§3) */
#define RLCA_TYPE_CMD_JOINT     0x01
#define RLCA_TYPE_TEL_FAST      0x02
#define RLCA_TYPE_CMD_MODE      0x03
#define RLCA_TYPE_TEL_STATUS    0x04
#define RLCA_TYPE_CMD_CALIB     0x05
#define RLCA_TYPE_TEL_LOG       0x06
#define RLCA_TYPE_TEL_ACK       0x07
#define RLCA_TYPE_HEARTBEAT     0x7F

/* payload lengths (§3) */
#define RLCA_LEN_CMD_JOINT      31
#define RLCA_LEN_TEL_FAST       96
#define RLCA_LEN_CMD_MODE       2
#define RLCA_LEN_TEL_STATUS     40
#define RLCA_LEN_CMD_CALIB      1
#define RLCA_LEN_TEL_ACK        2
#define RLCA_LEN_HEARTBEAT      0

/* TEL_FAST payload-relative offsets (§4.4).
 * Frame byte offset = 5 + payload offset. */
#define RLCA_TF_OFF_Q           0     /* int16[12]  0.0001 rad   */
#define RLCA_TF_OFF_QD          24    /* int16[12]  0.0001 rad/s */
#define RLCA_TF_OFF_TAU         48    /* int16[12]  0.001 N.m    */
#define RLCA_TF_OFF_QUAT        72    /* int16[4]   w,x,y,z 1e-4 */
#define RLCA_TF_OFF_GYRO        80    /* int16[3]   0.001 rad/s  */
#define RLCA_TF_OFF_GAIT_SIN    86    /* int16      1/32767      */
#define RLCA_TF_OFF_GAIT_COS    88    /* int16      1/32767      */
#define RLCA_TF_OFF_CYCLE       90    /* uint16     100 Hz counter */
#define RLCA_TF_OFF_VOLTAGE     92    /* uint16     mV           */
#define RLCA_TF_OFF_STATUS      94    /* uint8      bitfield     */
#define RLCA_TF_OFF_ERR         95    /* uint8      bitfield     */
#define RLCA_TF_STATUS_FRAME_OFF (5 + RLCA_TF_OFF_STATUS)

/* TEL_FAST quantisation */
#define RLCA_Q_SCALE            0.0001f
#define RLCA_QD_SCALE           0.0001f
#define RLCA_TAU_SCALE          0.001f
#define RLCA_QUAT_SCALE         0.0001f
#define RLCA_GYRO_SCALE         0.001f

/* status bits (§4.5) */
#define RLCA_STAT_READY         (1u << 0)
#define RLCA_STAT_CALIBRATED    (1u << 1)
#define RLCA_STAT_LINK_OK       (1u << 2)   /* Pi (USART1) alive -- NOT any link */
#define RLCA_STAT_IMU_OK        (1u << 3)
#define RLCA_STAT_BATT_WARN     (1u << 4)
#define RLCA_STAT_BATT_STOP     (1u << 5)
#define RLCA_STAT_BATT_FORCE    (1u << 6)
#define RLCA_STAT_ESTOP         (1u << 7)

/* err_flags bits (§4.6) */
#define RLCA_ERR_VOLTAGE        (1u << 0)
#define RLCA_ERR_ANGLE          (1u << 1)
#define RLCA_ERR_OVERHEAT       (1u << 2)
#define RLCA_ERR_OVERCURRENT    (1u << 3)
#define RLCA_ERR_ANGLE_LIMIT    (1u << 4)
#define RLCA_ERR_OVERLOAD       (1u << 5)
#define RLCA_ERR_BUS_TIMEOUT    (1u << 6)
#define RLCA_ERR_CALIB_INVALID  (1u << 7)

/* CMD_JOINT payload-relative offsets (§4.1) */
#define RLCA_CJ_OFF_MODE        0
#define RLCA_CJ_OFF_FLAGS       1
#define RLCA_CJ_OFF_Q           2     /* int16[12] 0.0001 rad, logical index A */
#define RLCA_CJ_OFF_KP_SCALE    27    /* int16 0.01 */
#define RLCA_CJ_OFF_LED         29
#define RLCA_CJ_Q_SCALE         0.0001f

/* mode enum (§4.2) */
#define RLCA_MODE_IDLE          0
#define RLCA_MODE_STAND         1
#define RLCA_MODE_ACTIVE        2
#define RLCA_MODE_DAMP          3
#define RLCA_MODE_ESTOP         4
#define RLCA_MODE_MAX           RLCA_MODE_ESTOP

/* flags bitfield (§4.3) */
#define RLCA_FLAG_ENABLE        (1u << 0)
#define RLCA_FLAG_RESET_ERR     (1u << 1)
#define RLCA_FLAG_REQ_STATUS    (1u << 2)
#define RLCA_FLAG_REQ_CALIB     (1u << 3)

/* ------------------------------------------------------------------ */
/* joint layout (30-firmware.md §2.1) -- logical index A, index = servoID-1 */
/* ------------------------------------------------------------------ */
#define RLCA_NUM_JOINTS         12
#define RLCA_Q_DEFAULT_RAD      0.0f          /* q=0 is the standing pose */

/* ------------------------------------------------------------------ */
/* arbitration (32-esp32-bridge.md §4.3)                               */
/* ------------------------------------------------------------------ */
/* When the Pi is online (STAT_LINK_OK set in a CRC-valid TEL_FAST), the ESP32
 * must NOT inject CMD_JOINT / CMD_MODE. Query-type commands stay allowed. */
#define RLCA_ARB_ENABLE             1
#define RLCA_ARB_STATUS_STALE_MS    500   /* no valid TEL_FAST for this long =>
                                             Pi state unknown => injection blocked */
#define RLCA_ARB_BLOCK_ON_UNKNOWN   1     /* fail safe: unknown => block, not allow */
#define RLCA_ARB_POLL_MS            5     /* frame-timeout / age refresh period */
#define RLCA_TASK_STACK_ARB         3072
#define RLCA_TASK_PRIO_ARB          6

/* one `s` command may move a joint by at most this much */
#define RLCA_STEP_MAX_DEG           30.0f
#define RLCA_DEG2RAD                0.0174532925f

/* ------------------------------------------------------------------ */
/* injection limits / robustness (32-esp32-bridge.md §6)              */
/* ------------------------------------------------------------------ */
#define RLCA_INJECT_MAX_PER_SEC     200   /* frames/s, protects the STM32 USART3 RX */
#define RLCA_INJECT_MIN_GAP_MS      2
#define RLCA_BRIDGE_STATS_MS        10000 /* console stats print interval */

/* ------------------------------------------------------------------ */
/* tasks / buffers                                                     */
/* ------------------------------------------------------------------ */
#define RLCA_TASK_STACK_NET           4096
#define RLCA_TASK_PRIO_NET            5     /* sockets + console */
#define RLCA_TASK_STACK_BRIDGE_RX     3072
#define RLCA_TASK_PRIO_BRIDGE_RX      10    /* byte pump: above the net task */
#define RLCA_TASK_STACK_BRIDGE_EVT    2560
#define RLCA_TASK_PRIO_BRIDGE_EVT     9     /* UART error events */
#define RLCA_UART_EVT_QUEUE           20    /* UART event queue depth */
#define RLCA_UART_EVT_WAIT_MS         1000  /* bounded wait so the TWDT gets fed */
#define RLCA_UART_TX_LOCK_MS          20    /* mutex wait before dropping a write */
#define RLCA_UART_LOOPBACK_WAIT_MS    100   /* power-on internal loopback timeout */
#define RLCA_TCP_BACKLOG              2
#define RLCA_NET_POLL_MS              200   /* poll() timeout */

/* ------------------------------------------------------------------ */
/* console (port 3334) command set (32-esp32-bridge.md §4.2, 'e' removed) */
/* ------------------------------------------------------------------ */
/* ?            help
 * i            request one TEL_STATUS from the STM32 (flags FLAG_REQ_STATUS)
 * s <j> <ddeg> move joint <j> (logical index 0..11) by <ddeg> degrees,
 *              relative to its last reported position (reuses CMD_JOINT;
 *              no protocol change, works before calibration)
 * z            unload torque      (CMD_MODE IDLE)
 * c            enter calibration  (CMD_CALIB action 0)
 * r            reset/re-sync local state
 * k            keep-alive
 * q            close console connection
 * stats        link / bridge counters
 * raw <hex>    push arbitrary bytes to the STM32 (bench test / simulator)
 * hb           send a HEARTBEAT frame (protocol smoke test)
 * NOTE: 'e' (soft e-stop) is explicitly out of scope for this round.
 */
#define RLCA_CONSOLE_MAX_LINE       256   /* must fit `raw <hex of a 103 B frame>`:
                                             "raw " + 206 hex chars = 210 */
#define RLCA_CONSOLE_REPLY_MAX      256
#define RLCA_CONSOLE_PROMPT         "rlca> "

/* ------------------------------------------------------------------ */
/* robustness (P4)                                                     */
/* ------------------------------------------------------------------ */
/* Task watchdog: the latency critical tasks (UART pump, socket set, arbiter)
 * subscribe and reset it.  A hung task is far more dangerous than a reboot --
 * the STM32 has its own link-lost protection (30 ms degrade / 100 ms damp), so
 * rebooting this bridge cannot leave the robot in a bad state. */
#define RLCA_TWDT_ENABLE            1
#define RLCA_TWDT_TIMEOUT_S         5
#define RLCA_TWDT_PANIC             1
#define RLCA_TWDT_OFF               0     /* value used where a task opts out */

/* ------------------------------------------------------------------ */
/* build switches                                                      */
/* ------------------------------------------------------------------ */
#define RLCA_SELFTEST_ON_BOOT       1     /* run protocol/CRC test vectors at boot */
#define RLCA_HEARTBEAT_LOG_MS       1000  /* P0 scaffold: periodic "alive" log */
