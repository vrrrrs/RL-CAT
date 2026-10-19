/**
  ******************************************************************************
  * @file    bsp_shell.c
  * @brief   极简命令行实现（上机自检用）。
  ******************************************************************************
  */

#include <string.h>

#include "bsp_shell.h"
#include "bsp_uart.h"
#include "bsp_tick.h"
#include "bsp_gpio.h"
#include "bsp_softi2c.h"
#include "bsp_flash.h"
#include "rlca_frame.h"
#include "servo_bus.h"
#include "link_pi.h"
#include "link_proto.h"
#include "battery.h"
#include "imu.h"
#include "esp_fwd.h"
#include "app_main.h"
#include "app_mode.h"
#include "app_safety.h"
#include "rlca_calib_proc.h"
#include "rlca_phase.h"

#if RLCA_SHELL_EN

/*==============================================================================
 * 输出辅助
 *
 *  ⭐⭐ 双端口（2026-10-11 新增）：所有输出走【当前输入端口】，
 *     而不是写死 USART1。
 *
 *  ⚠️ 为什么：USART1(PA9/PA10) 在整机装好后手伸不进去，
 *     而 ESP32 提供了 WiFi 透传（TCP 3333 → USART3）。
 *     让 shell 也读 USART3，就能无线敲全部调试命令。
 *
 *  ⚠️ 绝不能让回显写死 USART1：那样从无线进来的命令，
 *     输出会从有线口跑掉，无线端只能看到"没反应"。
 *============================================================================*/

/** 当前端口：命令从哪条线进来的，输出就回哪条线。 */
static bsp_uart_t *s_cur = NULL;

/*==============================================================================
 * ⭐⭐ shell 输出期间的"接收保命"（2026-10-17 修的关键 bug）
 *
 *  ⚠️⚠️ 现场确诊：`t` 的分步计时抓到
 *        shell_poll     = 169813 us   ← 一条命令阻塞 170 ms
 *        poll_gap_max_us = 175049 us  ← 与它完全对上
 *     而 `max_us`（周期内部）只有 15 ms、`cycle_100hz = 15917 us`。
 *
 *  ⇒ 阻塞在 shell 把输出打到 UART3（115200 阻塞写）。
 *     输出多长就阻塞多久：'R' ≈ 1400 字符 ≈ 122 ms，'K' ≈ 3150 字符 ≈ 274 ms。
 *
 *  ⭐ 后果链（这才是"关节来回摆"的真因）：
 *      shell 阻塞 170 ms
 *        → `link_pi_poll()` 停 170 ms（它只在 `app_poll()` 里跑）
 *        → `s_last_rx_ms` 冻结
 *        → `link_pi_is_alive()` 为假 ⇒ STAT_LINK_OK 熄灭
 *        → ACTIVE 准入门第 ② 条挡住 ⇒ 降级 STAND
 *        → STAND 忽略 Pi 目标、保持 q_def = 0 ⇒ 腿往 0 掉
 *        → 阻塞结束 ⇒ 恢复 ACTIVE ⇒ 腿弹回目标
 *        ⇒ **看起来就是关节在往复摆动**
 *
 *  ⭐ 修法：`sh_puts()` 每打一小段就顺手把接收解析跑一次。
 *     · 只是把"本来该做的事"插到阻塞中间做，不改变任何逻辑
 *     · `link_pi_poll()` 不递归、可重入安全
 *     · 输出总时长不变（串口带宽是物理限制），但 `s_last_rx_ms`
 *       全程保持新鲜 ⇒ LINK_OK 不会因为"我在看诊断输出"而熄灭
 *
 *  ⚠️ 彻底做法是给 UART3 加 TX 环形缓冲，那要改中断与缓冲区管理；
 *     本条是 20 行内见效的正确折中。 */

/** 每输出这么多字节，就让接收解析跑一次（115200 下约 5 ms）。 */
#define SH_PUMP_CHUNK  64u

static void sh_puts(const char *s)
{
    /* ⚠️ 启动横幅在 bsp_shell_init() 里打印，那时还没有输入端口，
     *    默认走 USART1（有线口），保持既有行为。 */
    bsp_uart_t *port = (s_cur != NULL) ? s_cur : &g_uart_pi;
    const uint16_t n = (uint16_t)strlen(s);

    if (n <= SH_PUMP_CHUNK) {
        (void)bsp_uart_write(port, (const uint8_t *)s, n);
        return;
    }

    uint16_t off = 0u;
    while (off < n) {
        uint16_t take = (uint16_t)(n - off);
        if (take > SH_PUMP_CHUNK) {
            take = SH_PUMP_CHUNK;
        }
        (void)bsp_uart_write(port, (const uint8_t *)(s + off), take);
        off = (uint16_t)(off + take);

        /* ⭐ 让 Pi 链路的接收解析在 shell 输出期间也能推进
         *   ⇒ `s_last_rx_ms` 保持新鲜 ⇒ LINK_OK 不熄灭。
         *   ⚠️ 启动横幅（s_cur == NULL）时没有链路要保，不必打断。 */
        if (s_cur != NULL) {
            link_pi_poll();
            bsp_uart_poll_dma();
        }
    }
}

#define SH_LINE_MAX  96u
/* ⚠️ 行缓冲的定义挪到文件末尾（需要 SH_N_PORTS，且要 2 份）——
 *    见 bsp_shell_poll 上方的 sh_line[SH_N_PORTS][SH_LINE_MAX]。 */

static void sh_put_u32(uint32_t v)
{
    char b[11];
    uint8_t i = 0u;
    if (v == 0u) { b[i++] = '0'; }
    while ((v > 0u) && (i < sizeof(b))) {
        b[i++] = (char)('0' + (v % 10u));
        v /= 10u;
    }
    char out[12];
    uint8_t n = 0u;
    while (i > 0u) { out[n++] = b[--i]; }
    out[n] = '\0';
    sh_puts(out);
}

static void sh_kv(const char *k, uint32_t v)
{
    sh_puts(k);
    sh_puts(" = ");
    sh_put_u32(v);
    sh_puts("\r\n");
}

/** 定点打印浮点数（3 位小数）。
 *  ⚠️ 刻意【不用 printf 的 %f】：设计书 §10 明确"不做浮点 printf"
 *     —— Flash 和栈都紧张，vfprintf 会把几百字节的格式化代码拉进来。
 *     这里只用整数运算。 */
static void sh_put_float(float v)
{
    /* NaN / Inf 的防御：直接按 0 打印并标注，避免打印出乱码 */
    if (v != v) { sh_puts("nan"); return; }
    if (v > 1.0e30f) { sh_puts("inf"); return; }
    if (v < -1.0e30f) { sh_puts("-inf"); return; }

    const bool neg = (v < 0.0f);
    /* 放大 1000 倍后四舍五入到整数 */
    const float scaled = neg ? (-v * 1000.0f) : (v * 1000.0f);
    uint32_t x = (uint32_t)(scaled + 0.5f);

    const uint32_t ip = x / 1000u;
    const uint32_t fp = x % 1000u;

    if (neg) { sh_puts("-"); }
    sh_put_u32(ip);
    sh_puts(".");
    /* 补前导零到 3 位 */
    if (fp < 100u) { sh_puts("0"); }
    if (fp < 10u)  { sh_puts("0"); }
    sh_put_u32(fp);
}

/** 打印带符号十进制（端口用）。 */
static void sh_put_i32(int32_t v)
{
    if (v < 0) {
        sh_puts("-");
        /* ⚠️ 不能写 -v：INT32_MIN 取负会溢出。转到 uint32 再取负。 */
        sh_put_u32((uint32_t)(-(int64_t)v));
    } else {
        sh_put_u32((uint32_t)v);
    }
}

/*==============================================================================
 * 十六进制解析
 *============================================================================*/

static int8_t hex_val(char c)
{
    if ((c >= '0') && (c <= '9')) { return (int8_t)(c - '0'); }
    if ((c >= 'a') && (c <= 'f')) { return (int8_t)(c - 'a' + 10); }
    if ((c >= 'A') && (c <= 'F')) { return (int8_t)(c - 'A' + 10); }
    return -1;
}

/*==============================================================================
 * 命令实现
 *============================================================================*/

static void cmd_help(void)
{
    /* ⚠️ 刻意用【一个】字符串字面量而不是几十次 sh_puts：
     *    每次 sh_puts 都要一次函数调用 + 一个独立的 "\r\n" 字面量，
     *    30 条命令就是 30 次调用 + 30 份 "\r\n" 存储 —— 纯浪费 Flash。
     *    实测：改成单字面量后省下数百字节（Keil GUI 构建余量从 60 B
     *    回到可用区间）。 */
    sh_puts(
        "RL-CAT STM32 shell (P2 self-test)\r\n"
        "  s               status\r\n"
        "  t               cycle timing (100 Hz budget check)\r\n"
"  n               clear timing stats (n, then motion, then t)\r\n"
        "  l <0|1>         LED off/on\r\n"
        "  m <off|on|slow|fast|estop>  LED mode\r\n"
        "  i               I2C bus + probe 0x50\r\n"
        "  A               servo bus: PING all 12 ids\r\n"
        "  T <0|1>         servo bus: torque off/on for ALL\r\n"
        "  P <j> <step>    move joint j (0..11) to step (via calib table)\r\n"
        "  W <j> <raw>     RAW write: pin joint j to encoder step 0..4095\r\n"
        "                  (no calib table involved; 'T 1' first)\r\n"
        "  F <step> [stride]  move ALL joints to step (+/-stride)\r\n"
        "  B               servo bus stats\r\n"
        "  J <0|1>         IMU bypass for ACTIVE gate (scripted gait debug)\r\n"
        "  Y <id>          PING one id, dump RAW reply + verdict\r\n"
        "  D               USART2/DMA regs + raw hw byte check\r\n"
        "  Z [ta] [ft] [gap]  bus timing + answer-rate (Z 0 clears)\r\n"
        "  X               restart UART2 RX (recover a stuck DMA)\r\n"
        "  U               UART2 loopback self-test (short PA2-PA3)\r\n"
        "  M               IMU detail (+ gravity check)\r\n"
        "  I               IMU raw registers (byte-order check)\r\n"
        "  N <mode>        request mode (idle/stand/active/damp/estop)\r\n"
        "  R               mode + safety + calibration report\r\n"
        "  C <sub>         calibration: start/cancel/zero/erase\r\n"
        "  V               battery detail (+ calibration aid)\r\n"
        "  E [0|1]         ESP32 mirror on/off (0 = clean text on UART3)\r\n"
        "  p [n]           echo n bytes on USART1\r\n"
        "  w <type> <len> <hex...>  send an RLCA frame\r\n"
        "  e               erase calib flash page\r\n"
        "  r               reset counters\r\n"
        "  h / ?           help\r\n"
    );
}

static void cmd_status(void)
{
    sh_puts("-- status --\r\n");
    sh_kv("sysclk_hz", HAL_RCC_GetSysClockFreq());
    sh_kv("pclk1_hz",  HAL_RCC_GetPCLK1Freq());
    sh_kv("pclk2_hz",  HAL_RCC_GetPCLK2Freq());
    sh_kv("dwt_ok",    bsp_tick_selftest() ? 1u : 0u);
    sh_kv("uptime_ms", bsp_tick_ms());

    sh_puts("-- sensors --\r\n");
    {
        const batt_state_t *b = battery_state();
        sh_kv("batt_mv",      (uint32_t)b->mv);
        sh_kv("batt_mv_raw",  (uint32_t)b->mv_raw);
        sh_kv("batt_adc_raw", (uint32_t)b->adc_raw);
        sh_kv("batt_level",   (uint32_t)b->level);
        sh_kv("batt_valid",   (uint32_t)b->valid);
        sh_kv("batt_n",       b->n_samples);
        sh_kv("batt_timeouts",b->n_timeouts);
    }
    {
        const imu_state_t *m = imu_state();
        sh_kv("imu_valid",  (uint32_t)m->valid);
        sh_kv("imu_version", (uint32_t)m->version);
        sh_kv("imu_ok",     m->n_ok);
        sh_kv("imu_err",    m->n_err);
        sh_kv("imu_spike",  m->n_spike);
        sh_kv("imu_age_ms", imu_age_ms());
    }

    sh_puts("-- uart rx --\r\n");
    sh_kv("pi_total",    g_uart_pi.rx_total);
    sh_kv("pi_avail",    bsp_uart_available(&g_uart_pi));
    sh_kv("pi_overflow", g_uart_pi.rx_overflow);
    sh_kv("pi_errors",   g_uart_pi.rx_errors);

    sh_kv("servo_total",    g_uart_servo.rx_total);
    sh_kv("servo_avail",    bsp_uart_available(&g_uart_servo));
    sh_kv("servo_overflow", g_uart_servo.rx_overflow);
    sh_kv("servo_errors",   g_uart_servo.rx_errors);

    sh_kv("esp_total",    g_uart_esp.rx_total);
    sh_kv("esp_overflow", g_uart_esp.rx_overflow);
    sh_kv("esp_errors",   g_uart_esp.rx_errors);

    /* ---- UART3 转发（TEL_FAST → ESP32）----
     *  ⭐ 排查"ESP32 收不到数据"就看这三行：
     *     fwd_try 在涨 + fwd_lost 不涨 = 正常发出
     *     fwd_lost 持续增长         = ESP32 侧没读走（TCP 掉线/它自己卡住）
     *     fwd_try 不涨              = 转发开关关了，或控制环没在跑 */
    {
        esp_fwd_stats_t fs;
        esp_fwd_stats(&fs);
        sh_kv("fwd_try",   fs.n_try);
        sh_kv("fwd_ok",    fs.n_ok);
        sh_kv("fwd_lost",  fs.n_lost);
        sh_kv("fwd_err",   fs.n_err);
        sh_kv("fwd_stall", fs.n_stall);
        sh_kv("fwd_busy",  (uint32_t)fs.busy);
    }

    sh_puts("-- flash calib --\r\n");
    sh_kv("loaded", bsp_flash_calib_loaded() ? 1u : 0u);
    uint8_t hdr[16];
    bsp_flash_dump_header(hdr);
    sh_puts("hdr  : ");
    static const char hexd[] = "0123456789ABCDEF";
    char line[16u * 3u + 4u];
    uint8_t n = 0u;
    for (uint8_t i = 0u; i < 16u; ++i) {
        line[n++] = hexd[(hdr[i] >> 4) & 0x0Fu];
        line[n++] = hexd[hdr[i] & 0x0Fu];
        line[n++] = (char)(((i & 7u) == 7u) ? ' ' : ' ');
    }
    line[n] = '\0';
    sh_puts(line);
    sh_puts("\r\n");
}

static void cmd_i2c(void)
{
    sh_puts("-- i2c self test --\r\n");
    bsp_softi2c_init();
    const bool bus = bsp_softi2c_bus_ok();
    sh_kv("bus_released_high", bus ? 1u : 0u);
    if (!bus) {
        sh_puts("  !! SCL/SDA not pulled high -> check 4.7k pull-ups / wiring\r\n");
    }
    /* JY901P 常见从机地址 */
    static const uint8_t probe[] = { 0x50u, 0x51u, 0x68u, 0x6Au, 0x6Bu };
    for (uint8_t i = 0u; i < (uint8_t)sizeof(probe); ++i) {
        sh_puts("  probe 0x");
        static const char hx[] = "0123456789ABCDEF";
        char a[3];
        a[0] = hx[(probe[i] >> 4) & 0x0Fu];
        a[1] = hx[probe[i] & 0x0Fu];
        a[2] = '\0';
        sh_puts(a);
        sh_puts(bsp_softi2c_probe(probe[i]) ? " -> ACK\r\n" : " -> no ack\r\n");
    }
}

/*==============================================================================
 * 舵机总线命令（P3）
 *============================================================================*/

/** A：逐个 PING 12 个 ID。
 *  ⚠️ 出厂默认 ID 都是 1，若没逐个改 ID，这里会只应答 1 号、其余全无响应
 *     —— 这正是"ID 冲突/未分配"最直接的证据。 */
static void cmd_servo_ping_all(void)
{
    sh_puts("-- servo bus ping (expect ids 1..12) --\r\n");
    uint8_t found = 0u;
    for (uint8_t id = RLCA_SERVO_ID_MIN; id <= RLCA_SERVO_ID_MAX; ++id) {
        const bool ok = servo_bus_ping(id);
        if (ok) { found++; }
        sh_puts("  id ");
        sh_put_u32(id);
        sh_puts(ok ? "  -> ACK\r\n" : "  -> no response\r\n");
    }
    sh_puts("found ");
    sh_put_u32(found);
    sh_puts(" / 12\r\n");
    if (found == 0u) {
        sh_puts("  !! none replied: check jumper (A), TX-RX STRAIGHT wiring,\r\n");
        sh_puts("     control-board power, and common ground.\r\n");
    } else if (found < 12u) {
        sh_puts("  !! missing ids: servos may still have the factory id (1).\r\n");
    }
}

static void cmd_servo_torque(const char *arg)
{
    if (*arg == '0') {
        /* ⚠️ 走 app 接口而不是直接 servo_bus_set_all_torque()：
         *    它同时会关闭"手动力矩覆盖"，让 ⑥b 的周期重申恢复。
         *    否则主循环下一个周期又会把力矩关掉，覆盖生效。 */
        app_set_manual_torque(false);
        sh_puts("torque OFF (all); periodic re-assert RESUMED\r\n");
    } else if (*arg == '1') {
        /* ⭐ 打开"手动力矩覆盖"：暂停 ⑥b 的周期重申。
         *    ⚠️ 不这么做的话，200 ms 后重申会 bus_abort_now() 掐断
         *       刚发出的位置帧 + 关掉力矩 ⇒ W 必须反复连发才走到目标。 */
        app_set_manual_torque(true);
        sh_puts("torque ON (all); periodic re-assert SUSPENDED\r\n");
        sh_puts("  (so 'W' frames survive; 'T 0' restores re-assert)\r\n");
    } else {
        sh_puts("usage: T <0|1>\r\n");
        sh_puts("  1 = torque on + suspend the 5 Hz torque-off re-assert\r\n");
        sh_puts("  0 = torque off + resume re-assert\r\n");
    }
}

/** J <0|1> —— ⭐⭐ IMU 失效的"调试旁路"开关（2026-10-16 加）
 *
 *  ⚠️ 为什么需要：
 *     ACTIVE 准入的第 ④ 道门是 imu_ok。IMU 失效时 ACTIVE 会被静默降级成
 *     STAND，而 STAND 分支【忽略 pi_q_target】、只保持 q_def = 0
 *     ⇒ "用 Pi 发角度调步态"这条通路被彻底堵死（pi/pose.py 会报
 *       "生效模式=STAND，关节没在跟踪"）。
 *
 *  ⚠️ 但 IMU 只对【策略推理】必需（47 维观测里有姿态）。
 *     按脚本直接下发关节角时观测不被使用 ⇒ IMU 失效不影响动作正确性。
 *
 *  ⚠️ 安全边界（完全可审计）：
 *     · 只由人显式敲 `J 1` 打开，默认关闭
 *     · 只在"IMU 是【唯一】阻塞原因"时放行（舵机报错/过温/电压照旧挡住）
 *     · RLCA_SAFE_R_IMU_STALE 位【照旧置位】⇒ K / R 里一眼看出
 *       "ACTIVE 是靠旁路进来的"
 *     · 不影响急停 / 降级 / 趴下 / 失联
 *
 *  ⚠️ 跑【策略】之前必须 `J 0` 关掉。 */
static void cmd_imu_bypass(const char *arg)
{
    if (*arg == '0') {
        app_safety_set_imu_bypass(false);
        sh_puts("IMU bypass OFF (safe default)\r\n");
        sh_puts("  ACTIVE will again be blocked while IMU is stale\r\n");
    } else if (*arg == '1') {
        app_safety_set_imu_bypass(true);
        sh_puts("IMU bypass ON  <-- for SCRIPTED gait debugging only\r\n");
        sh_puts("  * only helps when IMU is the ONLY blocker\r\n");
        sh_puts("  * RLCA_SAFE_R_IMU_STALE(0x40) still reported in K/R\r\n");
        sh_puts("  * TURN IT OFF BEFORE RUNNING THE POLICY ('J 0')\r\n");
    } else {
        sh_puts("usage: J <0|1>\r\n");
        sh_puts("  1 = allow ACTIVE despite stale IMU (scripted gait debug)\r\n");
        sh_puts("  0 = restore the original rule (default)\r\n");
        sh_puts("current: ");
        sh_put_u32(app_safety_get_imu_bypass() ? 1u : 0u);
        sh_puts("\r\n");
    }
}

/** 解析一个十进制整数（可带负号） */static bool parse_int(const char **ps, int32_t *out)
{
    const char *s = *ps;
    while ((*s == ' ') || (*s == '\t')) { s++; }
    bool neg = false;
    if (*s == '-') { neg = true; s++; }
    if ((*s < '0') || (*s > '9')) { return false; }
    int32_t v = 0;
    while ((*s >= '0') && (*s <= '9')) { v = v * 10 + (*s - '0'); s++; }
    *out = neg ? -v : v;
    *ps = s;
    return true;
}

/** P <joint 0..11> <step> —— 单关节直接控位 */
static void cmd_servo_pos_one(char *args)
{
    int32_t j = 0;
    int32_t step = 0;
    const char *p = args;
    if (!parse_int(&p, &j) || !parse_int(&p, &step)) {
        sh_puts("usage: P <joint 0..11> <step>\r\n");
        return;
    }
    if ((j < 0) || (j >= (int32_t)RLCA_N_JOINTS)) {
        sh_puts("joint out of range\r\n");
        return;
    }
    if ((step < -32768) || (step > 32767)) {
        sh_puts("step out of range\r\n");
        return;
    }
    const bool ok = servo_bus_write_one((uint8_t)j, (int16_t)step);
    sh_puts(ok ? "sent\r\n" : "FAILED\r\n");
}

/** W <joint 0..11> <raw_step> —— ⭐ 按【原始步数】写单个舵机的目标位置
 *
 *  ⚠️ 与 P 的关键区别：
 *     P 的参数是"【当前标定表下】的步数目标"，会经 rlca_q_to_step 换算；
 *     而标定表随时可能变（尤其未标定时 off=0），所以你无法用它判断
 *     "舵机的物理读数 与 固件认为的步数" 是否一致。
 *
 *     W 只把步数夹到编码器物理范围 [0,4095]，【不碰标定表、不做角度换算】。
 *     语义严格是："让这个舵机停在物理步数 raw_step"。
 *
 *  用途（项目负责人要做的验证）：
 *     T 1              ← 先使能力矩（否则写了也不动）
 *     W 4 2560         ← 让 j4 去物理步数 2560
 *     K                ← 看 raw pos_step 是否停在 2560
 *     逐个试，就能判断 offset 对不对。
 *
 *  ⚠️ 不改任何状态机、不影响 s_armed（这是调试通道，不是控制通道）。
 *  ⚠️ 会 bus_abort_now()（清掉在飞行中的事务）。 */
static void cmd_servo_write_raw(char *args)
{
    int32_t j = 0;
    int32_t step = 0;
    const char *p = args;
    if (!parse_int(&p, &j) || !parse_int(&p, &step)) {
        sh_puts("usage: W <joint 0..11> <raw_step 0..4095>\r\n");
        sh_puts("  writes the RAW encoder target (no calib table involved)\r\n");
        sh_puts("  enable torque first:  T 1\r\n");
        return;
    }
    if ((j < 0) || (j >= (int32_t)RLCA_N_JOINTS)) {
        sh_puts("joint out of range (0..11)\r\n");
        return;
    }
    if ((step < 0) || (step > 4095)) {
        sh_puts("step out of range (0..4095)\r\n");
        return;
    }

    const bool ok = servo_bus_write_one_raw((uint8_t)j, (int16_t)step);

    if (!ok) {
        sh_puts("FAILED (bus busy?)\r\n");
        return;
    }
    sh_puts("sent: j");
    sh_put_u32((uint32_t)j);
    sh_puts(" id=");
    sh_put_u32(RLCA_SERVO_ID((uint8_t)j));
    sh_puts(" raw_step=");
    sh_put_u32((uint32_t)step);
    sh_puts("\r\n  (torque must be ON: 'T 1'.  Read back with 'K')\r\n");
}

/** F <step> [stride] —— 全部关节同时到 step（+i*stride） */
static void cmd_servo_pos_all(char *args)
{
    int32_t step = 0;
    int32_t stride = 0;
    const char *p = args;
    if (!parse_int(&p, &step)) {
        sh_puts("usage: F <step> [stride]\r\n");
        return;
    }
    (void)parse_int(&p, &stride);

    /* 走 SyncWrite：12 个舵机必须【同时】开始动 */
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        const int32_t s = step + (int32_t)i * stride;
        (void)servo_bus_write_one(i, (int16_t)s);
    }
    sh_puts("sent 12 single writes (use A/T first to enable torque)\r\n");
}

static void cmd_servo_stats(void)
{
    const servo_bus_stats_t *s = servo_bus_stats();
    sh_puts("-- servo bus stats --\r\n");
    sh_kv("state",        (uint32_t)servo_bus_state());
    sh_kv("tx_frames",    s->tx_frames);
    sh_kv("rx_frames",    s->rx_frames);
    sh_kv("ok_reads",     s->n_ok_reads);
    sh_kv("failed_reads", s->n_failed_reads);
    sh_kv("timeouts",     s->n_timeout);
    sh_kv("idle_timeouts",s->n_idle_timeout);
    sh_kv("crc_err",      s->n_crc_err);
    sh_kv("short",        s->n_short);
    sh_kv("err_accum",    s->err_accum);
    sh_kv("err_flags",    (uint32_t)servo_bus_err_flags());
    sh_kv("last_txn_us",  s->last_txn_us);
    sh_kv("max_txn_us",   s->max_txn_us);
    sh_kv("uart_avail",   bsp_uart_available(&g_uart_servo));
    sh_kv("uart_overflow",g_uart_servo.rx_overflow);
    sh_kv("uart_errors",  g_uart_servo.rx_errors);
    /* ⭐ DMA 重启次数：>0 说明发生过"接收错误导致 DMA 重启"。
     *  ⚠️ 如果它一直是 0、而 rx_frames 也是 0，那问题不在错误恢复，
     *     而在更前面（DMA 没启动 / 接线 / 控制板方向 / PA2-PA3 还短接着）。 */
    sh_kv("dma_restarts", g_uart_servo.dma_restarts);

    /* ⭐⭐ 解析诊断：把"被丢弃处的原始字节"打出来。
     *
     *  ⚠️ 为什么需要（实测卡住很久）：修好 DMA 搬运后 `A`(PING) 报
     *     id 5/9 都 ACK，而 `B`(SyncRead) 里只有 id 5 有数据。
     *     说明有个应答到了却【没被解析出来】—— 而原代码只累加
     *     `short`/`crc_err` 计数，看不到被拒的到底是什么字节。
     *
     *  判读：
     *     drop_raw 里有另一个舵机的完整帧头（FF FF 09 ...）
     *        -> 字节是对的，但解析边界/ID 匹配有问题
     *     drop_raw 里是残缺/错位字节
     *        -> 总线时序导致帧被截断（接收问题，不是解析问题）
     *     drop_count = 0
     *        -> 没有字节被丢弃；那"缺一个舵机"的原因在别处 */
    {
        uint8_t  tr[24];
        uint16_t trn = 0u, troff = 0u, trtot = 0u;
        servo_bus_get_trace(tr, (uint16_t)sizeof(tr), &trn, &troff, &trtot);
        static const char hx4[] = "0123456789ABCDEF";

        sh_kv("parse_fail", servo_bus_parse_fail_count());
        sh_kv("unknown_id", (uint32_t)servo_bus_unknown_id());
        sh_kv("drop_off",   (uint32_t)troff);
        sh_kv("drop_total", (uint32_t)trtot);
        sh_kv("drop_len",   (uint32_t)trn);
        sh_kv("drop_reason",(uint32_t)servo_bus_trace_reason());
        if (trn > 0u) {
            sh_puts("  drop_raw:");
            for (uint16_t i = 0u; i < trn; ++i) {
                char b3[4];
                b3[0] = ' ';
                b3[1] = hx4[(tr[i] >> 4) & 0x0Fu];
                b3[2] = hx4[tr[i] & 0x0Fu];
                b3[3] = '\0';
                sh_puts(b3);
            }
            sh_puts("\r\n");
        }
    }

    /* 12 个关节的反馈一览 */
    const servo_fb_t *f = servo_bus_feedback();
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        sh_puts("  j");
        sh_put_u32(i);
        sh_puts(" id=");
        sh_put_u32(RLCA_SERVO_ID(i));
        sh_puts(" pos=");
        sh_put_u32((uint32_t)(int32_t)f[i].pos_step);
        sh_puts(" spd=");
        sh_put_u32((uint32_t)(int32_t)f[i].speed_step);
        sh_puts(" load=");
        sh_put_u32((uint32_t)(int32_t)f[i].load_raw);
        sh_puts(" V=");
        sh_put_u32(f[i].voltage_dv);
        sh_puts(" T=");
        sh_put_u32(f[i].temp_c);
        sh_puts(" err=");
        sh_put_u32(f[i].err);
        sh_puts(f[i].valid ? " V\r\n" : " -\r\n");
    }
}

static void cmd_echo(const char *arg)
{
    uint32_t n = 16u;
    if ((arg != NULL) && (*arg != '\0')) {
        n = 0u;
        while ((*arg >= '0') && (*arg <= '9')) {
            n = (n * 10u) + (uint32_t)(*arg - '0');
            arg++;
        }
    }
    if (n > 256u) { n = 256u; }

    sh_puts("echo ");
    sh_put_u32(n);
    sh_puts(" bytes:\r\n");
    for (uint32_t i = 0u; i < n; ++i) {
        const uint8_t b = (uint8_t)(i & 0xFFu);
        (void)bsp_uart_write((s_cur != NULL) ? s_cur : &g_uart_pi, &b, 1u);
    }
    sh_puts("\r\necho done\r\n");
}

/** w <type> <len> <hex bytes...>  —— 组一个 RLCA 协议帧发出去 */
static void cmd_frame(char *args)
{
    /* 解析 type */
    while ((*args == ' ') || (*args == '\t')) { args++; }
    uint32_t type = 0u;
    while ((*args >= '0') && (*args <= '9')) { type = type * 10u + (uint32_t)(*args - '0'); args++; }
    while ((*args == ' ') || (*args == '\t')) { args++; }

    /* 解析 len */
    uint32_t len = 0u;
    while ((*args >= '0') && (*args <= '9')) { len = len * 10u + (uint32_t)(*args - '0'); args++; }
    while ((*args == ' ') || (*args == '\t')) { args++; }

    if (len > RLCA_FRAME_MAX_PAYLOAD) {
        sh_puts("len too big\r\n");
        return;
    }

    uint8_t payload[RLCA_FRAME_MAX_PAYLOAD];
    memset(payload, 0, sizeof(payload));

    for (uint32_t i = 0u; i < len; ++i) {
        while ((*args == ' ') || (*args == '\t')) { args++; }
        const int8_t hi = hex_val(args[0]);
        const int8_t lo = (hi >= 0) ? hex_val(args[1]) : -1;
        if ((hi < 0) || (lo < 0)) {
            sh_puts("bad hex\r\n");
            return;
        }
        payload[i] = (uint8_t)(((uint8_t)hi << 4) | (uint8_t)lo);
        args += 2;
    }

    uint8_t frame[RLCA_FRAME_OVERHEAD + RLCA_FRAME_MAX_PAYLOAD];
    const uint16_t n = rlca_frame_pack(frame, (uint16_t)sizeof(frame),
                                      (uint8_t)type, 0u, payload, (uint8_t)len);
    if (n == 0u) {
        sh_puts("pack failed\r\n");
        return;
    }

    sh_puts("frame ");
    sh_put_u32(n);
    sh_puts(" bytes, crc_behind. hex:\r\n");
    static const char hx[] = "0123456789ABCDEF";
    char line[8];
    for (uint16_t i = 0u; i < n; ++i) {
        line[0] = hx[(frame[i] >> 4) & 0x0Fu];
        line[1] = hx[frame[i] & 0x0Fu];
        line[2] = ' ';
        line[3] = '\0';
        sh_puts(line);
        if (((i + 1u) % 16u) == 0u) { sh_puts("\r\n"); }
    }
    sh_puts("\r\n");
    (void)bsp_uart_write(&g_uart_pi, frame, n);
    sh_puts("sent\r\n");
}

/*==============================================================================
 * 树莓派链路命令（P4）
 *============================================================================*/

#if RLCA_SHELL_LINK_CMDS_EN

static void cmd_link_stats(void)
{
    const link_pi_stats_t *s = link_pi_stats();
    const uint32_t now = bsp_tick_ms();

    sh_puts("-- link_pi stats (USART1) --\r\n");
    sh_kv("mode",          (uint32_t)link_pi_mode());
    sh_kv("alive",         link_pi_is_alive(now) ? 1u : 0u);
    sh_kv("lost_ms",       link_pi_lost_ms(now));
    sh_kv("rx_frames",     s->rx_frames);
    sh_kv("cmd_joint",     s->cmd_joint_count);
    sh_kv("crc_errors",    s->crc_errors);
    sh_kv("len_errors",    s->len_errors);
    sh_kv("rx_timeouts",   s->rx_timeouts);
    sh_kv("unknown_type",  s->unknown_type);
    sh_kv("seq_lost",      s->seq_lost);
    sh_kv("wait_timeouts", s->wait_timeouts);
    sh_kv("uart_avail",    bsp_uart_available(&g_uart_pi));
    sh_kv("uart_overflow", g_uart_pi.rx_overflow);
    sh_kv("uart_errors",   g_uart_pi.rx_errors);
}

/** 主动发一帧，用来验证"Pi 侧能不能收到合法帧"。
 *  g  发 TEL_FAST（LEN=96）
 *  u  发 TEL_STATUS（LEN=40）
 *  k  发 HEARTBEAT（LEN=0） */
static void cmd_link_send(char which)
{
    const uint32_t now = bsp_tick_ms();

    if (which == 'g') {
        tel_fast_src_t ts;
        memset(&ts, 0, sizeof(ts));
        ts.quat_wxyz[0] = 1.0f;
        rlca_phase_get_sincos_i16(&ts.gait_sin_i16, &ts.gait_cos_i16);
        ts.cycle_cnt  = (uint16_t)(now & 0xFFFFu);
        ts.voltage_mv = 0u;
        ts.status     = RLCA_ST_READY;
        sh_puts(link_pi_send_tel_fast(&ts) ? "TEL_FAST sent (103 B)\r\n"
                                           : "TEL_FAST FAILED\r\n");
    } else if (which == 'u') {
        static uint8_t temps[RLCA_N_JOINTS];
        const servo_fb_t *f = servo_bus_feedback();
        for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) { temps[i] = f[i].temp_c; }

        tel_status_src_t ss;
        memset(&ss, 0, sizeof(ss));
        ss.uptime_ms   = now;
        ss.temp_deci_c = temps;
        ss.mode        = link_pi_mode();
        ss.status      = RLCA_ST_READY;
        sh_puts(link_pi_send_tel_status(&ss) ? "TEL_STATUS sent (47 B)\r\n"
                                             : "TEL_STATUS FAILED\r\n");
    } else if (which == 'k') {
        sh_puts(link_pi_send_heartbeat() ? "HEARTBEAT sent (7 B)\r\n"
                                         : "HEARTBEAT FAILED\r\n");
    } else {
        sh_puts("usage: g <frames>  (all callable from shell)\r\n");
    }
}

/** 连续发 n 次 TEL_FAST，用于 Pi 侧误码率测试（协议 §9 单元测试 9/10）。 */
static void cmd_link_stream(char *args)
{
    uint32_t n = 100u;
    if ((args != NULL) && (*args >= '0') && (*args <= '9')) {
        n = 0u;
        while ((*args >= '0') && (*args <= '9')) {
            n = (n * 10u) + (uint32_t)(*args - '0');
            args++;
        }
    }
    if (n > 100000u) { n = 100000u; }

    sh_puts("streaming ");
    sh_put_u32(n);
    sh_puts(" TEL_FAST frames...\r\n");

    tel_fast_src_t ts;
    memset(&ts, 0, sizeof(ts));
    ts.quat_wxyz[0] = 1.0f;

    uint32_t ok = 0u;
    for (uint32_t i = 0u; i < n; ++i) {
        ts.cycle_cnt = (uint16_t)(i & 0xFFFFu);
        if (link_pi_send_tel_fast(&ts)) { ok++; }
    }
    sh_puts("sent ");
    sh_put_u32(ok);
    sh_puts(" / ");
    sh_put_u32(n);
    sh_puts("\r\n");
}

#endif /* RLCA_SHELL_LINK_CMDS_EN */

/*==============================================================================
 * 传感器命令（P5）
 *============================================================================*/

/** M：IMU 详细状态（含原始换算值，便于与上位机对照标定）。 */
static void cmd_imu(void)
{
    const imu_state_t *m = imu_state();

    sh_puts("-- imu (JY901P) --\r\n");
    sh_kv("valid",  (uint32_t)m->valid);
    sh_kv("version",   (uint32_t)m->version);
    sh_kv("n_ok",   m->n_ok);
    sh_kv("n_err",  m->n_err);
    sh_kv("n_spike",m->n_spike);
    sh_kv("age_ms", imu_age_ms());

    /* ⭐⭐ 最近一次失败的原因（2026-10-16 加）。
     *  ⚠️ 没有这一项时，`valid=0` 只能看到 n_err/n_spike 涨，
     *     无法区分"三次 I2C 读哪一次挂了"和"离群值保护触发" —— 排查方向完全不同。 */
    {
        static const char *const fail_names[] = {
            "NONE (last poll OK)",
            "NO_PORT (imu_init not called?)",
            "READ_ACC  (0x34) failed",
            "READ_GYRO (0x37) failed",
            "READ_ANGLE(0x3D) failed",
            "SPIKE (gyro jump > 20 rad/s)"
        };
        const uint8_t lf = m->last_fail;
        sh_puts("last_fail    = ");
        sh_put_u32((uint32_t)lf);
        sh_puts("  <- ");
        sh_puts((lf < (uint8_t)(sizeof(fail_names) / sizeof(fail_names[0])))
                ? fail_names[lf] : "?");
        sh_puts("\r\n");

        if (m->valid == 0u) {
            sh_puts("  >> valid=0 会置 RLCA_SAFE_R_IMU_STALE(0x40) 并【禁止 ACTIVE】\r\n");
            sh_puts("  >> NONE 却 valid=0：说明是 mark_stale 因 100 ms 无成功而清掉的\r\n");
        }
    }

    sh_puts("quat : ");
    for (uint8_t i = 0u; i < 4u; ++i) { sh_puts(" "); sh_put_float(m->quat_wxyz[i]); }
    sh_puts("\r\n");

    sh_puts("gyro : ");
    for (uint8_t i = 0u; i < 3u; ++i) { sh_puts(" "); sh_put_float(m->gyro_rad_s[i]); }
    sh_puts("  (rad/s)\r\n");

    sh_puts("accel: ");
    for (uint8_t i = 0u; i < 3u; ++i) { sh_puts(" "); sh_put_float(m->accel_g[i]); }
    sh_puts("  (g)\r\n");

    sh_puts("euler: roll ");
    sh_put_float(m->roll_rad);
    sh_puts("  pitch ");
    sh_put_float(m->pitch_rad);
    sh_puts("  yaw ");
    sh_put_float(m->yaw_rad);
    sh_puts("  (rad)\r\n");

    /* ⭐ 最有用的自检：机身静止水平时，机身系重力应为 (0,0,-1)。
     * 这一行能一次性验证"寄存器映射 + 字节序 + 欧拉角顺序 + 坐标约定"全对。 */
    {
        const float w = m->quat_wxyz[0];
        const float x = m->quat_wxyz[1];
        const float y = m->quat_wxyz[2];
        const float z = m->quat_wxyz[3];
        const float gx = -2.0f * ((x * z) - (w * y));
        const float gy = -2.0f * ((y * z) + (w * x));
        const float gz = -(1.0f - (2.0f * ((x * x) + (y * y))));
        sh_puts("gravity(body):");
        sh_puts(" "); sh_put_float(gx);
        sh_puts(" "); sh_put_float(gy);
        sh_puts(" "); sh_put_float(gz);
        sh_puts("   <- level & still should be 0 0 -1\r\n");
    }

    /* ⚠️ 若这里 12 项都不对，先怀疑寄存器映射（rlcat_config.h §3b 未核实）。 */
}

/** V：电池详情。 */
static void cmd_batt(void)
{
    const batt_state_t *b = battery_state();
    sh_puts("-- battery --\r\n");
    sh_kv("adc_raw",   (uint32_t)b->adc_raw);
    sh_kv("mv_raw",    (uint32_t)b->mv_raw);
    sh_kv("mv_filtered",(uint32_t)b->mv);
    sh_kv("level",     (uint32_t)b->level);
    sh_kv("valid",     (uint32_t)b->valid);
    sh_kv("n_samples", b->n_samples);
    sh_kv("n_timeouts",b->n_timeouts);
    sh_kv("status_bits", (uint32_t)batt_status_bits(b->mv));

    sh_puts("thresholds: L1 ");
    sh_put_u32(RLCA_BATT_WARN_MV);
    sh_puts(" mV, L2 ");
    sh_put_u32(RLCA_BATT_STOP_MV);
    sh_puts(" mV, L3 ");
    sh_put_u32(RLCA_BATT_FORCE_MV);
    sh_puts(" mV\r\n");

    /* ⚠️ 标定用：把这里读到的 adc_raw 与万用表量到的电池电压一起记下来，
     *    就能反推 RLCA_ADC_VREF_MV 与分压比（见验收单 P5-4）。 */
    sh_puts("for calibration: compare mv_filtered with a multimeter\r\n");
}

/*==============================================================================
 * 模式与标定命令（P6）
 *============================================================================*/

static void cmd_mode(char *args)
{
    uint8_t m;
    if      (strcmp(args, "idle")   == 0) { m = RLCA_MODE_IDLE; }
    else if (strcmp(args, "stand")  == 0) { m = RLCA_MODE_STAND; }
    else if (strcmp(args, "active") == 0) { m = RLCA_MODE_ACTIVE; }
    else if (strcmp(args, "damp")   == 0) { m = RLCA_MODE_DAMP; }
    else if (strcmp(args, "estop")  == 0) { m = RLCA_MODE_ESTOP; }
    else {
        sh_puts("usage: N <idle|stand|active|damp|estop>\r\n");
        return;
    }
    app_mode_request(m);
    link_pi_set_mode(m);
    sh_puts("mode requested: ");
    sh_puts(args);
    sh_puts("\r\n");
    sh_puts("actual mode + why it differs -> use 'R'\r\n");
}

/** R：报告模式 + 安全 + 标定状态。
 *  ⭐ 这是排查"为什么进不了 ACTIVE"的第一站。 */
static void cmd_run_state(void)
{
    const app_mode_state_t *ms = app_mode_state();
    const app_safety_state_t *ss = app_safety_state();

    sh_puts("-- mode / safety --\r\n");
    sh_kv("mode_requested", (uint32_t)app_mode_requested());
    sh_kv("mode_active",    (uint32_t)app_mode_current());
    sh_kv("torque_on",      (uint32_t)ms->torque_on);
    /* ⭐ 实际下发的力矩（2026-10-17 加）。
     *  ⚠️ 与上一行不同 ⇒ 被 s_armed 锁挡住了 ——
     *     这解释了"torque_on=1 但机器人一动不动"。
     *     要接管力矩，Pi 发 CMD_JOINT 时必须带 FLAG_ENABLE(0x01)。 */
    sh_kv("torque_actual",  app_torque_actual() ? 1u : 0u);
    sh_kv("armed",          app_is_armed() ? 1u : 0u);
    sh_kv("mode_changes",   ms->n_mode_changes);
    /* ⚠️ 下面这一行是"ACTIVE 为什么没生效"的直接答案：
     *    0=通过 1=未标定 2=链路不活 3=IMU 不可用 4=安全层禁止 5=反馈不新鲜 */
    sh_kv("active_block",   (uint32_t)ms->active_block);

    sh_kv("safety_reasons", (uint32_t)app_safety_reasons());
    sh_kv("safety_severity",(uint32_t)app_safety_severity());

    /* ⭐⭐ 链路实时状态（2026-10-17 加）——"ESTOP 到底为什么"的一句话答案。
     *
     *  ⚠️ 只看 safety_reasons=8(LINK_LOST) 还不够，必须知道**现在**链路空档多大：
     *      · lost_ms ≤  30  → 链路健康（LINK_OK 会置位）
     *      · lost_ms ≤ 100  → 降级（severity 2，ACTIVE 进不去，但还不是急停）
     *      · lost_ms > 100  → 急停（severity 3，安全层把关节推向趴下姿态）
     *
     *  ⚠️ 注意：**没有上层工具在发帧时，lost_ms 必然一直增长**。
     *     Pi 侧只有 pose.py / jog.py / acceptance.py 会 100 Hz 连续发帧；
     *     status_panel.py 靠 --keepalive 保活；calib.py 只在交互时发。
     *     ⇒ 静止观察时进 ESTOP 是【设计行为】，不是 bug。 */
    {
        const uint32_t now_ms = bsp_tick_ms();
        sh_kv("link_alive",   link_pi_is_alive(now_ms) ? 1u : 0u);
        sh_kv("link_lost_ms", link_pi_lost_ms(now_ms));   /* >30 降级, >100 急停 */
        sh_kv("link_rx_frames", link_pi_stats()->rx_frames);
        /* ⭐⭐ 链路错误分解（2026-10-18 加）
         *   ⚠️ 只有 rx_frames 看不出"为什么没收到"。这三项能分开：
         *      crc_errors   > 0  ⇒ 字节到了但被破坏 ⇒ 线路/波特率
         *      len_errors   > 0  ⇒ 帧头对上了但长度不对 ⇒ 半帧/错位
         *      rx_timeouts  > 0  ⇒ 一帧中途断掉 ⇒ 字节丢失
         *      三者都 0 而 rx_frames 偏少 ⇒ 帧根本没到（Pi 没发/写失败）
         *      seq_lost            ⇒ 从 SEQ 推出的累计丢帧数 */
        sh_kv("link_crc_errors", link_pi_stats()->crc_errors);
        sh_kv("link_len_errors", link_pi_stats()->len_errors);
        sh_kv("link_rx_timeouts", link_pi_stats()->rx_timeouts);
        sh_kv("link_seq_lost", link_pi_stats()->seq_lost);
        sh_kv("link_cmd_joint", link_pi_stats()->cmd_joint_count);
        sh_kv("link_unknown_type", link_pi_stats()->unknown_type);

        /* ⭐⭐⭐ 最近一次急停现场（2026-10-18 加）
         *
         *  ⚠️ 用来看"到底哪一条触发了 ESTOP"。ESTOP 只有 4 个来源：
         *       operator(未实现) / 链路(link_level>=2) / 电压(<=9300) / 过温(>70)
         *     ⇒ 对照下面这组值就能一眼定罪，不必再靠 Pi 侧数据反推。
         *  ⚠️ reason 位：1=VOLT_FORCE 2=VOLT_STOP 4=VOLT_WARN 8=LINK_LOST
         *      16=OVERTEMP 32=SERVO_ERR 64=IMU_STALE 128=OPERATOR */
        {
            const app_estop_snapshot_t *es = app_safety_last_estop();
            sh_kv("last_estop_count",  es->count);
            sh_kv("last_estop_reason", es->reason);
            sh_kv("last_estop_link_level", es->link_level);
            sh_kv("last_estop_batt_mv",    es->batt_mv);
            sh_kv("last_estop_batt_valid", es->batt_valid);
            sh_kv("last_estop_temp_c",     es->temp_c);
            sh_kv("last_estop_temp_raw",   es->temp_raw);  /* >125 = 坏读数 */
            sh_kv("last_estop_servo_err",  es->servo_err);
            sh_kv("last_estop_imu_ok",     es->imu_ok);

            /* ⭐ 分原因计数：哪一个判据在真正触发（见 app_safety.c 的说明） */
            sh_kv("estop_cnt_link",    app_safety_estop_count_cause(0u));
            sh_kv("estop_cnt_volt",    app_safety_estop_count_cause(1u));
            sh_kv("estop_cnt_temp",    app_safety_estop_count_cause(2u));
            sh_kv("estop_cnt_oper",    app_safety_estop_count_cause(3u));
            sh_kv("estop_cnt_unknown", app_safety_estop_count_cause(4u));
            sh_kv("estop_lvl0", app_safety_estop_count_level(0u));
            sh_kv("estop_lvl1", app_safety_estop_count_level(1u));
            sh_kv("estop_lvl2", app_safety_estop_count_level(2u));
        }

        /* ⭐⭐⭐ UART 层计数（2026-10-18 加）—— 区分"字节丢了"和"帧没到"
         *
         *  ⚠️ 为什么必须分这一层：
         *     上面那组 crc/len/timeout 全为 0、而 rx_frames 仍偏少时，
         *     光靠链路层看不出字节去哪了。USART1 走【单字节中断】接收
         *     （不是 DMA），500 kbps 下一个字节只有 20 us ——
         *     ISR 延迟超过 20 us 就会 ORE，**丢 1 个字节 = 废 1 帧**。
         *
         *      rx_errors   > 0  ⇒ 发生过 ORE/FE/NE/PE ⇒ ISR 被延迟
         *                          （最常见：IMU 软件 I2C 阻塞 20 ms）
         *      rx_overflow > 0  ⇒ 环形缓冲满而丢字节 ⇒ 主循环太久没 poll
         *      两者都 0 且 rx_total 也几乎不涨 ⇒ 字节根本没到 ⇒ 查线路 */
        sh_kv("link_uart_rx_bytes",  g_uart_pi.rx_total);
        sh_kv("link_uart_rx_errors", g_uart_pi.rx_errors);
        sh_kv("link_uart_rx_ovf",    g_uart_pi.rx_overflow);
        sh_kv("link_uart_restarts",  g_uart_pi.dma_restarts);
        /* ⭐ 总线连续失败轮数（诊断）：0~1 = 偶发抖动（正常）；
         *   ≥ RLCA_BUS_FAIL_STREAK_ERR = 总线真的不通。
         *   ⚠️ 累计值看 K 里的 bus_timeout。 */
        sh_kv("bus_fail_streak", (uint32_t)servo_bus_fail_streak());
        /* ⭐⭐ 链路空档的历史极值（诊断"ESTOP 到底是不是链路造成的"）：
         *   max_gap_ms 远小于 RLCA_LINK_ESTOP_MS ⇒ 不是链路
         *   over_estop_cnt 就是"空档越过阈值"的次数，和 mode_changes 对照看 */
        sh_kv("link_max_gap_ms",  link_pi_max_gap_ms());
        sh_kv("link_over_estop",  link_pi_over_estop_count());
        sh_kv("link_estop_thr_ms", (uint32_t)RLCA_LINK_ESTOP_MS);
        /* ⭐ 标定表结构性检查（诊断"app_safety 是否走了未标定早退"） */
        sh_kv("calib_valid", rlca_calib_is_valid(app_calib_table()) ? 1u : 0u);
    }

    sh_kv("blocks_active",  app_safety_blocks_active() ? 1u : 0u);
    sh_kv("amp_scale_x100", (uint32_t)(app_safety_amplitude_scale() * 100.0f));
    sh_kv("sitdown_active", ss->active ? 1u : 0u);
    sh_kv("sitdown_done",   ss->done ? 1u : 0u);
    sh_kv("sitdown_torque_off", ss->torque_off ? 1u : 0u);
    sh_kv("servo_max_temp", (uint32_t)servo_bus_max_temp());

    sh_puts("-- calibration --\r\n");
    sh_kv("calib_state",    (uint32_t)calib_proc_state());
    sh_kv("calib_err",      (uint32_t)app_calib_err());
    sh_kv("calib_progress", (uint32_t)app_calib_progress());
    sh_kv("calib_busy",     app_calib_busy() ? 1u : 0u);
    sh_kv("flash_loaded",   bsp_flash_calib_loaded() ? 1u : 0u);

    /* ⭐ `flash_loaded = 0` 的含义（2026-10-18 加）
     *    本次上电时【没有】从 flash 成功读出标定表。
     *    此时内存里的表可能来自：① 默认表 ② 本次运行期标定 / 手动零点。
     *    ⇒ 后者只在 RAM 里，**重启就丢**。
     *    判据：flash_loaded=0 但下面的 offset 非 0 ⇒ 运行期标定的，要重新落盘。
     *    （全自动 `C start` 会自己保存；手动零点不保存。） */

    const rlca_calib_t *c = app_calib_table();
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        sh_puts("  j");
        sh_put_u32(i);
        sh_puts(" id=");
        sh_put_u32(RLCA_SERVO_ID(i));
        sh_puts(" off=");
        sh_put_u32((uint32_t)(int32_t)c->offset[i]);
        sh_puts(" dir=");
        sh_puts(c->dir[i] > 0 ? "+1" : "-1");
        sh_puts("\r\n");
    }
}

/** K —— ⭐⭐ 上电安全诊断
 *
 *  现场问题（2026-10-16）：上电后 1 秒内 12 个舵机开始持续同方向运动，
 *  只能切电源。已确认：拔掉 UART2 信号线就【不动】⇒ 指令来自 STM32。
 *
 *  本命令回答三个问题：
 *    ① 泄力到底有没有发出去、发了多少组（reassert_count）
 *    ② 闸门期的"目标位置镜像"有没有真的发生（goal_seeded / writes）
 *    ③ 闸门期能不能读到有效反馈（n_valid）
 *
 *  ⚠️ 判读：
 *    · goal_seeded=0  → 闸门期一个有效反馈都没读到
 *                        ⇒ 镜像不可能发生，问题在【总线读】，不在播种
 *    · reassert_count 很小或为 0 → 泄力根本没发出去
 *    · torque_on=1     → 期望力矩是开的（那就不该动，除非上层逻辑错）
 */
static void cmd_boot_safety(void)
{
    bool     seeded = false;
    uint32_t writes = 0u;
    uint32_t tq_count = 0u;
    uint32_t tq_burst_left = 0u;

    app_boot_goal_seed_stat(&seeded, &writes);
    app_torque_reassert_stat(&tq_count, &tq_burst_left);

    const app_mode_state_t *ms = app_mode_state();

    sh_puts("-- boot safety (v2026-10-16) --\r\n");

    sh_puts("-- torque re-assert (⑥b) --\r\n");
    sh_kv("reassert_count",   tq_count);       /* 累计重申的帧组数（每组 12 帧） */
    sh_kv("burst_left",       tq_burst_left);  /* 密集重申剩余周期数 */
    sh_kv("want_torque",      (uint32_t)ms->torque_on);
    sh_kv("manual_override",  app_get_manual_torque() ? 1u : 0u);  /* T 1 打开 */
    sh_kv("mode_active",      (uint32_t)app_mode_current());

    sh_puts("-- position-write gate (⑦)  ⭐ 核心规则 --\r\n");
    bool     boot_calib = false;
    bool     pw_allowed = false;
    bool     q_seeded   = false;
    uint32_t pw_blocked = 0u;
    app_pos_write_gate_stat(&boot_calib, &pw_allowed, &pw_blocked, &q_seeded);

    sh_kv("boot_calib_present", boot_calib ? 1u : 0u);  /* 上电时有标定吗 */
    sh_kv("pos_write_allowed",  pw_allowed ? 1u : 0u);  /* ⭐ 1 = 才会写位置 */
    sh_kv("pos_write_blocked",  pw_blocked);            /* 被挡住的周期数 */
    sh_kv("q_seeded",           q_seeded ? 1u : 0u);

    sh_puts("  >> 未标定时上面必须是 allowed=0；若为 0 而舵机仍动，\r\n");
    sh_puts("  >> 说明位置写出自别处，或卸力未生效（去抓 UART2 波形）。\r\n");

    sh_puts("-- gate-period goal mirror (已删除的历史兜底) --\r\n");
    sh_kv("goal_seeded",      seeded ? 1u : 0u);
    sh_kv("goal_seed_writes", writes);
    sh_kv("feedback_fresh",   servo_bus_feedback_fresh() ? 1u : 0u);

    /* 闸门期实际读到的位置：这是"镜像"用的原始步数 */
    const servo_fb_t *fb = servo_bus_feedback();
    uint8_t n_valid = 0u;
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        if (fb[i].valid != 0u) { n_valid++; }
    }
    sh_kv("fb_valid_count",  (uint32_t)n_valid);   /* ⚠️ <12 就播不了种 */

    sh_puts("  raw pos_step (what gets mirrored):\r\n");
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        sh_puts("  j");
        sh_put_u32(i);
        sh_puts(" id=");
        sh_put_u32(RLCA_SERVO_ID(i));
        sh_puts(" pos=");
        sh_put_u32((uint32_t)(int32_t)fb[i].pos_step);
        sh_puts(" moving=");
        sh_put_u32((uint32_t)fb[i].moving);
        sh_puts(" load=");
        sh_put_u32((uint32_t)(int32_t)fb[i].load_raw);   /* ⭐ 有力矩时非 0 */
        sh_puts("\r\n");
    }

    sh_puts("-- bus stats --\r\n");
    const servo_bus_stats_t *st = servo_bus_stats();
    uint32_t rounds = 0u, frames = 0u, bytes = 0u;
    servo_bus_get_diag(&rounds, &frames, &bytes);
    sh_kv("bus_rounds",    rounds);
    sh_kv("bus_rx_frames", st->rx_frames);
    sh_kv("bus_timeout",   st->n_timeout);
    sh_kv("bus_crc_err",   st->n_crc_err);
    sh_kv("bus_state",     (uint32_t)servo_bus_state());

    /*==========================================================================
     * ⭐⭐ 零位与编码器限位（2026-10-16 新增）
     *
     *  回答"offset 是什么、限位在哪、怎么验证"三个问题。
     *
     *  ⚠️ 三个"零"：
     *     · URDF 零位 = 弯腿站姿       → q = 0        → 编码器 = offset
     *     · 标定姿态 = 腿伸直、髋水平   → q = q_calib  → 编码器 = 2048（上位机设）
     *     · 编码器零                    → 与上面无关    → 0
     *
     *  ⚠️ 验证方法（把机器人摆成弯腿站姿，用上面的 raw pos_step 对照）：
     *     实测 pos_step 应该 ≈ 本节的 enc_zero_*
     *------------------------------------------------------------------------*/
    const rlca_calib_t *cal = app_calib_table();
    static const float q_cal[RLCA_N_JOINTS] = RLCA_CALIB_POSE_Q;

    sh_puts("-- ZERO / LIMITS  (validate: put robot in BENT-LEG STAND) --\r\n");
    sh_kv("calib_pose_step", (uint32_t)RLCA_CALIB_POSE_STEP);  /* 腿伸直 = 2048 */

    sh_puts("  j | id | dir | off  | q_cal_deg | enc@qcal | enc@zero |"
            " joint_deg_lo | joint_deg_hi\r\n");

    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        const int32_t dir = (int32_t)cal->dir[i];
        const int32_t off = (int32_t)cal->offset[i];

        /* 标定姿态（腿伸直）下的编码器值 —— 应该 ≈ 2048。
         * ⚠️ 用 rlca_q_to_step 而不手算，这样环绕行为与真正下发时一致。 */
        int32_t enc_at_qcal = (int32_t)rlca_q_to_step(cal, i, q_cal[i]) % 4096;
        if (enc_at_qcal < 0) { enc_at_qcal += 4096; }

        /* URDF 零位（弯腿站姿）下的编码器值 —— 就是 offset */
        int32_t enc_at_zero = off % 4096;
        if (enc_at_zero < 0) { enc_at_zero += 4096; }

        /* ⚠️⚠️ limit_min/max 是【带符号的相对步数】（见 rlca_calib_fill_limits：
         *    它用 offset=0、dir=+1 换算，所以对负限位会得到负步数）。
         *    必须先转 int32 保留符号，否则 −39 会被当成 65533，
         *    乘出来就是天文数字（曾因此在 K 里打印出 4294967257）。 */
        const int32_t lim_lo_step = (int32_t)cal->limit_min[i];
        const int32_t lim_hi_step = (int32_t)cal->limit_max[i];

        /* ⭐ 关节角范围（度）：步数 × rad/step，再按 dir 还原成关节角。
         *
         *  ⚠️ dir = −1 时，"步数增"对应"关节角减"，所以 limit_min 步
         *     换出来的角度反而【大于】limit_max 步换出来的角度。
         *     直接按 (lo, hi) 打印会出现 "35..-89" 这种倒挂 —— 那不是错，
         *     但读起来容易误判，所以这里按【数值大小】排序后再打印。 */
        const float q_a = (float)lim_lo_step * RLCA_SERVO_RAD_PER_STEP * (float)dir;
        const float q_b = (float)lim_hi_step * RLCA_SERVO_RAD_PER_STEP * (float)dir;
        const float q_lo = (q_a < q_b) ? q_a : q_b;
        const float q_hi = (q_a < q_b) ? q_b : q_a;

        sh_puts("  j");
        sh_put_u32(i);
        sh_puts(" | ");
        sh_put_u32(RLCA_SERVO_ID(i));
        sh_puts(" | ");
        sh_puts(dir > 0 ? "+1" : "-1");
        sh_puts(" | ");
        sh_put_u32((uint32_t)off);
        sh_puts(" | ");
        sh_put_u32((uint32_t)(q_cal[i] * 180.0f / 3.14159265f + 0.5f));
        sh_puts(" | ");
        sh_put_u32((uint32_t)enc_at_qcal);
        sh_puts(" | ");
        sh_put_u32((uint32_t)enc_at_zero);
        sh_puts(" | ");
        sh_put_i32((int32_t)(q_lo * 180.0f / 3.14159265f + 0.5f));
        sh_puts(" | ");
        sh_put_i32((int32_t)(q_hi * 180.0f / 3.14159265f + 0.5f));
        sh_puts("\r\n");
    }

    sh_puts("  legend: off/enc@zero = encoder at URDF zero (BENT-LEG STAND);\r\n");
    sh_puts("          enc@qcal should read 2048 in the STRAIGHT-LEG pose;\r\n");
    sh_puts("          q_cal_deg is a MAGNITUDE (sign lives in 'dir')\r\n");
}

/** C <start|cancel|zero|erase> —— 标定控制 */
static void cmd_calib(char *args)
{
    if (strcmp(args, "start") == 0) {
        if (app_calib_start()) {
            sh_puts("calibration started (~10 s)\r\n");
            sh_puts("  unload -> settle 1.5s -> sample -> dir test x12 -> save\r\n");
            sh_puts("  WARNING: every joint will move ~0.15 rad. Keep clear.\r\n");
        } else {
            sh_puts("start FAILED. two possible reasons:\r\n");
        sh_puts("  a) servo bus busy  -> wait 20 ms and retry\r\n");
        sh_puts("  b) no fresh 12-servo feedback yet -> check 'B' ok_reads is rising\r\n");
        }
    } else if (strcmp(args, "cancel") == 0) {
        app_calib_cancel();
        sh_puts("cancelled\r\n");
    } else if (strcmp(args, "zero") == 0) {
        /* ⚠️ 手动零点：先把机器人摆成【装配站姿】，再敲这一条。
         *    比全自动流程准，因为姿态是人为摆正的。 */
        app_calib_manual_zero();
        sh_puts("manual zero recorded from current positions\r\n");
        sh_puts("  (dir keeps current values; 'C start' measures dir)\r\n");
    } else if (strcmp(args, "erase") == 0) {
        sh_puts(app_calib_erase() ? "flash erased, defaults restored\r\n"
                                  : "erase FAILED\r\n");
    } else {
        sh_puts("usage: C <start|cancel|zero|erase>\r\n");
        sh_puts("  start - automatic (unload/settle/sample/dir/save)\r\n");
        sh_puts("  zero  - record current positions as zero (pose it first!)\r\n");
        sh_puts("  erase - wipe the flash calib page\r\n");
    }
}

/** T：周期耗时统计。
 *  ⭐ 这是排查"100 Hz 是否达标"的唯一直接手段。
 *     若 max_us 远大于 10000（10 ms），说明某一环阻塞了 ——
 *     而状态灯会以 5 Hz 快闪来提示这件事（app_main.c 的超时判据）。 */
static void cmd_timing(void)
{
    const app_cycle_stats_t *st = app_cycle_stats();
    const link_pi_stats_t   *ls = link_pi_stats();

    sh_puts("-- cycle timing (100 Hz -> budget 10000 us) --\r\n");
    sh_kv("cycles",   st->n);
    sh_kv("last_us",  st->last_us);
    sh_kv("max_us",   st->max_us);
    sh_kv("avg_us",   (st->n > 0u) ? (st->sum_us / st->n) : 0u);
    sh_kv("overruns", st->overruns);

    /* ⭐⭐ 主循环相邻两次 app_poll 的最大间隔（2026-10-17 加）
     *   ⚠️ `link_pi_poll()` 只在 `app_poll()` 里跑，所以本值若
     *      > RLCA_LINK_LOST_MS，就证明"接收解析被周期阻塞饿死"——
     *      而不是线上丢帧。表现就是 LINK_OK 熄灭 → ACTIVE 降级 STAND
     *      → 关节在【目标角】和【q_def=0】之间来回摆。 */
    sh_kv("poll_gap_max_us", st->poll_gap_max_us);

    /* ⭐⭐ 主循环分步最大耗时（2026-10-17 加）—— 定位 173 ms 那个阻塞。
     *   ⚠️ 谁最大，谁就是饿死接收解析的元凶。 */
    sh_puts("-- main-loop step max (us) --\r\n");
    {
        uint8_t worst = 0u;
        uint32_t wv = 0u;
        for (uint8_t i = 0u; i < (uint8_t)APP_STEP_N; ++i) {
            sh_puts("  ");
            sh_puts((char *)app_step_name(i));
            sh_puts(" = ");
            sh_put_u32(st->step_max_us[i]);
            sh_puts("\r\n");
            if (st->step_max_us[i] > wv) { wv = st->step_max_us[i]; worst = i; }
        }
        sh_puts("  >>> slowest step: ");
        sh_puts((char *)app_step_name(worst));
        sh_puts(" = ");
        sh_put_u32(wv);
        sh_puts(" us\r\n");
    }
    sh_kv("poll_gap_thr_us", (uint32_t)RLCA_LINK_LOST_MS * 1000u);
    /* ⚠️ 这段以前会把 poll_gap 超阈值直接判成"链路被饿死"。现在**不成立**了：
     *    `sh_puts()` 已改成每 64 字节调一次 `link_pi_poll()`，
     *    所以即使 shell 打 90 ms，`s_last_rx_ms` 也是新鲜的。
     * ⇒ poll_gap 大只说明"主循环这一圈长"，**不等于**链路判定会误触发。
     *    真正的判据要看 R 里的 link_rx_* 计数：
     *      rx_timeouts / crc_errors / len_errors > 0  ⇒ 字节在线上丢了
     *      三者全 0 而 rx_frames 偏少                 ⇒ 帧根本没到 */
    if (st->poll_gap_max_us > ((uint32_t)RLCA_LINK_LOST_MS * 1000u)) {
        sh_puts("NOTE: poll_gap exceeded the link timeout, but sh_puts()\r\n");
        sh_puts("      pumps link_pi_poll() in 64-byte chunks, so the RX\r\n");
        sh_puts("      path was NOT starved. Judge wire health from the\r\n");
        sh_puts("      link_rx_timeouts / crc_errors / len_errors counters.\r\n");
    }

    /* ⚠️ wait_timeouts 持续增长 = 每周期都在白等那 4 ms（链路不活）*/
    sh_kv("wait_timeouts", ls->wait_timeouts);

    if (st->max_us > (RLCA_CTRL_PERIOD_MS * 1000u)) {
        sh_puts("WARN: max cycle exceeds the 10 ms budget ->\r\n");
        sh_puts("      check (a) wait_timeouts growth  (b) IMU/I2C slow\r\n");
        sh_puts("            (c) servo bus timeouts      (d) ESP32 fwd stall\r\n");
    } else {
        sh_puts("OK: every cycle met the 10 ms budget.\r\n");
    }
}

/** I：IMU 寄存器【原始字节】dump。
 *
 *  ⭐ 排查"姿态不对"的终极手段。三种原因只能这样区分：
 *     ① 寄存器地址错  → 数值量级/内容明显不对
 *     ② 字节序错      → 大端与小端两种解释差别巨大（下面同时打印两者）
 *     ③ 换算系数错    → 原始值合理但换算结果不对
 *
 *  ⚠️ 大写 I，与小写 i（I2C 总线探测）区分。 */
static void cmd_imu_raw(void)
{
    static const struct { uint8_t reg; uint8_t len; const char *name; } dump[] = {
        { 0x2Eu, 2u, "VERSION   (0x2E)" },
        { 0x1Au, 2u, "IICADDR   (0x1A)" },   /* ⭐ I2C 从机地址，手册默认 0x50 */
        { 0x02u, 2u, "RSW       (0x02)" },
        { 0x03u, 2u, "RRATE     (0x03)" },   /* ⭐ 回传速率，要设成 9 = 100Hz */
        { 0x1Fu, 2u, "BANDWIDTH (0x1F)" },   /* ⭐ 带宽，上位机最高 98Hz -> 编码 2 */
        { 0x20u, 2u, "GYRORANGE (0x20)" },
        { 0x21u, 2u, "ACCRANGE  (0x21)" },
        { 0x23u, 2u, "ORIENT    (0x23)" },
        { 0x24u, 2u, "AXIS6     (0x24)" },
        { 0x34u, 6u, "ACC  0x34~36" },
        { 0x37u, 6u, "GYRO 0x37~39" },
        { 0x3Du, 6u, "ANGLE 0x3D~3F" },
        { 0x51u, 8u, "QUAT 0x51~54" },
    };
    static const char hx[] = "0123456789ABCDEF";
    uint8_t buf[8];

    sh_puts("-- IMU raw registers --\r\n");

    for (uint8_t k = 0u; k < (uint8_t)(sizeof(dump) / sizeof(dump[0])); ++k) {
        sh_puts(dump[k].name);
        sh_puts(" = ");
        if (!imu_read_raw(dump[k].reg, buf, dump[k].len)) {
            sh_puts("READ FAIL\r\n");
            continue;
        }
        for (uint8_t i = 0u; i < dump[k].len; ++i) {
            /* ⚠️ 没有 sh_putc，用两字符串 + 结束符（长度 3 的栈缓冲足够）。
             *    这里刻意不走 sh_put_u32（那是十进制）。 */
            char b2[3];
            b2[0] = hx[(buf[i] >> 4) & 0x0Fu];
            b2[1] = hx[buf[i] & 0x0Fu];
            b2[2] = '\0';
            sh_puts(b2);
            sh_puts(" ");
        }
        /* 成对的 16 位寄存器：同时给出大端与小端两种解释。
         * ⚠️ 这一栏是判断"字节序对不对"的关键：若一种解释合理、
         *    另一种离谱，结论就很明确。 */
        if ((dump[k].len % 2u) == 0u) {
            sh_puts("|BE:");
            for (uint8_t i = 0u; i < dump[k].len; i = (uint8_t)(i + 2u)) {
                const int16_t be = (int16_t)(((uint16_t)buf[i] << 8) | buf[i + 1u]);
                sh_puts(" ");
                sh_put_i32((int32_t)be);
            }
            sh_puts("  |LE:");
            for (uint8_t i = 0u; i < dump[k].len; i = (uint8_t)(i + 2u)) {
                const int16_t le = (int16_t)(((uint16_t)buf[i + 1u] << 8) | buf[i]);
                sh_puts(" ");
                sh_put_i32((int32_t)le);
            }
        }
        sh_puts("\r\n");
    }

    sh_puts("hints:\r\n");
    sh_puts("  ORIENT 0=level / 1=vertical(Y arrow up)\r\n");
    sh_puts("  fw assumes ACC=+/-16g, GYRO=+/-2000dps, ANGLE span=+/-180deg\r\n");
    sh_puts("  at rest |acc| should be ~32768 (1g under the +/-16g assumption)\r\n");
}

/** E [0|1]：UART3（ESP32）遥测镜像开关。
 *
 *  ⭐ 这是"无线调试时看不到乱码"的关键手段。
 *
 *  ⚠️ UART3 物理上只有一条线，它同时承载：
 *       · 遥测镜像（二进制协议帧，10 Hz）
 *       · shell 的文本输入输出
 *     两者混在一起 → 文本终端里 ASCII 夹杂二进制 → 看起来就是乱码。
 *     这与 UART1 上一直存在的现象同源（协议帧 + shell 共用一条线）。
 *
 *  用法：
 *     E       -> 查看当前状态
 *     E 0     -> 关闭镜像，UART3 变成【纯文本通道】（无线调试用这个）
 *     E 1     -> 恢复镜像（ESP32 才能收到遥测）
 *
 *  ⚠️ 关闭镜像会让 ESP32 收不到遥测（其控制口的 s/z/c 失去门控依据），
 *     所以这是【调试期临时手段】，调完记得 E 1 打开。 */
static void cmd_esp_fwd(char *args)
{
    while ((*args == ' ') || (*args == '\t')) { args++; }

    if (*args == '0') {
        esp_fwd_set_enabled(false);
        sh_puts("esp mirror OFF  (UART3 = clean text only)\r\n");
    } else if (*args == '1') {
        esp_fwd_set_enabled(true);
        sh_puts("esp mirror ON   (UART3 = text + 10 Hz telemetry)\r\n");
    } else {
        sh_puts("esp mirror = ");
        sh_puts(esp_fwd_is_enabled() ? "ON" : "OFF");
        sh_puts("\r\n  usage: E 0 = off (clean text) / E 1 = on\r\n");
    }
}

/*==============================================================================
 * UART2 回环自测
 *============================================================================*/

/** X：强制重启 USART2（舵机总线）的 DMA 接收。
 *
 *  ⭐ 用途：如果接收被一次 ORE 溢出卡死，本命令能【不用断电】恢复。
 *
 *  ⚠️ 为什么需要它：实测发现一次接收错误会把 DMA 接收静默卡死
 *     （`rx_frames` 永久归零、`A` 全 no response，而 TX 完全正常）。
 *     自动恢复逻辑已加到 bsp_uart_poll_dma()（检出错误即重启），
 *     本命令是【手动兜底】—— 万一自动恢复没触发，还能现场救回来。
 *
 *  判读：敲完后敲 B，若 rx_frames 开始增长、id=9 的 pos/V/T 有值，
 *        说明接收恢复。若仍是 0，问题在更前面（接线/控制板/短接线没拔）。 */
static void cmd_uart2_restart(void)
{
    bsp_uart_restart_rx(&g_uart_servo);
    sh_puts("USART2 RX restarted (DMA re-armed, ring buffer cleared)\r\n");
    sh_puts("  now run B and watch rx_frames / dma_restarts\r\n");
}

/** U：UART2（舵机总线）回环自测。
 *
 *  ⭐ 把"STM32 侧路径坏"与"控制板/舵机不响应"彻底分开的唯一手段。
 *
 *  做法：短接 PA2(TX) 与 PA3(RX)，然后敲本命令。
 *  它直接【绕过 servo_bus 状态机】和舵机协议，做两件事：
 *    ① 裸字节回环：写 16 个已知字节，看能不能原样收回来
 *    ② STS 帧回环：按舵机帧格式发一个 PING，看能不能整帧收回来
 *
 *  判读：
 *    D1 通过 + D2 通过 → STM32 的 TX/RX、波特率、DMA 全都正常
 *                        → "no response" 只能是控制板或舵机侧
 *    D1 通过 + D2 失败 → 能收发字节，但帧解析/超时有问题
 *    D1 失败           → PA2/PA3 没短接对、或 USART2 没起、或 DMA 没收
 *
 *  ⚠️ 本命令会阻塞最多约 40 ms（两次回环各等 50 ms 上限）。
 *     仅用于台面诊断，不要在闭环运行时敲。
 *  ⚠️ 用完【务必拔掉短接线】—— 留着会让舵机总线收发互相打架。 */
static void cmd_uart2_loop(void)
{
    sh_puts("-- UART2 loopback self-test --\r\n");
    sh_puts("  (short PA2 <-> PA3 and run again if this fails)\r\n");

    /* 先清空残留，避免把旧字节误当回环结果 */
    bsp_uart_flush(&g_uart_servo);

    /* ---- ① 裸字节回环 ---- */
    static const uint8_t pat[16] = {
        0x00u, 0xFFu, 0xAAu, 0x55u, 0x01u, 0x80u, 0x7Fu, 0x3Cu,
        0xC3u, 0x0Fu, 0xF0u, 0x12u, 0x34u, 0x56u, 0x78u, 0x9Au
    };
    (void)bsp_uart_write(&g_uart_servo, pat, sizeof(pat));

    uint32_t t0 = bsp_tick_ms();
    while ((bsp_uart_available(&g_uart_servo) < (uint16_t)sizeof(pat)) &&
           ((bsp_tick_ms() - t0) < 50u)) {
        bsp_uart_poll_dma();
    }

    uint8_t got[16];
    const uint16_t n = bsp_uart_read(&g_uart_servo, got, (uint16_t)sizeof(got));

    sh_kv("D1 raw recv bytes", n);
    bool same = (n == (uint16_t)sizeof(pat));
    if (same) {
        for (uint8_t i = 0u; i < (uint8_t)sizeof(pat); ++i) {
            if (got[i] != pat[i]) { same = false; }
        }
    }
    sh_puts("D1 raw loopback = ");
    sh_puts(same ? "PASS\r\n" : "FAIL\r\n");
    if (!same && (n > 0u)) {
        static const char hx2[] = "0123456789ABCDEF";
        sh_puts("   got:");
        for (uint16_t i = 0u; i < n; ++i) {
            char b3[4];
            b3[0] = ' ';
            b3[1] = hx2[(got[i] >> 4) & 0x0Fu];
            b3[2] = hx2[got[i] & 0x0Fu];
            b3[3] = '\0';
            sh_puts(b3);
        }
        sh_puts("\r\n");
    }

    /* ---- ② STS 帧回环：发一个 PING(1)，看能否整帧收回 ---- */
    bsp_uart_flush(&g_uart_servo);
    uint8_t frame[8];
    uint16_t fn = 0u;
    uint16_t rn = 0u;
    bool d2 = false;
    if (rlca_sts_build_ping(1u, frame, (uint16_t)sizeof(frame), &fn) == STS_TX_OK) {
        (void)bsp_uart_write(&g_uart_servo, frame, fn);
        t0 = bsp_tick_ms();
        while ((bsp_tick_ms() - t0) < 50u) {
            bsp_uart_poll_dma();
            if (bsp_uart_available(&g_uart_servo) >= fn) { break; }
        }
        uint8_t rsp[16];
        rn = bsp_uart_read(&g_uart_servo, rsp, (uint16_t)sizeof(rsp));
        /* 回环收到的应该是一模一样的 PING 请求帧 */
        d2 = (rn == fn);
        if (d2) {
            for (uint16_t i = 0u; i < fn; ++i) {
                if (rsp[i] != frame[i]) { d2 = false; }
            }
        }
        sh_kv("D2 sts sent bytes", fn);
        sh_kv("D2 sts recv bytes", rn);
    }
    sh_puts("D2 sts frame loopback = ");
    sh_puts(d2 ? "PASS\r\n" : "FAIL\r\n");

    sh_puts("hints:\r\n");
    sh_puts("  D1 pass + D2 pass -> STM32 TX/RX fine; fault is board/servo side\r\n");
    sh_puts("  D1 fail           -> PA2/PA3 not shorted, or USART2/DMA dead\r\n");
    sh_puts("  REMOVE the jumper afterwards!\r\n");
}

/*==============================================================================
 * PING 原始字节抓取
 *============================================================================*/

/** Y <id>：PING 单个 ID，并【无条件 dump 原始应答字节】。
 *
 *  ⭐ 这是把"舵机没回"与"回了但解析失败"分开的唯一手段。
 *
 *  ⚠️ 为什么需要它（实测踩到）：
 *     接了 ID 4 和 ID 8 两个舵机，`B` 里却只有 id=5 有数据。
 *     而 `A` 只报 ACK / no-response —— 但"no response"有两种截然不同的原因：
 *       ① 舵机根本没回（接线、ID 不对、控制板方向）
 *       ② 舵机回了，但格式/长度/校验与预期不符，解析器拒收
 *     只看 A 的结论【区分不了】这两者，必须看原始字节。
 *
 *  判读：
 *     收到 FF FF <id> ...  -> 舵机在回；A 若仍报 no-response，问题在解析
 *     一个字节都没收到     -> 舵机/接线问题，与解析无关 */
static void cmd_ping_raw(char *args)
{
    while ((*args == ' ') || (*args == '\t')) { args++; }

    int32_t id = 0;
    {
        const char *p = args;
        if (!parse_int(&p, &id)) {
            sh_puts("usage: Y <servo_id 1..12>\r\n");
            return;
        }
    }
    if ((id < 1) || (id > 12)) {
        sh_puts("id out of range (1..12)\r\n");
        return;
    }

    sh_puts("-- ping raw dump, id = ");
    sh_put_u32((uint32_t)id);
    sh_puts(" --\r\n");

    /* 直接构造并发送 PING，然后【自己】收字节 —— 不走 servo_bus 的解析 */
    bsp_uart_flush(&g_uart_servo);

    uint8_t tx[8];
    uint16_t n = 0u;
    if (rlca_sts_build_ping((uint8_t)id, tx, (uint16_t)sizeof(tx), &n) != STS_TX_OK) {
        sh_puts("build ping FAILED\r\n");
        return;
    }
    sh_kv("sent bytes", n);
    (void)bsp_uart_write(&g_uart_servo, tx, n);

    /* 等最多 50 ms，边等边搬 DMA */
    uint8_t rx[64];
    uint16_t total = 0u;
    const uint32_t t0 = bsp_tick_ms();
    while ((bsp_tick_ms() - t0) < 50u) {
        bsp_uart_poll_dma();
        const uint16_t av = bsp_uart_available(&g_uart_servo);
        if (av > 0u) {
            const uint16_t room = (uint16_t)(sizeof(rx) - total);
            const uint16_t want = (av < room) ? av : room;
            if (want > 0u) {
                total = (uint16_t)(total + bsp_uart_read(&g_uart_servo, &rx[total], want));
            }
        }
        if (total >= 8u) { break; }   /* PING 应答实测 8 字节 */
    }

    sh_kv("recv bytes", total);
    if (total == 0u) {
        sh_puts("  -> NO BYTES AT ALL: servo did not reply\r\n");
        sh_puts("     (wiring / wrong id / bus adapter direction) - NOT a parse bug\r\n");
        return;
    }

    static const char hx3[] = "0123456789ABCDEF";
    sh_puts("  raw:");
    for (uint16_t i = 0u; i < total; ++i) {
        char b3[4];
        b3[0] = ' ';
        b3[1] = hx3[(rx[i] >> 4) & 0x0Fu];
        b3[2] = hx3[rx[i] & 0x0Fu];
        b3[3] = '\0';
        sh_puts(b3);
    }
    sh_puts("\r\n");

    if ((total >= 4u) && (rx[0] == 0xFFu) && (rx[1] == 0xFFu)) {
        sh_kv("  frame id",  rx[2]);
        sh_kv("  frame len", rx[3]);
        sh_puts("  -> header OK, reply arrived.\r\n");
    } else {
        sh_puts("  -> header is NOT FF FF: bytes are noise or misaligned.\r\n");
    }

    /* ⭐ 第二条路径：走 servo_bus_ping()（内部含总线互斥 + 解析）。
     *
     * ⚠️ 为什么要两条都跑：这一步能把"硬件收不到"与"解析/互斥有问题"分开。
     *     两条结果对照：
     *        raw 有字节 + ping_ok = 1  -> 全链路正常 ✅
     *        raw 有字节 + ping_ok = 0  -> 收到了但判据拒绝（解析/超时问题）
     *        raw 无字节 + ping_ok = 0  -> 舵机/接线问题，与固件无关
     *
     *     ⚠️ 这正是用来验证"后台同步读与 shell 抢总线"那个竞态是否修好的手段：
     *        修好前 `Y` 能收到而 `A` 全失败；修好后两者应当一致。 */
    sh_kv("servo_bus_ping ok", servo_bus_ping((uint8_t)id) ? 1u : 0u);
}

/*==============================================================================
 * 总线时序在线调整
 *============================================================================*/

/** Z [turnaround_us] [frame_timeout_us] [idle_gap_us]：查看/设置总线时序。
 *
 *  ⭐⭐ 这是排查"舵机偶尔才应答"的决定性工具。
 *
 *  ⚠️ 背景（实测）：接上舵机后 `B` 里只有 1 个关节偶尔有数据
 *     （≈1% 成功率），而 PING 是 0%。这看着就是半双工总线的
 *     【收发切换余量不够】—— 1 Mbps 下 1 字节 = 10 µs，
 *     而默认切换等待只有 20 µs（仅 2 个字节时间）。
 *
 *  ⚠️ 这个值该多大【只能靠实测】（取决于适配器、线长、舵机型号）。
 *     做成在线可调 + 直接显示成功率，就能边调边看，
 *     不必"改代码→重编译→重烧"地试。
 *
 *  用法：
 *     Z                       -> 显示当前时序与成功率
 *     Z 0                     -> 清零诊断计数（改参数前先清）
 *     Z <ta>                  -> 设切换等待
 *     Z <ta> <ft> <gap>       -> 同时设 切换等待 / 整帧超时 / 静默判据
 *
 *  判读：改一个值 -> `Z 0` -> 等 5 秒 -> `Z`
 *        看 frames_per_100_rounds；试几个值取最高的。 */
static void cmd_bus_timing(char *args)
{
    while ((*args == ' ') || (*args == '\t')) { args++; }

    if (*args != '\0') {
        const char *p = args;
        int32_t v1 = 0, v2 = 0, v3 = 0;
        const bool has1 = parse_int(&p, &v1);
        const bool has2 = parse_int(&p, &v2);
        const bool has3 = parse_int(&p, &v3);

        if (has1 && (v1 == 0) && !has2) {
            servo_bus_reset_diag();
            sh_puts("diag counters cleared\r\n");
            return;
        }
        if (has1) {
            servo_bus_set_timing((uint16_t)v1,
                                 has2 ? (uint16_t)v2 : 0u,
                                 has3 ? (uint16_t)v3 : 0u);
            sh_puts("timing updated\r\n");
        }
    }

    uint16_t ta = 0u, ft = 0u, gap = 0u;
    servo_bus_get_timing(&ta, &ft, &gap);
    uint32_t rounds = 0u, frames = 0u, bytes = 0u;
    servo_bus_get_diag(&rounds, &frames, &bytes);

    sh_puts("-- servo bus timing --\r\n");
    sh_kv("turnaround_us",   ta);
    sh_kv("frame_timeout_us",ft);
    sh_kv("idle_gap_us",     gap);
    sh_puts("-- answer rate (since last 'Z 0') --\r\n");
    sh_kv("rounds", rounds);
    sh_kv("frames", frames);
    sh_kv("bytes",  bytes);
    if (rounds > 0u) {
        sh_kv("frames_per_100_rounds", (frames * 100u) / rounds);
        sh_puts("  (with N servos connected, expect ~N per round)\r\n");
    }
    sh_puts("usage: Z | Z 0 | Z <ta> [ft] [gap]\r\n");
}

/*==============================================================================
 * USART2 / DMA 寄存器直读
 *============================================================================*/

/** D：直读 USART2 与它的 RX DMA 寄存器。
 *
 *  ⭐⭐ 排查"接收完全收不到字节"的终极手段 ——
 *     前面的命令只能看到"软件层面有没有搬字节"，
 *     这个命令能看到【硬件层面 DMA 是否还在工作】。
 *
 *  ⚠️ 背景（实测）：接上舵机后，开机早期能收到 96 帧，
 *     之后就【永久 0 字节】—— 把收发切换等待加到 25 倍也毫无改善。
 *     `bytes = 0` 说明既不是解析问题、也不是时序问题，
 *     而是"字节根本没进 DMA 缓冲"。到底是 DMA 停了、
 *     还是 USART2 的接收被关了，只有读寄存器才知道。
 *
 *  判读：
 *     dma_en = 0                -> DMA 通道被关了 → 按 X 重新武装
 *     dma_wr_pos 一直不变       -> DMA 在等，但总线上真的没字节（舵机没回）
 *     dma_wr_pos 在变但 bytes 不涨 -> drain_dma 逻辑有问题 */
static void cmd_uart2_regs(void)
{
    extern UART_HandleTypeDef huart2;

    sh_puts("-- USART2 + RX DMA registers --\r\n");

    /* ⭐⭐⭐ 先测【不可污染】的指标：纯硬件字节计数。
     *
     *  ⚠️ 采样 2 秒，看它有没有增长。这一个数字就能定性：
     *      · 在涨  -> USART2 接收线上【确实有字节】；
     *                那么"收不到数据"是我软件搬运/解析的问题
     *      · 不动  -> 线上【真的没字节】；
     *                那是代码之外的问题（接线 / 适配器方向 /
     *                舵机没回 / DMA 没武装）
     *
     *  ⚠️ 它绕过了 drain_dma、环形缓冲、dma_last、解析器
     *     —— 也就是绕过了我所有可能出错的地方。 */
    bsp_uart_dma_hw_reset();
    {
        /* ⚠️⚠️ 采样期间必须【主动发 PING】，否则这个测试没有意义。
         *
         *  我第一次写成了"静默采样 2 秒"—— 但只有在"有请求发出去"
         *  的前提下，"没有字节回来"才能说明"舵机没应答"。
         *  静默采样得到 0 既可能是舵机没回，也可能只是没人问它。
         *
         *  这里每 100 ms 发一次 PING(5)，共 20 次。判读：
         *    hw > 0  -> 舵机确实会回；那"收不到"是我的软件问题
         *    hw == 0 -> 发了 20 次 PING 都没回；问题在代码之外
         *               （接线 / 适配器方向 / 舵机 / 我发出去的帧是坏的） */
        const uint32_t t0 = bsp_tick_ms();
        uint32_t n_ping = 0u;
        uint8_t  ptx[8];
        uint16_t pn = 0u;
        (void)rlca_sts_build_ping(5u, ptx, (uint16_t)sizeof(ptx), &pn);

        while ((bsp_tick_ms() - t0) < 2000u) {
            bsp_uart_poll_dma();
            if (((bsp_tick_ms() - t0) % 100u) < 3u) {
                if (pn > 0u) {
                    (void)bsp_uart_write(&g_uart_servo, ptx, pn);
                    n_ping++;
                }
            }
        }
        sh_kv("pings_sent_in_2s", n_ping);
    }
    const uint32_t hw = bsp_uart_dma_hw_bytes();

    sh_puts("=== RAW HW CHECK (2s, pinging id 5) ===\r\n");
    sh_kv("dma_hw_bytes_2s", hw);
    sh_puts(hw > 0u ? "  -> servo IS replying; fault is in MY software\r\n"
                    : "  -> ZERO bytes even after PINGs; fault is on the wire\r\n");
    sh_puts("======================================\r\n");

    sh_kv("usart2_SR",   (uint32_t)huart2.Instance->SR);
    sh_kv("usart2_BRR",  (uint32_t)huart2.Instance->BRR);
    sh_kv("usart2_CR1",  (uint32_t)huart2.Instance->CR1);
    /* CR1: bit2 = RE(收使能), bit13 = UE(USART 总使能) */
    sh_kv("  RE(bit2)",  (uint32_t)((huart2.Instance->CR1 >> 2) & 1u));
    sh_kv("  UE(bit13)", (uint32_t)((huart2.Instance->CR1 >> 13) & 1u));

    sh_puts("-- HAL state --\r\n");
    sh_kv("hal_gState",    (uint32_t)huart2.gState);
    sh_kv("hal_RxState",   (uint32_t)huart2.RxState);
    sh_kv("hal_ErrorCode", (uint32_t)huart2.ErrorCode);

    if (huart2.hdmarx != NULL) {
        DMA_Channel_TypeDef *ch = (DMA_Channel_TypeDef *)huart2.hdmarx->Instance;
        sh_puts("-- RX DMA channel --\r\n");
        sh_kv("dma_ccr",    (uint32_t)ch->CCR);
        sh_kv("  dma_en",   (uint32_t)(ch->CCR & 1u));
        sh_kv("dma_cndtr",  (uint32_t)ch->CNDTR);
        sh_kv("dma1_isr",   (uint32_t)DMA1->ISR);
        sh_kv("hal_dma_state", (uint32_t)huart2.hdmarx->State);
        sh_kv("dma_buf_size",  (uint32_t)g_uart_servo.dma_size);
        /* 由 CNDTR 反推"已写入位置"，与 drain_dma 的算法一致 */
        sh_kv("dma_wr_pos", (uint32_t)(g_uart_servo.dma_size - ch->CNDTR));
        sh_kv("dma_last",   (uint32_t)g_uart_servo.dma_last);
    } else {
        sh_puts("  hdmarx = NULL (DMA not bound!)\r\n");
    }

    sh_kv("ring_rx_total", g_uart_servo.rx_total);
    sh_kv("ring_overflow", g_uart_servo.rx_overflow);
    sh_puts("hint: dma_en=0 -> DMA off; use X\r\n");
}

/*==============================================================================
 * 解析与分发
 *============================================================================*/

static void shell_run_line(char *line)
{
    /* 去掉尾部空白 */
    size_t n = strlen(line);
    while ((n > 0u) && ((line[n - 1u] == '\r') || (line[n - 1u] == '\n') ||
                        (line[n - 1u] == ' ') || (line[n - 1u] == '\t'))) {
        line[--n] = '\0';
    }
    if (n == 0u) {
        return;
    }

    char *args = line + 1;
    while ((*args == ' ') || (*args == '\t')) { args++; }

    switch (line[0]) {
    case 'h': case '?': cmd_help(); break;
    case 's': cmd_status(); break;
    case 't': cmd_timing(); break;
    case 'n': app_stats_clear(); sh_puts("timing stats cleared\r\n"); break;
    case 'i': cmd_i2c(); break;
    case 'I': cmd_imu_raw(); break;
    case 'A': cmd_servo_ping_all(); break;
    case 'T': cmd_servo_torque(args); break;
    case 'P': cmd_servo_pos_one(args); break;
    case 'W': cmd_servo_write_raw(args); break;
    case 'F': cmd_servo_pos_all(args); break;
    case 'B': cmd_servo_stats(); break;
    case 'M': cmd_imu(); break;
    case 'V': cmd_batt(); break;
    case 'N': cmd_mode(args); break;
    case 'R': cmd_run_state(); break;
    case 'K': cmd_boot_safety(); break;
    case 'J': cmd_imu_bypass(args); break;
    case 'C': cmd_calib(args); break;
#if RLCA_SHELL_LINK_CMDS_EN
    case 'L': cmd_link_stats(); break;
    case 'g': cmd_link_send(args[0]); break;
    case 'G': cmd_link_stream(args); break;
#endif
    case 'p': cmd_echo(args); break;
    case 'w': cmd_frame(args); break;
    case 'E': cmd_esp_fwd(args); break;
    case 'U': cmd_uart2_loop(); break;
    case 'X': cmd_uart2_restart(); break;
    case 'Y': cmd_ping_raw(args); break;
    case 'D': cmd_uart2_regs(); break;
    case 'Z': cmd_bus_timing(args); break;
    case 'r':
        g_uart_pi.rx_overflow = 0u;    g_uart_pi.rx_errors = 0u;
        g_uart_servo.rx_overflow = 0u; g_uart_servo.rx_errors = 0u;
        g_uart_esp.rx_overflow = 0u;   g_uart_esp.rx_errors = 0u;
        sh_puts("counters reset\r\n");
        break;
    case 'l':
        if (*args == '0') { bsp_led_set(BSP_LED_OFF); }
        else if (*args == '1') { bsp_led_set(BSP_LED_ON); }
        else { sh_puts("usage: l <0|1>\r\n"); }
        sh_puts("ok\r\n");
        break;
    case 'm':
        if      (strcmp(args, "off")   == 0) { bsp_led_set(BSP_LED_OFF); }
        else if (strcmp(args, "on")    == 0) { bsp_led_set(BSP_LED_ON); }
        else if (strcmp(args, "slow")  == 0) { bsp_led_set(BSP_LED_SLOW_BLINK); }
        else if (strcmp(args, "fast")  == 0) { bsp_led_set(BSP_LED_FAST_BLINK); }
        else if (strcmp(args, "estop") == 0) { bsp_led_set(BSP_LED_PATTERN_ESTOP); }
        else { sh_puts("usage: m <off|on|slow|fast|estop>\r\n"); }
        sh_puts("ok\r\n");
        break;
    case 'e':
        sh_puts(bsp_flash_erase_calib() ? "erased\r\n" : "erase FAILED\r\n");
        break;
    default:
        sh_puts("unknown cmd; type h\r\n");
        break;
    }
}

/*==============================================================================
 * 对外接口
 *============================================================================*/

void bsp_shell_init(void)
{
    sh_puts("\r\nRL-CAT STM32 firmware -- P2 self-test shell\r\n");
    sh_puts("type 'h' for help\r\n");
}

#define SH_N_PORTS  2u

/** 每条端口独立的行缓冲与写入位置。
 *  ⚠️ 必须【每端口一份】：否则一条从 USART1 敲了一半的命令，
 *     会被 USART3 的字节接着拼起来，产生莫名其妙的命令。
 *     反正只用 2 × 96 B，RAM 够。 */
static char     sh_line[SH_N_PORTS][SH_LINE_MAX];
static uint16_t sh_idx[SH_N_PORTS];

/** 处理某条端口上收到的字节。 */
static void sh_feed(bsp_uart_t *port, uint8_t slot)
{
    int16_t c;
    while ((c = bsp_uart_getc(port)) >= 0) {
        /* ⭐ 关键：把"输出端口"切到本端口，这样回显走回命令来的那条线 */
        s_cur = port;

        if ((c == '\r') || (c == '\n')) {
            if (sh_idx[slot] > 0u) {
                sh_line[slot][sh_idx[slot]] = '\0';

                /* ⭐⭐ 命令执行期间【暂停舵机总线状态机】（2026-10-17 修复）
                 *
                 *  ⚠️ 为什么必须暂停：
                 *     `sh_puts()` 走 `bsp_uart_write()`，是【阻塞】的 —— 输出发完才返回。
                 *     ESP32 无线口只有 115200 baud，于是：
                 *         'R' ≈ 1400 字符 ≈ 122 ms
                 *         'K' ≈ 3150 字符 ≈ 274 ms
                 *     这段时间里 `servo_bus_poll()` 一次都不会被调用。
                 *     而总线状态机有一个 3 ms 的整帧超时：
                 *         若某一轮停在 SVB_READ_RSP 等 12 帧回复，
                 *         3 ms 后必然超时 ⇒ raise_err(BUS_TIMEOUT)
                 *     ⇒ **每按一次 R/K 就白送一次总线超时**
                 *     ⇒ BUS_TIMEOUT 2 s 后才自动解除 ⇒ 表现为"错误在 0 和 BUS_TIMEOUT 之间跳"
                 *     ⇒ 而 SERVO_ERR 会把动作幅度压到 75%（amp_scale_x100 = 75）
                 *
                 *  ⭐ 暂停的语义（见 servo_bus_pause）：
                 *     · 清掉在飞行中的事务（bus_abort_now），**不**计超时
                 *     · 期间 `servo_bus_poll()` 直接返回，不会发起新事务
                 *     ⚠️ 直接写路径不受影响：servo_bus_write_one / set_all_torque
                 *        内部自己 bus_abort_now() 再发，不检查 s_paused。
                 *        所以 'A' / 'T' / 'P' / 'W' / 'F' 这些命令照常工作。
                 *     ⚠️ 那些自己直接操作 g_uart_servo 的诊断命令
                 *        （'Y' ping / 'D' regs / 'U' loopback / 'Z' timing）
                 *        本来就绕过状态机，暂停对它们没有副作用。 */
                servo_bus_pause();
                shell_run_line(sh_line[slot]);
                servo_bus_resume();

                sh_idx[slot] = 0u;
            }
        } else if ((sh_idx[slot] + 1u) < SH_LINE_MAX) {
            sh_line[slot][sh_idx[slot]++] = (char)c;
        } else {
            /* 行太长：丢弃并复位，避免溢出 */
            sh_idx[slot] = 0u;
            sh_puts("line too long\r\n");
        }
    }
}

void bsp_shell_poll(void)
{
    /* ⭐ 两条端口都抽干。
     *  ⚠️ 顺序无所谓：两条线的字节各自进自己的缓冲，
     *     不会互相污染（见 sh_line 的注释）。
     *  ⚠️ USART3 就是 ESP32 的无线透传口（TCP 3333 → USART3）——
     *     于是"无线敲命令、无线看回显"成立。 */
    /* ⭐⭐ USART1 默认**不**喂（2026-10-18 修的关键 bug）
     *
     *  ⚠️ USART1 是 Pi 的二进制协议链路（0xAA55 帧），`link_pi_poll()`
     *     读的是**同一个接收环**。两个消费者抢一条字节流 ⇒ 帧被切碎
     *     ⇒ 解码器反复半帧超时（实测 5134 次 vs 只发了 2928 帧）。
     *  ⇒ 默认只喂 USART3（ESP32 无线），那是真正的"人机口"。
     *  ⚠️ 要用 USB-TTL 有线调试时把 RLCA_SHELL_ON_PI_PORT 置 1，
     *     并且**不要同时接树莓派**。 */
#if (RLCA_SHELL_ON_PI_PORT != 0)
    sh_feed(&g_uart_pi,  0u);   /* USART1：有线（USB-TTL 调试专用） */
#endif
    sh_feed(&g_uart_esp, 1u);   /* USART3：无线（ESP32 WiFi 透传）  */
}

#endif /* RLCA_SHELL_EN */
