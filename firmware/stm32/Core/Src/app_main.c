/**
  ******************************************************************************
  * @file    app_main.c
  * @brief   上电自检 + 100 Hz 主循环调度。
  *
  *  P2 阶段的职责边界（明确写清，避免越界）：
  *    本阶段【只做】BSP 层与调度骨架的验证：
  *      · 时钟确实 72 MHz
  *      · DWT 微秒计时可用
  *      · 三路 UART 能收（计数可见）、USART1 能发（shell）
  *      · 软件 I2C 总线电平正常
  *      · Flash 标定页读写
  *      · LED / 心跳
  *    【不做】：舵机总线事务、树莓派帧协议、IMU 解析、状态机、安全逻辑
  *      —— 这些分别是 P3/P4/P5/P6。
  *
  *  ⚠️ 因此本阶段的 app_cycle_100hz() 里没有 SyncWrite/SyncRead，
  *     只有占位与统计。这样 P2 的上机验收能独立完成，不被后续阶段阻塞。
  ******************************************************************************
  */

#include <string.h>

#include "app_main.h"

#include "main.h"
#include "rlcat_config.h"
#include "bsp_uart.h"
#include "bsp_tick.h"
#include "bsp_gpio.h"
#include "bsp_softi2c.h"
#include "bsp_flash.h"
#include "bsp_shell.h"
#include "rlca_phase.h"
#include "servo_bus.h"
#include "servo_port.h"
#include "link_pi.h"
#include "link_proto.h"
#include "battery.h"
#include "imu.h"
#include "sensor_ports.h"
#include "app_mode.h"
#include "app_safety.h"
#include "rlca_calib_proc.h"
#include "esp_fwd.h"

/*==============================================================================
 * 内部状态
 *============================================================================*/

static volatile uint32_t s_cycle_cnt = 0u;
static app_cycle_stats_t s_stats = { 0 };

/* 100 Hz 调度基准（⚠️ 用"累加 10 ms"而不是"= now"，
 * 避免 HAL_Delay 之类造成的漂移被永久固化） */
static uint32_t s_next_cycle_ms = 0u;

static app_boot_result_t s_boot = { 0 };

/* 链路层需要缓存的状态 */
static uint32_t s_last_cmd_ms   = 0u;      /* 上次收到有效 CMD_JOINT 的时刻 */
static bool     s_have_cmd      = false;

/** ⭐⭐⭐ 上电"上锁"：在收到【明确指令】之前，不准上力矩、不准写位置。
 *
 *  ⚠️ 项目负责人明确要求（2026-10-16）：
 *     "在没有明确指令之前，不准上力矩，不准写位置。"
 *
 *  为什么需要这个锁（现场教训）：
 *     · 上电后固件会自行进入各种"自动"路径（播种、限速、ESTOP 趴下斜坡），
 *       而这些路径都会在【未标定】状态下算出错误的步数目标
 *       ⇒ 12 个舵机一起冲向一个错位置。
 *     · 标定完成后 s_calib_ok 变 true，闭环路径也会【立刻】开始写位置，
 *       同样不需要任何外部指令。
 *
 *  ⇒ 解药就是"要求一个显式的接管请求"：
 *     只有收到带 RLCA_FLAG_ENABLE 的 CMD_JOINT（Pi 或 ESP32 发的都算），
 *     才解除上锁。在此之前：
 *        · want_torque 被强制为 false（不上力矩）
 *        · 位置写被拒绝（不写位置）
 *        · 只保留"持续重申卸力"这一件安全动作
 *
 *  ⚠️ 这个标志【上电清零、只由明确指令置位】，不做超时复位 ——
 *     失联时的降级/急停由安全层负责，不在这里处理。 */
static bool     s_armed         = false;

/** ⭐ 实际下发的力矩状态（= 经过 s_armed 锁与安全层之后的最终值）。
 *  ⚠️ 与 `app_mode_state()->torque_on`（模式状态机的期望值）区分：
 *     两者不一致，就说明被 s_armed 锁或其它覆盖挡住了。 */
static bool     s_torque_actual = false;

bool app_torque_actual(void)
{
    return s_torque_actual;
}

bool app_is_armed(void)
{
    return s_armed;
}

static uint32_t s_status_last_ms = 0u;     /* TEL_STATUS 的 10 Hz 节流 */
static bool     s_req_status    = false;   /* CMD_JOINT.flags 请求立即上报 */
static bool     s_link_degraded = false;
static bool     s_calib_ok      = false;

/* 关节状态快照（供 TEL_FAST 组装）。
 * ⚠️ 单独存一份而不是直接用 servo_bus 的数组：这样即使总线本轮读失败，
 *    TEL_FAST 也能报出"上一次有效值 + valid 标志位"，而不是突然全 0。 */
static rlca_joint_state_t s_js[RLCA_N_JOINTS];
static bool s_js_valid[RLCA_N_JOINTS];

/* 上次实际下发的目标角（限速的基准） */
static float s_q_cmd[RLCA_N_JOINTS];
static float s_q_limited[RLCA_N_JOINTS];

/* 当前生效的标定表（供限幅/换算用） */
static rlca_calib_t s_calib;

/** 标定表是否有未落盘的改动。
 *  ⚠️ Flash 擦写会让 CPU 停 20~40 ms，绝不能放在 100 Hz 周期里，
 *     所以标定完成后只置这个标志，真正的写在 app_housekeeping 里做。 */
static bool s_calib_dirty = false;

/** ⭐ 上电后还需重发几次 torque-off。
 *  ⚠️ 见 app_init 里的详细说明：控制板适配器与 STM32 同时上电时，
 *     上电头几百毫秒内适配器不理 UART，第一条 torque-off 会丢。
 *     这里在 100 Hz 循环里补发，丢掉的那些会被后面的补上。 */
static uint16_t s_torque_burst_left = 0u;

/** ⭐ 允许开始【同步写位置】的时刻（ms）。
 *
 *  ⚠️⚠️ 为什么需要这道闸（实测撞机事故的第二层机制）：
 *     舵机把"目标位置"存在自己的非易失存储里。若上次停在 step 2000，
 *     而固件第一条 SyncWrite 让它去 step 0，它会【全速走 176°】——
 *     link_limits_apply 的 0.15 rad/周期限速【拦不住】这一跳，
 *     因为限速只约束"指令之间的变化"，不约束"舵机当前位置与首个指令
 *     之间的落差"。
 *
 *     更要命的是这条指令可能比适配器就绪还早，于是：
 *     位置写了但力矩没关 → 舵机冲向那个陈旧的目标位置。
 *
 *  所以上电后【先只读、不发位置】，等适配器就绪并读到真实位置后，
 *  再开始正常事务。见 RLCA_SERVO_STARTUP_GATE_MS。 */
static uint32_t s_servo_gate_ms = 0u;

/** ⭐ 目标值是否已用真实反馈播种（见 ⑦ 的闸门逻辑）。
 *  ⚠️ 只有它为 true 才允许发同步【写】事务 —— 这是防撞机的最后一道闸。 */
static bool s_q_seeded = false;

/** ⭐⭐ 上电闸门期的安全诊断（见 ⑦）。
 *  ⚠️ "闸门期镜像写位置"那段兜底已在 2026-10-16 删除 ——
 *     它本身是位置写，而"未标定就不该写位置"才是正确原则。
 *     这两个计数现在只用于上报"曾经被拒绝写位置多少次"，证明规则生效。 */
static bool     s_boot_goal_seeded      = false;   /* 恒为 false（历史字段，保留兼容） */
static uint32_t s_boot_goal_seed_writes = 0u;      /* 恒为 0（历史字段，保留兼容） */

/** ⭐⭐ 持续重申泄力（见 ⑥b）。
 *  ⚠️ 现场证据：上电后舵机仍在一秒内持续运动 ⇒ 限时补发的 torque-off 没生效。
 *     改成"只要 want_torque == false 就周期性重申"，不再依赖窗口假设。 */
static uint32_t s_torque_reassert_ms    = 0u;      /* 上次重申的时刻 */
static uint32_t s_torque_reassert_count = 0u;      /* 累计重申帧组数（诊断用） */

/** ⭐⭐ 操作者"手动力矩覆盖"（2026-10-16 增，供 W 命令调试）。
 *
 *  ⚠️ 为什么需要：⑥b 每 200 ms 重申一次卸力，而那次调用的内部
 *     会 bus_abort_now() 掐断刚发出的位置帧 + 关掉力矩
 *     ⇒ 用 W 调试时必须反复连发，中途会停。
 *
 *  ⚠️ 语义：只有【人在 shell 上敲 T 1】才能置位；T 0 或收到明确链路指令
 *     会清除。它只暂停"周期性重申"，不改变正常控制路径的任何判据。 */
static bool     s_manual_torque = false;

/** ⭐⭐⭐ 上电时 Flash 里【有没有】一份合法标定表。
 *
 *  ⚠️ 为什么要单独存一份：s_calib_ok 是可变的（标定过程中/手动零点都会改），
 *     但"上电那一刻有没有标定"这个事实要留给诊断与状态上报用。
 *
 *  ⚠️ 与位置写许可的关系（见 ⑦）：
 *     未标定 ⇒ 关节角度没有意义 ⇒ 一条位置指令都不发 + 持续卸力矩。
 *     本标志只是【记录】，真正的判据是 s_calib_ok（标定完成后会置位）。 */
static bool s_boot_calib_present = false;

/** 上电后是否【曾经】被拒绝写位置（诊断：证明"未标定不写位置"生效了）。 */
static uint32_t s_pos_write_blocked_count = 0u;

static const rlca_calib_t *app_calib(void);

/** IMU 是否可用。
 *  ⚠️ 直接读 imu_is_ok()：它要求"近期读到过合法数据"。
 *     没有 IMU 时 STAT_IMU_OK 不会被置位 —— Pi 侧启动自检会据此拒绝进入
 *     ACTIVE（31-pi-runtime.md §5.1 第④步），这正是我们要的行为：
 *     "没接 IMU 就别跑策略"，而不是拿着假姿态硬跑。 */
static bool s_boot_imu_ok(void)
{
    return imu_is_ok();
}

/** 组装 TEL_FAST/TEL_STATUS 的 status 位域（协议 §4.5） */
static uint8_t app_status_bits(uint32_t now)
{
    uint8_t st = 0u;

    if (s_boot.clock_ok && s_boot.dwt_ok) {
        st |= RLCA_ST_READY;
    }
    if (s_calib_ok) {
        st |= RLCA_ST_CALIBRATED;
    }
    /* ⚠️ STAT_LINK_OK 必须反映"近期收到过指令"，
     *    这是 ESP32 做指令仲裁（32-esp32-bridge.md §4.3）的唯一依据。 */
    if (link_pi_is_alive(now)) {
        st |= RLCA_ST_LINK_OK;
    }
    if (s_boot_imu_ok()) {
        st |= RLCA_ST_IMU_OK;
    }
    /* 电池分级位：读数失效时不置任何电压位（不要把"读不到"当成"电压正常"） */
    if (battery_is_valid()) {
        st |= batt_status_bits(battery_mv());
    }
    /* ⚠️ STAT_ESTOP 用【实际的生效模式】判定，而不是 Pi 请求的模式 ——
     *    安全层可以在 Pi 没要求的情况下自己进急停，那才是要上报的。 */
    if (app_mode_current() == (uint8_t)RLCA_MODE_ESTOP) {
        st |= RLCA_ST_ESTOP;
    }
    return st;
}

/*==============================================================================
 * 标定流程的舵机硬件适配
 *============================================================================*/

/* ⚠️ 标定流程通过注入的接口访问舵机，这样它能被离线单测。
 *    这里是"真实硬件"的实现：单关节位置读写走 servo_bus 的单机指令。 */

static bool calib_hw_get_pos(uint8_t joint_idx, int16_t *pos_step)
{
    if ((joint_idx >= RLCA_N_JOINTS) || (pos_step == NULL)) {
        return false;
    }
    const servo_fb_t *fb = servo_bus_feedback();
    if (fb[joint_idx].valid == 0u) {
        return false;
    }
    *pos_step = fb[joint_idx].pos_step;
    return true;
}

static bool calib_hw_set_pos(uint8_t joint_idx, int16_t pos_step)
{
    return servo_bus_write_one(joint_idx, pos_step);
}

static void calib_hw_torque_all(bool enable)
{
    /* ⚠️⚠️⚠️ 使能力矩【之前】必须先把 12 个关节的当前步数写成目标
     *        （2026-10-16 修复的严重撞机 bug）
     *
     *  现场现象：标定做"方向测试"的微动之后，出现【大幅度的猛烈运动】，撞机。
     *            有时则表现为标定期完全不动、标定失败。
     *
     *  根因：标定流程在 CAL_SAMPLE 采完样本后调用 s_hw->set_torque_all(true)
     *        使能力矩 —— 但那时【只写了第 0 个关节的目标位置】：
     *            goal0 = base[0] + CAL_DIR_DELTA_STEP;
     *            s_hw->set_pos(0u, goal0);
     *        另外 11 个舵机的 GOAL_POSITION 寄存器里仍然是【它们自己存的旧值】
     *        （舵机把目标位置存在非易失存储里，断电不清除）。
     *
     *        ⇒ 力矩一使能，那 11 个舵机【同时】奔向各自那个陈旧目标。
     *          若旧目标在远端（很可能是上一次标定/控制留下的错值），
     *          就是全速猛冲 → 撞机。
     *        ⇒ 有时"不动"则是因为旧目标恰好等于当前位置，于是被判 NO_MOTION。
     *
     *  ⭐ 修法：把"使能力矩"与"先钉住当前位置"绑成一个原子动作 ——
     *        先给全部 12 个关节写 goal = 当前实测步数，再使能力矩。
     *        这样无论舵机内部存着什么旧目标，力矩一开都是"待在原地"。
     *
     *  ⚠️⚠️ 关键：这一步必须【完全绕开角度域】，用 servo_bus_start_cycle_raw()。
     *
     *     不能写成 servo_bus_write_one(i, fb[i].pos_step)：
     *       那个入口会把传进去的 int16 当作【当前标定表下的步数目标】，
     *       物理位置是 step_to_q(target) —— 而标定表随时可能刚被改过
     *       （标定流程自己就会改 offset/dir）。一旦表变了，
     *       "写 step X" 与 "停在物理步数 X" 就不是一回事了，可能差一两千步。
     *
     *     也不能写成 write_one(i, q_to_step(calib, i, step_to_q(calib, i, fb[i].pos_step)))：
     *       那个往返虽然数学上恒等，但 q_to_step 之前会被 rlca_q_clamp 按软限位夹住
     *       （apply_q_clamp=true），而软限位是按标定表算的 —— 未标定时它可能
     *       把角度夹到边界，于是仍然算出错误的步数。
     *
     *     start_cycle_raw() 只把步数夹到编码器物理范围 [0,4095]，
     *     完全不碰标定表 ⇒ 物理位置严格等于 fb[i].pos_step，恒等成立。
     *
     *  ⚠️ 为什么放在这里而不是改 calib_proc：标定流程是"纯逻辑"模块
     *     （rlca_calib_proc.c），它的契约是"注入的硬件接口负责真实硬件语义"。
     *     "使能力矩前先钉住位置"正是硬件语义，放在适配层最合适，
     *     也不需要改任何单测。 */
    if (enable) {
        const servo_fb_t *fb = servo_bus_feedback();
        bool all_valid = true;

        for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
            if (fb[i].valid == 0u) {
                all_valid = false;
            }
        }

        /* ⚠️ 只在 12 个反馈都有效时才钉位置：
         *    少一个就用旧值写，那个舵机会跑到一个没根据的位置。
         *    若不全，宁可不钉（力矩照常使能，但风险由操作者承担 ——
         *    所以标定流程的入口已经要求"反馈必须新鲜"。）
         *
         * ⚠️ 必须用 servo_bus_write_one_raw()（阻塞、立即发出），
         *    不能用 servo_bus_start_cycle_raw()：后者只是【排队】，
         *    实际发送在 servo_bus_poll() 里，而紧接着的
         *    servo_bus_set_all_torque() 会 bus_abort_now() 把它清掉
         *    —— 等于根本没钉住。 */
        if (all_valid) {
            for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
                (void)servo_bus_write_one_raw(i, fb[i].pos_step);
            }
        }
    }
    servo_bus_set_all_torque(enable);
}

static const rlca_calib_hw_t s_calib_hw = {
    calib_hw_get_pos,
    calib_hw_set_pos,
    calib_hw_torque_all,
    bsp_tick_ms
};

/* ---- 标定流程与外部之间的中转（供 shell 与 link 调用） ---- */

bool app_calib_start(void);
void app_calib_manual_zero(void);
bool app_calib_take_result(rlca_calib_t *out);
bool app_calib_busy(void);
uint8_t app_calib_progress(void);
bool app_calib_failed(void);

bool app_calib_start(void)
{
#if (RLCA_CALIB_LOCK != 0)
    /* ⭐ 标定流程已锁（RLCA_CALIB_LOCK = 1）
     * ⚠️ 用几何表之后不需要标定；留着入口只为"换舵机"。
     *    真要标定时把该开关临时置 0、标定完再置回 1。
     * ⚠️ 这里【不】返回 true —— 那会让调用方以为标定开始了。 */
    return false;
#endif
    /* ⚠️ 标定不能在 ACTIVE 模式里启动：那会和策略争抢舵机目标角。 */
    if (!servo_bus_is_idle()) {
        return false;
    }

    /* ⚠️⚠️ 必须先有一份【新鲜的 12 帧反馈】，理由有两层：
     *     ① 标定流程的 get_pos() 读的就是 servo_bus_feedback()，
     *        反馈没生效过时它一律返回 false，标定第一步就 READ_POS 失败；
     *     ② 手动零点（C zero）依赖"当前站姿的读数"，
     *        拿旧值当零点会让整个标定白做。
     *     上电后若还没读全过，这里直接拒绝，让操作者等一两个周期再试。 */
    if (!servo_bus_feedback_fresh()) {
        return false;
    }

    app_mode_enter_calib();
    calib_proc_init(&s_calib_hw);
    calib_proc_start(&s_calib);
    return true;
}

/** 操作者把机器人摆到装配站姿后调用：直接采样当前位置作为零点。
 *  ⚠️ 比全自动流程更准（姿态是人摆正的），但需要人确认姿态。 */
void app_calib_manual_zero(void)
{
#if (RLCA_CALIB_LOCK != 0)
    /* ⭐ 同上：手动零点也锁掉，避免误触把几何表覆盖 */
    return;
#endif
    rlca_calib_t c = s_calib;

    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        const servo_fb_t *fb = servo_bus_feedback();
        if (fb[i].valid != 0u) {
            c.offset[i] = fb[i].pos_step;
        }
        /* ⚠️ dir 保留当前值（默认全 +1）。手动零点只能定 offset，
         *    方向仍需靠全自动流程或人工确认来定。 */
    }
    c.magic   = RLCA_CALIB_MAGIC;
    c.version = RLCA_CALIB_VERSION;
    c.crc16   = rlca_calib_crc(&c);

    if (rlca_calib_is_valid(&c)) {
        s_calib = c;
        /* ⭐⭐ 必须同步给 servo_bus：它内部是【复制】一份表的（servo_bus_init 里
         *     `s_calib = *calib;`）。不刷新的话，角度换算会一直用上电时的旧表，
         *     表现为"R 里的 offset 是对的，但关节角是错的（±π）"，且必须重启才生效。 */
        servo_bus_set_calib(&s_calib);
        s_calib_dirty = true;      /* 落盘在主循环里做（Flash 擦写会停 CPU） */
    }
}

bool app_calib_take_result(rlca_calib_t *out)
{
    rlca_calib_t r;
    if (!calib_proc_take_done(&r)) {
        return false;
    }
    s_calib = r;
    /* ⭐⭐ 同步给 servo_bus（原因见 app_calib_manual_zero 里的说明）。
     *  ⚠️ 这是"标定完必须重启才生效"那个 bug 的修复点 —— 标定流程走到这里
     *     才真正把新表交给角度换算。 */
    servo_bus_set_calib(&s_calib);
    s_calib_dirty = true;
    app_mode_exit_calib();
    if (out != NULL) {
        *out = r;
    }
    return true;
}

bool app_calib_busy(void)
{
    const calib_state_t st = calib_proc_state();
    return (st != CAL_IDLE) && (st != CAL_DONE) && (st != CAL_FAILED);
}

uint8_t app_calib_progress(void)
{
    return calib_proc_progress();
}

bool app_calib_failed(void)
{
    return calib_proc_failed();
}

uint8_t app_calib_err(void)
{
    return (uint8_t)calib_proc_err();
}

void app_calib_cancel(void)
{
    calib_proc_cancel();
    app_mode_exit_calib();
}

bool app_calib_erase(void)
{
    if (!bsp_flash_erase_calib()) {
        return false;
    }
    rlca_calib_load_defaults(&s_calib);
    s_calib_ok = false;
    servo_bus_init(servo_port_hardware(), &s_calib);
    return true;
}

const rlca_calib_t *app_calib_table(void)
{
    return &s_calib;
}

void app_boot_goal_seed_stat(bool *seeded, uint32_t *writes)
{
    /* ⚠️ 这两个字段是历史遗留（"闸门期镜像写位置"已删除，见 ⑦）。
     *    保留接口是为了不破坏 shell 的 K 命令输出格式。 */
    if (seeded != NULL) { *seeded = s_boot_goal_seeded; }
    if (writes != NULL) { *writes = s_boot_goal_seed_writes; }
}

void app_torque_reassert_stat(uint32_t *count, uint32_t *burst_left)
{
    if (count != NULL)      { *count = s_torque_reassert_count; }
    if (burst_left != NULL) { *burst_left = (uint32_t)s_torque_burst_left; }
}

void app_pos_write_gate_stat(bool *boot_calib_present, bool *pos_write_allowed,
                             uint32_t *blocked_count, bool *q_seeded)

{
    if (boot_calib_present != NULL) { *boot_calib_present = s_boot_calib_present; }
    if (blocked_count != NULL)      { *blocked_count = s_pos_write_blocked_count; }
    if (q_seeded != NULL)           { *q_seeded = s_q_seeded; }
    if (pos_write_allowed != NULL) {
        /* ⚠️ 复述 ⑦ 的判据（同一套条件，不重复实现） */
        const uint32_t now = bsp_tick_ms();
        const bool gate_open = ((int32_t)(now - s_servo_gate_ms) >= 0);
        *pos_write_allowed = (s_calib_ok && s_armed && gate_open && !app_calib_busy());
    }
}

/*=============================================================================
 * ⭐⭐ 操作者手动力矩覆盖（见 s_manual_torque 的说明）
 *
 *  ⚠️ 实现放在这里而不是文件开头：必须能访问 s_manual_torque 与
 *     s_torque_burst_left（它们在文件中部才声明）。
 *===========================================================================*/

void app_set_manual_torque(bool on)
{
    s_manual_torque = on;
    if (on) {
        /* 打开覆盖时立刻把力矩打开，免得等主循环那一拍 */
        servo_bus_set_all_torque(true);
    } else {
        s_torque_burst_left = RLCA_TORQUE_OFF_BURST;   /* 重新来一轮密集重申 */
        servo_bus_set_all_torque(false);
    }
}

bool app_get_manual_torque(void)
{
    return s_manual_torque;
}

void app_clear_manual_torque(void)
{
    s_manual_torque = false;
}

/*==============================================================================
 * 初始化
 *============================================================================*/

void app_init(void)
{
    /* BSP 层：必须按顺序 —— tick 最优先（后面都要用它计时） */
    bsp_tick_init();
    bsp_gpio_init();
    bsp_uart_init_all();
    bsp_softi2c_init();

    /* ⚠️ 上电自检必须【在 shell 之前】跑，shell 里要报告它的结果 */
    s_boot = app_boot_selftest();

    /*==========================================================================
     * ⚠️⚠️ 第 0 优先：上电第一件事【卸掉所有舵机力矩】
     *
     *  ⭐ 这是踩过一次严重撞机事故后加的（2026-10-11）。
     *
     *  【事故现象】第一次接上控制板，一插电、什么命令都没发，
     *     12 个关节同时朝同一方向冲到底。
     *
     *  【根因】舵机把"目标位置"和"力矩使能"存在自己的非易失存储里。
     *     断电只是没电，**力矩状态并没有清除**。
     *     所以一上电，力矩还是使能的，舵机立刻冲向【上次存的目标位置】。
     *     —— 这与固件无关，是上电瞬间就发生的，比 Cortex-M3 启动还早。
     *     固件原来要等到第 1 个 100 Hz 周期才发 torque-off，
     *     那时舵机早就冲出去了。
     *
     *  【正确做法】凡是"接执行器"的系统，都必须
     *     ① 上电最先发卸力指令  ② 先记录当前位置再使能
     *     现在是第 ① 步的兜底。
     *
     *  ⚠️ 顺序说明：这里必须放在 servo_bus_init 之【后】
     *     （它绑定 s_port 并复位状态），但一定要尽量靠前。
     *  ⚠️ 此时总线状态机是 IDLE，12 条 torque-off 会直接写出去，
     *     不会与同步读写事务冲突。
     *
     *  ⚠️⚠️ 为什么【一条远远不够】—— 已实测确认的现场机制：
     *     控制板（Waveshare Bus Servo Adapter）与 STM32 常从同一路电源上电，
     *     而适配器自身也要启动。上电头几百毫秒内它【不理 UART】，
     *     这一条写出去会石沉大海，而舵机仍处于力矩使能状态。
     *     更糟的是 servo_bus_set_torque() 无条件 return true，
     *     固件【以为】力矩已关，实际没关。
     *
     *     实测判据：断开 UART2 只给控制板上电 → 舵机不动；
     *               接上 UART2 一起上电 → 12 个关节冲到底。
     *     => 动的指令来自 STM32，而不是舵机自己的上电行为。
     *
     *     所以下面还要在 100 Hz 循环里【反复重发】一段时间
     *     （见 s_torque_burst_left），丢掉的那些会被后面的补上。
     *
     *  ⚠️⚠️⚠️ 顺序修正（2026-10-16）：servo_bus_init 必须在前！
     *     原先这里先发 torque-off、后面才 servo_bus_init，而
     *     servo_bus_set_torque() 开头就是 `if (s_port == NULL) return false;`
     *     ⇒ s_port 还是 NULL，这 12 条 torque-off 【全部被静默丢弃】，
     *       一条都没上总线。下一条真正到达舵机的命令要等到
     *       RLCA_SERVO_STARTUP_GATE_MS（1 s）之后的同步写 ——
     *       而舵机在这 1 s 里一直保持"上次的力矩使能 + 上次的目标位置"，
     *       于是上电即冲向陈旧目标。这正是本文档描述的那起撞机事故。
     *
     *     ⇒ 先 load 标定表 → servo_bus_init（只绑定端口，不发帧）
     *       → 立刻 torque-off。这样第一条上总线的命令就是卸力矩。
     */
    rlca_calib_t cal;

#if (RLCA_CALIB_GEOMETRIC_EN != 0)
    /* ⭐⭐⭐ 几何标定表（2026-10-18）：完全不依赖 flash
     *
     *  ⚠️ 前提：标定姿态（腿伸直）下 12 个舵机编码器都读到 2048。
     *     项目负责人已在机械上逐台确认。
     *
     *  ⭐ 为什么不读 flash 了：
     *     现场出现过"重启后有概率变成未标定"，根因是
     *     `s_calib_ok` 被捆在 flash 落盘成功与否上
     *     （落盘失败 ⇒ 明明有合法表却判未标定），而且失败的落盘是静默的。
     *     改成常量推导后，这条故障路径**从结构上消失**：
     *       · 结果只由 `RLCA_JOINT_DIR_INIT` + `RLCA_CALIB_POSE_Q` 决定
     *       · 重启 100 次也是同一张表
     *       · 不存在"未标定"这个中间态
     *
     *  ⚠️ 读取 flash 的那条路径（下面 #else 分支）保留着，
     *     把 RLCA_CALIB_GEOMETRIC_EN 置 0 即可回到"逐台标定"模式。 */
    rlca_calib_build_geometric(&cal);
    s_calib    = cal;
    s_calib_ok = true;       /* 几何表恒合法；没有"未标定"这个状态 */

    /* ⚠️ 不读 flash，但仍要把 s_calib_dirty 清掉 —— 否则若上一次运行
     *    留下了待落盘标志，housekeeping 会去擦写那一页（无意义且有风险）。 */
    s_calib_dirty = false;
#else
    if (!bsp_flash_load_calib(&cal)) {
        rlca_calib_load_defaults(&cal);
    }
    s_calib = cal;
    s_calib_ok = false;      /* 由下面统一判定，见「默认表识别」 */

    /* ⭐⭐⭐ 「默认表识别」：区分"真的标定过" vs "只是一张合法但全零的默认表"
     *
     *  ⚠️ 背景（2026-10-17 实测踩到）：
     *     rlca_calib_load_defaults() 产出的表【结构上完全合法】
     *     （magic / version / dir / 限位 / CRC 都正常），所以
     *     rlca_calib_is_valid() 判它通过、bsp_flash_load_calib() 也能读出来。
     *     但它的 offset[] **全是 0**。
     *
     *     offset 的物理含义是"URDF 零位下舵机的编码器读数"。
     *     offset = 0 意味着"q=0 对应编码器 0" —— 对装在 2048 中位的舵机
     *     是完全错的。现场表现：在标定姿态（pos=2048）下上报
     *     q = 2048 × dir × 2π/4096 = ±π = ±3.14 rad，
     *     而正确值应是 q_calib = ±0.785 / ±1.571 rad。
     *
     *     更糟的是 STAT_CALIBRATED 会亮 ✅ —— 假的"已标定"比明确报未标定更危险。
     *
     *  ⚠️ 判据：12 个 offset 全为 0 ⇒ 认定"未标定"。
     *     这不会误伤真标定：真实舵机的装配中位不可能 12 个关节都恰好落在 0。
     *     （即使某个关节真在 0，也不可能 12 个全在 0。） */
    {
        bool all_zero = true;
        for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
            if (cal.offset[i] != 0) {
                all_zero = false;
                break;
            }
        }
        if (all_zero) {
            /* 保留 s_calib（角度换算仍会用到它），但明确标记为"未标定" */
            s_calib_ok = false;
        } else {
            s_calib_ok = bsp_flash_calib_loaded();
        }
    }
#endif /* RLCA_CALIB_GEOMETRIC_EN */

    servo_bus_init(servo_port_hardware(), &s_calib);

    /* ⭐ 现在 s_port 已绑定，torque-off 才真的能发出去 */
    servo_bus_set_all_torque(false);
    s_torque_burst_left = RLCA_TORQUE_OFF_BURST;
    s_servo_gate_ms     = bsp_tick_ms() + RLCA_SERVO_STARTUP_GATE_MS;
    s_q_seeded          = false;   /* ⚠️ 必须复位：否则 erase 后不会重新播种 */

    /* 相位时钟归零（P4/P6 会用到；此处先初始化，保证 TEL_FAST 的
     * gait_sin/cos 从确定值开始） */
    rlca_phase_reset();

    /* 树莓派链路 */
    link_pi_init(&s_calib);

    /* ⚠️⚠️ 这里【曾经】有一行无条件的 `s_calib_ok = bsp_flash_calib_loaded();`
     *
     *  它把上面 `#if` 分支里刚算好的 `s_calib_ok` **全部覆盖掉**，危害有三：
     *
     *   ① 几何表模式（RLCA_CALIB_GEOMETRIC_EN = 1）下，启动根本不读 flash，
     *      于是 `bsp_flash_calib_loaded()` 恒为 false ⇒ **STAT_CALIBRATED 熄灭**
     *      ⇒ ACTIVE 准入第 ① 道门（NOT_CALIB）永远挡住 ⇒ 机器人只能 STAND。
     *      现场表现：状态位 0x0D（READY|LINK_OK|IMU_OK，就是没有 CALIBRATED），
     *      `stand_test.py` 报"3 秒内没能进入 ACTIVE"。
     *
     *   ② 即使在 flash 模式（= 0）下，它也让"是否已标定"退化成
     *      "这次上电有没有从 flash 成功读出表"，与"表本身是否合法"脱钩 ——
     *      这正是历史上"重启有概率变成未标定"的来源之一。
     *
     *   ③ 它读起来"像是在确认启动事实"，实际是**晚到的重复赋值**，
     *      覆盖了前面更权威的判断，非常隐蔽。
     *
     *  ⇒ 已删除。`s_calib_ok` 只在两个地方被权威地设定：
     *      · 几何表模式：`#if` 分支里置 true
     *      · flash 模式 ：`#else` 分支里按"全零默认表识别"判定
     *    后续若要改，请改那两处，**不要**在这里再覆盖一次。 */

    /* ⭐ 记录"上电时有没有标定"这个事实（供诊断与状态上报）。
     *  ⚠️ s_calib_ok 之后会随标定流程变化，所以单独存一份。 */
    s_boot_calib_present = s_calib_ok;

    /* 传感器：ADC 必须先启动（CubeMX 只生成配置，不生成启动调用），
     * 再绑定端口。IMU 走 PB8/PB9 软件 I2C，端口初始化已在 bsp_softi2c_init 里做过。 */
    battery_port_start();
    battery_init(battery_port_hardware());
    imu_init(imu_port_hardware());
    (void)imu_selftest();      /* 失败只记录 version，不阻塞启动 */
    s_boot.imu_ok    = imu_is_ok();
    s_boot.imu_chip  = (uint8_t)(imu_state()->version & 0xFFu);

    /* 目标角初值 = 默认站姿（q_default = 0，即装配好的弯腿站姿） */
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        s_q_cmd[i]     = RLCA_Q_DEFAULT_RAD;
        s_q_limited[i] = RLCA_Q_DEFAULT_RAD;
        s_js_valid[i]  = false;
    }
    s_last_cmd_ms   = bsp_tick_ms();
    s_status_last_ms = s_last_cmd_ms;

    /* 模式状态机与标定流程 */
    app_mode_init();
    calib_proc_init(&s_calib_hw);

    /* ESP32 转发统计清零（UART3 的收发机制已在 bsp_uart_init_all 里启动） */
    esp_fwd_init();

    /* 100 Hz 调度基准 */
    s_next_cycle_ms = bsp_tick_ms() + RLCA_CTRL_PERIOD_MS;

    /* 上电自检结果的视觉指示：
     *   全部通过   → 状态灯灭（后续由业务状态接管）
     *   I2C 总线异常 → 慢闪（最可能是上拉没装 / 接错）
     *   其它异常   → 快闪 */
    if (!s_boot.i2c_bus_ok) {
        bsp_led_set(BSP_LED_SLOW_BLINK);
    } else if (!s_boot.dwt_ok || !s_boot.clock_ok) {
        bsp_led_set(BSP_LED_FAST_BLINK);
    } else {
        bsp_led_set(BSP_LED_OFF);
    }

#if RLCA_SHELL_EN && RLCA_BOOT_BANNER_EN
    bsp_shell_init();
    bsp_shell_poll();       /* 立刻处理 help 输出，避免下一条命令被吞 */
#endif
}

/*==============================================================================
 * 上电自检
 *============================================================================*/

app_boot_result_t app_boot_selftest(void)
{
    app_boot_result_t r = { false, false, false, false, 0u };

    /* ① 系统时钟必须是 72 MHz（CubeMX 配置清单 §2.1：时钟不对后面全白干） */
    const uint32_t sysclk = HAL_RCC_GetSysClockFreq();
    r.clock_ok = (sysclk == RLCA_SYS_CLK_HZ);

    /* ② DWT 微秒计时（舵机总线整帧超时依赖它） */
    r.dwt_ok = bsp_tick_selftest();

    /* ③ 软件 I2C 总线：释放后 SCL/SDA 应被外部上拉变高。
     *    ⚠️ 这一条能在"读不到 IMU 数据"之前就暴露硬件问题（4.7k 上拉没装）。 */
    r.i2c_bus_ok = bsp_softi2c_bus_ok();

    /* ④ 尝试加载标定数据 —— 失败不算致命（会自动进 CALIB），但要显式确认 */
    rlca_calib_t cal;
    (void)bsp_flash_load_calib(&cal);

    return r;
}

/*==============================================================================
 * 主循环任务
 *============================================================================*/

/** ⭐⭐ 主循环分步计时（2026-10-17 加）
 *
 *  ⚠️ 背景：`poll_gap_max_us = 173 ms` 而 `max_us`（周期内部）只有 15 ms。
 *     ⇒ 阻塞在 `app_poll()` 的某一步或 `app_housekeeping()` 里。
 *     ⇒ 加了这组分步统计，`t` 会直接告诉我们是最慢的是哪一步。
 *
 *  ⚠️ 超过 1 秒的样本丢弃（调试器停顿 / 时间源伪影），理由同 max_us。 */
static const char *const k_step_names[APP_STEP_N] = {
    "uart_poll_dma", "servo_bus_poll", "calib_proc", "link_pi_poll",
    "shell_poll",    "led_poll",       "housekeeping", "cycle_100hz"
};

const char *app_step_name(uint8_t idx)
{
    return (idx < (uint8_t)APP_STEP_N) ? k_step_names[idx] : "?";
}

/** ⭐⭐ 清零全部计时统计（给 shell 的 X 命令用）
 *
 *  ⚠️ 为什么需要：这些统计都是【上电以来累计】的最大值。
 *     现场排查时"跑完运动再敲 t"往往隔了很久（要切 WiFi 才能连 ESP32），
 *     于是最大值的来源可能是中途敲过的 R/K，而不是那次运动本身。
 *  ⭐ 用法：先 X 清零 → 跑运动 → 之后再敲 	 看，就只反映那次运动。 */
void app_stats_clear(void)
{
    memset(&s_stats, 0, sizeof(s_stats));
}

void app_step_record(uint8_t idx, uint32_t t0_us)
{
    if (idx >= (uint8_t)APP_STEP_N) {
        return;
    }
    const uint32_t d = bsp_tick_us() - t0_us;
    if ((d != 0u) && (d <= 1000000u) && (d > s_stats.step_max_us[idx])) {
        s_stats.step_max_us[idx] = d;
    }
}

void app_poll(void)
{
    /* ⭐⭐ 测"主循环相邻两次 app_poll 的间隔"（2026-10-17 加）
     *
     *  ⚠️ 目的：定位"带力矩运动时 LINK_OK 反复熄灭（40~150 ms）"的真凶。
     *     接收解析 `link_pi_poll()` 只在本函数里跑，所以只要主循环被
     *     `app_cycle_100hz()` 阻塞，接收就会被饿死、`s_last_rx_ms` 冻结。
     *     若本值 ≈ 40~150 ms，就证明是"周期阻塞"而不是"线上丢帧"。
     *  ⚠️ 超过 1 秒的样本丢弃（调试器停顿 / 时间源伪影），理由同 max_us。 */
    {
        static uint32_t s_prev_poll_us = 0u;
        const uint32_t now_u = bsp_tick_us();
        if (s_prev_poll_us != 0u) {
            const uint32_t gap = now_u - s_prev_poll_us;
            if ((gap != 0u) && (gap <= 1000000u)
                && (gap > s_stats.poll_gap_max_us)) {
                s_stats.poll_gap_max_us = gap;
            }
        }
        s_prev_poll_us = now_u;
    }

    /* ⚠️ 顺序要紧：先搬 DMA（USART2 舵机总线），
     * 再让上层取数据；否则 1 Mbps 下可能来不及。
     * ⭐ 每一步都计时（见 app_step_record），t 命令会显示最慢的那一步。 */
    uint32_t ts0;

    ts0 = bsp_tick_us(); bsp_uart_poll_dma();
    app_step_record(APP_STEP_UART_DMA, ts0);

    /* 舵机总线事务状态机：必须高频推进（它自己不阻塞主循环） */
    ts0 = bsp_tick_us(); servo_bus_poll();
    app_step_record(APP_STEP_SERVO_BUS, ts0);

    /* 标定流程状态机：同样高频推进（它自己非阻塞） */
    ts0 = bsp_tick_us(); calib_proc_poll();
    app_step_record(APP_STEP_CALIB, ts0);

    /* 树莓派链路：把 USART1 的字节灌进帧状态机并分发。
     * ⚠️ 必须在这里（高频）而不是只在 100 Hz 里调用 ——
     *    模型 A 的 4 ms 等待窗口内要靠它把 CMD_JOINT 收进来。 */
    ts0 = bsp_tick_us(); link_pi_poll();
    app_step_record(APP_STEP_LINK_RX, ts0);

#if RLCA_SHELL_EN
    ts0 = bsp_tick_us(); bsp_shell_poll();
    app_step_record(APP_STEP_SHELL, ts0);
#endif

    /* 心跳/状态灯由 SysTick 的毫秒时基推进，放这里保证 1 ms 级响应 */
    ts0 = bsp_tick_us(); bsp_led_poll(); bsp_heartbeat_poll();
    app_step_record(APP_STEP_LED, ts0);
}

void app_cycle_100hz(void)
{
#if RLCA_DEBUG_PERF
    const uint32_t t0 = bsp_tick_us();
#endif

    s_cycle_cnt++;
    const uint32_t now = bsp_tick_ms();

    /*==========================================================================
     * ① 取上一轮 SyncRead 的反馈 → 关节状态快照
     *========================================================================*/
    if (servo_bus_is_idle() && servo_bus_take_new_feedback()) {
        const rlca_joint_state_t *js = servo_bus_joint_states();
        for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
            s_js[i]       = js[i];
            s_js_valid[i] = (js[i].valid != 0u);
        }
    }

    /* ⚠️⚠️ 2026-10-16 删除了"闸门期镜像 GOAL_POSITION"的那段兜底。
     *
     *  那段试图在闸门期内把目标位置改成实测位置，绕开角度换算。
     *  但它本身【就是一条位置写】，而且写在"未标定"状态下 ——
     *  现场证据（用户观察）：
     *    · 拔掉 UART2 信号线 → 舵机不动（收不到位置写）
     *    · 接上 UART2         → 上电即动
     *    · 没接齐 12 个舵机   → 不动（播种门 n_valid==12 没过）
     *  ⇒ 动的指令必然是【位置写】，而不是卸力帧。
     *
     *  ⭐ 正确的原则（用户提出，已采纳）：
     *     未标定 ⇒ 关节角度没有意义 ⇒ 【一条位置指令都不发】+ 持续卸力矩。
     *     只有标定完成后才允许写位置。
     *     见 ⑦ 的 s_pos_write_allowed。 */

    /*==========================================================================
     * ② 收 Pi 指令
     *    ⚠️ 时机很重要：TEL_FAST 必须【先发】，Pi 才有东西可推理。
     *       所以先发遥测，再等本帧指令。
     *========================================================================*/
    link_pi_poll();                       /* 先把已到的字节灌进状态机 */

    /* ③ 取 IMU 姿态 + 电池电压（P5 已接入）
     *    ⚠️ IMU 每周期都读（姿态是策略输入，100 Hz 要求）；
     *       电池是 10 Hz 节流，节流在 battery_poll 内部。 */
    imu_poll();
    battery_poll();

    /*==========================================================================
     * ④ 组并发 TEL_FAST（100 Hz）
     *========================================================================*/
    {
        tel_fast_src_t ts;
        memset(&ts, 0, sizeof(ts));

        for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
            /* ⚠️ 位置是【相对 q_default】的（协议 §4.4 注释 1）。
             *    q_default = 0，所以就是 q 本身；这里仍显式减去，
             *    以便将来 q_default 变化时不会静默错掉。 */
            ts.q_rad[i]     = s_js_valid[i] ? (s_js[i].q_rad - RLCA_Q_DEFAULT_RAD)
                                            : 0.0f;
            ts.qd_rad_s[i]  = s_js_valid[i] ? s_js[i].qd_rad_s : 0.0f;
            ts.tau_nm[i]    = s_js_valid[i] ? s_js[i].tau_nm   : 0.0f;
        }

        /* ---- IMU：姿态与角速度 ----
         * ⚠️ IMU 读不到时给"单位四元数 + 零角速度"，而不是全 0：
         *    全 0 不是合法四元数，Pi 侧反算重力会得到 NaN。
         *    Pi 侧靠 STAT_IMU_OK 判断数据是否可信（见 app_status_bits）。 */
        const imu_state_t *im = imu_state();
        if (imu_is_ok()) {
            ts.quat_wxyz[0] = im->quat_wxyz[0];
            ts.quat_wxyz[1] = im->quat_wxyz[1];
            ts.quat_wxyz[2] = im->quat_wxyz[2];
            ts.quat_wxyz[3] = im->quat_wxyz[3];
            ts.gyro_rad_s[0] = im->gyro_rad_s[0];
            ts.gyro_rad_s[1] = im->gyro_rad_s[1];
            ts.gyro_rad_s[2] = im->gyro_rad_s[2];
        } else {
            ts.quat_wxyz[0] = 1.0f;
        }

        rlca_phase_get_sincos_i16(&ts.gait_sin_i16, &ts.gait_cos_i16);

        ts.cycle_cnt  = (uint16_t)(s_cycle_cnt & 0xFFFFu);
        ts.voltage_mv = battery_is_valid() ? battery_mv() : 0u;
        ts.status     = app_status_bits(now);
        ts.err_flags  = servo_bus_err_flags();

        /* ⚠️ 只调用一次：link_pi 内部会「组一帧、发两路」
         *    （树莓派 + ESP32 镜像），保证两边收到的是同一份字节。
         *    这里若再单独组一次帧给 ESP32，SEQ 就会与 Pi 链路不一致。 */
        (void)link_pi_send_tel_fast(&ts);
    }

    /*==========================================================================
     * ④ 等本帧 CMD_JOINT（模型 A）或用上一帧（模型 B）
     *
     *  ⚠️⚠️ 链路不活时【不要等】—— 踩过的坑：
     *     `link_pi_wait_cmd()` 等不到帧时会结结实实地自旋满 4 ms。
     *     而 100 Hz 的时隙只有 10 ms，于是每周期变成 14 ms
     *     → 周期必然超时 → 状态灯 5 Hz 快闪，并直接违反
     *     RL 的 decimation=2 时序契约（实测就是这个现象）。
     *
     *     桌面调试（没接树莓派）时 100% 命中：`s_ever_rx` 一直是 0，
     *     永远不会收到 CMD_JOINT，那 4 ms 纯粹白等。
     *
     *  ⚠️ 判据用 `link_pi_is_alive(now)`（内部含 s_ever_rx）：
     *     从未收到过帧 → 不等；30 ms 内没收到 → 也不等。
     *     正常接 Pi 时第一帧到达后即进入"等"的分支，行为不变。
     *========================================================================*/
#if !RLCA_LINK_MODE_B
    if (link_pi_is_alive(now)) {
        if (link_pi_wait_cmd(RLCA_LINK_CMD_WAIT_US)) {
            s_last_cmd_ms = bsp_tick_ms();
            s_have_cmd    = true;
        }
    }
#endif

    /*==========================================================================
     * ⑤ 消费指令
     *========================================================================*/
    {
        pi_cmd_t c;
        if (link_pi_take_cmd_joint(&c)) {
            s_last_cmd_ms = now;
            s_have_cmd    = true;
            s_req_status  = ((c.flags & RLCA_FLAG_REQ_STATUS) != 0u);

            /* ⭐⭐ 上电"上锁"状态的解除条件（2026-10-16）
             *
             *  ⚠️ 规则（项目负责人明确要求）：
             *     在没有明确指令之前，【不准上力矩、不准写位置】。
             *
             *  ⇒ 这里判断"明确指令"：CMD_JOINT 带 ENABLE 标志。
             *     它语义上就是"我现在要求你接管并输出力矩"。
             *  ⇒ 只有收到它之后 s_armed 才置位；而 s_armed 是
             *     力矩使能 与 位置写 的共同前提（见 ⑥ 与 ⑦）。 */
            if ((c.flags & RLCA_FLAG_ENABLE) != 0u) {
                s_armed = true;
            }

            /* ⚠️ 收到明确指令 ⇒ 解除"手动力矩覆盖"，把力矩管控交回主循环。
             *    否则调试时敲过的 T 1 会让主循环永远不再卸力。 */
            app_clear_manual_torque();

            /* 模式与目标角都交给模式状态机（协议 §4.2） */
            link_pi_set_mode(c.mode);
            link_pi_set_pi_q_target(c.q_target);
            app_mode_request(c.mode);
        }
    }

    /* 模式切换请求（CMD_MODE）优先于 CMD_JOINT.mode */
    {
        pi_mode_t m;
        if (link_pi_take_cmd_mode(&m)) {
            link_pi_set_mode(m.mode);
            app_mode_request(m.mode);
            s_last_cmd_ms = now;
            s_have_cmd    = true;
        }
    }

    /* 标定请求（CMD_CALIB） */
    {
        pi_calib_t cb;
        if (link_pi_take_cmd_calib(&cb)) {
            s_last_cmd_ms = now;
            switch (cb.action) {
            case RLCA_CALIB_START:
                (void)app_calib_start();
                (void)link_pi_send_ack(cb.action, 0u);
                break;
            case RLCA_CALIB_CANCEL:
                calib_proc_cancel();
                app_mode_exit_calib();
                (void)link_pi_send_ack(cb.action, 0u);
                break;
            case RLCA_CALIB_MANUAL_ZERO:
                app_calib_manual_zero();
                (void)link_pi_send_ack(cb.action, 0u);
                break;
            case RLCA_CALIB_RESTORE_DEFAULT: {
#if (RLCA_CALIB_LOCK != 0)
                /* ⭐ 恢复默认会写入全零 offset 的默认表 ⇒ 等于"清空标定"。
                 *    锁住，避免一条误发的命令让机器人变成未标定。 */
                break;
#endif
                rlca_calib_t def;
                rlca_calib_load_defaults(&def);
                s_calib = def;
                /* ⭐ 同步给 servo_bus（它内部是复制一份表的） */
                servo_bus_set_calib(&s_calib);
                s_calib_dirty = true;

                /* ⚠️⚠️ 必须把 s_calib_ok 置 **false**（2026-10-17 修）
                 *
                 *  原来的实现是 `s_calib_ok = true`，理由是"默认表结构上合法"。
                 *  但**结构合法 ≠ 标定过**：
                 *      rlca_calib_load_defaults() 的 offset[] 全是 0，
                 *      而 offset 的物理含义是"URDF 零位下舵机的编码器读数"。
                 *      offset = 0 意味着"q=0 对应编码器 0"，
                 *      这对装在 2048 中位的舵机是**完全错**的。
                 *
                 *  现场后果（实测）：
                 *      在【标定姿态】（pos = 2048）下，固件上报
                 *          q = (2048 − 0) × dir × 2π/4096 = ±π = ±3.14 rad
                 *      而正确值应该是 q_calib = ±0.785 / ±1.571 rad。
                 *      于是 12 个关节角全部显示成 ±3.1 rad，且 STAT_CALIBRATED
                 *      还亮着 ✅ —— 假的"已标定"，比明确报"未标定"更危险。
                 *
                 *  ⇒ 语义修正：恢复默认表 = **放弃标定**，所以 s_calib_ok = false。
                 *     这样 STAT_CALIBRATED 会熄灭，ACTIVE 也会被第一道门挡住，
                 *     操作者一眼就知道"需要重新标定"。
                 *
                 *  ⚠️ 注意：默认表仍然会被落盘（s_calib_dirty = true）。
                 *     那是有意的 —— 让"恢复默认"在断电后仍然生效；
                 *     而 s_calib_ok 是靠 bsp_flash_calib_loaded() 之外的另一层
                 *     判断吗？不是 —— 重启后 bsp_flash_load_calib() 会成功，
                 *     s_calib_ok 又变 true。所以下面还要加"默认表识别"。 */
                s_calib_ok = false;
                (void)link_pi_send_ack(cb.action, 0u);
                break;
            }
            default:
                /* QUERY / SAVE：状态已经在 TEL_STATUS 里持续上报，无需额外动作 */
                (void)link_pi_send_ack(cb.action, 0u);
                break;
            }
        }
    }

    /*==========================================================================
     * ⑥ 安全评估 + 模式状态机 → 生成要下发的目标角
     *
     *  ⚠️ 顺序很重要：
     *    ① app_safety 先评估，并在急停时【就地推进 q_ref 的趴下斜坡】
     *    ② app_mode 再决定"用哪个 q_ref"（ESTOP 分支不会覆盖它）
     *    ③ 最后才限幅限速并下发
     *========================================================================*/
    const uint8_t link_level = link_pi_check_link(now);
    s_link_degraded = (link_level != 0u);

    /* Pi 的目标角（仅 ACTIVE 使用） */
    const float *pi_q = link_pi_pi_q_target();

    /* ⭐ 当前实测关节角（12 维）。
     *  未标定时 app_mode 用它当目标（"待在原地"），
     *  避免 q_ref=0 被换算成舵机 step 0 而撞机。
     *  ⚠️ 反馈无效的关节退化为 q_default，保持数组始终完整。 */
    float q_now[RLCA_N_JOINTS];
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        q_now[i] = s_js_valid[i] ? s_js[i].q_rad : RLCA_Q_DEFAULT_RAD;
    }

    /* 先进安全层：它在急停时会把 s_q_cmd 往趴下姿态推
     * ⚠️ 未标定（calib 表不合法）时它【不推进斜坡】，直接卸力矩，见实现里的短路 */
    app_safety_update(now, link_level, app_calib(), s_q_cmd,
                      servo_bus_max_temp(), servo_bus_err_flags());

    /* 再进模式状态机：输出"本周期最终要下发的 q_ref 与力矩状态" */
    bool want_torque = false;

    /* ⭐⭐ IMU 调试旁路（见 app_safety_set_imu_bypass 的说明）
     *  ⚠️ 必须是【两个条件同时成立】才放行 ACTIVE 门的第 ④ 条：
     *     · app_safety_blocks_active() 已经确认"IMU 是唯一阻塞原因"
     *       （其他原因如舵机报错/过温照旧挡住 ACTIVE 门第 ② 条）
     *     · 且人显式打开了旁路
     *  这里传 true 只是让 active_gate 的第 ④ 条过 —— 
     *  RLCA_SAFE_R_IMU_STALE 位本身不受影响，K/R 里照旧可见。 */
    const bool imu_for_gate =
        (imu_is_ok() || (app_safety_get_imu_bypass() && !app_safety_blocks_active()));

    app_mode_update(now, app_calib(),
                    link_pi_is_alive(now),
                    imu_for_gate,
                    s_calib_ok,
                    servo_bus_feedback_fresh(),
                    pi_q,
                    q_now,
                    s_q_cmd,
                    &want_torque);

    /*==========================================================================
     * ⑥ 力矩通断
     *
     *  ⭐⭐ 上锁规则（2026-10-16）：没有明确指令之前【不准上力矩】。
     *     ⇒ want_torque 只有在 s_armed 为真时才可能为 true。
     *
     *  ⚠️ 为什么必须加这一道：上电后安全层会因"链路失联 > 100 ms"进 ESTOP，
     *     而 ESTOP 分支在斜坡未完成时 torque_out = true
     *     （app_mode.c: case RLCA_MODE_ESTOP → *torque_out = !torque_off）。
     *     于是【一个从未被指令过的机器人】也会自动上力矩并开始趴下运动。
     *     这正是"上电就动"的一条来源。
     *========================================================================*/
    if (!s_armed) {
        want_torque = false;
    }

    /* ⭐⭐ 记录【实际下发】的力矩状态（2026-10-17 修上报错误）
     *
     *  ⚠️ 问题：`app_mode_update()` 里会把 `ms->torque_on` 设成它算出来的期望值，
     *     而上面那句 `if (!s_armed) want_torque = false;` 是【之后】才生效的。
     *     于是 `R` 里打印的 `torque_on` 是【覆盖前】的值 ——
     *     实测踩到过：R 显示 torque_on=1，但机器人一动不动，
     *     因为真正的 want_torque 被 s_armed 锁成了 false。
     *     ⇒ 上报值和实际值不一致，会让人误判"力矩是开的却没动"。
     *
     *  ⇒ 把最终值单独存一份，`R` 里两个都打印，一眼看出差在哪。 */
    s_torque_actual = want_torque;

    /* 力矩通断：只在【变化】时下发，避免每周期都发 12 条指令占总线 */
    {
        static bool last_torque = false;
        static bool torque_known = false;
        if (!torque_known || (want_torque != last_torque)) {
            servo_bus_set_all_torque(want_torque);
            last_torque  = want_torque;
            torque_known = true;
        }
    }

    /*==========================================================================
     * ⑥b ⭐ 上电兜底：反复重发 torque-off
     *
     *  ⚠️⚠️ 为什么需要这一段（已实测的撞机事故）：
     *     控制板适配器与 STM32 从同一路电源上电，适配器自身要启动。
     *     上电头几百毫秒内它【不理 UART】—— app_init 里那条 torque-off
     *     会石沉大海，而舵机仍在力矩使能状态，且目标位置寄存器里
     *     可能存着"0"（默认标定表算出来的），于是 12 个关节冲到底。
     *
     *     实测判据：断开 UART2 只给控制板上电 → 舵机不动；
     *               接上 UART2 一起上电 → 12 个关节冲到底。
     *
     *  ⚠️ 只在【上电后一小段时间】内重发（RLCA_TORQUE_OFF_BURST 个周期，
     *     约 100 ms 量级）—— 不是永久反复发，否则会一直占总线带宽，
     *     且会与正常控制指令抢总线。
     *
     *  ⚠️ 只在 want_torque == false 时重发：万一操作者已经主动使能力矩，
     *     这里绝不能把它压回去。
     *
     *  ⚠️⚠️ 2026-10-16 第二次修正 —— 从"限时补发"改成"持续重申"：
     *     原来只有 s_torque_burst_left 归零之前才补发（前 3 s）。
     *     现场证明这不够：上电后舵机【仍然】在一秒内开始持续运动，
     *     说明那些 torque-off 一条都没生效（适配器启动比窗口长，
     *     或者窗口刚好错过）。而 servo_bus_set_torque() 无条件返回 true，
     *     固件无从知道"到底关掉没有"，只能盲目重试。
     *
     *     ⇒ 现在：前 RLCA_TORQUE_OFF_BURST 个周期【每周期】重申（密集），
     *        之后降到每 RLCA_TORQUE_REASSERT_MS 重申一次（持续）。
     *
     *     ⭐ 为什么持续重申是安全的：
     *        · 写的是【力矩寄存器 40】，不是位置寄存器 42
     *          ⇒ 不改变目标位置，不会让机器人动
     *        · 对"已泄力"的舵机是幂等操作，无副作用
     *        · 对"仍使能"的舵机正是纠正动作（这就是我们要的）
     *        · 只在 want_torque == false 时发，不覆盖主动使能
     *      代价：5 Hz × 12 帧 = 60 帧/s ≈ 1 Mbps 的 0.6%，可忽略。
     *
     *  ⚠️⚠️ 例外：操作者显式"手动力矩覆盖"期间【暂停重申】（2026-10-16 增）
     *
     *     现场问题：用 W 命令调试时，必须【反复连发】才能让关节走到目标，
     *     中途会停。原因就是这里的重申 ——
     *       · servo_bus_set_all_torque(false) 内部会 bus_abort_now()，
     *         把你刚发出的位置帧【掐断】
     *       · 同时把力矩关掉，舵机立刻失力停在半路
     *     每 200 ms 打断一次，于是 W 一发的行程只有 200 ms 的量。
     *
     *     ⇒ 修法：给操作者一个显式覆盖开关。敲 'T 1' 时置位，敲 'T 0' 时清除。
     *        覆盖期间本段完全跳过（不发重申、不断总线）。
     *        ⚠️ 这不削弱安全性：覆盖只能由【人在 shell 上】打开，
     *           而且收到明确链路指令（CMD_JOINT）时会自动清除，回到正常管控。 */
    if (!want_torque && !s_manual_torque) {
        bool reassert = false;

        if (s_torque_burst_left > 0u) {
            s_torque_burst_left--;
            reassert = true;                       /* 上电头一段：每周期 */
        } else if ((uint32_t)(now - s_torque_reassert_ms) >= RLCA_TORQUE_REASSERT_MS) {
            s_torque_reassert_ms = now;
            reassert = true;                       /* 之后：低频持续 */
        }

        if (reassert) {
            servo_bus_set_all_torque(false);
            s_torque_reassert_count++;
        }
    }

    /*==========================================================================
     * ⑦ 合法化 → 发起舵机事务
     *
     *  ⭐⭐⭐ 唯一一条规则（2026-10-16 采纳项目负责人提出的方案）：
     *
     *      未标定 ⇒ 关节角度【没有意义】⇒ 一条位置指令都不发 + 持续卸力矩
     *      已标定 ⇒ 才允许写位置
     *
     *  ⚠️⚠️ 为什么必须这么严（前面几版都没抓住根本）：
     *     "播种 → 限幅 → 换算"整条链都在【角度域】里，而角度域的含义
     *     完全依赖标定表：
     *         q_rad → rlca_q_clamp（按标定表推出的限位夹）
     *               → rlca_q_to_step（按标定表的 offset/dir 换算）
     *     未标定时用的是默认虚表（offset=0、dir=+1），于是
     *         "待在原地"这个意图被换算成一个【固定的错步数】，
     *         12 个舵机同向、同速奔向它。
     *
     *     现场三条证据完全吻合：
     *       ① 拔掉 UART2 信号线 → 不动（收不到位置写）
     *       ② 接上 UART2         → 上电即动
     *       ③ 没接齐 12 个舵机   → 不动（播种门 n_valid==12 没过）
     *     ⇒ 动的指令是【位置写】，不是卸力帧。
     *
     *  ⚠️ 由此推论的正确设计：
     *     卸力矩不需要角度（写的是力矩寄存器），所以未标定时它安全；
     *     而写位置需要角度，所以未标定时它一定不安全。
     *     ⇒ 两件事必须分开对待，不能靠"播种"去修补。
     *
     *  ⚠️ 标定流程是例外：它【本来】就要驱动舵机（方向测试要挪 +98 步），
     *     app_calib_busy() 期间由 calib_proc 自己经 s_calib_hw->set_pos 下发，
     *     不经过这里。所以本函数在标定期间只需继续跑【只读】事务。
     *
     *  ⚠️ 标定期间必须继续跑只读事务：标定流程的 get_pos() 读的是
     *     servo_bus_feedback()，而那份反馈是由同步读事务刷新的。
     *     如果标定期间完全不发起事务，反馈永远是旧值，标定第一步就报
     *     READ_POS 失败 —— 这个坑是代码复核时发现的。 */
    const bool servo_gate_open = ((int32_t)(now - s_servo_gate_ms) >= 0);

    /* ⭐ 位置写许可：已标定 且 收到明确指令 且 上电闸门已开 且 不在标定中
     *  ⚠️ s_armed 是 2026-10-16 加的"上锁"：没有明确指令之前不写位置。 */
    const bool pos_write_allowed = (s_calib_ok && s_armed &&
                                    servo_gate_open && !app_calib_busy());

    if (!pos_write_allowed) {
        /* 未标定 / 闸门未开 / 标定中：只读，刷新反馈，绝不写位置 */
        s_pos_write_blocked_count++;
        if (servo_bus_is_idle()) {
            (void)servo_bus_start_read_only();
        }
    } else {
        /* ⭐⭐ 已标定后的第一次写位置：用【闸门期读到的真实位置】播种。
         *
         *  ⚠️ 此时标定表【合法】，所以角度域往返无损：
         *        位置 → q_rad → 夹软限位 → 换回步数 → 同一步数
         *     ⇒ "待在原地"这个意图成立。
         *
         *     （未标定时这一步是错的，所以整段被上移到 pos_write_allowed 后面。）
         *
         *  ⚠️ 只在【全部 12 个】反馈都有效时播种：只播一半会让未播的那些跳变。
         *     若闸门期内没读全，就【继续只读】—— 宁可不动作，也不要盲目发位置。 */
        if (!s_q_seeded) {
            const rlca_joint_state_t *js = servo_bus_joint_states();
            uint8_t n_valid = 0u;
            for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
                if (js[i].valid != 0u) {
                    n_valid++;
                }
            }
            if (n_valid == (uint8_t)RLCA_N_JOINTS) {
                for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
                    s_q_limited[i] = js[i].q_rad;
                    s_q_cmd[i]     = js[i].q_rad;
                }
                s_q_seeded = true;
            }
        }

        if (!s_q_seeded) {
            /* 反馈不全：继续只读，本周期不发位置 */
            if (servo_bus_is_idle()) {
                (void)servo_bus_start_read_only();
            }
        } else {
            link_limits_apply(app_calib(), s_q_cmd, s_q_limited, s_q_limited);

            if (servo_bus_is_idle()) {
                (void)servo_bus_start_cycle(s_q_limited);
            }
        }
    }

    /*==========================================================================
     * ⑧ TEL_STATUS：10 Hz + 事件触发
     *========================================================================*/
    if (s_req_status || ((now - s_status_last_ms) >= 100u)) {
        s_status_last_ms = now;
        s_req_status     = false;

        tel_status_src_t ss;
        memset(&ss, 0, sizeof(ss));
        ss.uptime_ms   = now;
        ss.voltage_mv  = 0u;               /* P5 接入电池 ADC */
        ss.rx_overflow = (uint16_t)(g_uart_pi.rx_overflow & 0xFFFFu);
        ss.bus_err     = (uint16_t)(servo_bus_stats()->n_crc_err & 0xFFFFu);
        ss.bus_timeout = (uint16_t)(servo_bus_stats()->n_timeout & 0xFFFFu);
        ss.mode        = link_pi_mode();
        ss.status      = app_status_bits(now);
        ss.err_flags   = servo_bus_err_flags();
        ss.seq_lost    = (uint8_t)(link_pi_stats()->seq_lost & 0xFFu);

        /* 12 个舵机温度（0.1 °C）。未标定的 0 值也照发，Pi 侧自行判断。 */
        static uint8_t temps[RLCA_N_JOINTS];
        const servo_fb_t *f = servo_bus_feedback();
        for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
            temps[i] = f[i].temp_c;
        }
        ss.temp_deci_c = temps;

        (void)link_pi_send_tel_status(&ss);
    }

    /*==========================================================================
     * ⑨ 相位时钟：与训练侧一致，每控制周期推进 0.03 圈
     *========================================================================*/
    (void)rlca_phase_advance();

#if RLCA_DEBUG_PERF
    const uint32_t dt = bsp_us_since(t0);

    /* ⚠️⚠️ 夹掉"不可能的"样本（2026-10-17 加）
     *
     *  现场实测（t 命令）：
     *      last_us = 3183            ← 真实值，3.2 ms ✅
     *      max_us  = 4235318156      ← 42 亿微秒 = 71 分钟！纯垃圾
     *      avg_us  = 487403          ← 被那一个垃圾样本摊成 487 ms
     *
     *  成因：`s_t0_us` 在每周期开头设，而 DWT 在 72 MHz 下约 59.6 秒回绕。
     *        若某一周期因为异常（长阻塞、调试器停住、从没走到这里）
     *        隔了很久才取到一个样本，dt 就是一个巨大的陈旧值。
     *        `max_us` / `avg_us` 被它一污染，就再也不会恢复 —— 因为 max 只增不减。
     *
     *  ⇒ 超过 1 秒的样本一律丢弃（正常周期是 3~10 ms，1 秒已是 100 倍余量）。
     *     ⚠️ 这只影响【显示统计】，不影响任何控制逻辑。
     *     `overruns` 仍然照实计数，不受影响。 */
    const uint32_t dt_ok = (dt <= 1000000u) ? dt : 0u;

    if (dt_ok != 0u) {
        s_stats.last_us = dt_ok;
        s_stats.sum_us += dt_ok;
        s_stats.n++;
        if (dt_ok > s_stats.max_us) {
            s_stats.max_us = dt_ok;
        }
    }
    /* ⚠️ overruns 用【原始 dt】判：真的超了就是超了，不因为"夹取"而漏记。
     *    但显然不可能"超 1 秒" —— 那是计时器伪影，不是真的跑了一秒。 */
    if (dt_ok > s_stats.step_max_us[APP_STEP_CYCLE]) {
        s_stats.step_max_us[APP_STEP_CYCLE] = dt_ok;
    }
    if ((dt_ok != 0u) && (dt_ok > (RLCA_CTRL_PERIOD_MS * 1000u))) {
        s_stats.overruns++;
        /* ⚠️ 周期超时是最危险的信号：说明某一环阻塞了。
         *    用状态灯 5 Hz 快闪把它暴露出来，而不是静默地掉帧。
         *    这直接关系到 RL 的 decimation=2 时序契约。 */
        s_stats.healthy_run = 0u;
        bsp_led_set(BSP_LED_FAST_BLINK);
    } else if (bsp_led_mode() == BSP_LED_FAST_BLINK) {
        /* ⭐ 自愈：连续若干个周期都达标后，把超时指示清掉。
         *
         * ⚠️ 为什么需要这一条（踩过的坑）：`bsp_led_set()` 没有恢复路径，
         *    一旦某次超时点亮了快闪，就【永久】停在快闪 ——
         *    于是"曾经超过一次"和"现在仍在超时"看起来完全一样，
         *    指示灯失去了"当前是否健康"的含义。
         *
         * ⚠️ 只清 FAST_BLINK，不碰其它模式：业务状态（急停图案等）
         *    由 app 逻辑控制，不能被这里覆盖。
         *
         * 取 100 个周期（约 1 秒）作为"确实恢复了"的门槛，
         * 避免在临界点上反复切换闪烁。 */
        s_stats.healthy_run++;
        if (s_stats.healthy_run >= 100u) {
            bsp_led_set(BSP_LED_OFF);
        }
    } else {
        s_stats.healthy_run = 0u;
    }
#endif
}

void app_housekeeping(void)
{
    const uint32_t hk_t0 = bsp_tick_us();
    /* ⚠️ 标定落盘必须在【非实时路径】做：
     *    Flash 擦写会让 CPU 停 20~40 ms，放 100 Hz 周期里必然掉帧。
     *    这里只是"标定完成后写一次"，属于低频事件。 */
#if (RLCA_CALIB_GEOMETRIC_EN != 0)
    /* ⭐ 几何表模式：标定流程已锁（RLCA_CALIB_LOCK），不会有新表产生，
     *    ⇒ 这里【完全不碰 flash】。
     *    ⚠️ 这同时消除了一整类故障：擦写失败 / 写一半断电 ⇒ 下次启动
     *       读到半页 ⇒ CRC 不过 ⇒ "未标定"。现在启动根本不读 flash。 */
    s_calib_dirty = false;
#else
    {
        rlca_calib_t done;
        if (app_calib_take_result(&done)) {
            /* ⚠️⚠️ 修（2026-10-18）：`s_calib_ok` 曾经被捆在**落盘成功**上
             *
             *  旧代码：`s_calib_ok = bsp_flash_save_calib(&done)`
             *  ⇒ flash 擦写偶发失败时，RAM 里明明是一张合法表，
             *    机器人却被判成"未标定"（ACTIVE 直接被拒）。
             *    这正是现场"重启有概率变成未标定"的来源之一。
             *
             *  ⭐ 正确语义：`s_calib_ok` 只表示"表本身是否合法"；
             *     落盘成功与否是**独立告警**（R 里 calib_dirty）。
             *     落盘失败 ⇒ 保留 dirty 让下轮重试，但**不降级**。 */
            const bool saved = bsp_flash_save_calib(&done);
            s_calib          = done;
            s_calib_ok       = rlca_calib_is_valid(&s_calib);
            s_calib_dirty    = !saved;
        }
    }
    if (s_calib_dirty) {
        s_calib_dirty = false;
        if (!bsp_flash_save_calib(&s_calib)) {
            s_calib_dirty = true;        /* 保留标志，下一轮再试 */
        }
        /* ⚠️ 只看表是否合法，不看落盘结果 */
        s_calib_ok = rlca_calib_is_valid(&s_calib);
    }
#endif /* RLCA_CALIB_GEOMETRIC_EN */
    app_step_record(APP_STEP_HOUSEKEEP, hk_t0);

    /* 目前没有额外的低频任务：心跳与状态灯在 app_poll() 里推进。
     * 保留此入口，P4/P6 会在这里加：
     *   · TEL_STATUS 10 Hz 上报
     *   · Flash 标定的延迟保存
     *   · 统计汇总 */
}

/*==============================================================================
 * 查询
 *============================================================================*/

uint32_t app_cycle_count(void)
{
    return s_cycle_cnt;
}

uint32_t app_last_cycle_us(void)
{
    return s_stats.last_us;
}

const app_cycle_stats_t *app_cycle_stats(void)
{
    return &s_stats;
}

static const rlca_calib_t *app_calib(void)
{
    return &s_calib;
}

/*==============================================================================
 * 100 Hz 调度（由 main.c 调用，因为它要跟 while(1) 绑在一起）
 *============================================================================*/

/** 返回 true = 本圈应该执行一次 100 Hz 任务。
 *  ⚠️ 用累加式基准，避免单次超时把后续节拍整体推后。 */
bool app_should_run_cycle(void)
{
    const uint32_t now = bsp_tick_ms();
    if ((int32_t)(now - s_next_cycle_ms) >= 0) {
        s_next_cycle_ms += RLCA_CTRL_PERIOD_MS;

        /* 若落后超过一个周期（例如 Flash 擦写把 CPU 停了 30 ms），
         * 直接对齐到当前时刻，避免追赶式连发。 */
        if ((int32_t)(now - s_next_cycle_ms) >= (int32_t)RLCA_CTRL_PERIOD_MS) {
            s_next_cycle_ms = now + RLCA_CTRL_PERIOD_MS;
        }
        return true;
    }
    return false;
}
