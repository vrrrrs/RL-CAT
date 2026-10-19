/**
  ******************************************************************************
  * @file    rlca_calib_proc.c
  * @brief   关节零点标定流程状态机实现。
  *
  *  ⚠️ 全程非阻塞：最重的动作是"发一条单关节位置指令"（约 8 字节 @1 Mbps）。
  *     所以标定可以在主循环里跑，不影响 100 Hz 控制环。
  *
  *  ⚠️ 标定过程中【不下发 100 Hz SyncWrite】—— 由上层（app_mode）在 CALIB
  *     状态里跳过正常控制环。否则两条路径会互相覆盖目标角。
  ******************************************************************************
  */

#include <string.h>

#include "rlca_calib_proc.h"

/*==============================================================================
 * 时序常量
 *============================================================================*/

/** 卸力矩后等机器人沉下去的时长（固件设计书 §6.2：1~2 s） */
#define CAL_SETTLE_MS        1500u

/** 每轮读取之间的间隔（给舵机位置更新时间；舵机最快 1 ms 更新一次） */
#define CAL_SAMPLE_GAP_MS    20u

/** 采样轮数（取平均压抖动） */
#define CAL_SAMPLE_ROUNDS    8u

/* ⚠️ 方向测试相关常量（CAL_DIR_SETTLE_MS / CAL_RETURN_SETTLE_MS /
 *    CAL_DIR_DELTA_STEP / CAL_DIR_MIN_MOVE）与 enter_per_joint() 均已删除
 *    （2026-10-16）：方向改由 RLCA_JOINT_DIR_INIT 给出，
 *    标定全程不再动任何关节。 */

/*==============================================================================
 * 内部
 *============================================================================*/

static const rlca_calib_hw_t *s_hw = NULL;
static calib_proc_t s_p;

static uint32_t now_ms(void)
{
    return (s_hw != NULL) ? s_hw->millis() : 0u;
}

static void set_state(calib_state_t st)
{
    s_p.state = st;
    s_p.state_t0 = now_ms();
}

static void fail(calib_err_t e)
{
    s_p.err = e;
    set_state(CAL_FAILED);
}

/*==============================================================================
 * 纯逻辑
 *============================================================================*/

int8_t calib_dir_from_delta(int16_t delta_step, int16_t min_move)
{
    if (min_move <= 0) {
        min_move = 1;
    }
    if (delta_step >= min_move) {
        return 1;
    }
    if (delta_step <= -min_move) {
        return -1;
    }
    return 0;       /* 没动或方向不明 */
}

calib_err_t calib_validate_result(const rlca_calib_t *c)
{
    if (c == NULL) {
        return CAL_ERR_LIMIT;
    }

    /* ⚠️ 先做【具体】检查，最后才做整体合法性检查。
     *    最初把 rlca_calib_is_valid() 放在最前面，它会因为 dir 非法而返回
     *    false，于是一律报 CAL_ERR_LIMIT，"方向不明"这个更具体的原因
     *    被盖掉了 —— 上机排查时看不出来到底是哪一条不满足。 */
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        /* dir 必须是 ±1 */
        if ((c->dir[i] != 1) && (c->dir[i] != -1)) {
            return CAL_ERR_DIR_AMBIGUOUS;
        }
        /* offset 必须落在合法编码器范围内。
         * ⚠️ 用与 rlca_calib_is_valid 同一个判据，不要两处各写一遍 ——
         *    最初这里写成 `offset > 4096`，对负值恒为真，负的超范围
         *    offset 会漏检（单测抓到的）。 */
        if (!rlca_calib_offset_ok(c->offset[i])) {
            return CAL_ERR_LIMIT;
        }
    }

    /* 整体结构（magic / version / 限位区间） */
    if (!rlca_calib_is_valid(c)) {
        return CAL_ERR_LIMIT;
    }
    return CAL_ERR_NONE;
}

/*==============================================================================
 * 生命周期
 *============================================================================*/

void calib_proc_init(const rlca_calib_hw_t *hw)
{
    s_hw = hw;
    memset(&s_p, 0, sizeof(s_p));
    s_p.state = CAL_IDLE;
    s_p.err   = CAL_ERR_NONE;
}

void calib_proc_start(const rlca_calib_t *current)
{
    if (s_hw == NULL) {
        return;
    }

    /* 保留限位（来自 URDF，不是标定出来的）；其余字段清空重算 */
    rlca_calib_t seed;
    if (current != NULL) {
        seed = *current;
    } else {
        rlca_calib_load_defaults(&seed);
    }

    memset(&s_p, 0, sizeof(s_p));
    s_p.result = seed;
    s_p.err    = CAL_ERR_NONE;

    /* ⭐ 方向直接取自配置表（RLCA_JOINT_DIR_INIT），不再测量。
     *  ⚠️ 原来的"微动 +98 步测方向"已删除，见 do_sample() 的说明。 */
    static const int8_t s_dir_init[RLCA_N_JOINTS] = RLCA_JOINT_DIR_INIT;
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        s_p.result.offset[i] = 0;
        s_p.result.dir[i]    = s_dir_init[i];
    }
    /* limit_* 保持不变（seed 里带着） */

    set_state(CAL_UNLOAD);
}

void calib_proc_cancel(void)
{
    memset(&s_p, 0, sizeof(s_p));
    s_p.state = CAL_IDLE;
    s_p.err   = CAL_ERR_NONE;
}

/*==============================================================================
 * 状态机
 *============================================================================*/

static void do_unload(void)
{
    s_hw->set_torque_all(false);
    /* 给"卸力矩指令"一点时间落下去，再进 SETTLE */
    set_state(CAL_SETTLE);
}

static void do_settle(void)
{
    if ((now_ms() - s_p.state_t0) >= CAL_SETTLE_MS) {
        /* 进入采样前清空累加器 */
        for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
            s_p.accum[i] = 0;
        }
        s_p.n_samples = 0u;
        s_p.round     = 0u;
        s_p.joint     = 0u;
        set_state(CAL_SAMPLE);
    }
}

static void do_sample(void)
{
    /* 每 CAL_SAMPLE_GAP_MS 采一个关节，轮流采完 12 个算一轮 */
    if ((now_ms() - s_p.state_t0) < CAL_SAMPLE_GAP_MS) {
        return;
    }
    s_p.state_t0 = now_ms();

    s_p.n_sample_calls++;                    /* ⭐ 诊断 */

    int16_t pos = 0;
    if (!s_hw->get_pos(s_p.joint, &pos)) {
        fail(CAL_ERR_READ_POS);
        return;
    }

    /* ⚠️⚠️ 累加本次读数（所有轮次一视同仁，不做特殊分支）。
     *   第 8 轮结束时 accum 里正好是 8 个值，除以 8 即得平均。 */
    s_p.accum[s_p.joint] += (int32_t)pos;

    s_p.joint++;
    if (s_p.joint < RLCA_N_JOINTS) {
        return;                               /* 本轮还没采完 */
    }

    /* 一轮采完 */
    s_p.joint = 0u;
    s_p.round++;

    if (s_p.round < CAL_SAMPLE_ROUNDS) {
        return;                               /* 还要继续采 */
    }

    /* ⭐⭐⭐ 全部轮次采完：一次性求平均，然后立刻进 FINAL_READ。
     *
     *  ⚠️ 这里的关键是"accum 里恰好是 CAL_SAMPLE_ROUNDS 个值，不多不少"。
     *     原实现把"累加"放在判轮数之前，于是第 9 次读数也进了 accum，
     *     而 base 只用前 8 个算 ⇒ accum 被污染；CAL_FINAL_READ 再拿
     *     accum/8 当 offset ⇒ offset 偏大约一个读数（实测 1794 步）
     *     ⇒ 标定完一进闭环就大幅运动。
     *     原代码里这个缺陷被"方向测试"掩盖（DIR 阶段会清 accum），
     *     删掉方向测试后才暴露。 */
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        s_p.base[i] = (int16_t)(s_p.accum[i] / (int32_t)CAL_SAMPLE_ROUNDS);
    }
    s_p.n_samples = CAL_SAMPLE_ROUNDS;

    /* 清空，让下一阶段（FINAL_READ）从干净状态开始 */
    for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
        s_p.accum[i] = 0;
    }
    s_p.round = 0u;

    set_state(CAL_FINAL_READ);
}

/* ⚠️ do_dir_phase() / do_dir_settle() 已于 2026-10-16 删除。
 *
 *  删除原因（两层）：
 *    ① 方向已由 RLCA_JOINT_DIR_INIT 给出（项目负责人实测），不需要再测。
 *    ② 那个测试本身是撞机源：它要先 s_hw->set_torque_all(true)，
 *       而那一刻另外 11 个舵机的 GOAL_POSITION 里还是它们自己存的旧值
 *       （舵机把目标存在非易失存储里）⇒ 一起奔向陈旧目标 ⇒ 猛冲。
 *       而且它依赖"机器人自己稳得住"，实际标定时机器人是自然下垂的，
 *       腿会跟着重力动 ⇒ 读数不可靠 ⇒ 随机报 NO_MOTION。
 *
 *  ⇒ 标定现在【全程不上力矩、不动任何关节】，只采样 12 个位置。
 */

/** 再读一轮位置 → 这就是 offset 的依据
 *
 *  ⚠️ 自己负责清空累加器：不要依赖"上一阶段留下的干净 accum"。
 *     标定姿态就是装配姿态（q_calib = 0），所以这一轮读数直接就是 offset。 */
static void do_final_read(void)
{
    if ((now_ms() - s_p.state_t0) < CAL_SAMPLE_GAP_MS) {
        return;
    }
    s_p.state_t0 = now_ms();

    /* ⚠️ 累加器已由 do_sample() 的收尾清空，这里直接用。 */

    int16_t pos = 0;
    if (!s_hw->get_pos(s_p.joint, &pos)) {
        fail(CAL_ERR_READ_POS);
        return;
    }
    s_p.accum[s_p.joint] += (int32_t)pos;

    s_p.joint++;
    if (s_p.joint >= RLCA_N_JOINTS) {
        s_p.joint = 0u;
        s_p.round++;
        if (s_p.round >= CAL_SAMPLE_ROUNDS) {
            /* ⚠️⚠️ offset 的定义是"q = 0（站姿）时的编码器读数"，
             *      而这里读到的是【标定姿态】下的读数 step_cal。
             *      标定姿态对应的关节角是 q_calib（RLCA_CALIB_POSE_Q）。
             *
             *      由  step = dir × q × STEPS_PER_RAD + offset
             *      ⇒    offset = step_cal − dir × q_calib × STEPS_PER_RAD
             *
             *  ⚠️ 早期实现直接把 step_cal 当作 offset —— 那等于强行认定
             *     "标定姿态 = q 0"。在站姿标定时这个巧合成立（q_calib = 0），
             *     换成"腿伸直"姿态后就不成立了，必须反推。
             *
             *  ⚠️ 用【最终读回的位置】作为 offset 的依据。
             */
            static const float s_q_calib[RLCA_N_JOINTS] = RLCA_CALIB_POSE_Q;

            for (uint8_t i = 0u; i < RLCA_N_JOINTS; ++i) {
                const float step_cal =
                    (float)(s_p.accum[i] / (int32_t)CAL_SAMPLE_ROUNDS);
                const int32_t dir  = (int32_t)s_p.result.dir[i];
                const int32_t dq   = (int32_t)(dir * s_q_calib[i] *
                                               RLCA_SERVO_STEPS_PER_RAD);
                int32_t off = (int32_t)((step_cal >= 0.0f)
                                        ? (step_cal + 0.5f)
                                        : (step_cal - 0.5f)) - dq;

                /* ⚠️⚠️ 环绕到 [0, 4095]（2026-10-12）
                 *
                 *  offset 是编码器步数，而编码器是【单圈 4096 步】的绝对位置，
                 *  读数恒在 0..4095。原实现直接就把它当 offset，于是"读数是 0..4095"
                 *  自动成立。
                 *
                 *  改成"标定姿态 ≠ 站姿"之后，offset 变成了【截距】：
                 *      offset = step_cal − dir × q_calib × SPR
                 *  它不再受 0..4095 约束。例如 calf（q_calib = −90°）：
                 *      Δ = −1024  ⇒  offset ∈ [1024, 5119]
                 *  一旦站姿读数 > 3071，offset 就超过 rlca_calib_offset_ok() 的
                 *  ±4096 上限 ⇒ 标定报 CAL_ERR_LIMIT 失败。
                 *
                 *  而 4096 步 = 一圈，所以 step 与 step ± 4096 物理上是同一个位置：
                 *      step = dir×q×SPR + (offset ± 4096)  ⇒  转成编码器位置时环绕掉
                 *  ⇒ 把 offset 环绕回 [0, 4095] 完全等价，且落回合法范围。
                 *
                 *  ⚠️ 注意 use 端必须容忍环绕：offset 在 [0,4095] 而 dir×q×SPR
                 *     可以很大，算出的 step 会超出 [0,4095]，由 rlca_sts 编码时环绕。
                 *     单测 test_calib_offset_backs_out_from_calib_pose 验证了往返一致。 */
                off %= 4096;
                if (off < 0) { off += 4096; }

                s_p.result.offset[i] = (int16_t)off;
            }
            set_state(CAL_VERIFY);
        }
    }
}

static void do_verify(void)
{
    /* 把方向也写进去（dir 已在 TEST 阶段填好） */
    s_p.result.magic   = RLCA_CALIB_MAGIC;
    s_p.result.version = RLCA_CALIB_VERSION;
    s_p.result.crc16   = rlca_calib_crc(&s_p.result);

    const calib_err_t e = calib_validate_result(&s_p.result);
    if (e != CAL_ERR_NONE) {
        fail(e);
        return;
    }
    /* 标定完把力矩卸掉，避免机器人保持一个别扭的姿态 */
    s_hw->set_torque_all(false);
    set_state(CAL_DONE);
}

void calib_proc_poll(void)
{
    if (s_hw == NULL) {
        return;
    }

    switch (s_p.state) {
    case CAL_IDLE:
    case CAL_DONE:
    case CAL_FAILED:
        return;

    case CAL_UNLOAD:     do_unload();      return;
    case CAL_SETTLE:     do_settle();      return;
    case CAL_SAMPLE:     do_sample();      return;
    /* ⚠️ CAL_DIR_TEST / CAL_DIR_SETTLE 已删除（2026-10-16）：
     *    方向改由 RLCA_JOINT_DIR_INIT 给出，标定全程不再动任何关节。
     *    保留枚举值只是为了不破坏协议/上报里的数字编号。 */
    case CAL_FINAL_READ: do_final_read();  return;
    case CAL_VERIFY:     do_verify();      return;

    default:
        set_state(CAL_IDLE);
        return;
    }
}

/*==============================================================================
 * 查询
 *============================================================================*/

calib_state_t calib_proc_state(void)
{
    return s_p.state;
}

const calib_proc_t *calib_proc_info(void)
{
    return &s_p;
}

bool calib_proc_take_done(rlca_calib_t *out)
{
    if (s_p.state != CAL_DONE) {
        return false;
    }
    if (s_p.result_taken) {
        return false;
    }
    s_p.result_taken = true;
    if (out != NULL) {
        *out = s_p.result;
    }
    return true;
}

bool calib_proc_failed(void)
{
    return (s_p.state == CAL_FAILED);
}

calib_err_t calib_proc_err(void)
{
    return s_p.err;
}

uint8_t calib_proc_progress(void)
{
    /* 粗略但单调：SETTLE 20%，SAMPLE 40%，DIR 40~90%，FINAL/VERIFY 100% */
    switch (s_p.state) {
    case CAL_IDLE:        return 0u;
    case CAL_UNLOAD:      return 5u;
    case CAL_SETTLE:      return 20u;
    case CAL_SAMPLE:      return (uint8_t)(20u + (s_p.round * 20u / CAL_SAMPLE_ROUNDS));
    case CAL_DIR_TEST:    return (uint8_t)(40u + (s_p.joint * 45u / RLCA_N_JOINTS));
    case CAL_DIR_SETTLE:  return 87u;
    case CAL_FINAL_READ:  return (uint8_t)(90u + (s_p.round * 8u / CAL_SAMPLE_ROUNDS));
    case CAL_VERIFY:      return 99u;
    case CAL_DONE:        return 100u;
    default:              return 0u;
    }
}
