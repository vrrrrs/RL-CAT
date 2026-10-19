/**
  ******************************************************************************
  * @file    imu.c
  * @brief   JY901P 读取、解析与四元数换算。
  *
  *  ⚠️ 欧拉角 → 四元数（ZYX 顺序）：
  *     WIT 的 I2C 角度寄存器给的是欧拉角，不是四元数。
  *     设 roll = φ（绕 X）、pitch = θ（绕 Y）、yaw = ψ（绕 Z），
  *     采用 ZYX 内旋（= R = Rz(ψ)·Ry(θ)·Rx(φ)），则
  *       w = cos(φ/2)cos(θ/2)cos(ψ/2) + sin(φ/2)sin(θ/2)sin(ψ/2)
  *       x = sin(φ/2)cos(θ/2)cos(ψ/2) − cos(φ/2)sin(θ/2)sin(ψ/2)
  *       y = cos(φ/2)sin(θ/2)cos(ψ/2) + sin(φ/2)cos(θ/2)sin(ψ/2)
  *       z = cos(φ/2)cos(θ/2)sin(ψ/2) − sin(φ/2)sin(θ/2)cos(ψ/2)
  *     ⚠️ 这个顺序【必须与 Isaac Gym 的坐标约定一致】，否则
  *        重力向量的方向会错，策略输入就是错的。
  *        上机验收单里有一条专门验证：机身水平时 gz ≈ −1。
  ******************************************************************************
  */

#include <math.h>
#include <string.h>

#include "imu.h"

/*==============================================================================
 * 纯逻辑
 *============================================================================*/

int16_t imu_i16(const uint8_t *p)
{
    /* ⚠️⚠️ 小端（低字节在前）—— 这是【上机实测】的结论，不是推断。
     *
     * 早期版本按【大端】解析，注释还写着"WIT 模块用大端"，
     * 那是从"WIT 串口协议用大端"外推到 I2C 寄存器的 —— 错的。
     * 手册通篇没写字节序，必须实测。
     *
     * 实测证据（I 命令 dump 的原始字节）：
     *   ACC  = 05 FF 84 00 F2 07
     *     小端 -> (-0.123, +0.064, +0.993) g, |a| = 1.003 g   ← 静止时应 ~1 g
     *     大端 -> (+0.750, -15.500, -1.747) g, |a| = 15.6 g   ← 离谱
     *   QUAT / ANGLE 两组在小端下推出的重力方向与加速度完全一致（maxErr 0.000），
     *   而大端下怎么排列都对不上。
     *   另：RSW=0x1E、RRATE=0x06、BANDWIDTH=0x04 三个默认值也只在【小端】下吻合手册。
     *
     * ⚠️ 这条结论有一个额外好处：I2C 侧与舵机总线、协议帧一样都是小端，
     *    全项目只有一种字节序，少了一类坑。 */
    return (int16_t)(((uint16_t)p[1] << 8) | (uint16_t)p[0]);
}

float imu_raw_to_fs(int16_t raw, float fs)
{
    return ((float)raw / RLCA_IMU_INT16_FS) * fs;
}

float imu_gyro_to_rad_s(int16_t raw)
{
    /* WIT 给的是 °/s → 先换算成物理量，再转 rad/s */
    const float dps = imu_raw_to_fs(raw, RLCA_IMU_GYRO_FS_DPS);
    return dps * (3.14159265359f / 180.0f);
}

float imu_accel_to_g(int16_t raw)
{
    return imu_raw_to_fs(raw, RLCA_IMU_ACC_FS_G);
}

float imu_angle_to_rad(int16_t raw)
{
    /* ⭐ 依据手册（reference/JY901P_I2C协议.md）：
     *      角度 = raw / 32768 × 180°      →  1 LSB ≈ 0.0054932°
     *   即 int16 满量程对应 ±180°。
     *
     * ⚠️⚠️ 早期版本写成 `raw * 0.01f`（"0.01°/LSB"）—— 那是【串口协议】
     *     的格式，I2C 寄存器不是。错了 180/32768/0.01 = 1.831 倍，
     *     90° 会被算成 164°。这个错误只有查手册才能发现。 */
    return imu_raw_to_fs(raw, RLCA_IMU_ANGLE_FS_DEG) * (3.14159265359f / 180.0f);
}

void imu_parse_angle_to_quat(const uint8_t *buf, float out_wxyz[4], float euler_out[3])
{
    if ((buf == NULL) || (out_wxyz == NULL)) {
        return;
    }

    const float roll  = imu_angle_to_rad(imu_i16(&buf[0]));
    const float pitch = imu_angle_to_rad(imu_i16(&buf[2]));
    const float yaw   = imu_angle_to_rad(imu_i16(&buf[4]));

    const float hr = roll  * 0.5f;
    const float hp = pitch * 0.5f;
    const float hy = yaw   * 0.5f;

    const float cr = cosf(hr);
    const float sr = sinf(hr);
    const float cp = cosf(hp);
    const float sp = sinf(hp);
    const float cy = cosf(hy);
    const float sy = sinf(hy);

    float w = (cr * cp * cy) + (sr * sp * sy);
    float x = (sr * cp * cy) - (cr * sp * sy);
    float y = (cr * sp * cy) + (sr * cp * sy);
    float z = (cr * cp * sy) - (sr * sp * cy);

    /* 单位化（防止数值漂移累积） */
    const float n = sqrtf((w * w) + (x * x) + (y * y) + (z * z));
    if (n > 1e-6f) {
        const float inv = 1.0f / n;
        w *= inv; x *= inv; y *= inv; z *= inv;
    } else {
        w = 1.0f; x = 0.0f; y = 0.0f; z = 0.0f;
    }

    out_wxyz[0] = w;
    out_wxyz[1] = x;
    out_wxyz[2] = y;
    out_wxyz[3] = z;

    if (euler_out != NULL) {
        euler_out[0] = roll;
        euler_out[1] = pitch;
        euler_out[2] = yaw;
    }
}

/*==============================================================================
 * 有状态层
 *============================================================================*/

static const imu_port_t *s_port = NULL;
static imu_state_t s_st;

/* 上一帧角速度（离群值保护用） */
static bool  s_have_prev_gyro = false;
static float s_prev_gyro[3];

/** ⭐ VERSION 重试的下次允许时刻（见 imu_poll 里的说明）。
 *  ⚠️ 上电瞬间 JY901P 可能还没启动完，imu_selftest() 那一发会失败；
 *     之后靠这里低频补读，避免 version 恒 0。 */
static uint32_t s_version_try_ms = 0u;

/** 单帧角速度变化上限（rad/s）。
 *  ⚠️ 依据：舵机空载 4.71 rad/s、URDF velocity 上限 8 rad/s。
 *     真实机身的角速度不该在 10 ms 内跳变超过这个量；
 *     一旦超过，几乎一定是 I2C 读串了字节（读到半帧/错位），
 *     而不是机器人真的翻转了。宁可丢一帧，也不要把垃圾喂给策略。 */
#define IMU_GYRO_SPIKE_LIMIT_RAD_S  20.0f

static void mark_stale_if_needed(uint32_t now){
    if ((s_st.valid != 0u) && (s_st.last_update_ms != 0u)) {
        if ((now - s_st.last_update_ms) > RLCA_IMU_STALE_MS) {
            s_st.valid = 0u;
        }
    }
}

void imu_init(const imu_port_t *port)
{
    s_port = port;
    memset(&s_st, 0, sizeof(s_st));
    s_have_prev_gyro = false;
    s_version_try_ms = 0u;      /* ⭐ 让第一次 poll 就能重试 VERSION */
    s_prev_gyro[0] = 0.0f;
    s_prev_gyro[1] = 0.0f;
    s_prev_gyro[2] = 0.0f;

    /* 四元数初值：单位四元数（水平朝前），而不是全 0 ——
     * 全 0 不是合法四元数，Pi 侧反算重力会得到 NaN。 */
    s_st.quat_wxyz[0] = 1.0f;
}

bool imu_selftest(void)
{
    if ((s_port == NULL) || (s_port->read_reg == NULL)) {
        return false;
    }

    /* ⚠️⚠️ 读【VERSION @ 0x2E】，不要读 0x00。
     *     手册里 0x00 是 SAVE（保存/重启/恢复出厂，**可写**）——
     *     对它做读操作至少无意义，更糟的是日后有人"顺手写一下"就会
     *     让模块重启或恢复出厂。只读的版本号在 0x2E。
     *
     * ⚠️ 不校验固定值：手册明说"不同产品，版本号不一样"。
     *     所以自检的判据是"**能读到**"，而不是"等于某个魔法数"。
     *     真正的 I2C 通断由 `i` 命令的总线探测（0x50 是否 ACK）保证。 */
    uint8_t ver[2] = { 0u, 0u };
    if (!s_port->read_reg(RLCA_IMU_I2C_ADDR, RLCA_IMU_REG_VERSION, ver, 2u)) {
        return false;
    }
    /* ⚠️ 用 imu_i16 解析，不要手工拼字节 ——
     *    这里原本写的是大端 `(ver[0]<<8)|ver[1]`，与实测的小端相反。
     *    实测证据：上机 VERSION = 05 C1，小端 = 0xC105。 */
    s_st.version = (uint16_t)imu_i16(ver);
    return true;
}

void imu_poll(void)
{
    if ((s_port == NULL) || (s_port->read_reg == NULL)) {
        s_st.last_fail = (uint8_t)IMU_FAIL_NO_PORT;
        return;
    }

    const uint32_t now = s_port->millis();

    /* ⭐⭐ VERSION 重试（2026-10-16 修）
     *
     *  ⚠️ 问题：VERSION 只在 app_init 里由 imu_selftest() 读【一次】。
     *     而 JY901P 上电后自己也要启动 —— 上电瞬间那次 I2C 读很可能失败，
     *     于是 s_st.version 永远是 0，之后再也不重试。
     *     现场证据：`M` 里 version = 0，但 `I` 现读 0x2E 得到 0xC105（正常）。
     *
     *  ⚠️ 影响：version 会被当作 imu_chip 上报给树莓派，恒 0 会让平台识别出错。
     *
     *  ⇒ 在成功之前【低频重试】（每 100 ms 一次），成功后不再读。
     *     ⚠️ 不能每周期都读：那会给已经紧张的 I2C 增加一次事务。 */
    if ((s_st.version == 0u) && ((now - s_version_try_ms) >= 100u)) {
        s_version_try_ms = now;
        uint8_t ver[2] = { 0u, 0u };
        if (s_port->read_reg(RLCA_IMU_I2C_ADDR, RLCA_IMU_REG_VERSION, ver, 2u)) {
            s_st.version = (uint16_t)imu_i16(ver);
        }
    }

    /* ---- 读三组寄存器 ---- */
    uint8_t acc[6];
    uint8_t gyro[6];
    uint8_t ang[6];

    const bool ok_acc  = s_port->read_reg(RLCA_IMU_I2C_ADDR, RLCA_IMU_REG_ACC,  acc,  6u);
    const bool ok_gyro = s_port->read_reg(RLCA_IMU_I2C_ADDR, RLCA_IMU_REG_GYRO, gyro, 6u);
    const bool ok_ang  = s_port->read_reg(RLCA_IMU_I2C_ADDR, RLCA_IMU_REG_ANGLE, ang, 6u);

    if (!ok_acc || !ok_gyro || !ok_ang) {
        /* ⭐ 记下是哪一次读挂了（诊断的关键：三次的排查方向完全不同） */
        s_st.last_fail = (uint8_t)(!ok_acc  ? IMU_FAIL_READ_ACC
                                 : !ok_gyro ? IMU_FAIL_READ_GYRO
                                            : IMU_FAIL_READ_ANGLE);
        s_st.n_err++;
        mark_stale_if_needed(now);
        return;
    }

    /* ---- 解析 ---- */
    float g[3];
    g[0] = imu_gyro_to_rad_s(imu_i16(&gyro[0]));
    g[1] = imu_gyro_to_rad_s(imu_i16(&gyro[2]));
    g[2] = imu_gyro_to_rad_s(imu_i16(&gyro[4]));

    /* ---- 离群值保护 ----
     * ⚠️ 必须在写入 s_st 之前判定，否则坏值会先污染状态。 */
    if (s_have_prev_gyro) {
        for (uint8_t i = 0u; i < 3u; ++i) {
            const float d = g[i] - s_prev_gyro[i];
            if ((d > IMU_GYRO_SPIKE_LIMIT_RAD_S) || (d < -IMU_GYRO_SPIKE_LIMIT_RAD_S)) {
                s_st.last_fail = (uint8_t)IMU_FAIL_SPIKE;
                s_st.n_spike++;
                s_st.n_err++;
                mark_stale_if_needed(now);
                return;             /* 丢这一帧，保留上一帧有效数据 */
            }
        }
    }

    float q[4];
    float euler[3];
    imu_parse_angle_to_quat(ang, q, euler);

    /* ---- 提交 ---- */
    s_st.quat_wxyz[0] = q[0];
    s_st.quat_wxyz[1] = q[1];
    s_st.quat_wxyz[2] = q[2];
    s_st.quat_wxyz[3] = q[3];

    s_st.gyro_rad_s[0] = g[0];
    s_st.gyro_rad_s[1] = g[1];
    s_st.gyro_rad_s[2] = g[2];

    s_st.accel_g[0] = imu_accel_to_g(imu_i16(&acc[0]));
    s_st.accel_g[1] = imu_accel_to_g(imu_i16(&acc[2]));
    s_st.accel_g[2] = imu_accel_to_g(imu_i16(&acc[4]));

    s_st.roll_rad  = euler[0];
    s_st.pitch_rad = euler[1];
    s_st.yaw_rad   = euler[2];

    s_prev_gyro[0] = g[0];
    s_prev_gyro[1] = g[1];
    s_prev_gyro[2] = g[2];
    s_have_prev_gyro = true;

    s_st.last_update_ms = now;
    s_st.valid = 1u;
    s_st.last_fail = (uint8_t)IMU_FAIL_NONE;    /* ⭐ 成功即清失败原因 */
    s_st.n_ok++;
}

const imu_state_t *imu_state(void)
{
    return &s_st;
}

bool imu_is_ok(void)
{
    return s_st.valid != 0u;
}

bool imu_read_raw(uint8_t reg, uint8_t *buf, uint8_t len)
{
    if ((s_port == NULL) || (s_port->read_reg == NULL) || (buf == NULL)) {
        return false;
    }
    return s_port->read_reg(RLCA_IMU_I2C_ADDR, reg, buf, len);
}
uint32_t imu_age_ms(void)
{
    if ((s_port == NULL) || (s_st.last_update_ms == 0u)) {
        return 0xFFFFFFFFu;
    }
    return s_port->millis() - s_st.last_update_ms;
}
