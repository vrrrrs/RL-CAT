/**
  ******************************************************************************
  * @file    imu.h
  * @brief   JY901P 姿态传感器：软件 I2C 读取四元数 + 角速度。
  *
  *  定位（固件设计书 §5.4）：
  *    JY901P【内部完成姿态融合】，MCU 只负责把结果读回来。
  *    所以这里没有 Mahony/Madgwick —— 那是被明确否决的方案
  *    （docs/20-electronics.md §3.2）。
  *
  *  ⚠️ 传四元数而不是直接传重力向量（协议 §4.4）：
  *     四元数是完整姿态，Pi 侧可以自行算重力向量/欧拉角/旋转矩阵，
  *     不必为了一个派生量改协议。
  *
  *  ✅ 寄存器映射【已核实】（2026-10-11）：依据 reference/JY901P_I2C协议.md
  *     并已上机通过 P5-2。映射与量程集中在 rlcat_config.h §3b。
  *     本文件把"读多少字节、怎么换算"与"寄存器在哪"分开，
  *     所以改映射不影响解析逻辑与测试。
  *
  *  ⚠️⚠️ 字节序是【小端】—— 手册没写，靠上机原始字节实测确定。
  *     早期版本按大端解析（"WIT 串口协议用大端"外推到 I2C），
  *     导致姿态全错、加速度模 15.6 g。详见 imu_i16() 的注释。
  *
  *  ⚠️ 与 battery 同样做端口注入：I2C 读写走 rlca_imu_port_t，
  *     于是解析、换算、超时、失效判定都能在 PC 上用假 JY901 单测。
  ******************************************************************************
  */

#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>

#include "rlcat_config.h"

/*==============================================================================
 * 数据结构
 *============================================================================*/

typedef struct {
    float quat_wxyz[4];      /* 四元数 w, x, y, z（单位化）          */
    float gyro_rad_s[3];     /* 角速度 x, y, z（rad/s，机身系）      */
    float accel_g[3];        /* 加速度 x, y, z（g），诊断用          */
    float roll_rad;          /* 欧拉角（诊断用，策略不使用）          */
    float pitch_rad;
    float yaw_rad;

    uint32_t last_update_ms; /* 最近一次成功读取的时刻               */
    uint32_t n_ok;           /* 成功次数                            */
    uint32_t n_err;          /* 连续/累计失败次数                    */
    uint32_t n_spike;        /* 被离群值保护拒绝的次数                */
    uint8_t  valid;          /* 本次数据是否有效                     */
    uint16_t version;        /* VERSION@0x2E 读到的版本号（诊断）      */

    /** ⭐ 诊断：最近一次失败的原因（2026-10-16 加）。
     *  ⚠️ 没有它就无法区分"三次 I2C 读哪一次挂了"和"离群值保护触发" ——
     *     只能看到 n_err/n_spike 涨，定位不到具体环节。
     *  取值见 imu_fail_t。 */
    uint8_t  last_fail;
} imu_state_t;

/** ⭐ 最近一次 imu_poll() 失败的原因（诊断用）。 */
typedef enum {
    IMU_FAIL_NONE = 0,      /* 还没失败过 / 最近一次是成功的        */
    IMU_FAIL_NO_PORT,       /* 端口未绑定（imu_init 没调？）        */
    IMU_FAIL_READ_ACC,      /* 读 ACC   0x34 失败                  */
    IMU_FAIL_READ_GYRO,     /* 读 GYRO  0x37 失败                  */
    IMU_FAIL_READ_ANGLE,    /* 读 ANGLE 0x3D 失败                  */
    IMU_FAIL_SPIKE          /* 陀螺帧间跳变超限，被离群值保护丢弃      */
} imu_fail_t;


/*==============================================================================
 * 端口（依赖注入）
 *============================================================================*/

typedef struct {
    /** 读寄存器（7 位地址 + 寄存器 + 长度）。
     *  @return true = 本次 I2C 事务成功（有 ACK、字节读全） */
    bool (*read_reg)(uint8_t dev, uint8_t reg, uint8_t *dst, uint8_t len);

    /** 毫秒时间戳（失效判定用） */
    uint32_t (*millis)(void);
} imu_port_t;

/*==============================================================================
 * 接口
 *============================================================================*/

/** 绑定端口并复位状态。不做 I2C 访问。 */
void imu_init(const imu_port_t *port);

/** 上电自检：读器件 ID 并与期望值比对。
 *  @return true = 读到期望的器件 ID */
bool imu_selftest(void);

/** 低频调用（10~100 Hz）。每次调用尝试读一帧；失败只累加计数，不阻塞。 */
void imu_poll(void);

/** 当前状态。 */
const imu_state_t *imu_state(void);

/** 姿态数据是否可用（近期读到过合法数据）。
 *  ⚠️ 这一位直接进 TEL_FAST.status 的 STAT_IMU_OK；
 *     Pi 侧启动自检会据此拒绝进入 ACTIVE（31-pi-runtime.md §5.1④）。 */
bool imu_is_ok(void);

/** 最近一次更新距现在多少毫秒（从未更新过返回 UINT32_MAX）。 */
uint32_t imu_age_ms(void);

/** ⭐ 裸寄存器 dump（诊断用）。
 *
 *  把指定寄存器区间的**原始字节**原样读出来，不做任何换算。
 *
 *  ⚠️ 为什么需要它：上机时"姿态不对"有三种完全不同的原因 ——
 *     ① 寄存器地址错（读到别的寄存器）
 *     ② 字节序错（大端/小端搞反）
 *     ③ 换算系数错（量程/单位假设不对）
 *     只看换算后的数值无法区分这三者，必须看原始字节。
 *
 *  @param reg  起始寄存器地址
 *  @param buf  输出缓冲
 *  @param len  读取字节数
 *  @return true = 读取成功
 */
bool imu_read_raw(uint8_t reg, uint8_t *buf, uint8_t len);

/** 寄存器 int16 → int16（⚠️ 小端，低字节在前）。
 *  ⚠️ 上机实测确认 JY901P 走小端；早期版本按大端解析导致姿态全错。
 *     详见 imu.c 里的实测证据。 */
int16_t imu_i16(const uint8_t *p);

/** 原始 int16 → 物理量。
 *  @param fs 满量程（例如 ±16 g 传 16.0） */
float imu_raw_to_fs(int16_t raw, float fs);

/** 角速度原始值 → rad/s。 */
float imu_gyro_to_rad_s(int16_t raw);

/** 加速度原始值 → g。 */
float imu_accel_to_g(int16_t raw);

/** 欧拉角原始值（0.01°，WIT 惯例）→ rad。 */
float imu_angle_to_rad(int16_t raw);

/** 解析一帧"角度寄存器"（8 字节）→ 四元数。
 *
 *  ⚠️ WIT 的 I2C 角度寄存器给的是【欧拉角】而不是四元数：
 *      roll(2) pitch(2) yaw(2) 版本(1) 保留(1)，单位 0.01°。
 *     所以这里要把欧拉角转成四元数（ZYX 顺序，即 yaw→pitch→roll）。
 *     这是与"直接读四元数"的重要差别，写文档时要讲清楚。
 *
 *  @param buf  至少 6 字节（后面 2 字节版本/保留会被忽略）
 *  @param out  写入的单位四元数 w,x,y,z
 *  @param euler_out 可选，写回计算的欧拉角（rad，roll/pitch/yaw） */
void imu_parse_angle_to_quat(const uint8_t *buf, float out_wxyz[4], float euler_out[3]);

#endif /* IMU_H */
