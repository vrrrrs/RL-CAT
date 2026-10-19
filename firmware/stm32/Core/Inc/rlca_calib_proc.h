/**
  ******************************************************************************
  * @file    rlca_calib_proc.h
  @brief   关节零点标定流程的状态机。
  *
  *  ⭐ 原理（固件设计书 §6.1）：
  *     URDF 的 q = 0 对应机器人【装配好的弯腿站立姿态】。
  *     所以 offset[i] = q 取 0 时舵机 i 的读数。
  *     ⚠️ 这个设计的优点是：标定姿态就是机器人自然站立的姿态，
  *        不需要专用夹具，也不要把腿掰直（那会落在运动学奇异点上）。
  *
  *  流程（固件设计书 §6.2）：
  *     ① 卸力矩，机器人靠自重沉到最稳姿态
  *     ② 等 1~2 s 让振荡停止
  *     ③ 读 12 个位置（多取几次平均，压掉抖动）
  *     ④ 逐关节做【方向测试】：给一个已知的 +Δq，看编码器是增还是减
  *        → 得到 dir[i] ∈ {+1, -1}，然后回到原位
  *     ⑤ 用步骤 ③ 的读数作为 offset
  *     ⑥ 保存到 Flash
  *
  *  ⚠️ 为什么需要步骤 ④：
  *     固件设计书 §3.4 说 dir 处理"左右腿舵机安装方向差异"，
  *     但那只说了"要有一个 dir"，没说它从哪来。
  *     只靠 URDF 推不出来（URDF 的轴方向描述的是关节正方向，
  *     不是舵机编码器方向）—— 取决于舵机实际怎么装的。
  *     所以必须实测：命令它动一点，看编码器往哪边走。
  *
  *  ⚠️ 本模块是【纯逻辑】：舵机访问走注入的 rlca_calib_hw_t，
  *     时间走注入的 tick，所以整条流程能在 PC 上用假舵机跑完。
  ******************************************************************************
  */

#ifndef RLCA_CALIB_PROC_H
#define RLCA_CALIB_PROC_H

#include <stdint.h>
#include <stdbool.h>

#include "rlcat_config.h"
#include "rlca_math.h"

/*==============================================================================
 * 注入的硬件接口
 *============================================================================*/

typedef struct {
    /** 读某关节的舵机位置（步）。返回 false = 本次读失败。 */
    bool (*get_pos)(uint8_t joint_idx, int16_t *pos_step);

    /** 写某关节的目标位置（步）。返回 false = 发送失败。 */
    bool (*set_pos)(uint8_t joint_idx, int16_t pos_step);

    /** 力矩使能/卸载（整机）。 */
    void (*set_torque_all)(bool enable);

    /** 毫秒时间戳。 */
    uint32_t (*millis)(void);
} rlca_calib_hw_t;

/*==============================================================================
 * 状态
 *============================================================================*/

typedef enum {
    CAL_IDLE = 0,
    CAL_UNLOAD,        /* 卸力矩                                      */
    CAL_SETTLE,        /* 等机器人沉到最稳姿态                         */
    CAL_SAMPLE,        /* 多轮读位置取平均                            */
    CAL_DIR_TEST,      /* 逐关节给 +Δ 测方向（含回位）                 */
    CAL_DIR_SETTLE,    /* ⭐ 等所有关节回到基准位置 —— 否则 offset 会读到"路上"的值 */
    CAL_FINAL_READ,    /* 最后再读一次位置 → 作为 offset              */
    CAL_VERIFY,        /* 校验结果是否合理                            */
    CAL_DONE,
    CAL_FAILED
} calib_state_t;

/** 标定失败的原因（诊断） */
typedef enum {
    CAL_ERR_NONE = 0,
    CAL_ERR_READ_POS,       /* 读位置失败                                  */
    CAL_ERR_WRITE_POS,      /* 写位置失败                                  */
    CAL_ERR_NO_MOTION,      /* 方向测试时舵机没动 → 力矩没使能/机械卡死      */
    CAL_ERR_DIR_AMBIGUOUS,  /* 位移太小，方向判不出来                        */
    CAL_ERR_LIMIT            /* 结果超出合理范围                             */
} calib_err_t;

typedef struct {
    calib_state_t state;
    calib_err_t   err;

    uint8_t  joint;          /* 当前处理的关节（DIR_TEST / DIR_RESTORE / SAMPLE） */
    uint8_t  round;          /* 采样轮次                                     */
    uint32_t state_t0;       /* 进入当前状态的时刻                            */

    int16_t  base[RLCA_N_JOINTS];      /* 目标姿态下的读数（步骤 ③）        */
    int32_t  accum[RLCA_N_JOINTS];     /* 采样累加（求平均用）              */
    int16_t  moved[RLCA_N_JOINTS];     /* 方向测试时"动过之后"的读数         */

    rlca_calib_t result;     /* 标定结果（成功后有效）                        */
    uint8_t  n_samples;      /* 已完成的有效采样轮数                          */

    /** ⚠️ 结果是否已被取走（calib_proc_take_done 用）。
     *  放在状态里而不是函数内的 static —— static 版本要等状态离开 CAL_DONE
     *  才自动复位，于是"连续标定两次"时第二次的结果取不出来。
     *  由 calib_proc_start() 清零。 */
    bool     result_taken;

    /** ⭐ 诊断：do_sample() 实际执行（读到位置）的次数。
     *  正常应为 CAL_SAMPLE_ROUNDS × RLCA_N_JOINTS = 96。 */
    uint32_t n_sample_calls;
} calib_proc_t;

/*==============================================================================
 * 接口
 *============================================================================*/

/** 初始化（复位到 IDLE）。 */
void calib_proc_init(const rlca_calib_hw_t *hw);

/** 从当前标定表出发开始一次标定。
 *  ⚠️ 会保留 limit_*（限位来自 URDF，不是标定出来的）。 */
void calib_proc_start(const rlca_calib_t *current);

/** 取消（立即回到 IDLE，不保存）。 */
void calib_proc_cancel(void);

/** 主循环高频调用：推进状态机。
 *  ⚠️ 全程非阻塞，最耗时的动作是"发一条单关节位置指令"（几十字节 @1 Mbps）。 */
void calib_proc_poll(void);

/** 当前状态。 */
calib_state_t calib_proc_state(void);

/** 详细状态（诊断/上报用）。 */
const calib_proc_t *calib_proc_info(void);

/** 是否已完成（成功）。取走后标志清除。 */
bool calib_proc_take_done(rlca_calib_t *out);

/** 是否已失败。 */
bool calib_proc_failed(void);

/** 失败原因。 */
calib_err_t calib_proc_err(void);

/** 整体进度（0~100），用于上报。 */
uint8_t calib_proc_progress(void);

/*==============================================================================
 * 纯逻辑（供单测）
 *============================================================================*/

/** 由"移动前后的读数差"判定方向。
 *  @param delta_step  移动后读数 − 移动前读数（已做回绕修正）
 *  @param min_move    判据阈值（步）；|delta| < min_move 视为"没动"或"方向不明"
 *  @return +1 / -1；0 = 无法判定 */
int8_t calib_dir_from_delta(int16_t delta_step, int16_t min_move);

/** 校验标定结果是否合理。
 *  @return 0 = 通过；否则是 calib_err_t */
calib_err_t calib_validate_result(const rlca_calib_t *c);

#endif /* RLCA_CALIB_PROC_H */
