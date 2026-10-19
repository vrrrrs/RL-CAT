#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RL-CAT 树莓派侧：所有参数集中在这里

⚠️ 范围：只覆盖【基础】部分（链路 / 遥测 / 标定 / 关节控制）。
   不含强化学习策略相关的任何参数。

⚠️ 数值来源（改任何一项都要同步这两处）：
   firmware/stm32/Core/Inc/rlcat_config.h
   docs/30-firmware.md §3 / §6
"""

from __future__ import annotations

# 与固件/协议共用的常量一律从 rlcat_proto 取，避免出现第二份定义
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from rlcat_proto import (  # noqa: E402
    ACTION_SCALE,
    GAIT_FREQ_HZ,
    LEN_CMD_JOINT,
    LEN_TEL_FAST,
    LEN_TEL_STATUS,
    MODE_ACTIVE,
    MODE_DAMP,
    MODE_ESTOP,
    MODE_IDLE,
    MODE_STAND,
    N_JOINTS,
    Q_UNIT,
    QD_UNIT,
    TAU_UNIT,
    # 位域
    ERRF_ANGLE,
    ERRF_ANGLE_LIMIT,
    ERRF_BUS_TIMEOUT,
    ERRF_CALIB_INVALID,
    ERRF_OVERLOAD,
    ERRF_OVERCURRENT,
    ERRF_OVERHEAT,
    ERRF_VOLTAGE,
    FLAG_ENABLE,
    FLAG_REQ_CALIB,
    FLAG_REQ_STATUS,
    FLAG_RESET_ERR,
    ST_BATT_FORCE,
    ST_BATT_STOP,
    ST_BATT_WARN,
    ST_CALIBRATED,
    ST_ESTOP,
    ST_IMU_OK,
    ST_LINK_OK,
    ST_READY,
)

# ============================================================================
# 1. 链路
# ============================================================================

SERIAL_PORT = "/dev/ttyAMA0"      # ⚠️ 专用 UART0(PL011)，不是 ttyS0/ttyAMA10
# ⚠️ 必须与固件 main.c 的 `huart1.Init.BaudRate` 完全一致。
#
#  ⚠️ 2026-10-18 从 1 Mbps 降到 500 kbps。原因（实测链路错误分解）：
#        link_rx_frames   = 285   完整帧
#        link_rx_timeouts = 349   半帧超时（收一半就断）—— 比完整帧还多！
#        link_crc_errors  = 10
#     ⇒ 字节在传输中途丢失 = 1 Mbps 信号完整性问题，不是软件 bug。
#     ⚠️ 换速率后如果丢帧归零，就确诊是线路；如果还丢，查共地/走线/屏蔽。
#  ⚠️ 带宽占用约 29%（见 main.c 里的核算），降速不影响功能。
SERIAL_BAUD = 500_000              # 与 STM32 USART1 一致
SERIAL_FORMAT = "8N1"
FRAME_TIMEOUT_MS = 5               # 收半个帧后复位状态机（协议 §6）
RX_CHUNK = 4096                    # 每次非阻塞读的字节数
LINK_LOST_MS = 150                 # 固件：>150 ms 无任何帧 → 降级
LINK_ESTOP_MS = 1200               # 固件：>1200 ms 无任何帧 → 急停

# ⚠️⚠️ 这两个值必须与固件 rlcat_config.h 里的
#      RLCA_LINK_LOST_MS / RLCA_LINK_ESTOP_MS **完全一致**，
#      否则 `PiLink.is_alive()` 的判断会和固件相反。
#
#  ⚠️ 2026-10-17 从 30/100 放大到 80/300。原因（实测）：
#     树莓派是普通 Linux + Python，不是实时系统。哪怕保活做到 100 Hz，
#     GIL（默认 5 ms 切换）、GC、往 SSH 管道 print、非 RT 调度，
#     都能造成几十毫秒的抖动；实测出现过 517 ms 的空档。
#     30 ms 阈值只有"3 个控制周期"的余量 ⇒ 降级/急停常态化误触发。
#
#  ⚠️ 急停侧现在是【只卸力矩】（固件 RLCA_SAFE_ESTOP_SITDOWN = 0），
#     不会主动折腿，所以"慢一点才停"不会更危险。
#
#  ⚠️⚠️ 2026-10-18 又从 80/300 放大到 **150/1200**（开发期放宽）。
#     原因（docs/49 §4-D2）：M1 站立测试里 ~6% 的周期被 ESTOP 卸力矩，
#     分原因计数显示 7/9 次是"过温"——而舵机实测只有 30℃、阈值是 70℃，
#     是 servo_bus 读到了坏温度值。阈值过紧让 87 ms 级瞬态就能触发。
#
#     ⚠️ 真保护（电池 ≤9300 mV、舵机 >70℃）一个字没动。
#     ⚠️ 这两个值必须在 M2/M3 验证后改回 80/300 再做安全回归。

# ============================================================================
# 2. 控制
# ============================================================================

CONTROL_HZ = 100
CONTROL_PERIOD_MS = 1000 // CONTROL_HZ
Q_DEFAULT = [0.0] * N_JOINTS       # ⚠️ 全 0 = 站立姿态（已实测确认）
MAX_DELTA_PER_CYCLE = 0.15         # rad；固件侧限速，Pi 侧对齐以免被削

# ============================================================================
# 3. 关节：逻辑索引 A（协议/固件/所有 12 维向量一律用这个顺序）
# ============================================================================

#              0        1        2        3        4          5          6          7          8         9         10        11
JOINT_NAMES = ["lf_hip", "lb_hip", "rf_hip", "rb_hip",
               "lf_thigh", "lb_thigh", "rf_thigh", "rb_thigh",
               "lf_calf", "lb_calf", "rf_calf", "rb_calf"]

JOINT_KIND = ["hip"] * 4 + ["thigh"] * 4 + ["calf"] * 4

# 舵机 ID = 逻辑索引 + 1（rlcat_config.h §2.3 的建议方案）
SERVO_ID = [i + 1 for i in range(N_JOINTS)]

# 物理限位（rad）—— rlcat_config.h:296-301 / docs/30-firmware.md §3.2
#   hip   : 0 ~ +40°        （不对称，不做内摆）
#   thigh : −35° ~ +90°     （⚠️ 用训练 URDF 的 90°，不是硬件 URDF 的 30°）
#   calf  : −60° ~ +60°
_Q_MIN = {
    "hip": 0.0,
    "thigh": -0.61086524,
    "calf": -1.0471975,
}
_Q_MAX = {
    "hip": 0.6981317,
    "thigh": 1.5707963,
    "calf": 1.0471975,
}

Q_MIN = [_Q_MIN[k] for k in JOINT_KIND]
Q_MAX = [_Q_MAX[k] for k in JOINT_KIND]

# 软限位：留 5° 余量，避免直接顶到硬件限位（标定误差会撞限位）
#
# ⚠️⚠️ 但不能因为留余量而把【站姿】排除在外！
#    hip 的硬件限位是 0°~+40°（不对称、不做内摆），而 q_default = 0°
#    正好落在 0° 这个端点上。若直接写成 [5°, 35°]，那么钳位函数会把
#    所有髋关节的目标从 0° 抬到 5° —— 机器人就再也站不正了。
#    所以这里对两端各取"与 q_default 更靠内"的那个值，保证 q_default 一定在区间内。
_SOFT_MARGIN_RAD = 5.0 * 3.141592653589793 / 180.0
Q_SOFT_MIN = [min(Q_MIN[i] + _SOFT_MARGIN_RAD, Q_DEFAULT[i])
              for i in range(N_JOINTS)]
Q_SOFT_MAX = [max(Q_MAX[i] - _SOFT_MARGIN_RAD, Q_DEFAULT[i])
              for i in range(N_JOINTS)]


def clamp_q(q: float, idx: int) -> float:
    """按【Pi 侧软限位】夹紧某个关节角（rad）。

    ⚠️ 软限位是 Pi 自己的安全余量，比固件的硬件限位更紧；
       固件那边还会再按硬件限位夹一次（见 clamp_q_hard）。
    """
    lo = Q_SOFT_MIN[idx]
    hi = Q_SOFT_MAX[idx]
    if q != q:            # NaN
        return Q_DEFAULT[idx]
    return lo if q < lo else (hi if q > hi else q)


def clamp_q_hard(q: float, idx: int) -> float:
    """按【固件侧硬件限位】夹紧（rlcat_config.h 的 rlca_q_range 值）。

    ⚠️ 这是假 STM32（测试替身）该用的那套 —— 固件夹的是硬件限位，
       不是 Pi 的软限位。两边用同一套会让测试看不出"Pi 侧过度钳位"这类 bug。
    """
    lo = Q_MIN[idx]
    hi = Q_MAX[idx]
    if q != q:
        return Q_DEFAULT[idx]
    return lo if q < lo else (hi if q > hi else q)


def clamp_q_vec(q: list[float]) -> list[float]:
    return [clamp_q(v, i) for i, v in enumerate(q)]


# ============================================================================
# 4. 电池阈值（与固件一致，判据是 <=）
# ============================================================================

BATT_WARN_MV = 10_500
BATT_STOP_MV = 9_900
BATT_FORCE_MV = 9_300

# ============================================================================
# 5. 舵机温度
# ============================================================================

TEMP_WARN_C = 60
TEMP_OVERHEAT_C = 70

# ============================================================================
# 6. CMD_CALIB action（协议 §4.9）
# ============================================================================

CALIB_START = 0
CALIB_CANCEL = 1
CALIB_QUERY = 2
CALIB_SAVE = 3
CALIB_RESTORE_DEFAULT = 4
# ⚠️ 固件里还有一个动作 5 = MANUAL_ZERO，但【解码器会拒收】它
#    （link_proto.c 只接受 0..4），所以树莓派发不出去，不能依赖。
CALIB_MANUAL_ZERO_UNREACHABLE = 5


# ============================================================================
# 自检
# ============================================================================

def _self_test() -> int:
    errs = []

    if len(JOINT_NAMES) != N_JOINTS:
        errs.append("JOINT_NAMES 长度 %d != %d" % (len(JOINT_NAMES), N_JOINTS))
    if len(Q_MIN) != N_JOINTS or len(Q_MAX) != N_JOINTS:
        errs.append("限位表长度不对")
    for i in range(N_JOINTS):
        if Q_MIN[i] > Q_MAX[i]:
            errs.append("关节 %d 限位反了" % i)
        if not (Q_MIN[i] <= Q_DEFAULT[i] <= Q_MAX[i]):
            errs.append("关节 %d 的 q_default 不在【硬件】限位内" % i)
        # ⚠️ 这一条是必须的：软限位一旦把 q_default 排除掉，
        #    钳位函数就会把站姿改掉（hip 的 0° 曾经被抬成 5°）。
        if not (Q_SOFT_MIN[i] <= Q_DEFAULT[i] <= Q_SOFT_MAX[i]):
            errs.append("关节 %d 的 q_default 不在【软】限位内 "
                        "(%.4f ~ %.4f) —— 站姿会被钳位改掉！"
                        % (i, Q_SOFT_MIN[i], Q_SOFT_MAX[i]))
        if Q_SOFT_MIN[i] > Q_SOFT_MAX[i]:
            errs.append("关节 %d 软限位反向（余量太大）" % i)
        if not (Q_MIN[i] <= Q_SOFT_MIN[i] and Q_SOFT_MAX[i] <= Q_MAX[i]):
            errs.append("关节 %d 软限位超出硬件限位" % i)

    if SERVO_ID != list(range(1, N_JOINTS + 1)):
        errs.append("SERVO_ID 不是 1..12")

    # 夹紧函数
    if clamp_q(99.0, 0) != Q_SOFT_MAX[0]:
        errs.append("clamp_q 上限失效")
    if clamp_q(-99.0, 0) != Q_SOFT_MIN[0]:
        errs.append("clamp_q 下限失效")
    if clamp_q(float("nan"), 0) != Q_DEFAULT[0]:
        errs.append("clamp_q 没处理 NaN")
    # ⚠️ 站姿必须原样通过软限位
    for i in range(N_JOINTS):
        if abs(clamp_q(Q_DEFAULT[i], i) - Q_DEFAULT[i]) > 1e-12:
            errs.append("关节 %d 的 q_default 被软限位改动了" % i)
    if clamp_q_hard(99.0, 0) != Q_MAX[0] or clamp_q_hard(-99.0, 0) != Q_MIN[0]:
        errs.append("clamp_q_hard 失效")

    if errs:
        for e in errs:
            print("  !! " + e)
        return 1
    print("  config.py 自检通过（12 关节 / 限位 / 软限位 / 夹紧函数）")
    return 0


if __name__ == "__main__":
    print("RL-CAT Pi 侧参数")
    print("  串口      : %s @ %d (%s)" % (SERIAL_PORT, SERIAL_BAUD, SERIAL_FORMAT))
    print("  控制      : %d Hz   关节 %d" % (CONTROL_HZ, N_JOINTS))
    print("  模式枚举  : IDLE=%d STAND=%d ACTIVE=%d DAMP=%d ESTOP=%d"
          % (MODE_IDLE, MODE_STAND, MODE_ACTIVE, MODE_DAMP, MODE_ESTOP))
    print()
    print("  索引  关节名      类型    软限位(deg)      舵机ID")
    for i in range(N_JOINTS):
        import math
        print("  %2d    %-10s  %-6s  %7.1f ~ %+7.1f     %d"
              % (i, JOINT_NAMES[i], JOINT_KIND[i],
                 math.degrees(Q_SOFT_MIN[i]), math.degrees(Q_SOFT_MAX[i]),
                 SERVO_ID[i]))
    print()
    sys.exit(_self_test())
