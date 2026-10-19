#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RL-CAT 树莓派侧：遥测语义层

把协议里的裸数字翻译成"人能读、能判断"的东西：
    · status / err_flags 位域 → 名字
    · 电压 → 电池等级（正常/警告/建议停/强制趴下）
    · ⭐ 请求模式 → **生效模式**（固件只上报"请求的模式"，生效的要自己推）
    · 舵机温度的两种单位（协议写 0.1°C，固件实际写原始 °C）
    · 关节角 → 一行紧凑可读

⚠️ 这一层【不做】任何控制，也不做观测组装。纯读。
"""

from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

import config as C
from rlcat_proto import (
    ERRF_ANGLE,
    ERRF_ANGLE_LIMIT,
    ERRF_BUS_TIMEOUT,
    ERRF_CALIB_INVALID,
    ERRF_OVERLOAD,
    ERRF_OVERCURRENT,
    ERRF_OVERHEAT,
    ERRF_VOLTAGE,
    MODE_ACTIVE,
    MODE_DAMP,
    MODE_ESTOP,
    MODE_IDLE,
    MODE_STAND,
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
# 位域 → 名字
# ============================================================================

STATUS_BITS = (
    (ST_READY, "READY"),
    (ST_CALIBRATED, "CALIBRATED"),
    (ST_LINK_OK, "LINK_OK"),
    (ST_IMU_OK, "IMU_OK"),
    (ST_BATT_WARN, "BATT_WARN"),
    (ST_BATT_STOP, "BATT_STOP"),
    (ST_BATT_FORCE, "BATT_FORCE"),
    (ST_ESTOP, "ESTOP"),
)

ERR_BITS = (
    (ERRF_VOLTAGE, "VOLTAGE"),
    (ERRF_ANGLE, "ANGLE"),
    (ERRF_OVERHEAT, "OVERHEAT"),
    (ERRF_OVERCURRENT, "OVERCURRENT"),
    (ERRF_ANGLE_LIMIT, "ANGLE_LIMIT"),
    (ERRF_OVERLOAD, "OVERLOAD"),
    (ERRF_BUS_TIMEOUT, "BUS_TIMEOUT"),
    (ERRF_CALIB_INVALID, "CALIB_INVALID"),
)

MODE_NAMES = {
    MODE_IDLE: "IDLE",
    MODE_STAND: "STAND",
    MODE_ACTIVE: "ACTIVE",
    MODE_DAMP: "DAMP",
    MODE_ESTOP: "ESTOP",
}


def status_str(bits: int) -> str:
    on = [n for b, n in STATUS_BITS if bits & b]
    return "|".join(on) if on else "-"


def err_str(bits: int) -> str:
    on = [n for b, n in ERR_BITS if bits & b]
    return "|".join(on) if on else "无"


def mode_name(m: int) -> str:
    return MODE_NAMES.get(m, "?%d" % m)


# ============================================================================
# 电池
# ============================================================================

BATT_LEVELS = (
    (0, "正常", ""),
    (1, "警告 L1", "LED 慢闪；策略侧应记录"),
    (2, "建议停 L2", "固件把动作幅度压到 50%"),
    (3, "强制趴下 L3", "固件卸载力矩并趴下"),
)


def battery_level(mv: int) -> Tuple[int, str]:
    """返回 (等级 0..3, 说明)。判据是 <=，与固件一致（rlcat_config.h）。"""
    if mv <= 0:
        return -1, "读数无效（TEL_FAST 的 voltage_mv = 0 表示电池采样失效）"
    if mv <= C.BATT_FORCE_MV:
        return 3, BATT_LEVELS[3][1]
    if mv <= C.BATT_STOP_MV:
        return 2, BATT_LEVELS[2][1]
    if mv <= C.BATT_WARN_MV:
        return 1, BATT_LEVELS[1][1]
    return 0, BATT_LEVELS[0][1]


# ============================================================================
# ⭐ 生效模式推断
# ============================================================================

def derive_effective_mode(req_mode: int, status: int) -> Tuple[int, str]:
    """把固件上报的【请求模式】推成【生效模式】。

    ⚠️ 为什么必须推：固件 TEL_STATUS.mode 给的是 link_pi_mode()，也就是
       "上层请求的模式"，**不是** app_mode 实际生效的那个。而 ACTIVE 有一道
       准入闸（标定 / 链路 / IMU / 舵机反馈 / 安全层），过不了就【静默降级成
       STAND】—— 不推的话你会以为机器人在跑策略，其实它只是站着。

    能看见的闸：CALIBRATED / LINK_OK / IMU_OK / ESTOP。
    看不见的闸：safety_blocks、舵机反馈 fb_ok —— 这两个推不出来，
               所以返回值可能偏乐观，注释里标注了。
    """
    if status & ST_ESTOP:
        return MODE_ESTOP, "ESTOP 位已置（安全层触发或主动请求）"

    if req_mode != MODE_ACTIVE:
        return req_mode, ""

    if not (status & ST_CALIBRATED):
        return MODE_STAND, "ACTIVE 被拒：未标定 (NOT_CALIB)"
    if not (status & ST_LINK_OK):
        return MODE_STAND, "ACTIVE 被拒：链路不活跃 (NO_LINK)"
    if not (status & ST_IMU_OK):
        return MODE_STAND, "ACTIVE 被拒：IMU 无效 (NO_IMU)"
    return MODE_ACTIVE, "（注意：安全层与舵机反馈这两道闸看不出来）"


# ============================================================================
# 舵机温度
# ============================================================================

TEMP_UNITS = ("raw", "deci")


def temp_to_c(value: int, unit: str = "raw") -> float:
    """舵机温度 → 摄氏度。

    ⚠️ 协议规范 §4.8 写 TEL_STATUS 的温度单位是 0.1 °C，
       但固件（app_main.c:809-815）实际塞的是 servo_fb_t.temp_c，即原始 °C。
       两者差 10 倍。默认按【固件实际行为】raw 处理。
       上真机时"用手捂一下某个舵机看数值变多少"就能一次分辨：
       原始 °C 会变几度，0.1°C 会变几十。
    """
    if unit == "deci":
        return value / 10.0
    return float(value)


def temp_level(celsius: float) -> int:
    """0 = 正常；1 = 偏高；2 = 超温（固件阈值 70 °C 会强制趴下）"""
    if celsius > C.TEMP_OVERHEAT_C:
        return 2
    if celsius >= C.TEMP_WARN_C:
        return 1
    return 0


# ============================================================================
# 格式化
# ============================================================================

def fmt_vec(v: Sequence[float], fmt: str = "%+.3f", sep: str = " ") -> str:
    return sep.join(fmt % x for x in v)


def fmt_joints(q: Sequence[float], start: int = 0, count: int = 12,
               fmt: str = "%+.3f") -> str:
    return fmt_vec(q[start:start + count], fmt)


def one_line_summary(tel, st=None, temp_unit: str = "raw") -> str:
    """一行紧凑摘要（日志模式用）。"""
    lvl, lvl_name = battery_level(tel.voltage_mv)
    eff, _ = derive_effective_mode(st.mode if st is not None else MODE_IDLE, tel.status)
    temps = [temp_to_c(v, temp_unit) for v in (st.temperature if st is not None else [])]
    tmax = max(temps) if temps else 0.0
    return ("V=%5dmV(%-9s) mode=%s/%s status=%-32s err=%s Tmax=%.1fC cyc=%5d"
            % (tel.voltage_mv, lvl_name,
               mode_name(st.mode if st is not None else -1), mode_name(eff),
               status_str(tel.status), err_str(tel.err_flags), tmax, tel.cycle_cnt))
