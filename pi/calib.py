#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RL-CAT 树莓派侧：阶段 3 舵机标定

⚠️⚠️ 先读这一节，否则会误判 ⚠️⚠️

"协议里到底能观察到多少标定信息" —— 实测/读码结论：

| 想知道的          | 协议里 (USART1)          | 壳 (shell) 里 |
|------------------|--------------------------|--------------|
| 能不能启动         | CMD_CALIB(0) 的 ACK      | `C start` 的打印 |
| 进度百分比         | ❌ 没有                   | ✅ `R` → calib_progress |
| 失败原因           | ❌ 没有（ACK 恒 result=0）| ✅ `R` → calib_err |
| 是否成功           | ✅ 只有 STAT_CALIBRATED 0→1 | ✅ `R` → calib_state |
| 逐关节 dir/offset  | ❌ 没有                   | ✅ `R` → j0 id=.. off=.. dir=.. |
| 手动零点 (zero)    | ❌ 动作 5 被解码器拒收     | ✅ `C zero` |

⚠️ 而且"壳"这条路在树莓派上**走不通**：固件里 `link_pi_poll()` 会把 USART1 的
   环形缓冲抽干，之后 `bsp_shell_poll()` 什么都拿不到（app_main.c:458-482 vs
   link_pi.c:184-224）。所以：

   · 本脚本只用【协议】控制标定 → 能启动、能判断"从未标定变成已标定"。
   · 要看进度 / 错误码 / 逐关节表 → 必须走 ESP32 无线口（TCP 3333）或台式
     USB-TTL 直接接 STM32 的 USART1（此时树莓派得先断开）。

⚠️⚠️ 另一个必须知道的坑：**已经标定过的机器人，失败的重标定无法用协议识别**。
   因为失败时固件不会清掉 STAT_CALIBRATED（旧表还在），位一直是 1，没有 0→1 跳变。
   所以本脚本在"起始就是已标定"时会明确告诉你这一点，不要当成成功。

⚠️ 标定会让【12 个关节都动约 0.15 rad】，必须先把机器人吊起或垫高。

用法：
    python3 calib.py --show                 # 看当前标定状态（只读，安全）
    python3 calib.py --start --yes          # 启动全自动标定并等待
    python3 calib.py --start --yes --timeout 25
    python3 calib.py --check                # 标定后自检：q 是否合理、能否站立

退出码：0 = 成功/已确认；1 = 失败或超时；2 = 打不开串口
"""

from __future__ import annotations

# ⚠️ Windows 控制台默认是 GBK，中文输出会乱码。在任何输出之前切成 UTF-8。
import sys as _sys
for _s in (_sys.stdout, _sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

import argparse
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import config as C                                      # noqa: E402
import telemetry as T                                   # noqa: E402
from link import PiLink, LinkError                      # noqa: E402
from rlcat_proto import (                               # noqa: E402
    MODE_IDLE,
    MODE_STAND,
    ST_CALIBRATED,
    ST_ESTOP,
    ST_IMU_OK,
    ST_LINK_OK,
    ST_READY,
)

# 固件里标定的成功路径约 8.4 s（rlca_calib_proc.c 各段之和），留足余量
CALIB_EXPECTED_S = 8.4
CALIB_DEFAULT_TIMEOUT_S = 20.0


class CalibResult:
    def __init__(self, verdict: str, detail: str, seconds: float):
        self.verdict = verdict        # ok / timeout / estop / link_lost / refused / no_ack
        self.detail = detail
        self.seconds = seconds

    @property
    def ok(self) -> bool:
        return self.verdict == "ok"

    def __str__(self) -> str:
        return "[%s] %s（用时 %.1f s）" % (self.verdict, self.detail, self.seconds)


class CalibSession:
    """标定会话：只用协议，不碰 shell。

    ⚠️ 全程必须【持续发帧】。固件的失联判定基于"距上次收到任意帧"：
       标定要 8~10 秒，中途一旦超过 100 ms 没收到帧，安全层就会开始趴下斜坡，
       把整次标定毁掉。
    """

    KEEPALIVE_HZ = 20.0

    def __init__(self, link: PiLink, quiet: bool = False):
        self.link = link
        self.quiet = quiet
        self._t_next_keep = 0.0

    def _say(self, msg: str) -> None:
        if not self.quiet:
            print(msg)

    def keepalive(self) -> None:
        now = time.perf_counter()
        if now >= self._t_next_keep:
            # mode=IDLE：标定期间固件本来就强制 IDLE，q_target 被忽略
            self.link.send_cmd_joint(C.Q_DEFAULT, mode=MODE_IDLE, flags=0)
            self._t_next_keep = now + 1.0 / self.KEEPALIVE_HZ

    # ------------------------------------------------------------------ 前置
    def precheck(self) -> tuple:
        """返回 (能否继续, 说明列表)。"""
        notes = []
        tel = self.link.wait_tel_fast(timeout_ms=2000)
        if tel is None:
            return False, ["2 秒内没收到 TEL_FAST —— 链路不通，先解决链路"]

        if not (tel.status & ST_READY):
            notes.append("⚠️ STAT_READY 未置位（时钟/DWT 自检没过）")
        if tel.status & ST_ESTOP:
            notes.append("⚠️ 当前处于 ESTOP —— 先发几帧让它解除")
        if tel.err_flags:
            notes.append("⚠️ err_flags=0x%02X（%s）—— latch，只有重启才清；"
                         "非 0 时安全层会把动作幅度压到 75%%"
                         % (tel.err_flags, T.err_str(tel.err_flags)))
        if not (tel.status & ST_IMU_OK):
            notes.append("⚠️ STAT_IMU_OK 未置位 —— 不影响标定，但之后 ACTIVE 过不了准入")

        st = self.link.wait_status(timeout_ms=1500)
        if st is not None and st.status & ST_CALIBRATED:
            notes.append("ℹ️ 当前【已经标定过】（STAT_CALIBRATED=1）")

        fatal = [n for n in notes if n.startswith("⚠️ STAT_READY")]
        return (not fatal), notes

    # ------------------------------------------------------------------ 启动
    def start(self) -> tuple:
        """发 CMD_CALIB(START)。返回 (是否收到 ACK, 说明)。"""
        self.link.last_ack = None
        if not self.link.send_cmd_calib(C.CALIB_START):
            return False, "CMD_CALIB 发送失败（串口写错误）"

        ack = self.link.wait_ack(timeout_ms=1500)
        if ack is None:
            return False, ("1.5 s 内没收到 TEL_ACK —— 固件没理这条命令"
                           "（检查是否把 CMD_CALIB 放进了别的 TYPE，或链路丢帧）")
        # ⚠️ 固件恒回 result=0，所以 result 不提供任何信息
        return True, "收到 TEL_ACK(action=%d, result=%d)（⚠️ result 恒为 0，无信息量）" \
            % (ack[0], ack[1])

    # ------------------------------------------------------------------ 等待
    def wait_done(self, timeout_s: float = CALIB_DEFAULT_TIMEOUT_S,
                  was_calibrated: bool = False) -> CalibResult:
        t0 = time.perf_counter()

        while True:
            self.keepalive()
            self.link.pump()

            dt = time.perf_counter() - t0

            # ⚠️ 必须只信【本次等待开始之后】收到的帧。否则上电那段的 ESTOP 帧
            #    （或任何历史帧）会被当成"标定过程中进入 ESTOP"，直接误判。
            tel = self.link.last_tel
            fresh = (tel is not None) and (self.link.t_rx_tel_fast >= t0)

            if fresh:
                if tel.status & ST_ESTOP:
                    return CalibResult("estop",
                                       "标定过程中进入了 ESTOP（失联/低压/超温）"
                                       "—— 标定被打断，结果不可信", dt)
                if tel.status & ST_CALIBRATED:
                    if was_calibrated:
                        return CalibResult("ok",
                                           "STAT_CALIBRATED 仍为 1（⚠️ 但起始就是 1，"
                                           "无法区分'新标定成功'与'旧表还在'）", dt)
                    return CalibResult("ok",
                                       "STAT_CALIBRATED 由 0 变 1 —— 新标定已完成并落盘", dt)

            if dt > timeout_s:
                if was_calibrated:
                    return CalibResult(
                        "timeout",
                        "超时且 STAT_CALIBRATED 始终为 1。⚠️ 因为起始就已标定，"
                        "这条路【无法判断】刚才是成功还是失败 —— 请用 ESP32 无线口"
                        "（TCP 3333）敲 R 看 calib_state / calib_err", dt)
                return CalibResult(
                    "timeout",
                    "%.0f s 内 STAT_CALIBRATED 没有变成 1 → 标定很可能失败。"
                    "⚠️ 协议里拿不到失败原因，请用 ESP32 无线口敲 R 看 calib_err" % timeout_s,
                    dt)

            time.sleep(0.005)

    # ------------------------------------------------------------------ 自检
    def postcheck(self) -> tuple:
        """标定后自检：让机器人用 STAND 站住，看 12 个关节角是否合理。

        ⚠️⚠️ 未标定时【拒绝】发 STAND。
           固件的 STAND 没有任何准入门（app_mode.c 只给 ACTIVE 加了准入）。
           而默认标定表是 dir=+1 / offset=0（rlca_calib_load_defaults 里 memset 出来的），
           q_target=0（=站姿）按 rlca_q_to_step 换算正好是【舵机 step 0】——
           于是 12 个舵机全部冲向绝对位置 0，也就是机械行程的一端。
           固件作者在 app_main.c:695-709 记录过这个撞机事故。
        """
        notes = []
        tel = self.link.wait_tel_fast(timeout_ms=1500)
        if tel is None:
            return False, ["收不到 TEL_FAST"]
        if not (tel.status & ST_CALIBRATED):
            return False, [
                "❌ 未标定 —— 拒绝发 STAND。",
                "⚠️ 固件的 STAND 没有准入门。未标定（默认表 dir=+1/offset=0）时，",
                "   q_target=0 会换算成舵机 step 0，把 12 个关节全部驱到机械行程一端。",
                "   请先跑 calib.py --start（并把机器人吊起）。",
            ]

        ok = True

        # 用 STAND 让固件把 q_ref 设成 q_default(=0)，这时读回的 q 就是"站姿读数"
        t_end = time.perf_counter() + 3.0
        while time.perf_counter() < t_end:
            self.link.send_cmd_joint(C.Q_DEFAULT, mode=MODE_STAND, flags=0)
            self.link.pump()
            time.sleep(0.02)

        tel = self.link.last_tel
        if tel is None:
            return False, ["收不到 TEL_FAST"]

        eff, why = T.derive_effective_mode(MODE_STAND, tel.status)
        notes.append("生效模式 %s" % T.mode_name(eff))

        worst = 0.0
        worst_i = -1
        for i in range(12):
            over = max(0.0, tel.q[i] - C.Q_MAX[i], C.Q_MIN[i] - tel.q[i])
            if over > worst:
                worst, worst_i = over, i
        if worst_i >= 0:
            ok = False
            notes.append("❌ 关节 %d(%s) 超出硬件限位 %.3f rad —— 标定零点可能不对"
                         % (worst_i, C.JOINT_NAMES[worst_i], worst))
        else:
            notes.append("✅ 12 个关节角都在硬件限位内")

        # q 的绝对值不该很大：标定正确时站姿读数应在 0 附近
        qmax = max(abs(v) for v in tel.q)
        if qmax > 0.6:
            notes.append("⚠️ 站姿下 |q| 最大 %.3f rad（>0.6）—— 零点可能偏了，"
                         "或机器人没被摆成装配站姿" % qmax)
        else:
            notes.append("✅ 站姿下 |q| 最大 %.3f rad，合理" % qmax)

        if not (tel.status & ST_CALIBRATED):
            ok = False
            notes.append("❌ STAT_CALIBRATED 仍未置位")
        return ok, notes


# ============================================================================
# 命令行
# ============================================================================

def cmd_show(link: PiLink) -> int:
    print("=" * 74)
    print(" 当前标定状态（只读）")
    print("=" * 74)
    tel = link.wait_tel_fast(timeout_ms=2000)
    if tel is None:
        print("!! 收不到 TEL_FAST")
        return 1
    st = link.wait_status(timeout_ms=1500)

    cal = bool(tel.status & ST_CALIBRATED)
    print("  STAT_CALIBRATED : %s" % ("✅ 已标定" if cal else "❌ 未标定 / 标定无效"))
    print("  STAT_READY      : %s" % bool(tel.status & ST_READY))
    print("  STAT_IMU_OK     : %s" % bool(tel.status & ST_IMU_OK))
    print("  STAT_LINK_OK    : %s" % bool(tel.status & ST_LINK_OK))
    print("  err_flags       : 0x%02X (%s)" % (tel.err_flags, T.err_str(tel.err_flags)))
    if st is not None:
        print("  请求模式        : %s" % T.mode_name(st.mode))
        print("  生效模式        : %s" % T.mode_name(T.derive_effective_mode(st.mode, tel.status)[0]))
        print("  舵机温度        : %s"
              % " ".join("%d" % v for v in st.temperature))
    print("  当前关节角(rad) :")
    for r, kind in ((0, "hip  "), (4, "thigh"), (8, "calf ")):
        print("      %s %s" % (kind, T.fmt_joints(tel.q, r, 4)))
    print()
    print("  ⚠️ 逐关节的 dir / offset 只能在 shell 里看（协议不提供）：")
    print("     用 ESP32 无线口：TCP 192.168.4.1:3333 连上后敲  R")
    print("  ⚠️ 手动零点（C zero）在协议里【发不出去】（action=5 被解码器拒收）。")
    print("     需要手动零点时，同样走无线口敲  C zero")
    return 0 if cal else 1


def cmd_start(link: PiLink, args) -> int:
    sess = CalibSession(link, quiet=False)

    print("=" * 74)
    print(" 阶段 3：舵机标定")
    print("=" * 74)
    print("⚠️⚠️ 标定会让【12 个关节都动约 0.15 rad】。")
    print("     请确认机器人已【吊起或垫高】，腿部悬空、周围无人。")
    print()

    cont, notes = sess.precheck()
    for n in notes:
        print("  " + n)
    if not cont:
        return 1

    tel = link.last_tel
    was_calibrated = bool(tel.status & ST_CALIBRATED)

    if not args.yes:
        print()
        try:
            r = input("确认开始标定？输入 yes 继续：").strip().lower()
        except EOFError:
            r = ""
        if r not in ("y", "yes"):
            print("已取消")
            return 1

    print()
    print("[1/3] 发送 CMD_CALIB(START) ...")
    ok, why = sess.start()
    print("      " + why)
    if not ok:
        return 1

    print("[2/3] 等待标定完成（预期 %.1f s，超时 %.0f s）..."
          % (CALIB_EXPECTED_S, args.timeout))
    print("      ⚠️ 期间持续发帧保活，否则固件会在 100 ms 后开始趴下")
    res = sess.wait_done(timeout_s=args.timeout, was_calibrated=was_calibrated)
    print("      " + str(res))

    if not res.ok:
        print()
        print("  下一步（拿失败原因）：")
        print("    · 用手机/电脑连 ESP32 的 WiFi（SSID RL-CAT-E959）")
        print("    · TCP 连 192.168.4.1:3333，敲  R  看 calib_state / calib_err")
        print("      （calib_err: 1=读位置失败 2=写位置失败 3=舵机没动 "
              "4=方向判不出 5=结果超范围）")
        return 1

    print("[3/3] 标定后自检 ...")
    ok2, notes2 = sess.postcheck()
    for n in notes2:
        print("      " + n)
    return 0 if ok2 else 1


def main() -> int:
    ap = argparse.ArgumentParser(description="RL-CAT 阶段 3：舵机标定")
    ap.add_argument("--port", default=C.SERIAL_PORT)
    ap.add_argument("--baud", type=int, default=C.SERIAL_BAUD)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--show", action="store_true", help="只读：打印当前标定状态")
    g.add_argument("--start", action="store_true", help="启动全自动标定")
    g.add_argument("--check", action="store_true", help="标定后自检")
    ap.add_argument("--yes", action="store_true", help="跳过交互确认（脚本用）")
    ap.add_argument("--timeout", type=float, default=CALIB_DEFAULT_TIMEOUT_S)
    args = ap.parse_args()

    try:
        link = PiLink(args.port, args.baud, verbose=False)
        link.open()
    except LinkError as e:
        print("!! %s" % e)
        return 2

    try:
        if args.show:
            return cmd_show(link)
        if args.check:
            sess = CalibSession(link)
            ok, notes = sess.postcheck()
            for n in notes:
                print("  " + n)
            return 0 if ok else 1
        return cmd_start(link, args)
    finally:
        try:
            link.send_cmd_mode(MODE_IDLE)
            time.sleep(0.2)
        except Exception:
            pass
        link.close()


if __name__ == "__main__":
    sys.exit(main())
