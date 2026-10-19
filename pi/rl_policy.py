#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RL 策略推理层：47 维观测 → 12 维动作 → 关节目标角。

⚠️ 本模块只做【纯计算】，不碰串口。这样它可以完全离线单测。
   把策略接到真机循环上的胶水在 `rl_run.py`。

设计要点
--------
1. **纯 numpy**，不依赖 torch / onnxruntime。
   网络只有 54220 个参数，前向 ~32 µs（x86）/ 65~97 µs（Pi 5）。
   理由见 docs/45-linux-tasks.md §2.2。

2. **47 维观测的布局是从仿真轨迹里反解出来的**（不是猜的）。
   依据 `pi/models/rlcat_flat_obs_action_trace.npz`（run=Oct06_03-32-41_，
   ckpt=500，1000 步，dt=0.01），逐段对齐后误差**精确为 0**：

       obs[t] = [
           gravity[t-1]            单位向量（机体系，模长恒 1）
           base_w[t-1]  * 0.25     陀螺（obs_scales.ang_vel）
           cmd[t-1]     * 2.0      ⚠️ 速度指令也乘 lin_vel=2.0（易漏！）
           q[t-1]       * 1.0      关节位置（相对 q_default）
           qd[t-1]      * 0.1      关节速度
           action[t-1]             上一帧动作
           phase[t]                (sin, cos)，**不平移**，步进 2π×0.03
       ]

   ⭐ 关键：**状态段全部是 t-1**（上一拍）。所以真机顺序天然正确：
      读遥测 → 构造 obs → 推理 → 下发。
   ⭐ 相位段是 t（当前），且步进与固件 `RLCA_GAIT_FREQ` 完全一致。

3. **动作 → 关节目标**（同样由轨迹验证，误差 0）：
      q_target = q_default + 0.25 * action      （action_scale = 0.25）

用法::

    from rl_policy import Policy, build_obs
    pol = Policy("models/policy_rlcat_flat.npz")
    act = pol.act(obs)                       # (12,) float32
    q_target = pol.action_to_q_target(act)   # (12,) 已夹软限位

    # 离线自测（用仿真轨迹逐位对拍）
    python3 rl_policy.py --selftest
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import config as C                                      # noqa: E402

# ============================================================================
# 观测规格（从仿真轨迹反解，改动前务必重跑 --selftest）
# ============================================================================

OBS_DIM = 47
ACT_DIM = 12

# 各段下标
OBS_GRAVITY = slice(0, 3)
OBS_ANG_VEL = slice(3, 6)
OBS_CMD = slice(6, 9)
OBS_DOF_POS = slice(9, 21)
OBS_DOF_VEL = slice(21, 33)
OBS_LAST_ACT = slice(33, 45)
OBS_PHASE = slice(45, 47)

# 缩放系数（来自 legged_gym 的 obs_scales）
SCALE_ANG_VEL = 0.25
SCALE_CMD = 2.0          # ⚠️ lin_vel=2.0 —— 作用在**指令**上，最容易漏的一个
SCALE_DOF_POS = 1.0
SCALE_DOF_VEL = 0.1

# 归一化裁剪（legged_gym 的 clip_observations / clip_actions）
CLIP_OBS = 100.0
CLIP_ACT = 100.0

# 动作缩放（仿真 control.action_scale = 0.25，与固件一致）
ACTION_SCALE = 0.25


# ============================================================================
# ⚠️⚠️ 关节顺序：训练侧与协议侧【不同】—— 这是最容易致命的坑
# ============================================================================
#
#  ⭐ 2026-10-18 由仿真轨迹实测确定（客观判据见 docs/48-sim2real-gaps.md G0）：
#
#    训练侧（Isaac Gym 的 dof 顺序 = URDF 里 <joint> 出现的顺序）= **leg-major**：
#        lf_hip, lf_thigh, lf_calf,
#        lb_hip, lb_thigh, lb_calf,
#        rf_hip, rf_thigh, rf_calf,
#        rb_hip, rb_thigh, rb_calf
#
#    协议 / 固件 / config.py                              = **group-major**：
#        lf_hip, lb_hip, rf_hip, rb_hip,
#        lf_thigh, lb_thigh, rf_thigh, rb_thigh,
#        lf_calf, lb_calf, rf_calf, rb_calf
#
#  ⚠️ 判据（用实机 URDF 限位衡量 1000 步仿真回放）：
#        group-major 解释 ⇒ 总限位违反 18384 度·步（rf_hip 到 −36°，实机下限是 0°）
#        leg-major   解释 ⇒ 总限位违反   544 度·步
#     ⇒ **leg-major 少 34 倍，因此策略输出是 leg-major。**
#
#  ⚠️⚠️ 顺序搞错的后果不是"精度下降"，而是**每条 action 都发到错误的关节**
#     ⇒ 上机立刻摔。所以本模块的对外 API **统一用协议顺序**，
#     内部负责换算 —— 调用者永远不需要关心这件事。
#
#  ⚠️ 另注：`Sim/legged_gym/.../rlcat_config.py` 里的 `JOINTS` 写的是
#     group-major，但它只用来建 `default_joint_angles` 字典（顺序无关），
#     所以没造成训练错误 —— 但它**误导性很强**，建议改成 leg-major。

#: 训练侧（leg-major）第 i 维 → 协议侧的关节下标
PERM_TRAIN_TO_PROTO = [0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11]
#: 协议侧第 i 维 → 训练侧的下标（上面那个的逆）
PERM_PROTO_TO_TRAIN = [0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11]


def train_to_proto(v) -> np.ndarray:
    """把训练顺序的 12 维向量换成协议顺序。"""
    a = np.asarray(v, dtype=np.float32).reshape(-1)
    return a[PERM_TRAIN_TO_PROTO].astype(np.float32)


def proto_to_train(v) -> np.ndarray:
    """把协议顺序的 12 维向量换成训练顺序。"""
    a = np.asarray(v, dtype=np.float32).reshape(-1)
    return a[PERM_PROTO_TO_TRAIN].astype(np.float32)


# ============================================================================
# 四元数 → 机体系重力向量
# ============================================================================

def quat_to_gravity(quat_wxyz) -> np.ndarray:
    """把"世界系重力 [0,0,-1]"旋转到机体系，返回单位向量。

    ⚠️ 输入是 (w, x, y, z) —— 这是固件 TEL_FAST 的约定。
       legged_gym 内部用 (x, y, z, w)，**顺序不同**，别混。

    ⚠️ 公式与 legged_gym 的 `quat_rotate_inverse(base_quat, [0,0,-1])` 等价：
       用四元数逆旋转矢量 v：
           t  = 2 * cross(-qv, v)
           v' = v + w*t + cross(-qv, t)
       （qv 是四元数的矢量部分；用 -qv 表示"逆旋转"）
    """
    q = np.asarray(quat_wxyz, dtype=np.float64).reshape(-1)
    if q.size != 4:
        raise ValueError("quat 必须是 (w,x,y,z) 四元数")
    w, x, y, z = q
    n = np.sqrt(w * w + x * x + y * y + z * z)
    if not np.isfinite(n) or n < 1e-9:
        # 退化输入：返回"竖直向上"（等于静置姿态），不要返回 NaN 污染策略
        return np.array([0.0, 0.0, -1.0], dtype=np.float32)
    w, x, y, z = w / n, x / n, y / n, z / n

    v = np.array([0.0, 0.0, -1.0])
    qv = np.array([-x, -y, -z])            # 逆旋转 ⇒ 取共轭
    t = 2.0 * np.cross(qv, v)
    out = v + w * t + np.cross(qv, t)
    return out.astype(np.float32)


# ============================================================================
# 观测构造
# ============================================================================

def build_obs(q, qd, gyro, quat_wxyz, cmd_vel, last_action,
              gait_sin, gait_cos) -> np.ndarray:
    """构造 47 维观测。

    @param q            关节角（rad，12 维）—— 固件 TEL_FAST.q
    @param qd           关节角速度（rad/s，12 维）
    @param gyro         机身角速度（rad/s，3 维，机体系）—— TEL_FAST.gyro
    @param quat_wxyz    姿态四元数 (w,x,y,z)
    @param cmd_vel      速度指令 (vx, vy, wz)，**单位 m/s 与 rad/s**
                        ⚠️ 传物理量，本函数内部才乘 2.0
    @param last_action  上一帧策略输出（12 维）；第一步传全 0
    @param gait_sin     相位 sin —— TEL_FAST.gait_sin
    @param gait_cos     相位 cos —— TEL_FAST.gait_cos

    ⚠️ `q` 要传**相对 q_default 的偏差**。当前 Q_DEFAULT 全 0，所以直接传
       固件的 q 即可；但这里显式减了一次，以后改 q_default 不会静默出错。
    """
    q = np.asarray(q, dtype=np.float32).reshape(-1)
    qd = np.asarray(qd, dtype=np.float32).reshape(-1)
    gyro = np.asarray(gyro, dtype=np.float32).reshape(-1)
    cmd = np.asarray(cmd_vel, dtype=np.float32).reshape(-1)
    last = np.asarray(last_action, dtype=np.float32).reshape(-1)

    if q.size != ACT_DIM or qd.size != ACT_DIM:
        raise ValueError("q / qd 必须是 %d 维" % ACT_DIM)
    if gyro.size != 3 or cmd.size != 3:
        raise ValueError("gyro / cmd_vel 必须是 3 维")
    if last.size != ACT_DIM:
        raise ValueError("last_action 必须是 %d 维" % ACT_DIM)

    obs = np.empty(OBS_DIM, dtype=np.float32)
    obs[OBS_GRAVITY] = quat_to_gravity(quat_wxyz)
    obs[OBS_ANG_VEL] = gyro * SCALE_ANG_VEL
    obs[OBS_CMD] = cmd * SCALE_CMD
    # ⚠️ 入参是【协议顺序】，而 obs 必须用【训练顺序】⇒ 这里换算
    #    （顺序的来龙去脉见文件顶部的 PERM_* 注释；搞错会每条 action 发错关节）
    obs[OBS_DOF_POS] = ((proto_to_train(q)
                         - np.asarray(C.Q_DEFAULT, np.float32))
                        * SCALE_DOF_POS)
    obs[OBS_DOF_VEL] = proto_to_train(qd) * SCALE_DOF_VEL
    obs[OBS_LAST_ACT] = last
    obs[45] = np.float32(gait_sin)
    obs[46] = np.float32(gait_cos)
    np.clip(obs, -CLIP_OBS, CLIP_OBS, out=obs)
    return obs


# ============================================================================
# 策略
# ============================================================================

class Policy:
    """numpy 版 MLP 策略。权重来自 tools/export_policy.py 导出的 .npz。"""

    def __init__(self, npz_path: str):
        d = np.load(npz_path)
        n = int(d["n_layers"])
        self.layers = []
        for i in range(n):
            W = np.ascontiguousarray(d["W%d" % i], dtype=np.float32)
            b = np.ascontiguousarray(d["b%d" % i], dtype=np.float32)
            self.layers.append((W, b))
        self.n_layers = n
        self.in_dim = self.layers[0][0].shape[1]
        self.out_dim = self.layers[-1][0].shape[0]
        if self.in_dim != OBS_DIM or self.out_dim != ACT_DIM:
            raise ValueError(
                "策略形状不符：入 %d（应 %d）出 %d（应 %d）"
                % (self.in_dim, OBS_DIM, self.out_dim, ACT_DIM))
        self.path = npz_path
        # 预热：首次 matmul 会初始化 BLAS，避免第一拍超时
        _ = self._forward(np.zeros((1, OBS_DIM), np.float32))

    # ---- 前向 ----
    @staticmethod
    def _elu(x):
        return np.where(x > 0.0, x, np.expm1(np.minimum(x, 0.0)))

    def _forward(self, x):
        h = np.asarray(x, dtype=np.float32)
        for i, (W, b) in enumerate(self.layers):
            h = h @ W.T + b
            if i != self.n_layers - 1:
                h = self._elu(h)
        return h

    def act(self, obs) -> np.ndarray:
        """单帧推理。obs 可以是 (47,) 或 (1,47)。返回 (12,)。"""
        x = np.asarray(obs, dtype=np.float32).reshape(1, -1)
        if x.shape[1] != OBS_DIM:
            raise ValueError("obs 必须是 %d 维" % OBS_DIM)
        y = self._forward(x)[0]
        return np.clip(y, -CLIP_ACT, CLIP_ACT).astype(np.float32)

    def act_batch(self, obs) -> np.ndarray:
        """批量推理（离线回放用）。obs (T,47) → (T,12)。"""
        x = np.asarray(obs, dtype=np.float32)
        if x.ndim != 2 or x.shape[1] != OBS_DIM:
            raise ValueError("obs 必须是 (T,%d)" % OBS_DIM)
        y = self._forward(x)
        return np.clip(y, -CLIP_ACT, CLIP_ACT).astype(np.float32)

    # ---- 动作 → 关节目标 ----
    @staticmethod
    def action_to_q_target(action, *, clamp: bool = True) -> np.ndarray:
        """q_target = q_default + action_scale * action。

        ⚠️ 与仿真 `control.action_scale = 0.25`、固件
           `q_target = q_default + 0.25*action` 三处一致。
        ⚠️ 默认夹到软限位：超出会让舵机顶到机械限位，声响很大且伤舵机。
        """
        a = np.asarray(action, dtype=np.float32).reshape(-1)
        if a.size != ACT_DIM:
            raise ValueError("action 必须是 %d 维" % ACT_DIM)
        # ⚠️ action 是【训练顺序】；q_target 要发给固件，必须是【协议顺序】
        a_proto = train_to_proto(a)
        q = np.asarray(C.Q_DEFAULT, np.float32) + ACTION_SCALE * a_proto
        if clamp:
            for i in range(ACT_DIM):
                q[i] = min(max(float(q[i]), C.Q_SOFT_MIN[i]), C.Q_SOFT_MAX[i])
        return q.astype(np.float32)


# ============================================================================
# 离线自测：用仿真轨迹逐位对拍
# ============================================================================

def selftest(trace_path: str, npz_path: str) -> int:
    """用仿真录制的 obs/action 轨迹验证本模块。

    ⚠️ 这是**唯一**能证明"真机 obs 构造与训练一致"的手段。
       ✅ 通过 ⇒ rl_run.py 里的观测拼装可以信任
       ❌ 不通过 ⇒ 不要上真机

    验证三件事：
      ① 用轨迹里的原始物理量重建 obs[3:47]，与录制值逐位对比
         （obs[0:3] 重力无法验证：轨迹没录四元数）
      ② 用重建的 obs 跑策略，与录制的 action 对比
      ③ q_target = 0.25 * action 的换算
    """
    d = np.load(trace_path, allow_pickle=True)
    obs_ref = d["obs"]
    act_ref = d["action"]
    q, qd = d["q"], d["qd"]
    w, cmd, ph = d["base_w"], d["cmd"], d["phase"]
    q_t_ref = d["q_target"]
    T = obs_ref.shape[0]

    print("=" * 74)
    print(" 策略层离线自测（对拍仿真轨迹）")
    print("=" * 74)
    print(" 轨迹 : %s" % trace_path)
    print(" 策略 : %s" % npz_path)
    print(" meta : %s" % (d["meta"] if "meta" in d.files else "?"))
    print(" 步数 : %d   dt = %s" % (T, d["dt"] if "dt" in d.files else "?"))

    fails = 0

    # ---- ① 重建 obs（状态段用 t-1，相位用 t）----
    rebuilt = np.empty_like(obs_ref)
    rebuilt[:, OBS_GRAVITY] = obs_ref[:, OBS_GRAVITY]      # 无法重建，直接沿用
    for t in range(1, T):
        rebuilt[t, OBS_ANG_VEL] = w[t - 1] * SCALE_ANG_VEL
        rebuilt[t, OBS_CMD] = cmd[t - 1] * SCALE_CMD
        rebuilt[t, OBS_DOF_POS] = q[t - 1] * SCALE_DOF_POS
        rebuilt[t, OBS_DOF_VEL] = qd[t - 1] * SCALE_DOF_VEL
    rebuilt[1:, OBS_LAST_ACT] = act_ref[:-1]
    rebuilt[0, OBS_LAST_ACT] = 0.0
    rebuilt[:, OBS_PHASE] = ph                             # 相位不偏移

    # ---- ⭐ ①b 关节顺序敏感性检查 ----
    # ⚠️ 这类错误"逐元素对拍"抓不到（对拍与顺序无关），必须单独测。
    print("\n ---- ①b 关节顺序（训练 leg-major ↔ 协议 group-major）----")
    obs_rt = build_obs(q=train_to_proto(q[0]), qd=train_to_proto(qd[0]),
                       gyro=[0, 0, 0], quat_wxyz=[1, 0, 0, 0],
                       cmd_vel=[0, 0, 0],
                       last_action=obs_ref[1, OBS_LAST_ACT],
                       gait_sin=ph[1, 0], gait_cos=ph[1, 1])
    d_pos = float(np.max(np.abs(obs_rt[OBS_DOF_POS] - q[0])))
    d_vel = float(np.max(np.abs(obs_rt[OBS_DOF_VEL] - qd[0] * SCALE_DOF_VEL)))
    print("   往返 dof_pos max|diff| = %.3e   %s" % (d_pos, "OK" if d_pos < 1e-6 else "FAIL"))
    print("   往返 dof_vel max|diff| = %.3e   %s" % (d_vel, "OK" if d_vel < 1e-6 else "FAIL"))
    if d_pos > 1e-6 or d_vel > 1e-6:
        fails += 1

    LEG_NAMES = ["lf_hip", "lf_thigh", "lf_calf", "lb_hip", "lb_thigh", "lb_calf",
                 "rf_hip", "rf_thigh", "rf_calf", "rb_hip", "rb_thigh", "rb_calf"]
    hip_bad = 0
    for i, nm in enumerate(LEG_NAMES):
        if nm.endswith("_hip"):
            if (np.any(q[:, i] < np.deg2rad(-3.0))
                    or np.any(q[:, i] > np.deg2rad(43.0))):
                hip_bad += 1
    print("   leg-major 下越界(>3 deg)的 hip 数 = %d   %s"
          % (hip_bad, "OK" if hip_bad == 0 else "FAIL: 顺序可能搞反"))
    if hip_bad != 0:
        fails += 1

    # 只比较 t>=1（t=0 的"上一帧"在轨迹里可能不是 0）
    seg_names = [("角速度", OBS_ANG_VEL), ("指令", OBS_CMD),
                 ("关节位置", OBS_DOF_POS), ("关节速度", OBS_DOF_VEL),
                 ("上一动作", OBS_LAST_ACT), ("相位", OBS_PHASE)]
    print("\n ---- ① obs 重建（除重力外的 44 维）----")
    worst = 0.0
    for name, sl in seg_names:
        diff = float(np.max(np.abs(rebuilt[1:, sl] - obs_ref[1:, sl])))
        worst = max(worst, diff)
        ok = diff < 1e-6
        print("   %-10s max|diff| = %.3e   %s" % (name, diff, "✅" if ok else "❌"))
        if not ok:
            fails += 1

    # ---- ② 用重建 obs 跑策略，与录制 action 对比 ----
    print("\n ---- ② 策略前向（用重建 obs）----")
    pol = Policy(npz_path)
    act_pred = pol.act_batch(rebuilt)
    # 只比较 t>=1：t=0 的 last_action 与轨迹里的初值可能不同
    diff_a = float(np.max(np.abs(act_pred[1:] - act_ref[1:])))
    scale = max(float(np.max(np.abs(act_ref[1:]))), 1e-9)
    print("   action max|diff| = %.3e   相对 = %.3e" % (diff_a, diff_a / scale))
    print("   抽样 #1  pred=%s" % np.round(act_pred[1, :3], 5))
    print("   抽样 #1  ref =%s" % np.round(act_ref[1, :3], 5))
    if diff_a / scale > 1e-5:
        print("   ❌ 策略输出不一致")
        fails += 1
    else:
        print("   ✅ 一致")

    # ---- ③ q_target 换算 ----
    print("\n ---- ③ q_target = q_default + 0.25*action ----")
    # ⚠️ `action_to_q_target` 返回【协议顺序】，而轨迹里的 q_target 是【训练顺序】
    #    ⇒ 必须换回来才能比较。这一步正好也验证了置换是自洽的。
    q_t_pred = np.stack([proto_to_train(Policy.action_to_q_target(a, clamp=False))
                         for a in act_ref])
    diff_q = float(np.max(np.abs(q_t_pred - q_t_ref)))
    print("   max|diff| = %.3e" % diff_q)
    if diff_q > 1e-5:
        print("   ❌ 换算不一致")
        fails += 1
    else:
        print("   ✅ 一致")

    print()
    print("=" * 74)
    if fails == 0:
        print(" ✅ 全部通过 —— rl_run.py 的观测拼装可以信任")
        print(" ⚠️ 唯一没验证的是 obs[0:3] 重力（轨迹没录四元数）；")
        print("    真机首次运行时请确认机身静止时它接近 [0,0,-1]。")
    else:
        print(" ❌ 有 %d 项失败 —— 不要上真机" % fails)
    print("=" * 74)
    return 0 if fails == 0 else 1


def main() -> int:
    ap = argparse.ArgumentParser(description="RL 策略推理层（纯 numpy）")
    ap.add_argument("--policy", default=os.path.join(_HERE, "models",
                                                    "policy_rlcat_flat.npz"))
    ap.add_argument("--trace", default=os.path.join(_HERE, "models",
                                                   "rlcat_flat_obs_action_trace.npz"))
    ap.add_argument("--selftest", action="store_true", help="对拍仿真轨迹")
    ap.add_argument("--info", action="store_true", help="只打印策略信息")
    args = ap.parse_args()

    if args.info:
        pol = Policy(args.policy)
        print("策略 %s" % pol.path)
        for i, (W, b) in enumerate(pol.layers):
            print("  %d: %d → %d  %s" % (i, W.shape[1], W.shape[0],
                                         "ELU" if i != pol.n_layers - 1 else "线性"))
        n = sum(W.size + b.size for W, b in pol.layers)
        print("  参数总数 = %d" % n)
        # 静止姿态下的重力自检（真机首次运行时的期望值）
        print("\n静止（单位四元数）时的重力观测应约等于 [0, 0, -1]：")
        print("  %s" % np.round(quat_to_gravity([1, 0, 0, 0]), 6))
        return 0

    if args.selftest:
        return selftest(args.trace, args.policy)

    ap.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())
