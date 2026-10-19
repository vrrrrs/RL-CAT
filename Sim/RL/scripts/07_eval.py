"""RL-CAT 策略评估：数值指标 +（可选）录帧

评估项（对应验收标准）：
  1. 速度跟踪误差 |v_actual - v_cmd|
  2. 摔倒次数 / 存活率
  3. 机身高度、姿态（projected_gravity 的 z 分量）
  4. 步态对称性（左右脚触地占空比差异）

用法:
    # 纯数值评估（headless）
    python scripts/07_eval.py --task rlcat --num_envs 64 --seconds 20 \
        --speeds "0.0,0.2,0.4,-0.2"

    # 录帧（需要 DISPLAY，会用 viewer 抓图）
    python scripts/07_eval.py --task rlcat --num_envs 16 --seconds 8 \
        --speeds "0.3" --record          # --out 缺省 = $RLCAT_FRAMES（见 rlcat_paths.py）
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rlcat_paths as P  # noqa: E402  ⭐ 统一路径解析，不写死个人目录

import isaacgym  # noqa: F401  ⚠️ 必须在 torch 之前导入，否则 isaacgym 会拒绝加载
import torch
from legged_gym.envs import *  # noqa: F401,F403
from legged_gym.utils import get_args, task_registry


def build_args(task, num_envs, headless, seed=1):
    argv = [sys.argv[0], "--task", task, "--num_envs", str(num_envs),
            "--seed", str(seed)]
    if headless:
        argv.append("--headless")
    old = sys.argv
    sys.argv = argv
    try:
        return get_args()
    finally:
        sys.argv = old


def make(opt):
    args = build_args(opt.task, opt.num_envs, headless=not opt.record, seed=opt.seed)

    env_cfg, train_cfg = task_registry.get_cfgs(name=opt.task)
    env_cfg.env.num_envs = opt.num_envs
    env_cfg.noise.add_noise = False
    env_cfg.domain_rand.randomize_friction = False
    env_cfg.domain_rand.randomize_base_mass = False
    env_cfg.domain_rand.push_robots = False
    env_cfg.commands.curriculum = False
    env_cfg.commands.heading_command = False
    env_cfg.commands.resampling_time = 1.0e9      # 命令由我们手写，不让它重采样
    env_cfg.terrain.mesh_type = "plane"
    env_cfg.terrain.curriculum = False
    if opt.record:
        env_cfg.viewer.pos = [1.2, -1.2, 0.6]
        env_cfg.viewer.lookat = [0.0, 0.0, 0.12]

    env, _ = task_registry.make_env(name=opt.task, args=args, env_cfg=env_cfg)

    train_cfg.runner.resume = True
    if opt.checkpoint is not None:
        train_cfg.runner.checkpoint = opt.checkpoint
    if opt.load_run is not None:
        train_cfg.runner.load_run = opt.load_run
    ppo_runner, train_cfg = task_registry.make_alg_runner(
        env=env, name=opt.task, args=args, train_cfg=train_cfg)
    policy = ppo_runner.get_inference_policy(device=env.device)
    return env, policy, train_cfg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", default="rlcat")
    ap.add_argument("--num_envs", type=int, default=64)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--speeds", default="0.0,0.2,0.4")
    ap.add_argument("--load_run", default=None)
    ap.add_argument("--checkpoint", type=int, default=None)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--record", action="store_true")
    ap.add_argument("--out", default=P.EVAL_FRAMES_DIR)
    opt = ap.parse_args()

    env, policy, train_cfg = make(opt)

    # legged_gym 不把 body 名存到 self 上，只能从 actor 句柄取
    try:
        body_names = list(env.gym.get_actor_rigid_body_names(
            env.envs[0], env.actor_handles[0]))
    except Exception:
        body_names = None
    feet = list(env.feet_indices.cpu().numpy()) if hasattr(env, "feet_indices") else []
    if body_names:
        print("[信息] 足端刚体顺序 =", [body_names[i] for i in feet])
    print("[信息] 观测维度 = %d, 动作维度 = %d, 环境数 = %d"
          % (env.num_obs, env.num_actions, env.num_envs))

    obs = env.get_observations()
    n_steps = int(opt.seconds / env.dt)
    speeds = [float(s) for s in opt.speeds.split(",") if s.strip()]
    results = []

    if opt.record:
        os.makedirs(opt.out, exist_ok=True)
        print("[录制] 帧输出到", opt.out)

    for vx_cmd in speeds:
        # 仅第一个环境跟随被测速度，其余保持同一指令便于统计
        env.commands[:, 0] = vx_cmd
        env.commands[:, 1] = 0.0
        env.commands[:, 2] = 0.0
        env.commands[:, 3] = 0.0

        frame_dir = os.path.join(opt.out, "v%+.2f" % vx_cmd)
        if opt.record:
            os.makedirs(frame_dir, exist_ok=True)
            print("[录制] v_cmd=%+.2f -> %s" % (vx_cmd, frame_dir))

        sum_vx = 0.0
        sum_err = 0.0
        sum_h = 0.0
        sum_up = 0.0
        n = 0
        falls = 0
        duty = np.zeros(len(feet)) if len(feet) else None
        img = 0

        for step in range(n_steps):
            with torch.no_grad():
                actions = policy(obs.detach())
            obs, _, _, dones, _ = env.step(actions.detach())

            # 命令保持固定（覆盖 env 内部的重采样/heading 逻辑）
            env.commands[:, 0] = vx_cmd
            env.commands[:, 1] = 0.0
            env.commands[:, 2] = 0.0
            env.commands[:, 3] = 0.0

            vx = env.base_lin_vel[:, 0]
            sum_vx += float(vx.mean())
            sum_err += float((vx - vx_cmd).abs().mean())
            sum_h += float(env.root_states[:, 2].mean())
            # 机身 z 轴与重力方向的夹角：直立时 projected_gravity[2] ≈ -1
            sum_up += float((-env.projected_gravity[:, 2]).mean())
            # 摔倒 = 非超时的 reset
            falls += int((dones & ~env.time_out_buf).sum()) if hasattr(env, "time_out_buf") else int(dones.sum())
            if duty is not None:
                contact = (env.contact_forces[:, env.feet_indices, 2] > 1.0).float()
                duty += contact.mean(dim=0).cpu().numpy()
            n += 1

            if opt.record:
                # 相机跟随 0 号环境，否则机器人走出画面
                p = env.root_states[0, :3].cpu().numpy()
                off = np.array([0.9, -0.9, 0.45])
                env.set_camera(p + off, p + np.array([0.0, 0.0, 0.06]))
                env.render()
                if step % 4 == 0:
                    fn = os.path.join(frame_dir, "%05d.png" % img)
                    try:
                        env.gym.write_viewer_image_to_file(env.viewer, fn)
                        img += 1
                    except Exception as e:
                        if step == 0:
                            print("[警告] 抓帧失败:", e)

        r = dict(vx_cmd=vx_cmd, vx=sum_vx / n, err=sum_err / n,
                 height=sum_h / n, upright=sum_up / n,
                 falls=falls, steps=n * env.num_envs)
        if duty is not None:
            r["duty"] = (duty / n)
        results.append(r)

    print("\n================= 评估结果 =================")
    print("%8s %10s %12s %10s %10s %8s" %
          ("v_cmd", "v_actual", "|误差|", "机身高度", "直立度", "摔倒数"))
    for r in results:
        print("%8.2f %10.3f %12.4f %10.4f %10.3f %8d"
              % (r["vx_cmd"], r["vx"], r["err"], r["height"], r["upright"], r["falls"]))

    if len(feet) and "duty" in results[-1]:
        print("\n--- 步态对称性（各足触地占空比）---")
        if body_names:
            print("  足端顺序:", [body_names[i] for i in feet])
        for r in results:
            d = r["duty"]
            print("  v_cmd=%+.2f  duty=%s" % (r["vx_cmd"], np.round(d, 3)))
        # 左右对称性：按名字配对
        if body_names:
            names = [body_names[i] for i in feet]
            idx = {n_: k for k, n_ in enumerate(names)}
            pairs = [("lf_calf", "rf_calf"), ("lb_calf", "rb_calf")]
            for r in results:
                diffs = []
                for a, b in pairs:
                    if a in idx and b in idx:
                        diffs.append(abs(r["duty"][idx[a]] - r["duty"][idx[b]]))
                if diffs:
                    print("  v_cmd=%+.2f  左右占空比差 = %s (越小越对称)"
                          % (r["vx_cmd"], np.round(diffs, 3)))

    ok = all(r["err"] < 0.15 and r["falls"] == 0 and r["upright"] > 0.9 for r in results)
    print("\n[结论] %s" % ("全部速度指令达标" if ok
                        else "存在未达标项（见上表：误差/摔倒/直立度）"))


if __name__ == "__main__":
    main()
