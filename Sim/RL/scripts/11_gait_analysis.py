"""步态分析：把策略跑出来的足端接触时序打成文本图，判断是真步态还是蹭地

输出：
  - 接触时序图（每行一个速度指令，列 = 时间，字符 = 哪几只脚着地）
  - 每只脚的触地占空比、平均滞空时间
  - 机身高度、俯仰、步频估计
  - 左右对称性评分

用法:
    python scripts/11_gait_analysis.py --task rlcat --checkpoint 650 --seconds 4
"""
import argparse
import sys

import numpy as np

import isaacgym  # noqa: F401  必须在 torch 之前
import torch
from legged_gym.envs import *  # noqa: F401,F403
from legged_gym.utils import get_args, task_registry

# 逻辑索引 A（交接文档 §2.2）：外摆0-3 / 髋俯仰4-7 / 膝8-11
A_NAMES = ["lf_hip", "lb_hip", "rf_hip", "rb_hip",
           "lf_thigh", "lb_thigh", "rf_thigh", "rb_thigh",
           "lf_calf", "lb_calf", "rf_calf", "rb_calf"]


def build_args(task, num_envs, seed=1):
    argv = [sys.argv[0], "--task", task, "--num_envs", str(num_envs),
            "--headless", "--seed", str(seed)]
    old = sys.argv
    sys.argv = argv
    try:
        return get_args()
    finally:
        sys.argv = old


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", default="rlcat")
    ap.add_argument("--num_envs", type=int, default=8)
    ap.add_argument("--checkpoint", type=int, default=650)
    ap.add_argument("--load_run", default=None)
    ap.add_argument("--speeds", default="0.3")
    ap.add_argument("--seconds", type=float, default=4.0)
    ap.add_argument("--env_index", type=int, default=0)
    opt = ap.parse_args()

    args = build_args(opt.task, opt.num_envs)
    env_cfg, train_cfg = task_registry.get_cfgs(name=opt.task)
    env_cfg.env.num_envs = opt.num_envs
    env_cfg.noise.add_noise = False
    env_cfg.domain_rand.randomize_friction = False
    env_cfg.domain_rand.push_robots = False
    env_cfg.commands.curriculum = False
    env_cfg.commands.heading_command = False
    env_cfg.commands.resampling_time = 1.0e9
    env_cfg.terrain.mesh_type = "plane"
    env, _ = task_registry.make_env(name=opt.task, args=args, env_cfg=env_cfg)

    train_cfg.runner.resume = True
    train_cfg.runner.checkpoint = opt.checkpoint
    if opt.load_run is not None:
        train_cfg.runner.load_run = opt.load_run
    ppo_runner, _ = task_registry.make_alg_runner(
        env=env, name=opt.task, args=args, train_cfg=train_cfg)
    policy = ppo_runner.get_inference_policy(device=env.device)

    # DOF 名称与顺序（Isaac 顺序 C）
    dof_names = list(env.dof_names)
    print("[信息] Isaac DOF 顺序:", dof_names)
    print("[信息] 足端索引:", env.feet_indices.cpu().numpy().tolist())
    try:
        print("[信息] 刚体名:", env.gym.get_actor_rigid_body_names(env.envs[0], env.actor_handles[0]))
    except Exception:
        pass

    obs = env.get_observations()
    n_steps = int(opt.seconds / env.dt)
    ei = opt.env_index

    for vx_cmd in [float(s) for s in opt.speeds.split(",") if s.strip()]:
        env.commands[:, 0] = vx_cmd
        env.commands[:, 1] = 0.0
        env.commands[:, 2] = 0.0

        contacts = []
        heights = []
        pitches = []
        dof_hist = []
        for _ in range(n_steps):
            with torch.no_grad():
                a = policy(obs.detach())
            obs, _, _, _, _ = env.step(a.detach())
            env.commands[:, 0] = vx_cmd
            env.commands[:, 1] = 0.0
            env.commands[:, 2] = 0.0
            c = (env.contact_forces[ei, env.feet_indices, 2] > 1.0).cpu().numpy()
            contacts.append(c)
            heights.append(float(env.root_states[ei, 2]))
            pitches.append(float(env.projected_gravity[ei, 2]))
            dof_hist.append(env.dof_pos[ei].cpu().numpy())

        contacts = np.array(contacts)              # (T, 4)
        heights = np.array(heights)
        pitches = np.array(pitches)
        dof_hist = np.array(dof_hist)              # (T, 12)

        print("\n" + "=" * 78)
        print("v_cmd = %+.2f   （环境 #%d）" % (vx_cmd, ei))
        print("=" * 78)

        # 接触时序图：每 2 步一列，字符表示哪几只脚着地
        print("\n接触时序（. = 全腾空, 数字/字母 = 着地脚的组合）:")
        step = 2
        symbols = {}
        line = ""
        for t in range(0, len(contacts), step):
            c = contacts[t]
            key = tuple(np.where(c)[0].tolist())
            if key not in symbols:
                symbols[key] = "." if not key else str(len(symbols) - 1)
            line += symbols[key]
        for i in range(0, len(line), 60):
            print("   t=%.2fs  %s" % (i * step * env.dt, line[i:i + 60]))
        print("   图例:", {v: ("腾空" if k == () else "脚" + str(list(k))) for k, v in symbols.items()})

        duty = contacts.mean(axis=0)
        print("\n各脚触地占空比:", np.round(duty, 3), " (和=%.2f)" % duty.sum())
        print("机身高度 mean=%.4f  min=%.4f  max=%.4f" % (heights.mean(), heights.min(), heights.max()))
        print("直立度 (proj_grav z, -1=完全直立) mean=%.4f" % pitches.mean())

        # 平均同时着地脚数
        n_contact = contacts.sum(axis=1)
        print("同时着地脚数: mean=%.2f  分布=%s"
              % (n_contact.mean(), {int(k): int(v) for k, v in zip(*np.unique(n_contact, return_counts=True))}))

        # 主导频率（对总接触数做 FFT）
        sig = contacts.sum(axis=1) - contacts.sum(axis=1).mean()
        if np.abs(sig).sum() > 1e-6:
            F = np.fft.rfft(sig)
            freqs = np.fft.rfftfreq(len(sig), d=env.dt)
            k = int(np.argmax(np.abs(F[1:])) + 1)
            print("主导步频 ≈ %.2f Hz (周期 %.2f s)" % (freqs[k], 1.0 / max(freqs[k], 1e-6)))

        # 关节使用情况：按 A 逻辑索引报告与 0 位（站立）的平均偏差
        dev = np.abs(dof_hist).mean(axis=0)
        order = [dof_names.index(n) for n in A_NAMES]
        print("\n各关节平均偏离站立位(rad) [按逻辑索引 A]:")
        for grp, rng in [("外摆 hip   ", range(0, 4)), ("髋俯仰 thigh", range(4, 8)), ("膝 calf    ", range(8, 12))]:
            vals = [dev[order[a]] for a in rng]
            print("   %s: %s  mean=%.3f" % (grp, np.round(vals, 3), np.mean(vals)))


if __name__ == "__main__":
    main()
