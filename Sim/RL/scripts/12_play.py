"""看策略走路 —— 3D 实时窗口，相机自动跟随，命令自动轮换

比看录好的 mp4 好：可以自己拖视角、暂停、看细节。

用法:
    source scripts/env.sh                    # 必须先激活环境
    python scripts/12_play.py                # 用默认（最终交付模型）
    python scripts/12_play.py --num_envs 4   # 同时看 4 只
    python scripts/12_play.py --speeds "0.0,0.3,0.5,-0.2"
    python scripts/12_play.py --load_run Oct06_03-32-41_ --checkpoint 500

窗口里的按键:
    鼠标拖动 = 转视角    滚轮 = 缩放
    ESC      = 退出
"""
import argparse
import sys
import time

import numpy as np

import isaacgym  # noqa: F401  ⚠️ 必须在 torch 之前
import torch
from legged_gym.envs import *  # noqa: F401,F403
from legged_gym.utils import get_args, task_registry

DEFAULT_RUN = "Oct06_03-32-41_"   # 第 9 轮（全部验收项达标）
DEFAULT_CKPT = 500


def build_args(task, num_envs, seed=1):
    argv = [sys.argv[0], "--task", task, "--num_envs", str(num_envs),
            "--seed", str(seed)]           # 注意：不加 --headless，才会开窗口
    old = sys.argv
    sys.argv = argv
    try:
        return get_args()
    finally:
        sys.argv = old


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", default="rlcat")
    ap.add_argument("--num_envs", type=int, default=1)
    ap.add_argument("--load_run", default=DEFAULT_RUN)
    ap.add_argument("--checkpoint", type=int, default=DEFAULT_CKPT)
    ap.add_argument("--speeds", default="0.0,0.2,0.4,0.5,-0.2")
    ap.add_argument("--turn", type=float, default=0.6, help="轮换时的偏航角速度 [rad/s]")
    ap.add_argument("--secs_per_cmd", type=float, default=4.0)
    ap.add_argument("--seed", type=int, default=1)
    opt = ap.parse_args()

    args = build_args(opt.task, opt.num_envs, opt.seed)
    env_cfg, train_cfg = task_registry.get_cfgs(name=opt.task)
    env_cfg.env.num_envs = opt.num_envs
    env_cfg.noise.add_noise = False            # 演示时关掉噪声，看清楚真实行为
    env_cfg.domain_rand.randomize_friction = False
    env_cfg.domain_rand.push_robots = False
    env_cfg.commands.curriculum = False
    env_cfg.commands.heading_command = False
    env_cfg.commands.resampling_time = 1e9     # 命令由本脚本手写
    env_cfg.terrain.mesh_type = "plane"

    env, _ = task_registry.make_env(name=opt.task, args=args, env_cfg=env_cfg)

    train_cfg.runner.resume = True
    train_cfg.runner.load_run = opt.load_run
    train_cfg.runner.checkpoint = opt.checkpoint
    ppo_runner, _ = task_registry.make_alg_runner(
        env=env, name=opt.task, args=args, train_cfg=train_cfg)
    policy = ppo_runner.get_inference_policy(device=env.device)

    if env.viewer is None:
        print("[FAIL] 没能创建窗口。检查：是否在桌面会话里、DISPLAY 是否可用")
        return 1

    print("\n" + "=" * 62)
    print(" 正在演示模型: %s / model_%d.pt" % (opt.load_run, opt.checkpoint))
    print(" 鼠标拖动转视角，滚轮缩放，ESC 退出")
    print("=" * 62)

    speeds = [float(s) for s in opt.speeds.split(",") if s.strip()]
    # 命令序列：(vx, vy, wz, 说明)
    script = [(v, 0.0, 0.0, "前进 %.2f m/s" % v) for v in speeds]
    script.append((0.0, 0.0, opt.turn, "原地左转 %.1f rad/s" % opt.turn))
    script.append((0.2, 0.15, 0.0, "斜着走 (vx=0.2, vy=0.15)"))

    obs = env.get_observations()
    dt = env.dt
    steps_per_cmd = max(1, int(opt.secs_per_cmd / dt))
    idx = 0
    t0 = time.time()

    while not env.gym.query_viewer_has_closed(env.viewer):
        vx, vy, wz, label = script[idx % len(script)]
        env.commands[:, 0] = vx
        env.commands[:, 1] = vy
        env.commands[:, 2] = wz
        env.commands[:, 3] = 0.0

        for _ in range(steps_per_cmd):
            if env.gym.query_viewer_has_closed(env.viewer):
                break
            with torch.no_grad():
                actions = policy(obs.detach())
            obs, _, _, _, _ = env.step(actions.detach())
            # env 内部会重采样命令，这里每步覆盖回去
            env.commands[:, 0] = vx
            env.commands[:, 1] = vy
            env.commands[:, 2] = wz
            env.commands[:, 3] = 0.0

            # 相机跟随 0 号环境
            p = env.root_states[0, :3].cpu().numpy()
            env.set_camera(p + np.array([0.85, -0.85, 0.42]),
                           p + np.array([0.0, 0.0, 0.05]))
            env.render()

        v_act = env.base_lin_vel[0, 0].item()
        print("  [%-22s] 指令 %+.2f  实测 %+.2f  误差 %.3f  机身高度 %.3f"
              % (label, vx, v_act, abs(v_act - vx), env.root_states[0, 2].item()))
        idx += 1

    print("\n窗口已关闭。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
