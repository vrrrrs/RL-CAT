"""从 tensorboard event 文件画训练曲线

用法:
    python scripts/10_plot_curves.py                      # 自动选最近的 rlcat_flat
    python scripts/10_plot_curves.py --run <运行目录> --out <输出png>
"""
import argparse
import glob
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# 中文字体：本机可用的是 Noto Sans CJK JP / Droid Sans Fallback，
# 不配的话标题会渲染成方框（tofu）
plt.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "Droid Sans Fallback",
                                   "WenQuanYi Zen Hei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rlcat_paths as P  # noqa: E402  ⭐ 统一路径解析，不写死个人目录

from tensorboard.backend.event_processing.event_accumulator import EventAccumulator


def load_scalars(run_dir):
    ev = glob.glob(os.path.join(run_dir, "events.out.tfevents.*"))
    if not ev:
        raise RuntimeError("没有 event 文件: %s" % run_dir)
    acc = EventAccumulator(sorted(ev)[-1], size_guidance={"scalars": 0})
    acc.Reload()
    out = {}
    for tag in acc.Tags().get("scalars", []):
        evs = acc.Scalars(tag)
        out[tag] = (np.array([e.step for e in evs]),
                    np.array([e.value for e in evs]))
    return out


def pick_run(explicit, exp="rlcat_flat"):
    if explicit:
        return explicit
    cands = sorted(glob.glob(os.path.join(P.LOGS_DIR, exp, "*", "")),
                   key=os.path.getmtime)
    if not cands:
        raise RuntimeError("找不到 %s 的运行目录" % exp)
    return cands[-1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", default=None)
    ap.add_argument("--exp", default=P.DEFAULT_EXPERIMENT)
    ap.add_argument("--out", default=os.path.join(P.PLOTS_DIR, "training_curves.png"))
    opt = ap.parse_args()

    run_dir = pick_run(opt.run, opt.exp)
    print("运行目录:", run_dir)
    data = load_scalars(run_dir)
    print("可用标量 tag (%d 个):" % len(data))
    for t in sorted(data):
        print("   ", t)

    def get(*names):
        for n in names:
            for t in data:
                if t == n or t.endswith("/" + n) or t.endswith(n):
                    return data[t]
        return None

    panels = [
        ("Train/mean_reward", ["mean_reward", "Mean reward"], "平均回报"),
        ("Train/mean_episode_length", ["mean_episode_length"], "平均 episode 长度 (步)"),
        ("Episode/rew_tracking_lin_vel", ["rew_tracking_lin_vel"], "线速度跟踪奖励"),
        ("Episode/rew_tracking_ang_vel", ["rew_tracking_ang_vel"], "角速度跟踪奖励"),
        ("Episode/rew_feet_air_time", ["rew_feet_air_time"], "抬脚时间奖励"),
        ("Episode/rew_orientation", ["rew_orientation"], "姿态惩罚"),
        ("Episode/rew_base_height", ["rew_base_height"], "机身高度惩罚"),
        ("Episode/rew_torques", ["rew_torques"], "力矩惩罚"),
    ]

    os.makedirs(os.path.dirname(opt.out), exist_ok=True)
    fig, axes = plt.subplots(4, 2, figsize=(14, 16))
    for ax, (tag, alts, title) in zip(axes.ravel(), panels):
        d = data.get(tag) or get(*alts)
        if d is None:
            ax.set_title(title + "（无数据）")
            ax.axis("off")
            continue
        s, v = d
        ax.plot(s, v, lw=1.6)
        ax.set_title(title)
        ax.set_xlabel("iteration")
        ax.grid(alpha=0.3)
    fig.suptitle("RL-CAT 行走训练曲线\n%s" % run_dir, fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(opt.out, dpi=110)
    print("\n已保存:", opt.out)


if __name__ == "__main__":
    main()
