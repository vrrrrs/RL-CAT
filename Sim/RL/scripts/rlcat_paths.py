#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""统一的路径解析 —— **不依赖任何写死的个人目录**。

⚠️ 为什么需要这个模块（而不是在每个脚本里写字面量路径）：

    原来 11 个脚本里都写着 `/home/<user>/Desktop/RL/urdf` 这类绝对路径。
    换一台机器 / 换一个用户名，**所有脚本立刻全部失效**，而且报错是
    "asset not found"这种和真正原因无关的提示。

    ⇒ 这是开源项目最常见的死因：别人 clone 下来跑不起来，
      而且不会提 issue，直接关掉页面。

⭐ 解析优先级（两者都可覆盖，所以本地开发和 CI 都能用）：

    ① 环境变量（RLCAT_*）      ← 换机器时只设变量，不改代码
    ② 相对本文件的缺省推导     ← 什么都不设也能跑（只要仓库结构完整）

⚠️ 本模块**只依赖标准库**，可以在任何 Python 3.6+ 下 import，
   包括没有装 isaacgym 的环境（这样 `--help` 也能用）。
"""

from __future__ import annotations

import os

# ---------------------------------------------------------------------------
# 目录锚点：从本文件位置反推，而不是写死
# ---------------------------------------------------------------------------

SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
#   .../Sim/RL/scripts  →  .../Sim/RL
SIM_RL_DIR = os.path.dirname(SCRIPTS_DIR)
#   .../Sim/RL          →  .../      （仓库根）
REPO_ROOT = os.path.dirname(os.path.dirname(SIM_RL_DIR))


def _resolve(env_name: str, default: str) -> str:
    """环境变量优先；否则用缺省值。都做 ~ 展开与绝对化。"""
    raw = os.environ.get(env_name) or default
    return os.path.abspath(os.path.expanduser(raw))


# ---------------------------------------------------------------------------
# 常用目录
# ---------------------------------------------------------------------------

#: 训练工作根目录（含 urdf/ eval_frames/ videos/ plots/）
RLCAT_ROOT = _resolve("RLCAT_ROOT", SIM_RL_DIR)

#: 仿真用 URDF 与网格
URDF_DIR = _resolve("RLCAT_URDF", os.path.join(RLCAT_ROOT, "urdf"))

#: 评估录帧的输出目录
EVAL_FRAMES_DIR = _resolve("RLCAT_FRAMES", os.path.join(RLCAT_ROOT, "eval_frames"))

#: 视频输出目录
VIDEOS_DIR = _resolve("RLCAT_VIDEOS", os.path.join(RLCAT_ROOT, "videos"))

#: 曲线图输出目录
PLOTS_DIR = _resolve("RLCAT_PLOTS", os.path.join(RLCAT_ROOT, "plots"))

#: legged_gym 仓库（训练框架）
LEGGED_GYM_DIR = _resolve("RLCAT_LEGGED_GYM",
                          os.path.join(REPO_ROOT, "Sim", "legged_gym"))

#: 训练日志根目录（每个实验一个子目录）
LOGS_DIR = _resolve("RLCAT_LOGS", os.path.join(LEGGED_GYM_DIR, "logs"))

#: isaacgym 的 python 包目录（⚠️ 需自行向 NVIDIA 申请，本仓库不含）
ISAACGYM_PY_DIR = _resolve(
    "RLCAT_ISAACGYM",
    os.path.join(os.path.expanduser("~"), "isaacgym_ws", "isaacgym", "python", "isaacgym"),
)

#: 默认的实验名（本项目的平地环境）
DEFAULT_EXPERIMENT = os.environ.get("RLCAT_EXP", "rlcat_flat")


def ensure_dir(path: str) -> str:
    """确保目录存在并返回它 —— 写输出前调用，避免"目录不存在"的半途失败。"""
    os.makedirs(path, exist_ok=True)
    return path


def describe() -> str:
    """一行打印当前解析结果，方便排查"为什么找不到资源"。"""
    return (
        "RLCAT_ROOT={RLCAT_ROOT}\n"
        "  URDF_DIR        ={URDF_DIR}\n"
        "  LEGGED_GYM_DIR  ={LEGGED_GYM_DIR}\n"
        "  LOGS_DIR        ={LOGS_DIR}\n"
        "  EVAL_FRAMES_DIR ={EVAL_FRAMES_DIR}\n"
        "  VIDEOS_DIR      ={VIDEOS_DIR}\n"
        "  ISAACGYM_PY_DIR ={ISAACGYM_PY_DIR}"
    ).format(**globals())


if __name__ == "__main__":
    # 直接运行本文件 = 打印当前解析结果（排查路径问题第一步）
    print(describe())
