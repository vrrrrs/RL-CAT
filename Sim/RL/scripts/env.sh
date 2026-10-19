#!/usr/bin/env bash
# ============================================================================
# RL-CAT 训练环境统一入口 —— 所有训练/测试脚本都先 source 这个
# ============================================================================
#
#  ⚠️ 这个文件解决的是"无 sudo 条件下装 Isaac Gym 环境"踩出来的 4 个坑：
#     1. 系统没有 C++ 编译器        -> conda-forge gxx_linux-64=10 / gcc_linux-64=10
#     2. isaacgym 用了 np.float     -> scripts/06_patch_isaacgym_numpy.py 打过补丁
#     3. sysroot 缺 crypt.h         -> conda-forge libxcrypt + CPATH 指到 $CONDA_PREFIX/include
#     4. CUDA toolkit 没进 PATH     -> /usr/local/cuda-11.8
#
#  ⭐ 用法（**不再依赖任何写死的个人路径**，全部可用环境变量覆盖）:
#
#       # 1) 先按自己的实际情况设一次（或写进 ~/.bashrc）
#       export RLCAT_ROOT=~/RL-CAT/Sim/RL          # 本目录（含 scripts/ urdf/）
#       export RLCAT_CONDA=~/miniconda              # conda 安装位置
#       export RLCAT_ENV=isaacgym                   # conda 环境名
#       export RLCAT_CUDA=/usr/local/cuda           # CUDA toolkit
#
#       # 2) 每次开新终端
#       source scripts/env.sh
#
#  ⇒ 若一个都不设，下面的缺省值会用"相对于本脚本"的路径推导，
#    只要仓库结构完整就能直接跑。
# ============================================================================

# ---- ① RLCAT_ROOT：从本脚本位置反推，缺省 = scripts/ 的上一级 ----
#   ⚠️ 用 BASH_SOURCE 反推而不是写死路径 —— 换机器/换用户名都不用改文件
_RLCAT_SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
export RLCAT_ROOT="${RLCAT_ROOT:-$(cd "${_RLCAT_SCRIPTS_DIR}/.." && pwd)}"

# ---- ② conda 位置与环境名 ----
export RLCAT_CONDA="${RLCAT_CONDA:-$HOME/miniconda}"
export RLCAT_ENV="${RLCAT_ENV:-isaacgym}"

# ---- ③ legged_gym 位置（训练框架；可由 RLCAT_LEGGED_GYM 覆盖）----
#   ⚠️ 缺省 = 仓库根下的 Sim/legged_gym，由 RLCAT_ROOT 反推
export RLCAT_LEGGED_GYM="${RLCAT_LEGGED_GYM:-$(cd "${RLCAT_ROOT}/../legged_gym" 2>/dev/null && pwd || echo "${RLCAT_ROOT}/../legged_gym")}"

# ---- ④ CUDA toolkit ----
export RLCAT_CUDA="${RLCAT_CUDA:-/usr/local/cuda}"

# --- conda ---
if [ -f "${RLCAT_CONDA}/etc/profile.d/conda.sh" ]; then
    # shellcheck disable=SC1091
    source "${RLCAT_CONDA}/etc/profile.d/conda.sh"
else
    echo "[env] ⚠️ 找不到 ${RLCAT_CONDA}/etc/profile.d/conda.sh" >&2
    echo "[env]    请设置 RLCAT_CONDA 指向你的 conda 安装根目录" >&2
fi

if command -v conda >/dev/null 2>&1; then
    conda activate "${RLCAT_ENV}" 2>/dev/null || {
        echo "[env] ⚠️ conda 环境 '${RLCAT_ENV}' 激活失败" >&2
        echo "[env]    可用环境: $(conda env list 2>/dev/null | awk '!/^#/ && NF {print $1}' | tr '\n' ' ')" >&2
        echo "[env]    用 RLCAT_ENV=<名字> 指定" >&2
    }
fi

# --- CUDA toolkit（gymtorch 编译与运行需要）---
export CUDA_HOME="${RLCAT_CUDA}"
export PATH="${CUDA_HOME}/bin:$PATH"

# --- C++ 头文件：让 Python.h 能找到 crypt.h ---
export CPATH="${CONDA_PREFIX:-}/include${CPATH:+:$CPATH}"

# --- 编译并行度：别把机器全占满 ---
export MAX_JOBS="${MAX_JOBS:-8}"

echo "[env] RLCAT_ROOT=$RLCAT_ROOT"
echo "[env] conda=$(basename "${CONDA_PREFIX:-none}")  python=$(python -V 2>&1 | awk '{print $2}')"
echo "[env] LEGGED_GYM=$RLCAT_LEGGED_GYM"
echo "[env] CUDA_HOME=$CUDA_HOME  MAX_JOBS=$MAX_JOBS"
