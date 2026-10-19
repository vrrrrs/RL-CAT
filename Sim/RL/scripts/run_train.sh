#!/usr/bin/env bash
# 训练启动器：统一环境 + 日志落盘 + 断点续训
#
# 用法:
#   bash scripts/run_train.sh rlcat_stand --num_envs 2048 --max_iterations 300
#   bash scripts/run_train.sh rlcat       --num_envs 2048 --max_iterations 1500
#   bash scripts/run_train.sh rlcat --resume          # 从最近检查点续训
#
# ⚠️ 不要加 set -u：conda 的 activate-binutils 脚本会引用未绑定的变量而报错
set +u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/env.sh"

TASK="${1:?用法: run_train.sh <task> [train.py 参数...]}"
shift

LOG_DIR="$RLCAT_ROOT/logs"
mkdir -p "$LOG_DIR"
TS="$(date +%Y%m%d_%H%M%S)"
LOG="$LOG_DIR/${TASK}_${TS}.log"

# ⚠️ 不再写死个人目录：legged_gym 位置由 env.sh 解析（可用 RLCAT_LEGGED_GYM 覆盖）
cd "${RLCAT_LEGGED_GYM:?请先 source scripts/env.sh，或设置 RLCAT_LEGGED_GYM}"
echo "[run] task=$TASK"
echo "[run] log=$LOG"
echo "[run] extra args: $*"
echo "[run] start $(date '+%F %T')"

# tee 保证既落盘又能实时看
python legged_gym/scripts/train.py --task="$TASK" --headless "$@" 2>&1 | tee "$LOG"
RC=${PIPESTATUS[0]}
echo "[run] exit=$RC  end $(date '+%F %T')"
exit "$RC"
