#!/usr/bin/env bash
# RL-CAT 树莓派侧：一键跑完所有【不需要硬件】的检查
#
# 用法：
#     cd ~/RL-CAT/pi && ./run_checks.sh
#
# ⚠️ 需要硬件的两项不在这里（见 pi/README.md §4）：
#     tools/uart_loopback.py    需要短接 GPIO14↔GPIO15
#     pi_hello.py               需要真的 STM32

set -u
cd "$(dirname "$0")"

FAIL=0
run() {
    local name="$1"; shift
    echo
    echo "############################## $name ##############################"
    if "$@"; then
        :
    else
        echo "  !!! $name 失败（退出码 $?）"
        FAIL=1
    fi
}

run "config.py     关节表/限位/夹紧"      python3 config.py
run "stats.py      有界统计"               python3 stats.py
run "link.py       链路层自检（PTY）"      python3 link.py
run "selfcheck.py  协议层（固件交付方）"   python3 selfcheck.py
run "test_loopback 模拟器回环（交付方）"   python3 test_loopback.py
run "离线端到端    链路/遥测/模式/标定"    python3 tests/test_basic_offline.py

# ⭐ RL 策略层：对拍仿真轨迹（2026-10-18 加）
#    ⚠️ 这是唯一能证明"真机 47 维观测与训练一致"的手段。
#       它用 pi/models/ 下的轨迹重建 obs，与录制值逐位对比（误差必须为 0），
#       再跑策略前向对比录制 action。
#    ⚠️ 需要 pi/models/ 下的两个 .npz；缺文件时本项会失败 —— 那是正确的，
#       因为"没有策略"不应该被当成"检查通过"。
run "rl_policy.py  策略层对拍仿真轨迹"     python3 rl_policy.py --selftest

echo
echo "======================================================================"
if [ "$FAIL" -eq 0 ]; then
    echo " 全部通过"
else
    echo " 有失败项，见上面"
fi
echo "======================================================================"
exit "$FAIL"
