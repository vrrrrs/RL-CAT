#!/usr/bin/env bash
# ============================================================================
# RL-CAT 固件主机单测（POSIX 版）—— 与 test-host.ps1 等价
# ============================================================================
#  ⚠️ 为什么需要两份：
#     Windows 上开发用 test-host.ps1；CI（ubuntu-latest）没有 PowerShell 的
#     那套 gcc 调用习惯，所以这里给一份纯 POSIX 的等价实现。
#
#  ⚠️ 两边的编译参数与用例/源文件对应关系必须【保持一致】，
#     否则"本地绿、CI 红"会变成一个长期的噪声源。
#     改动这里的 suites 表时，请同步改 test-host.ps1 第 83-91 行。
#
#  用法:  bash firmware/tools/test-host.sh
#  退出码: 0 = 全绿
# ============================================================================
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
FW_ROOT="$(cd "$HERE/.." && pwd)"
SRC_DIR="$FW_ROOT/stm32/Core/Src"
INC_DIR="$FW_ROOT/stm32/Core/Inc"
TEST_DIR="$FW_ROOT/tests/host"
OUT_DIR="${OUT_DIR:-$(mktemp -d)}"

CC="${CC:-gcc}"
command -v "$CC" >/dev/null 2>&1 || { echo "❌ 找不到编译器: $CC" >&2; exit 1; }

# ---- 纯逻辑源（与 HAL 无关，全部用例共用）----
PURE=(
  rlca_crc16.c rlca_frame.c rlca_phase.c rlca_math.c rlca_sts.c
  link_proto.c link_pi.c battery.c imu.c rlca_safety.c rlca_calib_proc.c
)
SERVO=(servo_bus.c)
APPMODE=(app_mode.c app_safety.c)
STUB="$TEST_DIR/host_link_port_stub.c"

# ---- 编译参数：与 test-host.ps1 的 $common 保持一致 ----
COMMON=(-std=gnu11 -Wall -Wextra -Werror -O1 -g -Wno-unused-parameter "-I$INC_DIR")

# ---- 用例表：name|test file|源集合（pure / servo / appmode）----
#   ⚠️ 顺序与 test-host.ps1 的 $suites 一致
SUITES=(
  "pure_logic|test_pure_logic.c|pure"
  "servo_bus|test_servo_bus.c|servo"
  "link_proto|test_link_proto.c|pure"
  "link_pi|test_link_pi.c|pure"
  "sensors|test_sensors.c|pure"
  "safety_calib|test_safety_calib.c|pure"
  "app_mode|test_app_mode.c|appmode"
)

echo "== RL-CAT host unit tests (POSIX) =="
echo "gcc : $("$CC" --version | head -1)"
echo

fail=0
for row in "${SUITES[@]}"; do
    IFS='|' read -r name test_file kind <<< "$row"

    srcs=()
    for f in "${PURE[@]}"; do srcs+=("$SRC_DIR/$f"); done
    case "$kind" in
        servo)   for f in "${SERVO[@]}";   do srcs+=("$SRC_DIR/$f"); done ;;
        appmode) for f in "${APPMODE[@]}"; do srcs+=("$SRC_DIR/$f"); done ;;
    esac
    srcs+=("$STUB")

    # 源文件必须都存在（早失败、给明确原因）
    for f in "${srcs[@]}" "$TEST_DIR/$test_file"; do
        [ -f "$f" ] || { echo "❌ 缺文件: $f" >&2; exit 1; }
    done

    exe="$OUT_DIR/test_$name"
    echo "--- $name ---"
    if ! "$CC" "${COMMON[@]}" -o "$exe" "$TEST_DIR/$test_file" "${srcs[@]}" -lm; then
        echo "  ❌ BUILD FAILED: $name"
        fail=$((fail + 1))
        continue
    fi
    if ! "$exe"; then
        echo "  ❌ TESTS FAILED: $name"
        fail=$((fail + 1))
    fi
    echo
done

if [ "$fail" -ne 0 ]; then
    echo "❌ $fail 套失败"
    exit 1
fi
echo "✅ ALL HOST TEST SUITES PASSED"
exit 0
