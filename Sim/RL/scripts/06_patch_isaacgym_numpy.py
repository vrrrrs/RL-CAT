"""Isaac Gym Preview 4 与新 numpy (>=1.24) 的兼容补丁

问题：isaacgym/torch_utils.py 里用了 numpy 1.24 已移除的别名 `np.float`，
      导致 `import isaacgym.torch_utils` 直接 AttributeError，整个训练栈起不来。

范围：全 isaacgym python 源码扫描，只有 1 处（torch_utils.py:135）。

修法：`np.float` -> `float`（语义完全等价，np.float 就是内置 float 的别名）。
      保留 numpy 1.24.4 不动，避免降级影响其他包。

幂等：重复运行安全。若 isaacgym 被重装（pip install -e .）导致补丁丢失，重跑本脚本即可。

用法:
    python scripts/06_patch_isaacgym_numpy.py [--check-only]
"""
import argparse
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rlcat_paths as P  # noqa: E402  ⭐ 统一路径解析，不写死个人目录

# ⚠️ 不再写死个人目录：用 rlcat_paths 的解析结果，并额外允许手动指定
CANDIDATES = [
    os.path.join(P.ISAACGYM_PY_DIR, "torch_utils.py"),
]
if os.environ.get("RLCAT_ISAACGYM_TORCH_UTILS"):
    CANDIDATES.insert(0, os.environ["RLCAT_ISAACGYM_TORCH_UTILS"])

PATTERN = re.compile(r"(?<![\w.])np\.float(?![\w])")


def find_target():
    for p in CANDIDATES:
        if os.path.isfile(p):
            return p
    # 退路：从已安装的 isaacgym 包定位
    try:
        import isaacgym
        p = os.path.join(os.path.dirname(isaacgym.__file__), "torch_utils.py")
        if os.path.isfile(p):
            return p
    except Exception:
        pass
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check-only", action="store_true")
    args = ap.parse_args()

    path = find_target()
    if path is None:
        print("[FAIL] 找不到 isaacgym/torch_utils.py")
        return 1

    print("目标文件:", path)
    with open(path, "r", encoding="utf-8") as f:
        src = f.read()

    hits = [(i + 1, l) for i, l in enumerate(src.splitlines()) if PATTERN.search(l)]
    if not hits:
        print("[OK] 没有 np.float，无需修补（已打过或已是新版）")
        return 0

    print("发现 %d 处 np.float:" % len(hits))
    for ln, line in hits:
        print("  %d: %s" % (ln, line.strip()))

    if args.check_only:
        print("\n[--check-only] 未修改")
        return 0

    new = PATTERN.sub("float", src)
    # 备份一次，方便回滚
    bak = path + ".numpy124.bak"
    if not os.path.exists(bak):
        with open(bak, "w", encoding="utf-8") as f:
            f.write(src)
        print("已备份:", bak)
    with open(path, "w", encoding="utf-8") as f:
        f.write(new)
    print("[OK] 已修补 %d 处 np.float -> float" % len(hits))
    return 0


if __name__ == "__main__":
    sys.exit(main())
