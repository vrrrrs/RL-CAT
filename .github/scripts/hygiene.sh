#!/usr/bin/env bash
# ============================================================================
# 仓库卫生检查 —— 阻断"会让别人 clone 下来跑不起来"或泄露信息的东西
# ============================================================================
#  ⚠️ 为什么单独写成脚本而不是塞在 ci.yml 里：
#     ① 可以在本地直接跑（提交前自查），不用等 CI
#     ② 避免 YAML + shell + 正则三层转义叠加导致的"看起来对、实际不生效"
#        （第一版写在 ci.yml 里就用了 grep -E 不支持的负向先行断言）
#
#  用法:  bash .github/scripts/hygiene.sh
#  退出码: 0 = 通过, 1 = 有问题
# ============================================================================
set -uo pipefail
fail=0

# 已确认合规、不需要检查的文件（占位符说明 / 变更记录 / 本脚本自身）
EXCLUDES=(
  ':!docs/51*'          # 开源方案文档：里面引用占位符示例
  ':!.github/*'         # 本脚本与模板
  ':!CONTRIBUTING.md'   # 贡献指南：有 /home/<you>/ 的反例
  ':!CHANGELOG.md'
  ':!NOTICE.md'
  ':!*.eprj2'           # 立创 EDA 二进制工程（已验证无绝对路径）
  ':!*.epro2'
  ':!*.stl' ':!*.npz' ':!*.onnx' ':!*.pt'
)

echo "== ① 写死的个人绝对路径 =="
# 匹配 /home/<具体名字>/ 或 /Users/<具体名字>[/\\]
# ⚠️ 占位符形如 /home/<user>/ 会被下一步过滤掉（含尖括号）
hits=$(git grep -nE '/(home|Users)/[A-Za-z0-9_.-]+' -- "${EXCLUDES[@]}" 2>/dev/null \
        | grep -vE '<[A-Za-z][A-Za-z _-]*>' || true)
if [ -n "$hits" ]; then
  echo "$hits"
  echo "::error::发现写死的个人绝对路径 —— 别人 clone 下来会跑不起来"
  fail=1
else
  echo "  ✅ 无"
fi

echo "== ② 私钥 / 口令 =="
hits=$(git grep -nE 'BEGIN (RSA |OPENSSH |EC |DSA )?PRIVATE KEY' -- "${EXCLUDES[@]}" 2>/dev/null || true)
if [ -n "$hits" ]; then
  echo "$hits"
  echo "::error::发现私钥"
  fail=1
else
  echo "  ✅ 无私钥"
fi

echo "== ③ 单文件 > 5 MB（应该走 Release 附件）=="
big=""
while IFS= read -r f; do
  [ -f "$f" ] || continue
  sz=$(stat -c%s "$f" 2>/dev/null || stat -f%z "$f" 2>/dev/null || echo 0)
  if [ "$sz" -gt 5242880 ]; then
    printf '  ⚠️ %s  %s 字节\n' "$f" "$sz"
    big=1
  fi
done < <(git ls-files)
if [ -n "$big" ]; then
  echo "::error::有 >5MB 的文件进了 git"
  fail=1
else
  echo "  ✅ 无"
fi

echo "== ④ 中文乱码 / 编码损坏的文件名 =="
bad=$(git ls-files | grep -P '[\x80-\xff]' 2>/dev/null | grep -vP '^[\x{4e00}-\x{9fff}\x{3000}-\x{303f}\x{ff00}-\x{ffef}A-Za-z0-9_./-]+$' || true)
if [ -n "$bad" ]; then
  echo "$bad"
  echo "::warning::有非 UTF-8 的中文文件名（可能显示为乱码）"
else
  echo "  ✅ 无"
fi

echo
if [ "$fail" -eq 0 ]; then
  echo "✅ 卫生检查通过"
else
  echo "❌ 卫生检查未通过"
fi
exit "$fail"
