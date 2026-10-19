#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 rsl_rl 的 .pt 检查点导出成【两种】可移植格式，并做三方交叉验证。

产物
----
1. `.onnx` —— **模型发布格式**：
     标准格式，Netron 可直接打开看结构；C++ / 网页 / MATLAB 都能加载。
     进仓库，让别人能独立验证"策略到底是不是这个"。

2. `.npz`  —— **树莓派运行时格式**：
     纯 numpy 权重。Pi 侧前向只要 ~32 µs（x86；Pi 5 约 65~97 µs），
     零框架依赖、无线程池、每层可打印。

为什么 Pi 上不用 onnxruntime（详见 docs/45-linux-tasks.md §2.2）
--------------------------------------------------------------
    · 这个网络只有 54220 个参数，numpy 前向 32 µs = 10 ms 预算的 0.3%
      ⇒ "哪个更快"不是决策依据
    · onnxruntime aarch64 装完 30~50 MB、import 0.3~1 s
    · ⚠️ ORT 默认按 CPU 核数开线程，在 Pi 5（4 核）上会和 100 Hz 控制循环
      抢 CPU，产生延迟毛刺 —— 这是真实的实时性风险
    ⇒ Pi 上跑 numpy；ONNX 只作为发布/共享格式

为什么需要 Linux
----------------
    需要 torch（开发机 Windows 上没有装）。

用法
----
    # 最常用：两种格式都出，并做三方验证
    python3 tools/export_policy.py \\
        Sim/legged_gym/logs/rlcat_flat/Oct06_03-15-34_/model_1125.pt \\
        -o pi/models/policy_rlcat_flat

    # 只要 npz（不想装 onnxruntime 时）
    python3 tools/export_policy.py <ckpt> -o <base> --no-onnx

    # 只要 onnx
    python3 tools/export_policy.py <ckpt> -o <base> --no-npz

⚠️ 三方验证 = `torch ≡ onnx ≡ numpy`。任何一方写错都会被抓到。
   如果连 rsl_rl 也 import 得到，会额外做一次"用 rsl_rl 真实 ActorCritic
   重建"的校验 —— 那是唯一**不循环**的检查（前面的三方都基于"层顺序假设"）。

退出码：0 成功；1 失败（含验证不一致）
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

# ⚠️ torch 延迟导入（2026-10-18）：
#    原来在模块顶层 import，导致 `--help` 在没有 torch 的机器上也会失败。
#    CLI 的 --help 必须永远可用 —— 那正是"这台机器没装 torch 我该怎么办"时
#    最先会敲的命令。
torch = None          # type: ignore[assignment]
nn = None             # type: ignore[assignment]


def require_torch() -> None:
    """真正要用 torch 时才导入；缺了就给出可执行的下一步。"""
    global torch, nn
    if torch is not None:
        return
    try:
        import torch as _t
        import torch.nn as _nn
        torch, nn = _t, _nn
    except ImportError:
        raise SystemExit(
            "❌ 需要 torch，但本机没装。\n"
            "   这是 Linux 侧任务，见 docs/45-linux-tasks.md。\n"
            "   ⚠️ 装 CPU 版就够（约 200 MB，不要装 CUDA 版 2 GB+）：\n"
            "       pip3 install --index-url https://download.pytorch.org/whl/cpu torch\n"
            "   ⚠️ Windows 上可以用 WSL（同一份文档 §7.1）。")


# ============================================================================
# 1. 加载检查点
# ============================================================================

def load_state_dict(path: str) -> dict:
    """读 rsl_rl 的 .pt，返回 model_state_dict。

    ⚠️ torch >= 2.6 的 `torch.load` 默认 `weights_only=True`，而 rsl_rl 的
       检查点除了权重还塞了 `infos`（含字符串/字典），会被拒。
       ⇒ 先试 weights_only=False；老版本 torch 没有这个参数，用 TypeError 兜。
    """
    try:
        ck = torch.load(path, map_location="cpu", weights_only=False)
    except TypeError:
        ck = torch.load(path, map_location="cpu")
    except Exception as e:                                  # noqa: BLE001
        raise SystemExit("❌ torch.load 失败：%s" % e)

    if isinstance(ck, dict) and "model_state_dict" in ck:
        return ck["model_state_dict"]
    if isinstance(ck, dict) and any(k.startswith("actor.") for k in ck):
        return ck
    raise SystemExit("❌ 这个文件里找不到 model_state_dict / actor.* —— 不是 rsl_rl 检查点？")


def extract_actor_mlp(sd: dict):
    """按顺序取出 actor 的所有 Linear 层，返回 [(W, b), ...]。

    rsl_rl 的 `ActorCritic.actor` 是 nn.Sequential：
        0: Linear, 1: 激活, 2: Linear, 3: 激活, ...
    ⇒ 线性层下标 0, 2, 4, ...，键名形如 `actor.0.weight`。
    """
    idx = sorted({int(k.split(".")[1])
                  for k in sd
                  if k.startswith("actor.") and k.endswith(".weight")})
    if not idx:
        raise SystemExit("❌ state_dict 里没有 actor.* 权重")

    layers = []
    for i in idx:
        wk, bk = "actor.%d.weight" % i, "actor.%d.bias" % i
        if wk not in sd or bk not in sd:
            raise SystemExit("❌ 缺少 %s 或 %s" % (wk, bk))
        W = sd[wk].detach().cpu().numpy().astype(np.float32)
        b = sd[bk].detach().cpu().numpy().astype(np.float32)
        layers.append((W, b))
    return layers


# ============================================================================
# 2. numpy 前向（必须与 pi/rl_policy.py 逐行一致）
# ============================================================================

def elu_np(x: np.ndarray) -> np.ndarray:
    return np.where(x > 0.0, x, np.expm1(np.minimum(x, 0.0)))


def forward_numpy(layers, x: np.ndarray) -> np.ndarray:
    """最后一个线性层之后【不】过激活（策略输出是原始 action）。"""
    h = np.asarray(x, dtype=np.float32)
    for n, (W, b) in enumerate(layers):
        h = h @ W.T + b
        if n != len(layers) - 1:
            h = elu_np(h)
    return h


# ============================================================================
# 3. torch 侧：从 state_dict 重建同构 MLP
# ============================================================================

def build_torch_mlp(layers):
    """按"Linear + ELU(除最后一层)"重建一个 nn.Sequential。"""
    mods = []
    for n, (W, b) in enumerate(layers):
        lin = nn.Linear(W.shape[1], W.shape[0])
        with torch.no_grad():
            lin.weight.copy_(torch.from_numpy(W))
            lin.bias.copy_(torch.from_numpy(b))
        mods.append(lin)
        if n != len(layers) - 1:
            mods.append(nn.ELU())
    m = nn.Sequential(*mods).eval()
    for p in m.parameters():
        p.requires_grad_(False)
    return m


# ============================================================================
# 4. 导出 ONNX
# ============================================================================

def export_onnx(model, path: str, obs_dim: int) -> bool:
    """导出单输入单输出 ONNX。返回是否成功。

    ⚠️ opset 依次尝试 17 / 13 / 11 —— 老版本 torch 可能不支持较新的 opset，
       而 MLP 只用 Gemm + Elu，三种 opset 都安全。
    """
    dummy = torch.zeros(1, obs_dim, dtype=torch.float32)
    last_err = None
    for opset in (17, 13, 11):
        try:
            torch.onnx.export(
                model, dummy, path,
                input_names=["obs"], output_names=["action"],
                opset_version=opset,
                do_constant_folding=True,
                dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}},
            )
            print("   ONNX 导出成功（opset %d）" % opset)
            return True
        except Exception as e:                              # noqa: BLE001
            last_err = e
    print("   ❌ ONNX 导出失败：%s" % last_err)
    return False


# ============================================================================
# 5. 三方验证
# ============================================================================

def verify_three_way(layers, torch_model, onnx_path, obs_dim: int,
                     n_samples: int = 5, rtol: float = 1e-5) -> bool:
    """在若干随机输入上比较 torch / onnx / numpy 的输出。

    ⚠️ 容差用**相对误差**而不是绝对误差：策略输出的量级因输入而异，
       固定绝对阈值要么太松要么太严。
    ⚠️ ORT 的图优化会改变浮点结果（~1e-6 量级），所以不能要求逐位相等。
    """
    print("\n ---- 三方一致性验证（torch / onnx / numpy）----")

    sess = None
    if onnx_path and os.path.isfile(onnx_path):
        try:
            import onnxruntime as ort
            so = ort.SessionOptions()
            # ⚠️ 单线程、顺序执行：与真机上的配置保持一致
            so.intra_op_num_threads = 1
            so.inter_op_num_threads = 1
            so.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
            sess = ort.InferenceSession(onnx_path, so,
                                        providers=["CPUExecutionProvider"])
            # 预热（第一次调用会分配内存并可能触发图优化）
            for _ in range(10):
                sess.run(None, {"obs": np.zeros((1, obs_dim), np.float32)})
        except ImportError:
            print("   ⚠️ 没装 onnxruntime ⇒ 跳过 onnx 对比")
            print("      装：pip3 install onnxruntime")
        except Exception as e:                              # noqa: BLE001
            print("   ⚠️ onnxruntime 加载失败：%s" % e)
    else:
        print("   （没有 .onnx ⇒ 只对比 torch / numpy）")

    rng = np.random.RandomState(12345)
    worst_abs_onnx, worst_rel_onnx = 0.0, 0.0
    worst_abs_np, worst_rel_np = 0.0, 0.0

    with torch.no_grad():
        for s in range(n_samples):
            x = rng.randn(1, obs_dim).astype(np.float32)
            y_torch = torch_model(torch.from_numpy(x)).numpy()
            y_np = forward_numpy(layers, x)

            scale = max(float(np.max(np.abs(y_torch))), 1e-9)
            d_np = float(np.max(np.abs(y_torch - y_np)))
            r_np = d_np / scale
            worst_abs_np = max(worst_abs_np, d_np)
            worst_rel_np = max(worst_rel_np, r_np)

            line = "   #%d  torch[0:3]=%s  numpy[0:3]=%s  rel=%.2e" % (
                s, np.round(y_torch[0, :3], 6), np.round(y_np[0, :3], 6), r_np)

            if sess is not None:
                y_onnx = sess.run(None, {"obs": x})[0]
                d_on = float(np.max(np.abs(y_torch - y_onnx)))
                r_on = d_on / scale
                worst_abs_onnx = max(worst_abs_onnx, d_on)
                worst_rel_onnx = max(worst_rel_onnx, r_on)
                line += "  onnx_rel=%.2e" % r_on
            print(line)

    print()
    print("   torch vs numpy : 最大绝对 %.3e  最大相对 %.3e" % (worst_abs_np, worst_rel_np))
    if sess is not None:
        print("   torch vs onnx  : 最大绝对 %.3e  最大相对 %.3e"
              % (worst_abs_onnx, worst_rel_onnx))

    ok_np = worst_rel_np <= rtol
    ok_on = (sess is None) or (worst_rel_onnx <= rtol)

    if ok_np and ok_on:
        print("   ✅ 一致（容差 %.0e）" % rtol)
        return True

    print("   ❌ 不一致（容差 %.0e）" % rtol)
    print("      ⇒ 激活函数或层顺序可能不对，**绝对不要上真机**")
    return False


def verify_with_rsl_rl(ckpt_path: str, layers, obs_dim: int,
                       n_samples: int = 3) -> bool:
    """用 rsl_rl 真实的 ActorCritic 重建，与我们的重建对比。

    ⚠️ 这一步的价值：前面三方（torch/onnx/numpy）都建立在"层顺序假设"上，
       如果假设本身错了，三方会**一致地错**。用 rsl_rl 重建是非循环的检查。

    ⭐ 返回值：True = 真的做了对比且通过；False = 跳过或失败。
    ⚠️⚠️ 曾经这里是 `-> None` 且所有失败都**静默 return**，
       结果 `layers[1:]` 那个 bug 让它每次都跳过而没人知道 ——
       文档却声称"这是唯一不循环的检查"。**验证路径不许静默失败。**
    """
    print("\n ---- 可选：用 rsl_rl 真实 ActorCritic 复核 ----")
    try:
        from rsl_rl.modules import ActorCritic
    except Exception as e:                                  # noqa: BLE001
        print("   ⚠️ rsl_rl 不可用（%s）" % str(e).splitlines()[0])
        print("   ⚠️⚠️ 独立验证【未执行】—— 前三项对比都基于层顺序假设，")
        print("       没有这一项就无法证明假设本身正确。请人工核对导出结果。")
        return False

    try:
        ck = torch.load(ckpt_path, map_location="cpu", weights_only=False)
    except TypeError:
        ck = torch.load(ckpt_path, map_location="cpu")
    sd = ck.get("model_state_dict", ck) if isinstance(ck, dict) else ck

    # ⚠️⚠️ 这里曾经写成 `layers[1:]`（2026-10-18 由 Linux 侧交付时抓到）
    #
    #    `layers` 是 [47→256, 256→128, 128→64, 64→12]。
    #    `layers[1:]` 会把**输出层**的 12 也当成隐藏层，于是构造出
    #    `47→128→64→12→12` 的错误网络，`load_state_dict` 必然失败。
    #    而下面的 `except` 把异常吞掉了 ⇒ **这段"唯一不循环的检查"被静默跳过**，
    #    看起来"跑过了"，其实根本没验证。
    #
    #  ⇒ 正确写法：`layers[:-1]`，只要除输出层之外的各层输出维度。
    #  ⇒ 更深一层的教训：**验证路径里不许静默吞异常**。
    #    现在失败时会明确告知"独立验证未执行"，并由 main() 汇总提示。
    hidden = [W.shape[0] for W, _ in layers[:-1]]
    try:
        ac = ActorCritic(obs_dim, obs_dim, layers[-1][0].shape[0],
                         actor_hidden_dims=hidden, critic_hidden_dims=hidden,
                         activation="elu")
        ac.load_state_dict(sd, strict=False)
        ac.eval()
    except Exception as e:                                  # noqa: BLE001
        print("   ⚠️ 构造 ActorCritic 失败：%s" % str(e).splitlines()[0])
        print("   ⚠️⚠️ 独立验证【未执行】—— 请把这段报错发我。")
        return False

    ours = build_torch_mlp(layers)
    rng = np.random.RandomState(777)
    worst = 0.0
    with torch.no_grad():
        for _ in range(n_samples):
            x = rng.randn(1, obs_dim).astype(np.float32)
            y_ref = ac.actor(torch.from_numpy(x)).numpy()
            y_our = ours(torch.from_numpy(x)).numpy()
            scale = max(float(np.max(np.abs(y_ref))), 1e-9)
            worst = max(worst, float(np.max(np.abs(y_ref - y_our))) / scale)
    print("   与 rsl_rl 真实 actor 的最大相对误差 = %.3e" % worst)
    if worst <= 1e-5:
        print("   ✅ 层顺序假设正确")
        return True
    print("   ❌ 层顺序/激活假设**错了** —— 请把这段输出发给我")
    return False


# ============================================================================
# 6. 保存 npz
# ============================================================================

def save_npz(layers, path: str) -> None:
    d = os.path.dirname(os.path.abspath(path))
    if d and not os.path.isdir(d):
        os.makedirs(d, exist_ok=True)
    payload = {"n_layers": np.int32(len(layers))}
    for n, (W, b) in enumerate(layers):
        payload["W%d" % n] = W
        payload["b%d" % n] = b
    np.savez_compressed(path, **payload)
    print("   ✅ 已保存 %s（%.0f KB）" % (path, os.path.getsize(path) / 1024.0))


# ============================================================================
# 7. 主流程
# ============================================================================

def main() -> int:
    ap = argparse.ArgumentParser(description="rsl_rl .pt → onnx + npz（含三方验证）")
    ap.add_argument("ckpt", help="model_XXXX.pt 路径")
    ap.add_argument("-o", "--out", default=None,
                    help="输出基名（不带扩展名）。默认与输入同名")
    ap.add_argument("--obs-dim", type=int, default=47, help="观测维度（rlcat_flat=47）")
    ap.add_argument("--no-onnx", action="store_true", help="不导出 onnx")
    ap.add_argument("--no-npz", action="store_true", help="不导出 npz")
    ap.add_argument("--no-verify", action="store_true", help="跳过一致性验证")
    ap.add_argument("--no-rsl-rl", action="store_true",
                    help="跳过 rsl_rl 复核（默认会尝试）")
    args = ap.parse_args()

    # ⚠️ 参数解析之后再要 torch —— 这样 --help / --version 在没有 torch 的机器上也能用
    require_torch()

    base = args.out or os.path.splitext(args.ckpt)[0]
    onnx_path = None if args.no_onnx else (base + ".onnx")
    npz_path = None if args.no_npz else (base + ".npz")

    print("=" * 74)
    print(" rsl_rl → onnx + npz  策略导出")
    print("=" * 74)
    print(" 输入 : %s" % args.ckpt)
    if onnx_path:
        print(" onnx : %s" % onnx_path)
    if npz_path:
        print(" npz  : %s" % npz_path)

    sd = load_state_dict(args.ckpt)
    layers = extract_actor_mlp(sd)

    # ---- 结构报告 ----
    print("\n actor 结构：")
    total = 0
    for n, (W, b) in enumerate(layers):
        act = "ELU" if n != len(layers) - 1 else "（输出层，无激活）"
        print("   %d: %-5d → %-5d  %-22s W%s b%s"
              % (n, W.shape[1], W.shape[0], act, W.shape, b.shape))
        total += W.size + b.size
    print("   参数总数 = %d" % total)

    in_dim, out_dim = layers[0][0].shape[1], layers[-1][0].shape[0]
    shape_ok = True
    if in_dim != args.obs_dim:
        print("\n ⚠️ 第一层输入 %d ≠ 预期 obs 维 %d（选错检查点？）"
              % (in_dim, args.obs_dim))
        shape_ok = False
    if out_dim != 12:
        print("\n ⚠️ 最后一层输出 %d ≠ 12 个关节" % out_dim)
        shape_ok = False

    # ---- 导出 ----
    torch_model = build_torch_mlp(layers)
    print()
    if onnx_path:
        d = os.path.dirname(os.path.abspath(onnx_path))
        if d and not os.path.isdir(d):
            os.makedirs(d, exist_ok=True)
        if not export_onnx(torch_model, onnx_path, args.obs_dim):
            return 1
        print("   大小 %.0f KB" % (os.path.getsize(onnx_path) / 1024.0))
    if npz_path:
        save_npz(layers, npz_path)

    # ---- 验证 ----
    verified = True
    independent = None          # None = 未尝试；True/False = 独立验证结果
    if not args.no_verify:
        verified = verify_three_way(layers, torch_model, onnx_path, args.obs_dim)
        if not args.no_rsl_rl:
            independent = verify_with_rsl_rl(args.ckpt, layers, args.obs_dim)
    else:
        print("\n ⚠️ 跳过了全部验证 —— 正式上真机前必须补跑一次")

    print()
    print("=" * 74)
    if verified and shape_ok:
        print(" ✅ 导出完成且验证通过")
        if independent is True:
            print("    ⭐ 含【独立验证】：rsl_rl 真实 ActorCritic 复核通过")
        elif independent is False:
            print("    ⚠️⚠️ 但【独立验证未执行】—— 前三项对比都建立在层顺序假设上；")
            print("        没有 rsl_rl 复核，无法证明假设本身正确。")
            print("        ⇒ 把上面的输出发人工核对后再上真机。")
        else:
            print("    （独立验证被 --no-rsl-rl 关闭）")
        print("=" * 74)
        if npz_path:
            print("\n 拷到树莓派：")
            print("   ssh <pi> 'mkdir -p ~/RL-CAT/pi/models'")
            print("   scp %s <pi>:~/RL-CAT/pi/models/" % npz_path)
        if onnx_path:
            print("\n .onnx 建议一并提交进仓库（发布格式，别人可用 Netron 验证）")
        return 0
    print(" ❌ 导出完成但验证/形状未通过 —— 不要上真机")
    print("=" * 74)
    return 1


if __name__ == "__main__":
    sys.exit(main())
