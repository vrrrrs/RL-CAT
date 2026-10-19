"""把训练好的策略导出成可部署的形式

产出三样东西：
  1. policy.pt        TorchScript，可以用 libtorch(C++) 或 torch(Python) 加载
  2. policy_forward.py 纯 numpy 前向实现，方便移植到固件（C/STM32）
  3. 部署契约说明      观测怎么拼、动作怎么用

用法:
    source scripts/env.sh
    python scripts/13_export_policy.py
    python scripts/13_export_policy.py --load_run Oct06_03-32-41_ --checkpoint 500 \
        --out ~/legged_gym/logs/rlcat_flat/Oct06_03-32-41_/exported
"""
import argparse
import os
import sys

import numpy as np
import torch
import torch.nn as nn

CKPT_DEFAULT = os.path.expanduser(
    "~/legged_gym/logs/rlcat_flat/Oct06_03-32-41_/model_500.pt")
OUT_DEFAULT = os.path.expanduser(
    "~/legged_gym/logs/rlcat_flat/Oct06_03-32-41_/exported")


class ActorMLP(nn.Module):
    """只保留 actor（策略网络）。critic 是训练用的，部署不需要。"""

    def __init__(self, state_dict):
        super().__init__()
        keys = sorted(k for k in state_dict if k.startswith("actor."))
        # actor.0 / actor.2 / actor.4 / actor.6 是 Linear，奇数索引是 ELU
        linear_idx = [int(k.split(".")[1]) for k in keys if k.endswith(".weight")]
        layers = []
        for i in sorted(linear_idx):
            w = state_dict["actor.%d.weight" % i]
            b = state_dict["actor.%d.bias" % i]
            layers.append(nn.Linear(w.shape[1], w.shape[0]))
            layers[-1].weight.data = w.clone()
            layers[-1].bias.data = b.clone()
            if i != max(linear_idx):          # 最后一层不加激活
                layers.append(nn.ELU())
        self.net = nn.Sequential(*layers)
        self.num_obs = state_dict["actor.%d.weight" % min(linear_idx)].shape[1]
        self.num_actions = state_dict["actor.%d.weight" % max(linear_idx)].shape[0]

    def forward(self, obs):
        return self.net(obs)


def dump_numpy_reference(model, path):
    """把权重导出成 .npz + 生成一个纯 numpy 的前向实现，方便移植到固件"""
    weights = {}
    for name, p in model.state_dict().items():
        weights[name.replace(".", "_")] = p.detach().cpu().numpy()
    np.savez(os.path.join(os.path.dirname(path), "policy_weights.npz"), **weights)

    src = '''"""策略网络的纯 numpy 前向实现（自动生成）

对应 TorchScript: policy.pt
权重文件:        policy_weights.npz

移植到固件时照抄 forward() 里的矩阵乘 + ELU 即可。
"""
import numpy as np
import os

_W = np.load(os.path.join(os.path.dirname(__file__), "policy_weights.npz"))


def _elu(x):
    return np.where(x > 0, x, np.exp(np.minimum(x, 0)) - 1.0)


def forward(obs):
    """obs: (num_obs,) 或 (N, num_obs) -> action: (num_actions,) 或 (N, num_actions)"""
    x = np.atleast_2d(np.asarray(obs, dtype=np.float32))
    i = 0
    while ("net_%d_weight" % i) in _W:
        x = x @ _W["net_%d_weight" % i].T + _W["net_%d_bias" % i]
        i += 2                       # 0=Linear, 1=ELU, 2=Linear, ...
        if ("net_%d_weight" % i) in _W:
            x = _elu(x)
    return x[0] if np.ndim(obs) == 1 else x


if __name__ == "__main__":
    import sys
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 47
    print("随机观测 ->", forward(np.zeros(n, dtype=np.float32)))
'''
    with open(path, "w", encoding="utf-8") as f:
        f.write(src)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", default=CKPT_DEFAULT)
    ap.add_argument("--out", default=OUT_DEFAULT)
    opt = ap.parse_args()

    os.makedirs(opt.out, exist_ok=True)
    print("加载检查点:", opt.ckpt)
    ck = torch.load(opt.ckpt, map_location="cpu")
    sd = ck["model_state_dict"]
    print("  检查点文件: %s" % os.path.basename(opt.ckpt))
    print("  参数项数: %d（含 critic，部署只用 actor）" % len(sd))

    model = ActorMLP(sd).eval()
    dims = [l.out_features for l in model.net if isinstance(l, nn.Linear)]
    print("  actor 结构: 输入 %d -> 隐藏 %s -> 输出 %d"
          % (model.num_obs, dims[:-1], model.num_actions))

    # ---------- 1. TorchScript 导出 ----------
    pt_path = os.path.join(opt.out, "policy.pt")
    example = torch.zeros(1, model.num_obs)
    traced = torch.jit.trace(model, example)
    traced.save(pt_path)
    print("\n[1] TorchScript -> %s (%.0f KB)" % (pt_path, os.path.getsize(pt_path) / 1024))

    # ---------- 2. 验证导出的模型和原模型一致 ----------
    loaded = torch.jit.load(pt_path).eval()
    x = torch.randn(8, model.num_obs)
    with torch.no_grad():
        diff = (loaded(x) - model(x)).abs().max().item()
    print("    一致性校验: 与原始网络的逐元素最大差 = %.3e  %s"
          % (diff, "[OK]" if diff < 1e-6 else "[!! 不一致]"))

    # ---------- 3. numpy 参考实现 ----------
    py_path = os.path.join(opt.out, "policy_forward.py")
    dump_numpy_reference(model, py_path)
    sys.path.insert(0, opt.out)
    import importlib
    mod = importlib.import_module("policy_forward")
    importlib.reload(mod)
    xn = np.random.randn(4, model.num_obs).astype(np.float32)
    with torch.no_grad():
        yn_torch = model(torch.from_numpy(xn)).numpy()
    yn_np = mod.forward(xn)
    d2 = np.abs(yn_torch - yn_np).max()
    print("[2] numpy 参考实现 -> %s" % py_path)
    print("    权重文件 -> %s" % os.path.join(opt.out, "policy_weights.npz"))
    print("    一致性校验: 与 torch 的最大差 = %.3e  %s"
          % (d2, "[OK]" if d2 < 1e-4 else "[!! 不一致]"))

    # ---------- 4. 打印部署契约 ----------
    print("\n" + "=" * 70)
    print("部署契约（固件必须严格照这个来）")
    print("=" * 70)
    print("""
观测 obs[47]（顺序对齐交接文档 §12，各维已按下面缩放后拼好再送进网络）:
   0- 2  重力向量（机体系）              = R^T · [0,0,-1]        ×1.0
   3- 5  机身角速度（陀螺，机体系）       = IMU 角速度            ×0.25
   6- 8  速度指令 [vx, vy, wz]                                    ×1.0
   9-20  关节位置 q - q_default（12 个，Isaac 顺序 C）             ×1.0
  21-32  关节速度 qd（12 个）                                     ×0.1
  33-44  上一帧动作（策略原始输出，12 个）                          ×1.0
  45-46  步态相位 sin/cos = sin/cos(2π·φ), φ 每步 += dt·%.2f Hz   ×1.0
""" % 3.0)
    print("""动作 act[12]（策略输出，未加限幅）
    q_target[i] = q_default[i] + 0.25 * act[i]      # rad
    q_target = clip(q_target, 软限位)                # 建议硬限位内缩 5°
    然后交给关节 PD：  tau = Kp*(q_target - q) - Kd*qd
    Kp ≈ 20 N·m/rad,  Kd ≈ 0.5 N·m·s/rad（仿真用值，真机需标定）

⚠️ 关节顺序：仿真内部是 Isaac 顺序 C = [lb, lf, rb, rf] × [hip, thigh, calf]
   固件按逻辑顺序 A = [hip: lf,lb,rf,rb][thigh: ...][calf: ...] 编排
   两者映射（实测）：A2C = [3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8]
   即： firmware[i] 对应 sim[A2C[i]]

⚠️ 步态相位 φ 是【内部时钟】，真机不需要任何传感器就能复现：
     φ = (φ + dt * 3.0) % 1.0      dt = 0.01 s（100 Hz）
   每个 episode 开始时 φ 归零。
""")
    print("导出完成，目录:", opt.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
