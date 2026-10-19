"""策略网络的纯 numpy 前向实现（自动生成）

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
