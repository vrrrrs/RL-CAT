# 仿真与训练（Simulation & RL）

> **文档编号**：DOC-SIM-001 ｜ **版本**：v0.1 ｜ **日期**：2026-10-09
>
> **状态**：🟡 **骨架**。完整内容待从 `Sim/RL/` 下的 4 份文档提炼。
> **⚠️ `Sim/` 目录暂未整理**，路径可能变动。

---

## 1. ⭐ 现有文档（暂时作为权威来源）

**⚠️ 本页只是索引。完整内容在 `Sim/RL/` 下的这几份文档里，它们的质量很高。**

| 文档 | 内容 | 建议 |
|---|---|---|
| [`Sim/RL/RL-CAT-训练实施报告.md`](../Sim/RL/RL-CAT-训练实施报告.md) | ⭐ **最权威**：环境搭建 5 个坑、资产改造、9 轮调参完整记录、失败分析 | **必读** |
| [`Sim/RL/RL-CAT-IsaacGym训练流程方案.md`](../Sim/RL/RL-CAT-IsaacGym训练流程方案.md) | 动手前的方案设计 | |
| [`Sim/RL/手动训练与调参指南.md`](../Sim/RL/手动训练与调参指南.md) | 实操手册 | |
| [`Sim/RL/从零到部署-四个问题详解.md`](../Sim/RL/从零到部署-四个问题详解.md) | 部署相关 | → 将并入 [`50-deployment.md`](50-deployment.md) |
| [`Sim/RL/怎么看效果.md`](../Sim/RL/怎么看效果.md) | 评估方法 | |

**⚠️ `Sim/RL/RL-CAT项目交接文档.md` 含已被证伪的结论，不要作为依据。**

**该文档已在 `archive/` 留档并标注了勘误：** 见 [`archive/README.md`](../archive/README.md) 的「勘误 1（thigh 限位）」「勘误 2（观测维数）」「勘误 3（零位性质）」。

**⚠️ `Sim/RL/` 下的同名文件尚未清理**（Sim 目录暂不整理），读取时请以 `archive/README.md` 的勘误为准。

---

## 2. 环境

| 项 | 值 |
|---|---|
| 操作系统 | Ubuntu 22.04 |
| GPU | RTX 4060 Laptop 8 GB |
| conda 环境 | `isaacgym`（Python 3.8.20 + torch 1.12.1+cu113） |
| 仿真器 | **Isaac Gym Preview 4** |
| 训练框架 | **legged_gym**（master 分支，重构版：`LeggedRobot` 继承 `BaseTask`） |
| RL 库 | **rsl_rl v1.0.2** ⚠️ README 明确要求，用 master 会 API 不匹配 |

**代码位置：**

```
Sim/
├── legged_gym/          legged_gym 源树（含 rlcat task）
│   ├── legged_gym/envs/rlcat/    ⭐ rlcat_config.py + rlcat_env.py
│   ├── legged_gym/envs/base/     父类
│   └── logs/                      训练日志
├── rsl_rl/              rsl_rl 源树
└── RL/                  训练产物与脚本
    ├── urdf/            URDF（robot.urdf + robot_sim.urdf）
    ├── scripts/         12 个脚本 + env.sh + run_train.sh
    ├── exported/        策略导出
    ├── logs/ plots/ videos/ eval_frames/
```

### 2.1 ⚠️ 环境搭建踩过的 5 个坑（无 sudo 解决）

| # | 症状 | 根因 | 解决 |
|---|---|---|---|
| 1 | `No module named 'isaacgym'` | 用了系统 python | `conda activate isaacgym` |
| 2 | `numpy has no attribute 'float'` | isaacgym 的 `torch_utils.py:135` 用了 numpy 1.24 已移除的别名 | [`06_patch_isaacgym_numpy.py`](../Sim/RL/scripts/06_patch_isaacgym_numpy.py) |
| 3 | `Command '['which','c++']' returned non-zero` | 系统无 C++ 编译器，gymtorch 需 JIT | `conda install -c conda-forge gxx_linux-64=10 gcc_linux-64=10` |
| 4 | `fatal error: crypt.h` | conda sysroot 缺 `crypt.h`（glibc 2.28+ 拆到 libxcrypt） | `conda install libxcrypt` + `CPATH=$CONDA_PREFIX/include` |
| 5 | `No module named 'tensorboard'` | rsl_rl 的 runner 导入它 | `pip install tensorboard` |

**⚠️ 两个容易误判的事：**

```
① nvidia-smi 第一次报"找不到驱动"是笔记本 dGPU 电源管理，重试即可，不是环境坏了
② use_gpu_pipeline=True 时调用 get_actor_rigid_body_states 会 CUDA illegal memory access
   —— 这是 API 用法违规。训练代码走 tensor API；调试脚本设 use_gpu_pipeline=False
```

**统一入口**：[`Sim/RL/scripts/env.sh`](../Sim/RL/scripts/env.sh) 收集所有环境变量，脚本先 `source` 它。

---

## 3. 资产改造（URDF → 训练用 URDF）

**原 `robot.urdf` 一字未动**，改动全部在生成物 `robot_sim.urdf`，生成器：[`05_make_sim_urdf.py`](../Sim/RL/scripts/05_make_sim_urdf.py)

### 3.1 三处改动

| # | 改动 | 说明 |
|---|---|---|
| **1** | **放宽 `*_thigh` 上限：30° → 90°** | ⚠️ 为了给摆腿留空间。**不是**修复"站立够不到"（那个说法已被证伪，见 `archive/README.md` 勘误 1） |
| **2** | **碰撞体：STL 三角网格 → 基本几何体** | `base_link`→box；`hip`/`thigh`→capsule；`calf`→sphere（足端 r=9mm）+ capsule |
| **3** | **修掉 CAD 导出的左右不对称** | 两处 1.5 mm 导出误差，会给策略系统性偏置 |

### 3.2 ⚠️ 足端球坐标（不能凭直觉填）

| link | origin xyz（在各自 calf 坐标系） |
|---|---|
| `lf_calf` | `0.071332 -0.079404 0.012500` |
| `lb_calf` | `0.075518 -0.074932 0.012500` |
| `rf_calf` | `0.072495 -0.078210 -0.004500` |
| `rb_calf` | `0.077654 -0.072452 -0.004500` |

**⚠️ 两个坑：**

```
① 本项目的 link 坐标系没对齐连杆方向（小腿网格在自身系里沿 (x,−y) 对角线延伸）
   → 不能填「(0,0,腿长)」
② 必须在「站立姿态的 base 系」里找网格最低点，再变换回 link 系
   → 直接在 link 系里取 argmin(z) 会选到错误顶点，球会放偏
```

**⚠️ 另一个坑：** 细长连杆用轴对齐 AABB box 会太"胖"，机器人会塌掉（机身从 0.130 掉到 0.068）。**必须用沿轴数据拟合的胶囊体。**

---

## 4. 训练环境（legged_gym task）

**配置**：`Sim/legged_gym/legged_gym/envs/rlcat/rlcat_config.py`
**覆写**：`Sim/legged_gym/legged_gym/envs/rlcat/rlcat_env.py`

### 4.1 观测：47 维（方案 C）

**⚠️ 完整定义见 [`30-firmware.md`](30-firmware.md) §5.1 —— 那张表是固件与仿真的共同权威。**

**顺序刻意对齐固件侧拼接顺序，这样固件不用改。**

### 4.2 四处必要覆写（不写就跑不起来/跑不对）

| # | 覆写 | 为什么必须 |
|---|---|---|
| 1 | `_reset_dofs` | 父类是 `default_dof_pos * rand(0.5,1.5)`；本项目默认姿态**全是 0**，`0 × rand = 0` → **毫无初始随机化** |
| 2 | `_get_noise_scale_vec` | 父类按它自己的 48 维布局硬编码下标，我们是 47 维 |
| 3 | `compute_observations` | 方案 C 的布局与相位时钟 |
| 4 | `_reward_feet_air_time` | upstream 把目标滞空时间**硬编码 0.5 s**，那是给几十 kg 的 anymal/a1 调的 |

### 4.3 关键配置

| 项 | 值 | 依据 |
|---|---|---|
| `default_joint_angles` | **全 0** | ✅ 实测 q=0 就是站立姿态 |
| `action_scale` | **0.25** | 与协议一致 |
| `decimation` | **2**（sim dt=1/200 → 100 Hz） | 对齐固件 |
| `foot_name` | `"calf"` | ⚠️ URDF 没有独立足端 link，只能用小腿充当 |
| `penalize_contacts_on` | `["thigh"]` | 必须把 calf 从惩罚里删掉（否则"一边用小腿当脚、一边惩罚它触地"） |
| `terminate_after_contacts_on` | `["base_link"]` | 根刚体叫 base_link |
| `terrain.mesh_type` | `plane` | 先平地 |
| `commands.lin_vel_x` | `[-0.3, 0.5]` | 机身仅 200 mm，1 m/s 太激进 |

**⚠️ 训练侧不需要关节顺序映射** —— legged_gym 用 `default_joint_angles[name]` 按名字查表。

**但导出到固件时【必须】对表，而且这是本项目最容易致命的一处**（顺序错 ⇒ 每条动作发到错误关节 ⇒ 上机立刻摔）。已确认的映射与两个独立证据见 [`50-deployment.md`](50-deployment.md) §2.3：

```
PERM_PROTO_TO_TRAIN = [0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11]
```

**⚠️ 旧版本这里写的 `A2C = [3,0,9,6,4,1,10,7,5,2,11,8]` 已废弃** —— 它把 `lf_hip` 映射到 `lb_hip`，与限位判据矛盾。

---

## 5. 训练与调参历程

**⚠️ 完整记录见 [`Sim/RL/RL-CAT-训练实施报告.md`](../Sim/RL/RL-CAT-训练实施报告.md) §5 —— 9 轮调参，含失败案例与根因分析。这是本项目最有价值的工程记录之一。**

### 5.1 简要历程

| 轮 | 配置变化 | 结果 |
|---|---|---|
| 1 | 基线 | iter 681 训练崩溃 |
| 2 | 目标改 0.15s + `feet_air_excess` | ✅ 稳定不发散 |
| 3 | 追加 `feet_airborne` −2.0 | ❌ 惩罚过重，机器人不走了 |
| 4 | 一次改三样（对称化+刚度+height） | ❌ 无法归因 |
| 5 | 恢复第 2 轮，只改对称化 URDF | ✅ 当时最佳 |
| 6 | 加 `gait_phase` 相位奖励 | 🟡 对称性改善 10~30 倍，但**速度跟踪崩了**（原地踏步刷奖励） |
| 7 | 相位奖励按指令门控 + 提高 tracking 权重 | ✅ 修好作弊 |
| 8 | tracking 权重再提高 | ❌ 过头，开始摔 |
| 9 | **只改 `tracking_sigma` 0.25 → 0.10** | ⭐ **全部验收项达标** |

### 5.2 ⭐ 最有价值的经验：奖励作弊

**第 6 轮的失败很有教育意义：**

```
策略发现「原地踏步把相位奖励刷满」比「真的往前走」更划算：
  相位奖励每步 +1.08，跟踪奖励最多 +1.0，
  而移动本身还要付出力矩、加速度、碰撞等惩罚。

结果：cmd=0.3 m/s 时实际倒退 −0.068 m/s。

修复：相位奖励按指令强度门控（rew *= clamp(|cmd|/0.3, 0, 1)）
      + 提高 tracking 权重
```

### 5.3 训练速度

```
2048 环境：12–19 万 steps/s，单次迭代 0.25–0.45 s，1000 迭代约 6–8 分钟
显存占用 3.5 / 8 GB
```

---

## 6. 工具脚本

**位置**：`Sim/RL/scripts/`

| 脚本 | 用途 |
|---|---|
| `01_load_asset.py` | 无渲染加载 URDF，打印所有关键属性 |
| `02_bisect_cuda.py` | 定位 CUDA illegal memory access 的最小复现 |
| `03_stand_test.py` | PD 站立测试，实测 q_default 与机身高度 |
| `04_viewer.py` | 可视化自检（需桌面环境） |
| `05_make_sim_urdf.py` | ⭐ 生成训练专用 URDF |
| `06_patch_isaacgym_numpy.py` | Isaac Gym 与 numpy ≥1.24 的兼容补丁 |
| `07_eval.py` | 策略评估（数值指标 + 录帧） |
| `09_frames_to_video.py` | PNG 帧序列 → mp4/GIF |
| `10_plot_curves.py` | 从 tensorboard 画训练曲线 |
| `11_gait_analysis.py` | 步态分析（接触时序文本图） |
| `12_play.py` | 3D 实时播放策略 |
| `13_export_policy.py` | ⭐ 导出策略（TorchScript + numpy 权重） |
| `env.sh` | 环境变量统一入口 |
| `run_train.sh` | 训练启动器（含日志落盘、断点续训） |

---

## 7. 策略导出

**位置**：`Sim/RL/exported/`

| 文件 | 说明 |
|---|---|
| `policy.pt` | TorchScript，可用 libtorch(C++) 或 torch(Python) 加载 |
| `policy_weights.npz` | numpy 权重 |
| `policy_forward.py` | 前向推理参考实现 |

**⚠️ 部署时需要注意的映射：** `PERM_PROTO_TO_TRAIN = [0,3,6,9,1,4,7,10,2,5,8,11]`（逻辑/协议索引 → 训练侧顺序）。
**两个独立证据见 [`50-deployment.md`](50-deployment.md) §2.3。旧值 `A2C = [3,0,9,6,4,1,10,7,5,2,11,8]` 已废弃。**

---

## 8. 待补充

```
□ 把 Sim/RL 的 4 份文档整合成本页（目前只是索引）
□ 奖励函数的完整定义（各项名称、权重、公式）
□ 域随机化的配置与取值范围
□ 地形课程学习的计划
□ 完整的评估指标定义
□ 环境搭建的逐步复现指南（目前只有坑的记录）
□ Sim/ 目录整理后的路径更新
```

---

**文档结束。**
