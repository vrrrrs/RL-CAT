# RL-CAT → Isaac Gym 行走训练流程方案

> 编制日期：2026-10-06
> 配套文档：`RL-CAT项目交接文档.md`（本文档是对它的续接与勘误）
> 状态：**仅方案，未动手**。URDF 未修改，未安装任何训练框架。
>
> 本文档中标注 ✅实测 的结论，均为本机真实跑出来的结果，不是推断。

---

## 0. 一页速览

**目标**：尽快在 Isaac Gym 上训出 RL-CAT 的行走策略。

**路线**：复用 ETH 的 `legged_gym` + `rsl_rl`，把它的 `a1`（Unitree A1，同样是 12 DOF 四足、每腿 hip/thigh/calf）配置移植成 RL-CAT。

**为什么这条路快**：

| 事实 | 意义 |
|---|---|
| ✅实测 `q_default = [0]*12`，且正好是站立姿态 | 不用做零位标定，不用逆解求站立姿态 |
| a1 也是 12 DOF、每腿 hip/thigh/calf | 奖励函数、PD、课程全部可平移 |
| `action_scale` 两边都是 0.25 rad | 动作接口天然一致 |
| ✅实测 legged_gym 按**关节名**索引默认角 | **关节顺序问题在训练侧自动消失**，不用写映射 |
| `a1` 注册时直接用基类 `LeggedRobot`，没有自定义 env 类 | 我们大概率也只需写 1 个配置文件 |

**需要新写的代码量**：1 个配置文件（~80 行）+ 1 个小覆写（~8 行）+ 1 行注册。**几乎不用写环境逻辑。**

---

## 1. 两个必须先纠正的事实

### 1.1 `legged_gym` 配的是 `rsl_rl`，不是 `rl_games`

原方案里写的「legged_gym + rl_games」是记混了。两套东西是分开的：

| 框架 | 出身 | PPO 实现 | 配置形式 | 四足任务 |
|---|---|---|---|---|
| **legged_gym** | ETH RSL | **rsl_rl** | Python 类继承 | anymal_b/c, **a1**, cassie |
| IsaacGymEnvs | NVIDIA | rl_games | YAML | anymal, ant, humanoid |

**结论：走 legged_gym + rsl_rl。** 理由是它的 `a1` 结构与 RL-CAT 最接近，且配置是 Python 类、改起来比 YAML 直观。

⚠️ 安装时 **rsl_rl 必须 checkout `v1.0.2`**（README 明确要求）。用 rsl_rl 的 master 会和方法签名对不上。

### 1.2 交接文档的「🔴 阻塞项 1」是误判

文档说：

> `*_thigh` 限位上限 +30° < 站立所需 +31.19° → 站立姿态超出限位 → 阻塞

**这是把两个不同的角混为一谈了：**

```
+31.19° = 大腿【连杆】相对【竖直方向】的绝对夹角（几何量，由余弦定理算出）
 URDF limit = 关节相对【CAD 装配零位】的转角（关节量）
```

既然 `q = 0` 就是 CAD 装配好的站立姿态，那么**站立时 thigh 关节角 = 0**，距 +30° 上限还有整整 30° 余量。

✅实测佐证：PD 把 12 个关节锁在 `q=0`，机器人稳稳站住，机身 0.130 m、roll −0.64°、pitch −3.42°，3 秒不动。

**所以这一项从「阻塞」降级为「可选优化」**（仍建议放宽到 60°~90°，给摆腿留空间，纯收益无风险）。

---

## 2. 技术栈全景

```
┌─────────────────────────────────────────────────────────────┐
│ 训练侧（已就绪）                                              │
│   conda env: isaacgym   (Python 3.8.20)                     │
│   ├── isaacgym 1.0rc4   ~/isaacgym_ws/isaacgym/python       │
│   ├── torch 1.12.1+cu113                                     │
│   └── GPU: RTX 4060 Laptop 8GB, 驱动 595.91.07               │
│       ✅实测 GPU PhysX 在 sm_89 上正常工作                    │
├─────────────────────────────────────────────────────────────┤
│ 待安装                                                       │
│   ├── legged_gym  (github.com/leggedrobotics/legged_gym)     │
│   └── rsl_rl      (github.com/leggedrobotics/rsl_rl, v1.0.2) │
├─────────────────────────────────────────────────────────────┤
│ 我们的新增                                                  │
│   ├── resources/robots/rlcat/urdf/   ← 拷贝现有 urdf/       │
│   ├── legged_gym/envs/rlcat/rlcat_config.py                  │
│   ├── legged_gym/envs/rlcat/rlcat_env.py  (可选，仅覆写1个函数)│
│   └── legged_gym/envs/__init__.py     (加 1 行注册)          │
└─────────────────────────────────────────────────────────────┘
```

---

## 3. ✅ 实测基线（本机真实结果，可直接采信）

### 3.1 资产加载核对

| 项 | 期望 | ✅实测 | 判定 |
|---|---|---|---|
| 刚体数 | 13 | **13** | ✅ |
| DOF 数 | 12 | **12** | ✅ |
| 总质量 | 1.386 kg | **1.3860 kg** | ✅ 精确 |
| 各刚体质量 | base 0.730 / hip 0.080 / thigh 0.070 / calf 0.014 | 完全一致 | ✅ |
| mesh scale | 26 处 `0.001` | 26/26 | ✅ |
| GPU PhysX | 可用 | `Physics Device: cuda:0`, `GPU Pipeline: enabled` | ✅ |

**生成的实例名（Isaac 内部顺序）：**
```
base_link, lb_hip, lb_thigh, lb_calf, lf_hip, lf_thigh, lf_calf,
rb_hip, rb_thigh, rb_calf, rf_hip, rf_thigh, rf_calf
```

### 3.2 ⭐ DOF 顺序：既不是 A 也不是 B，是第三种

交接文档猜 Isaac 顺序等于 URDF 声明顺序（B 表）—— **不对**。✅实测：

```
Isaac 实际 DOF 顺序 C:  lb_hip, lb_thigh, lb_calf,   lf_...,   rb_...,   rf_...
                        └──────────────────────────┘ 按腿分组，但腿序是 lb, lf, rb, rf
```
（B 表的腿序是 lf, lb, rf, rb —— 前半段两条腿被交换了。）

**逻辑索引 A → Isaac 索引 C 的映射（✅实测）：**
```python
A2C = [3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8]
```

**但是——这个映射在训练阶段根本用不到。** 见 §4.2。

### 3.3 ⭐ q_default 实测：`[0]*12` 就是站立姿态

```
PD 锁 q=0，stiffness=30, damping=1.0，放开底座 + 地面，跑 3 秒：

  t(s)    base_z   roll(deg)  pitch(deg)
  0.00    0.1998      0.00       0.00     ← 起始从 0.20 落下
  0.25    0.1263     -0.91      -5.69
  0.50    0.1305     -0.31      -3.65
  1.00    0.1301     -0.57      -3.44
  3.00    0.1300     -0.64      -3.42     ← 收敛后完全静止
```

**结论：**
- ✅ `q_default = [0]*12` **成立**，交接文档 §6.3 的推断得到证实
- ✅ **机身稳态高度 = 0.130 m**（这个值后面配 `base_height_target` 直接用）
- ✅ PD 站立测试（文档 §9 步骤 8）**直接通过**，stiffness 30 / damping 1.0 够用
- ⚠️ 静态 pitch = −3.42°，来自 URDF 的 1.5 mm CAD 不对称（`lf_thigh` origin z=0.0586，其余 ±0.0571）。影响很小，RL 能吸收，但记录在案

### 3.4 ⚠️ 踩到的坑：GPU pipeline 下禁止调非 tensor API

```python
# 这样写会炸，而且报的是吓人的 CUDA 错误，不是普通异常：
sim_params.use_gpu_pipeline = True
...
gym.simulate(sim); gym.fetch_results(sim, True)
gym.get_actor_rigid_body_states(env, actor, gymapi.STATE_ALL)   # ← 这里
```
```
[Error] Gym cuda error: an illegal memory access was encountered
[Error] Failed to fill rigid body state tensor
```

**看起来像环境崩了，其实是 API 用法违规。** 两个解法：
- 训练代码：走 **tensor API**（`acquire_rigid_body_state_tensor` 等）—— legged_gym 本来就是这么写的，不受影响
- 调试脚本：设 `use_gpu_pipeline = False`

### 3.5 ✅ 版本兼容性：已逐项核对源码，完全对得上

把 legged_gym (master) 和 rsl_rl (v1.0.2) 的源码拉下来逐行比对，**接口契约全部吻合**：

| 契约 | legged_gym 的调用 | rsl_rl v1.0.2 的实现 | 判定 |
|---|---|---|---|
| 构造 runner | `OnPolicyRunner(env, train_cfg_dict, log_dir, device=args.rl_device)` | `__init__(self, env, train_cfg, log_dir=None, device='cpu')` | ✅ |
| 加载模型 | `runner.load(resume_path)` | `def load(self, path, load_optimizer=True)` | ✅ |
| 开始训练 | `learn(num_learning_iterations=..., init_at_random_ep_len=True)` | `def learn(self, num_learning_iterations, init_at_random_ep_len=False)` | ✅ |
| 策略网络 | `policy_class_name='ActorCritic'` + `actor_hidden_dims` 等 | `ActorCritic(num_actor_obs, num_critic_obs, num_actions, actor_hidden_dims, critic_hidden_dims, activation, init_noise_std)` | ✅ |
| env 属性 | — | runner 需要 `num_envs / num_obs / num_actions / num_privileged_obs / max_episode_length / episode_length_buf / get_observations / get_privileged_observations / reset / step` | ✅ 全部由 `BaseTask` + `LeggedRobot` 提供 |
| reset 返回值 | — | `_, _ = self.env.reset()` 期望 2 元组 | ✅ `BaseTask.reset()` 返回 `(obs, privileged_obs)` |

**一个值得知道的结构变化**：master 版的 `LeggedRobot` 继承的是 `BaseTask`（`envs/base/base_task.py`），**不是**老版本的 `VecEnv`。`task_registry` 仍然 `from rsl_rl.env import VecEnv`，但那只是类型标注，不影响运行。

> ⚠️ 即便如此，**P1 阶段那个「先跑官方 a1 冒烟测试」仍然不能省** —— 它是唯一能证明「这套 master + v1.0.2 组合真的能跑」的手段。源码比对只能证明接口对得上，不能证明没有运行时问题。

**附带好消息**：`a1` 的 URDF 是**随仓库提供**的（`resources/robots/a1/urdf/a1.urdf`），冒烟测试开箱即用，不需要额外下载任何资产。

### 3.6 ✅ numpy 弃用风险排除

全仓库扫描 `np.float` / `np.int` / `np.bool` / `np.object` / `np.str` / `np.long`：

```
[OK] 没有发现任何已移除的 numpy 别名
```

所以本机 numpy 1.24.4 **不需要降级**。（原风险清单第 9 条排除。）

### 3.7 ⚠️ 新发现：visual 和 collision mesh 内容完全相同

| calf link | visual mesh | collision mesh |
|---|---|---|
| lf_calf | part_7.stl，618 顶点，最低点 z=−0.1374 | part_8.stl，618 顶点，最低点 z=−0.1374 |
| lb_calf | 618 顶点，z=−0.1432 | 618 顶点，z=−0.1432 |
| rf_calf | 642 顶点，z=−0.1379 | 642 顶点，z=−0.1379 |
| rb_calf | 642 顶点，z=−0.1455 | 642 顶点，z=−0.1455 |

交接文档说「新版把每个 link 的 visual 和 collision 网格分开导出」——**是分成了两份文件，但几何一模一样**。

**含义**：当前碰撞体就是**全精度三角网格**（每腿 600+ 顶点 × 4 腿），PhysX 要在 2048 个环境里对它们做三角网格碰撞检测 —— 这**更加确认 §7 的改动 2/3（换 box/sphere）不是可选优化，而是必做项**。

### 3.8 ⚠️ 重要修正：仿真站立高度比几何预测低 15.5 mm

用正向运动学（q=0）算出足端最低点在 base 坐标系的位置：

```
几何预测：最低点 z = −0.1455 m  →  机身站立高度应为 0.1455 m
实测（三角网格碰撞 + PD 锁 q=0）：机身高度 = 0.1300 m
                                    差 15.5 mm
```

FK 本身经过交叉验证（膝关节点预测 vs 实测，前腿误差仅 2.6~4.2 mm），所以这 15.5 mm **是足端三角网格陷入地面的穿透量**，不是算错。

**两个直接后果：**

1. ⚠️ **`rewards.base_height_target` 必须重新实测**。我上一版方案里写 0.13 是「三角网格碰撞」下的值；**换成球碰撞体后穿透量会变，这个数就失效了**。正确顺序是：先改碰撞体 → 再跑一次 `03_stand_test.py` → 用新高度填 `base_height_target`。
2. 也说明当前物理的足-地接触是「软」的，浮动可达 1.5 cm。球碰撞体会明显改善这一点，这本身对学步态是好事。

---

## 4. 接口设计（关键决策区）

### 4.1 观测向量：三种候选，必须选一个

✅实测 legged_gym 的 `compute_observations()` 精确构成（源码 `legged_robot.py:209`）：

```python
obs = cat( base_lin_vel * 2.0,          # 3
           base_ang_vel * 0.25,         # 3
           projected_gravity,           # 3
           commands[:, :3],             # 3
           (dof_pos - default) * 1.0,   # 12
           dof_vel * 0.05,              # 12
           actions )                    # 12
# = 48 维；若 measure_heights=True 再拼 187 维地形高度 → 235 维（基础配置默认值）
```

对比交接文档 §12 的 50 维设计：

| 维度内容 | 文档 50 维 | legged_gym 48 维 | 真机可测？ |
|---|---|---|---|
| 重力向量 | 0–2 ✅ | ✅ | ✅ IMU 内部融合 |
| 机身角速度 | 3–5 ✅ | ✅ | ✅ 陀螺 |
| 速度指令 | 6–8 ✅ | ✅ | ✅（指令本身） |
| 关节位置 | 9–20 ✅ | ✅ | ✅ 编码器 |
| 关节速度 | 21–32 ✅ | ✅ | ✅ 舵机回读 |
| 上一帧动作 | 33–44 ✅ | ✅ | ✅（自己存的） |
| **机身线速度** | ❌ 没有 | **有 3 维** | ❌ **真机测不到** |
| 步态相位 sin/cos | 45–46 ✅ | ❌ 没有 | ✅ 内部时钟 |
| 保留 | 47–49 ⚠️ | ❌ 没有 | — |

**核心矛盾：legged_gym 塞了 3 维机身线速度，而真机没有这个传感器。** 策略一旦依赖它，sim-to-real 必崩。

> ⚠️ **关于陀螺仪**：上表里的「机身角速度」**就是陀螺仪**（`base_ang_vel`，机体系角速度，正是陀螺测的量）。
> **四个方案（A/B/C/D）都包含陀螺仪，它不是可选项** —— 它是四足平衡控制最关键的观测量之一，去掉了基本学不会走路。
> 交接文档 §12 已经给它指定了位置：**索引 3–5**。JY901P 直接输出角速度，无需额外处理。
>
> 与之配套的「重力向量」（索引 0–2）由 JY901P 的**内部姿态融合**结果反算（`R^T · [0,0,−1]`），也不需要额外传感器。
> 所以 IMU 的两组观测量（重力 3 维 + 角速度 3 维）在真机上都是现成的。

**三个候选：**

| 方案 | 维度 | 优点 | 缺点 |
|---|---|---|---|
| **A. 原生 48 维** | 48 | 零改动，最快 | 含真机测不到的线速度，部署要处理 |
| **B. 删线速度 45 维** | 45 | 全部可真机复现，干净 | 比文档少 5 维 |
| **C. 45 + 相位 47 维** ⭐ | 47 | 全部可真机复现 + 相位帮学步态 | 要覆写 `compute_observations`（~15 行） |

**推荐 C。** 理由：
1. 相位信号是**内部时钟，不需要任何传感器**就能在固件里复现，且对学习周期步态帮助很大；
2. 文档那 3 个「保留维」本来就是待定项，别为了凑 50 硬塞；
3. ⭐ **如果维度顺序按交接文档 §12 来排，C 方案正好 = 文档 50 维设计的【前 47 维】**（重力→角速度→指令→关节位置→关节速度→上一帧动作→相位）。固件可以先实现 47 个槽位，将来想加那 3 个「保留维」直接往后接，**不用动已有顺序**。

> ⚠️ 这个选择会影响**固件侧的观测拼接顺序**。选 C 的话，交接文档 §12 只需把「47–49 保留」标注为「暂不实现」，其余一字不用改 —— 这是 C 相对 B 的额外好处。

**量纲约定（仿真/固件必须一致，否则策略行为漂移）：**

| 量 | 缩放 | 备注 |
|---|---|---|
| 重力向量 | ×1.0 | |
| 机身角速度 | **×0.25** | 与文档 §12 一致 |
| 速度指令 | ×1.0 | |
| 关节位置 | ×1.0 | 相对 `q_default` 的偏差 |
| 关节速度 | **×0.1** | 文档写 ×0.1，legged_gym 默认 0.05 —— **本项以文档为准，需在配置里覆盖** |
| 上一帧动作 | ×4.0（等价于裸策略输出） | 见 §6.2 注释 |
| 步态相位 | ×1.0 | |

> ⚠️ 注意 legged_gym 的 `normalization.obs_scales.dof_vel` 默认是 **0.05**，而交接文档 §12 要求 **0.1**。这是两者唯一的缩放冲突，配置时必须显式改成 0.1（或反过来改固件），**不能放任不一致**。

### 4.2 ⭐ 关节顺序：训练侧自动解决，不用写映射

✅实测源码 `legged_robot.py:526-530`：

```python
self.default_dof_pos = torch.zeros(self.num_dof)
for i in range(self.num_dofs):
    name = self.dof_names[i]                                  # 取 asset 实际顺序 C
    angle = self.cfg.init_state.default_joint_angles[name]    # 按【名字】查配置字典
    self.default_dof_pos[i] = angle
```

**legged_gym 永远用 asset 的实际顺序（C），配置里的 `default_joint_angles` 是按关节名索引的字典。**

推论：
- ✅ **只要字典的 key 用真实关节名**（`lf_hip`…`rb_calf`），顺序问题**自动消失**，不需要 A2C 映射
- ✅ 策略的动作/观测向量自然就是 C 顺序，训练全程自洽
- ⚠️ **A2C 映射只在「导出到固件」时需要对一次表**（因为固件按 A 表分组）
- ⚠️ 我们的 `q_default` 全是 0，所以**顺序连数值都不影响**，纯索引问题，最好排查

这也解释了为什么交接文档 §2.4 那条🟡风险（"DOF 顺序必须做映射"）实际不是训练期的活。

### 4.3 PD 增益：按关节组配置

✅实测源码 `legged_robot.py:532-535`，是**子串匹配**：

```python
for dof_name in self.cfg.control.stiffness.keys():
    if dof_name in name:        # 子串包含，不是正则
        self.p_gains[i] = ...
```
所以 `{'hip': 20., 'thigh': 20., 'calf': 20.}` 能正确匹配 `lf_hip` / `lf_thigh` / `lf_calf`。

⚠️ 如果某个关节没匹配到，legged_gym 会把它的 PD 增益**静默设为 0**（只打印一行提示），机器人就瘫了。配置后务必检查这行提示没出现。

### 4.4 ⭐ 一个隐蔽的坑：默认姿态全 0 → reset 随机化失效

✅实测源码 `legged_robot.py:385`：

```python
self.dof_pos[env_ids] = self.default_dof_pos * torch_rand_float(0.5, 1.5, ...)
```

a1 的默认姿态非零（hip ±0.1、thigh 0.8、calf −1.5），乘个随机因子就是有效随机化。
**但我们的 `default_dof_pos` 全是 0 → `0 × rand = 0` → 每次 reset 都把关节精确摆在 0，完全没有初始状态随机化。**

后果：策略学得更慢、更容易过拟合到「从精确站立姿态出发」，鲁棒性差。

**修法**（约 8 行，必须做）：
```python
class RlcatRobot(LeggedRobot):
    def _reset_dofs(self, env_ids):
        # 默认姿态是全 0，父类的 0*rand 会退化成无随机化，改成绝对噪声
        self.dof_pos[env_ids] = self.default_dof_pos + \
            0.15 * torch_rand_float(-1., 1., (len(env_ids), self.num_dof), device=self.device)
        self.dof_vel[env_ids] = 0.
        env_ids_int32 = env_ids.to(dtype=torch.int32)
        self.gym.set_dof_state_tensor_indexed(
            self.sim, gymtorch.unwrap_tensor(self.dof_state),
            gymtorch.unwrap_tensor(env_ids_int32), len(env_ids_int32))
```

### 4.5 ⚠️ 我们的 URDF 没有「脚」这个 link

✅实测刚体名里**没有 `foot`**，只有 `base_link` + 每腿 `hip/thigh/calf`（共 13 个）。

而 legged_gym 靠 `foot_name` 找脚，用于：步态奖励 `feet_air_time`、`feet_stumble`、接触力惩罚。

**解法：`foot_name = "calf"`**（把小腿当脚），同时把 `penalize_contacts_on` 里的 `"calf"` **删掉**（否则一边用小腿当脚、一边惩罚小腿触地，奖励自相矛盾）。

配套：URDF 改碰撞体时，在 calf 末端加一个 **r=9 mm 的球**（对应真机足端橡胶球），这样 "calf 的接触" ≈ 真正的足端接触。

---

## 5. 从 a1 移植到 RL-CAT：逐项对照表

| 配置项 | a1 原值 | **RL-CAT 建议值** | 依据 |
|---|---|---|---|
| `init_state.pos` | `[0,0,0.42]` | `[0,0,0.20]` | ✅实测从 0.20 落到 0.13 稳定 |
| `default_joint_angles` | hip ±0.1, thigh 0.8/1.0, calf −1.5 | **全部 0.0** | ✅实测 q=0 = 站立 |
| `control.stiffness` | `{'joint': 20.}` | `{'hip':20.,'thigh':20.,'calf':20.}` | ✅实测 30/1.0 站得住，从 20 起调 |
| `control.damping` | `{'joint': 0.5}` | `{'hip':0.5,'thigh':0.5,'calf':0.5}` | 同上 |
| `action_scale` | 0.25 | **0.25（不变）** | 与文档 §12 一致 ✅ |
| `control.decimation` | 4 | **2** | sim dt=1/200 × 2 = **100 Hz**，对齐固件 |
| `sim.dt` | 0.005 | **0.005（不变）** | = 1/200，与文档 §7.4 一致 ✅ |
| `asset.foot_name` | `"foot"` | **`"calf"`** | URDF 无独立足端 link |
| `asset.penalize_contacts_on` | `["thigh","calf"]` | **`["thigh"]`** | calf 就是脚 |
| `asset.terminate_after_contacts_on` | `["base"]` | **`["base_link"]`** | 我们的根刚体叫 base_link |
| `asset.self_collisions` | 1 | 1（不变） | |
| `terrain.mesh_type` | `'trimesh'` | **`'plane'`（先平地）** | 先出走路再上地形 |
| `terrain.measure_heights` | True | **False** | 平地不需要 |
| `terrain.curriculum` | True | **False** | 平地无课程 |
| `env.num_observations` | 235 | **48**（或选 §4.1 的 47） | 235 含 187 维高度，平地要改 |
| `env.num_envs` | 4096 | **2048** 起步，稳了再加 | 8 GB 显存 |
| `env.episode_length_s` | 20 | 20（不变） | |
| `rewards.base_height_target` | 0.25 | **0.13（换碰撞体后必须重测，见 §3.8）** | ✅实测机身高度，但当前是三角网格碰撞下的值 |
| `rewards.torques` | `-0.0002` | `-0.0005` | 我们力矩上限 3 N·m，量级更小 |
| `rewards.dof_pos_limits` | `-10.0` | `-10.0`（不变） | |
| `commands.ranges.lin_vel_x` | `[-1.0, 1.0]` | **`[-0.4, 0.6]`** | 机身仅 200 mm 长、1.386 kg，1 m/s 太激进 |
| `commands.ranges.lin_vel_y` | `[-1.0, 1.0]` | **`[-0.2, 0.2]`** | 髋外摆只往一侧开，横移能力有限 |
| `commands.ranges.ang_vel_yaw` | `[-1, 1]` | **`[-0.6, 0.6]`** | |
| `policy.actor_hidden_dims` | `[512,256,128]` | **`[256,128,64]`** | 文档 §12 的设计值 |
| `policy.critic_hidden_dims` | `[512,256,128]` | **`[256,128,64]`** | 同上 |
| `runner.experiment_name` | `'rough_a1'` | **`'rlcat_flat'`** | |

---

## 6. 要写的代码（就这么多）

### 6.1 新建 `legged_gym/envs/rlcat/rlcat_config.py`

```python
from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class RlcatFlatCfg(LeggedRobotCfg):
    class init_state(LeggedRobotCfg.init_state):
        pos = [0.0, 0.0, 0.20]              # 实测：0.20 落下，稳定在 0.130
        default_joint_angles = {            # 实测确认 q=0 就是站立姿态
            'lf_hip': 0.0, 'lb_hip': 0.0, 'rf_hip': 0.0, 'rb_hip': 0.0,
            'lf_thigh': 0.0, 'lb_thigh': 0.0, 'rf_thigh': 0.0, 'rb_thigh': 0.0,
            'lf_calf': 0.0, 'lb_calf': 0.0, 'rf_calf': 0.0, 'rb_calf': 0.0,
        }

    class control(LeggedRobotCfg.control):
        control_type = 'P'
        stiffness = {'hip': 20., 'thigh': 20., 'calf': 20.}
        damping   = {'hip': 0.5, 'thigh': 0.5, 'calf': 0.5}
        action_scale = 0.25
        decimation = 2                       # 1/200 * 2 = 100 Hz，对齐固件

    class asset(LeggedRobotCfg.asset):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/rlcat/urdf/robot.urdf'
        name = "rlcat"
        foot_name = "calf"                   # URDF 无独立足端 link
        penalize_contacts_on = ["thigh"]
        terminate_after_contacts_on = ["base_link"]
        self_collisions = 1

    class terrain(LeggedRobotCfg.terrain):
        mesh_type = 'plane'
        measure_heights = False
        curriculum = False

    class commands(LeggedRobotCfg.commands):
        curriculum = True
        class ranges(LeggedRobotCfg.commands.ranges):
            lin_vel_x = [-0.4, 0.6]
            lin_vel_y = [-0.2, 0.2]
            ang_vel_yaw = [-0.6, 0.6]
            heading = [-3.14, 3.14]

    class rewards(LeggedRobotCfg.rewards):
        base_height_target = 0.13            # 三角网格碰撞下实测；换碰撞体后重测！见 §3.8
        soft_dof_pos_limit = 0.9
        class scales(LeggedRobotCfg.rewards.scales):
            torques = -0.0005
            dof_pos_limits = -10.0
            stand_still = -0.5               # 阶段一用来先学会站住

    class env(LeggedRobotCfg.env):
        num_envs = 2048
        num_observations = 48                # 若采用 §4.1 方案 C 则填 47
        episode_length_s = 20

    class normalization(LeggedRobotCfg.normalization):
        class obs_scales(LeggedRobotCfg.normalization.obs_scales):
            # ⚠️ legged_gym 默认 dof_vel=0.05，交接文档 §12 要求 0.1。
            #    仿真和固件必须一致，这里显式覆盖成文档值。
            dof_vel = 0.1
            ang_vel = 0.25
            dof_pos = 1.0

class RlcatFlatCfgPPO(LeggedRobotCfgPPO):
    class policy(LeggedRobotCfgPPO.policy):
        actor_hidden_dims  = [256, 128, 64]  # 文档 §12 设计值
        critic_hidden_dims = [256, 128, 64]
    class algorithm(LeggedRobotCfgPPO.algorithm):
        entropy_coef = 0.01
    class runner(LeggedRobotCfgPPO.runner):
        experiment_name = 'rlcat_flat'
        max_iterations = 1000
```

### 6.2 新建 `legged_gym/envs/rlcat/rlcat_env.py`

```python
import torch
from isaacgym import gymtorch
from isaacgym.torch_utils import torch_rand_float
from legged_gym.envs.base.legged_robot import LeggedRobot

class RlcatRobot(LeggedRobot):
    def _reset_dofs(self, env_ids):
        """默认姿态是全 0，父类的 default*rand 会退化成无随机化，改为绝对噪声。"""
        self.dof_pos[env_ids] = self.default_dof_pos + \
            0.15 * torch_rand_float(-1., 1., (len(env_ids), self.num_dof), device=self.device)
        self.dof_vel[env_ids] = 0.
        env_ids_int32 = env_ids.to(dtype=torch.int32)
        self.gym.set_dof_state_tensor_indexed(
            self.sim, gymtorch.unwrap_tensor(self.dof_state),
            gymtorch.unwrap_tensor(env_ids_int32), len(env_ids_int32))

    # 若采用 §4.1 方案 C（47 维观测），再覆写 compute_observations。
    # ⚠️ 注意：legged_gym 基类里【没有】sin_phase / cos_phase 这两个 buffer，
    #    需要在 __init__ 里自己建（按步数累加的步态时钟），否则会 AttributeError。
    #
    # ⚠️ 维度顺序刻意【对齐交接文档 §12】，这样固件侧的拼接顺序不用改，
    #    且 C 方案正好 = 文档 50 维设计的前 47 维（后 3 个"保留维"留空）。
    #
    # def __init__(self, cfg, sim_params, physics_engine, sim_device, headless):
    #     super().__init__(cfg, sim_params, physics_engine, sim_device, headless)
    #     self.phase = torch.zeros(self.num_envs, device=self.device)   # 0~1 循环
    #
    # 每个 step 后推进相位；步频先给固定值，后面可以做成可学习/随速度变化：
    #     self.phase = (self.phase + self.cfg.control.decimation
    #                   * self.sim_params.dt * freq) % 1.0
    #     self.sin_phase = torch.sin(2 * torch.pi * self.phase).unsqueeze(1)
    #     self.cos_phase = torch.cos(2 * torch.pi * self.phase).unsqueeze(1)
    #
    # def compute_observations(self):
    #     self.obs_buf = torch.cat((
    #         self.projected_gravity,            # 0-2   重力向量（IMU 姿态反算）
    #         self.base_ang_vel * 0.25,          # 3-5   机身角速度（陀螺）  ← 陀螺仪在这里
    #         self.commands[:, :3],              # 6-8   速度指令
    #         (self.dof_pos - self.default_dof_pos) * 1.0,   # 9-20  关节位置
    #         self.dof_vel * 0.1,                # 21-32 关节速度
    #         self.actions,                      # 33-44 上一帧动作（原始策略输出）
    #         self.sin_phase, self.cos_phase,    # 45-46 步态相位（内部时钟）
    #     ), dim=-1)                             # = 47
    #
    # 说明：文档 §12 写的"上一帧动作 ×4.0"与这里的裸 self.actions 是等价的 ——
    #   文档指的是固件存的是已乘 0.25 的弧度值，再乘 4.0 还原成策略原始输出量级。
```

### 6.3 改 `legged_gym/envs/__init__.py`（加 2 行 import + 1 行注册）

```python
from legged_gym.envs.rlcat.rlcat_config import RlcatFlatCfg, RlcatFlatCfgPPO
from legged_gym.envs.rlcat.rlcat_env import RlcatRobot
task_registry.register("rlcat", RlcatRobot, RlcatFlatCfg(), RlcatFlatCfgPPO())
```

### 6.4 资产拷贝

```
legged_gym/resources/robots/rlcat/urdf/
├── robot.urdf          ← 拷贝，或拷贝后改过的 robot_sim.urdf
└── meshes/part_*.stl   ← 26 个，必须与 urdf 同级
```
⚠️ 若文件名改成 `robot_sim.urdf`，`cfg.asset.file` 要同步改。

---

## 7. URDF 改造清单（P2 阶段）

**原则：不要原地改 `~/Desktop/RL/urdf/robot.urdf`**（那是 CAD 导出的可追溯基线）。新建一份训练专用副本，例如 `robot_sim.urdf`。

| # | 改动 | 必须性 | 说明 |
|---|---|---|---|
| 1 | `*_thigh` 的 `upper`：`0.52359878`(30°) → `1.5707963`(90°) | 建议 | 不是阻塞（§1.2 已证），但 30° 会限制摆腿幅度 |
| 2 | 4 个 `<collision>` 从 STL 换成 **box**（base/thigh/calf） | **强烈建议** | PhysX 处理三角网格极慢，4096 环境会被拖垮 |
| 3 | calf 末端加 **r=9 mm sphere** 作为足端碰撞球 | **强烈建议** | 对应真机足端橡胶球，接触检测才准 |
| 4 | （可选）`lf_thigh` origin z `0.0586` → `0.0571` | 可选 | 消除 1.5 mm CAD 不对称 |
| 5 | （可选）收紧 `velocity` `8.0` → `4.71` | 可选 | 真机空载转速上限 |

**改动 2/3 的做法**：直接编辑 URDF 的 `<collision>` 段。URDF 里 visual 仍用 `<mesh>`（保留精细外观），collision 换成 `<box>`/`<sphere>` —— 两者是独立标签，互不影响。

#### ⭐ 足端碰撞球的确切坐标（已算好，直接可用）

下面 4 个球心已按「球的**底面**正好与现有 mesh 的最低点重合」标定，配 `radius="0.009"`（真机足端橡胶球半径）：

```xml
<!-- lf_calf 的 <collision> 里 -->
<origin xyz="0.071332 -0.079404 0.012500" rpy="0 0 0" />
<geometry><sphere radius="0.009" /></geometry>

<!-- lb_calf -->
<origin xyz="0.075518 -0.074932 0.012500" rpy="0 0 0" />
<geometry><sphere radius="0.009" /></geometry>

<!-- rf_calf -->
<origin xyz="0.072495 -0.078210 -0.004500" rpy="0 0 0" />
<geometry><sphere radius="0.009" /></geometry>

<!-- rb_calf -->
<origin xyz="0.077654 -0.072452 -0.004500" rpy="0 0 0" />
<geometry><sphere radius="0.009" /></geometry>
```

⚠️ 这些坐标是**在各自 calf link 坐标系里**的，不是 base 坐标系，也不是臂长。原因是本项目的 link 坐标系没有对齐连杆方向（小腿 mesh 在自身坐标系里沿 (x,−y) 对角线延伸），所以不能凭直觉填「(0, 0, 腿长)」。

⚠️ **每个 calf 还需要保留一段 shank（小腿本体）的碰撞体**（box 或 capsule），否则小腿会穿过障碍物，只有脚底有碰撞。

⚠️ **改完碰撞体后必须重测站立高度**（见 §3.8），再填 `base_height_target`。

⚠️ **改动 2/3 会改变物理，必须在正式训练前完成**，否则要重训。

---

## 8. 执行流程与命令

### P0 ✅ 环境自检（已完成）
```bash
conda activate isaacgym
python scripts/01_load_asset.py     # 13/12/1.386kg 核对
python scripts/03_stand_test.py     # q_default + PD 站立
```

### P1 ⬜ 安装训练框架（~20 min）
```bash
cd ~ && git clone https://github.com/leggedrobotics/legged_gym.git
cd legged_gym && pip install -e .

cd ~ && git clone https://github.com/leggedrobotics/rsl_rl.git
cd rsl_rl && git checkout v1.0.2 && pip install -e .     # ⚠️ tag 必须对
```
**先验性检查**：`python legged_gym/scripts/train.py --task=a1 --headless --num_envs=512 --max_iterations=5`
→ 官方 a1 能跑通，说明训练栈没问题，再换我们的机器人。**这一步别省**，否则出问题时分不清是框架问题还是我们的配置问题。
（a1 的 URDF 随仓库提供，无需额外下载资产；接口已核对兼容，但源码比对证明不了运行时无问题，这步是唯一的运行时证据。）

### P2 ⬜ URDF 改造（~30–60 min）
1. 按 §7 生成 `robot_sim.urdf`（放宽 thigh 限位 + 换 box/sphere 碰撞体 + 加足端球）
2. 拷进 `resources/robots/rlcat/urdf/`（连同 `meshes/`）
3. ⭐ **改完碰撞体后，用 `03_stand_test.py` 重测站立高度**，得到新的 `base_height_target`（§3.8：换碰撞体后旧值 0.13 会失效）

### P3 ⬜ 接入 RL-CAT（~30 min）
写 §6.1 / §6.2 / §6.3 三个文件，然后：
```bash
python legged_gym/scripts/train.py --task=rlcat --headless --num_envs=64 --max_iterations=3
```
**只验证能否起来**：观察 debug 输出里 `default_dof_pos` 是否全 0、PD 增益提示行有没有出现（出现说明有关节没匹配上）。

### P4 ⬜ 阶段一：先学会站立（~10–20 min）
把 `commands.ranges` 全设为 `[0,0]`，`stand_still` 给负权重，`tracking_lin_vel` 先关掉。
```bash
python legged_gym/scripts/train.py --task=rlcat --headless --num_envs=2048 --max_iterations=300
```
**验收**：base_height ≈ 0.13±0.02 m，roll/pitch < 10°，`episode_length` 跑满。
**把站立调稳再开速度** —— 这一步能挡掉 90% 的"奖励写错了"类问题。

### P5 ⬜ 阶段二：速度跟踪行走（~30 min – 2 h）
恢复 `tracking_lin_vel=1.0` / `tracking_ang_vel=0.5` / `feet_air_time=1.0`，打开 §5 的指令范围。
```bash
python legged_gym/scripts/train.py --task=rlcat --headless --num_envs=2048 --max_iterations=1000
```

### P6 ⬜ 可视化与评估
```bash
python legged_gym/scripts/play.py --task=rlcat            # 开窗口看
# 训练中途也可以随时开 viewer 看：去掉 --headless
```
**验收**：跟踪 vx 误差 < 0.1 m/s；连续 20 s 不摔；左右腿 air_time 相近（步态对称）。

---

## 9. 资源预算（RTX 4060 Laptop 8 GB）

| 项 | 估计 | 说明 |
|---|---|---|
| 显存占用 @2048 envs | ~2–4 GB | 13 刚体的小机器人，宽裕 |
| 显存占用 @4096 envs | ~4–7 GB | 可能吃紧，注意观察 |
| 单次迭代（24 步 × 2048 envs） | ~0.3–1 s | |
| 训练到能走（~800–1000 iter） | **约 20–60 min** | 平地四足的经验区间 |
| 磁盘 | ~1 GB | logs + checkpoint |

⚠️ 训练务必 **headless**（`--headless`），开渲染会抢显存和算力。`max_gpu_contact_pairs` 默认 `2**23`，2048 环境下够用。

---

## 10. 风险清单

| # | 风险 | 严重度 | 应对 |
|---|---|---|---|
| 1 | ~~`rsl_rl` 版本搞错，API 对不上~~ | ✅ **已排除** | 已逐项核对源码，legged_gym master ↔ rsl_rl v1.0.2 接口全吻合（§3.5）。仍保留 a1 冒烟测试作为运行时验证 |
| 2 | 观测含真机测不到的机身线速度 → sim-to-real 崩 | 🔴 高 | 选 §4.1 方案 B 或 C 删掉 |
| 3 | `default_dof_pos=0` 导致 reset 无随机化 | 🟡 中 | §4.4 覆写 `_reset_dofs` |
| 4 | STL 碰撞体拖垮 PhysX | 🔴 **高**（已升级） | §7 改动 2/3。已证实 visual/collision mesh 几何完全相同，即当前是 600+ 顶点全精度三角网格碰撞（§3.7） |
| 5 | `use_gpu_pipeline=True` 下误用非 tensor API | 🟡 中 | ✅已踩过，调试脚本设 False |
| 6 | PD 增益子串没匹配上 → 增益静默为 0 → 机器人瘫 | 🟡 中 | 检查有无 "PD gain ... not defined" 提示 |
| 7 | URDF 无 foot link，`feet_air_time` 奖励取不到 | 🟡 中 | `foot_name="calf"` + 删 `penalize_contacts_on` 里的 calf |
| 8 | 8 GB 显存，envs 开太大 OOM | 🟡 中 | 2048 起步 |
| 9 | ~~numpy 1.24 vs 老代码 `np.float` 弃用~~ | ✅ **已排除** | 全仓库扫描无任何已移除别名（§3.6） |
| 10 | `nvidia-smi` 偶发连不上（笔记本 dGPU 电源管理） | 🟢 低 | ✅今天遇到过一次，重试即恢复 |
| 11 | 训练到一半忘了 `--headless`，慢一倍 | 🟢 低 | 写进脚本 |
| 12 | legged_gym master 是重构版，文档/教程多按旧版写 | 🟡 中 | 对照源码而非博客；教程里 `VecEnv` 相关说法对本版不适用（§3.5） |
| 13 | 碰撞体一换，`base_height_target` 失效 → 奖励目标错 | 🟡 中 | §3.8：改完碰撞体先重测站立高度再填 |
| 14 | 观测缩放不一致：legged_gym `dof_vel=0.05` vs 文档 §12 要求 `0.1` | 🟡 中 | §4.1 量纲表：配置里显式覆盖成 0.1，仿真/固件统一 |

---

## 11. 待你决策

| # | 决策点 | 选项 | 我的建议 |
|---|---|---|---|
| 1 | **观测接口** | A 原生 48 / **B 删线速度 45** / **C 45+相位 47** / D 严格按文档 50 | **C（47 维）** —— 全部真机可复现，且不为凑数硬塞 |
| 2 | URDF 改动方式 | 原地改 / **新建 `robot_sim.urdf`** | **新建**，保留 CAD 基线可追溯 |
| 3 | 地形节奏 | 平地直接上 / **平地 → 上地形** | **先平地**，快速拿到"能走"的正反馈 |
| 4 | 是否先跑官方 a1 验证 | 跳过 / **先跑 5 个 iteration** | **先跑**，20 分钟买一个"训练栈没问题"的结论 |

---

## 12. 与交接文档的差异汇总（请同步更新交接文档）

| # | 交接文档的说法 | 实测/核实结果 |
|---|---|---|
| 1 | 🔴 阻塞：thigh +30° < 站立需要 +31.19° | ❌ **误判**。绝对连杆角 vs 关节转角混淆。q=0 站得住（实测） |
| 2 | 🟡 DOF 顺序可能等于 URDF 声明顺序（B 表） | ❌ 实测是第三种（lb/lf/rb/rf 分组）；且训练侧按名字索引，**不需要映射** |
| 3 | 🔴 q_default 待验证，推断 `[0]*12` | ✅ **已证实** |
| 4 | §5.6 站立姿态 thigh +31.19° / calf +25.76° | ⚠️ 是绝对几何角，不是关节角；关节角在 q=0 时为 0 |
| 4b | §3.3「髋俯仰轴→膝轴 = 63.61~64.96mm（大腿）」「膝轴→足端 = 75.78mm（小腿）」 | ❌ **张冠李戴**。URDF 里 `*_thigh` joint origin \|d\|=64.96mm 是**髋外摆轴→髋俯仰轴**；`*_calf` joint origin \|d\|=75.78mm 才是**髋俯仰轴→膝轴 = 大腿长**。所以**大腿 = 75.78 mm**，小腿长度 URDF 里没有（要看 mesh）。§5.6 的余弦定理把 63.61 当大腿、75.78 当小腿，**输入本身就是错的** —— 这正是它算出「站立超限位」却与实际不符的根因 |
| 5 | §3.1 站立髋高 158.4 mm | ⚠️ 几何预测机身 145.5 mm，**仿真实测 130.0 mm**（差 15.5mm 是三角网格足端陷入地面，§3.8） |
| 6 | §12 观测 50 维 | ⚠️ 其中 3 维「保留」未定义；legged_gym 原生 48 维含真机测不到的线速度，需取舍 |
| 7 | 未提及 | ⚠️ 新增坑：GPU pipeline 下禁用非 tensor API |
| 8 | 未提及 | ⚠️ 新增坑：URDF 无 foot link，需用 calf 充当 |
| 9 | 未提及 | ⚠️ 新增坑：默认姿态全 0 导致 reset 随机化退化 |
| 10 | §9 步骤 8 PD 站立测试 | ✅ **提前通过**（stiffness 30 / damping 1.0） |
| 11 | §5.1「visual/collision 网格分开导出」 | ⚠️ 文件确实分两份，但**几何完全相同**（同顶点数、同最低点）。意味着当前碰撞体是全精度三角网格（§3.7） |
| 12 | §10 路线图跳过「训练框架选型与版本锁定」 | ⚠️ 实际这里是最大的一次性风险。已核实 legged_gym master ↔ rsl_rl v1.0.2 接口全吻合（§3.5） |
| 13 | §13 提醒 9「numpy 兼容」类担忧 | ✅ 已全仓库扫描，无任何已移除 numpy 别名（§3.6） |
