"""RL-CAT 环境：在 legged_gym 的 LeggedRobot 上做三处必要覆写

1. _reset_dofs            —— 默认姿态全 0，父类的 `default * rand` 退化成无随机化
2. _get_noise_scale_vec   —— 基类那份是按它自己的 48 维布局写死的，我们是 47 维
3. compute_observations   —— 方案 C 的 47 维布局，对齐交接文档 §12

其余全部复用 legged_gym：奖励、课程、域随机化、tensor API 交互。
"""
import math

import torch

from isaacgym import gymtorch
from isaacgym.torch_utils import torch_rand_float

from legged_gym.envs.base.legged_robot import LeggedRobot


class RlcatRobot(LeggedRobot):

    # ------------------------------------------------------------------ 初始化
    def __init__(self, cfg, sim_params, physics_engine, sim_device, headless):
        super().__init__(cfg, sim_params, physics_engine, sim_device, headless)
        # 步态相位时钟。它是【内部时钟】，真机固件不需要任何传感器就能复现，
        # 所以放进观测不会制造 sim-to-real 缺口。
        self.phase = torch.zeros(self.num_envs, device=self.device)
        self.gait_freq = float(getattr(cfg.control, "gait_freq", 1.5))

        # 每条足端在 trot 里的期望相位符号（+1 / -1），用于 _reward_gait_phase。
        # trot 的配对规则：对角腿同相 —— 左前+右后 一相，右前+左后 另一相。
        # 不能硬编码足端顺序（Isaac 会重排刚体），运行时按名字推。
        try:
            names = self.gym.get_actor_rigid_body_names(self.envs[0], self.actor_handles[0])
            idxs = self.feet_indices.cpu().numpy().tolist()
            signs = []
            for i in idxs:
                n = names[i]
                is_front = n.startswith("lf") or n.startswith("rf")
                is_left = n.startswith("lf") or n.startswith("lb")
                signs.append(1.0 if (is_front == is_left) else -1.0)
            self.phase_sign = torch.tensor(signs, device=self.device)
            print("[rlcat] 足端顺序 =", [names[i] for i in idxs])
            print("[rlcat] trot 期望相位符号 =", signs)
        except Exception as e:              # 兜底：退化成"不启用相位奖励"
            print("[rlcat] 无法推断足端相位符号，gait_phase 将恒为 0:", e)
            self.phase_sign = None

    # ------------------------------------------------------- 覆写 1：初始状态随机化
    def _reset_dofs(self, env_ids):
        """默认姿态是全 0（实测 q=0 即站立姿态）。

        父类写的是 `self.default_dof_pos * torch_rand_float(0.5, 1.5, ...)`，
        a1 的默认姿态非零所以有效；我们这里是 0 * rand == 0，
        等于每个 episode 都从精确的站立姿态出发，没有任何初始扰动。
        改成围绕默认姿态的绝对噪声。
        """
        self.dof_pos[env_ids] = self.default_dof_pos + \
            0.15 * torch_rand_float(-1.0, 1.0, (len(env_ids), self.num_dof),
                                    device=self.device)
        self.dof_vel[env_ids] = 0.0

        # 相位时钟在每个 episode 开始时归零（各环境 reset 时刻不同，天然去相关）
        if hasattr(self, "phase"):
            self.phase[env_ids] = 0.0

        env_ids_int32 = env_ids.to(dtype=torch.int32)
        self.gym.set_dof_state_tensor_indexed(
            self.sim,
            gymtorch.unwrap_tensor(self.dof_state),
            gymtorch.unwrap_tensor(env_ids_int32),
            len(env_ids_int32))

    # --------------------------------------------------- 覆写 2：噪声尺度（47 维）
    def _get_noise_scale_vec(self, cfg):
        """基类版本按 48 维布局（lin_vel/ang_vel/gravity/...）逐段硬编码下标，
        我们用 47 维（gravity/ang_vel/commands/...），下标全变了，必须重写。"""
        n = self.cfg.env.num_observations
        noise_vec = torch.zeros(n, device=self.device)
        self.add_noise = self.cfg.noise.add_noise
        ns = self.cfg.noise.noise_scales
        nl = self.cfg.noise.noise_level
        sc = self.obs_scales

        noise_vec[0:3] = ns.gravity * nl                        # 重力向量
        noise_vec[3:6] = ns.ang_vel * nl * sc.ang_vel           # 机身角速度（陀螺）
        noise_vec[6:9] = 0.0                                    # 速度指令（指令本身无噪声）
        noise_vec[9:21] = ns.dof_pos * nl * sc.dof_pos          # 关节位置
        noise_vec[21:33] = ns.dof_vel * nl * sc.dof_vel         # 关节速度
        noise_vec[33:47] = 0.0                                  # 上一帧动作 + 相位
        return noise_vec

    # ------------------------------------------- 覆写 3：观测（方案 C，47 维）
    def compute_observations(self):
        """顺序刻意对齐交接文档 §12，这样固件侧拼接顺序不用改：
             0-2   重力向量（IMU 姿态反算）
             3-5   机身角速度（陀螺）
             6-8   速度指令
             9-20  关节位置（相对 q_default）
            21-32  关节速度
            33-44  上一帧动作
            45-46  步态相位 sin/cos
        """
        # 推进相位时钟：每个策略步一次，dt = decimation * sim.dt
        self.phase = (self.phase + self.dt * self.gait_freq) % 1.0
        sin_p = torch.sin(2.0 * math.pi * self.phase).unsqueeze(-1)
        cos_p = torch.cos(2.0 * math.pi * self.phase).unsqueeze(-1)

        self.obs_buf = torch.cat((
            self.projected_gravity,                                            # 0-2
            self.base_ang_vel * self.obs_scales.ang_vel,                       # 3-5
            self.commands[:, :3] * self.commands_scale,                        # 6-8
            (self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos,   # 9-20
            self.dof_vel * self.obs_scales.dof_vel,                            # 21-32
            self.actions,                                                      # 33-44
            sin_p, cos_p,                                                      # 45-46
        ), dim=-1)

        if self.add_noise:
            self.obs_buf += (2.0 * torch.rand_like(self.obs_buf) - 1.0) \
                * self.noise_scale_vec

    # ------------------------------------- 覆写 4：抬脚滞空奖励的尺度适配
    def _reward_feet_air_time(self):
        """legged_gym 把目标滞空时间【硬编码】成 0.5 s —— 那是给 anymal/a1
        （几十 kg、腿长半米）调的。RL-CAT 机身只有 200 mm、1.386 kg，
        自然步频高得多，0.5 s 的目标会让这一项长期为负，等于【在惩罚抬脚】。

        实测后果（model_650 步态分析）：策略发现"永远不落地"最优 ——
        退化成了两条腿蹦（右后腿触地占空比 0.00，机身高度冲到 0.202 m）。
        这里改成从 cfg 读。
        """
        contact = self.contact_forces[:, self.feet_indices, 2] > 1.0
        contact_filt = torch.logical_or(contact, self.last_contacts)
        self.last_contacts = contact
        first_contact = (self.feet_air_time > 0.0) * contact_filt
        self.feet_air_time += self.dt

        target = float(getattr(self.cfg.rewards, "feet_air_time_target", 0.5))
        rew_air_time = torch.sum((self.feet_air_time - target) * first_contact, dim=1)
        rew_air_time *= torch.norm(self.commands[:, :2], dim=1) > 0.1
        self.feet_air_time *= ~contact_filt
        return rew_air_time

    def _reward_feet_air_excess(self):
        """堵住 _reward_feet_air_time 的漏洞。

        原奖励只在【落地瞬间】结算：一直抬着不落地的脚既拿不到奖励、也不受罚，
        于是"把两条腿永久抬着用另外两条蹦"成了局部最优。
        这里显式惩罚"滞空时间超过目标两倍"的脚，把这个漏洞堵死。
        """
        contact = self.contact_forces[:, self.feet_indices, 2] > 1.0
        target = float(getattr(self.cfg.rewards, "feet_air_time_target", 0.5))
        excess = torch.clamp(self.feet_air_time - 2.0 * target, min=0.0, max=0.5)
        # 乘上"正在腾空"掩码：只有违反约束的脚才被罚。
        # 上限 0.5 是为了让惩罚有界，避免再次出现"某项奖励爆炸 -> 策略被摧毁"。
        return torch.sum(excess * (~contact).float(), dim=1)

    # ------------------------------- 覆写 5：给会爆炸的平方惩罚加截断
    # 实测：iter 681 训练崩溃，机器人被弹飞 -> base_height 平方惩罚冲到 -2417、
    # lin_vel_z -277、ang_vel_xy -408，天量优势值直接把策略摧毁。
    # 这些项都是【平方】形式，越界后增长极快，必须截断。
    def _reward_base_height(self):
        """改用【绝对偏差】而不是平方偏差。

        平方形式在目标附近梯度趋零：实测机身掉到 0.102 m（目标 0.13）、
        起跳到 0.202 m，平方惩罚却只有 -0.005，完全压不住蹦跳。
        绝对偏差在整个区间都有恒定梯度，才能把姿态钉住。
        """
        base_height = torch.mean(
            self.root_states[:, 2].unsqueeze(1) - self.measured_heights, dim=1)
        err = base_height - self.cfg.rewards.base_height_target
        return torch.abs(torch.clip(err, -0.15, 0.15))

    def _reward_lin_vel_z(self):
        return torch.square(torch.clip(self.base_lin_vel[:, 2], -1.5, 1.5))

    def _reward_ang_vel_xy(self):
        return torch.sum(
            torch.square(torch.clip(self.base_ang_vel[:, :2], -4.0, 4.0)), dim=1)

    def _reward_feet_airborne(self):
        """直接惩罚"三只以上脚同时腾空"。

        实测 model_1000：平均只有 1.27 只脚着地，60 个时刻四脚全腾空，
        机身高度冲到 0.202 m —— 这是【蹦】不是【走】。
        抬脚时间奖励只能间接约束，这里直接对腾空脚数下手。
        """
        contact = self.contact_forces[:, self.feet_indices, 2] > 1.0
        n_air = (~contact).sum(dim=1).float()
        return torch.clamp(n_air - 2.0, min=0.0)

    def _reward_gait_phase(self):
        """显式约束 trot 结构：对角腿同相、相邻腿反相。

        动机：抬脚时间奖励只能告诉策略"该抬脚"，不能告诉它"哪两条腿该同时抬"。
        实测策略收敛到的是一个【跛行小跑】—— 确实是对角配对，但两侧严重不均
        （触地占空比 lb 0.71 / rf 0.43 / rb 0.14 / lf 0.03）。
        这里用观测里已有的相位时钟直接给出期望触地模式，
        让策略不必自己摸索四腿协调关系。

        返回值域 [-1, 1]：+1 = 四条腿全部符合期望相位，-1 = 全部相反。

        ⚠️ 必须按指令强度门控：实测（run 6）不加门控时，策略发现
        "原地踏步刷满相位奖励"比"真的往前走"更划算 ——
        cmd=0 时它甚至以 −0.20 m/s 倒退，速度增益只剩 0.5 且带 −0.20 的偏移。
        """
        if self.phase_sign is None:
            return torch.zeros(self.num_envs, device=self.device)
        s = torch.sin(2.0 * math.pi * self.phase).unsqueeze(1)     # (N,1)
        want = torch.sign(s) * self.phase_sign.unsqueeze(0)        # (N,4) >0 期望着地
        contact = (self.contact_forces[:, self.feet_indices, 2] > 1.0).float()
        actual = contact * 2.0 - 1.0                               # +1 着地 / -1 腾空
        rew = torch.mean(want * actual, dim=1)
        # 门控：指令越强，相位奖励才有意义（0 指令时不给，避免"原地踏步刷分"）
        cmd_mag = torch.norm(self.commands[:, :2], dim=1)
        return rew * torch.clamp(cmd_mag / 0.3, 0.0, 1.0)
