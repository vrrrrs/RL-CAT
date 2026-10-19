"""RL-CAT (12 DOF 四足) 的 legged_gym 环境配置

移植自 a1_config.py，关键差异全部标注了实测依据：
  - default_joint_angles 全是 0 —— 已实测 q=0 就是站立姿态（机身 0.130 m 稳定站住）
  - foot_name = "calf"       —— URDF 没有独立足端 link（13 刚体里没有 foot）
  - decimation = 2           —— 1/200 * 2 = 100 Hz 策略频率，对齐固件
  - 观测 47 维（方案 C）      —— 见 rlcat_env.py 的 compute_observations
"""
import math

from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

JOINTS = ["lf_hip", "lb_hip", "rf_hip", "rb_hip",
          "lf_thigh", "lb_thigh", "rf_thigh", "rb_thigh",
          "lf_calf", "lb_calf", "rf_calf", "rb_calf"]


class RlcatFlatCfg(LeggedRobotCfg):
    class env(LeggedRobotCfg.env):
        num_envs = 2048
        num_observations = 47          # 方案 C，见 rlcat_env.py
        num_privileged_obs = None
        num_actions = 12
        env_spacing = 2.0
        episode_length_s = 20

    class init_state(LeggedRobotCfg.init_state):
        pos = [0.0, 0.0, 0.20]         # 实测：从 0.20 落下，稳定在 0.130
        default_joint_angles = {j: 0.0 for j in JOINTS}   # 实测 q=0 = 站立姿态

    class control(LeggedRobotCfg.control):
        control_type = 'P'
        # 实测 stiffness=30/damping=1.0 能稳住站立
        # 第三轮调优提到 40/1.5：策略学会了靠"弹跳"前进（机身高度冲到 0.202 m），
        # 提高关节刚度让腿更硬、不易被压弯反弹
        stiffness = {'hip': 20.0, 'thigh': 20.0, 'calf': 20.0}
        damping = {'hip': 0.5, 'thigh': 0.5, 'calf': 0.5}
        action_scale = 0.25            # 与交接文档 §12 一致
        decimation = 2                 # 1/200 * 2 = 100 Hz，对齐固件
        gait_freq = 3.0                # 步态相位时钟 [Hz]；实测策略主导接触频率约 6 Hz，
                                       # 对应腿周期 ~3 Hz，故设 3.0（真机为内部时钟，可复现）

    class asset(LeggedRobotCfg.asset):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/rlcat/urdf/robot_sim.urdf'
        name = "rlcat"
        foot_name = "calf"             # URDF 无足端 link，用小腿充当
        penalize_contacts_on = ["thigh"]
        terminate_after_contacts_on = ["base_link"]
        self_collisions = 1            # 1 = 关闭自碰撞
        flip_visual_attachments = False
        thickness = 0.001

    class terrain(LeggedRobotCfg.terrain):
        mesh_type = 'plane'
        measure_heights = False
        curriculum = False

    class commands(LeggedRobotCfg.commands):
        curriculum = True
        max_curriculum = 1.0
        num_commands = 4
        resampling_time = 8.0
        heading_command = True
        class ranges(LeggedRobotCfg.commands.ranges):
            # 机身仅 200 mm、1.386 kg。0.5 m/s = 2.5 倍体长/秒，偏激进，收到 0.4
            lin_vel_x = [-0.3, 0.5]
            lin_vel_y = [-0.2, 0.2]
            ang_vel_yaw = [-0.6, 0.6]
            heading = [-3.14, 3.14]

    class rewards(LeggedRobotCfg.rewards):
        only_positive_rewards = False   # 允许负奖励，站立早期更容易学到
        tracking_sigma = 0.10   # run9 单变量实验：锐化跟踪奖励，压欠响应
        soft_dof_pos_limit = 0.9
        base_height_target = 0.13       # 实测（robot_sim.urdf，胶囊碰撞体）
        max_contact_force = 15.0        # 本机总重才 13.6 N，upstream 的 100 N 形同虚设
        # ⚠️ upstream 在 _reward_feet_air_time 里硬编码 0.5 s，那是给几十 kg 的
        #    anymal/a1 调的。RL-CAT 只有 200 mm / 1.386 kg，步频高得多。
        #    实测 0.5 s 会让策略退化成"两条腿蹦"（右后腿永不落地）。
        #    按 L=0.14 m、步幅≈0.5L、速度 0.3 m/s 估，摆动相约 0.12~0.15 s。
        feet_air_time_target = 0.15
        class scales(LeggedRobotCfg.rewards.scales):
            termination = -0.0
            tracking_lin_vel = 3.0      # 提到 3.0：实测 1.0 时策略会为了相位/抬脚奖励
                                        # 牺牲前进（run6 速度增益只剩 0.5 且有 -0.2 偏移）
            tracking_ang_vel = 0.5
            lin_vel_z = -2.0
            ang_vel_xy = -0.05
            orientation = -0.5
            torques = -0.0005
            dof_vel = -0.0
            dof_acc = -2.5e-7
            base_height = -3.0          # 配合 env 里的【绝对偏差】版本；实测机身掉到 0.10
            feet_air_time = 1.0         # 让位给 gait_phase（run4/5 曾是 3.0）
            feet_air_excess = -3.0      # 惩罚"一直抬着不落地"的脚
            feet_airborne = 0.0         # 惩罚"三只以上脚同时腾空"（-2.0 会把策略压死）
            gait_phase = 1.0            # 显式 trot 相位约束，见 rlcat_env.py
            feet_contact_forces = 0.0   # 惩罚冲击性落地
            collision = -1.0
            feet_stumble = -0.0
            action_rate = -0.01
            stand_still = -0.0
            dof_pos_limits = -10.0

    class normalization(LeggedRobotCfg.normalization):
        clip_observations = 100.
        clip_actions = 100.
        class obs_scales(LeggedRobotCfg.normalization.obs_scales):
            # ⚠️ legged_gym 默认 dof_vel=0.05，交接文档 §12 要求 0.1，以文档为准
            lin_vel = 2.0
            ang_vel = 0.25
            dof_pos = 1.0
            dof_vel = 0.1

    class noise(LeggedRobotCfg.noise):
        add_noise = True
        noise_level = 1.0
        class noise_scales(LeggedRobotCfg.noise.noise_scales):
            dof_pos = 0.01
            dof_vel = 1.5
            lin_vel = 0.1
            ang_vel = 0.2
            gravity = 0.05

    class domain_rand(LeggedRobotCfg.domain_rand):
        randomize_friction = True
        friction_range = [0.5, 1.25]
        randomize_base_mass = False
        push_robots = True
        push_interval_s = 15
        max_push_vel_xy = 1.0

    class sim(LeggedRobotCfg.sim):
        dt = 0.005                     # 物理 200 Hz
        substeps = 1


class RlcatFlatCfgPPO(LeggedRobotCfgPPO):
    class policy(LeggedRobotCfgPPO.policy):
        init_noise_std = 1.0
        actor_hidden_dims = [256, 128, 64]    # 交接文档 §12 设计值
        critic_hidden_dims = [256, 128, 64]
        activation = 'elu'

    class algorithm(LeggedRobotCfgPPO.algorithm):
        entropy_coef = 0.01
        learning_rate = 1.e-3
        num_learning_epochs = 5
        num_mini_batches = 4
        schedule = 'adaptive'
        desired_kl = 0.01

    class runner(LeggedRobotCfgPPO.runner):
        policy_class_name = 'ActorCritic'
        algorithm_class_name = 'PPO'
        num_steps_per_env = 24
        max_iterations = 1000
        save_interval = 25          # 存密一点，防止崩溃时丢掉最好的检查点
        experiment_name = 'rlcat_flat'
        run_name = ''


# --------------------------------------------------------------------------
# 阶段一：只学站立（速度指令恒为 0）
# 目的：在开速度跟踪之前，先用最简单的任务验证「奖励函数 + 物理 + 观测」没写错。
# --------------------------------------------------------------------------
class RlcatStandCfg(RlcatFlatCfg):
    class commands(RlcatFlatCfg.commands):
        curriculum = False
        heading_command = False
        class ranges(RlcatFlatCfg.commands.ranges):
            lin_vel_x = [0.0, 0.0]
            lin_vel_y = [0.0, 0.0]
            ang_vel_yaw = [0.0, 0.0]
            heading = [0.0, 0.0]

    class rewards(RlcatFlatCfg.rewards):
        class scales(RlcatFlatCfg.rewards.scales):
            tracking_lin_vel = 0.0
            tracking_ang_vel = 0.0
            feet_air_time = 0.0        # 站着不该奖励抬脚
            stand_still = -0.5         # 指令为 0 时惩罚乱动
            base_height = -2.0         # 明确要求保持高度


class RlcatStandCfgPPO(RlcatFlatCfgPPO):
    class runner(RlcatFlatCfgPPO.runner):
        experiment_name = 'rlcat_stand'
        max_iterations = 300
