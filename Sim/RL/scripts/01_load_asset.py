"""RL-CAT / Isaac Gym 加载自检脚本（对应交接文档 §9 步骤 3 + 步骤 5 的一部分）

用途：无渲染加载 robot.urdf，打印所有关键属性，验证：
  1) Isaac Gym 能在本机 GPU 上跑起来（RTX 4060 = sm_89，Preview 4 官方构建的兼容性命门）
  2) 刚体数 13 / DOF 数 12 / 总质量 1.386 kg
  3) DOF 顺序 是否等于 URDF 声明顺序（交接文档 §2.4 的 B 表）
  4) q = [0]*12 时四足是否着地（判断零位是不是站立姿态）

用法：
    source ~/miniconda/etc/profile.d/conda.sh && conda activate isaacgym
    python scripts/01_load_asset.py
"""
import os
import numpy as np
from isaacgym import gymapi, gymutil

ASSET_ROOT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "urdf")
URDF_FILE = "robot.urdf"

# 交接文档 §2.4 B 表：URDF 声明顺序
B_NAMES = [
    "lf_hip", "lf_thigh", "lf_calf",
    "lb_hip", "lb_thigh", "lb_calf",
    "rf_hip", "rf_thigh", "rf_calf",
    "rb_hip", "rb_thigh", "rb_calf",
]

# 交接文档 §2.4 A 表：固件/训练逻辑索引
A_NAMES = [
    "lf_hip", "lb_hip", "rf_hip", "rb_hip",
    "lf_thigh", "lb_thigh", "rf_thigh", "rb_thigh",
    "lf_calf", "lb_calf", "rf_calf", "rb_calf",
]


def build_sim(gym):
    sp = gymapi.SimParams()
    sp.dt = 1.0 / 200.0          # 物理 200 Hz
    sp.substeps = 2              # 控制 100 Hz（与固件一致）
    sp.up_axis = gymapi.UP_AXIS_Z
    sp.gravity = gymapi.Vec3(0.0, 0.0, -9.81)
    sp.physx.solver_type = 1
    sp.physx.num_position_iterations = 4
    sp.physx.num_velocity_iterations = 1
    sp.physx.use_gpu = True
    sp.use_gpu_pipeline = True
    sim = gym.create_sim(0, 0, gymapi.SIM_PHYSX, sp)
    if sim is None:
        raise RuntimeError("create_sim 失败 —— GPU / CUDA 层有问题")
    return sim


def main():
    gym = gymapi.acquire_gym()
    sim = build_sim(gym)
    print("[OK] sim 创建成功（GPU PhysX 可用）")

    opts = gymapi.AssetOptions()
    opts.fix_base_link = True          # 先焊死底座，只看姿态
    opts.default_dof_drive_mode = gymapi.DOF_MODE_POS
    opts.use_mesh_materials = True
    opts.vhacd_enabled = False
    opts.replace_cylinder_with_capsule = True
    opts.thickness = 0.001

    asset = gym.load_asset(sim, ASSET_ROOT, URDF_FILE, opts)
    if asset is None:
        raise RuntimeError("load_asset 失败，检查 %s" % ASSET_ROOT)

    n_body = gym.get_asset_rigid_body_count(asset)
    n_dof = gym.get_asset_dof_count(asset)
    body_names = gym.get_asset_rigid_body_names(asset)
    dof_names = list(gym.get_asset_dof_names(asset))
    props = gym.get_asset_dof_properties(asset)

    print("\n===== 基本量 =====")
    print("刚体数 : %d   (期望 13)" % n_body)
    print("DOF  数 : %d   (期望 12)" % n_dof)
    print("刚体名 :", body_names)
    print("DOF  名 :", dof_names)

    print("\n===== DOF 顺序核对 =====")
    if dof_names == B_NAMES:
        print("[OK] DOF 顺序 == URDF 声明顺序（交接文档 §2.4 B 表）")
        A2C = [dof_names.index(n) for n in A_NAMES]
        print("逻辑索引 A -> Isaac DOF 索引 C 的映射 A2C =", A2C)
    else:
        print("[!!] DOF 顺序【不等于】URDF 声明顺序，必须做映射")
        print("     URDF 声明顺序 :", B_NAMES)
        print("     Isaac 实际顺序:", dof_names)
        if sorted(dof_names) == sorted(B_NAMES):
            A2C = [dof_names.index(n) for n in A_NAMES]
            print("     逻辑索引 A -> Isaac DOF 索引 C 的映射 A2C =", A2C)
        else:
            print("     [!!] 名字集合都对不上，需要人工检查")

    print("\n===== 关节限位 / 驱动器 =====")
    print("%-12s %9s %9s %8s %8s" % ("name", "lower", "upper", "effort", "vel"))
    for i, n in enumerate(dof_names):
        print("%-12s %9.4f %9.4f %8.3f %8.3f"
              % (n, props["lower"][i], props["upper"][i],
                 props["effort"][i], props["velocity"][i]))

    thigh_idx = [i for i, n in enumerate(dof_names) if n.endswith("thigh")]
    bad = [dof_names[i] for i in thigh_idx if props["upper"][i] < np.deg2rad(31.19)]
    if bad:
        print("\n[!!] 以下 thigh 限位上限 < 站立所需 31.19°，站立姿态不可达：", bad)
        print("     -> 需要按交接文档 §9 步骤 6 放宽 upper")
    else:
        print("\n[OK] 所有 thigh 上限 >= 31.19°，站立姿态在限位内")

    # ---------- 建环境，看 q=[0]*12 的姿态 ----------
    print("\n===== q = [0]*12 姿态检查（交接文档 §9 步骤 5）=====")
    import math
    env = gym.create_env(sim, gymapi.Vec3(-1, -1, 0), gymapi.Vec3(1, 1, 1), 1)
    actor = gym.create_actor(env, asset, gymapi.Transform(), "cat", 0, 0)
    gym.set_actor_dof_properties(env, actor, props)

    rb_props = gym.get_actor_rigid_body_properties(env, actor)
    total_mass = sum(rb_props[i].mass for i in range(n_body))
    print("总质量 : %.4f kg (期望 1.386)" % total_mass)
    print("各刚体质量 :", ["%s=%.3f" % (body_names[i], rb_props[i].mass) for i in range(n_body)])

    dof_states = gym.get_actor_dof_states(env, actor, gymapi.STATE_ALL)
    dof_states["pos"] = np.zeros(n_dof)          # q = 0
    dof_states["vel"] = np.zeros(n_dof)
    gym.set_actor_dof_states(env, actor, dof_states, gymapi.STATE_ALL)
    # 用位置驱动锁在 0，防止重力把腿压弯后再判断
    props["stiffness"][:] = 200.0
    props["damping"][:] = 10.0
    props["driveMode"][:] = gymapi.DOF_MODE_POS
    gym.set_actor_dof_properties(env, actor, props)
    gym.set_actor_dof_position_targets(env, actor, np.zeros(n_dof, dtype=np.float32))

    for _ in range(200):                          # 跑 1 s（200 步物理）
        gym.simulate(sim)
        gym.fetch_results(sim, True)

    body_states = gym.get_actor_rigid_body_states(env, actor, gymapi.STATE_ALL)
    print("%-12s %9s %9s %9s" % ("body", "x", "y", "z"))
    for i, n in enumerate(body_names):
        p = body_states["pose"]["p"][i]
        print("%-12s %9.4f %9.4f %9.4f" % (n, p["x"], p["y"], p["z"]))

    # 足端最低点 vs 机身高度
    foot_z = [body_states["pose"]["p"][i]["z"] for i, n in enumerate(body_names) if n.endswith("calf")]
    base_z = body_states["pose"]["p"][body_names.index("base_link")]["z"]
    print("\n机身 z = %.4f m" % base_z)
    print("小腿(足端近似) z =", ["%.4f" % z for z in foot_z])
    print("\n判读：")
    print("  base_link z 若 ≈ 0.12~0.16 m 且四足接近同一水平面 -> q=0 就是站立姿态")
    print("  base_link z 若 ≈ 0.06 m 左右 -> 腿是折叠的；若 ≈ 0.14 m 且腿笔直 -> 接近伸直")
    print("  四条腿 z 差异大 -> 姿态不对称/歪斜，需要实测真实 q_default")


if __name__ == "__main__":
    main()
