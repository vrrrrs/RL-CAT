"""RL-CAT 站立 / q_default 实测（交接文档 §9 步骤 5 + 步骤 8）

放开底座 + 加地面，用 PD 把 12 个关节锁在 q=0，跑几秒，看：
  - 机身高度是否稳定（决定 rewards.base_height_target）
  - roll/pitch 是否接近 0
  - 哪些刚体真的接触了地面（验证只有足端球着地，没有盒子顶起来）
  - 需要多大力矩撑住

用法:
    python scripts/03_stand_test.py                                  # 原始 robot.urdf
    python scripts/03_stand_test.py --urdf robot_sim.urdf            # 换碰撞体后的
    python scripts/03_stand_test.py --urdf robot_sim.urdf --sweep    # 扫一组 PD 增益
"""
import argparse
import os
import sys

import numpy as np
from isaacgym import gymapi

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rlcat_paths as P  # noqa: E402  ⭐ 统一路径解析，不写死个人目录

ASSET_ROOT_DEFAULT = P.URDF_DIR
DEG = 180.0 / np.pi


def quat_to_rpy(q):
    """Isaac Gym 四元数顺序为 (x, y, z, w)"""
    x, y, z, w = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def build(gym, stiffness, damping, asset_root, urdf, collision_group=1):
    sp = gymapi.SimParams()
    sp.dt = 1.0 / 200.0
    sp.substeps = 2
    sp.up_axis = gymapi.UP_AXIS_Z
    sp.gravity = gymapi.Vec3(0, 0, -9.81)
    sp.physx.solver_type = 1
    sp.physx.num_position_iterations = 4
    sp.physx.num_velocity_iterations = 1
    sp.physx.use_gpu = True
    sp.use_gpu_pipeline = False          # 关掉 GPU pipeline，方便直接读状态
    sim = gym.create_sim(0, 0, gymapi.SIM_PHYSX, sp)

    plane = gymapi.PlaneParams()
    plane.normal = gymapi.Vec3(0, 0, 1)
    plane.static_friction = 1.0
    plane.dynamic_friction = 1.0
    gym.add_ground(sim, plane)

    o = gymapi.AssetOptions()
    o.fix_base_link = False
    o.default_dof_drive_mode = int(gymapi.DOF_MODE_POS)
    o.use_mesh_materials = True
    o.vhacd_enabled = False
    o.replace_cylinder_with_capsule = True
    o.thickness = 0.001
    asset = gym.load_asset(sim, asset_root, urdf, o)
    if asset is None:
        raise RuntimeError("load_asset 失败: %s/%s" % (asset_root, urdf))

    env = gym.create_env(sim, gymapi.Vec3(-1, -1, 0), gymapi.Vec3(1, 1, 1), 1)
    pose = gymapi.Transform()
    pose.p = gymapi.Vec3(0, 0, 0.20)     # 起始抬高，自由落到地上
    # 第 6 个参数是碰撞组：1 = 关闭自碰撞，与 legged_gym 的 self_collisions=1 一致
    actor = gym.create_actor(env, asset, pose, "cat", 0, collision_group, 0)

    props = gym.get_asset_dof_properties(asset)
    props["driveMode"][:] = int(gymapi.DOF_MODE_POS)
    props["stiffness"][:] = stiffness
    props["damping"][:] = damping
    gym.set_actor_dof_properties(env, actor, props)

    n_dof = gym.get_asset_dof_count(asset)
    gym.set_actor_dof_position_targets(env, actor, np.zeros(n_dof, dtype=np.float32))
    body_names = list(gym.get_asset_rigid_body_names(asset))
    return sim, env, actor, n_dof, body_names


def run_once(args, stiffness, damping):
    gym = gymapi.acquire_gym()
    sim, env, actor, n_dof, body_names = build(
        gym, stiffness, damping, args.asset_root, args.urdf, args.self_collisions)
    i_base = body_names.index("base_link")
    n_steps = int(args.seconds * 200)

    print("\n=== 站立测试  %s  stiffness=%.1f damping=%.2f ===" % (args.urdf, stiffness, damping))
    print("%6s %9s %10s %10s %10s" % ("t(s)", "base_z", "roll(deg)", "pitch(deg)", "min_body_z"))

    for step in range(n_steps):
        gym.simulate(sim)
        gym.fetch_results(sim, True)
        if step % 100 == 0 or step == n_steps - 1:
            bs = gym.get_actor_rigid_body_states(env, actor, gymapi.STATE_ALL)
            bp = bs["pose"]["p"][i_base]
            roll, pitch, _ = quat_to_rpy(bs["pose"]["r"][i_base])
            zs = [bs["pose"]["p"][i]["z"] for i in range(len(body_names))]
            print("%6.2f %9.4f %10.2f %10.2f %10.4f"
                  % (step / 200.0, bp["z"], roll * DEG, pitch * DEG, min(zs)))

    bs = gym.get_actor_rigid_body_states(env, actor, gymapi.STATE_ALL)
    base_z = bs["pose"]["p"][i_base]["z"]
    roll, pitch, _ = quat_to_rpy(bs["pose"]["r"][i_base])

    # 哪些刚体接触了地面
    contacts = {}
    try:
        from isaacgym import gymtorch
        cf = gymtorch.wrap_tensor(gym.acquire_net_contact_force_tensor(sim))
        gym.refresh_net_contact_force_tensor(sim)
        cf = gymtorch.wrap_tensor(gym.acquire_net_contact_force_tensor(sim))
        norms = cf.norm(dim=-1).cpu().numpy()
        for i, n in enumerate(body_names):
            if norms[i] > 1e-3:
                contacts[n] = float(norms[i])
    except Exception as e:
        print("[警告] 读接触力失败: %s" % e)

    print("\n--- 终态 ---")
    print("base_z = %.4f m   roll = %.2f deg   pitch = %.2f deg"
          % (base_z, roll * DEG, pitch * DEG))
    print("接触地面的刚体:", {k: round(v, 3) for k, v in sorted(contacts.items(), key=lambda kv: -kv[1])} or "（读不到）")

    print("\n最终各刚体位置:")
    print("%-12s %9s %9s %9s" % ("body", "x", "y", "z"))
    for i, n in enumerate(body_names):
        p = bs["pose"]["p"][i]
        print("%-12s %9.4f %9.4f %9.4f" % (n, p["x"], p["y"], p["z"]))

    touching = set(contacts.keys())
    feet = {n for n in body_names if n.endswith("_calf")}
    bad = touching - feet
    print("\n--- 判读 ---")
    if base_z > 0.08 and abs(roll * DEG) < 15 and abs(pitch * DEG) < 15:
        print("[OK] 站得住且姿态水平")
        print("     实测机身高度 = %.4f m  ->  rewards.base_height_target 建议填 %.2f"
              % (base_z, round(base_z, 2)))
    else:
        print("[!!] 撑不住 / 姿态歪")
    if contacts:
        if bad:
            print("[!!] 除足端外还有刚体触地: %s （会顶起机器人，需要收缩这些碰撞体）" % sorted(bad))
        else:
            print("[OK] 只有足端着地")

    gym.destroy_sim(sim)
    return base_z, roll * DEG, pitch * DEG


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--asset-root", default=ASSET_ROOT_DEFAULT)
    ap.add_argument("--urdf", default="robot.urdf")
    ap.add_argument("--stiffness", type=float, default=30.0)
    ap.add_argument("--damping", type=float, default=1.0)
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--self-collisions", type=int, default=1,
                    help="碰撞组：1=关闭自碰撞（与 legged_gym 一致），0=开启")
    ap.add_argument("--sweep", action="store_true",
                    help="扫一组 (stiffness, damping)")
    args = ap.parse_args()

    if args.sweep:
        print("===== PD 增益扫描 =====")
        for k, d in [(20, 0.5), (20, 1.0), (30, 1.0), (40, 1.5), (60, 2.0)]:
            z, r, p = run_once(args, k, d)
            print(">>> stiffness=%-4.1f damping=%-4.2f -> base_z=%.4f roll=%.2f pitch=%.2f"
                  % (k, d, z, r, p))
    else:
        run_once(args, args.stiffness, args.damping)


if __name__ == "__main__":
    main()
