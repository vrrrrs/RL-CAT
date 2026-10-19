"""定位 Isaac Gym 仿真步进时 CUDA illegal memory access 的最小复现 / 二分测试。

逐级加复杂度，看哪一步开始崩：
  T1 只建 sim，空步进
  T2 + env/actor（焊死底座），不控制
  T3 + DOF 位置驱动
  T4 同 T3 但关掉 GPU pipeline
  T5 同 T3 但物理也用 CPU
"""
import os
import sys

import numpy as np
from isaacgym import gymapi

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rlcat_paths as P  # noqa: E402  ⭐ 统一路径解析，不写死个人目录

ASSET_ROOT = P.URDF_DIR


def mk_sim(gym, gpu_physx=True, gpu_pipe=True, substeps=2):
    sp = gymapi.SimParams()
    sp.dt = 1.0 / 200.0
    sp.substeps = substeps
    sp.up_axis = gymapi.UP_AXIS_Z
    sp.gravity = gymapi.Vec3(0.0, 0.0, -9.81)
    sp.physx.solver_type = 1
    sp.physx.num_position_iterations = 4
    sp.physx.num_velocity_iterations = 1
    sp.physx.use_gpu = gpu_physx
    sp.use_gpu_pipeline = gpu_pipe
    return gym.create_sim(0, 0, gymapi.SIM_PHYSX, sp)


def load(gym, sim, fix_base):
    o = gymapi.AssetOptions()
    o.fix_base_link = fix_base
    o.default_dof_drive_mode = int(gymapi.DOF_MODE_POS)
    o.use_mesh_materials = True
    o.vhacd_enabled = False
    o.replace_cylinder_with_capsule = True
    o.thickness = 0.001
    return gym.load_asset(sim, ASSET_ROOT, "robot.urdf", o)


def run(name, gpu_physx, gpu_pipe, with_actor, with_drive, steps=100):
    print("\n---- %s ----" % name, flush=True)
    gym = gymapi.acquire_gym()
    try:
        sim = mk_sim(gym, gpu_physx, gpu_pipe)
        if sim is None:
            print("  [FAIL] create_sim 返回 None")
            return
        if with_actor:
            asset = load(gym, sim, fix_base=True)
            n_dof = gym.get_asset_dof_count(asset)
            env = gym.create_env(sim, gymapi.Vec3(-1, -1, 0), gymapi.Vec3(1, 1, 1), 1)
            actor = gym.create_actor(env, asset, gymapi.Transform(), "cat", 0, 0)
            if with_drive:
                p = gym.get_asset_dof_properties(asset)
                p["driveMode"][:] = int(gymapi.DOF_MODE_POS)
                p["stiffness"][:] = 200.0
                p["damping"][:] = 10.0
                gym.set_actor_dof_properties(env, actor, p)
                gym.set_actor_dof_position_targets(
                    env, actor, np.zeros(n_dof, dtype=np.float32))
        gym.prepare_sim(sim)
        for i in range(steps):
            gym.simulate(sim)
            gym.fetch_results(sim, True)
        z = None
        if with_actor:
            bs = gym.get_actor_rigid_body_states(env, actor, gymapi.STATE_ALL)
            z = float(bs["pose"]["p"][0]["z"])
        print("  [OK] 步进 %d 次无报错, base_z=%s" % (steps, z), flush=True)
    except Exception as e:
        print("  [EXC]", type(e).__name__, e, flush=True)
    finally:
        try:
            gym.destroy_sim(sim)
        except Exception:
            pass


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "all"
    tests = {
        "t1": ("T1 空 sim + GPU pipeline", True, True, False, False),
        "t2": ("T2 actor(焊死) 不控制 + GPU", True, True, True, False),
        "t3": ("T3 actor + DOF 位置驱动 + GPU", True, True, True, True),
        "t4": ("T4 actor + 驱动, use_gpu_pipeline=False", True, False, True, True),
        "t5": ("T5 actor + 驱动, 物理也用 CPU", False, False, True, True),
    }
    keys = list(tests) if which == "all" else [which]
    for k in keys:
        n, gp, gpipe, wa, wd = tests[k]
        run(n, gp, gpipe, wa, wd)
