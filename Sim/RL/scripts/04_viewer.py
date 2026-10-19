"""RL-CAT 可视化自检：地面 + q=0 站立 + 键盘/自动演示（用人眼确认姿态与轴向）

用法（需要桌面环境，不能用 SSH 纯终端）:
    python scripts/04_viewer.py

操作：
    默认自动播放 4 个阶段：静止 q=0 -> 小幅摆腿 -> 抬腿 -> 复位
    按 ESC 退出，按空格暂停
"""
import os
import sys

import numpy as np
from isaacgym import gymapi, gymutil

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rlcat_paths as P  # noqa: E402  ⭐ 统一路径解析，不写死个人目录

ASSET_ROOT = P.URDF_DIR


def main():
    gym = gymapi.acquire_gym()

    sp = gymapi.SimParams()
    sp.dt = 1.0 / 200.0
    sp.substeps = 2
    sp.up_axis = gymapi.UP_AXIS_Z
    sp.gravity = gymapi.Vec3(0, 0, -9.81)
    sp.physx.solver_type = 1
    sp.physx.num_position_iterations = 4
    sp.physx.num_velocity_iterations = 1
    sp.physx.use_gpu = True
    sp.use_gpu_pipeline = False
    sim = gym.create_sim(0, 0, gymapi.SIM_PHYSX, sp)

    plane = gymapi.PlaneParams()
    plane.normal = gymapi.Vec3(0, 0, 1)
    gym.add_ground(sim, plane)

    # 相机
    cam_props = gymapi.CameraProperties()
    cam_props.width = 1280
    cam_props.height = 720
    viewer = gym.create_viewer(sim, cam_props)
    if viewer is None:
        print("[FAIL] 无法创建 viewer —— 检查 DISPLAY 环境变量 / 是否在桌面会话里")
        return
    gym.viewer_camera_look_at(viewer, None,
                              gymapi.Vec3(0.6, -0.6, 0.35),
                              gymapi.Vec3(0.0, 0.0, 0.12))

    o = gymapi.AssetOptions()
    o.fix_base_link = False
    o.default_dof_drive_mode = int(gymapi.DOF_MODE_POS)
    o.use_mesh_materials = True
    o.vhacd_enabled = False
    o.replace_cylinder_with_capsule = True
    o.thickness = 0.001
    asset = gym.load_asset(sim, ASSET_ROOT, "robot.urdf", o)

    env = gym.create_env(sim, gymapi.Vec3(-1, -1, 0), gymapi.Vec3(1, 1, 1), 1)
    pose = gymapi.Transform()
    pose.p = gymapi.Vec3(0, 0, 0.20)
    actor = gym.create_actor(env, asset, pose, "cat", 0, 0)

    props = gym.get_asset_dof_properties(asset)
    props["driveMode"][:] = int(gymapi.DOF_MODE_POS)
    props["stiffness"][:] = 30.0
    props["damping"][:] = 1.0
    gym.set_actor_dof_properties(env, actor, props)

    dof_names = list(gym.get_asset_dof_names(asset))
    n_dof = len(dof_names)
    print("DOF 顺序:", dof_names)

    # 逻辑索引 A -> Isaac 索引 C（实测）
    A_NAMES = ["lf_hip", "lb_hip", "rf_hip", "rb_hip",
               "lf_thigh", "lb_thigh", "rf_thigh", "rb_thigh",
               "lf_calf", "lb_calf", "rf_calf", "rb_calf"]
    A2C = [dof_names.index(n) for n in A_NAMES]
    print("A2C =", A2C)

    q0 = np.zeros(n_dof, dtype=np.float32)
    t = 0
    while not gym.query_viewer_has_closed(viewer):
        # 阶段化演示：0-2s 站立，2-4s 髋外摆，4-6s 大腿前后摆，6-8s 膝盖屈伸
        sec = t / 200.0
        target = q0.copy()
        amp = 0.25
        if 2.0 <= sec < 4.0:
            for a in range(4):                       # 外摆
                target[A2C[a]] = amp * np.sin(2 * np.pi * (sec - 2.0) / 2.0)
        elif 4.0 <= sec < 6.0:
            for a in range(4, 8):                    # 大腿前后摆
                target[A2C[a]] = amp * np.sin(2 * np.pi * (sec - 4.0) / 2.0)
        elif 6.0 <= sec < 8.0:
            for a in range(8, 12):                   # 膝屈伸
                target[A2C[a]] = amp * np.sin(2 * np.pi * (sec - 6.0) / 2.0)

        gym.set_actor_dof_position_targets(env, actor, target)
        gym.simulate(sim)
        gym.fetch_results(sim, True)
        gym.step_graphics(sim)
        gym.draw_viewer(viewer, sim, True)
        gym.sync_frame_time(sim)
        t += 1

    gym.destroy_viewer(viewer)
    gym.destroy_sim(sim)


if __name__ == "__main__":
    main()
