"""生成 RL-CAT 训练专用 URDF（robot_sim.urdf）—— 交接文档 §9 步骤 6/7 的实现

做三件事（原 robot.urdf 一字不动）：
  1. 放宽 *_thigh 的 upper：30° -> 90°
  2. 碰撞体换基本几何体（STL 三角网格在 2048 环境下会拖垮 PhysX）：
       base_link  -> box      （机身本来就是方块状）
       *_hip      -> capsule  （细长连杆，沿轴拟合）
       *_thigh    -> capsule
       *_calf     -> sphere r=9mm（足端橡胶球）+ capsule（小腿本体）
  3. 输出前解析式自检：在 q=0 下逐个碰撞体算它在 base 系的最低点，
     凡是比足端球底面还低的都会让机器人"浮起来"，必须报警。

⚠️ 早期版本对细长连杆用了轴对齐 AABB box，太"胖"，实测会让机器人塌掉
   （机身高度从 0.130 掉到 0.068，四条大腿同时触地）。改用沿轴胶囊体。

用法:
    python scripts/05_make_sim_urdf.py
    python scripts/05_make_sim_urdf.py --thigh-upper-deg 60 --out /tmp/x.urdf
"""
import argparse
import math
import os
import re
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rlcat_paths as P  # noqa: E402  ⭐ 统一路径解析，不写死个人目录

SRC_DIR_DEFAULT = P.URDF_DIR
BALL_R = 0.009


# ---------------------------------------------------------------- STL / 数学
def stl_points(path):
    """返回 STL 顶点 (N,3)，单位米（新版 STL 是毫米）"""
    with open(path, "rb") as f:
        head = f.read(2048)
    if head[:5] == b"solid" and b"facet" in head:
        txt = open(path, errors="ignore").read()
        v = re.findall(r"vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)", txt)
        return np.array(v, dtype=float) * 0.001
    pts = []
    with open(path, "rb") as f:
        f.read(80)
        n = struct.unpack("<I", f.read(4))[0]
        for _ in range(n):
            d = f.read(50)
            if len(d) < 50:
                break
            v = struct.unpack("<12f", d[:48])
            pts += [(v[3], v[4], v[5]), (v[6], v[7], v[8]), (v[9], v[10], v[11])]
    return np.array(pts) * 0.001


def rpy_mat(rpy):
    """URDF 约定 R = Rz(yaw) * Ry(pitch) * Rx(roll)"""
    r, p, y = rpy
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def mat_to_rpy(R):
    """rpy_mat 的逆：从旋转矩阵取回 URDF 的 (roll, pitch, yaw)"""
    sp = -R[2, 0]
    sp = max(-1.0, min(1.0, sp))
    pitch = math.asin(sp)
    if abs(sp) < 0.9999:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:                       # 万向锁退化
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


def axis_to_rpy(d):
    """求把局部 +z 转到单位向量 d 的旋转，返回 URDF rpy"""
    d = np.asarray(d, dtype=float)
    d = d / (np.linalg.norm(d) + 1e-12)
    a = np.array([1.0, 0.0, 0.0]) if abs(d[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x = np.cross(a, d)
    x = x / (np.linalg.norm(x) + 1e-12)
    y = np.cross(d, x)
    R = np.column_stack([x, y, d])
    return mat_to_rpy(R)


def origin_of(el):
    o = el.find("origin")
    if o is None:
        return np.zeros(3), np.eye(3)
    t = np.array([float(x) for x in (o.get("xyz") or "0 0 0").split()])
    R = rpy_mat([float(x) for x in (o.get("rpy") or "0 0 0").split()])
    return t, R


def T_of(t, R):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


def fit_capsule(P, axis, radius_pct=85.0):
    """把一组顶点拟合成沿 axis 的胶囊：返回 (半径, 长度, 中心, rpy)

    半径取顶点到轴线距离的 85 分位（避开接头凸台的极值），
    长度取顶点沿轴投影的范围。
    """
    d = np.asarray(axis, dtype=float)
    d = d / (np.linalg.norm(d) + 1e-12)
    proj = P @ d
    perp = P - np.outer(proj, d)
    rad = np.linalg.norm(perp, axis=1)
    radius = float(np.percentile(rad, radius_pct))
    lo, hi = float(proj.min()), float(proj.max())
    length = max(hi - lo, 1e-3)
    center = d * (lo + hi) / 2.0
    return radius, length, center, axis_to_rpy(d)


# ---------------------------------------------------------------- 主流程
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=SRC_DIR_DEFAULT)
    ap.add_argument("--out", default=None)
    ap.add_argument("--thigh-upper-deg", type=float, default=90.0)
    ap.add_argument("--radius-pct", type=float, default=85.0,
                    help="胶囊半径取顶点到轴距离的百分位")
    ap.add_argument("--calf-capsule", action="store_true", default=True)
    ap.add_argument("--no-calf-capsule", dest="calf_capsule", action="store_false")
    ap.add_argument("--symmetrize", action="store_true", default=True)
    ap.add_argument("--no-symmetrize", dest="symmetrize", action="store_false",
                    help="不改对称性（默认会修掉 CAD 导出的 1.5mm 不对称）")
    args = ap.parse_args()

    src_dir = os.path.abspath(args.src)
    out_path = os.path.abspath(args.out or os.path.join(src_dir, "robot_sim.urdf"))
    src_urdf = os.path.join(src_dir, "robot.urdf")

    tree = ET.parse(src_urdf)
    root = tree.getroot()
    joints = {j.get("name"): j for j in root.findall("joint")}
    links = {l.get("name"): l for l in root.findall("link")}
    child_of = {}
    for j in root.findall("joint"):
        child_of[j.find("parent").get("link")] = j

    def chain_to(link_name):
        ch, cur = [], link_name
        while cur != "base_link":
            j = [jj for jj in root.findall("joint") if jj.find("child").get("link") == cur]
            if not j:
                raise RuntimeError("链路断裂: " + cur)
            ch.append(j[0].get("name"))
            cur = j[0].find("parent").get("link")
        return list(reversed(ch))

    def link_transform(link_name):
        T = np.eye(4)
        for jn in chain_to(link_name):
            t, R = origin_of(joints[jn])
            T = T @ T_of(t, R)
        return T

    # ---------- 0. 修掉 CAD 导出的左右不对称 ----------
    # 实测原始 robot.urdf：lf_hip 的 origin x = 0.0395，而其他三个髋是 ±0.041；
    # lf_thigh 的 origin z = 0.0586，而其他三个大腿是 ±0.0571。
    # 这是导出遗留的 1.5 mm 误差，会给策略一个系统性的左右偏置 ——
    # 实测训练出来的策略明显偏用左后腿（触地占空比 0.73 vs 右前 0.29，右后几乎不落地）。
    # 用右侧腿的值镜像出左侧腿（y 取反，rpy 不动：轴符号本身已经编码了镜像）。
    if args.symmetrize:
        for lj, rj in [("lf_hip", "rf_hip"), ("lb_hip", "rb_hip"),
                       ("lf_thigh", "rf_thigh"), ("lb_thigh", "rb_thigh")]:
            if lj in joints and rj in joints:
                ro = joints[rj].find("origin")
                lo = joints[lj].find("origin")
                if ro is None or lo is None:
                    continue
                xyz = [float(v) for v in ro.get("xyz").split()]
                old = lo.get("xyz")
                xyz[1] = -xyz[1]
                lo.set("xyz", "%.9g %.9g %.9g" % tuple(xyz))
                if old != lo.get("xyz"):
                    print("  对称化 %-10s xyz %s -> %s" % (lj, old, lo.get("xyz")))

    # ---------- 1. 放宽 thigh 限位 ----------
    new_upper = math.radians(args.thigh_upper_deg)
    n_thigh = 0
    for jn, j in joints.items():
        if jn.endswith("_thigh"):
            lim = j.find("limit")
            old = float(lim.get("upper"))
            lim.set("upper", "%.7f" % new_upper)
            n_thigh += 1
            print("  限位  %-10s upper %.6f (%.1f°) -> %.6f (%.1f°)"
                  % (jn, old, math.degrees(old), new_upper, args.thigh_upper_deg))

    # ---------- 2. 碰撞体拟合 ----------
    colliders = {}

    def mesh_points_of(link_el, tag="collision"):
        el = link_el.find(tag)
        if el is None:
            return None
        m = el.find("geometry/mesh")
        if m is None:
            return None
        t, R = origin_of(el)
        fn = m.get("filename")
        path = fn if os.path.isabs(fn) else os.path.join(src_dir, fn)
        V = stl_points(path)
        return (R @ V.T).T + t              # 本 link 坐标系

    for name, link in links.items():
        if link.find("collision") is None:
            continue
        P = mesh_points_of(link)
        if P is None:
            continue

        if name.endswith("_calf"):
            # 足端球：球心 = mesh 在【站立姿态 base 系】最低点 + 9mm
            # ⚠️ 必须在 base 系里找最低点：连杆是转过的，link 系里 argmin 会选错顶点
            T = link_transform(name)
            Vw = (T[:3, :3] @ P.T).T + T[:3, 3]
            lo_world = Vw[int(np.argmin(Vw[:, 2]))]
            lo = (np.linalg.inv(T) @ np.append(lo_world, 1.0))[:3]
            up_local = T[:3, :3].T @ np.array([0.0, 0.0, 1.0])
            colliders[name] = [dict(kind="sphere", size=BALL_R,
                                    center=lo + up_local * BALL_R, rpy=(0.0, 0.0, 0.0))]
            if args.calf_capsule:
                # 小腿本体也用胶囊（否则只有脚底有碰撞，小腿会穿地）
                jc = child_of.get(name)
                if jc is not None:
                    # calf 没有子关节，用"膝->足端"方向当轴
                    d = lo_world - T[:3, 3]
                    r, L, c, rpy = fit_capsule(P, d, args.radius_pct)
                    colliders[name].append(dict(kind="capsule", size=(r, L),
                                                center=c, rpy=rpy))

        elif name == "base_link":
            mn, mx = P.min(axis=0), P.max(axis=0)
            colliders[name] = [dict(kind="box", size=mx - mn,
                                    center=(mn + mx) / 2.0, rpy=(0.0, 0.0, 0.0))]

        elif name.endswith("_hip") or name.endswith("_thigh"):
            jc = child_of.get(name)
            if jc is None:
                continue
            t, _ = origin_of(jc)
            r, L, c, rpy = fit_capsule(P, t, args.radius_pct)
            colliders[name] = [dict(kind="capsule", size=(r, L), center=c, rpy=rpy)]
        else:
            mn, mx = P.min(axis=0), P.max(axis=0)
            colliders[name] = [dict(kind="box", size=mx - mn,
                                    center=(mn + mx) / 2.0, rpy=(0.0, 0.0, 0.0))]

    # 写回 XML
    for name, link in links.items():
        if name not in colliders:
            continue
        for old in link.findall("collision"):
            link.remove(old)
        for spec in colliders[name]:
            col = ET.SubElement(link, "collision")
            o = ET.SubElement(col, "origin")
            o.set("xyz", "%.6f %.6f %.6f" % tuple(spec["center"]))
            o.set("rpy", "%.6f %.6f %.6f" % tuple(spec["rpy"]))
            geo = ET.SubElement(col, "geometry")
            k = spec["kind"]
            if k == "box":
                b = ET.SubElement(geo, "box")
                b.set("size", "%.6f %.6f %.6f" % tuple(spec["size"]))
            elif k == "sphere":
                s = ET.SubElement(geo, "sphere")
                s.set("radius", "%.6f" % spec["size"])
            else:
                # 用 cylinder，配合 Isaac Gym 的 replace_cylinder_with_capsule 变成胶囊
                cy = ET.SubElement(geo, "cylinder")
                cy.set("radius", "%.6f" % spec["size"][0])
                cy.set("length", "%.6f" % spec["size"][1])

    # ---------- 3. 解析式自检 ----------
    print("\n=== 站立姿态(q=0)下各碰撞体在 base 系的最低点 ===")
    print("%-12s %-9s %12s   %s" % ("link", "type", "最低 z (m)", "判定"))
    rows = []
    for name, specs in colliders.items():
        T = link_transform(name)
        R, t = T[:3, :3], T[:3, 3]
        for spec in specs:
            k, size, center, rpy = spec["kind"], spec["size"], spec["center"], spec["rpy"]
            if k == "box":
                c8 = np.array([[sx * size[0] / 2, sy * size[1] / 2, sz * size[2] / 2]
                               for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
                zmin = float(((R @ (c8 + center).T).T + t)[:, 2].min())
            elif k == "sphere":
                zmin = float((R @ center + t)[2]) - size
            else:
                r, L = size
                ends = np.array([center + np.array([0, 0, L / 2]),
                                 center - np.array([0, 0, L / 2])])
                Rc = rpy_mat(rpy)
                zw = ((R @ Rc @ ends.T).T + t)[:, 2]
                zmin = float(zw.min()) - r          # 保守：再减掉半径
            rows.append((name, k, zmin, size))

    foot_bottom = min((r[2] for r in rows if r[1] == "sphere"), default=None)
    limit = foot_bottom if foot_bottom is not None else -1e9
    bad = []
    for name, k, zmin, size in sorted(rows, key=lambda r: r[2]):
        ok = zmin >= limit - 0.003
        if not ok:
            bad.append("%s(%s)" % (name, k))
        extra = ("size=(%.1f,%.1f,%.1f)mm" % tuple(np.array(size) * 1000)
                 if k == "box" else
                 ("r=%.1fmm L=%.1fmm" % (size[0] * 1000, size[1] * 1000) if k == "capsule"
                  else "r=%.1fmm" % (size * 1000)))
        print("%-12s %-9s %12.4f   %-28s %s" % (name, k, zmin, extra, "OK" if ok else "!! 低于脚底"))
    print("\n脚底最低点 = %.4f m  ->  几何预测机身站立高度 = %.4f m" % (foot_bottom, -foot_bottom))
    print("[OK] 没有碰撞体低于足端" if not bad else "[!!] 这些碰撞体会先触地: %s" % bad)

    # ---------- 输出 ----------
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    tree.write(out_path, encoding="utf-8", xml_declaration=True)
    print("\n已写出: %s" % out_path)
    print("meshes 相对路径基准: %s （必须与 meshes/ 同级）" % src_dir)
    print("thigh 限位放宽 %d 个；碰撞体替换 %d 个 link" % (n_thigh, len(colliders)))


if __name__ == "__main__":
    main()
