"""Generate urdf/arm.urdf from the DH table in dh.py.

Run:  python3 make_urdf.py            (writes ../urdf/arm.urdf)

DH (classic) says each joint i is:   A_i = Rz(q_i + offset_i) * F_i,   F_i = Tz(d_i) Tx(r_i) Rx(alpha_i)
URDF says each joint is:             child_frame = parent_frame * origin * Rz(q)

So the chain  Rz(q1+o1) F1 Rz(q2+o2) F2 ... Rz(q6+o6) F6  becomes:
    joint_i origin = F_(i-1) * Rz(o_i)      (F_0 = identity), axis z, value q_i
    final fixed joint "tool" origin = F_6   -> the tool frame is exactly frame 6 of the Arduino code.
Lengths are converted from mm to meters, since ROS uses meters and radians.
"""

import os
import sys
import numpy as np
from dh import DH, DH_CAD

# The model uses the CAD-measured table unless you pass --algorithm-dh (then it uses the original DH table).
USE_DH = DH if "--algorithm-dh" in sys.argv else DH_CAD

MM_TO_M = 0.001

# CAD meshes: meshes/link<i>.stl, exported from SolidWorks in mm, in the base frame, arm at home (all q = 0).
# A link with a mesh file gets the mesh; links without one keep the stick drawing.
# MESH_DIR is replaced with the real folder path by display.launch.py when it starts.
MESH_SCALE = 0.001  # mm -> m
MESH_COLORS = {     # rgba, one per link. Change these to taste.
    0: "0.25 0.25 0.28 1",
}
DEFAULT_MESH_COLOR = "0.75 0.75 0.78 1"


def rot_x(deg):
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1.0]])


def rot_z(deg):
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])


def trans(x, y, z):
    t = np.eye(4)
    t[:3, 3] = [x, y, z]
    return t


def fixed_part(d_mm, r_mm, alpha_deg):
    """F = Tz(d) Tx(r) Rx(alpha), translation in meters."""
    return trans(0, 0, d_mm * MM_TO_M) @ trans(r_mm * MM_TO_M, 0, 0) @ rot_x(alpha_deg)


def matrix_to_rpy(m):
    """URDF rpy: R = Rz(yaw) Ry(pitch) Rx(roll). Returns (roll, pitch, yaw) in radians."""
    r = m[:3, :3]
    sy = -r[2, 0]
    pitch = np.arcsin(np.clip(sy, -1.0, 1.0))
    if abs(abs(sy) - 1.0) > 1e-9:
        roll = np.arctan2(r[2, 1], r[2, 2])
        yaw = np.arctan2(r[1, 0], r[0, 0])
    elif sy > 0:  # pitch = +90 deg (gimbal): choose yaw = 0
        yaw = 0.0
        roll = np.arctan2(r[0, 1], r[0, 2])
    else:         # pitch = -90 deg (gimbal): choose yaw = 0
        yaw = 0.0
        roll = np.arctan2(-r[0, 1], -r[0, 2])
    return roll, pitch, yaw


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    m = np.eye(4)
    m[:3, :3] = rz @ ry @ rx
    return m


def clean(v):
    """Round tiny numerical noise to exactly 0 and format compactly."""
    v = 0.0 if abs(v) < 1e-10 else float(v)
    return f"{v:.9f}".rstrip("0").rstrip(".") if v != 0.0 else "0"


def origin_xml(m):
    roll, pitch, yaw = matrix_to_rpy(m)
    xyz = " ".join(clean(v) for v in m[:3, 3])
    rpy = " ".join(clean(v) for v in (roll, pitch, yaw))
    return f'<origin xyz="{xyz}" rpy="{rpy}"/>'


def bone_visual(p, material):
    """A thin cylinder from the link origin (0,0,0) to point p (meters), expressed in the link frame."""
    length = float(np.linalg.norm(p))
    if length < 1e-6:
        return ""
    z = np.array([0.0, 0.0, 1.0])
    u = p / length
    axis = np.cross(z, u)
    s = np.linalg.norm(axis)
    c = float(np.dot(z, u))
    if s < 1e-9:
        rot = np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    else:
        k = axis / s
        kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        rot = np.eye(3) + s * kx + (1 - c) * (kx @ kx)
    m = np.eye(4)
    m[:3, :3] = rot
    m[:3, 3] = p / 2.0
    return (
        f'    <visual>\n      {origin_xml(m)}\n'
        f'      <geometry><cylinder radius="0.006" length="{clean(length)}"/></geometry>\n'
        f'      <material name="{material}"/>\n    </visual>\n'
    )


def mesh_visual(i, zero_pose):
    """Visual for link i from meshes/link<i>.stl. The STL is in the base frame at q = 0, so inside the
    link's own frame it sits at inverse(zero_pose), where zero_pose = pose of this link's frame at q = 0."""
    return (
        f'    <visual>\n      {origin_xml(np.linalg.inv(zero_pose))}\n'
        f'      <geometry><mesh filename="file://MESH_DIR/link{i}.stl" scale="{MESH_SCALE} {MESH_SCALE} {MESH_SCALE}"/></geometry>\n'
        f'      <material name="mesh{i}"/>\n    </visual>'
    )


def build():
    here = os.path.dirname(os.path.realpath(__file__))
    mesh_dir = os.path.normpath(os.path.join(here, "..", "meshes"))
    has_mesh = [os.path.exists(os.path.join(mesh_dir, f"link{i}.stl")) for i in range(7)]
    # Origin of joint i (i = 1..6) and of the fixed tool joint
    origins = []
    f_prev = np.eye(4)
    for i, (offset, d, r, alpha) in enumerate(USE_DH):
        origins.append(f_prev @ rot_z(offset))
        f_prev = fixed_part(d, r, alpha)
    tool_origin = f_prev  # F_6

    # Where each link's next joint sits, in that link's own frame (for the bone drawing)
    next_pos = [origins[i + 1][:3, 3] for i in range(5)] + [tool_origin[:3, 3]]
    next_pos = [np.zeros(3)] + next_pos  # link index 0 = base_link: first joint is at its origin

    out = []
    out.append('<?xml version="1.0"?>')
    out.append('<!-- Generated by tools/make_urdf.py from tools/dh.py. Do not edit by hand. -->')
    out.append('<robot name="arm">')
    out.append('  <material name="blue"><color rgba="0.2 0.4 0.9 1"/></material>')
    out.append('  <material name="orange"><color rgba="0.95 0.55 0.1 1"/></material>')
    for i in range(7):
        if has_mesh[i]:
            rgba = MESH_COLORS.get(i, DEFAULT_MESH_COLOR)
            out.append(f'  <material name="mesh{i}"><color rgba="{rgba}"/></material>')
    # Pose of each link frame at q = 0 (link i frame = origin_1 * ... * origin_i with all q = 0)
    zero_poses = [np.eye(4)]
    for i in range(6):
        zero_poses.append(zero_poses[-1] @ origins[i])
    out.append('')

    out.append('  <link name="base_link">')
    if has_mesh[0]:
        out.append(mesh_visual(0, zero_poses[0]))
    else:
        out.append('    <visual><geometry><cylinder radius="0.03" length="0.01"/></geometry><material name="blue"/></visual>')
    out.append('  </link>')

    for i in range(1, 7):
        out.append(f'  <link name="link{i}">')
        if has_mesh[i]:
            out.append(mesh_visual(i, zero_poses[i]))
        elif "--no-sticks" in sys.argv:
            pass  # hide the placeholder stick and ball for links that have no CAD mesh yet
        else:
            out.append('    <visual><geometry><sphere radius="0.012"/></geometry><material name="orange"/></visual>')
            out.append(bone_visual(next_pos[i], "blue").rstrip("\n"))
        out.append('  </link>')
    out.append('  <material name="tcp_red"><color rgba="1 0.1 0.1 1"/></material>')
    out.append('  <link name="tool"></link>')
    out.append('')

    for i in range(1, 7):
        parent = "base_link" if i == 1 else f"link{i - 1}"
        out.append(f'  <joint name="joint{i}" type="revolute">')
        out.append(f'    <parent link="{parent}"/>')
        out.append(f'    <child link="link{i}"/>')
        out.append(f'    {origin_xml(origins[i - 1])}')
        out.append('    <axis xyz="0 0 1"/>')
        # PLACEHOLDER limits: +/-180 deg. Replace with the arm's real joint limits.
        out.append('    <limit lower="-3.14159" upper="3.14159" effort="1.0" velocity="1.0"/>')
        out.append('  </joint>')
    out.append('  <joint name="tool_joint" type="fixed">')
    out.append('    <parent link="link6"/>')
    out.append('    <child link="tool"/>')
    out.append(f'    {origin_xml(tool_origin)}')
    out.append('  </joint>')
    out.append('</robot>')
    return "\n".join(line for line in out if line.strip() != "") + "\n"


if __name__ == "__main__":
    here = os.path.dirname(os.path.realpath(__file__))
    path = os.path.join(here, "..", "urdf", "arm.urdf")
    with open(path, "w") as f:
        f.write(build())
    print("wrote", os.path.normpath(path))
