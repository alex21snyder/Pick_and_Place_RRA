"""Check the URDF against the DH model. Needs only numpy (no ROS).

Run:  python3 check_fk.py

Check 1: chain the joint origins from urdf/arm.urdf (as ROS would) and compare the tool pose with dh.fk()
         for random joint angles. They must agree to numerical precision.
Check 2: run dh.fk() at the joint angles the arm printed for Motions 1-3 and compare the end effector
         position with the Xd targets in main (tolerance: the arm's own 1 mm epsilon, plus rounding of
         the printed angles).
"""

import os
import xml.etree.ElementTree as ET
import numpy as np
from dh import fk, DH, DH_CAD
from make_urdf import rpy_to_matrix, MM_TO_M

HERE = os.path.dirname(os.path.realpath(__file__))
URDF = os.path.join(HERE, "..", "urdf", "arm.urdf")


def rz(q):
    c, s = np.cos(q), np.sin(q)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])


def parse_origin(joint):
    o = joint.find("origin")
    xyz = [float(v) for v in o.get("xyz").split()]
    rpy = [float(v) for v in o.get("rpy").split()]
    m = rpy_to_matrix(*rpy)
    m[:3, 3] = xyz
    return m


def urdf_tool_pose(q_deg):
    """Tool pose in meters, chaining the joints exactly as the URDF defines them."""
    joints = {j.get("name"): j for j in ET.parse(URDF).getroot().findall("joint")}
    f = np.eye(4)
    for i in range(1, 7):
        f = f @ parse_origin(joints[f"joint{i}"]) @ rz(np.radians(q_deg[i - 1]))
    return f @ parse_origin(joints["tool_joint"])


def main():
    ok = True

    # Check 1: URDF chain vs DH
    # The model is built from DH_CAD by default (CAD shaft positions); with make_urdf.py --algorithm-dh it uses DH.
    # Check against whichever table the URDF matches best, and say which one it was.
    def worst_for(table):
        rng = np.random.default_rng(0)
        wp = wr = 0.0
        for _ in range(200):
            q = rng.uniform(-180, 180, 6)
            a = fk(q, table)
            b = urdf_tool_pose(q)
            wp = max(wp, np.abs(a[:3, 3] * MM_TO_M - b[:3, 3]).max())
            wr = max(wr, np.abs(a[:3, :3] - b[:3, :3]).max())
        return wp, wr
    results = {"DH_CAD (3D model table)": worst_for(DH_CAD), "DH (original table)": worst_for(DH)}
    which = min(results, key=lambda k: results[k][0])
    worst_pos, worst_rot = results[which]
    print(f"URDF was built from: {which}")
    # The URDF text is written with 9 decimals, so ~1e-9 differences are expected; 1e-7 is far below anything visible.
    check1 = worst_pos < 1e-7 and worst_rot < 1e-7
    ok &= check1
    print(f"Check 1  URDF chain vs table FK (200 random poses): worst position error {worst_pos:.2e} m, "
          f"worst rotation error {worst_rot:.2e}  ->  {'PASS' if check1 else 'FAIL'}")

    # Check 2: FK at the arm's printed joint angles vs the targets in main
    motions = [
        ("Motion 1", [-29.63, -13.77, 52.52, 45.81, -38.32, -39.98], [188.0, -104.0, 312.0],
         [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]),
        ("Motion 2", [10.09, 7.42, 53.52, 65.99, 2.06, -70.90], [104.0, 0.0, 394.0],
         [[0.7071, 0, 0.7071], [0, 1, 0], [-0.7071, 0, 0.7071]]),
        ("Motion 3", [37.38, -13.30, 56.32, 106.37, 40.31, -114.07], [190.0, 104.0, 305.0],
         [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]),
    ]
    for name, q, xd, rd in motions:
        f = fk(np.array(q))
        pos_err = np.linalg.norm(f[:3, 3] - np.array(xd))
        r_err = f[:3, :3] @ np.array(rd).T  # relative rotation; angle from its trace
        theta = np.arccos(np.clip((np.trace(r_err) - 1) / 2, -1, 1))
        # rra stops once thetaE < epsilon_o = 0.1 rad, so the arm ends right at the edge of that tolerance.
        # 0.11 rad = the arm's own 0.1 rad tolerance plus rounding of the printed angles.
        good = pos_err < 1.5 and theta < 0.11
        ok &= good
        print(f"Check 2  {name}: FK position {np.round(f[:3, 3], 2)} mm vs target {xd}  "
              f"(error {pos_err:.2f} mm, orientation error {np.degrees(theta):.2f} deg)  ->  {'PASS' if good else 'FAIL'}")

    print("\nALL CHECKS PASSED" if ok else "\nSOMETHING FAILED")
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
