"""DH model of the 6-DOF arm: the single source of truth for the geometry.

Same table and same transform as kin_function.ino / homogeneous_transformation.ino in v2_2026.
Angles are in degrees, lengths in mm. q are the "artificial" joint angles (state.q in the Arduino code).
"""

import numpy as np

# One row per joint: (theta_offset_deg, d_mm, r_mm, alpha_deg)
# theta used in the transform = q + theta_offset
DH = [
    (0.0,   35.5,  -4.0,  90.0),
    (270.0, -8.7, -269.0, 180.0),
    (270.0, -8.7,   0.0,  90.0),
    (0.0,  110.2,  -5.5,  90.0),
    (0.0,   25.5,   0.0, 270.0),
    (0.0,   55.0,  11.5,   0.0),
]

# DH table used ONLY for the 3D model (make_urdf.py), so the CAD parts rotate on their real shafts.
# Measured in SolidWorks (Arm_00 frame): servo 2 and servo 3 shaft axes are at x = +4 mm (z = 35.5 and 304.5),
# while the original table above puts them at x = -4. Only row 1's r changes: -4 -> +4.
# The algorithm (rra.py) keeps using DH, so the three motions still match the Arduino output exactly.
DH_CAD = [list(row) for row in DH]
DH_CAD[0][2] = 4.0
# Row 4: servo 5 shaft measured at x = +9.13 (5.13 in front of servo 4), table has -5.5. Model only.
DH_CAD[3][2] = 5.13
# Row 3: servo 4's vertical shaft measured at y = -6.39 (table puts it at y = 0): d3 -8.7 -> -15.09. Model only.
DH_CAD[2][1] = -8.7 - 6.39
# Row 5: servo 6 shaft measured at y = -34.59, i.e. 28.2 from servo 4's axis (table 25.5). Model only.
DH_CAD[4][1] = 28.2
DH_CAD = [tuple(row) for row in DH_CAD]


def dh_transform(theta_deg, d, r, alpha_deg):
    """One 4x4 homogeneous transform. Same matrix as dh_transform() in the Arduino code."""
    t = np.radians(theta_deg)
    a = np.radians(alpha_deg)
    return np.array([
        [np.cos(t), -np.cos(a) * np.sin(t),  np.sin(a) * np.sin(t), r * np.cos(t)],
        [np.sin(t),  np.cos(t) * np.cos(a), -np.sin(a) * np.cos(t), r * np.sin(t)],
        [0.0,        np.sin(a),              np.cos(a),              d],
        [0.0,        0.0,                    0.0,                    1.0],
    ])


def fk(q_deg, table=None):
    """Forward kinematics: joint angles (6, degrees) -> 4x4 pose of frame 6 (mm).
    table defaults to the original DH (the algorithm's); pass DH_CAD for the 3D model's table."""
    f = np.eye(4)
    for q, (offset, d, r, alpha) in zip(q_deg, DH if table is None else table):
        f = f @ dh_transform(q + offset, d, r, alpha)
    return f
