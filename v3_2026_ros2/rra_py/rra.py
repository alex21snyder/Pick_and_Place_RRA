"""Resolved Rates Algorithm (RRA) in numpy. Line-for-line port of rra.ino / jacobian_calc.ino / kin_function.ino (v2_2026).

Units match the Arduino code: q in degrees, lengths in mm, Xd in mm, Rd a 3x3 rotation matrix.
No ROS in this file, so it can be tested on its own:  python3 rra.py
"""

import math
import sys
import os
from dataclasses import dataclass
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "tools"))
from dh import DH, dh_transform  # same DH table as kin_function.ino

DAMPING_RHO = 1.0


@dataclass
class RRAParams:
    delta_time: float = 0.028
    vmin: float = 7.0
    vmax: float = 24.0
    wmin: float = 0.30
    wmax: float = 0.55
    epsilon_p: float = 1.0
    epsilon_o: float = 0.1
    lam: float = 5.0          # "lambda" in the C++ (reserved word in Python)
    max_iterations: int = 1500


def kin(q):
    """q (6,) degrees -> list of frames f1..f6 (4x4 each)."""
    frames = []
    f = np.eye(4)
    for qi, (offset, d, r, alpha) in zip(q, DH):
        f = f @ dh_transform(qi + offset, d, r, alpha)
        frames.append(f)
    return frames


def jacobian(frames):
    """6x6 Jacobian from frames 1-6. Rows 0-2 linear, rows 3-5 angular."""
    z = [np.array([0.0, 0.0, 1.0])] + [f[:3, 2] for f in frames[:5]]
    p = [np.zeros(3)] + [f[:3, 3] for f in frames[:5]]
    p6 = frames[5][:3, 3]
    J = np.zeros((6, 6))
    for i in range(6):
        J[:3, i] = np.cross(z[i], p6 - p[i])
        J[3:, i] = z[i]
    return J


def rra_steps(q, Xd, Rd, params):
    """Generator: yields (q_deg, posE, thetaE, done) once per iteration. q is updated in a copy.

    Final yield has done=True if converged. If it gives up, the last yield has done=False
    and the generator returns (matching 'rra gave up' in the C++).
    """
    q = np.array(q, dtype=float).copy()
    Xd = np.asarray(Xd, dtype=float)
    Rd = np.asarray(Rd, dtype=float)
    P = params

    for _ in range(P.max_iterations):
        frames = kin(q)
        J = jacobian(frames)
        Xc = frames[5][:3, 3]
        Rc = frames[5][:3, :3]

        posE = float(np.linalg.norm(Xd - Xc))
        RotE = Rd @ Rc.T
        thetaE = math.acos(max(-1.0, min(1.0, (np.trace(RotE) - 1.0) / 2.0)))

        if posE < P.epsilon_p and thetaE < P.epsilon_o:
            yield q.copy(), posE, thetaE, True
            return

        # linear velocity command
        nhat = (Xd - Xc) / np.linalg.norm(Xd - Xc)
        v_ramp = ((P.vmax - P.vmin) * (posE - P.epsilon_p)) / (P.epsilon_p * (P.lam - 1))
        vhat = P.vmax if posE / P.epsilon_p > P.lam else P.vmin + v_ramp
        Pddot = nhat * vhat

        # angular velocity command (same odd /epsilon_p term as the C++)
        w_ramp = ((P.wmax - P.wmin) * (thetaE - P.epsilon_o)) / (P.epsilon_o * (P.lam - 1))
        w = P.wmax if thetaE / P.epsilon_o / P.epsilon_p > P.lam else P.wmin + w_ramp

        s = math.sin(thetaE)
        if s > 1e-9:
            mhat = np.array([RotE[2, 1] - RotE[1, 2],
                             RotE[0, 2] - RotE[2, 0],
                             RotE[1, 0] - RotE[0, 1]]) / (2.0 * s)
        else:
            mhat = np.zeros(3)  # C++ would divide by zero here; no rotation needed anyway
        wd = mhat * w

        Xddot = np.concatenate([Pddot, wd])

        # damped pseudo inverse: J^T (J J^T + rho I)^-1
        Jpinv = J.T @ np.linalg.inv(J @ J.T + DAMPING_RHO * np.eye(6))
        qddot = Jpinv @ Xddot                      # rad/s
        q = q + np.degrees(qddot) * P.delta_time   # deg

        yield q.copy(), posE, thetaE, False

    yield q.copy(), posE, thetaE, False
    print(f"rra gave up: goal not reached in {P.max_iterations} iterations. posE = {posE:.3f}, thetaE = {thetaE:.3f}")


def home_steps(q, seconds=10.0, delta_time=0.028):
    """Motion 4: straight line in joint space back to home (all q = 0, the Perch position).

    Real arm: 2000 steps x 5 ms = 10 s. Same duration here, at the node's 28 ms tick.
    Yields (q_deg, 0.0, 0.0, done) like rra_steps so the node can treat both the same.
    """
    q = np.array(q, dtype=float)
    n = max(1, int(round(seconds / delta_time)))
    step = -q / n
    for i in range(n):
        q = q + step
        yield q.copy(), 0.0, 0.0, i == n - 1
    # land exactly on home, like the "confirm motors are exactly at home" lines in main_06.ino


def rra(q, Xd, Rd, params):
    """Run to the end. Returns (q_deg, converged, iterations)."""
    last = None
    n = 0
    for last in rra_steps(q, Xd, Rd, params):
        n += 1
    return last[0], last[3], n


# The three motions from main_06.ino
R1 = np.array([[0, 0, 1.0], [0, 1.0, 0], [-1.0, 0, 0]])
R2 = np.array([[0.7071, 0, 0.7071], [0, 1.0, 0], [-0.7071, 0, 0.7071]])
MOTIONS = [
    ("Motion 1", np.array([188.0, -104.0, 312.0]), R1, {}),
    ("Motion 2", np.array([104.0, 0.0, 394.0]), R2, {}),
    ("Motion 3", np.array([190.0, 104.0, 305.0]), R1, {"wmin": 0.75, "wmax": 1.375}),
]

if __name__ == "__main__":
    q = np.zeros(6)
    params = RRAParams()
    expected = [
        [-29.63, -13.77, 52.52, 45.81, -38.32, -39.98],
        [10.09, 7.42, 53.52, 65.99, 2.06, -70.90],
        [37.38, -13.30, 56.32, 106.37, 40.31, -114.07],
    ]
    ok = True
    for (name, Xd, Rd, over), exp in zip(MOTIONS, expected):
        for k, v in over.items():
            setattr(params, k, v)
        q, conv, n = rra(q, Xd, Rd, params)
        err = np.abs(q - np.array(exp)).max()
        print(f"{name}: converged={conv} iters={n}  q={np.round(q, 2).tolist()}  max diff vs arm = {err:.2f} deg")
        ok &= conv and err < 0.1
    print("MATCHES ARDUINO" if ok else "MISMATCH")
