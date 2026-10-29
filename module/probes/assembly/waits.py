"""Where each arm waits: its first station's stand-off, pulled back and lifted out of the
belt's sweep IN JOINT SPACE, so the wait and the work are on the same wrist branch.

The parks were asked of the planner as a flange pose up and back over each arm's own base,
and the planner answered on whichever branch it liked. Every park and every first move
then turned the wrist half a turn: arm-a's move from its park into the inside shot was
1098 degrees of joint travel (j1 266, j6 302), arm-b's park between the pin and the flag
706, arm-c's home after the reject 386 with j5 alone 186 (taught program, 2026-10-14).

A wait derived from the station's own joints cannot do that. Only the shoulder, elbow and
first wrist joint move: j2 and j3 place the wrist in the arm's plane, and j4 takes up
their sum so the tool keeps its orientation (the three axes are parallel). The wrist is
pulled toward the shoulder and lifted until no link and no part of the tool is inside
the belt's sweep, and the smallest such move is the wait.

UR5e kinematics, checked against every taught pose's recorded flange to 0.2 mm.
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

D = (0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996)
A = (0.0, -0.425, -0.3922, 0.0, 0.0, 0.0)
ALPHA = (math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0)
# The planner's joint clamp (build_fragment): +-200, with these two wider.
LIMITS = [(-200.0, 200.0), (-275.0, 95.0), (-200.0, 200.0), (-200.0, 200.0),
          (-275.0, 275.0), (-205.0, 205.0)]
LINK_RADIUS_MM = 70.0         # UR5e links and wrist housings, with a little over
CLEAR_MM = 40.0               # beyond the sweep box, which already has its own margin
BACK_STEPS_MM = range(0, 501, 25)
LIFT_STEPS_MM = range(0, 451, 25)


def frames(q_deg: Sequence[float], base_mm: Sequence[float], yaw_deg: float) -> List[np.ndarray]:
    """World transforms (mm) of the base and each joint frame 1..6; the last is the flange."""
    y = math.radians(yaw_deg)
    t = np.array([[math.cos(y), -math.sin(y), 0, base_mm[0]],
                  [math.sin(y), math.cos(y), 0, base_mm[1]],
                  [0, 0, 1, base_mm[2]], [0, 0, 0, 1.0]])
    out = [t.copy()]
    for i in range(6):
        th = math.radians(q_deg[i])
        ct, st = math.cos(th), math.sin(th)
        ca, sa = math.cos(ALPHA[i]), math.sin(ALPHA[i])
        t = t @ np.array([[ct, -st * ca, st * sa, A[i] * 1000 * ct],
                          [st, ct * ca, -ct * sa, A[i] * 1000 * st],
                          [0, sa, ca, D[i] * 1000], [0, 0, 0, 1.0]])
        out.append(t.copy())
    return out


def flange(q_deg, base_mm, yaw_deg) -> Tuple[List[float], List[float]]:
    """Flange position (mm) and tool axis (the flange's +z) in the world."""
    t = frames(q_deg, base_mm, yaw_deg)[-1]
    return [float(v) for v in t[:3, 3]], [float(v) for v in t[:3, 2]]


def _plane(q):
    j2, j3 = math.radians(q[1]), math.radians(q[2])
    a2, a3 = A[1] * 1000, A[2] * 1000
    return (a2 * math.cos(j2) + a3 * math.cos(j2 + j3),
            a2 * math.sin(j2) + a3 * math.sin(j2 + j3))


def _near(angle: float, to: float) -> float:
    """`angle` (deg) plus whole turns, as close as possible to `to`."""
    return angle + 360.0 * round((to - angle) / 360.0)


def moved(q_deg: Sequence[float], back_mm: float, lift_mm: float) -> Optional[List[float]]:
    """The joints with the wrist pulled `back_mm` toward the shoulder and lifted `lift_mm`,
    the tool's orientation unchanged. None if out of reach."""
    x0, z0 = _plane(q_deg)
    r0 = abs(x0)
    if back_mm >= r0 - 50.0:
        return None
    x = x0 * (r0 - back_mm) / r0
    z = z0 + lift_mm
    a2, a3 = A[1] * 1000, A[2] * 1000
    c3 = (x * x + z * z - a2 * a2 - a3 * a3) / (2 * a2 * a3)
    if abs(c3) > 0.999:
        return None
    j3 = math.copysign(math.acos(c3), math.radians(q_deg[2]) or 1.0)
    j2 = math.atan2(z, x) - math.atan2(a3 * math.sin(j3), a2 + a3 * math.cos(j3))
    j2d = _near(math.degrees(j2), q_deg[1])
    j3d = _near(math.degrees(j3), q_deg[2])
    q = list(q_deg)
    q[3] = q_deg[3] - ((j2d - q_deg[1]) + (j3d - q_deg[2]))
    q[1], q[2] = j2d, j3d
    if any(not lo <= v <= hi for v, (lo, hi) in zip(q, LIMITS)):
        return None
    return [round(v, 3) for v in q]


# Where the links really run: the DH frames cut the corners. The upper arm runs 138 mm out
# along the shoulder's axis, the forearm 7 mm (138 less the elbow's 131 back), and the wrist
# joins at d4. Modelled through the frame origins alone, arm-a's and arm-c's waits read
# 389 mm apart while their forearm and upper arm overlapped for 149 s of the cycle (audit,
# 2026-10-14).
SHOULDER_OFFSET_MM = 138.0
FOREARM_OFFSET_MM = 7.0


def links(q, base_mm, yaw_deg, tool_len_mm: float = 0.0):
    """The arm (and its tool) as line segments in world mm."""
    fs = frames(q, base_mm, yaw_deg)
    p = [f[:3, 3] for f in fs]
    side = fs[1][:3, 2]                      # the shoulder/elbow/wrist-1 axis
    segs = [(p[0], p[1]),                                         # base to shoulder
            (p[1], p[1] + side * SHOULDER_OFFSET_MM),             # shoulder housing
            (p[1] + side * SHOULDER_OFFSET_MM, p[2] + side * SHOULDER_OFFSET_MM),  # upper arm
            (p[2] + side * SHOULDER_OFFSET_MM, p[2] + side * FOREARM_OFFSET_MM),   # elbow
            (p[2] + side * FOREARM_OFFSET_MM, p[3] + side * FOREARM_OFFSET_MM),    # forearm
            (p[3] + side * FOREARM_OFFSET_MM, p[4]),              # wrist 1
            (p[4], p[5]), (p[5], p[6])]                           # wrists 2 and 3
    if tool_len_mm:
        segs.append((p[6], p[6] + fs[6][:3, 2] * tool_len_mm))
    return segs


_segments = links


def seg_dist(a0, a1, b0, b1) -> float:
    """Closest distance between two segments."""
    d1, d2, r = a1 - a0, b1 - b0, a0 - b0
    aa, ee, ff = d1 @ d1, d2 @ d2, d2 @ r
    if aa < 1e-9 and ee < 1e-9:
        return float(np.linalg.norm(r))
    if aa < 1e-9:
        s, t = 0.0, min(1.0, max(0.0, ff / ee))
    else:
        c = d1 @ r
        if ee < 1e-9:
            t, s = 0.0, min(1.0, max(0.0, -c / aa))
        else:
            b = d1 @ d2
            den = aa * ee - b * b
            s = min(1.0, max(0.0, (b * ff - c * ee) / den)) if den > 1e-9 else 0.0
            t = (b * s + ff) / ee
            if t < 0.0:
                t, s = 0.0, min(1.0, max(0.0, -c / aa))
            elif t > 1.0:
                t, s = 1.0, min(1.0, max(0.0, (b - c) / aa))
    return float(np.linalg.norm((a0 + d1 * s) - (b0 + d2 * t)))


def apart(segs_a, segs_b) -> float:
    return min(seg_dist(a0, a1, b0, b1) for a0, a1 in segs_a for b0, b1 in segs_b)


def inside(segs, box_lo, box_hi, pad: float) -> bool:
    lo, hi = np.asarray(box_lo) - pad, np.asarray(box_hi) + pad
    for a, b in segs:
        for s in np.linspace(0.0, 1.0, 16):
            p = a + (b - a) * s
            if np.all(p >= lo) and np.all(p <= hi):
                return True
    return False


# Two arms' links (each LINK_RADIUS_MM) and this much air between them.
ARM_GAP_MM = 2 * LINK_RADIUS_MM + 60.0


def wait_for(q0: Sequence[float], base_mm, yaw_deg: float, tool_len_mm: float,
             box_lo, box_hi, tool_radius_mm: float = 60.0, others=()) -> Optional[Dict]:
    """The smallest pull-back-and-lift from `q0` that clears the sweep box and stands
    ARM_GAP_MM clear of every arm configuration in `others` (lists of segments)."""
    pad = max(LINK_RADIUS_MM, tool_radius_mm) + CLEAR_MM
    best = None
    for back in BACK_STEPS_MM:
        for lift in LIFT_STEPS_MM:
            q = moved(q0, back, lift)
            if q is None:
                continue
            segs = _segments(q, base_mm, yaw_deg, tool_len_mm)
            if inside(segs, box_lo, box_hi, pad):
                continue
            if others and min(apart(segs, o) for o in others) < ARM_GAP_MM:
                continue
            cost = sum(abs(a - b) for a, b in zip(q, q0))
            if best is None or cost < best["cost"]:
                pos, axis = flange(q, base_mm, yaw_deg)
                best = {"joints_deg": q, "back_mm": back, "lift_mm": lift,
                        "cost_deg": round(cost, 1), "cost": cost,
                        "flange_mm": [round(v, 2) for v in pos],
                        "tool_axis": [round(v, 4) for v in axis]}
    return best
