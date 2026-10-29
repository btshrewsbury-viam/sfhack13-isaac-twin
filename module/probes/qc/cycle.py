"""What one part should cost this cell, on a line.

Derived from the cell's own stations rather than recalled from somewhere, because the
only honest source for "how long should this take" is how far the arms have to move and
how fast they are allowed to move. The speeds and dwells are stated once in the
fragment's `cycle` block; everything here is arithmetic on top of them.

Two numbers come out, and which one applies is a question about the cell and not about
the arms:

  SERIAL     one arm works, then the other. What the round does today, because arm-a
             cannot start the next part until arm-b has cleared the fixture.
  PIPELINED  arm-a picks and inspects part N+1 while arm-b finishes part N. What a line
             does, and then the cycle is the slower arm's own loop rather than the sum.

The gap between them is the single biggest lever on this cell's throughput - bigger than
any arm speed - because it nearly halves the cycle without the arms moving any faster.
"""
from __future__ import annotations

import math
from typing import Dict, List, Tuple

from .layout import Cell

# Where each arm parks. Taught poses, read off the round's own trace rather than derived,
# because home is a joint configuration and not a point the layout chooses.
HOME_FLANGE_MM = {"arm-a": (424.0, -218.0, 1118.0), "arm-b": (607.0, 352.0, 1219.0)}

# The approach heights the round actually uses, from round.py.
PICK_CLEAR_MM = 35.0
LIFT_MM = 219.0
SEAT_CLEAR_MM = 160.0
PLACE_CLEAR_MM = 202.0


def move_seconds(distance_mm: float, speed_mm_s: float, accel_mm_s2: float) -> float:
    """A trapezoidal move: ramp up, cruise, ramp down.

    Short moves never reach the cruise speed, and most of this cell's moves are short -
    a 35 mm descent is all ramp. Treating every move as distance/speed would under-count
    them by a factor of three.
    """
    if distance_mm <= 0.0:
        return 0.0
    ramp = speed_mm_s * speed_mm_s / accel_mm_s2
    if distance_mm < ramp:
        return 2.0 * math.sqrt(distance_mm / accel_mm_s2)
    return 2.0 * speed_mm_s / accel_mm_s2 + (distance_mm - ramp) / speed_mm_s


def legs(cell: Cell, pick_mm=None) -> Dict[str, List[Tuple[str, float]]]:
    """Each arm's loop, as (name, distance) pairs."""
    from .round import nominal_handoff_flanges, nominal_place_flange

    pick = cell.side_grip(pick_mm or cell.pick_mm, cell.GRIP_SIDE)[0]
    give, take = nominal_handoff_flanges(cell)
    place = nominal_place_flange(cell, "good")[0]
    rolls_a, free_a = (cell.present_poses("arm-a")[0][1][0],
                       cell.present_poses("arm-a")[-1][1][0])
    rolls_b, free_b = (cell.present_poses("arm-b")[0][1][0],
                       cell.present_poses("arm-b")[-1][1][0])
    up = lambda p, by: (p[0], p[1], p[2] + by)  # noqa: E731

    def chain(points):
        return [(name, math.dist(a, b)) for name, a, b in points]

    return {
        "arm-a": chain([
            ("home -> above pick", HOME_FLANGE_MM["arm-a"], up(pick, PICK_CLEAR_MM)),
            ("descend to the pick", up(pick, PICK_CLEAR_MM), pick),
            ("lift off the belt", pick, up(pick, LIFT_MM)),
            ("carry to the inspection", up(pick, LIFT_MM), rolls_a),
            ("inspection -> free face", rolls_a, free_a),
            ("free face -> fixture", free_a, up(give, SEAT_CLEAR_MM)),
            ("descend to the seat", up(give, SEAT_CLEAR_MM), give),
            ("retreat", give, up(give, SEAT_CLEAR_MM)),
            ("home", up(give, SEAT_CLEAR_MM), HOME_FLANGE_MM["arm-a"]),
        ]),
        "arm-b": chain([
            ("home -> above take", HOME_FLANGE_MM["arm-b"], up(take, PICK_CLEAR_MM)),
            ("descend to the take", up(take, PICK_CLEAR_MM), take),
            ("lift off the fixture", take, up(take, PICK_CLEAR_MM)),
            ("carry to the inspection", up(take, PICK_CLEAR_MM), rolls_b),
            ("inspection -> free face", rolls_b, free_b),
            ("free face -> place", free_b, up(place, PLACE_CLEAR_MM)),
            ("descend to the place", up(place, PLACE_CLEAR_MM), place),
            ("retreat", place, up(place, PLACE_CLEAR_MM)),
            ("home", up(place, PLACE_CLEAR_MM), HOME_FLANGE_MM["arm-b"]),
        ]),
    }


def budget(cell: Cell) -> Dict[str, object]:
    """The cycle this cell should achieve, and where the time goes."""
    c = cell.cycle
    if not c:
        raise ValueError(
            "the fragment states no `cycle` block, so there is no budget to measure "
            "against; a cycle time has to be a stated target, not a number a probe picks")
    speed, accel = c["tcp_speed_mm_s"], c["tcp_accel_mm_s2"]
    # Four rolls, each a reorientation plus a frame; the free face is reached by a move
    # that is already counted, so it adds a frame and no reorientation.
    fixed = (4.0 * (c["reorient_s"] + c["camera_dwell_s"]) + c["camera_dwell_s"]
             + c["grip_s"] + c["release_s"])
    arms, detail = {}, {}
    for arm, chain in legs(cell).items():
        spent = [(name, d, move_seconds(d, speed, accel)) for name, d in chain]
        detail[arm] = spent
        arms[arm] = sum(s for _, _, s in spent) + fixed
    return {
        "per_arm_s": arms,
        "detail": detail,
        "handling_s": fixed,
        "serial_s": sum(arms.values()),
        "pipelined_s": max(arms.values()),
        "parts_per_hour_serial": 3600.0 / sum(arms.values()),
        "parts_per_hour_pipelined": 3600.0 / max(arms.values()),
    }


def report(cell: Cell, measured_s: float = None) -> str:
    b = budget(cell)
    lines = [f"  cycle budget, from the cell's own distances at "
             f"{cell.cycle['tcp_speed_mm_s']:.0f} mm/s and "
             f"{cell.cycle['tcp_accel_mm_s2']:.0f} mm/s^2:"]
    for arm, spent in b["detail"].items():
        lines.append(f"    {arm}")
        for name, d, s in spent:
            lines.append(f"      {name:<26} {d:7.1f} mm  {s:5.2f}s")
        lines.append(f"      {'handling (rolls, grip, release)':<26} {'':7}     "
                     f"{b['handling_s']:5.2f}s")
        lines.append(f"      {'-> arm loop':<26} {'':7}     {b['per_arm_s'][arm]:5.2f}s")
    lines.append(f"    serial    {b['serial_s']:6.1f}s  "
                 f"({b['parts_per_hour_serial']:.0f} parts/hour)")
    lines.append(f"    pipelined {b['pipelined_s']:6.1f}s  "
                 f"({b['parts_per_hour_pipelined']:.0f} parts/hour)")
    if measured_s is not None:
        over = measured_s / b["serial_s"]
        lines.append(f"    measured  {measured_s:6.1f}s  "
                     f"= {over:.1f}x the serial budget")
    return "\n".join(lines)
