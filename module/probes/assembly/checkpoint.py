"""Stage the cell straight at a step of the cycle, and measure whether it holds.

A learner on the pin page should simulate the pin, not the 160 s before it. So a
CHECKPOINT is the cell exactly as it is when that step begins, built from a reset with the
same world commands the cycle uses:

    inside   shell at the inside stop, door fitted flat, pin in its nest, flag at its stop
    pin      the same, the belt indexed to the pin stop
    flag     + the pin seated in the hinge (placed on its seat, then its seat joint made)
    grasp    a reject (paint drip), unassembled, at the reject stop

Placing a seated part is the step this cell has the most history with - jointed parts
teleported into contact have left it at 423 m - so each checkpoint is built `--repeat`
times and measured: how long staging took, whether the shell sits square at its stop,
whether every seated part is still on its seat 3 s later, whether the feeders are ready.

    python probes/assembly/checkpoint.py [--repeat 3] [--only flag,grasp] [--run]

`--run` then replays the flag step from the taught program off the flag checkpoint, to
prove a step runs on its own.
"""
from __future__ import annotations

import argparse
import asyncio
import math
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from assembly.arms import home as arm_home                          # noqa: E402
from assembly.asset import insertions, part                         # noqa: E402
from assembly.build_fragment import chosen                          # noqa: E402
from assembly.creds import options                                  # noqa: E402
from assembly.fixture import (feed_grasp_mm, index_to, pallet_x_mm,  # noqa: E402
                              restage, restock_all)
from viam.components.arm import Arm, JointPositions                 # noqa: E402
from viam.components.generic import Generic                         # noqa: E402
from viam.components.gripper import Gripper                         # noqa: E402
from viam.robot.client import RobotClient                           # noqa: E402
# Imported BEFORE connecting: the client only builds clients for the APIs registered when
# it connects, so a motion service imported afterwards is "not found".
from viam.services.motion import MotionClient                       # noqa: E402

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")

# checkpoint -> (belt step it sits at, parts already seated, feeders restocked)
CHECKPOINTS = {
    "inside": ("inside", (), True),
    "pin": ("pin", (), True),
    "flag": ("flag", ("Pin",), True),
    "grasp": ("handle", (), False),     # a reject: arm-c only handles those, unassembled
}
SETTLE_S = 3.0
SQUARE_DEG = 1.0
STOP_MM = 2.0
SEAT_MM = 1.0


def _quat_from_axes(x, y, z):
    """(w, x, y, z) of the rotation whose columns are the given axes."""
    m = [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]]
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((0.25 * s), (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s,
                (m[1][0] - m[0][1]) / s)
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        return ((m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s,
                (m[0][2] + m[2][0]) / s)
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        return ((m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s,
                (m[1][2] + m[2][1]) / s)
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
    return ((m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s,
            0.25 * s)


async def _poses(world, names):
    return (await world.do_command({"command": "prim_poses",
                                    "paths": [f"/World/{n}" for n in names]}))["prims"]


def _in_shell(sh, local):
    ax = (sh["x_axis"], sh["y_axis"], sh["z_axis"])
    return [sh["position_mm"][i] + sum(ax[k][i] * local[k] for k in range(3))
            for i in range(3)]


async def seat(world, which: str) -> None:
    """Put a part on its seat in the shell's live frame, then make its seat joint - the
    joint the cycle makes when the arm seats it, with the same frames."""
    ins = insertions()[which]
    name = which.lower()
    sh = (await _poses(world, ["shell"]))["/World/shell"]
    origin = _in_shell(sh, ins.seated_mm)
    quat = _quat_from_axes(sh["x_axis"], sh["y_axis"], sh["z_axis"])
    await world.do_command({"command": "unjoin", "name": f"{name}_seat"})
    await world.do_command({"command": "place_prop", "name": name,
                            "position_mm": origin, "orientation_wxyz": list(quat)})
    await world.do_command({
        "command": "join", "name": f"{name}_seat",
        "body0": "/World/shell", "body1": f"/World/{name}", "kind": ins.joint,
        "local0_mm": list(ins.joint_anchor_mm),
        "local1_mm": [ins.joint_anchor_mm[i] - ins.seated_mm[i] for i in range(3)],
        "axis": list(ins.joint_axis_mm), "limits_deg": list(ins.joint_limits_deg),
        **({"drive": {k: v for k, v in ins.joint_drive.items() if not k.startswith("_")}}
           if ins.joint_drive else {})})


async def measure(world, cell, step: str, seated) -> dict:
    """The checkpoint as it stands: the datum, each seated part, the feeders."""
    names = ["shell", "pin", "flag"]
    got = await _poses(world, names)
    sh = got["/World/shell"]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, abs(float(sh["y_axis"][1]))))))
    off_stop = sh["position_mm"][0] - cell.index_x(step)
    out = {"tilt": tilt, "x_off": off_stop, "z": sh["position_mm"][2], "seat": {}}
    ins = insertions()
    for which in seated:
        want = _in_shell(sh, ins[which].seated_mm)
        out["seat"][which] = math.dist(got[f"/World/{which.lower()}"]["position_mm"], want)
    for which in ("Pin", "Flag"):
        if which in seated:
            continue
        p = got[f"/World/{which.lower()}"]
        local = [v * 1000.0 for v in part(which).at("assembly:graspPosition")]
        feat = [p["position_mm"][i] + local[0] * p["x_axis"][i] + local[1] * p["y_axis"][i]
                + local[2] * p["z_axis"][i] for i in range(3)]
        out.setdefault("fed", {})[which] = math.dist(feat, feed_grasp_mm(cell, which))
    return out


async def stage(world, arms, grips, cell, name: str) -> dict:
    step, seated, feeders = CHECKPOINTS[name]
    t0 = time.perf_counter()
    for g in grips.values():
        try:
            await g.open()
        except Exception:                                          # noqa: BLE001
            pass
    for a, handle in arms.items():
        await handle.move_to_joint_positions(JointPositions(values=arm_home(a)),
                                             timeout=120)
    t_park = time.perf_counter() - t0
    if feeders:
        await restock_all(world, cell)          # restages, then feeds both parts
    else:
        await restage(world, settle=2.5)
    t_reset = time.perf_counter() - t0 - t_park
    x = cell.index_x(step)
    if abs(await pallet_x_mm(world) - x) > STOP_MM:
        await index_to(world, cell, x)
    t_index = time.perf_counter() - t0 - t_park - t_reset
    for which in seated:
        await seat(world, which)
    await asyncio.sleep(0.5)
    staged = time.perf_counter() - t0
    first = await measure(world, cell, step, seated)
    await asyncio.sleep(SETTLE_S)
    later = await measure(world, cell, step, seated)
    ok = (later["tilt"] <= SQUARE_DEG and abs(later["x_off"]) <= STOP_MM
          and all(v <= SEAT_MM for v in later["seat"].values())
          and all(abs(later["seat"][k] - first["seat"][k]) <= 0.3 for k in later["seat"]))
    return {"name": name, "ok": ok, "staged_s": staged, "park_s": t_park,
            "reset_s": t_reset, "index_s": t_index, "first": first, "later": later}


async def run_flag_step(motion, world, arms, grips, cell) -> str:
    """Replay the flag step's taught legs off the flag checkpoint."""
    from assembly.film_cycle import CYCLE, _station
    from assembly.reach_check import declared, layout_hash
    from assembly.replay import Legs
    stamp = layout_hash()
    legs = Legs.load(stamp)
    firsts = [k for k, lab in sorted(legs.labels.items())
              if str(lab).startswith("flag_hole_look:")]
    legs.step = int(firsts[0].split(":")[0]) - 1
    for step, arm, pose, tool, _what in CYCLE:
        if step != "flag" or not arm:
            continue
        await _station(arm, pose, tool, arms=arms, grips=grips, motion=motion, world=world,
                       stamp=stamp, state=await declared(world, moving=arm), legs=legs,
                       cell=cell, carry_state=await declared(
                           world, moving=arm, motion=motion, grip=grips.get(arm), tools=True),
                       held_state=await declared(
                           world, moving=arm, motion=motion, grip=grips.get(arm)))
    got = await measure(world, cell, "flag", ("Pin", "Flag"))
    return f"flag step from its checkpoint: flag {got['seat']['Flag']:.2f} mm from its seat"


async def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeat", type=int, default=3)
    ap.add_argument("--only", default=",".join(CHECKPOINTS))
    ap.add_argument("--run", action="store_true")
    args = ap.parse_args()
    robot = await RobotClient.at_address(FQDN, options())
    world = Generic.from_robot(robot, "sim-world")
    arms = {n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b", "arm-c")}
    grips = {"arm-b": Gripper.from_robot(robot, "grip-b"),
             "arm-c": Gripper.from_robot(robot, "grip-c")}
    cell = chosen()
    motion = MotionClient.from_robot(robot, "builtin")
    results = []
    for name in [n for n in args.only.split(",") if n]:
        for k in range(args.repeat):
            r = await stage(world, arms, grips, cell, name)
            results.append(r)
            L = r["later"]
            print(f"  {name:<7} #{k + 1}  {'ok  ' if r['ok'] else 'FAIL'}  staged in "
                  f"{r['staged_s']:5.1f} s (park {r['park_s']:.1f}, reset+feed "
                  f"{r['reset_s']:.1f}, index {r['index_s']:.1f})  shell "
                  f"{L['tilt']:.2f} deg, {L['x_off']:+.1f} mm off its stop, z "
                  f"{L['z']:.1f}  seats {({k: round(v, 2) for k, v in L['seat'].items()})}"
                  f"  fed {({k: round(v, 1) for k, v in L.get('fed', {}).items()})}",
                  flush=True)
            if args.run and name == "flag" and k == 0:
                print("  " + await run_flag_step(motion, world, arms, grips, cell),
                      flush=True)
    good = sum(r["ok"] for r in results)
    print(f"\n  {good} of {len(results)} checkpoints staged and held", flush=True)
    await robot.close()
    return 0 if good == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
