"""What one mailbox costs this cell, measured and then budgeted.

Two numbers, and they answer different questions:

  MEASURED   what the cell does now, timed station by station with the deliberate pauses
             taken out. This is a real figure for this cell at these speeds and nothing
             more - it includes planning time, which a line would not pay per part because
             the paths would be taught once and replayed.
  PIPELINED  what the same work costs when the three arms overlap. Today the cycle is the
             SUM of three arms' work because the belt indexes between them and only one
             arm moves at a time. On a line each arm works on a different mailbox and the
             cycle is the busiest arm's own loop.

The gap between them is the biggest lever on this cell's throughput, and it is a layout
question rather than an arm-speed one: arm-b does nearly all the assembly, so pipelining
is worth little until its work is split.
"""
from __future__ import annotations

import asyncio
import json
import os
import sys
import time
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from assembly.creds import options                             # noqa: E402
from assembly.arms import clear_and_park, home as arm_home, park_if_adrift  # noqa: E402
from assembly.build_fragment import SHELL_KINEMATIC, chosen         # noqa: E402
from assembly.film_cycle import CYCLE, LOOSE_STOP_MM, LOOSE_STOPS, _station, line_event                  # noqa: E402
from assembly.fixture import index_to, restage, restock_all, sim_now  # noqa: E402
from assembly.reach_check import step_out                              # noqa: E402
from assembly import shell_locate                                      # noqa: E402
from viam.components.camera import Camera   # before connecting      # noqa: E402
from assembly.replay import Legs, fly as fly_leg                                 # noqa: E402
from assembly.reach_check import POSES, declared, layout_hash, orientation_for            # noqa: E402
from viam.components.arm import Arm, JointPositions                 # noqa: E402
from viam.components.generic import Generic                         # noqa: E402
from viam.components.gripper import Gripper                         # noqa: E402
from viam.robot.client import RobotClient                           # noqa: E402
from viam.services.motion import MotionClient                       # noqa: E402

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
OUT = Path(".scratch/runs/assembly-cycle-time.json").resolve()


async def main() -> int:
    robot = await RobotClient.at_address(
        FQDN, options())
    world = Generic.from_robot(robot, "sim-world")
    motion = MotionClient.from_robot(robot, "builtin")
    arms = {n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b", "arm-c")}
    shell_locate.CLIENT = Camera.from_robot(robot, shell_locate.CAMERA)
    grips = {"arm-b": Gripper.from_robot(robot, "grip-b"),
             "arm-c": Gripper.from_robot(robot, "grip-c")}
    cell, stamp = chosen(), layout_hash()

    # Measured on the path the cell RUNS, which is replay. The planning path was measured
    # first and the number it gave - 323.2 s against a 120 s target - is not this cell's
    # cycle time: it is the motion service searching, 30-60 s a leg, which no speed
    # setting touches. `--plan` still measures it, for the comparison.
    planning = "--plan" in sys.argv[1:]
    # `--good`: a good shell alone - every check, both inserts and its ride off the end,
    # without the second shell and arm-c's reject take that the full cycle goes on to.
    cycle = CYCLE
    if "--good" in sys.argv[1:]:
        cycle = CYCLE[:next(i for i, s in enumerate(CYCLE) if s[0] == "away") + 1]
    legs = None if planning else Legs.load(stamp)
    print(f"  layout {stamp}; "
          + ("PLANNING every leg - not a cycle time" if planning
             else f"replaying {len(legs.paths)} taught legs"), flush=True)

    # `restage`, not a bare reset - the one way this cell resets. See `fixture.restage`:
    # a reset with the door's hinge live lands the shell 4.15 degrees out of square and
    # 5.2 mm high, every time, and a cycle time measured on a crooked shell is a
    # measurement of corrections rather than of the cycle.
    await restage(world, settle=3.0)
    await clear_and_park(world, arms, cell)
    # The flag and the pin ride in on the tray; the door arrives already on the shell, so
    # its hinge is made here rather than by an arm. Both are what the cycle does, and a
    # cycle time measured with the door being fetched and capped is a measurement of a
    # cycle this cell no longer runs.
    # `restock_all`, not a loop of single restocks. The singular one teleports the part
    # onto its slot unconditionally, and a `place_prop` onto a body already in resting
    # contact is what threw the flag anywhere between 435 mm and 2.4 m up, two to four
    # times per load. `restock_all` looks first and leaves a part the restage already
    # staged alone, which is every part in the normal case.
    await restock_all(world, cell)
    # The door's hinge is made by `restage` above, during the settling fall, and re-making
    # it here would author it a second time over a door that has since come to rest - which
    # is the 4.15 degree case exactly.
    await asyncio.sleep(2.0)

    steps, index_s, per_arm = [], 0.0, defaultdict(float)
    at, fitted = None, []
    wall_from = time.perf_counter()
    t_cycle = await sim_now(world)
    if legs is not None:
        legs.step = 0
    for step_no, (step, arm, pose, tool, _what) in enumerate(cycle):
        t0 = await sim_now(world)
        if await line_event(world, cell, tool):
            # The line between shells - a passed one riding off, the next arriving.
            index_s += await sim_now(world) - t0
            at = None if tool == "ride" else cell.index_x("tunnel")
            fitted = [] if tool != "ride" else fitted
            continue
        x = cell.index_x(step)
        if at != x:
            t0 = await sim_now(world)
            at = await index_to(world, cell, x,
                                fitted=tuple(fitted) if not SHELL_KINEMATIC else (),
                                tolerance_mm=(LOOSE_STOP_MM if step in LOOSE_STOPS else None))
            index_s += await sim_now(world) - t0
        if not arm:
            continue
        await park_if_adrift(arms[arm], arm)
        t0 = await sim_now(world)
        # `cell` is not optional any more. The insert re-references itself into the
        # shell's measured frame, and that needs the cell's nominal origin - without it
        # the seat branch reaches `cell.belt_y_mm` on None. The pick corrections need it
        # too, and a cycle time measured with them skipped is a measurement of a cycle
        # this cell does not run.
        await _station(arm, pose, tool, arms=arms, grips=grips, motion=motion,
                       world=world, stamp=stamp,
                       state=await declared(world, moving=arm), legs=legs, cell=cell,
                       carry_state=await declared(world, moving=arm, motion=motion, grip=grips.get(arm), tools=True),
                       held_state=await declared(world, moving=arm, motion=motion, grip=grips.get(arm)))
        if tool and tool.startswith("seat:"):
            fitted.append(tool.split(":", 1)[1])
        work = await sim_now(world) - t0
        # Park ONLY IF THE NEXT STATION IS A DIFFERENT ARM, which is what the cycle does.
        # Parking after every station was thirteen round trips to nowhere and a third of
        # the measured time; arm-b does five stations in a row and parks once.
        t0 = await sim_now(world)
        # The SAME parks the film makes, as the same TAUGHT legs, so the program's leg
        # numbering matches: the park when the next station is another arm or another
        # stop, and arm-c's home after the reject (film_cycle, 2026-10-13).
        nxt = cycle[step_no + 1][1] if step_no + 1 < len(cycle) else None
        nxt_step = cycle[step_no + 1][0] if step_no + 1 < len(cycle) else None
        if nxt == arm and nxt_step != step:
            await step_out(arms, cell, arm)           # as the film does
        elif (nxt is not None and (nxt != arm or nxt_step != step)) or pose == "repaint":
            try:
                pj = json.loads((POSES / arm / "park.json").read_text())
                await fly_leg(legs, arms, motion, arm, pj["flange_mm"],
                              orientation_for(pj["approach"], pj.get("theta_deg", 0.0)),
                              await declared(world, moving=arm, tools=True),
                              label="home" if pose == "repaint" else "park")
            except Exception as exc:                               # noqa: BLE001
                print(f"    park {arm}: {str(exc)[:100]}", flush=True)
            await arms[arm].move_to_joint_positions(
                JointPositions(values=arm_home(arm)), timeout=180)
        park = await sim_now(world) - t0
        steps.append({"step": step, "arm": arm, "pose": pose,
                      "work_s": round(work, 2), "park_s": round(park, 2)})
        per_arm[arm] += work + park
        print(f"  {step:<8} {arm}  {pose:<18} work {work:6.2f}s  park {park:5.2f}s",
              flush=True)
    total = await sim_now(world) - t_cycle

    busiest = max(per_arm.values()) if per_arm else 0.0
    report = {
        "layout": stamp,
        "measured_total_s": round(total, 1),
        "belt_indexing_s": round(index_s, 1),
        "arm_work_s": {k: round(v, 1) for k, v in sorted(per_arm.items())},
        "serial_s": round(sum(per_arm.values()) + index_s, 1),
        "pipelined_s": round(busiest + index_s, 1),
        "steps": steps,
        "replaying": legs is not None,
        "clock": "sim seconds (status sim_clock), not wall",
        "_note": ("measured with every leg PLANNED, which is the motion service searching "
                  "at 30-60 s a leg and is not this cell's cycle time - run without "
                  "--plan to measure the replay path the cell actually runs."
                  if legs is None else
                  "measured on the REPLAY path: the taught joint paths are driven and "
                  "nothing is planned, which is what the cell runs and what the film "
                  "shows. Parks happen only when the next station is a different arm."),
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(report, indent=1) + "\n")
    print(f"\n  sim seconds; wall {time.perf_counter() - wall_from:.1f}s")
    print(f"\n  measured {total:.1f}s   belt {index_s:.1f}s   "
          f"arms {dict(report['arm_work_s'])}")
    print(f"  serial {report['serial_s']}s   pipelined {report['pipelined_s']}s "
          f"(busiest arm + indexing)")
    print(f"  wrote {OUT}")
    await robot.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
