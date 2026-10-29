"""Plan the round once, keep the joint paths, and save them as the cell's program.

    .venv/bin/python probes/teach_round.py            # teach, then verify by replay

A cell like this runs one fixed sequence over and over. Planning it again every cycle
asks a randomised algorithm for a fresh answer each time, and gets one - the same
station reached through a different arm configuration on different runs. Teaching it
once and replaying is what an industrial cell does, and it is the difference between
motion that is merely usually fine and motion that is the same every cycle.

The teaching pass is the one that does the planning, with every obstacle declared. What
is saved is what the planner produced and the arm actually executed.
"""

import asyncio
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from qc.layout import Cell  # noqa: E402
from qc.round import Clients, Program, Trace, run_round  # noqa: E402
from viam.components.arm import Arm  # noqa: E402
from viam.components.camera import Camera  # noqa: E402
from viam.components.generic import Generic  # noqa: E402
from viam.components.gripper import Gripper  # noqa: E402
from viam.robot.client import RobotClient  # noqa: E402
from viam.services.motion import MotionClient  # noqa: E402

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
PROGRAM = Path(__file__).resolve().parents[1] / "fragments" / "qc-cell-program.json"
ARMS = ("arm-a", "arm-b")


async def connect():
    robot = await RobotClient.at_address(
        FQDN, RobotClient.Options.with_api_key(
            api_key=os.environ["VIAM_API_KEY"],
            api_key_id=os.environ["VIAM_API_KEY_ID"]))
    return robot, Clients(
        motion=MotionClient.from_robot(robot, "builtin"),
        world=Generic.from_robot(robot, "sim-world"),
        arms={n: Arm.from_robot(robot, n) for n in ARMS},
        grippers={"arm-a": Gripper.from_robot(robot, "suction-a"),
                  "arm-b": Gripper.from_robot(robot, "suction-b")},
        camera=Camera.from_robot(robot, "inspect-cam"),
        frames_base=Path(__file__).resolve().parents[1] / ".scratch" / "frames")


async def main() -> int:
    marks = {name: True for name in sys.argv[1:] if not name.startswith("--")}
    robot, clients = await connect()
    try:
        for arm in ARMS:
            await clients.arms[arm].do_command({"command": "forget"})

        # Teach several times and keep the cheapest. Planning is randomised, so passes
        # differ in which arm configuration they pick at each station, and a pass that
        # flips the shoulder somewhere pays for it on every cycle afterwards. Failed
        # passes are simply not candidates - a program taught from a round that did not
        # finish has a hole in it.
        attempts = int(os.environ.get("TEACH_ATTEMPTS", "6"))
        wanted = int(os.environ.get("TEACH_KEEP_BEST", "3"))
        # Steps already taught are REPLAYED rather than re-planned, so teaching the
        # second ending extends one program instead of starting a rival one. Without
        # this the two endings sit on different shared prefixes and only one of them
        # is replayable.
        known = Program(teaching=True)
        if PROGRAM.exists() and "--fresh" not in sys.argv:
            saved = Program.from_json(json.loads(PROGRAM.read_text()))
            known = Program(teaching=True, paths=dict(saved.paths),
                            targets=dict(saved.targets))
            print(f"  building on the {len(known.paths)} moves already taught")
        best = None
        completed = 0
        salvage: dict = {}
        salvage_targets: dict = {}
        for attempt in range(1, attempts + 1):
            for arm in ARMS:
                await clients.arms[arm].do_command({"command": "forget"})
            clients.program = Program(teaching=True, paths=dict(known.paths),
                                      targets=dict(known.targets))
            trace = Trace()
            try:
                await run_round(clients, Cell.from_fragment(), defects=marks,
                                trace=trace)
            except Exception as exc:  # noqa: BLE001
                # Keep what the failed pass DID teach, as a prefix for the next run.
                #
                # Teaching this round plans every move with every obstacle declared, and
                # viam-server's memory grows with the number of plans it has served - a
                # pass reached 12.8 GiB of a 13 GiB cap and was killed, and a bigger cap
                # only moves where that happens. So teaching has to be able to make
                # progress across restarts, which the replay-the-known-prefix design
                # already allows for; it just had nothing to replay, because a pass that
                # did not finish was thrown away whole.
                partial = {}
                for arm in ARMS:
                    try:
                        reply = await clients.arms[arm].do_command(
                            {"command": "stop_recording"})
                        partial.update(reply["taught"])
                    except Exception:  # noqa: BLE001 - the cell may be gone
                        pass
                partial = {k: Program.tidy(v) for k, v in partial.items() if v}
                if len(partial) > len(salvage):
                    salvage, salvage_targets = partial, dict(clients.program.targets)
                # Long on purpose. Truncated to 100 characters this said only "the cell
                # could not show the camera every face" and hid WHICH check and why -
                # the third time in this project that a truncated error cost a cycle.
                print(f"  pass {attempt}: did not finish after {len(partial)} moves"
                      f" - {str(exc)[:1200]}")
                print(f"      reached: {' -> '.join(trace.names[-6:])}")
                for stage in trace.stages:
                    if "missed" in stage["stage"]:
                        print(f"      {stage['stage']}: "
                              f"{str(stage.get('error'))[:400]}")
                continue
            taken = {}
            for arm in ARMS:
                reply = await clients.arms[arm].do_command(
                    {"command": "stop_recording"})
                taken.update(reply["taught"])
            taken = {k: Program.tidy(v) for k, v in taken.items()}
            cost = Program.travel(taken)
            completed += 1
            mark = ""
            if best is None or cost < best[0]:
                best = (cost, taken, dict(clients.program.targets), trace)
                mark = "  <- best so far"
            print(f"  pass {attempt}: {cost:7.0f} degrees of joint travel{mark}")
            if completed >= wanted:
                break
        if best is None:
            if not salvage:
                print("  no teaching pass completed, and none taught a single move")
                return 1
            # Save the furthest partial as a prefix and say so. It is NOT a program: it
            # has a hole after the last move it reached, and replaying it will stop
            # there. It is the thing the next teaching run builds on.
            existing = {}
            existing_targets = {}
            if PROGRAM.exists() and "--fresh" not in sys.argv:
                was = Program.from_json(json.loads(PROGRAM.read_text()))
                existing, existing_targets = dict(was.paths), dict(was.targets)
            existing.update(salvage)
            existing_targets.update(salvage_targets)
            PROGRAM.write_text(json.dumps(
                Program(paths=existing, targets=existing_targets).to_json(),
                indent=1) + "\n")
            print(f"  no pass completed. Saved the furthest partial - {len(salvage)} "
                  f"moves, {len(existing)} in the file - as a PREFIX to build on. "
                  f"Run again (without --fresh) after restarting the module.")
            return 1
        cost, paths, targets, trace = best
        print(f"  keeping the {cost:.0f} degree program")
        # Keep whatever was taught before, so teaching the clean round and the defective
        # one builds one program with both endings rather than replacing each other.
        if PROGRAM.exists() and "--fresh" not in sys.argv:
            existing = Program.from_json(json.loads(PROGRAM.read_text()))
            merged = dict(existing.paths)
            merged.update(paths)
            paths = merged
            merged_targets = dict(existing.targets)
            merged_targets.update(targets)
            targets = merged_targets
        taught = Program(paths=paths, targets=targets)
        PROGRAM.write_text(json.dumps(taught.to_json(), indent=1) + "\n")
        points = sum(len(p) for p in paths.values())
        print(f"  program now holds {len(paths)} moves, {points} joint waypoints "
              f"-> {PROGRAM.name}")
        print(f"  the round reached: {trace.names[-1]}")
        return 0
    finally:
        await robot.close()


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
