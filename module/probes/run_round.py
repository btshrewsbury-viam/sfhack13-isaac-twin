"""Drive one QC round against the live machine and print what happened.

    .venv/bin/python probes/run_round.py                    # a clean part
    .venv/bin/python probes/run_round.py defect_smudge      # one mark, on the +y panel
    .venv/bin/python probes/run_round.py --plan             # re-plan instead of replay

If the cell has a taught program it is replayed, which is what makes one cycle look
like the last. `--plan` asks the motion service instead, which is how the program is
taught in the first place and how a changed cell is re-taught.

Which tray the part ends in is the round's answer, not this script's instruction.
"""

import asyncio
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from qc.cycle import report as cycle_report  # noqa: E402
from qc.layout import Cell  # noqa: E402
from qc.round import Clients, Program, Trace, run_round  # noqa: E402
from viam.components.arm import Arm  # noqa: E402
from viam.components.camera import Camera  # noqa: E402
from viam.components.generic import Generic  # noqa: E402
from viam.components.gripper import Gripper  # noqa: E402
from viam.robot.client import RobotClient  # noqa: E402
from viam.services.motion import MotionClient  # noqa: E402

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")


async def connect() -> tuple:
    robot = await RobotClient.at_address(
        FQDN,
        RobotClient.Options.with_api_key(
            api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"]))
    clients = Clients(
        motion=MotionClient.from_robot(robot, "builtin"),
        world=Generic.from_robot(robot, "sim-world"),
        arms={n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b")},
        grippers={"arm-a": Gripper.from_robot(robot, "suction-a"),
                  "arm-b": Gripper.from_robot(robot, "suction-b")},
        camera=Camera.from_robot(robot, "inspect-cam"),
        frames_base=Path(__file__).resolve().parents[1] / ".scratch" / "frames",
    )
    return robot, clients


async def main() -> int:
    arguments = [a for a in sys.argv[1:] if not a.startswith("--")]
    replaying = "--plan" not in sys.argv
    defects = {name: True for name in arguments}
    robot, clients = await connect()
    program_file = Path(__file__).resolve().parents[1] / "fragments" / "qc-cell-program.json"
    if replaying and program_file.exists():
        clients.program = Program.from_json(json.loads(program_file.read_text()))
        print(f"  replaying the taught program ({len(clients.program.paths)} moves)")
    else:
        print("  planning every move")
    trace = Trace()
    failure = None
    try:
        await run_round(clients, Cell.from_fragment(), defects=defects, trace=trace)
    except Exception as exc:  # noqa: BLE001
        # Print the trace anyway. A round that dies mid-way is exactly when the record
        # of what it managed is worth most, and an exception that eats it costs a run.
        failure = str(exc)
    finally:
        await robot.close()

    for stage in trace.stages:
        name = stage["stage"]
        detail = {k: v for k, v in stage.items() if k != "stage"}
        at, took = detail.pop("at_s", 0.0), detail.pop("took_s", 0.0)
        if "part" in detail:
            detail["part"] = [round(v) for v in detail["part"]]
        if "flange" in detail:
            detail["flange"] = [round(v) for v in detail["flange"]]
        if "offset" in detail:
            detail["offset"] = [round(v) for v in detail["offset"]]
        detail.pop("orientation_wxyz", None)
        if "frame" in detail and detail["frame"]:
            detail["frame"] = Path(detail["frame"]).name
        print(f"  {at:7.1f}s +{took:5.1f}s  {name:>24}  {detail}")

    # The cycle time, and where it went. Printed for every round rather than asked for,
    # because the question "how long does one part take" is the first one anyone puts to
    # a cell and the answer was not in the record at all.
    print(f"\n  cycle: {trace.elapsed_s:.1f}s over {len(trace.stages)} stages")
    try:
        print(cycle_report(Cell.from_fragment(), measured_s=trace.elapsed_s))
    except Exception as exc:  # noqa: BLE001 - a missing budget must not eat the trace
        print(f"  (no cycle budget: {exc})")
    print("  slowest stages:")
    for stage in trace.slowest(8):
        print(f"    {stage['took_s']:6.1f}s  {stage['stage']}")
    if failure:
        print(f"\n  FAILED: {failure[:1600]}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
