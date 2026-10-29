"""How far do the arms actually travel in a round, and where?

"The movements are wonky" is a real observation and unactionable until it is a number.
This samples both arms' joints while a round runs and reports the total angular travel,
per joint, per stage - so a change can be judged instead of eyeballed.

The number that matters most is travel divided by the distance actually covered: a leg
that moves the flange 300 mm should not cost 400 degrees of joint motion.
"""
import asyncio, json, os, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from qc.layout import Cell
from qc.round import Clients, Program, Trace, run_round
from viam.components.arm import Arm
from viam.components.camera import Camera
from viam.components.generic import Generic
from viam.components.gripper import Gripper
from viam.robot.client import RobotClient
from viam.services.motion import MotionClient

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
HZ = 8.0


async def main():
    robot = await RobotClient.at_address(FQDN, RobotClient.Options.with_api_key(
        api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"]))
    c = Clients(motion=MotionClient.from_robot(robot, "builtin"),
                world=Generic.from_robot(robot, "sim-world"),
                arms={n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b")},
                grippers={"arm-a": Gripper.from_robot(robot, "suction-a"),
                          "arm-b": Gripper.from_robot(robot, "suction-b")})
    program_file = Path(__file__).resolve().parents[1] / "fragments" / "qc-cell-program.json"
    if "--plan" not in sys.argv and program_file.exists():
        c.program = Program.from_json(json.loads(program_file.read_text()))
        print(f"  replaying the taught program ({len(c.program.paths)} moves)")
    else:
        print("  planning every move")

    trace = Trace()
    sampling = True
    travel = {"arm-a": [0.0] * 6, "arm-b": [0.0] * 6}
    per_stage = {}
    last = {}

    async def sample():
        while sampling:
            stage = trace.names[-1] if trace.names else "start"
            for arm in ("arm-a", "arm-b"):
                try:
                    now = list((await c.arms[arm].get_joint_positions()).values)
                except Exception:  # noqa: BLE001
                    continue
                if arm in last:
                    step = sum(abs(a - b) for a, b in zip(now, last[arm]))
                    for i in range(6):
                        travel[arm][i] += abs(now[i] - last[arm][i])
                    key = (stage, arm)
                    per_stage[key] = per_stage.get(key, 0.0) + step
                last[arm] = now
            await asyncio.sleep(1.0 / HZ)

    task = asyncio.create_task(sample())
    started = time.perf_counter()
    failure = None
    try:
        await run_round(c, Cell.from_fragment(), defects={"defect_smudge": True},
                        trace=trace)
    except Exception as exc:  # noqa: BLE001
        failure = str(exc)[:160]
    sampling = False
    await task
    took = time.perf_counter() - started

    print(f"\n  round took {took:.0f}s" + (f"  FAILED: {failure}" if failure else ""))
    for arm in ("arm-a", "arm-b"):
        total = sum(travel[arm])
        print(f"  {arm}: {total:7.0f} deg total   per joint "
              f"{[round(v) for v in travel[arm]]}")
    print(f"  BOTH ARMS: {sum(sum(v) for v in travel.values()):.0f} deg")
    print("\n  worst stages by joint travel:")
    for (stage, arm), deg in sorted(per_stage.items(), key=lambda kv: -kv[1])[:12]:
        print(f"    {deg:7.0f} deg  {arm}  during {stage}")
    await robot.close()

asyncio.run(main())
