"""Do the pick and the handoff have to be on different elbow branches?

Both are reached with pan near -155, but the planner solves the pick with the elbow at
+81 and the handoff with it at -102 - opposite signs, a different arm shape at the same
base angle. The round pays for that once a part, as the 270 degree move that shows the
part's free side, and again on the way back. If one shape reaches both, the flip is the
planner's choice and the round can be taught out of it; if it is not, the flip is the
cost of the layout and the place to spend effort is the layout.

Seeded each way round, because which shape the planner starts from is the whole
question.
"""
import asyncio, os, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from qc.layout import Cell
from qc.round import Clients, lifted, props_of, world_state, move
from viam.components.arm import Arm, JointPositions
from viam.components.generic import Generic
from viam.components.gripper import Gripper
from viam.robot.client import RobotClient
from viam.services.motion import MotionClient

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")

PICK_SHAPE = [-155.5, -189.9, 80.7, 108.3, 24.8, -90.0]
HANDOFF_SHAPE = [-153.5, -116.2, -97.4, -146.2, -153.5, -90.0]


async def main():
    robot = await RobotClient.at_address(FQDN, RobotClient.Options.with_api_key(
        api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"]))
    c = Clients(motion=MotionClient.from_robot(robot, "builtin"),
                world=Generic.from_robot(robot, "sim-world"),
                arms={n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b")},
                grippers={"arm-a": Gripper.from_robot(robot, "suction-a"),
                          "arm-b": Gripper.from_robot(robot, "suction-b")})
    cell = Cell.from_fragment()
    for g in c.grippers.values():
        await g.open()
    await c.world.do_command({"command": "reset"})
    await asyncio.sleep(2.0)
    props = await props_of(c)
    state = world_state(props, cell=cell)

    trials = [
        ("pick approach from the handoff shape", HANDOFF_SHAPE,
         lifted(cell.pick("+y"), 140.0)),
        ("handoff give from the pick shape", PICK_SHAPE,
         cell.handoff_grip("-y", cell.RELEASE_GAP_MM)),
        ("presentation from the pick shape", PICK_SHAPE,
         cell.present_poses("arm-a")[0][1]),
        ("free side from the pick shape", PICK_SHAPE,
         cell.present_poses("arm-a")[-1][1]),
        ("free side from the handoff shape", HANDOFF_SHAPE,
         cell.present_poses("arm-a")[-1][1]),
    ]
    for name, seed, pose in trials:
        await c.arms["arm-a"].move_to_joint_positions(JointPositions(values=seed))
        try:
            await move(c, "arm-a", pose, state)
        except Exception as exc:
            print(f"  {name:<40} no solution: {str(exc)[:60]}")
            continue
        got = [round(v, 1) for v in (await c.arms["arm-a"].get_joint_positions()).values]
        worst = max(abs(a - b) for a, b in zip(seed, got))
        kept = "kept the shape " if (got[2] > 0) == (seed[2] > 0) else "FLIPPED elbow  "
        print(f"  {name:<40} {kept} worst joint {worst:5.0f}  {got}")

    await robot.close()


if __name__ == "__main__":
    asyncio.run(main())
