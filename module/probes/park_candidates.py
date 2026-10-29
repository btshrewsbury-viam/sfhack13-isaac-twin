"""Where a park pose on the working branch actually puts the tool.

The park POSITION the round used - close in, high, inboard - has no cheap solution on
the branch the working poses live on: asked for it from the handoff configuration the
planner spent 315 degrees on arm-a and 751 on arm-b getting there, which is worse than
the branch crossing it was meant to remove. So park is not a position to be reached but
a configuration to retreat INTO: the arm stays where it is and lifts its shoulder until
the tool is clear. This reports where each amount of lift puts the flange, so the
retreat can be chosen against the cell's geometry rather than guessed.
"""
import asyncio, os, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from qc.layout import Cell
from qc.round import Clients, flange_of, props_of
from viam.components.arm import Arm, JointPositions
from viam.components.generic import Generic
from viam.components.gripper import Gripper
from viam.robot.client import RobotClient
from viam.services.motion import MotionClient

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")

FINISHED = {"arm-a": [-153.5, -116.2, -97.4, -146.2, -153.5, -90.0],
            "arm-b": [-168.0, -143.7, -79.2, 42.9, 78.0, 89.9]}


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

    for arm in ("arm-a", "arm-b"):
        base = cell.arm_bases_mm[arm]
        for lift in (0.0, 15.0, 30.0, 45.0):
            joints = list(FINISHED[arm])
            joints[1] += lift
            await c.arms[arm].move_to_joint_positions(JointPositions(values=joints))
            where = await flange_of(c, arm)
            print(f"  {arm} lift +{lift:4.0f}  flange {[round(v) for v in where]}  "
                  f"tool tip z {where[2] - cell.tool_mm:5.0f}  "
                  f"out {round(((where[0]-base[0])**2 + (where[1]-base[1])**2) ** 0.5)}")
        print()

    await robot.close()


if __name__ == "__main__":
    asyncio.run(main())
