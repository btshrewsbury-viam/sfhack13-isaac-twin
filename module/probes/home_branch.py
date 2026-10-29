"""Find the park pose's solution on the branch the round actually works on.

Measured: arm-a's pick and both handoff grips have no solution with the arm facing the
station - the planner swings pan to -155 and the shoulder past -130 however it is
seeded, while every presentation pose is reachable facing forwards. So the working half
of the round lives on one branch and the park pose was on the other, and the round paid
to cross twice a part: 185 degrees on the way out and 217 on the way back, the two
largest moves it made.

This seeds the arm at the configuration it is in when it finishes at the handoff and
asks the planner for the same park POSITION from there. What comes back is the park
pose on the working branch - a pose the planner has accepted, rather than one picked by
hand and commanded without checking.
"""
import asyncio, os, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from qc.layout import Cell
from qc.round import Clients, home_pose, props_of, world_state, move
from viam.components.arm import Arm, JointPositions
from viam.components.generic import Generic
from viam.components.gripper import Gripper
from viam.robot.client import RobotClient
from viam.services.motion import MotionClient

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")

# Where each arm is when it has finished its half of the round, from the taught program.
WORKING = {"arm-a": [-153.5, -131.4, -102.1, -126.4, -153.5, -90.0],
           "arm-b": [-176.9, -157.0, -95.0, 72.0, -3.0, 90.0]}


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

    for arm in ("arm-a", "arm-b"):
        seed = WORKING[arm]
        await c.arms[arm].move_to_joint_positions(JointPositions(values=seed))
        try:
            await move(c, arm, home_pose(cell, arm), state)
        except Exception as exc:
            print(f"  {arm}: the park pose has no solution on the working branch: "
                  f"{str(exc)[:90]}")
            continue
        got = [round(v, 1) for v in (await c.arms[arm].get_joint_positions()).values]
        cost = sum(abs(a - b) for a, b in zip(seed, got))
        print(f"  {arm}: park at {got}  ({cost:.0f} degrees from the working pose)")
        # And back out again, which is the move the round makes at the start.
        await move(c, arm, home_pose(cell, arm), state)

    await robot.close()


if __name__ == "__main__":
    asyncio.run(main())
