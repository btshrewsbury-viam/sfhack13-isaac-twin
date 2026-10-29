"""Why does the free-face presentation fail when the wrists are allowed past half a turn?

Widening wrist 1 and 2 to +-270 fixes what it was meant to - the handoff descent's flip
falls from 307 degrees to 203 - and breaks this one pose, on both arms, twice out of
twice. It was reverted without an explanation, which leaves the cell choosing between two
known-imperfect states for no stated reason.

A missed presentation records only that three attempts failed, so the round cannot say
why. This asks for that one pose directly and prints the planner's whole answer: seeded
from the shape the arm is in when it has just finished the rolls, which is where the round
asks from, and again from park, in case the seed is what matters.
"""
import asyncio, os, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from qc.layout import Cell
from qc.round import Clients, lifted, props_of, world_state
from viam.components.arm import Arm, JointPositions
from viam.components.generic import Generic
from viam.components.gripper import Gripper
from viam.proto.common import Pose, PoseInFrame
from viam.robot.client import RobotClient
from viam.services.motion import MotionClient

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")


async def try_pose(c, arm, pose, state, label):
    position, orientation = pose
    try:
        await c.motion.move(
            component_name=arm,
            destination=PoseInFrame(reference_frame="world", pose=Pose(
                x=position[0], y=position[1], z=position[2], **orientation)),
            world_state=state)
    except Exception as exc:  # noqa: BLE001
        print(f"    {label}: REFUSED\n      {str(exc)[:400]}")
        return None
    got = [round(v, 1) for v in (await c.arms[arm].get_joint_positions()).values]
    print(f"    {label}: reached {got}")
    return got


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
        poses = cell.present_poses(arm)
        rolls = [p for face, p in poses[:4]]
        free_face, free = poses[-1]
        park = cell.home_joints_deg[arm]
        print(f"  {arm}: free face {free_face} at "
              f"{[round(v) for v in free[0]]}")
        # From park, which is where the round is before anything else happens.
        await c.arms[arm].move_to_joint_positions(JointPositions(values=park))
        await try_pose(c, arm, free, state, "from park")
        # And from the last roll, which is where the round actually asks from.
        await c.arms[arm].move_to_joint_positions(JointPositions(values=park))
        landed = await try_pose(c, arm, rolls[-1], state, "the last roll first")
        if landed is not None:
            await try_pose(c, arm, free, state, "then the free face")
        print()

    await robot.close()


if __name__ == "__main__":
    asyncio.run(main())
