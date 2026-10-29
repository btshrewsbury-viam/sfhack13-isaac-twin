"""Which of the part's faces can the arm actually take it by?

Choosing a grip face has two halves and they are easy to confuse. `derive_mailbox.py`
answers the CUP's half - how much clear flat wall a face offers, how far the standoff is,
what stands proud of it - and at full scale it picks the -x side without hesitation: 86 mm
standoff against 229-252 for the ends, 481 mm of clear wall, nothing in the way.

The arm's half is a different question. A grip on -x needs the cup pointing +x, radially
away from the base, and this cell clamps every joint to +-180 so that one pose has one
solution. Measured, that pose produces zero IK solutions - so the face the cup wants is a
face the arm cannot present a tool to.

This asks the planner about each face in turn, at the approach and at the grip, so the
choice is made against both halves instead of one.
"""
import asyncio, os, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from qc.layout import Cell
from qc.round import Clients, lifted, props_of, world_state
from viam.components.arm import Arm
from viam.components.generic import Generic
from viam.components.gripper import Gripper
from viam.proto.common import Pose, PoseInFrame
from viam.robot.client import RobotClient
from viam.services.motion import MotionClient

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")


async def reachable(c, arm, pose, state) -> str:
    (x, y, z), orientation = pose
    try:
        await c.motion.move(
            component_name=arm,
            destination=PoseInFrame(reference_frame="world", pose=Pose(
                x=x, y=y, z=z, **orientation)),
            world_state=state)
        return "reached"
    except Exception as exc:  # noqa: BLE001
        text = str(exc)
        if "zero IK solutions" in text:
            return "no IK at all"
        if "did not settle" in text:
            return "reached but could not hold"
        return text[:60]


async def main() -> int:
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
    await asyncio.sleep(3.0)
    props = await props_of(c)
    state = world_state(props, cell=cell)
    faces = cell.grip_faces or {}
    print(f"  {'face':5s} {'standoff':>9} {'clear':>7} {'reach':>7}  approach 140 up      "
          f"at the grip")
    for side in ("-x", "+x", "+y", "-y"):
        pose = cell.side_grip(cell.part.centre_mm, side)
        got = faces.get(side, {})
        away = await reachable(c, "arm-a", lifted(pose, 140.0), state)
        near = await reachable(c, "arm-a", pose, state) if away == "reached" else "-"
        print(f"  {side:5s} {got.get('standoff_mm', 0):>9.0f} "
              f"{got.get('clear_mm', 0):>7.0f} "
              f"{cell.reach_mm('arm-a', pose[0]):>7.0f}  {away:<20} {near}")
    await robot.close()
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
