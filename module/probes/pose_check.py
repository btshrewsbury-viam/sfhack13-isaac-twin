"""Visit every station and report the arm configuration it lands in.

"The trajectories look weird" is a real observation and a useless bug report until it is
a number. This reports, per station: the pan angle away from the arm's own forward
direction, how far out the flange is, how high above the base it sits, and the joint
solution the planner chose - so a layout change can be judged rather than eyeballed.
"""
import asyncio, math, os, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from qc.layout import Cell, reach_budget
from qc.round import Clients, props_of, park, world_state, move
from viam.components.arm import Arm
from viam.components.generic import Generic
from viam.components.gripper import Gripper
from viam.robot.client import RobotClient
from viam.services.motion import MotionClient

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
JOINTS = ("pan", "lift", "elbow", "w1", "w2", "w3")


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
    for arm in ("arm-a", "arm-b"):
        await park(c, arm, props)

    print(f"  {'station':<22} {'arm':<6} {'out':>5} {'up':>5} {'pan':>6}  joints")
    for name, (arm, position) in cell.stations().items():
        base = cell.arm_bases_mm[arm]
        # the arm faces +x; pan is the angle of the station away from that
        pan = math.degrees(math.atan2(position[1] - base[1], position[0] - base[0]))
        out = math.dist((position[0], position[1]), (base[0], base[1]))
        up = position[2] - base[2]
        try:
            # every station's orientation comes from the layout, so re-derive it
            pose = next(p for n, p in _all_poses(cell) if n == name)
            await move(c, arm, pose, world_state(props, cell=cell))
            joints = [round(v) for v in (await c.arms[arm].get_joint_positions()).values]
            note = ""
            if abs(joints[2]) > 120: note += " ELBOW-FOLDED"
            if abs(pan) > 30: note += " ACROSS-BODY"
            if up > 350: note += " HIGH"
        except Exception as exc:
            joints, note = None, f"  UNREACHED: {str(exc)[:70]}"
        print(f"  {name:<22} {arm:<6} {out:5.0f} {up:5.0f} {pan:6.1f}  {joints}{note}")
    await robot.close()


def _all_poses(cell):
    yield "pick", cell.pick("+y")
    yield "handoff_give", cell.handoff_grip("-y", cell.RELEASE_GAP_MM)
    yield "handoff_take", cell.handoff_grip("+y")
    for tray in cell.trays:
        yield f"place_{tray}", cell.place(tray, "+y", cell.RELEASE_GAP_MM)
    for arm in cell.arm_bases_mm:
        for face, pose in cell.present_poses(arm):
            yield f"present_{arm}_{face}", pose


asyncio.run(main())
