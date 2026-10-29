"""Ask whether each station has a solution on the arm's natural branch.

The taught program reaches every working pose with the arm folded back over its own
shoulder - arm-a picks from [550, -375], which is straight out in front of a base at
y = -380, with pan at -155 rather than the 0 the geometry asks for. It costs 185 degrees
to get onto that branch at the start of the round and 217 to come off it at the end, and
those are the two biggest single moves the cell makes.

What is not known from the program alone is WHY. The planner seeds from the current
configuration and scores by joint distance, so it would have taken the near solution if
there were one; something must rule it out. This drives the arm onto the natural branch
by hand and asks the planner for each station from there. Either it stays - and the
flip is the planner's choice, fixable by seeding it differently - or it flips anyway,
and the message says which joint the near solution would have needed.
"""
import asyncio, math, os, sys
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
JOINTS = ("pan", "lift", "elbow", "w1", "w2", "w3")


def facing(cell, arm, position):
    """Home's shape, panned to point at a station: the natural branch, by construction."""
    base = cell.arm_bases_mm[arm]
    pan = math.degrees(math.atan2(position[1] - base[1], position[0] - base[0]))
    return [pan] + list(cell.home_joints_deg[arm][1:])


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

    probes = [("pick approach", "arm-a", lifted(cell.pick("+y"), 140.0)),
              ("pick", "arm-a", cell.pick("+y")),
              ("handoff give", "arm-a", cell.handoff_grip("-y", cell.RELEASE_GAP_MM)),
              ("handoff take", "arm-b", cell.handoff_grip("+y"))]
    probes += [(f"present {face}", "arm-a", pose)
               for face, pose in cell.present_poses("arm-a")]
    probes += [(f"present-b {face}", "arm-b", pose)
               for face, pose in cell.present_poses("arm-b")]

    for name, arm, pose in probes:
        position, orientation = pose
        seed = facing(cell, arm, position)
        try:
            await c.arms[arm].move_to_joint_positions(JointPositions(values=seed))
        except Exception as exc:
            print(f"  {name:<24} {arm}  seed refused: {str(exc)[:70]}")
            continue
        try:
            await move(c, arm, pose, state)
        except Exception as exc:
            print(f"  {name:<24} {arm}  no solution: {str(exc)[:70]}")
            continue
        got = (await c.arms[arm].get_joint_positions()).values
        cost = max(abs(a - b) for a, b in zip(seed, got))
        flipped = "FLIPPED" if abs(got[0] - seed[0]) > 90 else "near   "
        print(f"  {name:<24} {arm}  {flipped}  seed pan {seed[0]:7.1f} -> "
              f"{got[0]:7.1f}  worst joint {cost:5.0f}  "
              f"{[round(v) for v in got]}")

    await robot.close()


DOWN = {"o_x": 0.0, "o_y": 0.0, "o_z": -1.0, "theta": 0.0}

if __name__ == "__main__":
    asyncio.run(main())
