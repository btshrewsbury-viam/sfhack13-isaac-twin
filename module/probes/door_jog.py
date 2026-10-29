"""Work the door in JOINT space, because a pose cannot hold the wrist's roll.

The hook only works with its tab pointing at the part, which leaves the stem 82 mm clear
of the mailbox. `aim` can find and verify that roll at the park pose - and the tab turns
over on the way in, arriving pointing straight up. An orientation vector does not pin the
roll: the same OV is satisfied by more than one IK branch, `HOLDING` constrains deviation
ALONG a path whose ends the planner may solve in the other branch, and creeping in 20 mm
bites with a check after each one flips just the same.

So nothing here asks for a pose. Once the arm is in the configuration that works, it is
jogged by JOINTS, using a Jacobian measured on the spot: nudge each joint, see where the
hook's finger actually went, and solve for the joint move that takes it where it is
needed. The planner never gets to choose a branch, so the roll cannot change.

The minimum-norm solution is used, which also keeps the wrist from wandering off to
satisfy a position three joints could have reached.
"""
import asyncio, math, os, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np
from qc.layout import Cell, part_origin_offset_mm
from qc.round import (Clients, lifted, move, part_pose, props_of, world_state,
                      PART_LABEL, STRAIGHT)
from door_actuate import hinge_mm
from door_check import door_angle
from door_hook import FINGER, TOOL_ACROSS, PREFERRED_ROLL_DEG, aim, finger_mm, finger_offset
from viam.components.arm import Arm, JointPositions
from viam.components.generic import Generic
from viam.components.gripper import Gripper
from viam.robot.client import RobotClient
from viam.services.motion import MotionClient

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
HANDLE_PROUD_MM = 22.0
HANDLE_TOP_ABOVE_HINGE_MM = 201.0
NUDGE_DEG = 2.5          # how far each joint is moved to measure its effect
MAX_STEP_DEG = 6.0       # and how far any joint may move in one correction
# How hard the solver works to keep the tab pointing where it started, in millimetres of
# equivalent error per unit of direction. High enough that the roll is held, low enough
# that it does not fight the position it is trying to reach.
DIRECTION_WEIGHT = 250.0


async def joints(arm) -> np.ndarray:
    got = await arm.get_joint_positions()
    return np.array([float(v) for v in got.values], dtype=float)


async def go(arm, q: np.ndarray):
    """Command joints, tolerating a near-miss.

    The arm reports a settle failure when a joint stops a degree or two short with a
    wrist drive at its 28 N.m limit. For a CLOSED-LOOP jog that is not a failure: the
    next measurement sees where the arm actually got to and the correction accounts for
    it. Treating it as fatal throws away a controller that is already converging.
    """
    try:
        await arm.move_to_joint_positions(JointPositions(values=[float(v) for v in q]))
    except Exception as exc:  # noqa: BLE001
        if "did not settle" not in str(exc):
            raise


async def task_of(c):
    """The six numbers the jog controls: where the tab is, and which way it points.

    Position alone is not enough. Driven on position only the solver is free to spend the
    wrist, and the tab rotated from pointing at the part to 30 degrees off it while the
    finger tracked its target to half a millimetre. The tab's DIRECTION is part of the
    task, not a by-product of it.
    """
    offset, _, finger = await finger_offset(c)
    direction = np.array(offset, dtype=float)
    direction /= (np.linalg.norm(direction) or 1.0)
    return np.concatenate([np.array(finger, dtype=float), direction * DIRECTION_WEIGHT])


async def jacobian(c, arm, q0, task0):
    """How the task moves per degree of each joint, measured rather than modelled."""
    cols = []
    for i in range(len(q0)):
        q = q0.copy()
        q[i] += NUDGE_DEG
        await go(arm, q)
        await asyncio.sleep(0.15)
        cols.append((await task_of(c) - task0) / NUDGE_DEG)
    await go(arm, q0)
    await asyncio.sleep(0.2)
    return np.array(cols).T          # 6 x n


async def jog_to(c, arm, target, keep, tol_mm=3.0, rounds=6):
    """Drive the finger to `target` by joints alone, holding the tab's direction."""
    want = np.concatenate([np.array(target, dtype=float),
                           np.array(keep, dtype=float) * DIRECTION_WEIGHT])
    for _ in range(rounds):
        q0 = await joints(arm)
        task0 = await task_of(c)
        here = task0[:3]
        error = want - task0
        if np.linalg.norm(error[:3]) < tol_mm:
            return here
        J = await jacobian(c, arm, q0, task0)
        # Minimum-norm joint move for the wanted finger move.
        dq, *_ = np.linalg.lstsq(J, error, rcond=None)
        big = np.max(np.abs(dq))
        if big > MAX_STEP_DEG:
            dq *= MAX_STEP_DEG / big
        await go(arm, q0 + dq)
        await asyncio.sleep(0.25)
    return np.array(await finger_mm(c.world))


async def main() -> int:
    robot = await RobotClient.at_address(FQDN, RobotClient.Options.with_api_key(
        api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"]))
    c = Clients(motion=MotionClient.from_robot(robot, "builtin"),
                world=Generic.from_robot(robot, "sim-world"),
                arms={n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b")},
                grippers={"arm-a": Gripper.from_robot(robot, "suction-a"),
                          "arm-b": Gripper.from_robot(robot, "suction-b")})
    cell = Cell.from_fragment()
    arm = c.arms["arm-a"]
    await c.world.do_command({"command": "reset"})
    await asyncio.sleep(3.0)
    for g in c.grippers.values():
        await g.open()
    centre = cell.resting_on(cell.fixture)
    off = part_origin_offset_mm()
    origin = [centre[i] - off[i] for i in range(3)]
    await c.world.do_command({"command": "place_prop", "name": "part",
                              "position_mm": [origin[0], origin[1], origin[2] + 5.0]})
    await asyncio.sleep(2.5)

    seated = await part_pose(c)
    props = await props_of(c)
    take = cell.side_grip(seated["position_mm"], cell.GRIP_SIDE)
    await move(c, "arm-b", lifted(take, 60.0), world_state(props, cell=cell))
    await move(c, "arm-b", take, world_state(props, cell=cell, ignore=[PART_LABEL]),
               STRAIGHT)
    await c.grippers["arm-b"].grab()
    await asyncio.sleep(1.0)
    print(f"arm-b holding "
          f"{(await c.grippers['arm-b'].is_holding_something()).is_holding_something}"
          f"   door {await door_angle(c.world):.1f}")

    hinge = await hinge_mm(c.world)
    props = await props_of(c)
    approach = world_state(props, cell=cell)
    state = world_state(props, cell=cell, ignore=[PART_LABEL])
    start = (await part_pose(c))["position_mm"]
    reach = (cell.hook or {}).get("reach_mm", 150.0)

    # The ONE pose asked of the planner: park, in the roll that points the tab at the
    # part. Everything after this is joints.
    # Just clear of the handle's top face, not 30 mm above it: the first press bite then
    # starts ON the handle instead of driving 38 mm down THROUGH it, which opened the
    # door 5 degrees and shoved the mailbox 109 mm in one move.
    above = [hinge[0], hinge[1] - HANDLE_PROUD_MM / 2.0,
             hinge[2] + HANDLE_TOP_ABOVE_HINGE_MM + 6.0]
    park = (above[0] - reach - 60.0, above[1], above[2] + 40.0)
    # Tab DOWN. Pointing it at the part would put the stem 60 mm clear in -y, but no
    # reachable roll presents that here - measured, this park pose offers -y, +z and -z.
    # Down is the one that works anyway: the stem then sits 60 mm ABOVE the tab, over the
    # top of a part whose own top is 827, so it clears the mailbox entirely instead of
    # grazing the door face. Tab up (+z) puts the stem at the door's own y and ploughs
    # into it - measured, the mailbox went 523 mm.
    pose, _ = await aim(c, approach, park, [0.0, 0.0, -1.0])
    offset, _, _ = await finger_offset(c)
    print(f"parked; tab {[round(v, 1) for v in offset]}")

    hold = np.array(offset, dtype=float)
    hold /= (np.linalg.norm(hold) or 1.0)
    got = await jog_to(c, arm, above, hold)
    offset, _, _ = await finger_offset(c)
    print(f"jogged above the handle: {[round(v, 1) for v in got]} "
          f"(wanted {[round(v, 1) for v in above]})  tab {[round(v, 1) for v in offset]}")
    turned = math.degrees(math.acos(max(-1.0, min(1.0, float(
        np.dot(np.array(offset) / np.linalg.norm(offset), hold))))))
    print(f"  tab held to within {turned:.1f} deg of where it started")

    print("pressing on the handle's top face:")
    for step in range(1, 24):
        angle = await door_angle(c.world)
        a = math.radians(angle)
        top = [above[0],
               hinge[1] - HANDLE_PROUD_MM * math.cos(a)
               - HANDLE_TOP_ABOVE_HINGE_MM * math.sin(a),
               hinge[2] + HANDLE_TOP_ABOVE_HINGE_MM * math.cos(a)
               - HANDLE_PROUD_MM * math.sin(a)]
        got = await jog_to(c, arm, [top[0], top[1] + HANDLE_PROUD_MM / 2.0,
                                    top[2] - 3.0 * step], hold, tol_mm=3.0, rounds=3)
        now = (await part_pose(c))["position_mm"]
        drift = max(abs(now[i] - start[i]) for i in range(3))
        print(f"  step {step:2d} -> door {await door_angle(c.world):6.1f} deg   "
              f"part moved {drift:5.1f} mm")
        if drift > 35.0:
            print("  STOP: the mailbox is being dragged")
            break
        if await door_angle(c.world) > 70.0:
            break
    opened = await door_angle(c.world)
    await jog_to(c, arm, [above[0], above[1], above[2] + 90.0], hold, tol_mm=8.0,
                 rounds=3)
    released = await door_angle(c.world)
    await asyncio.sleep(1.5)
    print(f"\nopened {opened:.1f}   at release {released:.1f}   "
          f"1.5 s later {await door_angle(c.world):.1f}")
    await robot.close()
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
