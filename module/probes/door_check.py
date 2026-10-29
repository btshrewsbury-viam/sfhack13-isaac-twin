"""Does the mailbox's door still swing, and can the cell tell a good one from a bad one?

The asset carries a MECHANICAL defect as well as paint: `door_condition` overrides the
hinge's drive stiffness, damping and upper limit, so the fault is not something a camera
sees on a still part - it is how the door behaves when it is made to move. From the
asset's own notes:

    ok       opens fully and springs closed    85 -> 0 degrees
    stuck    barely moves                      3
    binding  crawls, stops early               9, and barely returns
    loose    no return spring                  85-92, and stays open

Two things make this awkward, and both come from the asset's author. At rest a LOOSE door
looks shut, because its centre of mass sits just behind the hinge line - so a static pose
scores loose as stuck. And `ok` and `loose` differ only in whether the door springs back,
which means the measurement is a rate over two looks, never one.

This probe is the first question before any of that: the cell strips the asset's
articulation root so the suction cup can grip it, which leaves the hinge as an ordinary
jointed pair rather than an articulation DOF. If that broke the hinge there is no door
check to build, so this tilts the part and reports the angle for each condition.
"""
import asyncio, math, os, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from qc.layout import Cell
from qc.round import (Clients, feed, lifted, lift_by_shoulder, measure_hold, move,
                      part_pose, props_of, world_state)
from viam.components.arm import Arm
from viam.components.camera import Camera
from viam.components.generic import Generic
from viam.components.gripper import Gripper
from viam.robot.client import RobotClient
from viam.services.motion import MotionClient

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
CONDITIONS = ("ok", "stuck", "binding", "loose")


def quat_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


async def door_angle(world) -> float:
    """The door's rotation relative to the shell, in degrees about the hinge axis.

    Relative, so it does not matter which way the arm is holding the part - which is the
    whole point, because the door is only made to move by tilting the part.
    """
    got = (await world.do_command({"command": "prim_poses", "paths": [
        "/World/part/Body", "/World/part/Door"]}))["prims"]
    body = got.get("/World/part/Body")
    door = got.get("/World/part/Door")
    if not body or not door:
        raise RuntimeError("the part has no Body/Door to compare")
    relative = quat_mul(quat_conj(tuple(body["orientation_wxyz"])),
                        tuple(door["orientation_wxyz"]))
    # A quaternion and its negation are the SAME rotation, and the physics backend returns
    # either. Taking the angle without normalising the sign turns a shut door into -360 and
    # a door at its 35 degree stop into -325, which is how this first read looked.
    if relative[0] < 0.0:
        relative = tuple(-v for v in relative)
    # The hinge turns about the body's Y, so the signed angle is read off that component.
    return math.degrees(2.0 * math.atan2(relative[2], relative[0]))


async def main() -> int:
    robot = await RobotClient.at_address(FQDN, RobotClient.Options.with_api_key(
        api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"]))
    world = Generic.from_robot(robot, "sim-world")
    clients = Clients(
        motion=MotionClient.from_robot(robot, "builtin"), world=world,
        arms={n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b")},
        grippers={"arm-a": Gripper.from_robot(robot, "suction-a"),
                  "arm-b": Gripper.from_robot(robot, "suction-b")})
    cell = Cell.from_fragment()
    print(f"  {'condition':10s} {'expected':>9} {'at rest':>9} {'tilted':>9} "
          f"{'+1.5s':>9}  verdict")
    try:
        for condition in CONDITIONS:
            # Choose the variant BEFORE the reset. A variant switch recomposes the prim
            # and invalidates the physics views, so doing it after leaves the next arm
            # move failing with "Failed to get DOF positions from backend" - the reset is
            # what rebuilds them.
            await world.do_command({
                "command": "set_variants", "prop": "part",
                "variants": {"door_condition": condition}})
            await world.do_command({"command": "reset"})
            await asyncio.sleep(3.0)
            await clients.grippers["arm-a"].open()
            truth = (await world.do_command({
                "command": "prop_attributes", "prop": "part",
                "names": ["mailbox:expectedDoorPass"]}))["attributes"]
            at_rest = await door_angle(world)

            # The part spawns upstream now and the belt brings it to the pick point, so
            # there is nothing to grip until it has been fed. And it is gripped by the
            # `-x` face the measurements chose, not the cantilevered `+y` end.
            await feed(clients, cell)
            props = await props_of(clients)
            grip = cell.side_grip((await part_pose(clients))["position_mm"],
                                  cell.GRIP_SIDE)
            await move(clients, "arm-a", lifted(grip, 140.0),
                       world_state(props, cell=cell))
            await move(clients, "arm-a", grip,
                       world_state(props, cell=cell, ignore=["part"]))
            await clients.grippers["arm-a"].grab()
            await asyncio.sleep(0.6)
            held = await measure_hold(clients, "arm-a", cell)
            await lift_by_shoulder(clients, "arm-a", degrees=16.0)

            # Tilt the part so gravity acts on the door, which is what makes a loose door
            # tell itself apart from a stuck one.
            centre = (await part_pose(clients))["position_mm"]
            tipped = ((centre[0], centre[1], centre[2] + 120.0),
                      {"o_x": 0.0, "o_y": -1.0, "o_z": -0.6, "theta": 25.0})
            try:
                await move(clients, "arm-a", tipped,
                           world_state(props, cell=cell, held=held))
            except Exception as exc:  # noqa: BLE001
                print(f"  {condition:10s} {str(truth.get('mailbox:expectedDoorPass')):>9} "
                      f"{at_rest:>9.1f}  tilt refused: {str(exc)[:60]}")
                continue
            await asyncio.sleep(0.5)
            tilted = await door_angle(world)
            await asyncio.sleep(1.5)
            later = await door_angle(world)
            swung = max(abs(tilted), abs(later))
            sprang = abs(later) < abs(tilted) - 5.0
            verdict = ("shut" if swung < 6 else
                       "opens and springs back" if sprang else
                       "opens and stays" if swung > 60 else "crawls")
            print(f"  {condition:10s} {str(truth.get('mailbox:expectedDoorPass')):>9} "
                  f"{at_rest:>9.1f} {tilted:>9.1f} {later:>9.1f}  {verdict}")
            await clients.grippers["arm-a"].open()
        return 0
    finally:
        await robot.close()


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
