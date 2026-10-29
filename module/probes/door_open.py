"""T5: arm-a opens and closes the door while arm-b holds the part in the fixture.

The door is hinged along its BOTTOM edge, 0-100 degrees, with the handle at the top - a
rural mailbox door that swings down and out. So it cannot be pushed open: pushing on the
outside of a door that opens outwards closes it. It has to be PULLED, which is what a
vacuum cup is for, and the cup then has to follow the hinge rather than travel in a
straight line.

Every number here is derived from the part's live pose rather than stated: the hinge sits
on the door's bottom edge, the handle 193 mm above it, and the cup stays normal to the
panel all the way round - so as the door falls open the cup's own direction rotates with
it, from +y at the shut position towards +z at 90 degrees.

Accept: the measured angle separates all four conditions - `ok` opens past 60 and returns
under 10, `stuck` stays under 10, `binding` stops near 35, `loose` opens past 60 and
stays there. Two looks 1.5 s apart, because `ok` and `loose` differ only in the return.
"""
import asyncio, math, os, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from qc.layout import Cell
from qc.round import (Clients, feed, lifted, measure_hold, move, part_pose, pick,
                      props_of, Trace, world_state, PART_LABEL, STRAIGHT)
from door_check import door_angle
from viam.components.arm import Arm
from viam.components.generic import Generic
from viam.components.gripper import Gripper
from viam.robot.client import RobotClient
from viam.services.motion import MotionClient

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
# The cup goes on the middle of the PANEL, not up by the handle. `DoorPanel_exterior`
# spans its own z 0.3..215.7 with the hinge 3.5 mm up from the part's underside, so the
# panel's centre is about 105 mm above the hinge. Aimed at the handle height instead, the
# cup lands on the top trim ring - which belongs to the shell, not the door - and the
# gripper reported `gripped: ['/World/part/Body']` and hauled the whole mailbox 777 mm.
HANDLE_ABOVE_HINGE_MM = 105.0
# The hinge sits in the MIDDLE of the panel's thickness, not on its outer face: the door
# spans asset x 218.7..258.2 and the joint is at 236.2. So a cup placed directly above
# the hinge is 22 mm INSIDE the part - it lands on the shell instead of the panel, and
# the grab then drags the whole mailbox (measured: door still at 0.0 degrees while the
# part travelled 598 mm off the fixture).
PANEL_OUT_MM = 22.0
OPEN_TO_DEG = 75.0
STEPS = 5


async def hinge_mm(world) -> tuple:
    """Where the hinge actually is, read from the stage.

    The revolute joint is authored with `localPos1 = (0, 0, 0)` on the Door body, so the
    Door prim's own origin IS the hinge - no arithmetic, and it follows the part wherever
    it is seated. Deriving it instead, as the part's -y extreme, put it 22 mm too far out:
    with the cup rigidly attached to the panel, a 22 mm error in the pivot levers the
    whole mailbox out of the fixture, which is exactly what happened.
    """
    got = (await world.do_command({"command": "prim_poses",
                                   "paths": ["/World/part/Door"]}))["prims"]
    return tuple(float(v) for v in got["/World/part/Door"]["position_mm"])


def door_pose(hinge, degrees: float, tool_mm: float, gap_mm: float):
    """Where the flange has to be for the cup to sit on the door at this angle.

    The contact point swings about the hinge on the door's bottom edge; the cup stays
    normal to the panel, so its direction turns with the door.
    """
    t = math.radians(degrees)
    # In the door's own frame the contact sits PANEL_OUT_MM along the outward normal and
    # HANDLE_ABOVE_HINGE_MM up from the hinge; both turn with the door.
    out = PANEL_OUT_MM * math.cos(t) + HANDLE_ABOVE_HINGE_MM * math.sin(t)
    up = HANDLE_ABOVE_HINGE_MM * math.cos(t) - PANEL_OUT_MM * math.sin(t)
    contact = (hinge[0], hinge[1] - out, hinge[2] + up)
    # The cup points INTO the panel: +y when shut, rotating towards +z as it falls open.
    nx, ny, nz = 0.0, math.cos(t), math.sin(t)
    back = tool_mm + gap_mm
    return ((contact[0] - nx * back, contact[1] - ny * back, contact[2] - nz * back),
            {"o_x": nx, "o_y": ny, "o_z": nz, "theta": 0.0})


async def main() -> int:
    robot = await RobotClient.at_address(FQDN, RobotClient.Options.with_api_key(
        api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"]))
    c = Clients(motion=MotionClient.from_robot(robot, "builtin"),
                world=Generic.from_robot(robot, "sim-world"),
                arms={n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b")},
                grippers={"arm-a": Gripper.from_robot(robot, "suction-a"),
                          "arm-b": Gripper.from_robot(robot, "suction-b")})
    cell = Cell.from_fragment()
    await c.world.do_command({"command": "reset"})
    await asyncio.sleep(3.5)
    for g in c.grippers.values():
        await g.open()
    await feed(c, cell)
    props = await props_of(c)
    held = await pick(c, cell, props, Trace(), arm="arm-a", side=cell.GRIP_SIDE)

    # Seat it, let go, and hand the hold to arm-b.
    seat = cell.fixture_grip(cell.GRIP_SIDE, cell.RELEASE_GAP_MM)
    await move(c, "arm-a", lifted(seat, 60.0), world_state(props, cell=cell, held=held))
    await move(c, "arm-a", seat, world_state(props, cell=cell, held=held), STRAIGHT)
    await asyncio.sleep(1.0)
    await c.grippers["arm-a"].open()
    await asyncio.sleep(1.0)
    props = await props_of(c)
    (sx, sy, sz), so = seat
    await move(c, "arm-a", ((sx - 140.0, sy, sz), so),
               world_state(props, cell=cell, ignore=[PART_LABEL]), STRAIGHT)
    # And get arm-a fully out of the way BEFORE arm-b comes in for the same face: left
    # parked at the fixture its tool is exactly where arm-b's wrist needs to be, and the
    # planner refuses on a robot-to-robot violation rather than anything to do with the
    # part.
    await move(c, "arm-a", lifted(((sx - 260.0, sy, sz), so), 160.0),
               world_state(props, cell=cell))
    seated = await part_pose(c)
    take = cell.side_grip(seated["position_mm"], cell.GRIP_SIDE)
    await move(c, "arm-b", lifted(take, 60.0), world_state(props, cell=cell))
    await move(c, "arm-b", take, world_state(props, cell=cell, ignore=[PART_LABEL]),
               STRAIGHT)
    await c.grippers["arm-b"].grab()
    await asyncio.sleep(1.0)
    print(f"arm-b holding: "
          f"{(await c.grippers['arm-b'].is_holding_something()).is_holding_something}")
    centre = (await part_pose(c))["position_mm"]
    print(f"part seated at {[round(v, 1) for v in centre]}   door angle "
          f"{await door_angle(c.world):.1f} deg")

    # arm-a takes the door panel and swings it open along the hinge's arc.
    props = await props_of(c)
    state = world_state(props, cell=cell, ignore=[PART_LABEL])
    hinge = await hinge_mm(c.world)
    print(f"hinge at {[round(v, 1) for v in hinge]}")
    # No grab. A bottom-hinged door that swings down and out is OPENED by pushing its
    # panel along the hinge's arc - the cup never has to seal on a moving surface. It was
    # worth finding out: aimed at the handle the cup sealed on the SHELL and hauled the
    # whole mailbox 777 mm, and aimed at the panel it sealed on nothing at all and pushed
    # the door open anyway.
    #
    # And the push is CLOSED-LOOP, because the door does not wait for the arm. Commanded
    # 15 degrees of arc it had already fallen to 77, and the next commanded step drove
    # the cup through a panel that was no longer there - into the part, which moved
    # 262 mm. So the cup is always sent just behind where the door actually is.
    await move(c, "arm-a", door_pose(hinge, 0.0, cell.tool_mm, 70.0), state)
    angle = await door_angle(c.world)
    for _ in range(12):
        if angle >= OPEN_TO_DEG:
            break
        await move(c, "arm-a",
                   door_pose(hinge, min(angle + 8.0, OPEN_TO_DEG), cell.tool_mm, 6.0),
                   state)
        moved = (await part_pose(c))["position_mm"]
        drift = max(abs(moved[i] - centre[i]) for i in range(3))
        angle = await door_angle(c.world)
        print(f"  pushing -> door {angle:6.1f}   part moved {drift:5.1f} mm")
        if drift > 60.0:
            raise RuntimeError(
                f"the door is levering the part out of the fixture: {drift:.0f} mm")
    opened = angle
    # Back the cup off the panel and watch what the door does on its own. This is the
    # whole discriminator: `ok` and `loose` both open, and differ only in the return.
    await move(c, "arm-a", door_pose(hinge, 0.0, cell.tool_mm, 220.0), state)
    released = await door_angle(c.world)
    await asyncio.sleep(1.5)
    settled = await door_angle(c.world)
    print(f"\nopened to {opened:.1f}, at release {released:.1f}, "
          f"1.5 s later {settled:.1f}")
    await robot.close()
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
