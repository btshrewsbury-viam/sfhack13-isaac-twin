"""Just the door: seat the part by hand, hold it, and try to work the hinge.

The full sequence - feed, pick, carry, seat, hand over - takes about three minutes before
the door is even reachable, which is a poor loop to debug a hinge in. `place_prop` puts
the part straight into the fixture, so this iterates on the one thing in question.

The door is hinged along its bottom edge and swings down and out, so it is PUSHED open,
not pulled: the cup never has to seal on a moving panel.
"""
import asyncio, math, os, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from qc.layout import Cell, part_origin_offset_mm
from qc.round import (Clients, flange_of, lifted, move, part_pose, props_of,
                      world_state, PART_LABEL, STRAIGHT)
from door_check import door_angle
from viam.components.arm import Arm
from viam.components.generic import Generic
from viam.components.gripper import Gripper
from viam.robot.client import RobotClient
from viam.services.motion import MotionClient

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
# The MIDDLE of the panel. Higher up is a bigger lever and lands on `Bead_trim`, which
# runs round the door opening and belongs to the SHELL - so the push moves the whole part
# and the hinge never sees a moment (measured: door 0.0 at every push from 190 mm, while
# the part crept 30 mm). At 105 the cup is on `DoorPanel_exterior` and the door swings.
PANEL_TOP_ABOVE_HINGE_MM = 105.0
# The cup is tilted 45 degrees: into the panel AND downwards. Straight down is useless -
# on a shut door the panel is vertical, so a vertical force at its top edge acts through
# the 22 mm the hinge is inset and produces almost no torque (measured: 0.0 degrees at
# every push). Straight in is mechanically right and geometrically impossible: it puts
# the flange at y -447, beyond arm-a's own base, pointing back at itself. At 45 the
# horizontal component still has a 190 mm lever and the flange sits beside the arm.
PUSH_TILT_DEG = 45.0
# MEASURED by raycasting the seated part, not derived from the asset's bbox: the door's
# collision face stands 0.5 mm out from the hinge, flat all the way up. The 22 mm I had
# taken from the panel's bbox (hinge at asset x 236.2, panel out to 258.2) is the VISUAL
# thickness; the collider does not follow it. Standing off 22 mm meant the cup never
# touched the door at all - which is why every push read 0.0 degrees.
PANEL_OUT_MM = 0.5

# Turn the tool 90 degrees about its own axis before it goes near the door. The EPick's
# four cups sit at +-79.75 x +-40.65, so the array is 159.5 mm across one axis and the
# door panel is 159.7 mm wide: unturned, the outer cups overhang the panel by 24 mm each
# side onto the bead and the shell, and the gripper attaches to `/World/part/Body`. The
# panel is 215 mm TALL, so turned, the long span runs up it with 3 mm to spare and the
# short span crosses the width with 39 mm each side.
TOOL_ROLL_DEG = 90.0


async def hinge_mm(world):
    got = (await world.do_command({"command": "prim_poses",
                                   "paths": ["/World/part/Door"]}))["prims"]
    return tuple(float(v) for v in got["/World/part/Door"]["position_mm"])


def on_panel(hinge, degrees, tool_mm, gap_mm):
    """Flange pose to press DOWN on the door's top edge at this angle.

    Pushing the panel face-on is the obvious move and it does not work: the door faces
    -y, so a cup pointing into it sits beyond arm-a's own base pointing back at itself,
    and the arm stalls holding that pose with j0 against its effort limit.

    A door hinged along its BOTTOM opens just as well pressed down on its top edge, and
    that pose is straight out in front of the arm with the tool pointing at the floor.
    The edge swings out and down as the door falls, so the contact follows it.
    """
    t = math.radians(degrees)
    out = PANEL_OUT_MM * math.cos(t) + PANEL_TOP_ABOVE_HINGE_MM * math.sin(t)
    up = PANEL_TOP_ABOVE_HINGE_MM * math.cos(t) - PANEL_OUT_MM * math.sin(t)
    contact = (hinge[0], hinge[1] - out, hinge[2] + up)
    # Normal to the PANEL, turning with it - this is the direction that actually swings
    # the door. A fixed 45 degree tilt contacts on an edge and slides (0.0 degrees at
    # every push); straight down has no moment arm at all.
    nx, ny, nz = 0.0, math.cos(t), math.sin(t)
    back = tool_mm + gap_mm
    return ((contact[0] - nx * back, contact[1] - ny * back, contact[2] - nz * back),
            {"o_x": nx, "o_y": ny, "o_z": nz, "theta": TOOL_ROLL_DEG})


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
    await asyncio.sleep(3.0)
    for g in c.grippers.values():
        await g.open()

    centre = cell.resting_on(cell.fixture)
    offset = part_origin_offset_mm()
    origin = [centre[i] - offset[i] for i in range(3)]
    await c.world.do_command({"command": "place_prop", "name": "part",
                              "position_mm": [origin[0], origin[1], origin[2] + 5.0]})
    await asyncio.sleep(2.5)
    seated = await part_pose(c)
    print(f"seated {[round(v, 1) for v in seated['position_mm']]}   "
          f"door at rest {await door_angle(c.world):.1f} deg")

    props = await props_of(c)
    take = cell.side_grip(seated["position_mm"], cell.GRIP_SIDE)
    await move(c, "arm-b", lifted(take, 60.0), world_state(props, cell=cell))
    await move(c, "arm-b", take, world_state(props, cell=cell, ignore=[PART_LABEL]),
               STRAIGHT)
    await c.grippers["arm-b"].grab()
    await asyncio.sleep(1.0)
    held = (await c.grippers["arm-b"].is_holding_something())
    print(f"arm-b holding {held.is_holding_something} {held.meta.get('gripped')}   "
          f"door {await door_angle(c.world):.1f}")

    hinge = await hinge_mm(c.world)
    props = await props_of(c)
    # The part is an obstacle on the way IN and not on the contact move. Ignoring it
    # throughout is how arm-a got routed straight through the mailbox and stalled with
    # j0 against its effort limit and `moving=False` - blocked, not out-torqued.
    approach_state = world_state(props, cell=cell)
    state = world_state(props, cell=cell, ignore=[PART_LABEL])
    start = (await part_pose(c))["position_mm"]
    print(f"hinge {[round(v, 1) for v in hinge]}")
    # Which prim is at the contact? The shell wraps the door opening, and pushing the
    # shell moves the whole part while the hinge never sees a moment.
    probe = await c.world.do_command({"command": "raycast",
        "from_mm": [hinge[0], hinge[1] - 320.0, hinge[2] + PANEL_TOP_ABOVE_HINGE_MM],
        "along": [0, 1, 0], "distance_mm": 400.0})
    print(f"  contact prim: {probe['ray'].get('collider')}")
    # The door is PULLED open, not pushed. Its joint is authored `lower: 0.0, upper: 100`
    # - it only turns one way, and that way swings the panel OUTWARD in -y. Every push so
    # far was driving it into its own lower limit, which is why the angle never moved off
    # 0.0 however the contact was arranged.
    #
    # Pulling needs suction on the panel, and that is why the cup has to be at the
    # measured 0.5 mm face rather than the 22 mm the visual bbox suggests: standing off
    # 22 mm it never touched the door to grip it.
    await move(c, "arm-a", on_panel(hinge, 0.0, cell.tool_mm, 30.0), approach_state)
    # Landed FLUSH the gripper's own raycast starts inside the door's thin collider and
    # passes straight through it to the shell behind: `gripped ['/World/part/Body']` on a
    # cup sitting on the door. The 6 mm approach gap is exactly what that constant is for.
    # Re-read the hinge after the approach. The part shifts 5-10 mm the moment the cup
    # first touches it, and a gap measured before that is buried by the drift: the
    # gripper's ray then STARTS inside the door's collider, passes through it, and
    # attaches to the shell behind - `gripped ['/World/part/Body']` on a cup that is
    # visibly on the door.
    # Close the loop on the gap instead of guessing it. At 6 mm the ray started inside
    # the door and attached to the shell behind; at 10 mm it reached nothing at all. The
    # window is a few millimetres wide and the part drifts within it, so the distance is
    # MEASURED from where the cup actually ended up and the last step is sized to land
    # the cup exactly CUP_APPROACH_GAP_MM off the panel.
    hinge = await hinge_mm(c.world)
    await move(c, "arm-a", on_panel(hinge, 0.0, cell.tool_mm, 12.0), state, STRAIGHT)
    flange = await flange_of(c, "arm-a")
    cup = [flange[0], flange[1] + cell.tool_mm, flange[2]]
    look = await c.world.do_command({"command": "raycast", "from_mm": cup,
                                     "along": [0, 1, 0], "distance_mm": 40.0})
    ray = look["ray"]
    print(f"  cup at {[round(v, 1) for v in cup]} sees {ray.get('body')} "
          f"at {ray.get('distance_mm')} mm")
    if not ray["hit"] or "Door" not in str(ray.get("body")):
        raise RuntimeError(f"the cup is not looking at the door: {ray}")
    step = float(ray["distance_mm"]) - 6.0
    await move(c, "arm-a", ((flange[0], flange[1] + step, flange[2]),
                            {"o_x": 0.0, "o_y": 1.0, "o_z": 0.0, "theta": 0.0}),
               state, STRAIGHT)
    caught = await c.grippers["arm-a"].grab()
    await asyncio.sleep(0.5)
    on = await c.grippers["arm-a"].is_holding_something()
    print(f"  cup on the panel: {caught}  gripped {on.meta.get('gripped')}")
    for deg in (15.0, 30.0, 45.0, 60.0):
        await move(c, "arm-a", on_panel(hinge, deg, cell.tool_mm, 6.0), state)
        now = (await part_pose(c))["position_mm"]
        drift = max(abs(now[i] - start[i]) for i in range(3))
        print(f"  pull {deg:5.1f} -> door {await door_angle(c.world):6.1f}   "
              f"part moved {drift:5.1f} mm")
        if drift > 60.0:
            print("  STOP: the part is being dragged")
            break
    opened = await door_angle(c.world)
    # Let go and see what the door does on its own - the whole discriminator between
    # `ok` (springs shut) and `loose` (stays open).
    await c.grippers["arm-a"].open()
    await asyncio.sleep(0.5)
    await move(c, "arm-a", on_panel(hinge, 60.0, cell.tool_mm, 90.0), state)
    released = await door_angle(c.world)
    await asyncio.sleep(1.5)
    print(f"\nopened {opened:.1f}   at release {released:.1f}   "
          f"1.5 s later {await door_angle(c.world):.1f}")
    await robot.close()
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
