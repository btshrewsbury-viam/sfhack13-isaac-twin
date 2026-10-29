"""Open the mailbox door with the hook on arm-a's tool plate.

The cup cannot do this job. The door is bottom-hinged and opens OUTWARD only, so it must
be pulled, and a suction seal on a panel whose collider stands 0.5 mm proud of the shell
behind it is not something to build a course on.

The door does carry a feature made for pulling: `Door/Collision/Handle`, a separate
collider standing 22 mm proud of the panel, 200 mm up from the hinge. The hook's finger
goes under it and pulls.

The finger's world position is READ from the stage rather than derived from the flange
pose and a roll. Where a tool's finger ends up depends on the wrist's roll about its own
axis, which is the part of a Viam orientation vector this project already has a note
about not behaving like a plain angle - so it is measured and corrected, not computed.
"""
import asyncio, math, os, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from qc.layout import Cell, part_origin_offset_mm
from qc.round import (Clients, lifted, move, part_pose, props_of, world_state,
                      HOLDING, PART_LABEL, STRAIGHT)
from door_check import door_angle
from viam.components.arm import Arm
from viam.components.generic import Generic
from viam.components.gripper import Gripper
from viam.robot.client import RobotClient
from viam.services.motion import MotionClient

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
FINGER = "/World/arm_a/wrist_3_link/flange/arm_a_hook/finger"
# Measured by raycasting the seated part: the handle is a SOLID lug spanning z+192 to
# z+201 above the hinge and standing 22 mm proud, with NO gap behind it - a ray started
# inside its depth hits it at 0.0 mm. So it cannot be hooked. It can be PRESSED: a
# downward force on its top face acts 22 mm out from the hinge line, which is torque in
# the opening direction and 44x the arm the flat panel offers (0.5 mm).
HANDLE_ABOVE_HINGE_MM = 196.0
HANDLE_TOP_ABOVE_HINGE_MM = 201.0
HANDLE_PROUD_MM = 22.0             # and stands this far off the panel
# Reach ACROSS the door end from +x rather than facing it from -y.
#
# Facing it works and does not finish: the door swings outward in -y, away from arm-a,
# so following it walks the flange behind the arm's own base. Measured, the hook opened
# the door to 6.9 degrees and then the next arc point wanted the flange at y -458, 78 mm
# behind a base at -380, and j0 stalled against its effort limit.
#
# From the side the arm never has to retreat: the flange sits out at x ~670 and the whole
# swing is a translation in -y and -z in front of it.
# +x, which is arm-a's OWN side of the mailbox. Reaching across from +x needs 919 mm and
# no wrist roll could make it; from -x the flange sits at about x 370 and the whole swing
# stays inside 600 mm.
TOOL_ACROSS = {"o_x": 1.0, "o_y": 0.0, "o_z": 0.0, "theta": 0.0}
TOOL_AT_DOOR = TOOL_ACROSS


async def finger_mm(world):
    got = (await world.do_command({"command": "prim_poses",
                                   "paths": [FINGER]}))["prims"]
    return [float(v) for v in got[FINGER]["position_mm"]]


async def finger_offset(c):
    """Where the finger sits relative to the flange, in world terms, right now.

    Measured rather than derived. The finger's direction depends on the wrist's roll
    about its own axis, and that is the part of a Viam orientation vector this project
    already has a note about not behaving like a plain angle - so the roll is CHOSEN by
    trying the four quarter turns and keeping whichever actually points the finger where
    it is needed.
    """
    from qc.round import flange_of
    flange = await flange_of(c, "arm-a")
    finger = await finger_mm(c.world)
    return [finger[i] - flange[i] for i in range(3)], flange, finger


# Measured: at the park pose this roll points the tab along +y, into the part, which
# leaves the stem 82 mm clear of it in -y. Tried first so the search is skipped when it
# still holds - four trial moves at ~16 s each is a minute of every run.
PREFERRED_ROLL_DEG = 90.0


async def aim(c, state, at, want):
    """Pick the wrist roll that points the finger along `want`, and report it."""
    best = None
    for theta in (PREFERRED_ROLL_DEG, 0.0, 180.0, 270.0):
        pose = {**TOOL_ACROSS, "theta": theta}
        try:
            await move(c, "arm-a", (at, pose), state)
        except Exception:  # noqa: BLE001 - a roll the arm cannot make is not a candidate
            continue
        offset, _, _ = await finger_offset(c)
        norm = math.sqrt(sum(v * v for v in offset)) or 1.0
        score = sum(offset[i] / norm * want[i] for i in range(3))
        print(f"  roll {theta:5.1f}: finger offset "
              f"{[round(v, 1) for v in offset]}  alignment {score:+.2f}")
        if best is None or score > best[0]:
            best = (score, theta, offset)
        if theta == PREFERRED_ROLL_DEG and score > 0.4:
            break
    if best is None:
        raise RuntimeError("no wrist roll put the hook anywhere reachable")
    # Go BACK to the winner. Trying each roll in turn leaves the arm standing at the LAST
    # one tried, not the best one - so everything downstream measured a tab pointing the
    # opposite way from the one that was chosen, and corrected 120 mm in the wrong
    # direction. The planner was never at fault here.
    chosen = {**TOOL_ACROSS, "theta": best[1]}
    await move(c, "arm-a", (at, chosen), state)
    settled, _, _ = await finger_offset(c)
    print(f"  -> using roll {best[1]:.0f} (alignment {best[0]:+.2f}); "
          f"tab now {[round(v, 1) for v in settled]}")
    return chosen, settled


async def creep(c, target, pose, state, bite_mm=20.0, tries=12):
    """Walk the finger to `target` in small bites, checking the tab has not turned over.

    One large move lets the planner land in a different IK branch: the same orientation
    vector is satisfied by more than one wrist roll, and `theta` does not pin it. HOLDING
    does not help either, because it constrains deviation ALONG a path whose two ends the
    planner is free to solve in the other branch. Measured: the tab left the park pose
    pointing at the part and arrived pointing straight up.

    Small bites keep each solve near the last one, and the tab is re-measured after every
    one so a flip is caught where it happens rather than three moves later.
    """
    from qc.round import flange_of
    for _ in range(tries):
        here = await finger_mm(c.world)
        error = [target[i] - here[i] for i in range(3)]
        span = math.sqrt(sum(v * v for v in error))
        if span < 4.0:
            return here
        scale = min(1.0, bite_mm / span)
        flange = await flange_of(c, "arm-a")
        await move(c, "arm-a",
                   (tuple(flange[i] + error[i] * scale for i in range(3)), pose),
                   state, HOLDING)
        offset, _, _ = await finger_offset(c)
        if offset[1] < 40.0:
            raise RuntimeError(
                f"the tab turned over mid-approach: it now points "
                f"{[round(v, 1) for v in offset]} after a {span * scale:.0f} mm bite")
    return await finger_mm(c.world)


async def put_finger(c, target, pose, state, tries=3):
    """Move until the hook's finger is at `target`, correcting from what it measures.

    Constrained with HOLDING. Without it the planner is free to take a different IK
    branch between one move and the next, and the tab silently turns over: `aim` measured
    it pointing -z at the park pose and it was pointing +z by the engage pose, so the
    correction drove the flange 120 mm the wrong way - it asked for z 721 where the pose
    needed 886. An orientation constraint is what stops a tool rolling over mid-path, and
    this cell already uses it for exactly that when carrying the part.
    """
    from qc.round import flange_of
    for _ in range(tries):
        here = await finger_mm(c.world)
        error = [target[i] - here[i] for i in range(3)]
        if max(abs(v) for v in error) < 4.0:
            return here
        flange = await flange_of(c, "arm-a")
        await move(c, "arm-a",
                   (tuple(flange[i] + error[i] for i in range(3)), pose), state, HOLDING)
    return await finger_mm(c.world)


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
    props = await props_of(c)
    take = cell.side_grip(seated["position_mm"], cell.GRIP_SIDE)
    await move(c, "arm-b", lifted(take, 60.0), world_state(props, cell=cell))
    await move(c, "arm-b", take, world_state(props, cell=cell, ignore=[PART_LABEL]),
               STRAIGHT)
    await c.grippers["arm-b"].grab()
    await asyncio.sleep(1.0)
    print(f"arm-b holding "
          f"{(await c.grippers['arm-b'].is_holding_something()).is_holding_something}"
          f"   door {await door_angle(c.world):.1f} deg")

    hinge = (await c.world.do_command({"command": "prim_poses",
        "paths": ["/World/part/Door"]}))["prims"]["/World/part/Door"]["position_mm"]
    hinge = [float(v) for v in hinge]
    props = await props_of(c)
    approach = world_state(props, cell=cell)
    state = world_state(props, cell=cell, ignore=[PART_LABEL])
    start = (await part_pose(c))["position_mm"]

    # Under the handle: out in the middle of the 22 mm it stands proud, and a little
    # below it, so the finger comes up behind the bar rather than in front of it.
    # AT the handle's height, not below it. Parked 30 mm low the finger sits in free
    # air in front of the panel with nothing to catch, and the pull finds nothing.
    arm_mm = HANDLE_ABOVE_HINGE_MM
    # Above the handle's top face, in the middle of the 22 mm it stands proud.
    under = [hinge[0], hinge[1] - HANDLE_PROUD_MM / 2.0,
             hinge[2] + HANDLE_TOP_ABOVE_HINGE_MM + 25.0]
    reach = (cell.hook or {}).get("reach_mm", 150.0)
    # Stand off to +x, clear of the mailbox, and find the roll that points the finger
    # INTO the door - the direction it has to travel to get under the handle.
    park = (under[0] - reach - 60.0, under[1], under[2] + 40.0)
    # Aim the tab DOWNWARD, not at the part.
    #
    # The tab has to sit in the handle's 22 mm gap while the stem stays out of the
    # mailbox. Pointing the tab at the part (+y) would put the stem 57 mm clear in -y,
    # and that is the obvious answer - but no reachable wrist roll presents it, measured
    # over all four quarter turns with the hook carried on each side.
    #
    # Pointing it DOWN works instead, and for a better reason: the stem then sits a tab's
    # length ABOVE the tab, at about z 851, which is over the top of the part (827). The
    # stem clears the mailbox entirely rather than squeezing past it.
    # Aim the tab at the PART. That leaves the stem 82 mm clear of the mailbox in -y,
    # which is what the stem needs; aiming it down instead works only because the stem
    # then rides over the top of the part, with much less room.
    pose, offset = await aim(c, approach, park, [0.0, 1.0, 0.0])
    got = await creep(c, [under[0] - 90.0, under[1], under[2]], pose, state)
    offset_now, _, _ = await finger_offset(c)
    print(f"finger parked beside the handle at {[round(v, 1) for v in got]}   "
          f"tab now {[round(v, 1) for v in offset_now]}")
    if offset_now[1] < 40.0:
        raise RuntimeError(
            f"the tab turned between the park pose and here: aim left it pointing at "
            f"the part, it now points {[round(v, 1) for v in offset_now]}")
    got = await creep(c, under, pose, state)
    print(f"finger under the handle at {[round(v, 1) for v in got]}  "
          f"(wanted {[round(v, 1) for v in under]})")

    print("pressing the door open on the handle's top face:")
    # PRESS, do not pull. The handle is solid with nothing behind it, so there is no
    # undercut to hook; but it stands 22 mm proud, and a downward force on its top face
    # is 22 mm of moment arm about a hinge whose spring is only 0.015. The flat panel
    # offers 0.5 mm, which is why every earlier push read 0.0 degrees.
    for step in range(1, 15):
        angle = await door_angle(c.world)
        a = math.radians(angle)
        top = [under[0],
               hinge[1] - HANDLE_PROUD_MM * math.cos(a)
               - HANDLE_TOP_ABOVE_HINGE_MM * math.sin(a),
               hinge[2] + HANDLE_TOP_ABOVE_HINGE_MM * math.cos(a)
               - HANDLE_PROUD_MM * math.sin(a)]
        # Just onto the top face, and a little further down each time.
        # CUMULATIVE. Recomputed from the door's angle each step, a fixed -6 mm offset
        # is the same target every time while the door has not moved - the press never
        # descended, and fourteen steps asked for one position fourteen times.
        target = [top[0], top[1] + HANDLE_PROUD_MM / 2.0, top[2] - 6.0 * step]
        try:
            await put_finger(c, target, pose, state, tries=2)
        except Exception as exc:  # noqa: BLE001
            print(f"  step {step:2d}: refused at door {angle:.1f} deg: {str(exc)[:110]}")
            break
        now = (await part_pose(c))["position_mm"]
        drift = max(abs(now[i] - start[i]) for i in range(3))
        print(f"  step {step:2d} -> door {await door_angle(c.world):6.1f} deg   "
              f"part moved {drift:5.1f} mm")
        if drift > 30.0:
            print("  STOP: the mailbox is being dragged")
            break
        if await door_angle(c.world) > 70.0:
            break
    opened = await door_angle(c.world)
    await move(c, "arm-a", ((under[0] - reach - 100.0, under[1], under[2] + 60.0),
                            pose), approach)
    released = await door_angle(c.world)
    await asyncio.sleep(1.5)
    print(f"\nopened {opened:.1f}   at release {released:.1f}   "
          f"1.5 s later {await door_angle(c.world):.1f}")
    await robot.close()
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
