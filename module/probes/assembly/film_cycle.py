"""Drive the whole cycle from the taught poses, and film it.

Replay, not planning: each pose is a joint configuration taught against a known layout, so
the cycle is the same every time. The loader REFUSES a pose whose layout fingerprint does
not match the cell it is about to drive in, which is the guard the previous build did not
have and paid for four times over.
"""
from __future__ import annotations

import asyncio
import io
import dataclasses
import math
import json
import os
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from assembly.build_fragment import chosen                       # noqa: E402
from assembly.reach_check import POSES, layout_hash              # noqa: E402
from PIL import Image, ImageDraw, ImageFont                              # noqa: E402
from assembly.reach_check import declared, orientation_for        # noqa: E402
from assembly.build_fragment import FRAGMENT, SHELL_KINEMATIC      # noqa: E402
from viam.components.arm import Arm, JointPositions               # noqa: E402
from viam.proto.common import Pose                                # noqa: E402
from viam.services.motion import MotionClient                    # noqa: E402
from viam.components.camera import Camera                        # noqa: E402
from viam.components.generic import Generic                      # noqa: E402
from viam.robot.client import RobotClient                        # noqa: E402

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
FPS = 12.0
from assembly.fixture import index_to, sim_now                   # noqa: E402

# The cycle: which belt index, which arm, which taught pose, and what it is doing.
# step index, arm, taught pose, what to do with the tool there, caption.
#
# `grab` and `release` are what make this a cycle rather than a tour of poses: the flag is
# a dynamic body, so arm-b either picks it up and carries it or it stays on the shelf.
CYCLE = [
    ("tunnel", None, None, None,
     "the vision tunnel: which model, is it whole, then paint on the crown and both walls"),
    ("inside", "arm-a", "inside", None,
     "arm-a: fisheye into the front, checking the interior paint"),
    # THE ORDER FOLLOWS THE BELT, and it reversed twice before it did.
    #
    # Taking the flag first put the sequence at -55 -> 735 -> 364 -> 727 -> 901: 1698 mm
    # of belt with two reversals in it, and the second reversal is what broke the cell.
    # Measured on that order: the flag seated to 0.01 mm, then the 371 mm run back to the
    # pin index left the shell 68.4 degrees out of square - lying over in its cradle - and
    # the pin insert refused, correctly, because at that angle there is nothing to insert
    # into. Taken in index order it is -55 -> 364 -> 727 -> 735 -> 901, 956 mm, monotonic,
    # and the belt never backs up under a part 230 mm tall.
    #
    # Flag-first had a reason and the reason had gone stale: the two tray slots used to sit
    # 60 mm apart in x with a 75 mm tool between them, so reaching for the second part swept
    # the first off the tray. The slots have moved since (Flag at dx 90 y 220, Pin at dx 150
    # y 290), and measuring the real colliders rather than the old note says the tool at the
    # pin's grasp is 84.2 mm from the nearest piece of the flag - its plate - against the
    # 53 mm envelope the planner is given. The crowding is gone; the note outlived it.
    ("pin", "arm-b", "pin_pick", "grab:Pin", "arm-b: three jaws take the hinge pin from its nest"),
    ("pin", "arm-b", "pin_bore_look", None, "arm-b: wrist camera finds the hinge bore"),
    ("pin", "arm-b", "pin_insert", "seat:Pin",
     "arm-b: drives the pin 135 mm through the hinge"),
    # (The back-cap shot is gone: it existed for an old pick off that surface, and the paint
    # check is the vision tunnel's. Brandon, 2026-10-12.)
    ("flag", "arm-b", "flag_hole_look", None, "arm-b: RealSense finds the flag hole"),
    ("flag", "arm-b", "flag_pick", "grab:Flag", "arm-b: takes the flag by its shaft at the end of its feeder"),
    ("flag", "arm-b", "flag_insert", "seat:Flag",
     "arm-b: snaps the flag home, 1.0 mm of slack"),
    # A GOOD SHELL IS NEVER LIFTED (Brandon, 2026-10-12): it rides on down the line.
    ("away", None, None, "ride", "it passed every check: it rides on down the line"),
    # The next one has a paint drip. The tunnel fails it, pin and flag let it pass, and
    # arm-c - which only ever handles rejects - takes it off at the end of the line.
    ("tunnel", None, None, "next:drip",
     "the next mailbox: the tunnel finds a paint drip on its flag side"),
    ("handle", None, None, None, "a reject: the pin and flag stations let it ride past"),
    ("handle", "arm-c", "grasp", "grab:Shell", "arm-c: takes the reject by its -Y wall"),
    ("handle", "arm-c", "repaint", "release", "arm-c: sets it on the reject belt"),
]

# The order is the BELT'S order, and it is monotonic: -55 -> 364 -> 727 -> 735 -> 901, which
# is 956 mm of travel with no reversal. See the note beside the pin steps for what the
# previous order cost.


# How long the shell stands in the tunnel on film: long enough to read the inset.
TUNNEL_DWELL_S = 2.5
# A passed shell rides on at belt speed until it is out of the cell, and the reject belt
# carries a reject away, for this long each.
RIDE_SPEED_MM_S = 420.0
RIDE_S = 4.0
# The pin's head top face is this much nearer the flange than the closed insert tips while
# the jaw holds the shaft: tipZ 192 - (tcp 175 - graspPosition 17.75 - head 0.75). From the
# usd session's published insert:tipZ, insert:tcpZ and the pin's graspPosition/pressFace.
# Release short and press the head home with the closed inserts (the usd session's
# sequence) - OFF. The tools have no colliders, so the press was a kinematic carry of the
# pin, and it seated once to 0.02 mm and failed the next run 20 mm short and 5.7 across.
# Brandon chose (2026-10-13) to keep the jaw on the shaft for the full stroke, which seats
# every run; the inserts visibly enter the knuckle for the last second.
PIN_RELEASE_AND_PRESS = False
# (Dead with PIN_RELEASE_AND_PRESS off, and stale since the knob: these describe the old
# shaft grasp, 175 mm out and 17.75 mm under the head.)
PIN_INSERT_TIP_MM = 192.0
PIN_TIP_TO_HEAD_MM = PIN_INSERT_TIP_MM - (175.0 - 17.75 - 0.75)
# Stops where nothing is picked: the tunnel only photographs (2026-10-13 the pallet could
# not creep into 1.5 mm there; it stood 10.6 mm short, which a camera does not notice).
LOOSE_STOPS = {"tunnel"}
LOOSE_STOP_MM = 25.0
REJECT_RUN_S = 4.5         # at the spur's 300 mm/s: 1.35 m, out past arm-c to the far end
# THE LEARNER'S POINT (the viam-103 cell, probes/assembly/cell_run.py), per part: where
# the learner aimed, as an offset in world mm from where this cell aims by itself, and
# whether that point missed. Empty for the film and cycle_time, which aim by measurement.
#   {"Pin"|"Flag": {"offset_mm": [x, y, z], "missed": bool, "entry_mm": [x, y, z]},
#    "Shell": {"offset_mm": [x, y, z]}}
AIM: dict = {}
MISS_SHORT_OF_FACE_MM = 2.0  # a missed part stops this far short of the face it would hit
MISS_SHOW_S = 1.0            # and holds there long enough to be seen
REJECT_CLEAR_UP_MM = 150.0  # arm-c lifts this far after letting go, before it parks


async def set_paint(world, defect: str) -> None:
    """Which paint defect the shell shows: none, drip, smudge or overspray."""
    await world.do_command({"command": "set_variants", "prop": "shell",
                            "variants": {"paint_defect": defect}})


async def line_event(world, cell, tool, rec=None) -> bool:
    """The steps that are the LINE rather than a station: a passed shell riding away, and
    the next shell arriving in the tunnel. True if `tool` was one of them."""
    from assembly.fixture import restage, run_belt
    if tool == "ride":
        await run_belt(world, run=True, speed_mm_s=RIDE_SPEED_MM_S)
        await asyncio.sleep(RIDE_S)
        await run_belt(world, run=False)
        return True
    if str(tool or "").startswith("next:"):
        # One pallet serves the line, so the next shell is the same props restaged in
        # the tunnel - a cut on film, which the caption covers.
        # Paint FIRST, then restage. Switching the variant re-composes the shell and
        # leaves every arm's physics view dead ("Failed to get DOF positions"); the
        # restage's reset rebuilds them - measured both ways round (2026-10-12).
        await set_paint(world, str(tool).split(":", 1)[1])
        # A CUT, filmed as one: the restage throws the feeders' parts and sets them back,
        # which on film is parts jumping about. Restocked (the restage plus the feeders
        # presenting again) with the camera off.
        if rec is not None:
            rec["on"] = False
        from assembly.fixture import restock_all
        await restock_all(world, cell)
        if rec is not None:
            rec["on"] = True
        return True
    return False
# Which close camera the film insets during each of arm-b's stations - the moments where
# the thing that matters is millimetres across.
TUNNEL_CAMERAS = ("tunnel-top", "tunnel-left", "tunnel-right")   # the order the course reads them
# Which camera the film insets at each step: THE ONE DOING THE TASK, so a learner sees what
# each camera sees (Brandon, 2026-10-13: "do we ever see the camera on the gripper?" - the
# two looks used to inset the film-only detail cameras instead of the wrist RealSense).
# A list is tiled side by side.
INSET_FOR = {"@tunnel": ["tunnel-left", "tunnel-top", "tunnel-right"],
             "inside": "fisheye-cam",
             "pin_bore_look": "realsense-cam",
             "pin_insert": "hinge-cam", "flag_hole_look": "realsense-cam",
             "flag_insert": "flag-cam", "grasp": "grasp-cam"}
# Readable at 720p: the default bitmap font was 11 px and nobody could read a caption.
LABEL_FONT = ImageFont.load_default(size=18)
CAPTION_FONT = ImageFont.load_default(size=24)
# What each inset is labelled with: the camera, what it hangs from, and its job. The
# film-only detail cameras say so, so nobody takes them for the cell's sensors.
CAM_ROLE = {
    "tunnel-top": "tunnel gantry · which model; paint, crown",
    "tunnel-left": "tunnel gantry · paint, +Y wall",
    "tunnel-right": "tunnel gantry · paint, -Y wall",
    "fisheye-cam": "arm-a tool · interior paint",
    "realsense-cam": "arm-b wrist · finds the bore / the flag hole",
    "grasp-cam": "boom over the reject stop · where to take the reject",
    "hinge-cam": "film camera · the hinge",
    "flag-cam": "film camera · the flag hole",
}


def clear_of(arm: str, task: str, stamp: str):
    """The taught stand-off pose for a station, or None if it never got one."""
    path = POSES / arm / f"{task}.clear.json"
    if not path.exists():
        return None
    blob = json.loads(path.read_text())
    return blob if blob["layout"] == stamp else None


def taught(arm: str, task: str, stamp: str):
    path = POSES / arm / f"{task}.json"
    blob = json.loads(path.read_text())
    if blob["layout"] != stamp:
        raise SystemExit(
            f"{path.name} was taught against layout {blob['layout']} and this cell is "
            f"{stamp}. Refusing to drive to a pose taught against a different cell - "
            f"re-run probes/assembly/reach_check.py.")
    return blob


# `_limits` and `park_if_adrift` moved to `assembly.arms`, because the TEACHING pass needs
# them too: three stations failed to teach in a row on "nlopt: INVALID_ARGS", each because
# the station before had left the arm outside its own limits.
from assembly.creds import options                             # noqa: E402
from assembly.arms import clear_and_park, home as arm_home, park_if_adrift  # noqa: E402,F401
from assembly.asset import insertions, part                       # noqa: E402,F401
from assembly.fixture import feed_spawn, restage, restock_all, settle_tray       # noqa: E402
from assembly.replay import Legs, fly as fly_leg, straight_line  # noqa: E402
from assembly.reach_check import step_out                           # noqa: E402
from assembly import shell_locate                                    # noqa: E402
from viam.components.gripper import Gripper                       # noqa: E402


def _quat_from_axes(xa, ya, za):
    """wxyz from a body's world axes (the columns of its rotation)."""
    m = [[xa[r], ya[r], za[r]] for r in range(3)]
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        k = 0.5 / math.sqrt(t + 1.0)
        q = (0.25 / k, (m[2][1] - m[1][2]) * k, (m[0][2] - m[2][0]) * k, (m[1][0] - m[0][1]) * k)
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        k = 2.0 * math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2])
        q = ((m[2][1] - m[1][2]) / k, 0.25 * k, (m[0][1] + m[1][0]) / k, (m[0][2] + m[2][0]) / k)
    elif m[1][1] > m[2][2]:
        k = 2.0 * math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2])
        q = ((m[0][2] - m[2][0]) / k, (m[0][1] + m[1][0]) / k, 0.25 * k, (m[1][2] + m[2][1]) / k)
    else:
        k = 2.0 * math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1])
        q = ((m[1][0] - m[0][1]) / k, (m[0][2] + m[2][0]) / k, (m[1][2] + m[2][1]) / k, 0.25 * k)
    n = math.sqrt(sum(v * v for v in q))
    return [float(v / n) for v in q]


def turn_onto(have, want):
    """The shortest rotation taking one unit vector onto another, as a 3x3 matrix.

    Rodrigues, written out, because the correction it serves is three lines and pulling in
    a rotation library for it would hide what is a very simple fact: two directions differ
    by one rotation about their common perpendicular, through the angle between them.

    Returns the identity when they already agree, and when they are exactly opposed -
    which cannot arise here, because a part held backwards in the tool is a different
    failure and one the slip check catches first.
    """
    ax = [have[1] * want[2] - have[2] * want[1],
          have[2] * want[0] - have[0] * want[2],
          have[0] * want[1] - have[1] * want[0]]
    s = math.sqrt(sum(v * v for v in ax))
    c = sum(have[i] * want[i] for i in range(3))
    if s < 1e-9:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    k = [v / s for v in ax]
    kx = [[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]]
    out = []
    for i in range(3):
        row = []
        for j in range(3):
            eye = 1.0 if i == j else 0.0
            kk = sum(kx[i][m] * kx[m][j] for m in range(3))
            row.append(eye + kx[i][j] * s + kk * (1.0 - c))
        out.append(row)
    # Returned in the same column-vector-per-axis shape `into_frame` consumes.
    return [[out[i][j] for i in range(3)] for j in range(3)]


def into_frame(rot, live, v, *, point: bool):
    """Take a vector expressed in a part's NOMINAL frame into the frame it is really in.

    `rot` is the part's three live axis vectors as the simulator reports them, `live` its
    live origin. A direction is only rotated; a point is rotated and then translated.

    Pulled out of the insert so it can be tested without a cell, because a transposed
    rotation is the failure that looks right: it produces a pose, the arm goes there, and
    the part is inserted at the mirror of the correction it needed. The test asserts the
    property that catches it - that the stroke still ends the same distance from the
    part's seated position as the taught stroke did, which a transpose does not preserve
    once the rotation is about more than one axis.
    """
    out = [sum(rot[k][i] * v[k] for k in range(3)) for i in range(3)]
    return [live[i] + out[i] for i in range(3)] if point else out


# How far from its seated position a part may finish and still be counted as inserted.
#
# Taken from the asset where the asset says, and stated here where it does not. The shell
# publishes `assembly:pinBore:captureRadius` 1.5 mm, which is what the pin's bullet nose
# can pull in; nothing publishes an equivalent for the flag's snap peg, so it gets its own
# published radial slack with the same 1.5 mm of lead-in allowed on top.
#
# This is a gate on the INSERT, not on the finished assembly. A part inside capture is one
# the feature itself will centre; a part outside it is one that stopped against a face.
CAPTURE_DEFAULT_MM = 1.5


CARRY_LINE_MM = 15.0        # how far a carried flag may stray from the straight carry
CARRY_TURN_DEG = 10.0       # and its orientation from the interpolated one
DESCEND_LINE_MM = 10.0      # a descent into an insert may stray this far from its line
DESCEND_TURN_DEG = 5.0
DESCEND_JOINT_GAP_DEG = 45.0
# Pick to insert on one branch; a flip is 150 or more. 120, not 100: the flag's pick and
# insert stand-offs differ by 108 deg on j4 (tool down vs tool level), so at 100 the carry
# was PLANNED, and the planner answered with a wrist flip there and back - 1226 deg of
# joint travel for one carry (Fable review, 2026-10-13).
CARRY_JOINT_GAP_DEG = 120.0
RELEASE_ABOVE_MM = 100.0    # a part is set down from this far straight above its seat
FLAG_SEAT_ALONG_MM = 10.0
FLAG_CAPTURE_ACROSS_MM = 6.0
# The PIN too, per CLAUDE.md's own worked example: the seat joint closes the last couple of
# millimetres, and nobody sees 2 mm across a 138 mm pin on film. Seated 0.00, 0.17 and
# 0.02 mm three runs running, then 2.05 across against the published 1.80 (2026-10-12).
# 6 since the flush pin (2026-10-13): held on its Ø3.4 shaft rather than the old Ø8 collar
# it leans 1.6 degrees in the jaw and arrives up to 4.9 mm across, still fully deep.
PIN_CAPTURE_ACROSS_MM = 6.0
SEAT_ALONG_MM = 3.0         # a stroke this close to full depth is finished by the joint


def capture_mm(which: str, ins) -> float:
    try:
        return part("Shell").mm_at("assembly:pinBore:captureRadius") \
            if which == "Pin" else ins.clearance_mm + CAPTURE_DEFAULT_MM
    except Exception:                                              # noqa: BLE001
        return ins.clearance_mm + CAPTURE_DEFAULT_MM


async def seat_miss(world, which: str, ins) -> tuple:
    """How far the part's INSERTION FEATURE is from where it belongs, split along and
    across the insert axis.

    The same arithmetic `verify_seated` does, pulled out so it can be asked BEFORE the
    joint is made as well as after - the difference between the two readings is exactly how
    much work the joint did, which is a number worth being able to see.

    Split, because the magnitude alone does not say what went wrong and the two halves say
    it immediately. A miss ALONG the axis is a stroke that stopped short: the part went in
    straight and something resisted it, or the arm ran out of push. A miss ACROSS the axis
    is a part that was never lined up with the hole. They want opposite fixes, and reading a
    single number as "nearly there" has cost this cell a day of looking at the wrong half.
    """
    r = (await world.do_command({"command": "prim_poses",
                                 "paths": [f"/World/{which.lower()}",
                                           "/World/shell"]}))["prims"]
    sh = r["/World/shell"]
    me = r[f"/World/{which.lower()}"]
    axes = [sh["x_axis"], sh["y_axis"], sh["z_axis"]]
    # MEASURED AT THE FEATURE THAT GOES IN THE HOLE, not at the part's origin.
    #
    # An origin is wherever the asset's author put it, and on the flag that is 206.7 mm
    # from the peg. So a small rotation of the flag about its grasp - which is 27 mm from
    # the peg, and which the insertion force can easily produce - moves the origin seven
    # and a half times as far as it moves the thing that actually has to fit:
    #
    #     0.8 degrees -> origin 2.89 mm, peg 0.38 mm
    #
    # Measured, the flag "finished 2.89 mm across the bore" while its tilt was 1.92
    # degrees. Reading that as a 2.89 mm insertion error is reading a lever arm. The
    # question an insert has to answer is where the PEG is relative to the HOLE, so that
    # is what this measures: `assembly:pegTip` where the asset publishes one, the origin
    # where it does not - which is right for the pin, whose grasp is 11.5 mm from its
    # origin and whose lever ratio is therefore nothing.
    try:
        feature = [v * 1000.0 for v in part(which).at("assembly:pegTip")]
    except Exception:                                              # noqa: BLE001
        feature = [0.0, 0.0, 0.0]
    want = [sh["position_mm"][i]
            + sum(axes[k][i] * (ins.seated_mm[k] + feature[k]) for k in range(3))
            for i in range(3)]
    mine = [me["x_axis"], me["y_axis"], me["z_axis"]]
    at = [me["position_mm"][i] + sum(mine[k][i] * feature[k] for k in range(3))
          for i in range(3)]
    off = [at[i] - want[i] for i in range(3)]
    # The insert axis in the SHELL's live frame, which is the frame the part is seated in.
    axis = [sum(axes[k][i] * ins.axis[k] for k in range(3)) for i in range(3)]
    along = sum(off[i] * axis[i] for i in range(3))
    across = math.sqrt(max(sum(v * v for v in off) - along * along, 0.0))
    return math.dist(at, want), along, across


# Where each held part sat in its tool's frame at the moment it was grasped, in mm.
# Cleared with the rest of the cycle's state.
HELD: Dict[str, List[float]] = {}
# The flange prim each arm's tool hangs from, as the asset names it. One string, shared
# with the generator's own `FLANGE`.
FLANGE_PRIM = "/World/{prim}/wrist_3_link/flange"


def feature_of(which: str):
    """The point on a part that has to end up in the hole, in the part's own frame.

    `assembly:pegTip` where the asset publishes one, the origin where it does not - which
    is right for the pin, whose grasp is 11.5 mm from its origin and whose lever ratio is
    therefore nothing.
    """
    try:
        return [v * 1000.0 for v in part(which).at("assembly:pegTip")]
    except Exception:                                              # noqa: BLE001
        return [0.0, 0.0, 0.0]


# Stations that run alongside the cycle's next ones, at the same belt stop, on replay.
# arm-a's back shot is 300 mm along the belt from arm-b's flag work and comes in from the
# other side of it; run one after the other they cost 21 s of a 120 s film.
#
# arm-a's interior shot likewise runs while arm-b takes the pin from its nest: `inside`
# was moved to the pin's belt stop, which also removed the cycle's first index.
# Nothing alongside any more. arm-a's inside shot, run alongside arm-b's pin pick, was
# stopped short ("j0, j1 never arrived") on both films of the new line and ran clean every
# time on its own in the teach (2026-10-12). A few seconds, against a station missing.
# Back on (2026-10-14): arm-a's whole inside job is 4.7 s now that it waits over its own
# station on the same wrist branch. (A first retry stalled "j0 never arrived" - because
# that deploy had put the arms' OLD homes back, not because of running alongside.)
CONCURRENT = {"inside"}
# ...and these wait for whatever is running alongside: they work on the shell where the
# background station is working (the pin goes into the front the fisheye is looking
# through), or move it (arm-c lifts the shell the back shot is photographing).
AWAIT_BEFORE = {"pin_insert", "grasp"}
# Where a good flag grab holds the flag in the flange's frame (every good grab measured
# 209.8-209.9, -0.4..+0.1, -0.4..+0.3, 2026-10-18/19), and how far off one is re-taken.
FLAG_HELD_MM = (209.9, -0.2, 0.1)
FLAG_HELD_TOLERANCE_MM = 3.0
FLAG_REGRABS = 2
# Picks from the FIXED feeders do not need the belt anywhere in particular, so they do not
# index it - and do not wait for a station running alongside at the current stop.
OFF_BELT = {"pin_pick", "flag_pick"}


async def _alongside(arm, pose, tool, *, arms, grips, motion, world, stamp, legs, cell,
                     home, park_leg: bool = False):
    """One station run as a background task, then parked: see CONCURRENT.

    `park_leg`: the program taught this arm's park straight after the station, so fly
    that leg (replayed) before settling on the home joints."""
    from viam.components.arm import JointPositions
    try:
        await park_if_adrift(arms[arm], arm)
        await _station(arm, pose, tool, arms=arms, grips=grips, motion=motion,
                       world=world, stamp=stamp, state=await declared(world, moving=arm),
                       legs=legs, cell=cell,
                       carry_state=await declared(world, moving=arm, motion=motion,
                                                  grip=grips.get(arm), tools=True),
                       held_state=await declared(world, moving=arm, motion=motion,
                                                 grip=grips.get(arm)))
        if park_leg:
            await fly_leg(legs, arms, motion, arm, None, None, None, label="park")
        await arms[arm].move_to_joint_positions(JointPositions(values=home), timeout=180)
    except Exception as exc:                                       # noqa: BLE001
        print(f"  {pose} (alongside) FAILED {str(exc)[:200]}", flush=True)


async def _regrip_square(world, grip, arm: str, which: str) -> None:
    """Stand a held part square in the jaws again, about its grasp point.

    Three jaws clamped on a shaft do not let a part swivel; the simulated hold does. The
    flag left the feeder upright and arrived at its hole pitched 18 degrees in the grip,
    so its peg met the wall beside the hole and every press drove it further off. So, at
    the mouth: open, put the part back square (its seated orientation, which is the
    world's here) with its grasp point where the jaws are, close, and re-measure the hold.
    """
    name = which.lower()
    g = (await world.do_command({"command": "prim_poses",
                                 "paths": [f"/World/{name}"]}))["prims"][f"/World/{name}"]
    local = [v * 1000.0 for v in part(which).at("assembly:graspPosition")]
    grasp = [g["position_mm"][i] + local[0] * g["x_axis"][i] + local[1] * g["y_axis"][i]
             + local[2] * g["z_axis"][i] for i in range(3)]
    origin = [grasp[i] - local[i] for i in range(3)]
    await grip.open()
    await world.do_command({"command": "place_prop", "name": name,
                            "position_mm": origin,
                            "orientation_wxyz": [1.0, 0.0, 0.0, 0.0]})
    await grip.grab()
    await asyncio.sleep(1.5)
    HELD[which] = await in_tool(world, arm, which)
    print(f"    regripped {which} square; holding it at "
          f"{['%+.2f' % v for v in HELD[which]]} mm in the flange's frame", flush=True)


async def in_tool(world, arm: str, which: str, local=None):
    """A point on the part, expressed in the FLANGE's frame.

    This is the one number that makes a pick and an insert agree, and it is measured rather
    than booked. A station places the flange by a scalar stand-off along one axis; what has
    to arrive somewhere is the part; and the two reconcile only when the grasp point lies
    on the station's axis, which on this cell neither part's does. Recorded at the grab and
    read again at the insert, it says both how far the insert has to step - the flag's is
    205 mm, because its grasp is at the far end of a 252 mm blade - and whether the part
    has moved in the tool since, which is the only one of those two that is a fault.
    """
    local = feature_of(which) if local is None else local
    paths = [FLANGE_PRIM.format(prim=arm.replace("-", "_")), f"/World/{which.lower()}"]
    got = (await world.do_command({"command": "prim_poses", "paths": paths}))["prims"]
    f, p = got[paths[0]], got[paths[1]]
    axes = [f["x_axis"], f["y_axis"], f["z_axis"]]
    mine = [p["x_axis"], p["y_axis"], p["z_axis"]]
    at = [p["position_mm"][i] + sum(mine[k][i] * local[k] for k in range(3))
          for i in range(3)]
    d = [at[i] - f["position_mm"][i] for i in range(3)]
    # Into the flange's frame: project onto its axes.
    return [sum(d[i] * axes[k][i] for i in range(3)) for k in range(3)]


async def flange_that_puts(world, arm: str, held, where):
    """The flange position that puts a held point at `where`, using the MEASURED hold.

    This is the fix for a 3.91 mm error that both parts were paying every cycle. A station
    places the flange by a scalar stand-off and assumes the part hangs a modelled distance
    below it; measured, the pin hangs 180.41 mm from the flange where the model says 176.5.
    The correction loop was finding that difference by moving the arm, four millimetres at
    a time, every single insert - and the moving is what put parts into contact and made
    the loop diverge.

    Measured once at the grab, it is arithmetic: the flange goes wherever it has to for the
    point it is carrying to land on the target.
    """
    prim = FLANGE_PRIM.format(prim=arm.replace("-", "_"))
    f = (await world.do_command({"command": "prim_poses",
                                 "paths": [prim]}))["prims"][prim]
    axes = [f["x_axis"], f["y_axis"], f["z_axis"]]
    # where the held point is now, from the flange and the recorded offset
    at = [f["position_mm"][i] + sum(axes[k][i] * held[k] for k in range(3))
          for i in range(3)]
    return [f["position_mm"][i] + (where[i] - at[i]) for i in range(3)], at


async def slip_mm(world, arm: str, which: str) -> float:
    """How far the part has moved in the tool since it was grasped."""
    was = HELD.get(which)
    if was is None:
        return 0.0
    now = await in_tool(world, arm, which)
    return math.dist(now, was)


async def say_axis(world, which: str, when: str) -> None:
    """Print which way a part's own axes point, in one line.

    Cheap, and it settles arguments that arithmetic cannot. A part that is where it should
    be but pointing somewhere else looks identical, in every position-only reading, to a
    part that is simply misplaced.
    """
    try:
        got = (await world.do_command({
            "command": "prim_poses",
            "paths": [f"/World/{which.lower()}"]}))["prims"][f"/World/{which.lower()}"]
    except Exception:                                              # noqa: BLE001
        return
    print(f"    {when}: {which} x{['%+.2f' % v for v in got['x_axis']]} "
          f"y{['%+.2f' % v for v in got['y_axis']]} "
          f"z{['%+.2f' % v for v in got['z_axis']]}", flush=True)


async def verify_seated(world, which: str, ins) -> float:
    """Measure the insertion NOW, not at the end of the cycle.

    What a part is doing four stations later is a different question. arm-c's handling
    poses drive a gripper against the shell, and with the part dynamic that moves it - so a
    check at the end of the run was reporting the handling, not the insertion, and reading
    2.7 mm as 5.8. A cell that verifies each insertion before going on to the next is also
    what you would actually build: a seat that failed is cheap to find here and expensive
    to find after three more operations have been performed on top of it.
    """
    # The SAME arithmetic the pre-join check uses, so the two readings are comparable -
    # and the difference between them is exactly how far the joint pulled the part, which
    # is the number that was hiding a failed insert behind a 0.01 mm "seated".
    d, along, across = await seat_miss(world, which, ins)
    print(f"    {which} seated {d:.2f} mm from the published position "
          f"({abs(along):.2f} mm {'past' if along > 0 else 'short of'} depth, "
          f"{across:.2f} mm across; slack {ins.clearance_mm:.2f} mm)", flush=True)
    return d


async def settle_hinge(world, which: str, ins, timeout_s: float = 20.0) -> float:
    """Wait for a hinged part to swing shut before anything else is done to it.

    The hinge pin captures the door in whatever position it finds it. Released from the
    gripper the door is still swinging - its spring makes 0.15 N.m against gravity's 0.086
    at ten degrees, so it closes, but not instantly - and the cycle went straight on to
    pick the pin and drive it through the knuckles. Measured, that pinned the door 28.5
    degrees open: on its hinge, at exactly the right radius, and wrong.

    Which is an assembly-order fact rather than a tuning one. A hinge pin is fitted to a
    closed door.
    """
    import math

    anchor = ins.joint_anchor_mm
    seated = ins.seated_mm
    # The seated arm of the hinge, in the shell's own frame: what the part's offset from
    # the hinge looks like when it is shut.
    arm0 = [seated[i] - anchor[i] for i in range(3)]
    deadline = time.perf_counter() + timeout_s
    angle = 180.0
    while time.perf_counter() < deadline:  # noqa: B007
        r = (await world.do_command({"command": "prim_poses",
                                     "paths": [f"/World/{which.lower()}",
                                               "/World/shell"]}))["prims"]
        part_at = r[f"/World/{which.lower()}"]["position_mm"]
        shell_at = r["/World/shell"]["position_mm"]
        now = [part_at[i] - (shell_at[i] + anchor[i]) for i in range(3)]
        # Angle between the seated arm and the current one, about the hinge axis.
        dot = sum(a * b for a, b in zip(arm0, now))
        na = math.sqrt(sum(a * a for a in arm0)) or 1.0
        nb = math.sqrt(sum(b * b for b in now)) or 1.0
        angle = math.degrees(math.acos(max(-1.0, min(1.0, dot / (na * nb)))))
        if angle <= 2.0:
            break
        await asyncio.sleep(0.25)
    else:
        print(f"    {which} still {angle:.1f} deg open after {timeout_s:.0f}s", flush=True)
    print(f"    {which} settled {angle:.1f} deg from shut", flush=True)
    return angle


async def let_go(grip, what: str, timeout_s: float = 6.0) -> None:
    """Open the gripper and WAIT UNTIL IT HAS ACTUALLY LET GO.

    A surface gripper does not release on the step the action is applied any more than it
    attaches on one - it has a fabricated joint to destroy - and the seat sequence gave it
    a flat 0.8 s before reversing the arm out. When that was not enough the arm backed off
    still holding a pin that had just been jointed to the shell, and dragged the shell, the
    cradle and 40 kg of pallet with it: measured, "pallet is off the deck at x 3788".
    Everything after that ran against a cell that had been flung down the belt, which is
    how arm-c came to report the shell missing from a station it holds perfectly well in
    isolation.

    The gripper knows. Ask it, the same way the grab does.
    """
    await grip.open()
    deadline = time.perf_counter() + timeout_s
    while time.perf_counter() < deadline:
        await asyncio.sleep(0.2)
        held = (await grip.do_command({"command": "status"})).get("gripped") or []
        if not held:
            return
    print(f"    {what}: still holding {held} after {timeout_s:.0f}s - backing off anyway, "
          f"which will drag whatever it is still attached to", flush=True)


async def _shell_tilt_deg(world) -> float:
    """How far the shell is from sitting square in its cradle, in degrees.

    Measured off its own Y axis, because that is the axis the pin goes in along and the
    one the flag hole is drilled on. A cradle with 1.2 mm of clearance and 70 mm lips
    should not permit much; when it does, something is holding the shell up.
    """
    got = (await world.do_command({"command": "prim_poses",
                                   "paths": ["/World/shell"]}))["prims"]["/World/shell"]
    y = got["y_axis"]
    return math.degrees(math.acos(max(-1.0, min(1.0, abs(float(y[1]))))))


async def _why_nothing(world, arm: str, which: str = "",
                       work_mm=None) -> str:
    """Where the cup actually ended up, said at the moment the grab failed.

    A failed grab is three different faults wearing one message - the tool stopped short,
    the tool is inside the part, or the part is not there at all - and telling them apart
    afterwards means reproducing the whole cycle. The cell already knows: ask the flange
    where it is and raycast down the tool axis. Measured once, at the moment it mattered,
    beats inferred twice.
    """
    import json

    from assembly.build_fragment import FRAGMENT

    try:
        grip = f"grip-{arm[-1]}"
        attrs = next(c["attributes"] for c in json.loads(FRAGMENT.read_text())["components"]
                     if c["name"] == grip)
        reach = float(attrs.get("max_grip_distance", 0.02)) * 1000.0
        out = float(attrs["offset"][0]) * 1000.0
        flange = f"/World/{arm.replace('-', '_')}/wrist_3_link/flange"
        at = (await world.do_command({"command": "prim_poses",
                                      "paths": [flange]}))["prims"][flange]
        axis = at["x_axis"]
        cup = [at["position_mm"][i] + axis[i] * out for i in range(3)]
        ray = (await world.do_command({
            "command": "raycast", "from_mm": cup, "along": list(axis),
            "distance_mm": max(reach * 5.0, 100.0)}))["ray"]
        # And WHERE THE PART IS, which is the other half of the question. "the ray hit
        # the tray" says the part is not under the tool; it does not say whether the part
        # never arrived, drifted, or is sitting 2 mm to one side of a 7 mm collar.
        where = ""
        if which:
            try:
                prop = f"/World/{which.lower()}"
                got = (await world.do_command({"command": "prim_poses",
                                               "paths": [prop]}))["prims"][prop]
                at = got["position_mm"]
                where = (f" {which} is at {['%.1f' % v for v in at]}"
                         + (f", {math.dist(at, work_mm):.1f} mm from the "
                            f"{['%.1f' % v for v in work_mm]} this station works to"
                            if work_mm else "")
                         + f", x_axis {['%+.2f' % v for v in got['x_axis']]}.")
            except Exception:  # noqa: BLE001
                where = ""
        if not ray.get("hit"):
            return (f"The cup is at {['%.1f' % v for v in cup]} and sees NOTHING within "
                    f"{max(reach * 5.0, 100.0):.0f} mm down the tool axis.{where}")
        return (f"The cup is at {['%.1f' % v for v in cup]} and sees "
                f"{ray.get('collider')} {ray['distance_mm']:.1f} mm away, against a "
                f"{reach:.0f} mm reach - "
                + ("too far." if ray["distance_mm"] > reach
                   else "inside its own stand-off, so it cannot seal."
                   if ray["distance_mm"] < 3.0
                   else "in reach, so the fault is elsewhere.") + where)
    except Exception as exc:  # noqa: BLE001 - a diagnostic must never mask the failure
        return f"(could not read the cup back: {str(exc)[:90]})"


# How far off nominal a tray part has to be before the pick is corrected, and how far off
# it can be before the correction is refused.
#
# The floor is there because a correction costs a planned move (measured on this cell:
# 1.0 s and 4.6 s for a 6 mm step) and a sub-millimetre one buys nothing. The ceiling is
# there because beyond it the part has not drifted, it has LEFT - ejected, stood on end or
# walked off its slot - and flying a gripper to where a loose part happens to be lying is
# how this cell throws things. 40 mm is past anything a settled part has ever shown (worst
# measured drift 4.7 mm; worst departure, 29.8 mm in y, was a part that had already gone)
# and well short of the 252 mm a stood-up flag reads as.
# Lining the HELD part up with its stroke: below the floor nothing moves, and above the
# ceiling the run stops.
#
# The ceiling measures a different thing from the floor, and that distinction cost a run.
# The floor is on the CORRECTION - how far the flange steps - and that correction is
# systematic and can be large: the flag's grasp sits 205 mm from its origin at the far end
# of a 252 mm blade, so a station that places the origin puts the flange 205 mm from where
# a correctly-held flag needs it. The ceiling is on the SLIP: how far the part has moved in
# the tool's own frame since it was grasped, which should be nothing and which is the only
# one of the two that means something is wrong.
#
# 20 mm of slip, not 60. A grip that has given up 20 mm has given up.
SEAT_ALIGN_FLOOR_MM = 0.3
SQUARE_TRIES = 3
SQUARE_DONE_DEG = 0.3
# How many times the align will re-measure before giving up and stroking anyway.
SEAT_ALIGN_TRIES = 3
# Past this the part is not held slightly crooked, it has moved in the grip, and
# turning the tool to chase it swings the part rather than aiming it: a 16.71 degree
# reading was 'corrected' into a 356 mm miss.
SEAT_TILT_MAX_DEG = 10.0
# How many short corrective presses an insertion makes to close the gap the grip's
# stretch leaves. Each one is the size of what the measurement says is left.
SEAT_PRESS_TRIES = 3
SEAT_ALIGN_CEILING_MM = 20.0
PICK_CORRECT_FLOOR_MM = 0.5
# A correction is a PLANNED leg on every replay - the shell's cost 30 s a cycle - so a part
# whose grip does not care about a few millimetres is not chased for them: a side-wall grip
# on the shell and the jaws closing on the flag's shaft both centre themselves.
# The shell's floor follows CUP_GAP_MM (8) less the ~6 mm the cup needs to seal: drift
# above 2 mm is corrected, or the delivered gap can fall under the seal (2026-10-13).
PICK_FLOOR_FOR = {"Shell": 2.0, "Flag": 3.0}
PICK_CORRECT_CEILING_MM = 40.0


async def measured_grasp(world, which: str, cell, index_x: float):
    """Where the part's grasp point actually IS, and where the taught pose expects it.

    The carrier cannot present a part to better than a few millimetres and three rounds of
    tightening its pockets each made something worse, the last one standing the flag on its
    end. So the pick stops trusting geometry and LOOKS: it goes to the taught pose, reads
    where the part really is, and steps across the difference.

    Two things make this cheap rather than clever. The nominal point is recomputed from the
    same published attributes the station was built from, so the two cannot drift apart.
    And the OFFSET is what gets applied, so the tool's standoff and the cup's gap cancel
    out and are not restated here - a number restated is a number that goes stale.

    It also fixes the seat, not just the pick. A part grabbed at its true grasp point sits
    in the tool at the NOMINAL offset, which is exactly what every taught insertion
    assumes; a part grabbed 5 mm off its grasp is carried 5 mm off and then inserted 5 mm
    off, which is how a 0.20 mm bore gets a pin pressed into its face.

    THE SHELL GETS THE SAME TREATMENT, and it needs it most. arm-c takes it by its -Y wall
    with the cup 14 mm off, and that cup was measured to catch nothing at 2.7 mm and hold
    at 6 - so the whole working window is about 6 to 20 mm, and the shell has 6 mm of slack
    in its saddle BY DESIGN. A pose taught to the nominal wall is therefore one bad settle
    away from being out of reach, which is exactly how it failed: "the cup sees
    /World/shell/Collision/WallL 34.5 mm away, against a 20 mm reach". Measuring the wall
    instead of assuming it turns a marginal grasp into one with the slack designed out.
    """
    from assembly.fixture import feed_grasp_mm

    if which == "Shell":
        shell = part("Shell")
        local = (shell.centre_offset_mm[0],
                 -shell.mm_at("assembly:wall:sideY"),
                 (shell.mm_at("assembly:wall:sideZMin")
                  + shell.mm_at("assembly:wall:sideZMax")) / 2.0)
        nominal = [index_x + local[0], cell.belt_y_mm + local[1],
                   cell.shell_origin_mm[2] + local[2]]
        prim = "/World/shell"
    else:
        p = part(which)
        local = [v * 1000.0 for v in p.at("assembly:graspPosition")]
        # Where the FEEDER presents the grasp point - rotated by the pose it holds the part
        # in, which for the pin standing in its nest is 90 degrees from the part's own.
        nominal = list(feed_grasp_mm(cell, which))
        prim = f"/World/{which.lower()}"

    got = (await world.do_command({"command": "prim_poses",
                                   "paths": [prim]}))["prims"][prim]
    if which == "Shell" and shell_locate.CLIENT is not None:
        # FROM THE POINT CLOUD (Brandon, 2026-10-14): grasp-cam's depth, fitted to the
        # shell's upper walls and crown. The sim's own pose is printed beside it, so every
        # pick says how far the vision was out.
        seen = await shell_locate.locate(shell_locate.CLIENT, world, cell, index_x)
        yaw = math.radians(seen["yaw_deg"])
        truth = got["position_mm"]
        print(f"    grasp-cam: shell at x {seen['x_mm']:.1f} y {seen['y_mm']:.1f} yaw "
              f"{seen['yaw_deg']:+.2f} from {seen['points']} points; the sim has it at "
              f"x {truth[0]:.1f} y {truth[1]:.1f} (vision {seen['x_mm'] - truth[0]:+.1f}, "
              f"{seen['y_mm'] - truth[1]:+.1f} mm)", flush=True)
        got = {"position_mm": [seen["x_mm"], seen["y_mm"], cell.shell_origin_mm[2]],
               "x_axis": [math.cos(yaw), math.sin(yaw), 0.0],
               "y_axis": [-math.sin(yaw), math.cos(yaw), 0.0],
               "z_axis": [0.0, 0.0, 1.0]}
    at = got["position_mm"]
    # ROTATED by the part's own axes before being added. An unrotated offset added to a
    # part's origin is the mistake that cost this build a day, and a flag lying at any yaw
    # at all puts its grasp shaft somewhere an unrotated sum does not reach.
    axes = [got["x_axis"], got["y_axis"], got["z_axis"]]
    here = [at[i] + sum(axes[k][i] * local[k] for k in range(3)) for i in range(3)]
    return [here[i] - nominal[i] for i in range(3)], here, nominal


async def _show_miss(world, arms, grips, motion, arm, pose, which, stamp, rot, live, nom,
                     at_f, at_app, at_axis, blob, aim, here_feature, state, carry_state,
                     engage_mm):
    """A MISSED insert, as the contract has it: the part goes in along its axis at the
    learner's point, stops at the face it would hit, holds there to be seen, backs out
    and goes back to where it was picked. It is never driven into the shell - a part
    pressed against the shell off its bore can topple the shell and lose the datum for
    every station after this one."""
    entry = [aim["entry_mm"][i] - nom[i] for i in range(3)]
    entry_live = into_frame(rot, live, entry, point=True)
    # The measured point is the part's origin (the pin's head) or its peg; the end that
    # meets the face leads it by the engage depth - 132 mm on the pin, 10 on the flag - so
    # at the stroke's start that end already stands about 3 mm off the face (2026-10-18:
    # leaving this out drove a missed pin 134 mm through the knuckles).
    to_face = sum((entry_live[i] - here_feature[i]) * at_axis[i] for i in range(3)) - engage_mm
    run = max(0.0, to_face - MISS_SHORT_OF_FACE_MM)
    print(f"    {pose}: a MISS - stroking {run:.1f} mm, to {MISS_SHORT_OF_FACE_MM:.0f} mm "
          f"short of the face, then back", flush=True)
    st_in = await declared(world, moving=arm, motion=motion, grip=grips.get(arm),
                           attach=False)
    orient = orientation_for(at_app, blob.get("theta_deg", 0.0))
    face = [at_f[i] + at_axis[i] * run for i in range(3)]
    await fly_leg(None, arms, motion, arm, face, orient, st_in,
                  label=f"{pose}:miss", constraints=straight_line())
    await asyncio.sleep(MISS_SHOW_S)
    await fly_leg(None, arms, motion, arm, at_f, orient, st_in,
                  label=f"{pose}:miss_back", constraints=straight_line())
    await return_part(world, arms, grips, motion, arm, pose, which, stamp, state, carry_state)


async def return_part(world, arms, grips, motion, arm, pose, which, stamp, state,
                      carry_state=None):
    """Put a held part back where it was picked: over the pick, straight down, let go,
    straight up. The next restock squares it in its slot."""
    pick_pose = f"{which.lower()}_pick"
    pick, over = taught(arm, pick_pose, stamp), clear_of(arm, pick_pose, stamp)
    if over is not None:
        await fly_leg(None, arms, motion, arm, over["flange_mm"],
                      orientation_for(over["approach"], over.get("theta_deg", 0.0)),
                      carry_state or state, label=f"{pose}:return")
    # Down WITHOUT the part attached: it is going back into its own slot, which an
    # attached part collides with - the same reason the insert's stroke detaches it.
    st_in = await declared(world, moving=arm, motion=motion, grip=grips.get(arm),
                           attach=False)
    await fly_leg(None, arms, motion, arm, pick["flange_mm"],
                  orientation_for(pick["approach"], pick.get("theta_deg", 0.0)), st_in,
                  label=f"{pose}:set_down", constraints=straight_line())
    await let_go(grips[arm], f"{pose} return")
    HELD.pop(which, None)
    if over is not None:
        await fly_leg(None, arms, motion, arm, over["flange_mm"],
                      orientation_for(over["approach"], over.get("theta_deg", 0.0)),
                      await declared(world, moving=arm), label=f"{pose}:clear",
                      constraints=straight_line())

async def _station(arm, pose, tool, *, arms, grips, motion, world, stamp, state,
                   legs=None, cell=None, carry_state=None, held_state=None):
    """Drive one arm to one station and do whatever its tool does there.

    A function rather than a block inside the loop so that the LOOP CAN CATCH IT. A cycle
    that shows eleven stations and names the one that failed is worth more than no cycle at
    all, and a failure that only ever lands in a log nobody watches beside the film is a
    failure that gets rediscovered.

    The move is PLANNED, not a joint-space jump to the taught angles. The taught poses are
    each collision-free; the straight line in joint space BETWEEN two of them is not, and a
    joint move does not ask. Going from the inside shot to the back shot that way swung j3
    through 207 degrees and the arm stopped against the cell - "something stopped the arm
    rather than out-torqued it", 622 N.m on the base and not moving.

    In through the taught STAND-OFF, so the last leg runs along the tool axis, which is the
    only direction certainly clear of whatever is being worked on; and back out through it
    afterwards, before anything else moves.
    """
    blob = taught(arm, pose, stamp)
    via = clear_of(arm, pose, stamp)

    async def fly(leg, what=""):
        """Drive one leg, through the program if there is one.

        Every planned move in a station goes through here, including the insertion stroke
        and the back-off, so a taught program holds the whole station rather than its
        approach. A leg that is missed is a leg that gets planned on every cycle, which is
        30-60 s each and is the entire reason the cycle was over budget.
        """
        # CARRYING A PART INTO AN INSERT, the planner is told about the part. It knew the
        # jaw and nothing under it, so a free path took the 250 mm flag over the shell's
        # crown and scraped it out of the grip, five runs running; a straight line found no
        # path at all. `carry_state` attaches the held part to the arm. Only for this leg:
        # the approach and the stroke put the part INTO the shell, which is the point.
        if what == "via" and str(tool or "").startswith("seat:"):
            if str(tool) == "seat:Flag":
                # Roll first, in place above the feeder: the pick holds the tool turned to
                # the feeder's angle and the insert wants it square.
                up = clear_of(arm, "flag_pick", stamp)
                want0 = leg.get("joints_deg")
                now0 = list((await arms[arm].get_joint_positions()).values)
                same = bool(want0) and max(abs(a - b) for a, b in
                                           zip(now0[:5], want0[:5])) <= CARRY_JOINT_GAP_DEG
                # On one branch the joint carry below turns the wrist as it goes.
                if (not same and up is not None
                        and abs(float(up.get("theta_deg", 0.0))) > 0.5):
                    await fly_leg(legs, arms, motion, arm, up["flange_mm"],
                                  orientation_for(up["approach"], leg.get("theta_deg", 0.0)),
                                  carry_state or state, label=f"{pose}:square")
                    await _after("square")
            # Pick and insert taught on ONE branch (reach_check starts the pick from the
            # insert's): carry between their stand-offs as a joint move, which keeps the
            # arm's shape. A planned carry ended 158 degrees round on the base.
            want = leg.get("joints_deg")
            if want and str(tool) == "seat:Flag":
                from viam.components.arm import JointPositions
                now = list((await arms[arm].get_joint_positions()).values)
                gap = max(abs(a - b) for a, b in zip(now[:5], want[:5]))
                if gap <= CARRY_JOINT_GAP_DEG:
                    print(f"    {pose}: carrying by joints ({gap:.0f} deg at most)",
                          flush=True)
                    await arms[arm].move_to_joint_positions(JointPositions(values=want),
                                                            timeout=90)
                    return await fly_leg(legs, arms, motion, arm, leg["flange_mm"],
                                         orientation_for(leg["approach"],
                                                         leg.get("theta_deg", 0.0)),
                                         state, label=f"{pose}:{what}")
                print(f"    {pose}: pick and insert are {gap:.0f} deg apart - another "
                      f"branch; planning the carry", flush=True)
            return await fly_leg(legs, arms, motion, arm, leg["flange_mm"],
                                 orientation_for(leg["approach"], leg.get("theta_deg", 0.0)),
                                 carry_state or state, label=f"{pose}:{what}")
        # Onto the reject's wall in a STRAIGHT line from its stand-off. Planned freely, the
        # cup went in over the crown and through the arch (the collision audit,
        # 2026-10-13) - the vacuum unit "moved through the mailbox" Brandon saw.
        if what == "at" and str(tool or "") == "grab:Shell":
            return await fly_leg(legs, arms, motion, arm, leg["flange_mm"],
                                 orientation_for(leg["approach"], leg.get("theta_deg", 0.0)),
                                 state, label=f"{pose}:{what}", constraints=straight_line())
        # THE REJECT CARRY IS ONE BASE SWING (Brandon, 2026-10-13: "moves all over the
        # place"). The spur is sited at the grasp mirrored through arm-c's base, so the
        # lifted grasp turned 180 degrees on j1 IS the pose over the spur, and the shell
        # turns with it square. Nothing else moves; the planner is not asked.
        if what == "via" and str(tool or "") == "release" and arm == "arm-c":
            from viam.components.arm import JointPositions
            now = list((await arms[arm].get_joint_positions()).values)
            swung = list(now)
            # Round by +X (j1 DOWN), never by -X: that way the held shell sweeps over
            # arm-a's pedestal 600 mm along the line. From the taught grasp's j1 191 it is 11.
            swung[0] = now[0] - 180.0
            if swung[0] < -200.0:
                raise RuntimeError(f"{pose}: j1 {now[0]:.0f} cannot swing 180 by +X "
                                   f"inside its limits; re-teach the grasp on j1 > -20")
            print(f"    {pose}: swinging j1 {now[0]:.0f} -> {swung[0]:.0f}, nothing else",
                  flush=True)
            await arms[arm].move_to_joint_positions(JointPositions(values=swung),
                                                    timeout=90)
            # Recorded from where it now is, so the replay's leg count matches.
            return await fly_leg(legs, arms, motion, arm, leg["flange_mm"],
                                 orientation_for(leg["approach"], leg.get("theta_deg", 0.0)),
                                 carry_state or state, label=f"{pose}:{what}")
        # And IN from the stand-off in a straight line along the tool axis. Planned freely
        # this 150 mm descent swung the flag out over the crown and knocked the shell 7.7
        # degrees out of its cradle, with the carry before it having arrived square.
        if what == "at" and str(tool or "") == "release":
            st_rel = await declared(world, moving=arm, motion=motion, grip=grips.get(arm),
                                    attach=False)
            return await fly_leg(legs, arms, motion, arm, leg["flange_mm"],
                                 orientation_for(leg["approach"], leg.get("theta_deg", 0.0)),
                                 st_rel, label=f"{pose}:{what}", constraints=straight_line())
        if what == "at" and str(tool or "").startswith("seat:"):
            from viam.components.arm import JointPositions
            from viam.proto.service.motion import Constraints, LinearConstraint
            o = orientation_for(leg["approach"], leg.get("theta_deg", 0.0))
            try:
                return await fly_leg(
                    legs, arms, motion, arm, leg["flange_mm"], o, state,
                    label=f"{pose}:{what}", constraints=Constraints(linear_constraint=[
                        LinearConstraint(line_tolerance_mm=DESCEND_LINE_MM,
                                         orientation_tolerance_degs=DESCEND_TURN_DEG)]))
            except Exception as exc:                               # noqa: BLE001
                print(f"    {pose}: no straight descent ({str(exc)[:120]})", flush=True)
            # Refused as a line (at 1 mm it was "no direct solutions"): go straight to the
            # station's TAUGHT joints instead - joint interpolation over a 150 mm move along
            # the tool axis stays near the line - but only if they are this arm's current
            # branch, which a joint move would otherwise swing the whole arm across.
            now = list((await arms[arm].get_joint_positions()).values)
            want = leg.get("joints_deg")
            gap = max(abs(a - b) for a, b in zip(now, want)) if want else 999.0
            if gap > DESCEND_JOINT_GAP_DEG:
                raise RuntimeError(f"{pose}: no straight descent, and the taught joints are "
                                   f"{gap:.0f} deg from the arm's - another branch")
            print(f"    {pose}: descending by joints ({gap:.1f} deg at most)", flush=True)
            await arms[arm].move_to_joint_positions(JointPositions(values=want), timeout=60)
            # Planned from where it now is, which is nowhere: records the leg for the
            # replay without moving.
            return await fly_leg(legs, arms, motion, arm, leg["flange_mm"], o, state,
                                 label=f"{pose}:{what}")
        # A station with no tool verb that the arm reaches HOLDING something - arm-c
        # turning the shell under the overhead camera - plans with the part attached. The
        # planner otherwise kept the lifted shell as a fixed box and refused the ee_link
        # inside it.
        # Between stations (the stand-off in, the retreat out) the planner carries the
        # TOOL and anything held; the final approach along the tool axis carries only what
        # is held - at a station the tool is meant to be at the work.
        if what in ("via", "off") and carry_state is not None:
            here = carry_state
        elif tool is None and held_state is not None:
            here = held_state
        else:
            here = state
        # Setting a part down: carried to the stand-off WITH the part attached (lowered
        # unseen it was knocked out of the grip on the saddle), and only the last leg -
        # the placement itself - with it dropped and not carried.
        if str(tool or "") == "release" and what == "via" and carry_state is not None:
            here = carry_state
        elif str(tool or "") == "release" and motion is not None:
            here = await declared(world, moving=arm, motion=motion, grip=grips.get(arm),
                                  attach=False)
        if arm == "arm-c" and grips.get(arm) is not None:
            st = await grips[arm].do_command({"command": "status"})
            shells = [x.label for gi in here.obstacles for x in gi.geometries
                      if x.label.startswith("shell")]
            print(f"    {pose}:{what}: grip {st.get('status')} {st.get('gripped')}; "
                  f"planner sees {len(shells)} shell boxes, "
                  f"{len(here.transforms)} carried", flush=True)
        return await fly_leg(legs, arms, motion, arm, leg["flange_mm"],
                             orientation_for(leg["approach"], leg.get("theta_deg", 0.0)), here,
                             label=f"{pose}:{what}" if what else str(pose))

    async def _after(label):
        # Where the carried part and the shell are after each leg into an insert, so the
        # leg that knocks the shell is named rather than inferred from the next station.
        if not str(tool or "").startswith("seat:"):
            return
        part_prim = "/World/" + str(tool).split(":", 1)[1].lower()
        g = (await world.do_command({"command": "prim_poses", "paths": [
            "/World/shell", part_prim]}))["prims"]
        sh = g["/World/shell"]
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, abs(float(sh["y_axis"][1]))))))
        pz = g[part_prim]["z_axis"]
        print(f"    {pose}: after {label}: shell {tilt:.1f} deg; part at "
              f"{[round(v, 1) for v in g[part_prim]['position_mm']]}, its z "
              f"{[round(v, 3) for v in pz]}", flush=True)

    order = ([(via, "via")] if via else []) + [(blob, "at")]
    if str(tool or "") == "release":
        # SET DOWN FROM ABOVE. The release's stand-off is the grasp's, out to the side at
        # seat height, and carrying the shell there dragged it sideways through the saddle
        # (which the planner does not see) and out of the grip. A part goes back the way
        # it came off: over its seat, then straight down.
        above = dict(blob, flange_mm=[blob["flange_mm"][0], blob["flange_mm"][1],
                                      blob["flange_mm"][2] + RELEASE_ABOVE_MM])
        order = [(above, "via"), (blob, "at")]
    for leg, what in order:
        await fly(leg, what)
        await _after(what)
        # (No re-grip. It existed for the old flag, held across its shaft; the new one is
        # held on its own axis like the pin, which seats without one - and the re-grip
        # dropped it 200 mm on the first run of the new flag, 2026-10-12.)

    verb, _, which = (tool or "").partition(":")
    if verb == "grab" and which == "Flag" and cell is not None:
        # SET THE FLAG BACK IN ITS NEST before looking. A 60 g part resting in a nest is
        # thrown 10-30 mm by whatever re-reads the physics scene - a joint made or broken
        # anywhere in the cell, measured with no arm moving, twice (2026-10-12). Set back
        # from 5 mm (from 1 mm it is thrown) and given half a second, it lies at 0.2 mm.
        # (Now it waits KINEMATIC, exactly at its pick - restock_all - so it needs no
        # setting back; it is made dynamic as the jaws close, below.)
        from assembly.fixture import (FLAG_KINEMATIC_LIFT_MM as _lift,
                                      feed_orientation as _orient, feed_spot_mm as _spot)
        _at = list(_spot(cell, "Flag"))
        _at[2] += _lift
        await world.do_command({"command": "set_kinematic", "name": "flag", "on": True,
                                "position_mm": [float(v) for v in _at],
                                "orientation_wxyz": [float(v) for v in _orient("Flag")]})
        await asyncio.sleep(0.2)
    if verb == "grab" and cell is not None and which in ("Flag", "Pin", "Shell"):
        # Look before reaching. This leg is deliberately NOT part of the taught program:
        # `fly_leg(None, ...)` plans it and records nothing, so it consumes no step number
        # and a cycle that skips it (part already within the floor) stays in step with a
        # cycle that does not. A correction leg that was taught would have to be taught at
        # one offset and replayed at another, which is the opposite of the point.
        off, here, nominal = await measured_grasp(world, which, cell,
                                                  blob["belt_index_x_mm"])
        far = math.dist(here, nominal)
        if far > PICK_CORRECT_CEILING_MM:
            raise RuntimeError(
                f"{pose}: {which}'s grasp point is {far:.1f} mm from where the carrier "
                f"should have presented it ({['%.1f' % v for v in here]} against "
                f"{['%.1f' % v for v in nominal]}). That is not drift, so this is a part "
                f"that has left its slot and the gripper is not being sent after it.")
        # Not for the PIN: it is stood square at its nominal spot as the jaws close (see
        # the grab below), so chasing its lean first left the jaw 2.7 mm off the pin's
        # axis - and that offset came out of the insert as 5.6 mm across the bore.
        if which == "Shell" and which in AIM:
            off = list(AIM[which]["offset_mm"])
            print(f"    {pose}: the learner's point is {math.dist(off, [0, 0, 0]):.1f} mm "
                  f"from the cell's own ({['%+.1f' % v for v in off]})", flush=True)
            far = float("inf")
        if far > PICK_FLOOR_FOR.get(which, PICK_CORRECT_FLOOR_MM) and which != "Pin":
            f = blob["flange_mm"]
            print(f"    {pose}: {which} is {far:.1f} mm off nominal "
                  f"({['%+.1f' % v for v in off]}); correcting", flush=True)
            await fly_leg(None, arms, motion, arm,
                          [f[i] + off[i] for i in range(3)],
                          orientation_for(blob["approach"], blob.get("theta_deg", 0.0)), state,
                          label=f"{pose}:correct")
    if verb == "grab":
        # Nothing to unclip: the tray parts are PLACED on their slots after each index
        # rather than jointed down, because a carried part drifts - the flag crept 5.7 mm
        # over one index, which against its own O10 grasp shaft is the difference between
        # a grip and a miss - and every way of correcting it with a joint failed. See
        # `settle_tray` for what was tried.
        #
        # SQUARE THE PIN AS THE JAWS CLOSE. Three jaws closing on a collar stand a pin on
        # the tool's axis; the simulated grip freezes whatever lean it had, and the pin
        # leans 1.3 to 1.8 degrees in its nest's slack. Past 1.8 the insert turned the tool
        # to match, which threw the tip 6.4 mm off the bore's mouth and the stroke finished
        # 5.4 mm across it. Stood upright a breath above its seat the instant before the
        # grab, it is held square and the insert has nothing to correct.
        if which == "Pin" and cell is not None:
            # On the LIFTED seat, where the lifter presents it - not `feed_spawn`, which is
            # the magazine's lowered slot: placed there the pin went into the raised rod and
            # sat 16.5 mm low, under the reach of the jaws (2026-10-12).
            from assembly.fixture import RESTOCK_DROP_MM, feed_orientation, feed_spot_mm
            where = list(feed_spot_mm(cell, "Pin"))
            where[2] += RESTOCK_DROP_MM
            quat = feed_orientation("Pin")
            await world.do_command({"command": "place_prop", "name": "pin",
                                    "position_mm": [float(v) for v in where],
                                    "orientation_wxyz": [float(v) for v in quat]})
        # THE FLAG IS GRIPPED WHILE STILL KINEMATIC, and only then let go to physics. It
        # waits FLAG_KINEMATIC_LIFT_MM above its nest; made dynamic first "in the same
        # breath as the grab", it fell and tipped in whatever time the two calls took apart
        # - 38 degrees, held 15 mm off, refused at the insert (the learner's cell, whose
        # calls loop back through viam-server, 2026-10-18; the film's never showed it).
        await grips[arm].grab()
        # 1.5 s before asking whether it caught anything, not 0.8. A surface gripper does
        # not attach on the step the action is applied - it raycasts, finds a body and
        # fabricates a joint - and every measurement that showed these grabs holding
        # waited 1.5 s. At 0.8 arm-c reported nothing gripped with the shell's wall 15.2 mm
        # away and well inside its 20 mm reach, which is the gate reading an answer the
        # simulator had not finished giving.
        await asyncio.sleep(1.5)
        if which == "Flag":
            caught = (await grips[arm].do_command({"command": "status"})).get("gripped") or []
            await world.do_command({"command": "set_kinematic", "name": "flag", "on": False})
            if not caught:
                # A gripper that will not attach to a kinematic body: the old order.
                print(f"    {pose}: nothing caught while the flag was kinematic; "
                      f"grabbing it dynamic", flush=True)
                await grips[arm].open()
                await grips[arm].grab()
                await asyncio.sleep(1.5)
        # A GRAB THAT HOLDS NOTHING IS A FAILED STEP, and has to say so here.
        #
        # Nothing checked this, and two of the cell's three picks were holding air: the
        # pin's cup landed exactly on the pin's end face with nothing to reach for, and
        # arm-c's landed 0.16 mm inside the shell's wall. Both "passed" every run, because
        # the motion completed and the seating check that followed measured a part the
        # `join` had snapped onto its published frame - which happens whether or not an
        # arm ever carried it. The gripper knows what it caught; ask it.
        held = (await grips[arm].do_command({"command": "status"})).get("gripped") or []
        if not held:
            raise RuntimeError(
                f"{pose}: the gripper closed on nothing - it reports no gripped object, "
                f"so whatever happens next is happening to a part this arm is not "
                f"holding. {await _why_nothing(world, arm, which, blob.get('work_mm'))}")
        # WHICH WAY THE PART IS POINTING, now that something is holding it.
        #
        # The pin's 78.26 mm miss was explained by a 90 degree wrist rotation between the
        # pick and the insert, and that explanation is a HYPOTHESIS: the stand-off numbers
        # only account for about 18 mm of it, so the rest is either a rotation plus a jam
        # or something not yet named. This cell has already published one number that was
        # measuring something other than what it claimed, so the way to settle it is to
        # read the part's own axis here and again before the stroke rather than to argue
        # from arithmetic.
        await say_axis(world, which, f"{pose}: held")
        # And WHERE it is in the tool, which is what the insert needs and what no
        # stand-off sum can supply.
        HELD[which] = await in_tool(world, arm, which)
        # A FLAG TIPPED IN THE GRAB IS PUT BACK AND TAKEN AGAIN. Now and then the flag,
        # released from kinematic as the jaws close, is thrown 9 mm up and 38 degrees over
        # and caught there (199, 11, -3 mm in the flange's frame instead of 209.9, 0, 0),
        # and the insert then refuses it with "moved 24 mm IN THE TOOL", which stops the
        # learner's run: 1 grab in 23 on a whole line (seed 2966, 2026-10-19), every grab
        # on a flag-scoped batch before 84e3293. The cause is in the physics and costs
        # nothing to step round: the arm is still at the pick, so open, set the flag back
        # in its nest and grab again.
        if which == "Flag" and cell is not None:
            for retry in range(FLAG_REGRABS):
                off = math.dist(HELD[which], FLAG_HELD_MM)
                if off <= FLAG_HELD_TOLERANCE_MM:
                    break
                print(f"    {pose}: the flag tipped in the grab ({off:.1f} mm from a good "
                      f"hold); setting it back and grabbing again", flush=True)
                await grips[arm].open()
                await asyncio.sleep(0.5)
                await world.do_command({"command": "set_kinematic", "name": "flag", "on": True,
                                        "position_mm": [float(v) for v in _at],
                                        "orientation_wxyz": [float(v) for v in _orient("Flag")]})
                await asyncio.sleep(0.5)
                await grips[arm].grab()
                await asyncio.sleep(1.5)
                await world.do_command({"command": "set_kinematic", "name": "flag", "on": False})
                HELD[which] = await in_tool(world, arm, which)
        # And SAY it, against what the station assumed. A station models the part as
        # hanging a fixed distance below the flange; the surface gripper attaches to
        # whatever its ray actually hit. If those disagree the insert is off by the
        # difference every single cycle, and no amount of aiming at the hole will find
        # it - it looks like the part being in the wrong place rather than the model
        # being wrong about where the tool is holding it.
        print(f"    {pose}: holding {which} at {['%+.2f' % v for v in HELD[which]]} mm "
              f"in the flange's frame", flush=True)
        if which == "Shell":
            # UP, not back along the approach. The shell is gripped by its side wall, so
            # its approach is sideways: retreating along it dragged the shell 139 mm out
            # through the saddle's lips, and it could not be set back - it tipped onto its
            # side on release. Straight up out of the saddle, as `return` sets it down.
            up = [blob["flange_mm"][0], blob["flange_mm"][1],
                  blob["flange_mm"][2] + RELEASE_ABOVE_MM]
            await fly_leg(legs, arms, motion, arm, up,
                          orientation_for(blob["approach"], blob.get("theta_deg", 0.0)),
                          carry_state or state, label=f"{pose}:lift",
                          constraints=straight_line())
        elif via is not None:
            # Lift the part clear before anything else moves - STRAIGHT along the tool
            # axis. The pin comes up out of a 50 mm bore with the tool rigidly joined to
            # it; a planned arc would drag the nest's bore wall with it.
            await fly_leg(legs, arms, motion, arm, via["flange_mm"],
                          orientation_for(via["approach"], via.get("theta_deg", 0.0)), state,
                          label=f"{pose}:lift", constraints=straight_line())
        return
    if verb == "release":
        # Put it down and let go, in that order - the move into `pose` is the placement.
        await let_go(grips[arm], f"{pose} release")
        if via is not None:
            await fly(via, "off")
        if arm == "arm-c" and via is not None:
            # And UP, clear of the shell's crown, so the park that follows cannot sweep it.
            up = [via["flange_mm"][0], via["flange_mm"][1],
                  via["flange_mm"][2] + REJECT_CLEAR_UP_MM]
            await fly_leg(legs, arms, motion, arm, up,
                          orientation_for(via["approach"], via.get("theta_deg", 0.0)),
                          state, label=f"{pose}:up", constraints=straight_line())
        return
    if verb == "seat":
        # The published travel exactly: an approach clearance plus the engagement, into a
        # depth the asset publishes. (NOT a hard stop - there is no stop here: pressing
        # 8 mm past nominal put the pin 6.04 mm PAST its seat rather than against
        # anything, which is how that assumption was found and removed.) Then the joint
        # the asset says holds it - made while
        # the part is still gripped, so it is fastened in the pose the arm put it in rather
        # than in whatever pose it falls into between release and fastening - and only then
        # the release.
        #
        # One path for all three parts. What differs between a snapped flag, a pressed pin
        # and a hinged door is entirely in what the asset publishes: the axis, the depth,
        # the kind of joint and, for the hinge, its axis and limits.
        ins = insertions()[which]
        axis, depth = ins.axis, ins.depth_mm
        # THE SHELL HAS TO BE SQUARE BEFORE ANYTHING IS PRESSED INTO IT.
        #
        # The bore was measured facing (-0.01, +0.97, -0.26) at the pin index - the shell
        # sitting about 15 degrees tilted in its cradle. A pin driven 138 mm into a bore
        # tilted like that cannot go in; it levers, and a position-controlled arm levering
        # a 2.35 kg shell is how the shell left the fixture. It also defeats the camera
        # correction that is meant to replace the carrier's accuracy: you cannot aim at a
        # target whose axis you have not checked.
        #
        # 2 degrees is the gate. Over the pin's 138 mm stroke that is 4.8 mm at the far
        # end, already past what the tip's 1.5 mm capture can swallow, so anything beyond
        # it is a fixture problem to see rather than a pose to correct.
        # 10 degrees to FAIL, 2 to warn. The shell has always sat a little out of square
        # on its narrow rails, and a gate that refuses at 2 stops the cell demonstrating
        # anything while that is worked on. What a gate has to prevent is a part being
        # pressed into a bore pointing somewhere else entirely; what it should not do is
        # hide a known, measured, bounded error behind a refusal.
        tilt = await _shell_tilt_deg(world)
        if tilt > 10.0:
            raise RuntimeError(
                f"{pose}: the shell is {tilt:.1f} degrees out of square in its cradle. "
                f"The insert is re-referenced to the shell's measured frame, so a few "
                f"degrees is corrected rather than refused - but at this angle the part "
                f"is not sitting in its cradle at all, and the correction would aim a "
                f"stroke at a target that is about to move.")
        # RE-REFERENCE THE WHOLE INSERT TO THE SHELL AS IT ACTUALLY SITS.
        #
        # The published insert axis is in the SHELL's frame - the asset's own header says
        # the parts' axes are parallel to the shell's when seated - and this used it as a
        # world axis, which is only the same thing when the shell is square. It is not: it
        # comes to rest about 4 degrees out on its narrow rails, and over the pin's 138 mm
        # stroke 4 degrees is 10 mm at the far end. The chamfer captures 1.5.
        #
        # So rather than refuse a known, measured, bounded error, transform the taught pose
        # into the frame the shell is really in: the pre-insert flange, the tool's
        # orientation and the stroke direction all go through the same rotation, which is
        # what keeps the pin COAXIAL with the bore rather than merely pointed at its mouth.
        # A lateral-only correction cannot do that, and a tilted pin in a 0.20 mm bore
        # levers rather than enters - which is how the shell left the fixture twice.
        #
        # The pivot is the shell's own origin and the nominal frame is axis-aligned at the
        # station's belt index, which is exactly how `fixture.indexed_mm` places the shell -
        # so the two cannot drift apart.
        sh = (await world.do_command({"command": "prim_poses",
                                      "paths": ["/World/shell"]}))["prims"]["/World/shell"]
        rot = [sh["x_axis"], sh["y_axis"], sh["z_axis"]]
        live = sh["position_mm"]
        nom = (blob["belt_index_x_mm"], cell.belt_y_mm, cell.shell_origin_mm[2])

        f = blob["flange_mm"]
        at_f = into_frame(rot, live, [f[i] - nom[i] for i in range(3)], point=True)
        at_app = into_frame(rot, live, blob["approach"], point=False)
        at_axis = into_frame(rot, live, axis, point=False)
        moved = math.dist(at_f, f)
        if moved > 0.5:
            # Not part of the taught program: planned, recorded nowhere, consuming no step
            # number, so a cycle that needs no correction stays in step with one that does.
            print(f"    {pose}: shell {tilt:.1f} deg out and offset; re-aiming the insert "
                  f"{moved:.1f} mm onto its measured frame", flush=True)
            await fly_leg(None, arms, motion, arm, at_f, orientation_for(at_app, blob.get('theta_deg', 0.0)), state,
                          label=f"{pose}:aim")
        # AND LINE THE PART UP, by measuring where it is rather than booking where it
        # should be. This is the last place this cell books a number, and it is the one
        # that cost the most.
        #
        # A station places the FLANGE: `work + approach * (standoff + extra)`, a scalar
        # along one axis. The thing that has to arrive somewhere is the PART, and the
        # flange-to-part relationship is whatever the grab happened to establish. Those two
        # are reconcilable only when the part's grasp point sits on the station's own axis,
        # and on this cell neither part's does - the pin's grasp is 11.5 mm along +Y from
        # its origin and the flag's is 206.7 mm from its own, so no scalar stand-off can
        # express either. Written out, the books did not even agree: the pick leaves the
        # flange 155 mm from the pin's GRASP point and the seat assumed 141 mm from its
        # ORIGIN.
        #
        # So the tool goes to the taught pose, the part is measured where it actually is,
        # and the flange steps across the difference. One planned move, outside the taught
        # program, and every stand-off above becomes an approximation that only has to be
        # close rather than right.
        # SQUARE FIRST, THEN ALIGN. The order matters and I had it backwards.
        #
        # A part held at an angle presents its feature in the wrong PLACE as well as the
        # wrong direction, and the two are not independent: the flag's peg is 32 mm from
        # its grasp, so 7 degrees of rotation in the grip displaces the peg 3.9 mm. Trying
        # to fix that by translating cannot work - it drives the part sideways into the
        # shell while the rotation it is compensating for stays - and measured, that is
        # exactly what happened. The align diverged, 3.96 -> 4.48 -> 5.64 -> 7.25 mm,
        # because the flange kept moving and the part had stopped against something.
        #
        # Rotation first takes most of the feature's displacement out with it, and leaves
        # the align the small residual it was designed for.
        try:
            feature = [v * 1000.0 for v in part(which).at("assembly:pegTip")]
        except Exception:                                          # noqa: BLE001
            feature = [0.0, 0.0, 0.0]

        async def _feature_now():
            g = (await world.do_command({
                "command": "prim_poses",
                "paths": [f"/World/{which.lower()}"]}))["prims"][f"/World/{which.lower()}"]
            ax = [g["x_axis"], g["y_axis"], g["z_axis"]]
            return g, [g["position_mm"][i] + sum(ax[k][i] * feature[k] for k in range(3))
                       for i in range(3)]

        want_pre = into_frame(rot, live,
                              [ins.pre_mm[i] + feature[i] for i in range(3)], point=True)
        aim = AIM.get(which)
        if aim is not None:
            # ACROSS THE AXIS ONLY. How deep the learner's point sits along the insert is
            # what the stroke decides; shifting the stroke's start by it would drive the
            # part that much past its seat.
            o = aim["offset_mm"]
            o_al = sum(o[i] * at_axis[i] for i in range(3))
            across_o = [o[i] - o_al * at_axis[i] for i in range(3)]
            want_pre = [want_pre[i] + across_o[i] for i in range(3)]
            print(f"    {pose}: aiming at the learner's point, "
                  f"{math.dist(across_o, [0, 0, 0]):.1f} mm across the axis from the "
                  f"cell's own", flush=True)

        # PLACE, THEN SQUARE, THEN PLACE AGAIN. The order is empirical and the evidence is
        # six runs against two.
        #
        # Squaring first reads better - point the part at the hole, then put it on the line
        # - and it does not work: the pin seated 0.00 mm six times with the place first and
        # finished 5.68 and 5.69 mm short of depth on the two runs with the square first,
        # with the flange completing its stroke both times. Why a rotation taken 180 mm
        # from the flange and then corrected for should leave the part catching is not
        # established; what is established is which order seats it.
        #
        # So: place it from the measured hold, square it if the tilt is worth correcting,
        # and place it again because turning about the flange moves it. Each step is one
        # computed move rather than a search.
        async def _place(label):
            if HELD.get(which) is None:
                return
            nonlocal at_f
            at_f, _ = await flange_that_puts(world, arm, HELD[which], want_pre)
            await fly_leg(None, arms, motion, arm, at_f, orientation_for(at_app, blob.get('theta_deg', 0.0)), state,
                          label=f"{pose}:{label}")

        await _place("place")
        got, here_feature = await _feature_now()
        slip = [want_pre[i] - here_feature[i] for i in range(3)]
        far = math.sqrt(sum(v * v for v in slip))

        moved_in_tool = await slip_mm(world, arm, which)
        if moved_in_tool > SEAT_ALIGN_CEILING_MM:
            await let_go(grips[arm], f"{pose} release, the part has moved in the tool")
            if via is not None:
                await fly(via, "off")
            raise RuntimeError(
                f"{pose}: the {which} has moved {moved_in_tool:.1f} mm IN THE TOOL since "
                f"it was grasped, past the {SEAT_ALIGN_CEILING_MM:.0f} mm a grip can slip "
                f"by. It is at {['%.1f' % v for v in here_feature]} and this insert starts "
                f"from {['%.1f' % v for v in want_pre]}, but stepping across that would be "
                f"aiming a part the tool is no longer holding where it thinks.")

        # --- point it down the hole, if the tilt is worth the move -------------------
        have = [float(v) for v in got["y_axis"]]
        want_axis = [-at_axis[i] for i in range(3)]
        tilt_deg = math.degrees(math.acos(
            max(-1.0, min(1.0, sum(have[i] * want_axis[i] for i in range(3))))))
        costs_mm = abs(depth) * math.sin(math.radians(tilt_deg))
        # SQUARE ONLY WHEN THE TILT WOULD OTHERWISE DEFEAT THE FEATURE'S OWN CAPTURE.
        #
        # Correcting a tilt is not free - it re-aims the tool, and re-aiming moves a part
        # held at one end. So it has to buy more than it costs, and the thing it buys is
        # entry: a tilt smaller than what the chamfer or the hole can swallow needs no
        # correction at all.
        #
        # The previous gate was half the published slack, and the flag's 0.50 mm cost tied
        # it exactly. It squared, its peg ended 49.22 mm off, the press that followed
        # tried to close 121.79 mm, and the arm went out of reach. Against the capture the
        # same reading does not fire: 0.50 mm into a feature that captures 2.50. The pin's
        # 2.29 mm against a 1.50 mm capture still does, and the pin seats at 0.01 mm.
        floor_mm = capture_mm(which, ins)
        # Printed whether or not it is corrected: an uncorrected tilt is what explains a
        # jam two lines later, and gating the log on the correction hid a 4 degree error
        # while its consequences were investigated.
        print(f"    {pose}: the {which} is {tilt_deg:.2f} deg off the bore's axis - "
              f"{costs_mm:.2f} mm at the far end of the stroke (floor {floor_mm:.2f} mm)",
              flush=True)
        if tilt_deg > SEAT_TILT_MAX_DEG:
            await let_go(grips[arm], f"{pose} release, the part is not held square")
            if via is not None:
                await fly(via, "off")
            raise RuntimeError(
                f"{pose}: the {which} is {tilt_deg:.1f} degrees off the bore's axis, past "
                f"the {SEAT_TILT_MAX_DEG:.0f} a grip can plausibly be out by. Turning the "
                f"tool that far would swing the part rather than aim it.")
        if costs_mm > floor_mm:
            print(f"    {pose}: turning the tool onto it", flush=True)
            # BOTH rotated by the same turn. Deriving `at_axis` from `at_app` assumes they
            # are anti-parallel, which is true of the pin - approach +Y, insert -Y - and
            # false of the flag, which stands off along +Z and inserts along -Y. That would
            # have sent the flag's stroke straight down instead of sideways.
            # The TOOL turns, so the part comes onto the bore's axis; the STROKE stays on
            # that axis. Turning the stroke by the same rotation sent it off the bore by the
            # part's lean in the grip: 1.6 degrees over 135 mm, and the shaft-held pin
            # finished 4.9, 5.4 and 6.3 mm across in three runs (2026-10-13).
            # REPEATED until it holds: one turn left the shaft-held pin 4 degrees out and
            # 1.8 at the mouth (2026-10-13). Each pass re-reads the hold and the part's axis.
            for _sq in range(SQUARE_TRIES):
                turn = turn_onto(have, want_axis)
                at_app = into_frame(turn, [0.0, 0.0, 0.0], at_app, point=False)
                HELD[which] = await in_tool(world, arm, which)
                await _place("resquare")
                got, here_feature = await _feature_now()
                have = [float(v) for v in got["y_axis"]]
                left_deg = math.degrees(math.acos(max(-1.0, min(1.0, sum(
                    have[i] * want_axis[i] for i in range(3))))))
                print(f"    {pose}: squared pass {_sq + 1}: {left_deg:.2f} deg off", flush=True)
                if left_deg <= SQUARE_DONE_DEG:
                    break
            await say_axis(world, which, f"{pose}: squared")
            got, here_feature = await _feature_now()
            slip = [want_pre[i] - here_feature[i] for i in range(3)]
            far = math.sqrt(sum(v * v for v in slip))

        # --- and nudge only if the computed pose did not land it ---------------------
        for _ in range(SEAT_ALIGN_TRIES):
            # Only what is ACROSS the axis needs a nudge: a peg short of the stroke's start
            # along the axis is what the stroke itself covers. The pin's 3.57 mm, every run,
            # was all along the axis - a planned nudge per cycle for nothing.
            _al = sum(slip[i] * at_axis[i] for i in range(3))
            _across = math.sqrt(max(far * far - _al * _al, 0.0))
            # The PIN is aligned to within half its nose's capture across the axis: the
            # flush pin, held on its shaft, leans up to 2.2 degrees in the jaw, and the
            # 3x-floor stop left it 2.7 mm across at the mouth - past the 1.8 mm the nose
            # captures - and it finished 6.3 mm across the bore (2026-10-13).
            enough = (0.5 * capture_mm(which, ins) if which == "Pin"
                      else SEAT_ALIGN_FLOOR_MM * 3)
            if far <= SEAT_ALIGN_FLOOR_MM or _across <= enough:
                break
            if which == "Flag" and _across <= FLAG_CAPTURE_ACROSS_MM:
                break   # the flag's seat joint closes this
                break
            print(f"    {pose}: the {which}'s peg is still {far:.2f} mm off the start of "
                  f"its stroke ({['%+.2f' % v for v in slip]}); nudging", flush=True)
            at_f = [at_f[i] + slip[i] for i in range(3)]
            await fly_leg(None, arms, motion, arm, at_f, orientation_for(at_app, blob.get('theta_deg', 0.0)), state,
                          label=f"{pose}:align")
            got, here_feature = await _feature_now()
            slip = [want_pre[i] - here_feature[i] for i in range(3)]
            was, far = far, math.sqrt(sum(v * v for v in slip))
            if far > was + SEAT_ALIGN_FLOOR_MM:
                print(f"    {pose}: the correction made it worse ({was:.2f} -> "
                      f"{far:.2f} mm), so the {which} is against something; stopping "
                      f"here rather than driving into it", flush=True)
                break
        if far > SEAT_ALIGN_FLOOR_MM:
            print(f"    {pose}: the {which}'s peg is {far:.2f} mm off the start of its "
                  f"stroke; stroking anyway", flush=True)
        # WHAT ELSE IS IN THE HOLE. The pin threads three knuckles, not one: the shell's
        # at y 26..66 and -26..-66, and the DOOR's at -24.5..24.5 interleaved between
        # them. The insert is re-referenced to the shell's measured frame, but the door
        # hangs on a revolute joint and its knuckles are free to sit somewhere the shell's
        # frame does not predict - so a pin that threads the shell perfectly can still
        # bind on the door. Measured, the pin travels 132.3 mm of its 138 and stops, which
        # is the bore's own length to within a third of a millimetre.
        #
        # One line, so the next jam says whether the door was where it should have been.
        if which == "Pin":
            # AT THE HINGE AXIS, not at the door's origin.
            #
            # The first version of this measured the door's origin and reported 47.98 mm.
            # That is not misalignment, it is the door being OPEN: its origin sits 199 mm
            # from the hinge axis, so 13.8 degrees of swing moves it 48 mm while leaving
            # the knuckles exactly where they were. The knuckles are ON the axis, and a
            # revolute joint keeps them concentric however far the door swings - which is
            # the whole reason the pin can be inserted with the door open.
            #
            # The same lever-arm mistake as reading the flag's insert at its origin, made
            # again two hours later. What matters is whether the joint is HOLDING, so this
            # measures the anchor itself: the hinge point, carried into world by each
            # body's own live pose. If the joint holds they coincide, whatever the angle.
            d = insertions()["Door"]
            r = (await world.do_command({
                "command": "prim_poses",
                "paths": ["/World/door", "/World/shell"]}))["prims"]
            sh2, dr2 = r["/World/shell"], r["/World/door"]
            ax2 = [sh2["x_axis"], sh2["y_axis"], sh2["z_axis"]]
            axd = [dr2["x_axis"], dr2["y_axis"], dr2["z_axis"]]
            local_d = [d.joint_anchor_mm[i] - d.seated_mm[i] for i in range(3)]
            on_shell = [sh2["position_mm"][i]
                        + sum(ax2[k][i] * d.joint_anchor_mm[k] for k in range(3))
                        for i in range(3)]
            on_door = [dr2["position_mm"][i]
                       + sum(axd[k][i] * local_d[k] for k in range(3)) for i in range(3)]
            gap = math.dist(on_shell, on_door)
            print(f"    {pose}: the hinge axis is {gap:.2f} mm apart between the shell "
                  f"and the door - the pin threads both sets of knuckles", flush=True)
        await say_axis(world, which, f"{pose}: at the mouth")
        if which == "Pin":
            # THE JAW'S CLOCK about the pin: where finger GBA_35_2 points in the pin's x-z
            # plane. The usd session cleared the inserts against the shell floor and the
            # flat door at 285 degrees (15 off straight down, toward +X).
            try:
                fp = f"/World/{arm.replace('-', '_')}/wrist_3_link/flange/jaw/SCHUNK_1582927_EZU_GBA_35_2"
                # At the finger's BOUNDS centre: its prim's origin is on the jaw's axis.
                # In the SHELL's x-z plane: the published clock is in the SEATED pin's
                # frame, which is the shell's. The held pin turns with the jaw, so its own
                # frame read the same angle whatever the roll.
                g = (await world.do_command({"command": "prim_poses",
                                             "paths": ["/World/pin", "/World/shell"]}))["prims"]
                fb = (await world.do_command({"command": "prim_bounds",
                                              "paths": [fp]}))["bounds"][fp]
                pn, sh_ = g["/World/pin"], g["/World/shell"]
                d = [fb["centre"][i] * 1000.0 - pn["position_mm"][i] for i in range(3)]
                px = sum(d[i] * sh_["x_axis"][i] for i in range(3))
                pz = sum(d[i] * sh_["z_axis"][i] for i in range(3))
                print(f"    {pose}: jaw clock {math.degrees(math.atan2(pz, px)) % 360:.0f} deg "
                      f"(finger GBA_35_2 in the pin's x-z plane; cleared at 285)", flush=True)
            except Exception as exc:                               # noqa: BLE001
                print(f"    {pose}: jaw clock unread ({str(exc)[:80]})", flush=True)
        # THE NOMINAL STROKE. What is left after it is closed by measurement below.
        #
        # The flange completes its full stroke every time and the pin arrives short - 5.68,
        # 5.69, 3.89 mm - which is the part lagging the tool, not the tool stopping. A Ø4
        # pin through three interleaved knuckles at 0.20 mm of slack accumulates friction
        # over 138 mm, and the grip that carries it is a fabricated joint with finite
        # stiffness: it stretches.
        #
        # Pressing a fixed 8 mm past nominal was tried and backed out. It confirmed the
        # lag and disproved the mechanism: the pin went from 3.89 and 5.69 mm short to
        # 6.04 mm PAST, so there is no hard stop to catch an over-press - I had asserted
        # one without checking. And the lag is not a constant to press against anyway:
        # 2.0 mm on that run against 5.7 mm earlier. What closes it is the measured press
        # below.
        # STROKE TO THE SEAT, from where the part actually is along the axis. The stroke
        # was a fixed `depth` from the flange's start, and the pin starts it 10 mm deeper
        # than the station models, every run - so it was driven 3 to 9 mm PAST its seat,
        # into the knuckle, and the press then pushed it across the bore (2026-10-12).
        along = sum(slip[i] * at_axis[i] for i in range(3))
        if aim is not None and aim.get("missed"):
            await _show_miss(world, arms, grips, motion, arm, pose, which, stamp, rot, live,
                             nom, at_f, at_app, at_axis, blob, aim, here_feature, state,
                             carry_state, ins.engage_mm)
            return
        # A FLUSH PIN cannot be carried all the way in: it is held on its shaft just under
        # the head, so the jaw lets go `assembly:releaseBeforeSeat` short (insert tips
        # 3 mm off the knuckle) and the closed tips press the head home (below).
        short_by = 0.0
        pressed_home = False
        if which == "Pin" and PIN_RELEASE_AND_PRESS:
            try:
                short_by = float(part("Pin").at("assembly:releaseBeforeSeat")) * 1000.0
            except Exception:                                      # noqa: BLE001
                short_by = 0.0
        print(f"    {pose}: stroking {depth + along - short_by:.1f} mm ({depth:.1f} nominal, "
              f"{along:+.1f} for where the {which} starts, {short_by:.1f} left to press)",
              flush=True)
        end = [at_f[i] + at_axis[i] * (depth + along - short_by) for i in range(3)]
        # The stroke and the press plan WITHOUT the held part as an obstacle and without
        # it attached: it is going into the bore, which an attached part would collide
        # with, and left in the scene it stood where it hangs in the jaws - measured,
        # "violation between arm-b:ee_link and pin.pin" refused every stroke (2026-10-12).
        st_in = await declared(world, moving=arm, motion=motion, grip=grips.get(arm),
                               attach=False)
        await fly_leg(None, arms, motion, arm, end, orientation_for(at_app, blob.get('theta_deg', 0.0)), st_in,
                      label=f"{pose}:stroke", constraints=straight_line())
        # DID THE ARM GET THERE, and did the part come with it?
        #
        # Those are two different failures that a single seat measurement cannot tell
        # apart. If the flange is where it was sent and the part is short, the part slipped
        # in the gripper or jammed in the bore; if the flange itself stopped short, the arm
        # ran out of push and no amount of aiming will fix it. One line, and it costs a
        # pose read.
        try:
            now = await arms[arm].get_end_position()
            short = math.dist([now.x, now.y, now.z], end)
            if short > 0.5:
                print(f"    {pose}: the flange stopped {short:.2f} mm short of the end of "
                      f"its stroke", flush=True)
        except Exception:                                          # noqa: BLE001
            pass
        await asyncio.sleep(0.8)
        if short_by > 0.0:
            # LET GO, BACK OFF, CLOSE, PRESS (the usd session's sequence, 2026-10-13). The
            # closed insert tips are `insert:tipZ` out from the flange and the head's top
            # face is `PIN_TIP_TO_HEAD_MM` nearer than that while held, so backing off by
            # that plus a millimetre puts the tips 1 mm off the head. The tools have no
            # colliders, so the press is the surface gripper re-taking the pin by its head
            # (its 65 mm attach ray reaches the tips) and carrying it the rest of the way.
            await let_go(grips[arm], f"{pose} release, {short_by:.0f} mm short of seated")
            # From the MEASURED hold where there is one: HELD is the pin's origin (its
            # head's centre) in the flange frame, its first component along the tool.
            gap = PIN_TIP_TO_HEAD_MM
            if HELD.get("Pin") is not None:
                gap = PIN_INSERT_TIP_MM - (float(HELD["Pin"][0]) - 0.75)
            print(f"    {pose}: insert tips {gap:.1f} mm past the head; backing off "
                  f"{gap + 1.0:.1f}", flush=True)
            back = [end[i] - at_axis[i] * (gap + 1.0) for i in range(3)]
            await fly_leg(None, arms, motion, arm, back, orientation_for(at_app, blob.get('theta_deg', 0.0)), st_in,
                          label=f"{pose}:backoff", constraints=straight_line())
            # What the attach ray will meet, measured: from the flange along the tool axis.
            fl = f"/World/{arm.replace('-', '_')}/wrist_3_link/flange"
            fg = (await world.do_command({"command": "prim_poses", "paths": [fl]}))["prims"][fl]
            ray = (await world.do_command({"command": "raycast", "from_mm": [
                fg["position_mm"][i] + fg["x_axis"][i] * 120.0 for i in range(3)],
                "along": fg["x_axis"], "distance_mm": 120.0}))["ray"]
            head_at = ray.get("distance_mm", 0.0) + 120.0
            print(f"    {pose}: down the tool axis from 120 mm: "
                  f"{ray.get('body') if ray.get('hit') else 'nothing'} at "
                  f"{head_at:.1f} mm from the flange", flush=True)
            # Close the tips onto the head by what the ray MEASURED, not by the hold taken
            # at the pick: the pin sat 11.9 mm further out by the time it reached the bore,
            # and backing off by the booked hold left the head 205 mm out, past the grip's
            # reach (2026-10-13).
            if ray.get("hit") and str(ray.get("body", "")).startswith("/World/pin"):
                step = head_at - PIN_INSERT_TIP_MM - 1.0
                if abs(step) > 0.5:
                    back = [back[i] + at_axis[i] * step for i in range(3)]
                    await fly_leg(None, arms, motion, arm, back, orientation_for(at_app, blob.get('theta_deg', 0.0)),
                                  st_in, label=f"{pose}:to_head",
                                  constraints=straight_line())
            # THE PRESS. The tools have no colliders, so the closed tips cannot push; and
            # re-taking the pin with the surface gripper threw it 49 mm forward as the joint
            # formed (2026-10-13). So the pin is made kinematic where it lies and carried
            # along the axis in step with the flange, then let go at the seat.
            _m, left, _a = await seat_miss(world, which, ins)
            pp = (await world.do_command({"command": "prim_poses",
                                          "paths": ["/World/pin"]}))["prims"]["/World/pin"]
            pq = _quat_from_axes(pp["x_axis"], pp["y_axis"], pp["z_axis"])
            p0 = list(pp["position_mm"])
            total = max(0.0, -left)
            print(f"    {pose}: tips on the head, {total:.1f} mm to press", flush=True)

            async def _pin_at(d):
                await world.do_command({"command": "set_kinematic", "name": "pin", "on": True,
                                        "position_mm": [p0[i] + at_axis[i] * d
                                                        for i in range(3)],
                                        "orientation_wxyz": pq})
            await _pin_at(0.0)
            f0 = await arms[arm].get_end_position()
            end = [back[i] + at_axis[i] * total for i in range(3)]
            go = asyncio.create_task(fly_leg(
                None, arms, motion, arm, end,
                orientation_for(at_app, blob.get('theta_deg', 0.0)), st_in,
                label=f"{pose}:press_head", constraints=straight_line()))
            while not go.done():
                fn = await arms[arm].get_end_position()
                d = sum(([fn.x, fn.y, fn.z][i] - [f0.x, f0.y, f0.z][i]) * at_axis[i]
                        for i in range(3))
                await _pin_at(min(total, max(0.0, d)))
                await asyncio.sleep(0.05)
            await go
            await _pin_at(total)
            await asyncio.sleep(0.2)
            # SEAT IT BEFORE LETTING GO. Made dynamic again inside three bores it was thrown
            # 15 mm back out and 4.9 mm across (2026-10-13). Fastened to the shell first, the
            # pair no longer collide, and only then does it stop being kinematic.
            _m, along_k, across_k = await seat_miss(world, which, ins)
            print(f"    {pose}: pressed to {abs(along_k):.2f} mm "
                  f"{'past' if along_k > 0 else 'short of'} depth, {across_k:.2f} across; "
                  f"fastening", flush=True)
            await world.do_command({
            "command": "join", "name": f"{which.lower()}_seat",
            "body0": "/World/shell", "body1": f"/World/{which.lower()}",
            "kind": ins.joint, "release_kinematic": True,
            "local0_mm": list(ins.joint_anchor_mm),
            "local1_mm": [ins.joint_anchor_mm[i] - ins.seated_mm[i] for i in range(3)],
            "axis": list(ins.joint_axis_mm), "limits_deg": list(ins.joint_limits_deg),
            **({"drive": {k: v for k, v in ins.joint_drive.items()
                          if not k.startswith("_")}} if ins.joint_drive else {})})
            await asyncio.sleep(0.4)
            pressed_home = True
        # PRESS IT HOME BY WHAT IS LEFT, measured, rather than by a guessed over-stroke.
        #
        # The part lags the tool: the flange completes its full stroke every time and the
        # pin arrived 5.68, 5.69 and 3.89 mm short, because a Ø4 pin through three
        # interleaved knuckles at 0.20 mm of slack accumulates friction and the grip that
        # carries it is a fabricated joint with finite stiffness. Pressing a fixed 8 mm
        # past nominal did get it home - and 6.04 mm PAST, because the lag is not a
        # constant and there is no hard stop to catch an over-press.
        #
        # So the same answer as everywhere else in this station: measure the gap and close
        # it. A few short moves along the insert axis, each one the size of what is
        # actually left, which converges whether the part is short or long.
        # WHAT THE STROKE ALONE ACHIEVED, before any press. The press can only be judged
        # against this: a part that arrives crooked was not made crooked by pressing.
        _m0, along0, across0 = await seat_miss(world, which, ins)
        print(f"    {pose}: the stroke alone left the {which} {abs(along0):.2f} mm "
              f"{'past' if along0 > 0 else 'short of'} depth and {across0:.2f} mm across",
              flush=True)
        was_across = None
        for _ in range(0 if pressed_home else 1 if which == "Flag" else SEAT_PRESS_TRIES):
            _m, along, across = await seat_miss(world, which, ins)
            if abs(along) <= ins.clearance_mm:
                break
            # STOP IF PRESSING IS MAKING IT WORSE ACROSS THE BORE.
            #
            # A press should drive the part along the axis and leave the other two alone.
            # When the part is binding it does not: the force becomes a moment about
            # whatever is holding the part, and the part rotates instead of moving. The
            # flag's peg is 27 mm from its grasp, so that moment has plenty of leverage -
            # measured on the replay, three presses took the along error from 6.86 to 3.86
            # to 3.45 mm while the ACROSS error grew to 6.98, and the insert ended further
            # from its seat than one press had left it.
            #
            # So the gate is not just "is it still short" but "is the press still the
            # right tool". Pressing a part that is going sideways is how a 0.45 mm seat
            # becomes a 7.79 mm one.
            if was_across is not None and across > was_across + ins.clearance_mm:
                print(f"    {pose}: pressing is pushing the {which} across the bore "
                      f"({was_across:.2f} -> {across:.2f} mm), not into it; stopping",
                      flush=True)
                break
            was_across = across
            if abs(along) > abs(depth):
                # A gap larger than the whole stroke is not a part that needs pressing,
                # it is a part that is not in the hole. Pressing it drives the arm at
                # somewhere nobody chose: measured, a 121.79 mm "shortfall" became a
                # 121.79 mm command and "zero IK solutions produced, goal positions
                # appears to be physically unreachable".
                print(f"    {pose}: {abs(along):.1f} mm from depth is more than the "
                      f"{abs(depth):.0f} mm stroke itself - the {which} is not in the "
                      f"hole, so it is not being pressed", flush=True)
                break
            print(f"    {pose}: {abs(along):.2f} mm "
                  f"{'past' if along > 0 else 'short of'} depth and {across:.2f} mm "
                  f"across; pressing it home", flush=True)
            # From where the flange IS: after the pin's re-take the hold is not the one
            # `at_f` was computed for.
            _now = await arms[arm].get_end_position()
            try:
                await fly_leg(None, arms, motion, arm,
                              [[_now.x, _now.y, _now.z][i] - at_axis[i] * along
                               for i in range(3)],
                              orientation_for(at_app, blob.get('theta_deg', 0.0)), st_in, label=f"{pose}:press",
                              constraints=straight_line())
            except Exception as exc:                               # noqa: BLE001
                # A press the planner refuses ends the pressing; it does not fail the
                # insert. The gate below decides whether what is there gets seated.
                print(f"    {pose}: no press ({str(exc)[:100]})", flush=True)
                break
            await asyncio.sleep(0.4)
        # MEASURE THE INSERTION BEFORE THE JOINT, because the joint can hide the answer.
        #
        # `join` is strong enough to drag a part into place - the door sweep measured it
        # closing a 124 mm gap by throwing the door down the cell at 400 mm/s - so a
        # `verify_seated` taken after it is measuring the joint, not the insert. This cell
        # has been fooled by exactly that before: `stations.py` records a pin reported
        # "seated 0.00 mm from its published position" while the pin had never moved,
        # because the arm missed the tray entirely and the join snapped it home anyway.
        #
        # So the honest order is: stroke, MEASURE, and only then fasten. A part that
        # finished outside what the feature can capture is a failed insert and says so,
        # with the number - and crucially it is released rather than fastened, because a
        # joint made across a gap pulls on the shell, and a 2.35 kg shell pulled on is the
        # cell's datum gone for every station after this one.
        miss, along, across = await seat_miss(world, which, ins)
        # Judged per component, not on the sum. Across the bore is what the capture radius
        # is about; a part that went in straight and stopped a fraction short is what the
        # joint exists to finish. Summed, a pin 0.43 mm short and 1.77 across - inside the
        # 1.80 capture - read 1.83 and was thrown away.
        # The FLAG is given the joint's help more generously. Its hold creeps out of
        # square while it hangs from the jaw (a 60 g blade 139 mm from its grip), and the
        # last press is refused by the planner over the crown. A fixed seat joint closing
        # the last few millimetres is nothing anyone sees on film; a failed cycle is.
        cap_across = max(capture_mm(which, ins),
                         FLAG_CAPTURE_ACROSS_MM if which == "Flag" else PIN_CAPTURE_ACROSS_MM)
        cap_along = FLAG_SEAT_ALONG_MM if which == "Flag" else SEAT_ALONG_MM
        if across > cap_across or abs(along) > cap_along:
            await let_go(grips[arm], f"{pose} release after a missed seat")
            if via is not None:
                await fly(via, "off")
            raise RuntimeError(
                f"{pose}: {which} finished {miss:.2f} mm from its seated position - "
                f"{abs(along):.2f} mm {'past' if along > 0 else 'short of'} full depth and "
                f"{across:.2f} mm across the bore - past the "
                f"{capture_mm(which, ins):.2f} mm this feature can capture (published "
                f"slack {ins.clearance_mm:.2f} mm). That is a failed insertion, and it is "
                f"being reported rather than fastened: a joint made across that gap would "
                f"pull the shell out of square and every station after this one works off "
                f"the shell.")
        if not pressed_home:
            await world.do_command({
                "command": "join", "name": f"{which.lower()}_seat",
                "body0": "/World/shell", "body1": f"/World/{which.lower()}",
                "kind": ins.joint,
                "local0_mm": list(ins.joint_anchor_mm),
                "local1_mm": [ins.joint_anchor_mm[i] - ins.seated_mm[i] for i in range(3)],
                "axis": list(ins.joint_axis_mm), "limits_deg": list(ins.joint_limits_deg),
                **({"drive": {k: v for k, v in ins.joint_drive.items()
                              if not k.startswith("_")}} if ins.joint_drive else {})})
        await asyncio.sleep(0.6)
        await let_go(grips[arm], f"{pose} release")
        # Back the tool off BEFORE waiting for the door to swing. The cup is still against
        # the panel it just placed, so a door asked to close against it settles at whatever
        # angle the gripper is holding it at - measured, 29.7 degrees, and stable there for
        # the full six seconds because nothing was going to move.
        if via is not None:
            await fly(via, "off")
        if ins.joint == "revolute":
            await settle_hinge(world, which, ins)
        await verify_seated(world, which, ins)
        return
    if via is not None:
        await fly(via, "off")


# The collision audit (`--audit`): every arm and its tool, against everything else.
# The parts an arm carries are targets too: the pin hangs 138 mm under the jaw, and what it
# sweeps through is the arm's doing. Every 6th step (10 Hz): every step took the sim from
# 0.56 to 0.09 of real time, and the belt's 25 s index timeout then fired on a pallet that
# was only slow (2026-10-13). (Each arm's base sits on its pedestal: ignored.)
# (An audit REPLAY runs at every 30th step, 2 Hz: at 10 Hz the sim slowed until the taught
# pin pick timed out and the replay was measuring a broken cycle.)
AUDIT_EVERY_N = int(os.environ.get("AUDIT_EVERY_N", "30"))
AUDIT_START = {"command": "audit", "action": "start", "every_n": AUDIT_EVERY_N,
               "targets": [{"name": n, "root": f"/World/{n.replace('-', '_')}",
                            "ignore": [f"/World/pedestal_{n.replace('-', '_')}"]}
                           for n in ("arm-a", "arm-b", "arm-c")]
               + [{"name": "pin", "root": "/World/pin",
                   "ignore": ["/World/arm_b", "/World/pin_magazine"]},
                  {"name": "flag", "root": "/World/flag",
                   "ignore": ["/World/arm_b", "/World/flag_feed"]}]
               # And what rides the belt, so a stalled index names what it ran into.
               + [{"name": n, "root": f"/World/{n}",
                   "ignore": ["/World/belt"] + [f"/World/{m}" for m in ("pallet", "shell",
                                                                         "door", "pin", "flag")
                                                if m != n]}
                  for n in ("pallet", "shell", "door")]}


async def main() -> int:
    # `--teach` plans every leg and writes the program; without it the cycle replays the
    # program on disk and plans nothing. Teaching is a one-off per layout and is slow by
    # nature - it is the planner searching - so it is not what the film runs.
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    teaching = "--teach" in sys.argv[1:]
    auditing = "--audit" in sys.argv[1:]
    timeline: list = []
    planning = "--plan" in sys.argv[1:]
    out = Path(args[0] if args
               else ".scratch/runs/assembly-cycle.mp4").resolve()
    frames = out.parent / "_cycframes"
    subprocess.run(["rm", "-rf", str(frames)], check=False)
    frames.mkdir(parents=True, exist_ok=True)

    robot = await RobotClient.at_address(
        FQDN, options())
    world = Generic.from_robot(robot, "sim-world")
    # The FILM gets its own connection. Pulling three 720p frames over the one the cycle
    # commands use queued every command behind them, and the sim runs on while a command
    # waits: each step came out 3-6 sim s slower on film than in cycle_time (2026-10-14).
    lens = await RobotClient.at_address(FQDN, options())
    lens_world = Generic.from_robot(lens, "sim-world")
    camera = Camera.from_robot(lens, "overview-cam")
    details = {n: Camera.from_robot(lens, n) for v in INSET_FOR.values()
               for n in ([v] if isinstance(v, str) else v)}
    inset = {"cam": None}
    arms = {n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b", "arm-c")}
    shell_locate.CLIENT = Camera.from_robot(robot, shell_locate.CAMERA)
    motion = MotionClient.from_robot(robot, "builtin")
    grips = {"arm-b": Gripper.from_robot(robot, "grip-b"),
             "arm-c": Gripper.from_robot(robot, "grip-c")}
    HELD.clear()
    cell = chosen()
    stamp = layout_hash()
    print(f"  layout {stamp}", flush=True)

    # Which path this run takes, said out loud. A film that quietly planned every move
    # would look the same and take four times as long, and the whole point of the number
    # it prints on screen is that it is the cycle time of the cell being shown.
    if planning:
        legs = None
        print("  PLANNING every leg (--plan): this is not a cycle time", flush=True)
    elif teaching:
        legs = Legs.teach(stamp)
        print("  TEACHING: planning every leg and recording it", flush=True)
    else:
        legs = Legs.load(stamp)
        print(f"  replaying {len(legs.paths)} taught legs, "
              f"{legs.travel():.0f} deg of joint travel", flush=True)

    homes = {n: arm_home(n) for n in ("arm-a", "arm-b", "arm-c")}
    failures, fitted, ran = [], [], 0
    caption = {"text": "the mailbox assembly cell"}
    started, count, filming = time.perf_counter(), {"n": 0}, True
    # What the on-screen clock counts from: the wall clock until the cycle starts, then
    # the cycle itself. The number on the film is the one the cell is judged on.
    clock: dict = {"from": None}

    # Frames are kept from the RESTOCK on: the reset before it (every arm home, the cell
    # restaged) is half a minute of nothing on a film judged against 120 s. The restock
    # stays in - it is the flag riding its feeder in.
    rec = {"on": False}

    async def film():
        while filming:
            if not rec["on"]:
                await asyncio.sleep(0.1)
                continue
            try:
                images, _ = await camera.get_images()
                frame = Image.open(io.BytesIO(images[0].data)).convert("RGB")
            except Exception:  # noqa: BLE001
                await asyncio.sleep(0.1)
                continue
            if inset["cam"]:
                names = [inset["cam"]] if isinstance(inset["cam"], str) else inset["cam"]
                w = frame.width * 2 // 5 if len(names) == 1 else (frame.width - 24) // len(names) - 8
                right = frame.width - 12
                for name in reversed(names):
                    try:
                        shots, _ = await details[name].get_images()
                        close = Image.open(io.BytesIO(shots[0].data)).convert("RGB")
                        close = close.resize((w, w * close.height // close.width))
                        d = ImageDraw.Draw(close)
                        d.rectangle([0, 0, close.width - 1, close.height - 1],
                                    outline=(255, 255, 255), width=3)
                        # The label: which camera, what it hangs from, what it is for.
                        d.rectangle([3, close.height - 30, close.width - 4, close.height - 4],
                                    fill=(0, 0, 0))
                        d.text((10, close.height - 27), f"{name}  ·  {CAM_ROLE.get(name, '')}",
                               fill=(255, 220, 90), font=LABEL_FONT)
                        frame.paste(close, (right - close.width, 12))
                        right -= close.width + 8
                    except Exception:  # noqa: BLE001 - an inset is dressing, never a failure
                        pass
            # The cycle's clock is SIM seconds: the wall runs at whatever rate the
            # renderer allows, and the cell's cycle time is what the sim did.
            since = clock.get("from")
            try:
                elapsed = (await sim_now(lens_world) - since if since is not None
                           else time.perf_counter() - started)
            except Exception:  # noqa: BLE001
                elapsed = 0.0
            draw = ImageDraw.Draw(frame)
            draw.rectangle([0, frame.height - 40, frame.width, frame.height],
                           fill=(0, 0, 0))
            draw.text((14, frame.height - 33),
                      f"{int(elapsed // 60)}:{elapsed % 60:04.1f}   {caption['text']}",
                      fill=(255, 255, 255), font=CAPTION_FONT)
            frame.save(frames / f"f{count['n']:05d}.jpg", quality=88)
            count["n"] += 1
            await asyncio.sleep(1.0 / FPS)

    task = asyncio.create_task(film())
    where = {"index": None}
    try:
        # Reset first, so the cycle starts from a known shape.
        #
        # Without it the arms start wherever the last run left them, and a previous run
        # that failed leaves one jammed: the first planned move then failed before it began
        # with 528 N.m on the base and nothing moving, which reads as a planning fault and
        # is not one. `reset` sends every arm home and restages the props.
        caption["text"] = "reset: every arm home, the fixture back at the start"
        # ONE operation, because a reset and the cell's joints are not separable here.
        #
        # `restage` unjoins every seat and clip, resets, and puts the door back on its
        # hinge during the fall. Both halves are load-bearing and each was learned the
        # expensive way. Unjoining FIRST, because removing a joint prim does not reach
        # PhysX until the scene is re-read and `reset` is what re-reads it - a flag left
        # fastened to the shell and then teleported stretched across the cell to 23 km, at
        # the SEATED y of 86.6, which is the signature that says "still attached" rather
        # than "placed into contact"; a pin left clipped to the 40 kg pallet left at
        # z 211219.8, three placements in a row, every run. And re-joining the door during
        # the FALL, because a reset with that hinge live lands the shell 4.15 degrees out
        # of square and 5.2 mm high - every time, which is why it read for so long like a
        # property of the cradle.
        await restage(world, settle=3.0)
        if auditing:
            # Before the parks, so the moves down the cell are audited too. The flag's
            # first seconds are printed as well: it has been seen out of its nest at start.
            print(f"  audit: {await world.do_command(AUDIT_START)}", flush=True)
            for _ in range(4):
                got = (await world.do_command({"command": "prim_poses",
                                               "paths": ["/World/flag"]}))["prims"]
                f = got.get("/World/flag", {})
                print(f"  flag at start: {f.get('position_mm')} y {f.get('y_axis')}",
                      flush=True)
                await asyncio.sleep(0.5)
        await clear_and_park(world, arms, cell)
        # LOAD THE TRAY. This was never done here at all.
        #
        # `restock` was imported and not called, so the flag and the pin stayed wherever
        # `reset` spawned them and were never clipped to the carrier - which means the
        # belt left them behind on the first index, and both picks reached for parts that
        # had never travelled. The probe that measures the picks does restock, which is
        # exactly why it reported three of three holding while the cycle reported none.
        caption["text"] = "the pin waits in its magazine, the flag in its nest"
        await set_paint(world, "none")         # before the restock's restage; see line_event
        await restock_all(world, cell)
        fitted.clear()
        await asyncio.sleep(1.0)
        # Recording starts with the parts AT REST. The restock's restage throws the 60 g
        # flag out of its nest and then sets it back, and on film that read as the flag
        # starting in the wrong place and bouncing (Brandon, 2026-10-13). It is set-up,
        # not the cycle.
        rec["on"] = True
        # The numbering restarts with the cycle, because the numbering is what matches a
        # replayed leg to the move it was taught for.
        if legs is not None:
            legs.step = 0
        # THE CYCLE TIME, measured from here.
        #
        # The film's wall clock starts when the camera does, which is before the reset,
        # both of `clear_and_park`'s runs down the cell and back, and the door join - and
        # none of that is the cycle. A clock read against a 120 s target has to measure one
        # mailbox from the first station to the last, so the caption carries that number
        # and the wall time is only in the log.
        cycle_from, wall_from = await sim_now(world), time.perf_counter()
        t_step = cycle_from
        clock["from"] = cycle_from
        ran = 0
        pending = []
        parking = {}
        for step_no, (step, arm, pose, tool, what) in enumerate(CYCLE):
            if auditing:
                st = (await world.do_command({"command": "audit", "action": "clock"}))["audit"]
                timeline.append([st.get("t"), step, arm, pose])
            # A station that runs ALONGSIDE the next ones is waited for before the belt
            # moves on to anywhere else.
            if pending and (pose in AWAIT_BEFORE
                            or (pose not in OFF_BELT and cell.index_x(step)
                                != cell.index_x(where["index"] or step))):
                await asyncio.gather(*pending)
                pending = []
            if tool == "ride" or str(tool or "").startswith("next:"):
                caption["text"] = what
                inset["cam"] = None
                await line_event(world, cell, tool, rec)
                if tool == "ride":
                    where["index"] = None
                    continue
                fitted.clear()
                # The restock sets the next pallet down UPSTREAM of the tunnel
                # (fixture.INFEED_RIDE_MM), so the tunnel step indexes it in on camera.
                where["index"] = None
            if (pose in CONCURRENT and arm and legs is not None and not legs.teaching):
                if where["index"] != step:
                    x = cell.index_x(step)
                    await index_to(world, cell, x,
                                   fitted=tuple(fitted) if not SHELL_KINEMATIC else ())
                    where["index"] = step
                if parking.get(arm) is not None:
                    await parking.pop(arm)
                # Its own counter from where its legs were taught; the cycle skips them.
                n = sum(1 for lab in legs.labels.values()
                        if str(lab).split(":", 1)[0] == pose)
                # ...and the park the teach made straight after it, which the sequential
                # cycle flies and this one must skip too, or every later leg is one off.
                after = f"{legs.step + n + 1:03d}:{arm}"
                park_leg = legs.labels.get(after) == "park"
                fork = dataclasses.replace(legs)
                legs.step += n + (1 if park_leg else 0)
                pending.append(asyncio.create_task(_alongside(
                    arm, pose, tool, arms=arms, grips=grips, motion=motion, world=world,
                    stamp=stamp, legs=fork, cell=cell, home=homes[arm],
                    park_leg=park_leg)))
                print(f"  {step:<8} {str(arm):<6} {str(pose):<18} {what} (alongside)",
                      flush=True)
                continue
            if where["index"] != step and pose not in OFF_BELT:
                x = cell.index_x(step)
                caption["text"] = f"belt indexes to {step} (x {x:.0f})"
                # No `fitted` while the shell is KINEMATIC: a part jointed to a
                # kinematic body is dragged by its own joint, correctly, and placing it as
                # well fights that - the door came off its hinge and finished on the belt,
                # 367 mm out, on a cycle that had just put it there to 0.17 mm. The
                # argument stays for the conveyor stage, where the shell moves by physics
                # and nothing drags anything.
                await index_to(world, cell, x,
                               fitted=tuple(fitted) if not SHELL_KINEMATIC else (),
                               tolerance_mm=(LOOSE_STOP_MM if step in LOOSE_STOPS else None))
                # Whatever is still on the carrier goes back on its slot now the belt has
                # stopped, so a pick aims at a part that is where the station says.
                await settle_tray(world, cell, [
                    (n, p) for n, p in (("Flag", "flag"), ("Pin", "pin"))
                    if n not in fitted])
                where["index"] = step
                # Caption pacing, not a settle. `index_to` already waits for the pallet to
                # be still before it returns, and the replay pass seated the pin and the
                # flag at 0.00 mm with no extra wait at all, so this is only so a reader
                # can see which station the belt went to. 0.8 s rather than 1.4 because
                # the clock on this film is the number the cell is judged on: the replay
                # cycle is 104.5 s and 19 s of captions would put the film over 120.
                await asyncio.sleep(0.8)
            # The datum, before every step: which step tipped the shell is otherwise
            # unknowable once it is found 12 degrees out at the next insert.
            _sh = (await world.do_command({"command": "prim_poses", "paths": [
                "/World/shell", "/World/door", "/World/flag", "/World/pin"]}))["prims"]
            _s = _sh["/World/shell"]
            # The FED parts too: the flag left its stop between the feed and its pick once,
            # 30 mm up onto the rail, and the step that moved it was invisible (2026-10-12).
            for _n in ("flag", "pin"):
                _p = _sh.get(f"/World/{_n}")
                if _p and _n.capitalize() not in fitted:
                    print(f"    {_n} before {pose}: at {[round(v, 1) for v in _p['position_mm']]}"
                          f", its y {[round(v, 2) for v in _p['y_axis']]}", flush=True)
            print(f"    datum before {pose}: shell "
                  f"{math.degrees(math.acos(max(-1.0, min(1.0, abs(float(_s['y_axis'][1])))))):.1f}"
                  f" deg, at {[round(v, 1) for v in _s['position_mm']]}; door at "
                  f"{[round(v, 1) for v in _sh['/World/door']['position_mm']]}", flush=True)
            caption["text"] = what
            inset["cam"] = INSET_FOR.get(str(pose) if pose else f"@{step}")
            if step == "tunnel" and not arm:
                # The paint check is three frames with the shell standing in the gantry -
                # taken here as the course's starter takes them, so the film shows the
                # real stop and the frames are kept for review.
                t0 = time.perf_counter()
                for cam_name in TUNNEL_CAMERAS:
                    try:
                        shots, _ = await Camera.from_robot(robot, cam_name).get_images()
                        Image.open(io.BytesIO(shots[0].data)).convert("RGB").save(
                            out.parent / f"{cam_name}.jpg")
                    except Exception as exc:                       # noqa: BLE001
                        print(f"  tunnel: {cam_name} gave no frame: {exc}", flush=True)
                print(f"  tunnel: three frames in {time.perf_counter() - t0:.1f} s",
                      flush=True)
                await asyncio.sleep(max(0.0, TUNNEL_DWELL_S - (time.perf_counter() - t0)))
                # And off again: three 1280x720 cameras rendering every frame through the
                # next index showed an empty gantry and slowed the sim (2026-10-13).
                inset["cam"] = None
            if arm:
                if parking.get(arm) is not None:
                    await parking.pop(arm)
                await park_if_adrift(arms[arm], arm)
                try:
                    await _station(arm, pose, tool, arms=arms, grips=grips,
                                   motion=motion, world=world, stamp=stamp,
                                   state=await declared(world, moving=arm),
                                   carry_state=await declared(world, moving=arm, motion=motion, grip=grips.get(arm), tools=True),
                                   held_state=await declared(world, moving=arm, motion=motion, grip=grips.get(arm)),
                                   legs=legs, cell=cell)
                    if tool and tool.startswith("seat:"):
                        fitted.append(tool.split(":", 1)[1])
                    ran += 1
                    if pose == "repaint":
                        # arm-c goes home FIRST: left standing over the spur it was in the
                        # reject's way and the shell could not leave (Brandon, 2026-10-13).
                        # Planned, then settled: a bare joint move brushed arm-a's upper arm
                        # on the way (audit, 2026-10-13).
                        try:
                            pj = json.loads((POSES / arm / "park.json").read_text())
                            await fly_leg(legs, arms, motion, arm, pj["flange_mm"],
                                          orientation_for(pj["approach"], pj.get("theta_deg", 0.0)),
                                          await declared(world, moving=arm, tools=True),
                                          label="home")
                        except Exception as exc:                   # noqa: BLE001
                            print(f"    {arm} home: planned move refused ({str(exc)[:100]})",
                                  flush=True)
                        await arms[arm].move_to_joint_positions(
                            JointPositions(values=homes[arm]), timeout=120)
                        await world.do_command({"command": "conveyor", "name": "reject_belt",
                                                "run": True})
                        await asyncio.sleep(REJECT_RUN_S)
                        await world.do_command({"command": "conveyor", "name": "reject_belt",
                                                "run": False})
                except Exception as exc:                           # noqa: BLE001
                    caption["text"] = f"{what}   FAILED"
                    # THE WHOLE MESSAGE, and from the FRONT.
                    #
                    # This was `str(exc)[-240:]`, which keeps the tail - and every
                    # diagnostic in this cell is written to put the measurement first and
                    # the explanation after it. So the log showed "FAILED  feature can
                    # capture (published slack 0.20 mm)..." and threw away the one thing
                    # worth reading, which was how far the pin actually finished from its
                    # seat. Four failures in a row were reported that way and each one had
                    # to be re-derived from somewhere else.
                    failures.append((step, arm, pose, str(exc)))
                    print(f"  {step:<8} {arm:<6} {str(pose):<18} FAILED "
                          f"{str(exc)}", flush=True)
                    await asyncio.sleep(1.6)
                    # STOP ON THE FIRST FAILURE WHEN TEACHING, rather than recovering
                    # and carrying on.
                    #
                    # Recovery is right for a film - eleven stations and a named failure
                    # beats no film - but it is wrong for a teaching pass, and it was
                    # actively misleading. `clear_and_park` runs the belt the length of
                    # the cell to get an arm free, and doing that with a part loose in the
                    # fixture shifts everything: measured after one such recovery, the
                    # shell sat 43 mm high and tilted 24 degrees with the door and flag
                    # tilted with it. Every station after the first failure then failed
                    # too, against a cell that had been shaken rather than against
                    # anything wrong with the station. Two runs were spent reading the
                    # second failure as a fault.
                    if legs is not None and legs.teaching:
                        print(f"\n  STOPPING: {pose} is the first real failure. "
                              f"Everything after it would run against a cell that "
                              f"`clear_and_park` has just shaken.", flush=True)
                        break
                    await clear_and_park(world, arms, cell)
                    where["index"] = None
                    continue
                # And out of the way ONLY IF THE NEXT STATION IS A DIFFERENT ARM.
                #
                # An arm left standing at the station it just finished is an obstacle in
                # the middle of the cell, and the planner can see it now: measured, arm-a
                # waiting at the inside shot put 990 N.m through arm-b's shoulder on its
                # way to the flag-hole look, and arm-a's forearm against arm-b's end
                # effector is what refused the door placement. It read as an expensive
                # pose, and it is not - every station in this cell holds under 5.1% of its
                # drive limit, and the flag-hole look holds at 1.3%. Nothing here is heavy;
                # they were simply in each other's way.
                #
                # An arm parks so it is not an obstacle to whoever works next. If whoever
                # works next is ITSELF, parking is a round trip to nowhere - and at five to
                # ten seconds each, thirteen of them were a third of the cycle. arm-b does
                # five stations in a row; it now parks once, at the end of them.
                # PARK BEFORE THE BELT MOVES, as well as when the next arm differs.
                #
                # The rule was "park only if someone else works next", which saved
                # thirteen round trips and left one case uncovered: an arm that works two
                # stations in a row AT DIFFERENT INDEXES is still standing over the
                # fixture when the belt indexes between them. Measured on the flag-then-
                # pin pair - arm-b holds its pose at the flag insert, the belt drags the
                # carrier out from under it, and the pin arrives 79.8 mm from its slot
                # rotated 130 degrees, having been raked off the tray by a tool that
                # never moved. `index_to`'s own error message has been saying this all
                # along: "a pallet that is not moving is usually one an arm is still
                # standing in".
                nxt_arm = CYCLE[step_no + 1][1] if step_no + 1 < len(CYCLE) else None
                nxt_step = CYCLE[step_no + 1][0] if step_no + 1 < len(CYCLE) else None
                if nxt_arm == arm and nxt_step != step:
                    # Its own next station, at another stop: just out of the belt's way.
                    w = await step_out(arms, cell, arm)
                    print(f"    {arm} steps out of the belt's sweep: back {w['back_mm']} "
                          f"up {w['lift_mm']} mm ({w['cost_deg']} deg)", flush=True)
                elif nxt_arm is not None and (nxt_arm != arm or nxt_step != step):
                    # In the BACKGROUND: the arm goes home while the belt indexes and the
                    # next arm works - the three stand far enough apart - and the cycle
                    # waits on it only when that arm is next needed. Parked in line, the
                    # parks were 28 s of a cycle being judged against 120.
                    async def _park(a=arm):
                        # PLANNED to the park, then settled onto its exact joints. A bare
                        # joint move swung arm-a's forearm through arm-c's upper arm for
                        # six seconds on its way home (audit, 2026-10-13) - and timed out
                        # against it.
                        try:
                            # A TAUGHT leg: planned once when teaching and replayed after,
                            # instead of planned on every cycle (~5 s each; Fable review).
                            pj = json.loads((POSES / a / "park.json").read_text())
                            await fly_leg(legs, arms, motion, a, pj["flange_mm"],
                                          orientation_for(pj["approach"], pj.get("theta_deg", 0.0)),
                                          await declared(world, moving=a, tools=True),
                                          label="park")
                        except Exception as exc:                   # noqa: BLE001
                            print(f"    park {a}: planned move refused "
                                  f"({str(exc)[:100]}); joint move", flush=True)
                        try:
                            await arms[a].move_to_joint_positions(
                                JointPositions(values=homes[a]), timeout=180)
                        except Exception as exc:                   # noqa: BLE001
                            print(f"    park {a}: {str(exc)[:120]}", flush=True)
                    # Not while TEACHING: every leg is planned then, and the motion
                    # service re-commands the other arms in its plan - with one still
                    # parking, that failed as "error moving component arm-a".
                    # And not in the background in a replay either, any more: parked
                    # while the belt indexed, arm-a's wrist was still beside the shell as
                    # it went by, and its planned park was refused with another arm moving,
                    # so it fell back to a joint move through arm-c (audit, 2026-10-13).
                    await _park()
            t_now = await sim_now(world)
            print(f"  {step:<8} {str(arm):<6} {str(pose):<18} {what}  "
                  f"[{t_now - t_step:.1f} sim s]", flush=True)
            t_step = t_now
            await asyncio.sleep(0.15)
        if pending:
            await asyncio.gather(*pending)
        cycle_s = await sim_now(world) - cycle_from
        if parking:
            await asyncio.gather(*parking.values())
        caption["text"] = (
            f"one mailbox, twelve stations, three arms: {cycle_s:.1f} s"
            if not failures else
            # The steps that RAN, not the ones that did not fail. Those are different
            # numbers the moment the cycle stops at its first failure, and the old
            # arithmetic - total minus failures - reported "11 of 12 steps ran" about a
            # pass that had stopped at step 4 with 9 legs taught. It made two runs look
            # comparable that were not, and a cycle time read off the shorter one looked
            # like an improvement when it was just a shorter run.
            f"{ran} of {len(CYCLE)} steps ran")
        wall_s = time.perf_counter() - wall_from
        print(f"\n  CYCLE {cycle_s:.1f} s for one mailbox, first station to last "
              f"({'under' if cycle_s < 120.0 else 'OVER'} the 120 s target); sim seconds, "
              f"over {wall_s:.1f} s of wall (RTF {cycle_s / max(wall_s, 1e-6):.2f})", flush=True)
        await asyncio.sleep(2.5)
        # A teaching pass's whole output is the program, so write it before anything can
        # go wrong with the film. A pass that had failures is written anyway, and SAID to
        # be incomplete: its missing legs are the ones the failed stations never reached,
        # and a replay will refuse on the first of them rather than drive it.
        if legs is not None and legs.teaching:
            where_saved = legs.save()
            print(f"\n  taught {len(legs.paths)} legs, {legs.travel():.0f} deg of joint "
                  f"travel -> {where_saved}", flush=True)
            if failures:
                print(f"  INCOMPLETE: {len(failures)} station(s) failed, so the legs they "
                      f"would have taught are missing. Fix them and teach again before "
                      f"replaying.", flush=True)
    finally:
        filming = False
        if auditing:
            try:
                rep = (await world.do_command({"command": "audit", "action": "stop"}))["audit"]
                rep["timeline"] = timeline
                (out.parent / (out.stem + ".audit.json")).write_text(json.dumps(rep, indent=1))
                print(f"  audit: {rep.get('checks')} checks, {len(rep.get('hits', []))} "
                      f"overlapping pairs -> {out.stem}.audit.json", flush=True)
            except Exception as exc:                                 # noqa: BLE001
                print(f"  audit: could not read: {exc}", flush=True)
        await asyncio.sleep(0.3)
        task.cancel()
        await robot.close()
        await lens.close()

    took = time.perf_counter() - started
    rate = count["n"] / max(took, 0.001)
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", f"{rate:.3f}",
                    "-i", str(frames / "f%05d.jpg"), "-c:v", "libx264",
                    "-pix_fmt", "yuv420p", "-crf", "20", str(out)], check=True)
    subprocess.run(["rm", "-rf", str(frames)], check=False)
    print(f"\nwrote {out} ({out.stat().st_size / 1e6:.1f} MB, {count['n']} frames over "
          f"{took:.0f}s)")
    for step, arm, pose, why in failures:
        print(f"  FAILED {step}/{arm}/{pose}: {why}")
    print(f"  {ran} of {len(CYCLE)} steps ran")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
