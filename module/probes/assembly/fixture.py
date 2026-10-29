"""The saddle the mailbox rides on, and why it is shaped the way it is.

Not a box. A solid 100 mm block under the shell's footprint blocks the hinge: the pin bore
sits 4.7 mm below the shell floor and the shell's rail bottoms are 10 mm below it, so the
pin travels 5.3 mm above whatever the shell is standing on, right at the front end. The usd
session caught that before anything was built.

So the saddle carries the shell on two rails under the shell's OWN rails - a footprint the
asset publishes - and its lips oppose exactly the forces the three insertions apply:

    the door inserts along -X  ->  a lip at the BACK takes it
    the flag inserts along -Y  ->  the -Y lip takes it
    the pin  inserts along -Y  ->  the -Y lip takes it

Nothing pushes +X or +Y, so there is no lip in front (which would block the door) and the
+Y lip stops short of the hinge (which would block the pin). A fixture whose lips oppose
the process rather than merely surrounding the part is also one that cannot be loaded
wrong.
"""
from __future__ import annotations

import math
from typing import List

from .asset import part
from .cell import Box

# How much slack the shell has in its saddle. 6 mm, not the 35 the old cell's nest used.
#
# There the part was placed into a nest by an arm and 10 mm sat inside the planner's own
# collision margin, so 35 bought reliability. Here the shell is held for the whole cycle
# and every insertion is referenced to it: the flag's peg has 1.00 mm of radial slack in
# its hole and the pin 0.20 mm, so a shell free to shift 35 mm is a shell every taught
# insertion misses by more than the hole is wide.
SEAT_SLACK_MM = 6.0
# Lips tall enough to hold a part that is DROPPED into them, not just one that is placed.
#
# At 26 mm they did not. The shell is 230 mm tall on a 126 mm support base, which is stable
# enough standing - it would take a 35 degree disturbance to walk its centre of mass past
# the rail - but `reset` restages props a few millimetres up and lets them fall, and a part
# that lands on one rail first rocks. Measured with the shell dynamic for the first time, it
# came to rest at 72 degrees over on its +Y side, having ridden straight over a lip it only
# had to climb 26 mm to clear. 70 mm engages the side wall properly (the asset publishes it
# as z 8 to 136) and a part cannot get over it without lifting half its own height.
# How much air between a lip and the wall it locates. Enough that the part drops in, little
# enough that it cannot walk: a fixture's job is to DATUM the part, and 2.2 mm of slack per
# side is not a tolerance, it is a licence to drift.
# Pieces are named `_my` / `_py`, not `_-y` / `_+y`.
#
# Usd prim names take only letters, digits and underscores, so the module sanitises
# everything else to "_" - and "lip_-y" and "lip_+y" both arrive as "lip__y". One silently
# replaced the other for the whole of this cell's life: the fixture had a wall on one side
# and open air opposite, which is exactly a part that walks one way and never comes back.
# It also made the original two-rail saddle a one-rail saddle, which is why a tall part
# "tipped off two narrow rails" - there was one. The module now refuses a collision instead
# of resolving it.
# 5 mm, not 1.2, and this is a diagnostic as much as a fix.
#
# The shell comes to rest 24.9 degrees out of square with its origin 42.3 mm high, and
# that number indicts the lips: rolling about a RAIL at |y| 50 would lift the origin
# 50 x sin 25 = 21 mm, while rolling about something at |y| 85 lifts it 36 - and the lips'
# inner faces are at 84.9. The shell is pivoting on a lip, not on its own rails.
#
# 1.2 mm was sized to hold the part to a tolerance. It cannot do that for a body that is
# 167 mm across with 1.2 mm of room: any yaw at all, or any error in where it is set down,
# and a corner catches a lip and climbs it. Lips stop a part leaving the fixture; they are
# not what locates it, and the rail ribs already do that job to 0.4 mm.
LIP_CLEARANCE_MM = 5.0
# The pallet locates on the shell's RAILS, not on its side walls.
#
# Side lips are a poor datum for this part and the asset's own numbers say why: the bare
# shell is at its full |y| 83.7 only across the 16 mm back-cap flange, with the wall at 80.0
# and the front bead at about 81.4. Lips at the flange width therefore touch over 16 mm of a
# 476 mm part, and everything else is free - so the part slid, measured at 0.2 mm a cycle
# and still going after ten.
#
# The rails are the opposite: flat, square, 430 mm long and published -
# `assembly:rail:yInner/yOuter/xMin/xMax`, 10 mm tall from `wall:railBottomZ`. Two ribs
# standing proud of the cradle, capturing the rails between them, give a datum with 27 times
# the lever arm of the flange. The lips stay, with their clearance opened so they retain the
# part without fighting the ribs for the locating job.
RAIL_RIB_CLEARANCE_MM = 0.4
RAIL_RIB_THICK_MM = 8.0
# Just under the rails' own 10 mm, so the shell still rests on the cradle rather than on
# the ribs: a part standing on its locators is a part with no vertical datum.
RAIL_RIB_HEIGHT_MM = 8.0
# How far the cradle drops away from a kinematic shell, so the two cannot wedge.
KINEMATIC_CLEARANCE_MM = 3.0
# The short close-fitting pair under the tall lips: how much room they leave the
# part, and how far up they reach. Low enough that a part is already between them
# before it matters, close enough to stop it rolling once it is.
# 3 mm, between the 1.2 that caught the shell outright and the 5 that let it fall
# over. At 2 it still caught a little: 4.4 degrees at rest against 0.0 at 5 mm.
LOCATOR_CLEARANCE_MM = 3.0
LOCATOR_HEIGHT_MM = 20.0
# Low, so the tunnel's side cameras see the walls down to the rails (Brandon, 2026-10-13).
# The 70 mm lips were sized against the RESET DROP (a shell rode over a 26 mm lip and lay
# 72 degrees over after a restage) - before the flat cradle, the rail ribs and the 20 mm
# locators existed. Checked by the restage tilt print, over every restage of a film.
LIP_HEIGHT_MM = 25.0
LIP_FRONT_MIN_MM = 15.0
LIP_THICK_MM = 14.0
RAIL_WIDTH_MM = 26.0
# The hinge hardware all sits within the front 20 mm, so the +Y lip stops well back of it.
HINGE_KEEP_OUT_MM = 70.0
# The door's shelf: how far under the flat plate's underside its top sits (the hinge's drive
# holds the door at its flat angle; the shelf only catches a sag), and how far outboard of
# the pin bore's axis it starts, so it stays out of the pin corridor (+-8 mm about the axis).
DOOR_SHELF_GAP_MM = 0.8
DOOR_SHELF_CLEAR_X_MM = 14.0


def saddle(work_mm, belt_top_mm: float, lift_mm: float) -> List[Box]:
    """Every box of the saddle, placed under a shell seated at `work_mm`.

    `work_mm` is where the shell's ORIGIN goes, not its centre - the shell's origin is its
    own reference for every published hole, so placing the origin is what makes those
    numbers usable directly.
    """
    shell = part("Shell")
    wx, wy, _ = work_mm
    rail_bottom = shell.mm_at("assembly:wall:railBottomZ")      # -10, the contact face
    low, high = shell.bounds_mm
    # Where the shell's own rails are, from the usd session: x -217..213 at |y| 44..56.
    rail_x = (-217.0, 213.0)
    # The face the shell's rails rest on - DROPPED when the shell is kinematic.
    #
    # A kinematic shell is immovable, and it is carried along the belt by being placed
    # once per poll of the index loop. At 420 mm/s that is about 13 mm of travel between
    # placements, so it is always a little behind the pallet - and a shell sitting inside
    # a cradle it is 13 mm out of step with is a wedge. Measured: the pallet ran 361 mm
    # and jammed, and `reach_check` could not clear the cell.
    #
    # It does not need the cradle to hold it up; nothing kinematic falls. Three
    # millimetres of air under the rails lets it lag without touching, and the cradle goes
    # on looking like the fixture it is.
    from .build_fragment import SHELL_KINEMATIC

    top = belt_top_mm + lift_mm
    if SHELL_KINEMATIC:
        top -= KINEMATIC_CLEARANCE_MM
    boxes: List[Box] = []

    # ONE FLAT CRADLE, not two rails.
    #
    # Two rails is how a fixture is usually drawn and it is the wrong shape here. The shell
    # has rails of its own, 12 mm wide at |y| 44 to 56, and landing one set of rails on
    # another makes the contact depend on alignment the drop does not guarantee: with the
    # shell dynamic for the first time it came to rest 72 degrees over on its side. Moving
    # the saddle rails outboard to widen the base made it worse, not better, and the
    # arithmetic says why - at |y| 65 they spanned 52 to 78 and overlapped the shell's own
    # rails over 4 mm of strip. A flat top the full width of the part cannot be missed, and
    # the shell is designed to stand on its rails, so standing them on a plate is what the
    # part itself expects.
    pad = 14.0
    boxes.append(Box(
        "saddle_block",
        (wx + (rail_x[0] + rail_x[1]) / 2.0, wy, top - lift_mm / 2.0),
        (rail_x[1] - rail_x[0], 2.0 * (56.0 + pad), lift_mm)))

    # The shell's floor sits `-rail_bottom` above the saddle top.
    floor_z = top - rail_bottom
    lip_z = floor_z + LIP_HEIGHT_MM / 2.0

    # The lips sit against the shell's REAL WALL, not its declared bounding box.
    #
    # `mailbox:collisionBounds` says +-83.7, and the walls are not there: the asset puts
    # their outer face at |y| 81.5 - the flag hole's entry is on it - so lips built to the
    # bounding box leave 2.2 mm of air each side before anything touches. Measured over ten
    # arrivals, the part did exactly what that allows: it walked sideways in the cradle by
    # about 0.3 mm a cycle, from +3.7 mm to +6.4 mm and still going, with no datum to stop
    # it. Along the belt, where the back lip DOES touch, the same ten arrivals held to a
    # standard deviation of 0.17 mm. One axis was datumed and the other was not.
    # Lips sized from the ENVELOPE, which is what the part actually occupies.
    env_lo, env_hi = shell.envelope_mm
    face = env_hi[1] + LIP_CLEARANCE_MM

    # The locating ribs, outboard of the rails, running their full length.
    rail_outer = shell.mm_at("assembly:rail:yOuter")
    rail_x0 = shell.mm_at("assembly:rail:xMin")
    rail_x1 = shell.mm_at("assembly:rail:xMax")
    rib_face = rail_outer + RAIL_RIB_CLEARANCE_MM
    for sign in (-1.0, 1.0):
        boxes.append(Box(
            f"rail_rib_{'my' if sign < 0 else 'py'}",
            (wx + (rail_x0 + rail_x1) / 2.0,
             wy + sign * (rib_face + RAIL_RIB_THICK_MM / 2.0),
             top + RAIL_RIB_HEIGHT_MM / 2.0),
            (rail_x1 - rail_x0, RAIL_RIB_THICK_MM, RAIL_RIB_HEIGHT_MM)))
    # LOCATORS: a short, close pair under the tall lips, so the cradle has a lead-in.
    #
    # The clearance cannot be one number. At 1.2 mm the lips caught the shell as it was
    # set down and it climbed one of them - 24.9 degrees out of square with its origin
    # 42.3 mm high, every single time, which is 85 x sin 25 and names the lip as the
    # pivot. At 5 mm nothing catches and the shell lands dead square, 0.0 degrees - and
    # then nothing restrains it either, so it works its way over to 88.6 by the pin
    # insert.
    #
    # So: loose where the part arrives, close where it sits. A tall lip at 5 mm takes the
    # part in without catching, and a 20 mm locator at 2 mm holds it once it is down.
    # Exactly the lead-in the pin got for the same reason - a tight fit you cannot enter
    # is not a fit.
    for sign in (-1.0, 1.0):
        # The +Y side stops short of the hinge, for the same reason `lip_py` does: that
        # is where the pin and its fingers come across, and the pin corridor is the one
        # volume nothing on this fixture may enter. The existing corridor test caught
        # this the moment the locator was added, which is what it is there for.
        ends_at = (env_hi[0] - HINGE_KEEP_OUT_MM) if sign > 0 else env_hi[0]
        boxes.append(Box(
            f"locate_{'my' if sign < 0 else 'py'}",
            (wx + (env_lo[0] + ends_at) / 2.0,
             wy + sign * (env_hi[1] + LOCATOR_CLEARANCE_MM + LIP_THICK_MM / 2.0),
             top + LOCATOR_HEIGHT_MM / 2.0),
            (ends_at - env_lo[0], LIP_THICK_MM, LOCATOR_HEIGHT_MM)))
    # -Y lip, full length: takes both the flag and the pin.
    boxes.append(Box("lip_my",
                     (wx + (low[0] + high[0]) / 2.0, wy - face - LIP_THICK_MM / 2.0,
                      lip_z),
                     (high[0] - low[0], LIP_THICK_MM, LIP_HEIGHT_MM)))
    # +Y lip, stopping short of the hinge so the pin and its fingers have room.
    plus_to = high[0] - HINGE_KEEP_OUT_MM
    boxes.append(Box("lip_py",
                     (wx + (low[0] + plus_to) / 2.0, wy + face + LIP_THICK_MM / 2.0,
                      lip_z),
                     (plus_to - low[0], LIP_THICK_MM, LIP_HEIGHT_MM)))
    # And a FRONT +Y locator, above the pin corridor.
    #
    # The +Y lip stops 70 mm short of the front so the pin and its fingers have room, and
    # that gap is a hinge: datumed at the back and free at the front, the part pivots, and
    # ten measured arrivals walked it from +0.8 mm to +2.9 mm - past the 0.8 mm of
    # clearance the lips allow, which is only possible by rotating. Narrowing the lips
    # halved the drift and could not stop it, because the drift is a yaw and not a slide.
    #
    # The corridor the pin sweeps is 40 mm tall about the bore, which this module already
    # computes; a locator that starts above it holds the front without standing in the
    # pin's way.
    corridor = pin_corridor(work_mm, belt_top_mm, lift_mm)
    above = corridor.centre_mm[2] + corridor.dims_mm[2] / 2.0 + 4.0
    front_top = lip_z + LIP_HEIGHT_MM / 2.0
    # Only while the lips are tall enough to leave a real locator above the corridor: at
    # 25 mm it was a 6 mm sliver, and the EZU's fingers ran through it at the pin insert
    # (audit, 2026-10-13).
    if front_top > above + LIP_FRONT_MIN_MM:
        boxes.append(Box("lip_py_front",
                         (wx + high[0] - HINGE_KEEP_OUT_MM / 2.0, wy + face
                          + LIP_THICK_MM / 2.0, (above + front_top) / 2.0),
                         (HINGE_KEEP_OUT_MM, LIP_THICK_MM, front_top - above)))
    # A SHELF under the door, which now lies FLAT in front of the shell.
    #
    # The door used to arrive shut on its hinge; it now lies open at its published
    # `assembly:flatPosition`, 90 degrees about the hinge, and a 230 mm plate cantilevered
    # off a 9 mm knuckle needs something under it besides the hinge's spring. The shelf is a
    # step DOWN from the cradle - the plate's underside is 17 mm below the saddle top - and
    # starts outboard of the pin corridor so the pin and its fingers still have their room.
    door = part("Door")
    flat_lo = [v * 1000.0 for v in door.at("assembly:flatEnvelopeMin")]
    flat_hi = [v * 1000.0 for v in door.at("assembly:flatEnvelopeMax")]
    shelf_top = floor_z + flat_lo[2] - DOOR_SHELF_GAP_MM
    shelf_x0 = shell.mm_at("assembly:pinBore:entry")[0] + DOOR_SHELF_CLEAR_X_MM
    shelf_x1 = flat_hi[0]
    shelf_floor = belt_top_mm + 2.0
    boxes.append(Box("door_shelf",
                     (wx + (shelf_x0 + shelf_x1) / 2.0, wy, (shelf_floor + shelf_top) / 2.0),
                     (shelf_x1 - shelf_x0, flat_hi[1] - flat_lo[1] - 6.0,
                      shelf_top - shelf_floor)))
    # Back lip: takes the door, which inserts along -X.
    boxes.append(Box("lip_back",
                     (wx + env_lo[0] - LIP_CLEARANCE_MM - LIP_THICK_MM / 2.0, wy,
                      lip_z),
                     (LIP_THICK_MM, 2.0 * face, LIP_HEIGHT_MM)))
    return boxes


def pin_corridor(work_mm, belt_top_mm: float, lift_mm: float) -> Box:
    """The volume the pin and its fingers sweep. Nothing may be in it."""
    shell = part("Shell")
    entry = shell.mm_at("assembly:pinBore:entry")
    wx, wy, _ = work_mm
    rail_bottom = shell.mm_at("assembly:wall:railBottomZ")
    z = belt_top_mm + lift_mm - rail_bottom + entry[2]
    # x: the knuckle OD, 9 mm, plus a little. y: from well outside to the far side of the
    # bore. z: the head's 8 mm plus room for a finger each side.
    # y runs from the far end of the seated shaft to clear of the pre-insertion head.
    # Measured from the usd session's own figures: the seated shaft spans y -69..+69 and
    # the pre-insertion head centre is at y 205.5.
    far, near = -69.0, 230.0
    return Box("pin_corridor", (wx + entry[0], wy + (far + near) / 2.0, z),
               (16.0, near - far, 40.0))


# The carried set: every prop that rides the belt together, and where each one lands when
# the belt indexes. One list, because a saddle placed at one index and a shell at another
# is a shell standing over bare belt - which is how the first version announced itself,
# the part falling the 100 mm the saddle would have lifted it.
# The pallet is ONE prop now - a composite body whose children are the cradle and its
# three lips - so the carried set is the pallet and whatever is standing in it.
# The pallet, the part, and the KIT that rides with it. A flag sitting on the pallet's own
# tray travels with the pallet; a recovery that puts the pallet back and leaves the flag
# where it fell has recovered half a cell.
# The flag and the pin no longer ride the pallet: the pin stands in a feeder nest beside the
# belt and the flag arrives on its own angled feeder. See FEEDERS.
CARRIED = ("pallet", "shell", "door")


# How far upstream of the tunnel a fresh pallet is set down, so the film sees it ride in
# and stop under the gantry (Brandon, 2026-10-13). The belt's upstream segments run to
# x -6497, and the gantry starts at x -1500, so 900 mm is clear of it with the door out front.
INFEED_RIDE_MM = 900.0


def start_index_x(cell) -> float:
    """Where a fresh pallet is set down: upstream of the vision tunnel, the line's first
    check, which the first index carries it into."""
    if "tunnel" in cell.index_x_for:
        return cell.index_x("tunnel") - INFEED_RIDE_MM
    return cell.index_x("inside")


def indexed_mm(cell, name: str, index_x: float):
    """Where this prop goes when the belt has indexed to `index_x`."""
    start = start_index_x(cell)
    if name in ("flag", "pin"):
        # Fixed points beside the belt: they do not index.
        return list(feed_spot_mm(cell, name.capitalize()))
    elif name == "door":
        # Pre-fitted and lying FLAT: the door arrives on the shell, open 90 degrees about
        # its hinge, at the pose the asset publishes for exactly this.
        flat = [v * 1000.0 for v in part("Door").at("assembly:flatPosition")]
        base = (start + flat[0], cell.belt_y_mm + flat[1],
                cell.shell_origin_mm[2] + flat[2])
    elif name == "shell":
        base = (start, cell.belt_y_mm, cell.shell_origin_mm[2])
    elif name == "pallet":
        base = (start, cell.belt_y_mm, cell.belt_top_mm)
    else:
        base = next(b.centre_mm for b in
                    saddle((start, cell.belt_y_mm, 0.0), cell.belt_top_mm,
                           cell.fixture_lift_mm) if b.name == name)
    return [base[0] - start + index_x, base[1], base[2]]


# FEEDERS: where the pin and the flag come from (rev 2, 2026-10-12, on the usd session's
# redesigned parts).
#
# The PIN comes from a MAGAZINE (`cell_props/pin_magazine.usda`): eight pins standing in a
# track, and a lifter under slot 0 that pushes the end pin straight up, clear of its
# neighbours, for the jaw to take by its collar. The queue behind it is decorative - static
# pins, since pins in resting contact cannot be teleported (HANDOFF trap 2) - and the live
# pin rides the lifter.
#
# The FLAG lies flat on its own belt, angled in plan to the main one, with its grip collar
# pointing UP: the usd session put the collar on the flag's insert axis, so it is picked
# straight down along that axis and inserted exactly like the pin. Its hub and peg hang
# below the plate, so the belt is two driven strips with a groove between them, and a stop
# at the end.
#
# Both are fixed points beside the belt on arm-b's side, so their stations are
# `on_belt=False`.
import json as _json
import math as _math
from pathlib import Path as _Path

PROPS_FRAMES = _Path(__import__("os").environ.get("CELL_USD_DIR", "/home/shrews/viam/usd")) / "cell_props" / "props_frames.json"
MAG_USD = str(PROPS_FRAMES.parent / "pin_magazine.usda")


def props_attr(asset: str, key: str):
    """A published attribute of a cell prop, in the units it is published in (metres)."""
    blob = _json.loads(PROPS_FRAMES.read_text())
    return blob["assets"][asset]["attributes"][key]


def mag_attr(key: str):
    return props_attr("pin_magazine.usda", key)


MAG_PICK_XY_MM = (470.0, 550.0)     # where the raised end pin is taken (the old nest spot)
MAG_STAND_TOP_MM = 680.0            # the magazine's base underside sits on its stand here
MAG_LIFTER_PRIM = "/World/pin_magazine/Lifter"
MAG_LIFT_STEPS = 10                 # the lifter's 20 mm, shown as a push rather than a jump

# THE LINE RUNS ON PAST THE CELL (Brandon, 2026-10-12): the working 2 m belt with more of
# the same conveyor end to end either side, so no camera sees an end and the cell reads as
# one stretch of a longer plant. Every segment is driven together, so a shell that leaves
# the working belt keeps riding.
BELT_PITCH_MM = 1999.4          # the asset's deck length: segments butt with no gap
BELT_SEGMENTS_UP = 3
BELT_SEGMENTS_DOWN = 3


def belt_segments():
    """(prop name, x offset in mm from the working belt) for every belt segment, the
    working belt first."""
    return ([("belt", 0.0)]
            + [(f"belt_up{k}", -k * BELT_PITCH_MM) for k in range(1, BELT_SEGMENTS_UP + 1)]
            + [(f"belt_dn{k}", k * BELT_PITCH_MM) for k in range(1, BELT_SEGMENTS_DOWN + 1)])


def deck_span_mm(cell):
    """(from, to) x of the whole run of belt - every segment, not just the working one.
    Checked against the working belt alone, the tunnel stop on the upstream segment read
    as "off the deck", and the recovery's teleport threw the pallet 15 m (2026-10-12)."""
    dxs = [dx for _n, dx in belt_segments()]
    return (cell.belt_from_x_mm + min(dxs), cell.belt_from_x_mm + max(dxs) + BELT_PITCH_MM)


async def run_belt(world, **kw) -> None:
    """Drive every belt segment: the WORKING belt first and on its own, then the rest.

    The pallet indexes on the working belt, so its command is the one whose latency is
    stopping distance. Sent alongside six others it landed late: measured, the pallet
    coasted 72.6 mm past an index the single belt used to hit to 2 mm (2026-10-12)."""
    import asyncio
    segs = belt_segments()
    await world.do_command({"command": "conveyor", "name": segs[0][0], **kw})
    await asyncio.gather(*(world.do_command({"command": "conveyor", "name": name, **kw})
                           for name, _dx in segs[1:]))


FLAG_FEED_YAW_DEG = -150.0      # the flag's +x in plan: its hub leads TOWARD the main belt, the feeder runs back away from it
FLAG_SHAFT_XY_MM = (720.0, 540.0)   # where the flag's axis (and collar) waits, at the stop
FLAG_FEED_TOP_MM = 700.0        # the strips' top: what the plate lies on
FLAG_FEED_LEN_MM = 820.0
FLAG_QUEUE_PITCH_MM = 270.0     # a flag is 252 mm hub to plate; the queue behind the nest
FLAG_QUEUE_N = 2
FLAG_FEED_SPEED_MM_S = 200.0    # commanded; measured, the deck carries a part about a quarter of it
# A PEG slot, not a hub pocket. At 32 mm the hub and the 13 mm arm both fell into the
# groove: measured on the cell, the flag rolled 30 deg onto its plate's edge within 0.5 s and
# sat wedged on the static groove floor while both strips ran (2026-10-12). At 10 mm only
# the 6 mm peg drops in; the 24 mm hub rides both strips and the plate rides them too.
FLAG_GROOVE_MM = 14.0           # between the strips, for the 6 mm peg: 4 mm a side
FLAG_GROOVE_DEPTH_MM = 22.0     # below the strips' top; the peg hangs 10
FLAG_STRIP_W_MM = 55.0          # the plate reaches 48 to one side of the axis
# Riding on its hub, the flag's origin (the hub's underside) is ON the strips; the published
# restOriginHeight is for the plate lying in a hub pocket, which this feeder is not. The
# plate sits 3.2 mm higher than the hub's underside, so it rests pitched about 0.8 deg.
FLAG_REST_ORIGIN_MM = 0.0
FLAG_RAIL_MM = (6.0, 20.0)      # each side rail's thickness and height above the strips
FLAG_STOP_H_MM = 20.0           # the stop's height above the strips; taller would meet the jaws at the collar
FLAG_STOP_GAP_MM = 8.0          # air between the hub's leading edge and the stop
FLAG_SPAWN_BACK_MM = 200.0      # how far upstream of its rest a fresh flag is spawned
FEED_ARRIVE_S = 15.0
PIN_FED_TOLERANCE_MM = 7.0     # raised, 20 mm of shaft stays in the bore: it leans ~2 deg, 6.3 mm at the collar; the jaws centre it


def _mag_origin_mm():
    """The magazine's origin (base underside centre), placed so slot 0's pick is at
    MAG_PICK_XY_MM. Yaw 0: the track runs along world x, slot 0 at its +x end."""
    pick = [v * 1000.0 for v in mag_attr("magazine:pickPosition")]
    return (MAG_PICK_XY_MM[0] - pick[0], MAG_PICK_XY_MM[1] - pick[1], MAG_STAND_TOP_MM)


def mag_slots_mm():
    """World positions of every slot's pin origin (slot 0 first), lowered."""
    o = _mag_origin_mm()
    return [tuple(o[i] + v * 1000.0 for i, v in enumerate(seat))
            for seat in mag_attr("magazine:slotSeatPositions")]


def _quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


def _flag_axes():
    """The flag's +x in plan (the hub's end leads), its +z in plan after lying flat, and
    the feeder's travel direction."""
    a = _math.radians(FLAG_FEED_YAW_DEG)
    fx = (_math.cos(a), _math.sin(a))
    side = (-_math.sin(a), _math.cos(a))
    return fx, side, fx


def _quat_rotate(q, v):
    w, x, y, z = q
    tx = 2.0 * (y * v[2] - z * v[1])
    ty = 2.0 * (z * v[0] - x * v[2])
    tz = 2.0 * (x * v[1] - y * v[0])
    return (v[0] + w * tx + (y * tz - z * ty),
            v[1] + w * ty + (z * tx - x * tz),
            v[2] + w * tz + (x * ty - y * tx))


def feed_orientation(part_name: str):
    """The part's (w, x, y, z) at its feeder: both stand collar-up (their +Y up)."""
    if part_name == "Pin":
        return tuple(float(v) for v in mag_attr("magazine:pinOrientation"))
    rest = tuple(float(v) for v in part("Flag").at("assembly:restOrientation"))
    half = _math.radians(FLAG_FEED_YAW_DEG) / 2.0
    return _quat_mul((_math.cos(half), 0.0, 0.0, _math.sin(half)), rest)


def feed_spot_mm(cell, part_name: str, index_x: float = None):
    """Where a fed part's ORIGIN rests at its pick point, in world mm."""
    if part_name == "Pin":
        o = _mag_origin_mm()
        return tuple(o[i] + v * 1000.0 for i, v in
                     enumerate(mag_attr("magazine:liftedSeatPosition")))
    return (FLAG_SHAFT_XY_MM[0], FLAG_SHAFT_XY_MM[1], FLAG_FEED_TOP_MM + FLAG_REST_ORIGIN_MM)


# Kept for the diagnostic probes that still ask for the old name.
carrier_spot_mm = feed_spot_mm


def feed_grasp_mm(cell, part_name: str):
    """Where a fed part's published grasp point is, at rest, in world mm."""
    spot = feed_spot_mm(cell, part_name)
    local = [v * 1000.0 for v in part(part_name).at("assembly:graspPosition")]
    turned = _quat_rotate(feed_orientation(part_name), local)
    return tuple(spot[i] + turned[i] for i in range(3))


def feed_spawn(cell, part_name: str):
    """Where a fresh part is spawned: position (mm) and orientation (w,x,y,z).

    The pin in slot 0 LOWERED, a breath above its seat - the lifter pushes it up. The flag
    up its feeder, so the belt brings it to the stop - which is the point of a feeder.
    """
    if part_name == "Pin":
        spot = list(mag_slots_mm()[0])
        spot[2] += RESTOCK_DROP_MM
        return spot, feed_orientation("Pin")
    # The flag is set down AT its pick, a breath above the chute: a STATIC NEST, not a
    # moving feeder. Driven against its stop it was thrown onto the stop or the rail five
    # ways running (2026-10-12); Brandon chose the nest. Nothing moves under it now.
    spot = list(feed_spot_mm(cell, part_name))
    spot[2] += FLAG_NEST_DROP_MM
    return spot, feed_orientation(part_name)


# How far above the nest the flag is let go. Not FLAT_DROP_MM's 1 mm: measured, dropped
# from 1 mm INTO the nest it was thrown 144-154 mm up every time (twice, and from a
# restage), while 1 mm anywhere up the chute and 5 mm into the nest both settled flat at
# 0.1-0.2 mm (2026-10-12). Something at the nest is inside a millimetre of it.
FLAG_NEST_DROP_MM = 5.0
# How far above the nest the kinematic flag is held: freed IN contact it was kicked by the
# depenetration before the jaw caught it, and taken 24 mm off the tool axis (2026-10-13);
# freed half a millimetre up it only drops.
FLAG_KINEMATIC_LIFT_MM = 0.5


def flag_queue_mm(cell):
    """World positions of the decorative flags queued up the chute behind the live one."""
    spot = feed_spot_mm(cell, "Flag")
    _fx, _side, d = _flag_axes()
    return [(spot[0] - d[0] * FLAG_QUEUE_PITCH_MM * k, spot[1] - d[1] * FLAG_QUEUE_PITCH_MM * k,
             spot[2]) for k in range(1, FLAG_QUEUE_N + 1)]


def _flag_lead_mm() -> float:
    """How far the flag reaches AHEAD of its axis along the travel (its hub's radius)."""
    hi = part("Flag").envelope_mm[1]
    return hi[0]


def feeder_boxes():
    """The fixed boxes the feeders are built from: (name, centre_mm, dims_mm, yaw_deg).

    `flag_feed` and `flag_feed_b` are the DRIVEN strips; the build marks them conveyors.
    """
    o = _mag_origin_mm()
    lo = [v * 1000.0 for v in mag_attr("asset:envelopeMin")]
    hi = [v * 1000.0 for v in mag_attr("asset:envelopeMax")]
    out = [("mag_stand", (o[0] + (lo[0] + hi[0]) / 2.0, o[1] + (lo[1] + hi[1]) / 2.0,
                          MAG_STAND_TOP_MM / 2.0),
            (hi[0] - lo[0], hi[1] - lo[1], MAG_STAND_TOP_MM), 0.0)]
    fx, side, d = _flag_axes()
    up = (-d[0], -d[1])                                       # upstream
    axis = FLAG_SHAFT_XY_MM
    stop = (axis[0] + d[0] * (_flag_lead_mm() + FLAG_STOP_GAP_MM),
            axis[1] + d[1] * (_flag_lead_mm() + FLAG_STOP_GAP_MM))
    yaw = FLAG_FEED_YAW_DEG
    mid = (stop[0] + up[0] * FLAG_FEED_LEN_MM / 2.0, stop[1] + up[1] * FLAG_FEED_LEN_MM / 2.0)
    off = FLAG_GROOVE_MM / 2.0 + FLAG_STRIP_W_MM / 2.0
    for name, k in (("flag_feed", 1.0), ("flag_feed_b", -1.0)):
        out.append((name, (mid[0] + side[0] * off * k, mid[1] + side[1] * off * k,
                           FLAG_FEED_TOP_MM - 15.0),
                    (FLAG_FEED_LEN_MM, FLAG_STRIP_W_MM, 30.0), yaw))
    out.append(("flag_groove_floor", (mid[0], mid[1], FLAG_FEED_TOP_MM - FLAG_GROOVE_DEPTH_MM - 5.0),
                (FLAG_FEED_LEN_MM, FLAG_GROOVE_MM + 2.0, 10.0), yaw))
    half_w = FLAG_GROOVE_MM / 2.0 + FLAG_STRIP_W_MM
    g_t, g_h = FLAG_RAIL_MM
    for k in (1.0, -1.0):
        o2 = (half_w + g_t / 2.0) * k
        out.append((f"flag_feed_rail_{'pm'[k < 0]}",
                    (mid[0] + side[0] * o2, mid[1] + side[1] * o2, FLAG_FEED_TOP_MM + g_h / 2.0),
                    (FLAG_FEED_LEN_MM, g_t, g_h), yaw))
    deck_w = 2.0 * (half_w + g_t)
    out.append(("flag_feed_stop",
                (stop[0] + d[0] * g_t / 2.0, stop[1] + d[1] * g_t / 2.0,
                 FLAG_FEED_TOP_MM - FLAG_GROOVE_DEPTH_MM / 2.0 + FLAG_STOP_H_MM / 2.0),
                (g_t, deck_w, FLAG_GROOVE_DEPTH_MM + FLAG_STOP_H_MM), yaw))
    for k, s in enumerate((60.0, FLAG_FEED_LEN_MM - 60.0)):
        leg = (stop[0] + up[0] * s, stop[1] + up[1] * s)
        h = FLAG_FEED_TOP_MM - FLAG_GROOVE_DEPTH_MM - 10.0
        out.append((f"flag_feed_leg_{k}", (leg[0], leg[1], h / 2.0), (40.0, deck_w - 40.0, h),
                    yaw))
    return out


def feed_travel():
    """The flag feeder's travel direction, as a world unit vector."""
    _fx, _side, d = _flag_axes()
    return [round(d[0], 6), round(d[1], 6), 0.0]


def fed_upright(part_name: str, got) -> bool:
    """Is a fed part standing the way its feeder holds it? Both now stand collar-up: their
    own +y must point up."""
    return float(got["y_axis"][2]) >= 0.98


async def raise_lifter(world, steps: int = MAG_LIFT_STEPS) -> None:
    """Push the end pin up: the magazine's lifter from down to its published travel."""
    import asyncio
    travel = mag_attr("magazine:lifterTravel") * 1000.0
    for k in range(1, steps + 1):
        await world.do_command({"command": "set_translate", "path": MAG_LIFTER_PRIM,
                                "translate_mm": [0.0, 0.0, travel * k / steps]})
        await asyncio.sleep(0.05)


async def lower_lifter(world) -> None:
    await world.do_command({"command": "set_translate", "path": MAG_LIFTER_PRIM,
                            "translate_mm": [0.0, 0.0, 0.0]})



# How far above its resting spot a dynamic part is dropped when the feeder is restocked.
#
# Not cosmetic. Teleporting a rigid body to exactly touching - which `carrier_spot_mm` is,
# by
# construction - puts PhysX in resting contact at zero separation on the same step the pose
# changes, and it resolves that by ejecting the part: measured, the flag left at 22 km with
# its x and y still correct, twice. A few millimetres of air and one second of settling is
# the whole fix.
# Placed AT its resting spot, not dropped onto it.
#
# This went round twice on a wrong theory. The part was being ejected to 9.9, 12.9, 22 and
# 39 km with its x and y perfectly correct, and the obvious reading - a rigid body
# teleported into contact, resolved by being thrown out - sent me to dropping it from
# clear air instead. Watching it frame by frame says otherwise:
#
#   place at +0 mm    -> settles at 858.2 and stays there, every time
#   place at +30 mm   -> topples, ends at 831.1 and 32 mm off in y
#   place at +120 mm  -> topples, ends at 831.1
#
# Of course it topples: a flag is a thin blade on a peg and nothing stands it up. Placing it
# exactly where it rests is what works, and the drop was making a second problem while the
# first one - a part still JOINTED to the shell from the previous cycle, which cannot be
# teleported anywhere - went on happening regardless of height.
# 1.5 mm, and the history here is worth keeping because it reverses.
#
# Dropping parts from height was the first theory and it was wrong: at +30 and +120 mm the
# flag toppled and finished 32 mm off in y, while placing it at exactly +0 settled at
# 858.2 every time. So this was 0.0, and on a FLAT TRAY that is right.
#
# It stopped being right when the pin went up on two 30 mm rest blocks. Placed at exactly
# touching, a rigid body meeting two separate supports at zero separation is a contact
# PhysX resolves by ejecting it: measured, the pin left at z 212083.4. A millimetre and a
# half of air costs nothing, is far below the height that topples anything, and lets the
# solver close the gap itself rather than having to undo an interpenetration it never
# chose.
RESTOCK_DROP_MM = 1.5
# The same courtesy for a part set down on a flat tray.
FLAT_DROP_MM = 1.0
RESTOCK_SETTLE_S = 2.5


def is_square(part_name: str, got) -> bool:
    """Is this part sitting the way it was placed, in the ways that MATTER for it?

    Not every axis means something. The pin is a rod: it is round about its own length, so
    rolling in its channel changes its x and z axes and changes nothing about the part.
    Demanding all three axes line up rejected a pin that had landed 1.5 mm from its slot,
    over and over, and the carrier loader spent its whole retry budget re-placing a part
    that was already right.

    So an axis is only checked when the part's shape makes it meaningful - when the two
    extents across it differ. A blade has three distinct extents and every axis counts; a
    rod has two that match and only its length does.
    """
    from .asset import part

    low, high = part(part_name).bounds_mm
    span = [high[i] - low[i] for i in range(3)]
    axes = (got["x_axis"], got["y_axis"], got["z_axis"])

    # Which axis, if any, the part is round about: the one whose two perpendicular
    # extents match. Rotation about THAT axis is not a fact about the part, so the two
    # axes perpendicular to it carry no information and only it is worth checking.
    #
    # Getting this backwards is why the loader kept rejecting the pin. The pin's x and z
    # extents are both 8 mm, so it is round about Y - and the first version read that as
    # "x and z are the meaningful ones", checked exactly the two axes a rolling rod moves,
    # and failed a part that had landed 0.6 mm from its slot.
    symmetric = next((k for k in range(3)
                      if abs(span[(k + 1) % 3] - span[(k + 2) % 3]) < 0.5), None)
    check = [symmetric] if symmetric is not None else [0, 1, 2]
    for i in check:
        if abs(abs(float(axes[i][i])) - 1.0) > 0.02:
            return False
    return True


async def clip(world, cell, part_name: str, prop: str, spot_mm=None) -> None:
    """Hold a carried part to the tray, the way a real carrier holds one.

    A flat tray does not locate anything. The flag is a thin blade on a peg with a grasp
    shaft on top and nothing stands it up: measured, it arrived on the tray and finished
    lying on its side 175 mm away, z_axis pointing -y. The pin is a rod on two flat blocks
    with nothing stopping it rolling or sliding, and it finished 139 mm off and 30 mm low.
    Both then read as failed picks, because the tool reached for a part that had left.

    Clips, not deeper pockets. A carrier that takes a 252 mm blade and a 161 mm rod
    through a moving cell holds them positively - spring clips, magnets, a shaped nest -
    and a fixed joint is the honest model of that. It also removes a whole class of
    simulation noise that has nothing to teach: a part that tips over on a conveyor is a
    fixture problem, and this cell is about the insertion.

    Released by `unclip` immediately before the pick, which is what a real clip does when
    the gripper takes the part.
    """
    at = (await world.do_command({"command": "prim_poses",
                                  "paths": [f"/World/{prop}", "/World/pallet"]}))["prims"]
    pallet_at = at["/World/pallet"]["position_mm"]
    # Where the part SHOULD be, not where it drifted to. A clip that captures the drift
    # makes the drift permanent, and the whole reason for a located carrier is that the
    # part's position relative to the pallet is a known number rather than an outcome.
    part_at = (list(spot_mm) if spot_mm is not None
               else at[f"/World/{prop}"]["position_mm"])
    await world.do_command({
        "command": "join", "name": f"{prop}_clip",
        "body0": "/World/pallet", "body1": f"/World/{prop}", "kind": "fixed",
        "local0_mm": [part_at[i] - pallet_at[i] for i in range(3)],
        "local1_mm": [0.0, 0.0, 0.0]})


async def settle_tray(world, cell, remaining) -> str:
    """Report how far the fed parts have moved from their pick points. Does NOT move them."""
    paths = [f"/World/{prop}" for _n, prop in remaining]
    if not paths:
        return ""
    at = (await world.do_command({"command": "prim_poses", "paths": paths}))["prims"]
    worst = []
    for name, prop in remaining:
        here = at.get(f"/World/{prop}", {}).get("position_mm")
        if here is None:
            continue
        off = math.dist(here, feed_spot_mm(cell, name))
        if off > 2.0:
            worst.append(f"{prop} {off:.1f} mm")
    return ", ".join(worst)


async def unclip(world, prop: str) -> None:
    """Let go of a carried part, so the tool can take it."""
    await world.do_command({"command": "unjoin", "name": f"{prop}_clip"})


# How long after a reset the door's hinge is made. The fall is the window; this is the
# middle of it. See `restage` for the measurements.
# A LADDER, not one number, because the window is narrower than it looked.
#
# The fall is the window: `reset` lifts the shell to about z 844 and drops it 84 mm, which
# takes sqrt(2 x 0.084 / 9.81) = 0.131 s. A sweep picked 0.5 s because 0.05 and 0.5 both
# measured clean while 1.0 and 3.0 did not - but 0.5 is already after the landing, so it
# was clean by luck rather than by being inside the window, and it stopped being clean:
# nine restages in a row left the shell 11.21 degrees out, to two decimal places.
#
# 0.1 s is inside the fall with margin. The others are there because the retry should not
# re-run an identical attempt - if the first delay lands badly, a different one is a
# different experiment rather than the same one again.
DOOR_JOIN_AFTER_S = (0.1, 0.05, 0.3)
# Above this, a restage says so. The cradle puts a shell at 0.00 deg when nothing is
# pulling on it, so anything past a few tenths is a fault rather than a tolerance.
SQUARE_WARN_DEG = 0.5


async def restage(world, *, settle: float = 3.0, tries: int = 3) -> float:
    """Reset the cell, and put the door back on its hinge the only way that works.

    `reset` on its own is not a safe operation in this cell, and that is the single most
    expensive thing anyone has learned about it. Measured, three times in a row, to a
    hundredth:

        joined during the fall                shell 0.00 deg, z 760.03
        after reset #1 WITH the joint live    shell 4.15 deg, z 765.24
        after reset #2 WITH the joint live    shell 4.15 deg, z 765.24
        after reset #3 WITH the joint live    shell 4.16 deg, z 765.24
        unjoin, reset, re-join in the fall    shell 0.04 deg, z 760.03

    A reset restages every prop a little above where it belongs and lets it drop. With the
    door's revolute joint live across that, the two bodies land fighting it and the shell
    comes to rest 4.15 degrees out of square and 5.2 mm high - every time, which is why it
    read like a property of the cradle rather than of the reset. It is not the cradle. The
    cradle holds a shell at 0.00 degrees.

    Four separate theories about that tilt were wrong before this measurement: the spawn,
    the cradle's lips, the belt's missing ramp, and the flag's ejections landing on it.
    Each cost a push, a restart and a re-teach. What none of them did was reset the cell
    twice and look.

    So this is the cell's reset, and a bare one is a bug. Unjoin first, because a joint
    edit does not reach PhysX until the scene is re-read and `reset` is what re-reads it;
    then join during the fall, where there is no resting contact for the joint to fight.
    """
    import asyncio
    import math as _math

    from .asset import insertions

    for attempt in range(tries):
        for which in ("flag", "door", "pin"):
            for joint in (f"{which}_seat", f"{which}_clip"):
                await world.do_command({"command": "unjoin", "name": joint})
        await world.do_command({"command": "reset"})
        wait = DOOR_JOIN_AFTER_S[min(attempt, len(DOOR_JOIN_AFTER_S) - 1)]
        await asyncio.sleep(wait)
        d = insertions()["Door"]
        await world.do_command({
            "command": "join", "name": "door_seat",
            "body0": "/World/shell", "body1": "/World/door", "kind": d.joint,
            "local0_mm": list(d.joint_anchor_mm),
            "local1_mm": [d.joint_anchor_mm[i] - d.seated_mm[i] for i in range(3)],
            "axis": list(d.joint_axis_mm), "limits_deg": list(d.joint_limits_deg),
            **({"drive": {k: v for k, v in d.joint_drive.items()
                          if not k.startswith("_")}} if d.joint_drive else {})})
        await asyncio.sleep(max(0.0, settle - wait))

        # AND CHECK IT LANDED SQUARE, every time, in one line.
        #
        # The cell spent a long day attributing a 4.15 degree tilt to four different things
        # because nothing ever stated the datum out loud. It is one `prim_poses` call, and
        # it turns "the shell is crooked and nobody knows when it happened" into "this
        # restage left it crooked".
        #
        # AND TRY AGAIN, because it works. The check earned this on its first outing: a
        # cell left dirty by a killed run restaged to 11.25 degrees, then 11.25, then 4.29
        # - improving each time as whatever was leaning on it came off. A loop that is
        # meant to run unattended should spend three seconds on that rather than hand a
        # crooked datum to every station downstream and fail eight minutes later.
        sh = (await world.do_command({"command": "prim_poses",
                                      "paths": ["/World/shell"]}))["prims"]["/World/shell"]
        tilt = _math.degrees(_math.acos(max(-1.0, min(1.0, abs(float(sh["y_axis"][1]))))))
        if tilt <= SQUARE_WARN_DEG:
            return tilt
        if attempt + 1 < tries:
            print(f"    restage left the shell {tilt:.2f} deg out of square at z "
                  f"{sh['position_mm'][2]:.2f} (joined {wait:.2f}s after the reset); "
                  f"restaging again with a different delay", flush=True)
    print(f"    restage left the shell {tilt:.2f} deg out of square at z "
          f"{sh['position_mm'][2]:.2f} after {tries} tries - it should land at 0.0, so "
          f"something is leaning on it that a reset does not clear", flush=True)
    return tilt


# How far from its computed slot a part can be and still count as "the reset staged it".
#
# Loose on purpose. This is not a quality gate: the pick measures where the part actually
# is and corrects to it, and refuses outright past 40 mm. All this number has to do is
# tell a part the reset put on the carrier from one that is not on the carrier at all.
# Measured residuals after a reset are 4.71 mm for the flag - almost entirely in z - and
# 1.49 for the pin.
STAGED_TOLERANCE_MM = 15.0


async def restock_all(world, cell, parts=(("Flag", "flag"), ("Pin", "pin")),
                      tries: int = 5) -> None:
    """Restage the cell and wait for both feeders to present their parts.

    A restage re-spawns every prop where the fragment put it: the pin a breath above its
    nest's seat, the flag up its feeder. So restocking is a reset and a wait - the feeder
    carries the flag to its stop - and NOTHING is teleported into contact, which is the one
    thing a resting part cannot survive here (HANDOFF, trap 2).
    """
    import asyncio
    import time as _time

    for attempt in range(tries):
        # The lifter is a transform, not a prop: a reset does not put it back. Down first,
        # so the pin re-spawns into slot 0 as the fragment placed it.
        try:
            await lower_lifter(world)
        except Exception:                                          # noqa: BLE001
            pass
        await restage(world, settle=2.5)
        if any(name == "Pin" for name, _p in parts):
            await raise_lifter(world)
        # A reset does not bring the flag back to its configured spawn: measured, it came
        # back 16 mm across the feeder and turned 15 degrees, jammed in the slot. So it is
        # put there, in free space a breath above the deck, before the belt starts.
        if any(name == "Flag" for name, _p in parts):
            # KINEMATIC IN ITS NEST, exactly at its pick, until the jaws close on it (the
            # flag pick makes it dynamic). Resting dynamic it slid 15 mm and rose 10 in four
            # seconds and was re-placed at the pick - on film, a flag starting in the wrong
            # place and jumping (Brandon, 2026-10-13).
            spot = list(feed_spot_mm(cell, "Flag"))
            spot[2] += FLAG_KINEMATIC_LIFT_MM
            await world.do_command({"command": "set_kinematic", "name": "flag", "on": True,
                                    "position_mm": [float(v) for v in spot],
                                    "orientation_wxyz": [float(v) for v in
                                                         feed_orientation("Flag")]})
            await asyncio.sleep(0.3)
        deadline = _time.monotonic() + FEED_ARRIVE_S
        while True:
            got = (await world.do_command({
                "command": "prim_poses",
                "paths": [f"/World/{prop}" for _n, prop in parts]}))["prims"]
            report, ok = [], True
            for name, prop in parts:
                here = got[f"/World/{prop}"]
                # At the GRASP POINT, not the origin: the flag rests pitched 1.3 degrees on
                # its hub and plate, which puts its origin 5 mm low with the shaft on its mark.
                local = [v * 1000.0 for v in part(name).at("assembly:graspPosition")]
                feature = [here["position_mm"][i] + local[0] * here["x_axis"][i]
                           + local[1] * here["y_axis"][i] + local[2] * here["z_axis"][i]
                           for i in range(3)]
                off = math.dist(feature, feed_grasp_mm(cell, name))
                up = fed_upright(name, here)
                # The pin leans 1.3 degrees in its 0.5 mm-slack bore, which reads a constant
                # 3.1 mm at the collar; the three jaws centre it as they close.
                ok = ok and up and off <= (PIN_FED_TOLERANCE_MM if name == "Pin"
                                           else STAGED_TOLERANCE_MM)
                report.append(f"{prop} {off:.1f} mm from its pick, upright={up}")
            if ok or _time.monotonic() > deadline:
                break
            await asyncio.sleep(0.1)
        if ok:
            print(f"    fed: {'; '.join(report)}", flush=True)
            sh = (await world.do_command({
                "command": "prim_poses",
                "paths": ["/World/shell"]}))["prims"]["/World/shell"]
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, abs(float(sh["y_axis"][1]))))))
            print(f"    shell at rest, before any index: {tilt:.1f} deg out of square, "
                  f"z {sh['position_mm'][2]:.1f}", flush=True)
            return
        print(f"    feeders not ready after {FEED_ARRIVE_S:.0f} s ({'; '.join(report)}); "
              f"restaging", flush=True)
    raise RuntimeError(
        "the feeders would not present their parts: a part that never arrives at its pick, "
        "or arrives lying down, is a feeder fault - look at the film before the code.")


async def restock(world, cell, part_name: str, prop: str, tries: int = 3):
    """Legacy single-part restock. The feeders are restocked together by a restage now, so
    this is `restock_all` for the one part."""
    await restock_all(world, cell, parts=((part_name, prop),), tries=tries)


# How the belt indexes: CLOSED LOOP on the pallet's own position.
#
# Not a timed run, and not a teleport. Teleporting was tried five ways and threw the
# assembly 423 m, 2.6 km, 3197 km and 13 m - moving a jointed assembly by setting poses is a
# constraint solve physx resolves however it likes. A timed run is no better, because the
# deck does not deliver the speed it is asked for: commanded 420 mm/s, the pallet measured
# about 1200. Whatever that factor is, it is not something to calibrate and depend on, so
# the belt runs while the pallet is watched and stops when it arrives.
#
# Two speeds, because one cannot do both jobs. Far away the belt runs flat out; inside
# CREEP_FROM_MM it runs slowly enough that a poll's worth of travel is smaller than the
# tolerance. At full speed one 80 ms poll is 95 mm of belt, which is not a stopping
# distance - it is most of a station.
# Tightened once the FIXTURE stopped being the dominant error.
#
# At a 6 mm stopping window the belt contributed sd 0.53 mm in x, which is comfortable
# against the flag's 1.00 mm of radial slack and not against the pin's 0.20. There was no
# point tightening it while the part was free to walk 2 mm in its cradle; now that the
# cradle holds it to 0.03 mm, the belt is the whole remaining error and worth the extra
# second.
INDEX_FAST_MM_S = 420.0
# 18 mm/s does not move the pallet at all - it stalled 64 mm short and sat there until the
# timeout. A surface velocity has to beat stiction before it beats distance, and below
# about 40 mm/s commanded this pallet simply stays put. 55 is the lowest that reliably
# creeps, and the window is tightened instead.
# 60 mm/s is the floor, and it is a hard one: at 55 the pallet stalled 42 mm short and at
# 18 it never started, both sitting there until the timeout. A surface velocity has to beat
# stiction before it beats distance. The window is tightened as far as that floor allows -
# a 30 ms poll at 60 mm/s commanded is about 4 mm of belt - and no further.
#
# Which leaves the belt contributing about half a millimetre: comfortable against the
# flag's 1.00 mm of radial slack, and still three times the pin's 0.20. No amount of belt
# tuning closes that gap; the pin needs the part located and the insertion corrected.
INDEX_CREEP_MM_S = 60.0
CREEP_FROM_MM = 90.0
# 1.5 mm, down from 4.
#
# The belt is far more repeatable than it is accurate: measured twice through every
# station, the spread between laps is 0.1 mm and the BIAS is about -2.9, because the
# creep-back loop stops the moment the error is inside the window and never closes the
# rest. That was invisible while the window was 4 mm and the targets were big. The pin's
# grip collar is 7.4 mm across, so 2.9 mm of bias leaves 0.8 mm of it under the tool.
INDEX_TOLERANCE_MM = 1.5
INDEX_POLL_S = 0.03
INDEX_TIMEOUT_S = 12.0        # with no progress: a stopped pallet
INDEX_HARD_LIMIT_S = 180.0    # and an index that never ends
CREEP_LIMIT_S = 8.0           # one settling creep, at most
INDEX_ACCEL_MM_S2 = (INDEX_FAST_MM_S - 60.0) / 0.45   # the launch ramp's, braking too
INDEX_RETRIES = 3             # creeps back after coasting out of the window
INDEX_WATCH_S = 0.1           # how often the client looks; the module decides the stop
INDEX_TRACE = True            # print each index's cut and rest, to measure the coast
# What counts as stopped: this much movement or less, this many polls running.
STILL_MM = 0.05
STILL_POLLS = 4
STILL_TIMEOUT_S = 4.0


async def pallet_x_mm(world) -> float:
    r = (await world.do_command({"command": "prim_poses",
                                 "paths": ["/World/pallet"]}))["prims"]
    return float(r["/World/pallet"]["position_mm"][0])


# HOW THE BELT GETS UP TO SPEED, because it did not: it got there in one step.
#
# `conveyor` sets a surface velocity, and the module applies it on the step it is asked -
# 0 to 420 mm/s with no ramp at all. Under a shell 230 mm tall standing on two narrow rails
# with 3 mm of locator clearance, that is a jolt, and the measurement says exactly that:
# the shell settles 0.1 degree out of square before the belt ever moves and is 4.2 degrees
# out after one index. It is not spawning crooked and the cradle is not holding it crooked.
# The index is tipping it.
#
# Over the pin's 138 mm stroke, 4.2 degrees is 10 mm at the far end against a chamfer that
# captures 1.5, so this is the difference between an insert that can be corrected and one
# that cannot - and a camera looking at the bore's mouth cannot measure a tilt at all, so
# no vision correction substitutes for getting this right.
#
# A ramp rather than a lower top speed, because the top speed is not the problem and the
# cell has a 120 s budget: 956 mm of travel at 420 mm/s is 2.3 s, and reaching it over
# 0.45 s costs about a tenth of a second per index while removing the jolt entirely. This
# is also what a real indexing conveyor does.
INDEX_LAUNCH_MM_S = 60.0        # the speed the belt starts from
INDEX_LAUNCH_S = 0.45           # how long it takes to reach INDEX_FAST_MM_S


def launch_speed(elapsed_s: float) -> float:
    """The belt's commanded speed `elapsed_s` into an index: a ramp, not a step."""
    if elapsed_s >= INDEX_LAUNCH_S:
        return INDEX_FAST_MM_S
    fraction = max(0.0, elapsed_s) / INDEX_LAUNCH_S
    return INDEX_LAUNCH_MM_S + fraction * (INDEX_FAST_MM_S - INDEX_LAUNCH_MM_S)


async def sim_now(world) -> float:
    """The sim's own seconds, which never rewind across a restage.

    Cycle time is counted on this, never on the wall: the sim runs slower or faster than
    real time depending on what it is rendering, and a learner's cell runs in real time,
    so only sim seconds are the cell's cycle time."""
    return float((await world.do_command({"command": "status"}))["sim_clock"])


async def index_to(world, cell, to_x: float, from_x: float = None, fitted=(),
                   tolerance_mm: float = None):
    """Run the belt until the pallet arrives at `to_x`, then stop it.

    `fitted` is accepted for call compatibility and ignored: a part jointed to the shell
    rides with it because the shell itself is being carried, which is the entire point of
    driving the belt rather than placing things.
    """
    import asyncio
    import time

    from .build_fragment import SHELL_KINEMATIC

    base = indexed_mm(cell, "pallet", 0.0)[0]      # pallet x when the index is zero
    # A stop nothing is aimed at (the tunnel's photographs) need not hold the arms' 1.5 mm.
    tol = INDEX_TOLERANCE_MM if tolerance_mm is None else float(tolerance_mm)

    # If the pallet is OFF THE DECK, put it back before trying to drive it.
    #
    # A belt can only move what is standing on it, and a run that ends badly can leave the
    # pallet past the end: measured at x -1424 against a deck that starts at -1400, sitting
    # on the floor with the belt running and nothing happening, until the 25 second timeout
    # reported "+4324 mm to go" about a pallet that was never going to move.
    #
    # Teleporting a dynamic assembly is what causes this cell's explosions, so it is done
    # only here, only when the alternative is a jam, and onto clear deck rather than into
    # anything - which is the one case where setting a pose is better than driving to one.
    got = (await world.do_command({"command": "prim_poses",
                                   "paths": ["/World/pallet"]}))["prims"]["/World/pallet"]
    here = float(got["position_mm"][0])
    upright = float(got["z_axis"][2])
    lo, hi = deck_span_mm(cell)
    off_deck = not (lo + 200.0 <= here <= hi - 200.0)
    # TIPPED counts as well as off the end, and nothing else was catching it.
    #
    # `world.reset()` restores what is registered in `world.scene` - the arms and the plain
    # cube props - and a COMPOSITE prop is neither that nor a referenced asset, so nothing
    # restaged the pallet. Measured after a bad run: lying across the belt with its local Z
    # pointing along world -X, yawed 73 degrees, wedged, and a reset at the top of every
    # pass that did not touch it. The belt ran and the x position moved 0.1 mm in 2.5
    # seconds, which reads as "something is in the way" and was the pallet itself.
    tipped = upright < 0.9
    if off_deck or tipped:
        why = "off the deck" if off_deck else f"tipped (z_axis z {upright:+.2f})"
        print(f"    pallet is {why} at x {here:.0f}; placing it back", flush=True)
        # UNJOIN FIRST, and reset so physx re-reads. A shell with parts seated on it is a
        # shell with joints attached, and teleporting a jointed body stretches those joints
        # across the cell - which this build has measured at 23 km, 3197 km and, on the
        # first version of this very recovery, 1.5 million mm. Removing a joint prim does
        # not take effect until the scene is re-read, and `reset` is what does that.
        await world.do_command({"command": "unjoin", "name": CLAMP_JOINT_NAME})
        await restage(world, settle=2.5)
        await world.do_command({
            "command": "place_props",
            "props": [{"name": prop, "position_mm": indexed_mm(cell, prop, to_x),
                       "yaw_deg": 0.0} for prop in CARRIED]})
        await asyncio.sleep(2.0)

    # THE MODULE STOPS THE BELT, not this loop.
    #
    # This loop used to poll the pallet over gRPC every 30 ms and command the stop itself,
    # and how far the pallet ran between a poll and the stop was set by the sim's speed:
    # tuned at 0.33 of real time, it stopped 27.7 mm out once the sim ran at 1.0
    # (2026-10-14). The module's `index` decides on every physics step - launch ramp,
    # braking to creep, cut inside the window, creep back if it coasts out - so the stop is
    # the same at any rate. This only watches, for a stall, in SIM seconds.
    if SHELL_KINEMATIC:
        raise RuntimeError("a kinematic shell is not carried by the module's index")
    names = [name for name, _dx in belt_segments()]
    got = (await world.do_command({"command": "index", "action": "start", "spec": {
        "conveyors": names, "prim": "/World/pallet", "axis": 0,
        "target_mm": base + to_x, "tol_mm": tol, "fast_mm_s": INDEX_FAST_MM_S,
        "creep_mm_s": INDEX_CREEP_MM_S, "accel_mm_s2": INDEX_ACCEL_MM_S2,
        "max_retries": INDEX_RETRIES}}))["index"]
    deadline = time.monotonic() + INDEX_HARD_LIMIT_S
    best, moved_at = None, await sim_now(world)
    try:
        while got["phase"] not in ("done", "failed"):
            await asyncio.sleep(INDEX_WATCH_S)
            got = (await world.do_command({"command": "index", "action": "read"}))["index"]
            if got["phase"] in ("idle", "stopped"):
                raise RuntimeError(f"the index was dropped mid-run ({got['phase']}): "
                                   f"a reset or another index took the belt")
            err = abs(got["error_mm"])
            now = await sim_now(world)
            if best is None or err < best - 1.0:
                best, moved_at = err, now
            if now - moved_at > INDEX_TIMEOUT_S or time.monotonic() > deadline:
                raise RuntimeError(
                    f"belt did not reach index {to_x:.0f} mm (no progress for "
                    f"{INDEX_TIMEOUT_S:.0f} sim s): pallet {got['error_mm']:+.0f} mm to go. "
                    f"A pallet that is not moving is usually one an arm is still standing in.")
    finally:
        if got["phase"] not in ("done", "failed"):
            await world.do_command({"command": "index", "action": "stop"})
    if INDEX_TRACE:
        print(f"    index {to_x:.0f}: {got['t_s']:.2f} sim s, stops {got['stops']}", flush=True)

    # ARRIVED, OR SAY SO. This used to return `to_x` whatever the pallet had done.
    #
    # The creep-back loop can run out its deadline, and when it did the function reported
    # the index it had been ASKED for rather than the one it reached - so a 9.3 mm
    # overshoot was handed on as a perfect arrival. Everything downstream is sited off the
    # index: the pin's grip collar is 7.4 mm across, so 9.3 mm of unreported error is a
    # tool reaching past a part it cannot touch, which is exactly how that pick failed.
    landed = await pallet_x_mm(world) - base
    if abs(to_x - landed) > tol:
        raise RuntimeError(
            f"belt stopped {landed - to_x:+.1f} mm from index {to_x:.0f} mm, outside the "
            f"{tol:.0f} mm the stations are sited against. Everything "
            f"picked at this index is aimed by this number.")
    return landed


# The station clamp, KEPT BUT NOT USED - see `index_to`, which holds by mass and friction.
#
# Joining the pallet to the deck at each station and releasing it to index does hold against
# insertion loads, and it is what a real line does. In this simulation it is not stable:
# making and breaking a fixed joint on a moving dynamic assembly, six times a cycle, threw
# the whole thing 3.1 metres on the eighth arrival of eight, and before that turned an
# indexing repeatability of 0.49 mm into 32. Every explosion in this cell has had the same
# shape - a constraint being created or destroyed around a body that is not where the solver
# expects - and this was no exception.
#
# A pallet that is heavy enough does not need the joint: mu * m * g with a 40 kg pallet on a
# 0.9 deck is 353 N of holding force, which is a great deal more than an arm pressing a pin
# through a bore, and it is arithmetic rather than a constraint solve. That is also why real
# pallets are heavy. Left here because a genuinely immovable station - one that must not
# move under ANY load - still wants a mechanical stop, and this is the shape of it.
# The station CLAMP.
#
# A conveyor delivers a pallet; it does not hold one. With everything dynamic the first full
# cycle made that obvious: the arms seat a pin 135 mm through a bore and cap a door onto the
# front, and those loads have to go somewhere - the whole fixture slid 110 mm off the belt's
# centreline and the assembly finished 64 to 146 mm from where it belonged. The belt had
# been carrying it correctly the entire time; nothing was holding it while it was worked on.
#
# Which is what a real line does: convey, stop, CLAMP, work, release, index. The clamp here
# is a fixed joint between the pallet and the belt - the belt is kinematic, so a pallet
# jointed to it cannot move - made where the pallet actually stopped rather than at a
# nominal pose, because a clamp's job is to hold the part where it IS.
# TWO joints, because a pallet clamp alone is not work holding.
#
# Clamping the pallet to the deck stopped the fixture sliding - 110 mm off the centreline
# became 3 mm - and left the part free to move INSIDE it. Capping the door drives along -X
# onto the front, and with the pallet now immovable that load levered the shell nose-up over
# the back lip: measured, pitched 7 degrees, its front raised 58 mm, with the flag, door and
# pin all riding along 17 to 30 mm out of place. The cradle constrains the part sideways and
# nothing constrains it in pitch but gravity.
#
# So a station clamp holds the PART to the pallet and the pallet to the deck. That is what a
# work-holding fixture is, as distinct from a conveyor stop, and the distinction is exactly
# the one this cell had to learn twice.
CLAMP_JOINT_NAME = "pallet_clamp"
CLAMP_JOINTS = ((CLAMP_JOINT_NAME, "/World/belt", "/World/pallet"),
                ("part_clamp", "/World/pallet", "/World/shell"))


async def clamp(world, on: bool = True):
    """Hold the part and its pallet while an arm works, or release both to index."""
    out = []
    for name, body0, body1 in CLAMP_JOINTS:
        if not on:
            out.append(await world.do_command({"command": "unjoin", "name": name}))
            continue
        r = (await world.do_command({"command": "prim_poses",
                                     "paths": [body1]}))["prims"]
        if body1 not in r or "position_mm" not in r[body1]:
            continue
        # Anchored where the body ACTUALLY is, not at a nominal pose: a clamp's job is to
        # hold the part where it stopped. (The seating joints are the opposite case and
        # use the asset's published frames, because there the pose is the point.)
        out.append(await world.do_command({
            "command": "join", "name": name, "kind": "fixed",
            "body0": body0, "body1": body1,
            "anchor_mm": list(r[body1]["position_mm"])}))
    return out
