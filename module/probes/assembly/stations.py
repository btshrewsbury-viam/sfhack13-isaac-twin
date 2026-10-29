"""Every place an arm works, DERIVED from what the parts publish.

Not a table of poses. Each station is computed from the asset's own numbers - the hole the
flag goes in, the aperture the door caps, the bore the pin travels - so a station cannot
drift from the part it refers to. The standoff lives on the TOOL that does the job, beside
the grasp it belongs to, never on the arm or the cell.

That arrangement is the point. The previous build lost more time to one bug class than to
anything else: a derived scalar true of one grasp reused for another. A 367 mm standoff
from an abandoned end grip applied to a side grip. A 175 mm hold applied to a 377 mm one. A
fixture sited against a pose the round never visited. Every one passed the full offline
suite and failed the run, because the suite checked the number and not where it came from.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

from .asset import insertions, part
from .fixture import FLAG_FEED_YAW_DEG, feed_grasp_mm
from .cell import (Cell, MAX_PAN_DEG, MAX_UP_MM, REACH_BUDGET_MM, REACH_COMFORT_MM)

Vec3 = Tuple[float, float, float]


@dataclass(frozen=True)
class Tool:
    """An end effector: what it is, how far back of the work its flange sits, and - for a
    tool that carries a camera - where along it the LENS is.

    Those last two are different numbers and conflating them is a real error, not a tidiness
    point. The RealSense's 320 mm is an OPTICAL distance: it is how far the lens stands off
    its subject, so the lens belongs near the flange and the 320 mm is the gap in front of
    it. Mounting the lens 320 mm down the tool axis instead puts it exactly ON the flag
    hole - which is to say inside the shell's wall. The fisheye is the other case: it is a
    camera on a stem, so its lens really is at the tip, 190 mm out.
    """

    name: str
    standoff_mm: float
    why: str
    lens_mm: float = 0.0


# These are DESIGN choices, not asset facts, and the sweep is what sized them.
#
# At a 250 mm fisheye stem and a 210 mm shell gripper no arrangement cleared every bound,
# and the stations that failed were exactly the ones those two tools serve - the inside and
# back shots at 902 and 937 mm of reach, the shell grasp at 932. A tool's length is mine to
# choose, so it is choosable here rather than something the layout has to work around.
#
# What bounds them from below is real. The fisheye has to put its tip 70 mm inside the door
# aperture with the flange clear of the shell's front at x 235: a 190 mm stem leaves the
# flange 120 mm outside the aperture plane, which clears. The shell gripper has to span a
# 167 mm wide box and reach past its own fingers, so 180 mm is about the shortest honest
# figure. The RealSense standoff is set by optics, not by reach - 320 mm is what puts an
# 8 mm hole across about 26 pixels - so it does not move.
# How far short of a surface a suction cup stops.
#
# A cup is not a probe: it has to arrive NEAR the face and let the seal close the gap, and
# a cup driven INTO geometry cannot seal at all. arm-c's grasp aimed the cup at the shell's
# published wall plane, |y| = 80 - and raycasting the live cell at the grasp point put the
# wall's surface at y -80.16, so the cup started 0.16 mm inside it. Measured result: the
# ray from the cup reported "hit at 0.0 mm", the surface gripper attached to nothing, and
# the shell moved 0.0 mm when the arm backed off. The cell called that station a pass for
# weeks because only motion completion was being checked.
#
# The gap has a FLOOR as well as a ceiling, which is the part that was not obvious.
#
# `max_grip_distance` is 20 mm and reads like the only constraint, so 5 mm looked safely
# inside it. Swept against the live cell, arm-c's cup catches nothing at 1.0 mm and
# nothing at 2.7 mm, and attaches at 6.0 mm - lifting the shell 59.70 mm on a 60 mm move.
# Isaac's surface gripper will not seal against a face it is already touching; it needs
# room to close the gap itself.
#
# 10 mm sits in the middle of the band that works, and the margin is needed: the 5 mm
# nominal only ever delivered 2.7 mm at the wall, because the published wall plane is
# |y| = 80.0 while the measured surface is 80.16 and the shell drifts up to 1.7 mm in its
# cradle. A nominal figure has to absorb both.
# 14, not 10, now that a pick aims at the grasp CENTRE rather than at a bounding-box
# face: the part's surface is half its grasp width nearer the tool than the point the
# station works to, so a 10 mm stand-off would deliver 5 mm at the flag's O10 shaft -
# under the 6 mm the cup was measured to need. 14 delivers 9 at the flag, 10.3 at the
# pin's O7.4 collar and 14 at the shell's flat wall, all inside the 20 mm ceiling.
# 8 since 2026-10-13, with the suction unit's MODEL mounted 7 mm further out than its
# attach point (build_fragment.SUCTION mount_mm), so on film the cup sits 1 mm off the wall
# - touching, as Brandon asked - while the attach ray still has the 6+ mm it needs to seal.
CUP_GAP_MM = 8.0


# Both fed parts are taken FROM ABOVE by the three-jaw gripper, and this is which of the
# part's OWN axes points up at its feeder - the direction the face gap is measured along.
#
# The pin stands head-up in its nest with its own +Y vertical, which is also the axis it is
# pressed along, so the tool holds it on the insert axis by construction. The flag stands as
# it is seated, its +Z up, and is taken by its vertical grasp shaft.
FED_UP = {"Pin": (0.0, 1.0, 0.0), "Flag": (0.0, 1.0, 0.0)}
# The EZU's 15 mm finger pads sit at its fingertips, 155..170 mm out from the flange
# (finger meshes 125..170 in the asset's own frame; pad 15 x 15 at the tip - the usd
# session's measurement), so their centre is 162.5 mm out. The usd session publishes each
# grasp point AS the centre of the pad band on the collar, so the flange stands that far
# from it: the jaw's 134 mm attach point plus this.
# The grip centre's distance from the flange: the usd session's `assembly:ezuTcpOffset`,
# read per part by `jaw_extra_mm`. 186.5 for both since the pin's Ø8 grip knob and the
# flag's Ø8 x 12 collar (2026-10-13); this is only the fallback.
JAW_PAD_MM = 186.5
# The surface gripper's attach point for the jaw, 160 mm out: ~20 mm short of the knob and
# collar tops, the gap it attaches across cleanly. At 134 (46 mm short since the 186.5 mm
# grip) the attach itself threw the parts - measured through a grab, the pin knocked 40 mm
# sideways and left 35 mm low, the flag spun 50 deg (2026-10-13).
JAW_ATTACH_MM = 160.0
JAW_PAD_EXTRA_MM = JAW_PAD_MM - JAW_ATTACH_MM


def jaw_extra_mm(part_name: str) -> float:
    """How far past the jaw's attach point (JAW_ATTACH_MM) the grip centre is, for THIS part: the
    usd session publishes it per part as `assembly:ezuTcpOffset` (pin 175, the 12 mm flag
    collar 186.5 since 2026-10-13)."""
    try:
        return float(part(part_name).at("assembly:ezuTcpOffset")) * 1000.0 - JAW_ATTACH_MM
    except Exception:                                              # noqa: BLE001
        return JAW_PAD_EXTRA_MM
BORE_LOOK_ELEVATION_DEG = 20.0
# The jaw's roll at the pin insert, chosen for the RealSense on its bracket: at -165 the D435
# sat over the flat door (21 deg in the shell's x-z plane, body z 800) and Brandon saw it hit
# the door. Swept live (rs_clock, 2026-10-13): j6 +45 puts it at 66 deg - above the door by
# ~85 mm and ~35 mm ahead of the shell's front face. j6 follows theta with the SAME sign
# (150 put the camera under the axis: the planner found it in the belt deck), so -120.
PIN_INSERT_ROLL_DEG = -120.0
# The FLAG is held 10 mm deeper in the tool than the jaw model puts it: 187.6, 187.5, 187.3,
# 187.3 mm flange-to-origin over four runs against 177.6, picked from the taught 877.6 with
# the flag measured in its nest, and unchanged once the tool turns horizontal - so not sag
# (2026-10-12). A constant, not noise: the insert stands back by it so the peg meets the
# mouth rather than the wall. Where in the grip it comes from is still to find.
# Still 9.8, re-measured 2026-10-13 with the 12 mm collar and the 160 mm attach point: the
# flag is held at 209.8 mm against 200.1 designed, run after run, while the pin is held
# exactly as designed. Zeroed, the teach planned around it and the replayed film drove the
# peg into the wall (5.5 deg, 8.5 mm short). It is a constant of the flag's grasp.
FLAG_HELD_DEEPER_MM = 9.8
# How far past the flag's grasp point the flange stands beyond the tool's own length.
#
# The flag stands ON its grasp shaft (the shaft is its lowest point, 12.5 mm below the grasp
# point), so the EZU's 170 mm fingers would reach into the feeder deck at the plain gap.
# 26 mm puts the finger tips 8.5 mm above the deck and the surface gripper's attach point
# 13.5 mm above the shaft's top, inside the 6-20 mm band it was measured to hold in.
FLAG_GAP_MM = 26.0


def _face_gap_mm(part_name: str, stood_off: Sequence[float]) -> float:
    """How far past the grasp POINT the cup has to sit to clear the face it meets.

    Reads the collider the grasp point is inside - not the part's overall bounds, which on
    the flag would measure to a plate 48 mm away from the shaft the tool actually takes -
    and returns how far that collider reaches along the stand-off, plus the cup's gap.

    Falls back to the plain gap when the grasp point is not inside any single collider,
    because a wrong face is worse than no face: the plain figure is the one every pick in
    this cell was built on and it is right for a crosswise approach.
    """
    from .colliders import pieces

    p = part(part_name)
    grasp = [v * 1000.0 for v in p.at("assembly:graspPosition")]
    for piece in pieces(part_name):
        if all(piece["min_mm"][i] - 0.01 <= grasp[i] <= piece["max_mm"][i] + 0.01
               for i in range(3)):
            # How far this collider reaches along the stand-off, from the grasp point.
            reach = max(
                sum(stood_off[i] * ((piece["max_mm"][i] if stood_off[i] > 0
                                     else piece["min_mm"][i]) - grasp[i])
                    for i in range(3)), 0.0)
            return reach + CUP_GAP_MM
    return CUP_GAP_MM


def _seat_work_mm(part_name: str, pre_mm, at):
    """Where the part's GRASP POINT will be at the start of its insert, in world mm.

    Not where its origin will be, which is what these stations used and what made the
    insert and the pick disagree by the whole length of the part.

    A station places the FLANGE at `work + approach * (standoff + extra)` - a scalar along
    one axis. The pick leaves the flange a known distance from the part's GRASP POINT, so
    the seat can only reproduce that relationship if its work point is the grasp point
    too. Aim at the ORIGIN instead and the two differ by the grasp offset, which on the
    flag is 205 mm because its grasp shaft is at the far end of a 252 mm blade. Measured,
    the live correction had to step exactly that: "the Flag sits 208.19 mm off the start of
    its stroke".

    With the work point moved, the taught pose already holds the part where the insert
    wants it and the correction has almost nothing left to do - which matters beyond
    tidiness, because a 208 mm correction is a 208 mm planned move with a 252 mm blade on
    the end of it, and it was converging at about 0.65 per pass rather than closing.
    """
    grasp = [v * 1000.0 for v in part(part_name).at("assembly:graspPosition")]
    return at(tuple(pre_mm[i] + grasp[i] for i in range(3)))


def _pick_station(cell, part_name: str, task: str, index_x: float) -> "Station":
    """A pick from the part's FEEDER: from above, at its published grasp point.

    The work point is the grasp point where the feeder presents it, rotated by the pose the
    feeder holds the part in (the pin stands turned 90 degrees, so an unrotated grasp
    offset would aim 11.5 mm sideways). The flange stands off straight up. A feeder is a
    fixed point, so the station does not ride the belt.
    """
    return Station("assemble", task, feed_grasp_mm(cell, part_name), (0.0, 0.0, 1.0),
                   "jaw", f"{part_name} assembly:graspPosition + fixture.FEEDERS", index_x,
                   on_belt=False, extra_mm=jaw_extra_mm(part_name),
                   theta_deg=(_pick_theta_deg(part_name) if part_name == "Flag" else 0.0))


def _ov_rows(o, theta_deg):
    """Rows of the rotation a viam orientation vector describes (RDK's ZYZ: Rz(lon)
    Ry(lat) Rz(theta)) - the same as reach_check._ov_matrix, here to avoid importing it."""
    ox, oy, oz = o
    lat = math.acos(max(-1.0, min(1.0, oz)))
    lon = math.atan2(oy, ox) if 1.0 - abs(oz) > 1e-4 else 0.0

    def rz(a):
        return [[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0],
                [0.0, 0.0, 1.0]]

    def ry(a):
        return [[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0],
                [-math.sin(a), 0.0, math.cos(a)]]
    return _mat(_mat(rz(lon), ry(lat)), rz(math.radians(theta_deg)))


def _mat(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _quat_rows(q):
    w, x, y, z = q
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]


def _pick_theta_deg(part_name: str) -> float:
    """The tool roll at the pick that brings the part to its insert SEATED-ORIENTED.

    The jaw holds the part rigidly, so the hold H is fixed: at the insert the part is in
    its seated orientation (the shell's, square: identity) with the tool at
    orientation_for((0,1,0), 0), so H = T_insert^-1; at the pick the part lies as the
    feeder holds it, R_pick, so the tool must be R_pick * T_insert. Solved for the roll
    about the straight-down approach. For the flag this decides which way its plate
    points when it reaches the hole - rearward, along -X, as the usd session built it.
    """
    from .fixture import feed_orientation
    want = _mat(_quat_rows(feed_orientation(part_name)), _ov_rows((0.0, -1.0, 0.0), 0.0))
    best = None
    for k in range(3600):
        t = k / 10.0 - 180.0
        r = _ov_rows((0.0, 0.0, -1.0), t)
        err = sum((r[i][j] - want[i][j]) ** 2 for i in range(3) for j in range(3))
        if best is None or err < best[0]:
            best = (err, t)
    if best[0] > 1e-3:
        raise ValueError(f"{part_name}: no roll about a straight-down pick reproduces the "
                         f"insert's hold (residual {best[0]:.3g}) - the feeder attitude and "
                         f"the insert axis do not agree")
    return best[1]


TOOLS = {
    # Read off cell_props/fisheye_tool.usda, not chosen: its `optics:opticalCenter` - the
    # lens's front vertex - is 227 mm out from the flange face.
    "fisheye": Tool("fisheye", 227.0,
                    "Basler ace 2 behind a FUJINON FE185C057HA-1: a 185 degree "
                    "equidistant fisheye on a stem, optical vertex 227 mm from the "
                    "flange", lens_mm=227.0),
    # The lens sits at the FINGERTIP plane (175 mm), 130 mm beside the axis, on a bracket:
    # at 40 mm it was behind the EZU's 195 mm body and the jaw filled 40% of every look
    # frame, with the flag hole out of shot (measured 2026-10-13). 450 mm of standoff
    # leaves the hole ~300 mm from the lens - still about 26 px for an 8 mm hole - and
    # tilts the camera in only 25 degrees.
    "realsense": Tool("realsense", 450.0,
                      "eye-in-hand on a bracket at the fingertips; ~300 mm lens to "
                      "feature puts an 8 mm hole across about 26 pixels", lens_mm=175.0),
    # Both of these are now REAL assets, and their standoffs are read off the asset
    # rather than chosen. A number picked for a cuboid stand-in and then kept after the
    # real tool went on is the single bug class this whole module is arranged against.
    #
    # Robotiq Hand-E: 151 mm from mounting face to finger tip, measured off
    # Robots/Robotiq/Hand-E/Robotiq_Hand_E_base.usd - base_link spans its own y -91..13
    # and the fingers reach to +60. The pads grip about 10 mm back from the tip.
    # Schunk EZU 35, three jaws at 120 degrees, from Isaac 6.1's own asset: body 0..124.5 mm
    # from the mounting face, finger tips at 170. 134 is the surface gripper's attach point:
    # with each pick's gap it puts the jaw pads' middle on the part's grasp point.
    "jaw": Tool("jaw", JAW_ATTACH_MM, "SCHUNK EGP 40 parallel gripper, slim V fingers, grip centre "
                "186.5 mm out; the pin, the flag"),
    # Universal Robots' own short suction unit, 161 mm from flange to cup face, measured
    # off Robots/UniversalRobots/ur10/grippers/short_gripper.usd.
    "big": Tool("big", 161.0, "UR short suction unit; the shell, by its flat side wall"),
}


@dataclass(frozen=True)
class Station:
    role: str          # inspect | assemble | handle
    task: str
    work_mm: Vec3      # in WORLD mm - the thing being worked on
    approach: Vec3     # unit vector the flange stands off along
    tool: str
    derived_from: str  # which published attribute this came from
    index_x_mm: float = 0.0   # where the belt had stopped the shell for this one
    # Does this station ride the belt? A feeder rack and a repaint belt do NOT - they are
    # fixed points in the cell - and treating them as though they did made the index
    # solver place the belt so the RACK landed under the arm, which threw the door seat
    # 460 mm away and read as 1203 mm of reach.
    on_belt: bool = True
    # Extra stand-off along the approach, beyond the tool's own length. Two uses, one
    # meaning: the distance between the tool's working point and the station's work point.
    #
    # For a CAMERA it is the optical gap. The fisheye's lens is at its stem's tip, so a
    # station whose work point is a surface puts the lens ON that surface: the back-cap
    # shot photographed the shell's unlit interior from inside its own back wall. `inside`
    # is the opposite case and WANTS the lens 70 mm in, so this cannot be a property of the
    # tool - it belongs to the station.
    #
    # For a GRIPPER it is how far the grasp face stands off the part's ORIGIN. A published
    # seatedPosition is the part's origin, and a suction cup holds a face: the flag's blade
    # runs from -32.5 to +27.5 about its origin, so a cup placed at the origin is 27.5 mm
    # inside the part. The same number has to appear at the pick and at the seat, or the
    # part is held at one offset and placed at another - which is the single bug class this
    # whole module is arranged to prevent.
    extra_mm: float = 0.0
    # Roll of the tool about its own axis, degrees, as a viam orientation-vector theta.
    # Zero everywhere but the flag's pick: the flag waits on its feeder turned to the
    # feeder's angle, and gripped at roll 0 it arrived at the hole turned with it - its
    # 250 mm arm swept into the shell and knocked it 36 degrees out of its cradle.
    theta_deg: float = 0.0

    @property
    def key(self) -> str:
        return f"{self.role}/{self.task}"

    def flange_mm(self) -> Vec3:
        d = TOOLS[self.tool].standoff_mm + self.extra_mm
        return tuple(self.work_mm[i] + self.approach[i] * d for i in range(3))

    def lens_mm(self) -> Vec3:
        """Where the camera's lens ends up - which is what decides what is in frame.

        The lens sits `lens_mm` DOWN THE TOOL AXIS from the flange, and the tool points
        along -approach, so that is all this is. The first version subtracted
        `standoff + extra - lens` instead, which is the distance from the lens to the WORK,
        and returned a point that distance back from the flange - so it reported the
        fisheye's lens at the flange when the lens is at the stem's tip, and put the
        back-cap lens 190 mm from the cap when the station is built to put it at 90.
        Nothing was driven from it - the camera's actual mount comes from `TOOL_LENS` in
        the generator, which was always right - but it is a number I quoted while
        reasoning about that station, and a wrong number quoted is a wrong number believed.
        """
        f = self.flange_mm()
        return tuple(f[i] - self.approach[i] * TOOLS[self.tool].lens_mm
                     for i in range(3))


def _unit(v: Sequence[float]) -> Vec3:
    n = math.sqrt(sum(c * c for c in v)) or 1.0
    return tuple(c / n for c in v)


def stations_for(cell: Cell) -> List[Station]:
    """The cycle's stations, in order, in world millimetres.

    The belt INDEXES the shell from one station to the next, which is the whole point of a
    conveyor that runs the length of the cell and the thing I built wrong first. Pinned at
    one work position the cycle's stations spanned about 900 mm of x - the fisheye enters
    the front at one end and photographs the back cap at the other, the door caps the front
    while the flag hole is 370 mm behind it - and no arm's 30 degree pan cone covers that.
    Every sweep failed on it, at best by 15 mm of reach and 2 degrees of pan.

    Indexed, each arm only ever works near its own pedestal and the belt brings the shell
    to it. It is also how a line actually works, and it means the two inspection shots can
    share ONE flange pose: advance the shell by its own length and the back cap is where
    the front aperture was.
    """
    shell = part("Shell")
    ins = insertions()
    _, oy, oz = cell.shell_origin_mm

    def at(index_x: float):
        def world(local: Sequence[float]) -> Vec3:
            return (index_x + local[0], oy + local[1], oz + local[2])
        return world

    aperture = shell.mm_at("assembly:doorAperture:center")
    aperture_n = _unit(shell.at("assembly:doorAperture:normal"))
    hole = shell.mm_at("assembly:flagHole:entry")
    hole_axis = _unit(shell.at("assembly:flagHole:axis"))     # (0,-1,0), into the wall
    back_x = shell.mm_at("assembly:wall:backX")
    _, high = shell.bounds_mm

    out: List[Station] = []

    # 2 - the fisheye goes in through the door aperture, 70 mm past its plane.
    ix = cell.index_x("inside")
    out.append(Station("inspect", "inside",
                       at(ix)((aperture[0] - 70.0, aperture[1], aperture[2])),
                       aperture_n, "fisheye", "assembly:doorAperture:center/normal", ix))
    # 3 - the RealSense looks at the flag hole, standing off along the hole's own axis.
    ix = cell.index_x("flag")
    out.append(Station("assemble", "flag_hole_look", at(ix)(hole),
                       tuple(-c for c in hole_axis), "realsense",
                       "assembly:flagHole:entry/axis", ix))
    # 4 - the flag, held from above by its plate, lowered onto the hole's axis.
    #     The INSERT axis is -Y and the GRASP approach is -Z; the flange is above the
    #     flag's origin, which is why the two cannot share one standoff.
    # Aimed at where the flag's GRASP POINT will be, not its origin - see `_seat_work_mm`.
    # `extra_mm` is the pick's, because the relationship the pick establishes is the one
    # this has to reproduce; the blade's own 27.5 mm top face was a different number for a
    # different question.
    # The PICK first: it has one wrist branch that reaches the feeder, and the insert is
    # taught from it (reach_check) so the carry between them keeps the arm's shape.
    out.append(_pick_station(cell, "Flag", "flag_pick", ix))
    # The flag is inserted exactly like the pin now: held coaxially by its collar, the
    # flange standing off along +Y, the stroke along -Y.
    out.append(Station("assemble", "flag_insert",
                       _seat_work_mm("Flag", ins["Flag"].pre_mm, at(ix)),
                       (0.0, 1.0, 0.0), "jaw", "Flag assembly:seatedPosition/insertAxis",
                       ix, extra_mm=jaw_extra_mm("Flag") + FLAG_HELD_DEEPER_MM))
    # 4a - the flag, off its spot on the carrier tray.
    #
    # Grasped from ABOVE: the flag publishes graspApproach (0,0,-1), so the tool comes down
    # and the flange stands off along +Z. That is a different vector from the insert axis
    # (0,-1,0), which is the whole reason this part needs a pick station and a seat station
    # rather than one pose - the cup holds the blade's top face while the peg goes sideways
    # into the wall.
    # The PIN index, not the door's.
    #
    # The door stations were removed - the door arrives already on the shell, and fetching
    # and capping one first cost two of arm-b's eight stations for something a line does
    # upstream - but this `ix` was left behind pointing at the door's index, 152.3, while
    # the cycle indexes the pin step to 363.8. So both pin stations were sited 211.5 mm
    # from where the pin actually stops, and the pick aimed at bare belt.
    #
    # It passed every run. The grab caught nothing, the arm carried nothing, and the
    # `seat:Pin` step that followed called `join`, which snaps the pin onto its published
    # frame whether or not anything ever held it - so the cell reported the pin seated
    # 0.00 mm from its published position while the pin had not moved at all.
    ix = cell.index_x("pin")
    # 6 - the pin: taken off the tray it arrived on, and driven -Y through the bore.
    #     Both at the same index, because the tray rides with the part.
    out.append(_pick_station(cell, "Pin", "pin_pick", ix))
    # Coaxial with the pin, and the PICK is what was made to match rather than this.
    #
    # The wrist was turning 90 degrees between the pick and the insert, because the pick
    # stood off along +Z and this stands off along +Y. The pin, rigidly held, turned with
    # it: a rod lying along Y arrived at the bore standing along Z, and finished 78.26 mm
    # from its seat with the shell at 0.1 degrees and nothing else to blame.
    #
    # Turning the INSERT crosswise to match the pick was tried on paper first and does not
    # fit. The Hand-E is declared to the planner as a 53 mm capsule, and a crosswise insert
    # ends with that capsule beside the hinge at shell-local x 236 - which reaches back
    # 53 mm into `WallR_Front` (x -130.5..235, y 78..80) and `Arch00` above it. The
    # gripper is simply too fat to stand next to the hinge; measuring the shell's real
    # collider pieces rather than its bounding box does not save it.
    #
    # Coaxial is also how a pin is actually pressed: the fingers run parallel to the shaft
    # and pinch the collar, and the gripper body trails BEHIND the pin along its axis,
    # outside the shell for the whole 138 mm stroke. So the insert keeps its axis and
    # `TRAY_APPROACH` brings the pick round to meet it.
    # The wrist camera looks at the BORE before the insert, as it does at the flag hole:
    # the course has learners locate both from this camera. Standing off along the bore's
    # own axis, at the pin's belt stop.
    bore = shell.mm_at("assembly:pinBore:entry")
    bore_axis = _unit(shell.at("assembly:pinBore:axis"))
    # 80 mm closer than the RealSense's 450, and from 20 degrees ABOVE the bore's axis:
    # level at 450 arm-b's forearm met its own pedestal, and level at 370 the tool met
    # the belt's guide (2026-10-13). The bore sits a little off the frame's centre.
    _up = math.tan(math.radians(BORE_LOOK_ELEVATION_DEG))
    _look = _unit((-bore_axis[0], -bore_axis[1], -bore_axis[2] + _up))
    # -50, not -80: 30 mm further off, so the pin hanging 330 mm down the tool axis from the
    # knob grip stays clear of the shell (the planner refused -80 once it saw the pin).
    out.append(Station("assemble", "pin_bore_look", at(ix)(bore), _look, "realsense",
                       "assembly:pinBore:entry/axis", ix, extra_mm=-50.0))
    out.append(Station("assemble", "pin_insert",
                       _seat_work_mm("Pin", ins["Pin"].pre_mm, at(ix)),
                       (0.0, 1.0, 0.0), "jaw", "Pin assembly:seatedPosition/insertAxis",
                       ix, extra_mm=jaw_extra_mm("Pin"), theta_deg=PIN_INSERT_ROLL_DEG))
    # 7 - a photograph of the back cap, from behind.
    # 7 - the back cap, photographed from above and behind rather than dead-on.
    #
    # Dead-on put the camera at x 300 on the belt's own centreline with the tool pointing
    # +X, and the planner refused it: a collision, with the arm having to come in low over
    # a 4.6 m conveyor. It is also the wrong claim I made for the indexing - the two
    # inspection shots do NOT share one flange pose. They share one flange POSITION, at
    # x 300, and the tool points opposite ways at it, which is a different configuration
    # and only one of them is clear.
    #
    # Angled up at 40 degrees the flange lifts clear of the belt and stands off behind it,
    # which is also how anyone photographs a vertical face without shooting its own glare
    # back down the lens.
    # (No back-cap shot: it left the cycle on 2026-10-12, and taught anyway it capped how
    # high the fixture may stand - at the 120 mm lift it was over budget.)
    # 8 - handling: take the shell by its SIDE WALL, lift it clear of the lips, and turn
    #     it under the overhead camera.
    #
    # By the side and not the crown, for a reason the arithmetic settles. A grasp from
    # above puts the flange a tool length ABOVE the part: with the shell's crown at 1080
    # and a 210 mm gripper that is a flange at 1290, and once it is lifted to be turned,
    # 1590 - which is 970 mm above an arm based at 620 and simply not a pose a UR5e has.
    # Held by the side the flange is displaced in y instead, and the height it works at is
    # the part's own.
    #
    # The side wall is also the only flat face big enough: |y| = 80 over x -219..217 and
    # z 8..136, which the asset publishes as assembly:wall:side*.
    # The -Y wall, not the +Y one.
    #
    # The shell has a flat wall on both sides, so which one the handling arm takes is free -
    # and it decides which side of the belt that arm can stand on. Taking +Y put the grasp
    # flange at y +260 and the handling arm, on the far side, reaching 1136 mm across the
    # conveyor for it. The assemble arm already owns +Y, because the flag hole and the pin
    # bore are both entered from there; handling takes -Y and the two arms never share a
    # side or reach over each other.
    wall_y = -shell.mm_at("assembly:wall:sideY")
    wall_mid_z = (shell.mm_at("assembly:wall:sideZMin")
                  + shell.mm_at("assembly:wall:sideZMax")) / 2.0
    ix = cell.index_x("handle")
    side = at(ix)((shell.centre_offset_mm[0], wall_y, wall_mid_z))
    out.append(Station("handle", "grasp", side, (0.0, -1.0, 0.0), "big",
                       "assembly:wall:sideY/sideZMin/sideZMax", ix,
                       extra_mm=CUP_GAP_MM))
    # (The two paint stops - lifted crown-up, then rolled +Y-up - are gone: the vision
    # tunnel sees the crown and both walls with the shell on the belt. Brandon, 2026-10-12.)
    out.append(Station("handle", "return", side, (0.0, -1.0, 0.0), "big",
                       "assembly:wall:sideY/sideZMin/sideZMax", ix,
                       extra_mm=CUP_GAP_MM))
    # 9 - a reject is set down on the REJECT SPUR beside arm-c, held as it was taken:
    #     the same -Y wall, the shell square, its rails on the spur's deck.
    #     It arrives turned 180 degrees - arm-c carries it by one base swing (film_cycle) -
    #     so the wall it is held by faces +Y, toward the arm, and the tool stands off +Y.
    rx, ry, rz = reject_origin_mm(cell)
    out.append(Station("handle", "repaint",
                       (rx - shell.centre_offset_mm[0], ry - wall_y, rz + wall_mid_z),
                       (0.0, 1.0, 0.0), "big", "reject spur: cell.repaint_x/y_mm", ix,
                       on_belt=False, extra_mm=CUP_GAP_MM))
    return out


def reject_origin_mm(cell) -> Vec3:
    """Where a rejected shell's ORIGIN stands on the reject spur: its rails on the deck,
    which is at the main belt's height."""
    rail_bottom = part("Shell").mm_at("assembly:wall:railBottomZ")
    return (cell.repaint_x_mm, cell.repaint_y_mm, cell.belt_top_mm - rail_bottom)


# ---- who covers what ---------------------------------------------------------

THREE = {"inspect": "arm-a", "assemble": "arm-b", "handle": "arm-c"}
# Two arms: handling joins the inspection arm, which is what forces a tool change - the
# fisheye and a shell-sized gripper cannot share a wrist.
TWO = {"inspect": "arm-a", "assemble": "arm-b", "handle": "arm-a"}


def verdict(cell: Cell, roles: Dict[str, str]) -> Tuple[float, List[str]]:
    """Worst margin over every station, and every fault. Higher margin is better."""
    worst, faults = 1e9, []
    for st in stations_for(cell):
        arm = roles[st.role]
        if arm not in cell.arm_bases_mm:
            faults.append(f"{st.key}: no arm {arm}")
            continue
        flange = st.flange_mm()
        reach = cell.reach_mm(arm, flange)
        pan = abs(cell.pan_deg(arm, flange))
        up = cell.up_mm(arm, flange)
        worst = min(worst, REACH_BUDGET_MM - reach, (MAX_PAN_DEG - pan) * 10.0,
                    MAX_UP_MM - up)
        if reach > REACH_BUDGET_MM:
            faults.append(f"{st.key} [{arm}]: reach {reach:.0f} > {REACH_BUDGET_MM:.0f}")
        if pan > MAX_PAN_DEG:
            faults.append(f"{st.key} [{arm}]: pan {pan:.0f} > {MAX_PAN_DEG:.0f}")
        if up > MAX_UP_MM:
            faults.append(f"{st.key} [{arm}]: up {up:.0f} > {MAX_UP_MM:.0f}")
    return worst, faults
