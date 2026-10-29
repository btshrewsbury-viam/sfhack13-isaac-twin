"""Ask the PLANNER whether every station the sweep sited is actually reachable, and save
the joint configuration it found.

The sweep answers reach, pan and height with arithmetic. That is not the same as an IK
solution, and the fable review named the gap exactly: a harness that re-implements the cell
answers against its own model. The previous build has the scar - a pick window reported
50 mm wide against a belt box that had been invented, and a fixture sited against a pose no
arm ever visited, both of which passed every offline test.

So every station gets one real motion-service call, and what comes back is saved as that
arm's taught pose for that task - with the provenance that makes it refusable later.
"""
from __future__ import annotations

import asyncio
import json
import math
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from assembly.asset import FRAMES                                  # noqa: E402
from assembly.build_fragment import FRAGMENT, chosen               # noqa: E402
from assembly.creds import options                             # noqa: E402
from assembly.arms import adrift, clear_and_park, home as _home_joints  # noqa: E402
from assembly.fixture import (CARRIED as SADDLE, index_to,          # noqa: E402
                               indexed_mm as _moved)
from assembly.stations import THREE, TOOLS, stations_for           # noqa: E402
from viam.components.arm import Arm, JointPositions                # noqa: E402
from viam.components.generic import Generic                        # noqa: E402
from viam.proto.common import (Geometry, GeometriesInFrame, Pose, PoseInFrame,
                               RectangularPrism, Vector3, WorldState)   # noqa: E402
from viam.robot.client import RobotClient                          # noqa: E402
from viam.services.motion import MotionClient                      # noqa: E402

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
POSES = Path(__file__).resolve().parents[2] / "fragments" / "poses"


def layout_hash() -> str:
    """A fingerprint of the geometry a pose was taught against.

    The countermeasure the fable review asked for, and the one the previous build did not
    have. `probes/stations.json` there went stale in silence - its own docstring records
    that "a stored pose cannot notice that the cell moved" - and four separate faults came
    from numbers outliving the grasp or the layout they were true of. A pose that carries
    the hash of the cell and the asset it was taught against CAN notice, and the loader
    refuses it rather than driving to it.
    """
    import hashlib
    # GEOMETRY only, not the whole fragment.
    #
    # Hashing the file was too strict and said so immediately: adding `home_joints_deg`
    # invalidated all eleven poses, and a park configuration does not move anything a pose
    # was taught against. The guard has to fire when the CELL MOVES - a prop, an arm's base
    # or facing, a camera, or the asset's own published frames - and stay quiet otherwise,
    # or it gets switched off the first time it cries wolf.
    fragment = json.loads(FRAGMENT.read_text())
    geometry = []
    for component in fragment["components"]:
        if component["model"].endswith(":scoreboard"):
            continue                    # counters, not geometry: it has no place in the cell
        if component["model"].endswith(":world"):
            geometry.append(("props", _geometry_only(component["attributes"]["props"])))
        else:
            geometry.append((component["name"], component.get("frame"),
                             component["attributes"].get("position"),
                             component["attributes"].get("yaw_deg"),
                             component["attributes"].get("target"),
                             component["attributes"].get("joint_limits_deg"),
                             component["attributes"].get("joint_limit_deg")))
    # The STATIONS too, not just the cell they stand in.
    #
    # A station is a derived pose, and changing how it is derived moves it just as surely
    # as moving an arm's base does. Giving the flag stations a 27.5 mm grasp offset - so
    # the cup sits on the blade's top face instead of 27.5 mm inside it - moved two flange
    # poses and left the fragment byte-identical, so every pose taught against the old
    # definition would have passed the guard and driven to the wrong place. That is the
    # exact failure this hash exists to prevent, one level up from where it was looking.
    stations = [(st.key, [round(v, 3) for v in st.flange_mm()], list(st.approach),
                 st.tool, st.extra_mm, round(st.index_x_mm, 3), st.on_belt)
                for st in stations_for(chosen())]
    h = hashlib.sha256()
    h.update(json.dumps(geometry, sort_keys=True).encode())
    h.update(json.dumps(stations, sort_keys=True).encode())
    h.update(FRAMES.read_bytes())
    return h.hexdigest()[:16]


# Prop fields that do not move anything, and so must not invalidate a taught pose.
#
# The guard has to fire when the CELL MOVES and stay quiet otherwise, or it gets switched
# off the first time it cries wolf - which is why it hashes geometry rather than the whole
# fragment. Mass is the same case as the park configuration that prompted that rule:
# changing a pallet from 6 kg to 40 kg, which is a holding-force decision and moves nothing,
# invalidated all thirteen stations and thirteen stand-offs. Colour and friction likewise.
NON_GEOMETRIC_PROP_KEYS = ("color", "mass_kg", "friction")


def _geometry_only(props):
    """Props with the fields that cannot move anything stripped out."""
    out = []
    for prop in props:
        if prop.get("obstacle") is False:
            continue        # scenery (the warehouse): nothing taught can depend on it
        clean = {k: v for k, v in prop.items() if k not in NON_GEOMETRIC_PROP_KEYS}
        # Where an asset's file sits is not geometry: a fragment pointing at the cell's
        # asset package hashes as the workstation's does.
        from assembly.build_fragment import canonical_path
        if "usd_path" in clean:
            clean["usd_path"] = canonical_path(clean["usd_path"])
        if "children" in clean:
            clean["children"] = [
                {k: (canonical_path(v) if k == "usd_path" else v) for k, v in child.items()
                 if k not in NON_GEOMETRIC_PROP_KEYS}
                for child in clean["children"]]
        if isinstance(clean.get("conveyor"), dict):
            clean["conveyor"] = {k: v for k, v in clean["conveyor"].items()
                                 if k != "speed_mm_s"}
        out.append(clean)
    return out


def orientation_for(approach, theta_deg: float = 0.0) -> dict:
    """The flange orientation that points the tool at the work.

    The tool points along -approach: `approach` is the direction the flange stands OFF in,
    so the tool looks back down it. Getting this backwards is how a cell photographs the
    wall behind the part.
    """
    o = tuple(-c for c in approach)
    return {"o_x": o[0], "o_y": o[1], "o_z": o[2], "theta": float(theta_deg)}


IDENTITY = {"o_x": 0.0, "o_y": 0.0, "o_z": 1.0, "theta": 0.0}


# How much to grow each obstacle before handing it to the planner.
#
# The planner's model of a ur5e's links is thinner than the mesh isaac collides with, so a
# path that clears in the plan does not clear in the cell: measured at the `inside` station,
# arm-a's wrist_1 ended up 18.5 mm inside the conveyor in y and 47.3 in z, stalled at
# 318 N.m, from a plan the motion service was happy with.
#
# The PART is deliberately not padded, and cannot be: the fisheye's whole job is to be 70 mm
# inside the shell's aperture, so a shell grown by 50 mm makes its own inspection station
# unreachable. Padding belongs on the things no tool is ever meant to touch.
BELT_PAD_MM = 60.0
HELD_PAD_MM = 10.0
PICK_FROM_INSERT_LINE_MM = 120.0
PICK_FROM_INSERT_TURN_DEG = 40.0          # around a part an arm is carrying
FIXTURE_PAD_MM = 20.0
# And the same allowance on the other arms, for the same reason and one more.
#
# A link's bounding box is the box of a CYLINDER, so it is already generous on the
# diagonals and tight nowhere - except that the planner's model of the arm it is moving is
# thinner than the mesh, which is what the belt padding is for. The cost of being wrong here
# is a cell where one station is always marginal and which one moves every time a park does:
# parking the arms behind their bases fixed the door placement and broke the flag-hole look,
# each as a 500 N.m stall on a path the planner was happy with.
ARM_PAD_MM = 50.0
# How far any single joint may move between a station and its own stand-off before the
# stand-off is treated as a different arm configuration rather than an approach to the same
# one. A stand-off is 50-150 mm away; a joint that has to travel 90 degrees to cover that is
# not approaching, it is flipping.
BRANCH_FLIP_DEG = 60.0


def _swing(a, b) -> float:
    """The largest joint motion between two configurations of the same arm.

    Wrist_3 is excluded and the rest are compared modulo a full turn. A viam orientation
    vector's theta does not pin wrist roll, so the planner is free to answer the same
    flange pose with the tool rolled half a turn - and it does, which made every stand-off
    on two arms look like a 180 degree branch flip when the arm was not changing shape at
    all. Both tools on this cell are symmetric about the tool axis, a suction cup and a
    camera, so that roll costs nothing but time. A full revolution on any joint is the same
    pose by definition.
    """
    worst = 0.0
    for i in range(5):                       # j0..j4; j5 is the roll
        d = abs(a[i] - b[i]) % 360.0
        worst = max(worst, min(d, 360.0 - d))
    return worst
UNPADDED = ("shell", "door", "flag", "pin")


def _pad_for(label: str) -> float:
    if any(label.startswith(p) for p in UNPADDED):
        return 0.0
    return BELT_PAD_MM if label == "belt" else FIXTURE_PAD_MM


# A ur5e's links, as the asset names them.
# Where an arm waits: this far BEHIND its own base along its facing, this far above it.
#
# Behind, not in front. Parked 300 mm forward, arm-a's wrist sat where arm-b needed to be to
# cap the door, and the planner refused the station - correctly, and only because it can see
# the other arms at all now. A park should be the pose that is out of everyone's way, and
# "out of the way" in a cell whose work all happens over one belt means back off the belt.
PARK_OUT_MM = -320.0
# Arms that park near their first station instead: (first station, lift above its stand-off).
# Not arm-a any more: with the overhead camera's boom over the infeed every lifted park over
# `inside` is out of reach, and the fallback that remained hung its wrist over the crown -
# the shell ran into it at the next index, three teaches running (2026-10-13). It parks up
# and back over its own base.
# Nor arm-c: parked over the handling stop, its wrist and arm-a's forearm stood inside each
# other for the whole cycle (audit, 2026-10-13), and the arm-a park timed out against it.
# Both wait up and back over their own bases.
PARK_AT: dict = {}
# And AWAY from each other along the line: parked straight back, 560 mm apart with the same
# joint shape, arm-a's wrist sat inside arm-c's forearm for the whole second half of the film
# (audit, 2026-10-13). A planned move to a park the arm already stands at is not checked,
# so the poses have to be right by construction.
PARK_ASIDE_MM = {"arm-a": -250.0, "arm-c": 250.0}
# The volume the belt sweeps the shell and its flat door through, which no arm may wait in:
# the shell's width plus this, from the deck to above the crown by this.
SWEEP_MARGIN_MM = 60.0
# Wider to the SIDES: the planner's link model is thinner than the meshes. arm-a's park with
# its flange 195 mm off the centreline passed at 60 mm and its wrist still caught the
# crown's edge (audit + film, 2026-10-13).
SWEEP_SIDE_MM = 160.0


# Each arm's first station in the cycle: where it waits (waits.py).
FIRST_STATION = {"arm-a": "inside", "arm-b": "pin_pick", "arm-c": "grasp"}


def _wait_pose(cell, arm: str, firsts, stamp: str):
    """The arm's wait, from its first station's taught stand-off, or None."""
    from assembly import waits
    first = FIRST_STATION.get(arm)
    path = POSES / arm / f"{first}.clear.json"
    if first not in firsts or not path.exists():
        return None
    clear = json.loads(path.read_text())
    if clear.get("layout") != stamp:
        return None
    g = belt_sweep(cell)
    c, d = g.center, g.box.dims_mm
    lo = [c.x - d.x / 2, c.y - d.y / 2, c.z - d.z / 2]
    hi = [c.x + d.x / 2, c.y + d.y / 2, c.z + d.z / 2]
    radius, length = TOOL_ENVELOPE[arm]
    w = waits.wait_for(clear["joints_deg"], cell.arm_bases_mm[arm], cell.arm_yaw_deg[arm],
                       length, lo, hi, radius)
    if w is None:
        print(f"  park/{arm}: no wait clears the sweep from {first}'s stand-off", flush=True)
        return None
    w.update(approach=clear["approach"], theta_deg=clear.get("theta_deg", 0.0), of=first)
    print(f"  park/{arm}: waits over {first}, back {w['back_mm']} mm, up {w['lift_mm']} mm "
          f"({w['cost_deg']} deg from its stand-off)", flush=True)
    return w


async def step_out(arms, cell, arm: str) -> dict:
    """Between two of an arm's OWN stations at different stops: pull back and lift just
    out of the belt's sweep from where it stands, on its own branch (waits.py), instead of
    going to its wait. arm-b's park between the pin and the flag was 478 degrees of joint
    travel to a wait over the pin magazine; stepping out of the sweep from the insert's
    stand-off is about 20 (2026-10-14). A joint move, short and in the arm's own plane,
    retreating from the work - not a taught leg."""
    from assembly import waits
    g = belt_sweep(cell)
    c, d = g.center, g.box.dims_mm
    lo = [c.x - d.x / 2, c.y - d.y / 2, c.z - d.z / 2]
    hi = [c.x + d.x / 2, c.y + d.y / 2, c.z + d.z / 2]
    q = [float(v) for v in (await arms[arm].get_joint_positions()).values]
    radius, length = TOOL_ENVELOPE[arm]
    w = waits.wait_for(q, cell.arm_bases_mm[arm], cell.arm_yaw_deg[arm], length, lo, hi,
                       radius)
    if w is None:
        raise RuntimeError(f"{arm}: no step out of the belt's sweep from {q}")
    if w["cost_deg"] > 0.0:
        await arms[arm].move_to_joint_positions(JointPositions(values=w["joints_deg"]),
                                                timeout=60)
    return w


def belt_sweep(cell) -> "Geometry":
    xs = [cell.index_x(k) for k in cell.index_x_for]
    lo_x, hi_x = min(xs) - 300.0, max(xs) + 600.0          # the shell behind, the door ahead
    half_y = cell.shell_mm[1] / 2.0 + SWEEP_SIDE_MM
    lo_z = cell.belt_top_mm
    hi_z = cell.belt_top_mm + cell.fixture_lift_mm + cell.shell_mm[2] + SWEEP_MARGIN_MM
    return Geometry(center=Pose(x=(lo_x + hi_x) / 2.0, y=cell.belt_y_mm,
                                z=(lo_z + hi_z) / 2.0, **IDENTITY),
                    box=RectangularPrism(dims_mm=Vector3(x=hi_x - lo_x, y=2.0 * half_y,
                                                         z=hi_z - lo_z)),
                    label="belt-sweep")
PARK_STANDOFF_MM = 150.0
ON_SHELL_MM = 500.0     # a fitted part's origin is within this of the shell's (door: 435)
PARK_UP_MM = 430.0

ARM_LINKS = ("shoulder_link", "upper_arm_link", "forearm_link",
             "wrist_1_link", "wrist_2_link", "wrist_3_link")
ARMS = ("arm-a", "arm-b", "arm-c")


# Each arm's TOOL, as the planner sees it: a capsule along the end frame's +z (the tool
# axis), from the flange out to the tool's tip, in mm. Measured from the assets in their
# own frames (2026-10-12): the fisheye with its ring flash 0..227 mm, radius 52; the EZU's
# body 0..125 radius ~60-80 (a connector to 100 on one side) and its open fingers 125..170;
# the suction unit -4..161, radius 71. Before this the planner knew no tool at all, and
# paths carried them through the shell.
# arm-c: radius 60 over the UR suction unit's ~53, length 165 - at 71/172 the capsule met the
# crown at the reject grasp once the cup came 6 mm closer (CUP_GAP_MM 14 -> 8, 2026-10-13).
# arm-b: the SCHUNK EGP 40 on its 54.5 mm mount (Sam's gripper:envelopeRadius/Length,
# 2026-10-19); the EZU 35 it replaced was 75 x 172.
TOOL_ENVELOPE = {"arm-a": (52.0, 232.0), "arm-b": (31.5, 193.5), "arm-c": (60.0, 165.0)}
# The D435 and its bracket in arm-b's end frame (the camera is at y -130, z 175 there):
# centre x, y, z and size, from wrist_d4xx.usda's bounds plus a few mm.
WRIST_CAMERA_BOX_MM = (0.0, -120.0, 132.0, 120.0, 60.0, 100.0)


async def declared(world, moving: str | None = None, *, motion=None,
                   grip=None, attach: bool = True, tools: bool = False) -> WorldState:
    """The cell, as the planner must see it: the props AND the other arms.

    Without the props the planner knows only the robot's own links: it planned the back-shot
    straight through the saddle and the arm pressed on it at 676 N.m, 11.3 degrees short and
    not moving. The ten stations that passed did so because their paths happened to miss.
    A reachability answer from a planner that cannot see the cell is not an answer.

    Without the OTHER ARMS it is the same failure one level up. Three arms stand around a
    420 mm belt; the motion service plans for one at a time and this module's arms publish
    no geometry, so the other two are invisible. Measured, arm-a drove its wrist into a
    neighbour's upper arm and stalled there at 320 N.m - twice, at the identical
    configuration, which is what a fixed obstacle looks like and what a planning failure
    does not. Their live link bounds go in as boxes, which is coarse for a cylinder and far
    better than nothing.
    """
    obstacles = (await world.do_command({"command": "obstacles"}))["obstacles"]
    held, transforms = [], []
    if moving and motion is not None and grip is not None:
        held, transforms = await _held_geometry(world, motion, grip, moving)
        if not attach:
            # Dropped from the world but not carried: setting a part DOWN, where its box
            # carried on the arm would collide with whatever it is being set onto.
            transforms = []
    boxes = []
    for o in obstacles:
        if o["label"] == "ground":
            continue
        if any(o["label"] == h or o["label"].startswith(h + ".") for h in held):
            continue                    # it rides with the arm now; see _held_geometry
        c, d = o["center_mm"], o["dims_mm"]
        pad = _pad_for(o["label"])
        turn = dict(IDENTITY, theta=float(o.get("yaw_deg") or 0.0))
        boxes.append(Geometry(center=Pose(x=c[0], y=c[1], z=c[2], **turn),
                              box=RectangularPrism(dims_mm=Vector3(
                                  x=d[0] + pad, y=d[1] + pad, z=d[2] + pad)),
                              label=o["label"]))
    for arm in ARMS:
        if arm == moving:
            continue
        prim = arm.replace("-", "_")
        paths = [f"/World/{prim}/{link}" for link in ARM_LINKS]
        found = (await world.do_command({"command": "prim_bounds",
                                         "paths": paths}))["bounds"]
        for path, b in found.items():
            if "min" not in b or "max" not in b:
                continue
            low = [float(v) * 1000.0 for v in b["min"]]
            high = [float(v) * 1000.0 for v in b["max"]]
            dims = [max(high[i] - low[i], 1.0) + ARM_PAD_MM for i in range(3)]
            centre = [(high[i] + low[i]) / 2.0 for i in range(3)]
            boxes.append(Geometry(
                center=Pose(x=centre[0], y=centre[1], z=centre[2], **IDENTITY),
                box=RectangularPrism(dims_mm=Vector3(x=dims[0], y=dims[1], z=dims[2])),
                # Hyphen, not colon: viam reads `remote:resource` in a name, so a label
                # like "arm-b:shoulder_link" comes back as a duplicate-geometry error
                # naming a geometry that appears exactly once in what was sent.
                label=f"{arm}-{path.rsplit('/', 1)[-1]}"))
    if tools and moving in TOOL_ENVELOPE:
        # The tool rides the arm: attached for the moves BETWEEN stations. The final
        # approach along the tool axis is planned without it - at a pick or an insert the
        # jaw is meant to be around the part and against the shell.
        from viam.proto.common import Capsule, Transform
        r, length = TOOL_ENVELOPE[moving]
        transforms = list(transforms) + [Transform(
            reference_frame=f"tool-{moving}",
            pose_in_observer_frame=PoseInFrame(reference_frame=moving, pose=Pose(
                x=0, y=0, z=length / 2.0, **IDENTITY)),
            physical_object=Geometry(center=Pose(x=0, y=0, z=0, **IDENTITY),
                                     capsule=Capsule(radius_mm=r, length_mm=length),
                                     label=f"tool-{moving}"))]
        if moving == "arm-b":
            # And the RealSense on its bracket, 130 mm aside - outside the jaw's capsule,
            # so without this the planner would route the camera through things.
            cx, cy, cz, dx, dy, dz = WRIST_CAMERA_BOX_MM
            transforms.append(Transform(
                reference_frame="tool-arm-b-camera",
                pose_in_observer_frame=PoseInFrame(reference_frame=moving, pose=Pose(
                    x=cx, y=cy, z=cz, **IDENTITY)),
                physical_object=Geometry(center=Pose(x=0, y=0, z=0, **IDENTITY),
                                         box=RectangularPrism(dims_mm=Vector3(
                                             x=dx, y=dy, z=dz)),
                                         label="tool-arm-b-camera")))
    return WorldState(obstacles=[GeometriesInFrame(reference_frame="world",
                                                   geometries=boxes)],
                      transforms=transforms)


async def _from_insert_branch(motion, arms, arm, st, flange, state, stamp,
                              source: str = "flag_insert") -> None:
    """Start the flag's pick from the flag INSERT's configuration, on its branch.

    Taught independently the two came out on opposite wrist branches (wrist_2 at -90 for
    the pick, +90 for the insert), so the carry between them had to turn the arm over with
    a 250 mm flag hanging from it: the planner either swept the flag through the shell or
    ended the carry 158 degrees round on the base, from where no straight descent existed.
    So the arm is put in the insert's stand-off and moved to the pick's in a line, which a
    planner answers from the configuration it is already in.
    """
    from viam.components.arm import JointPositions
    from viam.proto.service.motion import Constraints, LinearConstraint

    # Run the other way round: measured, the PICK has only one wrist branch that reaches
    # the feeder (started from the insert's, its last 110 mm still flipped 180 degrees),
    # so the insert is taught from the pick's.
    path = POSES / arm / f"{source}.clear.json"
    if not path.exists():
        return
    blob = json.loads(path.read_text())
    if blob.get("layout") != stamp:
        return
    try:
        await arms[arm].move_to_joint_positions(JointPositions(values=blob["joints_deg"]),
                                                timeout=60)
        out = tuple(flange[i] + st.approach[i] * 150.0 for i in range(3))
        await motion.move(
            component_name=arm,
            destination=PoseInFrame(reference_frame="world", pose=Pose(
                x=out[0], y=out[1], z=out[2], **orientation_for(st.approach, st.theta_deg))),
            world_state=state,
            constraints=Constraints(linear_constraint=[LinearConstraint(
                line_tolerance_mm=PICK_FROM_INSERT_LINE_MM,
                orientation_tolerance_degs=PICK_FROM_INSERT_TURN_DEG)]))
        now = [round(v, 3) for v in (await arms[arm].get_joint_positions()).values]
        print(f"  {'':<28} {arm}  ..   approached from {source}'s branch: "
              f"{_swing(now, blob['joints_deg']):.0f} deg from its stand-off", flush=True)
    except Exception as exc:                                       # noqa: BLE001
        print(f"  {'':<28} {arm}  ..   could not start from {source}'s branch "
              f"({str(exc)[:140]})", flush=True)


def _ov_matrix(p) -> list:
    """Rows of the rotation a viam orientation vector describes: Rz(lon) Ry(lat) Rz(theta),
    the ZYZ decomposition RDK's OrientationVector.Quaternion uses."""
    ox, oy, oz, th = p.o_x, p.o_y, p.o_z, math.radians(p.theta)
    n = math.sqrt(ox * ox + oy * oy + oz * oz) or 1.0
    ox, oy, oz = ox / n, oy / n, oz / n
    lat = math.acos(max(-1.0, min(1.0, oz)))
    lon = math.atan2(oy, ox) if 1.0 - abs(oz) > 1e-4 else 0.0

    def rz(a):
        return [[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0],
                [0.0, 0.0, 1.0]]

    def ry(a):
        return [[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0],
                [-math.sin(a), 0.0, math.cos(a)]]

    def mul(a, b):
        return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
                for i in range(3)]

    return mul(mul(rz(lon), ry(lat)), rz(th))


async def _held_geometry(world, motion, grip, arm: str):
    """What the arm is carrying, as geometry that moves WITH it.

    The planner knew the jaw and nothing under it, so a free path carried the 250 mm flag
    over the shell's crown: the flag was scraped out of the grip or the shell knocked out
    of its cradle, five runs running. Each held part's bounds are taken at their live pose,
    expressed in the arm's own end frame (asked of viam, so the convention is viam's), and
    attached there as a box. The part's own world obstacle is dropped, since it is no
    longer where the world list says.
    """
    from viam.proto.common import Transform

    status = await grip.do_command({"command": "status"})
    names = []
    for path in status.get("gripped") or []:
        bits = str(path).split("/")
        if len(bits) > 2 and bits[2] not in names:
            names.append(bits[2])
    if not names:
        return [], []
    if "shell" in names:
        # What is fitted to the shell rides with it: the door on its hinge, the pin and
        # the flag on their seat joints. Left in the world as fixed boxes, the seated flag
        # refused the overhead presentation as a collision with the shell carrying it.
        # Only what is ON it, though: a REJECT has neither pin nor flag, which are still
        # in their feeders - carried anyway, the nest flag was "dragged" through the belt
        # and refused the set-down on the reject spur (2026-10-12).
        sh = (await world.do_command({"command": "prim_poses", "paths": [
            "/World/shell", "/World/door", "/World/pin", "/World/flag"]}))["prims"]
        at = sh["/World/shell"]["position_mm"]
        names += [n for n in ("door", "pin", "flag") if n not in names
                  and math.dist(sh[f"/World/{n}"]["position_mm"], at) < ON_SHELL_MM]
    frag = json.loads(FRAGMENT.read_text())
    props = next(c["attributes"]["props"] for c in frag["components"]
                 if c["model"].endswith(":world"))
    bounds = {str(p.get("name")): p.get("bounds_m") for p in props}
    end = (await motion.get_pose(component_name=arm, destination_frame="world")).pose
    rot, t = _ov_matrix(end), (end.x, end.y, end.z)
    out = []
    for name in names:
        boxes = bounds.get(name)
        if name in ("pin", "flag"):
            # The PART's own frame, not the prop's `bounds_m`: those are already turned
            # into the feeder pose (`_turned_bounds`), and turning them again by the live
            # axes laid the pin's box ACROSS the jaw - harmless at the old wrist rolls, and
            # into the belt deck at the insert's -120 (2026-10-13).
            from assembly.asset import part as _part
            lo_p, hi_p = _part(name.capitalize()).bounds_mm
            boxes = [{"name": name, "min": [v / 1000.0 for v in lo_p],
                      "max": [v / 1000.0 for v in hi_p]}]
        if not boxes:
            continue
        pp = (await world.do_command({"command": "prim_poses",
                                      "paths": [f"/World/{name}"]}))["prims"][f"/World/{name}"]
        pos = pp["position_mm"]
        axes = (pp["x_axis"], pp["y_axis"], pp["z_axis"])
        local = []
        for b in boxes:
            lo, hi = [v * 1000.0 for v in b["min"]], [v * 1000.0 for v in b["max"]]
            for cx in (lo[0], hi[0]):
                for cy in (lo[1], hi[1]):
                    for cz in (lo[2], hi[2]):
                        w = [pos[i] + cx * axes[0][i] + cy * axes[1][i] + cz * axes[2][i]
                             - t[i] for i in range(3)]
                        local.append([sum(rot[k][i] * w[k] for k in range(3))
                                      for i in range(3)])
        lo = [min(c[i] for c in local) - HELD_PAD_MM for i in range(3)]
        hi = [max(c[i] for c in local) + HELD_PAD_MM for i in range(3)]
        c = [(lo[i] + hi[i]) / 2.0 for i in range(3)]
        out.append(Transform(
            reference_frame=f"held-{name}",
            pose_in_observer_frame=PoseInFrame(reference_frame=arm, pose=Pose(
                x=c[0], y=c[1], z=c[2], **IDENTITY)),
            physical_object=Geometry(center=Pose(x=0, y=0, z=0, **IDENTITY),
                                     box=RectangularPrism(dims_mm=Vector3(
                                         x=hi[0] - lo[0], y=hi[1] - lo[1],
                                         z=hi[2] - lo[2])),
                                     label=f"held-{name}")))
        print(f"    planner: {arm} carries {name} as a "
              f"{[round(hi[i] - lo[i]) for i in range(3)]} mm box at "
              f"{[round(v) for v in c]} in its end frame", flush=True)
    return names, out


async def main() -> int:
    robot = await RobotClient.at_address(
        FQDN, options())
    motion = MotionClient.from_robot(robot, "builtin")
    world = Generic.from_robot(robot, "sim-world")
    arms = {n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b", "arm-c")}
    cell = chosen()
    stamp = layout_hash()

    saved, failed = 0, []
    only = [a for a in sys.argv[1:] if not a.startswith("-")]
    at_index = None
    # Start from a cell that can be taught from, whatever the last pass left behind.
    await clear_and_park(world, arms, cell)
    print(f"  layout {stamp}, cell cleared and parked", flush=True)

    # Teach each arm a PARK, before anything else.
    #
    # The park was being derived from whichever taught pose sorted first alphabetically,
    # which is how arm-a came to park with its tool hanging over the belt at the back-shot
    # station. Everything downstream paid for that: the park swings 190 degrees on the base
    # through wherever the belt has just delivered a part, and the arm would only park at
    # all once the part had been taken out of the cell. Adding the stand-off poses made it
    # worse by changing which file sorted first.
    #
    # A park is its own requirement - up, back, over the arm's own base, out of everyone
    # else's way - so it is asked for directly and verified by the planner like any other
    # pose. Taught against a CLEARED cell, because that is the state it has to be reachable
    # from when something has gone wrong.
    firsts = {st.task: st for st in stations_for(cell)}
    for arm in ARMS:
        base = cell.arm_bases_mm[arm]
        yaw = math.radians(cell.arm_yaw_deg[arm])
        flange = (base[0] + math.cos(yaw) * PARK_OUT_MM + PARK_ASIDE_MM.get(arm, 0.0),
                  base[1] + math.sin(yaw) * PARK_OUT_MM,
                  base[2] + PARK_UP_MM)
        park_or = orientation_for((0.0, 0.0, 1.0))
        # NEAR THE FIRST MOVE (Brandon, 2026-10-12): an arm waits over the stand-off of the
        # station it goes to first, lifted clear of the belt's sweep and already in that
        # station's orientation, rather than up and back over its own base.
        tries = [(flange, park_or)]
        if arm in PARK_AT and PARK_AT[arm][0] in firsts:
            # The full lift can leave the arm's reach (arm-a over `inside`, measured
            # 2026-10-12), so step the lift down, and keep the old park as the last resort.
            st0 = firsts[PARK_AT[arm][0]]
            f0 = st0.flange_mm()
            near_or = orientation_for(st0.approach, st0.theta_deg)
            # And pulled back toward the arm's own base: lifted straight up over arm-a's
            # inside shot was out of reach at every height, which left it parked facing
            # the other way and swinging 185 degrees past arm-c into its first station -
            # where it was stopped, every film (2026-10-12).
            sx, sy = f0[0] + st0.approach[0] * PARK_STANDOFF_MM, \
                f0[1] + st0.approach[1] * PARK_STANDOFF_MM
            bx, by = base[0] - sx, base[1] - sy
            bn = math.hypot(bx, by) or 1.0
            tries = [((sx + bx / bn * back, sy + by / bn * back,
                       f0[2] + st0.approach[2] * PARK_STANDOFF_MM + lift), near_or)
                     for back in (0.0, 200.0, 350.0)
                     for lift in (PARK_AT[arm][1], 200.0, 150.0, 100.0)
                     if lift <= PARK_AT[arm][1]] + tries
        # WAIT ON THE FIRST STATION'S OWN BRANCH (waits.py): its stand-off joints, pulled
        # back and lifted out of the belt's sweep with only j2-j4 moving. Needs that
        # stand-off taught against this layout; the first teach of a new layout falls back
        # to the park over the base and the next one picks this up.
        wait = _wait_pose(cell, arm, firsts, stamp)
        if wait is not None:
            tries = [(wait["flange_mm"], orientation_for(wait["approach"], wait["theta_deg"]))]
        errors = []
        for flange, park_or in tries:
            try:
                # A park must be clear of where the belt WILL carry the shell, not just of
                # where it is now: arm-a's fallback park passed with its wrist beside the
                # crown, and the shell ran into it at the next index (audit, 2026-10-13).
                ws = await declared(world, moving=arm, tools=True)
                ws.obstacles[0].geometries.append(belt_sweep(cell))
                await motion.move(
                    component_name=arm,
                    destination=PoseInFrame(reference_frame="world", pose=Pose(
                        x=flange[0], y=flange[1], z=flange[2], **park_or)),
                    world_state=ws)
                break
            except Exception as exc:                               # noqa: BLE001
                errors.append(str(exc)[-120:])
        else:
            print(f"  {'park/' + arm:<28} {arm}  FAILED: {errors}", flush=True)
            failed.append(("park/" + arm, arm, "no park pose"))
            continue
        if errors:
            print(f"  {'park/' + arm:<28} {arm}  ..   {len(errors)} closer park(s) out of reach",
                  flush=True)
        if wait is not None:
            # The planner reached the pose on whatever branch; settle onto the wait's own
            # joints, which are the point of it.
            await arms[arm].move_to_joint_positions(
                JointPositions(values=wait["joints_deg"]), timeout=120)
        joints = [round(v, 3) for v in (await arms[arm].get_joint_positions()).values]
        folder = POSES / arm
        folder.mkdir(parents=True, exist_ok=True)
        (folder / "park.json").write_text(json.dumps({
            "arm": arm, "task": "park", "role": "park", "joints_deg": joints,
            "flange_mm": [round(v, 2) for v in flange],
            "approach": wait["approach"] if wait else [0.0, 0.0, 1.0],
            "theta_deg": wait["theta_deg"] if wait else 0.0, "layout": stamp,
            **({"wait_of": wait["of"], "back_mm": wait["back_mm"], "lift_mm": wait["lift_mm"]}
               if wait else {}),
            "_note": ("Where this arm waits: its first station's stand-off pulled back and "
                      "lifted out of the belt's sweep on the same wrist branch (waits.py)."
                      if wait else
                      "Where this arm waits. Up and back over its own base, tool down, "
                      "clear of the belt and of the other arms."),
        }, indent=1) + "\n")
        print(f"  {'park/' + arm:<28} {arm}  ok   joints {joints}", flush=True)
    await clear_and_park(world, arms, cell)
    for st in stations_for(cell):
        if only and st.task not in only:
            continue
        # INDEX THE BELT FIRST, then ask the planner.
        #
        # A station's work point is written for the shell at that station's own index, so
        # planning to it while the part sits at a different index asks the planner to avoid
        # an obstacle that is not where the station thinks it is. It went unnoticed while
        # `place_prop` silently refused to move a kinematic prop - the shell never left
        # x -55, which is the `inside` index, so every plan happened to be consistent with
        # a cell that had one station. The first run after that was fixed jammed arm-a into
        # the shell at 533 N.m on its way to a station 956 mm away from it.
        if st.on_belt and st.index_x_mm != at_index:
            at_index = await index_to(world, cell, st.index_x_mm, at_index)
        arm = THREE[st.role]
        if adrift(arm, list((await arms[arm].get_joint_positions()).values)):
            # Parking an adrift arm in place does not work: it is adrift BECAUSE it is at
            # a station, and the park swings its wrist back through whatever it was
            # working on. Measured, arm-a sat at the inside clear pose with wrist_3 at
            # -262.5 against a +-205 clamp, and the park stalled on j4 and j5 against the
            # shell. Taking the part out of the cell first is what works, so the recovery
            # is the same one the pass opens with.
            print(f"    {arm} is adrift; clearing the cell to park it", flush=True)
            await clear_and_park(world, arms, cell)
            at_index = None
        if st.on_belt and st.index_x_mm != at_index:
            at_index = await index_to(world, cell, st.index_x_mm, at_index)
        state = await declared(world, moving=arm)
        # The stand-off is reached WITH the tool in the planner (it is a move between
        # stations); the station itself without (the tool is meant to be at the work).
        state_t = await declared(world, moving=arm, tools=True)
        flange = st.flange_mm()
        pose = Pose(x=flange[0], y=flange[1], z=flange[2], **orientation_for(st.approach, st.theta_deg))

        # Two attempts, the second from a cleared cell.
        #
        # The planner's answer depends on where the arm starts, and where it starts is
        # wherever the previous station left it - so the same station solves on one pass
        # and picks a colliding branch on the next. That is not flakiness to be retried
        # blindly; it is a real dependence on the starting configuration, and the honest
        # response is to give the second attempt a KNOWN start rather than another roll of
        # the same dice.
        # And via the STAND-OFF, which is taught first.
        #
        # Teaching the station and then backing out of it asks the planner for the
        # stand-off from the station, and it is free to answer with a different arm
        # configuration - measured, a 150 mm move came back 100 to 178 degrees away on
        # j0..j4, which is not an approach but a reconfiguration with the tool next to the
        # part. Taught the other way round the station inherits the stand-off's branch,
        # because a short move from a pose the arm is already in is what a planner keeps.
        # It is also the order the CYCLE runs in, which is the point.
        err = None
        if st.task == "flag_insert":
            await _from_insert_branch(motion, arms, arm, st, flange, state, stamp,
                                      source="flag_pick")
        for attempt in range(2):
            try:
                # The ladder covers the PAIR, not just the stand-off.
                #
                # A stand-off that solves is not enough: what matters is that the station
                # solves FROM it without changing the arm's shape. Trying stand-offs until
                # one solves and only then planning the station leaves no way back when the
                # station answers 139 degrees away, which is what the flag pick did from
                # every distance the first version offered it. Both legs are flown for each
                # distance and the distance is accepted only if the pair holds together.
                clear_joints, clear_mm, best = None, None, None
                # Never closer than 80 for the pin's pick: its stand-off is also where the
                # lift ends, and the pin has to come all the way out of a 50 mm bore.
                ladder = ((150.0, 110.0, 80.0) if st.task == "pin_pick"
                          else (150.0, 110.0, 80.0, 50.0))
                for back in ladder:
                    out = tuple(flange[i] + st.approach[i] * back for i in range(3))
                    try:
                        await motion.move(
                            component_name=arm,
                            destination=PoseInFrame(reference_frame="world", pose=Pose(
                                x=out[0], y=out[1], z=out[2],
                                **orientation_for(st.approach, st.theta_deg))),
                            world_state=state_t)
                        candidate = [round(v, 3) for v in
                                     (await arms[arm].get_joint_positions()).values]
                        await motion.move(
                            component_name=arm,
                            destination=PoseInFrame(reference_frame="world", pose=pose),
                            world_state=state)
                    except Exception as exc:                       # noqa: BLE001
                        err = str(exc)
                        continue
                    reached = [round(v, 3) for v in
                               (await arms[arm].get_joint_positions()).values]
                    swing = _swing(candidate, reached)
                    if best is None or swing < best[0]:
                        best = (swing, candidate, back, reached)
                    if swing <= BRANCH_FLIP_DEG:
                        clear_joints, clear_mm, err = candidate, back, None
                        break
                    print(f"  {'':<28} {arm}  --   {back:.0f} mm stand-off lands "
                          f"{swing:.0f} deg from the station; trying shorter", flush=True)
                if clear_joints is None and best is not None:
                    # Nothing held together. Take the closest pair anyway and SAY SO, so a
                    # station that is marginal is visible rather than missing.
                    swing, clear_joints, clear_mm, _ = best
                    print(f"  {'':<28} {arm}  ~~   best stand-off is {swing:.0f} deg from "
                          f"the station at {clear_mm:.0f} mm; keeping it", flush=True)
                    err = None
                if err is None:
                    break
            except Exception as exc:  # noqa: BLE001
                err = str(exc)
                if attempt == 0:
                    print(f"  {st.key:<28} {arm}  retrying from a cleared cell",
                          flush=True)
                    await clear_and_park(world, arms, cell)
                    if st.on_belt:
                        at_index = await index_to(world, cell, st.index_x_mm)
                    state = await declared(world, moving=arm)
        if err is not None:
            # Keep the NAMES. "collision" on its own says a station failed and nothing
            # about what it hit, and a planner that has just told you which two geometries
            # overlap is the cheapest diagnosis available - throwing it away has cost this
            # build whole cycles more than once.
            import re as _re
            pairs = _re.findall(r"violation between [\w.:-]+ and [\w.:-]+", err)
            short = ("no IK" if "zero IK" in err else
                     "too far" if "too far" in err else
                     ("collision: " + "; ".join(sorted(set(pairs)))[:400]) if pairs else
                     "self-collision" if "self-collision" in err else err[-700:])
            failed.append((st.key, arm, short))
            print(f"  {st.key:<28} {arm}  FAILED: {short}", flush=True)
            continue
        joints = [round(v, 3) for v in (await arms[arm].get_joint_positions()).values]
        folder = POSES / arm
        folder.mkdir(parents=True, exist_ok=True)
        (folder / f"{st.task}.json").write_text(json.dumps({
            "arm": arm, "task": st.task, "role": st.role,
            "joints_deg": joints,
            "flange_mm": [round(v, 2) for v in flange],
            "approach": list(st.approach), "theta_deg": st.theta_deg,
            "tool": st.tool, "tool_standoff_mm": TOOLS[st.tool].standoff_mm,
            "belt_index_x_mm": round(st.index_x_mm, 2),
            "on_belt": st.on_belt,
            "derived_from": st.derived_from,
            "layout": stamp,
            "_note": "Taught by probes/assembly/reach_check.py. `layout` fingerprints the "
                     "fragment and the asset frames this was taught against; a loader that "
                     "sees a different one must refuse rather than drive here.",
        }, indent=1) + "\n")
        saved += 1
        print(f"  {st.key:<28} {arm}  ok   joints {joints}", flush=True)

        # The stand-off was taught on the way IN; what is left is to check it leads here.
        #
        # A stand-off on a different branch from its own station is not a stand-off, and
        # entering through one stalled arm-b's shoulder at 1028 N.m of a 1000 N.m limit.
        # Wrist_3 is excluded from the comparison and the rest taken modulo a turn: an
        # orientation vector's theta does not pin wrist roll, so the planner rolls the tool
        # half a turn as it pleases, and both tools here are symmetric about that axis.
        if clear_joints is None:
            print(f"  {'':<28} {arm}  !!   no stand-off solves", flush=True)
            failed.append((st.key + " [clear]", arm, "no standoff solves"))
        else:
            (folder / f"{st.task}.clear.json").write_text(json.dumps({
                "arm": arm, "task": st.task + ".clear", "role": st.role,
                "joints_deg": clear_joints,
                "flange_mm": [round(flange[i] + st.approach[i] * clear_mm, 2)
                              for i in range(3)],
                "approach": list(st.approach), "theta_deg": st.theta_deg,
                "clear_mm": clear_mm, "of_task": st.task,
                "tool": st.tool, "layout": stamp,
                "_note": "Where the tool stands off this station. Taught on the way IN, so "
                         "the station inherits this configuration; enter and leave by it.",
            }, indent=1) + "\n")
            print(f"  {'':<28} {arm}  ok   clear at {clear_mm:.0f} mm "
                  f"({_swing(clear_joints, joints):.0f} deg from the station)", flush=True)

        # Park before the next station is taught.
        #
        # An arm left standing where it finished is an obstacle for whichever arm works
        # next, and the planner can see the arms now: teaching the door placement with
        # arm-a still at the inside shot failed on "arm-a-wrist_3_link and arm-b:ee_link",
        # which is a true statement about a cell nobody would run that way. It also makes
        # the pass depend on the ORDER the stations happen to be listed in. Parking after
        # each one removes both, and it is what the cycle does.
        try:
            await arms[arm].move_to_joint_positions(
                JointPositions(values=_home_joints(arm)), timeout=180)
        except Exception as exc:                                   # noqa: BLE001
            print(f"  {'':<28} {arm}  --   park: {str(exc)[:110]}", flush=True)

    print(f"\n  {saved} poses saved under fragments/poses/, {len(failed)} unreachable "
          f"(layout {stamp})")
    for key, arm, why in failed:
        print(f"    {key} [{arm}]: {why}")
    await robot.close()
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
