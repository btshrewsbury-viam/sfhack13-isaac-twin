"""A QC round: pick, check, hand over, check again, place.

Importable on purpose. The tests drive these functions rather than re-implementing the
sequence, and the eventual move into `qc:cell` verbs should be a rename.

Everything takes its clients as arguments. `record_cell.py` grew as closures over
module-level clients and nothing in it could be imported, which meant the only way to
test the round was to run the script and read its output.
"""

from __future__ import annotations

import asyncio
import math
import time
from pathlib import Path
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

from viam.components.arm import JointPositions
from viam.proto.common import (
    Capsule, GeometriesInFrame, Geometry, Pose, PoseInFrame, RectangularPrism,
    Transform, Vector3, WorldState,
)
from viam.proto.service.motion import (
    Constraints, LinearConstraint, OrientationConstraint,
)
from viam.spatialmath import OrientationVector, Quaternion
from viam.spatialmath import Vector3 as SdkVector3

from .layout import (CUP_APPROACH_GAP_MM, Cell, Vec3,
                     part_origin_offset_local_mm)

IDENTITY = {"o_x": 0.0, "o_y": 0.0, "o_z": 1.0, "theta": 0.0}
PART_LABEL = "part"


@dataclass
class Program:
    """The cell's taught motion: one joint path per move, in the order they happen.

    An industrial cell runs one fixed sequence over and over. Asking a randomised
    planner to re-derive it every cycle gets a different answer every cycle - RDK seeds
    IK from wherever the arm is and scores by joint distance, so the same station is
    reached through a different arm configuration on different runs. That is what the
    wandering trajectories were, and why the marginal poses failed on some cycles and
    not others.

    So the sequence is planned once, with the scene declared and every obstacle checked,
    and the joint path that came out is kept and replayed. Replay does no planning and
    no collision checking, which is safe for exactly the reason a taught robot program
    is safe: the path was validated when it was taught, and nothing in the cell moves
    between cycles except the part, which the taught path already accounts for.

    Steps are numbered rather than named, because the numbering IS the claim: cycle N
    makes the same calls in the same order as the teaching run, and if it does not, the
    program does not fit and the cell says so instead of replaying the wrong path.
    """

    paths: Dict[str, List[List[float]]] = field(default_factory=dict)
    # Where each step was going, in world millimetres. Not used to replay - the joint
    # path is what gets executed - but a program you cannot read is a program you
    # cannot debug, and the numbered steps alone say nothing about which leg is which.
    targets: Dict[str, List[float]] = field(default_factory=dict)
    teaching: bool = False
    # Left in the file for programs already saved with it; the arm uses its own
    # configured joint_speed_deg_s and this is no longer sent.
    speed_deg_s: float = 60.0
    step: int = 0
    # The round is one sequence until the verdict and two after it, because the part
    # goes to a different conveyor. Steps taken after the cell has decided are keyed by
    # what it decided, so one program holds both endings.
    branch: str = ""

    # While teaching: where each arm was last sent, so a move's budget can be worked out
    # from the distance it has to cover, and which steps came in over it anyway.
    reached: Dict[str, List[float]] = field(default_factory=dict)
    overspent: Dict[str, Tuple[float, float]] = field(default_factory=dict)

    @staticmethod
    def travel(paths: Dict[str, List[List[float]]]) -> float:
        """Total joint travel a program commands, in degrees.

        What makes one taught program better than another. Planning is randomised, so
        two passes over the same round produce different arm configurations at the same
        stations - and the difference is not small: a pass that picks a flipped shoulder
        for the set-down pays a 217 degree reconfiguration for it. Teaching several
        times and keeping the cheapest costs a few minutes once and is paid back on
        every cycle after.
        """
        return sum(sum(max(abs(a - b) for a, b in zip(x, y))
                       for x, y in zip(path, path[1:]))
                   for path in paths.values())

    @staticmethod
    def tidy(path: List[List[float]],
             epsilon: float = 0.05) -> List[List[float]]:
        """Drop waypoints the arm would not move for.

        The planner hands back paths that are mostly repeats - one taught step had 66
        waypoints whose median stride was 0.0 degrees and one stride of 217. They cost
        nothing to execute, since the executor interpolates between whatever it is
        given, but they make a program impossible to read and they hide exactly the
        thing worth looking for: a single enormous stride in an otherwise dense path is
        a joint wrapping, and it cannot be seen next to sixty repeats of the same pose.
        """
        out: List[List[float]] = []
        for point in path:
            if not out or max(abs(a - b) for a, b in zip(out[-1], point)) > epsilon:
                out.append(list(point))
        return out or [list(path[-1])]

    def key(self, arm: str) -> str:
        prefix = f"{self.branch}:" if self.branch else ""
        return f"{prefix}{self.step:03d}:{arm}"

    def to_json(self) -> Dict[str, Any]:
        return {"speed_deg_s": self.speed_deg_s, "targets": self.targets,
                "paths": self.paths}

    @classmethod
    def from_json(cls, blob: Dict[str, Any]) -> "Program":
        return cls(paths={str(k): [[float(v) for v in w] for w in path]
                          for k, path in blob["paths"].items()},
                   targets={str(k): [float(v) for v in t]
                            for k, t in (blob.get("targets") or {}).items()},
                   speed_deg_s=float(blob.get("speed_deg_s", 60.0)))


@dataclass
class Clients:
    motion: Any
    world: Any
    arms: Dict[str, Any]
    grippers: Dict[str, Any]
    # The inspection camera. Optional so the motion probes need not connect one, but
    # without it a round has no check - only a rehearsal of the moves around one.
    camera: Any = None
    # Where frames go. The caller sets the base; each round puts its own frames in a
    # subdirectory named for the marks on that part, so two rounds through the same
    # stages do not overwrite each other's images while the first round's tests still
    # hold their paths.
    frames_base: Optional[Path] = None
    frames_dir: Optional[Path] = None
    # The taught motion, if there is any. Without it every move is planned.
    program: Optional[Program] = None


@dataclass
class Held:
    """What an arm is carrying, and where it sits relative to that arm's flange.

    Measured at the moment of the grasp rather than derived. The cup lands where its
    raycast hits, which on a curved face is not where the arithmetic would put it, and
    the difference is what the planner needs to know about.
    """

    arm: str
    offset_mm: Vec3
    dims_mm: Vec3
    orientation: Dict[str, float] = field(
        default_factory=lambda: dict(IDENTITY))
    # The part's world orientation when it last sat upright on a surface. Carried along
    # so a set-down can ask for "upright, yawed a half turn" and have the flange
    # orientation derived from it, rather than guessing a theta.
    upright_wxyz: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)


@dataclass
class Trace:
    """What happened, for the tests to assert over.

    Every stage carries the wall-clock second it was recorded at, and the gap since the
    previous one, because a cycle time is the first thing anyone will ask of this cell
    and a trace without timestamps cannot answer it. `at_s` is measured from the first
    recorded stage rather than from process start, so it does not include connecting.
    """

    stages: List[Dict[str, Any]] = field(default_factory=list)
    started: Optional[float] = None

    def record(self, stage: str, **facts: Any) -> None:
        now = time.monotonic()
        if self.started is None:
            self.started = now
        previous = self.stages[-1]["at_s"] if self.stages else 0.0
        self.stages.append({"stage": stage,
                            "at_s": round(now - self.started, 2),
                            "took_s": round(now - self.started - previous, 2),
                            **facts})

    @property
    def elapsed_s(self) -> float:
        """How long the round took, by the trace's own clock."""
        return self.stages[-1]["at_s"] if self.stages else 0.0

    def slowest(self, count: int = 10) -> List[Dict[str, Any]]:
        return sorted(self.stages, key=lambda s: -s["took_s"])[:count]

    def at(self, stage: str) -> Dict[str, Any]:
        return next(s for s in self.stages if s["stage"] == stage)

    @property
    def names(self) -> List[str]:
        return [s["stage"] for s in self.stages]


# ---- world state -------------------------------------------------------------


def _box(label: str, centre: Sequence[float], dims: Sequence[float]) -> Geometry:
    return Geometry(
        center=Pose(x=centre[0], y=centre[1], z=centre[2], **IDENTITY),
        box=RectangularPrism(dims_mm=Vector3(x=dims[0], y=dims[1], z=dims[2])),
        label=label,
    )


def tool_transform(arm: str, cell: Cell) -> Transform:
    """The cup, carried by the arm holding it.

    Its length and radius are the gripper's own config, read from the fragment. They
    used to be repeated here as literals, in a file that exists because repeated
    literals went stale.
    """
    return Transform(
        reference_frame=f"tool-{arm}",
        pose_in_observer_frame=PoseInFrame(
            reference_frame=arm,
            pose=Pose(x=0.0, y=0.0, z=cell.tool_mm / 2.0, **IDENTITY)),
        physical_object=Geometry(
            center=Pose(x=0.0, y=0.0, z=0.0, **IDENTITY),
            capsule=Capsule(radius_mm=cell.cup_radius_mm, length_mm=cell.tool_mm),
            label=f"tool-{arm}"),
    )


def hook_transform(arm: str, cell: Cell) -> Optional[Transform]:
    """The door hook, if this tool carries one.

    The cup's visual has no collider precisely so it cannot fight the capsule the
    planner already reasons about. The hook is the other case: it exists to push and
    pull real geometry, so it has a CollisionAPI in the stage - and anything with a
    collider that the planner cannot see is something the arm will be routed through.

    Declared as one box around the whole L, which is coarser than the hook but never
    smaller than it.
    """
    if not cell.hook:
        return None
    reach = cell.hook.get("reach_mm", 150.0)
    finger = cell.hook.get("finger_mm", 45.0)
    side = cell.hook.get("side_mm", 70.0)
    thick = cell.hook.get("thickness_mm", 12.0)
    # In the arm's end-effector frame the cup runs along +z; the hook is carried beside
    # it, so the box spans the side offset and the finger's turn back across the axis.
    return Transform(
        reference_frame=f"hook-{arm}",
        pose_in_observer_frame=PoseInFrame(
            reference_frame=arm,
            pose=Pose(x=0.0, y=side - finger / 2.0, z=reach / 2.0, **IDENTITY)),
        physical_object=Geometry(
            center=Pose(x=0.0, y=0.0, z=0.0, **IDENTITY),
            box=RectangularPrism(dims_mm=Vector3(
                x=thick, y=finger + thick, z=reach)),
            label=f"hook-{arm}"),
    )


def world_state(props: Sequence[Dict[str, Any]], *, cell: Optional[Cell] = None,
                held: Optional[Held] = None,
                ignore: Sequence[str] = ()) -> WorldState:
    """The scene as the planner should see it at this moment.

    A carried part is not scenery and is not absent: it is part of the moving arm. Left
    in the obstacle list it blocks the arm carrying it; left out entirely it sweeps
    through everything on the way. It goes in as a transform on the holding arm's frame,
    which is the only shape that travels.
    """
    skip = set(ignore)
    if held is not None:
        skip.add(PART_LABEL)
    # A prop reported as several boxes carries labels like `conveyor_in.deck`, and a
    # caller naming the surface it is setting down onto means the whole prop, not one
    # of its parts - so match on the prop name as well as the full label.
    obstacles = [_box(p["label"], p["center_mm"], p["dims_mm"]) for p in props
                 if p["label"] not in skip and p["label"].split(".")[0] not in skip]
    shape = cell or Cell.from_fragment()
    transforms = [tool_transform("arm-a", shape), tool_transform("arm-b", shape)]
    transforms += [t for t in (hook_transform("arm-a", shape),) if t is not None]
    if held is not None:
        transforms.append(Transform(
            reference_frame=f"carried-{held.arm}",
            pose_in_observer_frame=PoseInFrame(
                reference_frame=held.arm,
                pose=Pose(x=held.offset_mm[0], y=held.offset_mm[1],
                          z=held.offset_mm[2], **IDENTITY)),
            physical_object=Geometry(
                center=Pose(x=0.0, y=0.0, z=0.0, **held.orientation),
                box=RectangularPrism(dims_mm=Vector3(
                    x=held.dims_mm[0], y=held.dims_mm[1], z=held.dims_mm[2])),
                label=PART_LABEL),
        ))
    return WorldState(
        obstacles=[GeometriesInFrame(reference_frame="world", geometries=obstacles)],
        transforms=transforms,
    )


# ---- reading the world -------------------------------------------------------


async def props_of(clients: Clients) -> List[Dict[str, Any]]:
    """The scene as obstacles, read fresh.

    Read again whenever something has moved. The reply reports dynamic props where they
    are now, so a list fetched before the pick describes the part on the belt and says
    nothing about the part on the handoff stand.
    """
    return list((await clients.world.do_command({"command": "obstacles"}))["obstacles"])


# How far above the infeed's guide rails the part's underside must be lifted before it
# is moved sideways. The planner's own margin plus a little.
GUIDE_CLEARANCE_MM = 25.0

# How far the part may shift in the cup over the whole round before the hold is a
# different hold. The carry repeats to under a millimetre and a presentation costs about
# 13 mm as the part settles against the cup's bellows, so this is loose enough not to
# fire on either; the failures it is here for moved 213 mm and 1792 mm.
HOLD_DRIFT_MM = 40.0

# How long the outfeed runs once the part is proved to be sitting on it. Long enough at
# 400 mm/s to carry a 495 mm part clear of where the arm works and out of the overview
# camera's frame, which is what makes a filmed round end rather than stop.
OUTFEED_RUN_S = 4.0


async def part_pose(clients: Clients) -> Dict[str, Any]:
    """Where the part is, reported at its geometric CENTRE.

    The sim reports a prop's prim pose, which for a cube is its middle and for a
    referenced asset is wherever the asset was authored about - this mailbox's origin sits
    at the bottom of its shell, 39.1 mm under the middle. Every caller here wants the
    middle: `measure_hold` builds the box the planner is told the arm is carrying, and the
    handoff derives the pose the second arm takes the part by. Measured, using the origin
    for those put arm-b's approach 39 mm low and had it reach across the cell to a part it
    thought was still on the belt.

    The offset is turned by the part's own orientation, because it is a vector in the
    PART, not in the world. Added unrotated it is right only while the part is upright at
    its spawn yaw - and the moment a presentation rolls the part, the reported centre is
    wrong by up to 151 mm.

    That is what the 149 mm "slide in the cup" was. It was smooth, monotonic, identical at
    5 and 20 deg/s, and reproducible to a millimetre, while the part's quaternion tracked
    the commanded roll to a tenth of a degree. A cup does not slip like that. The part was
    exactly where it had always been.

    `origin_mm` is kept for anything that genuinely wants the prim.
    """
    reply = await clients.world.do_command(
        {"command": "prop_poses", "names": [PART_LABEL]})
    pose = dict(reply["props"][PART_LABEL])
    origin = [float(v) for v in pose["position_mm"]]
    offset = _rotate_by_quat(
        tuple(float(v) for v in pose["orientation_wxyz"]),
        part_origin_offset_local_mm())
    pose["origin_mm"] = origin
    pose["position_mm"] = [origin[i] + offset[i] for i in range(3)]
    return pose


async def flange_of(clients: Clients, arm: str) -> Vec3:
    pose = await clients.arms[arm].get_end_position()
    return (pose.x, pose.y, pose.z)


# Orientation maths is the SDK's, not ours. A hand-rolled pair of conversions here was
# self-consistent and still wrong: viam's theta is not measured from the shortest arc
# that takes +z onto the vector, so the two agreed for some vectors and differed by a
# quarter or a half turn for others - a quarter turn about x came back as a half turn
# about y. That value declares the carried part's box to the planner and decides where a
# set-down puts the flange, and the error read as the arm stalling against nothing. The
# only convention that matters is the one the motion service uses, and it ships with it.


def _ov_to_quat(o_x: float, o_y: float, o_z: float, theta: float):
    quat = OrientationVector(o_x, o_y, o_z, math.radians(theta)).to_quaternion()
    return (quat.w, quat.i, quat.j, quat.k)


def _quat_to_ov(quat) -> Dict[str, float]:
    vector = Quaternion(*quat).to_orientation_vector()
    return {"o_x": vector.o_x, "o_y": vector.o_y, "o_z": vector.o_z,
            "theta": math.degrees(vector.theta)}


def _quat_mul(a, b):
    quat = Quaternion(*a) * Quaternion(*b)
    return (quat.w, quat.i, quat.j, quat.k)


def _quat_conj(quat):
    conj = Quaternion(*quat).conjugate()
    return (conj.w, conj.i, conj.j, conj.k)


def _rotate_into_frame(quat, vector: Vec3) -> Vec3:
    """Express a world-frame vector in the frame that quaternion describes."""
    return _rotate_by_quat(_quat_conj(quat), vector)


def _rotate_by_quat(quat, vector: Vec3) -> Vec3:
    """Express a frame-local vector in the world."""
    out = Quaternion(*quat).rotate_vector(SdkVector3(*vector))
    return (out.x, out.y, out.z)


def yawed(quat, degrees: float):
    """Turn an orientation about the world's z axis."""
    half = math.radians(degrees) / 2.0
    return _quat_mul((math.cos(half), 0.0, 0.0, math.sin(half)), quat)


def flange_for(held: "Held", part_centre_mm: Vec3, part_wxyz):
    """Where the flange goes to put the held part at a pose.

    This replaces asking for a flange pose and hoping the part follows. It could not:
    `side_grip` puts the flange a fixed 25 mm below the part's centre, which is true at
    the pick and false everywhere the part has since been turned. Set down after a half
    turn the part hangs 25 mm BELOW the flange instead of above it, so a flange placed by
    that arithmetic drove the part 50 mm into the stand - measured, the arm stalled with
    the part already resting and 35 mm of commanded travel left.

    The hold is measured, so derive from it. The part sits at `offset_mm` in the flange's
    own frame, turned by `orientation`; asking for a part pose therefore fixes both the
    flange's orientation and its position, with nothing left to guess:

        part = flange * relative        ->  flange = part * relative^-1
        part_centre = flange_position + R(flange) * offset
    """
    o = held.orientation
    relative = _ov_to_quat(o["o_x"], o["o_y"], o["o_z"], o["theta"])
    flange_quat = _quat_mul(tuple(part_wxyz), _quat_conj(relative))
    offset = _rotate_by_quat(flange_quat, held.offset_mm)
    position = tuple(part_centre_mm[i] - offset[i] for i in range(3))
    return position, _quat_to_ov(flange_quat)


def part_for(held: "Held", flange_pose):
    """The inverse of `flange_for`: where the part ends up if the flange goes here.

    Needed because every pose this cell states for a presentation is a FLANGE pose, and
    what has to stay clear of the cell on the way there is the PART.
    """
    position, orientation = flange_pose
    flange_quat = _ov_to_quat(orientation["o_x"], orientation["o_y"],
                              orientation["o_z"], orientation["theta"])
    o = held.orientation
    relative = _ov_to_quat(o["o_x"], o["o_y"], o["o_z"], o["theta"])
    offset = _rotate_by_quat(flange_quat, held.offset_mm)
    return (tuple(position[i] + offset[i] for i in range(3)),
            _quat_mul(flange_quat, relative))


async def measure_hold(clients: Clients, arm: str, cell: Cell,
                       upright_wxyz=(1.0, 0.0, 0.0, 0.0)) -> Held:
    """Where the part sits relative to the arm's frame, measured now.

    The pose comes from the motion service, not from `get_end_position`. Those are two
    different frames: GetEndPosition reports the isaac flange prim, whose tool axis is
    +x, while a `Transform` parented to `arm-a` is read in the frame system's
    end-effector frame, whose tool axis is +z. Measuring in one and declaring in the
    other puts the carried box a quarter turn out - which shows up as the arm colliding
    with the part it is holding.
    """
    frame = (await clients.motion.get_pose(
        component_name=arm, destination_frame="world")).pose
    part = (await part_pose(clients))["position_mm"]
    delta = tuple(part[i] - (frame.x, frame.y, frame.z)[i] for i in range(3))
    quat = _ov_to_quat(frame.o_x, frame.o_y, frame.o_z, frame.theta)
    # The box's ORIENTATION in that frame too. Declaring world-axis dimensions in the
    # arm's frame describes a box that is the right size and the wrong shape, so the
    # planner clears a path the real part does not fit through - which reads as the arm
    # stalling against nothing.
    part_quat = tuple(float(v) for v in
                      (await part_pose(clients))["orientation_wxyz"])
    relative = _quat_mul((quat[0], -quat[1], -quat[2], -quat[3]), part_quat)
    # The box's dimensions have to be in the box's OWN frame, because they are declared
    # alongside an orientation that turns it. `cell.part.dims_mm` is not: it is a
    # WORLD-aligned extent measured with the part at its -90 yaw. Declaring those numbers
    # as local and then applying the part's orientation turns the part twice, and the
    # planner is told a 495 mm mailbox lies ACROSS the cell rather than along it.
    #
    # `cell.part_local_dims_mm` is that extent turned back by the yaw it was measured at
    # - once, as arithmetic, in the layout. It used to be recomputed here as the extent
    # of the measured pose, which is the same number only while the part is level. Held
    # at the 16 degree shoulder tilt it came back 55 and 42 mm over, the planner took
    # that as the part's size, and it refused all five of arm-a's presentations with the
    # carried box reported 99.8% inside the belt deck. A part's size is not a function
    # of how it is being held.
    local = cell.part_local_dims_mm
    return Held(arm=arm, offset_mm=_rotate_into_frame(quat, delta),
                dims_mm=local, orientation=_quat_to_ov(relative),
                upright_wxyz=tuple(upright_wxyz))


# ---- moves -------------------------------------------------------------------


# Holding the part, the tool's orientation has to stay put along the way. Without this
# the planner is free to pick a solution that flips the wrist - a 164 degree roll on what
# should be a straight lift - and a welded part swings through a wide arc and hits the
# cell. The pose at each END was always the same; it was the path between them that span.
# Measured, on the move from the lift to the first presentation - a 90 degree
# reorientation of the tool: free, the planner takes 20.9 s and the part ends at
# [-639, 708, 114], on the floor behind the arms; constrained, it takes 45.5 s and the
# part arrives at the inspect point with the hold intact (209 mm at the pick, 196 here).
# The ends were identical in both cases. It is the path that throws the part, and it
# throws it because the planner is entitled to reorient the tool however it likes along
# the way - which for a 495 mm part on a 206 mm lever is a flail, not a carry.
#
# This constant existed, and a comment below claimed "only moves made WHILE HOLDING are
# constrained", and it was passed to nothing. The comment described the intent and the
# code did not implement it.
HOLDING = Constraints(orientation_constraint=[
    OrientationConstraint(orientation_tolerance_degs=15.0)])

# Kept for reference, not used. Measured: with the carried box declared *and* its
# orientation, a strict path fails where either alone succeeds - the box needs the
# planner's freedom to adjust the wrist, and a straight line removes it. The box turns
# out to prevent the wrist flip these were added for, by making the sweep a collision.
#
# Short moves to and from a surface go in a straight line. An orientation constraint
# alone only fixes the tool angle, not the route: asked to lift 240 mm it kept the angle
# and swung the arm round the back of the cell to do it, pan heading for 140 degrees.
# A lift is a lift.
# Short moves to and from a surface go in a straight line, because that is what they
# are: a descent onto a belt, a lift off a table, a retreat. Left free, the planner is
# entitled to any path that avoids the obstacles and takes a swooping one - which reads
# as the arm flailing on its way to a pose 140 mm below where it started.
#
# This was tried once before and dropped, because with the carried box declared it
# produced detours or no path at all. That was measured while the orientation-vector
# maths was wrong, so the box the planner was avoiding was the right size and the wrong
# shape. Worth having again now the box is right.
STRAIGHT = Constraints(linear_constraint=[
    LinearConstraint(line_tolerance_mm=20.0, orientation_tolerance_degs=10.0)])

# Only moves made WHILE HOLDING are constrained. An empty arm can take whatever route it
# likes - it has nothing to swing - and constraining those just removed the freedom the
# planner needed to get into a tight approach at all.


# What a move is allowed to spend, in degrees of joint travel: a quarter turn of wrist,
# plus about a degree for every five millimetres the tool is carried. Calibrated against
# the steps that are known to be sane - 324 mm for 43 degrees, 408 mm for 90, 187 mm for
# 15 - and it fails the ones found by hand to change arm configuration rather than just
# move. `tests/test_taught_program.py` asserts the same thing on the saved program, and
# imports these, so the check a teaching pass applies and the check the program has to
# pass cannot drift apart.
TRAVEL_BASE_DEG = 100.0
TRAVEL_PER_MM = 0.20


def path_travel(path) -> float:
    """Joint travel along a path, as the largest joint's movement summed over strides."""
    return sum(max(abs(a - b) for a, b in zip(x, y)) for x, y in zip(path, path[1:]))


def travel_budget(previous, position) -> float:
    """What this move's geometry justifies. `previous` may be None for an arm's first."""
    distance = 0.0 if previous is None else math.dist(previous, position)
    return TRAVEL_BASE_DEG + TRAVEL_PER_MM * distance


def worst_joint(path) -> float:
    """How far the busiest single joint travels along a path, in degrees."""
    if len(path) < 2:
        return 0.0
    return max(sum(abs(y[j] - x[j]) for x, y in zip(path, path[1:]))
               for j in range(len(path[0])))


# A joint turning more than half a turn is the signature worth re-planning on, and it is
# not the same thing as being over budget. Over budget alone catches moves that are
# simply long - a tool reorientation costs a wrist 178 degrees and no replan will make it
# cheaper, and asking three times just spends the round's time. Over budget AND a joint
# past half a turn is a different animal: the arm reached the same pose through the other
# wrist solution. Measured in the program this was taught from, an 80 mm descent that
# spent 220 degrees of wrist 2 and a 190 mm move that spent 270 of wrist 3, both of which
# re-plan away, while the 178 degree reorientation comes back the same however often it
# is asked.
FLIP_DEG = 180.0
# Budget re-plans get their own allowance. Sharing one with genuine planning failures
# means a round that re-planned three cheap-looking moves has nothing left for the move
# that actually refuses - measured, that turned a teach that used to pass on the second
# attempt into five failures in a row.
BUDGET_RETRIES = 2
# And a move that sweeps a joint this far is re-planned however many times it takes,
# because it is not merely expensive - it is dangerous and it looks wrong.
#
# Measured in the taught program that came out of 2 retries: the approach from home to
# above the pick swept one joint 330 degrees in a SINGLE interpolated segment, and the
# place swept 227 degrees on the shoulder while the wrist turned exactly 180. The arm
# visibly collides with itself on the first of those, which the planner permitted because
# it validates the configurations it hands back rather than the straight joint-space line
# the executor draws between two of them - and after `tidy` drops the repeats, two
# configurations is all that is left.
#
# Retrying is NOT what fixes it, and the measurement is worth keeping. Asked 20 times,
# the approach from home to above the pick returned exactly 330.3 degrees every single
# time - the planner is deterministic for that pose, so a bigger allowance only spends
# teach time. The real cause was arm-a's pan limit: home sits at -153.5 and the working
# pose at +176.8, which are 29.7 degrees apart physically but 330 apart in joint value,
# because the +-180 wrap point sits in the middle of arm-a's working range. Widening the
# limit to [-200, 180] - a real UR5e pans +-360 - let the planner express the working
# pose at -183.2 and the move became 30 degrees.
#
# A big allowance also does harm. A budget re-plan rewinds by replaying the path
# backwards, and doing that eight to twenty times while the arm is holding the part moves
# the part in the cup: at 8 and 20 the teaching pass failed at the second check, with
# every face refused by the hold check, where at 2 it passes.
#
# So the allowance stays small and the SEVERE label stays for what it is worth - a loud
# note that a move needs a structural fix, not another attempt. The moves still over
# budget after this are genuine: 003:arm-a turns its wrist 180.9 degrees and j3 195.7,
# which is the reorientation the ordinary budget already treats as a preference because
# it comes back the same however often it is asked.
SEVERE_FLIP_DEG = 270.0
SEVERE_RETRIES = 4


async def move(clients: Clients, arm: str, pose, state: WorldState,
               constraints: Constraints = None, *,
               clients_for_diag: Optional[Clients] = None) -> None:
    program = clients.program
    if program is not None:
        key = program.key(arm)
        program.step += 1
        # Replay a step that is already taught, whether or not this is a teaching pass.
        #
        # For a teaching pass that matters more than it looks. The round is one sequence
        # until the verdict and two after it, so both endings need teaching, so two
        # rounds are run - and if the second re-plans the shared prefix, the program ends
        # up holding one pass's shared steps and the other pass's tail. Measured: the
        # defective ending's place step started 263 degrees away from where the clean
        # pass left arm-b, because they are the same step number taught from two
        # different runs. Replay puts both endings on one shared prefix by construction.
        path = program.paths.get(key)
        if path is not None:
            await clients.arms[arm].do_command(
                {"command": "replay", "path": path})
            program.reached[arm] = [float(v) for v in pose[0]]
            return
        if not program.teaching:
            raise RuntimeError(
                f"the taught program has no step {key}. It was taught from a "
                f"different sequence of moves than this round is making, so "
                f"replaying it would run the wrong path.")
    position, orientation = pose
    # Planning is randomised, so a refusal or a stall is often just this attempt.
    # Re-planning the same move usually succeeds, and at teaching time that is the
    # difference between a program and another failed pass. The recording for this step
    # is cleared each try, so a retried move contributes one path rather than the failed
    # attempt's waypoints followed by the good ones.
    #
    # Randomised also means a move that SUCCEEDS can still be a bad answer. The same
    # station gets reached through a different arm configuration on different tries, and
    # the expensive ones cost a wrist flip - measured, an 80 mm descent that spent 220
    # degrees of wrist 2, and a 190 mm move that spent 270 of wrist 3. Nothing fails; the
    # part arrives; the cell just moves like that on every cycle afterwards. So a taught
    # move is checked against what its geometry justifies and re-planned if it is over,
    # rewinding the path it just drove so the next attempt starts where this one did.
    #
    # A preference, not a gate: if no attempt comes in under budget the last one is kept.
    # Some moves genuinely have to reorient the tool and a wrist really does turn 180
    # degrees to do it, and a round that refused to finish over that would teach nothing.
    teaching = clients.program is not None and clients.program.teaching
    last: Optional[Exception] = None
    # The budget is worked out from where the arm was last SENT, which is only advanced
    # once a step is accepted. Advancing it before the check made the retry's distance
    # zero, so the second look at a 324 mm move judged it against a 100 degree budget
    # instead of 165 - the retry was graded harder than the attempt it was retrying.
    came_from = clients.program.reached.get(arm) if teaching else None
    failures = 0
    rejects = 0
    while True:
        if teaching:
            clients.program.targets[key] = [round(float(v), 1) for v in position]
            await clients.arms[arm].do_command(
                {"command": "record", "label": key, "reset": True})
        try:
            await clients.motion.move(
                component_name=arm,
                destination=PoseInFrame(reference_frame="world", pose=Pose(
                    x=position[0], y=position[1], z=position[2], **orientation)),
                world_state=state,
                constraints=constraints,
            )
        except Exception as exc:  # noqa: BLE001
            last = exc
            failures += 1
            if failures >= PLAN_ATTEMPTS:
                break
            # Put the arm back where the move started before trying again.
            #
            # A refusal moves nothing and needs none of this. A move that PLANNED and then
            # failed to settle has already driven most of the way, and `stop` holds the
            # arm wherever it got - so the next attempt plans from halfway into a failed
            # trajectory, and the one after that from somewhere worse. Measured: the
            # handoff descent failed all three attempts on five teaching passes in a row,
            # every time, while the same descent replayed from the previous program
            # without trouble. The first attempt was the only real one; the other two
            # were planning from wreckage.
            if teaching:
                partial = (await clients.arms[arm].do_command(
                    {"command": "close_recording"})).get("path") or []
                if partial:
                    await clients.arms[arm].do_command(
                        {"command": "replay", "path": [partial[0]]})
            await asyncio.sleep(SETTLE_S)
            continue
        if not teaching:
            return
        reply = await clients.arms[arm].do_command({"command": "close_recording"})
        path = reply.get("path") or []
        spent = path_travel(path)
        budget = travel_budget(came_from, position)
        flipped = worst_joint(path) > FLIP_DEG
        severe = worst_joint(path) > SEVERE_FLIP_DEG
        allowance = SEVERE_RETRIES if severe else BUDGET_RETRIES
        if (spent <= budget or not flipped) and not severe:
            pass_it = True
        else:
            pass_it = rejects >= allowance
        if pass_it:
            if spent > budget:
                clients.program.overspent[key] = (spent, budget)
            clients.program.reached[arm] = [float(v) for v in position]
            return
        # Never rewind while the arm is holding the part.
        #
        # A budget re-plan rewinds by driving the taught path backwards, and that is a
        # move nobody planned: it is the same geometry, but it starts from a standstill at
        # the far end and it is being made for prettiness rather than for the round. With
        # arm-b holding the part by its +y END - a 377 mm cantilever, the weakest grip in
        # the cell - it breaks the grip. Measured, that is what stopped the last four
        # teaching passes: the second check reported "the hold has moved 221 mm since the
        # lift and the gripper reports holding=False" with the part back on the fixture at
        # [747, 24, 729], and the same round planned without teaching held it every time.
        #
        # So a move made while holding keeps whatever the planner gave. An ugly taught
        # move can be fixed later from the recording; a dropped part ends the pass.
        if (await clients.grippers[arm].is_holding_something()).is_holding_something:
            clients.program.overspent[key] = (spent, budget)
            clients.program.reached[arm] = [float(v) for v in position]
            print(f"    {key} spent {spent:.0f} of {budget:.0f} degrees with a "
                  f"{worst_joint(path):.0f} degree joint"
                  f"{' (SEVERE)' if severe else ''}; keeping it - the arm is holding the "
                  f"part and a rewind would risk the grip")
            return
        rejects += 1
        print(f"    {key} spent {spent:.0f} of {budget:.0f} degrees with a "
              f"{worst_joint(path):.0f} degree joint"
              f"{' (SEVERE)' if severe else ''}; re-planning {rejects}/{allowance}")
        # Drive back the way it came, so the next attempt plans from where this one
        # started rather than from the far configuration this one ended in.
        await clients.arms[arm].do_command(
            {"command": "replay", "path": list(reversed(path))})

    # Out of attempts. Say where the part and the arm actually were: a stall reports
    # joint angles, which do not tell you what the arm was leaning on.
    try:
        where = (await part_pose(clients))["position_mm"]
        flange = await flange_of(clients, arm)
        raise RuntimeError(
            f"{arm} -> {[round(v) for v in position]} failed {PLAN_ATTEMPTS} times, "
            f"last with the part at {[round(v) for v in where]} and the flange at "
            f"{[round(v) for v in flange]}: {last}") from last
    except RuntimeError:
        raise
    except Exception:
        raise last


async def lift_by_shoulder(clients: Clients, arm: str, degrees: float = 16.0) -> None:
    """Raise the arm by moving one joint, leaving every other angle alone.

    A Cartesian lift goes through IK, and IK is free to hand back a different wrist for
    the same tool pose. Moving the shoulder cannot: whatever the wrist was holding, it
    still holds it the same way at the top.

    Which way to move it is measured, not assumed. This used to take a signed angle, and
    the sign that raised the arm in one configuration drove it into the belt in another
    - the cell was rebuilt, the pick moved 46 mm, the planner picked a different branch,
    and a hard-coded -16 degrees became a hard-coded 16 degrees in the wrong direction.
    The arm then pressed the part into the belt and reported a stuck joint. So: nudge
    three degrees, look at where the flange went, and go that way.
    """
    start = list((await clients.arms[arm].get_joint_positions()).values)
    before_z = (await flange_of(clients, arm))[2]
    for sign in (1.0, -1.0):
        probe = list(start)
        probe[1] = start[1] + sign * 3.0
        try:
            await clients.arms[arm].move_to_joint_positions(
                JointPositions(values=probe))
        except Exception:  # noqa: BLE001 - that way is blocked, so it is not the way up
            continue
        if (await flange_of(clients, arm))[2] > before_z + 1.0:
            arrived = list(start)
            arrived[1] = start[1] + sign * abs(degrees)
            await clients.arms[arm].move_to_joint_positions(
                JointPositions(values=arrived))
            return
        await clients.arms[arm].move_to_joint_positions(JointPositions(values=start))
    raise RuntimeError(
        f"{arm}: neither direction on the shoulder raises the flange from "
        f"{[round(v, 1) for v in start]}")


async def move_in_steps(clients: Clients, arm: str, start, pose, state: WorldState,
                        step_mm: float = 60.0,
                        constraints: Constraints = None) -> None:
    """Walk to a pose in short hops rather than one jump.

    RDK seeds IK from the current configuration and scores by joint distance, so a small
    move stays on the branch the arm is already in. A big one does not: asked to lift
    240 mm in one go it repeatedly picked the wrist-flipped solution, a 158 degree roll
    of j5.

    The branch choice is real and this still guards against it. What was NOT established
    is the explanation that used to be written here - that the roll "jams the moment a
    part is welded to the cup". Every such jam in this project turned out to be the
    arm's undamped drives ringing rather than settling (see `drive_damping_ratio`), and
    a held part rolls 180 degrees in 30 degree steps with zero error. Whether stepping
    is still needed now the drives are damped has not been measured.
    """
    (sx, sy, sz), _ = start
    (tx, ty, tz), orientation = pose
    span = max(abs(tx - sx), abs(ty - sy), abs(tz - sz))
    steps = max(1, int(span / step_mm))
    for index in range(1, steps + 1):
        fraction = index / steps
        waypoint = ((sx + (tx - sx) * fraction, sy + (ty - sy) * fraction,
                     sz + (tz - sz) * fraction), orientation)
        await move(clients, arm, waypoint, state, constraints)


def lifted(pose, by_mm: float):
    position, orientation = pose
    return (position[0], position[1], position[2] + by_mm), orientation


# ---- the round ---------------------------------------------------------------

# How far above the grip the pick approaches from, and therefore how long the straight
# segment down onto the part is. It is NOT free headroom: the descent is constrained to a
# line, and a linear path has to be IK-valid at every point along it where a free path
# only has to be valid at its ends. Measured at the -x grip on the infeed:
#
#     approach 140 mm   straight-down REFUSED   free-down ok
#     approach 100 mm   straight-down REFUSED   free-down ok
#     approach  70 mm   straight-down ok
#     approach  45 mm   straight-down ok
#     approach  25 mm   straight-down ok
#
# Re-measured after the arms were raised to 620 mm and the infeed moved to x 800, which
# changes which configurations the line is reachable through:
#
#     approach  60 mm   REFUSED
#     approach  40 mm   ok
#     approach  25 mm   ok
#     approach  15 mm   ok
#
# `pick_window` called this pose reachable, and it was: it asked for a free move. The
# planner failing "to find path" on the constrained one says nothing about the goal.
CLEAR_MM = 35.0
# The set-down approaches from higher up than other moves, but not by as much as it
# once did. The 275 mm it used to take was measured back when the set-down had to turn
# the part through a half turn on the way in, and it was the single most expensive pair
# of moves in the round - over 2000 degrees of joint travel between going up and coming
# back down. The turn is decided before the approach now, so the headroom only has to
# clear the stand.
SET_DOWN_CLEAR_MM = 160.0
# And the place onto an outfeed conveyor comes down from higher than a plain move too,
# for a different reason: the two outfeeds are stepped, and approaching the low one puts
# the tool - which hangs 145 mm off the flange along the grip axis - straight through the
# high one's deck. Measured, at 140 mm of clearance it fouled it by 6 mm and the planner
# refused every solution.
PLACE_CLEAR_MM = 200.0
SETTLE_S = 0.6
# How many times a move is re-planned before the round gives up on it.
PLAN_ATTEMPTS = 3


def home_pose(cell: Cell, arm: str) -> Pose:
    """Each arm's home: out in front of its own base, tool pointing away.

    Every leg of the round starts and ends here, which is most of what stops the
    trajectories looking strange. The planner seeds IK from wherever the arm happens to
    be and scores by joint distance, so two legs that begin from different poses get
    different branches - measured, the pan joint swung 144 degrees between one station
    and the next, and 101 degrees between two poses of the same check. Returning to one
    known pose between legs means every leg is planned from the same place.

    Derived from the base rather than written down, so moving an arm moves its home with
    it. In front, not behind: with the 180 degree base yaw a pose behind the arm sits on
    the pan wrap point, and going there between every move is what wound the joints up.

    Tool outward, away from the cell and away from the arm's own body. A tucked home
    folds the flange back toward the upper arm, and since both tools are declared to the
    planner at all times, an idle arm intersecting its own tool makes every plan for the
    OTHER arm fail - which reads as the moving arm being stuck.
    """
    base = cell.arm_bases_mm[arm]
    outward = 1.0 if base[1] > 0 else -1.0
    # On the SAME SIDE of the cell as the work, which matters more than it sounds.
    #
    # Pan is clamped to +-180 so that one pose has one solution. Home used to sit
    # outboard of the base while every station is inboard of it, which puts the two on
    # opposite sides of that wrap: measured, home was at joint pan +151 degrees and the
    # stations at -153, and the arm could not take the 56 degree route between them
    # because it crosses the limit. It unwound 304 degrees the other way instead, twice
    # a round, and that was the largest single movement in the cell.
    #
    # The tool still points outward, away from the cell and the other arm, so an idle
    # arm is neither in the way nor folded back on itself.
    return ((base[0] + 300.0, base[1] - outward * 150.0, base[2] + 250.0),
            {"o_x": 0.0, "o_y": outward, "o_z": 0.0, "theta": 0.0})


def carry_neutral_pose(cell: Cell, arm: str) -> Pose:
    """A plain pose between legs, on the arm's own side of the cell."""
    base = cell.arm_bases_mm[arm]
    inward = -1.0 if base[1] > 0 else 1.0
    return ((430.0, base[1] + inward * 150.0, 780.0),
            {"o_x": 0.0, "o_y": -inward, "o_z": 0.0, "theta": 0.0})


async def home(clients: Clients, arm: str, props, *, cell: Optional[Cell] = None,
               trace: Optional[Trace] = None, stage: str = "") -> None:
    """Send an arm to its park CONFIGURATION, in joint space.

    Not a cartesian goal handed to the planner. A pose has many configurations and the
    planner picks one on the day: measured from the taught program, arm-a reached the
    same park position at pan +83 on one leg and pan +20 on another, so "home" was a
    different shape each time and the move away from it unwound 265 degrees of pan
    against the +-180 clamp. Park is one configuration, and commanding it as joints is
    what makes it one.

    WHICH configuration matters more than it looks. Park used to be a position - a bit
    up and in from the base - solved for on the day, and the planner solved it with the
    arm facing forwards. Measured, nothing else in arm-a's round can be reached that
    way: the pick and both handoff grips have no solution facing the station, only with
    pan swung to -155 and the shoulder past -130. Worse, the forward-facing park led the
    planner into the pick with the elbow the other way up, and from THAT shape the
    part's free side has no solution at all - so the round flipped the elbow back
    mid-presentation, at 270 degrees, having already spent 185 getting onto the wrong
    branch and 217 coming off it.

    So park is not a place to be reached but a shape to retreat into: the arm holds the
    shape it works in and lifts its shoulder 30 degrees, which puts arm-a's tool tip at
    769 mm and arm-b's at 870, clear of the belt at 584 and both conveyors at 400. The
    round then never changes branch. `home_pose` remains the cartesian description of
    roughly where park is, for the layout tests, and is no longer what is commanded.

    A joint move with no collision checking, which is safe here for the same reason the
    rest of the replay is: park is 30 degrees of one joint away from a pose the planner
    accepted, along an arc that lifts the tool away from everything.
    """
    shape = cell or Cell.from_fragment()
    joints = shape.home_joints_deg.get(arm)
    if joints:
        await clients.arms[arm].move_to_joint_positions(JointPositions(values=joints))
    else:
        await move(clients, arm, home_pose(shape, arm), world_state(props, cell=shape))
    if trace is not None:
        trace.record(stage or f"home_{arm}", arm=arm,
                     flange=await flange_of(clients, arm))


# Kept so the older probes keep working; the round says `home`.
park = home


async def feed(clients: Clients, cell: Cell, trace: Optional[Trace] = None,
               timeout_s: float = 20.0, attempts: int = 3) -> Dict[str, Any]:
    """Run the infeed until the beam stops the part, and report where it came to rest.

    The part spawns upstream on open deck; the conveyor brings it to the pick point and
    the beam decides where that is. Nothing may pick before this has returned.

    **It jams, and that is not a bug to tune away.** A 495 mm part on a roller bed with
    side guides wedges across the channel at a few degrees of skew, and this mailbox is
    asymmetric - a hinged door on one side, a flag on the other - so it develops that
    skew on its own. Measured over 6 feeds per setting: a 194 mm channel delivered 2/6,
    3/6 once the guides were given UHMW friction, and a 210 mm channel 5/6; widening to
    230 also gave 5/6 but doubled the skew at the stop, and halving the belt speed made
    it worse, not better. The residual rate is about one jam in six.

    A real line clears a jam by REVERSING, which un-wedges the part and lets it re-enter
    square, so that is what this does rather than reporting a broken cell. What must not
    happen is a jam falling through to a pick: a jammed part sits a metre from where the
    arm expects it, and the arm would find out by planning a perfectly good move to thin
    air.
    """
    forward = float((await clients.world.do_command(
        {"command": "conveyor"}))["conveyor"]["speed_mm_s"])
    for attempt in range(1, attempts + 1):
        await clients.world.do_command(
            {"command": "conveyor", "run": True, "speed_mm_s": forward})
        track: List[float] = []
        waited, stopped = 0.0, False
        while waited < timeout_s:
            await asyncio.sleep(0.5)
            waited += 0.5
            state = (await clients.world.do_command(
                {"command": "conveyor"}))["conveyor"]
            track.append(round((await part_pose(clients))["position_mm"][1], 1))
            if state["stopped_by_beam"]:
                stopped = True
                break
        await clients.world.do_command({"command": "conveyor", "run": False})
        if stopped:
            await asyncio.sleep(SETTLE_S)
            where = await part_pose(clients)
            if trace is not None:
                trace.record("fed", part=where["position_mm"], attempt=attempt)
            return where
        if attempt < attempts:
            # Back it off and let it re-enter square.
            await clients.world.do_command(
                {"command": "conveyor", "run": True, "speed_mm_s": -abs(forward) / 2})
            await asyncio.sleep(1.5)
            await clients.world.do_command({"command": "conveyor", "run": False})
            await asyncio.sleep(1.5)
    raise RuntimeError(
        f"the infeed jammed on all {attempts} attempts, reversing between each; the "
        f"part last tracked {track[:3]}...{track[-3:]} and is at "
        f"{[round(v) for v in (await part_pose(clients))['position_mm']]}. A pick from "
        f"here would be planned to thin air.")


async def pick(clients: Clients, cell: Cell, props, trace: Trace,
               arm: str = "arm-a", side: Optional[str] = None) -> Held:
    """Take the part off the belt by a flat side.

    The part is an obstacle on the way in and not on the last move: approaching with it
    declared keeps the arm from swiping it off the belt, and the final move is a contact
    by intention rather than a collision.
    """
    side = side or cell.GRIP_SIDE
    spawn = await part_pose(clients)
    # Grip where the part IS, not where the layout says it should be. The beam stops it
    # within about 2 mm along the belt but its lateral rest varies by 12 mm inside the
    # guide channel - sometimes hugging a rail, sometimes not - and the cup stands off
    # only 6 mm from the face. Built from `cell.pick()` the cup would have been driven
    # up to 7 mm INSIDE the part on a rail-hugging stop. `Cell.pick_mm` remains what the
    # layout is checked against; it is not what the arm is commanded to.
    grip = cell.side_grip(spawn["position_mm"], side)
    # The part is upright on the belt now, and this is the only moment that is certain.
    # Every later set-down asks for this orientation, turned by a stated amount.
    upright = tuple(float(v) for v in spawn["orientation_wxyz"])
    trace.record("before_pick", part=spawn["position_mm"],
                 orientation_wxyz=list(upright))
    await move(clients, arm, lifted(grip, CLEAR_MM), world_state(props, cell=cell))
    trace.record("approach_pick", arm=arm, flange=await flange_of(clients, arm))
    await move(clients, arm, grip,
               world_state(props, cell=cell, ignore=[PART_LABEL]), STRAIGHT)
    trace.record("at_contact", part=(await part_pose(clients))["position_mm"],
                 flange=await flange_of(clients, arm))

    caught = await clients.grippers[arm].grab()
    await asyncio.sleep(SETTLE_S)
    held = await measure_hold(clients, arm, cell, upright_wxyz=upright)
    landed = (await part_pose(clients))["position_mm"]
    trace.record("after_grab", claimed=caught, part=landed,
                 flange=await flange_of(clients, arm), offset=held.offset_mm)
    # A cell that did not pick the part up has nothing to inspect, and every measurement
    # after this point is derived from a hold that does not exist.
    #
    # This was recorded and not checked, and the cost was the whole round: the gripper
    # reported `claimed: False`, the arm went on to present five faces of a part still
    # sitting on the belt, set down nothing, and the failure finally surfaced as arm-b
    # being asked to reach 1020 mm into arm-a's half of the cell for a part it thought was
    # on the stand. The same rule the presentations already follow - a check that could
    # not see a face is a fault, not a pass - applies to the grasp.
    if not caught:
        raise RuntimeError(
            f"{arm} closed on the part at {[round(v) for v in grip[0]]} and reported "
            f"nothing held; the part is at {[round(v) for v in landed]}. Everything after "
            f"this would be derived from a hold that does not exist.")

    # Lift in JOINT space, by raising the shoulder, before anything asks for a roll.
    #
    # Both halves of that matter. The part has to come up first: at the grasp its
    # underside is level with the belt, so turning it about its long axis would drive a
    # corner straight into the belt. And the lift avoids IK because asking for a pose
    # 240 mm up let the planner pick the wrist-flipped branch, a 158 degree roll.
    #
    # The observation that first motivated this - "the wrist would not move at all while
    # holding, at any commanded angle" - was real but wrongly explained. The cause was
    # the arm's undamped drives, not the belt and not the load. The belt clearance
    # argument stands on its own geometry; the load argument does not.
    await lift_by_shoulder(clients, arm, degrees=16.0)
    lifted_to = (await part_pose(clients))["position_mm"]
    trace.record("after_lift", part=lifted_to, flange=await flange_of(clients, arm))
    # And the part has to have come UP with the arm. `grab()` reporting success is the
    # gripper's opinion; the part moving is the simulator's. They disagreed once already
    # in this project - "holding=True with the carton still on the belt" - which is why
    # probes/grasp_reliability.py scores both.
    rose = lifted_to[2] - spawn["position_mm"][2]
    # Clearing the guide rail is the real bar, not "moved at all". guide_l stands 40 mm
    # proud of the deck BETWEEN arm-a and the part, and the grip face can be resting
    # flush against it, so a part lifted 20 mm is still 20 mm below the rail top and the
    # first move towards the arm would drag its underside across it.
    need = 20.0
    if cell.guide_top_mm is not None:
        underside = spawn["position_mm"][2] - cell.part.dims_mm[2] / 2.0
        need = max(need, cell.guide_top_mm - underside + GUIDE_CLEARANCE_MM)
    if rose < need:
        raise RuntimeError(
            f"{arm} says it is holding the part but the part rose only {rose:.0f} mm "
            f"while the arm lifted; it is not being carried")
    return held


async def lift_from(clients: Clients, cell: Cell, props, held: Held, target,
                    surface: str) -> None:
    """Take the part straight up off a surface it is resting on."""
    await move_in_steps(clients, held.arm, target, lifted(target, CLEAR_MM),
                        world_state(props, cell=cell, held=held, ignore=[surface]))


async def set_down(clients: Clients, cell: Cell, props, trace: Trace, held: Held,
                   centre_mm, part_wxyz, stage: str, surface: str = "",
                   clear_mm: float = SET_DOWN_CLEAR_MM) -> None:
    """Put the part down on a surface, at a stated pose, and let go.

    The caller says where the PART goes, not where the flange goes, and `flange_for`
    derives the rest from the measured hold. Asking for a flange pose is what broke the
    handoff: the arithmetic that places a flange relative to a part is only true for the
    way the part was first picked up, and a set-down turns it.

    Two other things this has to get right.

    The surface drops out of the obstacle set for the descent: setting a part down is
    touching the thing you set it on, and with the surface declared the planner refuses -
    correctly, since only the caller knows the contact is intended.

    The approach height is the caller's, because the two set-downs need different
    things. The handoff turns the part to face the other arm and needs room above the
    stand to do it - measured, the same pose is reachable at 900 mm and refused at 773.
    A tray place turns nothing, and buying that headroom there costs reach the far tray
    does not have: approached 275 mm up, the bad tray produced no IK solutions at all.
    """
    arm = held.arm
    # The surface stays DECLARED for the descent.
    #
    # It used to be dropped from the obstacle set, on the reasoning that setting a part
    # down is touching the thing you set it on and the planner refuses to plan into
    # contact. But the part is released with a gap, so it never actually reaches the
    # surface - and dropping the surface does not just permit the contact, it hides the
    # stand from the planner for the whole descent, including from the ARM. Measured, that
    # is what the handoff's last hop kept failing on: "j3 never arrived, with no drive at
    # its limit, so something stopped the arm rather than out-torqued it" - the planner
    # had routed the wrist through a stand it could not see, and the arm jammed on it.
    #
    # If the planner does refuse, the answer is a larger release gap, not a blind descent.
    ignore: List[str] = []
    # Measure the hold again rather than trusting the one taken at the grasp. Everything
    # below is derived from it, and by now the part has been through five presentations -
    # any slip on the cup during those would be silently designed into the set-down.
    held = await measure_hold(clients, arm, cell, upright_wxyz=held.upright_wxyz)
    target = flange_for(held, centre_mm, part_wxyz)
    above = lifted(target, clear_mm)
    await unwind_wrist(clients, arm)
    await move(clients, arm, above, world_state(props, cell=cell, held=held))
    trace.record(f"pre_{stage}", arm=arm, flange=await flange_of(clients, arm))
    # Down in short hops, unconstrained. A linear constraint is refused here even for a
    # 69 mm hop once the carried part is declared - measured, "motion planner failed to
    # find path" - and it buys nothing anyway: 60 mm waypoints are already a straight
    # line to look at. The constraint is worth having only where it is cheap, which is
    # the moves made with an empty arm.
    await move_in_steps(clients, arm, above, target,
                        world_state(props, cell=cell, held=held, ignore=ignore))

    # Where the part is while the cup still has it, BEFORE letting go.
    #
    # Without this a bad placement and a bad settle are the same record. Measured, the
    # round reported the part at [697, 91, 744] against a seat at [780, 0, 733] and there
    # was no way to tell whether the arm had put it there or whether it had been placed
    # correctly and then tipped off the fixture, which are different faults with different
    # fixes.
    trace.record(f"seated_{stage}", arm=arm,
                 part=(await part_pose(clients))["position_mm"],
                 wanted=[round(float(v), 1) for v in centre_mm],
                 flange=await flange_of(clients, arm))
    await clients.grippers[arm].open()
    await asyncio.sleep(SETTLE_S * 2)
    trace.record(stage, part=(await part_pose(clients))["position_mm"],
                 flange=await flange_of(clients, arm))
    # One straight move, not a sequence of hops. Hopping out cost 956 degrees of joint
    # travel for 160 mm of retreat, because each hop was a fresh plan free to pick a
    # different arm configuration; asked for the whole thing at once, with the arm now
    # empty, it is one short line.
    await move(clients, arm, above, world_state(props, cell=cell), STRAIGHT)
    trace.record(f"retreat_{stage}", arm=arm, flange=await flange_of(clients, arm))


async def unwind_wrist(clients: Clients, arm: str, step_deg: float = 30.0) -> None:
    """Bring the tool roll back toward zero, in joint space, a step at a time.

    A presentation leaves the wrist wherever it ended, and the next Cartesian plan then
    starts from a wound-up configuration. Joint space does not reconsider the wrist the
    way IK does.

    This was added when such moves failed and the failure was read as the loaded arm
    being unable to complete the roll. That reading was wrong - the drives were
    undamped and ringing - so treat this as a precaution whose necessity is unmeasured,
    not as evidence about what a loaded arm can do.
    """
    for _ in range(12):
        joints = list((await clients.arms[arm].get_joint_positions()).values)
        roll = joints[5]
        if abs(roll) < step_deg:
            joints[5] = 0.0
            try:
                await clients.arms[arm].move_to_joint_positions(
                    JointPositions(values=joints))
            except Exception:  # noqa: BLE001
                pass
            return
        joints[5] = roll - step_deg * (1 if roll > 0 else -1)
        try:
            await clients.arms[arm].move_to_joint_positions(
                JointPositions(values=joints))
        except Exception:  # noqa: BLE001 - as far as it gets is far enough
            return


async def carry_neutral(clients: Clients, cell: Cell, props, held: Held) -> None:
    """Bring the part back to a plain pose before the next leg.

    Planning from the end of a presentation - wrist rolled, arm folded across itself -
    is where the planner times out. A neutral pose in between costs one move and gives
    every following plan a sane start.
    """
    await unwind_wrist(clients, held.arm)
    await move(clients, held.arm, carry_neutral_pose(cell, held.arm),
               world_state(props, cell=cell, held=held), HOLDING)


async def handoff(clients: Clients, cell: Cell, props, trace: Trace, held: Held,
                  to: str = "arm-b", side: str = "-y") -> Held:
    """Pass the part between the arms by setting it down and letting the other take an end.

    Set down the way it was carried, NOT turned. The half turn this used to ask for is
    geometrically impossible for this cell and was never reachable: arm-a holds the part by
    the face pointing world -x, so turning the part half a turn swings that face - and
    arm-a's flange with it - to the far side of the part, 452 mm further out, and leaves
    arm-a reaching to x 668 with its tool pointing back at itself. Measured, sweeping the
    approach height: no IK at +160, +100, +60 or 0 mm above the seat. Unturned, the same
    set-down solves at every one of those heights with the flange at [252, 11, 735] and
    479 mm of reach.

    That is the same fact the cell established long ago in a different place - "neither arm
    has IK for +x; both bases sit at x=0, so gripping it means reaching past the part and
    pointing the tool back at yourself" - and the handoff asked for exactly that pose
    anyway. It is also why the fixture had been pushed out to x 900: the position was
    chosen to make this pose reach, and no position can.

    What the turn was for still has to hold, and it does without it. The second check has
    to see the face arm-a's cup was covering, so arm-b must cover a different one - and
    arm-b takes the part by its +y END, 0 degrees off its own forward direction and 474 mm
    out, which leaves both big side panels, the far end, the crown and the base free. The
    face arm-a covered is among them.

    The cost is arm-a's give, 57 degrees off its forward direction - outside the cell's
    comfort bound, inside its IK. That is a real trade and `stations()` reports it rather
    than hiding it.

    Not mid-air either. Both cups lock all six degrees of freedom, so with both closed on
    one rigid body any drive mismatch is reported as joint load, and load is what breaks
    the shear limit.
    """
    # No neutral pose on the way. It was added when planning straight from a rolled
    # wrist timed out, which is what the homes do now, and it cost a full traverse of
    # the cell in each direction for nothing.
    #
    # Upright on the stand, yawed half a turn from the way it came off the belt. That
    # single sentence is the whole handoff: the turn swings arm-a's covered face round to
    # arm-a's own side and the free face round to arm-b's, and "upright" is what keeps
    # the mailbox on its base rather than its crown.
    await set_down(clients, cell, props, trace, held,
                   cell.resting_on(cell.fixture, cell.RELEASE_GAP_MM),
                   held.upright_wxyz, "after_set_down",
                   surface="fixture")
    # The part is on the stand now, and the obstacle list still has it on the belt.
    props = await props_of(clients)
    await home(clients, held.arm, props, cell=cell)
    trace.record("giver_home", part=(await part_pose(clients))["position_mm"],
                 flange=await flange_of(clients, held.arm))

    # arm-b takes the face now pointing +y - its own side, and the opposite face to
    # the one arm-a held.
    # Where the part actually ended up, not where it was aimed. It settles a few
    # millimetres into the stand, and a cup that approaches the nominal pose misses.
    grip = cell.side_grip((await part_pose(clients))["position_mm"], "+y")
    await move(clients, to, lifted(grip, CLEAR_MM), world_state(props, cell=cell))
    trace.record("approach_take", arm=to, flange=await flange_of(clients, to))
    await move(clients, to, grip,
               world_state(props, cell=cell, ignore=[PART_LABEL, "fixture"]))
    caught = await clients.grippers[to].grab()
    await asyncio.sleep(SETTLE_S)
    taken = await measure_hold(clients, to, cell, upright_wxyz=held.upright_wxyz)

    # The take has to have taken, and the part has to come up with the arm.
    #
    # The pick has checked both since the round was written - "everything after this would
    # be derived from a hold that does not exist" - and the handoff checked neither. So a
    # failed take read as five missed faces at the SECOND check, several moves later, with
    # the real cause - the part still sitting on the fixture - nowhere in the error. It
    # cost four teaching passes before the hold check's own message gave it away.
    #
    # The same rule, in the same place in the sequence: fail here, where it happened.
    before = (await part_pose(clients))["position_mm"]
    if not caught:
        raise RuntimeError(
            f"{to} closed on the part at {[round(v) for v in grip[0]]} and reported "
            f"nothing held; the part is at {[round(v) for v in before]} on the fixture. "
            f"Everything after this would be derived from a hold that does not exist.")
    await lift_from(clients, cell, props, taken, grip, "fixture")
    lifted_to = (await part_pose(clients))["position_mm"]
    rose = lifted_to[2] - before[2]
    if rose < CLEAR_MM / 2.0:
        raise RuntimeError(
            f"{to} says it is holding the part but it rose only {rose:.0f} mm off the "
            f"fixture while the arm lifted {CLEAR_MM:.0f}; it is not being carried. The "
            f"part is at {[round(v) for v in lifted_to]}.")
    trace.record("after_handoff", claimed=caught, part=lifted_to,
                 flange=await flange_of(clients, to), offset=taken.offset_mm)
    return taken


# The part is laid on an outfeed conveyor turned a quarter turn from the way it was
# carried, and that is the difference between a place that works and one that does not.
#
# Every other move in this round grips a +y face, so the tool points along y - across
# the direction the arm is reaching. That leaves the wrist trailing about 275 mm in +y,
# straight over whichever conveyor sits beside the one being placed on, and the arm
# fouls it. Stepping the deck heights, moving them in y, moving them in x and turning
# them to run along x each moved which LINK hit which deck, and none of them fixed it:
# the planner picks a different arm configuration for each layout, so a clearance fitted
# to one is wrong for the next.
#
# Turned a quarter turn, the cup ends up on a face pointing back toward arm-b, the
# flange sits BEHIND the part instead of beside it, and the tool points along the
# direction the arm reaches. Nothing of the arm is over either deck. The measured tool
# angle away from radial goes from about 105 degrees to 0.
PLACE_YAW_DEG = 90.0


def laid_flat(held: Held, centre_mm: Vec3):
    """The quarter turn that puts the flange BEHIND the part rather than in front.

    Which way to turn is worked out, not written down. A quarter turn one way puts the
    cup on the face nearer arm-b and the flange behind the part; the other way puts it
    on the far face, and the arm has to reach past the part and point back at itself.
    Which is which depends on the hold - on which material face the cup happened to land
    and how the part was turned at the handoff - so it is decided from the hold that was
    actually measured. Written down as a sign, it was wrong, and the arm reached 345 mm
    past the part to a pose the planner called physically unreachable.
    """
    options = [yawed(held.upright_wxyz, PLACE_YAW_DEG),
               yawed(held.upright_wxyz, -PLACE_YAW_DEG)]
    behind = [q for q in options if flange_for(held, centre_mm, q)[0][0] < centre_mm[0]]
    if not behind:
        raise RuntimeError(
            "neither quarter turn puts the flange behind the part; the hold is not "
            "what this place assumes")
    return behind[0]


def nominal_place_flange(cell: Cell, tray: str):
    """Where the place flange lands, for a cell being reasoned about offline.

    The round derives this from the hold it measured at the time. Offline there is no
    measurement, so this reconstructs ARM-B's hold - which is not arm-a's. arm-b takes the
    part off the fixture by the face pointing world +y, with the part lying the way it came
    off the belt; its flange stands off 175 mm on +y and sits 25 mm below the part's centre.

    Two things it used to get wrong, both the same shape - a number true of an older grip:

    It reconstructed the part half-turned, because the handoff used to ask for that. It no
    longer does; no arm in this cell has IK for the turned set-down at any height.

    And it stood the flange off 175 mm, which was arm-b taking a SIDE face. arm-b takes the
    +y END now, and that stands off 377. The difference is not a detail: at 175 the flange
    sits beside the part and the place reads as 892 mm of reach, while at 377 it sits
    377 mm UPSTREAM of it and the same place reads 871 - and, more to the point, the part's
    far end reaches 247 mm back over the fixture, which is what the round actually failed
    on. The standoff is now read from the cell rather than written here.
    """
    upright = (1.0, 0.0, 0.0, 0.0)
    take_flange = _ov_to_quat(0.0, -1.0, 0.0, 0.0)   # cup pointing -y, at +y of the part
    stand = cell.grip_standoff_mm(cell.hold_face("arm-b")) + CUP_APPROACH_GAP_MM
    held = Held(
        arm="arm-b",
        offset_mm=_rotate_into_frame(take_flange, (0.0, -stand, 0.0)),
        dims_mm=cell.part_local_dims_mm,
        orientation=_quat_to_ov(_quat_mul(_quat_conj(take_flange), upright)),
        upright_wxyz=upright)
    centre = cell.outfeed_point_mm(tray, cell.RELEASE_GAP_MM)
    return flange_for(held, centre, laid_flat(held, centre))


def nominal_handoff_flanges(cell: Cell):
    """The two flanges the handoff actually visits at the fixture, offline.

    `fixture_grip(GRIP_SIDE)` is NOT either of them, and `stations()` reporting it as
    both is what mis-sited the fixture. The round sets the part down YAWED half a turn,
    so the face arm-a is holding swings from world -x round to world +x and arm-a's
    flange goes with it - 452 mm further out than the unyawed grip, which at x 900 asked
    arm-a for 1178 mm of reach against a 1016 mm limit and failed every attempt. arm-b
    then takes the part by its +y end, not by -x.

    So the fixture was positioned against a pose no arm visits: measured on the poses
    above, the window that suits both arms is x 440 to 480, and it was at 900.

    Returns (give, take) flange positions. Nominal: the round derives the real ones from
    the hold it measured at the time, which offline there is no measurement for.
    """
    centre = cell.resting_on(cell.fixture, cell.RELEASE_GAP_MM)
    # The part goes down the way it was carried, so the give IS the grip pose. The half
    # turn this used to mirror is not reachable by any arm in this cell at any height.
    give = cell.side_grip(centre, cell.GRIP_SIDE)[0]
    take = cell.side_grip(centre, "+y")[0]
    return give, take


async def place(clients: Clients, cell: Cell, props, trace: Trace, held: Held,
                tray: str, side: str = "-y") -> None:
    """Lay the part on an outfeed conveyor, turned a quarter turn. See `laid_flat`."""
    centre = cell.outfeed_point_mm(tray, cell.RELEASE_GAP_MM)
    await set_down(clients, cell, props, trace, held, centre,
                   laid_flat(held, centre), f"after_place_{tray}",
                   surface=f"{tray}_conveyor", clear_mm=PLACE_CLEAR_MM)


def _slerp(a, b, fraction: float):
    """Shortest-arc interpolation between two orientations."""
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0.0:                      # same rotation, opposite sign: take the short way
        b, dot = tuple(-v for v in b), -dot
    dot = max(-1.0, min(1.0, dot))
    angle = math.acos(dot)
    if angle < 1e-6:
        return tuple(a)
    sin_angle = math.sin(angle)
    wa = math.sin((1.0 - fraction) * angle) / sin_angle
    wb = math.sin(fraction * angle) / sin_angle
    return tuple(wa * x + wb * y for x, y in zip(a, b))


# How much the tool may reorient in one planned move while holding the part.
#
# The whole argument for this is in one log line from the motion service: "goal requires
# 91.0 deg of reorientation under a 15 deg orientation constraint; the search must stay
# within a 15 deg tube around the direct reorientation path, which can slow planning."
# It does: that plan took 45.5 s, and it took the cell past an 11 GiB memory cap and was
# OOM-killed mid-round. Asked for 30 degrees at a time the tube is short and each plan is
# cheap.
#
# It is the same lesson `roll_to` already carries for a roll in place - "asked for 90
# degrees in one move the planner has to clear that whole arc at once and gives up;
# asked for 45 it does not" - and the reason is the same. The only new part is that this
# reorients the tool's DIRECTION as well as its roll, so the steps are a slerp rather
# than a sequence of thetas.
REORIENT_STEP_DEG = 30.0


# How high the part arcs over the middle of a carry, per millimetre it travels sideways,
# and the ceiling on it.
#
# Interpolating the part in a straight line keeps the PART clear and says nothing about
# the arm. Measured on the move to the free-face station - 294 mm of flange travel in y
# across the infeed - the planner refused it with "violation between arm-a:wrist_2_link
# and belt.deck 98.43%": the part passed over the deck and the wrist went through it.
#
# So the carry arcs. It is what anyone moving a part across an obstacle does, it costs
# nothing in a planned move, and it gives the links behind the flange the same clearance
# the part already has.
# Half the sideways travel was not enough: at 109 mm of arc the planner still refused
# with "arm-a:ee_link and belt.deck 98.96%" - the link a hundred millimetres behind the
# flange, which needs the clearance the flange has.
CARRY_ARC_FRACTION = 1.0
CARRY_ARC_MAX_MM = 250.0


async def carry_to(clients: Clients, arm: str, pose, state: WorldState,
                   held: "Held" = None, step_deg: float = REORIENT_STEP_DEG) -> None:
    """Take the held part to a flange pose by moving the PART along a clear path.

    The waypoints interpolate the PART's pose and the flange is derived from each one,
    rather than the other way round. That is the difference between a path that works and
    one that does not, and it took three tries to see it.

    Interpolating the FLANGE's pose - position and orientation together - swings the part
    through an arc around the flange. Over the infeed that arc dips the part into the deck:
    part-way through the turn a 495 mm part hangs 247 mm below the hold against a deck
    topping out at 769 mm, and the round dragged the part through it.

    Turning in place at one end instead only moves the problem to whichever end. Leaving
    the rolls' station, a turn there swings the part out to y -324, over the belt; a turn at
    the free station swings it to y +88, across the centreline into arm-b's half. Measured,
    both failed - the first with a 100% violation against belt.deck, the second with the arm
    arriving and unable to turn out of it.

    The part is what has to stay clear, so the part is what gets interpolated: it travels
    the straight line between where it is and where it is going - along the camera axis,
    down the middle of the cell, which is clear by construction - and turns as it goes. The
    flange swings around it, which is the arm's problem and the planner's to solve.

    Falls back to interpolating the flange when there is no measured hold to derive a part
    pose from, which is every move made with an empty arm.
    """
    position, orientation = pose
    current = (await clients.motion.get_pose(
        component_name=arm, destination_frame="world")).pose
    here = (current.x, current.y, current.z)
    now_ov = {"o_x": current.o_x, "o_y": current.o_y, "o_z": current.o_z,
              "theta": current.theta}
    start_quat = _ov_to_quat(current.o_x, current.o_y, current.o_z, current.theta)
    goal_quat = _ov_to_quat(orientation["o_x"], orientation["o_y"],
                            orientation["o_z"], orientation["theta"])
    dot = abs(sum(a * b for a, b in zip(start_quat, goal_quat)))
    turn = math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))
    steps = max(1, math.ceil(turn / step_deg))

    if held is None:
        for index in range(1, steps + 1):
            await move(clients, arm,
                       (position, _quat_to_ov(_slerp(start_quat, goal_quat, index / steps))),
                       state)
        return

    (from_centre, from_quat) = part_for(held, (here, now_ov))
    (to_centre, to_quat) = part_for(held, pose)
    sideways = math.dist(from_centre[:2], to_centre[:2])
    arc = min(CARRY_ARC_MAX_MM, sideways * CARRY_ARC_FRACTION)
    steps = max(steps, math.ceil(sideways / 120.0)) if arc else steps
    for index in range(1, steps + 1):
        fraction = index / steps
        centre = tuple(from_centre[i] + (to_centre[i] - from_centre[i]) * fraction
                       for i in range(3))
        centre = (centre[0], centre[1], centre[2] + arc * math.sin(math.pi * fraction))
        await move(clients, arm,
                   flange_for(held, centre, _slerp(from_quat, to_quat, fraction)), state)


async def roll_to(clients: Clients, cell: Cell, props, held: Held, pose,
                  state: WorldState, step_deg: float = 45.0,
                  from_theta: float = None) -> None:
    """Turn the part to a new roll in stages.

    A presentation turns the part about the tool axis without moving it, which swings
    its far corner through an arc more than 200 mm across. Asked for 90 degrees in one
    move the planner has to clear that whole arc at once and gives up; asked for 45 it
    does not.
    """
    position, orientation = pose
    # Where the roll starts from has to be the INTENDED theta, not the measured one.
    #
    # The number of steps decides how many moves this makes, and a taught program is
    # keyed by move number - so if the count ever differs between teaching and replay,
    # every later key is off by one and the replay refuses with "the taught program has
    # no step 019:arm-b". Derived from the measured pose it does differ: a degree of
    # drift either side of a multiple of `step_deg` changes the count. At 20 deg/s the
    # drift was small enough to land the same way every time and this looked safe; at
    # 60 it did not, and two presentations showed the wrong face because the sequence had
    # diverged.
    #
    # The intended thetas are 0, 90, 180, 270 in order, so this is always the same two
    # steps - which is the point. `from_theta` is what the caller asked for last time,
    # and the measured pose is only a fallback for a caller that does not track it.
    if from_theta is None:
        current = await clients.motion.get_pose(
            component_name=held.arm, destination_frame="world")
        from_theta = current.pose.theta
    start = from_theta
    target = orientation["theta"]
    delta = (target - start + 180.0) % 360.0 - 180.0
    steps = max(1, int(round(abs(delta) / step_deg)))
    for index in range(1, steps + 1):
        theta = start + delta * index / steps
        await move(clients, held.arm, (position, {**orientation, "theta": theta}), state,
                   HOLDING)


def _rotation(wxyz):
    w, x, y, z = (float(v) for v in wxyz)
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def face_towards(wxyz, part_mm: Vec3, camera_mm: Vec3) -> str:
    """Which face of the PART points at the camera, in the part's own frame.

    The world axis a presentation aims at the lens says nothing about the part once the
    part has been turned, and the handoff turns it half a turn on purpose.
    """
    rows = _rotation(wxyz)
    towards = tuple(camera_mm[i] - part_mm[i] for i in range(3))
    body = tuple(sum(rows[i][axis] * towards[i] for i in range(3)) for axis in range(3))
    axis = max(range(3), key=lambda i: abs(body[i]))
    return f"{'+' if body[axis] > 0 else '-'}{'xyz'[axis]}"


async def capture(clients: Clients, tag: str) -> Optional[str]:
    """Grab an inspection frame and say where it went.

    This frame is the check's input. A round that never takes one cannot be said to have
    inspected anything.

    The name is the PART's face, not the world axis the presentation aimed at the lens.
    Those agree for the first check and disagree for the second, because the handoff
    turns the part half a turn: a frame called `second_check_+x` shows the part's -x
    face, so a learner labelling frames by filename would train on +x and -x swapped.

    The extension follows what the camera actually returned. It encodes JPEG unless PNG
    is asked for, so `.png` was a lie on every frame on disk.
    """
    if clients.camera is None or clients.frames_dir is None:
        return None
    images, _ = await clients.camera.get_images()
    image = images[0]
    suffix = ".png" if bytes(image.data)[:4] == b"\x89PNG" else ".jpg"
    clients.frames_dir.mkdir(parents=True, exist_ok=True)
    path = clients.frames_dir / f"{tag}{suffix}"
    path.write_bytes(bytes(image.data))
    return str(path)


async def _stopped(clients: Clients, cap: float = 4.0) -> None:
    """Wait until neither arm is moving, so two frames differ only by what changed.

    A fixed sleep cannot do this: how long an arm takes to stop depends on how far it
    just came, and the blink comparison is only meaningful between frames of a scene that
    has stopped. Capped, because a drive that never quite settles must not hang a round -
    the comparison then just measures what it measures and the test says so.
    """
    deadline = asyncio.get_event_loop().time() + cap
    while asyncio.get_event_loop().time() < deadline:
        moving = [await arm.is_moving() for arm in clients.arms.values()]
        if not any(moving):
            return
        await asyncio.sleep(0.1)


async def _blink(clients: Clients, cell: Cell, trace: Trace, stage: str,
                 seen: str) -> Dict[str, Any]:
    """If a mark is on the face now facing the lens, photograph it switched off too.

    This is the one thing the round does that is about pixels. Everything else about a
    defect is bookkeeping: the oracle knows a mark is on because it switched it on, and
    knows the face was presented because it did the arithmetic from the simulator's
    poses. None of that says the mark landed inside the frustum, that the cup or the
    wrist was not in front of it, or that a 50 mm patch resolves at this lens - and the
    camera rendered the part as a black silhouette for the whole of this project's life
    while every one of those checks stayed green.

    Both frames are taken the same way, of a scene that has stopped moving, so the
    comparison is between the mark and nothing else. The first version reused the
    presentation's own frame, taken before the arm had finished settling, and compared it
    against one taken after - so the difference it measured was partly the arm still
    moving. Waiting a fixed settle instead was better and still not right: measured, a
    pair taken 0.6 s apart differed in 3433 scattered pixels across a 560x277 region,
    which is an arm that had not finished drifting, and the test that asks a mark to be a
    compact patch rightly refused it. So wait for the arms to report still.
    """
    recorded = [x for x in trace.stages if x["stage"] == "defects"]
    showing = (recorded[-1].get("showing") if recorded else {}) or {}
    on = [name for name, face in cell.defect_faces().items()
          if face == seen and showing.get(name)]
    if not on or clients.frames_dir is None:
        return {}
    await _stopped(clients)
    with_mark = await capture(clients, f"{stage}_{seen}_with")
    # Blink by VISIBILITY, not by choosing a different defect. Which mark the part carries
    # is settled before the round moves; hiding it for one frame must not recompose the
    # part, because recomposition invalidates the physics views and the next arm move then
    # fails reading its own joint velocities.
    await clients.world.do_command(
        {"command": "show_defects", "prop": PART_LABEL, "visible": False})
    await _stopped(clients)
    without = await capture(clients, f"{stage}_{seen}_without")
    await clients.world.do_command(
        {"command": "show_defects", "prop": PART_LABEL, "visible": True})
    return {"marks_here": sorted(on), "frame_with": with_mark,
            "frame_without": without}


def _upright_through_turn(held: Held, pose, from_flange, step_deg: float = 15.0):
    """Pick the roll of a presentation that keeps the part's long axis vertical.

    The free face is reached by swinging the tool 90 degrees in the x-y plane, and the roll
    it arrives at is free as far as framing goes - the roll spins the part about the tool
    axis and the tool axis points at the camera, so every roll shows the same face. It is
    not free for clearance, and the difference is the whole pose.

    The part's long axis is always perpendicular to the tool, so a tool swinging within the
    x-y plane can leave that axis pointing along z - and if it is vertical at both ends,
    the rotation between them is about z and it stays vertical the whole way. Then the only
    thing the part presents to the swing is its 172 x 227 cross-section. Pick any other
    roll and the 495 mm length sweeps through the horizontal instead, where its axis-aligned
    extent reaches 400 mm and more in every direction.

    Measured, walking the turn in fortieths against the cell's real obstacles: the roll that
    makes this a pure z rotation clears the infeed deck by 30 mm, and the next best of the
    twelve tried is 94 mm INSIDE it. So this is chosen, from the measured hold, rather than
    written down - the right roll depends on how the part is being held, and the two arms
    hold it differently.
    """
    position, orientation = pose
    now = _ov_to_quat(from_flange.o_x, from_flange.o_y, from_flange.o_z, from_flange.theta)
    _, from_quat = part_for(held, ((from_flange.x, from_flange.y, from_flange.z),
                                  _quat_to_ov(now)))
    best, best_axis = orientation, -1.0
    for step in range(int(360 // step_deg)):
        candidate = {**orientation, "theta": step * step_deg}
        _, to_quat = part_for(held, (position, candidate))
        turn = _quat_mul(to_quat, _quat_conj(from_quat))
        axis = turn[1:]
        length = math.sqrt(sum(v * v for v in axis))
        upright = abs(axis[2]) / length if length > 1e-9 else 1.0
        if upright > best_axis:
            best, best_axis = candidate, upright
    return position, best


async def present(clients: Clients, cell: Cell, props, trace: Trace, held: Held,
                  stage: str) -> List[str]:
    """Show the camera every face the cup is not covering.

    Returns the faces actually reached. A face the arm could not get to is not a face
    the check saw, and a round that silently skips one is a round that passes a defect.
    """
    shown: List[str] = []
    free_face = "+y" if cell.present_side(held.arm) == "-y" else "-y"
    asked_theta = None
    for face, pose in cell.present_poses(held.arm):
        state = world_state(props, cell=cell, held=held)
        try:
            position, orientation = pose
            current = await clients.motion.get_pose(
                component_name=held.arm, destination_frame="world")
            same_place = all(abs(a - b) < 1.0 for a, b in zip(
                position, (current.pose.x, current.pose.y, current.pose.z)))
            if same_place:
                # The theta the cell asked for last, not the one the arm reports.
                # A pure roll, so roll: no unwinding first. This used to wind the wrist
                # back to zero before every pose and then out again to 0, 90, 180, 270,
                # paying for each turn twice and spending more of the round's joint
                # travel on the wrist than on everything else together. `roll_to` takes
                # the short way from wherever the wrist already is.
                await roll_to(clients, cell, props, held, pose, state,
                              from_theta=asked_theta)
            else:
                # No unwind before a pose that moves either, now. It was added when
                # dropping it cost the free-face presentation about two rounds in three
                # - but that was with the old command-and-wait executor and with the
                # free face shown from the rolls' position, both since fixed. The wrist
                # was the most travelled joint in the cell by a wide margin, and most of
                # that was winding back to zero only to wind out again.
                # The rolls' station is the cell's turning space - the one place with
                # room under a part this size - so a pose that ARRIVES there turns after
                # it travels and the one that LEAVES turns before. The four rolls arrive;
                # the free face is the only pose with its own station, so it leaves.
                if face == free_face:
                    # Unwind before leaving the station. The four rolls are 0, 90, 180, 270
                    # and `roll_to` takes the short way each time, so they wind the wrist a
                    # cumulative 270 degrees and leave it there. Measured, that is what the
                    # free face and everything after it were failing on: the pose cost 92 s
                    # over three attempts and still failed, and the set-down that followed
                    # reached the right POSITION with its tool 20 degrees off the commanded
                    # direction - the arm had no wrist left to turn. The part was then
                    # seated 81 mm out in y, overhung the fixture and fell off it. Unwound,
                    # the same pose costs 18 s.
                    #
                    # `unwind_wrist`'s docstring called itself a precaution whose necessity
                    # was unmeasured. It is measured now, here.
                    await unwind_wrist(clients, held.arm)
                    pose = _upright_through_turn(held, pose, current.pose)
                await carry_to(clients, held.arm, pose, state, held=held)
        except Exception as exc:  # noqa: BLE001
            # Kept long on purpose. The arm model goes to real trouble to say WHICH of the
            # several conditions it is - out-torqued, blocked, or a standing error - and
            # truncating that to 400 characters threw the diagnosis away and left the
            # bare RDK wrapper "error moving component arm-a to inputs [[...]]", which
            # says nothing. I have lost a cycle to a truncated planner error in this
            # project before.
            trace.record(f"{stage}_missed_{face}", error=str(exc)[:1600])
            continue
        # Still holding it, and holding it the same way.
        #
        # Without this the round reported all five faces shown with the part lying on the
        # floor at [-668, 500, 113]: the presentations reached their poses, `shown` filled
        # up, and every frame was of an empty gripper. The same rule the reach failures
        # already follow - a check that could not see a face is a fault, not a pass -
        # has to cover a check whose subject is not there.
        #
        # Measured against the hold, not against the gripper's opinion, because the two
        # disagree: at the 213 mm slide the cup still reported holding something.
        gripping = await clients.grippers[held.arm].is_holding_something()
        moved = math.dist(
            (await measure_hold(clients, held.arm, cell)).offset_mm, held.offset_mm)
        if not gripping.is_holding_something or moved > HOLD_DRIFT_MM:
            trace.record(
                f"{stage}_missed_{face}",
                error=(f"the part is not being held as it was: the hold has moved "
                       f"{moved:.0f} mm since the lift (limit {HOLD_DRIFT_MM:.0f}) and "
                       f"the gripper reports "
                       f"holding={gripping.is_holding_something}. The part is at "
                       f"{[round(v) for v in (await part_pose(clients))['position_mm']]}"))
            continue
        asked_theta = pose[1]["theta"]
        shown.append(face)
        # The hold as measured here, not the world-frame gap between part and flange.
        # A presentation rolls the part about the tool on purpose, so the world-frame
        # offset is supposed to change; what must not change is where the part sits in
        # the flange's own frame, which is the only thing a slip on the cup would move.
        now = await measure_hold(clients, held.arm, cell,
                                 upright_wxyz=held.upright_wxyz)
        # The part's own orientation, so a test can work out which of the PART's faces
        # the camera is looking at. `face` here is the world axis the presentation aims
        # to show, which says nothing about the part once the part has been turned - and
        # both checks report the same five world axes whether or not the handoff turned
        # anything.
        where = await part_pose(clients)
        seen = face_towards(where["orientation_wxyz"], where["position_mm"],
                            cell.camera_mm)
        trace.record(f"{stage}_{face}",
                     part=where["position_mm"],
                     orientation_wxyz=list(where["orientation_wxyz"]),
                     showing_face=seen,
                     flange=await flange_of(clients, held.arm),
                     offset=list(now.offset_mm),
                     frame=await capture(clients, f"{stage}_{seen}"),
                     **await _blink(clients, cell, trace, stage, seen))
    trace.record(stage, shown=shown)
    return shown


async def set_defects(clients: Clients, wanted: Dict[str, bool]) -> Dict[str, bool]:
    """Set exactly which marks are on the part, and wait for them to be drawn."""
    reply = await clients.world.do_command(
        {"command": "set_defects", "prop": PART_LABEL, "defects": wanted})
    return dict(reply["defects"])


async def known_defects(clients: Clients) -> Dict[str, bool]:
    reply = await clients.world.do_command(
        {"command": "defects", "prop": PART_LABEL})
    return dict(reply["defects"])


@dataclass
class Shot:
    """One inspection frame and what the cell knows about how it was taken."""

    stage: str          # "first_check" or "second_check"
    face: str           # which face of the PART was towards the lens
    path: Optional[str]  # the image on disk


def make_oracle(showing: Dict[str, bool], faces: Dict[str, str]):
    """The judgement seat, filled by a stand-in that cheats.

    The seat itself takes only the frames, because that is all a learner's judge can
    honestly have: this is the exercise the course is built around, and a signature that
    also handed over the simulator's answer key would let a judge score full marks
    without opening an image. The oracle closes over the answer key so the seat does not
    have to.

    What the oracle establishes, precisely: the mark was switched on, and some
    presentation turned its face towards the lens. What it does not establish is
    anything about pixels - not that the mark was inside the frustum, not that the cup
    or the wrist was not in front of it, not that a 50 mm patch resolves at this lens.
    `test_the_camera_can_actually_see_the_mark` is what covers that, and R2's annotator
    is the complete answer.

    The intersection matters. A mark on a face neither check presented was not found,
    however certainly it exists, and a verdict that counted it anyway would paper over
    exactly the failure the two checks exist to catch.
    """
    async def judge(shots: List[Shot]) -> Dict[str, Any]:
        presented = {shot.face for shot in shots}
        found = sorted(name for name, on in showing.items()
                       if on and faces.get(name) in presented)
        missed = sorted(name for name, on in showing.items()
                        if on and faces.get(name) not in presented)
        return {"verdict": "bad" if found else "good", "found": found,
                "missed": missed, "presented": sorted(presented)}

    return judge


async def run_round(clients: Clients, cell: Cell, *,
                    defects: Optional[Dict[str, bool]] = None,
                    judge=None, trace: Optional[Trace] = None) -> Trace:
    """One full round, from the belt to whichever tray the checks call for.

    `defects` says which marks are on this part; anything not named is off, so a round
    always starts from a stated condition rather than from whatever the last one left.
    `judge` is an async callable taking the list of `Shot`s the checks captured and
    returning a verdict - the seat the course's learner fills. It is given the frames
    and nothing else. The default is an oracle that closes over the simulator's answer
    key, which is a stand-in and says so.
    """
    trace = trace or Trace()

    # The part's condition is chosen BEFORE the reset, and that order is load-bearing.
    #
    # Choosing which defect exists is a variant change, and a variant change recomposes the
    # prim - which invalidates the physics views, INCLUDING the arms'. `show_defects` exists
    # to avoid that mid-round and says so. Doing it after the reset put the recompose after
    # the only thing that rebuilds those views, so the first arm command of the round then
    # failed with "Failed to get DOF positions from backend".
    #
    # That is why the fault looked random. It only fires when the variants actually CHANGE,
    # so a clean round straight after a defective one failed and a clean round after a clean
    # one did not - which is every other round, and exactly the pattern: of the replays run
    # so far the first after teaching the defective ending failed and the two after it
    # passed. Done before the reset, the reset rebuilds the views and the recompose costs
    # nothing.
    wanted = {name: False for name in await known_defects(clients)}
    wanted.update({str(k): bool(v) for k, v in (defects or {}).items()})
    showing = await set_defects(clients, wanted)

    await clients.world.do_command({"command": "reset"})
    await asyncio.sleep(3.5)
    # A reset snaps the part back but leaves the cups closed, so a round that failed
    # while holding would weld this one to an arm before it started.
    for gripper in clients.grippers.values():
        await gripper.open()

    if clients.frames_base is not None:
        marks = "-".join(sorted(k for k, v in (defects or {}).items() if v)) or "clean"
        clients.frames_dir = Path(clients.frames_base) / marks

    trace.record("defects", showing=dict(showing),
                 faces=cell.defect_faces())

    # The part starts upstream on the infeed and the conveyor brings it in. Until the
    # beam stops it there is nothing at the pick point to pick.
    await feed(clients, cell, trace)

    props = await props_of(clients)

    # Both arms start from home, so the first plan of the round is made from the same
    # place as every later one.
    await home(clients, "arm-b", props, cell=cell, trace=trace)
    await home(clients, "arm-a", props, cell=cell, trace=trace)

    # arm-a: approach, pick, lift, show the camera five faces.
    # The face the measurements chose. `+y` is an END of a 495 mm part: it needs a
    # 229-252 mm standoff and hangs the whole length off one cup, which is the
    # cantilever the cell was explicitly asked not to do.
    held = await pick(clients, cell, props, trace, arm="arm-a", side=cell.GRIP_SIDE)
    await present(clients, cell, props, trace, held, "first_check")

    # arm-a: pre-place, set down on the centre table, release, retreat, home.
    # arm-b: approach, pick, lift.
    taken = await handoff(clients, cell, props, trace, held, to="arm-b", side="-y")
    await present(clients, cell, props, trace, taken, "second_check")

    # The verdict comes from the checks, not from the caller. A round told which tray to
    # use proves the arm can reach that tray and nothing about the cell.
    # A check that could not reach a pose did not inspect that face, and a cell that
    # answers anyway is answering from evidence it does not have. Measured: when
    # arm-b's free-face presentation failed, the round reported a defective part as
    # good - quietly, with the defect listed under `missed`, and the part went to the
    # good conveyor. A missed inspection is a fault, not a pass.
    skipped = [s["stage"] for s in trace.stages if "_missed_" in s["stage"]]
    if skipped:
        raise RuntimeError(
            f"the cell could not show the camera every face - {skipped} - so it has no "
            f"grounds for a verdict. The errors are in the trace.")

    shots = [Shot(stage=s["stage"].split("_check_")[0] + "_check",
                  face=s["showing_face"], path=s.get("frame"))
             for s in trace.stages if "showing_face" in s]
    called = await (judge or make_oracle(showing, cell.defect_faces()))(shots)
    verdict_tray = "bad" if called["verdict"] == "bad" else "good"
    trace.record("verdict", **called, tray=verdict_tray)
    if clients.program is not None:
        clients.program.branch = verdict_tray

    # arm-b: pre-place, place on the conveyor the verdict names, release, retreat.
    await place(clients, cell, props, trace, taken, verdict_tray, side="+y")
    await home(clients, taken.arm, props, cell=cell, trace=trace, stage="taker_home")

    # Where the part finished, read twice. The trays are solid blocks with no walls, so
    # "inside the tray in xy" is true of a part lying on its side half over the edge and
    # still sliding. Two reads a moment apart, plus the orientation, are what make
    # "upright and at rest" a claim rather than a coordinate.
    await asyncio.sleep(SETTLE_S)
    settled = await part_pose(clients)
    await asyncio.sleep(0.6)
    again = await part_pose(clients)
    trace.record("at_rest", tray=verdict_tray, part=settled["position_mm"],
                 again=again["position_mm"],
                 orientation_wxyz=list(settled["orientation_wxyz"]),
                 upright_wxyz=list(taken.upright_wxyz))

    # And then the conveyor takes it away, which is the point of an outfeed.
    #
    # After `at_rest`, not before: at_rest is the proof that the place was good - two
    # reads a moment apart with the part in the same spot - and a part already riding
    # away cannot show that. So the cell proves the set-down first and then runs the
    # belt, which is also the order a line works in.
    await clients.world.do_command(
        {"command": "conveyor", "name": f"{verdict_tray}_conveyor", "run": True})
    await asyncio.sleep(OUTFEED_RUN_S)
    carried = await part_pose(clients)
    await clients.world.do_command(
        {"command": "conveyor", "name": f"{verdict_tray}_conveyor", "run": False})
    trace.record("carried_away", tray=verdict_tray,
                 part=carried["position_mm"],
                 travelled_mm=round(carried["position_mm"][0]
                                    - settled["position_mm"][0], 1))
    return trace
