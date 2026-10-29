"""Where the cell's stations are, computed from the cell.

Station poses used to live in `stations.json`, and by the time anyone looked they encoded
a tool-down grip the cell no longer uses, an 80 mm cube the part no longer is, and a scene
with none of the eighteen props now in it. One station had started failing IK and the rest
were wrong in ways nothing checked. A stored pose cannot notice that the cell moved.

So nothing here is stored. Everything is derived from the fragment - the one place the
cell is actually described - and `tests/test_cell_layout.py` asserts the two properties
the cell's design depends on before any of it reaches a robot.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

FRAGMENT = Path(__file__).resolve().parents[2] / "fragments" / "qc-cell.json"

# Measured from the motion service, not a datasheet: asking the planner for a pose it
# refuses reports "max: 1016.72". The UR5e's 850 mm figure is to the wrist centre.
REACH_MM = 1016.7
# The flange-to-cup standoff and the cup's radius. These are the gripper's own config
# and are read from the fragment below; the values here are only what a cell without a
# gripper would fall back to. They were duplicated by hand in five places before this,
# which is the same drift `stations.json` was deleted for.
TOOL_MM = 120.0
CUP_RADIUS_MM = 25.0
# How far below the part's centre the cup sits on a flat side: low enough to be on the
# flat panel rather than the curved crown. Derived from the part's own flat panel.
# The gap the cup is brought to rather than pressed through: the gripper's own raycast
# starts this far past its attachment point, so a cup landed flush has no room to look.
# Matches CUP_APPROACH_GAP_MM in the module's asset catalogue, with a millimetre over so
# the ray starts outside rather than exactly at the surface.
CUP_APPROACH_GAP_MM = 6.0
GRIP_DROP_MM = 25.0
# Keep stations this far inside the reach bound. Solutions near full extension are where
# a spherical bound stops being honest, because the orientation may not be achievable.
REACH_MARGIN = 0.10

Vec3 = Tuple[float, float, float]
Pose = Tuple[Vec3, Dict[str, float]]

# Viam orientation vectors for a cup pointing along each world axis.
TOOL_MINUS_Y = {"o_x": 0.0, "o_y": -1.0, "o_z": 0.0, "theta": 0.0}
TOOL_PLUS_Y = {"o_x": 0.0, "o_y": 1.0, "o_z": 0.0, "theta": 0.0}
TOOL_DOWN = {"o_x": 0.0, "o_y": 0.0, "o_z": -1.0, "theta": 0.0}
TOOL_MINUS_X = {"o_x": -1.0, "o_y": 0.0, "o_z": 0.0, "theta": 0.0}
TOOL_PLUS_X = {"o_x": 1.0, "o_y": 0.0, "o_z": 0.0, "theta": 0.0}


DERIVED = Path(__file__).resolve().parents[2] / "fragments" / "mailbox-derived.json"


@lru_cache(maxsize=1)
def part_origin_offset_mm() -> Tuple[float, float, float]:
    """How far the part's geometric centre sits from the prim its pose is reported at.

    Zero for a cube, whose origin IS its centre. Not zero for a referenced asset: this
    mailbox's origin is at the bottom of its shell, 39.1 mm under the middle. Everything
    the cell computes from a live part pose - the box the planner is told it is carrying,
    the pose the second arm takes it by - is about the middle, so the offset belongs in
    one place rather than at each call.
    """
    fragment = json.loads(FRAGMENT.read_text())
    world = next(c for c in fragment["components"]
                 if c["model"].endswith(":world"))
    part = next(p for p in world["attributes"]["props"] if p["name"] == "part")
    if str(part.get("type", "cube")) != "usd":
        return (0.0, 0.0, 0.0)
    return tuple(float(v) for v in asset_facts()["origin_to_centre_mm"])


def part_origin_offset_local_mm() -> Tuple[float, float, float]:
    """The same offset, in the PART's own frame, so it can be rotated with the part.

    `part_origin_offset_mm` is a world-frame offset: the asset's own figure turned by the
    yaw the part is configured at. That is only the right vector while the part is at
    that yaw. Added to a prim pose unrotated - which is what `part_pose` did - a part
    rolled 90 degrees has its centre reported 151 mm from where it is, because the
    offset is 106.8 mm long and turning it a quarter turn moves its tip by 1.41 times
    that.

    It read as the part sliding in the cup: 149 mm over a 90 degree presentation roll,
    smooth, monotonic, and identical at a quarter of the joint speed. A real slip is not
    reproducible to a millimetre at two speeds, and the part's own quaternion tracked the
    commanded roll to 0.1 degrees the whole way. Nothing was moving but the arithmetic.
    """
    world = part_origin_offset_mm()
    yaw = configured_part_yaw_deg()
    turns = yaw / 90.0
    if abs(turns - round(turns)) > 1e-6:
        raise ValueError(
            f"the part is configured at {yaw} degrees of yaw, so turning its "
            f"origin offset {world} back into the part's own frame is not an axis swap")
    x, y, z = world
    # undo a yaw about z, which at a multiple of 90 degrees is exact
    for _ in range(int(round(turns)) % 4):
        x, y = y, -x
    return (x, y, z)


def configured_part_yaw_deg() -> float:
    """The yaw the fragment spawns the part at."""
    fragment = json.loads(FRAGMENT.read_text())
    world = next(c for c in fragment["components"]
                 if c["model"].endswith(":world"))
    part = next(p for p in world["attributes"]["props"] if p["name"] == "part")
    return float(part.get("yaw_deg", 0.0))


@lru_cache(maxsize=1)
def asset_facts() -> Dict[str, Any]:
    """What was measured from the part asset in a running scene.

    Written by `probes/derive_mailbox.py`, checked against a live cell by
    `tests/test_mailbox_asset.py`. Read here rather than measured, because the layout has
    to derive stations without a machine - but generated rather than authored, so nobody
    keeps two numbers in step by hand.
    """
    if not DERIVED.exists():
        raise FileNotFoundError(
            f"{DERIVED.name} is missing: the part is a usd asset whose size is measured, "
            f"so run probes/derive_mailbox.py against a running cell first")
    return json.loads(DERIVED.read_text())


@dataclass(frozen=True)
class Box:
    centre_mm: Vec3
    dims_mm: Vec3

    @property
    def top_mm(self) -> float:
        return self.centre_mm[2] + self.dims_mm[2] / 2.0


def _local_dims(world_dims_mm: Vec3, yaw_deg: float) -> Vec3:
    """A world-aligned extent, turned back into the part's own, by the yaw it was
    measured at.

    Arithmetic, not a measurement. At a multiple of 90 degrees this is an exact axis
    swap and the only honest way to state a part's size; at any other yaw a world
    extent is strictly larger than the part and cannot be undone, so that is refused
    rather than quietly approximated.
    """
    turns = yaw_deg / 90.0
    if abs(turns - round(turns)) > 1e-6:
        raise ValueError(
            f"the part is configured at {yaw_deg} degrees of yaw, which is not a "
            f"multiple of 90, so its measured world extent {world_dims_mm} is larger "
            f"than the part itself and cannot be turned back into the part's own size. "
            f"Measure the asset's local bounds instead.")
    x, y, z = (float(v) for v in world_dims_mm)
    return (y, x, z) if int(round(turns)) % 2 else (x, y, z)


@dataclass(frozen=True)
class Cell:
    """The cell as the fragment describes it, in millimetres."""

    arm_bases_mm: Dict[str, Vec3]
    part: Box
    belt: Box
    trays: Dict[str, Box]
    fixture: Box
    camera_mm: Vec3
    camera_target_mm: Vec3
    # Each arm's home, as a joint configuration. Home is a taught pose, not a place to
    # be planned to: asked for it as a cartesian goal the planner reached it through
    # whatever configuration it liked, so "home" was a different shape on different
    # runs and the move back from it unwound hundreds of degrees.
    # The part's extent in its OWN frame, which is not `part.dims_mm`.
    #
    # `part.dims_mm` is a world-aligned extent, measured with the part spawned at its
    # configured yaw, so it describes the part's bounding box rather than the part. The
    # two differ by that yaw and, at a multiple of 90 degrees, by an exact axis swap.
    #
    # It has to be a constant, derived once from the declared yaw, rather than worked
    # out from wherever the part happens to be. Re-deriving it from a live pose was the
    # bug: a part hanging at the 16 degree shoulder tilt gave an axis-aligned extent
    # 55 and 42 mm larger than the part, the planner was handed that as the part's own
    # size, and it then refused all five of arm-a's presentations with the carried box
    # 99.8% intersecting the belt deck. The part had not changed size; only its tilt had.
    part_local_dims_mm: Vec3 = (0.0, 0.0, 0.0)
    # What the cell is expected to achieve on a line, as the fragment states it.
    cycle: Dict[str, float] = field(default_factory=dict)
    home_joints_deg: Dict[str, List[float]] = field(default_factory=dict)
    # The gripper as the fragment describes it, so nothing here re-states it.
    tool_mm: float = TOOL_MM
    cup_radius_mm: float = CUP_RADIUS_MM
    grip_drop_mm: float = GRIP_DROP_MM
    # Where a suction cup can seal on the part's side, when that is not simply "anywhere":
    # the wall's own standoff and a window along the part clear of whatever stands proud
    # of it. None for a plain box. Measured, see probes/derive_mailbox.py.
    grip_patch: Optional[Dict[str, Any]] = None
    # Every face a cup could take the part by, measured: standoff, how much clear wall and
    # where, and what stands proud of it. The cell picks by the numbers rather than by
    # convention - at full scale this asset's sides need an 86 mm standoff and its ends 229,
    # and a cup on an end carries the whole 500 mm length cantilevered off its face.
    grip_faces: Optional[Dict[str, Any]] = None
    # Where the infeed's break beam is, if it has one. The pick point follows from it.
    beam_y_mm: Optional[float] = None
    # The highest thing standing proud of the infeed deck between the arm and the part.
    # The part has to be lifted clear of this before it is moved sideways at all.
    guide_top_mm: Optional[float] = None
    # The door hook on arm-a's tool plate, if it has one: how far it reaches down the
    # cup's axis, how far its finger turns back across that axis, how thick it is, and
    # how far to the side of the cup it is carried.
    hook: Optional[Dict[str, float]] = None

    @classmethod
    def from_fragment(cls, path: Path = FRAGMENT) -> "Cell":
        config = json.loads(Path(path).read_text())
        components = config["components"]
        world = next(c for c in components if c["model"].endswith(":world"))
        props = {p["name"]: p for p in world["attributes"]["props"]}

        def box(name: str) -> Box:
            prop = props[name]
            centre = tuple(float(v) * 1000.0 for v in prop["position"])
            # A referenced asset's size is its geometry's, not a number in the config: a
            # cube states `size` and `scale` and that is the whole story, while a usd prop
            # states a path and a scale and the extent has to be measured. It is measured
            # once by probes/derive_mailbox.py and read back here, so the layout still
            # derives every station from one stated size rather than from a guess.
            if str(prop.get("type", "cube")) == "usd":
                # `position` places the prim's ORIGIN, which is not the middle of the
                # geometry: this asset's origin sits at the bottom of the shell, 39.1 mm
                # under its centre. Everything the cell does - where the cup lands, where
                # the part rests, what the planner is told to avoid - is about the
                # geometry, so the measured offset is added here once rather than being
                # carried around by every caller.
                bounds = prop.get("bounds_m")
                if bounds:
                    # An asset that declares its extent is taken at its word, whatever
                    # asset it is. This used to read the mailbox's measured facts for
                    # ANY usd prop, which was fine while the part was the only one and
                    # silently wrong the moment the belt became a real conveyor.
                    #
                    # A prop declared as several boxes is several obstacles, but as a
                    # station it is one surface: the deck. That is the box a part rests
                    # on and the one every station height is derived from, so it wins
                    # over the union - which would put the "top" at the post tops.
                    boxes = bounds if isinstance(bounds, list) else [bounds]
                    named = [b for b in boxes if str(b.get("name", "")) == "deck"]
                    use = named or boxes
                    low = [min(float(b["min"][i]) for b in use) for i in range(3)]
                    high = [max(float(b["max"][i]) for b in use) for i in range(3)]
                    return Box(
                        centre_mm=tuple(centre[i] + (low[i] + high[i]) / 2.0 * 1000.0
                                        for i in range(3)),
                        dims_mm=tuple((high[i] - low[i]) * 1000.0 for i in range(3)),
                    )
                facts = asset_facts()
                offset = facts["origin_to_centre_mm"]
                return Box(
                    centre_mm=tuple(centre[i] + offset[i] for i in range(3)),
                    dims_mm=tuple(facts["dims_mm"]),
                )
            size = float(prop.get("size", 1.0))
            scale = [float(v) for v in (prop.get("scale") or (1.0, 1.0, 1.0))]
            return Box(
                centre_mm=centre,
                dims_mm=tuple(size * scale[i] * 1000.0 for i in range(3)),
            )

        bases = {}
        for component in components:
            if component.get("type") == "arm":
                translation = component["frame"]["translation"]
                bases[component["name"]] = (
                    float(translation["x"]), float(translation["y"]),
                    float(translation["z"]),
                )
        homes = {c["name"]: [float(v) for v in c["attributes"]["home_joints_deg"]]
                 for c in components
                 if c.get("type") == "arm" and c["attributes"].get("home_joints_deg")}

        # The hook is tooling on the ARM, not on the gripper: it carries a collider, so
        # it has to exist before the articulation is built.
        arm_a = next((c for c in components if c["name"] == "arm-a"), None)
        hook = None
        if arm_a is not None and arm_a["attributes"].get("hook"):
            hook = {k: float(v) for k, v in arm_a["attributes"]["hook"].items()
                    if isinstance(v, (int, float))}

        gripper = next((c for c in components if c.get("type") == "gripper"), None)
        tool_mm, cup_mm = TOOL_MM, CUP_RADIUS_MM
        if gripper is not None:
            attributes = gripper["attributes"]
            offset = attributes.get("offset")
            if offset:
                tool_mm = max(abs(float(v)) for v in offset) * 1000.0
            cup_mm = float(attributes.get("geometry", {}).get(
                "radius_mm", CUP_RADIUS_MM))


        # Where the cup sits on a flat side: the middle of the panel it lands on, which
        # is what keeps it off the curved crown. The panel is a child of the part.
        part_prop = props["part"]
        patch = None
        faces_by_side = None
        if str(part_prop.get("type", "cube")) == "usd":
            facts = asset_facts()
            drop_mm = float(facts["grip_drop_mm"])
            patch = facts.get("grip_patch")
            faces_by_side = facts.get("grip_faces")
        else:
            panel = next((c for c in part_prop.get("children", [])
                          if c["name"] == "side_l"), None)
            drop_mm = (abs(float(panel["position"][2])) * 1000.0
                       if panel else GRIP_DROP_MM)

        camera = next(c for c in components if c["name"] == "inspect-cam")
        translation = camera["frame"]["translation"]
        return cls(
            home_joints_deg=homes,
            tool_mm=tool_mm,
            cup_radius_mm=cup_mm,
            grip_drop_mm=drop_mm,
            grip_patch=patch,
            grip_faces=faces_by_side,
            hook=hook,
            arm_bases_mm=bases,
            cycle={k: float(v) for k, v in (world["attributes"].get("cycle") or {}).items()
                   if isinstance(v, (int, float))},
            part=box("part"),
            part_local_dims_mm=_local_dims(
                box("part").dims_mm, float(part_prop.get("yaw_deg", 0.0))),
            belt=box("belt"),
            beam_y_mm=(lambda c: None if not c else float(
                (c.get("beam") or {}).get("y_mm")))(props["belt"].get("conveyor")),
            guide_top_mm=max(
                [float(pr["position"][2]) * 1000.0
                 + float(pr.get("size", 1.0)) * float(pr["scale"][2]) * 1000.0 / 2.0
                 for pr in props.values() if pr["name"].startswith("guide_")] or [0.0])
            or None,
            trays={"good": box("good_conveyor"), "bad": box("bad_conveyor")},
            fixture=box("fixture"),
            camera_mm=(float(translation["x"]), float(translation["y"]),
                       float(translation["z"])),
            camera_target_mm=tuple(
                float(v) * 1000.0 for v in camera["attributes"]["target"]),
        )

    # ---- grips ---------------------------------------------------------------

    def side_grip(self, centre_mm: Vec3, side: str) -> Pose:
        """Flange pose that puts the cup on a part of the side a cup can actually seal on.

        `side` is the face the cup covers, named by the world axis its outward normal
        points along. The arm stands off along that normal and the cup points back at the
        part.

        Half the part's width is the WRONG standoff for a part with anything on its side,
        and the mailbox has a flag. Its outer bound in y is the flag, 8.5 mm proud of the
        wall and, at this cell's scale, a 5 mm tall arm - so a cup aimed at the bound
        stops short of the wall everywhere the flag is not, and cannot seal on the flag
        where it is. Measured: the cup landed square on the flag and the gripper reported
        `Open` on every attempt, and lengthening the raycast from 20 mm to 50 changed
        nothing, because the problem was never distance.

        So where a patch has been measured, stand off from the WALL and slide the cup
        along the part to an x the flag and rails do not reach. The cube part needs none
        of this, which is why it never came up: a box's side is all seal.
        """
        # Stand off far enough for the cup's OWN ray to start outside the part.
        #
        # The gripper finds its payload by raycasting from a clearance offset - 5 mm past
        # the cup's attachment point - out to its max grip distance. Putting the attachment
        # point exactly on the surface, which "half the part plus the tool" does by
        # construction, starts that ray 5 mm inside the part, where there is nothing ahead
        # of it to hit. Measured with a scene query: from the cup point the part is at
        # 0.0 mm, so the cup was flush and the gripper reported Open every time on a face
        # that was solid, queryable and exactly where it was meant to be.
        #
        # So approach to a gap rather than to contact. The cup closes the gap itself; that
        # is what its stroke is for.
        patch = (self.grip_faces or {}).get(side) or self.grip_patch
        drop = self.grip_drop_mm
        along = 0.0
        if patch is not None:
            reach = patch.get("standoff_mm", patch.get("wall_y_mm"))
            along = patch.get("offset_mm", patch.get("x_offset_mm", 0.0))
            low, high = patch.get("height_mm", patch.get("wall_z_mm", (0.0, 0.0)))
            # Land mid-wall in height, within the wall's own extent.
            drop = -(low + high) / 2.0
        else:
            reach = self.part.dims_mm[1 if side in ("+y", "-y") else 0] / 2.0
        stand = reach + self.tool_mm + CUP_APPROACH_GAP_MM
        # `along` slides the cup across the face it lands on, so which world axis that is
        # depends on the face: a grip on +-y slides in x, a grip on +-x slides in y.
        poses = {
            "+y": ((centre_mm[0] + along, centre_mm[1] + stand, centre_mm[2] - drop),
                   TOOL_MINUS_Y),
            "-y": ((centre_mm[0] + along, centre_mm[1] - stand, centre_mm[2] - drop),
                   TOOL_PLUS_Y),
            "+x": ((centre_mm[0] + stand, centre_mm[1] + along, centre_mm[2] - drop),
                   TOOL_MINUS_X),
            "-x": ((centre_mm[0] - stand, centre_mm[1] + along, centre_mm[2] - drop),
                   TOOL_PLUS_X),
        }
        if side not in poses:
            raise ValueError(
                f"unknown grip side {side!r}; the flat faces are {sorted(poses)}")
        return poses[side]

    # Which face the cell takes the part by, chosen from what was measured rather than by
    # convention. At full scale this asset's sides want an 86 mm standoff with 481 mm of
    # clear wall and nothing proud of them, while its ends want 229-252 mm and hang the
    # whole 500 mm length cantilevered off the cup - which is neither what a vacuum cup is
    # good at nor kind to the part. The -x side is also the one facing the arms, and the
    # one the flag is NOT on.
    GRIP_SIDE = "-x"

    # How far past the beam the part coasts before it comes to rest. Measured over three
    # runs at 300 mm/s (probes/beam_check.py): +5.3, +5.3, +7.5 mm.
    BEAM_OVERSHOOT_MM = 6.0

    @property
    def pick_mm(self) -> Vec3:
        """Where the part's centre ends up, which is NOT where it spawns.

        The part spawns upstream on open deck and is carried to the pick point by the
        conveyor, so `part.centre_mm` - the spawn - is 1.2 m from where the arm has to
        take it. Everything about the pick derives from the BEAM: it is the beam that
        defines the pick point, and the beam is placed where the arm can reach.

        Nominal, for a cell that has to be reasoned about without a simulator. The round
        itself grips from the part's measured pose, so this is what the layout is checked
        against rather than what the arm is commanded to.
        """
        beam = self.beam_y_mm
        if beam is None:
            return self.part.centre_mm
        leading = beam + self.BEAM_OVERSHOOT_MM
        return (self.part.centre_mm[0],
                leading - self.part.dims_mm[1] / 2.0,
                self.part.centre_mm[2])

    def pick(self, side: Optional[str] = None) -> Pose:
        return self.side_grip(self.pick_mm, side or self.GRIP_SIDE)

    # Let go a few millimetres up rather than pressing the part onto the surface.
    # Driving a rigid part onto a rigid surface is contact the arm cannot push through,
    # so it stalls short of its commanded pose and the move fails - with the part
    # already where it needs to be.
    # Enough that the carried part's box clears the surface's while the planner is still
    # being asked to avoid that surface - 8 mm was inside the margin it plans with, which
    # is why the descent used to be planned blind instead.
    RELEASE_GAP_MM = 20.0

    def resting_on(self, surface: Box, gap_mm: float = 0.0) -> Vec3:
        """Where the part's centre sits when it rests on a surface."""
        return (surface.centre_mm[0], surface.centre_mm[1],
                surface.top_mm + self.part.dims_mm[2] / 2.0 + gap_mm)

    def fixture_grip(self, side: str, gap_mm: float = 0.0) -> Pose:
        """Flange pose for the part seated in the fixture, held by `side`."""
        return self.side_grip(self.resting_on(self.fixture, gap_mm), side)

    # How far along an outfeed conveyor the part is set down. A conveyor is metres long
    # and its centre is nowhere near the arm, so a place derived from the prop's centre
    # would be a place the arm cannot reach. This is the near end, chosen so arm-b's
    # flange - which always stands off 175 mm on +y - lands close to arm-b's own
    # centreline and the arm does not have to swing its base to either conveyor.
    # How far out along an outfeed conveyor the part is set down.
    #
    # The outfeeds run along x, away from the cell, and that is not cosmetic. Arm-b's
    # wrist trails about 270 mm behind the flange in +y and hangs some 100 mm below it,
    # so a conveyor running along y is underneath the wrist for the whole of the place:
    # measured, `wrist_1_link` came to rest 57 mm off the deck and the descent stalled
    # every time, at three different conveyor positions and two different heights.
    # Turned to run along x, the deck is only 180 mm wide in y and the wrist trails
    # clear past it.
    #
    # The value itself is far enough out that arm-a cannot reach either conveyor, which
    # is what makes the handoff something the layout forces rather than something the
    # round merely chooses.
    # Far enough along the outfeed that the PART clears the fixture, which straddles the
    # lane - and the part reaches a long way back from the flange here.
    #
    # At 900 the placed part sat inside the fixture: it spans z 350..576 lying on a 330 mm
    # deck, the fixture occupies 520..600 over x 655..905, and they overlapped by 56 mm in
    # z and 91 in x. The planner refused with a 99.97% violation against it.
    #
    # Nothing else could give, and each was checked rather than assumed. The fixture cannot
    # move in y - the part seats on it along y, so its 495 mm length reaches 247 mm either
    # side, and any centre below y 0 puts that under the infeed deck. It cannot move up -
    # arm-a's rolls stand the part on end down to z 658 and clear the fixture's top by
    # 58 mm; measured at a 700 mm top the lane gains 63 mm and the rolls lose 42 into the
    # fixture. And the place cannot go upstream of the fixture, attractive though 463 mm of
    # reach looked: the conveyor would carry the part straight into it.
    #
    # How far downstream is set by the part, not the flange. arm-b holds the +y END, so at
    # the place the flange sits 377 mm UPSTREAM of the part's centre and the part's near
    # end reaches 247 mm back beyond that. 1200 puts that near end 53 mm clear of the
    # fixture with the two trays 866 and 877 mm out, inside `reach_budget()` by 38.
    PLACE_PART_X_MM = 1200.0

    def outfeed_point_mm(self, tray: str, gap_mm: float = 0.0) -> Vec3:
        """Where the part's centre sits when set down on an outfeed conveyor.

        Centred ACROSS the belt, which is where a part rides and what keeps arm-b's
        wrist - which trails 270 mm behind the flange - clear of the deck. The belts are
        a narrow build for that reason: a full 900 mm deck is wider than this part needs
        and puts its own centreline half a metre further from the arm.
        """
        deck = self.trays[tray]
        return (self.PLACE_PART_X_MM, deck.centre_mm[1],
                deck.top_mm + self.part.dims_mm[2] / 2.0 + gap_mm)

    def place(self, tray: str, side: str, gap_mm: float = 0.0) -> Pose:
        """What a side grip WOULD give on an outfeed conveyor.

        Not what the round does: the part is laid down turned a quarter turn, so the
        flange ends up behind it rather than beside it, and `round.nominal_place_flange`
        is the pose that actually gets used. Kept because several constraints are still
        naturally stated about a side grip - notably which faces the belt leaves
        approachable.
        """
        return self.side_grip(self.outfeed_point_mm(tray, gap_mm), side)

    def stations(self) -> Dict[str, Tuple[str, Vec3]]:
        """Every flange position the round visits, and which arm visits it.

        Derived, and here because `probes/stations.json` used to hold this as stored
        numbers. By the time anyone looked it encoded a tool-down grip the cell had
        stopped using, an 80 mm cube the part no longer was, and a scene with none of
        the props now in it - one station had started failing IK and nothing noticed.
        A stored pose cannot notice that the cell moved.
        """
        # Imported here rather than at the top: `round` imports this module, and the
        # place is the one station whose flange this module cannot derive on its own.
        # It is not a `side_grip` pose - the part is laid down turned a quarter turn, so
        # the flange ends up behind it - and the turn is chosen from the hold that was
        # measured at the time.
        from .round import nominal_handoff_flanges, nominal_place_flange

        give, take = nominal_handoff_flanges(self)
        out: Dict[str, Tuple[str, Vec3]] = {
            # The side the cell actually grips, not the `+y` end it used to. Measured
            # against the part position instead, the pick reads 867 mm from arm-a and
            # the flange the arm really goes to reads 666 - a side grip stands the arm
            # OFF the part, so the part is always further away than the arm reaches.
            "pick": ("arm-a", self.pick(self.GRIP_SIDE)[0]),
            # The poses the HANDOFF makes, which are not `fixture_grip`. The part is
            # set down yawed half a turn, so arm-a's flange ends up on the far side of
            # it, and arm-b takes the +y end. Reporting the unyawed grip here is what
            # sited the fixture 440 mm beyond arm-a's reach - every number looked fine.
            "handoff_give": ("arm-a", give),
            "handoff_take": ("arm-b", take),
        }
        for tray in self.trays:
            out[f"place_{tray}"] = ("arm-b", nominal_place_flange(self, tray)[0])
        for arm in self.arm_bases_mm:
            for face, (position, _) in self.present_poses(arm):
                out[f"present_{arm}_{face}"] = (arm, position)
        return out

    # ---- constraints ---------------------------------------------------------

    def reach_mm(self, arm: str, flange_mm: Sequence[float]) -> float:
        return math.dist(tuple(flange_mm), self.arm_bases_mm[arm])

    def grip_clears_belt(self, side: str) -> bool:
        """Is there room to stand off on this side without being over the belt?

        The belt runs away in -y from the part, so approaching that side puts the arm
        above it with almost no clearance and the planner refuses. This is a property of
        the layout, not of the gripper, and it is what makes the +y grip the only one
        the pick can use.
        """
        (_, y, _), _ = self.pick(side)
        belt_near_edge = self.belt.centre_mm[1] + self.belt.dims_mm[1] / 2.0
        return y > belt_near_edge


    # Where the PART sits while each arm shows it to the camera.
    #
    # Both arms hold the part's world +y face - arm-a because the belt leaves no other
    # side to approach at the pick, arm-b because the part is set down yawed half a turn
    # - so the flange always stands off 175 mm on +y and the two arms' presentation
    # geometry is identical. What differs is only which base it is reached from, and
    # that is why these are two points rather than one: putting each nearer its own arm
    # is the only thing that keeps either arm from swinging its base across its body.
    #
    # How far apart they may be is set by the lens, so these, the inspection distance
    # and the fov were chosen together by a search over the geometry. Stated as PART
    # positions because that is what the camera has to frame; the flange follows.
    # Moved out 15 mm each when the part became the mailbox asset, because these depend
    # on the STANDOFF and the standoff depends on the part. The cube was gripped across
    # its 110 mm width, 175 mm flange to centre; the mailbox is taken by its back cap,
    # 211 mm. That extra 36 mm pushes the flange the same distance inboard, and at -240
    # arm-a's roll presentation came out 30.3 degrees off its forward direction against a
    # 30.0 comfort bound - the bound doing exactly its job. At -255 it is 29.2.
    # arm-a shows its faces from further towards its own side than arm-b does. The
    # flange for the +x presentation sits 367 mm +y of the part, so at -255 that pose
    # swung arm-a 39 degrees off its forward direction - the arm reaching across itself,
    # which is what the station budget exists to prevent. At -410 it is 29.
    # Both arms show the part at the SAME place: on the camera's own axis, in the
    # middle of the cell, with each arm standing off on its own side of it.
    #
    # Each arm used to have its own inspect y - arm-a at -410, arm-b at 175 - which put
    # every presentation out over that arm's own side of the cell, and both sides have
    # hardware on them. Measured against the live obstacle list, with a 495 mm part hung
    # vertically by a side cup: arm-a's presentations reached 111.8 mm INTO belt.deck and
    # arm-b's 2.5 mm into fixture_wall_y. On the camera axis the worst clearance in the
    # cell becomes positive, and both arms' pans improve - arm-a from 29.4 degrees to
    # 5.3, arm-b to -25.9.
    #
    # The middle of the cell is also the only place a part this size can go. The usable
    # band is narrow and is set by the two things either side of it: the infeed deck ends
    # at y -250 and the fixture's +y wall starts at 252, so the part's own 172 mm width
    # has to sit between them. That leaves y -164 to 166, and the camera axis is -117.5.
    #
    # It costs the separate free-face position, which is no loss: that pose existed to
    # bring the part nearer the arm showing it, and on the camera axis both arms already
    # reach it comfortably (663 and 722 mm).

    # The free flat side keeps its own position, on the arm's own side, and can.
    #
    # The split is physical, not a preference. The four rolls turn the part about the
    # tool axis, and two of them stand its 495 mm length VERTICAL - reaching 247 mm below
    # the hold, which over an infeed deck that tops out at 769 mm is 111 mm inside it. So
    # the rolls have to happen in the middle of the cell. The free face is a single
    # orientation with the length HORIZONTAL and the part only 227 mm tall, which clears
    # that same deck by 23 mm - so it can be shown over the belt, on the arm's own side,
    # where the arm reaches out to it instead of folding under itself.
    #
    # Nearer the arm than the rolls on purpose: at -340 against rolls at -410 it was
    # FURTHER from arm-a and the arm folded under itself again.
    # Just off the camera axis, on each arm's own side, and 30 mm is not arbitrary.
    #
    # Getting to this pose turns the part 90 degrees - the tool has to swing from along y
    # to along x - and what has to clear the cell is the part THROUGHOUT that turn, not at
    # its ends. With the part's length vertical the turn is a pure rotation about z and the
    # part only presents its 172 x 227 cross-section to the swing, but the diagonal of that
    # cross-section is 282 mm, so at 45 degrees through the turn it reaches 141 mm either
    # side of the station. Measured by walking the turn in fortieths against the live
    # obstacle list: at the camera axis that overhangs the infeed deck by 10 mm, at y -90
    # it clears by 5, at -60 by 19 and at -30 by 30. Pan goes the other way - 23.9 degrees
    # at the camera axis, 30.5 at -30 - so -30 is where the two meet.
    #
    # The part's length cannot lie along the direction the arm reaches for this pose - the
    # cup is on a side panel, so the length is always perpendicular to the tool - which
    # leaves it along y or standing on end. Laid along y over the infeed it clears that
    # deck by 14 mm, and the planner refuses that; stood on end it is only 172 mm wide and
    # clears the deck's near edge at y -250 by 46 mm. So this pose wants the same thing
    # the rolls want and for the same reason: the part stood on end, where the belt is
    # not. For arm-a that is the camera axis, 23.8 degrees off its forward direction and
    # 706 mm out. For arm-b, y 260 is already 510 mm clear of the infeed and nothing else
    # is under it, so it keeps the shorter reach.
    PRESENT_FREE_Y_MM = {"arm-a": -30.0, "arm-b": 30.0}
    # And further OUT than the rolls, because this is the one pose where the ARM has to
    # stand between the part and the camera's side of the cell.
    #
    # The camera looks back along -x, so the free face can only be shown with the arm on
    # the part's -x side - there is no choice about that. At the rolls' x of 600 that puts
    # the flange at 394, right alongside the infeed's near wall (x 224..348), and the
    # planner refused all four rolls of the pose with a 100% violation against it. Moved
    # out to 800 the flange sits at 594, clear of the wall by 246 mm, and both arms reach
    # it nearly straight ahead: arm-a pan -1.0 degrees, arm-b -11.4.
    #
    # It costs lens - the camera has to frame a part 200 mm nearer than the rolls - which
    # is the same trade this pose has always made for its own position.
    # Stated as where the FLANGE goes, not where the part goes, because the two arms hold
    # the part by faces with very different standoffs - 206 mm and 377 - and this is the
    # one pose where the arm stands between the part and the camera. Fixing the part's x
    # for both put arm-b's flange 171 mm nearer its own base and 39.2 degrees off its
    # forward direction; fixing the FLANGE's x puts both arms in the same comfortable
    # place, 30.5 degrees and 743 mm, and lets the part sit wherever each grip implies.
    FREE_FLANGE_X_MM = 594.0

    def present_side(self, arm: str) -> str:
        """Which side of the part `arm` stands off on: its own.

        Derived from where the arm's base is relative to the part, not configured. An arm
        reaching past the part to grip its far face is the thing this prevents, and it is
        exactly what a hard-coded `+y` did to arm-a.
        """
        return "+y" if self.arm_bases_mm[arm][1] > self.camera_target_mm[1] else "-y"

    def free_point_mm(self, arm: str) -> Vec3:
        """Where the part sits while `arm` shows the camera its free flat side."""
        return (self.FREE_FLANGE_X_MM + self.grip_standoff_mm(self.hold_face(arm)),
                self.PRESENT_FREE_Y_MM[arm], self.camera_target_mm[2])

    # How much of the part reaches along y at the rolls, which depends on the face the arm
    # is holding, so the two arms need different inspect points.
    #
    # arm-a holds a side panel, so the part's long axis is perpendicular to the tool and
    # only its 172 mm width lies along y. arm-b holds an END, so the tool axis IS the long
    # axis and 495 mm lies along y - the part reaches 247 mm either side of the station
    # instead of 86.
    #
    # On the camera axis that put arm-b's part 370 mm into the infeed's y range, and with
    # the rolls that stand it 227 mm tall its underside came to 782 mm against a deck
    # topping out at 769. Two of the four rolls cleared by 40 mm and two by 13, and the
    # planner refused exactly those two. Height cannot fix it: 30 mm more would put the
    # station 296 mm above arm-b's base, past the 280 mm bound. So arm-b inspects far
    # enough along +y that even its long axis clears the deck.
    BELT_CLEARANCE_MM = 30.0

    def inspect_point_mm(self, arm: str = "arm-a") -> Vec3:
        """Where the part sits to be inspected: on the camera's axis where that clears.

        The camera frames a band, not a line - 70 degrees at this range is over a metre
        wide - so moving a station off the axis costs framing, not visibility, and is the
        cheaper of the two things that can give.
        """
        target = self.camera_target_mm
        # Half the part's reach along y at the rolls: the long axis when the arm holds an
        # end, the width when it holds a side.
        half = (self.part_local_dims_mm[0] if self.hold_face(arm) in ("+y", "-y")
                else self.part_local_dims_mm[1]) / 2.0
        near_edge = self.belt.centre_mm[1] + self.belt.dims_mm[1] / 2.0
        least = near_edge + half + self.BELT_CLEARANCE_MM
        return (target[0], max(target[1], least), target[2])

    # Which face each arm is holding the part by, which is NOT the same face.
    #
    # arm-a picks the part off the belt by its -x side panel. arm-b takes it off the
    # fixture by its +y END, because the panel arm-a held is the one the second check has
    # to see. The two stand off very differently - 206 mm for the panel, 377 for the end -
    # and every pose that holds the part is built from that number.
    #
    # Assuming arm-a's for both is what put arm-b's presentations 171 mm out: its part
    # landed at y -289, over the infeed, instead of on the camera axis, and the planner
    # refused the rolls with the carried box 100% inside belt.deck. It is the same mistake
    # as standing off by half the part's length, one layer further in - a number true of
    # one grip used for another.
    HOLD_FACE = {"arm-a": GRIP_SIDE, "arm-b": "+y"}

    def hold_face(self, arm: str) -> str:
        return self.HOLD_FACE.get(arm, self.GRIP_SIDE)

    def grip_standoff_mm(self, side: str) -> float:
        """How far the flange sits from the part's centre, gripping this face.

        The one number every pose that holds the part is built from, derived from the
        measured face rather than from the part's bounding box. Measured against the
        live cell: the `-x` side grip reads 206 mm here and the hold measures 209.
        """
        patch = (self.grip_faces or {}).get(side) or self.grip_patch
        if patch is not None:
            reach = float(patch.get("standoff_mm", patch.get("wall_y_mm")))
        else:
            reach = self.part.dims_mm[1 if side in ("+y", "-y") else 0] / 2.0
        return reach + self.tool_mm

    def present_poses(self, arm: str, side: str = "") -> List[Tuple[str, Pose]]:
        """Flange poses that show the camera each face the cup is not covering.

        Rolling about the tool axis shows four faces - the two ends and the crown and
        base - but never the free flat side, which stays edge-on to the camera however
        far it rolls. That one needs the part yawed a quarter turn so it faces the
        camera, and yawed the way that leaves the ARM behind the part rather than
        between it and the lens.
        """
        side = side or self.present_side(arm)
        centre = self.inspect_point_mm(arm)
        # How far the flange stands off from the part's CENTRE, which is a property of
        # the grip the cell makes and not of the part's length.
        #
        # This used to be half the part's long axis plus the tool - 367 mm - which is the
        # standoff of the `+y` END grip the cell abandoned. The cell grips a SIDE, and a
        # side grip stands off 206 mm. So every presentation put the part 161 mm short of
        # the point the camera is aimed at: the inspect point moved and the part did not
        # follow it, which is also why raising the station cleared less of the belt than
        # the arithmetic said it would.
        #
        # Rolling about the tool axis does not change it. The centre sits on that axis,
        # so it is the same standoff at every roll - which is what makes one number right
        # for all four poses.
        standoff = self.grip_standoff_mm(self.hold_face(arm))
        sign = 1.0 if side == "+y" else -1.0
        covered = side
        free = "-y" if side == "+y" else "+y"

        poses: List[Tuple[str, Pose]] = []
        # Rolling about the tool axis: the tool stays along -+y, the part spins.
        rolled = {0.0: "+x", 90.0: "+z", 180.0: "-x", 270.0: "-z"}
        for theta, face in rolled.items():
            position = (centre[0], centre[1] + sign * standoff,
                        centre[2] - self.grip_drop_mm)
            orientation = {"o_x": 0.0, "o_y": -sign, "o_z": 0.0, "theta": theta}
            poses.append((face, (position, orientation)))

        # A quarter turn about z puts the free side towards the camera. The camera looks
        # back along -x, so the arm must stand off on -x for the part to be in front of
        # it rather than the other way round - which is what makes this the awkward one,
        # and why it is shown from its own position rather than where the rolls happen.
        free_centre = self.free_point_mm(arm)
        # Which roll this pose uses is free for framing - it spins the part about the tool
        # axis, and that axis points at the camera, so every roll keeps the free face in
        # frame - and it is not free for clearance. theta 0 stands the part's 495 mm
        # length on end, which is what this pose wants: on end it is only 172 mm wide and
        # fits between the infeed's near edge and the fixture. Laid flat instead it is
        # 495 mm deep in y and overhangs the infeed, where it clears the deck by 14 mm and
        # the planner refuses it.
        poses.append((free, ((free_centre[0] - standoff, free_centre[1],
                              free_centre[2] - self.grip_drop_mm),
                             {"o_x": 1.0, "o_y": 0.0, "o_z": 0.0, "theta": 0.0})))
        assert covered not in [face for face, _ in poses]
        return poses


    # ---- defects -------------------------------------------------------------

    @classmethod
    def defect_faces(cls, path: Path = FRAGMENT) -> Dict[str, str]:
        """Which face of the part each defect mark sits on, in the part's own frame.

        Derived from where the child is, not from a table someone keeps in step. A mark
        is a thin patch on one face, so the axis it is furthest out along names the face
        it is on.
        """
        config = json.loads(Path(path).read_text())
        world = next(c for c in config["components"]
                     if c["model"].endswith(":world"))
        part = next(p for p in world["attributes"]["props"] if p["name"] == "part")
        if str(part.get("type", "cube")) == "usd":
            # The asset's marks are geometry, so which face each sits on was measured from
            # the composed stage rather than read off the config. Same derivation either
            # way - the axis a mark is furthest out along names its face.
            return {f"defect_{name}": face
                    for name, face in asset_facts()["decal_faces"].items()}
        half = [v / 2.0 for v in part["scale"]]
        faces = {}
        for child in part["children"]:
            name = child["name"]
            if not name.startswith("defect_"):
                continue
            position = child["position"]
            axis = max(range(3), key=lambda i: abs(position[i]) / half[i])
            faces[name] = f"{'+' if position[axis] > 0 else '-'}{'xyz'[axis]}"
        return faces


def reach_budget() -> float:
    """The distance a station may sit from its arm's base."""
    return REACH_MM * (1.0 - REACH_MARGIN)
