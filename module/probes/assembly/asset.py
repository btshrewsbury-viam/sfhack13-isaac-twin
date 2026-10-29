"""What the mailbox parts say about themselves.

Read from `assembly_frames.json`, which the usd session generates by reading its own built
stage back - so these numbers are the asset's, not a transcription of them. Every insertion
in the cell is derived from here.

This is the countermeasure that matters. The previous build's worst recurring bug was a
derived scalar true of one grasp being reused for another - a 367 mm standoff from an
abandoned end grip applied to a side grip, a 175 mm hold applied to a 377 mm one - and
every one of them passed the whole offline suite and failed the run. The fix is not to
store the numbers more tidily; it is for there to be exactly one place they come from, and
for that place to be the part itself.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Dict, Sequence, Tuple

Vec3 = Tuple[float, float, float]

# The usd tree: this workstation's, or the cell's asset package on a learner's machine
# (the cell service sets CELL_USD_DIR from its `assets_dir` before importing this).
USD_DIR = Path(os.environ.get("CELL_USD_DIR", "/home/shrews/viam/usd"))
FRAMES = USD_DIR / "mailbox" / "assembly" / "assembly_frames.json"
ASSEMBLY_DIR = FRAMES.parent


@lru_cache(maxsize=1)
def frames() -> Dict:
    if not FRAMES.exists():
        raise FileNotFoundError(
            f"{FRAMES} is missing. The assembly parts are built by the usd session's "
            f"tools/build_all.sh; the cell will not guess their geometry.")
    return json.loads(FRAMES.read_text())


def mm(value) -> float | Vec3:
    """The asset speaks metres; the cell speaks millimetres."""
    if isinstance(value, (list, tuple)):
        return tuple(float(v) * 1000.0 for v in value)
    return float(value) * 1000.0


@dataclass(frozen=True)
class Part:
    """One assembly part, as the asset describes it."""

    name: str
    usd_path: str
    prim: str
    attrs: Dict
    variant_sets: Dict[str, Sequence[str]]

    def at(self, key: str):
        if key not in self.attrs:
            raise KeyError(
                f"{self.name} publishes no {key!r}; it has "
                f"{sorted(k for k in self.attrs if k.startswith('assembly:'))}")
        return self.attrs[key]

    def mm_at(self, key: str):
        return mm(self.at(key))

    @property
    def bounds_mm(self) -> Tuple[Vec3, Vec3]:
        """The COLLIDERS: what contact and grasping see."""
        return (mm(self.at("mailbox:collisionBoundsMin")),
                mm(self.at("mailbox:collisionBoundsMax")))

    @property
    def envelope_mm(self) -> Tuple[Vec3, Vec3]:
        """EVERYTHING, rendered or colliding: what a fixture and a planner must clear.

        The two are genuinely different boxes and the difference is not small. The shell's
        hinge knuckles carry no colliders - by design, so the pin can pass through them -
        and they stand 5.7 mm in front of the collision bound. A fixture built to the
        collider box is a fixture the part does not fit in, which is what happened here:
        lips sized to +-83.7 sat inside a part that measures wider, and it rode on them
        instead of between them.
        """
        if "mailbox:envelopeMin" not in self.attrs:
            return self.bounds_mm
        return (mm(self.at("mailbox:envelopeMin")), mm(self.at("mailbox:envelopeMax")))

    @property
    def dims_mm(self) -> Vec3:
        low, high = self.bounds_mm
        return tuple(high[i] - low[i] for i in range(3))

    @property
    def centre_offset_mm(self) -> Vec3:
        """Where the part's geometric centre sits relative to its ORIGIN.

        Not zero, and that has cost this project a day before: a pose added to a prim's
        origin as though it were the centre reported a rolled part 151 mm from where it
        was. Rotate this with the part's orientation; never add it in the world frame.
        """
        low, high = self.bounds_mm
        return tuple((low[i] + high[i]) / 2.0 for i in range(3))

    @property
    def mass_kg(self) -> float:
        return float(self.attrs.get("physics:mass", 0.0))


def part(name: str) -> Part:
    blob = frames()["parts"][name]
    return Part(name=name, usd_path=str(ASSEMBLY_DIR / blob["usd"]),
                prim=blob["prim"], attrs=blob["attributes"],
                variant_sets=blob.get("variantSets", {}))


@dataclass(frozen=True)
class Insertion:
    """A part going into the shell: where it ends up, how it gets there, what holds it.

    Derived, not written down. `pre` is the published seated position pulled back along
    the published insert axis by the published depth, which is the usd session's own
    statement of how to do it - "pre-insertion pose = seatedPosition - insertAxis *
    insertDepth; then move along insertAxis".

    `insertDepth` now carries a 3 mm approach clearance on top of the engagement, so the
    pre-insertion pose stops short of first contact instead of exactly at the hole's mouth.
    That matters: the first version of the asset put the peg's tip on the lip, which spent
    the whole 1.00 mm radial budget before the peg entered anything.

    `joint` is the asset's answer to the question the physics run asked. Geometry alone
    never held the flag - 2.7 mm of wall with 1.00 mm of slack lets the peg tilt 36 degrees
    against a 140 mm lever - and the asset's intent is that seating CREATES a joint: a snap
    or rivet for the flag, a press fit for the pin, a hinge for the door. The cell has to
    make that joint; it is not a property the parts have on their own.
    """

    part: str
    seated_mm: Vec3          # in the SHELL's frame
    axis: Vec3               # unit, the direction of travel
    depth_mm: float          # insertDepth: approach clearance PLUS engagement
    engage_mm: float         # first contact to seated
    approach_mm: float       # how far short of first contact the pre-insertion pose is
    clearance_mm: float      # radial slack in the hole, per side
    joint: str               # what holds it once seated: "fixed" | "revolute"
    joint_anchor_mm: Vec3    # where that joint is, in the SHELL's frame
    joint_axis_mm: Vec3 = (0.0, 0.0, 0.0)
    joint_limits_deg: Vec3 = (0.0, 0.0)
    joint_drive: Dict = None            # stiffness/damping for a sprung hinge, if any

    @property
    def pre_mm(self) -> Vec3:
        return tuple(self.seated_mm[i] - self.axis[i] * self.depth_mm for i in range(3))

    @property
    def contact_mm(self) -> Vec3:
        """Where the part first touches - `approach_mm` past the pre-insertion pose."""
        return tuple(self.seated_mm[i] - self.axis[i] * self.engage_mm for i in range(3))


def insertions() -> Dict[str, Insertion]:
    shell = part("Shell")
    out = {}
    for name, slack in (("Door", None), ("Flag", None), ("Pin", None)):
        p = part(name)
        axis = tuple(float(v) for v in p.at("assembly:insertAxis"))
        out[name] = Insertion(
            part=name, seated_mm=p.mm_at("assembly:seatedPosition"), axis=axis,
            depth_mm=p.mm_at("assembly:insertDepth"),
            engage_mm=p.mm_at("assembly:engageDepth"),
            approach_mm=p.mm_at("assembly:approachClearance"),
            clearance_mm=_slack(name, shell, p),
            joint=str(p.at("assembly:seatJoint")),
            joint_anchor_mm=p.mm_at("assembly:seatJointAnchor"),
            joint_axis_mm=(tuple(float(v) for v in p.at("assembly:seatJointAxis"))
                           if "assembly:seatJointAxis" in p.attrs else (0.0, 0.0, 0.0)),
            joint_limits_deg=(tuple(float(v) for v in p.at("assembly:seatJointLimits"))
                              if "assembly:seatJointLimits" in p.attrs else (0.0, 0.0)),
            joint_drive=_drive(name, p))
    return out


# (The door's spring was hard-coded here while the asset did not publish it. It does now -
# `assembly:seatJointDrive*` - and the usd session's validator fails their build if the
# authored drive ever disagrees with the published values, so this reads it like everything
# else and a change on their side reaches the cell without a message.)


def _drive(name: str, p: Part):
    """The drive a seated joint needs, where the asset publishes one.

    A hinge without its spring is a different mechanism, not a simplified one. This door's
    centre of mass sits 112.7 mm straight above its hinge axis with a 1.5 mm horizontal
    offset, so a plain revolute makes it an inverted pendulum: measured, it fell to its 100
    degree limit and hung there, 305 mm from seated, with the radius about the hinge exactly
    right in both poses - the hinge was correct and only the restoring torque was missing.
    """
    # An angular drive's stiffness is honoured per DEGREE on an articulation joint and per
    # RADIAN on a plain one, and this cell makes plain ones.
    #
    # Not a blanket physx fact, which is what I first concluded and wrote here. The usd
    # session's own door - an ARTICULATION joint in their one-piece mailbox.usda - springs
    # shut from 85 degrees on the published 0.015 N.m/deg, which per radian it could not do.
    # The cell's `join` makes a bare UsdPhysics.RevoluteJoint between two maximal-coordinate
    # bodies, and there the same number behaves per radian.
    #
    # Measured on the plain joint: the door should make 0.44 N.m at 29.5 degrees against
    # gravity's 0.245 and slam shut in under a second. It hung at 29.5 for a full twenty
    # seconds and was still 11.5 open at the end of the cycle - which is 0.015 N.m/RAD
    # giving 0.0077 N.m, a thirtieth of what is needed. Scaled by 57.3 it settles to 1.0
    # degree within the wait and seats at 0.74.
    #
    # So this conversion is correct for the joints THIS module creates and would be wrong
    # for an articulated one. If `join` ever learns to make articulation joints, it comes
    # out. The asset's per-degree values are the right ones to publish either way.
    DEG_PER_RAD = 57.29577951308232
    keys = {"stiffness": "assembly:seatJointDriveStiffness",
            "damping": "assembly:seatJointDriveDamping",
            "target_deg": "assembly:seatJointDriveTarget",
            "max_force": "assembly:seatJointDriveMaxForce"}
    drive = {k: float(p.attrs[a]) for k, a in keys.items() if a in p.attrs}
    # The door now lies open at its published flat angle and stays there for the whole
    # cycle, so that is where its spring has to hold it. Left at the seated target the
    # drive pulls a flat door shut the moment `restage` joins it.
    if "target_deg" in drive and "assembly:flatAngle" in p.attrs:
        drive["target_deg"] = float(p.attrs["assembly:flatAngle"])
    for k in ("stiffness", "damping"):
        if k in drive:
            drive[k] *= DEG_PER_RAD      # plain revolute; see above
    return drive or None


def _slack(name: str, shell: Part, p: Part) -> float:
    """Radial clearance per side, from the asset's own diameters where it states them."""
    if name == "Flag":
        # The peg in the hole. The usd session states 6 mm peg in an 8 mm hole.
        return (shell.mm_at("assembly:flagHole:diameter") - 6.0) / 2.0
    if name == "Pin":
        return (shell.mm_at("assembly:pinBore:diameter")
                - shell.mm_at("assembly:pin:diameter")) / 2.0
    if name == "Door":
        # NOT a bore clearance, and not a number to invent.
        #
        # The door CAPS OVER the shell's front end - its flange slides 18 mm along the
        # outside - so there is no hole it fits inside. Computing aperture width minus
        # flange width gave -3.70 mm, which read as an interference and is simply the
        # wrong subtraction: the flange is meant to be wider than the opening.
        #
        # The asset does not publish the flange's internal clearance and nothing here
        # should guess it. Returning a NaN makes every consumer say so instead of
        # quietly planning against a fabricated tolerance, which is the mistake this
        # module exists to stop.
        return float("nan")
    return 0.0
