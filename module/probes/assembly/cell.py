"""The assembly cell, as one description.

ONE source. The Viam fragment is generated from this (see `build_fragment.py`), the
reachability checks read this, and the round reads this. The previous build kept the
layout in a fragment and re-derived stations in three places, and the bill for that was a
recurring bug: a number true of an older grasp still used for the current one - a 367 mm
standoff from an abandoned end grip, a fixture sited against a pose no arm visits, both
arms sharing one arm's standoff. Each passed every offline test and failed every run.

Nothing here touches Isaac or viam-server. Reach, pan, height and clearance are
arithmetic, and arithmetic answers in milliseconds instead of the seventy seconds a config
push and a module restart used to cost. Only IK and collision need the planner.
"""
from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

Vec3 = Tuple[float, float, float]

# ---- what a UR5e will do, measured on this cell rather than quoted ------------
#
# The hard limit is the one the planner enforced in the two-arm cell, 1016.7 mm, found by
# asking it; the datasheet's 850 mm is the wrist centre and the planner is stricter than
# either. 915 is 90% of it and is what `reach_budget` meant there. 800 is comfort - past
# it a spherical bound stops being honest about orientation.
REACH_HARD_MM = 1016.7
REACH_BUDGET_MM = 915.0
REACH_COMFORT_MM = 800.0
# How far off an arm's forward direction a station may sit. 35 in the old cell, and only
# because one station could not be brought inside 30; start at 30 again and make anything
# wider argue for itself.
MAX_PAN_DEG = 30.0
# And how far above its base. Past this an arm rears up over itself.
MAX_UP_MM = 280.0


@dataclass(frozen=True)
class Box:
    """An axis-aligned box in millimetres, by its centre and size."""

    name: str
    centre_mm: Vec3
    dims_mm: Vec3

    @property
    def top_mm(self) -> float:
        return self.centre_mm[2] + self.dims_mm[2] / 2.0

    def span(self, axis: int) -> Tuple[float, float]:
        return (self.centre_mm[axis] - self.dims_mm[axis] / 2.0,
                self.centre_mm[axis] + self.dims_mm[axis] / 2.0)

    def clearance_to(self, other: "Box") -> float:
        """Smallest gap along any axis. Negative means they overlap."""
        return max(max(self.span(a)[0] - other.span(a)[1],
                       other.span(a)[0] - self.span(a)[1]) for a in range(3))


@dataclass(frozen=True)
class Station:
    """A place an arm works, named by the arm and the task that goes there.

    The name is the point. `fragments/poses/<arm>/<task>.json` holds what was taught for
    it, so "where does arm-b go to pick a flag" is answerable without reading the round.
    """

    arm: str
    task: str
    point_mm: Vec3          # where the WORK is - the part, the hole, the rack
    approach: Vec3 = (0.0, 0.0, 1.0)   # unit vector the tool comes in along
    standoff_mm: float = 0.0           # how far back along it the flange sits

    @property
    def key(self) -> str:
        return f"{self.arm}/{self.task}"

    def flange_mm(self) -> Vec3:
        return tuple(self.point_mm[i] + self.approach[i] * self.standoff_mm
                     for i in range(3))


@dataclass
class Cell:
    """The assembly cell."""

    # The mailbox SHELL, in its own frame. Placeholder until the stripped asset lands;
    # the current asset publishes 494.9 x 172.2 x 226.9 as collision bounds and the shell
    # without its flag and door cannot be larger.
    shell_mm: Vec3 = (494.9, 172.2, 226.9)

    # The conveyor: along x, across the whole cell, narrow.
    belt_from_x_mm: float = -1400.0
    belt_to_x_mm: float = 3200.0
    belt_y_mm: float = 0.0
    belt_width_mm: float = 420.0
    belt_top_mm: float = 750.0

    # The fixture that rides it and raises the mailbox 100 mm, with lips that hold it.
    fixture_lift_mm: float = 100.0
    fixture_wall_mm: float = 18.0
    lip_mm: float = 28.0
    lip_clearance_mm: float = 6.0

    # Where the work happens, along the belt.
    work_x_mm: float = 600.0

    arm_bases_mm: Dict[str, Vec3] = field(default_factory=dict)
    # Which way each pedestal faces, in degrees about z. Free at install time.
    arm_yaw_deg: Dict[str, float] = field(default_factory=dict)
    extras: List[Box] = field(default_factory=list)

    # The door feeder rack and the repaint belt. Heights derive from the main belt so the
    # whole cell moves together when the belt does - the previous build had three heights
    # stated independently and spent a day discovering they had drifted apart.
    rack_x_mm: float = -150.0
    rack_y_mm: float = 760.0
    rack_above_belt_mm: float = 180.0
    repaint_x_mm: float = 1040.0
    repaint_y_mm: float = -820.0
    repaint_above_belt_mm: float = 160.0
    # How far the shell is lifted to be turned. Enough to clear the saddle's 26 mm lips
    # and no more: the overhead camera is fixed and looks down, so what the check needs is
    # the shell free to rotate, not the shell nearer the lens.
    handle_lift_mm: float = 90.0
    # Where the arms are mounted. Swept, not chosen - see `site.py`.
    arm_base_z_mm: float = 620.0

    # Where the belt stops the shell for each operation. DERIVED, not chosen: each arm
    # works at one x and the belt brings the shell to whatever position puts that
    # operation's flange there.
    #
    # The belt indexes both ways. A one-way index would force the assembly order to follow
    # the geometry - door, pin, then flag, because their flange offsets run +448, +236,
    # -135 from the shell's origin - and that is the conveyor dictating the process. An
    # indexing conveyor that can step back is ordinary, and the previous cell's infeed
    # already reversed to clear a jam, so the process keeps the order it wants.
    index_x_for: Dict[str, float] = field(default_factory=dict)

    def index_x(self, step: str) -> float:
        """Where the shell's ORIGIN sits when the belt has indexed it for `step`."""
        return self.index_x_for.get(step, self.work_x_mm)

    @property
    def door_rack_mm(self) -> Vec3:
        return (self.rack_x_mm, self.rack_y_mm,
                self.belt_top_mm + self.rack_above_belt_mm)

    @property
    def repaint_mm(self) -> Vec3:
        return (self.repaint_x_mm, self.repaint_y_mm,
                self.belt_top_mm + self.repaint_above_belt_mm)

    @property
    def shell_origin_mm(self) -> Vec3:
        """Where the shell's ORIGIN sits, seated in the saddle.

        The origin, not the centre: every hole the asset publishes is in the shell's own
        frame, so placing the origin is what makes those numbers usable without a second
        transform to get wrong.
        """
        from .asset import part as _part
        rail_bottom = _part("Shell").mm_at("assembly:wall:railBottomZ")
        return (self.work_x_mm, self.belt_y_mm,
                self.belt_top_mm + self.fixture_lift_mm - rail_bottom)

    # ---- derived geometry ----------------------------------------------------

    @property
    def belt(self) -> Box:
        length = self.belt_to_x_mm - self.belt_from_x_mm
        return Box("belt",
                   ((self.belt_from_x_mm + self.belt_to_x_mm) / 2.0, self.belt_y_mm,
                    self.belt_top_mm - 20.0),
                   (length, self.belt_width_mm, 40.0))

    @property
    def fixture(self) -> Box:
        """The box that raises the mailbox, sized to the shell it carries."""
        return Box("fixture",
                   (self.work_x_mm, self.belt_y_mm,
                    self.belt_top_mm + self.fixture_lift_mm / 2.0),
                   (self.shell_mm[0] + 2 * self.fixture_wall_mm,
                    self.shell_mm[1] + 2 * self.fixture_wall_mm,
                    self.fixture_lift_mm))

    @property
    def shell_centre_mm(self) -> Vec3:
        """Where the mailbox's centre sits, seated in the fixture."""
        return (self.work_x_mm, self.belt_y_mm,
                self.belt_top_mm + self.fixture_lift_mm + self.shell_mm[2] / 2.0)

    @property
    def shell(self) -> Box:
        return Box("shell", self.shell_centre_mm, self.shell_mm)

    def lips(self) -> List[Box]:
        """Four lips, standing proud of the fixture, holding the shell in place.

        Clearance is 6 mm rather than the old cell's 35: there the part was placed by an
        arm into a nest and 10 mm sat inside the planner's own collision margin, while
        here the fixture holds the part for the whole cycle and every insertion in the
        sequence is referenced to it. A part that can shift 35 mm is a part every taught
        insertion misses.
        """
        out, f = [], self.fixture
        gap = self.lip_clearance_mm
        for axis, sign in itertools.product((0, 1), (-1, 1)):
            centre = list(f.centre_mm)
            dims = [0.0, 0.0, self.lip_mm]
            half = self.shell_mm[axis] / 2.0 + gap + self.fixture_wall_mm / 2.0
            centre[axis] = f.centre_mm[axis] + sign * half
            centre[2] = f.top_mm + self.lip_mm / 2.0
            dims[axis] = self.fixture_wall_mm
            dims[1 - axis] = self.shell_mm[1 - axis] + 2 * gap
            out.append(Box(f"lip_{'xy'[axis]}{'-+'[sign > 0]}", tuple(centre), tuple(dims)))
        return out

    def obstacles(self) -> List[Box]:
        return [self.belt, self.fixture, *self.lips(), *self.extras]

    # ---- what an arm is being asked to do -----------------------------------

    def reach_mm(self, arm: str, point: Sequence[float]) -> float:
        return math.dist(tuple(point), self.arm_bases_mm[arm])

    def pan_deg(self, arm: str, point: Sequence[float]) -> float:
        """How far off this arm's OWN forward direction the point sits.

        Relative to the base's yaw, not to world +X. Both arms in the previous cell faced
        +X and so this was the same thing there, which is why it was never parametrised -
        and leaving it unparametrised here made the siting sweep fail every arrangement:
        a station 450 mm off in y and 350 in x reads 52 degrees off a +X-facing arm and 0
        degrees off one bolted down facing it. A pedestal's yaw is free at install time and
        costs nothing, so it belongs in the search.
        """
        base = self.arm_bases_mm[arm]
        bearing = math.degrees(math.atan2(point[1] - base[1], point[0] - base[0]))
        return (bearing - self.arm_yaw_deg.get(arm, 0.0) + 180.0) % 360.0 - 180.0

    def up_mm(self, arm: str, point: Sequence[float]) -> float:
        return point[2] - self.arm_bases_mm[arm][2]

    def judge(self, station: Station) -> List[str]:
        """What is wrong with asking this arm to work here. Empty means nothing."""
        flange = station.flange_mm()
        faults = []
        reach = self.reach_mm(station.arm, flange)
        if reach > REACH_BUDGET_MM:
            faults.append(f"reach {reach:.0f} past the {REACH_BUDGET_MM:.0f} budget")
        elif reach > REACH_COMFORT_MM:
            faults.append(f"reach {reach:.0f} past comfort")
        pan = self.pan_deg(station.arm, flange)
        if abs(pan) > MAX_PAN_DEG:
            faults.append(f"pan {pan:.0f} off forward")
        up = self.up_mm(station.arm, flange)
        if up > MAX_UP_MM:
            faults.append(f"{up:.0f} mm above its base")
        return faults
