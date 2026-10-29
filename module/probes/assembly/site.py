"""Where the arms go, and whether two of them are enough.

Swept, not argued. The previous cell's siting was the hard part of the whole build - a 5 mm
window for the place, a 12 mm one for the pick, one station that could only be reached at
35 degrees of pan, and one pose an arm could not hold at all - and every one of those was
found by running out of options rather than by choosing badly at the start. A third arm
that must reach the same work makes that search harder, so the arm count is decided by
which arrangement leaves the most margin, not by which has fewer mechanisms.
"""
from __future__ import annotations

import itertools
import math
from dataclasses import replace
from typing import Dict, Iterable, List, Sequence, Tuple

from .cell import Cell, MAX_PAN_DEG, MAX_UP_MM, REACH_BUDGET_MM, REACH_COMFORT_MM
from .stations import THREE, TWO, stations_for


# A comfortable radius for a feeder, and how far off the arm's facing it sits.
radius, off = 620.0, math.radians(26.0)


# Which operation's flange offset from the shell's origin, along x. Derived once from the
# asset so the belt's index positions can be solved rather than guessed.
def _flange_dx(cell: Cell) -> Dict[str, float]:
    from .stations import stations_for
    probe = Cell(belt_top_mm=cell.belt_top_mm, arm_base_z_mm=cell.arm_base_z_mm)
    out = {}
    for st in stations_for(probe):
        if not st.on_belt:
            continue
        out.setdefault(_step_of(st.task), st.flange_mm()[0] - st.index_x_mm)
    return out


def _step_of(task: str) -> str:
    return {"inside": "inside", "back_shot": "back",
            "flag_hole_look": "flag", "flag_insert": "flag",
            "door_place": "door", "door_pick": "door", "pin_insert": "pin",
            "grasp": "handle", "present_overhead": "handle", "return": "handle",
            "repaint": "handle"}.get(task, "inside")


def _indexes(cell: Cell, bases: Dict[str, object], roles: Dict[str, str]) -> Dict[str, float]:
    """Solve the belt's stop positions so each arm works at its own x.

    One equation per operation: index + flange_dx = the serving arm's x. That is the whole
    reason a conveyor runs the length of the cell - it moves the work to the arm so the arm
    does not have to cover the work's whole span, which no 30 degree pan cone does.
    """
    dx = _flange_dx(cell)
    owner = {"inside": "inspect", "back": "inspect", "flag": "assemble",
             "door": "assemble", "pin": "assemble", "handle": "handle"}
    return {step: bases[roles[owner[step]]][0] - dx[step] for step in dx}


def _staged(belt: float, base_z: float, bases: Dict[str, object],
            yaws: Dict[str, float], roles: Dict[str, str],
            radius: float, off: float) -> Cell:
    """A cell with the feeder and the repaint belt placed inside the serving arms' cones."""
    def beside(arm: str, taken: List[Tuple[float, float]]):
        """A feeder spot in this arm's cone that clears the belt and everything else.

        Searched, not offset by a fixed amount. A single fixed offset put feeders either
        into the conveyor - the arm faces the shell, so rotating toward its facing walks
        onto the belt - or on top of another pedestal: measured, the repaint spot landed
        101 mm from arm-a. A feeder's position is free, so the cell looks for a free one
        instead of asserting one and failing.
        """
        bx, by, _ = bases[arm]
        edge = 210.0 + CLEAR_OF_BELT_MM
        best = None
        for degrees in (20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0):
            for sign in (-1.0, 1.0):
                for r in (radius, radius + 120.0):
                    a = math.radians(yaws.get(arm, 0.0)) + sign * math.radians(degrees)
                    spot = (bx + r * math.cos(a), by + r * math.sin(a))
                    if abs(spot[1]) < edge:
                        continue
                    if any(math.dist(spot, t) < 420.0 for t in taken):
                        continue
                    # And inside the arm's own pan cone. The search used to check only
                    # that a spot was free, which left the feeder at 38 degrees of pan and
                    # the repaint belt at 40 - both outside the bound, and both only
                    # because nothing had asked. A feeder's angle is as free as its
                    # distance, so it is chosen the same way.
                    bearing = math.degrees(math.atan2(spot[1] - by, spot[0] - bx))
                    # Generously, because the yaw has not settled yet: an arm standing
                    # beside the belt faces the belt, so at this moment ANY spot clear of
                    # the conveyor is well off its facing. Demanding the final bound here
                    # rejected every candidate. The yaw iteration then turns the pedestal
                    # to split the difference between the work and the feeder, and
                    # `margins` is what finally judges it.
                    if abs((bearing - yaws.get(arm, 0.0) + 180.0) % 360.0 - 180.0) > 75.0:
                        continue
                    # Prefer the BEST ALIGNED workable spot, not the nearest.
                    #
                    # Nearest was the wrong preference and the sweep said so: every station
                    # that rides the belt cleared with 46 to 242 of margin and the only two
                    # failures left in the cell were the feeder and the repaint belt, at 38
                    # and 40 degrees of pan. Reach was never the constraint for them - both
                    # sit around 500 to 650 mm, well inside the budget - so trading a
                    # little reach for alignment is free.
                    cost = abs((bearing - yaws.get(arm, 0.0) + 180.0) % 360.0 - 180.0)
                    if best is None or cost < best[1]:
                        best = (spot, cost)
        return best[0] if best else (bx, by)

    pedestals = [b[:2] for b in bases.values()]
    rack_x, rack_y = beside(roles["assemble"], pedestals)
    rep_x, rep_y = beside(roles["handle"], pedestals + [(rack_x, rack_y)])
    staged = Cell(belt_top_mm=belt, arm_base_z_mm=base_z, arm_bases_mm=dict(bases),
                  arm_yaw_deg=dict(yaws), rack_x_mm=rack_x, rack_y_mm=rack_y,
                  repaint_x_mm=rep_x, repaint_y_mm=rep_y)
    return Cell(belt_top_mm=belt, arm_base_z_mm=base_z, arm_bases_mm=dict(bases),
                arm_yaw_deg=dict(yaws), rack_x_mm=rack_x, rack_y_mm=rack_y,
                repaint_x_mm=rep_x, repaint_y_mm=rep_y,
                index_x_for=_indexes(staged, bases, roles))


# How much room things need from the main conveyor, and how high a conveyor may be.
#
# Added after the sweep returned a layout that was optimal and absurd: a 300 mm high belt
# with the repaint belt at y -207, which is INSIDE the main belt's own y -210..210. The
# harness was answering reach and pan against a model that did not know two belts cannot
# occupy the same space - which is the failure mode of a harness that re-implements the
# cell rather than sharing it, and worth catching here rather than in Isaac.
CLEAR_OF_BELT_MM = 160.0
BELT_TOP_RANGE_MM = (600.0, 950.0)


def physical(cell: Cell) -> bool:
    """Is this arrangement a thing that could be built?"""
    if not (BELT_TOP_RANGE_MM[0] <= cell.belt_top_mm <= BELT_TOP_RANGE_MM[1]):
        return False
    edge = cell.belt_width_mm / 2.0 + CLEAR_OF_BELT_MM
    # Feeders, the repaint belt and every pedestal stand clear of the main conveyor.
    for point in (cell.door_rack_mm, cell.repaint_mm, *cell.arm_bases_mm.values()):
        if abs(point[1] - cell.belt_y_mm) < edge:
            return False
    # And clear of each other.
    spots = [cell.door_rack_mm[:2], cell.repaint_mm[:2],
             *[b[:2] for b in cell.arm_bases_mm.values()]]
    for a, b in itertools.combinations(spots, 2):
        if math.dist(a, b) < 420.0:
            return False
    return True


def margins(cell: Cell, roles: Dict[str, str]) -> Tuple[float, Dict[str, float]]:
    """Worst margin over all stations, and the worst per station.

    One scale for three different bounds, so they can be compared: millimetres of reach,
    millimetres of height, and degrees of pan counted at 10 mm each. A pan bound busted by
    3 degrees is about as bad as a reach bound busted by 30 mm, which is the right order.
    """
    per: Dict[str, float] = {}
    for st in stations_for(cell):
        arm = roles[st.role]
        base = cell.arm_bases_mm.get(arm)
        if base is None:
            per[st.key] = -1e6
            continue
        f = st.flange_mm()
        per[st.key] = min(REACH_BUDGET_MM - cell.reach_mm(arm, f),
                          (MAX_PAN_DEG - abs(cell.pan_deg(arm, f))) * 10.0,
                          MAX_UP_MM - cell.up_mm(arm, f))
    return (min(per.values()) if per else -1e6), per


def sweep(roles: Dict[str, str], belt_tops: Sequence[float], base_zs: Sequence[float],
          xs: Sequence[float], ys: Sequence[float],
          side: Dict[str, float] = None) -> List[Tuple[float, Cell]]:
    """Try arrangements and return them best-margin first.

    The arms sit in a line either side of the belt, which is what a cell built around a
    through-conveyor looks like: two arms face each other across it and a third stands
    further along. Mirrored in y rather than swept independently, because an asymmetric
    cell has to earn its asymmetry and this one has no reason to.
    """
    names = sorted(set(roles.values()))
    out: List[Tuple[float, Cell]] = []
    for belt, base_z in itertools.product(belt_tops, base_zs):
        for combo in itertools.product(itertools.product(xs, ys), repeat=len(names)):
            bases, yaws = {}, {}
            ok = True
            for name, (x, y) in zip(names, combo):
                bases[name] = (x, y, base_z)
            # No two arms in the same place, and none on the belt's own centreline.
            points = list(bases.values())
            if any(math.dist(a[:2], b[:2]) < 500.0
                   for a, b in itertools.combinations(points, 2)):
                ok = False
            if any(abs(p[1]) < 350.0 for p in points):
                ok = False
            # Which side of the belt a role must stand on, where the ASSET decides it.
            # The flag hole's axis is (0,-1,0) and the pin bore's is (0,-1,0), so both are
            # entered from +Y: an assemble arm on -Y has to reach across the shell for
            # every insertion it makes, which is what the sweep kept reporting as 938 mm
            # of reach. That is not a siting preference, it is the part's geometry.
            for role, want in (side or {}).items():
                arm = roles[role]
                if arm in bases and bases[arm][1] * want <= 0:
                    ok = False
            if not ok:
                continue
            # Face each pedestal at the shell, which is what anyone bolting one down would
            # do, and turns pan from a constraint the layout fights into one it mostly
            # satisfies by construction.
            probe = Cell(belt_top_mm=belt, arm_base_z_mm=base_z, arm_bases_mm=bases)
            shell_at = probe.shell_origin_mm
            for name in names:
                base = bases[name]
                yaws[name] = math.degrees(math.atan2(shell_at[1] - base[1],
                                                     shell_at[0] - base[0]))
            # Then settle it. Facing the shell is the right first guess and the wrong
            # answer: an arm also serves its feeder and, for the assemble arm, a pin that
            # enters from the side - so the yaw that minimises pan faces the CENTROID of
            # everything it serves, and that centroid moves when the feeder moves, which
            # moves when the yaw does. Three passes converge it.
            for _ in range(3):
                staged = _staged(belt, base_z, bases, yaws, roles, radius, off)
                for name in names:
                    # Weight the work twice as heavily as a feeder. A rack 600 mm out to
                    # one side otherwise drags the facing off the shell and every
                    # insertion pays for it in pan: measured, 60 degrees at the flag hole.
                    served = []
                    for st in stations_for(staged):
                        if roles[st.role] != name:
                            continue
                        served.extend([st.flange_mm()] * (2 if st.on_belt else 1))
                    if not served:
                        continue
                    cx = sum(p[0] for p in served) / len(served)
                    cy = sum(p[1] for p in served) / len(served)
                    base = bases[name]
                    yaws[name] = math.degrees(math.atan2(cy - base[1], cx - base[0]))
            # The rack and the repaint belt are PLACED, not given. Where a feeder sits and
            # where rejects go are free choices, so pinning them at arbitrary coordinates
            # and then asking whether an arm can reach them is asking the wrong question -
            # and it was the wrong question loudly: they were the two worst stations in the
            # sweep purely because I had put them somewhere.
            #
            # So each goes at a comfortable radius inside the serving arm's own pan cone,
            # on the opposite side of its facing from the shell, which is also where it
            # belongs physically: the arm swings between the work and the feeder without
            # crossing itself.
            cell = _staged(belt, base_z, bases, yaws, roles, radius, off)
            if not physical(cell):
                continue
            worst, _ = margins(cell, roles)
            out.append((worst, cell))
    out.sort(key=lambda t: -t[0])
    return out


def report(label: str, roles: Dict[str, str], best: Tuple[float, Cell]) -> str:
    worst, cell = best
    _, per = margins(cell, roles)
    lines = [f"{label}: worst margin {worst:+.0f}",
             f"  belt top {cell.belt_top_mm:.0f}  arm bases z {cell.arm_base_z_mm:.0f}"]
    for name in sorted(cell.arm_bases_mm):
        lines.append(f"    {name} at {[round(v) for v in cell.arm_bases_mm[name]]}"
                     f"  ({roles_of(roles, name)})")
    lines.append(f"    rack at {[round(v) for v in cell.door_rack_mm]}, "
                 f"repaint at {[round(v) for v in cell.repaint_mm]}")
    for st in stations_for(cell):
        arm = roles[st.role]
        f = st.flange_mm()
        reach, pan, up = (cell.reach_mm(arm, f), abs(cell.pan_deg(arm, f)),
                          cell.up_mm(arm, f))
        why = []
        if reach > REACH_BUDGET_MM:
            why.append(f"reach {reach:.0f}")
        if pan > MAX_PAN_DEG:
            why.append(f"pan {pan:.0f}")
        if up > MAX_UP_MM:
            why.append(f"up {up:.0f}")
        lines.append(f"    {st.key:<30} {per[st.key]:+7.0f}  "
                     f"reach {reach:4.0f} pan {pan:4.0f} up {up:4.0f}"
                     f"{'   <-- ' + ', '.join(why) if why else ''}")
    return "\n".join(lines)


def roles_of(roles: Dict[str, str], arm: str) -> str:
    return ", ".join(sorted(r for r, a in roles.items() if a == arm))
