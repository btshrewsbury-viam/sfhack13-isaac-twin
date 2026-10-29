"""Teach the assembly cycle once, then replay it.

Why this exists, in one measurement. The cycle was timed at 323.2 s with every move
planned, against a 120 s target, and no speed setting can close that gap because the time
is not in the arm. The sister cell's `docs/plan-full-scale-cell.md` measured the same split
on the same machine: stages that are the arm moving cost 1-9 s, stages that are the motion
service searching cost 29-63, and a round that budgeted 25.7 s of motion measured 399.1.
The documented lever order there is replay rather than plan, then pipelining, then fewer
moves, and only then the drives. This is the first lever, for this cell.

What replay is and is not. A planned move asks the motion service for a collision-free
path from wherever the arm is; a replayed move drives a joint path that was planned ONCE,
with the scene declared and every obstacle checked, and kept. Replay does no planning and
no collision checking - safe for exactly the reason a taught robot program is safe: the
path was validated when it was taught, and nothing in this cell moves between cycles
except the part, which the taught path already accounts for.

Legs are NUMBERED, not named, and the numbering is the claim. Cycle N makes the same calls
in the same order as the teaching run; if it does not, the program does not fit and this
refuses to replay rather than driving the wrong path at the wrong moment. That is the one
failure mode a taught program has and a planned one does not, so it is the one thing
checked on every step.

This mirrors `probes/qc/round.py`'s `Program` deliberately rather than importing it: that
one carries the QC round's two verdict endings and their branch keying, which the assembly
cycle - one linear sequence - has no use for.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

PROGRAM = (Path(__file__).resolve().parents[2]
           / "fragments" / "assembly-cycle-program.json")


@dataclass
class Legs:
    """One joint path per planned move of the cycle, in the order they happen."""

    paths: Dict[str, List[List[float]]] = field(default_factory=dict)
    # Where each leg was going, in world millimetres. Not used to replay - the joint path
    # is what executes - but a program you cannot read is a program you cannot debug, and
    # numbered legs alone say nothing about which move is which.
    targets: Dict[str, List[float]] = field(default_factory=dict)
    # What the cycle called the step the leg belongs to, for the same reason.
    labels: Dict[str, str] = field(default_factory=dict)
    teaching: bool = False
    step: int = 0
    # The layout the program was taught against. A taught path is only valid for the cell
    # it was taught in: replaying a program recorded before the conveyor moved would drive
    # the arm at where the belt used to be. The fragment already carries a geometry-only
    # fingerprint, so the program carries the same one and the mismatch is an error rather
    # than a surprise.
    layout: str = ""
    # Legs to PLAN rather than replay, by key: a leg reached from somewhere it was not
    # taught from (the learner's cell sends arm-b from home to the flag's look, a leg taught
    # from beside the bore; replayed from home, its joint move swept the feeders and
    # knocked the flag, 2026-10-18). Planned, it goes round what the world declares.
    plan_instead: set = field(default_factory=set)

    def key(self, arm: str) -> str:
        return f"{self.step:03d}:{arm}"

    @classmethod
    def teach(cls, layout: str = "") -> "Legs":
        return cls(teaching=True, layout=layout)

    @classmethod
    def load(cls, layout: str = "") -> "Legs":
        """The program on disk, refusing a program taught against another cell."""
        if not PROGRAM.exists():
            raise FileNotFoundError(
                f"{PROGRAM.name} does not exist: teach the cycle before replaying it")
        blob = json.loads(PROGRAM.read_text())
        was = str(blob.get("layout", ""))
        if layout and was and was != layout:
            raise RuntimeError(
                f"{PROGRAM.name} was taught against layout {was} and this cell is "
                f"{layout}. A taught path is only valid for the cell it was taught in - "
                f"replaying this one would drive the arm at where the belt used to be. "
                f"Re-teach.")
        return cls(paths={k: [[float(v) for v in pt] for pt in path]
                          for k, path in blob["paths"].items()},
                   targets=dict(blob.get("targets") or {}),
                   labels=dict(blob.get("labels") or {}),
                   layout=was)

    def save(self) -> Path:
        PROGRAM.parent.mkdir(parents=True, exist_ok=True)
        PROGRAM.write_text(json.dumps({
            "_comment": "Taught by probes/assembly/film_cycle.py --teach. Numbered legs, "
                        "in the order the cycle makes them. Replay drives these joint "
                        "paths and plans nothing.",
            "layout": self.layout,
            "legs": len(self.paths),
            "travel_deg": round(self.travel(), 1),
            "labels": self.labels,
            "targets": self.targets,
            "paths": self.paths,
        }, indent=2) + "\n")
        return PROGRAM

    def travel(self) -> float:
        """Total joint travel the program commands, in degrees.

        What makes one taught program better than another. Planning is randomised, so two
        teaching passes of the same cycle produce different programs, and the cheap one is
        worth keeping: a move that succeeds can still cost a wrist flip nobody asked for.
        """
        total = 0.0
        for path in self.paths.values():
            for a, b in zip(path, path[1:]):
                total += sum(abs(y - x) for x, y in zip(a, b))
        return total


# How straight an insertion stroke has to be, and how well it has to hold its orientation.
#
# A free plan is allowed to reach a goal pose by any route, and for most of this cycle that
# is what you want. An INSERTION is the exception: the pin travels 138 mm inside a bore
# with 0.20 mm of radial slack and a 1.5 mm chamfer at its mouth, so any bow in the path is
# a lever against the shell. While the stroke was a taught leg this did not arise - the
# recorded path was whatever had worked - but the stroke is now planned on every cycle
# against a measured shell frame, so the straightness has to be asked for rather than
# inherited.
#
# 1 mm and 2 degrees: inside what the chamfer captures, loose enough that the solver has
# somewhere to go. A tolerance tighter than the clearance it protects only fails to plan.
STRAIGHT = (1.0, 2.0)


def straight_line():
    """A constraint that makes a planned move travel in a straight line."""
    from viam.proto.service.motion import Constraints, LinearConstraint

    return Constraints(linear_constraint=[LinearConstraint(
        line_tolerance_mm=STRAIGHT[0], orientation_tolerance_degs=STRAIGHT[1])])


async def fly(legs: Optional[Legs], arms, motion, arm: str, flange_mm, orientation,
              state, *, label: str = "", constraints=None) -> Dict[str, Any]:
    """One leg of the cycle: replayed if it is taught, planned and recorded if not.

    `legs` of None plans everything and records nothing, which is what the cycle did
    before there was a program and is still what a one-off probe wants.

    `constraints` is passed straight to the planner. It is only ever set for a leg that
    has to travel a particular WAY rather than merely arrive - which in this cell means an
    insertion stroke, and nothing else.
    """
    from viam.proto.common import Pose, PoseInFrame

    async def planned() -> None:
        await motion.move(
            component_name=arm,
            destination=PoseInFrame(reference_frame="world", pose=Pose(
                x=flange_mm[0], y=flange_mm[1], z=flange_mm[2], **orientation)),
            world_state=state, constraints=constraints)

    if legs is None:
        await planned()
        return {"leg": None, "how": "planned"}

    legs.step += 1
    key = legs.key(arm)
    path = legs.paths.get(key)
    if path is not None:
        # Replay a taught leg whether or not this is a teaching pass. A teaching pass that
        # re-planned a leg it already has would hand the program two different answers for
        # one step number, and the later one starts from wherever the earlier one left the
        # arm - which is how the QC round ended up with a place step 263 degrees from where
        # the previous step finished.
        if key in legs.plan_instead and not legs.teaching:
            print(f"    {key} ({label or 'unlabelled'}): planned, not replayed - "
                  f"{arm} is not where this leg was taught from", flush=True)
            await planned()
            return {"leg": key, "how": "planned instead of replayed"}
        await arms[arm].do_command({"command": "replay", "path": path})
        return {"leg": key, "how": "replayed"}
    if not legs.teaching:
        raise RuntimeError(
            f"the taught program has no leg {key} ({label or 'unlabelled'}). It was "
            f"taught from a different sequence of moves than this cycle is making, so "
            f"replaying it would run the wrong path.")

    await arms[arm].do_command({"command": "record", "label": key, "reset": True})
    await planned()
    reply = await arms[arm].do_command({"command": "close_recording"})
    got = [[float(v) for v in pt] for pt in (reply.get("path") or [])]
    if not got:
        raise RuntimeError(
            f"leg {key} ({label or 'unlabelled'}) planned but recorded no waypoints, so "
            f"there is nothing to replay. The arm's recorder did not arm - check the "
            f"`record` command took before teaching the rest.")
    legs.paths[key] = got
    legs.targets[key] = [round(float(v), 1) for v in flange_mm]
    if label:
        legs.labels[key] = label
    return {"leg": key, "how": "taught", "waypoints": len(got)}
