"""The rules of the Viam 103 mailbox cell: verbs, refusals, truth, score.

The learner contract (viam-103 `learner-contract.md` section 2) fixes every request and
reply shape, refusal reason and scoreboard key. The course's offline fake
(`exercises/offline_cell.py`) already implements them, and the starter computes page 2's
baseline from the same score table, so this file is that fake's state machine, ported line
for line where it can be: the seeded batch uses the fake's draw and salt, so seed 103's
first shell is good here exactly as it is offline, and the points are the starter's.

What this file does NOT do is move anything. Every verb that changes the cell calls a
`Motion` hook, and the rules wait for it; `NullMotion` moves nothing, which is the mock
cell and the unit tests. The physical cell plugs in its own `Motion` and nothing here
changes.

Inserts and grasps are scored on the learner's POINT, not on the physical fit (plan R1):
`error_mm` is the distance from the target to the feature's entry centre, seated within
2 mm, missed to 10, refused beyond. Whether the part then seats is the cell's own gate.
"""

from __future__ import annotations

import asyncio
import math
import random
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

BELT_ORDER = ("infeed", "paint", "inside", "pin", "flag", "grasp")
STATIONS = BELT_ORDER
DIFFICULTIES = ("easy", "standard", "hard")
VERDICTS = {
    "infeed": ("line", "repaint", "skip"),
    "inside": ("pass", "fail"),
    "paint": ("pass", "fail"),
}
STAGE_CASES = {
    # case: (default station, stations it may be staged at)
    "good": ("infeed", ("infeed", "pin", "flag", "grasp")),
    "damaged": ("infeed", ("infeed",)),
    "empty": ("infeed", ("infeed",)),
    "inside_clean": ("inside", ("inside",)),
    "inside_defect": ("inside", ("inside",)),
    "paint_none": ("paint", ("paint",)),
    "paint_smudge": ("paint", ("paint",)),
    "paint_drip": ("paint", ("paint",)),
    "paint_overspray": ("paint", ("paint",)),
}
POINTS = {
    "correct": 100, "false_pass": -75, "false_fail": -25,
    "seated": 50, "held": 50, "missed": -50, "slipped": -50, "refused": -10, "skipped": 0,
}
# The starter's CELL_NUMBERS (mailbox_line.py), which the pages quote.
NUMBERS = {
    "pin_bore_entry": (556.0, 66.0, 775.0),
    "flag_hole_entry": (925.0, 82.0, 880.0),
    "side_wall_centre": (734.0, -80.0, 852.0),
    "side_wall_span_x": (516.0, 952.0),
    "side_wall_span_z": (788.0, 916.0),
    "grasp_cup_radius_mm": 15.0,
    "insert_seated_mm": 2.0,
    "insert_refused_mm": 10.0,
    "grasp_band_mm": 10.0,
    "grasp_band_deg": 5.0,
    "grasp_normal": (0.0, -1.0, 0.0),
}
# Where along the belt the cup carries the shell (world x of the aim): measured on the cell,
# a staged good shell at the grasp stop, the aim at (737, -83.4, 884.9) moved along the belt,
# held and carried from -70 to +150, dropped mid-carry at -80, no carry from -90 (2026-10-19).
# Pulled in by the band, so that a held grade is always an aim the cup was seen to carry.
GRASP_CARRY_X = (667.0, 887.0)
# Only the confidence range matters to the draw: it consumes the same random numbers the
# fake's does, so the truth sequence is identical.
_CONF = {"easy": (0.80, 0.99), "standard": (0.60, 0.97), "hard": (0.45, 0.90)}


def counter_keys() -> List[str]:
    keys = []
    for s in ("infeed", "inside", "paint"):
        keys += [f"{s}_correct", f"{s}_false_pass", f"{s}_false_fail"]
    keys.append("infeed_empty_misrouted")
    for p in ("pin", "flag"):
        keys += [f"{p}_seated", f"{p}_missed", f"{p}_refused", f"{p}_skipped"]
    keys += ["inserts_seated", "inserts_missed", "inserts_refused"]
    keys += ["grasps_held", "grasps_slipped", "grasps_skipped"]
    return keys


def judge_outcome(station: str, truth: str, verdict: str) -> str:
    """'correct', 'false_pass' or 'false_fail' (contract, "What counts as correct")."""
    if station == "infeed":
        right = {"good": "line", "damaged": "repaint", "empty": "skip"}[truth]
        if verdict == right:
            return "correct"
        if truth == "good":
            return "false_fail"
        if truth == "damaged":
            return "false_pass"
        return "false_pass" if verdict == "line" else "false_fail"
    defect = truth not in ("clean", "none")
    if verdict == ("fail" if defect else "pass"):
        return "correct"
    return "false_pass" if defect else "false_fail"


DAMAGES = ("chip", "crack", "scrape")
DAMAGE_PLACEMENTS = ("d0", "d1", "d2")    # all on the roof, where tunnel-top looks


def _damage(seed: int, i: int) -> Tuple[str, str]:
    """A damaged shell's damage, from its own stream so no other draw moves."""
    d = random.Random(f"viam-103-offline/{seed}/{i}/damage")
    return d.choice(DAMAGES), d.choice(DAMAGE_PLACEMENTS)


PAINT_PLACEMENTS = ("p0", "p1", "p2", "p3", "p4", "p5")
PAINT_SCALES = ("s100", "s60")
INTERIOR_LOOK = {"defect": ("none", "scratch", "rust", "bare_spot", "smudge", "missed_paint",
                            "paint_run", "overspray_splotch"),
                 "placement": ("i0", "i1", "i2"), "scale": ("s100", "s60"),
                 "texture": ("t0", "t1", "t2")}
PAINT_TEXTURES = ("t0", "t1")          # t2, the low-contrast one, is training-only (Felix)


def _paint_look(seed: int, i: int, paint: str) -> Tuple[str, str, str]:
    """A paint mark's placement, scale and texture, from its own stream so no other draw
    moves. Smudges stay full size: at s60 they are under the detector's pixels (Brandon)."""
    if paint == "none":
        return "none", "none", "none"
    d = random.Random(f"viam-103-offline/{seed}/{i}/paint")
    placement, scale, texture = (d.choice(PAINT_PLACEMENTS), d.choice(PAINT_SCALES),
                                 d.choice(PAINT_TEXTURES))
    return placement, "s100" if paint == "smudge" else scale, texture


# A batch's inside defect look: rust, an interior smudge or one of the large paint faults
# (Brandon, 2026-10-19: the inside check is a paint QC check), the kind uniform, then a
# placement uniform over the ones that look reads at the straight-on inside look with the
# lamp - short side >= 60 px rectified and >= 25 levels from clean (Felix, measured on assets
# 0a2d02b): the dark floor (i0) hides all but rust, and rust on the light -Y wall (i1) is too
# faint. Size s100 and texture t0 always.
INTERIOR_DRAWN = ("rust", "smudge", "missed_paint", "paint_run", "overspray_splotch")
INTERIOR_PLACEMENTS = {"rust": ("i0", "i2"), "smudge": ("i1", "i2"),
                       "missed_paint": ("i1", "i2"), "paint_run": ("i1", "i2"),
                       "overspray_splotch": ("i1", "i2")}


def _inside_look(seed: int, i: int, inside: str) -> Tuple[str, str]:
    """A defect shell's interior look, from its own stream so no other draw moves: which
    shells are defective, and every score, are the draw's as before."""
    if inside != "defect":
        return "none", "none"
    d = random.Random(f"viam-103-offline/{seed}/{i}/inside")
    defect = d.choice(INTERIOR_DRAWN)
    return defect, d.choice(INTERIOR_PLACEMENTS[defect])


def case_truth(case: str) -> Dict[str, str]:
    truth = {"infeed": "good", "inside": "clean", "paint": "none",
             "paint_placement": "none", "paint_scale": "none", "paint_texture": "none",
             "damage": "none", "damage_placement": "none",
             "interior_defect": "none", "interior_placement": "none"}
    if case in ("damaged", "empty"):
        truth["infeed"] = case
        if case == "damaged":
            truth["damage"], truth["damage_placement"] = "chip", "d0"
    elif case.startswith("inside_"):
        truth["inside"] = case[len("inside_"):]
        if truth["inside"] == "defect":
            truth["interior_defect"], truth["interior_placement"] = "rust", "i0"
    elif case.startswith("paint_"):
        truth["paint"] = case[len("paint_"):]
        if truth["paint"] != "none":
            truth["paint_placement"], truth["paint_scale"], truth["paint_texture"] = "p0", "s100", "t0"
    return truth


def draw_batch(seed: int, difficulty: str, size: int, scope: str) -> List[Dict[str, str]]:
    """Each shell's truth, the offline fake's draw exactly (same salt, same calls)."""
    rng = random.Random(f"viam-103-offline/{seed}/{difficulty}/{scope}")
    lo, hi = _CONF[difficulty]
    shells = []
    for i in range(size):
        infeed = rng.choices(["good", "damaged", "empty"], [6, 2, 2])[0] \
            if scope in ("all", "infeed") else "good"
        inside = rng.choices(["clean", "defect"], [7, 3])[0] \
            if scope in ("all", "inside") else "clean"
        paint = rng.choices(["none", "smudge", "drip", "overspray"], [5, 2, 2, 1])[0] \
            if scope in ("all", "paint") else "none"
        if infeed == "damaged":
            paint = "none"                    # the damage is the defect (Sam's variant)
        if infeed == "empty":
            inside, paint = "clean", "none"
        if i == 0 and scope == "all" and infeed == "good":
            inside, paint = "clean", "none"
        # The fake's per-shell noise, drawn and dropped so the next shell's truth matches.
        rng.uniform(lo, hi), rng.random(), rng.uniform(-1, 1), rng.uniform(-1, 1)
        rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-1, 1)
        rng.randrange(3), rng.uniform(0.35, 0.95), rng.random(), rng.uniform(0.25, 0.45)
        damage, placement = _damage(seed, i) if infeed == "damaged" else ("none", "none")
        look = _paint_look(seed, i, paint)
        interior = _inside_look(seed, i, inside)
        shells.append({"infeed": infeed, "inside": inside, "paint": paint,
                       "paint_placement": look[0], "paint_scale": look[1], "paint_texture": look[2],
                       "damage": damage, "damage_placement": placement,
                       "interior_defect": interior[0], "interior_placement": interior[1]})
    return shells


def refuse(reason: str, **extra: Any) -> Dict[str, Any]:
    return {"ok": False, "reason": reason, **extra}


def as_point(p: Any) -> Optional[Tuple[float, float, float]]:
    try:
        return (float(p["x"]), float(p["y"]), float(p["z"]))
    except (TypeError, KeyError, ValueError):
        return None


def angle_deg(a, b) -> float:
    na, nb = math.hypot(*a), math.hypot(*b)
    if na == 0 or nb == 0:
        return 180.0
    c = sum(x * y for x, y in zip(a, b)) / (na * nb)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


class Motion:
    """What the rules ask the cell to do. Each hook returns once the cell has done it.

    `shell` is the truth dict of the shell concerned. The rules decide every outcome before
    calling; a hook moves things to match and returns nothing the rules depend on.
    """

    async def reset(self) -> None:
        """Clear the line: no shell on it, parts back in their feeders."""

    async def bring(self, shell: Dict[str, str], station: str) -> None:
        """Put the next shell at `station`: ride in from the start (a full-line batch) or
        stage it there with what the earlier stations would have done (a scoped one)."""

    async def stage(self, case: str, shell: Dict[str, str], station: str) -> None:
        """Put the shell for `case` at `station`, outside any batch."""

    async def move_on(self, shell: Dict[str, str], station: str, to: str) -> None:
        """Index the shell from `station` to `to` (`none`: off the end, or off the spur
        after a grasp)."""

    async def inspect_inside(self, shell: Dict[str, str]) -> None:
        """Put the fisheye's lens in the door aperture."""

    async def insert(self, part: str, result: str, target) -> None:
        """Insert `part`: seat it (`seated`), show the miss at the bore face and return the
        part (`missed`), or do nothing (`refused`, `skipped` on a staged shell)."""

    async def grasp(self, shell: Dict[str, str], result: str, target, normal) -> None:
        """Lift the reject onto the spur: at the learner's point when `held`, at the cell's
        own hold after a slip or a skip."""


class NullMotion(Motion):
    """Moves nothing: the mock cell and the tests."""


@dataclass
class CellState:
    was_reset: bool = False
    status: str = "idle"
    seed: int = 0
    difficulty: str = "standard"
    scope: str = "all"
    shells: List[Dict[str, str]] = field(default_factory=list)
    index: int = -1
    station: str = "none"
    staged: Optional[Dict[str, str]] = None
    counters: Dict[str, int] = field(default_factory=lambda: {k: 0 for k in counter_keys()})
    score: int = 0
    shells_done: int = 0
    last_event: str = ""


class CellRules:
    """The `cell` service's verbs (contract section 2) over a `Motion`."""

    def __init__(self, motion: Optional[Motion] = None, numbers: Optional[dict] = None):
        self.motion = motion or NullMotion()
        self.n = dict(NUMBERS, **(numbers or {}))
        self.s = CellState()
        self._lock = asyncio.Lock()

    # -- reading

    def current(self) -> Optional[Dict[str, str]]:
        s = self.s
        if s.staged is not None:
            return s.staged
        if s.status == "running" and 0 <= s.index < len(s.shells) and s.station != "none":
            return s.shells[s.index]
        return None

    def readings(self) -> Dict[str, Any]:
        """`scoreboard.get_readings()`: every number a float, as it arrives over gRPC."""
        s = self.s
        r: Dict[str, Any] = {
            "status": s.status, "scope": s.scope, "seed": float(s.seed),
            "batch_size": float(len(s.shells)), "shell": float(max(s.index, 0)),
            "shells_done": float(s.shells_done), "difficulty": s.difficulty,
            "station": s.station, "last_event": s.last_event, "score": float(s.score),
        }
        r.update({k: float(v) for k, v in s.counters.items()})
        return r

    # -- the verbs

    async def do_command(self, command: Dict[str, Any]) -> Dict[str, Any]:
        if len(command) != 1:
            raise ValueError(f"expected one verb, got {sorted(command)}")
        verb, args = next(iter(command.items()))
        args = dict(args or {})
        handler = getattr(self, f"_verb_{verb}", None)
        if handler is None:
            raise ValueError(f"unknown cell verb: {verb}")
        if verb != "reset" and not self.s.was_reset:
            return refuse("not_reset")
        if self._lock.locked():
            return refuse("busy")
        async with self._lock:
            return await handler(args)

    async def _verb_reset(self, a: dict) -> dict:
        seed = int(a.get("seed", 103))
        difficulty = a.get("difficulty", "standard")
        if difficulty not in DIFFICULTIES:
            raise ValueError(f"difficulty must be one of {DIFFICULTIES}")
        scope = a.get("station", "all")
        if scope != "all" and scope not in STATIONS:
            return refuse("bad_station")
        size = int(a.get("batch", 12))
        await self.motion.reset()
        self.s = CellState(was_reset=True, seed=seed, difficulty=difficulty, scope=scope,
                           shells=draw_batch(seed, difficulty, size, scope),
                           last_event=f"reset seed {seed}, {size} shells, scope {scope}")
        reply = {"ok": True, "seed": seed, "difficulty": difficulty, "batch": size,
                 "station": "none"}
        if "station" in a:
            reply["scope"] = scope
        return reply

    async def _verb_start(self, a: dict) -> dict:
        s = self.s
        if s.status == "done":
            return refuse("batch_done")
        if s.staged is not None:
            await self.motion.reset()
        s.staged, s.status, s.station, s.last_event = None, "running", "none", "start"
        return {"ok": True, "station": "none"}

    async def _verb_advance(self, a: dict) -> dict:
        s = self.s
        if s.status == "done":
            return refuse("batch_done")
        if s.status != "running":
            return refuse("not_started")
        if s.station != "none":
            return refuse("not_at_station", expected=s.station)
        if s.index + 1 >= len(s.shells):
            s.status, s.last_event = "done", "batch done"
            truth = [{"shell": i, **t} for i, t in enumerate(s.shells)]
            return {"ok": True, "done": True, "station": "none", "truth": truth}
        s.index += 1
        station = BELT_ORDER[0] if s.scope == "all" else s.scope
        await self.motion.bring(s.shells[s.index], station)
        s.station, s.last_event = station, f"shell {s.index} at {station}"
        return {"ok": True, "shell": s.index, "station": station}

    async def _verb_stage(self, a: dict) -> dict:
        s = self.s
        if s.status == "running":
            return refuse("running")
        case = a.get("case")
        if case not in STAGE_CASES:
            return refuse("bad_case")
        default, allowed = STAGE_CASES[case]
        station = a.get("station", default)
        if station not in allowed:
            return refuse("bad_station")
        truth = case_truth(case)
        shell = truth
        # A capture's interior look (not the learner's): any kind, placement, scale and
        # texture at the inside stop, applied before the restage as a batch's paint is.
        look = a.get("interior")
        if look is not None:
            if not case.startswith("inside_") or not isinstance(look, dict) or any(
                    k not in INTERIOR_LOOK or look[k] not in INTERIOR_LOOK[k] for k in look):
                return refuse("bad_interior")
            shell = {**truth, "interior_look": {k: look.get(k, v[0])
                                                for k, v in INTERIOR_LOOK.items()}}
        await self.motion.stage(case, shell, station)
        s.staged, s.status, s.station = truth, "staged", station
        s.last_event = f"staged {case} at {station}"
        return {"ok": True, "case": case, "station": station}

    def _station_check(self, station: str) -> Optional[dict]:
        s = self.s
        if s.staged is None:
            if s.status == "done":
                return refuse("batch_done")
            if s.status != "running":
                return refuse("not_started")
        if s.station != station:
            return refuse("not_at_station", expected=s.station)
        return None

    @staticmethod
    def _after(station: str, reject: bool = False, empty: bool = False) -> str:
        """Where the shell goes after `station`. A reject skips every later check and
        stops at `grasp`; a rejected empty fixture has nothing to lift and leaves."""
        if reject:
            return "grasp" if station != "grasp" and not empty else "none"
        for nxt in BELT_ORDER[BELT_ORDER.index(station) + 1:]:
            if nxt != "grasp":
                return nxt
        return "none"

    async def _leave_or_move(self, shell: Dict[str, str], next_station: str) -> str:
        s = self.s
        if s.staged is not None:
            return s.station                  # a staged shell stays put for another try
        if s.scope != "all" and next_station != s.scope:
            next_station = "none"
        await self.motion.move_on(shell, s.station, next_station)
        s.station = next_station
        if next_station == "none":
            s.shells_done += 1
        return next_station

    def _scored(self) -> bool:
        return self.s.staged is None

    def _add(self, key: str, outcome: str) -> None:
        self.s.counters[key] += 1
        self.s.score += POINTS[outcome]

    async def _verb_route(self, a: dict) -> dict:
        station, verdict = a.get("station"), a.get("verdict")
        if station not in VERDICTS:
            return refuse("bad_station")
        if verdict not in VERDICTS[station]:
            return refuse("bad_verdict")
        if (r := self._station_check(station)) is not None:
            return r
        shell = self.current()
        truth = shell[station]
        outcome = judge_outcome(station, truth, verdict)
        if self._scored():
            self._add(f"{station}_{outcome}", outcome)
            if station == "infeed" and truth == "empty" and verdict == "line":
                self.s.counters["infeed_empty_misrouted"] += 1
        empty = shell["infeed"] == "empty"
        if station == "infeed" and verdict == "repaint":
            nxt = self._after(station, reject=True, empty=empty)
        elif station == "infeed":
            nxt = self._after(station) if verdict == "line" else "none"
        elif station == "paint" and empty and verdict == "pass":
            nxt = "none"
        else:
            nxt = self._after(station, reject=verdict == "fail", empty=empty)
        self.s.last_event = f"route {station} {verdict}"
        reply = {"ok": True, "station": await self._leave_or_move(shell, nxt)}
        if not self._scored():
            reply["correct"] = outcome == "correct"
        return reply

    async def _verb_inspect_inside(self, a: dict) -> dict:
        if (r := self._station_check("inside")) is not None:
            return r
        await self.motion.inspect_inside(self.current())
        self.s.last_event = "inspect_inside: lens in the door aperture"
        return {"ok": True, "camera": "fisheye-cam", "station": "inside"}

    async def _verb_insert(self, a: dict) -> dict:
        part = a.get("part")
        if part not in ("pin", "flag"):
            return refuse("bad_part")
        skip = bool(a.get("skip", False))
        target = None if skip else as_point(a.get("target"))
        if not skip and target is None:
            return refuse("bad_target")
        if (r := self._station_check(part)) is not None:
            return r
        shell = self.current()
        nxt = self._after(part)
        if skip:
            if self._scored():
                self.s.counters[f"{part}_skipped"] += 1
            await self.motion.insert(part, "skipped", None)
            self.s.last_event = f"insert {part} skipped"
            return {"ok": True, "result": "skipped",
                    "station": await self._leave_or_move(shell, nxt)}
        nominal = self.n["pin_bore_entry"] if part == "pin" else self.n["flag_hole_entry"]
        err = math.dist(target, nominal)
        if err <= self.n["insert_seated_mm"]:
            result = "seated"
        elif err <= self.n["insert_refused_mm"]:
            result = "missed"
        else:
            result = "refused"
        await self.motion.insert(part, result, target)
        if self._scored():
            self._add(f"{part}_{result}", result)
            self.s.counters[f"inserts_{result}"] += 1
        self.s.last_event = f"insert {part} {result} {err:.1f} mm"
        return {"ok": True, "result": result, "error_mm": round(err, 2),
                "station": await self._leave_or_move(shell, nxt)}

    def _wall_error(self, target) -> float:
        """Distance from `target` to the -Y wall's face, inset by the cup's radius, and along
        the belt to where the cup carries the shell (GRASP_CARRY_X)."""
        wall = self.n["side_wall_centre"]
        r = self.n["grasp_cup_radius_mm"]
        (x0, x1), (z0, z1) = self.n["side_wall_span_x"], self.n["side_wall_span_z"]
        band = self.n["grasp_band_mm"]
        x = min(max(target[0], x0 + r, GRASP_CARRY_X[0] + band),
                x1 - r, GRASP_CARRY_X[1] - band)
        z = min(max(target[2], z0 + r), z1 - r)
        return math.dist(target, (x, wall[1], z))

    async def _verb_grasp(self, a: dict) -> dict:
        skip = bool(a.get("skip", False))
        target = None if skip else as_point(a.get("target"))
        normal = as_point(a.get("normal", {"x": 0.0, "y": -1.0, "z": 0.0}))
        if not skip and (target is None or normal is None):
            return refuse("bad_target")
        if (r := self._station_check("grasp")) is not None:
            return r
        shell = self.current()
        if skip:
            if self._scored():
                self.s.counters["grasps_skipped"] += 1
            await self.motion.grasp(shell, "skipped", None, None)
            self.s.last_event = "grasp skipped; lifted at the cell's own hold"
            return {"ok": True, "result": "skipped",
                    "station": await self._leave_or_move(shell, "none")}
        err = self._wall_error(target)
        err_deg = angle_deg(normal, self.n["grasp_normal"])
        held = err <= self.n["grasp_band_mm"] and err_deg <= self.n["grasp_band_deg"]
        result = "held" if held else "slipped"
        await self.motion.grasp(shell, result, target, normal)
        if self._scored():
            self._add(f"grasps_{result}", result)
        self.s.last_event = f"grasp {result} {err:.1f} mm {err_deg:.1f} deg"
        return {"ok": True, "result": result, "error_mm": round(err, 2),
                "error_deg": round(err_deg, 2),
                "station": await self._leave_or_move(shell, "none")}


# One rules object per configured cell, so the scoreboard reads the cell it names.
CELLS: Dict[str, CellRules] = {}
