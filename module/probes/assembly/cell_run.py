"""The learner's cell in motion: the cycle's own stations, run one verb at a time.

`cell_rules.CellRules` decides every outcome of a learner's verb; this is the `Motion` it
calls to make the cell match. It runs the SAME station code the film and `cycle_time`
run (`film_cycle._station`, the taught program, `index_to`, `restock_all`) - the cell does
not get a second way of doing anything - but a step at a time, in the order the learner's
verbs ask for, rather than as one fixed cycle.

Two things differ from the fixed cycle, and both come from the learner choosing the order:

  * A verb can reach a station from somewhere the program was never taught from (a pin
    skipped, so arm-b goes from home straight to the flag). The program's legs are
    numbered in the cycle's order, so each station sets the step counter to its own first
    leg. A station's entry leg is a joint move from wherever the arm stands to its via
    pose - which is how every entry leg of the taught cycle runs, from home (measured
    2026-10-18: each arm is at home after a reset, and each first leg's waypoints are the
    via pose alone).
  * An arm parks when its work for this shell is over, wherever that was.

The clients are whatever the caller has: a RobotClient's resources from a script, or the
cell service's dependencies inside the module, which are the same clients.
"""
from __future__ import annotations

import asyncio
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from viam.components.arm import JointPositions                       # noqa: E402

from assembly.arms import clear_and_park, home as arm_home, park_if_adrift  # noqa: E402
from assembly.build_fragment import SHELL_KINEMATIC, chosen          # noqa: E402
from assembly import film_cycle                                      # noqa: E402
from assembly.film_cycle import (CYCLE, LOOSE_STOP_MM, LOOSE_STOPS, REJECT_RUN_S,  # noqa: E402
                                 RIDE_S, RIDE_SPEED_MM_S, _station, set_paint)
from assembly.fixture import index_to, restock_all, run_belt, sim_now  # noqa: E402
from assembly.reach_check import POSES, declared, layout_hash, orientation_for  # noqa: E402
from assembly.replay import Legs, fly as fly_leg                      # noqa: E402
from assembly import shell_locate                                     # noqa: E402

# Where each learner station stands on the belt, by the cycle's own step names.
STOP = {"infeed": "tunnel", "paint": "tunnel", "inside": "inside", "pin": "pin",
        "flag": "flag", "grasp": "handle"}
# The cycle's steps a shell's ARRIVAL at a stop runs, before the learner is asked
# anything: arm-b's wrist camera has to be looking at the bore or the flag hole when the
# learner reads it, and the cycle looks at the bore with the pin already in the jaw.
ARRIVE = {"pin": ["pin_pick", "pin_bore_look"], "flag": ["flag_hole_look"]}
# The cycle's steps for each verb that moves an arm, in order.
STEPS = {
    "inside": ["inside"],
    "pin": ["pin_insert"],
    "flag": ["flag_pick", "flag_insert"],
    "grasp": ["grasp", "repaint"],
}
# The interior defect a shell shows when its truth is `defect` but carries no drawn look
# (cell_rules._inside_look draws rust or an interior smudge for a batch).
INTERIOR_DEFECT = "rust"
# Off the line, for an empty fixture: the shell parked out of every camera's view.
PARKED_SHELL_MM = [-2600.0, 2200.0, 50.0]
# The points the cell aims at by itself, the same numbers the pages quote and the rules
# score against (cell_rules.NUMBERS): the learner's point drives the arm as an offset from
# these, so a learner exactly on them gets the cell's own measured aim.
try:
    from isaac_module.cell_rules import NUMBERS as _N                # in the module
except ImportError:                                                  # a probe script
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
    from isaac_module.cell_rules import NUMBERS as _N                # noqa: E402
ENTRY = {"pin": _N["pin_bore_entry"], "flag": _N["flag_hole_entry"]}
WALL_CENTRE = _N["side_wall_centre"]
# Which arm works each station.
ARM_FOR = {"inside": "arm-a", "pin": "arm-b", "flag": "arm-b", "grasp": "arm-c"}


def _cycle_row(pose: str):
    return next(r for r in CYCLE if r[2] == pose)


class CellMotion:
    """`cell_rules.Motion` over the sim cell. Every hook returns when the cell has done it."""

    def __init__(self, world, arms: Dict[str, Any], grips: Dict[str, Any], motion,
                 grasp_cam, log=print):
        self.world, self.arms, self.grips, self.motion = world, arms, grips, motion
        self.log = log
        shell_locate.CLIENT = grasp_cam
        self.cell = chosen()
        self.stamp = layout_hash()
        self.legs = Legs.load(self.stamp)
        # The program's first leg for each station, from its labels ("pin_pick:via" ...).
        # Keyed by arm too: "park" is arm-a's leg, and arm-b has none.
        self.first_leg: Dict[tuple, int] = {}
        for key in sorted(self.legs.paths):
            step, arm = key.split(":", 1)
            label = self.legs.labels.get(key, "").split(":", 1)[0]
            self.first_leg.setdefault((label, arm), int(step))
        self.at: Optional[float] = None
        self.fitted: List[str] = []
        self.parked = {n: True for n in arms}
        self.looking = False                # arm-a's lens left in the door aperture

    # -- the line

    async def reset(self) -> None:
        from assembly.fixture import restage
        await set_paint(self.world, "none")
        await self._paint_look("none", "none", "none")
        await self._interior("none")
        await self._damage("none", "none")
        await restage(self.world, settle=3.0)
        await clear_and_park(self.world, self.arms, self.cell)
        await restock_all(self.world, self.cell)
        self.at, self.fitted = None, []
        self.parked = {n: True for n in self.arms}
        self.looking = False

    async def _interior(self, look: str, placement: str = "i0", scale: str = "s100",
                        texture: str = "t0") -> None:
        """The interior mark, on its own size and texture sets (Sam): a paint mark's draw
        never reaches inside."""
        await self.world.do_command({"command": "set_variants", "prop": "shell", "variants": {
            "interior_defect": look, "interior_placement": placement,
            "interior_scale": scale, "interior_texture": texture}})

    async def _paint_look(self, placement: str, scale: str, texture: str) -> None:
        """Where the paint mark sits, its size and texture; none puts back the asset's
        defaults, which a damage mark also reads (defect_scale, defect_texture)."""
        await self.world.do_command({"command": "set_variants", "prop": "shell", "variants": {
            "paint_placement": "p0" if placement == "none" else placement,
            "defect_scale": "s100" if scale == "none" else scale,
            "defect_texture": "t0" if texture == "none" else texture}})

    async def _damage(self, damage: str, placement: str) -> None:
        want = {"damage": damage}
        if placement != "none":
            want["damage_placement"] = placement
        await self.world.do_command({"command": "set_variants", "prop": "shell",
                                     "variants": want})

    async def _new_shell(self, shell: Dict[str, str]) -> None:
        """The next shell, as it arrived: its paint and interior, then restaged in the
        tunnel. Variants FIRST: switching one re-composes the shell and the restage's reset
        is what rebuilds the arms' physics views (film_cycle.line_event)."""
        await set_paint(self.world, shell["paint"])
        await self._paint_look(shell.get("paint_placement", "none"),
                               shell.get("paint_scale", "none"), shell.get("paint_texture", "none"))
        look = shell.get("interior_look")
        if look:                            # a capture's staged interior (cell_rules.stage)
            await self._interior(look["defect"], look["placement"], look["scale"],
                                 look["texture"])
        else:
            defect = shell.get("interior_defect", "none")
            placement = shell.get("interior_placement", "none")
            if shell["inside"] != "defect":
                defect, placement = "none", "i0"
            elif defect == "none":              # a truth from before the interior draw
                defect, placement = INTERIOR_DEFECT, "i0"
            await self._interior(defect, placement)
        await self._damage(shell.get("damage", "none"), shell.get("damage_placement", "none"))
        await restock_all(self.world, self.cell)
        self.at, self.fitted = None, []
        if shell["infeed"] == "empty":
            await self.world.do_command({"command": "place_prop", "name": "shell",
                                         "position_mm": PARKED_SHELL_MM})

    async def _index(self, station: str) -> None:
        step = STOP[station]
        x = self.cell.index_x(step)
        if self.at != x:
            self.at = await index_to(
                self.world, self.cell, x,
                fitted=tuple(self.fitted) if not SHELL_KINEMATIC else (),
                tolerance_mm=(LOOSE_STOP_MM if step in LOOSE_STOPS else None))

    async def bring(self, shell: Dict[str, str], station: str) -> None:
        """The next shell at `station`. A full-line batch starts every shell in the tunnel;
        a scoped one stages it at its station with what the earlier stations would have
        done to it - the checkpoints of `checkpoint.py`: at the pin stop with the door on,
        at the flag stop with the pin seated too, at the reject stop as it arrived."""
        await self._park("arm-a")           # a staged look left in the aperture
        await self._new_shell(shell)
        if station == "flag" and shell["infeed"] != "empty":
            # The pin goes in by the cell's own pick and insert, as on a whole line. Seated by
            # a teleport and a joint (checkpoint.seat), the flag tipped 38 degrees at its grab
            # every time, and a 3 s settle after the seat held only 2 of 4 (2026-10-19).
            await self._index("pin")
            for pose in ARRIVE["pin"] + STEPS["pin"]:
                await self._run(pose)
        await self._index(station)
        await self._arrive(station)

    async def _arrive(self, station: str) -> None:
        # The flag's look is taught from beside the bore, where the pin insert leaves arm-b.
        # From home (a pin skipped, missed or refused, or a flag-scoped batch) its first leg
        # is planned instead: replayed, the joint move swept the feeders and knocked the
        # flag askew before its pick (the jaw then held it 15 mm off, 2026-10-18).
        if station == "flag" and self.parked.get("arm-b"):
            first = self.first_leg.get(("flag_hole_look", "arm-b"))
            if first is not None:
                self.legs.plan_instead = {f"{first:03d}:arm-b"}
        try:
            for pose in ARRIVE.get(station, ()):
                await self._run(pose)
        finally:
            self.legs.plan_instead = set()

    async def stage(self, case: str, shell: Dict[str, str], station: str) -> None:
        await self.bring(shell, station)

    async def move_on(self, shell: Dict[str, str], station: str, to: str) -> None:
        # Every arm parks except the one the next station needs: arm-b goes from the pin
        # straight on to the flag, as taught.
        for arm in self.arms:
            if arm != ARM_FOR.get(to):
                await self._park(arm)
        if to == "none":
            if station == "grasp":
                return                      # the reject is already off down the spur
            await run_belt(self.world, run=True, speed_mm_s=RIDE_SPEED_MM_S)
            await asyncio.sleep(RIDE_S if shell["infeed"] != "empty" else 2 * RIDE_S)
            await run_belt(self.world, run=False)
            self.at = None
            return
        await self._index(to)
        await self._arrive(to)

    # -- the stations

    async def _run(self, pose: str, target=None) -> None:
        step, arm, _pose, tool, _what = _cycle_row(pose)
        await self._index_for(step)
        if self.parked.get(arm):
            await park_if_adrift(self.arms[arm], arm)
        first = self.first_leg.get((pose, arm))
        if first is not None:
            self.legs.step = first - 1
        world, arms, grips, motion = self.world, self.arms, self.grips, self.motion
        await _station(arm, pose, tool, arms=arms, grips=grips, motion=motion, world=world,
                       stamp=self.stamp, state=await declared(world, moving=arm),
                       legs=self.legs, cell=self.cell,
                       carry_state=await declared(world, moving=arm, motion=motion,
                                                  grip=grips.get(arm), tools=True),
                       held_state=await declared(world, moving=arm, motion=motion,
                                                 grip=grips.get(arm)))
        self.parked[arm] = False
        if tool and tool.startswith("seat:"):
            self.fitted.append(tool.split(":", 1)[1])

    async def _index_for(self, step: str) -> None:
        x = self.cell.index_x(step)
        if self.at != x:
            self.at = await index_to(self.world, self.cell, x,
                                     fitted=tuple(self.fitted) if not SHELL_KINEMATIC else ())

    async def _park(self, arm: str, label: str = "park") -> None:
        """Back to the arm's park, planned (or the taught leg when it has one), then home."""
        if self.parked.get(arm):
            return
        try:
            pj = json.loads((POSES / arm / "park.json").read_text())
            first = self.first_leg.get((label, arm))
            legs = self.legs
            if first is not None:
                legs.step = first - 1
            else:
                legs = None
            await fly_leg(legs, self.arms, self.motion, arm, pj["flange_mm"],
                          orientation_for(pj["approach"], pj.get("theta_deg", 0.0)),
                          await declared(self.world, moving=arm, tools=True), label=label)
        except Exception as exc:                                   # noqa: BLE001
            self.log(f"    park {arm}: {str(exc)[:160]}")
        await self.arms[arm].move_to_joint_positions(
            JointPositions(values=arm_home(arm)), timeout=180)
        self.parked[arm] = True
        if arm == "arm-a":
            self.looking = False

    async def inspect_inside(self, shell: Dict[str, str]) -> None:
        """The lens into the door aperture, and LEFT there: the learner reads fisheye-cam
        after the verb returns. The next verb's move_on (or a new shell) parks arm-a."""
        if self.looking:
            return                          # already in the aperture: a second look
        await self._run("inside")
        self.looking = True

    async def insert(self, part: str, result: str, target) -> None:
        if result in ("skipped", "refused"):
            # Nothing goes in (contract); a pin already in the jaw for the look goes back.
            if part == "pin" and film_cycle.HELD.get("Pin") is not None:
                await film_cycle.return_part(
                    self.world, self.arms, self.grips, self.motion, "arm-b", "pin_insert",
                    "Pin", self.stamp, await declared(self.world, moving="arm-b"),
                    await declared(self.world, moving="arm-b", motion=self.motion,
                                   grip=self.grips.get("arm-b"), tools=True))
                await self._park("arm-b")   # the flag's look is taught from home, not the nest
            return
        which = part.capitalize()
        entry = ENTRY[part]
        film_cycle.AIM[which] = {
            "offset_mm": [float(target[i]) - entry[i] for i in range(3)],
            "missed": result == "missed", "entry_mm": list(entry)}
        try:
            for pose in STEPS[part]:
                await self._run(pose)
        finally:
            film_cycle.AIM.pop(which, None)
        if result == "missed":
            self.fitted = [f for f in self.fitted if f != which]
            await self._park("arm-b")       # it put the part back; the next look starts home

    async def grasp(self, shell: Dict[str, str], result: str, target, normal) -> None:
        # Held: the cup goes to the learner's point on the wall's face - in the face's
        # plane, since how far off the plane the point was is what the score judges and
        # the cup has to touch the wall to hold. Slipped or skipped: the cell's own hold.
        if result == "held":
            film_cycle.AIM["Shell"] = {"offset_mm": [float(target[0]) - WALL_CENTRE[0], 0.0,
                                                     float(target[2]) - WALL_CENTRE[2]]}
        try:
            for pose in STEPS["grasp"]:
                await self._run(pose)
        finally:
            film_cycle.AIM.pop("Shell", None)
        await self._park("arm-c", label="home")
        await self.world.do_command({"command": "conveyor", "name": "reject_belt",
                                     "run": True})
        await asyncio.sleep(REJECT_RUN_S)
        await self.world.do_command({"command": "conveyor", "name": "reject_belt",
                                     "run": False})
