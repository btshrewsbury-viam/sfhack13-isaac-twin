"""The cell's verbs against the viam-103 learner contract section 2, with no motion."""

import asyncio
import os
import sys
from pathlib import Path

import pytest

from isaac_module.cell_rules import CellRules, draw_batch, POINTS


def run(rules, verb, args=None):
    return asyncio.run(rules.do_command({verb: args or {}}))


def started(seed=103, **kw):
    rules = CellRules()
    run(rules, "reset", {"seed": seed, **kw})
    run(rules, "start")
    return rules


def test_every_verb_but_reset_is_refused_first():
    rules = CellRules()
    assert run(rules, "advance") == {"ok": False, "reason": "not_reset"}
    assert run(rules, "reset", {"seed": 103, "difficulty": "standard", "batch": 12}) == {
        "ok": True, "seed": 103, "difficulty": "standard", "batch": 12, "station": "none"}
    assert run(rules, "advance") == {"ok": False, "reason": "not_started"}


def test_seed_103_first_shell_is_good_and_walks_every_check():
    for difficulty in ("easy", "standard", "hard"):
        assert draw_batch(103, difficulty, 12, "all")[0] == {
            "infeed": "good", "inside": "clean", "paint": "none",
            "paint_placement": "none", "paint_scale": "none", "paint_texture": "none",
            "damage": "none", "damage_placement": "none",
            "interior_defect": "none", "interior_placement": "none"}
    rules = started()
    assert run(rules, "advance") == {"ok": True, "shell": 0, "station": "infeed"}
    assert run(rules, "route", {"station": "infeed", "verdict": "line"})["station"] == "paint"
    assert run(rules, "route", {"station": "paint", "verdict": "pass"})["station"] == "inside"
    assert run(rules, "inspect_inside")["camera"] == "fisheye-cam"
    assert run(rules, "route", {"station": "inside", "verdict": "pass"})["station"] == "pin"
    got = run(rules, "insert", {"part": "pin", "target": {"x": 556.0, "y": 66.0, "z": 776.4}})
    assert got == {"ok": True, "result": "seated", "error_mm": 1.4, "station": "flag"}
    got = run(rules, "insert", {"part": "flag", "skip": True})
    assert got == {"ok": True, "result": "skipped", "station": "none"}
    r = asyncio.run(_readings(rules))
    assert r["score"] == 3 * POINTS["correct"] + POINTS["seated"]
    assert r["pin_seated"] == 1.0 and r["flag_skipped"] == 1.0 and r["shells_done"] == 1.0
    assert all(isinstance(v, float) for k, v in r.items()
               if k not in ("status", "scope", "difficulty", "station", "last_event"))


def test_damaged_shells_carry_damage_not_paint():
    from isaac_module.cell_rules import DAMAGES, DAMAGE_PLACEMENTS
    shells = draw_batch(103, "standard", 200, "all")
    damaged = [s for s in shells if s["infeed"] == "damaged"]
    assert damaged
    for s in damaged:
        assert s["paint"] == "none"
        assert s["damage"] in DAMAGES and s["damage_placement"] in DAMAGE_PLACEMENTS
    assert all((s["damage"], s["damage_placement"]) == ("none", "none")
               for s in shells if s["infeed"] != "damaged")


def test_paint_marks_draw_placement_scale_texture_and_smudges_stay_full_size():
    from collections import Counter
    from isaac_module.cell_rules import PAINT_PLACEMENTS, PAINT_SCALES, PAINT_TEXTURES
    shells = [s for seed in range(100, 140) for s in draw_batch(seed, "standard", 12, "all")]
    for s in shells:
        look = (s["paint_placement"], s["paint_scale"], s["paint_texture"])
        if s["paint"] == "none":
            assert look == ("none", "none", "none")
        else:
            assert look[0] in PAINT_PLACEMENTS and look[1] in PAINT_SCALES
            assert look[2] in PAINT_TEXTURES
    assert {s["paint_scale"] for s in shells if s["paint"] == "smudge"} == {"s100"}
    assert {s["paint_scale"] for s in shells if s["paint"] == "drip"} == set(PAINT_SCALES)
    # Every smudge placement occurs, so each tunnel camera keeps its two.
    assert set(Counter(s["paint_placement"] for s in shells
                       if s["paint"] == "smudge")) == set(PAINT_PLACEMENTS)


async def _readings(rules):
    return rules.readings()


def test_insert_bands_and_wrong_station():
    rules = started()
    run(rules, "advance")
    assert run(rules, "insert", {"part": "pin", "target": {"x": 0, "y": 0, "z": 0}}) == {
        "ok": False, "reason": "not_at_station", "expected": "infeed"}
    assert run(rules, "insert", {"part": "nail"})["reason"] == "bad_part"
    run(rules, "route", {"station": "infeed", "verdict": "line"})
    run(rules, "route", {"station": "paint", "verdict": "pass"})
    run(rules, "route", {"station": "inside", "verdict": "pass"})
    got = run(rules, "insert", {"part": "pin", "target": {"x": 556.0, "y": 61.0, "z": 775.0}})
    assert (got["result"], got["error_mm"]) == ("missed", 5.0)
    got = run(rules, "insert", {"part": "flag", "target": {"x": 925.0, "y": 82.0, "z": 900.0}})
    assert (got["result"], got["station"]) == ("refused", "none")


def test_a_reject_rides_to_the_grasp_and_a_wrong_grasp_slips():
    rules = started()
    run(rules, "advance")
    run(rules, "route", {"station": "infeed", "verdict": "line"})
    assert run(rules, "route", {"station": "paint", "verdict": "fail"})["station"] == "grasp"
    got = run(rules, "grasp", {"target": {"x": 734.0, "y": -80.0, "z": 885.0}})
    assert (got["result"], got["station"]) == ("held", "none")
    r = rules.readings()
    assert r["paint_false_fail"] == 1.0 and r["grasps_held"] == 1.0


def test_stage_refused_during_a_batch_and_scores_nothing():
    rules = started()
    assert run(rules, "stage", {"case": "good"}) == {"ok": False, "reason": "running"}
    rules = CellRules()
    run(rules, "reset", {"seed": 103})
    assert run(rules, "stage", {"case": "paint_drip"}) == {
        "ok": True, "case": "paint_drip", "station": "paint"}
    got = run(rules, "route", {"station": "paint", "verdict": "fail"})
    assert got == {"ok": True, "station": "paint", "correct": True}
    assert rules.readings()["score"] == 0.0
    assert run(rules, "stage", {"case": "damaged", "station": "pin"})["reason"] == "bad_station"


def test_stage_interior_look_reaches_motion_not_truth():
    from isaac_module.cell_rules import NullMotion

    class Seen(NullMotion):
        async def stage(self, case, shell, station):
            self.shell = shell

    rules = CellRules(motion=Seen())
    run(rules, "reset", {"seed": 103})
    look = {"defect": "scratch", "placement": "i2", "texture": "t1"}
    got = run(rules, "stage", {"case": "inside_clean", "interior": look})
    assert got == {"ok": True, "case": "inside_clean", "station": "inside"}
    assert rules.motion.shell["interior_look"] == {**look, "scale": "s100"}
    assert "interior_look" not in rules.s.staged
    for bad in ({"defect": "mold"}, {"colour": "red"}, "rust"):
        got = run(rules, "stage", {"case": "inside_clean", "interior": bad})
        assert got == {"ok": False, "reason": "bad_interior"}
    got = run(rules, "stage", {"case": "good", "interior": {"defect": "rust"}})
    assert got == {"ok": False, "reason": "bad_interior"}


def test_scoped_batch_and_end_of_batch_truth():
    rules = CellRules()
    got = run(rules, "reset", {"seed": 103, "batch": 2, "station": "pin"})
    assert got["scope"] == "pin"
    run(rules, "start")
    assert run(rules, "advance") == {"ok": True, "shell": 0, "station": "pin"}
    assert run(rules, "insert", {"part": "pin", "skip": True})["station"] == "none"
    run(rules, "advance")
    run(rules, "insert", {"part": "pin", "skip": True})
    done = run(rules, "advance")
    assert done["done"] and done["station"] == "none" and len(done["truth"]) == 2
    assert run(rules, "advance") == {"ok": False, "reason": "batch_done"}
    assert rules.readings()["status"] == "done"


COURSE = Path(os.environ.get("VIAM_103", "/home/shrews/viam/wt-103-cv/viam-103")) / "exercises"


@pytest.mark.skipif(not (COURSE / "offline_cell.py").exists(), reason="no viam-103 checkout")
def test_same_truth_as_the_course_fake():
    pytest.importorskip("cv2")
    sys.path.insert(0, str(COURSE))
    import offline_cell
    for seed in (103, 7, 2026):
        for difficulty in ("easy", "standard", "hard"):
            for scope in ("all", "infeed", "inside", "paint", "pin"):
                fake = [s["truth"] for s in offline_cell.draw_batch(seed, difficulty, 12, scope)]
                assert draw_batch(seed, difficulty, 12, scope) == fake


def test_a_grasp_aimed_where_the_cup_drops_the_shell_slips():
    """Along the belt the cup carries only from 737 - 70 to 737 + 150 (measured): an aim past
    that slips, though it is on the wall's face."""
    for x, want in ((737.0, "held"), (667.0, "held"), (887.0, "held"),
                    (657.0, "slipped"), (637.0, "slipped"), (900.0, "slipped")):
        rules = started()
        run(rules, "advance")
        run(rules, "route", {"station": "infeed", "verdict": "line"})
        run(rules, "route", {"station": "paint", "verdict": "fail"})
        got = run(rules, "grasp", {"target": {"x": x, "y": -80.0, "z": 885.0}})
        assert got["result"] == want, (x, got)


def test_an_inside_defect_draws_one_of_five_looks_and_no_other_draw_moves():
    """Which shells are defective is the draw's as before (seed 103: shells 5 and 10 of the
    whole line, 0 and 7 inside-scoped); only their look comes from the new stream."""
    from isaac_module.cell_rules import INTERIOR_DRAWN, INTERIOR_PLACEMENTS
    for scope, size, defective in (("all", 12, [5, 10]), ("inside", 8, [0, 7])):
        batch = draw_batch(103, "standard", size, scope)
        assert [i for i, s in enumerate(batch) if s["inside"] == "defect"] == defective
        for s in batch:
            look = (s["interior_defect"], s["interior_placement"])
            if s["inside"] != "defect":
                assert look == ("none", "none")
            else:
                assert look[0] in INTERIOR_DRAWN and look[1] in INTERIOR_PLACEMENTS[look[0]]
    looks = {(s["interior_defect"], s["interior_placement"])
             for seed in range(80) for s in draw_batch(seed, "standard", 12, "inside")}
    assert looks == {("none", "none")} | {(k, p) for k, ps in INTERIOR_PLACEMENTS.items()
                                          for p in ps}

