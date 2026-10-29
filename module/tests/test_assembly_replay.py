"""The taught-program guard, checked without a machine.

Replay does no planning and no collision checking. That is safe for exactly the reason a
taught robot program is safe - the path was validated when it was taught - and it is safe
ONLY while two things hold: the program was taught against this cell, and the cycle asks
for the legs it was taught for, in the same order. Both are claims the code makes in its
docstrings, so both are checked here. A program that silently replays the wrong path drives
an arm at where the belt used to be, which is the one failure a planned cycle cannot have.
"""

import asyncio
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "probes"))

from assembly.replay import Legs, fly                               # noqa: E402

PATH = [[0.0] * 6, [1.0] * 6, [2.0, 1.0, 1.0, 1.0, 1.0, 1.0]]


class _Arm:
    """Records the do_commands a leg makes, and answers them the way the module does."""

    def __init__(self, path=None):
        self.calls = []
        self._path = path

    async def do_command(self, cmd):
        self.calls.append(dict(cmd))
        if cmd.get("command") == "close_recording":
            return {"path": self._path if self._path is not None else PATH}
        return {}


class _Motion:
    def __init__(self):
        self.moves = 0

    async def move(self, **kwargs):
        self.moves += 1


def _fly(legs, arm_client, motion, **kw):
    return asyncio.run(fly(legs, {"arm-b": arm_client}, motion, "arm-b",
                           (1.0, 2.0, 3.0), {}, None, **kw))


# ---- teaching ------------------------------------------------------------------------

def test_teaching_plans_the_leg_and_keeps_what_it_recorded():
    legs, arm, motion = Legs.teach("abc"), _Arm(), _Motion()
    out = _fly(legs, arm, motion, label="pin_pick:at")
    assert out["how"] == "taught" and motion.moves == 1
    assert legs.paths["001:arm-b"] == PATH
    assert legs.labels["001:arm-b"] == "pin_pick:at"
    assert [c["command"] for c in arm.calls] == ["record", "close_recording"]


def test_teaching_refuses_a_leg_that_recorded_nothing():
    """A leg with no waypoints is nothing to replay, and says so at teaching time."""
    legs, arm, motion = Legs.teach("abc"), _Arm(path=[]), _Motion()
    with pytest.raises(RuntimeError, match="recorded no waypoints"):
        _fly(legs, arm, motion, label="pin_pick:at")


def test_a_taught_leg_is_replayed_even_on_a_teaching_pass():
    """Re-planning a leg already taught gives one step number two different answers."""
    legs, arm, motion = Legs.teach("abc"), _Arm(), _Motion()
    _fly(legs, arm, motion)
    legs.step = 0
    out = _fly(legs, arm, motion)
    assert out["how"] == "replayed" and motion.moves == 1


# ---- replay --------------------------------------------------------------------------

def test_replay_drives_the_taught_path_and_plans_nothing():
    legs = Legs(paths={"001:arm-b": PATH})
    arm, motion = _Arm(), _Motion()
    out = _fly(legs, arm, motion)
    assert out["how"] == "replayed" and motion.moves == 0
    assert arm.calls == [{"command": "replay", "path": PATH}]


def test_replay_refuses_a_leg_the_program_does_not_have():
    """The numbering IS the claim: a cycle that asks for a leg it was not taught stops."""
    legs = Legs(paths={"002:arm-b": PATH})
    with pytest.raises(RuntimeError, match="no leg 001:arm-b"):
        _fly(legs, _Arm(), _Motion(), label="pin_pick:at")


def test_no_program_plans_everything_and_records_nothing():
    arm, motion = _Arm(), _Motion()
    out = _fly(None, arm, motion)
    assert out["how"] == "planned" and motion.moves == 1 and arm.calls == []


# ---- the layout the program was taught against ---------------------------------------

def test_load_refuses_a_program_taught_against_another_cell(tmp_path, monkeypatch):
    import assembly.replay as replay

    saved = tmp_path / "assembly-cycle-program.json"
    saved.write_text(json.dumps({"layout": "OLD", "paths": {"001:arm-b": PATH}}))
    monkeypatch.setattr(replay, "PROGRAM", saved)
    with pytest.raises(RuntimeError, match="taught against layout OLD"):
        Legs.load("NEW")
    assert Legs.load("OLD").paths["001:arm-b"] == PATH


def test_load_says_so_when_the_cell_has_not_been_taught(tmp_path, monkeypatch):
    import assembly.replay as replay

    monkeypatch.setattr(replay, "PROGRAM", tmp_path / "nothing-here.json")
    with pytest.raises(FileNotFoundError, match="teach the cycle"):
        Legs.load("abc")


def test_travel_is_the_joint_distance_the_program_commands():
    legs = Legs(paths={"001:arm-b": PATH})
    # 6 joints move 1.0, then one joint moves 1.0.
    assert legs.travel() == pytest.approx(7.0)
