"""What the taught program costs, step by step. Reads the file; needs no machine.

    .venv/bin/python probes/program_report.py

"The arm movements are still wonk" is the observation this exists to turn into numbers.
Per step: how far the tool had to go, what the joints spent getting it there, and which
joints did the spending. A step whose travel is out of proportion to its distance is
doing something other than the move - reaching the same place through a different arm
shape, or taking a quarter turn the long way because a wrist ran into its limit - and
neither shows up as a failure, because the part still arrives.
"""
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests"))

from qc.round import Program  # noqa: E402
from test_taught_program import TRAVEL_BASE_DEG, TRAVEL_PER_MM, runs  # noqa: E402

PROGRAM = Path(__file__).resolve().parents[1] / "fragments" / "qc-cell-program.json"
JOINTS = ("pan", "lift", "elbow", "w1", "w2", "w3")


def main() -> int:
    program = Program.from_json(json.loads(PROGRAM.read_text()))
    worst = 0.0
    total = 0.0
    seen = set()
    for arm in ("arm-a", "arm-b"):
        for branch, sequence in runs(program, arm):
            print(f"\n  {arm} / {branch}")
            print(f"    {'step':<18} {'n':>3} {'mm':>5} {'deg':>5} {'budget':>6}   joints")
            previous = None
            for key in sequence:
                path = program.paths[key]
                travel = sum(max(abs(a - b) for a, b in zip(x, y))
                             for x, y in zip(path, path[1:]))
                spent = [sum(abs(y[j] - x[j]) for x, y in zip(path, path[1:]))
                         for j in range(6)]
                target = program.targets.get(key)
                distance = (0.0 if previous is None or target is None
                            else math.dist(previous, target))
                budget = TRAVEL_BASE_DEG + TRAVEL_PER_MM * distance
                if target is not None:
                    previous = target
                if key not in seen:
                    seen.add(key)
                    total += travel
                worst = max(worst, travel)
                busiest = ", ".join(f"{JOINTS[j]} {spent[j]:.0f}" for j in range(6)
                                    if spent[j] > 20)
                flag = "  OVER" if travel > budget else ""
                print(f"    {key:<18} {len(path):>3} {distance:>5.0f} {travel:>5.0f} "
                      f"{budget:>6.0f}{flag}   {busiest}")
    print(f"\n  {len(program.paths)} moves, {sum(len(p) for p in program.paths.values())}"
          f" waypoints, {total:.0f} degrees of joint travel over both endings, "
          f"worst single step {worst:.0f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
