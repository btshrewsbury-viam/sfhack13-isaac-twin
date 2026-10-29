# Merging erh/viam-isaac-sim main into `gripper`

Not done yet. This is what it will cost, recorded while someone who has already done it
still remembers — the xarm6 session merged upstream into `xarm6-package-shelf` (ffe6f29,
local only) at Brandon's request and hit five conflicts. `gripper` will hit the same five,
because both branches diverge from the same point.

Written down because a merge nobody has notes for is a merge that gets attempted twice.

## The one that breaks this cell outright

**Upstream deletes `run.sh`.** It is generated now, by `make run.sh`, and the generated
launcher **ignores `ISAAC_PYTHON`**.

This cell cannot start without both:

- `probes/assembly/build_fragment.py` writes `executable_path:
  /home/shrews/viam/viam-isaac-sim/run.sh` into every fragment
- `.scratch/push_config.py` writes the same path **and** passes
  `env: {ISAAC_PYTHON: …/.venv-isaac/bin/python}`, which is how the module gets an
  interpreter that has `isaacsim` in it. Mock mode uses this repo's own venv; real Isaac
  cannot.

So after merging: run `make run.sh`, and restore the interpreter override — the xarm6
session put it back in `run.nodetect.sh`. Until both are done the cell will come up in
mock or not at all, and the symptom will be a world that boots and has no physics.

**And the variable is renamed: `ISAAC_PYTHON` → `VIAM_ISAAC_PYTHON`.** Not cosmetic. Viam's
new sim sandbox is built from an Isaac Lab image which sets `ISAAC_PYTHON=/isaac-sim/
python.sh` globally, so a module reading the unprefixed name is handed an interpreter with
no `viam-sdk` in it and dies on `import viam`. Measured by the xarm6 session in that
sandbox.

`.scratch/push_config.py` passes the old name today. Change it in the same commit as
`make run.sh`, or the launcher falls through to `uv run` with Isaac 5.0 — which is a
different wrong answer, and a quieter one.

## The other four, as they resolved them

| Conflict | Resolution |
|---|---|
| `models/arm.py` | Keep this branch's tracked executor and stall detection; drop upstream's streamed executor (their #6) and its densify/attribute tests. Take upstream's new `Arm` stubs and `IsMoving` from commanded intent (#8) via a counter on our two move methods. |
| `requirements.txt` | `viam-sdk>=0.82`. Upstream's #7 stubs lift the `==0.80.0` pin — which is pinned here deliberately, because 0.82 adds five abstract methods to `Arm`. |
| `models/world.py`, `sim_manager.py` | Keep this branch's ground/lighting system. Upstream's `isaac_default_environment=false` maps to ground `"none"` plus its fallback lights. |
| `tests/test_arm_is_moving.py` (upstream, new) | It stops the process-wide mock sim at teardown, which hangs every later test module. Make it start-if-needed/never-stop and give its arm its own name. |

Their result: 149 pass, plus this branch's 2 pre-existing `test_taught_program` failures.

## Order of work

Do it when the cell is **not** mid-measurement. The merge touches `arm.py`'s executor and
stall detection, which is what every taught replay and every "did not settle" diagnosis in
this cell runs through, so a merge lands squarely on the machinery the insertion numbers
were measured with. Re-teach and re-measure arrival after merging rather than assuming the
numbers carry: 0.01 mm in the pallet and 0.49 mm on the belt are claims about this
executor.
