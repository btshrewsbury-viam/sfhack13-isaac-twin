# Queued changes, written and tested, not yet applied

Each file here is a one-shot Python script that edits the repo in place. Run it with the
project venv from the repo root:

    .venv/bin/python queued/apply_cv.py

They are scripts rather than patch files because each one asserts the exact text it is
replacing and refuses if the tree has moved under it - a patch that half-applies to this
cell is worse than one that does not apply at all. Re-check them all at once with:

    .venv/bin/python -c "import pathlib,importlib.util
    for n in ['apply_cv','apply_cv_cycle','apply_prop_orientation','apply_drop_pin_stops']:
        sp=importlib.util.spec_from_file_location(n, f'queued/{n}.py')
        m=importlib.util.module_from_spec(sp); sp.loader.exec_module(m)"

**Every one of these changes the layout fingerprint, so each costs a full re-teach (about
eight minutes).** Apply them in as few pushes as the dependencies allow, and only once the
cycle is running - a vision loop debugged on top of a broken cycle is two faults at once.

| script | what it does | blocked on |
| --- | --- | --- |
| `apply_cv.py` | adds the `pin_bore_look` station (RealSense on the bore's published axis, dropped 11 degrees so the leaf plate does not occlude it) **and** moves the film camera to the assemble side | nothing - apply first |
| `apply_cv_cycle.py` | wires the camera into the insert: cycle entry, Jacobian measured at teach time, lateral correction from the frame | `apply_cv.py` |
| `test_bore_origin.py` | goes to `tests/` with the above; checks the frame-from-feature algebra | `apply_cv_cycle.py` |
| `apply_prop_orientation.py` | lets a prop be spawned at a full orientation, not just a yaw - needed for the door lying flat | nothing, but it is a `sim_manager` change so the module restarts |
| `apply_drop_pin_stops.py` | removes the pin's end stops and corner posts, which do nothing and sit 7 mm from a coaxial pick's tool axis | only needed if the planner refuses the pin pick |
| `apply_cameras.py` | the fisheye's real optics. **Half of it is guarded off**: the 185 degree field cannot be expressed as a pinhole FOV and needs `OmniLensDistortionFthetaAPI` in `sim_manager` first | a module change |

## The honest caveat on the CV work

With the insert now placed from a measured hold and re-referenced to the shell's measured
frame, the lateral error at the bore is already small - so a camera correction may read
near zero and change nothing. `apply_cv_cycle.py` is written so the camera is
load-bearing anyway: the bore's LATERAL position comes from the frame and nothing else,
its depth from the belt index, and the shell's rotation from the simulator, because a
single centroid cannot measure a tilt. That is a deliberate choice to teach from, not an
accident - but it does mean declining a more accurate number that is sitting right there,
and it is worth confirming with Brandon before shipping it as the course's vision lesson.
