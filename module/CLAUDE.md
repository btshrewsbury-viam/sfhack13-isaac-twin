# Working in this repo

This is a **course asset**, not a machine. It exists to be filmed for Viam 103. Judge
everything against that.

## Done means

A film under 120 s where a human watching sees the parts go in and the arms not collide.
Nothing else counts. Millimetres do not count. A number inside a published tolerance that
nobody can see on camera is not progress.

## Stop and ask after 5 attempts

If one sub-problem costs more than five attempts without converging, **stop** and come
back with three options and what each costs. Do not continue. A failing sub-problem is
usually a design question wearing a bug's clothes, and the design question is Brandon's
to answer, cheaply, early.

## The spec is negotiable

Asset tolerances, clearances, mechanisms and the sequence are all changeable. If one is
making the work hard, **say so before engineering around it.** Published numbers are
someone's earlier choice, not physics.

Worked example, from the week this file came out of: the pin is specified with 0.20 mm of
radial slack over a 138 mm insert through three interleaved knuckles. A week went into
making a position-controlled arm achieve that. The cycle already creates a fixed joint
when the part seats, so the precision bought nothing but the instant before the joint
snapped it. Widening the clearance and letting the joint seat the part was always
available and nobody would see the difference on film.

## Reporting

Work without reporting. Message only when **done**, **blocked**, or when a **decision is
Brandon's**. Status updates between those are interruptions that make him babysit the
session. Do not build heartbeats or monitors to work around this.

## Measuring

- **Batch instrumentation.** The feedback loop here is ~7 minutes (push, respawn, site,
  teach). Adding one reading per experiment is the single most expensive habit available.
  Print everything about a stage in one pass, then change control.
- **Never change control before measuring.** Most regressions in this repo came from a
  change that read well and was never checked against the cell first.
- **If a correction returns the same magnitude every run, it is a constant, not a loop
  working.** Look it up. A 3.91 mm error in how a station modelled the tool's grip hid for
  hours inside a correction loop that kept reporting it.
- **Split every magnitude into components that fail differently** — along an axis vs
  across it, a part's origin vs the feature that has to fit, commanded vs achieved.
  A single number hides which fault produced it.
- **Measure at the feature, not the part's origin.** An origin is wherever the asset's
  author put it; a small rotation can move it many times further than the thing that has
  to fit the hole.

## Operational traps

- `viam machines part restart` **shuts the cell down** — it runs under
  `isaac-arcade/scripts/run_guarded.sh`, not a service. Kill the module pid to restart the
  module; relaunch the guard to restart the part. `MAX_SECONDS` defaults to 1800; use
  21600.
- `pgrep -f` matches its own shell. Match `argv[1]` via `/proc/$pid/cmdline`. A bare
  `exit 144` is the tell.
- A `reset` with a joint live lands the assembly out of square. Use `fixture.restage()`.
- Never use `/tmp` on this machine. Scratch goes in `.scratch/`.
- `touch .scratch/autorun/HOLD` before staging a commit — the driver rewrites poses and
  the fragment on its own schedule.

## Documentation

Write it at the end, not during. `docs/HANDOFF.md` is the entry point and should stay
short. Do not write narrative accounts of how faults were found while there is still work
to do.
