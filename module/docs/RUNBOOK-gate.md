# Launching the course machine (the "gate")

The course machine is a Viam machine in the Education org (machine 65ca3f3d, part 355a68f3)
whose config is fragment `viam-103-cell` (c5310da2). It runs on the GCP VM in
`scripts/cloud_vm/vm.env`, which has one L4: run one cell at a time.

## Start

From the workstation:

    bash scripts/cloud_vm/stop_cell.sh          # stops any cell on the VM (gate or dev)
    ssh $VM 'cd ~/viam/viam-isaac-sim
      echo "gate (who, why)" > .scratch/cell/HOLD
      L=.scratch/runs/gate-$(date +%H%M%S).log
      MAX_SECONDS=21600 MEM_MAX=14G setsid nohup \
        ~/viam/isaac-arcade/scripts/run_guarded.sh /opt/viam/bin/viam-server \
        -config ~/viam/viam-isaac-sim/.scratch/secrets/gate-cloud.json > $L 2>&1 < /dev/null &'

`run_guarded.sh` caps RAM, GPU and lifetime (MAX_SECONDS, 6 h); never use
`viam machines part restart` (it shuts the cell down). The registry module and the fragment
come from the cloud config; nothing local is used.

A machine in another organization (a learner's, or the rehearsal machine) runs the same way
with its own cloud config, plus `ISAAC_PYTHON` exported to an installed Isaac venv, because
its per-part module data directory has no install marker on this VM:

    export ISAAC_PYTHON=~/.viam/module-data/355a68f3-96c0-43c0-b3e2-8d50b20120c8/isaac-sim/isaac-venv/bin/python

**Which Isaac Sim runs.** The gate imports `isaacsim` 5.0.0.0, from the module's own venv
(`~/.viam/module-data/<part>/isaac-sim/isaac-venv`), which the module's `first_run.sh`
builds on a fresh part (5.0.0 on python3.11 for Ubuntu 24.04). The VM's `bootstrap.sh`
installs the same 5.0.0.0 into `~/viam/isaac-arcade/.venv-isaac`, so `ISAAC_PYTHON` can point
there on a fresh VM. VMs bootstrapped before 2026-10-20 have 6.1.0.0 in that venv; point
`ISAAC_PYTHON` at the gate part's module venv instead.

## The watch keeps it up

Cron on the VM runs `scripts/cloud_vm/watch.sh` every 10 minutes. If no process has
`gate-cloud.json` on its command line (the guard or viam-server), it relaunches the gate
exactly as above. That includes after the guard's 6 h MAX_SECONDS kill, which would otherwise
leave the gate down. It never launches if anything has `gate-cloud.json` on its command line
(a second viam-server on the gate's config deletes the gate's Isaac venv in seconds). It
never launches while another viam-server (a dev cell) is running, or while
`.scratch/cell/HOLD` exists. After 3 relaunches in an hour it writes HOLD itself. To stop
the gate on purpose, write HOLD first (or use `stop_cell.sh`); to hand it back to the watch,
remove HOLD. The log is `.scratch/cell/watch.log`.

## Wait for it

    set -a; . ~/.viam-103-env; set +a; .venv/bin/python .scratch/vmtools/gate_wait.py

prints `GATE UP` when every learner resource answers (about 2.5 min).

## Prove it

From the course's `exercises/` directory, with `~/.viam-103-env` sourced:

    python .scratch/vmtools/launch_proof.py OUT_DIR

checks the app's view (part online, last seen, its fragment), an image from every camera
(saved to OUT_DIR), a joint read from every arm, the grippers, the scoreboard, and the cell's
`reset`. 2026-10-20 on 0.1.5: online (0.4 s), 36 resources, 10 cameras 0.09-0.53 s each,
3 arms, 2 grippers, scoreboard 34 readings, reset ok.

Page 10 (batch 4) timing, OPEN since 2026-10-20: 0.1.7 ran 178-184 s on the gate (06:14Z);
0.1.8 ran 194-197 s (19:41-19:52Z, warm, and the same with the morning's starter), score 750
each time, though the module code page 10 runs is byte-identical. Suspect the path from the
client, not the release; but 0.1.9 with the client on its own VM also took 196 s. Next: re-time
when 0.1.9 is on the gate, and if still over 190 s run page 10 from a client on the gate VM.

## Trying a candidate tarball as a local module

To soak an unreleased build, a part's config can name the tarball as a local module
(`{"type": "local", "name": ..., "executable_path": "<.tar.gz>"}`). viam-server unpacks it
into `~/.viam/packages-local/data/module/synthetic-<name>-0_0_0` and does NOT unpack again
when only the tarball changes: pointing the same module name at a new tarball keeps running
the old code (2026-10-20, a "0.1.8" soak that was really 0.1.7). Give each candidate its own
module name (e.g. `isaac-sim-018`), and check the unpacked code before measuring, e.g.
`grep` for the new feature under that `synthetic-...` directory.
