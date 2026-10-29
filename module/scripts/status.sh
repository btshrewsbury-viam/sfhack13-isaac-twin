#!/usr/bin/env bash
# One line saying what this cell rebuild is doing right now.
#
# Two readers: a person who wants to know whether anything is happening without reading a
# log, and the heartbeat monitor that wakes the session. Both get the same line, so there
# is no second source to disagree with.
#
#   scripts/status.sh "teaching, attempt 3"     # write it
#   scripts/status.sh                           # read it
set -uo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
FILE="$REPO/.scratch/autorun/STATUS"
mkdir -p "$(dirname "$FILE")"

if [ $# -gt 0 ]; then
    printf '%s  %s\n' "$(date '+%H:%M:%S')" "$*" > "$FILE"
    exit 0
fi

# Is the driver actually alive? argv[1] match, not a grep over the whole command line -
# a bare `pgrep -f autorun` matches the shell doing the matching and reports itself.
alive=no
for pid in $(ls /proc 2>/dev/null | grep -E '^[0-9]+$'); do
    [ -r "/proc/$pid/cmdline" ] || continue
    mapfile -d '' -t argv < "/proc/$pid/cmdline" 2>/dev/null || continue
    case "${argv[1]:-}" in *autorun.py) alive=yes; break;; esac
done

printf 'driver=%s | %s | %s\n' \
    "$alive" \
    "$(cat "$FILE" 2>/dev/null || echo 'no status written yet')" \
    "$(tail -1 "$REPO/.scratch/autorun/driver.log" 2>/dev/null | cut -c1-110)"
