#!/bin/bash
#
# save_tune.sh - Capture a flight-validated dRehmFlight tune with provenance.
#
# Queries MSP identity (read-only gate), captures the firmware's `diff all`
# (self-stamped provenance header + every param as `set name = value`), and
# writes it to tunes/<model>.txt. Refuses to save a tune whose firmware can't
# be tied to a clean commit (fw_git "unknown" or "-dirty") — a tune you can't
# rebuild byte-for-byte is not a backup.
#
# Safe on a powered board, props OFF: only sends '#', 'diff all', 'exit' — no
# param write, no save, no arm path.
#
# Usage:
#   tools/save_tune.sh <model> [port]
#     <model>  output basename, e.g. AIR75_BEFH-G473  -> tunes/AIR75_BEFH-G473.txt
#     [port]   serial port (default /dev/ttyACM0)
#
# (serial /dev nodes are hidden inside the sandbox).
#
# Exit codes: 0 saved · 1 capture/validation failed · 2 port/board not alive

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

if [ $# -lt 1 ]; then
    echo "Usage: $0 <model> [port]"
    echo "  e.g. $0 AIR75_BEFH-G473 /dev/ttyACM0"
    exit 1
fi

MODEL="$1"
PORT="${2:-/dev/ttyACM0}"
OUT="$ROOT_DIR/tunes/${MODEL}.txt"
TMP="$(mktemp)"
trap 'rm -f "$TMP"' EXIT

echo "=== save_tune: ${MODEL} on ${PORT} ==="

# 1. Identity / liveness gate (read-only MSP). Must pass before we touch the CLI.
echo "--- MSP identity ---"
if ! "$SCRIPT_DIR/msp_query.py" "$PORT"; then
    echo "ABORT: MSP liveness/identity gate failed (board not alive on $PORT)."
    exit 2
fi

# 2. Enter CLI (modal '#' trigger; flaky during bring-up, so retry).
echo "--- entering CLI ---"
entered=false
for attempt in 1 2 3; do
    if "$SCRIPT_DIR/serial_io.py" "$PORT" --send-raw '#' --expect 'dRehmFlight CLI' --timeout 3 --quiet; then
        entered=true
        break
    fi
    echo "  CLI enter attempt $attempt failed, retrying..."
    sleep 0.5
done
if [ "$entered" != true ]; then
    echo "ABORT: could not enter CLI after 3 attempts."
    exit 1
fi

# 3. Capture `diff all`. Use a never-match pattern so serial_io reads the full
#    window and prints every streamed line; we filter to the tune content after.
echo "--- capturing 'diff all' ---"
"$SCRIPT_DIR/serial_io.py" "$PORT" --send 'diff all' --expect '__SAVE_TUNE_NOMATCH__' --timeout 4 \
    > "$TMP" || true   # expect never matches -> exit 1 on timeout; the capture is what matters

# Return the FC to MSP mode (best effort; don't fail the save on this).
"$SCRIPT_DIR/serial_io.py" "$PORT" --send 'exit' --timeout 1 >/dev/null 2>&1 || true

# 4. Validate the capture: header present, and firmware ties to a CLEAN commit.
if ! grep -q 'fw_git:' "$TMP"; then
    echo "ABORT: no provenance header captured (CLI capture failed/empty)."
    echo "       Raw capture:"; sed 's/^/         /' "$TMP"
    exit 1
fi

FW_GIT="$(grep -m1 'fw_git:' "$TMP" | sed -E 's/.*fw_git:[[:space:]]*([^[:space:]]+).*/\1/')"
echo "--- firmware fw_git: ${FW_GIT} ---"

case "$FW_GIT" in
    *-dirty)
        echo "REFUSE: firmware is '-dirty' — built from an uncommitted tree, not reproducible."
        echo "        Commit the firmware, reflash, then re-run."
        exit 1
        ;;
    unknown|nogit|"")
        echo "REFUSE: firmware fw_git is '${FW_GIT:-empty}' — not tied to a commit."
        echo "        ('nogit'/'unknown' = built outside a git repo.) Build inside the repo,"
        echo "        reflash, then re-run so the tune ties to a commit."
        exit 1
        ;;
esac

# 5. Reconstruct the tune: header lines ('# <text>'), a blank separator line
#    (Betaflight style), then 'set ...' lines. '^# \S' keeps header content but
#    drops the bare '# ' CLI prompt; the command echo and stray lines are dropped.
mkdir -p "$ROOT_DIR/tunes"
{ grep -E '^# \S' "$TMP"; echo; grep -E '^set ' "$TMP"; } > "$OUT"

SET_LINES="$(grep -c '^set ' "$OUT" || true)"
if [ "${SET_LINES:-0}" -lt 1 ]; then
    echo "ABORT: capture had a header but no 'set' lines (incomplete diff)."
    rm -f "$OUT"
    exit 1
fi

echo "=== saved ${SET_LINES} params -> ${OUT} ==="
echo "    Commit it: git add tunes/${MODEL}.txt (git history is the tune history)."
