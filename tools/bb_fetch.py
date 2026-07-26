#!/usr/bin/env python3
"""Fetch the in-RAM blackbox from dRehmFlight over USB-CDC. Read-only.

Enters the CLI, reads the record count from 'bb' status, runs 'bb dump', and
streams every row to a timestamped CSV under test_logs/blackbox/ until the
BB_END sentinel — then verifies the row count against what the board said it
holds. Run after landing + disarm (the ring freezes at disarm and holds the
last ~20 s of flight at 100 Hz). Dump BEFORE removing power (the ring is RAM)
and before any sustained re-arm (a 2 s re-arm wipes it).

Exit status: 0 on a complete dump, 1 on truncation/abort/refusal.

Usage:  ./ci/bb_fetch.py [port]      (default /dev/ttyACM0)
"""
import os
import re
import serial
import sys
import time

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"


def main():
    p = serial.Serial(PORT, 115200, timeout=3)
    time.sleep(0.2)
    p.write(b"#")             # enter CLI (guard delay applies)
    time.sleep(0.4)
    p.reset_input_buffer()

    # Ask the board how many records it holds, so completeness is checkable.
    expected = None
    p.write(b"bb\r\n")
    deadline = time.time() + 2.0
    while time.time() < deadline:
        line = p.readline().decode(errors="replace").strip()
        m = re.search(r"Blackbox: (\d+) records", line)
        if m:
            expected = int(m.group(1))
            break
    p.reset_input_buffer()
    p.write(b"bb dump\r\n")

    os.makedirs("test_logs/blackbox", exist_ok=True)
    path = time.strftime("test_logs/blackbox/bb_%Y%m%d_%H%M%S.csv")
    rows = 0
    got_header = False
    complete = False
    with open(path, "w") as f:
        while True:
            line = p.readline().decode(errors="replace").strip()
            if not line:
                if got_header:      # timeout after data started: assume done
                    print("WARNING: timeout before BB_END — dump may be incomplete")
                    break
                continue            # still waiting for output to start
            if line == "BB_END":
                complete = True
                break
            if line == "BB_ABORT":
                print("WARNING: FC aborted the dump (host stopped draining USB)")
                break
            if line.startswith("t_ms,"):
                got_header = True
                f.write(line + "\n")
                continue
            if got_header and line[0].isdigit():
                f.write(line + "\n")
                rows += 1
            # anything else (echo, prompt, refusals) is skipped — but surface refusals
            if "Refused" in line:
                print(f"FC refused: {line}")
                break
    p.write(b"exit\r\n")
    p.close()

    if expected is not None:
        verdict = "COMPLETE" if (rows == expected and complete) else "INCOMPLETE"
        print(f"{rows}/{expected} rows ({verdict}) -> {path}")
        return 0 if verdict == "COMPLETE" else 1
    print(f"{rows} rows (count unverified) -> {path}")
    return 0 if complete else 1


if __name__ == "__main__":
    sys.exit(main())
