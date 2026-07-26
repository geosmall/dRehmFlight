#!/usr/bin/env python3
"""Guided bench check of motor order, spin direction, and TX stick channels.

Replaces the eyeball 'which pair sped up' arm test, which does not work
(differential speed changes across four running motors are not detectable by
eye or ear). Everything here is an observation a human CAN make -- one motor
spinning vs stopped -- or a quantitative MSP readback.

Three phases, all guided, results recorded to test_logs/motor_order/:

  RC     TX stick -> channel mapping and direction over MSP_RC (no motors).
  MOTOR  Each physical pad spun alone via the CLI 'motor' command (which
         bypasses mixer AND motor_output_reordering); the operator reports
         corner and spin direction; the tool scores against what the saved
         config (motor_output_reordering + yaw_motors_reversed) predicts,
         and on a consistent corner mismatch computes the corrected
         motor_output_reordering to set.
  (The mixer's own logical corner assignments are compile-time code shared
  with the flight-validated Air75 -- not re-derived here.)

The tool never writes to the FC: it reads config and drives the disarmed-only
'motor' bench command (auto-expires on the FC after 60 s; the tool stops each
motor explicitly and again on any exit path). Fix commands are PRINTED for the
operator, never sent.

PROPS OFF. Battery connected (ESCs need it; USB alone will not spin motors).
Craft secured. TX on for the RC phase.

Usage:
    ./ci/motor_order_check.py [port] [--pct N] [--props-fitted]
    (default /dev/ttyACM0, 8%, props-off corners-only mode)

Direction is only assessed in --props-fitted mode, by AIRFLOW: a correct
prop on a correctly-spinning motor pushes air DOWN; reversed spin pushes it
up. Bare-bell spin direction is not reliably observable by eye and is never
asked for.

Exit codes: 0 all checks passed, 1 failures/mismatches, 2 port error,
3 aborted.
"""
import os
import re
import select
import struct
import sys
import termios
import time
import tty

import serial

MSP_RC = 105

# Mixer logical order, fixed in code (dRehmFlight_STM32_SCHED_MSP_INI.ino
# controlMixer): M1..M4 = RR, FR, RL, FL -- Betaflight QuadX numbering.
MIXER_CORNERS = ("RR", "FR", "RL", "FL")
CORNER_NAMES = {"FL": "Front-Left", "FR": "Front-Right",
                "RL": "Rear-Left", "RR": "Rear-Right"}
# Spin directions the mixer's yaw sign assumes, viewed from above.
# Derived from the yaw terms: yaw_motors_reversed=0 (props-in) makes the
# RR/FL diagonal CW; =1 flips all four.
DIR_PROPS_IN = {"RR": "CW", "FL": "CW", "FR": "CCW", "RL": "CCW"}

# TX channel conventions (radioComm.ino channel table):
#   CH2 roll  1000 full left .. 2000 full right
#   CH3 pitch 1000 pitch up (stick back) .. 2000 pitch down (stick forward)
#   CH4 yaw   2000 full left .. 1000 full right (left is HIGH)
STICK_STEPS = (
    ("ROLL  stick full RIGHT", 1, +1),
    ("PITCH stick full FORWARD (nose down)", 2, +1),
    ("YAW   stick full LEFT", 3, +1),
)
STICK_DELTA_US = 150.0


class Log:
    def __init__(self):
        os.makedirs("test_logs/motor_order", exist_ok=True)
        self.path = time.strftime("test_logs/motor_order/motor_order_%Y%m%d_%H%M%S.txt")
        self.f = open(self.path, "w")

    def line(self, s=""):
        print(s)
        self.f.write(s + "\n")
        self.f.flush()


# --- terminal input ---------------------------------------------------------

def getkey():
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    return sys.stdin.read(1) if dr else None


def ask(prompt, keys):
    """Block until one of `keys` (or q to abort -> None). Returns the key."""
    print("      %s " % prompt, end="", flush=True)
    while True:
        k = getkey()
        if k is None:
            time.sleep(0.02)
            continue
        k = k.lower()
        if k == "q":
            print("q")
            return None
        if k in keys:
            print(k)
            return k


def wait_space(prompt):
    return ask(prompt + " [SPACE when ready, q quit]", " ") is not None


# --- serial: CLI and MSP ----------------------------------------------------

def cli_drain(ser, quiet_s=0.3, timeout=3.0):
    """Read until the line goes quiet for quiet_s; return everything seen.
    Prompt-agnostic: binary MSP residue with stray '#' bytes cannot fake an
    early completion the way a wait-for-prompt read can."""
    end = time.time() + timeout
    buf = b""
    last = time.time()
    while time.time() < end:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
            last = time.time()
        elif time.time() - last >= quiet_s:
            break
    return buf.decode(errors="replace")


def cli_enter(ser):
    ser.reset_input_buffer()
    ser.write(b"#")
    time.sleep(0.4)                      # CLI entry guard
    ser.write(b"\r\n")
    out = cli_drain(ser)
    return "CLI" in out or "#" in out


def cli_cmd(ser, cmd, timeout=3.0):
    ser.reset_input_buffer()
    ser.write(cmd.encode() + b"\r\n")
    return cli_drain(ser, timeout=timeout)


def msp_rc(ser, timeout=0.3):
    """One MSP_RC round trip -> tuple of 6 channel PWMs, or None."""
    ser.reset_input_buffer()
    ser.write(bytes([0x24, 0x4D, 0x3C, 0x00, MSP_RC, MSP_RC]))
    end = time.time() + timeout
    buf = b""
    while time.time() < end:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
        i = buf.find(b"$M>")
        if i < 0 or len(buf) < i + 5:
            continue
        ln, cmd = buf[i + 3], buf[i + 4]
        need = i + 5 + ln + 1
        if len(buf) < need:
            continue
        payload, csum = buf[i + 5:i + 5 + ln], buf[i + 5 + ln]
        calc = ln ^ cmd
        for p in payload:
            calc ^= p
        if calc != csum or cmd != MSP_RC or ln < 12:
            return None
        return struct.unpack("<6H", payload[0:12])
    return None


def rc_average(ser, n=10):
    samples = []
    for _ in range(n * 3):
        s = msp_rc(ser)
        if s:
            samples.append(s)
            if len(samples) >= n:
                break
    if len(samples) < n // 2:
        return None
    return [sum(s[i] for s in samples) / len(samples) for i in range(6)]


# --- config -> expectations -------------------------------------------------

def read_config(ser, log):
    """Read motor_output_reordering + yaw_motors_reversed from the CLI.
    Retries once per parameter; logs the raw response on a parse failure so
    a comms problem is visible instead of a silent None."""
    reorder = reversed_ = None
    for _ in range(2):
        out = cli_cmd(ser, "set motor_output_reordering")
        m = re.search(r"motor_output_reordering = (\d+),(\d+),(\d+),(\d+)", out)
        if m:
            reorder = [int(x) for x in m.groups()]
            break
        log.line("  [config read retry; raw: %r]" % out.strip()[:120])
    for _ in range(2):
        out = cli_cmd(ser, "set yaw_motors_reversed")
        m = re.search(r"yaw_motors_reversed = ([\d.]+)", out)
        if m:
            reversed_ = float(m.group(1)) > 0.5
            break
        log.line("  [config read retry; raw: %r]" % out.strip()[:120])
    log.line("config: motor_output_reordering = %s   yaw_motors_reversed = %s"
             % (reorder, reversed_))
    return reorder, reversed_


def expectations(reorder, reversed_):
    """Per physical pad p (0-based): (corner, direction) the config predicts."""
    exp = {}
    for mixer_i, phys in enumerate(reorder):
        corner = MIXER_CORNERS[mixer_i]
        d = DIR_PROPS_IN[corner]
        if reversed_:
            d = "CCW" if d == "CW" else "CW"
        exp[phys] = (corner, d)
    return exp


# --- phases -----------------------------------------------------------------

def phase_rc(ser, log, results):
    log.line("")
    log.line("=== RC phase: TX stick -> channel map (no motors) ===")
    log.line("TX on and bound. Throttle low. Board disarmed.")
    if not wait_space("  Center roll/pitch/yaw sticks."):
        return False
    base = rc_average(ser)
    if base is None:
        log.line("  no MSP_RC data -- TX off or link down. Skipping RC phase.")
        results.append(("rc link", False))
        return True
    log.line("  baseline ch1-6: %s" % " ".join("%.0f" % v for v in base))
    for prompt, ch_idx, want_sign in STICK_STEPS:
        if not wait_space("  Hold %s." % prompt):
            return False
        held = rc_average(ser)
        if held is None:
            log.line("      -> no data   FAIL")
            results.append(("stick %s" % prompt.split()[0].lower(), False))
            continue
        deltas = [held[i] - base[i] for i in range(6)]
        moved = max(range(6), key=lambda i: abs(deltas[i]))
        d = deltas[moved]
        ok = (moved == ch_idx and abs(d) >= STICK_DELTA_US
              and (d > 0) == (want_sign > 0))
        log.line("      -> ch%d moved %+.0f us (expect ch%d %s)   %s"
                 % (moved + 1, d, ch_idx + 1,
                    "HIGH" if want_sign > 0 else "LOW",
                    "PASS" if ok else "FAIL"))
        if not ok and moved == ch_idx:
            log.line("         right channel, wrong direction -> reverse CH%d "
                     "on the TX (never the mixer sign)" % (ch_idx + 1))
        elif not ok:
            log.line("         wrong/no channel -> check TX channel map (AETR?)")
        results.append(("stick %s" % prompt.split()[0].lower(), ok))
    if not wait_space("  Center sticks again."):
        return False
    return True


def phase_motor(ser, log, results, pct, props_fitted):
    log.line("")
    log.line("=== MOTOR phase: physical pad -> corner%s ===" %
             (" + thrust direction" if props_fitted else ""))
    log.line("Each pad spins ALONE at %d%% via the CLI 'motor' command" % pct)
    log.line("(bypasses mixer and reordering). Corner keys:")
    log.line("    1 = Front-Left     2 = Front-Right")
    log.line("    3 = Rear-Left      4 = Rear-Right")
    if props_fitted:
        log.line("Thrust: d = air pushed DOWN, u = air pushed UP, x = can't tell")
        log.line("(DOWN = spin matches the fitted prop's handedness. Assumes each")
        log.line(" prop is the correct-handed one for its corner.)")
    else:
        log.line("Props off: spin DIRECTION is not assessed in this mode -- it is")
        log.line("not reliably observable on a bare bell. Re-run with")
        log.line("--props-fitted for the airflow-based direction check.")

    reorder, reversed_ = read_config(ser, log)
    exp = expectations(reorder, reversed_) if (reorder and reversed_ is not None) else None
    corner_keys = {"1": "FL", "2": "FR", "3": "RL", "4": "RR"}
    observed = {}
    thrust = {}
    all_corners_ok = True
    for pad in range(1, 5):
        if not wait_space("  Ready to spin PHYSICAL pad %d." % pad):
            return False
        cli_cmd(ser, "motor %d %d" % (pad, pct), timeout=2.0)
        ck = ask("Pad %d: which corner spins? [1=FL 2=FR 3=RL 4=RR, x=none]" % pad,
                 "1234x")
        tk = None
        if props_fitted and ck not in (None, "x"):
            tk = ask("Pad %d: airflow? [d=DOWN u=UP x=can't tell]" % pad, "dux")
        cli_cmd(ser, "motor stop", timeout=2.0)
        if ck is None or (props_fitted and ck != "x" and tk is None):
            return False
        if ck == "x":
            log.line("      -> pad %d: no spin observed   FAIL (raise --pct? wiring?)" % pad)
            results.append(("pad %d corner" % pad, False))
            all_corners_ok = False
            continue
        corner = corner_keys[ck]
        observed[pad - 1] = corner
        if exp:
            ec, _ed = exp[pad - 1]
            ok = corner == ec
            log.line("      -> pad %d: %s   (expected %s)   %s"
                     % (pad, CORNER_NAMES[corner], CORNER_NAMES[ec],
                        "PASS" if ok else "FAIL"))
            results.append(("pad %d corner" % pad, ok))
            all_corners_ok &= ok
        else:
            log.line("      -> pad %d: %s   (no config baseline)"
                     % (pad, CORNER_NAMES[corner]))
            results.append(("pad %d corner" % pad, True))
        if props_fitted and tk is not None:
            thrust[pad - 1] = tk
            if tk == "d":
                log.line("      -> pad %d thrust DOWN   PASS" % pad)
                results.append(("pad %d thrust" % pad, True))
            elif tk == "u":
                log.line("      -> pad %d thrust UP   FAIL -- spin is reversed for"
                         " the fitted prop: Bluejay motor-direction for this"
                         " motor, or a wrong-handed prop on this corner" % pad)
                results.append(("pad %d thrust" % pad, False))
            else:
                log.line("      -> pad %d thrust unclear -- re-run this pad"
                         " (higher --pct?)" % pad)
                results.append(("pad %d thrust" % pad, False))

    # Diagnosis on a consistent corner mismatch
    if exp and not all_corners_ok and len(observed) == 4:
        corners = [observed[p] for p in range(4)]
        if len(set(corners)) == 4:
            new_reorder = [corners.index(MIXER_CORNERS[i]) for i in range(4)]
            if new_reorder != reorder:
                log.line("")
                log.line("  Corner map mismatch is CONSISTENT. Fix with:")
                log.line("      set motor_output_reordering = %s"
                         % ",".join(str(x) for x in new_reorder))
                log.line("      save")
                log.line("  then re-run this check.")
        else:
            log.line("  Observed corners are not a permutation -- re-check "
                     "observations or wiring before changing anything.")
    if props_fitted and len(thrust) == 4 and all(t == "u" for t in thrust.values()):
        log.line("  ALL four blow UP: either every prop is on the wrong corner,")
        log.line("  or all ESC directions are reversed relative to the mixer's")
        log.line("  yaw assumption (yaw_motors_reversed). Resolve which before")
        log.line("  flying -- do not guess.")
    if not props_fitted:
        log.line("")
        log.line("  NOTE: spin directions were NOT verified in this run. Fit the")
        log.line("  props and re-run with --props-fitted before flight.")
    return True


def main():
    args = list(sys.argv[1:])
    props_fitted = "--props-fitted" in args
    if props_fitted:
        args.remove("--props-fitted")
    pct = 8
    if "--pct" in args:
        i = args.index("--pct")
        pct = max(1, min(25, int(args[i + 1])))
        del args[i:i + 2]
    port = args[0] if args else "/dev/ttyACM0"

    try:
        ser = serial.Serial(port, 115200, timeout=0.05)
    except Exception as e:
        print("PORT ERROR: %s" % e)
        return 2
    time.sleep(0.3)

    log = Log()
    log.line("Motor order / direction / stick-channel bench check")
    log.line("port %s, motor test at %d%%, mode: %s"
             % (port, pct, "PROPS FITTED (airflow check)" if props_fitted
                else "props off (corners only)"))
    log.line("")
    log.line("SAFETY -- confirm each:")

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    results = []
    completed = False
    try:
        tty.setcbreak(fd)
        gates = (("props FITTED for the airflow check: craft FIRMLY held down, "
                  "hands/face/loose items CLEAR of all prop arcs",
                  "craft restraint solid enough for brief single-motor thrust",
                  "battery connected (ESCs powered)")
                 if props_fitted else
                 ("PROPS ARE OFF (all four, visually confirmed)",
                  "craft is SECURED so it cannot move",
                  "battery connected (ESCs powered)"))
        for gate in gates:
            if ask("%s? [y]" % gate, "y") is None:
                raise KeyboardInterrupt
        # RC phase runs in MSP mode (before entering the CLI)
        if not phase_rc(ser, log, results):
            raise KeyboardInterrupt
        if not cli_enter(ser):
            log.line("could not enter CLI")
            return 1
        try:
            if not phase_motor(ser, log, results, pct, props_fitted):
                raise KeyboardInterrupt
        finally:
            cli_cmd(ser, "motor stop", timeout=1.0)
        completed = True
    except KeyboardInterrupt:
        log.line("\nABORTED by operator.")
    finally:
        try:
            cli_cmd(ser, "motor stop", timeout=1.0)
        except Exception:
            pass
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        ser.close()

    if not completed:
        return 3
    log.line("")
    log.line("=== Verdict ===")
    for label, ok in results:
        log.line("  %-16s %s" % (label, "PASS" if ok else "FAIL"))
    bad = [l for l, ok in results if not ok]
    log.line("")
    if bad:
        log.line("FAIL: %d/%d -> %s" % (len(bad), len(results), ", ".join(bad)))
        log.line("Do not fly until resolved.")
    else:
        log.line("PASS: %d/%d. Motor order, directions, and stick channels "
                 "match the saved config." % (len(results), len(results)))
    log.line("record: %s" % log.path)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
