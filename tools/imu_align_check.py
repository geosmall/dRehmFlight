#!/usr/bin/env python3
"""Fleet-reusable IMU alignment bench acceptance (requirements R5). Read-only.

Verifies that a board's *physical* IMU behavior matches the alignment its
firmware believes it has. Works on any target: nothing here is board-specific,
and no expected value is hard-coded per board -- the verdicts are stated in
terms of the vehicle body frame (+X forward, +Y left, +Z up), which every
target shares by construction.

Reads MSP_RAW_IMU (102) only. No MSP SET / write / save / reboot / arm path --
safe on a powered board. RUN WITH PROPS OFF.

MSP_RAW_IMU reports POST-alignment values: dRehmFlight applies boardAlignMatrix
at the end of getIMUdata(), so what arrives here is already in the vehicle
frame. That is exactly the quantity under test.

WHY THIS IS SOUND (and why two earlier methods were not)
--------------------------------------------------------
  Phase A  Static poses. Gravity is an absolute external reference. Holding a
           known face up pins the ACCEL frame absolutely -- no dynamics, no
           integration, no operator timing.

  Phase B  Supervised sweeps. One slow ~90 deg rotation per axis in a stated
           direction chosen to be POSITIVE in the right-handed FLU frame
           (left side up = +X, nose down = +Y, nose left = +Z). The gyro's
           net per-axis rotation over the sweep must land on the intended
           axis with the intended sign. Gyro-only: translation, pivot offset,
           sample rate, and speed are all irrelevant. The operator supplies
           the direction -- exactly the trust Phase A places in "hold it
           left side up".

  A pins the accel frame absolutely (gravity as the reference); B pins the
  gyro frame absolutely (supervised direction as the reference). Run both.

Two prior Phase B designs are retired, each killed by the known-good-craft
rule (R5: prove the procedure on the Air75 before trusting it anywhere):

  1. Freehand per-axis path integration (July 2026, predates this tool):
     rotations do not commute, so a wandering path dumps real energy into
     off-axis integrals and fakes cross-axis coupling on a healthy sensor.
     The self-test still demonstrates the artifact.
  2. Gyro-vs-gravity correlation via the transport theorem
     d(g_body)/dt = -omega x g_body (this tool's first version, retired
     2026-07-21): physically correct, but its verdict was hostage to
     operator ergonomics -- pivot-offset linear acceleration, slow wrist
     rates near the tremor floor, and sampling artifacts swung r from 0.43
     to 0.87 across sessions on the SAME healthy Air75. An instrument that
     only works for a perfect operator is not an instrument.

The sweep judge keeps what survived: the only per-axis integral it takes is
over a single supervised monotone sweep with coarse decision bars (45/30
deg), an order above the 8-15 deg non-commutativity artifact, and ambiguous
motion earns a REDO, never a fault verdict.

Usage:
    ./ci/imu_align_check.py [port] [--phase a|b|all]

Exit codes:
    0  every executed check passed
    1  one or more checks failed or were inconclusive
    2  port could not be opened
    3  aborted by operator

Invoke via the ./ci/ prefix so it matches the sandbox excludedCommands entry
(serial nodes are hidden inside the sandbox; this tool needs the real /dev).
"""
import math
import random
import select
import struct
import sys
import termios
import time
import tty

import serial

MSP_RAW_IMU = 102
ACC_LSB_PER_G = 512.0          # mspComm.ino: (int16_t)(AccX * 512)

# --- verdict thresholds -----------------------------------------------------
STILL_DPS = 6.0                # all-axis gyro rate that counts as "still"
STILL_HOLD_S = 1.2             # how long it must stay still to capture a pose
G_TOL = 0.18                   # |g| may deviate this much from 1.0 at rest
POSE_DOMINANT_MIN = 0.80       # dominant axis must read at least this many g
POSE_OFFAXIS_MAX = 0.28        # every other axis must stay under this
SWEEP_ACTIVE_DPS = 15.0        # gyro rate above which a sweep is "in motion"
SWEEP_MIN_SAMPLES = 20         # need this many segment samples to judge
SWEEP_END_STILL_S = 0.6        # stillness that ends a sweep capture
SWEEP_TIMEOUT_S = 20.0         # give up waiting for / capturing a sweep
SWEEP_MIN_ANGLE = 45.0         # net rotation (deg) that counts as a real
                               # sweep -- the operator is asked for ~90
SWEEP_OFFAXIS_MAX = 30.0       # net off-axis rotation (deg) above which the
                               # sweep was too diagonal to judge -- redo, not
                               # a fault verdict

AXES = ("X", "Y", "Z")

# Phase A: (prompt, body axis pointing up, expected sign)
POSES = (
    ("LEVEL, upright  (normal flight attitude)",        2, +1),
    ("INVERTED        (flip it over, belly up)",         2, -1),
    ("NOSE UP         (stand it on its tail)",           0, +1),
    ("NOSE DOWN       (stand it on its nose)",           0, -1),
    ("LEFT SIDE UP    (roll right until left side up)",  1, +1),
    ("RIGHT SIDE UP   (roll left until right side up)",  1, -1),
)

# Phase B: (prompt, body axis, expected sign of the net gyro rotation)
#
# Each motion is chosen to be POSITIVE in the right-handed FLU body frame
# (+X forward, +Y left, +Z up): left side rising is +X, nose dropping is +Y,
# nose swinging left is +Z. The operator supplies the direction -- exactly the
# same trust Phase A places in "hold it left side up". Verified against the
# firmware: GyroX/Y/Z feed Madgwick unmodified as FLU body rates
# (dRehmFlight_STM32_SCHED_MSP_INI.ino:658,735).
SWEEPS = (
    ("ROLL  -- from level, roll LEFT SIDE UP ~90 deg, one slow sweep",  0, +1),
    ("PITCH -- from level, tip the NOSE DOWN ~90 deg, one slow sweep",  1, +1),
    ("YAW   -- stay level, swing the NOSE LEFT ~90 deg, one slow sweep", 2, +1),
)


def frame(cmd):
    """MSP V1 request: $M< len=0 cmd csum (csum of a bare request is cmd)."""
    return bytes([0x24, 0x4D, 0x3C, 0x00, cmd, cmd])


def read_raw_imu(ser, timeout=0.20):
    """One MSP_RAW_IMU round trip -> ((gx,gy,gz) deg/s, (ax,ay,az) g) or None.

    Checksum is verified; a corrupt frame returns None rather than a wrong
    reading, so a flaky link degrades into fewer samples, never bad ones.
    """
    ser.reset_input_buffer()
    ser.write(frame(MSP_RAW_IMU))
    ser.flush()
    end = time.time() + timeout
    buf = b""
    while time.time() < end:
        # Read whatever is pending (min 1 byte, blocking up to the port
        # timeout). Never ask for more than the ~24-byte reply holds: a fixed
        # read(64) blocks for the FULL port timeout on every poll, capping the
        # loop at ~13 Hz against a board that answers at 100 Hz.
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
        if calc != csum or cmd != MSP_RAW_IMU or ln < 18:
            return None
        v = struct.unpack("<9h", payload[0:18])
        acc = tuple(x / ACC_LSB_PER_G for x in v[0:3])
        gyr = tuple(float(x) for x in v[3:6])
        return gyr, acc
    return None


def getkey():
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    return sys.stdin.read(1) if dr else None


def wait_for_key(prompt):
    """Block until SPACE (continue) or q (abort). Returns False to abort."""
    print(prompt, end="", flush=True)
    while True:
        k = getkey()
        if k == " ":
            print()
            return True
        if k in ("q", "Q"):
            print("\nABORTED by operator.")
            return False
        time.sleep(0.02)


def norm(v):
    return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


# --- Phase A: static poses (pins the accel frame absolutely) ----------------

def capture_still(ser, max_wait=30.0):
    """Average accel over a stillness window. Returns (acc, n) or None.

    Stillness is judged on the GYRO (all axes under STILL_DPS) and on ||acc||
    being close to 1 g, so a pose captured while the operator is still moving
    or bracing is rejected rather than averaged in.
    """
    end = time.time() + max_wait
    still_since = None
    acc_sum = [0.0, 0.0, 0.0]
    n = 0
    while time.time() < end:
        if getkey() in ("q", "Q"):
            return None
        s = read_raw_imu(ser)
        if not s:
            continue
        gyr, acc = s
        quiet = max(abs(g) for g in gyr) < STILL_DPS and abs(norm(acc) - 1.0) < G_TOL
        if not quiet:
            still_since, acc_sum, n = None, [0.0, 0.0, 0.0], 0
            continue
        now = time.time()
        if still_since is None:
            still_since = now
        for i in range(3):
            acc_sum[i] += acc[i]
        n += 1
        if now - still_since >= STILL_HOLD_S and n >= 8:
            return tuple(a / n for a in acc_sum), n
    return None


def phase_a(ser, results):
    print("\n=== Phase A -- static poses (accel frame, gravity-referenced) ===")
    print("Hold each pose steady; capture is automatic once the board is still.")
    print("Roughly level by eye is fine -- the thresholds are generous.\n")
    for prompt, axis, sign in POSES:
        if not wait_for_key("  %-52s [SPACE to capture, q quit] " % prompt):
            return False
        cap = capture_still(ser)
        if cap is None:
            print("      -> no steady capture (still moving, or aborted)  INCONCLUSIVE")
            results.append(("pose %s%s" % ("+" if sign > 0 else "-", AXES[axis]), False))
            continue
        acc, n = cap
        want = sign * 1.0
        off = [abs(acc[i]) for i in range(3) if i != axis]
        ok = (abs(acc[axis] - want) < (1.0 - POSE_DOMINANT_MIN) + G_TOL
              and max(off) < POSE_OFFAXIS_MAX)
        dominant = max(range(3), key=lambda i: abs(acc[i]))
        print("      -> acc = (%+.2f, %+.2f, %+.2f) g   n=%d   dominant %s%s   %s"
              % (acc[0], acc[1], acc[2], n,
                 "+" if acc[dominant] > 0 else "-", AXES[dominant],
                 "PASS" if ok else "FAIL"))
        if not ok:
            print("         expected %s%s near %+.2f g with others under %.2f"
                  % ("+" if sign > 0 else "-", AXES[axis], want, POSE_OFFAXIS_MAX))
        results.append(("pose %s%s" % ("+" if sign > 0 else "-", AXES[axis]), ok))
    return True


# --- Phase B: supervised sweeps (pins the gyro frame via known directions) --

def collect_sweep(ser):
    """Capture one supervised sweep: wait for motion, record until stillness.

    Returns the in-motion sample segment (each entry (t, gyro deg/s, acc g)),
    [] if no motion arrived before the timeout, or None on operator abort.
    """
    t0 = time.time()
    buf = []
    consec = 0
    start_i = None
    last_active = None
    while True:
        if getkey() in ("q", "Q"):
            return None
        now = time.time()
        if now - t0 > SWEEP_TIMEOUT_S:
            break
        s = read_raw_imu(ser)
        if not s:
            continue
        buf.append((now, s[0], s[1]))
        active = max(abs(g) for g in s[0]) > SWEEP_ACTIVE_DPS
        if active:
            last_active = now
            if start_i is None:
                consec += 1
                if consec >= 3:
                    start_i = len(buf) - consec
        else:
            if start_i is None:
                consec = 0
            elif now - last_active > SWEEP_END_STILL_S:
                break               # sweep finished
    if start_i is None:
        return []
    end_i = max(i for i in range(len(buf))
                if max(abs(g) for g in buf[i][1]) > SWEEP_ACTIVE_DPS)
    return buf[start_i:end_i + 1]


def judge_sweep(segment, axis, want_sign):
    """Return (verdict, detail, ang): verdict 'pass' | 'fail' | 'redo', ang the
    net per-axis rotation (deg) the gyro reported over the sweep.

    The judge is gyro-only: translation, pivot offset, sample rate, and speed
    are all irrelevant. The operator supplies the direction (same trust basis
    as Phase A's "hold it left side up"); the gyro's job is to agree on which
    axis rotated and which way.

    On the per-axis trapezoid sum: this is a net-angle estimate over ONE
    supervised, monotone, single-axis sweep with coarse decision bars (45/30
    deg). The discredited freehand-path integration faked 8-15 deg of off-axis
    rotation on wandering multi-axis paths -- an order below these bars, and a
    monotone sweep has no such wander. Ambiguous motion triggers a REDO, never
    a fault verdict.
    """
    if len(segment) < SWEEP_MIN_SAMPLES:
        return ("redo", "no sustained motion captured", (0.0, 0.0, 0.0))
    ang = [0.0, 0.0, 0.0]
    for (t1, g1, _), (t2, g2, _) in zip(segment, segment[1:]):
        dt = t2 - t1
        if dt <= 0 or dt > 0.25:
            continue
        for k in range(3):
            ang[k] += 0.5 * (g1[k] + g2[k]) * dt
    main = max(range(3), key=lambda k: abs(ang[k]))
    off = max(abs(ang[k]) for k in range(3) if k != axis)
    if abs(ang[main]) < SWEEP_MIN_ANGLE:
        return ("redo", "only %.0f deg of net rotation -- sweep further (~90 deg)"
                % abs(ang[main]), tuple(ang))
    if main != axis:
        return ("fail", "the sweep registered on gyro %s, not %s -- axis mapping fault"
                % (AXES[main], AXES[axis]), tuple(ang))
    if ang[axis] * want_sign < 0:
        return ("fail", "gyro %s read %+.0f deg for a motion that must read %s -- sign fault"
                % (AXES[axis], ang[axis], "positive" if want_sign > 0 else "negative"),
                tuple(ang))
    if off > SWEEP_OFFAXIS_MAX:
        return ("redo", "%.0f deg of off-axis rotation -- keep the sweep about one axis"
                % off, tuple(ang))
    return ("pass", "", tuple(ang))


def phase_b(ser, results):
    print("\n=== Phase B -- supervised sweeps (gyro axis + sign) ===")
    print("Each step is ONE slow steady sweep of about 90 deg in the stated")
    print("direction, then hold still. Speed, smoothness, and translation do")
    print("not matter (gyro-only) -- the DIRECTION does: it is what the gyro")
    print("sign is checked against. Rotate in place; capture starts when the")
    print("board moves and ends when it stops.\n")
    for prompt, axis, want in SWEEPS:
        while True:
            if not wait_for_key("  %-58s [SPACE, then sweep; q quit] " % prompt):
                return False
            print("      waiting for the sweep ...", flush=True)
            seg = collect_sweep(ser)
            if seg is None:
                print("\n      ABORTED.")
                return False
            verdict, detail, ang = judge_sweep(seg, axis, want)
            if seg:
                dur = seg[-1][0] - seg[0][0]
                print("      -> gyro net rotation: X%+6.0f  Y%+6.0f  Z%+6.0f deg over %.1f s"
                      % (ang[0], ang[1], ang[2], dur))
            if verdict == "redo":
                print("      -> REDO: %s" % detail)
                continue
            ok = verdict == "pass"
            print("      -> %s%s" % ("PASS" if ok else "FAIL",
                                     "" if ok else ": " + detail))
            results.append(("sweep %s sign" % AXES[axis], ok))
            break
    return True


# --- self-test: prove the judgment math without hardware -------------------
#
# R5 requires the procedure itself be validated before its verdict is trusted
# anywhere. This discharges the offline half of that: it simulates supervised
# sweeps through a realistic acquisition (quantization, timing jitter, hand
# tremor, pivot-offset linear acceleration) and checks that healthy sweeps
# pass at both fast and slow link rates, that injected sign flips and axis
# swaps fail, that a wrong-direction sweep fails (operator direction IS the
# reference -- the tool cannot tell operator error from hardware fault), and
# that undersized or diagonal motion earns a REDO rather than a verdict.
# Run it after touching any threshold or judging code.

def _simulate(omega_of_t, seconds=8.0, fault=None, tilt=0.0, rate_hz=100.0,
              jitter_s=0.004, pivot_m=0.15, tremor_dps=15.0, seed=1):
    """Synthetic (t, gyro deg/s, acc g) samples through a REALISTIC acquisition.

    Truth is integrated at 1 kHz, then sampled at rate_hz with receive-time
    jitter, MSP quantization (accel 512 LSB/g, gyro 1 LSB per deg/s), hand
    tremor (2-3 Hz components on every axis -- real hands are not smooth), and
    the linear acceleration of a hand pivot offset pivot_m from the IMU
    (a_lin = alpha x r + omega x (omega x r)). The pivot term is what the
    original float-only simulation omitted, and why it passed a judging
    algorithm that failed on real hardware (2026-07-20 Air75 false failure).

    `fault` corrupts only what the gyro REPORTS, leaving the physics -- and
    therefore the accel -- untouched, which is how a real frame/sign fault
    presents."""
    rng = random.Random(seed)
    tr = math.radians(tilt)
    g = [math.sin(tr), 0.0, math.cos(tr)]
    r_vec = (pivot_m, 0.4 * pivot_m, 0.15 * pivot_m)
    tremor_f = (2.1, 2.9, 1.7)
    tremor_ph = (0.3, 1.9, 4.0)
    out = []
    dt_p = 0.001
    steps = int(seconds / dt_p)
    w_prev = None
    next_emit = 0.0
    for s in range(steps):
        t = s * dt_p
        w_true = list(omega_of_t(t))
        for k in range(3):
            w_true[k] += tremor_dps * math.sin(
                2 * math.pi * tremor_f[k] * t + tremor_ph[k])
        w_rad = [math.radians(x) for x in w_true]
        # transport gravity through the true rotation
        d = cross(w_rad, g)
        g = [g[i] - d[i] * dt_p for i in range(3)]
        m = norm(g)
        g = [x / m for x in g]
        # pivot-offset linear acceleration, in g units
        alpha = ([0.0, 0.0, 0.0] if w_prev is None else
                 [(w_rad[k] - w_prev[k]) / dt_p for k in range(3)])
        w_prev = w_rad
        a_lin = [alpha[1] * r_vec[2] - alpha[2] * r_vec[1],
                 alpha[2] * r_vec[0] - alpha[0] * r_vec[2],
                 alpha[0] * r_vec[1] - alpha[1] * r_vec[0]]
        wxr = cross(w_rad, r_vec)
        cent = cross(w_rad, wxr)
        a_meas = [g[k] + (a_lin[k] + cent[k]) / 9.81 for k in range(3)]
        if t >= next_emit:
            next_emit += 1.0 / rate_hz
            w = list(w_true)
            if fault == "flip_x":
                w[0] = -w[0]
            elif fault == "flip_y":
                w[1] = -w[1]
            elif fault == "flip_z":
                w[2] = -w[2]
            elif fault == "swap_xy":
                w[0], w[1] = w[1], w[0]
            gyr = tuple(float(round(x)) for x in w)          # 1 LSB = 1 deg/s
            acc = tuple(round(x * 512.0) / 512.0 for x in a_meas)  # 512 LSB/g
            out.append((t + rng.uniform(0.0, jitter_s), gyr, acc))
    return out


def _sweep_motion(axis, angle_deg=90.0, T=3.0):
    """One slow raised-cosine sweep: net rotation angle_deg over T seconds,
    then still. A small zero-mean off-axis wobble rides along (real hands)."""
    A = 2.0 * angle_deg / T
    def f(t):
        w = [0.0, 0.0, 0.0]
        if t < T:
            w[axis] = A * math.sin(math.pi * t / T) ** 2
            for o in range(3):
                if o != axis:
                    w[o] += 7.0 * math.sin(2 * math.pi * 0.4 * t + 1.3 * o)
        return w
    return f


def _diag(a1, a2, angle_deg=65.0, T=3.0):
    """Two simultaneous sweeps -- a sloppy diagonal motion."""
    f1, f2 = _sweep_motion(a1, angle_deg, T), _sweep_motion(a2, angle_deg, T)
    def f(t):
        w1, w2 = f1(t), f2(t)
        return [w1[k] + w2[k] for k in range(3)]
    return f


def _freehand(axis):
    """Intended axis dominates, but the other two wobble at incommensurate
    frequencies -- a non-closed path whose per-axis integrals do not return to
    zero. This is what fooled the July 2026 probe."""
    def f(t):
        w = [0.0, 0.0, 0.0]
        w[axis] = 90.0 * math.sin(2 * math.pi * 0.35 * t)
        for o in range(3):
            if o != axis:
                w[o] = 22.0 * math.sin(2 * math.pi * 0.23 * t + 1.1 * o)
        return w
    return f


def _naive_path_integral(samples):
    """The discredited method, kept only to quantify its error."""
    integ = [0.0, 0.0, 0.0]
    for (t1, g1, _), (t2, g2, _) in zip(samples, samples[1:]):
        dt = t2 - t1
        for i in range(3):
            integ[i] += 0.5 * (g1[i] + g2[i]) * dt
    return integ


def self_test():
    print("Self-test: judgment math against synthetic IMU data (no hardware)\n")

    cases = []
    for ax in range(3):
        cases.append(("sweep %s, healthy" % AXES[ax],
                      _simulate(_sweep_motion(ax), seconds=4.0), ax, +1, "pass"))
    cases += [
        ("sweep X, slow link (13 Hz)",
         _simulate(_sweep_motion(0), seconds=4.0, rate_hz=13.0), 0, +1, "pass"),
        ("sweep X, gyro X sign flipped",
         _simulate(_sweep_motion(0), seconds=4.0, fault="flip_x"), 0, +1, "fail"),
        ("sweep Y, gyro Y sign flipped",
         _simulate(_sweep_motion(1), seconds=4.0, fault="flip_y"), 1, +1, "fail"),
        ("sweep Z, gyro Z sign flipped",
         _simulate(_sweep_motion(2), seconds=4.0, fault="flip_z"), 2, +1, "fail"),
        ("sweep X, gyro X/Y swapped",
         _simulate(_sweep_motion(0), seconds=4.0, fault="swap_xy"), 0, +1, "fail"),
        ("sweep X done the wrong way",
         _simulate(_sweep_motion(0, angle_deg=-90.0), seconds=4.0), 0, +1, "fail"),
        ("sweep Y too small (20 deg)",
         _simulate(_sweep_motion(1, angle_deg=20.0), seconds=4.0), 1, +1, "redo"),
        ("diagonal X+Y sweep",
         _simulate(_diag(0, 1), seconds=4.0), 0, +1, "redo"),
    ]
    allok = True
    for name, samples, axis, want, expect in cases:
        verdict, _, ang = judge_sweep(samples, axis, want)
        ok = verdict == expect
        allok &= ok
        print("  %-32s X%+5.0f Y%+5.0f Z%+5.0f -> %-5s %s"
              % (name, ang[0], ang[1], ang[2], verdict.upper(),
                 "ok" if ok else "*** EXPECTED %s ***" % expect.upper()))

    print("\nFor contrast, the discredited freehand path-integration VERDICT on")
    print("healthy wandering data -- off-axis totals would be ~0 if it were valid:")
    for ax in range(3):
        integ = _naive_path_integral(_simulate(_freehand(ax)))
        off = [integ[i] for i in range(3) if i != ax]
        print("    about %s: intended %+6.1f deg, off-axis %+6.1f / %+6.1f deg"
              % (AXES[ax], integ[ax], off[0], off[1]))
    print("  That phantom off-axis rotation is why sweeps must be single-axis and")
    print("  monotone, and why ambiguous motion earns a REDO, never a verdict.")

    print("\n%s" % ("SELF-TEST PASS" if allok else "SELF-TEST FAIL"))
    return 0 if allok else 1


def main():
    args = [a for a in sys.argv[1:]]
    if "--self-test" in args:
        return self_test()
    phase = "all"
    if "--phase" in args:
        i = args.index("--phase")
        if i + 1 < len(args):
            phase = args[i + 1].lower()
            del args[i:i + 2]
    port = args[0] if args else "/dev/ttyACM0"

    try:
        ser = serial.Serial(port, 115200, timeout=0.05)
    except Exception as e:
        print("PORT ERROR: %s" % e)
        return 2
    time.sleep(0.3)

    probe = read_raw_imu(ser, timeout=1.0)
    if probe is None:
        print("No MSP_RAW_IMU reply on %s -- is the board running dRehmFlight?" % port)
        ser.close()
        return 1

    print("IMU alignment bench acceptance on %s" % port)
    print("PROPS OFF. Read-only: this tool never writes, saves, reboots, or arms.")
    print("Body frame under test: +X forward, +Y left, +Z up (values are")
    print("post-alignment, straight off boardAlignMatrix).")

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    results = []
    completed = True
    try:
        tty.setcbreak(fd)
        if phase in ("a", "all"):
            completed &= bool(phase_a(ser, results))
        if completed and phase in ("b", "all"):
            completed &= bool(phase_b(ser, results))
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        ser.close()

    if not completed:
        return 3

    print("\n=== Verdict ===")
    for label, ok in results:
        print("  %-24s %s" % (label, "PASS" if ok else "FAIL"))
    bad = [l for l, ok in results if not ok]
    if bad:
        print("\nFAIL: %d/%d checks bad -> %s" % (len(bad), len(results), ", ".join(bad)))
        print("Do NOT fly this board on this alignment.")
        return 1
    print("\nPASS: %d/%d checks good. Alignment behaves as the firmware believes."
          % (len(results), len(results)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
