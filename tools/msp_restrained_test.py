#!/usr/bin/env python3
"""Guided restrained props-on test for dRehmFlight over USB-CDC. Read-only.

Steps through the restrained-test phases. For each phase it shows the
instruction; you get into position (TX/plate), tap SPACE, get a 3 s countdown,
then it records ~6 s of samples (attitude, throttle, mixer outputs M1-M4) and
prints a phase summary. All samples land in a timestamped CSV under
test_logs/restrained/ for offline analysis.

MSP_MOTOR is MIXER order: M1=rear-right, M2=front-right, M3=rear-left,
M4=front-left (1000 = off/idle-min .. 2000 = full).

Usage:  ./ci/msp_restrained_test.py [port]      (default /dev/ttyACM0)
Keys:   SPACE = start phase recording,  s = skip phase,  q = quit
"""
import os, serial, struct, sys, time, termios, tty, select

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
MSP_STATUS, MSP_MOTOR, MSP_RC, MSP_ATTITUDE = 101, 104, 105, 108
RECORD_S = 6.0
COUNTDOWN_S = 3

PHASES = [
    ("baseline",     "DISARMED, motors off, plate LEVEL, throttle down"),
    ("arm-idle",     "ARM (switch OFF->ON), throttle fully DOWN, plate LEVEL"),
    ("throttle-mid", "Raise throttle to ~HALF stick and HOLD, plate LEVEL"),
    ("tilt-right",   "Keep ~half throttle, TILT plate RIGHT ~30 deg and HOLD"),
    ("nose-down",    "Keep ~half throttle, tilt NOSE DOWN ~30 deg and HOLD"),
    ("yaw-left",     "Plate LEVEL, ~half throttle, hold YAW stick LEFT ~half"),
    ("yaw-right",    "Plate LEVEL, ~half throttle, hold YAW stick RIGHT ~half"),
]


def poll(p, cmd):
    p.reset_input_buffer()
    p.write(b"$M<" + bytes([0, cmd, cmd]))
    deadline = time.time() + 0.1
    buf = b""
    while time.time() < deadline:
        buf += p.read(64)
        i = buf.find(b"$M>")
        if i >= 0 and len(buf) >= i + 5:
            ln = buf[i + 3]
            if len(buf) >= i + 5 + ln + 1:
                return buf[i + 5:i + 5 + ln]
    return None


def sample(p):
    st = poll(p, MSP_STATUS)
    att = poll(p, MSP_ATTITUDE)
    mo = poll(p, MSP_MOTOR)
    rc = poll(p, MSP_RC)
    if not (st and att and mo and rc) or len(st) < 10 or len(att) < 6 or len(mo) < 8 or len(rc) < 12:
        return None
    armed = struct.unpack("<I", st[6:10])[0] & 1
    r, pi, y = struct.unpack("<3h", att[:6])
    m = struct.unpack("<4H", mo[:8])
    thr = struct.unpack("<H", rc[0:2])[0]
    return {"armed": armed, "roll": r / 10.0, "pitch": pi / 10.0, "yaw": y,
            "thr": thr, "m": m}


def getkey():
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    return sys.stdin.read(1) if dr else None


def summarize(name, rows):
    if not rows:
        return f"  {name}: NO SAMPLES"
    rolls = [r["roll"] for r in rows]
    pitchs = [r["pitch"] for r in rows]
    thrs = [r["thr"] for r in rows]
    ms = [[r["m"][k] for r in rows] for k in range(4)]
    mean = lambda v: sum(v) / len(v)
    pp = lambda v: max(v) - min(v)
    lbl = ["M1(RR)", "M2(FR)", "M3(RL)", "M4(FL)"]
    mstr = "  ".join("%s=%4.0f(max %4d)" % (lbl[k], mean(ms[k]), max(ms[k])) for k in range(4))
    return ("  %-12s n=%-3d armed=%d thr~%4.0f | roll %+6.1f (pp %4.1f)  pitch %+6.1f (pp %4.1f) | %s"
            % (name, len(rows), rows[-1]["armed"], mean(thrs),
               mean(rolls), pp(rolls), mean(pitchs), pp(pitchs), mstr))


def main():
    p = serial.Serial(PORT, 115200, timeout=0.05)
    time.sleep(0.2)
    os.makedirs("test_logs/restrained", exist_ok=True)
    log_path = time.strftime("test_logs/restrained/restrained_%Y%m%d_%H%M%S.csv")
    log = open(log_path, "w")
    log.write("t,phase,armed,thr,roll,pitch,yaw,m1_rr,m2_fr,m3_rl,m4_fl\n")

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    summaries = []
    t0 = time.time()
    try:
        tty.setcbreak(fd)
        print("Restrained props-on test. PROPS ON — craft STRAPPED DOWN. Kill = arm switch OFF.")
        print(f"Logging to {log_path}\n")
        for name, instr in PHASES:
            print(f">>> {name}: {instr}")
            print("    Get in position, then tap SPACE to record (s = skip, q = quit)")
            while True:
                k = getkey()
                if k == ' ':
                    break
                if k in ('s', 'S'):
                    print("    skipped\n")
                    name = None
                    break
                if k in ('q', 'Q'):
                    raise KeyboardInterrupt
                time.sleep(0.03)
            if name is None:
                continue
            for c in range(COUNTDOWN_S, 0, -1):
                sys.stdout.write(f"\r    recording in {c}... ")
                sys.stdout.flush()
                time.sleep(1.0)
            rows = []
            end = time.time() + RECORD_S
            while time.time() < end:
                v = sample(p)
                if v:
                    rows.append(v)
                    log.write("%.2f,%s,%d,%d,%.1f,%.1f,%d,%d,%d,%d,%d\n"
                              % (time.time() - t0, name, v["armed"], v["thr"],
                                 v["roll"], v["pitch"], v["yaw"],
                                 v["m"][0], v["m"][1], v["m"][2], v["m"][3]))
                    sys.stdout.write("\r    rec %4.1fs  r=%+6.1f p=%+6.1f | M %4d %4d %4d %4d   "
                                     % (end - time.time(), v["roll"], v["pitch"],
                                        v["m"][0], v["m"][1], v["m"][2], v["m"][3]))
                    sys.stdout.flush()
            s = summarize(name, rows)
            summaries.append(s)
            print("\n" + s + "\n")
        print(">>> DONE. Throttle down, DISARM (arm switch OFF).\n")
        print("==== phase summaries ====")
        for s in summaries:
            print(s)
        print(f"\nFull log: {log_path}")
    except KeyboardInterrupt:
        print("\nstopped — throttle down, DISARM (arm switch OFF)")
        if summaries:
            print("==== phase summaries (partial) ====")
            for s in summaries:
                print(s)
        print(f"Partial log: {log_path}")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        log.close()
        p.close()


if __name__ == "__main__":
    main()
