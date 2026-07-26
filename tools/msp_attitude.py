#!/usr/bin/env python3
"""Attitude check for dRehmFlight over USB-CDC (MSP_ATTITUDE, cmd 108). Read-only.

Two modes:
  (default) guided: steps through ~45 deg moves on each axis; press SPACE at each
            held extreme to record the max. Prints a summary of recorded maxima so
            roll/pitch/yaw sign and tracking can be confirmed.
  --live    continuous single-line readout at ~20 Hz. Flick the board briskly and
            watch the numbers track: crisp tracking = clean; lag or a momentary
            wrong-direction swing during fast motion = gyro-rate sign problem.

Usage:  ./ci/msp_attitude.py [port] [--live]      (default port /dev/ttyACM0)
Keys (guided):  SPACE = record & advance,  q = quit
"""
import serial, struct, sys, time, termios, tty, select

args = [a for a in sys.argv[1:] if not a.startswith("--")]
PORT = args[0] if args else "/dev/ttyACM0"
LIVE = "--live" in sys.argv

STEPS = [
    ("PITCH  nose DOWN  ~45 deg", "pitch", "expect +"),
    ("PITCH  nose UP    ~45 deg", "pitch", "expect -"),
    ("ROLL   LEFT       ~45 deg", "roll",  "expect -"),
    ("ROLL   RIGHT      ~45 deg", "roll",  "expect +"),
    ("YAW    CCW        ~45 deg", "yaw",   "record"),
    ("YAW    CW         ~45 deg", "yaw",   "record"),
]


def read_att(p):
    p.reset_input_buffer()
    p.write(b"$M<" + bytes([0, 108, 108]))          # MSP_ATTITUDE request, csum = 0^108
    deadline = time.time() + 0.12
    buf = b""
    while time.time() < deadline:
        buf += p.read(64)
        i = buf.find(b"$M>")
        if i >= 0 and len(buf) >= i + 5:
            ln = buf[i + 3]
            if len(buf) >= i + 5 + ln + 1 and ln >= 6:
                r, pi, y = struct.unpack("<3h", buf[i + 5:i + 5 + 6])
                return {"roll": r / 10.0, "pitch": pi / 10.0, "yaw": float(y)}
    return None


def getkey():
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    return sys.stdin.read(1) if dr else None


def run_live(p):
    print(f"Live attitude on {PORT}. Flick the board; watch for lag / wrong-way swings. Ctrl-C to quit.\n")
    try:
        while True:
            v = read_att(p)
            if v:
                sys.stdout.write("\rroll=%7.1f  pitch=%7.1f  yaw=%6.0f   " % (v["roll"], v["pitch"], v["yaw"]))
                sys.stdout.flush()
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nstopped")


def run_guided(p):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    results = []
    try:
        tty.setcbreak(fd)
        print("Guided attitude check. Hold the board LEVEL to start.")
        print("For each step: move to ~45 deg, hold at the max, press SPACE. 'q' quits.\n")
        for prompt, axis, hint in STEPS:
            v = None
            while v is None:
                v = read_att(p)
            base = v["yaw"] if axis == "yaw" else 0.0
            ext = 0.0
            while True:
                v = read_att(p)
                if v:
                    val = v[axis] - base
                    if abs(val) > abs(ext):
                        ext = val
                    sys.stdout.write("\r  %-26s (%s)  now=%7.1f  max=%+7.1f   [SPACE] "
                                     % (prompt, hint, val, ext))
                    sys.stdout.flush()
                k = getkey()
                if k == ' ':
                    break
                if k in ('q', 'Q'):
                    raise KeyboardInterrupt
                time.sleep(0.03)
            results.append((prompt, axis, ext))
            sys.stdout.write("\n   -> %s max = %+.1f\n\n" % (axis, ext))
        print("==== recorded maxima ====")
        for prompt, axis, ext in results:
            print("  %-26s  %-5s = %+7.1f" % (prompt, axis, ext))
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    p = serial.Serial(PORT, 115200, timeout=0.1)
    time.sleep(0.2)
    try:
        (run_live if LIVE else run_guided)(p)
    finally:
        p.close()


if __name__ == "__main__":
    main()
