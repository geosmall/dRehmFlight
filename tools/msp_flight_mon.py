#!/usr/bin/env python3
"""Live flight monitor for dRehmFlight over USB-CDC. Read-only.

One-line readout at ~8 Hz: arm state, throttle, attitude (+ a 1 s peak-to-peak
jitter figure for roll/pitch — the vibration indicator), and mixer outputs
M1-M4 (MSP_MOTOR is MIXER order: M1=rear-right, M2=front-right, M3=rear-left,
M4=front-left; 1000 = off/idle-min .. 2000 = full).

For the restrained props-on test: watch that attitude stays level and jitter
stays small as throttle rises, and that tilting the rig raises the correct
mixer outputs (tilt right -> M1+M2 rise; nose down -> M2+M4 rise).

Usage:  ./ci/msp_flight_mon.py [port]      (default /dev/ttyACM0)
Quit:   Ctrl-C
"""
import serial, struct, sys, time
from collections import deque

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
MSP_STATUS, MSP_MOTOR, MSP_RC, MSP_ATTITUDE = 101, 104, 105, 108


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


def main():
    p = serial.Serial(PORT, 115200, timeout=0.05)
    time.sleep(0.2)
    print(f"Flight monitor on {PORT}.  M1=RR M2=FR M3=RL M4=FL (mixer order, 1000-2000).")
    print("jit = 1 s peak-to-peak of roll/pitch, degrees — the vibration indicator.\n")
    hist = deque()  # (t, roll, pitch)
    try:
        while True:
            st = poll(p, MSP_STATUS)
            att = poll(p, MSP_ATTITUDE)
            mo = poll(p, MSP_MOTOR)
            rc = poll(p, MSP_RC)
            if not (st and att and mo and rc) or len(st) < 10 or len(att) < 6 or len(mo) < 8 or len(rc) < 12:
                sys.stdout.write("\rno MSP reply (link?)                                                            ")
                sys.stdout.flush()
                time.sleep(0.2)
                continue
            armed = struct.unpack("<I", st[6:10])[0] & 1
            r, pi, y = struct.unpack("<3h", att[:6])
            roll, pitch = r / 10.0, pi / 10.0
            m = struct.unpack("<4H", mo[:8])
            thr = struct.unpack("<H", rc[0:2])[0]
            now = time.time()
            hist.append((now, roll, pitch))
            while hist and hist[0][0] < now - 1.0:
                hist.popleft()
            jr = max(h[1] for h in hist) - min(h[1] for h in hist)
            jp = max(h[2] for h in hist) - min(h[2] for h in hist)
            sys.stdout.write(
                "\r%s thr=%4d | r=%+6.1f p=%+6.1f y=%4d jit=%4.1f/%4.1f | "
                "M1(RR)=%4d M2(FR)=%4d M3(RL)=%4d M4(FL)=%4d  "
                % ("ARMED   " if armed else "disarmed", thr, roll, pitch, y, jr, jp,
                   m[0], m[1], m[2], m[3]))
            sys.stdout.flush()
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        p.close()


if __name__ == "__main__":
    main()
