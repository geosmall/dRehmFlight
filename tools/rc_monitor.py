#!/usr/bin/env python3
"""Live RC channel monitor for dRehmFlight over USB-CDC (MSP_RC, cmd 105).

Run this in your own terminal and move ONE control at a time — the channel that
moves is marked '<<'. Verifies the RX -> internal channel mapping the flight code
uses (post AETR->TAER remap): ch1=THR ch2=ROLL ch3=PITCH ch4=YAW ch5=ARM ch6=AUX1.

Usage:  ./ci/rc_monitor.py [port]      (default /dev/ttyACM0)
Quit:   Ctrl-C
"""
import serial, struct, sys, time

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
LAB = ["THR", "ROLL", "PITCH", "YAW", "ARM", "AUX1"]


def read_rc(p):
    p.reset_input_buffer()
    p.write(b"$M<" + bytes([0, 105, 105]))          # MSP_RC request, csum = 0^105
    deadline = time.time() + 0.12
    buf = b""
    while time.time() < deadline:
        buf += p.read(64)
        i = buf.find(b"$M>")
        if i >= 0 and len(buf) >= i + 5:
            ln = buf[i + 3]
            if len(buf) >= i + 5 + ln + 1 and ln >= 12:
                return struct.unpack("<6H", buf[i + 5:i + 5 + 12])
    return None


def main():
    p = serial.Serial(PORT, 115200, timeout=0.1)
    time.sleep(0.2)
    print(f"RC monitor on {PORT}. Move one control at a time; mover marked '<<'. Ctrl-C to quit.\n")
    rest = None
    try:
        while True:
            v = read_rc(p)
            if v:
                if rest is None:
                    rest = list(v)
                cells = []
                for k in range(6):
                    mark = "<<" if abs(v[k] - rest[k]) > 150 else "  "
                    cells.append(f"{LAB[k]}:{v[k]:4d}{mark}")
                sys.stdout.write("\r" + " ".join(cells) + "  ")
                sys.stdout.flush()
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        p.close()


if __name__ == "__main__":
    main()
