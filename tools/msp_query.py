#!/usr/bin/env python3
"""Passive MSP V1 liveness query for dRehmFlight over USB-CDC.

Focused read-only liveness/identity GATE for CI — sends the 5 standard request
frames, decodes the replies, and exits non-zero if any reply is missing or
fails its checksum. Intentionally NOT a general MSP shell: no MSP SET / write /
save / reboot / arm path (that capability has no place in an auto-approved,
unsandboxed tool — it could arm motors). Safe on a powered board, props off.

MSP V1 framing: $M< + len + cmd + payload + csum, where csum = XOR of len,
cmd, and every payload byte. A bare request has len=0, so csum = cmd.

Usage:
    ./ci/msp_query.py [port]      # default port /dev/ttyACM0

Exit codes (so it works as a CI gate):
    0  all 5 liveness replies received with valid checksum
    1  one or more replies missing or BAD-CSUM (board not alive/healthy)
    2  port could not be opened

Invoke via the ./ci/ prefix so it matches the sandbox excludedCommands entry
(serial nodes are hidden inside the sandbox; this tool needs the real /dev).
"""
import sys, struct, time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"

MSP_API_VERSION = 1
MSP_FC_VARIANT  = 2
MSP_FC_VERSION  = 3
MSP_BOARD_INFO  = 4
MSP_STATUS      = 101

def frame(cmd):
    return bytes([0x24, 0x4d, 0x3c, 0x00, cmd, cmd])  # $ M < len=0 cmd csum=cmd

def read_reply(ser, want_cmd, timeout=1.0):
    """Parse one MSP V1 response: $M> len cmd payload csum."""
    end = time.time() + timeout
    buf = b""
    while time.time() < end:
        b = ser.read(1)
        if not b:
            continue
        buf += b
        if len(buf) > 512:
            buf = buf[-512:]
        i = buf.find(b"$M>")
        if i < 0 or len(buf) < i + 5:
            continue
        ln = buf[i+3]
        cmd = buf[i+4]
        need = i + 5 + ln + 1
        while len(buf) < need and time.time() < end:
            chunk = ser.read(need - len(buf))
            if chunk:
                buf += chunk
        if len(buf) < need:
            return None
        payload = buf[i+5:i+5+ln]
        csum = buf[i+5+ln]
        calc = ln ^ cmd
        for p in payload:
            calc ^= p
        ok = (calc == csum)
        if cmd == want_cmd:
            return (payload, ok)
        buf = buf[need:]  # different cmd echoed; keep scanning
    return None

def query(ser, cmd, label, report):
    """Send one request, print the decoded reply, and record pass/fail in
    `report` (a list of (label, ok) tuples used to compute the exit code).
    A reply counts as a pass only if it arrives with a valid checksum."""
    ser.reset_input_buffer()
    ser.write(frame(cmd))
    ser.flush()
    r = read_reply(ser, cmd)
    if r is None:
        print(f"  {label:14s}: NO REPLY")
        report.append((label, False))
        return None
    payload, ok = r
    flag = "ok" if ok else "BAD-CSUM"
    print(f"  {label:14s}: {payload.hex():<24s} [{flag}]")
    report.append((label, ok))
    return payload

def main():
    try:
        ser = serial.Serial(PORT, 115200, timeout=0.2)
    except Exception as e:
        print(f"PORT ERROR: {e}")
        return 2

    time.sleep(0.3)
    print(f"MSP liveness on {PORT}")
    report = []

    p = query(ser, MSP_API_VERSION, "API_VERSION", report)
    if p and len(p) >= 3:
        print(f"      -> protocol {p[0]}, API {p[1]}.{p[2]}")

    p = query(ser, MSP_FC_VARIANT, "FC_VARIANT", report)
    if p:
        print(f"      -> '{p.decode('ascii','replace')}'")

    p = query(ser, MSP_FC_VERSION, "FC_VERSION", report)
    if p and len(p) >= 3:
        print(f"      -> {p[0]}.{p[1]}.{p[2]}")

    p = query(ser, MSP_BOARD_INFO, "BOARD_INFO", report)
    if p and len(p) >= 4:
        ident = p[0:4].decode('ascii', 'replace')
        # layout: ident[4] hwrev[2] cap[1] flag[1] nameLen[1] name[nameLen]
        name = ""
        if len(p) >= 9:
            nl = p[8]
            name = p[9:9+nl].decode('ascii', 'replace')
        print(f"      -> board '{ident}' name '{name}'")

    p = query(ser, MSP_STATUS, "STATUS", report)
    if p and len(p) >= 11:
        cycle, i2c_err, sensors, flags = struct.unpack_from("<HHHI", p, 0)
        armed = bool(flags & 0x1)
        print(f"      -> cycle {cycle}us  i2c_errors {i2c_err}  "
              f"sensors 0x{sensors:02x}  flags 0x{flags:08x}  armed={armed}")

    ser.close()
    failed = [label for label, ok in report if not ok]
    if failed:
        print(f"FAIL: {len(failed)}/{len(report)} queries bad -> {', '.join(failed)}")
        return 1
    print(f"PASS: {len(report)}/{len(report)} liveness queries ok")
    return 0

if __name__ == "__main__":
    sys.exit(main())
