#!/usr/bin/env python3
"""
General-purpose serial send/receive tool.

Send text over a serial port and capture the response.
Designed to be called from shell scripts for automated serial interaction.

Usage:
    serial_io.py <port> [options]

Options:
    --send <text>         Send text with \\r\\n appended
    --send-raw <text>     Send text as-is (no newline)
    --expect <pattern>    Read until regex pattern matches (exit 0)
    --timeout <seconds>   Read timeout (default 3)
    --baud <rate>         Baud rate (default 115200)
    --quiet               Suppress received output (just set exit code)

Examples:
    # Send '#' to enter CLI mode, wait for banner
    serial_io.py /dev/ttyACM0 --send-raw '#' --expect 'dRehmFlight CLI'

    # Send a command, wait for prompt
    serial_io.py /dev/ttyACM0 --send 'set Kp_roll_angle 0.99' --expect '# '

    # Send reboot (no expect — just send and exit)
    serial_io.py /dev/ttyACM0 --send 'reboot'

    # Read output for 5 seconds
    serial_io.py /dev/ttyACM0 --timeout 5

Exit codes:
    0  Pattern found (or send-only completed)
    1  Timeout / pattern not found
    2  Port error
"""

import serial
import time
import sys
import re
import argparse


def main():
    parser = argparse.ArgumentParser(description='Serial send/receive tool')
    parser.add_argument('port', help='Serial port (e.g., /dev/ttyACM0)')
    parser.add_argument('--send', help='Send text with \\r\\n appended')
    parser.add_argument('--send-raw', dest='send_raw', help='Send text as-is')
    parser.add_argument('--expect', help='Regex pattern to wait for')
    parser.add_argument('--timeout', type=float, default=3.0, help='Read timeout in seconds (default 3)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default 115200)')
    parser.add_argument('--quiet', action='store_true', help='Suppress output')
    args = parser.parse_args()

    # Open port
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 2

    time.sleep(0.05)
    ser.reset_input_buffer()

    # Send data
    if args.send_raw is not None:
        ser.write(args.send_raw.encode())
    elif args.send is not None:
        ser.write((args.send + "\r\n").encode())

    # If no expect pattern and no timeout read, just send and exit
    if args.expect is None and args.send is not None:
        time.sleep(0.05)
        ser.close()
        return 0

    # Read response
    pattern = re.compile(args.expect) if args.expect else None
    found = False
    start = time.time()
    buf = ""

    while time.time() - start < args.timeout:
        data = ser.read(512)
        if data:
            text = data.decode('utf-8', errors='ignore')
            buf += text
            # Process complete lines
            while '\n' in buf:
                line, buf = buf.split('\n', 1)
                line = line.rstrip('\r')
                if not args.quiet:
                    print(line, flush=True)
                if pattern and pattern.search(line):
                    found = True
            # Also check partial buffer for prompt-style patterns (no newline)
            if pattern and pattern.search(buf):
                if not args.quiet and buf.strip():
                    print(buf.rstrip(), flush=True)
                found = True
            if found:
                break

    ser.close()

    if args.expect:
        return 0 if found else 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
