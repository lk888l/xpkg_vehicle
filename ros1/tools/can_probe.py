#!/usr/bin/env python3
"""Receive-only Classic CAN / CAN-FD inventory. This script never calls send()."""
import argparse
from collections import Counter
import json
import socket
import struct
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("interface")
    parser.add_argument("--seconds", type=float, default=5)
    options = parser.parse_args()
    counts, samples = Counter(), {}
    with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
        bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
        bus.bind((options.interface,))
        bus.settimeout(.1)
        start = time.monotonic()
        while time.monotonic() - start < options.seconds:
            try:
                frame = bus.recv(72)
            except socket.timeout:
                continue
            identifier, length = struct.unpack_from("=IB", frame)
            key = "{:08X}:{}:{}".format(identifier & 0x1fffffff,
                "extended" if identifier & 0x80000000 else "standard",
                "FD" if len(frame) == 72 else "classic")
            counts[key] += 1
            samples[key] = frame[8:8 + length].hex(" ")
    print(json.dumps({"interface": options.interface, "seconds": time.monotonic() - start,
                      "receive_only": True, "frames": dict(counts), "last_payloads": samples}, indent=2))


if __name__ == "__main__":
    main()
