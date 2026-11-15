"""Sends synthetic attitude data to the indicator, for testing without the simulator.

Runs on macOS, Linux or Windows:

    python fake_sender.py                  # find the device automatically
    python fake_sender.py --port COM5      # or name the port
    python fake_sender.py --pattern steps  # hold fixed attitudes for checking signs
"""

from __future__ import annotations

import argparse
import math
import time

from attitude_link import STATUS_CONNECTED, STATUS_VALID, Link, Sample

STEPS = [  # (pitch, bank, label) held for 3 seconds each
    (0, 0, "level"),
    (10, 0, "10 deg nose up: more sky, horizon below the airplane"),
    (-10, 0, "10 deg nose down: more ground"),
    (0, 30, "30 deg right bank: horizon right side up, pointer on the right-hand 30 mark"),
    (0, -30, "30 deg left bank"),
    (5, 20, "climbing right turn"),
]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port, e.g. COM5 or /dev/cu.usbmodem1101")
    ap.add_argument("--rate", type=float, default=30.0, help="samples per second (default 30)")
    ap.add_argument("--pattern", choices=["sweep", "steps"], default="sweep")
    ap.add_argument("--seconds", type=float, default=0, help="stop after this long (0 = run forever)")
    args = ap.parse_args()

    link = Link(args.port)
    period = 1.0 / args.rate
    start = time.monotonic()
    sent = 0
    last_label = None
    while args.seconds <= 0 or time.monotonic() - start < args.seconds:
        t = time.monotonic() - start
        if args.pattern == "sweep":
            pitch = 12 * math.sin(t * 0.7)
            bank = 45 * math.sin(t * 0.4)
        else:
            pitch, bank, label = STEPS[int(t // 3) % len(STEPS)]
            if label != last_label:
                print(f"{pitch:+5.1f} pitch {bank:+6.1f} bank   {label}")
                last_label = label
        link.send(Sample(pitch, bank, 0.0, STATUS_CONNECTED | STATUS_VALID))
        if link.connected():
            sent += 1
        time.sleep(max(0.0, period - ((time.monotonic() - start) % period)))
    link.close()
    print(f"sent {sent} samples")


if __name__ == "__main__":
    main()
