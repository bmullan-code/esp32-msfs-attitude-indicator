"""MSFS 2020 -> ESP32 attitude indicator bridge (Windows).

Reads the aircraft attitude from Microsoft Flight Simulator through SimConnect and streams
it to the attitude indicator over USB serial. Leave it running: it waits for the simulator,
finds the device by itself, and reconnects to either if they go away.

    python msfs_bridge.py                        # defaults
    python msfs_bridge.py --port COM5 --rate 40
    python msfs_bridge.py --source aircraft      # true attitude instead of the gauge
    python msfs_bridge.py --show-raw             # print raw sim values (for checking signs)

Settings can also live in config.ini next to this file, or next to msfs_bridge.exe
(see config.example.ini); command-line options take precedence.
"""

from __future__ import annotations

import argparse
import configparser
import math
import sys
import time
from pathlib import Path

from attitude_link import STATUS_CONNECTED, STATUS_PAUSED, STATUS_VALID, Link, Sample

try:
    from SimConnect import SimConnect
    try:
        from SimConnect import Request
    except ImportError:  # older package layout
        from SimConnect.RequestList import Request
except ImportError:
    SimConnect = None
    Request = None

# Simulation variables, requested in degrees so no unit conversion is needed.
SIMVARS = {
    # What the cockpit gauge shows: includes gyro drift, spin-up and failures.
    "instrument": (b"ATTITUDE INDICATOR PITCH DEGREES", b"ATTITUDE INDICATOR BANK DEGREES"),
    # The aircraft's true attitude.
    "aircraft": (b"PLANE PITCH DEGREES", b"PLANE BANK DEGREES"),
}
SLIP_VAR = (b"TURN COORDINATOR BALL", b"Position")            # -127..127 in "Position" units
FAILED_VAR = (b"PARTIAL PANEL ATTITUDE", b"Enum")             # non-zero when failed


# When packaged by PyInstaller, __file__ points into a temporary folder, so look for
# config.ini beside the .exe instead.
FROZEN = getattr(sys, "frozen", False)
APP_DIR = Path(sys.executable).parent if FROZEN else Path(__file__).parent


def log(msg: str) -> None:
    print(time.strftime("%H:%M:%S"), msg, flush=True)


def load_settings() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port, e.g. COM5 (default: find automatically)")
    ap.add_argument("--rate", type=float, help="samples per second (default 30)")
    ap.add_argument("--source", choices=SIMVARS.keys(), help="instrument (default) or aircraft")
    ap.add_argument("--invert-pitch", choices=["yes", "no"], help="flip the sim's pitch sign (default yes)")
    ap.add_argument("--invert-bank", choices=["yes", "no"], help="flip the sim's bank sign (default yes)")
    ap.add_argument("--show-raw", action="store_true", help="print raw sim values once a second")
    args = ap.parse_args()

    cfg = configparser.ConfigParser()
    cfg.read(APP_DIR / "config.ini")
    c = cfg["bridge"] if cfg.has_section("bridge") else {}
    args.port = args.port or c.get("port") or None
    args.rate = args.rate or float(c.get("rate", 30))
    args.source = args.source or c.get("source", "instrument")
    # MSFS reports nose-up pitch as negative; the indicator wants nose-up positive.
    args.invert_pitch = (args.invert_pitch or c.get("invert_pitch", "yes")) == "yes"
    args.invert_bank = (args.invert_bank or c.get("invert_bank", "yes")) == "yes"
    return args


class Sim:
    """SimConnect session that tolerates the simulator starting and stopping."""

    def __init__(self, source: str):
        self.source = source
        self.sm = None
        self.req = {}
        self._next_try = 0.0

    def ensure(self) -> bool:
        if self.sm is not None:
            return True
        if time.monotonic() < self._next_try:
            return False
        self._next_try = time.monotonic() + 5.0
        try:
            self.sm = SimConnect()
        except Exception:
            self.sm = None
            return False
        pitch_var, bank_var = SIMVARS[self.source]
        # _time=0: fetch a fresh value on every read.
        self.req = {
            "pitch": Request((pitch_var, b"Degrees"), self.sm, _time=0),
            "bank": Request((bank_var, b"Degrees"), self.sm, _time=0),
            "slip": Request(SLIP_VAR, self.sm, _time=0),
            "failed": Request(FAILED_VAR, self.sm, _time=0),
        }
        log(f"Connected to MSFS (source: {self.source} attitude)")
        return True

    def read(self) -> dict | None:
        try:
            values = {k: r.value for k, r in self.req.items()}
        except Exception as e:  # the simulator closed
            log(f"Lost MSFS: {e}")
            self.close()
            return None
        if values["pitch"] is None or values["bank"] is None:
            return None
        values["paused"] = bool(getattr(self.sm, "paused", False))
        return values

    def close(self) -> None:
        try:
            if self.sm is not None:
                self.sm.exit()
        except Exception:
            pass
        self.sm = None


def main() -> None:
    if SimConnect is None:
        sys.exit("The SimConnect package is missing. Install it with: pip install -r requirements.txt")
    args = load_settings()
    log(f"Bridge starting: {args.rate:g} Hz, source {args.source}, "
        f"invert pitch {args.invert_pitch}, invert bank {args.invert_bank}")

    link = Link(args.port, log=log)
    sim = Sim(args.source)
    period = 1.0 / args.rate
    sent, last_report, waiting_logged = 0, time.monotonic(), False

    while True:
        t0 = time.monotonic()
        if not sim.ensure():
            if not waiting_logged:
                log("Waiting for MSFS ...")
                waiting_logged = True
            link.ensure()                     # find the device meanwhile
            time.sleep(0.5)
            continue
        waiting_logged = False

        v = sim.read()
        if v is not None:
            pitch = -v["pitch"] if args.invert_pitch else v["pitch"]
            bank = -v["bank"] if args.invert_bank else v["bank"]
            slip = max(-1.0, min(1.0, (v["slip"] or 0) / 127.0))
            status = STATUS_CONNECTED
            if not v.get("failed"):
                status |= STATUS_VALID
            if v["paused"]:
                status |= STATUS_PAUSED
            if math.isfinite(pitch) and math.isfinite(bank):
                link.send(Sample(pitch, bank, slip, status))
                if link.connected():
                    sent += 1
            if args.show_raw and time.monotonic() - last_report >= 1.0:
                log(f"raw pitch {v['pitch']:+7.2f}  raw bank {v['bank']:+7.2f}  -> "
                    f"sent pitch {pitch:+7.2f} bank {bank:+7.2f}")

        if time.monotonic() - last_report >= 10.0:
            elapsed = time.monotonic() - last_report
            if not args.show_raw:
                log(f"{sent / elapsed:4.1f} samples/s to the indicator"
                    + ("" if link.connected() else " (indicator not found)"))
            sent, last_report = 0, time.monotonic()
        elif args.show_raw and time.monotonic() - last_report >= 1.0:
            sent, last_report = 0, time.monotonic()

        time.sleep(max(0.0, period - (time.monotonic() - t0)))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except SystemExit as e:
        # Started by double-clicking the .exe: keep the window open so the message can be read.
        if FROZEN and e.code not in (None, 0):
            print(e.code if isinstance(e.code, str) else f"Exited with code {e.code}")
            input("Press Enter to close.")
            sys.exit(1)
        raise
    except Exception:
        if not FROZEN:
            raise
        import traceback
        traceback.print_exc()
        input("Press Enter to close.")
        sys.exit(1)
