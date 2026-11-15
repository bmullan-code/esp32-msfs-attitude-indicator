"""Serial link to the ESP32 attitude indicator.

Line protocol (ASCII, one sample per line, NMEA-style XOR checksum):

    $ATT,<pitch>,<bank>,<slip>,<status>*HH\\n

    pitch   degrees, nose up positive
    bank    degrees, right bank positive
    slip    turn coordinator ball, -1..1 (reserved, not drawn yet)
    status  bit0 sim connected, bit1 sim paused, bit2 instrument valid
    HH      XOR of every character between '$' and '*', two hex digits

Sending a line containing just "?" makes the device answer "ATTITUDE-IND,<version>,<board>",
which is how the bridge finds the right serial port.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import serial
from serial.tools import list_ports

BAUD = 115200
IDENT_PREFIX = "ATTITUDE-IND"

STATUS_CONNECTED = 0x01
STATUS_PAUSED = 0x02
STATUS_VALID = 0x04


@dataclass
class Sample:
    pitch: float = 0.0
    bank: float = 0.0
    slip: float = 0.0
    status: int = STATUS_CONNECTED | STATUS_VALID


def format_sample(s: Sample) -> bytes:
    body = f"ATT,{s.pitch:.2f},{s.bank:.2f},{s.slip:.3f},{s.status}"
    checksum = 0
    for ch in body:
        checksum ^= ord(ch)
    return f"${body}*{checksum:02X}\n".encode("ascii")


def _open(port: str, timeout: float = 0.3) -> serial.Serial:
    # Keep DTR and RTS asserted from the moment the port opens and never toggle them:
    # on the ESP32-S3's native USB port, toggling these lines resets the chip.
    s = serial.Serial()
    s.port = port
    s.baudrate = BAUD
    s.timeout = timeout
    s.write_timeout = 1.0
    s.dtr = True
    s.rts = True
    s.open()
    return s


def probe(port: str) -> str | None:
    """Return the device identity string if the attitude indicator answers on this port."""
    try:
        with _open(port) as s:
            time.sleep(0.2)
            s.reset_input_buffer()
            for _ in range(3):
                s.write(b"?\n")
                deadline = time.monotonic() + 0.6
                while time.monotonic() < deadline:
                    line = s.readline().decode("ascii", "ignore").strip()
                    if line.startswith(IDENT_PREFIX):
                        return line
    except (serial.SerialException, OSError):
        return None
    return None


def find_device(preferred: str | None = None) -> tuple[str, str] | None:
    """Locate the attitude indicator. Returns (port, identity) or None."""
    # Only USB serial devices are worth probing; Espressif (native USB) and WCH (the UART
    # port's CH343) first.
    usb = [p for p in list_ports.comports() if p.vid is not None]
    usb.sort(key=lambda p: 0 if p.vid in (0x303A, 0x1A86) else 1)
    candidates = [preferred] if preferred else []
    candidates += [p.device for p in usb if p.device not in candidates]
    for port in candidates:
        ident = probe(port)
        if ident:
            return port, ident
    return None


class Link:
    """Owns the serial connection and reconnects when the device is unplugged."""

    def __init__(self, preferred_port: str | None = None, log=print):
        self.preferred = preferred_port
        self.log = log
        self.port: serial.Serial | None = None
        self._next_search = 0.0

    def connected(self) -> bool:
        return self.port is not None

    def ensure(self) -> bool:
        if self.port is not None:
            return True
        now = time.monotonic()
        if now < self._next_search:
            return False
        self._next_search = now + 3.0
        found = find_device(self.preferred)
        if not found:
            return False
        name, ident = found
        try:
            self.port = _open(name, timeout=0)
        except serial.SerialException as e:
            self.log(f"Could not open {name}: {e}")
            return False
        self.log(f"Attitude indicator found on {name} ({ident})")
        return True

    def send(self, sample: Sample) -> None:
        if not self.ensure():
            return
        try:
            self.port.write(format_sample(sample))
            self.port.reset_input_buffer()          # discard the device's log output
        except (serial.SerialException, OSError) as e:
            self.log(f"Lost the attitude indicator: {e}")
            self.close()

    def close(self) -> None:
        if self.port is not None:
            try:
                self.port.close()
            except Exception:
                pass
        self.port = None
