# esp32-msfs-attitude-indicator

A physical attitude indicator for Microsoft Flight Simulator 2020, built on the
**Waveshare ESP32-S3-Touch-LCD-2.1** (round 480×480 touch display).

A small Python bridge on the Windows PC reads the aircraft attitude from MSFS through
SimConnect and streams it over USB serial. The ESP32 draws a classic attitude indicator in
the style of the FAA *Pilot's Handbook of Aeronautical Knowledge*: a blue and brown card that
rotates with bank and slides with pitch, a pitch ladder, a bank scale with a fixed pointer,
and a fixed orange miniature airplane.

Without the simulator, the instrument runs in demo mode and follows the board's own tilt
through its motion sensor: lean it left and it shows a left bank.

```
 MSFS 2020 ──SimConnect──▶ bridge/msfs_bridge.py ──USB serial, 30 Hz──▶ ESP32-S3 + round LCD
```

## Repository layout

```
firmware/                 PlatformIO project for the ESP32-S3 (Arduino-ESP32 3.x)
  platformio.ini          board, memory and upload settings
  src/main.cpp            modes, smoothing, input handling, render loop
  src/renderer.cpp        draws the instrument
  src/display.cpp         ST7701 panel set-up, double-buffered RGB output, backlight
  src/simlink.cpp         serial protocol receiver
  src/imu.cpp             QMI8658 accelerometer, for demo-mode tilt
  src/touch.cpp           CST820 touch gestures
  src/expander.cpp        TCA9554 I/O expander (panel/touch reset, panel CS, buzzer)
  src/settings.cpp        settings saved in flash
  src/board.h             pin map
bridge/                   Python, runs on the Windows PC
  msfs_bridge.py          SimConnect -> serial bridge
  attitude_link.py        serial protocol and automatic port discovery
  fake_sender.py          synthetic data for testing without the simulator (any OS)
  config.example.ini      optional settings file
```

## The instrument

| Mode    | When                                                     | Shows                              |
|---------|----------------------------------------------------------|------------------------------------|
| DEMO    | no data since power-up, or toggled with a double tap     | follows the board's own tilt      |
| LIVE    | samples arriving from the bridge                         | the simulator's attitude           |
| NO DATA | samples stopped for more than a second, or no sim        | red flag, card frozen              |
| OFF     | the bridge reports the gauge failed                      | red OFF flag                       |

**Controls**

- **Swipe up or down** raises or lowers the miniature airplane, like the real adjustment knob.
- **Long press** re-centres the miniature airplane.
- **Double tap** toggles demo mode.
- **BOOT button** cycles the backlight through 20, 40, 60, 80 and 100%.

The airplane position and brightness are saved in flash.

The display redraws at about 30 frames per second and smooths between samples, so motion
stays fluid at any bridge rate from 20 to 60 Hz.

## Serial protocol

One ASCII line per sample at 115200 baud, with an NMEA-style checksum:

```
$ATT,<pitch>,<bank>,<slip>,<status>*HH
$ATT,5.00,20.00,0.000,5*6D
```

| Field  | Meaning                                                                   |
|--------|---------------------------------------------------------------------------|
| pitch  | degrees, nose up positive                                                 |
| bank   | degrees, right bank positive                                              |
| slip   | turn coordinator ball, −1 to 1 (sent, not drawn yet)                      |
| status | bit 0: sim connected, bit 1: sim paused, bit 2: instrument valid          |
| HH     | XOR of all characters between `$` and `*`, two hex digits                 |

Sending a line containing only `?` makes the device reply `ATTITUDE-IND,1,touch-lcd-2.1`.
The bridge uses this to find the right port automatically. The device accepts data on
either of the board's USB-C ports.

## Firmware: build and flash

Install [PlatformIO](https://platformio.org/) (the VS Code extension or `pip install platformio`), then
run the commands below. The first build downloads the ESP32 platform and takes a few minutes.

```
cd firmware
pio run -t upload --upload-port COM5          # Windows, use your port
pio run -t upload --upload-port /dev/cu.usbmodem1101   # macOS
```

The board has two USB-C ports. Either works for flashing and for the bridge.

- **USB** is the ESP32-S3's native port. On macOS, resetting through this port leaves the
  chip in download mode, so `platformio.ini` reboots it through the watchdog after upload
  and keeps DTR/RTS asserted while monitoring.
- **UART** goes through a CH343 USB-serial chip and resets normally on every OS.

If an upload ever leaves the board unresponsive, hold BOOT, tap RESET, release BOOT, and
upload again.

## Bridge: set up on the Windows PC

1. Install Python 3.10 or newer from python.org, ticking "Add python.exe to PATH".
2. In a terminal in the `bridge` folder:

   ```
   pip install -r requirements.txt
   python msfs_bridge.py
   ```

3. Start MSFS and load a flight. The bridge waits for the simulator, finds the indicator
   on its own, and reports the sample rate every 10 seconds.

Options, which can also go in `config.ini` (copy `config.example.ini`):

| Option                     | Default      | Purpose                                              |
|----------------------------|--------------|------------------------------------------------------|
| `--port COM5`              | automatic    | use a specific serial port                           |
| `--rate 40`                | 30           | samples per second                                   |
| `--source aircraft`        | `instrument` | true attitude instead of the cockpit gauge's reading |
| `--invert-pitch yes/no`    | yes          | flip the sim's pitch sign                            |
| `--invert-bank yes/no`     | yes          | flip the sim's bank sign                             |
| `--show-raw`               | off          | print the raw sim values once a second               |

The default `instrument` source reads `ATTITUDE INDICATOR PITCH/BANK DEGREES`, which
behaves like the real gauge: it lags slightly, drifts, and tumbles when failed or unpowered.

### Checking the signs on first use

MSFS reports nose-up pitch as negative, so the bridge flips the signs by default. Run
`python msfs_bridge.py --show-raw`, then in a parked or level aircraft:

- **Pitch up.** More sky should appear, with the horizon dropping below the miniature airplane.
- **Roll right.** The horizon's right side should rise, and the fixed pointer should sit over
  the right-hand bank marks.

If either moves the wrong way, set the matching `invert_` option to `no`.

### Testing without the simulator

`fake_sender.py` runs on any OS and needs only `pyserial`:

```
python fake_sender.py                  # smooth sweep
python fake_sender.py --pattern steps  # holds known attitudes, printing what to expect
```

## Hardware notes

- **Board:** Waveshare ESP32-S3-Touch-LCD-2.1, with an ESP32-S3R8, 16 MB flash and 8 MB
  octal PSRAM.
- **Display:** an ST7701 panel on a 16-bit RGB interface. The panel is configured once over
  3-wire SPI on GPIO1/2, and its chip-select and reset are on the TCA9554 expander.
- **Panel register sequence:** taken from the factory firmware shipped on the board. The
  sequence in Waveshare's current demo download is for a different panel batch. On this
  board it makes the image roll and can leave the panel washed out or tinted for a while
  afterwards. If a future board shows rolling lines, compare its factory firmware's sequence
  first. Building with `-DTEST_PATTERN` shows four solid colour quadrants for checking this.
- **Touch:** a CST820 at I2C 0x15. It doesn't answer on I2C until it has been touched, so
  the firmware reads it only after its interrupt line (GPIO16) fires.
- **Motion sensor:** a QMI8658 at I2C 0x6B drives demo mode. Its axes, measured on this
  board: +X points to the top of the screen, +Y to the right, and −Z towards the viewer.
  Leaning the board left shows a left bank, and tilting its top away shows nose up. Level
  flight shows when the board's top is tilted back about 25°, for easier viewing; change
  `TILT_LEVEL_BACK_DEG` in `main.cpp` to adjust. Without the sensor, demo mode falls back
  to a slow sweep.
- **I2C bus:** SDA on GPIO15 and SCL on GPIO7. The bus carries the expander (0x20), the RTC
  (0x51) and the IMU (0x6B).
- **Rendering:**
  - Frames are drawn in 48-row strips in internal RAM, then copied into one of two PSRAM
    frame buffers.
  - Buffers swap in sync with the panel refresh, so there is no tearing.
  - Only the inner disc is copied after the first frames, because the bezel never changes.
  - Rendering runs on core 0. The display's bounce-buffer interrupt runs on core 1.

## Status

**Tested on the board:**
- The instrument draws correctly in demo mode, checked by eye.
- Demo mode follows the board's tilt correctly in both bank and pitch.
- The firmware runs at about 30 frames per second.
- The serial protocol works, including the checksum and automatic port discovery.
- The firmware switches correctly between DEMO, LIVE and NO DATA.
- `fake_sender.py` works.
- `msfs_bridge.py` works against a stand-in for the SimConnect package. It flips the signs,
  finds the device, and the device goes LIVE with the expected attitude.

**Still to verify:**
- The touch gestures.
- The bridge against MSFS itself on the Windows PC. The sign check above is the first thing
  to do there.
