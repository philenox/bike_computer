# STATUS

Living handoff doc — overwrite freely. Most recent session at top.

## 2026-05-01 — IMU + SD bring-up, serial-log tooling, env restructure

**Done**

- **ICM-20948 IMU on I2C** (Qwiic, SDA=GPIO8, SCL=GPIO9). Breakout
  presents at 0x69 (AD0 high). WHO_AM_I=0xEA confirmed; scaled
  accel/gyro/mag/temp streaming at ~10 Hz. Sanity checks pass: |a|≈1g
  at rest, |m|≈36 µT (within Earth-field band). Magnetometer is
  accessed via `INT_PIN_CFG.BYPASS_EN` — internal I2C master mode is
  *not* used.
- **microSD on shared SPI** (SCK=12, MISO=13, MOSI=11, SD_CS=10).
  Mounts an SDHC card, prints card/FAT info, lists root, round-trips
  a write/read. Bus clocked conservatively at 4 MHz for jumper-wire
  prototype; bump once on perfboard. **Decision locked**: SD and
  Sharp display share this SPI bus (Sharp CS = GPIO14 reserved, not
  yet wired).
- **Serial-log tooling**: `scripts/log_serial.py` captures the port
  for N seconds to `logs/serial-<ts>.log`, reads port/baud from
  `platformio.ini`, and pulses RTS on open to force a fresh boot
  (caught the hard way during SD bring-up — sketches with quiet
  `loop()` were silent by the time we connected). `--no-reset`
  available. `scripts/flash_and_log.sh [env] [seconds]` chains pio
  upload with the logger.
- **Per-peripheral build envs**: each smoke test lives in
  `src/smoke_<name>/` with its own `[env:...]` block selecting it via
  `build_src_filter`. `src/main.cpp` is gone. Adding a new test is
  "drop a file, add a block." `default_envs = smoke_imu` for bare
  `pio run`.
- CH340 USB-UART suffix drifted from `-210` to `-110` between
  sessions; updated `platformio.ini`. Expect this to churn.

**Hardware on hand**

- Olimex ESP32-S3-DevKit-LiPo — working.
- ICM-20948 — **TESTED**.
- microSD breakout + 32 GB card — **TESTED**.
- Adafruit Sharp Memory LCD 2.7" 400x240 — unboxed, untested.
- 600 mAh LiPo — unboxed, untested.
- SparkFun NEO-M9N GNSS — **NOT YET ARRIVED**.

## Next session

The only remaining bench peripheral is the **Sharp Memory LCD** —
takes the same shared SPI bus, plus its own CS on GPIO14. First goal:
clear the screen, draw a single line of text. Framebuffer is 13.5 KB,
must live in MCU RAM; PSRAM not yet enabled in `platformio.ini` so
we'll be on internal SRAM (~320 KB free), which is fine.

After Sharp, the IMU side is ripe for **Madgwick fusion** — turning
the raw 9-DoF stream into a usable orientation. Open question still
unresolved: roll our own (~200 lines) vs. an Arduino library.

GNSS work blocked until the NEO-M9N arrives.

**Open questions**

- Madgwick implementation choice (roll-our-own vs. library).
- MapsForge reader implementations for embedded targets — most refs
  are JVM. Likely a port-or-replace problem.
- Enable PSRAM in `platformio.ini` once we need a map tile cache (the
  13.5 KB framebuffer alone fits in SRAM).

## 2026-04-30 — toolchain + initial commit

**Done**

- PlatformIO project building and flashing on the Olimex
  ESP32-S3-DevKit-LiPo. Upload over `/dev/cu.usbserial-210` at 460800
  baud (the CH340 USB-UART would not ACK the stub bump to 921600 — see
  `platformio.ini` for the note).
- Blink test in `src/main.cpp` drives the green user LED on **GPIO 38**
  (confirmed from the Rev B schematic — `LED1` net is `GPIO38\LED1`),
  with a Serial heartbeat at 115200. End-to-end round-trip verified by
  changing the blink rate and re-flashing.
- Initial commit pushed to `origin/main`
  (`github.com/philenox/bike_computer`).
- `.gitignore` covers PlatformIO, IDE, macOS, common build cruft, and
  defensive paths for secrets / local overrides.
- Confirmed via esptool: module is **ESP32-S3-WROOM-1 N8R8** (8 MB flash
  + 8 MB PSRAM). PSRAM is NOT yet enabled in `platformio.ini` — flag
  this when we start needing it (framebuffer, map tile cache).
- BMP390 / barometer removed from the spec. v1 will use GNSS-derived
  altitude only.

**Hardware on hand**

- Olimex ESP32-S3-DevKit-LiPo — working, flashed, blink confirmed.
- ICM-20948 IMU breakout — unboxed, untested.
- Adafruit Sharp Memory LCD 2.7" 400x240 — unboxed, untested.
- microSD breakout (SPI) + 32 GB card — unboxed, untested.
- 600 mAh LiPo — unboxed, untested. (Board can run from USB without it.)
- SparkFun NEO-M9N GNSS — **NOT YET ARRIVED**. Skip GPS-related work
  until the module is in hand.

**Open from CLAUDE.md (at session start)**

- Olimex pin breakouts confirmed for all GPIOs we've used so far
  (8, 9, 10, 11, 12, 13). 14 reserved for Sharp CS, not yet
  validated as broken out.
- SD/Sharp SPI bus sharing — RESOLVED (shared, see above).
- Madgwick implementation choice — still open.
- Embedded MapsForge readers — still open.
