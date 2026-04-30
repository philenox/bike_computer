# STATUS

Living handoff doc — overwrite freely. Most recent session at top.

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

## Next session

User wants to start playing with peripherals. Suggested order
(easy → involved):

1. **IMU (ICM-20948) on I2C** — simplest of the three; just read WHO_AM_I
   over I2C as a smoke test, then raw accel/gyro/mag. No display needed
   yet, just Serial output. Confirms I2C bus wiring on the Olimex pins
   we end up choosing.
2. **microSD on SPI** — list root directory, write a test file, read it
   back. Establishes the SPI bus and file-system layer that the Sharp
   display will share.
3. **Sharp Memory LCD on SPI** — once SD is up, share or split the SPI
   bus (open question — see CLAUDE.md). First goal: clear the screen
   and draw a single line of text. Framebuffer is 13.5 KB, must live in
   MCU RAM.

**Pin assignments are still TBD** — confirm Olimex pinout before wiring.
Don't invent pin numbers; check the board PDF or have the user verify.

**Open from CLAUDE.md still unresolved**

- Confirm Olimex board pinout matches ESP32-S3-DevKitC-1 standard.
- Decide whether SD and Sharp display share an SPI bus.
- Choose Madgwick implementation (roll our own vs. library).
- Investigate embedded-friendly MapsForge readers.
