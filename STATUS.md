# STATUS

Living handoff doc — overwrite freely. Most recent session at top.

## 2026-05-05 — Madgwick AHRS yaw fixed; all smoke tests complete

**Done this session**

- Re-ran `smoke_mag_cal` with thorough rotation (26k samples, full
  sphere coverage). New offsets:
  - `MAG_OFFSET_X_uT = +14.78` (was −3.45 — axis under-covered before)
  - `MAG_OFFSET_Y_uT = -53.55` (unchanged)
  - `MAG_OFFSET_Z_uT = +88.88` (was +108.75)
  Span is now balanced at X=101.9, Y=89.4, Z=109.1 µT (was 81/81/217).
  The old Z-dominated sphere was the root cause of the yaw drift.
- Updated offsets in `src/smoke_madgwick/main.cpp`.
- **Yaw drift: ~0.02°/s** (was 0.7°/s). 35× improvement.
- **360° rotation validated**: yaw sweeps smoothly, crosses ±180°
  boundary correctly, re-anchors cleanly. Roll/pitch stay bounded.
- β=0.3 experiment: tried and reverted. Higher β amplified the bad cal
  into roll/pitch contamination (~18°/s drift). The cal was the
  bottleneck, not β. Kept at 0.1.

**What we learned about the axis/cal situation**
- No axis transform needed — X/Y horizontal, Z vertical confirmed by
  rotation diagnostic (mz constant during flat spin).
- The first cal was under-covering X, giving a weak horizontal
  projection (~5 µT). After thorough recal it's ~20 µT — enough to
  anchor the filter against gyro drift.
- Re-run `smoke_mag_cal` whenever the breadboard layout changes
  (anything ferrous moving near the IMU will shift offsets).

**Hardware on hand**

- Olimex ESP32-S3-DevKit-LiPo — working.
- ICM-20948 IMU — **TESTED** (Madgwick fusion working, yaw stable).
- microSD breakout + 32 GB card — **TESTED**.
- Adafruit Sharp Memory LCD 2.7" 400x240 — **TESTED**.
- 600 mAh LiPo — unboxed, untested.
- SparkFun NEO-M9N GNSS — **NOT YET ARRIVED**.

**Next session**

All bench peripherals validated. The natural next move is the first
"real" app sketch (`src/app/`):

1. **Mode state machine** — RIDING/HIKING/LOGGING/SLEEP skeleton, even
   if all modes do the same thing initially.
2. **IMU heading on the display** — draw the Madgwick yaw angle as a
   compass rose or simple heading number on the Sharp LCD. Proves the
   sensor → filter → display pipeline end-to-end.
3. **GNSS bring-up** once the NEO-M9N arrives — I2C at 0x42,
   SparkFun u-blox library.

**Mechanical reminders**

- Port suffix drifts on reconnect: `ls /dev/cu.usbserial-*` and update
  `platformio.ini` if needed. Was `/dev/cu.usbserial-110` this session.
- `python scripts/log_serial.py [seconds]` + `tail -F logs/latest.log`
  in a second terminal for live output.
- Re-run `smoke_mag_cal` if breadboard layout changes.

## 2026-05-04 — breadboard migration + Sharp display bring-up

**Done**

- **Solderless breadboard**: ESP32-S3 + IMU + SD + Sharp display all
  on one board, no more loose dupont wires holding the project
  together. Validated with the smoke tests:
  - `smoke_imu` ✓ (|a| ≈ 1.03 g flat)
  - `smoke_sd`  ✓ (mount + listing + write/read round-trip)
  - `smoke_display` ✓ (init, full-frame draw, refresh ticking)
- **Sharp Memory LCD bring-up** (Adafruit 2.7" 400x240). Hardware
  SPI through the shared bus, software-driven VCOM toggle (EXTMODE
  and EXTCOMIN both tied to GND, library handles bit flip in
  `refresh()`). Boot screen draws three lines of text + a border;
  loop updates a tick counter once per second to exercise refresh.
- **Pin assignments now locked in** (constrained by which header
  bank is breadboard-accessible on the Olimex):
  - I2C: SDA=8, SCL=9
  - Shared SPI: SCK=12, MISO=13, MOSI=11
  - SD CS=10
  - Sharp CS=7 (relocated from 14, which isn't on the accessible
    bank)
  - Spare GPIOs: 4, 5, 6, 15, 16, 17, 18
- Library deps pinned: `Adafruit SHARP Memory Display @ 1.1.4`,
  `Adafruit GFX Library @ 1.12.6`.
- CH340 USB-UART name has now drifted across `-210` → `-110` →
  `-10`. Update `platformio.ini` whenever it changes.

**Hardware on hand**

- Olimex ESP32-S3-DevKit-LiPo — working.
- ICM-20948 IMU — **TESTED** (direct pin headers, no Qwiic).
- microSD breakout + 32 GB card — **TESTED**.
- Adafruit Sharp Memory LCD 2.7" 400x240 — **TESTED**.
- 600 mAh LiPo — unboxed, untested.
- SparkFun NEO-M9N GNSS — **NOT YET ARRIVED**.

## Next session

All bench peripherals are now validated; the only hardware bring-up
work remaining is the GNSS once the NEO-M9N arrives. Software-side
candidates ranked by leverage:

1. **Madgwick fusion on the IMU** — turn the raw 9-DoF stream into
   an orientation estimate. Open question: roll our own (~200
   lines, well-understood) vs. an Arduino library. Roll-our-own is
   probably the right call given how much the project's about
   "fun to work on" rather than fastest path.
2. **First "real" main app sketch** — a `src/app/` env that owns
   the actual application loop, with mode state machine
   (RIDING/HIKING/LOGGING/SLEEP per CLAUDE.md). Could start as
   little more than "draw IMU heading on the display."
3. **MapsForge tile reading** investigation. JVM-only references,
   so this is mostly a "decide between port-or-replace" research
   spike, not coding yet.
4. **Battery test** — power the rig from the 600 mAh LiPo, measure
   draw, sanity-check the 6h cycling / 16h hiking targets in
   `CLAUDE.md`. Useful even before GNSS to baseline the
   non-GNSS power budget.

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
