# CLAUDE.md

Context for Claude Code working on this project.

## Project overview

A DIY multi-mode outdoor GPS navigation device, targeting both cycling and
backpacking use cases. Pre-loaded offline maps and routes (no on-the-fly
download for v1). Sunlight-readable display. USB-C top-up charging assumed
as part of the workflow (overnight charging, not a multi-week off-grid device).

This is a hobby project. Practical utility is secondary to the project being
fun to work on. There are commercial alternatives (Garmin Edge, Wahoo
ELEMNT) that compete strongly on price/feature for cycling specifically;
this project is not trying to beat them on their home turf. The compelling
niche is the dual-mode (cycle + hike) use case with a hackable software
stack.

## Hardware

All hardware ordered and is being prototyped on dev boards / breakouts (no
custom PCB for v1).

- **MCU**: Olimex ESP32-S3-DevKit-LiPo
  - ESP32-S3 dual-core Xtensa LX7 @ 240 MHz
  - 8 MB PSRAM, 8 MB flash (confirm on arrival)
  - Built-in LiPo charger and USB-C
  - JTAG debugger onboard
- **GNSS**: SparkFun NEO-M9N Breakout, chip antenna, Qwiic
  - 4-constellation concurrent (GPS, GLONASS, Galileo, BeiDou)
  - Onboard backup battery for hot starts
  - Default I2C addr 0x42; also supports UART and SPI
- **IMU**: ICM-20948 breakout (9-DoF: accel + gyro + magnetometer)
  - Standard I2C, no clock-stretching quirks (unlike BNO08x family)
  - Fusion done on the ESP32 (Madgwick or Mahony filter)
- **Barometer**: Adafruit BMP390 STEMMA QT
  - Pressure-based altimetry and weather trend data
  - Very low power (~3 µA in low-power mode)
- **Display**: Adafruit Sharp Memory LCD breakout, 2.7" 400x240 monochrome
  - Reflective (sunlight-readable, brighter in sun)
  - SPI write-only; entire 13.5 KB framebuffer must be held in MCU RAM
  - Static power ~50 µA, refresh ~5 mA
- **Storage**: microSD breakout (SPI)
  - 32 GB card, FAT32
  - Holds offline maps (MapsForge format) and pre-loaded GPX routes
- **Battery**: 600 mAh LiPo
  - Target ~16 hours hiking, ~6 hours cycling between charges

## Software stack

- **Build system**: PlatformIO (CLI)
- **Framework**: Arduino-ESP32 (for library ecosystem; many breakouts above
  have well-maintained Adafruit/SparkFun Arduino libraries)
- **Map format**: MapsForge (vector) — efficient, well-documented, used by
  the Wahoo ELEMNT family among others
- **Reference projects worth reading** (do not copy verbatim):
  - github.com/lspr98/bike-computer-32 — closest prior art for ESP32 +
    breadcrumb nav + offline maps. Map preprocessing pipeline is in
    `software/cpp/`.
  - github.com/jgauchia/IceNav-v3 — more ambitious ESP32-S3 + LVGL stack;
    useful patterns for SD-card map handling.
  - github.com/hishizuka/pizero_bikecomputer — Pi-based but the hardware
    integration patterns are useful reference.

## Modes

The firmware should be architected around explicit modes from day one. Each
mode has its own duty cycle for GPS / display / IMU.

- `RIDING` — GPS at 1 Hz, IMU active for heading reference, display
  refreshes on movement, heading-up map orientation.
- `HIKING` — GPS at 1 fix per 10–30s with power-save mode, display only
  refreshes on user button press, IMU low-power or off, north-up map
  orientation.
- `LOGGING` — GPS only, display off, log GPX track to SD. Used for
  passive tracking when nav isn't needed.
- `SLEEP` — ESP32 deep sleep (~10 µA), GPS fully powered down via load
  switch (TBD on prototype, may need a MOSFET added). Wake on button.

## Workflow

- Pre-load maps and GPX routes to SD card on a laptop before each trip.
  No on-device file transfer for v1.
- USB-C charging is assumed nightly during multi-day trips, topping up from
  a power bank carried for other reasons.
- The microSD slot is externally accessible during prototyping. Future
  versions may move it internal once USB MSC mode is implemented.

## V1 scope (locked, do not expand without discussion)

In scope:

- Multi-constellation GNSS fix and 1 Hz position logging
- MapsForge offline map rendering on the Sharp display
- Breadcrumb-style display of pre-loaded GPX route on the map
- Mode switching between RIDING / HIKING / LOGGING / SLEEP
- IMU-derived heading reference (Madgwick filter on raw 9-DoF data)
- BMP390-derived altitude and climb rate
- Pre-loaded SD workflow only

Out of scope for v1, deferred to later:

- Turn-by-turn cue-based navigation (breadcrumb only for v1)
- USB MSC mode for drag-and-drop file transfer
- WiFi or BLE connectivity of any kind
- Companion phone app
- Custom PCB / enclosure
- Solar charging
- ANT+ sensor support (heart rate, cadence, etc.)

## Coding preferences

- Iterative, simple v1 implementations. Prefer "make it work" over "make
  it general" on first pass. Defer complexity until it's earned.
- C++ is fine where it's idiomatic in Arduino-land; plain C where it isn't.
- Avoid heap allocation in the hot path (sensor reads, render loop). Prefer
  static buffers and stack allocation for predictable performance.
- The Sharp Memory LCD framebuffer (13.5 KB) is one such static buffer.
- Prefer reading register / data sheets directly over guessing from library
  source when behaviour is unclear.
- When suggesting library additions to `platformio.ini`, pin to specific
  versions, not floating ranges. Embedded library churn breaks builds.
- Comments should explain *why*, not *what*. Hardware quirks and non-obvious
  pin choices deserve comments; trivial code does not.

## Honesty preferences

I value Claude correcting itself honestly when it gets something wrong over
defending an earlier statement. If you realise during a session that an
earlier suggestion was wrong, say so plainly and explain what changed.

I'd rather hear "this is a known quirk, here's the workaround" than discover
the quirk myself three hours into debugging. If a library, breakout, or
chip has a well-known gotcha, surface it before I hit it.

Do not invent pin assignments, library APIs, or datasheet values. If you're
not sure, say so and I'll check.

## Repo layout (placeholder, refine as project grows)

```
bike_tracker/
├── CLAUDE.md           # this file
├── README.md           # human-facing overview
├── platformio.ini      # build config
├── src/
│   └── main.cpp        # entry point
├── lib/                # project-specific libraries
├── include/            # shared headers
└── docs/
    ├── pinout.md       # GPIO assignments (TBD)
    └── power-budget.md # measured vs estimated power draws
```

## Pin assignments

TBD — to be filled in once the Olimex board arrives and pin conflicts are
resolved between SD (SPI), Sharp display (SPI, can share bus), GPS (I2C),
IMU (I2C), and BMP390 (I2C).

## Open questions for early sessions

- Confirm Olimex board pinout matches ESP32-S3-DevKitC-1 standard.
- Decide whether to share the SPI bus between SD card and Sharp display
  (saves pins, may complicate timing) or use separate buses.
- Choose Madgwick filter implementation: roll our own (~200 lines, well
  understood) or pull in an existing Arduino library.
- Investigate MapsForge reader implementations for embedded targets — most
  reference implementations are JVM-based and will need porting or
  replacement.
