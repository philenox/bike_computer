#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>

// Shared SPI bus (SD now, Sharp display later). These are the Arduino-ESP32
// framework defaults for SPI on ESP32-S3, so libraries that fall back to
// SPI defaults pick the same pins.
static constexpr int PIN_SCK   = 12;
static constexpr int PIN_MISO  = 13;
static constexpr int PIN_MOSI  = 11;
static constexpr int PIN_SD_CS = 10;
// PIN_SHARP_CS = 14 reserved for the display, not used here.

// Conservative SPI clock for prototype jumper wiring. SD cards happily run
// 20+ MHz over a clean trace, but signal integrity on long dupont wires
// gets dicey fast. Bump later once the bus moves to the perfboard.
static constexpr uint32_t SD_HZ = 4000000;

static const char* TEST_PATH = "/bike_test.txt";

static const char* cardTypeStr(uint8_t t) {
  switch (t) {
    case CARD_NONE: return "none";
    case CARD_MMC:  return "MMC";
    case CARD_SD:   return "SD";
    case CARD_SDHC: return "SDHC";
    default:        return "unknown";
  }
}

static void listRoot() {
  File root = SD.open("/");
  if (!root || !root.isDirectory()) {
    Serial.println("[sd] cannot open /");
    return;
  }
  Serial.println("[sd] root listing:");
  uint16_t n = 0;
  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    Serial.printf("  %s  %s%s  %llu bytes\n",
                  f.isDirectory() ? "DIR " : "FILE",
                  f.name(),
                  f.isDirectory() ? "/" : "",
                  (uint64_t)f.size());
    f.close();
    if (++n >= 64) { Serial.println("  (truncated at 64 entries)"); break; }
  }
  if (n == 0) Serial.println("  (empty)");
  root.close();
}

static bool writeReadRoundTrip() {
  char payload[64];
  int len = snprintf(payload, sizeof(payload),
                     "bike_computer SD smoke @ millis=%lu\n",
                     (unsigned long)millis());
  if (len <= 0 || len >= (int)sizeof(payload)) {
    Serial.println("[sd] payload format failed");
    return false;
  }

  // Remove first so each boot writes a deterministic file rather than
  // depending on FILE_WRITE's truncate-vs-append semantics.
  SD.remove(TEST_PATH);

  File w = SD.open(TEST_PATH, FILE_WRITE);
  if (!w) {
    Serial.printf("[sd] open %s for write failed\n", TEST_PATH);
    return false;
  }
  size_t wrote = w.write((const uint8_t*)payload, len);
  w.close();
  if ((int)wrote != len) {
    Serial.printf("[sd] short write: %u of %d\n", (unsigned)wrote, len);
    return false;
  }

  File r = SD.open(TEST_PATH, FILE_READ);
  if (!r) {
    Serial.printf("[sd] open %s for read failed\n", TEST_PATH);
    return false;
  }
  char readback[64] = {0};
  int got = r.readBytes(readback, sizeof(readback) - 1);
  r.close();

  Serial.printf("[sd] wrote %d bytes, read back %d bytes\n", len, got);
  Serial.printf("[sd] content: %s", readback);  // payload ends in \n

  if (got != len || memcmp(payload, readback, len) != 0) {
    Serial.println("[sd] MISMATCH between write and readback");
    return false;
  }
  Serial.println("[sd] write/read round-trip OK");
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("bike_computer: microSD SPI smoke test");
  Serial.printf("[spi] SCK=%d MOSI=%d MISO=%d  SD_CS=%d  clock=%lu Hz\n",
                PIN_SCK, PIN_MOSI, PIN_MISO, PIN_SD_CS,
                (unsigned long)SD_HZ);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SD_CS);
  if (!SD.begin(PIN_SD_CS, SPI, SD_HZ)) {
    Serial.println("[sd] mount failed -- check wiring, card seated, FAT32 formatted");
    while (1) { delay(2000); Serial.println("[sd] halted"); }
  }

  uint8_t type = SD.cardType();
  Serial.printf("[sd] card type: %s\n", cardTypeStr(type));
  if (type == CARD_NONE) {
    Serial.println("[sd] no card detected after begin()");
    while (1) { delay(2000); Serial.println("[sd] halted"); }
  }

  uint64_t cardBytes = SD.cardSize();
  Serial.printf("[sd] card size:  %llu bytes (%.2f GB)\n",
                cardBytes, cardBytes / (1024.0 * 1024.0 * 1024.0));
  Serial.printf("[sd] FAT total:  %llu bytes\n", (uint64_t)SD.totalBytes());
  Serial.printf("[sd] FAT used:   %llu bytes\n", (uint64_t)SD.usedBytes());

  listRoot();
  writeReadRoundTrip();
  Serial.println("---- done ----");
}

void loop() {
  delay(5000);
}
