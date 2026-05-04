#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>

// Shared SPI bus with the SD card. Sharp is write-only so MISO is unused
// from the display's perspective, but we still bind it on SPI.begin so the
// bus is fully configured for whichever device is active.
static constexpr int PIN_SCK      = 12;
static constexpr int PIN_MISO     = 13;
static constexpr int PIN_MOSI     = 11;
static constexpr int PIN_SHARP_CS = 7;

static constexpr uint16_t W = 400;
static constexpr uint16_t H = 240;

// Sharp library color convention: BLACK = ink/dark pixel, WHITE = reflective.
#define COLOR_BLACK 0
#define COLOR_WHITE 1

// Hardware SPI through the shared SPI peripheral. 2 MHz is the library's
// default; conservative for prototype wiring, plenty for an unmanaged
// 12 KB framebuffer push at ~10 fps if we ever want it.
Adafruit_SharpMem display(&SPI, PIN_SHARP_CS, W, H, 2000000);

static uint32_t tick = 0;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("bike_computer: Sharp Memory LCD smoke test");
  Serial.printf("[spi]   SCK=%d MISO=%d MOSI=%d  CS=%d  size=%ux%u\n",
                PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SHARP_CS, W, H);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);

  if (!display.begin()) {
    Serial.println("[sharp] begin() failed");
    while (1) { delay(2000); Serial.println("[sharp] halted"); }
  }
  Serial.println("[sharp] init OK");

  display.clearDisplay();
  display.setTextColor(COLOR_BLACK);

  // Border just inside the screen edge.
  display.drawRect(0, 0, W, H, COLOR_BLACK);

  display.setCursor(20, 20);
  display.setTextSize(3);
  display.println("bike_computer");

  display.setCursor(20, 60);
  display.setTextSize(2);
  display.println("Sharp 400x240");

  display.setCursor(20, 100);
  display.setTextSize(1);
  display.println("display smoke test");

  display.refresh();
  Serial.println("[sharp] initial frame pushed");
}

void loop() {
  delay(1000);
  ++tick;

  // Bottom-left tick counter. Three jobs:
  //   1. Visible heartbeat to confirm refresh() is actually pushing pixels.
  //   2. Forces VCOM toggle (the library flips the VCOM bit on each refresh,
  //      preventing the LCD's DC-bias damage mode).
  //   3. Catches "first frame works but subsequent ones don't" bugs.
  char buf[32];
  snprintf(buf, sizeof(buf), "tick %lu", (unsigned long)tick);

  display.fillRect(20, 200, 200, 20, COLOR_WHITE);
  display.setCursor(20, 200);
  display.setTextSize(2);
  display.setTextColor(COLOR_BLACK);
  display.print(buf);
  display.refresh();

  Serial.printf("[sharp] %s\n", buf);
}
