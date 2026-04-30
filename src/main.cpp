#include <Arduino.h>

// User LED (green) on the Olimex ESP32-S3-DevKit-LiPo, per schematic Rev B:
// net label "GPIO38\LED1" wired to LED1 marked "USER LED".
static constexpr uint8_t LED_PIN = 38;

// Half-period of the blink. Change this and re-flash to confirm the toolchain
// is round-tripping: a visibly different rate proves the new binary is live.
static constexpr uint32_t BLINK_HALF_PERIOD_MS = 250;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  Serial.println();
  Serial.println("bike_computer: blink test online");
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  delay(BLINK_HALF_PERIOD_MS);
  digitalWrite(LED_PIN, LOW);
  delay(BLINK_HALF_PERIOD_MS);
  Serial.print(".");
}
