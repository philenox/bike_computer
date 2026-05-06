#include <Arduino.h>
#include <Wire.h>
#include "ICM20948.h"
#include "Madgwick.h"

static constexpr int I2C_SDA = 8;
static constexpr int I2C_SCL = 9;
static constexpr uint32_t I2C_HZ = 400000;

// Hard-iron offsets from smoke_mag_cal (2026-05-05, current breadboard layout).
// Re-run smoke_mag_cal if anything ferrous moves near the IMU.
static constexpr float MAG_OFFSET_X_uT = +14.78f;
static constexpr float MAG_OFFSET_Y_uT = -53.55f;
static constexpr float MAG_OFFSET_Z_uT = +88.88f;

static constexpr uint32_t LOOP_HZ        = 200;
static constexpr uint32_t LOOP_PERIOD_US = 1000000UL / LOOP_HZ;
static constexpr uint32_t PRINT_PERIOD_MS = 200;  // 5 Hz

// β=0.1: converges from identity in ~3 s; mild enough not to look jittery.
// Higher β (tried 0.3) lets bad mag cal corrupt roll/pitch — don't increase
// unless the hard-iron cal has been thoroughly redone.
static Madgwick filter(0.1f);
static ICM20948  imu(0x69);

static uint32_t last_loop_us  = 0;
static uint32_t last_print_ms = 0;

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println("bike_computer: Madgwick AHRS smoke test");

    Wire.begin(I2C_SDA, I2C_SCL, I2C_HZ);

    if (!imu.begin()) {
        Serial.println("[icm] init failed");
        while (1) { delay(2000); Serial.println("[icm] halted"); }
    }

    imu.calibrateGyro();

    Serial.printf("[run] fusion @ %lu Hz, prints @ %lu ms (β=%.3f)\n",
                  (unsigned long)LOOP_HZ, (unsigned long)PRINT_PERIOD_MS,
                  filter.beta());
    Serial.println("---- streaming ----");
    last_loop_us = micros();
}

void loop() {
    uint32_t now = micros();
    if ((uint32_t)(now - last_loop_us) < LOOP_PERIOD_US) return;
    float dt = (now - last_loop_us) * 1e-6f;
    last_loop_us = now;

    float ax, ay, az, gx, gy, gz;
    imu.readAccelGyro(ax, ay, az, gx, gy, gz);

    // Mag at 100 Hz; cache the last valid reading so the filter always
    // has a reference (avoids silently dropping to 6-DoF mid-stream).
    static float mx_uT = 0.0f, my_uT = 0.0f, mz_uT = 0.0f;
    float mx_raw, my_raw, mz_raw;
    if (imu.readMag(mx_raw, my_raw, mz_raw)) {
        mx_uT = mx_raw - MAG_OFFSET_X_uT;
        my_uT = my_raw - MAG_OFFSET_Y_uT;
        mz_uT = mz_raw - MAG_OFFSET_Z_uT;
    }

    filter.update(gx, gy, gz, ax, ay, az, mx_uT, my_uT, mz_uT, dt);

    uint32_t ms = millis();
    if ((uint32_t)(ms - last_print_ms) >= PRINT_PERIOD_MS) {
        last_print_ms = ms;
        Serial.printf("roll=%+7.2f  pitch=%+7.2f  yaw=%+7.2f   "
                      "mag[uT]=(%+6.1f, %+6.1f, %+6.1f)\n",
                      filter.roll(), filter.pitch(), filter.yaw(),
                      mx_uT, my_uT, mz_uT);
    }
}
