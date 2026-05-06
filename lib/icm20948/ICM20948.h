#pragma once
#include <Arduino.h>
#include <Wire.h>

// Driver for the ICM-20948 9-DoF IMU (accel + gyro + AK09916 mag).
// The AK09916 is exposed through the ICM's I2C bypass mode at 0x0C and
// is initialised automatically in begin().
//
// Typical usage:
//   ICM20948 imu(0x69);
//   Wire.begin(SDA_PIN, SCL_PIN, 400000);
//   if (!imu.begin()) { /* wiring fault */ }
//   imu.calibrateGyro();          // hold still for ~4 s
//   // then every loop iteration:
//   float ax, ay, az, gx, gy, gz;
//   imu.readAccelGyro(ax, ay, az, gx, gy, gz);
//   float mx, my, mz;
//   if (imu.readMag(mx, my, mz)) { /* fresh mag sample */ }
//
// Hard-iron mag offsets are NOT applied here so the driver stays
// hardware-generic. The caller subtracts calibration values.

class ICM20948 {
public:
    explicit ICM20948(uint8_t addr = 0x69);

    // Initialise ICM-20948 and AK09916. Wire.begin() must be called first.
    // Returns false if WHO_AM_I doesn't match — indicates a wiring fault.
    bool begin(TwoWire &wire = Wire);

    // Blocking gyro bias calibration.
    // Prints a countdown for countdown_s seconds then averages `samples`
    // readings at ~200 Hz to estimate the zero-rate bias. Hold still.
    void calibrateGyro(int countdown_s = 3, int samples = 200);

    // Read accel (g) and gyro (rad/s). Gyro is bias-corrected using the
    // result of the last calibrateGyro() call (zero if never called).
    void readAccelGyro(float &ax, float &ay, float &az,
                       float &gx, float &gy, float &gz);

    // Read magnetometer (µT, raw). Returns false if DRDY not set.
    // Hard-iron calibration offsets are NOT applied — caller's responsibility.
    bool readMag(float &mx, float &my, float &mz);

private:
    TwoWire *_wire  = nullptr;
    uint8_t  _addr;
    float    _gx_bias = 0.0f;  // rad/s
    float    _gy_bias = 0.0f;
    float    _gz_bias = 0.0f;

    bool writeReg (uint8_t dev, uint8_t reg, uint8_t val);
    bool readRegs (uint8_t dev, uint8_t reg, uint8_t *buf, uint8_t n);
    bool readReg  (uint8_t dev, uint8_t reg, uint8_t &v);
    bool selectBank(uint8_t bank);
};
