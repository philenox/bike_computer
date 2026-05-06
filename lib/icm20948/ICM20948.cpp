#include "ICM20948.h"

// ---------- Register map -------------------------------------------------

// Bank 0
static constexpr uint8_t REG_BANK_SEL     = 0x7F;
static constexpr uint8_t B0_WHO_AM_I      = 0x00;
static constexpr uint8_t B0_USER_CTRL     = 0x03;
static constexpr uint8_t B0_PWR_MGMT_1   = 0x06;
static constexpr uint8_t B0_PWR_MGMT_2   = 0x07;
static constexpr uint8_t B0_INT_PIN_CFG  = 0x0F;
static constexpr uint8_t B0_ACCEL_XOUT_H = 0x2D;  // accel[6] + gyro[6]
static constexpr uint8_t ICM_WHO_AM_I_VAL = 0xEA;

// AK09916 magnetometer (exposed via I2C bypass at 0x0C)
static constexpr uint8_t AK_ADDR           = 0x0C;
static constexpr uint8_t AK_WIA2           = 0x01;
static constexpr uint8_t AK_ST1            = 0x10;
static constexpr uint8_t AK_HXL            = 0x11;
static constexpr uint8_t AK_CNTL2          = 0x31;
static constexpr uint8_t AK_CNTL3          = 0x32;
static constexpr uint8_t AK_WHO_AM_I_VAL   = 0x09;
static constexpr uint8_t AK_MODE_CONT_100HZ = 0x08;

// ---------- Scale factors -------------------------------------------------
static constexpr float ACCEL_LSB_PER_G  = 16384.0f;  // ±2 g  (reset default)
static constexpr float GYRO_LSB_PER_DPS = 131.0f;    // ±250°/s (reset default)
static constexpr float DPS_TO_RAD       = 0.017453292519943295f;
static constexpr float MAG_UT_PER_LSB   = 0.15f;

// ---------- Byte-order helpers -------------------------------------------
// ICM-20948 sensor registers are big-endian (HIGH byte at lower address).
static inline int16_t big16(uint8_t hi, uint8_t lo) {
    return (int16_t)(((uint16_t)hi << 8) | lo);
}
// AK09916 output registers are little-endian (LOW byte at lower address).
static inline int16_t lit16(uint8_t lo, uint8_t hi) {
    return (int16_t)(((uint16_t)hi << 8) | lo);
}

// =========================================================================

ICM20948::ICM20948(uint8_t addr) : _addr(addr) {}

bool ICM20948::begin(TwoWire &wire) {
    _wire = &wire;

    // Reset, then verify WHO_AM_I before proceeding.
    selectBank(0);
    writeReg(_addr, B0_PWR_MGMT_1, 0x80);  // device reset
    delay(100);

    uint8_t who = 0;
    if (!readReg(_addr, B0_WHO_AM_I, who) || who != ICM_WHO_AM_I_VAL) {
        Serial.printf("[icm] WHO_AM_I=0x%02X (want 0x%02X)\n", who, ICM_WHO_AM_I_VAL);
        return false;
    }

    writeReg(_addr, B0_PWR_MGMT_1, 0x01);  // wake, auto-select best clock
    delay(20);
    writeReg(_addr, B0_PWR_MGMT_2, 0x00);  // all sensors on
    delay(20);

    // I2C bypass: expose AK09916 directly on the external I2C bus at 0x0C.
    writeReg(_addr, B0_USER_CTRL, 0x00);
    writeReg(_addr, B0_INT_PIN_CFG, 0x02);
    delay(10);

    // --- Initialise AK09916 ---
    uint8_t ak_who = 0;
    if (!readReg(AK_ADDR, AK_WIA2, ak_who) || ak_who != AK_WHO_AM_I_VAL) {
        Serial.printf("[ak] WIA2=0x%02X (want 0x%02X)\n", ak_who, AK_WHO_AM_I_VAL);
        return false;
    }
    writeReg(AK_ADDR, AK_CNTL3, 0x01);  // soft reset
    delay(10);
    writeReg(AK_ADDR, AK_CNTL2, AK_MODE_CONT_100HZ);
    delay(10);

    return true;
}

void ICM20948::calibrateGyro(int countdown_s, int samples) {
    Serial.println();
    Serial.println("[cal] hold the device STILL — gyro bias cal in...");
    for (int i = countdown_s; i >= 1; --i) {
        Serial.printf("[cal]   %d\n", i);
        delay(1000);
    }

    selectBank(0);
    long sx = 0, sy = 0, sz = 0;
    for (int i = 0; i < samples; ++i) {
        uint8_t b[12];
        readRegs(_addr, B0_ACCEL_XOUT_H, b, 12);
        sx += big16(b[6], b[7]);
        sy += big16(b[8], b[9]);
        sz += big16(b[10], b[11]);
        delayMicroseconds(5000);  // 200 Hz
    }
    _gx_bias = (sx / (float)samples) / GYRO_LSB_PER_DPS * DPS_TO_RAD;
    _gy_bias = (sy / (float)samples) / GYRO_LSB_PER_DPS * DPS_TO_RAD;
    _gz_bias = (sz / (float)samples) / GYRO_LSB_PER_DPS * DPS_TO_RAD;

    Serial.printf("[cal] gyro bias dps: %+.3f %+.3f %+.3f\n",
                  _gx_bias / DPS_TO_RAD, _gy_bias / DPS_TO_RAD, _gz_bias / DPS_TO_RAD);
    Serial.println("[cal] done — moving the device is now fine");
}

void ICM20948::readAccelGyro(float &ax, float &ay, float &az,
                              float &gx, float &gy, float &gz) {
    selectBank(0);
    uint8_t b[12];
    readRegs(_addr, B0_ACCEL_XOUT_H, b, 12);
    ax = big16(b[0], b[1]) / ACCEL_LSB_PER_G;
    ay = big16(b[2], b[3]) / ACCEL_LSB_PER_G;
    az = big16(b[4], b[5]) / ACCEL_LSB_PER_G;
    gx = big16(b[6],  b[7])  / GYRO_LSB_PER_DPS * DPS_TO_RAD - _gx_bias;
    gy = big16(b[8],  b[9])  / GYRO_LSB_PER_DPS * DPS_TO_RAD - _gy_bias;
    gz = big16(b[10], b[11]) / GYRO_LSB_PER_DPS * DPS_TO_RAD - _gz_bias;
}

bool ICM20948::readMag(float &mx, float &my, float &mz) {
    uint8_t st1 = 0;
    if (!readReg(AK_ADDR, AK_ST1, st1) || !(st1 & 0x01)) return false;
    uint8_t b[8];  // HXL,HXH,HYL,HYH,HZL,HZH,TMPS,ST2
    if (!readRegs(AK_ADDR, AK_HXL, b, 8)) return false;
    mx = lit16(b[0], b[1]) * MAG_UT_PER_LSB;
    my = lit16(b[2], b[3]) * MAG_UT_PER_LSB;
    mz = lit16(b[4], b[5]) * MAG_UT_PER_LSB;
    return true;
}

// ---------- Private I2C helpers ------------------------------------------

bool ICM20948::writeReg(uint8_t dev, uint8_t reg, uint8_t val) {
    _wire->beginTransmission(dev);
    _wire->write(reg);
    _wire->write(val);
    return _wire->endTransmission() == 0;
}

bool ICM20948::readRegs(uint8_t dev, uint8_t reg, uint8_t *buf, uint8_t n) {
    _wire->beginTransmission(dev);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) return false;
    if (_wire->requestFrom(dev, n) != n) return false;
    for (uint8_t i = 0; i < n; i++) buf[i] = _wire->read();
    return true;
}

bool ICM20948::readReg(uint8_t dev, uint8_t reg, uint8_t &v) {
    return readRegs(dev, reg, &v, 1);
}

bool ICM20948::selectBank(uint8_t bank) {
    return writeReg(_addr, REG_BANK_SEL, (uint8_t)((bank & 0x03) << 4));
}
