#include <Arduino.h>
#include <Wire.h>
#include "Madgwick.h"

// I2C bus / IMU registers — same as smoke_imu. Kept inline rather than
// factored out into a shared header until we have a third copy that
// wants to do something different.
static constexpr int I2C_SDA = 8;
static constexpr int I2C_SCL = 9;
static constexpr uint32_t I2C_HZ = 400000;

static constexpr uint8_t ICM_ADDR = 0x69;
static constexpr uint8_t REG_BANK_SEL    = 0x7F;
static constexpr uint8_t B0_WHO_AM_I     = 0x00;
static constexpr uint8_t B0_USER_CTRL    = 0x03;
static constexpr uint8_t B0_PWR_MGMT_1   = 0x06;
static constexpr uint8_t B0_PWR_MGMT_2   = 0x07;
static constexpr uint8_t B0_INT_PIN_CFG  = 0x0F;
static constexpr uint8_t B0_ACCEL_XOUT_H = 0x2D;
static constexpr uint8_t WHO_AM_I_ICM    = 0xEA;

static constexpr uint8_t AK_ADDR  = 0x0C;
static constexpr uint8_t AK_WIA2  = 0x01;
static constexpr uint8_t AK_ST1   = 0x10;
static constexpr uint8_t AK_HXL   = 0x11;
static constexpr uint8_t AK_CNTL2 = 0x31;
static constexpr uint8_t AK_CNTL3 = 0x32;
static constexpr uint8_t WHO_AM_I_AK = 0x09;

static constexpr float ACCEL_LSB_PER_G   = 16384.0f;
static constexpr float GYRO_LSB_PER_DPS  = 131.0f;
static constexpr float MAG_UT_PER_LSB    = 0.15f;
static constexpr float DPS_TO_RAD        = 0.017453292519943295f;

// Hard-iron offsets measured by smoke_mag_cal on the current hardware
// build (breadboard layout, 2026-05-04 session). The Z offset is
// large because something nearby on the breadboard is generating a
// steady field on that axis. Re-cal whenever the physical layout
// changes (perfboard, enclosure).
static constexpr float MAG_OFFSET_X_uT = +14.78f;
static constexpr float MAG_OFFSET_Y_uT = -53.55f;
static constexpr float MAG_OFFSET_Z_uT = +88.88f;

// Filter loop runs much faster than display/print rate. 200 Hz gives
// Madgwick lots of margin to track fast rotations; 5 Hz prints give
// a readable serial stream.
static constexpr uint32_t LOOP_HZ        = 200;
static constexpr uint32_t LOOP_PERIOD_US = 1000000UL / LOOP_HZ;
static constexpr uint32_t PRINT_PERIOD_MS = 200;  // 5 Hz

// Mag continuous mode 4 (0x08) = 100 Hz. Madgwick will get a fresh
// mag sample every other fusion step at most — plenty for slow yaw
// drift correction.
static constexpr uint8_t  AK_MODE_CONTINUOUS_100HZ = 0x08;

// β: starting at 0.1 -- aggressive enough to converge in a few seconds
// from the identity quaternion, mild enough to not look jittery. Drop
// toward 0.033 (the paper's recommendation) once we trust the cal.
//
// Tried β=0.3 to compensate for weak horizontal mag projection; that
// made yaw drift jump to ~18°/s AND started corrupting roll/pitch (the
// bad mag is now strong enough in the combined gradient to contaminate
// tilt). The mag cal itself is the bottleneck, not β.
static Madgwick filter(0.1f);

// Gyro bias measured at boot, in dps. Subtracted from every read.
static float gx_bias_dps = 0.0f;
static float gy_bias_dps = 0.0f;
static float gz_bias_dps = 0.0f;

// ---------- I2C helpers (same shape as smoke_imu) -------------------------

static bool writeReg(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool readRegs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, n) != n) return false;
  for (uint8_t i = 0; i < n; ++i) buf[i] = Wire.read();
  return true;
}

static bool readReg(uint8_t addr, uint8_t reg, uint8_t &v) {
  return readRegs(addr, reg, &v, 1);
}

static bool selectBank(uint8_t bank) {
  return writeReg(ICM_ADDR, REG_BANK_SEL, (uint8_t)((bank & 0x03) << 4));
}

static int16_t be16(uint8_t hi, uint8_t lo) {
  return (int16_t)(((uint16_t)hi << 8) | lo);
}
static int16_t le16(uint8_t lo, uint8_t hi) {
  return (int16_t)(((uint16_t)hi << 8) | lo);
}

// ---------- IMU init / reads ---------------------------------------------

static bool initIMU() {
  selectBank(0);
  writeReg(ICM_ADDR, B0_PWR_MGMT_1, 0x80);  // reset
  delay(100);

  uint8_t who = 0;
  if (!readReg(ICM_ADDR, B0_WHO_AM_I, who) || who != WHO_AM_I_ICM) {
    Serial.printf("[icm] WHO_AM_I = 0x%02X (want 0x%02X)\n", who, WHO_AM_I_ICM);
    return false;
  }

  if (!writeReg(ICM_ADDR, B0_PWR_MGMT_1, 0x01)) return false;  // wake
  delay(20);
  if (!writeReg(ICM_ADDR, B0_PWR_MGMT_2, 0x00)) return false;  // all on
  delay(20);

  // Bypass mode → AK09916 reachable directly at 0x0C.
  if (!writeReg(ICM_ADDR, B0_USER_CTRL, 0x00)) return false;
  if (!writeReg(ICM_ADDR, B0_INT_PIN_CFG, 0x02)) return false;
  delay(10);
  return true;
}

static bool initMag() {
  uint8_t who = 0;
  if (!readReg(AK_ADDR, AK_WIA2, who) || who != WHO_AM_I_AK) {
    Serial.printf("[ak] WIA2 = 0x%02X (want 0x%02X)\n", who, WHO_AM_I_AK);
    return false;
  }
  writeReg(AK_ADDR, AK_CNTL3, 0x01);  // soft reset
  delay(10);
  if (!writeReg(AK_ADDR, AK_CNTL2, AK_MODE_CONTINUOUS_100HZ)) return false;
  delay(10);
  return true;
}

static void readAccelGyroRaw(int16_t &ax, int16_t &ay, int16_t &az,
                             int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t b[12];  // 6 accel + 6 gyro (skip temp)
  readRegs(ICM_ADDR, B0_ACCEL_XOUT_H, b, 12);
  ax = be16(b[0],  b[1]);  ay = be16(b[2], b[3]);  az = be16(b[4],  b[5]);
  gx = be16(b[6],  b[7]);  gy = be16(b[8], b[9]);  gz = be16(b[10], b[11]);
}

// Returns true only when fresh mag data was available this call.
static bool readMagRaw(int16_t &mx, int16_t &my, int16_t &mz) {
  uint8_t st1 = 0;
  if (!readReg(AK_ADDR, AK_ST1, st1) || !(st1 & 0x01)) return false;
  uint8_t b[8];  // HXL..HZH, TMPS, ST2 (ST2 read releases DRDY latch)
  if (!readRegs(AK_ADDR, AK_HXL, b, 8)) return false;
  mx = le16(b[0], b[1]);
  my = le16(b[2], b[3]);
  mz = le16(b[4], b[5]);
  return true;
}

// ---------- Gyro-bias auto-cal -------------------------------------------

static void calibrateGyroBias() {
  Serial.println();
  Serial.println("[cal] hold the device STILL — gyro bias cal in...");
  for (int i = 3; i >= 1; --i) {
    Serial.printf("[cal]   %d\n", i);
    delay(1000);
  }

  // ~1 second of samples at the loop rate. Average gives the bias.
  constexpr int N = 200;
  long sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < N; ++i) {
    int16_t ax, ay, az, gx, gy, gz;
    readAccelGyroRaw(ax, ay, az, gx, gy, gz);
    sx += gx; sy += gy; sz += gz;
    delayMicroseconds(LOOP_PERIOD_US);
  }
  gx_bias_dps = (sx / (float)N) / GYRO_LSB_PER_DPS;
  gy_bias_dps = (sy / (float)N) / GYRO_LSB_PER_DPS;
  gz_bias_dps = (sz / (float)N) / GYRO_LSB_PER_DPS;
  Serial.printf("[cal] gyro bias dps: %+.3f %+.3f %+.3f\n",
                gx_bias_dps, gy_bias_dps, gz_bias_dps);
  Serial.println("[cal] done — moving the device is now fine");
}

// ---------- Setup / loop --------------------------------------------------

static uint32_t last_loop_us = 0;
static uint32_t last_print_ms = 0;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("bike_computer: Madgwick AHRS smoke test");
  Wire.begin(I2C_SDA, I2C_SCL, I2C_HZ);

  if (!initIMU()) {
    Serial.println("[icm] init failed");
    while (1) { delay(2000); Serial.println("[icm] halted"); }
  }
  if (!initMag()) {
    Serial.println("[ak] init failed -- AHRS will be IMU-only (yaw drifts)");
  }

  calibrateGyroBias();

  Serial.printf("[run] fusion @ %lu Hz, prints @ %lu ms (β=%.3f)\n",
                (unsigned long)LOOP_HZ, (unsigned long)PRINT_PERIOD_MS,
                filter.beta());
  Serial.println("[run] heading is NOT trustworthy until mag hard-iron cal (step 2)");
  Serial.println("---- streaming ----");
  last_loop_us = micros();
}

void loop() {
  uint32_t now = micros();
  if ((uint32_t)(now - last_loop_us) < LOOP_PERIOD_US) return;
  float dt = (now - last_loop_us) * 1e-6f;
  last_loop_us = now;

  int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
  readAccelGyroRaw(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw);

  float ax_g = ax_raw / ACCEL_LSB_PER_G;
  float ay_g = ay_raw / ACCEL_LSB_PER_G;
  float az_g = az_raw / ACCEL_LSB_PER_G;
  // Subtract bias, convert dps -> rad/s.
  float gx_rs = (gx_raw / GYRO_LSB_PER_DPS - gx_bias_dps) * DPS_TO_RAD;
  float gy_rs = (gy_raw / GYRO_LSB_PER_DPS - gy_bias_dps) * DPS_TO_RAD;
  float gz_rs = (gz_raw / GYRO_LSB_PER_DPS - gz_bias_dps) * DPS_TO_RAD;

  // Mag at 100 Hz, fusion at 200 Hz: roughly every other cycle has a
  // fresh sample. When stale, we re-use the last reading rather than
  // dropping to IMU-only mode mid-stream.
  //
  // Hard-iron offsets subtracted; no axis transform applied.
  // Empirical: rotation diagnostic showed chip mag X/Y/Z line up with
  // body X/Y/Z (mz held constant during a controlled flat 360° yaw,
  // mx and my swept the expected sinusoids). Tried Z-negate and
  // swap+negate transforms — both made yaw drift WORSE. Best state:
  // identity transform with hard-iron cal, drift ~0.7 deg/s. See
  // STATUS.md for the planned next attempts.
  static float mx_uT = 0, my_uT = 0, mz_uT = 0;
  int16_t mx_raw, my_raw, mz_raw;
  if (readMagRaw(mx_raw, my_raw, mz_raw)) {
    mx_uT = mx_raw * MAG_UT_PER_LSB - MAG_OFFSET_X_uT;
    my_uT = my_raw * MAG_UT_PER_LSB - MAG_OFFSET_Y_uT;
    mz_uT = mz_raw * MAG_UT_PER_LSB - MAG_OFFSET_Z_uT;
  }

  filter.update(gx_rs, gy_rs, gz_rs, ax_g, ay_g, az_g,
                mx_uT, my_uT, mz_uT, dt);

  uint32_t ms = millis();
  if ((uint32_t)(ms - last_print_ms) >= PRINT_PERIOD_MS) {
    last_print_ms = ms;
    // Mag values printed for diagnosis: if these are constant when the
    // device is still, the filter has stable inputs but isn't using them
    // (axis problem). If they wander while still, something's perturbing
    // the magnetic environment.
    Serial.printf("roll=%+7.2f  pitch=%+7.2f  yaw=%+7.2f   "
                  "mag[uT]=(%+6.1f, %+6.1f, %+6.1f)\n",
                  filter.roll(), filter.pitch(), filter.yaw(),
                  mx_uT, my_uT, mz_uT);
  }
}
