#include <Arduino.h>
#include <Wire.h>

// Magnetometer hard-iron calibration sketch.
//
// What this does:
//   Reads the AK09916 (inside the ICM-20948) at 100 Hz and tracks the
//   running min/max per axis. While the user rotates the device through
//   all orientations, the mag readings trace out (approximately) a
//   sphere of radius |Earth field| ≈ 25-65 µT. Hard-iron sources
//   (chip's own offset, nearby ferrous PCB material) shift the sphere
//   center off the origin. The offset is just (min + max) / 2 per axis.
//
// What the user does:
//   - Sit the device flat and rotate slowly around the vertical axis.
//   - Tip it 90° forward, spin again.
//   - Tip 90° sideways, spin.
//   - Figure-8 patterns work well in the air.
//   - Goal: each axis sees both its +max and -min. The "span" field
//     on each axis should approach ~2 × |Earth field| (50-130 µT).
//
// Output: per-axis min, max, offset (mid-sphere), span (max-min).
// Every 5 s we also dump ready-to-paste C constants for smoke_madgwick.

static constexpr int I2C_SDA = 8;
static constexpr int I2C_SCL = 9;
static constexpr uint32_t I2C_HZ = 400000;

static constexpr uint8_t ICM_ADDR        = 0x69;
static constexpr uint8_t REG_BANK_SEL    = 0x7F;
static constexpr uint8_t B0_USER_CTRL    = 0x03;
static constexpr uint8_t B0_PWR_MGMT_1   = 0x06;
static constexpr uint8_t B0_PWR_MGMT_2   = 0x07;
static constexpr uint8_t B0_INT_PIN_CFG  = 0x0F;

static constexpr uint8_t AK_ADDR  = 0x0C;
static constexpr uint8_t AK_WIA2  = 0x01;
static constexpr uint8_t AK_ST1   = 0x10;
static constexpr uint8_t AK_HXL   = 0x11;
static constexpr uint8_t AK_CNTL2 = 0x31;
static constexpr uint8_t AK_CNTL3 = 0x32;
static constexpr uint8_t WHO_AM_I_AK = 0x09;
static constexpr uint8_t AK_MODE_CONTINUOUS_100HZ = 0x08;

static constexpr float MAG_UT_PER_LSB = 0.15f;

// Running min/max in raw LSB. Initialize to extremes so the first
// sample sets the baseline.
static int16_t mx_min =  32767, mx_max = -32768;
static int16_t my_min =  32767, my_max = -32768;
static int16_t mz_min =  32767, mz_max = -32768;
static uint32_t samples = 0;

// --- I2C helpers ---------------------------------------------------------

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

static int16_t le16(uint8_t lo, uint8_t hi) {
  return (int16_t)(((uint16_t)hi << 8) | lo);
}

// --- Init ---------------------------------------------------------------

static bool initIMU() {
  // Bank 0
  writeReg(ICM_ADDR, REG_BANK_SEL, 0x00);
  writeReg(ICM_ADDR, B0_PWR_MGMT_1, 0x80);  // reset
  delay(100);
  if (!writeReg(ICM_ADDR, B0_PWR_MGMT_1, 0x01)) return false;  // wake
  delay(20);
  if (!writeReg(ICM_ADDR, B0_PWR_MGMT_2, 0x00)) return false;
  delay(20);
  // Bypass -> AK09916 visible at 0x0C
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
  writeReg(AK_ADDR, AK_CNTL3, 0x01);
  delay(10);
  if (!writeReg(AK_ADDR, AK_CNTL2, AK_MODE_CONTINUOUS_100HZ)) return false;
  delay(10);
  return true;
}

// --- Main ---------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("bike_computer: magnetometer hard-iron calibration");
  Wire.begin(I2C_SDA, I2C_SCL, I2C_HZ);

  if (!initIMU()) { Serial.println("[icm] init failed"); while (1) delay(2000); }
  if (!initMag()) { Serial.println("[ak] init failed"); while (1) delay(2000); }

  Serial.println();
  Serial.println("INSTRUCTIONS:");
  Serial.println("  Rotate the device slowly through every orientation.");
  Serial.println("  - flat and spin around vertical");
  Serial.println("  - tip 90deg forward, spin again");
  Serial.println("  - tip 90deg sideways, spin");
  Serial.println("  - figure-8s in the air work well");
  Serial.println();
  Serial.println("  Each axis 'span' should approach ~2x Earth field");
  Serial.println("  (50-130 uT) when you've covered enough orientations.");
  Serial.println("  Stop when min/max stop expanding.");
  Serial.println();
  Serial.println("---- streaming ----");
}

static uint32_t last_print_ms = 0;
static uint32_t last_summary_ms = 0;

void loop() {
  // Read mag when DRDY is set. ST2 read (last byte) releases the latch.
  uint8_t st1 = 0;
  if (readReg(AK_ADDR, AK_ST1, st1) && (st1 & 0x01)) {
    uint8_t b[8];  // HXL,HXH,HYL,HYH,HZL,HZH,TMPS,ST2
    if (readRegs(AK_ADDR, AK_HXL, b, 8)) {
      int16_t mx = le16(b[0], b[1]);
      int16_t my = le16(b[2], b[3]);
      int16_t mz = le16(b[4], b[5]);
      if (mx < mx_min) mx_min = mx;
      if (mx > mx_max) mx_max = mx;
      if (my < my_min) my_min = my;
      if (my > my_max) my_max = my;
      if (mz < mz_min) mz_min = mz;
      if (mz > mz_max) mz_max = mz;
      ++samples;
    }
  }

  uint32_t now = millis();

  if ((uint32_t)(now - last_print_ms) >= 500 && samples > 0) {
    last_print_ms = now;
    float mxn = mx_min * MAG_UT_PER_LSB, mxx = mx_max * MAG_UT_PER_LSB;
    float myn = my_min * MAG_UT_PER_LSB, myx = my_max * MAG_UT_PER_LSB;
    float mzn = mz_min * MAG_UT_PER_LSB, mzx = mz_max * MAG_UT_PER_LSB;
    float ox = (mxn + mxx) * 0.5f;
    float oy = (myn + myx) * 0.5f;
    float oz = (mzn + mzx) * 0.5f;
    float sx = mxx - mxn, sy = myx - myn, sz = mzx - mzn;
    Serial.printf("min(%+6.1f,%+6.1f,%+6.1f)  max(%+6.1f,%+6.1f,%+6.1f)  "
                  "off(%+6.1f,%+6.1f,%+6.1f)  span(%5.1f,%5.1f,%5.1f)  n=%lu\n",
                  mxn, myn, mzn, mxx, myx, mzx,
                  ox, oy, oz, sx, sy, sz, (unsigned long)samples);
  }

  if ((uint32_t)(now - last_summary_ms) >= 5000 && samples > 0) {
    last_summary_ms = now;
    float ox = (mx_min + mx_max) * 0.5f * MAG_UT_PER_LSB;
    float oy = (my_min + my_max) * 0.5f * MAG_UT_PER_LSB;
    float oz = (mz_min + mz_max) * 0.5f * MAG_UT_PER_LSB;
    Serial.println();
    Serial.println("// paste into smoke_madgwick once span has plateaued:");
    Serial.printf("static constexpr float MAG_OFFSET_X_uT = %+6.2ff;\n", ox);
    Serial.printf("static constexpr float MAG_OFFSET_Y_uT = %+6.2ff;\n", oy);
    Serial.printf("static constexpr float MAG_OFFSET_Z_uT = %+6.2ff;\n", oz);
    Serial.println();
  }

  delay(5);  // ~200 Hz read attempts; mag updates at 100 Hz so this is fine
}
