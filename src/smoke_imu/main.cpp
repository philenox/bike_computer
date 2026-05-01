#include <Arduino.h>
#include <Wire.h>

// I2C bus to the Qwiic header.
static constexpr int I2C_SDA = 8;
static constexpr int I2C_SCL = 9;
static constexpr uint32_t I2C_HZ = 400000;

// --- ICM-20948 (accel + gyro) ---------------------------------------------
static constexpr uint8_t ICM_ADDR = 0x69;  // AD0 pulled high on this breakout

// REG_BANK_SEL is mirrored across all four register banks; selects bank
// for subsequent accesses. Value = bank << 4.
static constexpr uint8_t REG_BANK_SEL = 0x7F;

// Bank 0
static constexpr uint8_t B0_WHO_AM_I     = 0x00;
static constexpr uint8_t B0_USER_CTRL    = 0x03;
static constexpr uint8_t B0_PWR_MGMT_1   = 0x06;
static constexpr uint8_t B0_PWR_MGMT_2   = 0x07;
static constexpr uint8_t B0_INT_PIN_CFG  = 0x0F;
static constexpr uint8_t B0_ACCEL_XOUT_H = 0x2D;  // 14 bytes thru TEMP_OUT_L

static constexpr uint8_t WHO_AM_I_ICM = 0xEA;

// --- AK09916 magnetometer (auxiliary die inside ICM-20948 package) -------
// In bypass mode the host MCU talks to it directly at 0x0C.
static constexpr uint8_t AK_ADDR  = 0x0C;
static constexpr uint8_t AK_WIA2  = 0x01;  // WHO_AM_I, expected 0x09
static constexpr uint8_t AK_ST1   = 0x10;
static constexpr uint8_t AK_HXL   = 0x11;  // HXL,HXH,HYL,HYH,HZL,HZH,TMPS,ST2
static constexpr uint8_t AK_CNTL2 = 0x31;
static constexpr uint8_t AK_CNTL3 = 0x32;
static constexpr uint8_t WHO_AM_I_AK = 0x09;

// We leave the chip at power-on full-scale defaults for v1.
//   accel ±2g     -> 16384 LSB/g
//   gyro  ±250dps -> 131   LSB/(deg/s)
//   mag           -> 0.15  µT/LSB
static constexpr float ACCEL_LSB_PER_G   = 16384.0f;
static constexpr float GYRO_LSB_PER_DPS  = 131.0f;
static constexpr float MAG_UT_PER_LSB    = 0.15f;

// --------------------------------------------------------------------------

static bool writeReg(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool readRegs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;  // repeated start
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

static bool initIMU() {
  // Soft reset; chip needs ~100 ms before responding again.
  selectBank(0);
  writeReg(ICM_ADDR, B0_PWR_MGMT_1, 0x80);
  delay(100);

  uint8_t who = 0;
  if (!readReg(ICM_ADDR, B0_WHO_AM_I, who) || who != WHO_AM_I_ICM) {
    Serial.printf("[icm] WHO_AM_I after reset = 0x%02X (want 0x%02X)\n",
                  who, WHO_AM_I_ICM);
    return false;
  }

  // Wake: clear SLEEP, auto-pick best clock source.
  if (!writeReg(ICM_ADDR, B0_PWR_MGMT_1, 0x01)) return false;
  delay(20);
  // All accel + gyro axes enabled.
  if (!writeReg(ICM_ADDR, B0_PWR_MGMT_2, 0x00)) return false;
  delay(20);

  // Route the internal AK09916 onto the host I2C bus:
  //   USER_CTRL.I2C_MST_EN = 0 (disable internal master)
  //   INT_PIN_CFG.BYPASS_EN = 1 (bit 1)
  if (!writeReg(ICM_ADDR, B0_USER_CTRL, 0x00)) return false;
  if (!writeReg(ICM_ADDR, B0_INT_PIN_CFG, 0x02)) return false;
  delay(10);
  return true;
}

static bool initMag() {
  uint8_t who = 0;
  if (!readReg(AK_ADDR, AK_WIA2, who)) {
    Serial.println("[ak] WIA2 read failed (bypass not active?)");
    return false;
  }
  Serial.printf("[ak] WIA2 = 0x%02X (want 0x%02X) %s\n",
                who, WHO_AM_I_AK, who == WHO_AM_I_AK ? "OK" : "MISMATCH");
  if (who != WHO_AM_I_AK) return false;

  // Soft reset, then continuous-mode-2 (20 Hz). Faster than our 10 Hz read
  // loop, so DRDY is fresh on every cycle.
  writeReg(AK_ADDR, AK_CNTL3, 0x01);
  delay(10);
  if (!writeReg(AK_ADDR, AK_CNTL2, 0x04)) return false;
  delay(10);
  return true;
}

static int16_t be16(uint8_t hi, uint8_t lo) {
  return (int16_t)(((uint16_t)hi << 8) | lo);
}
static int16_t le16(uint8_t lo, uint8_t hi) {
  return (int16_t)(((uint16_t)hi << 8) | lo);
}

static void readAndPrint() {
  // Accel(6) + gyro(6) + temp(2) live in one contiguous block at 0x2D.
  uint8_t buf[14];
  if (!readRegs(ICM_ADDR, B0_ACCEL_XOUT_H, buf, sizeof(buf))) {
    Serial.println("[icm] burst read failed");
    return;
  }
  int16_t ax = be16(buf[0],  buf[1]);
  int16_t ay = be16(buf[2],  buf[3]);
  int16_t az = be16(buf[4],  buf[5]);
  int16_t gx = be16(buf[6],  buf[7]);
  int16_t gy = be16(buf[8],  buf[9]);
  int16_t gz = be16(buf[10], buf[11]);
  int16_t t  = be16(buf[12], buf[13]);

  // Magnetometer: ST1 -> data -> ST2 (ST2 read releases DRDY latch).
  float mx_uT = 0, my_uT = 0, mz_uT = 0;
  bool mag_fresh = false;
  uint8_t st1 = 0;
  if (readReg(AK_ADDR, AK_ST1, st1) && (st1 & 0x01)) {
    uint8_t m[8];  // HXL..HZH, TMPS, ST2
    if (readRegs(AK_ADDR, AK_HXL, m, sizeof(m))) {
      mx_uT = le16(m[0], m[1]) * MAG_UT_PER_LSB;
      my_uT = le16(m[2], m[3]) * MAG_UT_PER_LSB;
      mz_uT = le16(m[4], m[5]) * MAG_UT_PER_LSB;
      mag_fresh = true;
    }
  }

  float ax_g = ax / ACCEL_LSB_PER_G;
  float ay_g = ay / ACCEL_LSB_PER_G;
  float az_g = az / ACCEL_LSB_PER_G;
  float gx_dps = gx / GYRO_LSB_PER_DPS;
  float gy_dps = gy / GYRO_LSB_PER_DPS;
  float gz_dps = gz / GYRO_LSB_PER_DPS;
  // Temp per ICM-20948 datasheet §3.4: T = raw/333.87 + 21°C.
  float t_C = t / 333.87f + 21.0f;

  Serial.printf("a[g]=% 6.2f % 6.2f % 6.2f  g[dps]=% 7.1f % 7.1f % 7.1f  ",
                ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps);
  if (mag_fresh) {
    Serial.printf("m[uT]=% 6.1f % 6.1f % 6.1f  ", mx_uT, my_uT, mz_uT);
  } else {
    Serial.print("m[uT]=  stale                ");
  }
  Serial.printf("T=%.1fC\n", t_C);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("bike_computer: ICM-20948 raw read");
  Wire.begin(I2C_SDA, I2C_SCL, I2C_HZ);

  if (!initIMU()) {
    Serial.println("[icm] init failed");
    while (1) { delay(1000); Serial.println("[icm] halted"); }
  }
  Serial.println("[icm] init OK");

  if (!initMag()) {
    Serial.println("[ak] init failed -- continuing without mag");
  } else {
    Serial.println("[ak] init OK");
  }
  Serial.println("---- streaming ----");
}

void loop() {
  readAndPrint();
  delay(100);  // ~10 Hz
}
