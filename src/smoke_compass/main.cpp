#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>
#include <math.h>
#include "ICM20948.h"
#include "Madgwick.h"

// Sharp color convention: 0 = black ink, 1 = white reflective.
#define COLOR_BLACK 0
#define COLOR_WHITE 1

// ---- Pin assignments -------------------------------------------------------
static constexpr int I2C_SDA  = 8;
static constexpr int I2C_SCL  = 9;
static constexpr int SPI_SCK  = 12;
static constexpr int SPI_MISO = 13;
static constexpr int SPI_MOSI = 11;
static constexpr int SHARP_CS = 7;
static constexpr uint32_t I2C_HZ = 400000;

// ---- Display ---------------------------------------------------------------
static constexpr int LCD_W = 400, LCD_H = 240;
static Adafruit_SharpMem display(&SPI, SHARP_CS, LCD_W, LCD_H, 2000000);

// ---- IMU / AHRS ------------------------------------------------------------
static ICM20948 imu(0x69);
static Madgwick filter(0.1f);

// Hard-iron offsets in the ICM-20948 accel/gyro frame (mag Y/Z already
// negated by the driver and by smoke_mag_cal). Recaptured 2026-05-07
// well clear of the laptop — earlier cal had laptop-magnet contamination
// that pulled Y to +106 µT.
static constexpr float MAG_OFF_X =  -1.4f;
static constexpr float MAG_OFF_Y = +52.6f;
static constexpr float MAG_OFF_Z = -67.9f;

// Rotation offset between the IMU X-axis (the filter's yaw=0 reference) and
// whichever physical direction on the breadboard you consider "display up".
// Set to 0, flash, then hold the device so the display top points at a known
// reference direction and note the displayed heading. That number is your
// offset. Update here and re-flash.
// Negate filter.yaw() at the call site because Madgwick uses right-hand
// Z-up (CCW = positive yaw), which is opposite to navigation convention
// (CW = positive heading) when holding the device with screen facing you.
// The empirical offset is 87°; +180° on top of that flips the needle so it
// reads correctly with the display held in its natural (un-rotated)
// orientation, where the LCD is mounted 180° from the IMU's "forward".
static constexpr float YAW_OFFSET_DEG = 267.0f;

static constexpr uint32_t IMU_HZ        = 200;
static constexpr uint32_t IMU_PERIOD_US = 1000000UL / IMU_HZ;
static constexpr uint32_t DISPLAY_MS    = 100;   // 10 Hz

// ---- Compass geometry (left half of 400px display) -------------------------
static constexpr int CX = 120;
static constexpr int CY = 120;
static constexpr int R  = 100;   // outer dial radius

// ---- Helpers ---------------------------------------------------------------

static const char *compass_point(float deg) {
    // Normalize to [0, 360) and map to 8 cardinal/intercardinal points.
    deg = fmodf(deg, 360.0f);
    if (deg < 0.0f) deg += 360.0f;
    static const char *const pts[] = { "N","NE","E","SE","S","SW","W","NW" };
    return pts[(int)((deg + 22.5f) / 45.0f) & 7];
}

static void draw_compass(float yaw_deg, float roll_deg, float pitch_deg) {
    display.clearDisplay();
    display.setTextColor(COLOR_BLACK);

    // --- Dial ---
    display.drawCircle(CX, CY, R,     COLOR_BLACK);
    display.drawCircle(CX, CY, R - 3, COLOR_BLACK);

    // Cardinal ticks + labels (N/E/S/W)
    static const char *const card[] = { "N","E","S","W" };
    for (int i = 0; i < 4; ++i) {
        float a  = i * (float)M_PI / 2.0f;
        float sa = sinf(a), ca = cosf(a);
        // Long tick from R-14 to R-4 (inside double ring)
        display.drawLine(CX + (int)(sa*(R-14)), CY - (int)(ca*(R-14)),
                         CX + (int)(sa*(R-4)),  CY - (int)(ca*(R-4)), COLOR_BLACK);
        // Label at R-26
        display.setTextSize(1);
        display.setCursor(CX + (int)(sa*(R-26)) - 3,
                          CY - (int)(ca*(R-26)) - 4);
        display.print(card[i]);
    }
    // Short intercardinal ticks (NE/SE/SW/NW)
    for (int i = 0; i < 4; ++i) {
        float a  = (i + 0.5f) * (float)M_PI / 2.0f;
        float sa = sinf(a), ca = cosf(a);
        display.drawLine(CX + (int)(sa*(R-8)), CY - (int)(ca*(R-8)),
                         CX + (int)(sa*(R-4)), CY - (int)(ca*(R-4)), COLOR_BLACK);
    }

    // --- Needle ---
    // The needle points toward magnetic north in the device's current orientation.
    // yaw_deg = heading of the IMU X-axis from magnetic north (0=N, 90=E, CW+).
    // North tip (screen coords with Y-down):
    //   ntx = CX - sin(yaw)*nl,  nty = CY - cos(yaw)*nl
    // Verify: yaw=0  → (CX, CY-nl) = screen up ✓
    //         yaw=90 → (CX-nl, CY) = screen left (north is left when facing east) ✓
    float yr = yaw_deg * (float)M_PI / 180.0f;
    float sn = sinf(yr), cn = cosf(yr);

    static constexpr int NL = R - 18;  // north tip reach
    static constexpr int SL = 30;      // south tail reach
    static constexpr int HW = 8;       // needle half-width at pivot

    int ntx = CX - (int)(sn * NL),  nty = CY - (int)(cn * NL);
    int stx = CX + (int)(sn * SL),  sty = CY + (int)(cn * SL);
    // Perpendicular at half-width: unit vector (cn, -sn) in screen coords
    int bx1 = CX + (int)(cn * HW),  by1 = CY - (int)(sn * HW);
    int bx2 = CX - (int)(cn * HW),  by2 = CY + (int)(sn * HW);

    display.fillTriangle(ntx, nty, bx1, by1, bx2, by2, COLOR_BLACK); // north: filled
    display.drawTriangle(stx, sty, bx1, by1, bx2, by2, COLOR_BLACK); // south: outline
    display.fillCircle(CX, CY, 5, COLOR_BLACK);

    // --- Right panel: heading readout ---
    float hdg = fmodf(yaw_deg, 360.0f);
    if (hdg < 0.0f) hdg += 360.0f;

    // Heading in degrees (large)
    display.setTextSize(4);
    char num[5];
    snprintf(num, sizeof(num), "%03d", (int)hdg);
    display.setCursor(252, 70);
    display.print(num);

    // Compass point label
    display.setTextSize(2);
    display.setCursor(282, 112);
    display.print(compass_point(hdg));

    // Roll / pitch for diagnostic
    display.setTextSize(1);
    char rp[26];
    snprintf(rp, sizeof(rp), "R%+.1f  P%+.1f", roll_deg, pitch_deg);
    display.setCursor(240, 164);
    display.print(rp);
}

// ---- State -----------------------------------------------------------------

static uint32_t last_imu_us  = 0;
static uint32_t last_disp_ms = 0;

// ---- Setup / loop ----------------------------------------------------------

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println("bike_computer: compass smoke test");

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, -1);

    if (!display.begin()) {
        Serial.println("[sharp] begin() failed");
        while (1) delay(1000);
    }
    display.clearDisplay();
    display.setTextColor(COLOR_BLACK);
    display.setTextSize(1);
    display.setCursor(4, 4);
    display.print("Calibrating gyro...");
    display.refresh();

    Wire.begin(I2C_SDA, I2C_SCL, I2C_HZ);
    if (!imu.begin()) {
        Serial.println("[imu] init failed");
        while (1) delay(1000);
    }
    imu.calibrateGyro();

    Serial.printf("[run] compass loop @ %lu Hz display, %lu Hz IMU  YAW_OFFSET=%.1f\n",
                  1000UL / DISPLAY_MS, (unsigned long)IMU_HZ, YAW_OFFSET_DEG);
    Serial.println("[run] hold still ~5 s for filter to converge on mag heading");
    last_imu_us  = micros();
    last_disp_ms = millis();
}

void loop() {
    // --- IMU update at 200 Hz ---
    uint32_t now_us = micros();
    if ((uint32_t)(now_us - last_imu_us) >= IMU_PERIOD_US) {
        float dt = (now_us - last_imu_us) * 1e-6f;
        last_imu_us = now_us;

        float ax, ay, az, gx, gy, gz;
        imu.readAccelGyro(ax, ay, az, gx, gy, gz);

        static float mx_uT = 0.0f, my_uT = 0.0f, mz_uT = 0.0f;
        float mx_raw, my_raw, mz_raw;
        if (imu.readMag(mx_raw, my_raw, mz_raw)) {
            mx_uT = mx_raw - MAG_OFF_X;
            my_uT = my_raw - MAG_OFF_Y;
            mz_uT = mz_raw - MAG_OFF_Z;
        }
        filter.update(gx, gy, gz, ax, ay, az, mx_uT, my_uT, mz_uT, dt);
    }

    // --- Display update at 10 Hz ---
    uint32_t now_ms = millis();
    if ((uint32_t)(now_ms - last_disp_ms) >= DISPLAY_MS) {
        last_disp_ms = now_ms;
        float yaw = -filter.yaw() + YAW_OFFSET_DEG;
        draw_compass(yaw, filter.roll(), filter.pitch());
        display.refresh();

        // Serial mirror so you can watch heading without looking at the display.
        float hdg = fmodf(yaw, 360.0f);
        if (hdg < 0.0f) hdg += 360.0f;
        Serial.printf("hdg=%05.1f (%s)  R=%+.1f  P=%+.1f\n",
                      hdg, compass_point(hdg), filter.roll(), filter.pitch());
    }
}
