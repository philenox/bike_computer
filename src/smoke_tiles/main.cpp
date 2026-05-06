#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>
#include <math.h>
#include "ICM20948.h"
#include "Madgwick.h"

// Sharp Memory LCD color convention: 0=black (ink), 1=white (reflective).
// The library does not define COLOR_WHITE/COLOR_BLACK globally.
#define COLOR_BLACK 0
#define COLOR_WHITE 1

// ---- Pin assignments (locked in from earlier smoke tests) ---------------
static constexpr int I2C_SDA    = 8;
static constexpr int I2C_SCL    = 9;
static constexpr int SPI_SCK    = 12;
static constexpr int SPI_MISO   = 13;
static constexpr int SPI_MOSI   = 11;
static constexpr int SD_CS      = 10;
static constexpr int SHARP_CS   = 7;
static constexpr uint32_t I2C_HZ = 400000;

// ---- Display ------------------------------------------------------------
static constexpr int LCD_W = 400, LCD_H = 240;
static Adafruit_SharpMem display(&SPI, SHARP_CS, LCD_W, LCD_H, 2000000);

// ---- IMU / AHRS ---------------------------------------------------------
static ICM20948 imu(0x69);
static Madgwick filter(0.1f);

// Hard-iron offsets (smoke_mag_cal, 2026-05-05 breadboard build).
static constexpr float MAG_OFF_X = +14.78f;
static constexpr float MAG_OFF_Y = -53.55f;
static constexpr float MAG_OFF_Z = +88.88f;

static constexpr uint32_t IMU_HZ        = 200;
static constexpr uint32_t IMU_PERIOD_US = 1000000UL / IMU_HZ;
static constexpr uint32_t MAP_PERIOD_MS = 200;   // 5 Hz display refresh

// ---- Fixed GPS position (replace when NEO-M9N arrives) ------------------
static constexpr double USER_LAT = 51.458658;
static constexpr double USER_LON = -0.993939;
static constexpr int    MAP_ZOOM = 15;

// ---- Tile buffers -------------------------------------------------------
// 2×2 tile grid. Each tile: 256×256 pixels × 1-bit = 8192 bytes.
// Index: [row 0=north/1=south][col 0=west/1=east]
// Total: 32 KB — fits comfortably in SRAM (320 KB free after framework).
static uint8_t tile_buf[2][2][8192];

// ---- Tile coordinate math -----------------------------------------------

// Slippy map: convert lat/lon → global pixel coordinate at given zoom.
// Uses double so sub-tile pixel offset is accurate to <0.1 px.
static void latlon_to_global_px(double lat, double lon, int zoom,
                                 double &px, double &py) {
    double n  = (double)(1 << zoom);
    double lr = lat * M_PI / 180.0;
    px = (lon + 180.0) / 360.0 * n * 256.0;
    py = (1.0 - log(tan(lr) + 1.0 / cos(lr)) / M_PI) / 2.0 * n * 256.0;
}

// Decompose global pixel into tile index + sub-tile pixel offset.
static void global_px_to_tile(double px, double py,
                               int &tx, int &ty,
                               int &sub_x, int &sub_y) {
    tx    = (int)(px / 256.0);
    ty    = (int)(py / 256.0);
    sub_x = (int)(px) & 255;
    sub_y = (int)(py) & 255;
}

// ---- SD tile loading ----------------------------------------------------

static bool load_one_tile(int tx, int ty, int row, int col) {
    char path[48];
    snprintf(path, sizeof(path), "/tiles/%d/%d/%d.bin", MAP_ZOOM, tx, ty);
    File f = SD.open(path);
    if (!f) {
        // Missing tile: fill black (off-map area). Not an error in normal use.
        memset(tile_buf[row][col], 0x00, 8192);
        Serial.printf("[tiles] missing %s\n", path);
        return false;
    }
    size_t got = f.read(tile_buf[row][col], 8192);
    f.close();
    if (got != 8192) {
        memset(tile_buf[row][col], 0x00, 8192);
        Serial.printf("[tiles] short read %s (%u bytes)\n", path, (unsigned)got);
        return false;
    }
    return true;
}

// Load the 2×2 grid whose top-left tile is (tx, ty).
static void load_tile_grid(int tx, int ty) {
    Serial.printf("[tiles] loading 2×2 grid origin (%d,%d)\n", tx, ty);
    uint32_t t0 = millis();
    load_one_tile(tx,     ty,     0, 0);
    load_one_tile(tx + 1, ty,     0, 1);
    load_one_tile(tx,     ty + 1, 1, 0);
    load_one_tile(tx + 1, ty + 1, 1, 1);
    Serial.printf("[tiles] loaded in %lu ms\n", (unsigned long)(millis() - t0));
}

// ---- Source pixel access ------------------------------------------------

// Read one pixel from the 2×2 tile grid.
// (sx, sy) are in [0, 511]; bit packing is MSB-first, row-major.
// Returns false (black) for out-of-bounds coordinates.
static inline bool src_pixel(int sx, int sy) {
    if ((unsigned)sx >= 512u || (unsigned)sy >= 512u) return false;
    const uint8_t *tile = tile_buf[sy >> 8][sx >> 8];
    int lx = sx & 255, ly = sy & 255;
    return (tile[ly * 32 + (lx >> 3)] >> (7 - (lx & 7))) & 1;
}

// ---- Rotated blit -------------------------------------------------------

// Render the 2×2 tile grid centred at (src_cx, src_cy) into the Sharp
// framebuffer, rotated by yaw_rad (map heading-up: N up when yaw=0,
// E up when yaw=π/2, etc.). One call per display frame.
static void blit_rotated(int src_cx, int src_cy, float yaw_rad) {
    // Inverse rotation: for each output pixel, find source pixel.
    // We rotate the source map by -yaw so the heading direction faces up.
    float cos_a =  cosf(yaw_rad);
    float sin_a =  sinf(yaw_rad);
    int   dst_cx = LCD_W / 2;
    int   dst_cy = LCD_H / 2;

    for (int y = 0; y < LCD_H; ++y) {
        float dy = (float)(y - dst_cy);
        // For a fixed row, precompute the terms that don't vary with x.
        // sx = cos_a*(x-dst_cx) + sin_a*dy + src_cx
        // sy = -sin_a*(x-dst_cx) + cos_a*dy + src_cy
        // At x=0: dx = -dst_cx, so:
        float sx = cos_a * (float)(-dst_cx) + sin_a * dy + (float)src_cx;
        float sy = -sin_a * (float)(-dst_cx) + cos_a * dy + (float)src_cy;
        // Step per pixel column: d(sx)/dx = cos_a, d(sy)/dx = -sin_a
        for (int x = 0; x < LCD_W; ++x, sx += cos_a, sy -= sin_a) {
            display.drawPixel(x, y, src_pixel((int)sx, (int)sy) ? COLOR_WHITE : COLOR_BLACK);
        }
    }
}

// Draw a 5×5 crosshair at screen centre — marks "you are here".
// Visible on both road (white) and background (black) because it
// alternates: a 3×3 black core surrounded by a 1-px white outline.
static void draw_position_marker() {
    int cx = LCD_W / 2, cy = LCD_H / 2;
    // White outline 5×5
    for (int dy = -2; dy <= 2; ++dy)
        for (int dx = -2; dx <= 2; ++dx)
            display.drawPixel(cx + dx, cy + dy, COLOR_WHITE);
    // Black core 3×3
    for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx)
            display.drawPixel(cx + dx, cy + dy, COLOR_BLACK);
}

// ---- Setup / loop -------------------------------------------------------

static uint32_t last_imu_us  = 0;
static uint32_t last_map_ms  = 0;
static int      src_cx = 128, src_cy = 128;  // user's position in tile buf

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println("bike_computer: tile renderer smoke test");

    // SPI — must come before SD and display begin().
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, -1);

    // Display
    if (!display.begin()) {
        Serial.println("[sharp] begin() failed");
        while (1) delay(1000);
    }
    display.clearDisplay();
    display.setTextColor(COLOR_BLACK);
    display.setTextSize(1);
    display.setCursor(4, 4);
    display.print("Loading...");
    display.refresh();

    // SD card
    if (!SD.begin(SD_CS)) {
        Serial.println("[sd] init failed");
        display.setCursor(4, 20);
        display.print("SD FAILED");
        display.refresh();
        while (1) delay(1000);
    }

    // Pre-compute tile coordinates for the fixed position.
    double gpx, gpy;
    latlon_to_global_px(USER_LAT, USER_LON, MAP_ZOOM, gpx, gpy);
    int tx, ty;
    global_px_to_tile(gpx, gpy, tx, ty, src_cx, src_cy);
    Serial.printf("[tiles] user @ tile (%d,%d)  sub-px (%d,%d)\n",
                  tx, ty, src_cx, src_cy);
    load_tile_grid(tx, ty);

    // IMU
    Wire.begin(I2C_SDA, I2C_SCL, I2C_HZ);
    if (!imu.begin()) {
        Serial.println("[imu] init failed");
        display.setCursor(4, 20);
        display.print("IMU FAILED");
        display.refresh();
        while (1) delay(1000);
    }
    imu.calibrateGyro();

    Serial.println("[run] entering render loop");
    last_imu_us = micros();
    last_map_ms = millis();
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

    // --- Map redraw at 5 Hz ---
    uint32_t now_ms = millis();
    if ((uint32_t)(now_ms - last_map_ms) >= MAP_PERIOD_MS) {
        last_map_ms = now_ms;

        float yaw_rad = filter.yaw() * (float)M_PI / 180.0f;
        blit_rotated(src_cx, src_cy, yaw_rad);
        draw_position_marker();
        display.refresh();
    }
}
