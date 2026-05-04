// Madgwick AHRS implementation. See Madgwick.h for the high-level
// description. The math here is a near-verbatim transcription of the
// reference C code at:
//   https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// Variable names match the paper / reference where possible to keep
// the math traceable. The auxiliary `_2q0`, `_4bz`, `q1q3` etc. are
// pre-computed common subexpressions to avoid re-doing the same
// multiplications.

#include "Madgwick.h"
#include <math.h>

static inline float invSqrt(float x) {
  // The reference uses Quake's fast inverse sqrt, but on the ESP32-S3
  // with a hardware FPU there's no win — the FPU's sqrtf is already
  // single-cycle-ish. Plain 1/sqrtf is more accurate and simpler.
  return 1.0f / sqrtf(x);
}

void Madgwick::update(float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float mx, float my, float mz,
                     float dt) {
  // If mag reads are all zero (sensor down or stale) fall back to IMU
  // update — running full AHRS with zero mag would NaN out the math.
  if (mx == 0.0f && my == 0.0f && mz == 0.0f) {
    updateIMU(gx, gy, gz, ax, ay, az, dt);
    return;
  }

  // qDot = rate of change of quaternion from gyro alone.
  // q' = (1/2) * q ⊗ ω, where ω = (0, gx, gy, gz).
  float qDot1 = 0.5f * (-q1_ * gx - q2_ * gy - q3_ * gz);
  float qDot2 = 0.5f * ( q0_ * gx + q2_ * gz - q3_ * gy);
  float qDot3 = 0.5f * ( q0_ * gy - q1_ * gz + q3_ * gx);
  float qDot4 = 0.5f * ( q0_ * gz + q1_ * gy - q2_ * gx);

  // Skip the gradient correction if accel is zero (sensor down).
  if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
    float recipNorm;

    // Normalise accel and mag — only the directions matter.
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

    // Pre-compute repeated terms.
    float _2q0mx = 2.0f * q0_ * mx;
    float _2q0my = 2.0f * q0_ * my;
    float _2q0mz = 2.0f * q0_ * mz;
    float _2q1mx = 2.0f * q1_ * mx;
    float _2q0 = 2.0f * q0_;
    float _2q1 = 2.0f * q1_;
    float _2q2 = 2.0f * q2_;
    float _2q3 = 2.0f * q3_;
    float _2q0q2 = 2.0f * q0_ * q2_;
    float _2q2q3 = 2.0f * q2_ * q3_;
    float q0q0 = q0_ * q0_;
    float q0q1 = q0_ * q1_;
    float q0q2 = q0_ * q2_;
    float q0q3 = q0_ * q3_;
    float q1q1 = q1_ * q1_;
    float q1q2 = q1_ * q2_;
    float q1q3 = q1_ * q3_;
    float q2q2 = q2_ * q2_;
    float q2q3 = q2_ * q3_;
    float q3q3 = q3_ * q3_;

    // Reference direction of Earth's mag field, projected from body
    // frame back to world frame and flattened to (bx, 0, bz). This is
    // how the filter handles "the mag points partly down due to
    // inclination" without us having to know the local declination.
    float hx = mx * q0q0 - _2q0my * q3_ + _2q0mz * q2_ + mx * q1q1
             + _2q1 * my * q2_ + _2q1 * mz * q3_ - mx * q2q2 - mx * q3q3;
    float hy = _2q0mx * q3_ + my * q0q0 - _2q0mz * q1_ + _2q1mx * q2_
             - my * q1q1 + my * q2q2 + _2q2 * mz * q3_ - my * q3q3;
    float _2bx = sqrtf(hx * hx + hy * hy);
    float _2bz = -_2q0mx * q2_ + _2q0my * q1_ + mz * q0q0 + _2q1mx * q3_
               - mz * q1q1 + _2q2 * my * q3_ - mz * q2q2 + mz * q3q3;
    float _4bx = 2.0f * _2bx;
    float _4bz = 2.0f * _2bz;

    // Gradient of the cost function: how much each q component
    // should change to reduce the (predicted - measured)^2 error.
    float s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax)
             +  _2q1 * (2.0f * q0q1 + _2q2q3 - ay)
             - _2bz * q2_ * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
             + (-_2bx * q3_ + _2bz * q1_) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
             +  _2bx * q2_ * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    float s1 =  _2q3 * (2.0f * q1q3 - _2q0q2 - ax)
             +  _2q0 * (2.0f * q0q1 + _2q2q3 - ay)
             - 4.0f * q1_ * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az)
             +  _2bz * q3_ * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
             + ( _2bx * q2_ + _2bz * q0_) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
             + ( _2bx * q3_ - _4bz * q1_) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    float s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax)
             +  _2q3 * (2.0f * q0q1 + _2q2q3 - ay)
             - 4.0f * q2_ * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az)
             + (-_4bx * q2_ - _2bz * q0_) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
             + ( _2bx * q1_ + _2bz * q3_) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
             + ( _2bx * q0_ - _4bz * q2_) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    float s3 =  _2q1 * (2.0f * q1q3 - _2q0q2 - ax)
             +  _2q2 * (2.0f * q0q1 + _2q2q3 - ay)
             + (-_4bx * q3_ + _2bz * q1_) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
             + (-_2bx * q0_ + _2bz * q2_) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
             +  _2bx * q1_ * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

    // Mix the gradient correction into qDot, scaled by beta.
    qDot1 -= beta_ * s0;
    qDot2 -= beta_ * s1;
    qDot3 -= beta_ * s2;
    qDot4 -= beta_ * s3;
  }

  // Integrate qDot to advance the quaternion by dt.
  q0_ += qDot1 * dt;
  q1_ += qDot2 * dt;
  q2_ += qDot3 * dt;
  q3_ += qDot4 * dt;

  // Renormalise — quaternion must stay unit length.
  float recipNorm = invSqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
  q0_ *= recipNorm; q1_ *= recipNorm; q2_ *= recipNorm; q3_ *= recipNorm;
}

void Madgwick::updateIMU(float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float dt) {
  float qDot1 = 0.5f * (-q1_ * gx - q2_ * gy - q3_ * gz);
  float qDot2 = 0.5f * ( q0_ * gx + q2_ * gz - q3_ * gy);
  float qDot3 = 0.5f * ( q0_ * gy - q1_ * gz + q3_ * gx);
  float qDot4 = 0.5f * ( q0_ * gz + q1_ * gy - q2_ * gx);

  if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
    float recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    float _2q0 = 2.0f * q0_;
    float _2q1 = 2.0f * q1_;
    float _2q2 = 2.0f * q2_;
    float _2q3 = 2.0f * q3_;
    float _4q0 = 4.0f * q0_;
    float _4q1 = 4.0f * q1_;
    float _4q2 = 4.0f * q2_;
    float _8q1 = 8.0f * q1_;
    float _8q2 = 8.0f * q2_;
    float q0q0 = q0_ * q0_;
    float q1q1 = q1_ * q1_;
    float q2q2 = q2_ * q2_;
    float q3q3 = q3_ * q3_;

    float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1_ - _2q0 * ay
             - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    float s2 = 4.0f * q0q0 * q2_ + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay
             - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    float s3 = 4.0f * q1q1 * q3_ - _2q1 * ax + 4.0f * q2q2 * q3_ - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

    qDot1 -= beta_ * s0;
    qDot2 -= beta_ * s1;
    qDot3 -= beta_ * s2;
    qDot4 -= beta_ * s3;
  }

  q0_ += qDot1 * dt;
  q1_ += qDot2 * dt;
  q2_ += qDot3 * dt;
  q3_ += qDot4 * dt;

  float recipNorm = invSqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
  q0_ *= recipNorm; q1_ *= recipNorm; q2_ *= recipNorm; q3_ *= recipNorm;
}

// Quaternion -> Euler (ZYX). Returns degrees.
//   roll  = atan2(2(q0 q1 + q2 q3), 1 - 2(q1^2 + q2^2))
//   pitch = asin(clamp(2(q0 q2 - q1 q3), -1, 1))
//   yaw   = atan2(2(q0 q3 + q1 q2), 1 - 2(q2^2 + q3^2))
static constexpr float RAD_TO_DEG = 57.29577951308232f;

float Madgwick::roll() const {
  return atan2f(2.0f * (q0_ * q1_ + q2_ * q3_),
                1.0f - 2.0f * (q1_ * q1_ + q2_ * q2_)) * RAD_TO_DEG;
}

float Madgwick::pitch() const {
  float t = 2.0f * (q0_ * q2_ - q1_ * q3_);
  if (t >  1.0f) t =  1.0f;
  if (t < -1.0f) t = -1.0f;
  return asinf(t) * RAD_TO_DEG;
}

float Madgwick::yaw() const {
  return atan2f(2.0f * (q0_ * q3_ + q1_ * q2_),
                1.0f - 2.0f * (q2_ * q2_ + q3_ * q3_)) * RAD_TO_DEG;
}
