// Madgwick AHRS sensor fusion filter.
//
// Adapted from Sebastian Madgwick's reference implementation:
//   https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
// Original paper: "An efficient orientation filter for inertial and
// inertial/magnetic sensor arrays" (Madgwick, 2010).
//
// State: a unit quaternion q = (q0, q1, q2, q3) representing the
// rotation from the world frame to the body frame.
//
// On each call to update(), three things happen:
//   1. Predict: integrate the gyro to advance q by `dt` seconds.
//   2. Correct: take one step of gradient descent on an error function
//      that says "if q were correct, accel would be (0,0,1)g rotated
//      into the body frame, and mag would be the local Earth-field
//      vector rotated in." beta scales how aggressive that correction
//      is per step.
//   3. Normalize and store.
//
// updateIMU() is the same thing without the mag (yaw will drift).
#pragma once

class Madgwick {
public:
  // beta: trust accel/mag vs gyro. Higher = faster convergence + more
  // jitter; lower = smoother but slower drift correction. Madgwick's
  // paper recommends ~0.033 for typical AHRS at 200 Hz; we start higher
  // to converge quickly during the smoke test and can tune down later.
  explicit Madgwick(float beta = 0.1f) : beta_(beta) {}

  // 9-DoF AHRS update. Units:
  //   gx,gy,gz: rad/s
  //   ax,ay,az: any consistent unit (g, m/s^2 — normalized internally)
  //   mx,my,mz: any consistent unit (uT, mGauss — normalized internally)
  //   dt: seconds since the previous update
  void update(float gx, float gy, float gz,
              float ax, float ay, float az,
              float mx, float my, float mz,
              float dt);

  // 6-DoF update (no mag). Yaw drifts because there's no absolute
  // heading reference.
  void updateIMU(float gx, float gy, float gz,
                 float ax, float ay, float az,
                 float dt);

  // Euler angles in degrees, derived from the quaternion (ZYX / yaw-pitch-roll).
  float roll() const;   // rotation about the X (forward) axis
  float pitch() const;  // rotation about the Y (right) axis
  float yaw() const;    // rotation about the Z (up) axis -- the heading

  float q0() const { return q0_; }
  float q1() const { return q1_; }
  float q2() const { return q2_; }
  float q3() const { return q3_; }

  void setBeta(float beta) { beta_ = beta; }
  float beta() const { return beta_; }

private:
  float beta_;
  // Initial orientation: identity quaternion (no rotation).
  float q0_ = 1.0f;
  float q1_ = 0.0f;
  float q2_ = 0.0f;
  float q3_ = 0.0f;
};
