#include <Wire.h>

// ── IMU (MPU-6050/9250) ───────────────────────────────────
#define IMU_ADDR     0x68
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B

float accelAngle = 0;
float gyroAngle  = 0;
float compAngle  = 0;

unsigned long lastTime = 0;

void imuReset() {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x80);
  Wire.endTransmission();
  delay(150);

  Wire.beginTransmission(IMU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(150);
}

int readIMU(int16_t &ax, int16_t &ay, int16_t &az,
            int16_t &gx, int16_t &gy) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return 0;
  if (Wire.requestFrom(IMU_ADDR, 14) != 14) return 0;

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  return 1;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  imuReset();
  lastTime = millis();
}

void loop() {
  int16_t ax, ay, az, gx, gy;

  if (!readIMU(ax, ay, az, gx, gy)) return;

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // Convert to physical units
  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;
  float gx_dps = gx / 131.0;  // degrees/sec

  // Accelerometer angle
  accelAngle = atan2(ay_g, az_g) * 180.0 / PI;

  // Gyroscope angle (integration – drifts over time)
  gyroAngle += gx_dps * dt;

  // Complementary filter (α = 0.98)
  float alpha = 0.98;
  compAngle = alpha * (compAngle + gx_dps * dt) + (1 - alpha) * accelAngle;

  // Output: Accel | Gyro | Complementary
  Serial.print("Accel: ");   Serial.print(accelAngle);
  Serial.print(" | Gyro: "); Serial.print(gyroAngle);
  Serial.print(" | Comp: "); Serial.println(compAngle);

  delay(10);
}
