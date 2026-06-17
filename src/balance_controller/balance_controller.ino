#include <Wire.h>
  
// ── IMU (MPU-6050) ────────────────────────────────────────
#define IMU_ADDR     0x68
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B

// ── IMU axis configuration ───────────────────────────────
#define ACC_Y_SIGN   -1.0
#define ACC_Z_SIGN  -1.0
#define GYRO_Y_SIGN -1.0

// ── Motor driver pins ─────────────────────────────────────
#define ENA 3
#define IN1 6
#define IN2 7
#define ENB 5
#define IN3 8
#define IN4 9

float PITCH_OFFSET = 0.0;
float MOTOR_TRIM   = 0.0;

float Kp = 15.0;
float Ki =  0.0;
float Kd =  10.0;

const float FALL_ANGLE = 30.0;
const float I_LIMIT    = 100.0;
const float DEAD_BAND  = 20.0;
const float CF_ALPHA   = 0.98;

// ── PID state ─────────────────────────────────────────────
float pitch     = 0.0;
float integral  = 0.0;
float prevError = 0.0;
unsigned long lastTime;

// ─────────────────────────────────────────────────────────
// IMU helpers
// ─────────────────────────────────────────────────────────
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

// ─────────────────────────────────────────────────────────
// Motor helpers
// ─────────────────────────────────────────────────────────
void setMotors(float left, float right) {
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);

  digitalWrite(IN1, left  >= 0 ? HIGH : LOW);
  digitalWrite(IN2, left  >= 0 ? LOW  : HIGH);
  analogWrite (ENA, abs((int)left));

  digitalWrite(IN3, right >= 0 ? HIGH : LOW);
  digitalWrite(IN4, right >= 0 ? LOW  : HIGH);
  analogWrite (ENB, abs((int)right));
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// ─────────────────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();

  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);

  imuReset();

  // Warmup
  unsigned long warmup = millis();
  while (millis() - warmup < 500) {
    int16_t ax, ay, az, gx, gy;
    if (readIMU(ax, ay, az, gx, gy)) {

      float ayg = (ay / 16384.0) * ACC_Y_SIGN;
      float azg = (az / 16384.0) * ACC_Z_SIGN;
      float gys = (gy / 131.0)   * GYRO_Y_SIGN;

      float accPitch = atan2(ayg, azg) * 57.2958;

      pitch = 0.98f * (pitch + gys * 0.005f)
            + 0.02f * accPitch;
    }
    delay(5);
  }

  lastTime = millis();
  Serial.println("Ready");
}

// ─────────────────────────────────────────────────────────
// Main loop
// ─────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  if (dt < 0.002 || dt > 0.1) {
    stopMotors();
    return;
  }

  int16_t ax, ay, az, gx, gy;
  if (!readIMU(ax, ay, az, gx, gy)) {
    stopMotors();
    return;
  }

  // ── Complementary filter with axis mapping ──────────────
  float ayg = (ay / 16384.0) * ACC_Y_SIGN;
  float azg = (az / 16384.0) * ACC_Z_SIGN;
  float gys = (gy / 131.0)   * GYRO_Y_SIGN;

  float accPitch = atan2(ayg, azg) * 57.2958;

  pitch = CF_ALPHA * (pitch + gys * dt)
        + (1.0 - CF_ALPHA) * accPitch;

  float error = pitch - PITCH_OFFSET;

  if (fabs(error) > FALL_ANGLE) {
    stopMotors();
    integral  = 0.0;
    prevError = 0.0;
    return;
  }

  integral  = constrain(integral + error * dt, -I_LIMIT, I_LIMIT);
  float derivative = (error - prevError) / dt;
  prevError = error;

  float output = Kp * error + Ki * integral + Kd * derivative;
  output = constrain(output, -255, 255);

  float drive = 0.0;
  if (fabs(output) >= 1.0) {
    float sign = (output > 0) ? 1.0 : -1.0;
    drive = sign * (DEAD_BAND + (fabs(output) / 255.0) * (255.0 - DEAD_BAND));
  }

  if (drive == 0.0) {
    stopMotors();
  } else {
    float trimFactor = 1.0 + MOTOR_TRIM / 255.0;
    setMotors(drive, drive * trimFactor);
  }

Serial.print(now);
Serial.print(",");
Serial.print(0.0);     // setpoint
Serial.print(",");
Serial.println(error, 3);
}
