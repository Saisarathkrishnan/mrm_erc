#include <Wire.h>
#include <Adafruit_BNO08x.h>

#define SDA_PIN 22
#define SCL_PIN 23

TwoWire I2C_2 = TwoWire(1);
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

bool calibrated = false;

void setReports() {
  bno08x.enableReport(SH2_ROTATION_VECTOR, 10000);
}

void quaternionToEuler(float qr, float qi, float qj, float qk,
                       float &yaw, float &pitch, float &roll) {

  float sinr_cosp = 2.0f * (qr * qi + qj * qk);
  float cosr_cosp = 1.0f - 2.0f * (qi * qi + qj * qj);
  roll = atan2(sinr_cosp, cosr_cosp);

  float sinp = 2.0f * (qr * qj - qk * qi);

  if (fabs(sinp) >= 1.0f)
    pitch = copysign(M_PI / 2.0f, sinp);
  else
    pitch = asin(sinp);

  float siny_cosp = 2.0f * (qr * qk + qi * qj);
  float cosy_cosp = 1.0f - 2.0f * (qj * qj + qk * qk);
  yaw = atan2(siny_cosp, cosy_cosp);

  yaw *= 180.0f / M_PI;
  pitch *= 180.0f / M_PI;
  roll *= 180.0f / M_PI;

  if (yaw < 0)
    yaw += 360.0f;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  I2C_2.begin(SDA_PIN, SCL_PIN);
  I2C_2.setClock(100000);

  if (!bno08x.begin_I2C(0x4A, &I2C_2)) {
    Serial.println("BNO08x not found on bus 2");
    while (1);
  }

  delay(100);

  setReports();

  Serial.println("Rotate IMU to calibrate...");
}

void loop() {

  if (bno08x.wasReset()) {
    calibrated = false;
    setReports();
  }

  if (!bno08x.getSensorEvent(&sensorValue))
    return;

  if (!calibrated) {

    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {

      uint8_t status = sensorValue.status;

      Serial.print("Cal STATUS:");
      Serial.println(status);

      if (status == 3) {
        calibrated = true;
        Serial.println("Calibration saved");
      }
    }

    delay(300);
    return;
  }

  if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {

    float qr = sensorValue.un.rotationVector.real;
    float qi = sensorValue.un.rotationVector.i;
    float qj = sensorValue.un.rotationVector.j;
    float qk = sensorValue.un.rotationVector.k;

    float yaw;
    float pitch;
    float roll;

    quaternionToEuler(
      qr,
      qi,
      qj,
      qk,
      yaw,
      pitch,
      roll
    );

    Serial.print("2_");
    Serial.print(yaw, 2);
    Serial.print("_");
    Serial.print(pitch, 2);
    Serial.print("_");
    Serial.println(roll, 2);
  }

  delay(10);
}