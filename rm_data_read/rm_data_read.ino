#include <Wire.h>
// #include <Adafruit_BNO08x.h>
#include <SparkFun_BNO080_Arduino_Library.h>

#define AS5600_ADDR 0x36
#define i2cmux 0x70

#define imu1_add 0x4B
#define imu2_add 0x4A

BNO080 imu1;
BNO080 imu2;
float r1=400;
float p1=400;
float p1c=400;
float yaw1=400;
float r2=400;
float p2=400;
float y2=400;
float normalize360(float angle)
{
    while (angle < 0.0f)
        angle += 360.0f;

    while (angle >= 360.0f)
        angle -= 360.0f;
      return angle;
}
void QuaternionToYPR(float qr, float qi, float qj, float qk,
                     float &roll, float &pitch, float &yaw)
{
    // Roll (X)
    roll = atan2(2.0f * (qr * qi + qj * qk),
                 1.0f - 2.0f * (qi * qi + qj * qj)) * RAD_TO_DEG;

    // Pitch (continuous around Y)
    pitch = atan2(2.0f * (qr * qj + qi * qk),
                  1.0f - 2.0f * (qj * qj + qi * qi)) * RAD_TO_DEG;

    // Yaw (Z)
    yaw = atan2(2.0f * (qr * qk + qi * qj),
                1.0f - 2.0f * (qj * qj + qk * qk)) * RAD_TO_DEG;
}


void mux(uint8_t bus) {
  Wire.beginTransmission(i2cmux);
  Wire.write(1 << bus);
  Wire.endTransmission();
}

uint16_t readRawAngle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0E);
  Wire.endTransmission(false);

  Wire.requestFrom(AS5600_ADDR, 2);

  if (Wire.available() < 2)
      return 0;

  uint8_t highByte = Wire.read();
  uint8_t lowByte = Wire.read();

  return ((highByte << 8) | lowByte) & 0x0FFF;
}

float getAngleDegrees() {
  uint16_t raw = readRawAngle();
  return (raw * 360.0) / 4096.0;
}


void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("STARTING");

  Wire.begin();
  Wire.setClock(400000);

  Serial.println("Initializing IMU1...");

  if (!imu1.begin(imu1_add, Wire)) {
    Serial.println("IMU1 NOT CONNECTED");
  } else {
    Serial.println("IMU1 CONNECTED");
    imu1.enableRotationVector(50);
  }

  delay(100);

  Serial.println("Initializing IMU2...");

  if (!imu2.begin(imu2_add, Wire)) {
    Serial.println("IMU2 NOT CONNECTED");
  } else {
    Serial.println("IMU2 CONNECTED");
    imu2.enableRotationVector(50);
  }

  Serial.println("SETUP COMPLETE");
}

void loop() {
float pz;

if (imu1.dataAvailable()) {
    float qr = imu1.getQuatReal();
    float qi = imu1.getQuatI();
    float qj = imu1.getQuatJ();
    float qk = imu1.getQuatK();

    QuaternionToYPR(qr, qi, qj, qk, r1, p1, yaw1);

    //normalize360(yaw1);
   p1=normalize360(p1);
   p1c=p1-270;
   p1c=normalize360(p1c);

   // p1 = p1 +90;

//if (p1 >= 360.0f)
  //  p1 -= 360.0f;
//  if (p1 < 0.0f)
    //p1 += 360.0f;
}

if (imu2.dataAvailable()) {
    float qr = imu2.getQuatReal();
    float qi = imu2.getQuatI();
    float qj = imu2.getQuatJ();
    float qk = imu2.getQuatK();

    QuaternionToYPR(qr, qi, qj, qk, r2, p2, y2);

    p2=normalize360(p2);
    p2=p2-270;
    p2=normalize360(p2);
    p2=p2-p1+180;
    p2=normalize360(p2);
}

 
  mux(0);
  uint16_t rawAngle1 = readRawAngle();
  float angleDeg1 = (rawAngle1 * 360.0) / 4096.0;

  mux(1);
  uint16_t rawAngle2 = readRawAngle();
  float angleDeg2 = (rawAngle2 * 360.0) / 4096.0;

  Serial.printf("l1_%.2f_l2_%.2f_bl_%.2f_br_%.2f_sw_%.2f\n", p1c, p2,angleDeg1,angleDeg2,y2);
  //Serial.printf("r_%.2f_p_%.2f_y_%.2f\n",r1,p1,yaw1);
 // Serial.printf("raw1: %d, ang1: %.2f     raw2: %d, ang2: %.2f\n", rawAngle1, angleDeg1, rawAngle2, angleDeg2);

  delay(50);
}