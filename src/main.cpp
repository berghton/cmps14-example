#include <Arduino.h>
#include <Wire.h>

// 0xC0 shifted one bit right => 0x60
#define CMPS14_ADDRESS 0x60

// 0x00 in the event of a read the CMPS14 will reply with the software version,
// for a write it acts as the command register.
#define COMMAND_8_REGISTER 0x00
//  0x01 is the bearing as a 0-255 value, this may be easier for some
//  applications than 0-3599 which requires two bytes.
#define BEARING_8_REGISTER 0x01
// 0x02 and 0x03 (high byte first) form a 16 bit unsigned integer in the range
// 0-3599 to represent the bearing (yaw angle). The result should be divided by
// 10 to scale 0-359.9°.
#define BEARING_16_REGISTER 0x02
// 0x04 and 0x05 are the pitch and roll angles, giving an angle of 0 when the
// board is flat and up to +/- 90° at maximum tilt in either direction.
#define PITCH_8_REGISTER 0x04
#define ROLL_8_REGISTER 0x05
// 0x06-0x0B Geomagnetic field calibrated for hard and soft iron effects such
// that the vector is aligned with the declination and heading of Earth’s
// magnetic field.The units are uTesla.The Q point is 4.
// Registers are ordered axis X MSB, axis X LSB, axis Y MSB, axis Y LSB, axis Z
// MSB, axis Z LSB
#define MAGNETOMETER_X_16_REGISTER 0x06
#define MAGNETOMETER_Y_16_REGISTER 0x08
#define MAGNETOMETER_Z_16_REGISTER 0x0A
// 0x0c-0x11 The linear acceleration sensor reports the acceleration of the
// device minus gravity. The units are m/s^2. The Q point is 8.
// Registers are ordered axis X MSB, axis X LSB, axis Y MSB, axis Y LSB, axis Z
// MSB, axis Z LSB
#define LINEAR_ACC_X_16_REGISTER 0x0C
#define LINEAR_ACC_Y_16_REGISTER 0x0E
#define LINEAR_ACC_Z_16_REGISTER 0x10
// 0x12-0x17 The gyroscope sensor reports raw readings from the physical
// gyroscope MEMS sensor. The units are ADCs.Interpretation of the reported
// values is sensor dependent. Registers are ordered axis X MSB, axis X LSB,
// axis Y MSB, axis Y LSB, axis Z MSB, axis Z LSB
#define GYRO_RAW_X_16_REGISTER 0x12
#define GYRO_RAW_Y_16_REGISTER 0x14
#define GYRO_RAW_Z_16_REGISTER 0x16
// Registers 0x1A (high byte) and 0x1B (low byte) form a 16 bit pitch angle for
// +/- 90 from the horizontal plane. Value is in tenths of degrees(range of +/-
// 900). Registers are ordered axis X MSB, axis X LSB, axis Y MSB, axis Y LSB,
// axis Z MSB, axis Z LSB
#define PITCH_16_REGISTER 0x1A
// Registers 0x1C (high byte) and 0x1D (low byte) form a 16 bit roll angle for
// +/- 180 from the horizontal plane. Value is in tenths of degrees(range of +/-
// 1800).
#define ROLL_16_REGISTER 0x1C
// 0x1E provides feedback on the degree of the calibration that the automatic
// calibration routines have achieved. Please see the calibration section for
// more details.
#define CALIBRATION_STATE_8_REGISTER 0x1E
// 0x1F – 0x24 The accelerometer sensor reports the total acceleration of the
// device. The units are m/s^2. The Q point is 8.
// Registers are ordered axis X MSB, axis X LSB, axis Y MSB, axis Y LSB, axis Z
// MSB, axis Z LSB
#define ACC_X_16_REGISTER 0x1F
#define ACC_Y_16_REGISTER 0x21
#define ACC_Z_16_REGISTER 0x23
// 0x25 – 0x2A The gyroscope calibrated sensor reports drift-compensated
// rotational velocity. The units are rad/s. The Q point is 9.
// Registers are ordered axis X MSB, axis X LSB, axis Y MSB, axis Y LSB, axis Z
// MSB, axis Z LSB
#define GYRO_CALIBRATED_X_16_REGISTER 0x25
#define GYRO_CALIBRATED_Y_16_REGISTER 0x27
#define GYRO_CALIBRATED_Z_16_REGISTER 0x29

void setup() {
  Wire.begin();
  Serial.begin(9600);
}

void loop() {
  unsigned int bearing;
  int pitch, roll, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
  unsigned char bearing_high, bearing_low, pitch_high, pitch_low, roll_high,
      roll_low, acc_x_high, acc_x_low, acc_y_high, acc_y_low, acc_z_high,
      acc_z_low, gyro_x_high, gyro_x_low, gyro_y_high, gyro_y_low, gyro_z_high,
      gyro_z_low;

  // Bearing
  Wire.beginTransmission(CMPS14_ADDRESS);
  Wire.write(BEARING_16_REGISTER);
  Wire.endTransmission();

  Wire.requestFrom(CMPS14_ADDRESS, 2);
  while (Wire.available() < 2)
    ;
  bearing_high = Wire.read();
  bearing_low = Wire.read();

  bearing = bearing_high;
  bearing <<= 8;
  bearing += bearing_low;

  // Pitch
  Wire.beginTransmission(CMPS14_ADDRESS);
  Wire.write(PITCH_16_REGISTER);
  Wire.endTransmission();

  Wire.requestFrom(CMPS14_ADDRESS, 2);
  while (Wire.available() < 2)
    ;
  pitch_high = Wire.read();
  pitch_low = Wire.read();

  pitch = pitch_high;
  pitch <<= 8;
  pitch += pitch_low;

  // Roll
  Wire.beginTransmission(CMPS14_ADDRESS);
  Wire.write(ROLL_16_REGISTER);
  Wire.endTransmission();

  Wire.requestFrom(CMPS14_ADDRESS, 2);
  while (Wire.available() < 2)
    ;
  roll_high = Wire.read();
  roll_low = Wire.read();

  roll = roll_high;
  roll <<= 8;
  roll += roll_low;

  // Accelerometer
  Wire.beginTransmission(CMPS14_ADDRESS);
  Wire.write(LINEAR_ACC_X_16_REGISTER);
  Wire.endTransmission();

  Wire.requestFrom(CMPS14_ADDRESS, 6);
  while (Wire.available() < 6)
    ;
  acc_x_high = Wire.read();
  acc_x_low = Wire.read();
  acc_y_high = Wire.read();
  acc_y_low = Wire.read();
  acc_z_high = Wire.read();
  acc_z_low = Wire.read();

  acc_x = acc_x_high;
  acc_x <<= 8;
  acc_x += acc_x_low;

  acc_y = acc_y_high;
  acc_y <<= 8;
  acc_y += acc_y_low;

  acc_z = acc_z_high;
  acc_z <<= 8;
  acc_z += acc_z_low;

  // Gyroscope
  Wire.beginTransmission(CMPS14_ADDRESS);
  Wire.write(GYRO_CALIBRATED_X_16_REGISTER);
  Wire.endTransmission();

  Wire.requestFrom(CMPS14_ADDRESS, 6);
  while (Wire.available() < 6)
    ;
  gyro_x_high = Wire.read();
  gyro_x_low = Wire.read();
  gyro_y_high = Wire.read();
  gyro_y_low = Wire.read();
  gyro_z_high = Wire.read();
  gyro_z_low = Wire.read();

  gyro_x = gyro_x_high;
  gyro_x <<= 8;
  gyro_x += gyro_x_low;

  gyro_y = gyro_y_high;
  gyro_y <<= 8;
  gyro_y += gyro_y_low;

  gyro_z = gyro_z_high;
  gyro_z <<= 8;
  gyro_z += gyro_z_low;

  // For serial plotting
  Serial.print(bearing / 10);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.print(roll);
  Serial.print(",");
  Serial.print(acc_x);
  Serial.print(",");
  Serial.print(acc_y);
  Serial.print(",");
  Serial.print(acc_z);
  Serial.print(",");
  Serial.print(gyro_x);
  Serial.print(",");
  Serial.print(gyro_y);
  Serial.print(",");
  Serial.println(gyro_z);

  delay(100);
}