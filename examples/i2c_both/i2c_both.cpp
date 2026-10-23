#include <Arduino.h>
#include <bmi088.h>

#include <Wire.h>

bmi088::BusI2C bus_accel(Wire, 0x18);
bmi088::BusI2C bus_gyro(Wire, 0x68);
bmi088::Imu imu(bus_accel, bus_gyro);

constexpr uint8_t PIN_SDA = 47;
constexpr uint8_t PIN_SCL = 14;

bool enable_continuous_read = false;
uint32_t last_time          = 0;

float filtered_angle_x = 0;
float filtered_angle_y = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(PIN_SDA, PIN_SCL);

  if (imu.init()) {
    Serial.println("BMI088 initialized successfully");
  } else {
    Serial.println("BMI088 initialization failed");
  }

  imu.accelerometer().setConfig(bmi088::Accel::Bandwidth::Normal,
    bmi088::Accel::DataRate::Odr1600Hz);

  imu.gyroscope().setDataRateAndBandwidth(bmi088::Gyro::OdrAndBandwidth::Odr2000HzFilter230Hz);
}

void loop() {

  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case '1':
      {
        enable_continuous_read = !enable_continuous_read;
        last_time              = micros();
      } break;

      case '2':
      {
        Serial.println("Set offset to accelerometer (keep the sensor flat, Z axis up)");
        if (!imu.accelerometer().calibrateOffset()) {
          Serial.println("Failed to read accelerometer data");
          break;
        }

        bmi088::RawData offset;
        imu.accelerometer().getRawOffset(offset);
        Serial.printf("Offset: X: %d, Y: %d, Z: %d\n", offset.x, offset.y, offset.z);
      } break;

      case '3':
      {
        Serial.println("Calibrate gyroscope bias (keep the sensor still)");
        if (!imu.gyroscope().calibrateOffset()) {
          Serial.println("Failed to read gyroscope data");
          break;
        }

        bmi088::RawData offset;
        imu.gyroscope().getRawOffset(offset);
        Serial.printf("Bias: X: %d, Y: %d, Z: %d\n", offset.x, offset.y, offset.z);
      } break;
    }
  }

  if (enable_continuous_read && (micros() - last_time >= 50000)) {
    const uint32_t now = micros();
    const float dt     = (now - last_time) * 1e-6f;
    last_time          = now;

    bmi088::ConvertedData accel_data;
    bmi088::ConvertedData gyro_data;

    if (!imu.accelerometer().getMilliGAcceleration(accel_data) ||
        !imu.gyroscope().getAngularVelocity(gyro_data)) {
      Serial.println("Failed to read data");
      return;
    }

    Serial.printf("- Accel: X: %.2f, Y: %.2f, Z: %.2f\n- Gyro: X: %.2f, Y: %.2f, Z: %.2f\n",
      accel_data.x,
      accel_data.y,
      accel_data.z,
      gyro_data.x,
      gyro_data.y,
      gyro_data.z);

    float absolute_angle_y = bmi088::Accel::getAbsoluteYAngle(accel_data);
    float absolute_angle_x = bmi088::Accel::getAbsoluteXAngle(accel_data);

    Serial.printf("Abs Angle X: %.2f° - Abs Angle Y: %.2f°\n", absolute_angle_x, absolute_angle_y);

    filtered_angle_x =
      bmi088::Imu::complementaryFilter(filtered_angle_x, absolute_angle_x, gyro_data.x, dt);
    filtered_angle_y =
      bmi088::Imu::complementaryFilter(filtered_angle_y, absolute_angle_y, gyro_data.y, dt);
    Serial.printf("Angle X: %.2f° - Angle Y: %.2f°\n", filtered_angle_x, filtered_angle_y);
    Serial.println();
  }
}
