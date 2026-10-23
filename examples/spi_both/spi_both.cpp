#include <Arduino.h>
#include <bmi088.h>

// The PS pin of the BMI088 must be connected to GND to select SPI
constexpr int8_t PIN_SCK      = 12;
constexpr int8_t PIN_MISO     = 13;
constexpr int8_t PIN_MOSI     = 11;
constexpr int8_t PIN_CS_ACCEL = 10;
constexpr int8_t PIN_CS_GYRO  = 9;

bmi088::BusSPI bus_accel(SPI, PIN_CS_ACCEL);
bmi088::BusSPI bus_gyro(SPI, PIN_CS_GYRO);
bmi088::Imu imu(bus_accel, bus_gyro);

uint32_t last_time = 0;

void setup() {
  Serial.begin(115200);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);

  if (imu.init()) {
    Serial.println("BMI088 initialized successfully");
  } else {
    Serial.println("BMI088 initialization failed");
  }

  Serial.printf("Accelerometer connected: %s\n", imu.accelerometer().isConnected() ? "Yes" : "No");
  Serial.printf("Gyroscope connected: %s\n", imu.gyroscope().isConnected() ? "Yes" : "No");
}

void loop() {
  if (millis() - last_time < 100) return;
  last_time = millis();

  bmi088::ConvertedData accel_data;
  bmi088::ConvertedData gyro_data;
  float temperature;

  if (!imu.accelerometer().getAcceleration(accel_data) ||
      !imu.gyroscope().getAngularVelocityRad(gyro_data) ||
      !imu.accelerometer().getTemperature(temperature)) {
    Serial.println("Failed to read data");
    return;
  }

  Serial.printf("Accel [m/s²]: X: %.3f, Y: %.3f, Z: %.3f\n",
    accel_data.x,
    accel_data.y,
    accel_data.z);
  Serial.printf("Gyro [rad/s]: X: %.3f, Y: %.3f, Z: %.3f\n", gyro_data.x, gyro_data.y, gyro_data.z);
  Serial.printf("Temperature: %.2f °C\n\n", temperature);
}
