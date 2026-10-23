#include <Arduino.h>
#include <bmi088.h>

bmi088::BusI2C bus(Wire, 0x68);
bmi088::Gyro gyro(bus);

constexpr uint8_t PIN_SDA = 47;
constexpr uint8_t PIN_SCL = 14;

bool enable_continuous_read = false;
uint32_t last_time          = 0;

float roll  = 0; // Roll angle x axis
float pitch = 0; // Pitch angle y axis
float yaw   = 0; // Yaw angle z axis

void setup() {
  Serial.begin(115200);
  Wire.begin(PIN_SDA, PIN_SCL);

  if (gyro.init()) {
    Serial.println("BMI088 initialized successfully");
  } else {
    Serial.println("BMI088 initialization failed");
  }
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case '1':
      {
        uint8_t id;
        if (gyro.getChipID(id)) {
          Serial.print("Chip ID: ");
          Serial.printf("0x%02X\n", id);
        } else {
          Serial.println("Failed to read Chip ID");
        }
      } break;

      case '2':
      {
        bmi088::RawData data;
        if (gyro.getRawData(data)) {
          Serial.printf("X: %d, Y: %d, Z: %d\n", data.x, data.y, data.z);
        } else {
          Serial.println("Failed to read angular velocity data");
        }
      } break;

      case '3':
      {
        enable_continuous_read = !enable_continuous_read;
        last_time              = micros();
      } break;

      case '4':
      {
        Serial.println("Set zero angles");
        pitch = 0;
        roll  = 0;
        yaw   = 0;
      } break;

      case '5':
      {
        Serial.println("Calibrate bias (keep the sensor still)");
        if (!gyro.calibrateOffset()) {
          Serial.println("Failed to read angular velocity data");
          break;
        }

        bmi088::RawData offset;
        gyro.getRawOffset(offset);
        Serial.printf("Bias: X: %d, Y: %d, Z: %d\n", offset.x, offset.y, offset.z);
      } break;

      case 't':
      {
        bool pass;
        if (gyro.selfTest(pass)) {
          Serial.printf("Self test: %s\n", pass ? "Pass" : "Fail");
        } else {
          Serial.println("Failed to run self test");
        }
      } break;

      default: break;
    }
  }

  if (enable_continuous_read && (micros() - last_time >= 100000)) {
    const uint32_t now = micros();
    const float dt     = (now - last_time) * 1e-6f;
    last_time          = now;

    bmi088::ConvertedData gyro_data;
    if (!gyro.getAngularVelocity(gyro_data)) {
      Serial.println("Failed to read angular velocity data");
      return;
    }
    Serial.printf("Gyro: X: %.2f, Y: %.2f, Z: %.2f\n", gyro_data.x, gyro_data.y, gyro_data.z);

    roll  = bmi088::Gyro::getRelativeEulerAngle(roll, gyro_data.x, dt);
    pitch = bmi088::Gyro::getRelativeEulerAngle(pitch, gyro_data.y, dt);
    yaw   = bmi088::Gyro::getRelativeEulerAngle(yaw, gyro_data.z, dt);

    Serial.printf("Pitch: %.2f, Roll: %.2f, Yaw: %.2f\n", pitch, roll, yaw);
    Serial.println("------------------------------------------------");
  }
}
