#include "bmi088.h"
#include "bmi088Strings.h"
#include <Arduino.h>

bmi088::BusI2C bus(Wire, 0x18);
bmi088::Accel accel(bus);

constexpr uint8_t PIN_SDA = 47;
constexpr uint8_t PIN_SCL = 14;

constexpr uint8_t PIN_INT1_ACCEL = 12;

void setup() {
  Serial.begin(115200);
  Wire.begin(PIN_SDA, PIN_SCL);

  if (accel.init()) {
    Serial.println("BMI088 initialized successfully");
  } else {
    Serial.printf("BMI088 initialization failed: %s\n", bmi088::toString(accel.getError()));
  }
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case '1':
      {
        uint8_t id;
        if (accel.getChipID(id)) {
          Serial.print("Chip ID: ");
          Serial.printf("0x%02X\n", id);
        } else {
          Serial.println("Failed to read Chip ID");
        }
      } break;

      case '2':
      {
        bmi088::RawData data;
        if (accel.getRawData(data)) {
          Serial.printf("X: %d, Y: %d, Z: %d\n", data.x, data.y, data.z);
        } else {
          Serial.println("Failed to read acceleration data");
        }
      } break;

      case '3':
      {
        bmi088::ConvertedData data;
        if (accel.getMilliGAcceleration(data)) {
          Serial.printf("X: %.2f, Y: %.2f, Z: %.2f\n", data.x, data.y, data.z);
        } else {
          Serial.println("Failed to read acceleration data");
        }
      } break;

      case '4':
      {
        float temperature;
        if (accel.getTemperature(temperature)) {
          Serial.printf("Temperature: %.2f °C\n", temperature);
        } else {
          Serial.println("Failed to read temperature");
        }
      } break;

      case '5':
      {
        Serial.println("Get config");

        bmi088::Accel::Bandwidth bw;
        bmi088::Accel::DataRate dr;

        if (accel.getConfig(bw, dr)) {
          Serial.printf("Bandwidth: %s, DataRate: %s\n",
            bmi088::toString(bw),
            bmi088::toString(dr));
        } else {
          Serial.println("Failed to read config");
        }
      } break;

      case '6':
      {
        Serial.println("Offset (keep the sensor flat, Z axis up)");
        if (!accel.calibrateOffset()) {
          Serial.println("Failed to read accelerometer data");
          break;
        }

        bmi088::RawData offset;
        accel.getRawOffset(offset);
        Serial.printf("Offset: X: %d, Y: %d, Z: %d\n", offset.x, offset.y, offset.z);
      } break;

      case '7':
      {
        Serial.println("Config pin INT1 to output");

        if (accel.pinConfig(bmi088::Accel::GpioPin::Int1,
              bmi088::Accel::GpioMode::Output,
              bmi088::Accel::GpioBehavior::OpenDrain)) {
          Serial.println("Pin configured successfully");
        } else {
          Serial.println("Failed to configure pin");
          break;
        }

        pinMode(PIN_INT1_ACCEL, INPUT_PULLUP);
      } break;

      case '8':
      {
        Serial.println("Change INT1 active level");

        static bmi088::Accel::GpioLevel level;

        level = (level == bmi088::Accel::GpioLevel::High) ? bmi088::Accel::GpioLevel::Low
                                                          : bmi088::Accel::GpioLevel::High;

        if (accel.setPinActiveLevel(bmi088::Accel::GpioPin::Int1, level)) {
          // With no interrupt mapped the pin stays at the inactive level
          Serial.printf("Active level: %s (pin idle %s)\n",
            level == bmi088::Accel::GpioLevel::High ? "High" : "Low",
            level == bmi088::Accel::GpioLevel::High ? "Low" : "High");
        } else {
          Serial.println("Failed to set pin active level");
          break;
        }

      } break;

      case '9':
      {
        Serial.println("Read digital pin level");
        bool state = digitalRead(PIN_INT1_ACCEL);
        Serial.printf("Pin level: %s\n", state ? "High" : "Low");
      } break;

      case '0':
      {
        Serial.println("Get pin config");
        bmi088::Accel::GpioMode mode;
        bmi088::Accel::GpioBehavior behavior;
        if (accel.getPinConfig(bmi088::Accel::GpioPin::Int1, mode, behavior)) {
          Serial.printf("Mode: %s, Behavior: %s\n",
            mode == bmi088::Accel::GpioMode::Input    ? "Input"
            : mode == bmi088::Accel::GpioMode::Output ? "Output"
                                                      : "Disabled",
            behavior == bmi088::Accel::GpioBehavior::OpenDrain ? "Open Drain" : "Push Pull");
        } else {
          Serial.println("Failed to read pin config");
        }
      } break;

      case 't':
      {
        Serial.println("Self test");
        bool result_x = false;
        bool result_y = false;
        bool result_z = false;

        if (!accel.selfTest(result_x, result_y, result_z)) {
          Serial.println("Failed to run self test");
          break;
        }

        bmi088::ConvertedData diff;
        accel.getSelfTestData(diff);

        Serial.printf("Self test results: X: %s (%.0f mg), Y: %s (%.0f mg), Z: %s (%.0f mg)\n",
          result_x ? "Pass" : "Fail",
          diff.x,
          result_y ? "Pass" : "Fail",
          diff.y,
          result_z ? "Pass" : "Fail",
          diff.z);
      } break;

      case 'r':
      {
        Serial.println("Resetting...");
        ESP.restart();
      }
    }
  }
}
