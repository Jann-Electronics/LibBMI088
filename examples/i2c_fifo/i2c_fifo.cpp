#include <Arduino.h>
#include <bmi088.h>

bmi088::BusI2C bus_accel(Wire, 0x18);
bmi088::BusI2C bus_gyro(Wire, 0x68);
bmi088::Imu imu(bus_accel, bus_gyro);

constexpr uint8_t PIN_SDA = 47;
constexpr uint8_t PIN_SCL = 14;

// The accelerometer FIFO holds up to 146 frames (1024 bytes) and the gyroscope FIFO 100 frames
bmi088::RawData accel_frames[64];
bmi088::RawData gyro_frames[64];

uint32_t last_time = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(PIN_SDA, PIN_SCL);

  if (imu.init()) {
    Serial.println("BMI088 initialized successfully");
  } else {
    Serial.println("BMI088 initialization failed");
  }

  bmi088::Accel& accel = imu.accelerometer();
  bmi088::Gyro& gyro   = imu.gyroscope();

  // Accelerometer at 100 Hz, FIFO in stream mode storing the acceleration
  accel.setConfig(bmi088::Accel::Bandwidth::Normal, bmi088::Accel::DataRate::Odr100Hz);
  accel.setFifoMode(bmi088::Accel::FifoMode::Stream);
  accel.enableFifo(true);
  accel.flushFifo();

  // Gyroscope at 100 Hz, FIFO in stream mode (writing the mode clears the FIFO)
  gyro.setDataRateAndBandwidth(bmi088::Gyro::OdrAndBandwidth::Odr100HzFilter32Hz);
  gyro.setFifoMode(bmi088::Gyro::FifoMode::Stream);

  last_time = millis();
}

void loop() {
  // Read the FIFOs every 200 ms, about 20 frames each
  if (millis() - last_time < 200) return;
  last_time = millis();

  size_t accel_count = 0;
  size_t gyro_count  = 0;

  if (!imu.accelerometer().readFifo(accel_frames, 64, accel_count)) {
    Serial.println("Failed to read accelerometer FIFO");
    return;
  }

  if (!imu.gyroscope().readFifo(gyro_frames, 64, gyro_count)) {
    Serial.println("Failed to read gyroscope FIFO");
    return;
  }

  Serial.printf("Accel frames: %u - Gyro frames: %u\n",
    static_cast<unsigned>(accel_count),
    static_cast<unsigned>(gyro_count));

  if (accel_count > 0) {
    bmi088::ConvertedData data;
    imu.accelerometer().convertToMilliG(accel_frames[accel_count - 1], data);
    Serial.printf("Last accel: X: %.2f, Y: %.2f, Z: %.2f mg\n", data.x, data.y, data.z);
  }

  if (gyro_count > 0) {
    bmi088::ConvertedData data;
    imu.gyroscope().convertToDps(gyro_frames[gyro_count - 1], data);
    Serial.printf("Last gyro: X: %.2f, Y: %.2f, Z: %.2f dps\n", data.x, data.y, data.z);
  }

  Serial.println();
}
