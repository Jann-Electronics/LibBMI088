#include <Arduino.h>
#include <bmi088.h>

bmi088::BusI2C bus_accel(Wire, 0x18);
bmi088::BusI2C bus_gyro(Wire, 0x68);
bmi088::Imu imu(bus_accel, bus_gyro);

constexpr uint8_t PIN_SDA        = 47;
constexpr uint8_t PIN_SCL        = 14;
constexpr uint8_t PIN_INT1_ACCEL = 12;
constexpr uint8_t PIN_INT3_GYRO  = 13;

volatile bool accel_ready = false;
volatile bool gyro_ready  = false;

void IRAM_ATTR onAccelReady() { accel_ready = true; }

void IRAM_ATTR onGyroReady() { gyro_ready = true; }

uint32_t accel_count = 0;
uint32_t gyro_count  = 0;
uint32_t last_print  = 0;

bmi088::ConvertedData accel_data = {};
bmi088::ConvertedData gyro_data  = {};

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

  // Accelerometer: 100 Hz data ready on INT1, push-pull output, active high
  accel.setConfig(bmi088::Accel::Bandwidth::Normal, bmi088::Accel::DataRate::Odr100Hz);
  accel.pinConfig(bmi088::Accel::GpioPin::Int1,
    bmi088::Accel::GpioMode::Output,
    bmi088::Accel::GpioBehavior::PushPull);
  accel.setPinActiveLevel(bmi088::Accel::GpioPin::Int1, bmi088::Accel::GpioLevel::High);
  accel.setMapInterrupt(true, false);

  // Gyroscope: 100 Hz data ready on INT3, push-pull output, active high
  gyro.setDataRateAndBandwidth(bmi088::Gyro::OdrAndBandwidth::Odr100HzFilter32Hz);
  gyro.pinConfig(bmi088::Gyro::GpioPin::Int3, bmi088::Gyro::GpioOutConfig::PushPull);
  gyro.setPinActiveLevel(bmi088::Gyro::GpioPin::Int3, bmi088::Gyro::GpioLevel::High);
  gyro.setMapInterrupt(bmi088::Gyro::GpioMapInterrupt::InterruptOnInt3);
  gyro.enableInterrupt(true);

  pinMode(PIN_INT1_ACCEL, INPUT);
  pinMode(PIN_INT3_GYRO, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_INT1_ACCEL), onAccelReady, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_INT3_GYRO), onGyroReady, RISING);
}

void loop() {
  if (accel_ready) {
    accel_ready = false;
    if (imu.accelerometer().getMilliGAcceleration(accel_data)) ++accel_count;
  }

  if (gyro_ready) {
    gyro_ready = false;
    if (imu.gyroscope().getAngularVelocity(gyro_data)) ++gyro_count;
  }

  if (millis() - last_print >= 1000) {
    last_print = millis();

    Serial.printf("Accel: %lu samples/s - X: %.2f, Y: %.2f, Z: %.2f mg\n",
      static_cast<unsigned long>(accel_count),
      accel_data.x,
      accel_data.y,
      accel_data.z);
    Serial.printf("Gyro: %lu samples/s - X: %.2f, Y: %.2f, Z: %.2f dps\n\n",
      static_cast<unsigned long>(gyro_count),
      gyro_data.x,
      gyro_data.y,
      gyro_data.z);

    accel_count = 0;
    gyro_count  = 0;
  }
}
