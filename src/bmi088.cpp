#include "bmi088.h"

namespace bmi088 {

namespace {
constexpr float RadToDeg = 57.29577951f;
constexpr float DegToRad = 0.01745329252f;

constexpr size_t AccelFifoFrameSize = 7; // Header + X, Y, Z
constexpr size_t GyroFifoFrameSize  = 6; // X, Y, Z

// FIFO burst sizes, rounded down to whole frames
constexpr size_t AccelFifoChunk = BMI088_FIFO_BUFFER_SIZE / AccelFifoFrameSize * AccelFifoFrameSize;
constexpr size_t GyroFifoChunk  = BMI088_FIFO_BUFFER_SIZE / GyroFifoFrameSize * GyroFifoFrameSize;
static_assert(AccelFifoChunk > 0 && GyroFifoChunk > 0, "BMI088_FIFO_BUFFER_SIZE must be >= 7");

// Payload size of an accelerometer FIFO frame from its header, 0 if it is not a valid header
size_t accelFifoPayload(uint8_t header) {
  switch (header & 0xFC) { // Bits 1:0 are the INT tags of the acceleration frame
    case 0x84: return 6;   // Acceleration
    case 0x40: return 1;   // Skip
    case 0x44: return 3;   // Sensor time
    case 0x48: return 1;   // Input config
    case 0x50: return 1;   // Sample drop
    default: return 0;     // 0x80: over-read, end of data
  }
}

RawData decodeRawData(const uint8_t* values) {
  RawData data;
  data.x = (static_cast<int16_t>(values[1]) << 8) | values[0];
  data.y = (static_cast<int16_t>(values[3]) << 8) | values[2];
  data.z = (static_cast<int16_t>(values[5]) << 8) | values[4];
  return data;
}
} // namespace

// MARK: Accel
/* ---------------------------------------------------------------------------------------------- */
/*                                     PIN BMI088 Acceleration                                    */
/* ---------------------------------------------------------------------------------------------- */

Accel::Accel(Bus& bus)
    : _bus(bus)
    , _register(0)
    , _values{}
    , _range(static_cast<uint8_t>(Range::Range6g))
    , _factor_mg(0)
    , _raw_offset{}
    , _test_result_data{}
    , _on_error(nullptr)
    , _error(Error::None) {
  _calculateFactorMilliG();
}

float Accel::getAbsoluteXAngle(const ConvertedData& data) {
  return atan2f(data.y, data.z) * RadToDeg;
}

float Accel::getAbsoluteYAngle(const ConvertedData& data) {
  return atan2f(-data.x, sqrtf(data.y * data.y + data.z * data.z)) * RadToDeg;
}

Accel::Error Accel::getError() { return _error; }

bool Accel::init() {
  if (!_bus.begin()) return false;
  _bus.enableDummyByte(true);

  // Dummy read: on SPI, a rising edge on CSB switches the accelerometer to SPI mode
  _bus.readRegister(static_cast<uint8_t>(Registers::ChipID), _register);

  uint8_t id;
  if (!getChipID(id)) return false;
  if (id != ChipID) {
    _setError(Error::InvalidChipID);
    return false;
  }

  if (!softReset()) return false;
  if (!setPowerMode(Mode::Active)) return false;
  if (!powerOn()) return false;

  _raw_offset       = {};
  _test_result_data = {};
  return true;
}

void Accel::onError(OnErrorCallback cb) { _on_error = cb; }

void Accel::clearErrorCallback() { _on_error = nullptr; }

bool Accel::getChipID(uint8_t& id) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::ChipID), id)) {
    _setError(Error::NoGetChipID);
    return false;
  }

  return true;
}

bool Accel::getChipError(bool& fatal_error, bool& config_error) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::ErrorReg), _register)) {
    _setError(Error::NoGetChipError);
    return false;
  }

  fatal_error  = _register & 0x01;
  config_error = ((_register >> 2) & 0x07) != 0;
  return true;
}

bool Accel::getStatus(bool& data_ready) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::Status), _register)) {
    _setError(Error::NoGetStatus);
    return false;
  }

  data_ready = (_register & 0x80) != 0;
  return true;
}

bool Accel::getRawData(RawData& data) {
  if (!_bus.readBlock(static_cast<uint8_t>(Registers::XLsb), _values, 6)) {
    _setError(Error::NoGetRawData);
    return false;
  }

  data.x = (static_cast<int16_t>(_values[1]) << 8) | _values[0];
  data.y = (static_cast<int16_t>(_values[3]) << 8) | _values[2];
  data.z = (static_cast<int16_t>(_values[5]) << 8) | _values[4];
  return true;
}

bool Accel::getRawXData(int16_t& x) {
  if (!_bus.readBlock(static_cast<uint8_t>(Registers::XLsb), _values, 2)) {
    _setError(Error::NoGetRawXData);
    return false;
  }

  x = (static_cast<int16_t>(_values[1]) << 8) | _values[0];
  return true;
}

bool Accel::getRawYData(int16_t& y) {
  if (!_bus.readBlock(static_cast<uint8_t>(Registers::YLsb), _values, 2)) {
    _setError(Error::NoGetRawYData);
    return false;
  }

  y = (static_cast<int16_t>(_values[1]) << 8) | _values[0];
  return true;
}

bool Accel::getRawZData(int16_t& z) {
  if (!_bus.readBlock(static_cast<uint8_t>(Registers::ZLsb), _values, 2)) {
    _setError(Error::NoGetRawZData);
    return false;
  }

  z = (static_cast<int16_t>(_values[1]) << 8) | _values[0];
  return true;
}

bool Accel::getMilliGAcceleration(ConvertedData& data) {
  RawData raw_data;
  if (!getRawData(raw_data)) return false;

  convertToMilliG(raw_data, data);
  return true;
}

bool Accel::getMilliGXAcceleration(float& x) {
  int16_t raw_x;
  if (!getRawXData(raw_x)) return false;
  x = (static_cast<float>(raw_x) - _raw_offset.x) * _factor_mg;
  return true;
}

bool Accel::getMilliGYAcceleration(float& y) {
  int16_t raw_y;
  if (!getRawYData(raw_y)) return false;
  y = (static_cast<float>(raw_y) - _raw_offset.y) * _factor_mg;
  return true;
}

bool Accel::getMilliGZAcceleration(float& z) {
  int16_t raw_z;
  if (!getRawZData(raw_z)) return false;
  z = (static_cast<float>(raw_z) - _raw_offset.z) * _factor_mg;
  return true;
}

bool Accel::getSensorTime(uint32_t& time) {
  if (!_bus.readBlock(static_cast<uint8_t>(Registers::SensorTime0), _values, 3)) {
    _setError(Error::NoGetTime);
    return false;
  }

  time = (static_cast<uint32_t>(_values[2]) << 16) | (static_cast<uint32_t>(_values[1]) << 8) |
         _values[0];
  return true;
}

bool Accel::getTemperature(float& temperature) {
  if (!_bus.readBlock(static_cast<uint8_t>(Registers::TemperatureMsb), _values, 2)) {
    _setError(Error::NoGetTemperature);
    return false;
  }

  int16_t temp = (static_cast<int16_t>(_values[0]) << 3) | (_values[1] >> 5);
  if (temp > 1023) temp -= 2048;
  temperature = temp * 0.125f + 23.0f; // 0.125 °C/LSB

  return true;
}

bool Accel::clearInterrupt() {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::InterruptStatus1), _register)) {
    _setError(Error::NoClearInterrupt);
    return false;
  }

  return true;
}

bool Accel::setConfig(Bandwidth bw, DataRate dr) {
  _register = ((static_cast<uint8_t>(bw) & 0x0F) << 4) | (static_cast<uint8_t>(dr) & 0x0F);

  if (!_bus.writeRegister(static_cast<uint8_t>(Registers::Config), _register)) {
    _setError(Error::NoSetConfig);
    return false;
  }

  return true;
}

bool Accel::getConfig(Bandwidth& bw, DataRate& dr) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::Config), _register)) {
    _setError(Error::NoGetConfig);
    return false;
  }

  bw = static_cast<Bandwidth>((_register >> 4) & 0x0F);
  dr = static_cast<DataRate>(_register & 0x0F);
  return true;
}

bool Accel::setBandwidth(Bandwidth bw) {
  Bandwidth current_bw;
  DataRate current_dr;
  if (!getConfig(current_bw, current_dr)) return false;
  return setConfig(bw, current_dr);
}

bool Accel::getBandwidth(Bandwidth& bw) {
  DataRate current_dr;
  return getConfig(bw, current_dr);
}

bool Accel::setDataRate(DataRate dr) {
  Bandwidth current_bw;
  DataRate current_dr;
  if (!getConfig(current_bw, current_dr)) return false;
  return setConfig(current_bw, dr);
}

bool Accel::getDataRate(DataRate& dr) {
  Bandwidth current_bw;
  return getConfig(current_bw, dr);
}

bool Accel::setRange(Range range) {
  if (!_bus.writeRegister(static_cast<uint8_t>(Registers::Range),
        static_cast<uint8_t>(range) & 0x03)) {
    _setError(Error::NoSetRange);
    return false;
  }

  _range = static_cast<uint8_t>(range) & 0x03;
  _calculateFactorMilliG();

  return true;
}

bool Accel::getRange(Range& range) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::Range), _register)) {
    _setError(Error::NoGetRange);
    return false;
  }

  _range = _register & 0x03;
  range  = static_cast<Range>(_range);
  _calculateFactorMilliG();
  return true;
}

bool Accel::pinConfig(GpioPin pin, GpioMode mode, GpioBehavior behavior) {
  const uint8_t reg = static_cast<uint8_t>(Registers::Interrupt1IOCtrl) + static_cast<uint8_t>(pin);
  const Error error =
    static_cast<Error>(static_cast<uint8_t>(Error::NoSetInt1PinConfig) + static_cast<uint8_t>(pin));

  if (!_bus.readRegister(reg, _register)) {
    _setError(error);
    return false;
  }

  _register &= 0x02; // Keep the active level
  _register |=
    ((static_cast<uint8_t>(mode) & 0x03) << 3) | ((static_cast<uint8_t>(behavior) & 0x01) << 2);

  if (!_bus.writeRegister(reg, _register)) {
    _setError(error);
    return false;
  }

  return true;
}

bool Accel::getPinConfig(GpioPin pin, GpioMode& mode, GpioBehavior& behavior) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::Interrupt1IOCtrl) +
                           static_cast<uint8_t>(pin),
        _register)) {
    _setError(static_cast<Error>(
      static_cast<uint8_t>(Error::NoGetInt1PinConfig) + static_cast<uint8_t>(pin)));
    return false;
  }

  mode     = static_cast<GpioMode>((_register >> 3) & 0x03);
  behavior = static_cast<GpioBehavior>((_register >> 2) & 0x01);
  return true;
}

bool Accel::setPinActiveLevel(GpioPin pin, GpioLevel level) {
  const uint8_t reg = static_cast<uint8_t>(Registers::Interrupt1IOCtrl) + static_cast<uint8_t>(pin);
  const Error error =
    static_cast<Error>(static_cast<uint8_t>(Error::NoSetInt1PinLevel) + static_cast<uint8_t>(pin));

  if (!_bus.readRegister(reg, _register)) {
    _setError(error);
    return false;
  }

  _register &= ~0x02;
  _register |= (static_cast<uint8_t>(level) & 0x01) << 1;

  if (!_bus.writeRegister(reg, _register)) {
    _setError(error);
    return false;
  }

  return true;
}

bool Accel::getPinActiveLevel(GpioPin pin, GpioLevel& level) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::Interrupt1IOCtrl) +
                           static_cast<uint8_t>(pin),
        _register)) {
    _setError(static_cast<Error>(
      static_cast<uint8_t>(Error::NoGetInt1PinLevel) + static_cast<uint8_t>(pin)));
    return false;
  }

  level = static_cast<GpioLevel>((_register >> 1) & 0x01);
  return true;
}

bool Accel::setMapInterrupt(bool int1, bool int2) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::InterruptMapData), _register)) {
    _setError(Error::NoSetMapInterrupt);
    return false;
  }

  _register &= ~0x44; // Keep the FIFO interrupts
  _register |= (int2 << 6) | (int1 << 2);

  if (!_bus.writeRegister(static_cast<uint8_t>(Registers::InterruptMapData), _register)) {
    _setError(Error::NoSetMapInterrupt);
    return false;
  }

  return true;
}

bool Accel::getMapInterrupt(bool& int1, bool& int2) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::InterruptMapData), _register)) {
    _setError(Error::NoGetMapInterrupt);
    return false;
  }

  int1 = (_register >> 2) & 0x01;
  int2 = (_register >> 6) & 0x01;
  return true;
}

bool Accel::setSelfTestMode(SelfTestMode mode) {
  if (!_bus.writeRegister(static_cast<uint8_t>(Registers::SelfTest), static_cast<uint8_t>(mode))) {
    _setError(Error::NoSetSelfTestMode);
    return false;
  }

  delay(50);
  return true;
}

bool Accel::selfTest(bool& x_pass, bool& y_pass, bool& z_pass) {
  Config config;
  RawData positive_data;
  RawData negative_data;

  // Save config
  if (!getAllConfig(config)) return false;

  // Config sensor for self test: active, range 24g, ODR 1600 Hz and normal bandwidth
  if (!setPowerMode(Mode::Active)) return false;
  if (!powerOn()) return false;
  if (!setRange(Range::Range24g)) return false;
  if (!setConfig(Bandwidth::Normal, DataRate::Odr1600Hz)) return false;
  delay(2);

  // Obtain positive and negative data (setSelfTestMode waits 50 ms)
  if (!setSelfTestMode(SelfTestMode::Positive)) return false;
  if (!getRawData(positive_data)) return false;
  if (!setSelfTestMode(SelfTestMode::Negative)) return false;
  if (!getRawData(negative_data)) return false;
  if (!setSelfTestMode(SelfTestMode::Off)) return false;

  _test_result_data.x = (static_cast<int32_t>(positive_data.x) - negative_data.x) * _factor_mg;
  _test_result_data.y = (static_cast<int32_t>(positive_data.y) - negative_data.y) * _factor_mg;
  _test_result_data.z = (static_cast<int32_t>(positive_data.z) - negative_data.z) * _factor_mg;

  x_pass = _test_result_data.x >= 1000.0f;
  y_pass = _test_result_data.y >= 1000.0f;
  z_pass = _test_result_data.z >= 500.0f;

  // The datasheet recommends a soft reset after the self test
  if (!softReset()) return false;
  if (!setPowerMode(Mode::Active)) return false;
  if (!powerOn()) return false;

  // Restore config
  return setAllConfig(config);
}

void Accel::getSelfTestData(ConvertedData& data) { data = _test_result_data; }

bool Accel::getSelfTestMode(SelfTestMode& mode) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::SelfTest), _register)) {
    _setError(Error::NoGetSelfTestMode);
    return false;
  }

  mode = static_cast<SelfTestMode>(_register);
  return true;
}

bool Accel::setPowerMode(Mode mode) {
  if (!_bus.writeRegister(static_cast<uint8_t>(Registers::PowerConfig),
        static_cast<uint8_t>(mode))) {
    _setError(Error::NoSetPowerMode);
    return false;
  }

  delay(5);
  return true;
}

bool Accel::getPowerMode(Mode& mode) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::PowerConfig), _register)) {
    _setError(Error::NoGetPowerMode);
    return false;
  }

  mode = static_cast<Mode>(_register);
  return true;
}

bool Accel::powerOn() {
  if (!_bus.writeRegister(static_cast<uint8_t>(Registers::PowerControl), 0x04)) {
    _setError(Error::NoSetPowerOn);
    return false;
  }

  delay(50);
  return true;
}

bool Accel::powerOff() {
  if (!_bus.writeRegister(static_cast<uint8_t>(Registers::PowerControl), 0x00)) {
    _setError(Error::NoSetPowerOff);
    return false;
  }

  delay(50);
  return true;
}

bool Accel::softReset() {
  if (!_bus.writeRegister(static_cast<uint8_t>(Registers::SoftReset), 0xB6)) {
    _setError(Error::NoSoftReset);
    return false;
  }

  delay(1);

  // Dummy read: on SPI, a rising edge on CSB switches the accelerometer back to SPI mode
  _bus.readRegister(static_cast<uint8_t>(Registers::ChipID), _register);

  // After the reset the range is 6g
  _range = static_cast<uint8_t>(Range::Range6g);
  _calculateFactorMilliG();
  return true;
}

void Accel::setRawOffset(const RawData& offset) { _raw_offset = offset; }

bool Accel::getAllConfig(Config& config) {
  if (!getChipID(config.id)) return false;
  if (!getRange(config.range)) return false;
  if (!getConfig(config.bw, config.dr)) return false;
  if (!getPowerMode(config.mode)) return false;
  if (!getPinConfig(GpioPin::Int1, config.int1.mode, config.int1.behavior)) return false;
  if (!getPinConfig(GpioPin::Int2, config.int2.mode, config.int2.behavior)) return false;
  if (!getPinActiveLevel(GpioPin::Int1, config.int1.level)) return false;
  if (!getPinActiveLevel(GpioPin::Int2, config.int2.level)) return false;
  if (!getMapInterrupt(config.int1.interrupt, config.int2.interrupt)) return false;

  return true;
}

bool Accel::setAllConfig(const Config& config) {
  if (!setRange(config.range)) return false;
  if (!setConfig(config.bw, config.dr)) return false;
  if (!pinConfig(GpioPin::Int1, config.int1.mode, config.int1.behavior)) return false;
  if (!pinConfig(GpioPin::Int2, config.int2.mode, config.int2.behavior)) return false;
  if (!setPinActiveLevel(GpioPin::Int1, config.int1.level)) return false;
  if (!setPinActiveLevel(GpioPin::Int2, config.int2.level)) return false;
  if (!setMapInterrupt(config.int1.interrupt, config.int2.interrupt)) return false;
  if (!setPowerMode(config.mode)) return false;

  return true;
}

bool Accel::isConnected() {
  uint8_t id;
  return getChipID(id) && id == ChipID;
}

bool Accel::calibrateOffset(uint16_t samples) {
  if (samples == 0) return false;

  DataRate dr;
  Range range;
  if (!getDataRate(dr)) return false;
  if (!getRange(range)) return false;

  // ODR = 1600 Hz / 2^(0x0C - dr), so the period is 625 us * 2^(0x0C - dr)
  const uint8_t code     = static_cast<uint8_t>(dr);
  const uint8_t steps    = (code >= 0x05 && code <= 0x0C) ? 0x0C - code : 7;
  const uint32_t wait_ms = ((625UL << steps) + 999) / 1000;

  int32_t sum_x = 0;
  int32_t sum_y = 0;
  int32_t sum_z = 0;
  RawData data;
  for (uint16_t i = 0; i < samples; ++i) {
    if (!getRawData(data)) return false;
    sum_x += data.x;
    sum_y += data.y;
    sum_z += data.z;
    delay(wait_ms);
  }

  // The Z axis keeps 1 g
  _raw_offset.x = sum_x / samples;
  _raw_offset.y = sum_y / samples;
  _raw_offset.z = sum_z / samples - static_cast<int16_t>(1000.0f / _factor_mg + 0.5f);
  return true;
}

void Accel::getRawOffset(RawData& offset) { offset = _raw_offset; }

void Accel::convertToMilliG(const RawData& raw, ConvertedData& data) {
  data.x = (static_cast<float>(raw.x) - _raw_offset.x) * _factor_mg;
  data.y = (static_cast<float>(raw.y) - _raw_offset.y) * _factor_mg;
  data.z = (static_cast<float>(raw.z) - _raw_offset.z) * _factor_mg;
}

bool Accel::getAcceleration(ConvertedData& data) {
  if (!getMilliGAcceleration(data)) return false;

  data.x *= StandardGravity / 1000.0f;
  data.y *= StandardGravity / 1000.0f;
  data.z *= StandardGravity / 1000.0f;
  return true;
}

bool Accel::getRawDataAndTime(RawData& data, uint32_t& time) {
  // Data and sensor time are consecutive registers (0x12 to 0x1A)
  if (!_bus.readBlock(static_cast<uint8_t>(Registers::XLsb), _values, 9)) {
    _setError(Error::NoGetRawData);
    return false;
  }

  data = decodeRawData(_values);
  time = (static_cast<uint32_t>(_values[8]) << 16) | (static_cast<uint32_t>(_values[7]) << 8) |
         _values[6];
  return true;
}

bool Accel::getInterruptStatus(bool& data_ready) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::InterruptStatus1), _register)) {
    _setError(Error::NoGetInterruptStatus);
    return false;
  }

  data_ready = (_register & 0x80) != 0;
  return true;
}

bool Accel::setFifoMode(FifoMode mode) {
  // Bit 1 must always be 1
  if (!_bus.writeRegister(static_cast<uint8_t>(Registers::FifoConfig0),
        0x02 | (static_cast<uint8_t>(mode) & 0x01))) {
    _setError(Error::NoSetFifoMode);
    return false;
  }

  return true;
}

bool Accel::getFifoMode(FifoMode& mode) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::FifoConfig0), _register)) {
    _setError(Error::NoGetFifoMode);
    return false;
  }

  mode = static_cast<FifoMode>(_register & 0x01);
  return true;
}

bool Accel::enableFifo(bool accel, bool int1_input, bool int2_input) {
  // Bit 4 must always be 1
  _register = 0x10 | (accel << 6) | (int1_input << 3) | (int2_input << 2);

  if (!_bus.writeRegister(static_cast<uint8_t>(Registers::FifoConfig1), _register)) {
    _setError(Error::NoSetFifoConfig);
    return false;
  }

  return true;
}

bool Accel::setFifoWatermark(uint16_t bytes) {
  if (!_bus.writeRegister(static_cast<uint8_t>(Registers::FifoWatermark0), bytes & 0xFF) ||
      !_bus.writeRegister(static_cast<uint8_t>(Registers::FifoWatermark1), (bytes >> 8) & 0x1F)) {
    _setError(Error::NoSetFifoWatermark);
    return false;
  }

  return true;
}

bool Accel::getFifoWatermark(uint16_t& bytes) {
  if (!_bus.readBlock(static_cast<uint8_t>(Registers::FifoWatermark0), _values, 2)) {
    _setError(Error::NoGetFifoWatermark);
    return false;
  }

  bytes = (static_cast<uint16_t>(_values[1] & 0x1F) << 8) | _values[0];
  return true;
}

bool Accel::setFifoDownsampling(uint8_t exponent) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::FifoDownsampling), _register)) {
    _setError(Error::NoSetFifoDownsampling);
    return false;
  }

  _register &= ~0x70;
  _register |= (exponent & 0x07) << 4;

  if (!_bus.writeRegister(static_cast<uint8_t>(Registers::FifoDownsampling), _register)) {
    _setError(Error::NoSetFifoDownsampling);
    return false;
  }

  return true;
}

bool Accel::getFifoLength(uint16_t& bytes) {
  if (!_bus.readBlock(static_cast<uint8_t>(Registers::FifoLength0), _values, 2)) {
    _setError(Error::NoGetFifoLength);
    return false;
  }

  bytes = (static_cast<uint16_t>(_values[1] & 0x3F) << 8) | _values[0];
  return true;
}

bool Accel::readFifoData(uint8_t* buffer, size_t length) {
  if (!_bus.readBlock(static_cast<uint8_t>(Registers::FifoData), buffer, length)) {
    _setError(Error::NoGetFifoData);
    return false;
  }

  return true;
}

bool Accel::readFifo(RawData* frames, size_t max_frames, size_t& frames_read) {
  frames_read = 0;

  uint16_t length;
  if (!getFifoLength(length)) return false;

  uint8_t buffer[AccelFifoChunk];
  size_t remaining = length;

  while (remaining > 0 && frames_read < max_frames) {
    size_t chunk      = remaining < AccelFifoChunk ? remaining : AccelFifoChunk;
    const size_t room = (max_frames - frames_read) * AccelFifoFrameSize;
    if (chunk > room) chunk = room;
    // When the FIFO is read in parts, every burst ends at a frame boundary
    if (chunk < remaining) chunk = chunk / AccelFifoFrameSize * AccelFifoFrameSize;
    if (chunk == 0) break;

    if (!readFifoData(buffer, chunk)) return false;
    frames_read += parseFifo(buffer, chunk, frames + frames_read, max_frames - frames_read);
    remaining -= chunk;
  }

  return true;
}

size_t Accel::parseFifo(const uint8_t* buffer, size_t length, RawData* frames, size_t max_frames) {
  size_t count = 0;
  size_t pos   = 0;

  while (pos < length && count < max_frames) {
    const uint8_t header = buffer[pos];
    const size_t payload = accelFifoPayload(header);
    if (payload == 0 || pos + 1 + payload > length) break;

    if ((header & 0xFC) == 0x84) frames[count++] = decodeRawData(buffer + pos + 1);
    pos += 1 + payload;
  }

  return count;
}

bool Accel::flushFifo() {
  uint16_t length;
  if (!getFifoLength(length)) return false;

  uint8_t buffer[AccelFifoChunk];
  while (length > 0) {
    const size_t chunk = length < AccelFifoChunk ? length : AccelFifoChunk;
    if (!readFifoData(buffer, chunk)) return false;
    length -= chunk;
  }

  return true;
}

bool Accel::setMapFifoInterrupt(GpioPin pin, bool watermark, bool full) {
  // INT1: bit 0 full, bit 1 watermark. INT2: bit 4 full, bit 5 watermark
  const uint8_t shift = 4 * static_cast<uint8_t>(pin);

  if (!_bus.readRegister(static_cast<uint8_t>(Registers::InterruptMapData), _register)) {
    _setError(Error::NoSetMapFifoInterrupt);
    return false;
  }

  _register &= ~(0x03 << shift);
  _register |= (full | (watermark << 1)) << shift;

  if (!_bus.writeRegister(static_cast<uint8_t>(Registers::InterruptMapData), _register)) {
    _setError(Error::NoSetMapFifoInterrupt);
    return false;
  }

  return true;
}

void Accel::_calculateFactorMilliG() {
  _factor_mg = 1000.0f * (1 << (_range + 1)) * 1.5f / 32768.0f;
}

void Accel::_setError(Error error) {
  _error = error;
  if (_on_error) _on_error(error);
}

/* ---------------------------------------------------------------------------------------------- */
// MARK: Gyro
/* ---------------------------------------------------------------------------------------------- */
/*                                      PIN BMI088 Gyroscope                                      */
/* ---------------------------------------------------------------------------------------------- */
Gyro::Gyro(Bus& bus)
    : _bus(bus)
    , _register(0)
    , _values{}
    , _range(static_cast<uint8_t>(Range::Dps2000))
    , _factor_angular_velocity(0)
    , _raw_offset{}
    , _on_error(nullptr)
    , _error(Error::None) {
  _calculateFactorAngularVelocity();
}

float Gyro::getRelativeEulerAngle(float last_angle, float angle_velocity, float dt) {
  float angle = fmodf(last_angle + angle_velocity * dt, 360.0f);
  if (angle < 0) angle += 360.0f;
  return angle;
}

Gyro::Error Gyro::getError() { return _error; }

bool Gyro::getAllConfig(Config& config) {
  if (!getChipID(config.id)) return false;
  if (!getRange(config.range)) return false;
  if (!getDataRateAndBandwidth(config.odr_and_bw)) return false;
  if (!getPowerMode(config.mode)) return false;
  if (!getPinsConfig(config.int3, config.int4)) return false;
  if (!getMapInterrupt(config.map_interrupt)) return false;

  return true;
}

bool Gyro::init() {
  if (!_bus.begin()) return false;

  uint8_t id;
  if (!getChipID(id)) return false;
  if (id != ChipID) {
    _setError(Error::InvalidChipID);
    return false;
  }

  _raw_offset = {};
  return softReset();
}

void Gyro::onError(OnErrorCallback cb) { _on_error = cb; }

void Gyro::clearErrorCallback() { _on_error = nullptr; }

bool Gyro::getChipID(uint8_t& id) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::ChipID), id)) {
    _setError(Error::NoGetChipID);
    return false;
  }
  return true;
}

bool Gyro::getRawData(RawData& data) {
  if (!_bus.readBlock(static_cast<uint8_t>(Registers::RateXLsb), _values, 6)) {
    _setError(Error::NoGetRawData);
    return false;
  }

  data.x = (static_cast<int16_t>(_values[1]) << 8) | _values[0];
  data.y = (static_cast<int16_t>(_values[3]) << 8) | _values[2];
  data.z = (static_cast<int16_t>(_values[5]) << 8) | _values[4];
  return true;
}

bool Gyro::getRawXData(int16_t& x) {
  if (!_bus.readBlock(static_cast<uint8_t>(Registers::RateXLsb), _values, 2)) {
    _setError(Error::NoGetRawXData);
    return false;
  }

  x = (static_cast<int16_t>(_values[1]) << 8) | _values[0];
  return true;
}

bool Gyro::getRawYData(int16_t& y) {
  if (!_bus.readBlock(static_cast<uint8_t>(Registers::RateYLsb), _values, 2)) {
    _setError(Error::NoGetRawYData);
    return false;
  }

  y = (static_cast<int16_t>(_values[1]) << 8) | _values[0];
  return true;
}

bool Gyro::getRawZData(int16_t& z) {
  if (!_bus.readBlock(static_cast<uint8_t>(Registers::RateZLsb), _values, 2)) {
    _setError(Error::NoGetRawZData);
    return false;
  }

  z = (static_cast<int16_t>(_values[1]) << 8) | _values[0];
  return true;
}

bool Gyro::getAngularVelocity(ConvertedData& data) {
  RawData raw_data;
  if (!getRawData(raw_data)) return false;

  convertToDps(raw_data, data);
  return true;
}

bool Gyro::getAngularVelocityX(float& x) {
  int16_t raw_x;
  if (!getRawXData(raw_x)) return false;
  x = (static_cast<float>(raw_x) - _raw_offset.x) * _factor_angular_velocity;
  return true;
}

bool Gyro::getAngularVelocityY(float& y) {
  int16_t raw_y;
  if (!getRawYData(raw_y)) return false;
  y = (static_cast<float>(raw_y) - _raw_offset.y) * _factor_angular_velocity;
  return true;
}

bool Gyro::getAngularVelocityZ(float& z) {
  int16_t raw_z;
  if (!getRawZData(raw_z)) return false;
  z = (static_cast<float>(raw_z) - _raw_offset.z) * _factor_angular_velocity;
  return true;
}

bool Gyro::setRange(Range range) {
  if (!_bus.writeRegister(static_cast<uint8_t>(Registers::Range), static_cast<uint8_t>(range))) {
    _setError(Error::NoSetRange);
    return false;
  }
  _range = static_cast<uint8_t>(range);
  _calculateFactorAngularVelocity();
  return true;
}

bool Gyro::getRange(Range& range) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::Range), _register)) {
    _setError(Error::NoGetRange);
    return false;
  }
  _range = _register & 0x07;
  range  = static_cast<Range>(_range);
  _calculateFactorAngularVelocity();
  return true;
}

bool Gyro::setDataRateAndBandwidth(OdrAndBandwidth bw) {
  if (!_bus.writeRegister(static_cast<uint8_t>(Registers::Bandwidth), static_cast<uint8_t>(bw))) {
    _setError(Error::NoSetOdrAndBandwidth);
    return false;
  }
  return true;
}

bool Gyro::getDataRateAndBandwidth(OdrAndBandwidth& bw) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::Bandwidth), _register)) {
    _setError(Error::NoGetOdrAndBandwidth);
    return false;
  }
  // Bit 7 is read only and always 1
  bw = static_cast<OdrAndBandwidth>(_register & 0x0F);
  return true;
}

bool Gyro::setPowerMode(Mode mode) {
  if (!_bus.writeRegister(static_cast<uint8_t>(Registers::Lpm1), static_cast<uint8_t>(mode))) {
    _setError(Error::NoSetPowerMode);
    return false;
  }
  return true;
}

bool Gyro::getPowerMode(Mode& mode) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::Lpm1), _register)) {
    _setError(Error::NoGetPowerMode);
    return false;
  }
  mode = static_cast<Mode>(_register & 0xA0);
  return true;
}

bool Gyro::softReset() {
  if (!_bus.writeRegister(static_cast<uint8_t>(Registers::Softreset), 0xB6)) {
    _setError(Error::NoSoftReset);
    return false;
  }

  delay(30);

  // After the reset the range is 2000 dps
  _range = static_cast<uint8_t>(Range::Dps2000);
  _calculateFactorAngularVelocity();
  return true;
}

bool Gyro::enableInterrupt(bool enable) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::InterruptControl), _register)) {
    _setError(Error::NoSetInterruptControl);
    return false;
  }

  _register &= ~0x80; // Keep the FIFO interrupt
  _register |= enable << 7;

  if (!_bus.writeRegister(static_cast<uint8_t>(Registers::InterruptControl), _register)) {
    _setError(Error::NoSetInterruptControl);
    return false;
  }
  return true;
}

bool Gyro::isEnabledInterrupt(bool& enable) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::InterruptControl), _register)) {
    _setError(Error::NoGetInterruptControl);
    return false;
  }
  enable = (_register & 0x80) != 0;
  return true;
}

bool Gyro::getPinsConfig(GpioOutConfig& conf_int3, GpioOutConfig& conf_int4) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::IntIOConfig), _register)) {
    _setError(Error::NoGetPinsConfig);
    return false;
  }
  conf_int3 = static_cast<GpioOutConfig>((_register >> 1) & 0x01);
  conf_int4 = static_cast<GpioOutConfig>((_register >> 3) & 0x01);
  return true;
}

bool Gyro::setPinsConfig(GpioOutConfig conf_int3, GpioOutConfig conf_int4) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::IntIOConfig), _register)) {
    _setError(Error::NoSetPinsConfig);
    return false;
  }

  _register &= 0x05; // Keep the active levels
  _register |= (static_cast<uint8_t>(conf_int3) & 0x01) << 1;
  _register |= (static_cast<uint8_t>(conf_int4) & 0x01) << 3;

  if (!_bus.writeRegister(static_cast<uint8_t>(Registers::IntIOConfig), _register)) {
    _setError(Error::NoSetPinsConfig);
    return false;
  }

  return true;
}

// INT3_INT4_IO_CONF: bit 0 INT3 level, bit 1 INT3 open drain, bit 2 INT4 level, bit 3 INT4 open
// drain. The level bit of each pin is at 2 * pin and the open drain bit at 2 * pin + 1.

bool Gyro::pinConfig(GpioPin pin, GpioOutConfig conf) {
  const uint8_t shift = 2 * static_cast<uint8_t>(pin) + 1;
  const Error error =
    static_cast<Error>(static_cast<uint8_t>(Error::NoSetInt3Config) + static_cast<uint8_t>(pin));

  if (!_bus.readRegister(static_cast<uint8_t>(Registers::IntIOConfig), _register)) {
    _setError(error);
    return false;
  }

  _register &= ~(1 << shift);
  _register |= (static_cast<uint8_t>(conf) & 0x01) << shift;

  if (!_bus.writeRegister(static_cast<uint8_t>(Registers::IntIOConfig), _register)) {
    _setError(error);
    return false;
  }

  return true;
}

bool Gyro::getPinConfig(GpioPin pin, GpioOutConfig& conf) {
  const uint8_t shift = 2 * static_cast<uint8_t>(pin) + 1;

  if (!_bus.readRegister(static_cast<uint8_t>(Registers::IntIOConfig), _register)) {
    _setError(
      static_cast<Error>(static_cast<uint8_t>(Error::NoGetInt3Config) + static_cast<uint8_t>(pin)));
    return false;
  }

  conf = static_cast<GpioOutConfig>((_register >> shift) & 0x01);
  return true;
}

bool Gyro::setPinActiveLevel(GpioPin pin, GpioLevel level) {
  const uint8_t shift = 2 * static_cast<uint8_t>(pin);
  const Error error =
    static_cast<Error>(static_cast<uint8_t>(Error::NoSetInt3Level) + static_cast<uint8_t>(pin));

  if (!_bus.readRegister(static_cast<uint8_t>(Registers::IntIOConfig), _register)) {
    _setError(error);
    return false;
  }

  _register &= ~(1 << shift);
  _register |= (static_cast<uint8_t>(level) & 0x01) << shift;

  if (!_bus.writeRegister(static_cast<uint8_t>(Registers::IntIOConfig), _register)) {
    _setError(error);
    return false;
  }

  return true;
}

bool Gyro::getPinActiveLevel(GpioPin pin, GpioLevel& level) {
  const uint8_t shift = 2 * static_cast<uint8_t>(pin);

  if (!_bus.readRegister(static_cast<uint8_t>(Registers::IntIOConfig), _register)) {
    _setError(
      static_cast<Error>(static_cast<uint8_t>(Error::NoGetInt3Level) + static_cast<uint8_t>(pin)));
    return false;
  }

  level = static_cast<GpioLevel>((_register >> shift) & 0x01);
  return true;
}

bool Gyro::setMapInterrupt(GpioMapInterrupt map_int) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::IntIOMap), _register)) {
    _setError(Error::NoSetMapInterrupt);
    return false;
  }

  _register &= ~0x81; // Keep the FIFO interrupts
  _register |= static_cast<uint8_t>(map_int);

  if (!_bus.writeRegister(static_cast<uint8_t>(Registers::IntIOMap), _register)) {
    _setError(Error::NoSetMapInterrupt);
    return false;
  }

  return true;
}

bool Gyro::getMapInterrupt(GpioMapInterrupt& map_int) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::IntIOMap), _register)) {
    _setError(Error::NoGetMapInterrupt);
    return false;
  }
  map_int = static_cast<GpioMapInterrupt>(_register & 0x81);
  return true;
}

bool Gyro::startSelfTest() {
  if (!_bus.writeRegister(static_cast<uint8_t>(Registers::SelfTest), 0x01)) {
    _setError(Error::NoStartSelfTest);
    return false;
  }
  return true;
}

bool Gyro::getSelfTestResult(bool& ready, bool& pass) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::SelfTest), _register)) {
    _setError(Error::NoGetSelfTestResult);
    return false;
  }
  ready = (_register & 0x02) != 0;
  pass  = ((_register & 0x04) == 0) && ready;
  return true;
}

bool Gyro::selfTest(bool& pass) {
  if (!startSelfTest()) return false;

  bool ready = false;
  for (uint8_t i = 0; i < 10; ++i) {
    delay(10);
    if (!getSelfTestResult(ready, pass)) return false;
    if (ready) return true;
  }

  _setError(Error::SelfTestTimeout);
  return false;
}

bool Gyro::isConnected() {
  uint8_t id;
  return getChipID(id) && id == ChipID;
}

bool Gyro::getStatus(bool& data_ready) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::InterruptStatus1), _register)) {
    _setError(Error::NoGetStatus);
    return false;
  }

  data_ready = (_register & 0x80) != 0;
  return true;
}

void Gyro::setRawOffset(const RawData& offset) { _raw_offset = offset; }

void Gyro::getRawOffset(RawData& offset) { offset = _raw_offset; }

bool Gyro::calibrateOffset(uint16_t samples) {
  if (samples == 0) return false;

  // Sample period in ms for each OdrAndBandwidth value: 2000, 2000, 1000, 400, 200, 100, 200, 100
  // Hz
  static const uint8_t periods_ms[] = {1, 1, 1, 3, 5, 10, 5, 10};

  OdrAndBandwidth bw;
  if (!getDataRateAndBandwidth(bw)) return false;
  const uint8_t index   = static_cast<uint8_t>(bw);
  const uint8_t wait_ms = index < sizeof(periods_ms) ? periods_ms[index] : 10;

  int32_t sum_x = 0;
  int32_t sum_y = 0;
  int32_t sum_z = 0;
  RawData data;
  for (uint16_t i = 0; i < samples; ++i) {
    if (!getRawData(data)) return false;
    sum_x += data.x;
    sum_y += data.y;
    sum_z += data.z;
    delay(wait_ms);
  }

  _raw_offset.x = sum_x / samples;
  _raw_offset.y = sum_y / samples;
  _raw_offset.z = sum_z / samples;
  return true;
}

void Gyro::convertToDps(const RawData& raw, ConvertedData& data) {
  data.x = (static_cast<float>(raw.x) - _raw_offset.x) * _factor_angular_velocity;
  data.y = (static_cast<float>(raw.y) - _raw_offset.y) * _factor_angular_velocity;
  data.z = (static_cast<float>(raw.z) - _raw_offset.z) * _factor_angular_velocity;
}

bool Gyro::getAngularVelocityRad(ConvertedData& data) {
  if (!getAngularVelocity(data)) return false;

  data.x *= DegToRad;
  data.y *= DegToRad;
  data.z *= DegToRad;
  return true;
}

bool Gyro::setFifoMode(FifoMode mode) {
  // Bits 1:0 select the stored data and must be 0 (X, Y and Z)
  if (!_bus.writeRegister(static_cast<uint8_t>(Registers::FifoConfig1),
        static_cast<uint8_t>(mode))) {
    _setError(Error::NoSetFifoMode);
    return false;
  }

  return true;
}

bool Gyro::getFifoMode(FifoMode& mode) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::FifoConfig1), _register)) {
    _setError(Error::NoGetFifoMode);
    return false;
  }

  mode = static_cast<FifoMode>(_register & 0xC0);
  return true;
}

bool Gyro::setFifoWatermark(uint8_t frames) {
  if (!_bus.writeRegister(static_cast<uint8_t>(Registers::FifoConfig0), frames & 0x7F)) {
    _setError(Error::NoSetFifoWatermark);
    return false;
  }

  return true;
}

bool Gyro::getFifoWatermark(uint8_t& frames) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::FifoConfig0), _register)) {
    _setError(Error::NoGetFifoWatermark);
    return false;
  }

  frames = _register & 0x7F;
  return true;
}

bool Gyro::enableFifoInterrupt(bool enable) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::InterruptControl), _register)) {
    _setError(Error::NoSetFifoInterrupt);
    return false;
  }

  _register &= ~0x40; // Keep the data ready interrupt
  _register |= enable << 6;

  if (!_bus.writeRegister(static_cast<uint8_t>(Registers::InterruptControl), _register) ||
      !_bus.writeRegister(static_cast<uint8_t>(Registers::FifoWmEnable), enable ? 0x88 : 0x08)) {
    _setError(Error::NoSetFifoInterrupt);
    return false;
  }

  return true;
}

bool Gyro::setMapFifoInterrupt(bool int3, bool int4) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::IntIOMap), _register)) {
    _setError(Error::NoSetMapFifoInterrupt);
    return false;
  }

  _register &= ~0x24; // Keep the data ready interrupts
  _register |= (int3 << 2) | (int4 << 5);

  if (!_bus.writeRegister(static_cast<uint8_t>(Registers::IntIOMap), _register)) {
    _setError(Error::NoSetMapFifoInterrupt);
    return false;
  }

  return true;
}

bool Gyro::getFifoStatus(uint8_t& frames, bool& overrun) {
  if (!_bus.readRegister(static_cast<uint8_t>(Registers::FifoStatus), _register)) {
    _setError(Error::NoGetFifoStatus);
    return false;
  }

  frames  = _register & 0x7F;
  overrun = (_register & 0x80) != 0;
  return true;
}

bool Gyro::readFifo(RawData* frames, size_t max_frames, size_t& frames_read) {
  frames_read = 0;

  uint8_t count;
  bool overrun;
  if (!getFifoStatus(count, overrun)) return false;

  const size_t total = count < max_frames ? count : max_frames;
  uint8_t buffer[GyroFifoChunk];

  while (frames_read < total) {
    size_t n = total - frames_read;
    if (n > GyroFifoChunk / GyroFifoFrameSize) n = GyroFifoChunk / GyroFifoFrameSize;

    if (!_bus.readBlock(static_cast<uint8_t>(Registers::FifoData), buffer, n * GyroFifoFrameSize)) {
      _setError(Error::NoGetFifoData);
      return false;
    }

    for (size_t i = 0; i < n; ++i)
      frames[frames_read + i] = decodeRawData(buffer + i * GyroFifoFrameSize);
    frames_read += n;
  }

  return true;
}

bool Gyro::flushFifo() {
  FifoMode mode;
  if (!getFifoMode(mode)) return false;
  return setFifoMode(mode);
}

void Gyro::_calculateFactorAngularVelocity() {
  // Full scale is 2000 dps for range 0 and halves on each step: 2000, 1000, 500, 250, 125 dps
  _factor_angular_velocity = 2000.0f / (1 << _range) / 32768.0f;
}

void Gyro::_setError(Error error) {
  _error = error;
  if (_on_error) _on_error(error);
}

// MARK: Imu
/* ---------------------------------------------------------------------------------------------- */
/*                                           PIN BMI088                                           */
/* ---------------------------------------------------------------------------------------------- */

Imu::Imu(Bus& bus_accel, Bus& bus_gyro)
    : _bus_accel(bus_accel)
    , _bus_gyro(bus_gyro)
    , _accel(bus_accel)
    , _gyro(bus_gyro) {}

bool Imu::init() {
  // Prepare both buses first, so with a shared SPI bus the gyroscope CS is high while the
  // accelerometer is initialized
  _bus_accel.begin();
  _bus_gyro.begin();

  bool state_accel = _accel.init();
  bool state_gyro  = _gyro.init();

  return state_accel && state_gyro;
}

Accel& Imu::accelerometer() { return _accel; }

Gyro& Imu::gyroscope() { return _gyro; }

float Imu::complementaryFilter(float last_angle, float accel_angle, float gyro_rate, float dt,
  float alpha) {
  return alpha * (last_angle + gyro_rate * dt) + (1 - alpha) * accel_angle;
}

} // namespace bmi088
