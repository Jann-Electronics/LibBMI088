#include "bmi088Bus.h"

namespace bmi088 {

BusI2C::BusI2C(TwoWire& wire, uint8_t address)
    : _wire(wire)
    , _address(address)
    , _error(0) {}

bool BusI2C::writeRegister(uint8_t reg, uint8_t value) {
  _wire.beginTransmission(_address);
  _wire.write(reg);
  _wire.write(value);
  _error = _wire.endTransmission();
  return (_error == 0);
}

bool BusI2C::readRegister(uint8_t reg, uint8_t& value) { return readBlock(reg, &value, 1); }

bool BusI2C::readBlock(uint8_t reg, uint8_t* buffer, size_t length) {
  if (length == 0 || length > 255) {
    _error = static_cast<uint8_t>(Error::InvalidSize);
    return false;
  }

  _wire.beginTransmission(_address);
  _wire.write(reg);
  _error = _wire.endTransmission(false);
  if (_error != 0) return false;

  size_t read = _wire.requestFrom(_address, static_cast<uint8_t>(length));

  if (read != length) {
    _error = static_cast<uint8_t>(Error::InvalidSize);
    return false;
  }

  for (size_t i = 0; i < length; ++i)
    buffer[i] = _wire.read();
  return true;
}

BusI2C::Error BusI2C::getError() { return static_cast<Error>(_error); }

BusSPI::BusSPI(SPIClass& spi, int8_t pin_cs, uint32_t clock)
    : _spi(spi)
    , _settings(clock, MSBFIRST, SPI_MODE0)
    , _pin_cs(pin_cs) {}

bool BusSPI::begin() {
  setCS(_pin_cs);
  return true;
}

void BusSPI::enableDummyByte(bool enable) { _dummy_byte = enable; }

void BusSPI::_select() {
  if (_before_transfer != nullptr) _before_transfer();
  _spi.beginTransaction(_settings);
  if (_pin_cs >= 0) digitalWrite(_pin_cs, LOW);
}

void BusSPI::_deselect() {
  if (_pin_cs >= 0) digitalWrite(_pin_cs, HIGH);
  _spi.endTransaction();
  if (_after_transfer != nullptr) _after_transfer();
}

void BusSPI::setCS(int8_t pin_cs) {
  _pin_cs = pin_cs;
  if (_pin_cs < 0) return;
  pinMode(_pin_cs, OUTPUT);
  digitalWrite(_pin_cs, HIGH);
}

void BusSPI::onBeforeTransfer(TransferCallback cb) { _before_transfer = cb; }

void BusSPI::onAfterTransfer(TransferCallback cb) { _after_transfer = cb; }

void BusSPI::clearOnBeforeTransfer() { _before_transfer = nullptr; }

void BusSPI::clearOnAfterTransfer() { _after_transfer = nullptr; }

bool BusSPI::writeRegister(uint8_t reg, uint8_t value) {
  _select();
  _spi.transfer(reg & 0x7F); // MSB=0 for write
  _spi.transfer(value);
  _deselect();
  return true;
}

bool BusSPI::readRegister(uint8_t reg, uint8_t& value) { return readBlock(reg, &value, 1); }

bool BusSPI::readBlock(uint8_t reg, uint8_t* buffer, size_t length) {
  _select();
  _spi.transfer(reg | 0x80);            // MSB=1 for read
  if (_dummy_byte) _spi.transfer(0x00); // Accelerometer sends a dummy byte first
  for (size_t i = 0; i < length; ++i)
    buffer[i] = _spi.transfer(0x00);
  _deselect();
  return true;
}

} // namespace bmi088
