#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

namespace bmi088 {

class Bus {
  public:
  /**
   * @brief Prepare the bus (e.g. configure the CS pin). Called by the sensor init().
   *
   * @return true Success.
   * @return false Failure.
   */
  virtual bool begin() { return true; }

  /**
   * @brief Enable the dummy byte sent by the accelerometer before the data on SPI reads. Called
   * by the accelerometer init(). It has no effect on I2C.
   *
   * @param enable True to discard the first byte of every read.
   */
  virtual void enableDummyByte(bool enable) { (void)enable; }

  virtual bool writeRegister(uint8_t reg, uint8_t value)              = 0;
  virtual bool readRegister(uint8_t reg, uint8_t& value)              = 0;
  virtual bool readBlock(uint8_t reg, uint8_t* buffer, size_t length) = 0;
  virtual ~Bus() {}
};

class BusI2C : public Bus {
  public:
  enum class Error : uint8_t {
    Success,
    DataTooLong,
    NACKAddress,
    NACKData,
    OtherError,
    Timeout,
    InvalidSize,
  };

  BusI2C(TwoWire& wire, uint8_t address);
  bool writeRegister(uint8_t reg, uint8_t value) override;
  bool readRegister(uint8_t reg, uint8_t& value) override;
  bool readBlock(uint8_t reg, uint8_t* buffer, size_t length) override;

  Error getError();

  private:
  TwoWire& _wire;
  uint8_t _address;
  uint8_t _error;
};

class BusSPI : public Bus {
  public:
  /**
   * @brief SPI bus. The BMI088 supports SPI mode 0 and 3, up to 10 MHz. If the accelerometer and
   * the gyroscope share the SPI bus and are used without bmi088::Imu, call begin() on both buses
   * before init() so every CS pin is high.
   *
   * @param spi SPI instance. spi.begin() must be called by the user.
   * @param pin_cs Chip select pin, -1 to handle it in the transfer callbacks.
   * @param clock SPI clock in Hz.
   */
  BusSPI(SPIClass& spi, int8_t pin_cs = -1, uint32_t clock = 10000000);
  bool begin() override;
  void enableDummyByte(bool enable) override;
  bool writeRegister(uint8_t reg, uint8_t value) override;
  bool readRegister(uint8_t reg, uint8_t& value) override;
  bool readBlock(uint8_t reg, uint8_t* buffer, size_t length) override;

  void setCS(int8_t pin_cs);

  typedef void (*TransferCallback)();
  void onBeforeTransfer(TransferCallback cb);
  void onAfterTransfer(TransferCallback cb);
  void clearOnBeforeTransfer();
  void clearOnAfterTransfer();

  private:
  SPIClass& _spi;
  SPISettings _settings;
  int8_t _pin_cs;
  bool _dummy_byte = false;

  TransferCallback _before_transfer = nullptr;
  TransferCallback _after_transfer  = nullptr;

  void _select();
  void _deselect();
};

} // namespace bmi088
