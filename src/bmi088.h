#pragma once

#include "bmi088Bus.h"
#include <Arduino.h>

#ifndef BMI088_FIFO_BUFFER_SIZE
/// Size in bytes of the stack buffer used by readFifo(). Every FIFO burst read is at most this
/// size, so with I2C it must fit in the Wire buffer (e.g. 32 bytes on AVR).
#define BMI088_FIFO_BUFFER_SIZE 126
#endif

namespace bmi088 {

constexpr float StandardGravity = 9.80665f; ///< m/s² per g.

struct RawData {
  int16_t x;
  int16_t y;
  int16_t z;
};

struct ConvertedData {
  float x;
  float y;
  float z;
};

// MARK: Accel
class Accel {
  public:
  static constexpr uint8_t ChipID = 0x1E;

  enum class Bandwidth : uint8_t {
    Osr4 = 0x08,
    Osr2,
    Normal,
  };

  enum class DataRate : uint8_t {
    Odr12_5Hz = 0x05,
    Odr25Hz,
    Odr50Hz,
    Odr100Hz,
    Odr200Hz,
    Odr400Hz,
    Odr800Hz,
    Odr1600Hz,
  };

  enum class Range : uint8_t {
    Range3g,
    Range6g,
    Range12g,
    Range24g,
  };

  enum class GpioPin : uint8_t {
    Int1,
    Int2,
  };

  enum class GpioMode : uint8_t {
    Disabled = 0x00,
    Output,
    Input,
  };

  enum class GpioBehavior : uint8_t {
    PushPull,
    OpenDrain,
  };

  enum class GpioLevel : uint8_t {
    Low,
    High,
  };

  enum class SelfTestMode : uint8_t {
    Off,
    Positive = 0x0D,
    Negative = 0x09,
  };

  enum class Mode : uint8_t {
    Active  = 0x00,
    Suspend = 0x03,
  };

  enum class FifoMode : uint8_t {
    Fifo,   ///< Stop writing when the FIFO is full.
    Stream, ///< Overwrite the oldest frames when the FIFO is full.
  };

  struct GpioConfig {
    GpioMode mode;
    GpioBehavior behavior;
    GpioLevel level;
    bool interrupt;
  };

  // Estructura con todas las configuraciones del sensor
  struct Config {
    uint8_t id;
    Bandwidth bw;
    DataRate dr;
    Range range;
    Mode mode;
    GpioConfig int1;
    GpioConfig int2;
  };

  enum class Error : uint8_t {
    None,
    NoGetChipID,
    NoGetChipError,
    NoGetStatus,
    NoGetRawData,
    NoGetRawXData,
    NoGetRawYData,
    NoGetRawZData,
    NoGetTime,
    NoGetTemperature,
    NoGetConfig,
    NoGetRange,
    NoGetInt1PinConfig,
    NoGetInt2PinConfig,
    NoGetInt1PinLevel,
    NoGetInt2PinLevel,
    NoGetMapInterrupt,
    NoGetSelfTestMode,
    NoGetPowerMode,
    NoClearInterrupt,
    NoSetConfig,
    NoSetRange,
    NoSetInt1PinConfig,
    NoSetInt2PinConfig,
    NoSetInt1PinLevel,
    NoSetInt2PinLevel,
    NoSetMapInterrupt,
    NoSetSelfTestMode,
    NoSetPowerMode,
    NoSetPowerOn,
    NoSetPowerOff,
    InvalidChipID,
    NoSoftReset,
    NoGetInterruptStatus,
    NoGetFifoLength,
    NoGetFifoData,
    NoGetFifoMode,
    NoGetFifoWatermark,
    NoSetFifoMode,
    NoSetFifoConfig,
    NoSetFifoWatermark,
    NoSetFifoDownsampling,
    NoSetMapFifoInterrupt,
  };

  Accel(Bus& bus);

  /**
   * @brief Error callback function, this function is called when an error occurs and give the error
   * code.
   *
   */
  typedef void (*OnErrorCallback)(Error error);

  /**
   * @brief On error callback, this function is called when an error occurs and give the error code.
   *
   * @param cb Error callback function.
   */
  void onError(OnErrorCallback cb);

  /**
   * @brief Delete the error callback.
   *
   */
  void clearErrorCallback();

  /**
   * @brief Obtain the absolute angle of the X-axis.
   *
   * @param data Acceleration data on X, Y and Z axis (any unit).
   * @return float Absolute angle in degrees.
   */
  static float getAbsoluteXAngle(const ConvertedData& data);

  /**
   * @brief Obtain the absolute angle of the Y-axis.
   *
   * @param data Acceleration data on X, Y and Z axis (any unit).
   * @return float Absolute angle in degrees.
   */
  static float getAbsoluteYAngle(const ConvertedData& data);

  /**
   * @brief Get the last error of the accelerometer.
   *
   * @return Error Last error.
   */
  Error getError();

  /**
   * @brief Initialize the BMI088 accelerometer: check the chip ID, soft reset, set active mode,
   * power on and set the range to 6g.
   *
   * @return true Success.
   * @return false Failure.
   */
  bool init();

  /**
   * @brief Obtain the chip ID, which is 0x1E.
   *
   * @param id ID number.
   * @return true Success.
   * @return false Failure.
   */
  bool getChipID(uint8_t& id);

  /**
   * @brief Obtain the error status of the chip.
   *
   * @param fatal_error True if fatal error.
   * @param config_error True if configuration error.
   * @return true Success.
   * @return false Failure.
   */
  bool getChipError(bool& fatal_error, bool& config_error);

  /**
   * @brief Obtain the status of the data ready.
   *
   * @param data_ready Data ready to read.
   * @return true Success.
   * @return false Failure.
   */
  bool getStatus(bool& data_ready);

  /**
   * @brief Obtain the raw data of the accelerometer in X, Y and Z axis.
   *
   * @param data Raw data.
   * @return true Success.
   * @return false Failure.
   */
  bool getRawData(RawData& data);

  /**
   * @brief Obtain the raw data of the accelerometer in X axis.
   *
   * @param x Raw data.
   * @return true Success.
   * @return false Failure.
   */
  bool getRawXData(int16_t& x);

  /**
   * @brief Obtain the raw data of the accelerometer in Y axis.
   *
   * @param y Raw data.
   * @return true Success.
   * @return false Failure.
   */
  bool getRawYData(int16_t& y);

  /**
   * @brief Obtain the raw data of the accelerometer in Z axis.
   *
   * @param z Raw data.
   * @return true Success.
   * @return false Failure.
   */
  bool getRawZData(int16_t& z);

  /**
   * @brief Obtain the acceleration in milli-g in X, Y and Z axis.
   *
   * @param data Data acceleration in X, Y and Z axis.
   * @return true Success.
   * @return false Failure.
   */
  bool getMilliGAcceleration(ConvertedData& data);

  /**
   * @brief Obtain the acceleration in milli-g in X axis.
   *
   * @param x X axis acceleration in milli-g.
   * @return true Success.
   * @return false Failure.
   */
  bool getMilliGXAcceleration(float& x);

  /**
   * @brief Obtain the acceleration in milli-g in Y axis.
   *
   * @param y Y axis acceleration in milli-g.
   * @return true Success.
   * @return false Failure.
   */
  bool getMilliGYAcceleration(float& y);

  /**
   * @brief Obtain the acceleration in milli-g in Z axis.
   *
   * @param z Z axis acceleration in milli-g.
   * @return true Success.
   * @return false Failure.
   */
  bool getMilliGZAcceleration(float& z);

  /**
   * @brief Obtain the sensor time. Increment every 39.0625 uS.
   *
   * @param time Sensor time.
   * @return true Success.
   * @return false Failure.
   */
  bool getSensorTime(uint32_t& time);

  /**
   * @brief Obtain the temperature. 11 bits signed integer. Resolution is 0.125 °C/LSB. Update
   * every 1.28 second.
   *
   * @param temperature Temperature in Celsius.
   * @return true Success.
   * @return false Failure.
   */
  bool getTemperature(float& temperature);

  /**
   * @brief Clear the data ready interrupt.
   *
   * @return true Success.
   * @return false Failure.
   */
  bool clearInterrupt();

  /**
   * @brief Set the configuration of the accelerometer. The frecuency of the filter depends of the
   * Output Data Rate. See the datasheet for more information.
   *
   * @param bw Bandwidth.
   * @param dr Data rate.
   * @return true Success.
   * @return false Failure.
   */
  bool setConfig(Bandwidth bw, DataRate dr);

  /**
   * @brief Get the configuration of the accelerometer.
   *
   * @param bw Bandwidth.
   * @param dr Data rate.
   * @return true Success.
   * @return false Failure.
   */
  bool getConfig(Bandwidth& bw, DataRate& dr);

  /**
   * @brief Set the bandwidth of the accelerometer.
   *
   * @param bw Bandwidth.
   * @return true Success.
   * @return false Failure.
   */
  bool setBandwidth(Bandwidth bw);

  /**
   * @brief Get the bandwidth of the accelerometer.
   *
   * @param bw Bandwidth.
   * @return true Success.
   * @return false Failure.
   */
  bool getBandwidth(Bandwidth& bw);

  /**
   * @brief Set the data rate of the accelerometer.
   *
   * @param dr Data rate.
   * @return true Success.
   * @return false Failure.
   */
  bool setDataRate(DataRate dr);

  /**
   * @brief Get the data rate of the accelerometer.
   *
   * @param dr Data rate.
   * @return true Success.
   * @return false Failure.
   */
  bool getDataRate(DataRate& dr);

  /**
   * @brief Set the range of the accelerometer.
   *
   * @param range Range.
   * @return true Success.
   * @return false Failure.
   */
  bool setRange(Range range);

  /**
   * @brief Get the range of the accelerometer.
   *
   * @param range Range.
   * @return true Success.
   * @return false Failure.
   */
  bool getRange(Range& range);

  /**
   * @brief Set the pin configuration. For accelerometer, pins INT1 and INT2 can be set as
   * input or output. The active level of the pin is kept.
   *
   * @param pin Pin number.
   * @param mode Pin mode.
   * @param behavior Pin behavior.
   * @return true Success.
   * @return false Failure.
   */
  bool pinConfig(GpioPin pin, GpioMode mode, GpioBehavior behavior);

  /**
   * @brief Get the pin configuration.
   *
   * @param pin Pin number.
   * @param mode Pin mode.
   * @param behavior Pin behavior.
   * @return true Success.
   * @return false Failure.
   */
  bool getPinConfig(GpioPin pin, GpioMode& mode, GpioBehavior& behavior);

  /**
   * @brief Set the active level (polarity) of the interrupt pin. While no interrupt is active the
   * pin stays at the opposite level, so an output pin with no interrupt mapped is Low when the
   * active level is High and vice versa.
   *
   * @param pin Pin number.
   * @param level Active level.
   * @return true Success.
   * @return false Failure.
   */
  bool setPinActiveLevel(GpioPin pin, GpioLevel level);

  /**
   * @brief Get the active level (polarity) of the interrupt pin.
   *
   * @param pin Pin number.
   * @param level Active level.
   * @return true Success.
   * @return false Failure.
   */
  bool getPinActiveLevel(GpioPin pin, GpioLevel& level);

  /**
   * @brief Set the interrupt mapping. This function is used to map the data ready interrupt to pin
   * INT1 and/or INT2.
   *
   * @param int1 Interrupt 1.
   * @param int2 Interrupt 2.
   * @return true Success.
   * @return false Failure.
   */
  bool setMapInterrupt(bool int1 = false, bool int2 = false);

  /**
   * @brief Get the interrupt mapping.
   *
   * @param int1 Interrupt 1.
   * @param int2 Interrupt 2.
   * @return true Success.
   * @return false Failure.
   */
  bool getMapInterrupt(bool& int1, bool& int2);

  /**
   * @brief Set the self test mode.
   *
   * @param mode Self test mode.
   * @return true Success.
   * @return false Failure.
   */
  bool setSelfTestMode(SelfTestMode mode);

  /**
   * @brief Self test the 3 axis of the accelerometer, following the datasheet procedure. The
   * sensor is reset at the end and the previous configuration is restored. This function uses
   * delay() (~270 ms). Stop read / write data during the self test.
   *
   * @param x_pass Axis X True if pass.
   * @param y_pass Axis Y True if pass.
   * @param z_pass Axis Z True if pass.
   * @return true Success.
   * @return false Failure.
   */
  bool selfTest(bool& x_pass, bool& y_pass, bool& z_pass);

  /**
   * @brief Get the difference between the positive and negative self test, in milli-g. The
   * minimum values to pass are 1000 mg in X and Y, and 500 mg in Z.
   *
   * @param data Self test difference in milli-g.
   */
  void getSelfTestData(ConvertedData& data);

  /**
   * @brief Get the self test mode.
   *
   * @param mode Self test mode.
   * @return true Success.
   * @return false Failure.
   */
  bool getSelfTestMode(SelfTestMode& mode);

  /**
   * @brief Set the power mode.
   *
   * @param mode Power mode.
   * @return true Success.
   * @return false Failure.
   */
  bool setPowerMode(Mode mode);

  /**
   * @brief Get the power mode.
   *
   * @param mode Power mode.
   * @return true Success.
   * @return false Failure.
   */
  bool getPowerMode(Mode& mode);

  /**
   * @brief Power on the accelerometer.
   *
   * @return true Success.
   * @return false Failure.
   */
  bool powerOn();

  /**
   * @brief Power off the accelerometer.
   *
   * @return true Success.
   * @return false Failure.
   */
  bool powerOff();

  /**
   * @brief Soft reset the accelerometer. After the reset the sensor is powered off, in suspend mode
   * and with all registers in their default values.
   *
   * @return true Success.
   * @return false Failure.
   */
  bool softReset();

  /**
   * @brief Set the raw offset of the accelerometer. It is subtracted from the raw data in the
   * milli-g conversion.
   *
   * @param offset Raw offset.
   */
  void setRawOffset(const RawData& offset);

  /**
   * @brief Get the All Config data.
   *
   * @param config All configuration data.
   * @return true Success.
   * @return false Failure.
   */
  bool getAllConfig(Config& config);

  /**
   * @brief Write all the configuration data. The chip ID is ignored.
   *
   * @param config All configuration data.
   * @return true Success.
   * @return false Failure.
   */
  bool setAllConfig(const Config& config);

  /**
   * @brief Check the communication with the accelerometer reading the chip ID.
   *
   * @return true The chip ID is 0x1E.
   * @return false Failure or wrong chip ID.
   */
  bool isConnected();

  /**
   * @brief Calibrate the raw offset averaging samples at rest. The sensor must be flat with the Z
   * axis up, so the Z axis keeps 1 g. Waits one sample period (from the data rate) between
   * samples.
   *
   * @param samples Number of samples to average.
   * @return true Success.
   * @return false Failure.
   */
  bool calibrateOffset(uint16_t samples = 100);

  /**
   * @brief Get the raw offset of the accelerometer.
   *
   * @param offset Raw offset.
   */
  void getRawOffset(RawData& offset);

  /**
   * @brief Convert raw data to milli-g, subtracting the raw offset and using the current range.
   * Useful for FIFO frames.
   *
   * @param raw Raw data.
   * @param data Acceleration in milli-g.
   */
  void convertToMilliG(const RawData& raw, ConvertedData& data);

  /**
   * @brief Obtain the acceleration in m/s² in X, Y and Z axis.
   *
   * @param data Acceleration in m/s².
   * @return true Success.
   * @return false Failure.
   */
  bool getAcceleration(ConvertedData& data);

  /**
   * @brief Obtain the raw data and the sensor time in a single burst read, so both belong to the
   * same sample.
   *
   * @param data Raw data.
   * @param time Sensor time. Increment every 39.0625 uS.
   * @return true Success.
   * @return false Failure.
   */
  bool getRawDataAndTime(RawData& data, uint32_t& time);

  /**
   * @brief Read the data ready interrupt status. Reading the status clears the interrupt.
   *
   * @param data_ready True if the data ready interrupt was active.
   * @return true Success.
   * @return false Failure.
   */
  bool getInterruptStatus(bool& data_ready);

  /**
   * @brief Set the FIFO mode.
   *
   * @param mode FIFO mode.
   * @return true Success.
   * @return false Failure.
   */
  bool setFifoMode(FifoMode mode);

  /**
   * @brief Get the FIFO mode.
   *
   * @param mode FIFO mode.
   * @return true Success.
   * @return false Failure.
   */
  bool getFifoMode(FifoMode& mode);

  /**
   * @brief Select the data stored in the FIFO. The FIFO always works in header mode.
   *
   * @param accel Store the acceleration data.
   * @param int1_input Store the INT1 input level (pin configured as input).
   * @param int2_input Store the INT2 input level (pin configured as input).
   * @return true Success.
   * @return false Failure.
   */
  bool enableFifo(bool accel, bool int1_input = false, bool int2_input = false);

  /**
   * @brief Set the FIFO watermark. Each acceleration frame uses 7 bytes.
   *
   * @param bytes Watermark level in bytes (0 to 1023).
   * @return true Success.
   * @return false Failure.
   */
  bool setFifoWatermark(uint16_t bytes);

  /**
   * @brief Get the FIFO watermark.
   *
   * @param bytes Watermark level in bytes.
   * @return true Success.
   * @return false Failure.
   */
  bool getFifoWatermark(uint16_t& bytes);

  /**
   * @brief Set the FIFO downsampling. The FIFO data rate is ODR / 2^exponent.
   *
   * @param exponent Downsampling exponent (0 to 7).
   * @return true Success.
   * @return false Failure.
   */
  bool setFifoDownsampling(uint8_t exponent);

  /**
   * @brief Get the number of bytes stored in the FIFO (up to 1024).
   *
   * @param bytes FIFO fill level in bytes.
   * @return true Success.
   * @return false Failure.
   */
  bool getFifoLength(uint16_t& bytes);

  /**
   * @brief Read raw bytes from the FIFO in a single burst. Use parseFifo() to extract the frames.
   * Read whole frames: in normal operation all frames are acceleration frames of 7 bytes.
   *
   * @param buffer Destination buffer.
   * @param length Number of bytes to read.
   * @return true Success.
   * @return false Failure.
   */
  bool readFifoData(uint8_t* buffer, size_t length);

  /**
   * @brief Read the acceleration frames stored in the FIFO, in bursts of up to
   * BMI088_FIFO_BUFFER_SIZE bytes. Frames that don't fit in max_frames stay in the FIFO.
   *
   * @param frames Destination array of raw data.
   * @param max_frames Size of the array.
   * @param frames_read Number of frames read.
   * @return true Success.
   * @return false Failure.
   */
  bool readFifo(RawData* frames, size_t max_frames, size_t& frames_read);

  /**
   * @brief Extract the acceleration frames from raw FIFO bytes. Skip, sensor time, config and drop
   * frames are ignored. It stops at the end of the data or at an incomplete frame.
   *
   * @param buffer Raw FIFO bytes.
   * @param length Number of bytes.
   * @param frames Destination array of raw data.
   * @param max_frames Size of the array.
   * @return size_t Number of acceleration frames extracted.
   */
  static size_t parseFifo(const uint8_t* buffer, size_t length, RawData* frames, size_t max_frames);

  /**
   * @brief Discard all the data stored in the FIFO.
   *
   * @return true Success.
   * @return false Failure.
   */
  bool flushFifo();

  /**
   * @brief Map the FIFO interrupts to a pin. The data ready mapping is kept.
   *
   * @param pin Pin number.
   * @param watermark FIFO watermark interrupt.
   * @param full FIFO full interrupt.
   * @return true Success.
   * @return false Failure.
   */
  bool setMapFifoInterrupt(GpioPin pin, bool watermark, bool full);

  private:
  /// Sensor time: 24-bit counter increment every 39.0625 uS. Total time is 655.36 seconds.
  /// Temperature: 11-bits signed integer. Resolution is 0.125 °C/LSB.
  enum class Registers : uint8_t {
    ChipID,                  ///< Identifier code of acceleration sensor.
    ErrorReg = 0x02,         ///< Report error conditions.
    Status,                  ///< Status if the sensor is ready to read data.
    XLsb = 0x12,             ///< X-axis acceleration least significant byte.
    XMsb,                    ///< X-axis acceleration more significant byte.
    YLsb,                    ///< Y-axis acceleration least significant byte.
    YMsb,                    ///< Y-axis acceleration more significant byte.
    ZLsb,                    ///< Z-axis acceleration least significant byte.
    ZMsb,                    ///< Z-axis acceleration more significant byte.
    SensorTime0,             ///< lower 8 bits of the sensor time.
    SensorTime1,             ///< middle 8 bits of the sensor time.
    SensorTime2,             ///< upper 8 bits of the sensor time.
    InterruptStatus1 = 0x1D, ///< Interrupt status for acceleration data ready.
    TemperatureMsb   = 0x22, ///< Temperature most significant byte.
    TemperatureLsb,          ///< Temperature least significant byte.
    FifoLength0,             ///< FIFO fill level, lower 8 bits.
    FifoLength1,             ///< FIFO fill level, upper 6 bits.
    FifoData,                ///< FIFO data output.
    Config = 0x40,           ///< Configuration register.
    Range,                   ///< Range settings register.
    FifoDownsampling = 0x45, ///< FIFO downsampling.
    FifoWatermark0,          ///< FIFO watermark, lower 8 bits.
    FifoWatermark1,          ///< FIFO watermark, upper 5 bits.
    FifoConfig0,             ///< FIFO mode.
    FifoConfig1,             ///< FIFO data selection.
    Interrupt1IOCtrl = 0x53, ///< Configures the input/output pin INT1.
    Interrupt2IOCtrl,        ///< Configures the input/output pin INT2.
    InterruptMapData = 0x58, ///< Map data ready interrupt to pin INT1 and/or INT2.
    SelfTest         = 0x6D, ///< Self-test register.
    PowerConfig      = 0x7C, ///< Active / Desactive suspend mode.
    PowerControl,            ///< On or Off accelerometer.
    SoftReset,               ///< Soft reset register.
  };

  Bus& _bus;
  uint8_t _register;
  uint8_t _values[9];
  uint8_t _range;
  float _factor_mg;

  RawData _raw_offset;
  ConvertedData _test_result_data;
  OnErrorCallback _on_error;
  Error _error;

  /**
   * @brief Calculate the factor of the accelerometer.
   */
  void _calculateFactorMilliG();

  /**
   * @brief Set the error.
   *
   * @param error Error.
   */
  void _setError(Error error);
};

// MARK: Gyro
class Gyro {
  public:
  static constexpr uint8_t ChipID = 0x0F;

  enum class Range : uint8_t {
    Dps2000,
    Dps1000,
    Dps500,
    Dps250,
    Dps125,
  };

  enum class OdrAndBandwidth : uint8_t {
    Odr2000HzFilter532Hz,
    Odr2000HzFilter230Hz,
    Odr1000HzFilter116Hz,
    Odr400HzFilter47Hz,
    Odr200HzFilter23Hz,
    Odr100HzFilter12Hz,
    Odr200HzFilter64Hz,
    Odr100HzFilter32Hz,
  };

  enum class Mode : uint8_t {
    Normal      = 0x00,
    Suspend     = 0x80,
    DeepSuspend = 0x20,
  };

  enum class FifoMode : uint8_t {
    Fifo   = 0x40, ///< Stop writing when the FIFO is full.
    Stream = 0x80, ///< Overwrite the oldest frames when the FIFO is full.
  };

  enum class GpioPin : uint8_t {
    Int3,
    Int4,
  };

  enum class GpioOutConfig : uint8_t {
    PushPull,
    OpenDrain,
  };

  enum class GpioLevel : uint8_t {
    Low,
    High,
  };

  enum class GpioMapInterrupt : uint8_t {
    NoInterrupt,
    InterruptOnInt3,
    InterruptOnInt4 = 0x80,
    InterruptOnBoth,
  };

  enum class Error : uint8_t {
    None,
    NoGetChipID,
    NoGetRawData,
    NoGetRawXData,
    NoGetRawYData,
    NoGetRawZData,
    NoGetRange,
    NoGetOdrAndBandwidth,
    NoGetPowerMode,
    NoGetInterruptControl,
    NoGetPinsConfig,
    NoGetInt3Config,
    NoGetInt4Config,
    NoGetInt3Level,
    NoGetInt4Level,
    NoGetMapInterrupt,
    NoGetSelfTestResult,
    NoSetRange,
    NoSetOdrAndBandwidth,
    NoSetPowerMode,
    NoSetInterruptControl,
    NoSetPinsConfig,
    NoSetInt3Config,
    NoSetInt4Config,
    NoSetInt3Level,
    NoSetInt4Level,
    NoSetMapInterrupt,
    NoStartSelfTest,
    InvalidChipID,
    NoSoftReset,
    NoGetStatus,
    NoGetFifoStatus,
    NoGetFifoData,
    NoGetFifoMode,
    NoGetFifoWatermark,
    NoSetFifoMode,
    NoSetFifoWatermark,
    NoSetFifoInterrupt,
    NoSetMapFifoInterrupt,
    SelfTestTimeout,
  };

  struct Config {
    uint8_t id;
    Range range;
    OdrAndBandwidth odr_and_bw;
    Mode mode;
    GpioOutConfig int3;
    GpioOutConfig int4;
    GpioMapInterrupt map_interrupt;
  };

  /**
   * @brief Error callback function, this function is called when an error occurs and give the error
   * code.
   *
   */
  typedef void (*OnErrorCallback)(Error error);

  /**
   * @brief On error callback, this function is called when an error occurs and give the error code.
   *
   * @param cb Error callback function.
   */
  void onError(OnErrorCallback cb);

  /**
   * @brief Delete the error callback.
   *
   */
  void clearErrorCallback();

  /**
   * @brief Integrate the angular velocity to get a relative angle, wrapped to [0, 360).
   *
   * @param last_angle Last angle in degrees.
   * @param angle_velocity Angle velocity in degrees per second.
   * @param dt Sampling time in seconds.
   * @return float Relative angle in degrees.
   */
  static float getRelativeEulerAngle(float last_angle, float angle_velocity, float dt);

  Gyro(Bus& bus);

  /**
   * @brief Get the last error of the gyroscope.
   *
   * @return Error Last error.
   */
  Error getError();

  /**
   * @brief Get the All Config object
   *
   * @param config All configuration data.
   * @return true Success.
   * @return false Failure.
   */
  bool getAllConfig(Config& config);

  /**
   * @brief Initialize the BMI088 gyroscope: check the chip ID, soft reset and set the range to
   * 2000 dps.
   *
   * @return true Success.
   * @return false Failure.
   */
  bool init();

  /**
   * @brief Get the chip ID, which is 0x0F.
   *
   * @param id ID number.
   * @return true Success.
   * @return false Failure.
   */
  bool getChipID(uint8_t& id);

  /**
   * @brief Get the raw data of the gyroscope in X, Y and Z axis.
   *
   * @param data Raw data.
   * @return true Success.
   * @return false Failure.
   */
  bool getRawData(RawData& data);

  /**
   * @brief Get the raw data of the gyroscope in X axis.
   *
   * @param x Raw data.
   * @return true Success.
   * @return false Failure.
   */
  bool getRawXData(int16_t& x);

  /**
   * @brief Get the raw data of the gyroscope in Y axis.
   *
   * @param y Raw data.
   * @return true Success.
   * @return false Failure.
   */
  bool getRawYData(int16_t& y);

  /**
   * @brief Get the raw data of the gyroscope in Z axis.
   *
   * @param z Raw data.
   * @return true Success.
   * @return false Failure.
   */
  bool getRawZData(int16_t& z);

  /**
   * @brief Get the angular velocity in degrees per second in X, Y and Z axis.
   *
   * @param data Angular velocity data.
   * @return true Success.
   * @return false Failure.
   */
  bool getAngularVelocity(ConvertedData& data);

  /**
   * @brief Get the angular velocity in degrees per second in X axis.
   *
   * @param x Angular velocity in degrees per second.
   * @return true Success.
   * @return false Failure.
   */
  bool getAngularVelocityX(float& x);

  /**
   * @brief Get the angular velocity in degrees per second in Y axis.
   *
   * @param y Angular velocity in degrees per second.
   * @return true Success.
   * @return false Failure.
   */
  bool getAngularVelocityY(float& y);

  /**
   * @brief Get the angular velocity in degrees per second in Z axis.
   *
   * @param z Angular velocity in degrees per second.
   * @return true Success.
   * @return false Failure.
   */
  bool getAngularVelocityZ(float& z);

  /**
   * @brief Set the Range.
   *
   * @param range Range.
   * @return true Success.
   * @return false Failure.
   */
  bool setRange(Range range);

  /**
   * @brief Get the Range.
   *
   * @param range Range.
   * @return true Success.
   * @return false Failure.
   */
  bool getRange(Range& range);

  /**
   * @brief Set the data rate and bandwidth.
   *
   * @param bw Data rate and bandwidth.
   * @return true Success.
   * @return false Failure.
   */
  bool setDataRateAndBandwidth(OdrAndBandwidth bw);

  /**
   * @brief Get the data rate and bandwidth.
   *
   * @param bw Data rate and bandwidth.
   * @return true Success.
   * @return false Failure.
   */
  bool getDataRateAndBandwidth(OdrAndBandwidth& bw);

  /**
   * @brief Set the power mode.
   *
   * @param mode Power mode.
   * @return true Success.
   * @return false Failure.
   */
  bool setPowerMode(Mode mode);

  /**
   * @brief Get the power mode.
   *
   * @param mode Power mode.
   * @return true Success.
   * @return false Failure.
   */
  bool getPowerMode(Mode& mode);

  /**
   * @brief Soft reset the gyroscope. All registers return to their default values. This function
   * uses delay() (30 ms).
   *
   * @return true Success.
   * @return false Failure.
   */
  bool softReset();

  /**
   * @brief Enable or disable the data ready interrupt.
   *
   * @param enable Bit to enable or disable the interrupt.
   * @return true Success.
   * @return false Failure.
   */
  bool enableInterrupt(bool enable);

  /**
   * @brief Get if the data ready interrupt is enabled.
   *
   * @param enable Bit to enable or disable the interrupt.
   * @return true Success.
   * @return false Failure.
   */
  bool isEnabledInterrupt(bool& enable);

  /**
   * @brief Get the configuration of the interrupt pin.
   *
   * @param conf_int3 INT3 configuration.
   * @param conf_int4 INT4 configuration.
   * @return true Success.
   * @return false Failure.
   */
  bool getPinsConfig(GpioOutConfig& conf_int3, GpioOutConfig& conf_int4);

  /**
   * @brief Set the configuration of the interrupt pins. In gyroscope, pins INT3 and INT4 can
   * be set only as output, but the behavior can be set as push-pull or open-drain. The active
   * levels of the pins are kept.
   *
   * @param conf_int3 INT3 configuration.
   * @param conf_int4 INT4 configuration.
   * @return true Success.
   * @return false Failure.
   */
  bool setPinsConfig(GpioOutConfig conf_int3, GpioOutConfig conf_int4);

  /**
   * @brief Set the configuration of the interrupt pin. In gyroscope, pins INT3 and INT4 can be set
   * as output, but the behavior can be set as push-pull or open-drain. The active level of the pin
   * is kept.
   *
   * @param pin Pin number.
   * @param conf Pin configuration.
   * @return true Success.
   * @return false Failure.
   */
  bool pinConfig(GpioPin pin, GpioOutConfig conf);

  /**
   * @brief Get the configuration of the interrupt pin.
   *
   * @param pin Pin number.
   * @param conf Pin configuration.
   * @return true Success.
   * @return false Failure.
   */
  bool getPinConfig(GpioPin pin, GpioOutConfig& conf);

  /**
   * @brief Set the active level (polarity) of the interrupt pin. While no interrupt is active the
   * pin stays at the opposite level.
   *
   * @param pin INT3 or INT4 pin.
   * @param level Active level.
   * @return true Success.
   * @return false Failure.
   */
  bool setPinActiveLevel(GpioPin pin, GpioLevel level);

  /**
   * @brief Get the active level (polarity) of the interrupt pin.
   *
   * @param pin INT3 or INT4 pin.
   * @param level Active level.
   * @return true Success.
   * @return false Failure.
   */
  bool getPinActiveLevel(GpioPin pin, GpioLevel& level);

  /**
   * @brief Set the data ready interrupt as None or INT3, INT4 or both.
   *
   * @param map_int Interrupt mapping.
   * @return true Success.
   * @return false Failure.
   */
  bool setMapInterrupt(GpioMapInterrupt map_int);

  /**
   * @brief Get the data ready interrupt mapping.
   *
   * @param map_int Interrupt mapping.
   * @return true Success.
   * @return false Failure.
   */
  bool getMapInterrupt(GpioMapInterrupt& map_int);

  /**
   * @brief Start the self test.
   *
   * @return true Success.
   * @return false Failure.
   */
  bool startSelfTest();

  /**
   * @brief Get the Self Test Result object.
   *
   * @param ready Selftest ready.
   * @param pass Selftest pass.
   * @return true Success.
   * @return false Failure.
   */
  bool getSelfTestResult(bool& ready, bool& pass);

  /**
   * @brief Run the built-in self test and wait for the result (up to 100 ms).
   *
   * @param pass True if the self test passed.
   * @return true Success.
   * @return false Failure or timeout.
   */
  bool selfTest(bool& pass);

  /**
   * @brief Check the communication with the gyroscope reading the chip ID.
   *
   * @return true The chip ID is 0x0F.
   * @return false Failure or wrong chip ID.
   */
  bool isConnected();

  /**
   * @brief Get the data ready status.
   *
   * @param data_ready True if new data is available.
   * @return true Success.
   * @return false Failure.
   */
  bool getStatus(bool& data_ready);

  /**
   * @brief Set the raw offset (bias) of the gyroscope. It is subtracted from the raw data in the
   * conversions.
   *
   * @param offset Raw offset.
   */
  void setRawOffset(const RawData& offset);

  /**
   * @brief Get the raw offset (bias) of the gyroscope.
   *
   * @param offset Raw offset.
   */
  void getRawOffset(RawData& offset);

  /**
   * @brief Calibrate the bias averaging samples at rest. The sensor must not move. Waits one
   * sample period (from the data rate) between samples.
   *
   * @param samples Number of samples to average.
   * @return true Success.
   * @return false Failure.
   */
  bool calibrateOffset(uint16_t samples = 100);

  /**
   * @brief Convert raw data to degrees per second, subtracting the raw offset and using the current
   * range. Useful for FIFO frames.
   *
   * @param raw Raw data.
   * @param data Angular velocity in degrees per second.
   */
  void convertToDps(const RawData& raw, ConvertedData& data);

  /**
   * @brief Get the angular velocity in radians per second in X, Y and Z axis.
   *
   * @param data Angular velocity in radians per second.
   * @return true Success.
   * @return false Failure.
   */
  bool getAngularVelocityRad(ConvertedData& data);

  /**
   * @brief Set the FIFO mode. Writing the mode clears the FIFO.
   *
   * @param mode FIFO mode.
   * @return true Success.
   * @return false Failure.
   */
  bool setFifoMode(FifoMode mode);

  /**
   * @brief Get the FIFO mode.
   *
   * @param mode FIFO mode.
   * @return true Success.
   * @return false Failure.
   */
  bool getFifoMode(FifoMode& mode);

  /**
   * @brief Set the FIFO watermark. Each frame uses 6 bytes (X, Y and Z).
   *
   * @param frames Watermark level in frames (0 to 100).
   * @return true Success.
   * @return false Failure.
   */
  bool setFifoWatermark(uint8_t frames);

  /**
   * @brief Get the FIFO watermark.
   *
   * @param frames Watermark level in frames.
   * @return true Success.
   * @return false Failure.
   */
  bool getFifoWatermark(uint8_t& frames);

  /**
   * @brief Enable or disable the FIFO watermark interrupt.
   *
   * @param enable True to enable.
   * @return true Success.
   * @return false Failure.
   */
  bool enableFifoInterrupt(bool enable);

  /**
   * @brief Map the FIFO interrupt to INT3 and/or INT4. The data ready mapping is kept.
   *
   * @param int3 Map to INT3.
   * @param int4 Map to INT4.
   * @return true Success.
   * @return false Failure.
   */
  bool setMapFifoInterrupt(bool int3, bool int4);

  /**
   * @brief Get the FIFO status.
   *
   * @param frames Number of frames stored in the FIFO (up to 100).
   * @param overrun True if frames were lost.
   * @return true Success.
   * @return false Failure.
   */
  bool getFifoStatus(uint8_t& frames, bool& overrun);

  /**
   * @brief Read the frames stored in the FIFO, in bursts of up to BMI088_FIFO_BUFFER_SIZE bytes.
   * Frames that don't fit in max_frames stay in the FIFO.
   *
   * @param frames Destination array of raw data.
   * @param max_frames Size of the array.
   * @param frames_read Number of frames read.
   * @return true Success.
   * @return false Failure.
   */
  bool readFifo(RawData* frames, size_t max_frames, size_t& frames_read);

  /**
   * @brief Discard all the data stored in the FIFO, writing the FIFO mode again.
   *
   * @return true Success.
   * @return false Failure.
   */
  bool flushFifo();

  private:
  enum class Registers : uint8_t {
    ChipID,                  ///< Identifier code of gyroscope sensor.
    RateXLsb = 0x02,         ///< Angular velocity X-axis least significant byte.
    RateXMsb,                ///< Angular velocity X-axis most significant byte.
    RateYLsb,                ///< Angular velocity Y-axis least significant byte.
    RateYMsb,                ///< Angular velocity Y-axis most significant byte.
    RateZLsb,                ///< Angular velocity Z-axis least significant byte.
    RateZMsb,                ///< Angular velocity Z-axis most significant byte.
    InterruptStatus1 = 0x0A, ///< Interrupt status for gyroscope data ready.
    FifoStatus       = 0x0E, ///< FIFO frame counter and overrun.
    Range,                   ///< Range settings register.
    Bandwidth,               ///< Bandwidth settings register.
    Lpm1,                    ///< Power modes register.
    Softreset = 0x14,        ///< Soft reset register.
    InterruptControl,        ///< Enable / Disable interrupts.
    IntIOConfig,             ///< Config logical properties of the interrupt pin.
    IntIOMap     = 0x18,     ///< Map data ready interrupt to pin INT3 and/or INT4.
    FifoWmEnable = 0x1E,     ///< Enable the FIFO watermark interrupt.
    SelfTest     = 0x3C,     ///< Self-test register.
    FifoConfig0  = 0x3D,     ///< FIFO watermark.
    FifoConfig1,             ///< FIFO mode.
    FifoData,                ///< FIFO data output.
  };

  Bus& _bus;
  uint8_t _register;
  uint8_t _values[6];
  uint8_t _range;
  float _factor_angular_velocity;
  RawData _raw_offset;
  OnErrorCallback _on_error;
  Error _error;

  /**
   * @brief Calculate the factor of the gyroscope.
   */
  void _calculateFactorAngularVelocity();

  /**
   * @brief Set the error.
   *
   * @param error Error.
   */
  void _setError(Error error);
};

// MARK: Imu
class Imu {
  public:
  Imu(Bus& bus_accel, Bus& bus_gyro);

  /**
   * @brief Initialize the BMI088 sensor. Both buses are prepared before initializing the
   * accelerometer and the gyroscope, so a shared SPI bus has both CS pins high.
   *
   * @return true Success.
   * @return false Failure.
   */
  bool init();

  /**
   * @brief Get the accelerometer object.
   *
   * @return Accel& Accelerometer object.
   */
  Accel& accelerometer();

  /**
   * @brief Get the gyroscope object.
   *
   * @return Gyro& Gyroscope object.
   */
  Gyro& gyroscope();

  /**
   * @brief Complementary filter to fuse the accelerometer and gyroscope data.
   *
   * @param last_angle Last angle in degrees.
   * @param accel_angle Accelerometer angle in degrees.
   * @param gyro_rate Gyroscope rate in degrees per second.
   * @param dt Sampling time in seconds.
   * @param alpha Filter coefficient, weight of the gyroscope. Default is 0.98.
   * @return float Filtered angle in degrees.
   */
  static float complementaryFilter(float last_angle, float accel_angle, float gyro_rate, float dt,
    float alpha = 0.98f);

  private:
  Bus& _bus_accel;
  Bus& _bus_gyro;
  Accel _accel;
  Gyro _gyro;
};

} // namespace bmi088
