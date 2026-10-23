#include "bmi088Strings.h"

namespace bmi088 {

// MARK: Accel

const char* toString(Accel::Bandwidth bw) {
  switch (bw) {
    case Accel::Bandwidth::Osr4: return "OSR4";
    case Accel::Bandwidth::Osr2: return "OSR2";
    case Accel::Bandwidth::Normal: return "Normal";
    default: return "Unknown";
  }
}

const char* toString(Accel::DataRate dr) {
  switch (dr) {
    case Accel::DataRate::Odr12_5Hz: return "12.5Hz";
    case Accel::DataRate::Odr25Hz: return "25Hz";
    case Accel::DataRate::Odr50Hz: return "50Hz";
    case Accel::DataRate::Odr100Hz: return "100Hz";
    case Accel::DataRate::Odr200Hz: return "200Hz";
    case Accel::DataRate::Odr400Hz: return "400Hz";
    case Accel::DataRate::Odr800Hz: return "800Hz";
    case Accel::DataRate::Odr1600Hz: return "1600Hz";
    default: return "Unknown";
  }
}

const char* toString(Accel::Range range) {
  switch (range) {
    case Accel::Range::Range3g: return "3g";
    case Accel::Range::Range6g: return "6g";
    case Accel::Range::Range12g: return "12g";
    case Accel::Range::Range24g: return "24g";
    default: return "Unknown";
  }
}

const char* toString(Accel::GpioPin pin) {
  switch (pin) {
    case Accel::GpioPin::Int1: return "INT1";
    case Accel::GpioPin::Int2: return "INT2";
    default: return "Unknown";
  }
}

const char* toString(Accel::GpioMode mode) {
  switch (mode) {
    case Accel::GpioMode::Disabled: return "Disabled";
    case Accel::GpioMode::Output: return "Output";
    case Accel::GpioMode::Input: return "Input";
    default: return "Unknown";
  }
}

const char* toString(Accel::GpioBehavior behavior) {
  switch (behavior) {
    case Accel::GpioBehavior::PushPull: return "Push Pull";
    case Accel::GpioBehavior::OpenDrain: return "Open Drain";
    default: return "Unknown";
  }
}

const char* toString(Accel::Error error) {
  switch (error) {
    case Accel::Error::None: return "No error";
    case Accel::Error::NoGetChipID: return "Error getting chip ID";
    case Accel::Error::NoGetChipError: return "Error getting chip error";
    case Accel::Error::NoGetStatus: return "Error getting status";
    case Accel::Error::NoGetRawData: return "Error getting raw data";
    case Accel::Error::NoGetRawXData: return "Error getting raw X data";
    case Accel::Error::NoGetRawYData: return "Error getting raw Y data";
    case Accel::Error::NoGetRawZData: return "Error getting raw Z data";
    case Accel::Error::NoGetTime: return "Error getting time";
    case Accel::Error::NoGetTemperature: return "Error getting temperature";
    case Accel::Error::NoGetConfig: return "Error getting config";
    case Accel::Error::NoGetRange: return "Error getting range";
    case Accel::Error::NoGetInt1PinConfig: return "Error getting INT1 pin config";
    case Accel::Error::NoGetInt2PinConfig: return "Error getting INT2 pin config";
    case Accel::Error::NoGetInt1PinLevel: return "Error getting INT1 pin level";
    case Accel::Error::NoGetInt2PinLevel: return "Error getting INT2 pin level";
    case Accel::Error::NoGetMapInterrupt: return "Error getting map interrupt";
    case Accel::Error::NoGetSelfTestMode: return "Error getting self test mode";
    case Accel::Error::NoGetPowerMode: return "Error getting power mode";
    case Accel::Error::NoClearInterrupt: return "Error clearing interrupt";
    case Accel::Error::NoSetConfig: return "Error setting config";
    case Accel::Error::NoSetRange: return "Error setting range";
    case Accel::Error::NoSetInt1PinConfig: return "Error setting INT1 pin config";
    case Accel::Error::NoSetInt2PinConfig: return "Error setting INT2 pin config";
    case Accel::Error::NoSetInt1PinLevel: return "Error setting INT1 pin level";
    case Accel::Error::NoSetInt2PinLevel: return "Error setting INT2 pin level";
    case Accel::Error::NoSetMapInterrupt: return "Error setting map interrupt";
    case Accel::Error::NoSetSelfTestMode: return "Error setting self test mode";
    case Accel::Error::NoSetPowerMode: return "Error setting power mode";
    case Accel::Error::NoSetPowerOn: return "Error setting power on";
    case Accel::Error::NoSetPowerOff: return "Error setting power off";
    case Accel::Error::InvalidChipID: return "Invalid chip ID";
    case Accel::Error::NoSoftReset: return "Error on soft reset";
    case Accel::Error::NoGetInterruptStatus: return "Error getting interrupt status";
    case Accel::Error::NoGetFifoLength: return "Error getting FIFO length";
    case Accel::Error::NoGetFifoData: return "Error getting FIFO data";
    case Accel::Error::NoGetFifoMode: return "Error getting FIFO mode";
    case Accel::Error::NoGetFifoWatermark: return "Error getting FIFO watermark";
    case Accel::Error::NoSetFifoMode: return "Error setting FIFO mode";
    case Accel::Error::NoSetFifoConfig: return "Error setting FIFO config";
    case Accel::Error::NoSetFifoWatermark: return "Error setting FIFO watermark";
    case Accel::Error::NoSetFifoDownsampling: return "Error setting FIFO downsampling";
    case Accel::Error::NoSetMapFifoInterrupt: return "Error setting FIFO map interrupt";
    default: return "Unknown error";
  }
}

const char* toString(Accel::FifoMode mode) {
  switch (mode) {
    case Accel::FifoMode::Fifo: return "FIFO";
    case Accel::FifoMode::Stream: return "Stream";
    default: return "Unknown";
  }
}

// MARK: Gyro

const char* toString(Gyro::Range range) {
  switch (range) {
    case Gyro::Range::Dps2000: return "2000 dps";
    case Gyro::Range::Dps1000: return "1000 dps";
    case Gyro::Range::Dps500: return "500 dps";
    case Gyro::Range::Dps250: return "250 dps";
    case Gyro::Range::Dps125: return "125 dps";
    default: return "Unknown";
  }
}

const char* toString(Gyro::OdrAndBandwidth bw) {
  switch (bw) {
    case Gyro::OdrAndBandwidth::Odr2000HzFilter532Hz: return "Odr: 2000Hz - Bw: 532Hz";
    case Gyro::OdrAndBandwidth::Odr2000HzFilter230Hz: return "Odr: 2000Hz - Bw: 230Hz";
    case Gyro::OdrAndBandwidth::Odr1000HzFilter116Hz: return "Odr: 1000Hz - Bw: 116Hz";
    case Gyro::OdrAndBandwidth::Odr400HzFilter47Hz: return "Odr: 400Hz - Bw: 47Hz";
    case Gyro::OdrAndBandwidth::Odr200HzFilter23Hz: return "Odr: 200Hz - Bw: 23Hz";
    case Gyro::OdrAndBandwidth::Odr100HzFilter12Hz: return "Odr: 100Hz - Bw: 12Hz";
    case Gyro::OdrAndBandwidth::Odr200HzFilter64Hz: return "Odr: 200Hz - Bw: 64Hz";
    case Gyro::OdrAndBandwidth::Odr100HzFilter32Hz: return "Odr: 100Hz - Bw: 32Hz";
    default: return "Unknown";
  }
}

const char* toString(Gyro::Mode mode) {
  switch (mode) {
    case Gyro::Mode::Normal: return "Normal";
    case Gyro::Mode::Suspend: return "Suspend";
    case Gyro::Mode::DeepSuspend: return "Deep Suspend";
    default: return "Unknown";
  }
}

const char* toString(Gyro::GpioPin pin) {
  switch (pin) {
    case Gyro::GpioPin::Int3: return "INT3";
    case Gyro::GpioPin::Int4: return "INT4";
    default: return "Unknown";
  }
}

const char* toString(Gyro::GpioOutConfig config) {
  switch (config) {
    case Gyro::GpioOutConfig::PushPull: return "Push Pull";
    case Gyro::GpioOutConfig::OpenDrain: return "Open Drain";
    default: return "Unknown";
  }
}

const char* toString(Gyro::GpioMapInterrupt config) {
  switch (config) {
    case Gyro::GpioMapInterrupt::NoInterrupt: return "No Interrupt";
    case Gyro::GpioMapInterrupt::InterruptOnInt3: return "Interrupt on INT3";
    case Gyro::GpioMapInterrupt::InterruptOnInt4: return "Interrupt on INT4";
    case Gyro::GpioMapInterrupt::InterruptOnBoth: return "Interrupt on both";
    default: return "Unknown";
  }
}

const char* toString(Gyro::Error error) {
  switch (error) {
    case Gyro::Error::None: return "No error";
    case Gyro::Error::NoGetChipID: return "Error getting chip ID";
    case Gyro::Error::NoGetRawData: return "Error getting raw data";
    case Gyro::Error::NoGetRawXData: return "Error getting raw X data";
    case Gyro::Error::NoGetRawYData: return "Error getting raw Y data";
    case Gyro::Error::NoGetRawZData: return "Error getting raw Z data";
    case Gyro::Error::NoGetRange: return "Error getting range";
    case Gyro::Error::NoGetOdrAndBandwidth: return "Error getting ODR and bandwidth";
    case Gyro::Error::NoGetPowerMode: return "Error getting power mode";
    case Gyro::Error::NoGetInterruptControl: return "Error getting interrupt control";
    case Gyro::Error::NoGetPinsConfig: return "Error getting pins config";
    case Gyro::Error::NoGetInt3Config: return "Error getting INT3 config";
    case Gyro::Error::NoGetInt4Config: return "Error getting INT4 config";
    case Gyro::Error::NoGetInt3Level: return "Error getting INT3 level";
    case Gyro::Error::NoGetInt4Level: return "Error getting INT4 level";
    case Gyro::Error::NoGetMapInterrupt: return "Error getting map interrupt";
    case Gyro::Error::NoGetSelfTestResult: return "Error getting self test result";
    case Gyro::Error::NoSetRange: return "Error setting range";
    case Gyro::Error::NoSetOdrAndBandwidth: return "Error setting ODR and bandwidth";
    case Gyro::Error::NoSetPowerMode: return "Error setting power mode";
    case Gyro::Error::NoSetInterruptControl: return "Error setting interrupt control";
    case Gyro::Error::NoSetPinsConfig: return "Error setting pins config";
    case Gyro::Error::NoSetInt3Config: return "Error setting INT3 config";
    case Gyro::Error::NoSetInt4Config: return "Error setting INT4 config";
    case Gyro::Error::NoSetInt3Level: return "Error setting INT3 level";
    case Gyro::Error::NoSetInt4Level: return "Error setting INT4 level";
    case Gyro::Error::NoSetMapInterrupt: return "Error setting map interrupt";
    case Gyro::Error::NoStartSelfTest: return "Error starting self test";
    case Gyro::Error::InvalidChipID: return "Invalid chip ID";
    case Gyro::Error::NoSoftReset: return "Error on soft reset";
    case Gyro::Error::NoGetStatus: return "Error getting status";
    case Gyro::Error::NoGetFifoStatus: return "Error getting FIFO status";
    case Gyro::Error::NoGetFifoData: return "Error getting FIFO data";
    case Gyro::Error::NoGetFifoMode: return "Error getting FIFO mode";
    case Gyro::Error::NoGetFifoWatermark: return "Error getting FIFO watermark";
    case Gyro::Error::NoSetFifoMode: return "Error setting FIFO mode";
    case Gyro::Error::NoSetFifoWatermark: return "Error setting FIFO watermark";
    case Gyro::Error::NoSetFifoInterrupt: return "Error setting FIFO interrupt";
    case Gyro::Error::NoSetMapFifoInterrupt: return "Error setting FIFO map interrupt";
    case Gyro::Error::SelfTestTimeout: return "Self test timeout";
    default: return "Unknown error";
  }
}

const char* toString(Gyro::FifoMode mode) {
  switch (mode) {
    case Gyro::FifoMode::Fifo: return "FIFO";
    case Gyro::FifoMode::Stream: return "Stream";
    default: return "Unknown";
  }
}

// MARK: BusI2C

const char* toString(BusI2C::Error error) {
  switch (error) {
    case BusI2C::Error::Success: return "Success";
    case BusI2C::Error::DataTooLong: return "Data too long to fit in transmit buffer";
    case BusI2C::Error::NACKAddress: return "Received NACK on transmit of address";
    case BusI2C::Error::NACKData: return "Received NACK on transmit of data";
    case BusI2C::Error::OtherError: return "Other error";
    case BusI2C::Error::Timeout: return "Timeout";
    case BusI2C::Error::InvalidSize: return "Invalid size";
    default: return "Unknown error";
  }
}

} // namespace bmi088
