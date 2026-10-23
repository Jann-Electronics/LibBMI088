#pragma once

#include "bmi088.h"

// Text representation of the enums of the library, useful for logs and debugging. Unused functions
// are discarded by the linker.

namespace bmi088 {

// MARK: Accel
const char* toString(Accel::Bandwidth bw);
const char* toString(Accel::DataRate dr);
const char* toString(Accel::Range range);
const char* toString(Accel::GpioPin pin);
const char* toString(Accel::GpioMode mode);
const char* toString(Accel::GpioBehavior behavior);
const char* toString(Accel::FifoMode mode);
const char* toString(Accel::Error error);

// MARK: Gyro
const char* toString(Gyro::Range range);
const char* toString(Gyro::OdrAndBandwidth bw);
const char* toString(Gyro::Mode mode);
const char* toString(Gyro::GpioPin pin);
const char* toString(Gyro::GpioOutConfig config);
const char* toString(Gyro::GpioMapInterrupt config);
const char* toString(Gyro::FifoMode mode);
const char* toString(Gyro::Error error);

// MARK: BusI2C
const char* toString(BusI2C::Error error);

} // namespace bmi088
