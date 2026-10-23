# LibBMI088

Arduino library for the Bosch **BMI088** 6-axis IMU: a 16-bit accelerometer (±3/6/12/24 g) and a
16-bit gyroscope (±125 to ±2000 °/s), over **I2C** or **SPI**.

- Readings in raw values, milli-g / °/s, and SI units (m/s², rad/s).
- Full configuration of range, data rate, bandwidth and power modes.
- Data ready and FIFO interrupts on the INT1–INT4 pins.
- FIFO buffer on both sensors.
- Offset calibration, self tests, temperature and sensor time.
- Tilt angles from the accelerometer and a complementary filter.
- Errors reported through return values, an error code and an optional callback.

## Contents

- [LibBMI088](#libbmi088)
  - [Contents](#contents)
  - [Installation](#installation)
  - [Wiring](#wiring)
  - [Quick start](#quick-start)
    - [I2C](#i2c)
    - [SPI](#spi)
  - [How the library is organized](#how-the-library-is-organized)
    - [Error handling](#error-handling)
    - [Blocking calls](#blocking-calls)
  - [Accelerometer — `bmi088::Accel`](#accelerometer--bmi088accel)
    - [Initialization and identification](#initialization-and-identification)
    - [Reading data](#reading-data)
    - [Configuration](#configuration)
    - [Power](#power)
    - [Calibration](#calibration)
    - [Interrupt pins (INT1 / INT2)](#interrupt-pins-int1--int2)
    - [FIFO](#fifo)
    - [Self test](#self-test)
    - [Tilt angles](#tilt-angles)
  - [Gyroscope — `bmi088::Gyro`](#gyroscope--bmi088gyro)
    - [Initialization and identification](#initialization-and-identification-1)
    - [Reading data](#reading-data-1)
    - [Configuration](#configuration-1)
    - [Calibration](#calibration-1)
    - [Interrupt pins (INT3 / INT4)](#interrupt-pins-int3--int4)
    - [FIFO](#fifo-1)
    - [Self test](#self-test-1)
  - [Orientation and sensor fusion](#orientation-and-sensor-fusion)
  - [Buses — `BusI2C` and `BusSPI`](#buses--busi2c-and-busspi)
  - [Text representation](#text-representation)
  - [Configuration macros](#configuration-macros)
  - [Examples](#examples)
  - [Notes and limitations](#notes-and-limitations)
  - [License](#license)

## Installation

With PlatformIO, add the repository to `lib_deps` in `platformio.ini`:

```ini
lib_deps =
  https://github.com/JojeNN97/LibBMI088.git
```

The library uses only the Arduino `Wire` and `SPI` libraries.

## Wiring

The BMI088 has two independent sensors inside, each with its own address or chip select.

| Interface | PS pin    | Accelerometer                               | Gyroscope                                   |
| --------- | --------- | ------------------------------------------- | ------------------------------------------- |
| I2C       | To VDDIO  | `0x18` (SDO1 to GND) or `0x19` (SDO1 to VDDIO) | `0x68` (SDO2 to GND) or `0x69` (SDO2 to VDDIO) |
| SPI       | To GND    | Its own CS pin (CSB1)                       | Its own CS pin (CSB2)                       |

Interrupt pins: **INT1** and **INT2** belong to the accelerometer; **INT3** and **INT4** belong to
the gyroscope.

## Quick start

### I2C

```cpp
#include <Arduino.h>
#include <bmi088.h>

bmi088::BusI2C bus_accel(Wire, 0x18);
bmi088::BusI2C bus_gyro(Wire, 0x68);
bmi088::Imu imu(bus_accel, bus_gyro);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!imu.init()) {
    Serial.println("BMI088 not found");
  }
}

void loop() {
  bmi088::ConvertedData accel; // milli-g
  bmi088::ConvertedData gyro;  // degrees per second

  if (imu.accelerometer().getMilliGAcceleration(accel) &&
      imu.gyroscope().getAngularVelocity(gyro)) {
    Serial.printf("Accel: %.1f %.1f %.1f mg\n", accel.x, accel.y, accel.z);
    Serial.printf("Gyro: %.2f %.2f %.2f dps\n", gyro.x, gyro.y, gyro.z);
  }

  delay(100);
}
```

### SPI

```cpp
bmi088::BusSPI bus_accel(SPI, PIN_CS_ACCEL);
bmi088::BusSPI bus_gyro(SPI, PIN_CS_GYRO);
bmi088::Imu imu(bus_accel, bus_gyro);

void setup() {
  SPI.begin();  // must be called before imu.init()
  imu.init();
}
```

Everything else is identical to I2C. The library handles the extra dummy byte that the
accelerometer sends on SPI reads.

## How the library is organized

Everything lives in the `bmi088` namespace and is included with `#include <bmi088.h>`.

| Class               | Purpose                                                               |
| ------------------- | --------------------------------------------------------------------- |
| `bmi088::BusI2C`    | I2C access to one sensor (`TwoWire` + address).                       |
| `bmi088::BusSPI`    | SPI access to one sensor (`SPIClass` + CS pin).                       |
| `bmi088::Accel`     | Accelerometer.                                                        |
| `bmi088::Gyro`      | Gyroscope.                                                            |
| `bmi088::Imu`       | Both sensors together, plus the complementary filter.                 |
| `bmi088::RawData`   | `int16_t x, y, z`: raw register values.                               |
| `bmi088::ConvertedData` | `float x, y, z`: values in physical units.                        |

`Accel` and `Gyro` can be used on their own (for example, only the gyroscope) or through `Imu`,
which gives access to them with `imu.accelerometer()` and `imu.gyroscope()`.

### Error handling

Every method that talks to the sensor returns `bool`: `true` on success, `false` on failure.
When something fails:

- `getError()` returns the last error (`Accel::Error` or `Gyro::Error`), for example
  `InvalidChipID` or `NoGetRawData`.
- If a callback was registered with `onError()`, it is called with the error code.
- `BusI2C::getError()` returns the low level I2C error (NACK, timeout, ...).

```cpp
void onAccelError(bmi088::Accel::Error error) {
  Serial.printf("Accel error: %s\n", bmi088::toString(error)); // needs bmi088Strings.h
}

imu.accelerometer().onError(onAccelError);
```

Use the values passed by reference only when the method returns `true`.

### Blocking calls

Some methods wait with `delay()` because the datasheet requires it:

| Method                    | Approximate time |
| ------------------------- | ---------------- |
| `Accel::init()`           | 56 ms            |
| `Accel::selfTest()`       | 270 ms           |
| `Accel::powerOn()` / `powerOff()` | 50 ms    |
| `Accel::setSelfTestMode()`| 50 ms            |
| `Accel::setPowerMode()`   | 5 ms             |
| `Gyro::init()`, `Gyro::softReset()` | 30 ms  |
| `Gyro::selfTest()`        | up to 100 ms     |
| `calibrateOffset()`       | `samples` × sample period (about 1 s with the defaults at 100 Hz) |

## Accelerometer — `bmi088::Accel`

### Initialization and identification

| Method                                  | Description                                                                 |
| --------------------------------------- | --------------------------------------------------------------------------- |
| `init()`                                | Checks the chip ID, soft resets, enables active mode and powers on.         |
| `isConnected()`                         | `true` if the chip ID is `0x1E`.                                            |
| `getChipID(id)`                         | Reads the chip ID.                                                          |
| `getChipError(fatal, config)`           | Reads the error register (fatal error, invalid configuration).             |
| `softReset()`                           | Restores every register to its default value. The sensor is left off.      |

After `init()` the accelerometer is at **±6 g, 100 Hz, normal bandwidth**, in active mode.

### Reading data

| Method                                   | Unit                | Notes                                         |
| ---------------------------------------- | ------------------- | --------------------------------------------- |
| `getRawData(data)`                       | LSB                 | X, Y and Z in one burst read.                 |
| `getRawXData(x)`, `getRawYData`, `getRawZData` | LSB           | One axis.                                     |
| `getMilliGAcceleration(data)`            | mg                  | Applies the offset and the current range.     |
| `getMilliGXAcceleration(x)`, `Y`, `Z`    | mg                  | One axis.                                     |
| `getAcceleration(data)`                  | m/s²                | Same as milli-g × 9.80665 / 1000.             |
| `getRawDataAndTime(data, time)`          | LSB + ticks         | Data and sensor time from the same sample.    |
| `getSensorTime(time)`                    | ticks of 39.0625 µs | 24-bit counter, wraps every 655.36 s.         |
| `getTemperature(temperature)`            | °C                  | 0.125 °C resolution, updated every 1.28 s.    |
| `getStatus(data_ready)`                  | —                   | `true` when a new sample is available.        |
| `convertToMilliG(raw, data)`             | mg                  | Converts raw data you already have (e.g. FIFO frames). |

### Configuration

| Method                                     | Description                                       |
| ------------------------------------------ | ------------------------------------------------- |
| `setRange(range)` / `getRange(range)`      | Measurement range.                                |
| `setConfig(bw, dr)` / `getConfig(bw, dr)`  | Bandwidth and data rate in one write.             |
| `setBandwidth(bw)` / `getBandwidth(bw)`    | Only the bandwidth.                               |
| `setDataRate(dr)` / `getDataRate(dr)`      | Only the data rate.                               |
| `getAllConfig(config)` / `setAllConfig(config)` | Save and restore the full configuration (`Accel::Config`). |

| `Accel::Range` | Full scale | Resolution   |
| -------------- | ---------- | ------------ |
| `Range3g`      | ±3 g       | 0.0916 mg/LSB |
| `Range6g`      | ±6 g       | 0.183 mg/LSB  |
| `Range12g`     | ±12 g      | 0.366 mg/LSB  |
| `Range24g`     | ±24 g      | 0.732 mg/LSB  |

`Accel::DataRate`: `Odr12_5Hz`, `Odr25Hz`, `Odr50Hz`, `Odr100Hz`, `Odr200Hz`, `Odr400Hz`, `Odr800Hz`,
`Odr1600Hz`.

`Accel::Bandwidth` selects the low-pass filter: `Normal` (widest), `Osr2` (2× oversampling) or
`Osr4` (4× oversampling, less noise and less bandwidth). The resulting cutoff depends on the data
rate; see the datasheet table.

### Power

| Method                                     | Description                                              |
| ------------------------------------------ | -------------------------------------------------------- |
| `powerOn()` / `powerOff()`                 | Turns the accelerometer on or off (`ACC_PWR_CTRL`).      |
| `setPowerMode(mode)` / `getPowerMode(mode)`| `Mode::Active` or `Mode::Suspend` (`ACC_PWR_CONF`).      |

### Calibration

| Method                       | Description                                                                 |
| ---------------------------- | --------------------------------------------------------------------------- |
| `calibrateOffset(samples)`   | Averages `samples` readings (100 by default) at rest. **The sensor must be flat with Z up**: Z keeps 1 g. |
| `setRawOffset(offset)` / `getRawOffset(offset)` | Set or read the offset manually, for example to store it in flash. |

The offset is subtracted in every milli-g and m/s² conversion. It is cleared by `init()`.

### Interrupt pins (INT1 / INT2)

| Method                                         | Description                                                    |
| ---------------------------------------------- | -------------------------------------------------------------- |
| `pinConfig(pin, mode, behavior)`               | `GpioMode::Output`, `Input` or `Disabled`; `PushPull` or `OpenDrain`. |
| `getPinConfig(pin, mode, behavior)`            | Reads the pin configuration.                                   |
| `setPinActiveLevel(pin, level)` / `getPinActiveLevel` | Interrupt polarity: `GpioLevel::High` or `Low`.         |
| `setMapInterrupt(int1, int2)` / `getMapInterrupt`| Routes the data ready interrupt to INT1 and/or INT2.         |
| `setMapFifoInterrupt(pin, watermark, full)`    | Routes the FIFO watermark / full interrupts to a pin.          |
| `getInterruptStatus(data_ready)`               | Reads the data ready interrupt flag (reading clears it).      |
| `clearInterrupt()`                             | Clears the data ready interrupt.                               |

Example, data ready on INT1 as an active-high push-pull output:

```cpp
accel.pinConfig(bmi088::Accel::GpioPin::Int1,
  bmi088::Accel::GpioMode::Output,
  bmi088::Accel::GpioBehavior::PushPull);
accel.setPinActiveLevel(bmi088::Accel::GpioPin::Int1, bmi088::Accel::GpioLevel::High);
accel.setMapInterrupt(true, false);

attachInterrupt(digitalPinToInterrupt(PIN_INT1), onAccelReady, RISING);
```

### FIFO

The accelerometer FIFO holds 1024 bytes. Each sample uses 7 bytes, so it stores about 146 samples.

| Method                                    | Description                                                          |
| ----------------------------------------- | -------------------------------------------------------------------- |
| `setFifoMode(mode)` / `getFifoMode(mode)` | `FifoMode::Fifo` (stops when full) or `FifoMode::Stream` (overwrites the oldest). |
| `enableFifo(accel, int1, int2)`           | Selects what is stored: acceleration and/or the INT1/INT2 input levels. |
| `setFifoWatermark(bytes)` / `getFifoWatermark` | Level that triggers the watermark interrupt.                   |
| `setFifoDownsampling(exponent)`           | Stores one sample every 2^exponent (0 to 7).                         |
| `getFifoLength(bytes)`                    | Bytes stored.                                                        |
| `readFifo(frames, max_frames, frames_read)` | Reads the acceleration samples as `RawData`.                       |
| `readFifoData(buffer, length)` + `parseFifo(...)` | Low level: read raw bytes and parse them yourself.           |
| `flushFifo()`                             | Discards the stored data.                                            |

```cpp
accel.setFifoMode(bmi088::Accel::FifoMode::Stream);
accel.enableFifo(true);

bmi088::RawData frames[64];
size_t count;
if (accel.readFifo(frames, 64, count)) {
  for (size_t i = 0; i < count; ++i) {
    bmi088::ConvertedData mg;
    accel.convertToMilliG(frames[i], mg);
  }
}
```

### Self test

| Method                                 | Description                                                              |
| -------------------------------------- | ------------------------------------------------------------------------ |
| `selfTest(x_pass, y_pass, z_pass)`     | Runs the datasheet procedure and restores the previous configuration.   |
| `getSelfTestData(data)`                | Difference measured in the last self test, in mg (minimum: 1000 mg on X and Y, 500 mg on Z). |
| `setSelfTestMode(mode)` / `getSelfTestMode` | Low level control of the self test.                                  |

### Tilt angles

`Accel::getAbsoluteXAngle(data)` and `Accel::getAbsoluteYAngle(data)` (static) return the angles
around X (roll) and Y (pitch) in degrees, computed from the direction of gravity. Any acceleration
unit works. See [Orientation and sensor fusion](#orientation-and-sensor-fusion).

## Gyroscope — `bmi088::Gyro`

### Initialization and identification

| Method                | Description                                     |
| --------------------- | ----------------------------------------------- |
| `init()`              | Checks the chip ID and soft resets.             |
| `isConnected()`       | `true` if the chip ID is `0x0F`.                |
| `getChipID(id)`       | Reads the chip ID.                              |
| `softReset()`         | Restores every register to its default value.  |

After `init()` the gyroscope is at **±2000 °/s, 2000 Hz with a 532 Hz filter**, in normal mode.

### Reading data

The gyroscope measures **angular velocity** (how fast it rotates), not the angle. At rest it reads
about 0 whatever its orientation.

| Method                                    | Unit  | Notes                                       |
| ----------------------------------------- | ----- | ------------------------------------------- |
| `getRawData(data)`                        | LSB   | X, Y and Z in one burst read.               |
| `getRawXData(x)`, `getRawYData`, `getRawZData` | LSB | One axis.                                 |
| `getAngularVelocity(data)`                | °/s   | Applies the offset and the current range.   |
| `getAngularVelocityX(x)`, `Y`, `Z`        | °/s   | One axis.                                   |
| `getAngularVelocityRad(data)`             | rad/s |                                             |
| `getStatus(data_ready)`                   | —     | `true` when a new sample is available.      |
| `convertToDps(raw, data)`                 | °/s   | Converts raw data you already have (e.g. FIFO frames). |

### Configuration

| Method                                                | Description                                   |
| ----------------------------------------------------- | --------------------------------------------- |
| `setRange(range)` / `getRange(range)`                 | Measurement range.                            |
| `setDataRateAndBandwidth(odr)` / `getDataRateAndBandwidth` | Data rate and filter, set together.      |
| `setPowerMode(mode)` / `getPowerMode(mode)`           | `Mode::Normal`, `Suspend` or `DeepSuspend`.   |
| `getAllConfig(config)`                                | Reads the full configuration (`Gyro::Config`). |

| `Gyro::Range` | Full scale | Resolution      |
| ------------- | ---------- | --------------- |
| `Dps2000`     | ±2000 °/s  | 0.061 °/s/LSB   |
| `Dps1000`     | ±1000 °/s  | 0.0305 °/s/LSB  |
| `Dps500`      | ±500 °/s   | 0.0153 °/s/LSB  |
| `Dps250`      | ±250 °/s   | 0.0076 °/s/LSB  |
| `Dps125`      | ±125 °/s   | 0.0038 °/s/LSB  |

| `Gyro::OdrAndBandwidth` | Data rate | Filter |
| ----------------------- | --------- | ------ |
| `Odr2000HzFilter532Hz`  | 2000 Hz   | 532 Hz |
| `Odr2000HzFilter230Hz`  | 2000 Hz   | 230 Hz |
| `Odr1000HzFilter116Hz`  | 1000 Hz   | 116 Hz |
| `Odr400HzFilter47Hz`    | 400 Hz    | 47 Hz  |
| `Odr200HzFilter23Hz`    | 200 Hz    | 23 Hz  |
| `Odr200HzFilter64Hz`    | 200 Hz    | 64 Hz  |
| `Odr100HzFilter12Hz`    | 100 Hz    | 12 Hz  |
| `Odr100HzFilter32Hz`    | 100 Hz    | 32 Hz  |

### Calibration

| Method                       | Description                                                       |
| ---------------------------- | ----------------------------------------------------------------- |
| `calibrateOffset(samples)`   | Averages `samples` readings (100 by default). **The sensor must not move.** |
| `setRawOffset(offset)` / `getRawOffset(offset)` | Set or read the bias manually.                  |

Calibrating the gyroscope bias is the most effective way to reduce drift when integrating angles.

### Interrupt pins (INT3 / INT4)

The gyroscope pins are always outputs.

| Method                                          | Description                                              |
| ----------------------------------------------- | -------------------------------------------------------- |
| `pinConfig(pin, conf)` / `getPinConfig(pin, conf)` | `GpioOutConfig::PushPull` or `OpenDrain` for one pin. |
| `setPinsConfig(int3, int4)` / `getPinsConfig`   | Both pins at once.                                       |
| `setPinActiveLevel(pin, level)` / `getPinActiveLevel` | Interrupt polarity.                                |
| `enableInterrupt(enable)` / `isEnabledInterrupt` | Enables the data ready interrupt.                       |
| `setMapInterrupt(map)` / `getMapInterrupt(map)` | Routes data ready: `NoInterrupt`, `InterruptOnInt3`, `InterruptOnInt4` or `InterruptOnBoth`. |
| `enableFifoInterrupt(enable)`                   | Enables the FIFO watermark interrupt.                    |
| `setMapFifoInterrupt(int3, int4)`               | Routes the FIFO interrupt to INT3 and/or INT4.           |

### FIFO

The gyroscope FIFO stores up to 100 samples of 6 bytes.

| Method                                      | Description                                                      |
| ------------------------------------------- | ---------------------------------------------------------------- |
| `setFifoMode(mode)` / `getFifoMode(mode)`   | `FifoMode::Fifo` or `FifoMode::Stream`. Writing the mode clears the FIFO. |
| `setFifoWatermark(frames)` / `getFifoWatermark` | Watermark level in samples.                                 |
| `getFifoStatus(frames, overrun)`            | Samples stored and whether samples were lost.                    |
| `readFifo(frames, max_frames, frames_read)` | Reads the samples as `RawData`; convert them with `convertToDps()`. |
| `flushFifo()`                               | Discards the stored data.                                        |

### Self test

| Method                                | Description                                          |
| ------------------------------------- | ---------------------------------------------------- |
| `selfTest(pass)`                      | Runs the built-in self test and waits for the result. |
| `startSelfTest()` / `getSelfTestResult(ready, pass)` | Non-blocking version: start, then poll. |

## Orientation and sensor fusion

The library offers three ways to estimate orientation. Each has trade-offs:

| Method | Function | Pros | Cons |
| ------ | -------- | ---- | ---- |
| Accelerometer | `Accel::getAbsoluteXAngle()`, `getAbsoluteYAngle()` | Absolute angle, no drift. | Noisy when the sensor moves or vibrates. Cannot measure yaw. |
| Gyroscope integration | `Gyro::getRelativeEulerAngle(last, rate, dt)` | Smooth, fast response. | Drifts over time; the angle is relative to the start. |
| Complementary filter | `Imu::complementaryFilter(last, accel_angle, gyro_rate, dt, alpha)` | Combines both: smooth and without drift. | Roll and pitch only. |

```cpp
float angle_x = 0;

void loop() {
  bmi088::ConvertedData accel, gyro;
  imu.accelerometer().getMilliGAcceleration(accel);
  imu.gyroscope().getAngularVelocity(gyro);

  float accel_angle = bmi088::Accel::getAbsoluteXAngle(accel);
  angle_x = bmi088::Imu::complementaryFilter(angle_x, accel_angle, gyro.x, dt); // alpha = 0.98
}
```

`alpha` is the weight of the gyroscope (0.98 by default): higher values give a smoother angle,
lower values correct drift faster. Measure `dt` (in seconds) between samples with `micros()`.

Yaw (rotation around the vertical axis) can only be integrated from the gyroscope and always
drifts: the BMI088 has no magnetometer.

## Buses — `BusI2C` and `BusSPI`

| Class / method                               | Description                                                   |
| -------------------------------------------- | ------------------------------------------------------------- |
| `BusI2C(wire, address)`                      | `Wire.begin()` must be called by the user.                    |
| `BusI2C::getError()`                         | Last I2C error: `NACKAddress`, `NACKData`, `Timeout`, ...     |
| `BusSPI(spi, pin_cs, clock)`                 | `clock` is 10 MHz by default (the maximum), SPI mode 0. `SPI.begin()` must be called by the user. |
| `BusSPI::setCS(pin)`                         | Changes the chip select pin.                                  |
| `BusSPI::onBeforeTransfer(cb)` / `onAfterTransfer(cb)` | Callbacks around each transfer, e.g. to drive CS through an IO expander (use `pin_cs = -1`). |

If the accelerometer and the gyroscope share the SPI bus and you use them **without** `Imu`, call
`begin()` on both buses before `init()` so both CS pins are high.

To use another transport, inherit from `bmi088::Bus` and implement `writeRegister()`,
`readRegister()` and `readBlock()`.

## Text representation

Include `bmi088Strings.h` to get `bmi088::toString()` for every enum of the library: configuration
values and errors. Unused functions are discarded by the linker.

```cpp
#include <bmi088Strings.h>

Serial.println(bmi088::toString(imu.accelerometer().getError())); // "Invalid chip ID"
Serial.println(bmi088::toString(bmi088::Gyro::Range::Dps500));     // "500 dps"
```

## Configuration macros

| Macro                     | Default | Description                                                         |
| ------------------------- | ------- | ------------------------------------------------------------------- |
| `BMI088_FIFO_BUFFER_SIZE` | `126`   | Maximum bytes per FIFO burst read, allocated on the stack by `readFifo()`. With I2C it must fit in the `Wire` buffer: use `32` on AVR. |

Define it in `build_flags`, for example `-DBMI088_FIFO_BUFFER_SIZE=32`.

## Examples

| Example                                               | Shows                                                       |
| ----------------------------------------------------- | ----------------------------------------------------------- |
| [i2c_accel](examples/i2c_accel/i2c_accel.cpp)         | Accelerometer menu: data, temperature, config, pins, calibration, self test. |
| [i2c_gyro](examples/i2c_gyro/i2c_gyro.cpp)            | Gyroscope data, angle integration, calibration, self test.  |
| [i2c_both](examples/i2c_both/i2c_both.cpp)            | Both sensors and the complementary filter.                  |
| [i2c_interrupt](examples/i2c_interrupt/i2c_interrupt.cpp) | Data ready interrupts with `attachInterrupt()`.         |
| [i2c_fifo](examples/i2c_fifo/i2c_fifo.cpp)            | Reading both FIFOs in stream mode.                          |
| [spi_both](examples/spi_both/spi_both.cpp)            | Both sensors over SPI in SI units.                          |

The examples are written for an ESP32-S3; adjust the pin numbers for your board. To build one with
PlatformIO, select it with `src_dir` in [platformio.ini](platformio.ini).

## Notes and limitations

- `init()` soft resets the sensor: configure it **after** `init()`.
- `setPinActiveLevel()` sets the interrupt polarity. With no interrupt mapped, an output pin rests at
  the *opposite* level (active high → pin low).
- The data synchronization mode of the BMI088 is not supported: it requires uploading Bosch's
  configuration file to the accelerometer.
- Methods are not thread safe and must not be called from an interrupt handler: set a flag in the
  handler and read the sensor in `loop()`, as in the `i2c_interrupt` example.

## License

Released under the [MIT License](LICENSE).
