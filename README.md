# LSM9DS1 driver

## Description

A C++ driver for the LSM9DS1 inertial module manufactured by STMicroelectronics and used in the Raspberry Pi Sense Hat
board.

## License

This piece of software is available under the terms of the [MIT License](LICENSE).

## Changelog

**r1.0.0**

* Initial release.

## Example

```cpp
#include "LSM9DS1.h"
#include <iostream>
#include <tuple>

int main(int argc, char** argv)
{
    LSM9DS1 module("/dev/i2c-1");

    std::tuple<double, double, double> angularRate = module.getAngularRate();
	std::cout << "Angular rate - X: " << std::get<0>(angularRate) << " deg/s - Y: " << std::get<1>(angularRate) << " deg/s - Z: " << std::get<2>(angularRate) << " deg/s" << std::endl;

	std::tuple<double, double, double> linearAcceleration = module.getLinearAcceleration();
	std::cout << "Linear acceleration - X: " << std::get<0>(linearAcceleration) << " G - Y: " << std::get<1>(linearAcceleration) << " G - Z: " << std::get<2>(linearAcceleration) << " G" << std::endl;

	std::tuple<double, double, double> magneticField = module.getMagneticField();
	std::cout << "Magnetic field - X: " << std::get<0>(magneticField) << " G - Y: " << std::get<1>(magneticField) << " G - Z: " << std::get<2>(magneticField) << " G" << std::endl;

    return 0;
}
```

## Documentation

### `LSM9DS1::LSM9DS1(const char* devicePath, uint8_t accelerometerGyroscopeDeviceAddress = LSM9DS1_ACCELEROMETER_GYROSCOPE_DEVICE_ADDRESS, uint8_t magnetometerDeviceAddress = LSM9DS1_MAGNETOMETER_DEVICE_ADDRESS)` 
---
Creates a LSM9DS1 object.

The *devicePath* parameter corresponds to the path of the I2C bus on the system.  
The optional parameter *accelerometerGyroscopeDeviceAddress* allows you to override the address of the accelerometer/gyroscope part of the module, in case you are using a standalone module or a modified version of the Sense Hat board.
The optional parameter *magnetometerDeviceAddress* allows you to override the address of the magnetometer part of the module, in case you are using a standalone module or a modified version of the Sense Hat board.

Upon creation of the object, the module is configured with the following options:

|Option|Value|
|--|--|
|Accelerometer data rate|LSM9DS1_ACCELEROMETER_DATA_RATE_10HZ|
|Accelerometer scale|LSM9DS1_ACCELEROMETER_SCALE_2G|
|Gyroscope data rate|LSM9DS1_GYROSCOPE_DATA_RATE_14_9_HZ|
|Gyroscope scale|LSM9DS1_GYROSCOPE_SCALE_245_DPS|
|Magnetometer data rate|LSM9DS1_MAGNETOMETER_DATA_RATE_1_25_HZ|
|Magnetometer scale|LSM9DS1_MAGNETOMETER_SCALE_4G|

Throws an `std::runtime_error` exception if the communication with the module fails for any reason.

### `LSM9DS1::~LSM9DS1()`
___
Destroys the object, closing the connection to the module.

### `std::tuple<double x, double y, double z> LSM9DS1::getAngularRate()`
___
Returns the angular rate value measured by the module, in degrees per second.

### `std::tuple<double x, double y, double z> LSM9DS1::getLinearAcceleration()`
___
Returns the linear acceleration value measured by the module, in multiples of the gravitational constant.

### `std::tuple<double x, double y, double z> LSM9DS1::getMagneticField()`
___
Returns the magnetic field value measured by the module, in gauss.

### `double LSM9DS1::getTemperature()`
___
Returns the temperature value measured by the module, in Celsius degrees.

### `void LSM9DS1::setAccelerometerDataRate(LSM9DS1AccelerometerDataRate_t dataRate)`
___
Sets the data rate of the accelerometer part of the module.

The *dataRate* parameter can take one of the following values:

|Value|Measurement interval|
|--|--|
|LSM9DS1_ACCELEROMETER_DATA_RATE_POWER_DOWN|-|
|LSM9DS1_ACCELEROMETER_DATA_RATE_10HZ|100 ms|
|LSM9DS1_ACCELEROMETER_DATA_RATE_50HZ|20 ms|
|LSM9DS1_ACCELEROMETER_DATA_RATE_119HZ|8.4 ms|
|LSM9DS1_ACCELEROMETER_DATA_RATE_238HZ|4.2 ms|
|LSM9DS1_ACCELEROMETER_DATA_RATE_476HZ|2.1 ms|
|LSM9DS1_ACCELEROMETER_DATA_RATE_952HZ|1.05 ms|

### `void LSM9DS1::setAccelerometerScale(LSM9DS1AccelerometerScale_t scale)`
___
Sets the scale of the accelerometer part of the module.

The *scale* parameter can take one of the following values:

|Value|Resolution|
|--|--|
|LSM9DS1_ACCELEROMETER_SCALE_2G|0.61 x 10e-5 G|
|LSM9DS1_ACCELEROMETER_SCALE_4G|1.22 x 10e-5 G|
|LSM9DS1_ACCELEROMETER_SCALE_8G|2.44 x 10e-5 G|
|LSM9DS1_ACCELEROMETER_SCALE_16G|4.88 x 10e-5 G|

### `void LSM9DS1::setGyroscopeDataRate(LSM9DS1GyroscopeDataRate_t dataRate)`
___
Sets the data rate of the gyroscope part of the module.

The *dataRate* parameter can take one of the following values:

|Value|Measurement interval|
|--|--|
|LSM9DS1_GYROSCOPE_DATA_RATE_POWER_DOWN|-|
|LSM9DS1_GYROSCOPE_DATA_RATE_14_9_HZ|67.1 ms|
|LSM9DS1_GYROSCOPE_DATA_RATE_59_5_HZ|16.8 ms|
|LSM9DS1_GYROSCOPE_DATA_RATE_119_HZ|8.4 ms|
|LSM9DS1_GYROSCOPE_DATA_RATE_238_HZ|4.2 ms|
|LSM9DS1_GYROSCOPE_DATA_RATE_476_HZ|2.1 ms|
|LSM9DS1_GYROSCOPE_DATA_RATE_952_HZ|1.05 ms|

### `void LSM9DS1::setGyroscopeScale(LSM9DS1GyroscopeScale_t scale)`
___
Sets the scale of the gyroscope part of the module.

The *scale* parameter can take one of the following values:

|Value|Resolution|
|--|--|
|LSM9DS1_GYROSCOPE_SCALE_245_DPS|0.74 x 10e-3 deg.s-1|
|LSM9DS1_GYROSCOPE_SCALE_500_DPS|1.52 x 10e-3 deg.s-1|
|LSM9DS1_GYROSCOPE_SCALE_2000_DPS|6.1 x 10e-3 deg.s-1|

### `void LSM9DS1::setMagnetometerDataRate(LSM9DS1MagnetometerDataRate_t dataRate)`
___
Sets the data rate of the magnetometer part of the module.

The *dataRate* parameter can take one of the following values:

|Value|Measurement interval|
|--|--|
|LSM9DS1_MAGNETOMETER_DATA_RATE_0_625_HZ|1600 ms|
|LSM9DS1_MAGNETOMETER_DATA_RATE_1_25_HZ|800 ms|
|LSM9DS1_MAGNETOMETER_DATA_RATE_2_5_HZ|400 ms|
|LSM9DS1_MAGNETOMETER_DATA_RATE_5_HZ|200 ms|
|LSM9DS1_MAGNETOMETER_DATA_RATE_10_HZ|100 ms|
|LSM9DS1_MAGNETOMETER_DATA_RATE_20_HZ|50 ms|
|LSM9DS1_MAGNETOMETER_DATA_RATE_40_HZ|25 ms|
|LSM9DS1_MAGNETOMETER_DATA_RATE_80_HZ|12.5ms|

### `void LSM9DS1::setMagnetometerPerformanceMode(LSM9DS1MagnetometerPerformanceMode_t performanceMode)`
___
Sets the performance mode of the magnetometer part of the module.

The *performanceMode* parameter can take one of the following values:

|Value|
|--|
|LSM9DS1_MAGNETOMETER_PERFORMANCE_MODE_LOW_POWER|
|LSM9DS1_MAGNETOMETER_PERFORMANCE_MODE_MEDIUM_PERFORMANCE|
|LSM9DS1_MAGNETOMETER_PERFORMANCE_MODE_HIGH_PERFORMANCE|
|LSM9DS1_MAGNETOMETER_PERFORMANCE_MODE_ULTRA_HIGH_PERFORMANCE|

### `void LSM9DS1::setMagnetometerScale(LSM9DS1MagnetometerScale_t scale)`
___
Sets the scale of the magnetometer part of the module.

The *scale* parameter can take one of the following values:

|Value|Resolution|
|--|--|
|LSM9DS1_MAGNETOMETER_SCALE_4G|1.22 x 10e-5 G|
|LSM9DS1_MAGNETOMETER_SCALE_8G|2.44 x 10e-5 G|
|LSM9DS1_MAGNETOMETER_SCALE_12G|3.66 x 10e-5 G|
|LSM9DS1_MAGNETOMETER_SCALE_16G|4.88 x 10e-5 G|

## Useful resources

[LSM9DS1 datasheet](https://www.st.com/resource/en/datasheet/lsm9ds1.pdf)
