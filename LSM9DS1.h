#ifndef LSM9DS1_H
#define LSM9DS1_H

#include <cmath>
#include <stdexcept>
#include <tuple>

#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "LSM9DS1_Registers.h"
#include "LSM9DS1_Types.h"

static constexpr auto LSM9DS1_ACCELEROMETER_GYROSCOPE_DEVICE_ADDRESS = 0x6A;
static constexpr auto LSM9DS1_MAGNETOMETER_DEVICE_ADDRESS			 = 0x1C;

class LSM9DS1
{
public:
    explicit LSM9DS1(
		const char* devicePath,
		uint8_t accelerometerGyroscopeDeviceAddress = LSM9DS1_ACCELEROMETER_GYROSCOPE_DEVICE_ADDRESS,
        uint8_t magnetometerDeviceAddress           = LSM9DS1_MAGNETOMETER_DEVICE_ADDRESS
	);
    virtual ~LSM9DS1();

	std::tuple<double, double, double> getAngularRate();
    std::tuple<double, double, double> getLinearAcceleration();
	std::tuple<double, double, double> getMagneticField();

	double getTemperature();

	void setAccelerometerDataRate(LSM9DS1AccelerometerDataRate_t dataRate);
	void setAccelerometerScale(LSM9DS1AccelerometerScale_t scale);
	void setGyroscopeDataRate(LSM9DS1GyroscopeDataRate_t dataRate);
	void setGyroscopeScale(LSM9DS1GyroscopeScale_t scale);
	void setMagnetometerDataRate(LSM9DS1MagnetometerDataRate_t dataRate);
	void setMagnetometerPerformanceMode(LSM9DS1MagnetometerPerformanceMode_t performanceMode);
	void setMagnetometerScale(LSM9DS1MagnetometerScale_t scale);
	void setMagnetometerTemperatureCompensation(LSM9DS1MagnetometerTemperatureCompensation_t temperatureCompensation);

private:
    int accelerometerGyroscopeDevice, magnetometerDevice;
};

#endif // #ifndef LSM9DS1_H