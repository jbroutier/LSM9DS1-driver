#include "LSM9DS1.h"

LSM9DS1::LSM9DS1(const char* devicePath, uint8_t accelerometerGyroscopeDeviceAddress, uint8_t magnetometerDeviceAddress)
{
    if ((accelerometerGyroscopeDevice = open(devicePath, O_RDWR)) < 0) {
        throw std::runtime_error("Failed to open the I2C bus.");
    }

    if (ioctl(accelerometerGyroscopeDevice, I2C_SLAVE, accelerometerGyroscopeDeviceAddress) < 0) {
        close(accelerometerGyroscopeDevice);
        throw std::runtime_error("Failed to configure the I2C device.");
    }

    if ((magnetometerDevice = open(devicePath, O_RDWR)) < 0) {
        throw std::runtime_error("Failed to open the I2C bus.");
    }

    if (ioctl(magnetometerDevice, I2C_SLAVE, magnetometerDeviceAddress) < 0) {
        close(magnetometerDevice);
        throw std::runtime_error("Failed to configure the I2C device.");
    }

    int32_t data;

    data = i2c_smbus_read_byte_data(accelerometerGyroscopeDevice, LSM9DS1_REGISTER_WHO_AM_I);

    if (data != 0x68) {
        throw std::runtime_error("Failed to verify the device identity.");
    }

    data = i2c_smbus_read_byte_data(magnetometerDevice, LSM9DS1_REGISTER_WHO_AM_I_M);

    if (data != 0x3D) {
        throw std::runtime_error("Failed to verify the device identity.");
    }

    data = 0;

    data |= LSM9DS1_ACCELEROMETER_DATA_RATE_10HZ;
    data |= LSM9DS1_ACCELEROMETER_SCALE_2G;

    i2c_smbus_write_byte_data(accelerometerGyroscopeDevice, LSM9DS1_REGISTER_CTRL_REG6_XL, (uint8_t)data);

    data = 0;

    data |= LSM9DS1_ACCELEROMETER_X_AXIS;
    data |= LSM9DS1_ACCELEROMETER_Y_AXIS;
    data |= LSM9DS1_ACCELEROMETER_Z_AXIS;

    i2c_smbus_write_byte_data(accelerometerGyroscopeDevice, LSM9DS1_REGISTER_CTRL_REG5_XL, (uint8_t)data);

    data = 0;

    data |= LSM9DS1_GYROSCOPE_DATA_RATE_14_9_HZ;
    data |= LSM9DS1_GYROSCOPE_SCALE_245_DPS;

    i2c_smbus_write_byte_data(accelerometerGyroscopeDevice, LSM9DS1_REGISTER_CTRL_REG1_G, (uint8_t)data);

    data = 0;

    data |= LSM9DS1_GYROSCOPE_X_AXIS;
    data |= LSM9DS1_GYROSCOPE_Y_AXIS;
    data |= LSM9DS1_GYROSCOPE_Z_AXIS;

    i2c_smbus_write_byte_data(accelerometerGyroscopeDevice, LSM9DS1_REGISTER_CTRL_REG4, (uint8_t)data);

    data = 0;

    data |= LSM9DS1_MAGNETOMETER_DATA_RATE_1_25_HZ;
    data |= LSM9DS1_MAGNETOMETER_PERFORMANCE_MODE_LOW_POWER;
    data |= LSM9DS1_MAGNETOMETER_TEMPERATURE_COMPENSATION_ENABLED;

    i2c_smbus_write_byte_data(magnetometerDevice, LSM9DS1_REGISTER_CTRL_REG1_M, (uint8_t)data);

    data = 0;

    data |= LSM9DS1_MAGNETOMETER_SCALE_4G;

    i2c_smbus_write_byte_data(magnetometerDevice, LSM9DS1_REGISTER_CTRL_REG2_M, (uint8_t)data);

    data = 0;

    data |= LSM9DS1_MAGNETOMETER_OPERATING_MODE_CONTINUOUS_CONVERSION;

    i2c_smbus_write_byte_data(magnetometerDevice, LSM9DS1_REGISTER_CTRL_REG3_M, (uint8_t)data);
}

std::tuple<double, double, double> LSM9DS1::getAngularRate()
{
    int32_t data, x, y, z;

    data = i2c_smbus_read_byte_data(accelerometerGyroscopeDevice, LSM9DS1_REGISTER_OUT_X_G_L);
    x = data;

    data = i2c_smbus_read_byte_data(accelerometerGyroscopeDevice, LSM9DS1_REGISTER_OUT_X_G_H);
    x |= data << 8;

    if (x > 32768) {
        x -= 65536;
    }

    data = i2c_smbus_read_byte_data(accelerometerGyroscopeDevice, LSM9DS1_REGISTER_OUT_Y_G_L);
    y = data;

    data = i2c_smbus_read_byte_data(accelerometerGyroscopeDevice, LSM9DS1_REGISTER_OUT_Y_G_H);
    y |= data << 8;

    if (y > 32768) {
        y -= 65536;
    }

    data = i2c_smbus_read_byte_data(accelerometerGyroscopeDevice, LSM9DS1_REGISTER_OUT_Z_G_L);
    z = data;

    data = i2c_smbus_read_byte_data(accelerometerGyroscopeDevice, LSM9DS1_REGISTER_OUT_Z_G_H);
    z |= data << 8;

    if (z > 32768) {
        z -= 65536;
    }

    data = i2c_smbus_read_byte_data(accelerometerGyroscopeDevice, LSM9DS1_REGISTER_CTRL_REG1_G);

    double scale;

    switch (data & (0b11 << 3)) {
    case LSM9DS1_GYROSCOPE_SCALE_245_DPS:
        scale = 1.0 / (32768.0 / 245.0);
        break;

    case LSM9DS1_GYROSCOPE_SCALE_500_DPS:
        scale = 1.0 / (32768.0 / 500.0);
        break;

    case LSM9DS1_GYROSCOPE_SCALE_2000_DPS:
        scale = 1.0 / (32768.0 / 2000.0);
        break;

    default:
        break;
    }

    return std::make_tuple(x * scale, y * scale, z * scale);
}

std::tuple<double, double, double> LSM9DS1::getLinearAcceleration()
{
    int32_t data, x, y, z;

    data = i2c_smbus_read_byte_data(accelerometerGyroscopeDevice, LSM9DS1_REGISTER_OUT_X_XL_L);
    x = data;

    data = i2c_smbus_read_byte_data(accelerometerGyroscopeDevice, LSM9DS1_REGISTER_OUT_X_XL_H);
    x |= data << 8;

    if (x > 32768) {
        x -= 65536;
    }

    data = i2c_smbus_read_byte_data(accelerometerGyroscopeDevice, LSM9DS1_REGISTER_OUT_Y_XL_L);
    y = data;

    data = i2c_smbus_read_byte_data(accelerometerGyroscopeDevice, LSM9DS1_REGISTER_OUT_Y_XL_H);
    y |= data << 8;

    if (y > 32768) {
        y -= 65536;
    }

    data = i2c_smbus_read_byte_data(accelerometerGyroscopeDevice, LSM9DS1_REGISTER_OUT_Z_XL_L);
    z = data;

    data = i2c_smbus_read_byte_data(accelerometerGyroscopeDevice, LSM9DS1_REGISTER_OUT_Z_XL_H);
    z |= data << 8;

    if (z > 32768) {
        z -= 65536;
    }

    data = i2c_smbus_read_byte_data(accelerometerGyroscopeDevice, LSM9DS1_REGISTER_CTRL_REG6_XL);

    double scale;

    switch (data & (0b11 << 3)) {
    case LSM9DS1_ACCELEROMETER_SCALE_2G:
        scale = 1.0 / (32768.0 / 2.0);
        break;

    case LSM9DS1_ACCELEROMETER_SCALE_4G:
        scale = 1.0 / (32768.0 / 4.0);
        break;

    case LSM9DS1_ACCELEROMETER_SCALE_8G:
        scale = 1.0 / (32768.0 / 8.0);
        break;

    case LSM9DS1_ACCELEROMETER_SCALE_16G:
        scale = 1.0 / (32768.0 / 16.0);
        break;

    default:
        break;
    }

    return std::make_tuple(x * scale, y * scale, z * scale);
}

std::tuple<double, double, double> LSM9DS1::getMagneticField()
{
    int32_t data, x, y, z;

    data = i2c_smbus_read_byte_data(magnetometerDevice, LSM9DS1_REGISTER_OUT_X_L_M);
    x = data;

    data = i2c_smbus_read_byte_data(magnetometerDevice, LSM9DS1_REGISTER_OUT_X_H_M);
    x |= data << 8;

    if (x > 32768) {
        x -= 65536;
    }

    data = i2c_smbus_read_byte_data(magnetometerDevice, LSM9DS1_REGISTER_OUT_Y_L_M);
    y = data;

    data = i2c_smbus_read_byte_data(magnetometerDevice, LSM9DS1_REGISTER_OUT_Y_H_M);
    y |= data << 8;

    if (y > 32768) {
        y -= 65536;
    }

    data = i2c_smbus_read_byte_data(magnetometerDevice, LSM9DS1_REGISTER_OUT_Z_L_M);
    z = data;

    data = i2c_smbus_read_byte_data(magnetometerDevice, LSM9DS1_REGISTER_OUT_Z_H_M);
    z |= data << 8;

    if (z > 32768) {
        z -= 65536;
    }

    data = i2c_smbus_read_byte_data(magnetometerDevice, LSM9DS1_REGISTER_CTRL_REG2_M);

    double scale;

    switch (data & (0b11 << 3)) {
    case LSM9DS1_MAGNETOMETER_SCALE_4G:
        scale = 1.0 / (32768.0 / 4.0);
        break;

    case LSM9DS1_MAGNETOMETER_SCALE_8G:
        scale = 1.0 / (32768.0 / 8.0);
        break;

    case LSM9DS1_MAGNETOMETER_SCALE_12G:
        scale = 1.0 / (32768.0 / 12.0);
        break;

    case LSM9DS1_MAGNETOMETER_SCALE_16G:
        scale = 1.0 / (32768.0 / 16.0);
        break;

    default:
        break;
    }

    return std::make_tuple(x * scale, y * scale, z * scale);
}

double LSM9DS1::getTemperature()
{
    int32_t data, temperature;

    data = i2c_smbus_read_byte_data(accelerometerGyroscopeDevice, LSM9DS1_REGISTER_OUT_TEMP_L);
    temperature = data;

    data = i2c_smbus_read_byte_data(accelerometerGyroscopeDevice, LSM9DS1_REGISTER_OUT_TEMP_H);
    temperature |= data << 8;

    if (temperature > 32768) {
        temperature -= 65536;
    }

    return 25.0 + (temperature / 16.0);
}

void LSM9DS1::setAccelerometerDataRate(LSM9DS1AccelerometerDataRate_t dataRate)
{
    int32_t data = i2c_smbus_read_byte_data(accelerometerGyroscopeDevice, LSM9DS1_REGISTER_CTRL_REG6_XL);

    data &= ~(0b111 << 5);
    data |= dataRate;

    i2c_smbus_write_byte_data(accelerometerGyroscopeDevice, LSM9DS1_REGISTER_CTRL_REG6_XL, (uint8_t)data);
}

void LSM9DS1::setAccelerometerScale(LSM9DS1AccelerometerScale_t scale)
{
    int32_t data = i2c_smbus_read_byte_data(accelerometerGyroscopeDevice, LSM9DS1_REGISTER_CTRL_REG6_XL);

    data &= ~(0b11 << 3);
    data |= scale;

    i2c_smbus_write_byte_data(accelerometerGyroscopeDevice, LSM9DS1_REGISTER_CTRL_REG6_XL, (uint8_t)data);
}

void LSM9DS1::setGyroscopeDataRate(LSM9DS1GyroscopeDataRate_t dataRate)
{
    int32_t data = i2c_smbus_read_byte_data(accelerometerGyroscopeDevice, LSM9DS1_REGISTER_CTRL_REG1_G);

    data &= ~(0b111 << 5);
    data |= dataRate;

    i2c_smbus_write_byte_data(accelerometerGyroscopeDevice, LSM9DS1_REGISTER_CTRL_REG1_G, (uint8_t)data);
}

void LSM9DS1::setGyroscopeScale(LSM9DS1GyroscopeScale_t scale)
{
    int32_t data = i2c_smbus_read_byte_data(accelerometerGyroscopeDevice, LSM9DS1_REGISTER_CTRL_REG1_G);

    data &= ~(0b11 << 3);
    data |= scale;

    i2c_smbus_write_byte_data(accelerometerGyroscopeDevice, LSM9DS1_REGISTER_CTRL_REG1_G, (uint8_t)data);
}

void LSM9DS1::setMagnetometerDataRate(LSM9DS1MagnetometerDataRate_t dataRate)
{
    int32_t data = i2c_smbus_read_byte_data(magnetometerDevice, LSM9DS1_REGISTER_CTRL_REG1_M);

    data &= ~(0b111 << 2);
    data |= dataRate;

    i2c_smbus_write_byte_data(magnetometerDevice, LSM9DS1_REGISTER_CTRL_REG1_M, (uint8_t)data);
}

void LSM9DS1::setMagnetometerPerformanceMode(LSM9DS1MagnetometerPerformanceMode_t performanceMode)
{
    int32_t data = i2c_smbus_read_byte_data(magnetometerDevice, LSM9DS1_REGISTER_CTRL_REG1_M);

    data &= ~(0b11 << 5);
    data |= performanceMode;

    i2c_smbus_write_byte_data(magnetometerDevice, LSM9DS1_REGISTER_CTRL_REG1_M, (uint8_t)data);
}

void LSM9DS1::setMagnetometerScale(LSM9DS1MagnetometerScale_t scale)
{
    int32_t data = i2c_smbus_read_byte_data(magnetometerDevice, LSM9DS1_REGISTER_CTRL_REG2_M);

    data &= ~(0b11 << 5);
    data |= scale;

    i2c_smbus_write_byte_data(magnetometerDevice, LSM9DS1_REGISTER_CTRL_REG2_M, (uint8_t)data);
}

void LSM9DS1::setMagnetometerTemperatureCompensation(LSM9DS1MagnetometerTemperatureCompensation_t temperatureCompensation)
{
    int32_t data = i2c_smbus_read_byte_data(magnetometerDevice, LSM9DS1_REGISTER_CTRL_REG2_M);

    data &= ~(0b1 << 7);
    data |= temperatureCompensation;

    i2c_smbus_write_byte_data(magnetometerDevice, LSM9DS1_REGISTER_CTRL_REG2_M, (uint8_t)data);
}

LSM9DS1::~LSM9DS1()
{
    close(accelerometerGyroscopeDevice);
    close(magnetometerDevice);
}
