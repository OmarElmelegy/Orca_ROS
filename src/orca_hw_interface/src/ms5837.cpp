#include "orca_hw_interface/ms5837.h"
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cmath>

namespace orca_hw_interface {

// MS5837 Commands
constexpr uint8_t MS5837_RESET = 0x1E;
constexpr uint8_t MS5837_ADC_READ = 0x00;
constexpr uint8_t MS5837_PROM_READ = 0xA0;
constexpr uint8_t MS5837_CONVERT_D1_256 = 0x40;
constexpr uint8_t MS5837_CONVERT_D1_512 = 0x42;
constexpr uint8_t MS5837_CONVERT_D1_1024 = 0x44;
constexpr uint8_t MS5837_CONVERT_D1_2048 = 0x46;
constexpr uint8_t MS5837_CONVERT_D1_4096 = 0x48;
constexpr uint8_t MS5837_CONVERT_D1_8192 = 0x4A;
constexpr uint8_t MS5837_CONVERT_D2_256 = 0x50;
constexpr uint8_t MS5837_CONVERT_D2_512 = 0x52;
constexpr uint8_t MS5837_CONVERT_D2_1024 = 0x54;
constexpr uint8_t MS5837_CONVERT_D2_2048 = 0x56;
constexpr uint8_t MS5837_CONVERT_D2_4096 = 0x58;
constexpr uint8_t MS5837_CONVERT_D2_8192 = 0x5A;

MS5837::MS5837(const std::string& i2c_bus, int address)
    : file_handle_(-1), address_(address), i2c_bus_(i2c_bus), d1_(0), d2_(0), pressure_(0), temperature_(0) {
  // Initialize calibration data
  for (int i = 0; i < 8; ++i) {
    cal_data_[i] = 0;
  }
}

MS5837::~MS5837() {
  if (file_handle_ >= 0) {
    close(file_handle_);
  }
}

bool MS5837::init() {
  // Open I2C bus
  file_handle_ = open(i2c_bus_.c_str(), O_RDWR);
  if (file_handle_ < 0) {
    ROS_ERROR("Failed to open I2C bus %s", i2c_bus_.c_str());
    return false;
  }

  // Set I2C slave address
  if (ioctl(file_handle_, I2C_SLAVE, address_) < 0) {
    ROS_ERROR("Failed to set I2C slave address 0x%02X", address_);
    close(file_handle_);
    file_handle_ = -1;
    return false;
  }

  // Reset the sensor
  if (!reset()) {
    ROS_ERROR("Failed to reset MS5837 sensor");
    close(file_handle_);
    file_handle_ = -1;
    return false;
  }

  // Read calibration data
  if (!readCalibration()) {
    ROS_ERROR("Failed to read MS5837 calibration data");
    close(file_handle_);
    file_handle_ = -1;
    return false;
  }

  return true;
}

bool MS5837::reset() {
  return writeCommand(MS5837_RESET);
}

bool MS5837::readCalibration() {
  // Wait for reset to complete
  ros::Duration(0.01).sleep();

  // Read calibration data from PROM
  for (int i = 0; i < 7; ++i) {
    if (!writeCommand(MS5837_PROM_READ + (i * 2))) {
      return false;
    }
    if (!readUInt16(cal_data_[i])) {
      return false;
    }
  }

  // Verify CRC
  return crc4(cal_data_);
}

bool MS5837::crc4(uint16_t n_prom[]) {
  uint16_t n_rem = 0;

  n_prom[0] = ((n_prom[0]) & 0x0FFF);
  n_prom[7] = 0;

  for (uint8_t i = 0; i < 16; i++) {
    if (i % 2 == 1) {
      n_rem ^= (uint16_t)((n_prom[i >> 1]) & 0x00FF);
    } else {
      n_rem ^= (uint16_t)(n_prom[i >> 1] >> 8);
    }
    for (uint8_t n_bit = 8; n_bit > 0; n_bit--) {
      if (n_rem & 0x8000) {
        n_rem = (n_rem << 1) ^ 0x3000;
      } else {
        n_rem = (n_rem << 1);
      }
    }
  }

  n_rem = ((n_rem >> 12) & 0x000F);

  return (n_rem == (n_prom[0] >> 12));
}

bool MS5837::read() {
  // Request D1 conversion (pressure)
  if (!writeCommand(MS5837_CONVERT_D1_8192)) {
    return false;
  }

  // Wait for conversion to complete
  ros::Duration(0.02).sleep();

  // Read D1
  if (!writeCommand(MS5837_ADC_READ)) {
    return false;
  }
  if (!readUInt24(d1_)) {
    return false;
  }

  // Request D2 conversion (temperature)
  if (!writeCommand(MS5837_CONVERT_D2_8192)) {
    return false;
  }

  // Wait for conversion to complete
  ros::Duration(0.02).sleep();

  // Read D2
  if (!writeCommand(MS5837_ADC_READ)) {
    return false;
  }
  if (!readUInt24(d2_)) {
    return false;
  }

  // Calculate temperature
  int32_t dT = d2_ - (static_cast<uint32_t>(cal_data_[5]) << 8);
  int32_t temp = 2000 + ((dT * static_cast<int64_t>(cal_data_[6])) >> 23);

  // Calculate temperature-compensated pressure
  int64_t OFF = static_cast<int64_t>(cal_data_[2]) << 16;
  OFF += (static_cast<int64_t>(cal_data_[4]) * dT) >> 7;

  int64_t SENS = static_cast<int64_t>(cal_data_[1]) << 15;
  SENS += (static_cast<int64_t>(cal_data_[3]) * dT) >> 8;

  // Second order temperature compensation
  if (temp < 2000) {
    // Low temperature
    int32_t T2 = (3 * (static_cast<int64_t>(dT) * dT)) >> 33;
    int64_t OFF2 = 3 * ((temp - 2000) * (temp - 2000)) / 2;
    int64_t SENS2 = 5 * ((temp - 2000) * (temp - 2000)) / 8;

    if (temp < -1500) {
      // Very low temperature
      OFF2 += 7 * ((temp + 1500) * (temp + 1500));
      SENS2 += 4 * ((temp + 1500) * (temp + 1500));
    }

    temp -= T2;
    OFF -= OFF2;
    SENS -= SENS2;
  }

  int32_t P = (((d1_ * SENS) >> 21) - OFF) >> 15;

  // Store calculated values
  temperature_ = temp / 100.0;
  pressure_ = P * 10.0;  // Convert to Pa

  return true;
}

double MS5837::getPressure() const {
  return pressure_;
}

double MS5837::getTemperature() const {
  return temperature_;
}

double MS5837::getDepth(double fluidDensity) const {
  // Calculate depth based on pressure difference from atmospheric pressure
  // P = P0 + rho * g * h
  // h = (P - P0) / (rho * g)
  // Assuming atmospheric pressure is 101325 Pa
  return (pressure_ - 101325.0) / (fluidDensity * 9.80665);
}

double MS5837::getAltitude() const {
  // Calculate altitude based on pressure
  // h = 44330 * (1 - (P/P0)^(1/5.255))
  // Assuming sea level pressure is 101325 Pa
  return 44330.0 * (1.0 - pow(pressure_ / 101325.0, 0.190295));
}

bool MS5837::writeCommand(uint8_t cmd) {
  if (file_handle_ < 0) {
    return false;
  }

  if (write(file_handle_, &cmd, 1) != 1) {
    ROS_ERROR("Failed to write command 0x%02X to MS5837", cmd);
    return false;
  }

  return true;
}

bool MS5837::readUInt16(uint16_t& value) {
  if (file_handle_ < 0) {
    return false;
  }

  uint8_t data[2];
  if (::read(file_handle_, data, 2) != 2) {
    ROS_ERROR("Failed to read 16-bit value from MS5837");
    return false;
  }

  value = (static_cast<uint16_t>(data[0]) << 8) | data[1];
  return true;
}

bool MS5837::readUInt24(uint32_t& value) {
  if (file_handle_ < 0) {
    return false;
  }

  uint8_t data[3];
  if (::read(file_handle_, data, 3) != 3) {
    ROS_ERROR("Failed to read 24-bit value from MS5837");
    return false;
  }

  value = (static_cast<uint32_t>(data[0]) << 16) | (static_cast<uint32_t>(data[1]) << 8) | data[2];
  return true;
}

}  // namespace orca_hw_interface
