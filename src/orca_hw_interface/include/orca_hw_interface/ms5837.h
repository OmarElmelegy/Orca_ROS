#ifndef ORCA_HW_INTERFACE_MS5837_H
#define ORCA_HW_INTERFACE_MS5837_H

#include <ros/ros.h>
#include <string>

namespace orca_hw_interface {

/**
 * @brief Driver for the MS5837-30BA pressure sensor
 * 
 * This class implements a driver for the MS5837-30BA pressure sensor
 * connected via I2C. It provides methods to read pressure and temperature.
 */
class MS5837 {
public:
  /**
   * @brief Constructor
   * @param i2c_bus I2C bus (e.g., "/dev/i2c-1")
   * @param address I2C address of the sensor
   */
  MS5837(const std::string& i2c_bus = "/dev/i2c-1", int address = 0x76);

  /**
   * @brief Destructor
   */
  ~MS5837();

  /**
   * @brief Initialize the sensor
   * @return True if initialization was successful, false otherwise
   */
  bool init();

  /**
   * @brief Read sensor data
   * @return True if reading was successful, false otherwise
   */
  bool read();

  /**
   * @brief Get pressure in Pascals
   * @return Pressure in Pascals
   */
  double getPressure() const;

  /**
   * @brief Get temperature in degrees Celsius
   * @return Temperature in degrees Celsius
   */
  double getTemperature() const;

  /**
   * @brief Get depth in meters
   * @param fluidDensity Fluid density in kg/m^3 (default: 1000 for freshwater)
   * @return Depth in meters
   */
  double getDepth(double fluidDensity = 1000.0) const;

  /**
   * @brief Get altitude in meters
   * @return Altitude in meters
   */
  double getAltitude() const;

private:
  /**
   * @brief Reset the sensor
   * @return True if reset was successful, false otherwise
   */
  bool reset();

  /**
   * @brief Read calibration data from the sensor
   * @return True if reading was successful, false otherwise
   */
  bool readCalibration();

  /**
   * @brief Calculate CRC4 checksum
   * @param n_prom Calibration data
   * @return True if CRC is valid, false otherwise
   */
  bool crc4(uint16_t n_prom[]);

  /**
   * @brief Write a command to the sensor
   * @param cmd Command to write
   * @return True if writing was successful, false otherwise
   */
  bool writeCommand(uint8_t cmd);

  /**
   * @brief Read a 16-bit value from the sensor
   * @param value Reference to store the read value
   * @return True if reading was successful, false otherwise
   */
  bool readUInt16(uint16_t& value);

  /**
   * @brief Read a 24-bit value from the sensor
   * @param value Reference to store the read value
   * @return True if reading was successful, false otherwise
   */
  bool readUInt24(uint32_t& value);

  int file_handle_;
  int address_;
  std::string i2c_bus_;

  // Calibration data
  uint16_t cal_data_[8];

  // Sensor readings
  uint32_t d1_;  // Pressure raw
  uint32_t d2_;  // Temperature raw
  double pressure_;
  double temperature_;

  // Constants
  static constexpr double Pa = 100.0;
  static constexpr double bar = 0.001;
  static constexpr double mbar = 1.0;
};

}  // namespace orca_hw_interface

#endif  // ORCA_HW_INTERFACE_MS5837_H
