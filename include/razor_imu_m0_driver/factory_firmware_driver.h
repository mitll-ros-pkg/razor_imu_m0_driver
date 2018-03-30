// -*- mode:c++; -*-

#ifndef SPARKFUN_9DOF_RAZOR_IMU_M0_FACTORY_FIRMWARE_DRIVER_H_
#define SPARKFUN_9DOF_RAZOR_IMU_M0_FACTORY_FIRMWARE_DRIVER_H_

#include <boost/atomic.hpp>
#include <pthread.h>

#include <string>

#include <serial/serial.h>

#include <ros/ros.h>

namespace razor_imu_m0_driver
{
class FactoryFirmwareDriver
{
public:
  FactoryFirmwareDriver(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~FactoryFirmwareDriver();

private:
  serial::Serial serial_;       ///< serial interface to the IMU
  pthread_t serial_thread_;     ///< serial port worker thread
  bool serial_thread_started_;  ///< status of serial port thread
  bool serial_thread_run_;      ///< serial port worker thread sentinel

  std::string port_;                     ///< serial port path
  std::string frame_id_;                 ///< IMU coordinate frame ID
  double rate_;                          ///< sensor output rate
  double gyro_fsr_;                      ///< gyro full scale range
  double accel_fsr_;                     ///< accelerometer full scale range
  double g_;                             ///< gavity constant for conversion to m/s^2
  double linear_acceleration_variance_;  ///< diagnal elements of the linear acceleration covariance matrix
  double angular_velocity_variance_;     ///< diagnal elements of the angular velocity covariance matrix
  double magnetic_field_variance_;       ///< diagnal elements of the magnetic field covariance matrix
  double orientation_variance_;          ///< diagnal elements of the orientation covariance matrix

  // ROS services
  ros::Publisher imu_data_raw_pub_;  ///< publisher for IMU message (raw intertial)
  ros::Publisher imu_data_pub_;      ///< publisher for IMU message (inertial and orientation)
  ros::Publisher imu_mag_pub_;       ///< publisher for magnetometer message

  /// serial port interface worker thread function
  void* serialThread(void);

  /// helper function to get class instance for serial port interface worker thread
  static void* serialThreadHelper(void* context)
  {
    return (static_cast<FactoryFirmwareDriver*>(context)->serialThread());
  }

  bool imuInterfaceLoop();
  bool measurementLoop(double timeout = 0.5);
  bool configureImu();
  bool serialWriteVerify(std::string const& data);
  bool sequentialCommand(std::string const& command, std::string const& response_format,
                         std::string const& response_desired_value, double response_timeout,
                         int max_sequential_commands);
  bool toggleSensorCommand(std::string const& command, bool turn_on, unsigned num_values);
  bool toggleEngineeringUnitsCommand(bool turn_on);
  bool togglePauseCommand(bool do_pause);
  void publishMeasurement(const std::vector<double>& measurement);
  bool readline(std::string& line, size_t size = 512, const std::string& eol = "\r\n");
  bool readlineWithTimeout(std::string& line, double timeout = 0.5, size_t size = 512, const std::string& eol = "\r\n");
};

}  // namespace razor_imu_m0_driver

#endif  // SPARKFUN_9DOF_RAZOR_IMU_M0_FACTORY_FIRMWARE_DRIVER_H_
