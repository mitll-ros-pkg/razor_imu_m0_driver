#include "razor_imu_m0_driver/factory_firmware_driver.h"

#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/cxx11/any_of.hpp>
#include <boost/assign.hpp>

#include <cassert>
#include <cstdlib>
#include <sstream>
#include <algorithm>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <ros/console.h>

namespace razor_imu_m0_driver
{
using boost::assign::list_of;
using boost::algorithm::any_of_equal;

void threadSleep(double duration, volatile bool sentinal);
std::vector<double> splitSensorMeasurement(std::string const& input);
bool isValidSensorMeasurement(std::vector<double> const& input);
std::string itos(int i);

FactoryFirmwareDriver::FactoryFirmwareDriver(ros::NodeHandle nh, ros::NodeHandle private_nh)
  : serial_(std::string(), 115200, serial::Timeout::simpleTimeout(50), serial::eightbits, serial::parity_none,
            serial::stopbits_one, serial::flowcontrol_none)
  , serial_thread_started_(false)
  , serial_thread_run_(true)
  , frame_id_("imu_link")
  , rate_(50)
  , gyro_fsr_(2000)
  , accel_fsr_(16)
  , g_(9.80665)
  , linear_acceleration_variance_(0.0)
  , angular_velocity_variance_(0.0)
  , magnetic_field_variance_(0.0)
  , orientation_variance_(0.0)
{
  // set the logger level to debug from a parameter, e.g. settable from launch
  bool set_log_debug(false);
  if (private_nh.getParam("set_logger_level_debug", set_log_debug) && set_log_debug)
  {
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
      ros::console::notifyLoggerLevelsChanged();
    }
  }
  ROS_DEBUG("Initializing razor_imu_m0_driver::FactoryFirmwareDriver");

  // imu serial port must be provided
  if (!private_nh.getParam("port", port_) || port_.empty())
  {
    ROS_FATAL("SparkFun 9DoF Razor IMU M0 paramaeter 'port' is required.");
    ros::shutdown();
    return;
  }

  // get IMU's coordinate frame id
  private_nh.getParam("frame_id", frame_id_);

  // get data sample rate and full-scale ranges from parameters
  private_nh.getParam("rate", rate_);
  if (!any_of_equal(list_of<double>(10)(20)(30)(40)(50)(60)(70)(80)(90)(100), rate_))
  {
    ROS_FATAL("SparkFun 9DoF Razor IMU M0 value for paramaeter 'rate' (%g) is invalid.", rate_);
    ros::shutdown();
    return;
  }
  private_nh.getParam("gyro_fsr", gyro_fsr_);
  if (!any_of_equal(list_of<double>(250)(500)(1000)(2000), gyro_fsr_))
  {
    ROS_FATAL("SparkFun 9DoF Razor IMU M0 value for paramaeter 'gyro_fsr' (%g) is invalid.", gyro_fsr_);
    ros::shutdown();
    return;
  }
  private_nh.getParam("accel_fsr", accel_fsr_);
  if (!any_of_equal(list_of<double>(2)(4)(8)(16), accel_fsr_))
  {
    ROS_FATAL("SparkFun 9DoF Razor IMU M0 value for paramaeter 'accel_fsr' (%g) is invalid.", accel_fsr_);
    ros::shutdown();
    return;
  }

  // get parameter values for gravity constant
  private_nh.getParam("g", g_);
  if (g_ < 9.6 || g_ > 9.9)
  {
    ROS_WARN("SparkFun 9DoF Razor IMU M0 value for gravity acceleration constant paramaeter 'g' (%g) is unusual.", g_);
  }

  // get parameter values for measurement covariance matrices
  double stddev_val;
  if (private_nh.getParam("linear_acceleration_stddev", stddev_val))
  {
    linear_acceleration_variance_ = stddev_val * stddev_val;
  }
  if (private_nh.getParam("angular_velocity_stddev", stddev_val))
  {
    angular_velocity_variance_ = stddev_val * stddev_val;
  }
  if (private_nh.getParam("magnetic_field_stddev", stddev_val))
  {
    magnetic_field_variance_ = stddev_val * stddev_val;
  }
  if (private_nh.getParam("orientation_stddev", stddev_val))
  {
    orientation_variance_ = stddev_val * stddev_val;
  }

  // create publishers for the IMU data
  ros::NodeHandle imu_nh(nh, "imu");
  imu_data_raw_pub_ = imu_nh.advertise<sensor_msgs::Imu>("data_raw", 10);
  imu_data_pub_ = imu_nh.advertise<sensor_msgs::Imu>("data", 10);
  imu_mag_pub_ = imu_nh.advertise<sensor_msgs::MagneticField>("mag", 10);

  // must start up a serial port worker thread
  if (0 != pthread_create(&serial_thread_, NULL, &FactoryFirmwareDriver::serialThreadHelper, this))
  {
    ROS_FATAL("SparkFun 9DoF Razor IMU M0 failed to create serial port worker thread.");
    ros::shutdown();
    return;
  }
  serial_thread_started_ = true;
}

FactoryFirmwareDriver::~FactoryFirmwareDriver()
{
  ROS_DEBUG("Destructing razor_imu_m0_driver::FactoryFirmwareDriver");

  // trigger serial port worker thread to terminate
  serial_thread_run_ = false;
  if (serial_thread_started_)
  {
    assert(0 == pthread_join(serial_thread_, NULL));
    serial_thread_started_ = false;
  }
}

/*! Manage serial port communication with the SparkFun 9DOF Razor M0.
 *
 *  Open the serial port, configure the imu, and continuously receive
 *  sensor measurements and publish data on ROS topics until
 *  serial_thread_run_ sentinel is set to false. If the serial port
 *  fails to open or if there is a serial port driver exception, it
 *  closes and retries opening the serial port.
 *
 */
void* FactoryFirmwareDriver::serialThread(void)
{
  ROS_DEBUG("razor_imu_m0_driver::FactoryFirmwareDriver::serialThread starting.");

  while (serial_thread_run_)
  {
    // attempt to open the serial port
    ROS_DEBUG("razor_imu_m0_driver::FactoryFirmwareDriver::openSerialPort opening serial device at port '%s'.",
              port_.c_str());
    try
    {
      serial_.setPort(port_);
      serial_.open();
    }
    catch (std::exception const& e)
    {
      ROS_ERROR("SparkFun 9DoF Razor IMU M0 couldn't open serial port '%s' (%s). Will retry in five seconds.",
                port_.c_str(), e.what());
      threadSleep(5, serial_thread_run_);
    }

    if (serial_.isOpen())
    {
      // on serial exception after opening serial port, close and try again
      try
      {
        // enter IMU interface loop, we don't care whether it exits normally or times out - the response is the same
        imuInterfaceLoop();
      }
      catch (std::exception const& e)
      {
        ROS_ERROR("SparkFun 9DoF Razor IMU M0 exception in "
                  "razor_imu_m0_driver::FactoryFirmwareDriver::serialThread "
                  "(%s).",
                  e.what());
      }

      // close the serial port
      serial_.close();
    }
  }

  ROS_DEBUG("razor_imu_m0_driver::FactoryFirmwareDriver::serialThread terminating.");
}

/*! Configure and continuously receive measurements from the IMU.
 *
 * Configure the IMU and receive / publish IMU data. If the IMU fails
 * to configure, retry every five seconds for a maximum of 5
 * attempts. Once IMU configuration is successful, repeatedly receive
 * sensor measurements from the IMU and publish the data on ROS topics
 * until serial_thread_run_ is false.
 *
 * \throw serial::PortNotOpenedException
 * \throw serial::SerialException
 */
bool FactoryFirmwareDriver::imuInterfaceLoop()
{
  // attempt to configure IMU, trying no more than 5 times
  int configure_attempts = 0;
  while (serial_thread_run_ && configure_attempts < 6)
  {
    configure_attempts++;
    if (configureImu())
    {
      configure_attempts = 0;

      // get measurements until serial_thread_run_ is set to false
      measurementLoop();
    }
    else
    {
      ROS_ERROR("SparkFun 9DoF Razor IMU M0 failed to configure IMU. Will retry in five seconds.");
      threadSleep(5, serial_thread_run_);
    }
  }

  return (configure_attempts < 6);
}

/*! Continuously read measurements lines from the serial port.
 *
 * Attempt to continously read measurement lines from the serial port
 * until serial_thread_run_ is false. Return true if
 * serial_thread_run_ is false. Return false if a valid measurement is
 * not received in the last timeout seconds.
 *
 * \param timeout Maximum length of time to wait for a line, in seconds, defaults to 0.5s.
 *
 * \return A bool indicating a normal or unexpected (timeout) termination condition.
 *
 * \throw serial::PortNotOpenedException
 * \throw serial::SerialException
 */
bool FactoryFirmwareDriver::measurementLoop(double timeout)
{
  std::string line;
  ros::WallTime expire(ros::WallTime::now() + ros::WallDuration(timeout));

  while (serial_thread_run_)
  {
    // check for timeout
    ros::WallTime now(ros::WallTime::now());
    if (now > expire)
    {
      ROS_WARN("SparkFun 9DoF Razor IMU M0 timeout while waiting for sensor measurement.");
      return false;
    }

    // get measurement line from serial port, note that the below will append to any existing content in line
    if (readline(line))
    {
      // parse measurement line
      std::vector<double> measurement;
      measurement = splitSensorMeasurement(line);
      if (isValidSensorMeasurement(measurement) && measurement.size() == 14)
      {
        // we have a good measurement, reset timeout timer
        expire = ros::WallTime::now() + ros::WallDuration(timeout);
        publishMeasurement(measurement);
      }
      else
      {
        ROS_WARN("SparkFun 9DoF Razor IMU M0 received an unexpected line while waiting for measurement, line is: '%s'.",
                 line.c_str());
      }

      // remove existing content from line
      line.clear();
    }
  }

  return true;  // normal exit
}

/*! Send configuration commands to the IMU
 */
bool FactoryFirmwareDriver::configureImu()
{
  // set output rate
  if (!sequentialCommand("r", "IMU rate set to (\\d+) Hz", itos(static_cast<int>(rate_)), 0.2, 12))
    return false;

  // set acceleration full-scale range
  if (!sequentialCommand("A", "Accel FSR set to \\+/-(\\d+) g", itos(static_cast<int>(accel_fsr_)), 0.2, 5))
    return false;

  // set gyroscope full-scale range
  if (!sequentialCommand("G", "Gyro FSR set to \\+/-(\\d+) dps", itos(static_cast<int>(gyro_fsr_)), 0.2, 5))
    return false;

  // unpause
  if (!togglePauseCommand(false))
    return false;

  // toggle time on
  if (!toggleSensorCommand("t", true, 1))
    return false;

  // toggle accelerometer on
  if (!toggleSensorCommand("a", true, 3))
    return false;

  // toggle gyro on
  if (!toggleSensorCommand("g", true, 3))
    return false;

  // toggle magnetometer on
  if (!toggleSensorCommand("m", true, 3))
    return false;

  // toggle quaternion on
  if (!toggleSensorCommand("q", true, 4))
    return false;

  // toggle Euler angle off
  if (!toggleSensorCommand("e", false, 3))
    return false;

  // toggle heading off
  if (!toggleSensorCommand("h", false, 1))
    return false;

  // toggle engineering units on
  if (!toggleEngineeringUnitsCommand(true))
    return false;

  return true;
}

std::vector<double> splitSensorMeasurement(std::string const& input)
{
  std::istringstream istream(input);

  std::vector<double> output;
  double current_val;
  while (istream >> current_val)
  {
    output.push_back(current_val);
    char comma_eater;
    if (istream >> comma_eater && comma_eater != ',')
    {
      // we expected either end of string or a comma
      output.clear();
      break;
    }
  }

  return output;
}

bool isValidSensorMeasurement(std::vector<double> const& input)
{
  // @todo: additional checks?
  return input.size() > 0;
}

void FactoryFirmwareDriver::publishMeasurement(const std::vector<double>& measurement)
{
  // @todo IMU provides a time value, should we use it somehow?
  // time = measurement[0]

  // imu_raw_msg contains the raw inertial measurements
  sensor_msgs::Imu::Ptr imu_raw_msg = boost::make_shared<sensor_msgs::Imu>();
  // set header
  imu_raw_msg->header.stamp = ros::Time::now();
  imu_raw_msg->header.frame_id = frame_id_;
  // acceleration in m/s^2
  imu_raw_msg->linear_acceleration.x = g_ * measurement[1];
  imu_raw_msg->linear_acceleration.y = g_ * measurement[2];
  imu_raw_msg->linear_acceleration.z = g_ * measurement[3];
  // accerlation uncertainty
  imu_raw_msg->linear_acceleration_covariance[0] = linear_acceleration_variance_;
  imu_raw_msg->linear_acceleration_covariance[4] = linear_acceleration_variance_;
  imu_raw_msg->linear_acceleration_covariance[8] = linear_acceleration_variance_;
  // gyroscope in rad/sec
  imu_raw_msg->angular_velocity.x = M_PI * measurement[4] / 180.0;
  imu_raw_msg->angular_velocity.y = M_PI * measurement[5] / 180.0;
  imu_raw_msg->angular_velocity.z = M_PI * measurement[6] / 180.0;
  // gyroscope uncertainty
  imu_raw_msg->angular_velocity_covariance[0] = angular_velocity_variance_;
  imu_raw_msg->angular_velocity_covariance[4] = angular_velocity_variance_;
  imu_raw_msg->angular_velocity_covariance[8] = angular_velocity_variance_;
  // imu_raw_msg does not provide orientation, so set the first element of the covariance field to -1
  imu_raw_msg->orientation_covariance[0] = -1;

  // msg_msg contains the magnetic field measurement
  sensor_msgs::MagneticField::Ptr mag_msg = boost::make_shared<sensor_msgs::MagneticField>();
  // header is the same as imu_raw_msg
  mag_msg->header = imu_raw_msg->header;
  // convert to Tesla, convert coordinate frame to match IMU
  mag_msg->magnetic_field.x = 1e-6 * measurement[8];       // accel/gyro x axis is mag y axis
  mag_msg->magnetic_field.y = 1e-6 * measurement[7];       // accel/gyro y axis is mag x axis
  mag_msg->magnetic_field.z = 1e-6 * -1 * measurement[9];  // accel/gyro z axis is negative mag z ax
  // magnetic field uncertainty
  mag_msg->magnetic_field_covariance[0] = magnetic_field_variance_;
  mag_msg->magnetic_field_covariance[4] = magnetic_field_variance_;
  mag_msg->magnetic_field_covariance[8] = magnetic_field_variance_;

  // imu_msg is includes both inertial and orientation measurements, start by copying imu_raw_msg
  sensor_msgs::Imu::Ptr imu_msg = boost::make_shared<sensor_msgs::Imu>(*imu_raw_msg);
  // provide orientation quaternion
  imu_msg->orientation.w = measurement[10];
  imu_msg->orientation.x = measurement[11];
  imu_msg->orientation.y = measurement[12];
  imu_msg->orientation.z = measurement[13];
  // orientation uncertainty (note: replaces -1 in the imu_raw_msg->orientation_covariance[0])
  imu_msg->orientation_covariance[0] = orientation_variance_;
  imu_msg->orientation_covariance[4] = orientation_variance_;
  imu_msg->orientation_covariance[8] = orientation_variance_;

  // publish messages
  imu_data_raw_pub_.publish(imu_raw_msg);
  imu_data_pub_.publish(imu_msg);
  imu_mag_pub_.publish(mag_msg);
}

/*! Attempt to read a line from the serial port, removing EOL sequence if successful.
 *
 * Reads a line terminated with EOL sequence from serial port,
 * appending to line.  Returns false if a full line is not read before
 * the serial port timeout. Note that a partial line may be appended
 * to line. If successful, removes the EOL sequence from the line and
 * returns true.
 *
 * \param line A std::string reference used to store the line.
 * \param size A maximum length of a line, defaults to 512.
 * \param eol A string to match against for the end-of-line, defaults to "\r\n".
 *
 * \return A bool indicating whether a full line was read from the serial port.
 *
 * \throw serial::PortNotOpenedException
 * \throw serial::SerialException
 */
bool FactoryFirmwareDriver::readline(std::string& line, size_t size, const std::string& eol)
{
  serial_.readline(line, size, eol);
  if (line.length() >= eol.length() && 0 == line.compare(line.length() - eol.length(), eol.length(), eol))
  {
    line.resize(line.length() - eol.length());
    return true;
  }
  return false;
}

/*! Attempt to read a line from the serial port, removing EOL sequence if successful.
 *
 * Reads a line terminated with EOL sequence from serial port, appending to line.
 * Returning false if a line is not read before timeout. If
 * successful, removes the EOL sequence from the line and returns
 * true.
 *
 * \param line A std::string reference used to store the line.
 * \param timeout Maximum length of time to wait for a line, in seconds, defaults to 0.5s.
 * \param size A maximum length of a line, defaults to 512.
 * \param eol A string to match against for the end-of-line, defaults to "\r\n".
 *
 * \return A bool indicating whether a full line was read from the serial port.
 *
 * \throw serial::PortNotOpenedException
 * \throw serial::SerialException
 */
bool FactoryFirmwareDriver::readlineWithTimeout(std::string& line, double timeout, size_t size, const std::string& eol)
{
  ros::WallTime expire(ros::WallTime::now() + ros::WallDuration(timeout));
  while (serial_thread_run_ && ros::WallTime::now() < expire)
  {
    serial_.readline(line, size, eol);
    if (line.length() >= eol.length() && 0 == line.compare(line.length() - eol.length(), eol.length(), eol))
    {
      line.resize(line.length() - eol.length());
      return true;
    }
  }
  return false;
}

/*! Return after duration seconds or sentinal is set to false.
 *
 * \param duration Duration to sleep.
 * \param sentinal Variale to monitor for thread exit condition.
 *
 */
void threadSleep(double duration, volatile bool sentinal)
{
  ros::WallTime expire(ros::WallTime::now() + ros::WallDuration(duration));
  while (sentinal && ros::WallTime::now() < expire)
  {
    ros::WallDuration(0.050).sleep();
  }
}

bool FactoryFirmwareDriver::serialWriteVerify(std::string const& data)
{
  if (!serial_thread_run_)
    return false;

  ROS_DEBUG_STREAM("Write: '" << data << "'");

  size_t num_written = serial_.write(data);
  if (num_written != data.length())
  {
    ROS_ERROR("Failed to write to SparkFun 9DoF Razor IMU M0 serial port. "
              "Attempted to write %zd bytes, actually wrote %zd bytes.",
              data.length(), num_written);
    return false;
  }

  return true;
}

bool FactoryFirmwareDriver::sequentialCommand(std::string const& command, std::string const& response_format,
                                              std::string const& response_desired_value, double response_timeout,
                                              int max_sequential_commands)
{
  // @todo a full regex seems a little over the top, replace with a simple function
  boost::regex regex(response_format);
  assert(regex.mark_count() > 0);

  int command_count(0);
  while (command_count < max_sequential_commands)
  {
    // send a command to step to the next setting
    if (!serialWriteVerify(command))
      return false;
    command_count++;

    // check the response to see if we've reached the desired setting
    ros::WallTime expire(ros::WallTime::now() + ros::WallDuration(response_timeout));
    while (ros::WallTime::now() < expire)
    {
      if (!serial_thread_run_)
        return false;
      std::string received = boost::trim_copy(serial_.readline());

      boost::cmatch matches;
      if (boost::regex_match(received.c_str(), matches, regex))
      {
        if (response_desired_value.compare(std::string(matches[1].first, matches[1].second)) == 0)
        {
          // successfully set command to desired value
          ROS_DEBUG_STREAM(received);
          return true;
        }
      }
      // else: received line could have been a sensor reading, continue
    }
  }

  ROS_ERROR("Sent %d '%s' commands to the IMU but did not receive an expected response matching "
            "format '%s' with value '%s'.",
            command_count, command.c_str(), response_format.c_str(), response_desired_value.c_str());
  return false;
}

bool FactoryFirmwareDriver::togglePauseCommand(bool do_pause)
{
  // wait for 3 * sensor sample period (1/rate_) for a measurement
  bool is_paused(true);
  ros::WallTime expire(ros::WallTime::now() + ros::WallDuration(std::max(3.0 / rate_, 0.1)));
  while (ros::WallTime::now() < expire)
  {
    if (!serial_thread_run_)
      return false;
    if (isValidSensorMeasurement(splitSensorMeasurement(boost::trim_copy(serial_.readline()))))
    {
      is_paused = false;
      break;
    }
  }

  // if current state is desired state return success
  if (do_pause == is_paused)
    return true;

  // current state is not the desired state, toggle pause
  if (!serialWriteVerify(" "))
    return false;

  return true;
}

bool FactoryFirmwareDriver::toggleSensorCommand(std::string const& command, bool turn_on, unsigned num_values)
{
  // wait for 3 * sensor sample period (1/rate_) for a measurement
  std::vector<double> initial_measurement;
  ros::WallTime expire(ros::WallTime::now() + ros::WallDuration(std::max(3.0 / rate_, 0.1)));
  while (ros::WallTime::now() < expire)
  {
    if (!serial_thread_run_)
      return false;
    initial_measurement = splitSensorMeasurement(boost::trim_copy(serial_.readline()));

    if (isValidSensorMeasurement(initial_measurement))
      break;
  }

  if (!isValidSensorMeasurement(initial_measurement))
    return false;  // timed out

  // send toggle command to see if increased or decreased number of measurements
  if (!serialWriteVerify(command))
    return false;

  // wait another 3 * sensor sample period for measurment length to change
  std::vector<double> new_measurement;
  expire = ros::WallTime::now() + ros::WallDuration(std::max(3.0 / rate_, 0.1));
  while (ros::WallTime::now() < expire)
  {
    if (!serial_thread_run_)
      return false;
    new_measurement = splitSensorMeasurement(boost::trim_copy(serial_.readline()));

    if (isValidSensorMeasurement(new_measurement) && new_measurement.size() != initial_measurement.size())
      break;
  }

  if (!(isValidSensorMeasurement(new_measurement) && new_measurement.size() != initial_measurement.size()))
    return false;  // timed out with a change in measurement size

  // is magnitude of change in the measurement length correct
  int delta = static_cast<int>(new_measurement.size()) - static_cast<int>(initial_measurement.size());
  if (abs(delta) != num_values)
    return false;  // unexpected number of values changed

  // did the measurement size change in the direction we wanted?
  if (turn_on && delta > 0)
    return true;  // success!
  else if (!turn_on && delta < 0)
    return true;  // success!

  // command had the opposite effect of desired, send it again to toggle to correct state
  initial_measurement = new_measurement;
  new_measurement.clear();
  if (!serialWriteVerify(command))
    return false;

  // wait another 3 * sensor sample period for measurment length to change
  expire = ros::WallTime::now() + ros::WallDuration(std::max(3.0 / rate_, 0.1));
  while (ros::WallTime::now() < expire)
  {
    if (!serial_thread_run_)
      return false;
    new_measurement = splitSensorMeasurement(boost::trim_copy(serial_.readline()));

    if (isValidSensorMeasurement(new_measurement) && new_measurement.size() != initial_measurement.size())
      break;
  }

  if (!(isValidSensorMeasurement(new_measurement) && new_measurement.size() != initial_measurement.size()))
    return false;  // timed out with a change in measurement size

  // is magnitude of change in the measurement length correct
  delta = static_cast<int>(new_measurement.size()) - static_cast<int>(initial_measurement.size());
  if (abs(delta) != num_values)
    return false;  // unexpected number of values changed

  // did the measurement size change in the direction we wanted?
  if (turn_on && delta > 0)
    return true;  // success!
  else if (!turn_on && delta < 0)
    return true;  // success!

  // not sure what is going on if we get here
  return false;
}

bool isSensorMeasurementEngineeringUnits(std::vector<double> const& input)
{
  // @todo heading seems to always be in floating point

  bool is_engineering_units(false);
  for (std::vector<double>::const_iterator i(input.begin()); i != input.end(); i++)
  {
    if (floor(*i) != *i)
    {
      is_engineering_units = true;
    }
  }
  return is_engineering_units;
}

bool FactoryFirmwareDriver::toggleEngineeringUnitsCommand(bool turn_on)
{
  // wait for 3 * sensor sample period (1/rate_) for a measurement
  std::vector<double> measurement;
  ros::WallTime expire(ros::WallTime::now() + ros::WallDuration(std::max(3.0 / rate_, 0.1)));
  while (ros::WallTime::now() < expire)
  {
    if (!serial_thread_run_)
      return false;
    measurement = splitSensorMeasurement(boost::trim_copy(serial_.readline()));

    if (isValidSensorMeasurement(measurement))
      break;
  }

  if (!isValidSensorMeasurement(measurement))
    return false;  // timed out

  bool is_engineering_units(isSensorMeasurementEngineeringUnits(measurement));

  if (turn_on && is_engineering_units)
    return true;
  else if (!turn_on && !is_engineering_units)
    return true;

  // send toggle command to change state of engineering units
  if (!serialWriteVerify("c"))
    return false;

  // wait for 3 * sensor sample period (1/rate_) for a measurement
  expire = ros::WallTime::now() + ros::WallDuration(std::max(3.0 / rate_, 0.1));
  while (ros::WallTime::now() < expire)
  {
    if (!serial_thread_run_)
      return false;
    measurement = splitSensorMeasurement(boost::trim_copy(serial_.readline()));

    if (isValidSensorMeasurement(measurement))
      break;
  }

  if (!isValidSensorMeasurement(measurement))
    return false;  // timed out

  is_engineering_units = isSensorMeasurementEngineeringUnits(measurement);

  if (turn_on && is_engineering_units)
    return true;
  else if (!turn_on && !is_engineering_units)
    return true;

  // command did not work
  return false;
}

std::string itos(int i)
{
  return static_cast<std::ostringstream&>(std::ostringstream() << std::dec << i).str();
}

class ConfigureException : public std::exception
{
  // Disable copy constructors
  ConfigureException& operator=(const ConfigureException&);
  std::string e_what_;

public:
  ConfigureException(const char* description)
  {
    std::ostringstream ss;
    ss << "ConfigureException: " << description << " failed.";
    e_what_ = ss.str();
  }

  ConfigureException(std::ostringstream& description)
  {
    e_what_ = description.str();
  }

  ConfigureException(const ConfigureException& other) : e_what_(other.e_what_)
  {
  }

  virtual ~ConfigureException() throw()
  {
  }

  virtual const char* what() const throw()
  {
    return e_what_.c_str();
  }
};

}  // namespace razor_imu_m0_driver
