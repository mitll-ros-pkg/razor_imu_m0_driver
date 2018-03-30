# Razor IMU M0 Driver Overview
razor_imu_m0_driver is a ROS interface to the SparkFun 9DoF Razor IMU M0 sensor with the factory-programmed firmware. This package provides a ROS node (and nodelet) that configures the IMU and publishes sensed motion data on ROS topics. The ROS API seeks to follow [REP-145](http://www.ros.org/reps/rep-0145.html).

# Supported Hardware
This package supports the SparkFun 9DoF Razor IMU M0 [(SparkFun Product Number SEN-14001)](https://www.sparkfun.com/products/14001) sensor board which provides an USB power and data interface to the TDK InvenSense MPU-9250 nine-axis (gyroscope, accelerometer, magnetometer) MEMS sensor via an Atmel SAMD21 microprocessor. Firmware executing on the SAMD21 microprocessor provides an interface between the MPU-9250's i2c serial interface and the USB interface to the host computer. 

## Factory-Programmed Firmware
SparkFun provides the Razor M0 with a factory-programmed "example" [firmware](https://github.com/sparkfun/9DOF_Razor_IMU) that is capable of configuring and reading sensor data from the MPU-9250, optionally converting sensed data to engineering units, and transmitting the data stream to a host computer over the USB interface. The factory_firmware_driver_node ROS node (and factory_firmware_driver_nodelet ROS nodelet) support the factory-programmed firmware.

### Measurements
The MPU-9250 includes a Digital Motion Processor (DMP) which provides features to calibrate and fuse the device's gyroscope and accelerometer measurements. The factory-programmed firmware uses the DMP to provided an orientation estimate from the gyroscope and accelerometer data which is reported in the orientation field of the imu/data topic. The DMP (and hence the imu/data topic's orientation field) does not leverage the magnetometer in the orientation calculation. The magnetometer measurements are separately published on the imu/mag topic.

### Limitations
The factory-programmed firmware's ability to configure the MPU-9250 is limited to a subset of the MPU-9250's features. For example, the firmware sets a fixed 5 Hz low-pass filter cutoff for the accelerometer and gyroscope measurements and 100 Hz sampling frequency for the magnetometer. Modified or custom firmware may be necessary for some applications.

# API Stability
As REP-145 is currently a draft, this node's ROS interface may change in the future.

# ROS API

The ROS API follows [REP-145](http://www.ros.org/reps/rep-0145.html).

## factory_firmware_driver_node
factory_firmware_driver_node is a standalone node that provides a ROS interface to the Razor M0 with factory-programmed firmware. The node opens a USB-serial connection to the board; configures the sensor; and then continuously publishes sensed acceleration, angular velocity, and orientation data.

### Published Topics
imu/data_raw ([sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html))
  * Streaming acceleration and angular velocity measurements from the Razor M0, i.e. raw inertial measurements. Measurements are provided in the gyroscope and accelerometer coordinate frame indicated on the Razor M0 board's silk screen.

imu/data ([sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html))
  * Streaming acceleration, angular velocity, and orientation measurements from the Razor M0. Measurements are provided in the gyroscope and accelerometer coordinate frame indicated on the Razor M0 board's silk screen. Acceleration and angular velocity estimates are raw measurements, i.e. they are not additionally processed by the node. The orientation estimate is provided by the MPU-9250's Digital Motion Processor (DMP) based on accelerometer and gyroscope measurements. Note that the DMP's orientation calculation may be invalid if the gyroscope full-scale range is not 2000.

imu/mag ([sensor_msgs/MagneticField](http://docs.ros.org/api/sensor_msgs/html/msg/MagneticField.html))
  * Streaming three-axis magnetic field measurements from the Razor M0. Per REP-145, the factory_firmware_driver_node transforms the orientation of the sensor's raw magnetometer measurements to the IMU's coordinate frame (the IMU and magnetometer within the MPU-9250 have different orientations).

### Parameters
~port (string, required)
  * Linux USB device port to which the Razor IMU M0 is connected, such as /dev/ttyACM0. Using udev to create a named symbolic link to the device port is recommended. A udev rules file is provided which maps the Razor M0 to a /dev/imuN port where N is the device number, such as /dev/imu0.

~frame_id (string, default: imu_link)
  * The name of the coordinate frame corresponding to the gyroscope and accelerometer axes indicated by the Razor IMU M0 silk screen.
    
~rate (double, default: 50)
  * Measurement sample (and publish) rate, in samples per second (Hz). Value must be one of: 10, 20, 30, 40, 50, 60, 70, 80, 90, or 100.

~gyro_fsr (double, default: 2000)
  * Gyroscope full-scale range, in degrees per second. Measurements saturate at angular rates above this value. Value must be one of: 250, 500, 1000, or 2000. Note that the orientation output (on topic imu/data) may be invalid if the full-scale range is anything but 2000.

~accel_fsr (double, default: 16)
  * Accelerometer full-scale range, in multiples of the gravitational constant g (~9.8 m/s^2). Measurements saturate at accelerations above this value. Value must be one of: 2, 4, 8, or 16.
    
~g (double, default: 9.80665)
  * Gravitational acceleration constant, in meters per second squared. Used to convert between sensor measurements reported multiples of g's to meters per second squared required by the sensor_msgs/Imu message type.

~linear_acceleration_stddev (double, default: 0.0)
  * Linear acceleration measurement uncertainty to be reported in sensor_msgs/Imu messages on the imu/data_raw and imu/data topics. Square root of the linear acceleration covariance matrix diagonal elements, in m/s^2. Default value is 0 to indicate that the uncertainty is unknown.

~angular_velocity_stddev (double, default: 0.0)
  * Angular velocity measurement uncertainty to be reported in sensor_msgs/Imu messages on the imu/data_raw and imu/data topics. Square root of the angular velocity covariance matrix diagonal elements, in rad/s. Default value is 0 to indicate that the uncertainty is unknown.

~magnetic_field_stddev (double, default: 0.0)
  * Magnetic field measurement uncertainty to be reported in sensor_msgs/MagneticField messages on the imu/mag topic. Square root of the magnetic field covariance matrix diagonal elements, in Tesla. Default value is 0 to indicate that the uncertainty is unknown.

~orientation_stddev (double, default: 0.0)
  * Orientation measurement uncertainty to be reported in sensor_msgs/Imu messages on the imu/data topic. Square root of the orientation covariance matrix diagonal elements, in rad. Default value is 0 to indicate that the uncertainty is unknown.

## factory_firmware_driver_nodelet
factory_firmware_driver_nodelet is a nodelet that provides the same functionality and API as factory_firmware_driver_node when loaded in a nodelet manager.

# Contract Acknowledgment
Distribution Statement A. Approved for public release. Distribution is unlimited. 

This material is based upon work supported under Air Force Contract No. FA8721-05-C-0002 and/or FA8702-15-D-0001. Any opinions, findings, conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of the U.S. Air Force.

Copyright 2018 Massachusetts Institute of Technology.

The software/firmware is provided to you on an As-Is basis

Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013 or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. Government rights in this work are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above. Use of this work other than as specifically authorized by the U.S. Government may violate any copyrights that exist in this work.

