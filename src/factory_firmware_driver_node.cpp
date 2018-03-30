#include <ros/ros.h>

#include "razor_imu_m0_driver/factory_firmware_driver.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_driver");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  razor_imu_m0_driver::FactoryFirmwareDriver driver(nh, private_nh);

  ros::spin();

  std::cout << "exiting" << std::endl;
  return 0;
}
