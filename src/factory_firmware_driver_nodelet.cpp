#include <boost/shared_ptr.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "razor_imu_m0_driver/factory_firmware_driver.h"

namespace razor_imu_m0_driver
{
class FactoryFirmwareDriverNodelet : public nodelet::Nodelet
{
public:
  FactoryFirmwareDriverNodelet()
  {
  }

private:
  virtual void onInit(void);

  boost::shared_ptr<FactoryFirmwareDriver> driver_;

};  // class FactoryFirmwareDriverNodelet

void FactoryFirmwareDriverNodelet::onInit()
{
  NODELET_DEBUG("Initializing SparkFun 9DoF Razor IMU M0 factory firmware driver nodelet");
  driver_.reset(new FactoryFirmwareDriver(getNodeHandle(), getPrivateNodeHandle()));
}

}  // namespace razor_imu_m0_driver

PLUGINLIB_EXPORT_CLASS(razor_imu_m0_driver::FactoryFirmwareDriverNodelet, nodelet::Nodelet);
