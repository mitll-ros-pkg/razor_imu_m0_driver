cmake_minimum_required(VERSION 2.8.3)
project(razor_imu_m0_driver)

## Add support for C++11, supported in ROS Kinetic and newer
#add_definitions(-std=c++11)

find_package(Boost REQUIRED COMPONENTS regex)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  pluginlib
  roscpp
  sensor_msgs
  serial
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS nodelet pluginlib roscpp sensor_msgs serial
)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# node executable
add_executable(factory_firmware_driver_node
  src/factory_firmware_driver.cpp
  src/factory_firmware_driver_node.cpp)
add_dependencies(factory_firmware_driver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(factory_firmware_driver_node
  ${catkin_LIBRARIES}
)

# nodelet library
add_library(razor_imu_m0_driver_factory_firmware_driver_nodelet
  src/factory_firmware_driver.cpp
  src/factory_firmware_driver_nodelet.cpp)
add_dependencies(razor_imu_m0_driver_factory_firmware_driver_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(razor_imu_m0_driver_factory_firmware_driver_nodelet
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS factory_firmware_driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS razor_imu_m0_driver_factory_firmware_driver_nodelet
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(PROGRAMS scripts/create_udev_rules
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES nodelet_plugin.xml
              razor_imu_m0_driver.rules
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

#############
## Testing ##
#############

# TODO
