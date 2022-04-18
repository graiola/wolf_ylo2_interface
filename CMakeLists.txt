cmake_minimum_required(VERSION 3.0.2)
project(wolf_ylo2_interface)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  realtime_tools
  wolf_hardware_interface
  tf
)
      
catkin_package(
 CATKIN_DEPENDS
   controller_manager
   realtime_tools
   wolf_hardware_interface
)

##########
# Build ##
##########

include_directories(
  moteus_driver/include
  include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(moteus_driver)

add_executable(ylo2_ros_node
    src/ylo2_robot_hw.cpp
    src/ylo2_ros_control.cpp
    src/ylo2_ros_node.cpp)
target_link_libraries(ylo2_ros_node moteus_driver ${catkin_LIBRARIES})
set_property(TARGET ylo2_ros_node PROPERTY CXX_STANDARD 11)

add_executable(ylo2_imu_node
    src/ylo2_imu_node.cpp)
target_link_libraries(ylo2_imu_node ${catkin_LIBRARIES})
set_property(TARGET ylo2_imu_node PROPERTY CXX_STANDARD 11)


#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ylo2_ros_node ylo2_imu_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY params/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params
  PATTERN ".svn" EXCLUDE)

