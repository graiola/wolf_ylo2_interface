cmake_minimum_required(VERSION 3.5)
project(ylo2_moteus)

if(NOT CMAKE_CXX_STANDARD)
  # set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CAN_LIBS pthread rt pcanbasic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include/)

find_package(ament_cmake REQUIRED)

file(GLOB MOTEUS_PCAN_SRC src/moteus_pcan/*.cpp)

add_executable(ylo2_moteus_node src/ylo2_moteus_node.cpp ${MOTEUS_PCAN_SRC})
target_link_libraries(ylo2_moteus_node ${CAN_LIBS})
ament_target_dependencies(ylo2_moteus_node ) #rclcpp std_msgs sensor_msgs k3lso_msgs)

install(TARGETS
ylo2_moteus_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
