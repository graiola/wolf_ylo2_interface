#ifndef YLO2_ROBOT_HW_H
#define YLO2_ROBOT_HW_H

#include <wolf_hardware_interface/wolf_robot_hw.h>
#include "moteus_driver/YloTwoPcanToMoteus.hpp"
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace ylo22ros{

class ylo2RobotHw : public hardware_interface::RobotHW, public hardware_interface::WolfRobotHwInterface{

public:

  ylo2RobotHw();
  virtual ~ylo2RobotHw();
  void init(const ros::NodeHandle &nh, bool dry_run = false);
  void read();
  void write();

private:

  /** @brief IMU realtime publisher */
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::Imu>> imu_pub_;

  /** @brief Sends a zero command to the robot */
  void send_zero_command();

  /** @brief Executes the robot's startup routine */
  void startup_routine();
  std::mutex startup_routine_mutex;

};

} // end namespace

#endif
