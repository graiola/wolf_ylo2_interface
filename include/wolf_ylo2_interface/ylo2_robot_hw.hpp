#ifndef YLO2_ROBOT_HW_H
#define YLO2_ROBOT_HW_H

#include <wolf_hardware_interface/wolf_robot_hw.h>
#include "moteus_driver/moteus_pcan_controller.h"
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace ylo22ros
{

class ylo2RobotHw : public hardware_interface::RobotHW, public hardware_interface::WolfRobotHwInterface
{
public:
  ylo2RobotHw();
  virtual ~ylo2RobotHw();

  void init(const ros::NodeHandle &nh);
  void read();
  void write();

  float pos = 0, vel = 0, tor = 0;

private:

  // normal rotation motors id's
  std::set<int> norm_rot_ids {5, 6, 7, 10, 11, 12}; // others need to reverse

  // @brief Map Ylo2 internal joint indices to WoLF joints order
  std::array<unsigned int, 12> ylo2_motor_idxs_
          {
          1, 2, 3,    // LF
          7, 8, 9,    // LH
          4, 5, 6,    // RF
          10, 11, 12, // RH
          };

  /** @brief IMU realtime publisher */
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::Imu>> imu_pub_;

  /** @brief Sends a zero command to the robot */
  void send_zero_command();

  /** @brief Executes the robot's startup routine */
  void startup_routine();

};

}

#endif
