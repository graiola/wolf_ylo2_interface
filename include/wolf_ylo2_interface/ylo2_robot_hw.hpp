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

struct MotorAdapter
{

public:

  MotorAdapter()
  {
    idx_ = -1;
    sign_ = 1;
    reduction_ = 1.0;
  }

  MotorAdapter(int idx, int sign, double reduction)
  {
    idx_ = idx;
    sign_ = sign;
    reduction_ = reduction;
  }

  const int& getIdx() {return idx_;}
  const int& getSign() {return sign_;}
  const double& getReduction() {return reduction_;}

  void setIdx(int idx) {idx_ = idx;}
  void setSign(int sign) {sign_ = sign;}
  void setReduction(double reduction) {reduction_ = reduction;};

private:

  int idx_;
  int sign_;
  double reduction_;
};

class ylo2RobotHw : public hardware_interface::RobotHW, public hardware_interface::WolfRobotHwInterface
{
public:
  ylo2RobotHw();
  virtual ~ylo2RobotHw();

  void init(const ros::NodeHandle &nh, bool dry_run = false);
  void read();
  void write();

private:

  void read_rx_queue(int motor_id, float& pos, float& vel, float& tor);

  void send_tau(int motor_id, float tor);

  void stop(int motor_id);

  void send_query_only_command(int motor_id);

  /** @brief moteus controller, enable communication with the motors (default is false) */
  MoteusPcanControllerPtr motors_interface_;

  std::vector<MotorAdapter> motor_adapters_;

  /** @brief map to access specific Peak port, regarding to queried Id */
  MoteusInterfaceMotorsMap motors_interface_map_ =
  {
    {"/dev/pcanpcifd0", {1,2,3,}}, {"/dev/pcanpcifd1", {4,5,6,}}, {"/dev/pcanpcifd2", {7,8,9,}}, {"/dev/pcanpcifd3", {10,11,12,}}
  };

  /** @brief IMU realtime publisher */
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::Imu>> imu_pub_;

  /** @brief Sends a zero command to the robot */
  void send_zero_command();

  /** @brief Executes the robot's startup routine */
  void startup_routine();
  std::mutex startup_routine_mutex;

  float tmp_pos_;
  float tmp_vel_;
  float tmp_tor_;

};

}

#endif
