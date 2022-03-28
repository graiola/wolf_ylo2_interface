#ifndef ylo2_ROBOT_HW_H
#define ylo2_ROBOT_HW_H

#include <wolf_ylo2_interface/wolf_robot_hw.h>
//#include "myahrs+/myahrs+_imu.hpp" // the imu code
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace ylo22ros
{

class ylo2RobotHw : public hardware_interface::RobotHW, public hardware_interface::WolfRobotHwInterface
{
public:
  ylo2RobotHw();
  virtual ~ylo2RobotHw();

  void init();
  void read();
  void write();

private:

  /** @brief Sends a zero command to the robot */
  void send_zero_command();

  /** @brief Executes the robot's startup routine */
  void startup_routine();

};

}

#endif
