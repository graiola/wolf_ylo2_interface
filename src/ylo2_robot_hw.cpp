#include "wolf_ylo2_interface/ylo2_robot_hw.hpp"
#include <chrono>
#include <thread>

namespace ylo22ros
{
using namespace hardware_interface;

int64_t utime_now() {
  struct timeval timeofday;
  gettimeofday(&timeofday,NULL);
  if (timeofday.tv_sec < 0 || timeofday.tv_sec > UINT_MAX)
    throw std::runtime_error("Timeofday is out of dual signed 32-bit range");
  uint32_t sec	= timeofday.tv_sec;
  uint32_t nsec = timeofday.tv_usec * 1000;
  return (int64_t) (((uint64_t)sec)*1000000 + ((uint64_t)nsec) / 1000);
}

ylo2RobotHw::ylo2RobotHw(){
  robot_name_ = "ylo2";
}

ylo2RobotHw::~ylo2RobotHw(){}


YloTwoPcanToMoteus command; // instance of class YloTwoPcanToMoteus


void ylo2RobotHw::init(const ros::NodeHandle& nh, bool dry_run)
{
  // Hardware interfaces: Joints
  auto joint_names = loadJointNamesFromSRDF();
  if(joint_names.size()>0)
  {
    WolfRobotHwInterface::initializeJointsInterface(joint_names);
    registerInterface(&joint_state_interface_);
    registerInterface(&joint_effort_interface_);
  }
  else
  {
    ROS_ERROR_NAMED(CLASS_NAME,"Failed to register joint interface.");
    return;
  }
  // Hardware interfaces: IMU
    auto imu_name = loadImuLinkNameFromSRDF();
    if(!imu_name.empty())
    {
      WolfRobotHwInterface::initializeImuInterface(imu_name);
      registerInterface(&imu_sensor_interface_);
    }

  if(!dry_run)
      //std::cout << "Not exist anymore Program changed !" << std::endl;
     //motors_interface_ = std::make_shared<MoteusPcanController>(motors_interface_map_);

  ylo2RobotHw::startup_routine();
}

void ylo2RobotHw::startup_routine()
{
  command.peak_fdcan_board_initialization();
  usleep(200);
  command.check_initial_ground_pose();
  std::cout << "startup_routine Done." << std::endl;
  usleep(200);
}

void ylo2RobotHw::read()
{
  for (unsigned int jj = 0; jj < n_dof_; jj++)
  {
    // Reset values
    float RX_pos = 0.0;
    float RX_vel = 0.0;
    float RX_tor = 0.0;
    float RX_volt = 0.0;
    float RX_temp = 0.0;
    float RX_fault = 0.0;

    auto ids  = command.motor_adapters_[jj].getIdx();
    int port  = command.motor_adapters_[jj].getPort();
    auto sign = command.motor_adapters_[jj].getSign();

    command.read_moteus_RX_queue(ids, port, RX_pos, RX_vel, RX_tor, RX_volt, RX_temp, RX_fault);  // query values;
    joint_position_[jj] = static_cast<double>(sign*(RX_pos*2*M_PI)); // joint angle converted to radians
    joint_velocity_[jj] = static_cast<double>(RX_vel);   // measured in revolutions / s
    joint_effort_[jj]   = static_cast<double>(RX_tor);   // measured in N*m
    usleep(120);
  }

  // IMU OK !
  // Publish the IMU data NOTE: missing covariances
  if(imu_pub_.get() && imu_pub_->trylock())
  {
    imu_pub_->msg_.orientation.w         = imu_orientation_[0];
    imu_pub_->msg_.orientation.x         = - imu_orientation_[1];
    imu_pub_->msg_.orientation.y         = - imu_orientation_[2];
    imu_pub_->msg_.orientation.z         = - imu_orientation_[3];
    imu_pub_->msg_.angular_velocity.x    = imu_ang_vel_[0];
    imu_pub_->msg_.angular_velocity.y    = imu_ang_vel_[1];
    imu_pub_->msg_.angular_velocity.z    = imu_ang_vel_[2];
    imu_pub_->msg_.linear_acceleration.x = imu_lin_acc_[0];
    imu_pub_->msg_.linear_acceleration.y = imu_lin_acc_[1];
    imu_pub_->msg_.linear_acceleration.z = imu_lin_acc_[2];
    imu_pub_->msg_.header.stamp = ros::Time::now();
    imu_pub_->unlockAndPublish();

  }
}

void ylo2RobotHw::write(){
  for (unsigned int jj = 0; jj < n_dof_; jj++){
      auto ids  = command.motor_adapters_[jj].getIdx();
      auto sign = command.motor_adapters_[jj].getSign();
      int port  = command.motor_adapters_[jj].getPort();
      //command.send_moteus_TX_frame(ids, port, sign*static_cast<float>(joint_effort_command_[jj])); 
      usleep(120);
  }
  command.send_power_board_order();
}
// usefull command ?
void ylo2RobotHw::send_zero_command(){
  std::array<float, 60> zero_command = {0};
}
} // namespace
