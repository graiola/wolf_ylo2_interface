#include "wolf_ylo2_interface/ylo2_robot_hw.hpp"

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

ylo2RobotHw::ylo2RobotHw()
{
  robot_name_ = "ylo2";
}

ylo2RobotHw::~ylo2RobotHw()
{

}

//-------------------//
// MOTEUS CONTROLLER //
//-------------------//

// PEAK FDCAN PCI M2 has 4 ports and each port controls one leg (3 moteus_controllers)
MoteusInterfaceMotorsMap interface_motors_map = 
{
  {"/dev/pcanpcifd0", {1,2,3,}}, {"/dev/pcanpcifd1", {4,5,6,}}, {"/dev/pcanpcifd2", {7,8,9,}}, {"/dev/pcanpcifd3", {10,11,12,}}
};

MoteusPcanController controller(interface_motors_map);

// query position, velocity, and torque, for all 12 motors in order (1-12)
void query(int motor_id, float& pos, float& vel, float& tor)
{
  controller._motors[motor_id]->get_feedback(pos, vel, tor); // query values
}

// send fftorque order to specific id, with specific torque
void send_tau(int motor_id, float tor)
{
  controller._motors[motor_id]->set_commands(tor);
}

// send a stop order to specific id
void stop(int motor_id)
{
  controller._motors[motor_id]->set_stop_commands();
}

// -----------------------------------------------------


void ylo2RobotHw::init(const ros::NodeHandle& nh)
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
  // send a stop command to all motors, to feed RX queue.
  for (unsigned int jj = 0; jj < n_dof_; ++jj){
    stop(jj+1);
  }
  std::cout << "all stop command sent." << std::endl;
}


void ylo2RobotHw::read()
{
  for (unsigned int jj = 0; jj < n_dof_; ++jj)
  {
    query(jj+1, pos, vel, tor); // query values;
    joint_position_[jj] = pos*6;// measured in revolutions, with a 6x reduction
    joint_velocity_[jj] = vel;   // measured in revolutions / s
    joint_effort_[jj]   = tor;   // measured in N*m
    //std::cout << jj+1 << "pos = " << pos << std::endl;
  }

  /*
  for (unsigned int jj = 0; jj < n_dof_; ++jj)
  {
    joint_position_[jj] = 0.0;
    joint_velocity_[jj] = 0.0;
    joint_effort_[jj]   = 0.0;
    //std::cout << jj+1 << "pos = " << pos << std::endl;
  }
  */


  // Publish the IMU data NOTE: missing covariances
  if(imu_pub_.get() && imu_pub_->trylock())
  {
    imu_pub_->msg_.orientation.w         = imu_orientation_[0];
    imu_pub_->msg_.orientation.x         = imu_orientation_[1];
    imu_pub_->msg_.orientation.y         = imu_orientation_[2];
    imu_pub_->msg_.orientation.z         = imu_orientation_[3];
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

void ylo2RobotHw::write()
{
  for (unsigned int jj = 0; jj < n_dof_; ++jj)
    {
      //send_tau(jj+1, static_cast<float>(joint_effort_command_[jj])/8);
      send_tau(jj+1, 0.5);
      //std::cout << jj+1 << "tau = " << static_cast<float>(joint_effort_command_[jj]) << std::endl;
    }
}

void ylo2RobotHw::send_zero_command()
{
  std::array<float, 60> zero_command = {0};
  // ylo2_interface_->SendCommand(zero_command); is equal to  // ylo2_interface_.SendCommand(zero_command); but for pointer
  // replace each ylo2_interface_. to moteus ones
  //TODO fonction zeroing_command
}

void ylo2RobotHw::startup_routine()
{
  send_zero_command();
}

} // namespace
