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
  tmp_pos_ = tmp_vel_ = tmp_tor_ = 0.0;

  // NOTE: we should load that from file
  motor_adapters_.resize(12);
  // LF
  //                   IDX                             SIGN                            REDUCTION
  /*HAA*/ motor_adapters_[0].setIdx(1);   motor_adapters_[0].setSign(-1);  motor_adapters_[0].setReduction(6.0);
  /*HFE*/ motor_adapters_[1].setIdx(2);   motor_adapters_[1].setSign(-1);  motor_adapters_[1].setReduction(1.0);
  /*KFE*/ motor_adapters_[2].setIdx(3);   motor_adapters_[2].setSign(-1);  motor_adapters_[2].setReduction(1.0);
  // LH
  /*HAA*/ motor_adapters_[3].setIdx(7);   motor_adapters_[3].setSign(1);   motor_adapters_[3].setReduction(6.0);
  /*HFE*/ motor_adapters_[4].setIdx(8);   motor_adapters_[4].setSign(-1);  motor_adapters_[4].setReduction(1.0);
  /*KFE*/ motor_adapters_[5].setIdx(9);   motor_adapters_[5].setSign(-1);  motor_adapters_[5].setReduction(1.0);
  // RF
  /*HAA*/ motor_adapters_[6].setIdx(4);   motor_adapters_[6].setSign(-1);  motor_adapters_[6].setReduction(6.0);
  /*HFE*/ motor_adapters_[7].setIdx(5);   motor_adapters_[7].setSign(1);   motor_adapters_[7].setReduction(1.0);
  /*KFE*/ motor_adapters_[8].setIdx(6);   motor_adapters_[8].setSign(1);   motor_adapters_[8].setReduction(1.0);
  // RH
  /*HAA*/ motor_adapters_[9].setIdx(10);  motor_adapters_[9].setSign(1);   motor_adapters_[9].setReduction(6.0);
  /*HFE*/ motor_adapters_[10].setIdx(11); motor_adapters_[10].setSign(1);  motor_adapters_[10].setReduction(1.0);
  /*KFE*/ motor_adapters_[11].setIdx(12); motor_adapters_[11].setSign(1);  motor_adapters_[11].setReduction(1.0);
}

ylo2RobotHw::~ylo2RobotHw()
{
}

// query position, velocity, and torque, for all 12 motors in order (1-12)
void ylo2RobotHw::query(int motor_id, float& pos, float& vel, float& tor)
{
  if(motors_interface_)
    motors_interface_->_motors[motor_id]->get_feedback(pos, vel, tor); // query values
}

// send fftorque order to specific id, with specific torque
void ylo2RobotHw::send_tau(int motor_id, float tor)
{
  if(motors_interface_)
    motors_interface_->_motors[motor_id]->set_commands(tor);
}

// send a stop order to specific id
void ylo2RobotHw::stop(int motor_id)
{
  if(motors_interface_)
    motors_interface_->_motors[motor_id]->set_stop_commands();
}
//-----------------------------------------------------------


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

  if(!dry_run)
     motors_interface_ = std::make_shared<MoteusPcanController>(motors_interface_map_);
}


void ylo2RobotHw::read()
{

  for (unsigned int jj = 0; jj < n_dof_; ++jj)
  {

    // Reset
    tmp_vel_ = 0.0;
    tmp_tor_ = 0.0;

    auto idx = motor_adapters_[jj].getIdx();
    auto sign = motor_adapters_[jj].getSign();
    auto red = motor_adapters_[jj].getReduction();

    stop(idx);
    query(idx, tmp_pos_, tmp_vel_, tmp_tor_); // query values;

    joint_position_[jj] = static_cast<double>(sign*tmp_pos_*red);
    joint_velocity_[jj] = static_cast<double>(tmp_vel_);   // measured in revolutions / s
    joint_effort_[jj]   = static_cast<double>(tmp_tor_);   // measured in N*m
  }

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
      //send_tau(ylo2_motor_idxs_[jj], 0.5); // testing tau to all motors, OK
      //std::cout << jj+1 << "tau = " << static_cast<float>(joint_effort_command_[jj]) << std::endl;
    }
}

void ylo2RobotHw::send_zero_command()
{
  std::array<float, 60> zero_command = {0};
  //TODO fonction zeroing_command
}

void ylo2RobotHw::startup_routine()
{
  send_zero_command();
}

} // namespace
