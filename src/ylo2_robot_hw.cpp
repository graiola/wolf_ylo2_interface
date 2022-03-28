#include "wolf_ylo2_interface/ylo2_robot_hw.hpp"
//TODO #include "ylo2_moteus/???"

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

void ylo2RobotHw::init()
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
    else
    {
      ROS_ERROR_NAMED(CLASS_NAME,"Failed to register imu interface.");
      return;
    }


    // ylo2_interface_.InitCmdData(ylo2_lowcmd_);
    // replace each ylo2_interface_. to moteus ones
    //TODO fonction initialize
    startup_routine();
}

void ylo2RobotHw::read()
{
    // Get robot data
    // ------
    // Joints
    // ------
    for (unsigned int jj = 0; jj < n_dof_; ++jj)
    {
        joint_position_[jj] = 0.0;
        joint_velocity_[jj] = 0.0;
        joint_effort_[jj]   = 0.0;

        // read real values of moteus motors
        //query(jj+1, pos, vel, tor);
        //joint_position_[jj] = pos;
        //joint_velocity_[jj] = vel;
        //joint_effort_[jj]   = tor;
    }

    // ---
    // IMU
    // ---

    imu_orientation_[0] = 0.0;  // w
    imu_orientation_[1] = 0.0;  // x
    imu_orientation_[2] = 0.0;  // y
    imu_orientation_[3] = 0.0;  // z

    imu_ang_vel_[0] = 0.0;
    imu_ang_vel_[1] = 0.0;
    imu_ang_vel_[2] = 0.0;

    imu_lin_acc_[0] = 0.0;
    imu_lin_acc_[1] = 0.0;
    imu_lin_acc_[2] = 0.0;

    /*
    Imu_Data(SERIAL_DEVICE, BAUDRATE); // feed the reference variable _imu_data
    imu_orientation_[0] = _imu_data[0];
    imu_orientation_[1] = _imu_data[1];
    imu_orientation_[2] = _imu_data[2];
    imu_orientation_[3] = _imu_data[3];
    imu_ang_vel_[0] = _imu_data[4];
    imu_ang_vel_[1] = _imu_data[5];
    imu_ang_vel_[2] = _imu_data[6];
    imu_lin_acc_[0] = _imu_data[7];
    imu_lin_acc_[1] = _imu_data[8];
    imu_lin_acc_[2] = _imu_data[9];
    */
}

void ylo2RobotHw::write()
{
    //for (unsigned int jj = 0; jj < n_dof_; ++jj)
    //ylo2_lowcmd_.motorCmd[ylo2_motor_idxs_[jj]].tau = static_cast<float>(joint_effort_command_[jj]  );

    // write real torque value to moteus motors
    //for (unsigned int jj = 0; jj < n_dof_; ++jj)
    //  send_tau(jj+1, static_cast<float>(joint_effort_command_[jj]));
}

void ylo2RobotHw::send_zero_command()
{
    //std::array<float, 60> zero_command = {0};
    // ylo2_interface_->SendCommand(zero_command); is equal to  // ylo2_interface_.SendCommand(zero_command); but for pointer
    // replace each ylo2_interface_. to moteus ones
    //TODO fonction zeroing_command
}

void ylo2RobotHw::startup_routine()
{
    send_zero_command();
}

} // namespace
