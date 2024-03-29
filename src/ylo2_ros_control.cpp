#include "wolf_ylo2_interface/ylo2_ros_control.hpp"


namespace ylo22ros {


ylo2ROSControl::ylo2ROSControl()
{

}


ylo2ROSControl::~ylo2ROSControl()
{

}


void ylo2ROSControl::init(bool dry_run)
{
	// Reset RobotHW
	robot_hw_.reset(new ylo22ros::ylo2RobotHw);

  // Reseting the namespace of the node handle
  node_handle_.reset(new ros::NodeHandle(robot_hw_->getRobotName()));

	// Initializing the hardware interface
  robot_hw_->init(*node_handle_.get());

	// Reseting the controller manager
	controller_manager_.reset(new controller_manager::ControllerManager(robot_hw_.get(), *node_handle_.get()));
}


void ylo2ROSControl::update(const ros::Time& time, const ros::Duration& period)
{
	// Reading sensor information
	robot_hw_->read();
	//usleep(200);
	// Updating the controller manager
	controller_manager_->update(time, period);

	// Writing to the actuator
	robot_hw_->write();
	//usleep(200);
}


}
