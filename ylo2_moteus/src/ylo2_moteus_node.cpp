#include <iostream>
#include <string.h>
#include <sstream>
#include <list>
#include <iostream>
#include <unistd.h> // used for usleep command

#include "moteus_pcan/moteus_pcan_controller.h"

using namespace std;

struct MotorInfo{
  std::string joint_name;
  std::string can_interface;
  int can_id;
  double offset;
  bool invert;
};

// PEAK FDCAN PCI M2 has 4 ports and each port controls one leg (3 moteus_controllers)
MoteusInterfaceMotorsMap interface_motors_map = {
  
  {"/dev/pcanpcifd0", {1,2,3}},
  {"/dev/pcanpcifd1", {4,5,6,99}},
  {"/dev/pcanpcifd2", {7,8,9}},
  {"/dev/pcanpcifd3", {10,11,12}}
};

MoteusPcanController controller(interface_motors_map);


// torque switch on/off, and target ID 
void activate_torque_cmd(int motor_id, bool activate)
{
  // send pcan order, using correct port (ex:PCAN_PCIBUS1), target id, and state
  // TODO : controller._motors[moteus_id] calls the target id, and it specific can_port ??
  controller._motors[motor_id]->set_torque_ena(activate);
}

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

int main(int argc, char **argv)
{

  float pos, vel, tor;

  // parse the motor_id
  int motor_id = -1;
  if (argc == 2) {
    motor_id = stoi(argv[1]);
    std::cout << "Selected motor_id: " << motor_id << std::endl;
  }
  else {
    // graiola: in case we have motor_id=-1, we could activate all of them instead
    // of throwing an error.
    std::cerr << "Usage: " << argv[0] << " motor_id" << std::endl;
    return 1;
  }

  // initializing all 12 moteus controllers.
  // Opens all ports present in interface_motors_map ?
  // pass initialization if port already open ? (ex: i have 3 times same port !)
  // graiola: Can we initialize a single motor to test it? Do we have to initialize all of them?
  if(!controller.is_initialized()){
    std::cerr << "Could not initialize Moteus controllers." << std::endl;
    return 1;
  }

  // start all 12 moteus controllers
  controller.start();

  // check if all 12 moteus controllers are running
  if(!controller.all_running())
  {
    std::cerr << "One or more Moteus controllers are not running." << std::endl;
    return 1;
  }

  std::cout << "Motors are running !!!" << std::endl;

  activate_torque_cmd(motor_id,true); // activate torque for motor 11
  // or perhaps easier : MoteusPcanController::set_torque_ena(true);

  usleep(5000000);

  //send_tau(motor_id,0.2);
  controller._motors[motor_id]->set_commands(0.2);
  usleep(2000);

  controller._motors[motor_id]->get_feedback(pos, vel, tor);
  std::cout << "Position : " << pos << std::endl;
  std::cout << "velocity : " << vel << std::endl;
  std::cout << "torque : " << tor << std::endl;
  
  usleep(2000);


  activate_torque_cmd(motor_id,false); // desactivate torque for motor 11
  usleep(2000);
  // or perhaps easier : MoteusPcanController::set_torque_ena(false);
}
