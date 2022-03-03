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


std::vector<MotorInfo> motors_info = {

  // joint_name;      can_interface;   can_id;   offset;   invert;
  // lf_leg

  // TODO : joint_names aren't used for now.
  // is my can_interface call correct ?

  {"lf_haa_joint",    "/dev/pcanpcifd0",    1,      0.00,     true},
  {"lf_hfe_joint",    "/dev/pcanpcifd0",    2,      0.00,     true},
  {"lf_kfe_joint",    "/dev/pcanpcifd0",    3,      0.00,     true},

  // rf_leg
  {"rf_haa_joint",    "/dev/pcanpcifd1",    4,      0.00,     true},
  {"rf_hfe_joint",    "/dev/pcanpcifd1",    5,      0.00,     true},
  {"rf_kfe_joint",    "/dev/pcanpcifd1",    6,      0.00,     true},

  // lh_leg
  {"lh_haa_joint",    "/dev/pcanpcifd2",    7,      0.00,     true},
  {"lh_hfe_joint",    "/dev/pcanpcifd2",    8,      0.00,     true},
  {"lh_kfe_joint",    "/dev/pcanpcifd2",    9,      0.00,     true},

  // rh_leg
  {"rh_haa_joint",    "/dev/pcanpcifd3",   10,      0.00,     true},
  {"rh_hfe_joint",    "/dev/pcanpcifd3",   11,      0.00,     true},
  {"rh_kfe_joint",    "/dev/pcanpcifd3",   12,      0.00,     true}
};


// PEAK FDCAN PCI M2 has 4 ports and each port controls one leg (3 moteus_controllers)
MoteusInterfaceMotorsMap interface_motors_map = {

  {"/dev/pcanpcifd0", {1}}, {"/dev/pcanpcifd0", {2}}, {"/dev/pcanpcifd0", {3}},

  {"/dev/pcanpcifd1", {4}}, {"/dev/pcanpcifd1", {5}}, {"/dev/pcanpcifd1", {6}},

  {"/dev/pcanpcifd2", {7}}, {"/dev/pcanpcifd2", {8}}, {"/dev/pcanpcifd2", {9}},

  {"/dev/pcanpcifd3", {10}}, {"/dev/pcanpcifd3", {11}}, {"/dev/pcanpcifd3", {12}}
};

MoteusPcanController controller(interface_motors_map);


// torque switch on/off, and target ID 
void activate_torque_cmd(int motor_id, bool activate)
{
  // send pcan order, using correct port (ex:PCAN_PCIBUS1), target id, and state
  // TODO : controller._motors[moteus_id] calls the target id, and it specific can_port ??
  controller._motors[motor_id]->set_torque_ena(activate);
  std::cout << "activated torque cmd for motor_id: " << motor_id << std::endl;
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

  //std::cout << "TEST" << std::endl;

  // parse the motor_id
  //int motor_id = -1;
  int motor_id = 99; // trying with SDK motor.

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

  usleep(2000000); // sleep 2s

  activate_torque_cmd(motor_id,true); // activate torque for motor 99
  // or perhaps easier : MoteusPcanController::set_torque_ena(true);

  usleep(2000000);

  // not sure it will work, think i need a loop ?!
  send_tau(motor_id,1.0); // send TAU (fftorque) order to motor 99, with a torque of 1.0 Nm

  usleep(2000000);

  activate_torque_cmd(motor_id,false); // desactivate torque for motor 99
  // or perhaps easier : MoteusPcanController::set_torque_ena(false);

  usleep(2000000);

}

