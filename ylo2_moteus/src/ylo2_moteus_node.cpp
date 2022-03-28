#include <iostream>
#include <string.h>
#include <sstream>
#include <list>
#include <iostream>
#include <unistd.h> // used for usleep command

#include "moteus_pcan/moteus_pcan_controller.h"

using namespace std;

std::vector<float> unique_feedback(3); // one vector per motor (pos, vel, tor)
std::vector<float> &_unique_feedback(unique_feedback); // reference
std::vector<vector<float>> global_feedback{}; // one vector for all motors, containing the 12 individual vectors
std::vector<vector<float>> &_global_feedback(global_feedback); // reference

// PEAK FDCAN PCI M2 has 4 ports and each port controls one leg (3 moteus_controllers)
MoteusInterfaceMotorsMap interface_motors_map = {
  
  {"/dev/pcanpcifd0", {1,2,3,}},
  {"/dev/pcanpcifd1", {4,5,6,}},
  {"/dev/pcanpcifd2", {7,8,9,}},
  {"/dev/pcanpcifd3", {10,11,12,}},
};

MoteusPcanController controller(interface_motors_map);


// torque switch on/off, and target ID 
void activate_torque_cmd(int motor_id, bool activate)
{
  // send pcan order, using correct port (ex:PCAN_PCIBUS1), target id, and state
  controller._motors[motor_id]->set_torque_ena(activate);
}

// query position, velocity, and torque, for all 12 motors in order (1-12)
void query(int motor_id, float& pos, float& vel, float& tor)
{
  controller._motors[motor_id]->get_feedback(pos, vel, tor); // query values
}

// build a vector(12) with the return of all 12 motors
std::vector<vector<float>> global_query(float& pos, float& vel, float& tor)
{
  for (unsigned int i=1; i<13; i++)
  {
    controller._motors[i]->get_feedback(pos, vel, tor); // query each motor in the range 1-12
    _unique_feedback[0] = pos;
    _unique_feedback[1] = vel;
    _unique_feedback[2] = tor;
    _global_feedback.push_back(_unique_feedback); // insert actual vector in the global vector
  }
  return _global_feedback; // return the global vector, containing all queries for the 12 motors, in order 1-12, and pos,vel,tor in each
  // global_feedback = {{pos, vel, tor}, {pos, vel, tor}, {pos, vel, tor}...}
}

// send fftorque order to specific id, with specific torque
void send_tau(int motor_id, float tor)
{
  controller._motors[motor_id]->set_commands(tor);
}


int main()
{

  float pos, vel, tor;

  if(!controller.is_initialized()){
    std::cerr << "Could not initialize Moteus controllers." << std::endl;
    return 1;
  }

  // start all 12 moteus controllers, and check them
  controller.start();
  if(!controller.all_running())
  {
    std::cerr << "One or more Moteus controllers are not running." << std::endl;
    return 1;
  }

  //std::cout << "Motors are running !!!" << std::endl;

  usleep(1000); // need to select a correct usleep delay, otherwise, rx queue will not be fully feeded !

  /*
  while(true)
  {
  std::vector<vector<float>> read = global_query(pos, vel, tor); // query all 12 motors

  // Displaying the 2D vector
  for (int i = 0; i < 12; i++)
  {
        for (auto it = read[i].begin(); it != read[i].end(); it++)
            cout << *it << " ";
        cout << endl;
  }
  usleep(1);
  }
  */

  query(8, pos, vel, tor);
  std::cout << "Position : " << pos << std::endl;
  std::cout << "velocity : " << vel << std::endl;
  std::cout << "torque : " << tor << std::endl;

}
