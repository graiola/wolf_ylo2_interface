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

    {"lf_haa_joint",    "PCAN_PCIBUS1",    1,      0.00,     true},
    {"lf_hfe_joint",    "PCAN_PCIBUS1",    2,      0.00,     true},
    {"lf_kfe_joint",    "PCAN_PCIBUS1",    3,      0.00,     true},

    // rf_leg
    {"rf_haa_joint",    "PCAN_PCIBUS2",    4,      0.00,     true},
    {"rf_hfe_joint",    "PCAN_PCIBUS2",    5,      0.00,     true},
    {"rf_kfe_joint",    "PCAN_PCIBUS2",    6,      0.00,     true},

    // lh_leg
    {"lh_haa_joint",    "PCAN_PCIBUS3",    7,      0.00,     true},
    {"lh_hfe_joint",    "PCAN_PCIBUS3",    8,      0.00,     true},
    {"lh_kfe_joint",    "PCAN_PCIBUS3",    9,      0.00,     true},

    // rh_leg
    {"rh_haa_joint",    "PCAN_PCIBUS4",   10,      0.00,     true},
    {"rh_hfe_joint",    "PCAN_PCIBUS4",   11,      0.00,     true},
    {"rh_kfe_joint",    "PCAN_PCIBUS4",   12,      0.00,     true}
};


// PEAK FDCAN PCI M2 has 4 ports and each port controls one leg (3 moteus_controllers)
MoteusInterfaceMotorsMap interface_motors_map = {

    {"PCAN_PCIBUS1", {1}}, {"PCAN_PCIBUS1", {2}}, {"PCAN_PCIBUS1", {3}},

    {"PCAN_PCIBUS2", {4}}, {"PCAN_PCIBUS2", {5}}, {"PCAN_PCIBUS2", {6}},

    {"PCAN_PCIBUS3", {7}}, {"PCAN_PCIBUS3", {8}}, {"PCAN_PCIBUS3", {9}},

    {"PCAN_PCIBUS4", {10}}, {"PCAN_PCIBUS4", {11}}, {"PCAN_PCIBUS4", {12}}
};

MoteusPcanController controller(interface_motors_map);


// torque switch on/off, and target ID 
void set_torque(bool choice, int motor_id)
    {
    auto state = choice;
    auto id = motor_id;
    // send pcan order, using correct port (ex:PCAN_PCIBUS1), target id, and state

    // TODO : controller._motors[moteus_id] calls the target id, and it specific can_port ??
    controller._motors[id]->set_torque_ena(state); 
    std::cout("torque switch on/off activated.");
    }



// query position, velocity, and torque, for all 12 motors in order (1-12)
int query()
    {
    for(const auto& motor_info: motors_info) // for the list of the 12 id's
        {
        int motor_id = motor_info.can_id; // each id
        float pos, vel, tor;
        controller._motors[motor_id]->get_feedback(pos, vel, tor); // query values
        }
        list commands = ;
        return (controller._motors);
    }


// send fftorque order to specific id, with specific torque
void send_tau(int motor_id, float torque)
    {
        auto id = motor_id;
        float fftorque = torque;
        controller._motors[id]->set_commands(fftorque);
    }


int main(int argc, char **argv)
{
    // initializing all 12 moteus controllers. 
    // Opens all ports present in interface_motors_map ?
    // pass initialization if port already open ? (ex: i have 3 times same port !)
    if(!controller.is_initialized()){
        std::cout("Could not initialize Moteus controllers.");
        return 1;
        }

    // start all 12 moteus controllers
    controller.start();
    
    // check if all 12 moteus controllers are running
    if(!controller.all_running())
        {
        std::cout("One or more Moteus controllers are not running.");
        break;
        }

    std::cout("motors running !!!");

    int query_pos_vel_tor = query(); // query_pos_vel_tor is a list like this : query_pos_vel_tor = {{.1, .1, .1}, {.1, .1, .1}, {.1, .1, .1}....} for all 12 motors
    // or perhaps easier : MoteusPcanMotor::get_feedback(&position, &velocity, &torque)

    usleep(2000000); // sleep 2s

    set_torque(true, 11); // activate torque for motor 11
    // or perhaps easier : MoteusPcanController::set_torque_ena(true);

    usleep(2000000);

    send_tau(11, 4.0); // send TAU (fftorque) order to motor 11, with a torque of 4.0
    // or perhaps easier : MoteusPcanController::set_command(11, 4.0)

    usleep(2000000);

    set_torque(false, 11); // desactivate torque for motor 11
    // or perhaps easier : MoteusPcanController::set_torque_ena(false);
}
