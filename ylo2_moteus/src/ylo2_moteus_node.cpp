#include <iostream>
#include <string.h>
#include <sstream>

#include "moteus_pcan/moteus_pcan_controller.h"

using namespace std::chrono_literals;

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


// les moteurs 1,2 et 3 sont connectés au bus PCAN_PCIBUS1 de la carte PEAK FDCAN PCI M2...
MoteusInterfaceMotorsMap interface_motors_map = {

    {"PCAN_PCIBUS1", {1}}, {"PCAN_PCIBUS1", {2}}, {"PCAN_PCIBUS1", {3}},

    {"PCAN_PCIBUS2", {4}}, {"PCAN_PCIBUS2", {5}}, {"PCAN_PCIBUS2", {6}},

    {"PCAN_PCIBUS3", {7}}, {"PCAN_PCIBUS3", {8}}, {"PCAN_PCIBUS3", {9}},

    {"PCAN_PCIBUS4", {10}}, {"PCAN_PCIBUS4", {11}}, {"PCAN_PCIBUS4", {12}}
};

MoteusPcanController controller(interface_motors_map);

/*
void set_torque(bool choice, int motor_id))
    {
        auto state = choice;
        auto id = request->ids[motor_id];
        controller._motors[id]->set_torque_ena(state, id);
        std::cout("torque activated on selected motor.")
    }

void query(int motor_id, float position, float velocity, float fftorque)
    {

    }

void send_tau(int motor_id, float torque)
    {

    }

*/

int main(int argc, char **argv)
{
    // initializing all 12 moteus controllers
    if(!controller.is_initialized()){
        std::cout("Could not initialize Moteus controllers.");
        return 1;
    }

    // start all 12 moteus controllers
    controller.start();
    
    // check if all 12 moteus controllers are running
    if(!controller.all_running()){
        std::cout("One or more Moteus controllers are not running.");
        break;
        }
    }
}
