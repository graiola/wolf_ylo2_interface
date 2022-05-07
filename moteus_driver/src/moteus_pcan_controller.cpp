#include "moteus_driver/moteus_pcan_controller.h"
MoteusPcanController::MoteusPcanController(const MoteusInterfaceMotorsMap& interface_motors_map) // string and vectors
    : _initialized(false)
    , _interface_motors_map(interface_motors_map)
{
    for (auto const& [interface, ids] : interface_motors_map)
    {
        _interfaces.push_back(std::make_shared<MoteusPcanInterface>(interface, ids));
        if(!_interfaces.back()->is_initialized()){
            std::cerr << "[ERROR]: " << "Unable to initialize CAN interface: " << interface << std::endl;
            return;
        }
        for(const auto& [id, motor]: _interfaces.back()->_motors){
            if(!KEY_IN_MAP(id, _motors)){
                _motors[id] = motor;
            }else{
                std::cerr << "[ERROR]: " << "Mutiple definition of ID: " << id << std::endl;
                return;
            }
        }
    }
    _initialized = true;
    return;
}

MoteusPcanController::~MoteusPcanController(){
    for(const auto& interface: _interfaces){
        interface->stop();
    }
}

bool MoteusPcanController::is_initialized(){
    return _initialized;
}

bool MoteusPcanController::set_command(int id, float fftorque){
    if(KEY_IN_MAP(id, _motors)){
        _motors[id]->set_commands(fftorque);
    }else{
        std::cerr << "[ERROR]: " << "ID not defined: " << id << std::endl;
        return false;
    }
    return true;
}

bool MoteusPcanController::all_running(){
    for(const auto& interface: _interfaces){
        if(!interface->is_running()){
            return false;
        }
    }
    return true;
}

