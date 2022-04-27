#ifndef MOTEUS_PCAN_CONTROLLER_H
#define MOTEUS_PCAN_CONTROLLER_H

#include <iostream>
#include <string>
#include <vector>
#include <map>

#define KEY_IN_MAP(_key, _map) _map.count(_key)

#include "moteus_pcan_interface.h"

typedef std::shared_ptr<MoteusPcanInterface> MoteusPcanInterfacePtr;
typedef std::map<std::string, std::vector<int>> MoteusInterfaceMotorsMap;

class MoteusPcanController{
public:

    MoteusPcanController(const MoteusInterfaceMotorsMap& interface_motors_map);
    ~MoteusPcanController();
    bool is_initialized();
    void start();
    bool all_running();

    //bool set_torque_ena(bool torque_ena);
    //bool set_torque_ena(bool torque_ena, int id);

    // send to individual controller ID
    bool set_command(int id, float fftorque);

    std::map<int, MoteusPcanMotorPtr> _motors; // Volver privado de nuevo
private:
    bool _initialized;
    MoteusInterfaceMotorsMap _interface_motors_map;
    std::vector<MoteusPcanInterfacePtr> _interfaces;
};

typedef std::shared_ptr<MoteusPcanController> MoteusPcanControllerPtr;

#endif
