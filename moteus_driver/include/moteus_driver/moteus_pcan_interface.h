#ifndef MOTEUS_PCAN_INTERFACE_H
#define MOTEUS_PCAN_INTERFACE_H

#include <iostream>
#include <thread>
#include <vector>
#include <map>

#include "PCANDevice.h"
#include "moteus_pcan_motor.h"

typedef std::shared_ptr<MoteusPcanMotor> MoteusPcanMotorPtr;

class MoteusPcanInterface{
public:
    MoteusPcanInterface(const std::string& interface, const std::vector<int>& ids);
    ~MoteusPcanInterface();
    bool is_initialized();
    bool is_running();
    //void start();
    void stop();

    std::map<int, MoteusPcanMotorPtr> _motors;
    int _freq;
private:
    bool _initialized;
    bool _running;
    int _fail_count;
    std::mutex _running_mutex;
    std::string _interface;
    PCANDevice _can_device;
    CANDevice::Config_t _can_config;
    //std::shared_ptr<std::thread> _loop_thread;
    //void loop();
};

#endif
