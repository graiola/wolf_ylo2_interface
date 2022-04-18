#include "moteus_driver/moteus_pcan_interface.h"

MoteusPcanInterface::MoteusPcanInterface(const std::string& interface, const std::vector<int>& ids)
    : _initialized(false)
    , _running(false)
    , _fail_count(0)
    , _interface(interface)
{
    // CAN Configuration
    _can_config.bitrate = 1e6; //1mbps
    _can_config.d_bitrate = 2e6; //2mbps
    _can_config.sample_point = .875; //87.5% 
    _can_config.d_sample_point = 0.8; //60%
    _can_config.clock_freq = 80e6; // 80mhz // Read from driver?  
    _can_config.mode_fd = 1; // FD Mode
    // Open CAN
    if(!_can_device.Open(interface, _can_config, false))
    {
        std::cout <<" error in initialization ports. "<< std::endl;
        return;
    }

    _can_device.ClearFilters();

    // Motors
    for(const auto& id: ids){
        _motors[id] = std::make_shared<MoteusPcanMotor>(id, &_can_device);
        // std::cout <<_motors[id]<< std::endl;  OK
    }
    // Everything ok
    _initialized = true;
    std::cout <<" port initialized and filtered. "<< std::endl;
    return;
}

MoteusPcanInterface::~MoteusPcanInterface(){} // destructor

bool MoteusPcanInterface::is_initialized(){
    // std::cout <<" is_initialized "<< std::endl;  OK
    return _initialized;
}

bool MoteusPcanInterface::is_running(){
    //std::lock_guard<std::mutex> guard(_running_mutex);
    std::cout <<" is_running "<< std::endl;
    return _running;
}

void MoteusPcanInterface::stop(){
    {
        //std::lock_guard<std::mutex> guard(_running_mutex);
        _running = false;
    }
    // _loop_thread->join();
}
