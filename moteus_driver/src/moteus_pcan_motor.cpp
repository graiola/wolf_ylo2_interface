// File adapted for Ylo2 robot, for pure torque commands
#include "moteus_driver/moteus_pcan_motor.h"
// TX
#define MSGTX_ADDR_POSITION 0x06
#define MSGTX_ADDR_VELOCITY 0x0A
#define MSGTX_ADDR_FFTORQUE 0x0E
#define MSGTX_ADDR_KP_SCALE 0x12
#define MSGTX_ADDR_KD_SCALE 0x16
#define MSGTX_ADDR_MAXTORQU 0x1A

// RX
#define MSGRX_ADDR_POSITION 0x02
#define MSGRX_ADDR_VELOCITY 0x06
#define MSGRX_ADDR_TORQUE   0x0A

using namespace std::chrono_literals;

MoteusPcanMotor::MoteusPcanMotor(uint32_t id, PCANDevice* can_device_ptr)
    : _id(id)
    , _can_device_ptr(can_device_ptr)
    //, _torque_ena(false)
    , _fail(0)
{
    
    // TX STOP PACKAGE
    _msg_tx_stop.id = 0x8000 | id; // in moteus lib, for example 0x8002 means 80 = query values, and 02 = ID
    _msg_tx_stop.length = 5;
    // Write Mode
    _msg_tx_stop.data[0] = 0x01; // Write uint8 (0x00) | Write 1 register (0x01)
    _msg_tx_stop.data[1] = 0x00; // Register to write: MODE
    _msg_tx_stop.data[2] = 0x00; // Value to write: STOPPED MODE
    // Query
    _msg_tx_stop.data[3] = 0x1F; // Read floats (0x1C) | Read 3 registers (0x03)
    _msg_tx_stop.data[4] = 0x01; // Starting register: POSITION, VELOCITY, TORQUE

    // -----------------------------------------------------------------------------

    // TX POS PACKAGE for Tau mode only (torque, kp=0, kd=0)
    _tx_msg.id = 0x8000 | id;
    _tx_msg.length = 32;
    // Write Mode
    _tx_msg.data[0] = 0x01; // Write uint8 (0x00) | Write 1 register (0x01)
    _tx_msg.data[1] = 0x00; // Register to write: MODE
    _tx_msg.data[2] = 0x0A; // Value to write: POSITION MODE
    // Write command
    _tx_msg.data[3] = 0x0C; // Write floats
    _tx_msg.data[4] = 0x06; // Write 6 registers
    _tx_msg.data[5] = 0x20; // Starting register: POSITION_COMM, VELOCITY_COMM, FFTORQUE_COMM, KP_SCALE, KD_SCALE, MAX_TORQUE
    // Query
    _tx_msg.data[30] = 0x1F; // Read floats (0x1C) | Read 3 registers (0x03)
    _tx_msg.data[31] = 0x01; // Starting register: POSITION, VELOCITY, TORQUE

    // Initial values
    _comm_position = 0.0;
    _comm_velocity = 0.0;
    _comm_fftorque = 1.5;
    _comm_kp_scale = 0.0;
    _comm_kd_scale = 0.0;
    _comm_maxtorqu = 2.0; // NaN for max
}

MoteusPcanMotor::~MoteusPcanMotor(){}

bool MoteusPcanMotor::set_stop_commands(){
    //std::lock_guard<std::mutex> guard(_command_mutex);
    if(!_can_device_ptr->Send(_msg_tx_stop)){
        std::cout << "Stop command error" << std::endl;
        return false;
    };
    return true;
}

bool MoteusPcanMotor::set_commands(float fftorque){
    //std::lock_guard<std::mutex> guard(_command_mutex);
    _comm_fftorque = fftorque;
    memcpy(&_tx_msg.data[MSGTX_ADDR_POSITION], &_comm_position, sizeof(float));
    memcpy(&_tx_msg.data[MSGTX_ADDR_VELOCITY], &_comm_velocity, sizeof(float));
    memcpy(&_tx_msg.data[MSGTX_ADDR_FFTORQUE], &_comm_fftorque, sizeof(float));
    memcpy(&_tx_msg.data[MSGTX_ADDR_KP_SCALE], &_comm_kp_scale, sizeof(float));
    memcpy(&_tx_msg.data[MSGTX_ADDR_KD_SCALE], &_comm_kd_scale, sizeof(float));
    memcpy(&_tx_msg.data[MSGTX_ADDR_MAXTORQU], &_comm_maxtorqu, sizeof(float));

    if(!_can_device_ptr->Send(_tx_msg)){
        std::cout << "Sent command error" << std::endl;
        return false;
    };
    //std::cout << "Sent command success" << std::endl;
    return true;
}

bool MoteusPcanMotor::get_feedback(float& position, float& velocity, float& torque){
    //std::lock_guard<std::mutex> guard(_feedback_mutex);
    while(true){
        if(!_can_device_ptr->Receive(_rx_msg)){ // si erreur,
            return false;
        }
        memcpy(&_position, &_rx_msg.data[MSGRX_ADDR_POSITION], sizeof(float));
        memcpy(&_velocity, &_rx_msg.data[MSGRX_ADDR_VELOCITY], sizeof(float));
        memcpy(&_torque,   &_rx_msg.data[MSGRX_ADDR_TORQUE],   sizeof(float));
        position = _position;   
        velocity = _velocity;
        torque = _torque;
        return true;
    }
}