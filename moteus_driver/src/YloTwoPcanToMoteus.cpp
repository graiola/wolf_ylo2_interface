/*
Copyright (c) 08/2022, Vincent FOUCAULT
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "moteus_driver/YloTwoPcanToMoteus.hpp"

YloTwoPcanToMoteus::YloTwoPcanToMoteus()
{

  pcanPorts_.resize(4); // resize the pcanports_ vector to the number or real ports.

  // NOTE: we should load that from file
  motor_adapters_.resize(12);  // exact motors order, on Ylo2

  //                   IDX                             SIGN                            REDUCTION                          PCAN BOARD PORTS
  // LF
  /*HAA*/ motor_adapters_[0].setIdx(3);  motor_adapters_[0].setSign(-1); motor_adapters_[0].setPort(PCAN_DEV1);
  /*HFE*/ motor_adapters_[1].setIdx(1);  motor_adapters_[1].setSign(-1); motor_adapters_[1].setPort(PCAN_DEV1);
  /*KFE*/ motor_adapters_[2].setIdx(2);  motor_adapters_[2].setSign(-1); motor_adapters_[2].setPort(PCAN_DEV1);

  // RF
  /*HAA*/ motor_adapters_[3].setIdx(9);  motor_adapters_[3].setSign(1); motor_adapters_[3].setPort(PCAN_DEV3);
  /*HFE*/ motor_adapters_[4].setIdx(7);  motor_adapters_[4].setSign(-1);  motor_adapters_[4].setPort(PCAN_DEV3);
  /*KFE*/ motor_adapters_[5].setIdx(8);  motor_adapters_[5].setSign(-1);  motor_adapters_[5].setPort(PCAN_DEV3);

  // LH
  /*HAA*/ motor_adapters_[6].setIdx(6);  motor_adapters_[6].setSign(-1);  motor_adapters_[6].setPort(PCAN_DEV2);
  /*HFE*/ motor_adapters_[7].setIdx(4);  motor_adapters_[7].setSign(1); motor_adapters_[7].setPort(PCAN_DEV2);
  /*KFE*/ motor_adapters_[8].setIdx(5);  motor_adapters_[8].setSign(1); motor_adapters_[8].setPort(PCAN_DEV2);

  // RH
  /*HAA*/ motor_adapters_[9].setIdx(12);  motor_adapters_[9].setSign(1);  motor_adapters_[9].setPort(PCAN_DEV4);
  /*HFE*/ motor_adapters_[10].setIdx(10); motor_adapters_[10].setSign(1); motor_adapters_[10].setPort(PCAN_DEV4);
  /*KFE*/ motor_adapters_[11].setIdx(11); motor_adapters_[11].setSign(1); motor_adapters_[11].setPort(PCAN_DEV4);

  // The default ID for the power_dist is '32'
  /*power_board*/ motor_adapters_[32].setIdx(32); motor_adapters_[32].setPort(PCAN_DEV3);

  /* ------------------------ TX STOP PACKAGE ------------------------------ */
  _stop.ID = 0x00;
  _stop.MSGTYPE = PCAN_MESSAGE_BRS | PCAN_MESSAGE_EXTENDED | PCAN_MESSAGE_FD;
  _stop.DLC = 7;
  _stop.DATA[0] = 0x01; // Write uint8 (0x00) | Write 1 register (0x01)
  _stop.DATA[1] = 0x00; // Register to write: MODE
  _stop.DATA[2] = 0x00; // Value to write: STOPPED MODE
  _stop.DATA[3] = 0x1F; // Read floats (0x1C) | Read 3 registers (0x03)
  _stop.DATA[4] = 0x01; // Starting register: POSITION, VELOCITY, TORQUE
  _stop.DATA[5] = 0x1F; // Read floats (0x1C) | Read 3 registers (0x03)
  _stop.DATA[6] = 0x0D; // Starting register: VOLTAGE, TEMPERATURE, FAULT

  /* ------------------------- TX ZERO PACKAGE ------------------------------ */
  _zero.ID = 0x00;
  _zero.MSGTYPE = PCAN_MESSAGE_BRS | PCAN_MESSAGE_EXTENDED | PCAN_MESSAGE_FD;
  _zero.DLC = 9; // 12 bytes ... ex : 0db0029a99193e    for a zero pos = 0.15
  _zero.DATA[0] = 0x0D; // write float (0x0C) | write 1 register (0x01)
  _zero.DATA[1] = 0xB0; // Register to write: 0x130(REZERO)
  _zero.DATA[2] = 0x02;
  _zero.DATA[7] = 0x1F; // Read floats (0x1C) | Read 3 registers (0x03)
  _zero.DATA[8] = 0x01; // Starting register: POSITION, VELOCITY, TORQUE
  _zero.DATA[9] = 0x1F; // Read floats (0x1C) | Read 3 registers (0x03)
  _zero.DATA[10] = 0x0D;// Starting register: VOLTAGE, TEMPERATURE, FAULT
  _zero.DATA[10] = 0x50;// pad unused bytes to 0x50

  /* --------------------------TX POS PACKAGE -------------------------------*/
  moteus_tx_msg.ID = 0x00;
  moteus_tx_msg.MSGTYPE = PCAN_MESSAGE_BRS | PCAN_MESSAGE_EXTENDED | PCAN_MESSAGE_FD;
  moteus_tx_msg.DLC = 14; // 13 = 32 bytes
  moteus_tx_msg.DATA[0] =  0x0c; // WRITE_REGISTERS - Type.F32
  moteus_tx_msg.DATA[1] =  0x06; // 6 registers
  moteus_tx_msg.DATA[2] =  0x20; // Starting at reg 0x020 POSITION
  moteus_tx_msg.DATA[3] =  _comm_position; // position value
  moteus_tx_msg.DATA[7] =  _comm_velocity; // velocity value
  moteus_tx_msg.DATA[11] = _comm_fftorque; // torque value
  moteus_tx_msg.DATA[15] = _comm_kp_scale; // Kp value
  moteus_tx_msg.DATA[19] = _comm_kd_scale; // Kd value
  moteus_tx_msg.DATA[23] = _comm_maxtorque; // Max torque
  moteus_tx_msg.DATA[27] = 0x1F; // READ_REGISTERS - F32 3 registers
  moteus_tx_msg.DATA[28] = 0x01; // Starting at reg 0x001(POSITION)
  moteus_tx_msg.DATA[29] = 0x13; // READ_REGISTERS - INT8 3 registers
  moteus_tx_msg.DATA[30] = 0x0D; // Starting at reg 0x00d(VOLTAGE)
  moteus_tx_msg.DATA[31] = 0x50; // Padding a NOP byte (moteus protocol)

  /* ------------------- TX POWER BOARD PACKAGE -----------------------------*/
  _power_board_tx_msg.ID = 0x00; // in moteus lib, for example 0x8002 means 80 = query values, and 02 = ID
  _power_board_tx_msg.MSGTYPE = PCAN_MESSAGE_BRS | PCAN_MESSAGE_EXTENDED | PCAN_MESSAGE_FD;
  _power_board_tx_msg.DLC = 3; // 3 bytes
  _power_board_tx_msg.DATA[0] = 0x1C; // Read floats (0x1C)
  _power_board_tx_msg.DATA[1] = 0x09; // Starting register: 0x000 STATE, FAULT CODE, SWITCH STATUS, LOCK TIME, BOOT TIME, OUT VOLT, OUT CURR, TEMP, ENERGY
  _power_board_tx_msg.DATA[2] = 0x00;
  // TODO mauvaises adresses !!!
  //--------------------------------------------------------------------------------------------
}

YloTwoPcanToMoteus::~YloTwoPcanToMoteus()
{
}

bool YloTwoPcanToMoteus::init_and_reset(){
  for (unsigned int p = 0; p < 4; p++){
    // open ports
    Status = CAN_InitializeFD(pcanPorts_[p], BitrateFD);
    CAN_GetErrorText(Status, 0, strMsg); // check the Status return state. If correct, it should return 0.
	  if (Status){std::cout << "Error: can't initialize " << pcanPorts_[p] << " port. Status = " << strMsg << std::endl;
      return(false);}
    usleep(200);

    //reset ports
    Status = CAN_Reset(pcanPorts_[p]);
    CAN_GetErrorText(Status, 0, strMsg);
    if(Status){std::cout << "Error: can't reset_buffer. " << pcanPorts_[p] << " port. Status = " << strMsg << std::endl;
      return(false);}
  }
  return(true);
}

bool YloTwoPcanToMoteus::send_moteus_stop_order(int id, int port){
    _stop.ID = 0x8000 | id;
    Status = CAN_WriteFD(port, &_stop);
    usleep(200);
    CAN_GetErrorText(Status, 0, strMsg);
    if(Status != PCAN_ERROR_OK){std::cout << "Error: can't stop motor " << id << " Status = " << strMsg << std::endl;
      return(false);}
    return(true);
}

bool YloTwoPcanToMoteus::send_moteus_TX_frame(int id, int port, float fftorque){
    _comm_fftorque = fftorque;
    moteus_tx_msg.ID = 0x8000 | id;
    memcpy(&moteus_tx_msg.DATA[11], &_comm_fftorque, sizeof(float)); // 0.0
    //std::cout<<("commands to send to moteus : ");
	//std::copy(std::begin(moteus_tx_msg.DATA), std::end(moteus_tx_msg.DATA), std::ostream_iterator<int>(std::cout, " "));
	//std::cout << "" << std::endl;
    Status = CAN_WriteFD(port, &moteus_tx_msg);
    usleep(200);
    CAN_GetErrorText(Status, 0, strMsg);
    if(Status == PCAN_ERROR_OK){return(true);}
    std::cout << "error into send_moteus_TX_frame() YloTwoPcanToMoteus function : id=" << id << ". Status = " << strMsg << std::endl;
    return(false);
}

bool YloTwoPcanToMoteus::read_moteus_RX_queue(int id, int port, float& position, float& velocity, float& torque, float& voltage, float& temperature, float& fault){
    moteus_rx_msg.ID = 0x8000 | id;
    Status = CAN_ReadFD(port,&moteus_rx_msg, NULL); // read can port
    usleep(200);
    //std::cout<<("read_moteus_RX_queue : ");
	  //std::copy(std::begin(moteus_rx_msg.DATA), std::end(moteus_rx_msg.DATA), std::ostream_iterator<int>(std::cout, " "));
	  //std::cout << "" << std::endl;
    CAN_GetErrorText(Status, 0, strMsg);
    if(Status != PCAN_ERROR_QRCVEMPTY){ // rx queue feeded.
        memcpy(&_position, &moteus_rx_msg.DATA[MSGRX_ADDR_POSITION], sizeof(float));
        memcpy(&_velocity, &moteus_rx_msg.DATA[MSGRX_ADDR_VELOCITY], sizeof(float));
        memcpy(&_torque,   &moteus_rx_msg.DATA[MSGRX_ADDR_TORQUE],   sizeof(float));
        memcpy(&_voltage, &moteus_rx_msg.DATA[MSGRX_ADDR_VOLTAGE], sizeof(float));
        memcpy(&_temperature, &moteus_rx_msg.DATA[MSGRX_ADDR_TEMPERATURE], sizeof(float));
        memcpy(&_fault,   &moteus_rx_msg.DATA[MSGRX_ADDR_FAULT],   sizeof(float));
        position    = _position;   
        velocity    = _velocity;
        torque      = _torque;
        voltage     = _voltage;
        temperature = _temperature;
        fault       = _fault;
        return true;}
    else
        return false;
}

/*  ZERO - Set Output Nearest
    When sent, this causes the servo to select a whole number of internal motor rotations 
    so that the final position is as close to the given position as possible*/
bool YloTwoPcanToMoteus::send_moteus_zero_order(int id, int port, float zero_position){
    _zero.ID = 0x8000 | id;
    _comm_position = zero_position;
    memcpy(&_zero.DATA[3], &_comm_position, sizeof(float));
    Status = CAN_WriteFD(port,&_zero);
    usleep(200);
    CAN_GetErrorText(Status, 0, strMsg);
    if(Status == PCAN_ERROR_OK){
        return true;}
    else{
        ROS_INFO("--ERROR IN WRITING : send_moteus_zero_order()--");
        return false;}   
}



/* POWER BOARD */
/* WRITE */
bool YloTwoPcanToMoteus::send_power_board_order(){
    _power_board_tx_msg.ID = 0x8000 | 32;
    auto port = PCAN_DEV3;
    Status = CAN_WriteFD(port, &_power_board_tx_msg);
    usleep(200);
    CAN_GetErrorText(Status, 0, strMsg);
    if(Status == PCAN_ERROR_OK){
        return(true);}
    else{
        std::cout << "error into send_power_board_order(). Status = " << strMsg << std::endl;
        return false;}
}


/* READ */
bool YloTwoPcanToMoteus::read_power_board_RX_queue(float& state, float& fault_code, float& switch_status, float& lock_time, 
                                            float& boot_time, float& out_volt, float& out_curr, float& board_temp, float& energy){
    _power_board_rx_msg.ID = 0x8000 | 32;
    //auto port = PCAN_DEV3;
    int port  = motor_adapters_[7].getPort();
    Status = CAN_ReadFD(port,&_power_board_rx_msg, NULL); // read can port
    usleep(200);
    CAN_GetErrorText(Status, 0, strMsg);
    std::cout << Status;
    if(Status != PCAN_ERROR_QRCVEMPTY){ // rx queue feeded.
        memcpy(&_state, &_power_board_rx_msg.DATA[MSGPBRX_ADDR_STATE], sizeof(float));
        memcpy(&_fault_code, &_power_board_rx_msg.DATA[MSGPBRX_ADDR_FAULT_CODE], sizeof(float));
        memcpy(&_switch_status,   &_power_board_rx_msg.DATA[MSGPBRX_ADDR_SWITCH_STATUS],   sizeof(float));
        memcpy(&_lock_time, &_power_board_rx_msg.DATA[MSGPBRX_ADDR_LOCK_TIME], sizeof(float));
        memcpy(&_boot_time, &_power_board_rx_msg.DATA[MSGPBRX_ADDR_BOOT_TIME], sizeof(float));
        memcpy(&_out_volt,   &_power_board_rx_msg.DATA[MSGPBRX_ADDR_OUT_VOLTAGE],   sizeof(float));
        memcpy(&_out_curr,   &_power_board_rx_msg.DATA[MSGPBRX_ADDR_OUT_CURRENT],   sizeof(float));
        memcpy(&_board_temp,   &_power_board_rx_msg.DATA[MSGPBRX_ADDR_TEMPERATURE],   sizeof(float));
        memcpy(&_energy,   &_power_board_rx_msg.DATA[MSGPBRX_ADDR_ENERGY],   sizeof(float));
        state = _state;   
        fault_code = _fault_code;
        switch_status = _switch_status;
        lock_time = _lock_time;
        boot_time = _boot_time;
        out_volt = _out_volt;
        out_curr = _out_curr;
        board_temp = _board_temp;
        energy = _energy;
        return true;
    }    
    else 
        return false;  
}

bool YloTwoPcanToMoteus::peak_fdcan_board_initialization(){
    if(!YloTwoPcanToMoteus::init_and_reset()){ // run and check the return of the function
        all_moteus_controllers_ok = false;
        ROS_INFO("--PEAK BOARD ERROR - can't send Initialization frame to can port--");
        return false;}
    else {
        ROS_INFO("--MOTEUS INITIALIZATION & RESET-> OK--");
        usleep(200);
        stop_motors();
        all_moteus_controllers_ok = true;
        usleep(200);
        return true;}
}

bool YloTwoPcanToMoteus::stop_motors(){
    for(unsigned int i=0; i<12;i++){
        auto ids = YloTwoPcanToMoteus::motor_adapters_[i].getIdx();
        int port  = YloTwoPcanToMoteus::motor_adapters_[i].getPort();
        // send a stop order, to avoid damages, and query its values.
        if(!YloTwoPcanToMoteus::send_moteus_stop_order(ids, port)){
            all_moteus_controllers_ok = false;
            ROS_INFO("-- PEAK BOARD ERROR - can't send Stop_command to id %d --", ids);
            return false;}
    }
    all_moteus_controllers_ok = true;
    ROS_INFO("--MOTEUS MOTORS STOPPED --------> OK--");
    usleep(200);
    return true;

}

bool YloTwoPcanToMoteus::check_initial_ground_pose(){
    // startup.
    int count = 0; // check zero for all 12 motors
    std::cout << ("\n-------------------------------------------------------------") << std::endl;
    std::cout << ("--  Zeroing joints. angle_joint tolerance is < 15 degrees  --") << std::endl;
    std::cout << ("-------------------------------------------------------------\n") << std::endl;

    while(count != 12){
        // --- LOOPING WITH THE 12 MOTORS UNTIL SUCCESS---
        count = 0;
        for(unsigned int i=0; i<12; i++){
            auto ids = YloTwoPcanToMoteus::motor_adapters_[i].getIdx();
            int port  = YloTwoPcanToMoteus::motor_adapters_[i].getPort();
            auto target_joint_position = initial_ground_joints_pose[i];
                
            // --- SENDING ZERO COMMAND ---
            if(!YloTwoPcanToMoteus::send_moteus_zero_order(ids, port, target_joint_position)){
                ROS_INFO("--- Error in send_moteus_zero_order() process. ---");
                can_error = true;
                return false;}
            usleep(200);

            // --- QUERYING VALUES ---
            if(!YloTwoPcanToMoteus::read_moteus_RX_queue(ids, port, RX_pos, RX_vel, RX_tor, RX_volt, RX_temp, RX_fault)){
                ROS_INFO("--- Error in read_moteus_RX_queue() process, on id %d. ---", ids);
                can_error = true;
                return false;}
            usleep(200);

            // --- CHECKING JOINT STARTUP ANGLE ---
            float value = (std::abs(RX_pos) - std::abs(target_joint_position));
            float angle_error = value*360;
            ROS_INFO("--Controleur ID : %d ; actual position : %f ; zero target position : %f ; difference : %f ; angle error in degrees : %f degrees", ids, RX_pos, target_joint_position, value, angle_error);
            if(std::abs(std::abs(RX_pos) - std::abs(target_joint_position)) > std::abs(calibration_error)){
                is_calibrated = false;
                ROS_INFO("-- Bad initial pose. Check motor %d.",ids);
                count -=1;
            }
            count +=1;
        }
        usleep(200);
    }
    std::cout << ("") << std::endl;
    ROS_INFO("--ROBOT CALIBRATION CHECKED ----> OK--");
    std::cout << ("") << std::endl;
}
