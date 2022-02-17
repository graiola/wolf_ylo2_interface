# Ylo2 moteus controllers package

Based on (find the original moteus lib)

## Dependencies

Install pcandriver and pcanbasic: https://github.com/raess1/K3lso-CAN-communication
## Thanks to Robin Frojd, for it public package

moteus_pcan_motor.h   contient les adresses hexa des données, ainsi que la fonction read/write

_motors[id] = std::make_shared<MoteusPcanMotor>(id, &_can_device);       dans moteus_pcan_interface

controller._motors[id]->set_torque_ena(state);
controller._motors[id]->set_commands(float position);
controller._motors[id]->get_feedback(float& position, float& velocity, float& torque);


write_read() envoie et recoit les commandes

            memcpy(&_msg_tx_pos.data[MSGTX_ADDR_FFTORQUE], &_comm_fftorque, sizeof(float));
            ...
            _can_device_ptr->Send(_msg_tx_pos);


            memcpy(&_position, &_msg_rx.data[MSGRX_ADDR_POSITION], sizeof(float));
            memcpy(&_velocity, &_msg_rx.data[MSGRX_ADDR_VELOCITY], sizeof(float));
            memcpy(&_torque,   &_msg_rx.data[MSGRX_ADDR_TORQUE],   sizeof(float));
            _can_device_ptr->Receive(_msg_rx);




---------------------------------------------------------------------------------------------------------

rappel fonctionnement pcan driver :

#define PCAN_DEVICE		PCAN_PCIBUS1
unsigned int pcan_device = PCAN_DEVICE;

Status = CAN_Initialize(pcan_device, PCAN_BAUD_500K, 0, 0, 0);
	Message.ID = 0x8001;    // 80 means query, and 01 is motor ID
	Message.LEN = 8;
    ...
while ((Status = CAN_Write(pcan_device, &Message)) == PCAN_ERROR_OK) {boucle jusqu'a ce que ca passe}

CAN_Uninitialize(pcan_device);


---------------------------------------------------------------------------------------------------------

bool PCANDevice::Open(const std::string &device_id, Config_t &config, bool bUseRXThread)
{
    fd_ = pcanfd_open(device_id.c_str(), OFD_BITRATE | OFD_BRPTSEGSJW | OFD_DBITRATE | OFD_BRPTSEGSJW | OFD_CLOCKHZ | PCANFD_INIT_FD, 1, 50, 29, 10, 1, 8, 7, 12, config.clock_freq);
    if (fd_ < 0)
    {
        perror("[ERROR]: PCANDevice::Open: Failed to Open PCANFD.");
        return false;
    }
    // Clear Filters
    ClearFilters();

---------------------------------------------------------------------------------------------------------

bool PCANDevice::Send(uint32_t dest_id, uint8_t *data, uint16_t length)

bool PCANDevice::Send(CAN_msg_t &msg)

bool PCANDevice::Receive(CAN_msg_t &msg)

bool PCANDevice::Close()

bool PCANDevice::AddFilter(uint32_t from, uint32_t to)

bool PCANDevice::ClearFilters()



_interface(interface)
if(!_can_device.Open(interface, _can_config, false))

MoteusPcanInterface(const std::string& interface, const std::vector<int>& ids);
for (auto const& [interface, ids] : interface_motors_map)


std::vector<MotorInfo> motors_info = {
    // joint_name;   can_interface;   can_id;   offset;    invert;
    {"fl_abad",      "PCAN_PCIBUS1",    1,      -0.00,     true}, // Hip
    {"fl_upper_leg", "PCAN_PCIBUS1",    2,       0.92,     true}, // Leg
    {"fl_lower_leg", "PCAN_PCIBUS1",    3,  -1.483529864,  true}, // Low Leg
    ...
};


MoteusInterfaceMotorsMap interface_motors_map = {
    {"PCAN_PCIBUS1", {1}},
    {"PCAN_PCIBUS1", {2}},
    {"PCAN_PCIBUS1", {3}},
    ...
};


MoteusPcanController controller(interface_motors_map);


memos :

_interfaces.push_back(valeur),  ajoute "valeur" au conteneur de liste existant "_interfaces"
_interfaces.back(),             retourne la derniere valeur de la liste "_interfaces"

a->b   est similaire à  "a.b" mais concerne les pointeurs   soit : (pointer_name)->(variable_name)

