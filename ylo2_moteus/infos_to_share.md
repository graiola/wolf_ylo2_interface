# Ylo2 moteus controllers package

Hey, Gennaro
I propose U this md file to share infos, or anything else about ylo2 controller code.

----------------------------------------------------------------------------------------

- CANDevice and PCANDevice are the proprietary PEAK driver, replacing ubuntu socketcan,

- moteus_pcan_xxx are a mod of the original moteus_controller library,

- ylo2_moteus_node.cpp is/should be  my gateway from moteus controller to Wolf controller.

1 / due to the fact that for each leg, the 3 motors are connected to the same PCAN port, we need an interface_motors_map{}

2 / functions :

    void set_torque(bool choice, int motor_id) 

            this function receives the true/false option to torque, and the target motor id
            when asking the id, the function automatically selects the correct port (controller._motors[id])
            Don't know if this function should return anything ?

    int query()

            this function query all motors contained in motor_info list
            it queries position, velocity and fftorque
            it return form should be : {{pos,vel,tor}, {pos,vel,tor},......}
                                            motor1        motor2     ......
                                            