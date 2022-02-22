# Ylo2 moteus controllers package

Hey, Gennaro
I propose U this md file to share infos, or anything else about ylo2 controller code.

----------------------------------------------------------------------------------------

- CANDevice and PCANDevice are the proprietary PEAK driver, replacing ubuntu socketcan,

- moteus_pcan_xxx are a mod of the original moteus_controller library,

- ylo2_moteus_node.cpp is/should be  my gateway from moteus controller to Wolf controller.

## 1 / due to the fact that for each leg, the 3 motors are connected to the same PCAN port, we need an interface_motors_map{}

## 2 / my functions :


    void set_torque(bool choice, int motor_id) 

            this function receives the true/false option to torque, and the target motor id
            when asking the id, the function automatically selects the correct port (controller._motors[id])
            Don't know if this function should return anything ?


    int query()

            this function query all motors contained in motor_info list
            it queries position, velocity and fftorque
            it return form should be : {{pos,vel,tor}, {pos,vel,tor},......}
                                            motor1        motor2     ......


    void send_tau(int motor_id, float torque)

            this function send TAU to target id, with a target force
            when asking the id, the function automatically selects the correct port (controller._motors[id])
            Don't know if this function should return anything ?

## 3 / General functions :

    controller.is_initialized()

            this function initialize all 4 PCAN ports (PCAN_PCIBUS1-4)
    
    controller.start()

            Is this function acting as a guard ? I failed to decrypt the function (too hard for my knowledge)
    
    controller.all_running()

            this function checks that the 4 PCAN_PCIBUS are always opened, I think (lol)
    


Well, I do my best to make a gateway, learning the existing libraries, with my actual c++ knowledge.
I don't anderstand anything about pointers, ~, and memory leaks, sorry.  (if only i could code python, lol)

Impatient to read U Gennaro