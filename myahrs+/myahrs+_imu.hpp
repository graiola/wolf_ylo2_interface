#ifndef MYAHRS_LIB
#define MYAHRS_LIB

#include <stdio.h>
#include <stdlib.h>

#include <vector>
#include <map>

#include "myahrs_plus.hpp" // myahrs+ imu lib

using namespace WithRobot; // imu namespace lib

static const char* SERIAL_DEVICE = "/dev/ttyACM0"; // Ylo2 UP Xtreme  IMU USB port

static const int BAUDRATE = 115200;

static const char* DIVIDER = "1";  // 100 Hz

void handle_error(const char* error_msg)
{
    fprintf(stderr, "ERROR: %s\n", error_msg);
    exit(1);
}

// imu variable initialization :
std::vector<double> imu_data(10); // 11 if temperature added


// Read Imu datas into a vector<double>
// Datas order : 
//     0-3 : orientation
//     4-6 : angular_velocity
//     7-9 : linear_acceleration
// Return vector.
void callback_data(void* context, int sensor_id, SensorData* sensor_data)
{
    Quaternion& q = sensor_data->quaternion;
    ImuData<float>& imu = sensor_data->imu;

    // without magnet values, neither temp
    imu_data = {q.x, q.y, q.z, q.w, imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz};

    // return (imu_data);
    std::cout << (imu_data[9]);
    
}


void Imu_data(const char* serial_device, int baudrate)
{
    //printf("\nSTARTING IMU FEEDBACK...\n");
    MyAhrsPlus sensor;
    int sample_counter = 0;

    // register a callback function to attribute changed event.
    //sensor.register_attribute_callback(ex3_callback_attribute, 0);

    // register a callback function to new data arrived event.
    sensor.register_data_callback(callback_data, &sample_counter);

    // start communication with the myAHRS+.
    if(sensor.start(serial_device, baudrate) == false) {
        handle_error("start() returns false, check ADRESS !!! or be sure to have done : sudo chmod 666 /dev/ttyACM0");
    }

    // set binary output format
    // - select Quaternion and IMU data
    if(sensor.cmd_binary_data_format("QUATERNION, IMU") == false) {
        handle_error("cmd_binary_data_format() returns false");
    }


    // set divider
    // - output rate(Hz) = max_rate/divider
    if(sensor.cmd_divider(DIVIDER) ==false) {
        handle_error("cmd_divider() returns false");
    }

    // set transfer mode
    // - BC : BINARY Message & Continuous mode
    if(sensor.cmd_mode("BC") ==false) {
        handle_error("cmd_mode() returns false");
    }

    while(true) // never stop ??!
    {
        Platform::msleep(100);
    }

    // stop communication
    sensor.stop();
}

#endif