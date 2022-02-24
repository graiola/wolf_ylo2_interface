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

/*------------------------------------           Terminal return          --------------------------------------/

0.18310.61040.79350.61040.36621.09871.40390.73240.0610-0.1221
0.42730.30520.06100.18310.67140.85450.30520.12210.1831-0.3052
-0.5493-0.3662-0.18310.00000.24410.30520.00000.12210.48830.2441
-0.18310.18310.48830.30520.12210.79351.28180.97660.91561.0376
0.85450.42730.24410.48830.67140.2441-0.24410.12210.91560.9766
0.54930.0000-0.12210.0000-0.1221-0.7324-0.24410.3052-0.0610-0.3662
-0.18310.18310.67140.30520.24410.12210.30520.30520.18310.4883
                                                                    ( missing spaces between values! see L65 )

  The order is quaternion, linear_acceleration, angular_velocity ( in Wolf order )
/--------------------------------------------------------------------------------------------------------------*/

// imu variable initialization :
// ------------------------------

std::vector<double> imu_feedback(10); // 11 if temperature added

/*
void ex3_callback_attribute(void* context, int sensor_id, const char* attribute_name, const char* value)
{
    printf(" ## sensor_id %d, Attribute has been changed(%s, %s)\n", sensor_id, attribute_name, value);
}
*/

// std::vector<double> callback_data(void* context, int sensor_id, SensorData* sensor_data)
void callback_data(void* context, int sensor_id, SensorData* sensor_data)
{
    Quaternion& q = sensor_data->quaternion;
    ImuData<float>& imu = sensor_data->imu;

    // without magnet values, neither temp
    imu_feedback = {q.x, q.y, q.z, q.w, imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz};

    // without magnet values, but with temp
    //imu_feedback = {q.x, q.y, q.z, q.w, imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz, imu.temperature};

    // with magnet values and temp
    // imu_feedback = (q.x, q.y, q.z, q.w, imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz, imu.mx, imu.my, imu.mz, imu.temperature); 

    //printf("%.4f", imu_feedback_old);  // doesn't make space between values, 'cause no space in imu_feedback variable
    //printf("%.4f", imu_feedback); // testing with  a vector double of 10 elements.  Why it returns only 000 ?

    // return (imu_feedback);
    std::cout << (imu_feedback[9]);
}


void feedback(const char* serial_device, int baudrate)
{
    //printf("\nSTARTING IMU FEEDBACK...\n");

    MyAhrsPlus sensor;

    int sample_counter = 0;

    /*
     * 	register a callback function to attribute changed event.
     */
    //sensor.register_attribute_callback(ex3_callback_attribute, 0);

    /*
     * 	register a callback function to new data arrived event.
     */
    sensor.register_data_callback(callback_data, &sample_counter);

    /*
     * 	start communication with the myAHRS+.
     */
    if(sensor.start(serial_device, baudrate) == false) {
        handle_error("start() returns false, check ADRESS !!! or be sure to have done : sudo chmod 666 /dev/ttyACM0");
    }

    /*
     *  set binary output format
     *   - select Quaternion and IMU data
     */
    if(sensor.cmd_binary_data_format("QUATERNION, IMU") == false) {
        handle_error("cmd_binary_data_format() returns false");
    }

    /*
     *  set divider
     *   - output rate(Hz) = max_rate/divider
     */
    if(sensor.cmd_divider(DIVIDER) ==false) {
        handle_error("cmd_divider() returns false");
    }

    /*
     *  set transfer mode
     *   - BC : BINARY Message & Continuous mode
     */
    if(sensor.cmd_mode("BC") ==false) {
        handle_error("cmd_mode() returns false");
    }

    while(true) // never stop ??!
    {
        Platform::msleep(100);
    }

    /*
     * 	stop communication
     */
    sensor.stop();
}


/******************************************************************************************************************************
 *
 *  RUN
 *
 ******************************************************************************************************************************/

int main(int argc, char* argv[]) {

    feedback(SERIAL_DEVICE, BAUDRATE);
    return 0;
}

