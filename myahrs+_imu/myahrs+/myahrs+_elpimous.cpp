
#include "myahrs_plus.hpp"

//------------------------------------------------------------------------------
using namespace Ylo2MyahrsImu;

//------------------------------------------------------------------------------
class AhrsDriver : public iMyAhrsPlus
{
private:

  Platform::Mutex lock_;
  SensorData sensor_data_;

  void OnSensorData(int sensor_id, SensorData data)
  {
    LockGuard _l(lock_);
    sensor_data_ = data;
    publish(sensor_id);
  }
  
  void OnAttributeChange(int sensor_id, std::string attribute_name, std::string value)
  {
    printf("OnAttributeChange(id %d, %s, %s)\n", sensor_id, attribute_name.c_str(), value.c_str());
  }

public:
  AhrsDriver(std::string port="/dev/ttyACM1", int baud_rate=115200)
  : iMyAhrsPlus(port, baud_rate)
  {}

  // initialization of imu variables
  float quaternion[4] = {};
  float angular_velocity[3] = {};
  float linear_acceleration[3] = {};
  float temp;
  
  ~AhrsDriver()
  {}

  bool initialize()
  {
    bool ok = false;

    do
    {
      if(start() == false) break;
      //Euler angle(x, y, z axis)
      //IMU(linear_acceleration, angular_velocity, magnetic_field)
      if(cmd_binary_data_format("EULER, IMU") == false) break;
      // 100Hz
      if(cmd_divider("1") == false) break;
      // Binary and Continue mode
      if(cmd_mode("BC") == false) break;
      ok = true;
    } while(0);

    return ok;
  }

  inline void get_data(SensorData& data)
  {
    LockGuard _l(lock_);
    data = sensor_data_;
  }

  inline SensorData get_data()
  {
    LockGuard _l(lock_);
    return sensor_data_;
  }

  void publish(int sensor_id,)
  {

    Quaternion& q = sensor_data->quaternion;
    ImuData<float>& imu = sensor_data_.imu;

    // Quaternion
    quaternion = {q.x, q.y, q.z, q.w};
    
    // Angular
    angular_velocity = {imu.gx, imu.gy, imu.gz};
    
    // Linear
    linear_acceleration = {imu.ax, imu.ay, imu.az};

    // celsius unit
    temp = imu.temperature;

  }
};


//------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  std::string port = std::string("/dev/ttyACM1");
  int baud_rate    = 115200;

  while(true)
  {
      AhrsDriver sensor(port, baud_rate);

      if(sensor.initialize() == false)
      {
        std::cout("Initialize() returns false, please check your devices.");
        return 0;
      }
      else
      {
        std::cout("Initialization OK!\n");
      }

      return 0;
  }
}

//------------------------------------------------------------------------------
