#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>


void callback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0,0.0,0.0));
  tf::Quaternion q;
  q.setW(imu_msg->orientation.w);
  q.setX(imu_msg->orientation.x);
  q.setY(imu_msg->orientation.y);
  q.setZ(imu_msg->orientation.z);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "trunk", "world"));
}


int main(int argc, char** argv)
{
  std::string ns = "ylo2_imu_node";

  ros::init(argc, argv, ns);

  ros::NodeHandle n(ns); // load the relative namespace

  ros::Subscriber imu_subscriber = n.subscribe("/imu", 20, callback);

  ros::spin();

  return 0;
}
