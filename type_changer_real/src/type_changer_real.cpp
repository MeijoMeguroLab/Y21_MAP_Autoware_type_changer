#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include <math.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "sensor_msgs/Temperature.h"

static std::string sub_lidar_topic_name;
static std::string sub_twist_topic_name;
static std::string sub_imu_topic_name;
static std::string pub_lidar_topic_name;
static std::string pub_twist_topic_name;
static std::string pub_imu_topic_name;

ros::Publisher packets_pub;
ros::Publisher twist_pub;
ros::Publisher imu_pub;


void packetsCallback(const velodyne_msgs::VelodyneScan::ConstPtr& msg) {
  velodyne_msgs::VelodyneScan velodyne_packets;
  velodyne_packets.header = msg->header;
  velodyne_packets.packets = msg->packets;

  velodyne_packets.header.frame_id = "velodyne_top";
  std::cout<< "pub "<<std::endl;
  packets_pub.publish(velodyne_packets);
}

void imu_Callback(const sensor_msgs::Imu::ConstPtr& msg) {
  sensor_msgs::Imu imu;

  imu.header = msg->header;
  imu.orientation = msg->orientation;
  imu.orientation_covariance = msg->orientation_covariance;
  imu.angular_velocity = msg->angular_velocity;
  imu.angular_velocity_covariance = msg->angular_velocity_covariance;
  imu.linear_acceleration = msg->linear_acceleration;
  imu.linear_acceleration_covariance = msg->linear_acceleration_covariance;

  imu_pub.publish(imu);
}

void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  geometry_msgs::TwistStamped twist;

  twist.header = msg->header;
  twist.twist = msg->twist;

  
  twist.header.frame_id = "base_link";
  
  std::cout<< "pub "<<std::endl;
  twist_pub.publish(twist);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "type_changer_real");
  ros::NodeHandle n;

  n.getParam("sub_lidar_topic_name",sub_lidar_topic_name);
  n.getParam("sub_twist_topic_name",sub_twist_topic_name);
  n.getParam("sub_imu_topic_name",sub_imu_topic_name);
  n.getParam("pub_lidar_topic_name",pub_lidar_topic_name);
  n.getParam("pub_twist_topic_name",pub_twist_topic_name);
  n.getParam("pub_imu_topic_name",pub_imu_topic_name);

  std::cout<< "sub_lidar_topic_name "<<sub_lidar_topic_name<<std::endl;
  std::cout<< "sub_twist_topic_name "<<sub_twist_topic_name<<std::endl;
  std::cout<< "sub_imu_topic_name "<<sub_imu_topic_name<<std::endl;
  std::cout<< "pub_lidar_topic_name "<<pub_lidar_topic_name<<std::endl;
  std::cout<< "pub_twist_topic_name "<<pub_twist_topic_name<<std::endl;
  std::cout<< "pub_imu_topic_name "<<pub_imu_topic_name<<std::endl;

  ros::Subscriber packets_sub = n.subscribe(sub_lidar_topic_name, 10, packetsCallback);
  ros::Subscriber twist_sub = n.subscribe(sub_twist_topic_name, 10, twistCallback);
  ros::Subscriber imu_sub = n.subscribe(sub_imu_topic_name, 10, imu_Callback);

  packets_pub = n.advertise<velodyne_msgs::VelodyneScan>(pub_lidar_topic_name, 10);
  twist_pub = n.advertise<geometry_msgs::TwistStamped>(pub_twist_topic_name, 10);
  imu_pub = n.advertise<sensor_msgs::Imu>(pub_imu_topic_name, 10);

  ros::spin();
  return 0;
}
