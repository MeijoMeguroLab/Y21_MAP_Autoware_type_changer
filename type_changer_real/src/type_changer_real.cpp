#include <ros/ros.h>
#include <std_msgs/String.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TwistStamped.h>
#include <math.h>

static std::string sub_lidar_topic_name;
static std::string pub_lidar_topic_name;
ros::Publisher packets_pub;

static std::string sub_twist_topic_name;
static std::string pub_twist_topic_name;
ros::Publisher twist_pub;

void packetsCallback(const velodyne_msgs::VelodyneScan::ConstPtr& msg) {
  velodyne_msgs::VelodyneScan velodyne_packets;
  velodyne_packets = *msg;
  velodyne_packets.header.frame_id = "velodyne_top";
  packets_pub.publish(velodyne_packets);
}

void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  geometry_msgs::TwistStamped twist;
  twist = *msg;
  twist.header.frame_id = "base_link";
  twist_pub.publish(twist);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "type_changer_real");
  ros::NodeHandle n;

  n.getParam("sub_lidar_topic_name",sub_lidar_topic_name);
  n.getParam("sub_twist_topic_name",sub_twist_topic_name);
  n.getParam("pub_lidar_topic_name",pub_lidar_topic_name);
  n.getParam("pub_twist_topic_name",pub_twist_topic_name);

  std::cout<< "sub_lidar_topic_name "<<sub_lidar_topic_name<<std::endl;
  std::cout<< "sub_twist_topic_name "<<sub_twist_topic_name<<std::endl;
  std::cout<< "pub_lidar_topic_name "<<pub_lidar_topic_name<<std::endl;
  std::cout<< "pub_twist_topic_name "<<pub_twist_topic_name<<std::endl;

  ros::Subscriber packets_sub = n.subscribe(sub_lidar_topic_name, 10, packetsCallback);
  ros::Subscriber twist_sub = n.subscribe(sub_twist_topic_name, 10, twistCallback);

  packets_pub = n.advertise<velodyne_msgs::VelodyneScan>(pub_lidar_topic_name, 10);
  twist_pub = n.advertise<geometry_msgs::TwistStamped>(pub_twist_topic_name, 10);

  ros::spin();
  return 0;
}
