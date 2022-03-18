#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

static std::string sub_lidar_topic_name;
static std::string pub_lidar_topic_name;
ros::Publisher points_raw_pub;

void pointCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  sensor_msgs::PointCloud2 points_raw;
  points_raw = *msg;
  points_raw.header.frame_id = "velodyne_top";
  points_raw_pub.publish(points_raw);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "type_changer_sim");
  ros::NodeHandle n;

  n.getParam("sub_lidar_topic_name",sub_lidar_topic_name);
  n.getParam("pub_lidar_topic_name",pub_lidar_topic_name);
  std::cout<< "sub_lidar_topic_name "<<sub_lidar_topic_name<<std::endl;
  std::cout<< "pub_lidar_topic_name "<<pub_lidar_topic_name<<std::endl;

  points_raw_pub = n.advertise<sensor_msgs::PointCloud2>(pub_lidar_topic_name, 10);
  ros::Subscriber point_sub = n.subscribe(sub_lidar_topic_name, 10, pointCallback);

  ros::spin();
  return 0;
}
