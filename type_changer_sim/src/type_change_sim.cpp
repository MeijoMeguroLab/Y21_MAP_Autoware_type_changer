#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher points_raw_pub;
ros::Publisher odom_pub;
static double point_time_stamp;
static double point_time_stamp_sec;
static double point_time_stamp_nsec;
static double odom_time_stamp;
static double odom_time_stamp_sec;
static double odom_time_stamp_nsec;
static bool start_flag;
void pointCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  sensor_msgs::PointCloud2 points_raw;

  points_raw.header = msg->header;
  points_raw.height = msg->height;
  points_raw.width = msg->width;
  points_raw.fields = msg->fields;
  points_raw.is_bigendian = msg->is_bigendian;
  points_raw.point_step = msg->point_step;
  points_raw.row_step = msg->row_step;
  points_raw.data = msg->data;
  points_raw.is_dense = msg->is_dense;

  points_raw.header.frame_id = "velodyne_top";

  points_raw_pub.publish(points_raw);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "type_changer_sim");

  ros::NodeHandle n;
  start_flag = true;

  points_raw_pub = n.advertise<sensor_msgs::PointCloud2>("/sensing/lidar/top/pointcloud_raw_ex", 10);
  ros::Subscriber point_sub = n.subscribe("/points_raw", 10, pointCallback);


  ros::spin();
  return 0;
}
