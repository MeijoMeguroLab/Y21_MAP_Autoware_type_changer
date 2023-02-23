#include <ros/ros.h>
#include <std_msgs/String.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <math.h>
#include <stdlib.h>

static std::string sub_lidar_topic_name;
static std::string pub_lidar_topic_name;
ros::Publisher packets_pub;

static std::string sub_twist_topic_name;
static std::string pub_twist_topic_name;
ros::Publisher twist_pub;

static std::string sub_imu_topic_name;
static std::string pub_imu_topic_name;
ros::Publisher imu_pub;
bool reverse_imu = false;
double velocity_scale_factor = 1.0;
int randam_num = 0;
int slip_cnt = 0;
int slip_cnt_tmp = 0;
double slip_sf = 1.0;
// int slip_time = 80/2 ; // 80/8 => 50/10 : 10 = 0.1s
int slip_probability = 50; // Probability of slip (default:50 = 100/50 = 2[%]), if the num equal 1,not use this.
int missing_can_probability = 10; // Probability of missing can data(default:10 = 100/10 = 10[%]), if the num equal 1,not use this.
int slip_time = 50;

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

  // velocity error due to scale factor
  twist.twist.linear.x = twist.twist.linear.x * velocity_scale_factor;

  // create randam num
  randam_num = rand();

  // velocity error due to slip
  if (slip_probability == 1){
    // don't use slip velocity
  }else if(randam_num % slip_probability == 0 || slip_cnt !=0 ){
    twist.twist.linear.x = twist.twist.linear.x + (twist.twist.linear.x * slip_sf * slip_cnt);
    double vel_tmp = twist.twist.linear.x * slip_sf * slip_cnt;
    double test =  double(slip_cnt / slip_time);
    std::cout << "slip: " << slip_cnt_tmp << " , " <<  slip_cnt << " , " << vel_tmp << " , " << test << std::endl;
    if (slip_cnt_tmp < slip_time){
      slip_cnt++;
    }else if (slip_cnt_tmp >= 2 * slip_time){
      slip_cnt_tmp = 0;
      slip_cnt = 0;
    }else{
      slip_cnt--;
    }
    slip_cnt_tmp++;
  }else if (slip_cnt == 0){
    slip_cnt_tmp = 0;
  }

  // can velocity drop
  if (missing_can_probability == 1 || randam_num % missing_can_probability != 1){
    twist_pub.publish(twist);
  }else{
    std::cout << "twist not pub" << std::endl;
  }
  // randam_num ++;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  sensor_msgs::Imu imu;
  imu = *msg;
  if(!reverse_imu){
    imu_pub.publish(imu);
  }else{
    imu.angular_velocity.z =  -1 * imu.angular_velocity.z;
    imu_pub.publish(imu);
  }
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
  n.getParam("reverse_imu",reverse_imu);
  n.getParam("velocity_scale_factor",velocity_scale_factor);
  n.getParam("slip_sf",slip_sf);
  n.getParam("slip_probability",slip_probability);
  n.getParam("missing_can_probability",missing_can_probability);

  std::cout<< "sub_lidar_topic_name "<<sub_lidar_topic_name<<std::endl;
  std::cout<< "sub_twist_topic_name "<<sub_twist_topic_name<<std::endl;
  std::cout<< "sub_imu_topic_name "<<sub_imu_topic_name<<std::endl;
  std::cout<< "pub_lidar_topic_name "<<pub_lidar_topic_name<<std::endl;
  std::cout<< "pub_twist_topic_name "<<pub_twist_topic_name<<std::endl;
  std::cout<< "pub_imu_topic_name "<<pub_imu_topic_name<<std::endl;
  std::cout<< "velocity_scale_factor "<<velocity_scale_factor<<std::endl;
  std::cout<< "slip_sf "<<slip_sf<<std::endl;
  std::cout<< "slip_probability "<<slip_probability<<std::endl;
  std::cout<< "missing_can_probability "<<missing_can_probability<<std::endl;

  ros::Subscriber packets_sub = n.subscribe(sub_lidar_topic_name, 10, packetsCallback);
  ros::Subscriber twist_sub = n.subscribe(sub_twist_topic_name, 10, twistCallback);
  ros::Subscriber imu_sub = n.subscribe(sub_imu_topic_name, 10, imuCallback);

  packets_pub = n.advertise<velodyne_msgs::VelodyneScan>(pub_lidar_topic_name, 10);
  twist_pub = n.advertise<geometry_msgs::TwistStamped>(pub_twist_topic_name, 10);
  imu_pub = n.advertise<sensor_msgs::Imu>(pub_imu_topic_name, 10);

  ros::spin();
  return 0;
}
