#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

double yaw_ini = 0;
bool init = true;

ros::Publisher pub;
ros::Publisher pub2;

std::string sub_name = "/mavros/imu/data";
std::string pub_name = "/imu/yaw";

void IMU_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  double roll_imu, pitch_imu_, yaw_imu_;
  geometry_msgs::Quaternion _q_imu = msg->orientation;
  tf::Quaternion _q_homog(_q_imu.x, _q_imu.y, _q_imu.z, _q_imu.w);                                                     
  tf::Matrix3x3(_q_homog).getRPY(roll_imu, pitch_imu_, yaw_imu_);

  if (init)
  {
    //std::cout << "YAW INICIAL: " << yaw_imu_*180/3.1416 << std::endl;
    yaw_ini = yaw_imu_;
    init = false;
  }
  else
  {
    //std::cout << "YAW: " << (yaw_imu_ - yaw_ini)*180/3.1416 << std::endl;

    tf::Quaternion q_t = tf::createQuaternionFromRPY(0, 0, (yaw_imu_ - yaw_ini)); 
    geometry_msgs::Quaternion _q_pose;
    _q_pose.x = q_t.x();
    _q_pose.y = q_t.y();
    _q_pose.z = q_t.z();
    _q_pose.w = q_t.w();

    geometry_msgs::QuaternionStamped m;
    nav_msgs::Odometry odom;
    m.header = msg->header;
    m.quaternion = _q_pose;
    odom.pose.pose.orientation = _q_pose;
    odom.header.frame_id = "laser"; 

    pub.publish(m);
    pub2.publish(odom);
  }
}

int main (int argc, char **argv)
{
  std::string _node_name = "imu_orientation";
  ros::init(argc, argv, _node_name);
  ros::NodeHandle n;

  n.getParam(_node_name + "/TOPIC_SUBSCRIBER",sub_name);
  n.getParam(_node_name + "/TOPIC_PUBLISHER",pub_name);

  //Publishers
  pub = n.advertise<geometry_msgs::QuaternionStamped>(pub_name, 1);
  pub2 = n.advertise<nav_msgs::Odometry>("/fake_odometry", 1);

  //Subscribers
  ros::Subscriber sub = n.subscribe(sub_name, 100, IMU_callback);

  //waiting messages
  ros::spin();

  return 0;
}
