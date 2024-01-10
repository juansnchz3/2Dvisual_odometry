#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>


//PUBLISHER
ros::Publisher nav;
ros::Publisher nav_path;

nav_msgs::Path msg_path;
geometry_msgs::PoseStamped m; 

float x = 0;
float y = 0;
float z = 0; 

float XY_homo = 0, XZ_homo = 0, XY_hough = 0, XZ_hough = 0, XY_gps = 0, XZ_gps = 0;

void gps_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  float xaux = 0;
  float yaux = 0;
  float zaux = 0; 

  geometry_msgs::PoseStamped dato; 
  dato.pose = msg->pose.pose;
  dato.header = msg->header;

  x = dato.pose.position.x;
  y = dato.pose.position.y;
  xaux = x;
  yaux = y;
  zaux = z;
  m.header = msg->header;

  // XY Rotation
  float xtrans = xaux;
  float ytrans = yaux;
  float theta = XY_gps * 3.1416/180; 
  xaux = xtrans*cos(theta) + ytrans*sin(theta);
  yaux = -xtrans*sin(theta) + ytrans*cos(theta); 

  // XZ Rotation
  xtrans = xaux;
  float ztrans = zaux;
  theta = XZ_gps * 3.1416/180; 
  xaux = xtrans*cos(theta) + ztrans*sin(theta);
  zaux = -xtrans*sin(theta) + ztrans*cos(theta);

  m.pose.position.x = xaux;
  m.pose.position.y = yaux;
  m.pose.position.z = zaux;

  msg_path.poses.push_back(m);
  nav_path.publish(msg_path);
  msg_path.header.seq++;
  nav.publish(m);
}

void hough_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  float xaux = 0;
  float yaux = 0;
  float zaux = 0; 

  geometry_msgs::PoseStamped dato; 
  dato.pose = msg->pose.pose;
  dato.header = msg->header;

  x = dato.pose.position.x;
  z = dato.pose.position.z;
  xaux = x;
  yaux = y;
  zaux = z;
  m.header = msg->header;

  // XY Rotation
  float xtrans = xaux;
  float ytrans = yaux;
  float theta = XY_hough * 3.1416/180; 
  xaux = xtrans*cos(theta) + ytrans*sin(theta);
  yaux = -xtrans*sin(theta) + ytrans*cos(theta); 

  // XZ Rotation
  xtrans = xaux;
  float ztrans = zaux;
  theta = XZ_hough * 3.1416/180;
  xaux = xtrans*cos(theta) + ztrans*sin(theta);
  zaux = -xtrans*sin(theta) + ztrans*cos(theta);

  m.pose.position.x = xaux;
  m.pose.position.y = yaux;
  m.pose.position.z = zaux;

  msg_path.poses.push_back(m);
  nav_path.publish(msg_path);
  msg_path.header.seq++;
  nav.publish(m);
}

void homo_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  float xaux = 0;
  float yaux = 0;
  float zaux = 0; 

  geometry_msgs::PoseStamped dato; 
  dato.pose = msg->pose.pose;
  dato.header = msg->header;

  y = dato.pose.position.y;
  xaux = x;
  yaux = y;
  zaux = z;
  m.header = msg->header;

  // XY Rotation
  float xtrans = xaux;
  float ytrans = yaux;
  float theta = XY_homo * 3.1416/180; 
  xaux = xtrans*cos(theta) + ytrans*sin(theta);
  yaux = -xtrans*sin(theta) + ytrans*cos(theta); 

  // XZ Rotation
  xtrans = xaux;
  float ztrans = zaux;
  theta = XZ_homo * 3.1416/180; 
  xaux = xtrans*cos(theta) + ztrans*sin(theta);
  zaux = -xtrans*sin(theta) + ztrans*cos(theta);

  m.pose.position.x = xaux;
  m.pose.position.y = yaux;
  m.pose.position.z = zaux;

  msg_path.poses.push_back(m);
  nav_path.publish(msg_path);
  msg_path.header.seq++;
  nav.publish(m);
}

int main (int argc, char **argv)
{
  ros::init (argc, argv, "navegation");
  ros::NodeHandle n("~");
  msg_path.header.frame_id = "world";

  //Parameters
	n.param<float>("XY_homografia",XY_homo,0);
	n.param<float>("XZ_homografia",XZ_homo,0);	
  n.param<float>("XY_hough",XY_hough,0);	
	n.param<float>("XZ_hough",XZ_hough,0);	
  n.param<float>("XY_gps",XY_gps,0);	
	n.param<float>("XZ_gps",XZ_gps,0);	

  //Publishers
  nav = n.advertise<geometry_msgs::PoseStamped>("/navegation/pose", 1);
  nav_path = n.advertise<nav_msgs::Path>("/navegation/path", 1);

  //Subscribers
  ros::Subscriber homo_sub = n.subscribe("/homo/odom", 1, homo_callback);
  ros::Subscriber hough_sub = n.subscribe("/hough/odom", 1, hough_callback);
  ros::Subscriber gps_sub = n.subscribe("/gps/odom", 1, gps_callback);

  //waiting messages
  ros::spin();

  return 0;
}
