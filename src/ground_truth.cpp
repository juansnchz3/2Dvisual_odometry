#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

//DEFINE
#define FREQ 100

float altura = 0;

//PUBLISHER
ros::Publisher pub;
ros::Publisher path;
ros::Publisher err;
ros::Publisher est;

//GLOBALS MSGS
sensor_msgs::PointCloud data;
bool publicar = false;
uint id;

bool ini = true;
float xi = 0;
float yi = 0;
float zi = 0;
float xa = 0;
float ya = 0;
float za = 0;
float error = 10;
double begin;
nav_msgs::Path msg_path;

float XY_angle = 0, XZ_angle = 0;

int ti = -1;

void transformation(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if (ros::Time::now().toSec() - begin > ti)
  {
    if (ini)
    {
      xi = msg->pose.position.x;
      yi = msg->pose.position.y;
      zi = msg->pose.position.z; //altura
      ini = false;
    }
    else
    {
      geometry_msgs::PoseStamped m;
      m.pose.position.y = yi - msg->pose.position.y;
      m.pose.position.x = xi - msg->pose.position.x;
      m.pose.position.z = msg->pose.position.z - zi;
      m.header = msg->header;

      // XY Rotation
      float ytrans = m.pose.position.y;
      float xtrans = m.pose.position.x;
      float theta = XY_angle * 3.1416/180 ; //camera: -35  25
      m.pose.position.x = xtrans*cos(theta) + ytrans*sin(theta);
      m.pose.position.y = -xtrans*sin(theta) + ytrans*cos(theta); 

      // XZ Rotation
      xtrans = m.pose.position.x;
      float ztrans = m.pose.position.z ;
      theta = XZ_angle * 3.1416/180 ; //0
      m.pose.position.x = xtrans*cos(theta) + ztrans*sin(theta);
      m.pose.position.z = -xtrans*sin(theta) + ztrans*cos(theta);

      // Y Traslation
      m.pose.position.y = m.pose.position.y ; //- 0.15

      if(abs(m.pose.position.x - xa) < error && abs(m.pose.position.y - ya) < error && abs(m.pose.position.z - za) < error)
      {
        msg_path.poses.push_back(m);
        path.publish(msg_path);
        msg_path.header.seq++;
        pub.publish(m);

        xa = m.pose.position.x;
        ya = m.pose.position.y;
        za = m.pose.position.z;
      }
    }
  }
}

void f_error(const nav_msgs::Path::ConstPtr& msg)
{
  if (!msg_path.poses.empty())
  {
    geometry_msgs::PoseStamped dato1 = msg->poses.back();
    geometry_msgs::PoseStamped dato2 = msg_path.poses.back();
    geometry_msgs::PoseStamped m;

    m.pose.position.x = abs(dato1.pose.position.x - dato2.pose.position.x);
    m.pose.position.y = abs(dato1.pose.position.y - dato2.pose.position.y);
    m.pose.position.z = abs(dato1.pose.position.z - dato2.pose.position.z);

    dato1.header.stamp = ros::Time::now();
    est.publish(dato1);
    m.header.frame_id = "error";
    m.header.stamp = ros::Time::now();
    err.publish(m);
  }
}

int main (int argc, char **argv)
{
  ros::init (argc, argv, "Read_Laser");
  ros::NodeHandle n("~");
  msg_path.header.frame_id = "base_link_init";
  begin = ros::Time::now().toSec();

  //Parameters
	n.param<float>("XY",XY_angle,0);	// Angulo maximo para filtrado
	n.param<float>("XZ",XZ_angle,0);	// Angulo minimo para filtrado
  //Data topic
  std::string read_topic;
	n.param<std::string>("path_topic",read_topic,"/scan");

  //Publishers
  path = n.advertise<nav_msgs::Path>("/benchmark_publisher/path", 1); 
  pub = n.advertise<geometry_msgs::PoseStamped>("/total", 1);
  est = n.advertise<geometry_msgs::PoseStamped>("/estimation", 1); 
  err = n.advertise<geometry_msgs::PoseStamped>("/error", 1); 

  //Subscribers
  ros::Subscriber total = n.subscribe("/totalstation/pose", 1, transformation);
  ros::Subscriber trajec = n.subscribe(read_topic, 1, f_error); 

  //waiting messages
  ros::spin();

  return 0;
}
