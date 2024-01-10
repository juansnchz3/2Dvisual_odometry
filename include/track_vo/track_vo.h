/**
*   @file       track_vo.h
*
*   @author     Juan Antonio Sanchez Diaz <jsanchez22@us.com>
*   @memberof   GRVC
*
*   @date       13/06/2022
*   @brief      Header file of TrackVO class
**/

#include <ros/ros.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>  
#include <vector>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <typeinfo>
#include <signal.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/foreach.hpp>
#include <tf/transform_broadcaster.h>
#include <mission_manager/mission_manager.h>
#include <mission_manager/Alarms.h>

#define foreach BOOST_FOREACH

using namespace cv;
using namespace std;

class TrackVO
{
private:
    // Declaration of ROS Node Handle 
    ros::NodeHandle node_; // ROS node handle
    std::string name;
    
    // TRACK PARAMETERS
    int min_points = 20;
    int new_points = 15;
    int init_points = 100;
    int lost_points = 5;
    int advise_threshold = 7;
    float edge_threshold = 0.2;
    int nrows = 3, ncols = 4;
    float minDistance = 30;
    int blockSize = 7;
    float responseK = 0.04;
    float escala = 1.15; //0.68;

    // TOPIC'S NAME
    string image_topic = "/camera/color/image_raw";
    string imageORB_topic = "/camera/ORB_image";
    string altimeter_topic = "/sf11";
    string path_topic = "/homo/path";
    string pose_topic = "/homo/pose";
    string odom_topic = "/homo/odom";
    string mission_manager_service = "mission_manager_homo";
    string alarm_service = "alarm";

    string odom_frame_id = "world";
    string path_frame_id = "world";

    bool debug = true;

    float XG = 0, YG = 0, YAWG = 0;
    bool takeoff = false;
    Eigen::MatrixXf ki = Eigen::MatrixXf(3,3);

    // SUSCRIBERS
    ros::Subscriber image;
    ros::Subscriber pose;

    // PUBLISHERS
    ros::Publisher ORBi;
    ros::Publisher pose_estimation;
    ros::Publisher path;
    ros::Publisher odom;
    ros::Publisher homop;

    // SERVICES
    ros::ServiceServer srv_manager;
    ros::ServiceClient srv_alarm;

    vector<Point2f> points1, points2; 
    vector<uchar> status;

    bool init = true;
    nav_msgs::Path msg_path;
    bool initz = false;
    float z = 1;
    float iz = 0;
    float xa = 0, ya = 0, za = 0;
    float min_high = 0;
    int dif = 0;
    float width = 0, height = 0;
    float XY_homo = 0, XZ_homo = 0; 

    Mat old_img, old_gray, frame_img, frame_gray;

    vector<float> int_matrix = vector<float>(9);
    
    // Subscriber callbacks 
    // \cb camera_callback : images callback
    void imageCallback(const sensor_msgs::Image::ConstPtr& image);
    void poseCallback(const sensor_msgs::Range::ConstPtr& msg);

    void equalize_ycrcb(Mat img);
    void drawFeatures(Mat frame, vector<Point2f>& points);
    void featureDetection(Mat img_1, vector<Point2f>& points, int maxCorners);
    tuple<int, int> minFeatureBox(Mat img, vector<Point2f>& points, int ncols, int nrows, int threshold);
    void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status);

    void getParameters();

    /* Service callbacks */
    // \cb node_init : service callback
    bool node_init(mission_manager::mission_manager::Request  &req, 
                   mission_manager::mission_manager::Response &res);
public:
    // Declaration of public variables 
    int HealthCount = 0;
    int Alarm_FLAG = false;
    int Alarm_TYPE = 0;

    // Headers of Public Functions 
    // \fnc LaserGeneric : Constructor
    TrackVO(ros::NodeHandle &_node, string node_name);

    // \fnc ~LaserGeneric : Destructor
    ~TrackVO();

    //\fnc spin : ROS spin loop
    void spin();
    void spinOnce();
};
