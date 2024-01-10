/**
*   @file       homo_piloting.cpp
*
*   @author     Juan Antonio Sanchez Diaz <juanantoniodiaz99@gmail.com>
*   @memberof   GRVC
*
*   @date       13/06/2022
*   @brief      Main node of camera odometry in the viaduct scenarios using 
*/

#include "homo_piloting.h"

// ORB PARAMETERS
float NUMBER_FEATURES = 200;
float SCALE_FACTOR = 1.2;
float N_LEVELS = 8;
float EDGE_THRESHOLD = 20;
float FIRST_LEVEL = 0;
float WTA_K = 2;
float SCORE_TYPE = 1;
float PATCH_SIZE = EDGE_THRESHOLD;
float FAST_THRESHOLD = 20; 
float escala = 10; //0.68;

// TOPIC'S NAME
std::string image_topic = "/usb_cam/image_raw";
std::string imageORB_topic = "/camera/ORB_image";
std::string altimeter_topic = "/sf11";
std::string path_topic = "/homo/path";
std::string pose_topic = "/homo/pose";
std::string odom_topic = "/homo/odom";
std::string mission_manager_service = "mission_manager_homo";

std::string odom_frame_id = "world";

bool debug = true;

const double nn_match_ratio = 0.8f;

float XG = 0;
float YG = 0;
float YAWG = 0;
bool takeoff = false;
Eigen::MatrixXf ki(3,3);

// SUSCRIBERS
ros::Subscriber image;
ros::Subscriber pose;

// PUBLISHERS
ros::Publisher ORBi;
ros::Publisher pose_estimation;
ros::Publisher path;
ros::Publisher odom;
ros::Publisher homop;

std::vector<cv::KeyPoint> kp0;
cv::Mat d0;

bool init= false;
nav_msgs::Path msg_path;
bool initz = false;
float z = 0;
float xa = 2, ya = 2, za = 0;
float min_high = 0;
float width = 0, height = 0;
float XY_homo = 0, XZ_homo = 0; 

std::vector<float> int_matrix(9);

void poseCallback(const sensor_msgs::Range::ConstPtr& msg)
{
    if(!takeoff)
    {
        geometry_msgs::PoseStamped m;

        z = msg->range; //msg->range

        m.pose.position.x = (XG + xa) / 2 * escala;
        m.pose.position.y = (YG + ya) / 2 * escala;
        m.pose.position.z = z;

        std::cout << m.pose.position.x << " , " << m.pose.position.y << " , " << m.pose.position.z << std::endl;

        // Obtain a quaternion from the estimated yaw
        tf::Quaternion q_t = tf::createQuaternionFromRPY(0, 0, YAWG); //- XY_homo*3.1416/180
        geometry_msgs::Quaternion _q_pose;
        _q_pose.x = q_t.x();
        _q_pose.y = q_t.y();
        _q_pose.z = q_t.z();
        _q_pose.w = q_t.w();
        m.pose.orientation = _q_pose;

        if(msg->range == 0.0) //FALLO EN LA MEDIDA
        {
            std::cout << "ERROR EN LA LECTURA DE ALTÍMETRO" << std::endl;
            m.pose.position.x = xa;
            m.pose.position.y = ya;
            m.pose.position.z = za;
        }

        xa = m.pose.position.x;
        ya = m.pose.position.y;
        za = m.pose.position.z;

        float xrot = 0;
        float yrot = 0;
        float zrot = 0; 

        xrot = m.pose.position.x;
        yrot = m.pose.position.y;
        zrot = m.pose.position.z;

        // XY Rotation
        float xtrans = xrot;
        float ytrans = yrot;
        float theta = XY_homo * 3.1416/180; 
        xrot = xtrans*cos(theta) + ytrans*sin(theta);
        yrot = -xtrans*sin(theta) + ytrans*cos(theta); 

        // XZ Rotation
        xtrans = xrot;
        float ztrans = zrot;
        theta = XZ_homo * 3.1416/180;
        xrot = xtrans*cos(theta) + ztrans*sin(theta);
        zrot = -xtrans*sin(theta) + ztrans*cos(theta);

        m.pose.position.x = xrot;
        m.pose.position.y = yrot;
        m.pose.position.z = zrot;

        msg_path.poses.push_back(m);
        path.publish(msg_path);
        msg_path.header.seq++;

        nav_msgs::Odometry odom;
        odom.pose.pose = m.pose;
        odom.header.frame_id = odom_frame_id;
        homop.publish(odom);
    }
}

void equalize_ycrcb(cv::Mat img)
{
    if(img.channels() >= 3)
    {
        cv::Mat ycrcb;

        cv::cvtColor(img,ycrcb,cv::COLOR_BGR2YCrCb);

        std::vector<cv::Mat> channels;
        cv::split(ycrcb,channels);

        cv::equalizeHist(channels[0], channels[0]);

        cv::Mat result;
        cv::merge(channels,ycrcb);

        cv::cvtColor(ycrcb,img,cv::COLOR_YCrCb2BGR);
    }
}

void imageCallback(const sensor_msgs::Image::ConstPtr& image)
{
    if(!takeoff)
    {
        ros::WallTime start_, end_;
        start_ = ros::WallTime::now();
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);  
        // Realizamos la extracción de features
        cv::Ptr<cv::ORB> orb = cv::ORB::create( NUMBER_FEATURES,
                                                SCALE_FACTOR,
                                                N_LEVELS,
                                                EDGE_THRESHOLD,
                                                FIRST_LEVEL,
                                                WTA_K,
                                                (cv::ORB::ScoreType)SCORE_TYPE,
                                                PATCH_SIZE,
                                                FAST_THRESHOLD );
        if (!init)
        {
            width = cv_ptr->image.cols;
            height = cv_ptr->image.rows;
            // std::cout << "Image size -> width:"<< width << " heigth:" << height << std::endl;

            equalize_ycrcb(cv_ptr->image);

            orb->detectAndCompute(cv_ptr->image,cv::noArray(),kp0,d0);
            
            init = true;
        }
        else
        {
            equalize_ycrcb(cv_ptr->image);

            cv::Mat d1;
            std::vector<cv::KeyPoint> kp1;
            orb->detectAndCompute(cv_ptr->image,cv::noArray(),kp1,d1);

            // Realizamos el matching entre los puntos
            cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");   
            std::vector<std::vector<cv::DMatch>> matches;
            std::vector<cv::KeyPoint> matched1, matched2;
            std::vector<cv::Point2f> p0,p1;
            
            // Realizar comprobacion 
            matcher->knnMatch(d0, d1, matches, 2);
            // std::cout << matches.size() << std::endl;
            for(unsigned i = 0; i < matches.size(); i++) 
            {
                if(matches[i][0].distance < nn_match_ratio * matches[i][1].distance) 
                {
                    matched1.push_back(kp0[matches[i][0].queryIdx]);
                    matched2.push_back(kp1[matches[i][0].trainIdx]);
                    p0.push_back(kp0[matches[i][0].queryIdx].pt);
                    p1.push_back(kp1[matches[i][0].trainIdx].pt);
                }
            }
            
            cv::Mat inlier_mask, homography;
            std::vector<cv::KeyPoint> inliers1, inliers2;
            std::vector<cv::DMatch> inlier_matches;
            if(matched1.size() >= 4) {
                homography = cv::findHomography(p0, p1,
                                            cv::RANSAC, 1, inlier_mask);
                // std::cout << "Homografia: " << std::endl << homography << std::endl;   
            }

            for(unsigned i = 0; i < matched1.size(); i++) {
                if(inlier_mask.at<uchar>(i)) {
                    int new_i = static_cast<int>(inliers1.size());
                    inliers1.push_back(matched1[i]);
                    inliers2.push_back(matched2[i]);
                    inlier_matches.push_back(cv::DMatch(new_i, new_i, 0));
                }
            }

            cv::Mat A(2*inliers1.size(), 4, CV_64FC1);
            cv::Mat b(2*inliers1.size(), 1, CV_64FC1);
            for(int i = 0; i < inliers1.size(); i++)
            {
                A.at<double>(i*2,0) = inliers1[i].pt.x - width / 2; A.at<double>(i*2,1) = - (inliers1[i].pt.y - height / 2); A.at<double>(i*2,2) = 1; A.at<double>(i*2,3) = 0; 
                A.at<double>(i*2 + 1,0) = inliers1[i].pt.y - height / 2; A.at<double>(i*2 + 1,1) = inliers1[i].pt.x - width / 2; A.at<double>(i*2 + 1,2) = 0; A.at<double>(i*2 + 1,3) = 1; 
                b.at<double>(i*2,0) = inliers2[i].pt.x - width / 2;
                b.at<double>(i*2 + 1,0) = inliers2[i].pt.y - height / 2;
            }
            
            Eigen::MatrixXd Ae;
            Eigen::MatrixXd be;
            cv::cv2eigen(A,Ae);
            cv::cv2eigen(b,be);
            Eigen::VectorXd xopt = Ae.colPivHouseholderQr().solve(be);
            float u = xopt(2);
            float v = xopt(3);
            
            // std::cout << "u:" << u << " v:" << v << " yaw:" << atan(xopt(1)/xopt(0)) << std::endl;

            float xi = (ki(0,0)*u + ki(0,1)*v); //+ ki(0,2)
            float yi = (ki(1,0)*u + ki(1,1)*v); //+ ki(1,2)

            if (z > min_high)
            {
                geometry_msgs::PoseStamped m_odom;
                tf::Quaternion q_t = tf::createQuaternionFromRPY(0, 0, atan(xopt(1)/xopt(0))); //- XY_homo*3.1416/180
                geometry_msgs::Quaternion _q_pose;
                _q_pose.x = q_t.x();
                _q_pose.y = q_t.y();
                _q_pose.z = q_t.z();
                _q_pose.w = q_t.w();
                m_odom.pose.orientation = _q_pose;
                m_odom.pose.position.x = - (xi * cos(YAWG) + yi * sin(YAWG)) * z;
                m_odom.pose.position.y = (-xi * sin(YAWG) + yi * cos(YAWG)) * z;
                m_odom.pose.position.z = 0;
                odom.publish(m_odom);

                //if (atan(xopt(1)/xopt(0)) < 0.1) YAWG += atan(xopt(1)/xopt(0));
                YAWG += atan(xopt(1)/xopt(0));
                XG = XG - (xi * cos(YAWG) + yi * sin(YAWG)) * z;
                YG = YG + (-xi * sin(YAWG) + yi * cos(YAWG)) * z;
            }

            end_ = ros::WallTime::now();
            double execution_time = (end_ - start_).toNSec() * 1e-6;

            if (debug)
            {
                std::cout << "Number of inliers: " << inliers1.size() << std::endl;
                std::cout << "Position: " << std::endl;
                std::cout << "X: " << XG << std::endl;
                std::cout << "Y: " << YG << std::endl;
                std::cout << "Z: " << z << std::endl;
                std::cout << "YAWG: " << YAWG*180/3.1416 << std::endl;
                std::cout << "Execution time: " << execution_time << std::endl;
                std::cout << "-----------------------------------------------------------" << std::endl;
            }

            // Reiniciamos
            kp0 = kp1;
            d0 = d1;

            drawKeypoints(cv_ptr->image, kp1, cv_ptr->image, cv::Scalar(0, 0, 255));
            ORBi.publish(cv_ptr->toImageMsg());

            // Posicion sin datos de altimetro
            /*z = 2;
            geometry_msgs::PoseStamped m;

            //m.pose.position.x = (- x * pow(msg->pose.position.z,2) * escala + xa) / 2; //* msg->pose.position.z
            //m.pose.position.y = (y * pow(msg->pose.position.z,2) * escala + ya) / 2; 
            m.pose.position.x = -x * z;
            m.pose.position.y = y * z;
            m.pose.position.z = z;

            msg_path.poses.push_back(m);
            path2.publish(msg_path);
            msg_path.header.seq++;*/
        }
    }
}

bool node_init(mission_manager::mission_manager::Request  &req, mission_manager::mission_manager::Response &res)
{
    std::cout << "[CAMERA]: service received" << std::endl;
    XG = req.x_pos;
    YG = req.y_pos;
    //YAWG = 0.0;
    YAWG = req.yaw_rot;
    //YAWG = -XY_homo*3.1416/180;
    xa = XG;
    ya = YG;

    takeoff = req.takeoff;
    return true;
}

int main(int argc, char **argv)
{
    ROS_INFO("Starting Homography node with camera info...");
    std::string _node_name = "visual_odometry" ; //homography
    ros::init(argc, argv, _node_name);
    ros::NodeHandle n;
    msg_path.header.frame_id = "/base_link_init";

    //Filter parameters
    n.getParam(_node_name + "/NUMBER_FEATURES",NUMBER_FEATURES);    // Maximum angle
    n.getParam(_node_name + "/SCALE_FACTOR",SCALE_FACTOR);	// Minimum angle
    n.getParam(_node_name + "/N_LEVELS",N_LEVELS);  // Minimum laser intensity 
    n.getParam(_node_name + "/EDGE_THRESHOLD",EDGE_THRESHOLD);  // Maximum laser distance
    n.getParam(_node_name + "/FIRST_LEVEL",FIRST_LEVEL);  // Maximum distance for belonging to a line
    n.getParam(_node_name + "/WTA_K",WTA_K); // Laser topic
    n.getParam(_node_name + "/SCORE_TYPE",SCORE_TYPE);  // Scale for image transformation
    n.getParam(_node_name + "/PATCH_SIZE",PATCH_SIZE); // Translation for image transformation
    n.getParam(_node_name + "/FAST_THRESHOLD",FAST_THRESHOLD); // Translation for image transformation
    n.getParam(_node_name + "/SCALE",escala); // Translation for image transformation
    n.getParam(_node_name + "/HIGH",min_high); // Mininum altitud for compute
    n.getParam(_node_name + "/ODOM_FRAME_ID",odom_frame_id); // 
    n.getParam(_node_name + "/ROT_XY",XY_homo); // 
    n.getParam(_node_name + "/ROT_XZ",XZ_homo); // 
    n.getParam(_node_name + "/INTRINSIC_PARAMETERS",int_matrix); // 
    n.getParam(_node_name + "/DEBUG",debug); // Debug mode
    n.getParam(_node_name + "/IMAGE_TOPIC",image_topic); 
    n.getParam(_node_name + "/ALTIMETER_TOPIC",altimeter_topic); 
    n.getParam(_node_name + "/PATCH_TOPIC",path_topic); 
    n.getParam(_node_name + "/POSE_TOPIC",pose_topic); 
    n.getParam(_node_name + "/ODOM_TOPIC",odom_topic); 
    n.getParam(_node_name + "/ORB_IMAGE_TOPIC",imageORB_topic); 
    n.getParam(_node_name + "/MISSION_MANAGER_SERVICE",mission_manager_service); 
    
    // Matriz de parametros intrínsecos invertida
    ki << int_matrix[0], int_matrix[1], int_matrix[2],
          int_matrix[3], int_matrix[4], int_matrix[5],
          int_matrix[6], int_matrix[7], int_matrix[8];

    image = n.subscribe(image_topic, 100, imageCallback);
    pose = n.subscribe(altimeter_topic, 100, poseCallback); // /totalstation/pose /sf11
    ORBi = n.advertise<sensor_msgs::Image>(imageORB_topic, 100); 
    //pose_estimation = n.advertise<geometry_msgs::PoseStamped>("/homo/pose", 100); 
    path = n.advertise<nav_msgs::Path>(path_topic, 100); 
    odom = n.advertise<geometry_msgs::PoseStamped>(odom_topic, 100); 
    homop = n.advertise<nav_msgs::Odometry>(pose_topic, 100); 

    ros::ServiceServer service = n.advertiseService(mission_manager_service, node_init);

    ROS_INFO("Waiting data...");
    std::cout << "odomORB = [" << std::endl;
    ros::spin();
}
