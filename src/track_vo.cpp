/**
*   @file       homo_piloting.cpp
*
*   @author     Juan Antonio Sanchez Diaz <juanantoniodiaz99@gmail.com>
*   @memberof   GRVC
*
*   @date       13/06/2022
*   @brief      Main node of camera odometry in the viaduct scenarios using 
*/

#include "track_vo.h"

int main(int argc, char **argv)
{
    ROS_INFO("Starting VO node with camera info...");
    string _node_name = "track_piloting" ; 
    ros::init(argc, argv, _node_name);
    ros::NodeHandle n;

    // Check is roscore is running
    if(!ros::master::check()){
        ROS_ERROR("[%s]: roscore is not running.", _node_name.c_str());
        return 0;
    }

    TrackVO VO(n, _node_name);
    // VO.spin();

    while(true){   
        ros::Duration(0.03).sleep();
        VO.spinOnce();
    }

    // Report that the node has been finished
    ROS_INFO("[%s]: node finished", _node_name.c_str());

    return 0;
}
