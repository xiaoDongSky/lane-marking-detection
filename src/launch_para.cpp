#include <ros/ros.h>
#include "../include/launch_para.h"
namespace vecan {
namespace perception {

void GetParametersFromLaunch(LaunchPara &paramters){
    ros::NodeHandle private_node_handle("~");//to receive args

    if (private_node_handle.getParam("mode", paramters.mode)){
        ROS_INFO("Node working mode: %s", paramters.mode.c_str());
    }//if
    else{
        ROS_INFO("Node working mode is not defined");
        paramters.mode = "off-line";
    }//else

    if(paramters.mode == "off-line"){
        if (private_node_handle.getParam("pictures_path", paramters.pictures_path)){
            ROS_INFO("pictures_path: %s", paramters.pictures_path.c_str());
        }//if
        else{
            ROS_INFO("pictures_path is not defined");
        }//else
    }//if
    else if(paramters.mode == "on-line"){
        if (private_node_handle.getParam("image_topic_name", paramters.image_topic_name)){
            ROS_INFO("image_topic_name: %s", paramters.image_topic_name.c_str());
        }//if
        else{
            ROS_INFO("image_topic_name is not defined");
        }//else
    }

    if (private_node_handle.getParam("road_topic_name", paramters.road_topic_name)){
        ROS_INFO("road_topic_name: %s", paramters.road_topic_name.c_str());
    }//if
    else{
        ROS_INFO("road_topic_name is not defined");
    }//else

    private_node_handle.getParam("show_flag", paramters.show_flag);
    private_node_handle.getParam("debug_flag", paramters.debug_flag);
    private_node_handle.getParam("save_flag", paramters.save_flag);

}//GetParametersFromLaunch
}//perception
}//vecan

