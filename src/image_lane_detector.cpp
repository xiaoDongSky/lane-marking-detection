/**
* @copyright 2018 VeCaN Lab
* @file image_lane_detector_node.cpp
* @brief the node to detect tha lane and stop line.
* @author zhyan
* @date 05/02/2018
*/

#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <string>

#include <local_messages/Road.h>

#include "../include/lane_detector.h"
#include "../include/lane.h"


//global image
cv::Mat image_source;
void imageCallback(const sensor_msgs::ImageConstPtr& msg){
    try{
        cv::Mat tmp = cv_bridge::toCvShare(msg, "bgr8")->image;
        image_source = tmp.clone();
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_lane_detector");
    ros::NodeHandle img_handle;
    image_transport::ImageTransport it(img_handle);
    image_transport::Subscriber camera_sub = it.subscribe("camera", 1, imageCallback);

    ros::NodeHandle road_handle;
    ros::Publisher road_pub = road_handle.advertise<local_messages::Road>("test", 2000);

    //sleep(5);
    //off-line pictures path
    std::string pictures_path_prefix = "/var/local/vu/image_data/lane_image1/";
    std::string pictures_path;
    int frame_index = 250;
    char postfix_buffer[20];

    vecan::perception::LaneDetector lane_detector(true,false,false);

    local_messages::Road road_msg;

    bool on_line = false;
    while (ros::ok()) {
        if(on_line){
            ros::spinOnce();
        }//if on_line
        else{
            sprintf(postfix_buffer, "%06d.bmp", frame_index);
            pictures_path = pictures_path_prefix + postfix_buffer;
            image_source = cv::imread(pictures_path);
        }//else
        std::cout << "frame: " << frame_index << "    ";
        lane_detector.DetectLane(image_source);
        lane_detector.PublishRoadMsg(road_msg);
        road_pub.publish(road_msg);
        frame_index++;
    }

    return 0;
}
