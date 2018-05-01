#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <string>

#include <local_messages/LaneMarker.h>
#include <local_messages/LaneMarkers.h>

#include "../include/lane_detector.h"
#include "../include/lane.h"

cv::Mat image_source;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::Mat tmp = cv_bridge::toCvShare(msg, "bgr8")->image;
    image_source = tmp.clone();
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_lane_detector");
    ros::NodeHandle img_handle;
    image_transport::ImageTransport it(img_handle);
    image_transport::Subscriber camera_sub = it.subscribe("camera", 1, imageCallback);

    //off-line pictures path
    std::string pictures_path_prefix = "/home/vecan/vecan_ugv/data/real_data/";
    std::string pictures_path;
    int frame_index = 0;
    char postfix_buffer[20];

    vecan::perception::LaneDetector lane_detector(1,1);
    bool on_line = false;
    cv::Mat img_output;

    while (ros::ok()) {
        if(on_line){
            ros::spinOnce();
        }
        else{
            sprintf(postfix_buffer, "%06d.bmp", frame_index);
            pictures_path = pictures_path_prefix + postfix_buffer;
            image_source = cv::imread(pictures_path);
            lane_detector.DetectLane(image_source,0);
            std::vector<vecan::perception::Lane> result;
            lane_detector.GetDetectionResult(result, img_output);
            //video_writer.write(img_output);
            std::cout << "frame: " << frame_index << "    " << result.size() << " lanes detected;" << std::endl;
            frame_index++;
        }
    }

    return 0;
}
