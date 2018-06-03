/**
* @copyright 2018 VeCaN Lab
* @file launch_para.h
* @brief Defines the package required parameters.
* @author zhyan
* @date 05/31/2018
*/

#ifndef _Launch_para_H_
#define _Launch_para_H_
#include <string>
namespace vecan {
namespace perception {
struct LaunchPara{
    std::string mode;
    std::string pictures_path;
    std::string image_topic_name;
    std::string road_topic_name;
    bool show_flag;
    bool debug_flag;
    bool save_flag;
};

void GetParametersFromLaunch(LaunchPara &paramters);

}//perception
}//vecan
#endif // _Launch_para_H_
