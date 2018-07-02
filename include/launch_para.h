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
    //The program working mode(on-line or off-line)
    std::string mode;
    //The path of pictures under off-line mode
    std::string pictures_path;
    //The name of topic under on-line mode
    std::string image_topic_name;
    //The name of the result topic
    std::string road_topic_name;
    //The show flag of program
    bool show_flag;
    //The debug flag of program
    bool debug_flag;
    //The save flag of program
    bool save_flag;
};

void GetParametersFromLaunch(LaunchPara &paramters);

}//perception
}//vecan
#endif // _Launch_para_H_
