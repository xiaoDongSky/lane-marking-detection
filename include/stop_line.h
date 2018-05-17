/**
* @copyright 2018 VeCaN Lab
* @file stop_line.h
* @brief Defines the stop line struct.
* @author zhyan
* @date 04/23/2018
*/

#ifndef _Stop_Line_H_ 
#define _Stop_Line_H_

#include <vector>

/**
* @namespace vecan::perception
* @brief vecan::perception
*/

namespace vecan {
namespace perception {
struct StopLine {
	enum StopLineColor {
		NO_COLOR = 0,
		WHITE,
		YELLOW,
	};
	// The parameters of stop line function (parabola):
    // row = line_factors[1] * col + line_factors[0]
    std::vector<double> line_factors;
	// The color of the lane line;
	StopLineColor color;
	// The veracity of the detection;
	float score;
    // The flag of detection
    bool detected_flag;
}; //struct LaneLine

} //namespace perception
} //namespace vecan


#endif //_Stop_Line_H_
