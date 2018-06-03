/**
* @copyright 2018 VeCaN Lab
* @file lane_line.h
* @brief Defines the lane line struct.
* @author zhyan
* @date 04/15/2018
*/

#ifndef _Lane_Line_H_ 
#define _Lane_Line_H_

#include <vector>

/**
* @namespace vecan::perception
* @brief vecan::perception
*/

namespace vecan {
namespace perception {

struct LaneLine {

	enum LaneLineType {
		CURB = 0,
		SOLID,
		DASHED,
		SOLID_DASHED,
		DASHED_SOLID,
		SOLID_SOLID,
		DASHED_DASHED,
	};

	enum LaneLineColor {
		NO_COLOR = 0,
		WHITE,
		YELLOW,
	};

	//The ID
    int id;
	// The parameters of lane line function (parabola):
	// col = lines_factors[2] * row^2 + line_factors[1] * row + line_factors[0]
	std::vector<double> lines_factors;
    std::vector<double> ransac_lines_factors;
	// The color of the lane line;
	LaneLineColor color;
	// The type of the lane line;
	LaneLineType type;
	// The curvature of the lane line;
	int curvature;
	// The veracity of the detection;
	float score;
    // The start raw
    int start_row;
    // The end raw
    int end_row;
}; //struct LaneLine

} //namespace perception
} //namespace vecan


#endif //_Lane_Line_H_
