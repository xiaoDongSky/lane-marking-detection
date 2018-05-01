/**
* @copyright 2018 VeCaN Lab
* @file lane.h
* @brief Defines the lane struct.
* @author zhyan
* @date 04/15/2018
*/

#ifndef _Lane_H_ 
#define _Lane_H_

#include "../include/lane_line.h"

/**
* @namespace vecan::perception
* @brief vecan::perception
*/

namespace vecan {
namespace perception {
struct Lane {

	enum ChangeType {
		PERMIT = 0,
		REJECT,
	};

	//left line of lane
	LaneLine left_line;
	//right line of lane
	LaneLine right_line;
	//able to change to the left lane
	ChangeType left_change_type;
	//able to change to the right lane
	ChangeType right_change_type;
	//center point
	int center;
	//lane detection score
	int score;
	//lane ID
	int ID;
	//tracking flag
	bool tracking;
	//find ==
	bool operator==(const Lane& objstruct) const {
		return (objstruct.center >= center - 50) && (objstruct.center <= center + 50);
	}
	bool operator<(const Lane& objstruct) const {
		return (objstruct.center > center + 50);
	}
	bool operator>(const Lane& objstruct) const {
		return (objstruct.center < center - 50);
	}
}; //struct Lane

} //namespace perception
} //namespace vecan


#endif //_Lane_H_
