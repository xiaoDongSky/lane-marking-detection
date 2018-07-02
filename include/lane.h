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
    int id;
	//tracking flag
	bool tracking;
    // The index of the lane with respect to the vehicle.
    // The lane where the vehicle is is indeitified as 0, the one on the left is -1,
    // the one on the right is 1, and so on.
    int offsetIndex;

    // Distance to the vehicle, with direction.
    // This value is negative when the lane is on the left side of the vehicle, positive when right.
    float offset;

    // Difference between the lane and the vehicle's heading, in rad.
    // This value is negative when the lane is tilt to the left of the vehicle, positive when right.
    float dirDiff;

    // Direct distance to the next stop line.
    float distanceToStop;

    cv::KalmanFilter kalman_filter_[2];

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
