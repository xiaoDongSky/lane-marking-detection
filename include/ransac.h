/**
* @copyright 2018 VeCaN Lab
* @file ransac.h
* @brief Defines the ransac.
* @author zhyan
* @date 05/28/2018
*/

#ifndef _Ransac_H_
#define _Ransac_H_

#include <stdlib.h>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

/**
* @namespace vecan::perception
* @brief vecan::perception
*/

namespace vecan {
namespace perception {
class RansacCurve{
public:
    //Default constructor with default parameters
    RansacCurve();
    //Constructor with the user-defined parameters
    RansacCurve(const int required_inliers_rate_, const int max_iterations, const double min_inliers_distance);
    //The function to do a curve fitting by RANSAC
    std::vector<double> GetBestModel(const std::vector<int>& points_x, const std::vector<int>& points_y);
private:
    static int get_random(int min, int max);
    std::vector<double> best_parameters_;
    int required_inliers_;
    int max_iterations_;
    double min_inliers_distance_;
    double required_inliers_rate_;
    std::vector<int> inliers_x_;
    std::vector<int> inliers_y_;
    std::vector<int> outliers_x_;
    std::vector<int> outliers_y_;

}; //struct RansacCurve
} //namespace perception
} //namespace vecan

#endif//_Ransac_H_
