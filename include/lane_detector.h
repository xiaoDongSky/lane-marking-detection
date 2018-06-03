/**
* @copyright 2018 VeCaN Lab
* @file lane_detector.h
* @brief Defines the function of lane detection.
* @author zhyan
* @date 04/15/2018
*/

#ifndef _Lane_detector_H_ 
#define _Lane_detector_H_

#include <opencv2/core/core.hpp>
#include "lane_line.h"
#include "lane.h"
#include "stop_line.h"
#include "local_messages/Road.h"

/**
* @namespace vecan::perception
* @brief vecan::perception
*/

namespace vecan {
namespace perception {

class LaneDetector{
public:
    LaneDetector(const bool show_flag, const bool debug_flag, const bool save_flag);
    int DetectLane(const cv::Mat frame);
	int GetDetectionResult(std::vector<Lane> &result, cv::Mat &img);
    int PublishRoadMsg(local_messages::Road &road_msg);

private:

	/**
	* @brief: get binary from HSV color space
	* @param [in]:  frame       The input image in BGR color space.
	* @param [in]:  HSV_low     The lower limit of the HSV color space.
	* @param [in]:  HSV_high    The higher limit of the HSV color space.
	* @param [out]: binary_HSV  The result of binaryzation in HSV color space.
	* @return true if binary successfully, otherwise return false
	*/
	static int GetBinaryFromHSV(const cv::Mat frame,
		                        const cv::Scalar HSV_low,
		                        const cv::Scalar HSV_high,
		                        cv::Mat &img_binary_HSV);

	/**
	* @brief: get binary from equalized gray-scale image
	* @param [in]:  image_gray              The input image in gray-scale.
	* @param [in]:  gray_threshold          The threshold of gray-scale.
	* @param [out]: img_binary_equalized    The result of equalized gray-scale binaryzation.
	* @return true if binary successfully, otherwise return false
	*/
	static int GetBinaryFromEqualizedGrayscale(const cv::Mat img_gray,
		                                       const int gray_threshold,
		                                       cv::Mat &img_binary_equalized);

	/**
	* @brief: get binary from rules of lane-line
	* @param [in]:  image_gray              The input image in gray-scale.
	* @param [in]:  lane_line_width         The width of lane-line.
	* @param [in]:  difference_threshold    The threshold of difference between laneline and road.
	* @param [out]: img_binary_rules        The result of rules based binaryzation.
	* @return true if binary successfully, otherwise return false
	*/
	static int GetBinaryFromRules(const cv::Mat img_gray,
		                          const int lane_line_width,
		                          const int difference_threshold,
		                          cv::Mat &img_binary_rules);


	int GetBinary(const cv::Mat frame_input);

	int GetStopLineBinary(const cv::Mat img_gray,
		const int stop_line_width,
		const int difference_threshold);

	int GetPerspectiveMatrix(const std::vector<cv::Point2f> corners_source,
		                     const std::vector<cv::Point2f> corners_trans);


    int GetLaneLineCenter(const int histogram_width,
                          const int windows_width,
                          const int windows_min_numbers,
                          const int start_row,
                          std::vector<int> &center_points) const;

	int GetStopLineCenter(const int histogram_width,
		const int histogram_min_pixels,
		int &center_point) const;

	int GetStopLine(const int number_windows,
		const int window_half_width,
		const int window_min_pixels,
		const int stop_line_min_pixels);

    int GetCandidateBySlidingWindows(const std::vector<int> center_points,
                                     const int number_windows,
                                     const int window_half_width,
                                     const int window_min_pixels,
                                     std::vector<std::vector<int> > &candidate_x, std::vector<std::vector<int> > &candidate_y);

    int GetLaneLines(const int lane_line_min_pixels);

    int GetLaneLineTypeAndRange(std::vector<int> candidate_x,
                                std::vector<int> candidate_y,
                                LaneLine &lane_line);

	static int CalculateCurvature(const std::vector<double> factors,const int point_row, double &curvature);

	int GetLane(const int min_lane_width, const int max_lane_width);

	int LaneLinesShow();
	int LineShow(const LaneLine line);
	int StopLineShow();
	int TrackLane(const int min_lane_width, const int max_lane_width);


private:
	StopLine stop_line_;
	std::vector<LaneLine> lane_lines_;
	std::vector<Lane> lanes_;
	std::vector<Lane> tracking_lanes_;
	
	cv::Mat img_source_;
	cv::Mat img_bird_eye_;
	cv::Mat img_bird_eye_binary_;
    cv::Mat img_binary_stop_line_;
	cv::Mat img_display_;
	cv::Mat perspective_matrix_;
	cv::Mat inverse_perspective_matrix_;
    cv::Mat img_windows_;

    cv::Point2f vehicle_center_;
    double pixel_to_ground_x_;
    double pixel_to_ground_y_;
    int last_lane_id;

    bool debug_flag_;
    bool show_flag_;
    bool save_flag_;

}; //struct Lane
} //namespace perception
} //namespace vecan


#endif //_Lane_detector_H_
