/**
* @copyright 2018 VeCaN Lab
* @file lane_detector.cpp
* @brief realize the lane detection.
* @author zhyan
* @date 04/15/2018
*/
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <numeric>
#include <set>
#include <vector>

#include "local_messages/Road.h"

#include "../include/lane_detector.h"
#include "../include/lane_line_fitter.h"

namespace vecan {
namespace perception {

LaneDetector::LaneDetector(const bool show_flag, const bool debug_flag, const bool save_flag){
    show_flag_ = show_flag;
    debug_flag_ = debug_flag;
    save_flag_ = save_flag;
}

int LaneDetector::DetectLane(const cv::Mat frame) {
    GetBinary(frame);
    //GetLaneLines(1000);
    GetLaneLines(2000);
    if (debug_flag_) {
        std::cout << lane_lines_.size() << " lines are detected" << "    ";
    }
    GetLane(200, 500);
    TrackLane(200, 500);
    GetStopLine(20,50,200,5000);
    LaneLinesShow();

    return true;
}

int LaneDetector::GetDetectionResult(std::vector<Lane> &result, cv::Mat &img) {
    result = lanes_;
    img = img_display_.clone();
    return true;
}

int LaneDetector::GetBinaryFromHSV(const cv::Mat frame,
                                   const cv::Scalar HSV_low,
                                   const cv::Scalar HSV_high,
                                   cv::Mat &img_binary_HSV)
{
    if (frame.empty()) {
        std::cout<<"error in GetBinaryFromHSV: frame is empty" <<std::endl;
        return false;
    }//if
    cv::Mat tmp_image_HSV;
    cvtColor(frame, tmp_image_HSV, cv::COLOR_BGR2HSV);
    cv::inRange(tmp_image_HSV, HSV_low, HSV_high, img_binary_HSV);
    return true;
}//GetBinaryFromHSV

int LaneDetector::GetBinaryFromEqualizedGrayscale(const cv::Mat img_gray,
                                                  const int gray_threshold,
                                                  cv::Mat &img_binary_equalized){
    if (img_gray.empty()) {
        std::cout<<"error in GetBinaryFromEqualizedGrayscale: img_gray is empty"<<std::endl;
        return false;
    }//if
    equalizeHist(img_gray, img_binary_equalized);
    threshold(img_gray, img_binary_equalized, gray_threshold, 255, cv::THRESH_BINARY);
    return true;
}//GetBinaryFromEqualizedGrayscale

int LaneDetector::GetBinaryFromRules(const cv::Mat img_gray,
                                     const int lane_line_width,
                                     const int difference_threshold,
                                     cv::Mat &img_binary_rules) {
    if (img_gray.empty()) {
        std::cout<<"error in GetBinaryFromRules: image_gray is empty"<<std::endl;
        return false;
    }//if

    img_binary_rules = cv::Mat(img_gray.size(), CV_8U);
    img_binary_rules.setTo(0);
    int diff_left, diff_right;
    for (int row = 0; row < img_gray.rows; row++){
        for (int col = 0 + lane_line_width; col < img_gray.cols - lane_line_width; col++){
            diff_left = img_gray.at<uchar>(row, col) - img_gray.at<uchar>(row, col - lane_line_width);
            diff_right = img_gray.at<uchar>(row, col) - img_gray.at<uchar>(row, col + lane_line_width);
            if (diff_left > difference_threshold && diff_right > difference_threshold){
                img_binary_rules.at<uchar>(row, col) = 255;
            }//if
        }//for col
    }//for row
    return true;
}//GetBinaryFromRules

int LaneDetector::GetStopLineBinary(const cv::Mat img_gray,
                                    const int stop_line_width,
                                    const int difference_threshold) {
    if (img_gray.empty()) {
        std::cout<<"error in GetStopLineBinary: image_gray is empty"<<std::endl;
        return false;
    }//if

    img_binary_stop_line_ = cv::Mat(img_gray.size(), CV_8U);
    img_binary_stop_line_.setTo(0);
    int diff_left, diff_right;
    for (int row = 0 + stop_line_width; row < img_gray.rows - stop_line_width; row++){
        for (int col = 0; col < img_gray.cols; col++){
            diff_left = img_gray.at<uchar>(row, col) - img_gray.at<uchar>(row - stop_line_width, col);
            diff_right = img_gray.at<uchar>(row, col) - img_gray.at<uchar>(row + stop_line_width, col);
            if (diff_left > difference_threshold && diff_right > difference_threshold){
                img_binary_stop_line_.at<uchar>(row, col) = 255;
            }
        }
    }
    return true;
}//GetStopLineBinary

int LaneDetector::GetBinary(const cv::Mat frame_input) {
    if (frame_input.empty()) {
        std::cout<<"error in GetBinary: no input frame"<<std::endl;
        return false;
    }//if
    std::vector<cv::Point2f> corners_source(4);
    std::vector<cv::Point2f> corners_trans(4);
    //corners_source[0] = cv::Point2f(530, 457);
    //corners_source[1] = cv::Point2f(775, 457);
    corners_source[0] = cv::Point2f(540, 457);
    corners_source[1] = cv::Point2f(785, 457);
    corners_source[2] = cv::Point2f(201, 712);
    corners_source[3] = cv::Point2f(1085, 712);

    int tmp_row_offset = 450;
    int tmp_col_offset_left = 460;
    int tmp_col_offset_right = 460;
    //int tmp_row_offset = 550;
    //int tmp_col_offset_left = 520;
    //int tmp_col_offset_right = 520;
    corners_trans[0] = cv::Point2f(0 + tmp_col_offset_left, tmp_row_offset);
    corners_trans[1] = cv::Point2f(frame_input.cols - tmp_col_offset_right, tmp_row_offset);
    corners_trans[2] = cv::Point2f(0 + tmp_col_offset_left, frame_input.rows);
    corners_trans[3] = cv::Point2f(frame_input.cols - tmp_col_offset_right, frame_input.rows);

    GetPerspectiveMatrix(corners_source, corners_trans);
    img_source_ = frame_input.clone();
    cv::warpPerspective(img_source_, img_bird_eye_, perspective_matrix_, img_source_.size());
    cv::Mat img_bird_eye_gray;
    cvtColor(img_bird_eye_, img_bird_eye_gray, cv::COLOR_BGR2GRAY);
    cv::Mat img_binary_rules;
    GetBinaryFromRules(img_bird_eye_gray, 15 ,25, img_binary_rules);
    GetStopLineBinary(img_bird_eye_gray,15,40);
    cv::Mat img_binary_equalized;
    GetBinaryFromEqualizedGrayscale(img_bird_eye_gray, 170, img_binary_equalized);//170
    cv::bitwise_and(img_binary_rules, img_binary_equalized, img_bird_eye_binary_);

    if (debug_flag_) {
        cv::imshow("img_bird_eye_", img_bird_eye_);
        cv::imshow("img_binary_rules", img_binary_rules);
        cv::imshow("img_binary_equalized", img_binary_equalized);
        cv::imshow("img_bird_eye_binary_", img_bird_eye_binary_);
        cv::imshow("img_binary_stop_line_", img_binary_stop_line_);
        cvWaitKey(1);
    }//if

    if (save_flag_) {
        cv::imwrite("../result/source_img.bmp", img_source_);
        cv::imwrite("../result/bird_eye_img.bmp", img_bird_eye_);
        cv::imwrite("../result/binary_rules_img.bmp", img_binary_rules);
        cv::imwrite("../result/binary_equalized_img.bmp", img_binary_equalized);
        cv::imwrite("../result/binary_bird_eye_img.bmp", img_bird_eye_binary_);
        cv::imwrite("../result/binary_stop_line.bmp", img_binary_stop_line_);
    }//if

    return true;
}//GetBinary

int LaneDetector::GetPerspectiveMatrix(const std::vector<cv::Point2f> corners_source,
                                       const std::vector<cv::Point2f> corners_trans) {
    if (corners_source.size() != 4 || corners_trans.size() != 4) {
        std::cout<< "error in GetPerspectiveMatrix" <<std::endl;
        return false;
    }//if
    perspective_matrix_ = cv::getPerspectiveTransform(corners_source, corners_trans);
    inverse_perspective_matrix_ = cv::getPerspectiveTransform(corners_trans, corners_source);
    return true;
}//GetPerspectiveMatrix

int LaneDetector::GetLaneLineCenter(const int histogram_width,
                                    const int windows_width,
                                    const int windows_min_numbers,
                                    const int start_row,
                                    std::vector<int> &center_points) const {
    if (img_bird_eye_binary_.empty()) {
        std::cout<<"error in GetLaneLineCenter: img_bird_eye_binary_ is empty"<<std::endl;
        return false;
    }//if

    if (start_row >= img_bird_eye_binary_.rows || start_row < 0) {
        std::cout<<"error in GetLaneLineCenter: start_row is not good"<<std::endl;
        return false;
    }//if

    //获取直方图
    int number_histogram = floor(img_bird_eye_binary_.cols / histogram_width);
    cv::Mat histogram = cv::Mat(1, number_histogram, CV_32S, 0.0);
    for (int number = 0; number < number_histogram; ++number)
        for (int row = start_row; row < img_bird_eye_binary_.rows; row++){
            for (int tmp_col = number*histogram_width; tmp_col < (number +1)*histogram_width; ++tmp_col){
                if (img_bird_eye_binary_.at<uchar>(row, tmp_col) == 255)
                    histogram.at<int>(0, number) = histogram.at<int>(0, number) + 1;
            }//for tmp_col
        }//for row

    //std::cout << histogram << std::endl;

    //得到直方图峰值
    int tmp_windows_width = windows_width/ histogram_width;
    for (int i = 0; i < histogram.cols - tmp_windows_width; ++i){
        int tmp_max = windows_min_numbers;
        int tmp_histogram_index = -1;
        for (int j = i; j < i + tmp_windows_width; j++){
            if (histogram.at<int>(0, j) > tmp_max){
                tmp_max = histogram.at<int>(0, j);
                if (tmp_histogram_index != -1){
                    histogram.at<int>(0, tmp_histogram_index) = 0;
                }//if
                tmp_histogram_index = j;
            }//if
            else{
                histogram.at<int>(0, j) = 0;
            }//else
        }//for j
    }//for i

    for (int i = 0; i < histogram.cols; i++){
        if (histogram.at<int>(0, i) > 0){
            center_points.push_back(i*histogram_width);
        }//if
    }//for

    if(debug_flag_){
        std::cout<<center_points.size()<<" center points are found:  ";
        for(int i = 0; i < center_points.size(); ++i){
            std::cout<<center_points[i]<<"  ";
        }
        std::cout<<std::endl;
    }//if

    return true;
}//function GetLaneLineCenter

int LaneDetector::GetStopLineCenter(const int histogram_width,
    const int histogram_min_pixels,
    int &center_point) const {
    if (img_binary_stop_line_.empty()) {
        return false;
    }//if

     //获取直方图
    int number_histogram = floor(img_binary_stop_line_.rows / histogram_width);
    cv::Mat histogram = cv::Mat(1, number_histogram, CV_32S, 0.0);
    for (int number = 0; number < number_histogram; ++number)
        for (int col = img_binary_stop_line_.cols/2; col < img_binary_stop_line_.cols; ++col) {
            for (int tmp_row = number*histogram_width; tmp_row < (number + 1)*histogram_width; ++tmp_row) {
                if (img_binary_stop_line_.at<uchar>(tmp_row, col) == 255)
                    histogram.at<int>(0, number) += 1;
            }//for tmp_col
        }//for row

    //得到直方图峰值
    double histogram_min, histogram_max;
    cv::Point min_loc, max_loc;
    cv::minMaxLoc(histogram, &histogram_min, &histogram_max, &min_loc, &max_loc);
    if (histogram_max > histogram_min_pixels) {
        center_point = max_loc.x * histogram_width + histogram_width/2;
    }
    else {
        center_point = -1;
    }
    return true;
}//function GetLaneLineCenter

int LaneDetector::GetStopLine(const int number_windows,
    const int window_half_width,
    const int window_min_pixels,
    const int stop_line_min_pixels) {
    if (img_binary_stop_line_.empty()) {
        return -1;
    }//if
    stop_line_.lines_factors.clear();
    int center_point = 0;
    GetStopLineCenter(10,500, center_point);
    if (center_point == -1) {
        return 0;
    }
    int tmp_center = center_point;
    cv::Mat windows_img;
    if (debug_flag_) {
        windows_img = img_binary_stop_line_.clone();
        cvtColor(windows_img, windows_img, cv::COLOR_GRAY2BGR);
    }//if
    int window_height = floor(img_bird_eye_binary_.cols / number_windows);
    std::vector<int> stop_line_inds_x;
    std::vector<int> stop_line_inds_y;
    for (int index_windows = 0; index_windows < number_windows; ++index_windows)
    {
        int win_y_low = tmp_center - window_half_width;
        int win_y_high = tmp_center + window_half_width;
        int win_x_low = index_windows * window_height;
        int win_x_high = (index_windows + 1) * window_height;
        win_x_high = cv::min(win_x_high, img_binary_stop_line_.cols);
        win_x_low = cv::max(win_x_low, 0);
        win_y_high = cv::min(win_y_high, img_binary_stop_line_.rows);
        win_y_low = cv::max(win_y_low, 0);
        if (debug_flag_) {
            cv::rectangle(windows_img, cv::Point(win_x_low, win_y_low), cv::Point(win_x_high, win_y_high), cv::Scalar(0, 255, 0), 2);
            cv::rectangle(windows_img, cv::Point(win_x_low, win_y_low), cv::Point(win_x_high, win_y_high), cv::Scalar(0, 255, 0), 2);
        }
        std::vector<int> window_inds_x;
        std::vector<int> window_inds_y;

        for (int row = win_y_low; row < win_y_high; row++)
            for (int col = win_x_low; col < win_x_high; col++)
            {
                if (img_binary_stop_line_.at<uchar>(row, col) == 255)
                {
                    window_inds_x.push_back(col);
                    window_inds_y.push_back(row);
                }

            }
        if (window_inds_x.size() > window_min_pixels)
        {
            stop_line_inds_x.insert(stop_line_inds_x.end(), window_inds_x.begin(), window_inds_x.end());
            stop_line_inds_y.insert(stop_line_inds_y.end(), window_inds_y.begin(), window_inds_y.end());
            tmp_center = std::accumulate(window_inds_y.begin(), window_inds_y.end(), 0.0) / window_inds_y.size();
        }
    }
    vecan::Fit fit_state;

    if (stop_line_inds_x.size() > stop_line_min_pixels) {

        fit_state.polyfit(stop_line_inds_x, stop_line_inds_y, 2, true);
        fit_state.getFactor(stop_line_.lines_factors);
    }
    if (debug_flag_) {
        cv::imshow("stop_line_windows", windows_img);
    }
    if (save_flag_) {
        cv::imwrite("..//result//stop_line_windows.bmp", windows_img);
    }
    return true;
}

int LaneDetector::GetCandidateBySlidingWindows(const std::vector<int> center_points,
    const int number_windows,
    const int window_half_width,
    const int window_min_pixels,
    std::vector< std::vector<int>> &candidate_x,
    std::vector< std::vector<int>> &candidate_y){

    if (img_bird_eye_binary_.empty()) {
        std::cout<<"error in GetCandidateBySlidingWindows: img_bird_eye_binary_ is empty "<<std::endl;
        return false;
    }//if

    std::vector<int> window_center = center_points;
    cv::Mat windows_img;
    if (debug_flag_){
        windows_img = img_bird_eye_binary_.clone();
        cvtColor(windows_img, windows_img, cv::COLOR_GRAY2BGR);
    }//if

    int window_height = floor(img_bird_eye_binary_.rows / number_windows);
    for (int line_index = 0; line_index < window_center.size(); line_index++) {
        std::vector<int> lane_lines_points_x;
        std::vector<int> lane_lines_points_y;
        int last_offset = 0;
        int tmp_last_window_flag = 0;
        for (int index_windows = 0; index_windows < number_windows; ++index_windows) {
            int window_y_low = img_bird_eye_binary_.rows - (index_windows + 1) * window_height;
            int window_y_high = img_bird_eye_binary_.rows - index_windows  * window_height;
            int window_x_low = window_center[line_index] - window_half_width;
            int window_x_high = window_center[line_index] + window_half_width;
            window_y_high = cv::min(window_y_high, img_bird_eye_binary_.rows);
            window_y_low = cv::max(window_y_low, 0);
            window_x_high = cv::min(window_x_high, img_bird_eye_binary_.cols);
            window_x_low = cv::max(window_x_low, 0);

            if (debug_flag_) {
                cv::rectangle(windows_img, cv::Point(window_x_low, window_y_low), cv::Point(window_x_high, window_y_high), cv::Scalar(0, 255, 0), 2);
            }//if

            std::vector<int> window_inds_x;
            std::vector<int> window_inds_y;

            for (int row = window_y_low; row < window_y_high; ++row)
                for (int col = window_x_low; col < window_x_high; ++col) {
                    if (img_bird_eye_binary_.at<uchar>(row, col) == 255) {
                        window_inds_x.push_back(col);
                        window_inds_y.push_back(row);
                    }//if
                }//for col
            if (window_inds_x.size() > window_min_pixels) {
                lane_lines_points_x.insert(lane_lines_points_x.end(), window_inds_x.begin(), window_inds_x.end());
                lane_lines_points_y.insert(lane_lines_points_y.end(), window_inds_y.begin(), window_inds_y.end());
                int tmp_center = std::accumulate(window_inds_x.begin(), window_inds_x.end(), 0.0) / window_inds_x.size();
                if (tmp_last_window_flag) {
                    last_offset = tmp_center - window_center[line_index];
                }

                window_center[line_index] = tmp_center;
                tmp_last_window_flag = 1;
            }//if
            else {
                window_center[line_index] += last_offset;
                tmp_last_window_flag = 0;
            }
        }//for index_windows
        candidate_x.push_back(lane_lines_points_x);
        candidate_y.push_back(lane_lines_points_y);
    }//for line_index

    if (debug_flag_) {
        cv::imshow("windows_img",windows_img);
    }//if
    if (save_flag_) {
        cv::imwrite("lane_windows_img.bmp", windows_img);
    }

}//function GetCandidateBySlidingWindows


int LaneDetector::GetLaneLines(const int lane_line_min_pixels){
    std::vector<int> line_center_points;
    GetLaneLineCenter(10, 100, 50, 600, line_center_points);

    std::vector< std::vector<int>> candidate_x;
    std::vector< std::vector<int>> candidate_y;
    GetCandidateBySlidingWindows(line_center_points,20,50,100, candidate_x, candidate_y);

    if (candidate_x.size() != candidate_y.size()) {
        std::cout<<"error in GetLaneLines: candidate points not good "<<std::endl;
        return false;
    }//if

    lane_lines_.clear();
    const cv::Scalar HSV_yello_low(10, 3, 150);
    const cv::Scalar HSV_yello_high(90, 255, 220);
    cv::Mat img_HSV;
    GetBinaryFromHSV(img_source_, HSV_yello_low, HSV_yello_high, img_HSV);

    Fit fitter;
    for (int line_index = 0; line_index < candidate_x.size(); ++line_index) {
        if (candidate_x[line_index].size() > lane_line_min_pixels) {
            std::vector<double> tmp_factors;
            LaneLine tmp_line;
            tmp_line.score = (candidate_y[line_index].size() - 3000.0) / 3000.0;
            fitter.polyfit(candidate_y[line_index], candidate_x[line_index], 2, true);
            fitter.getFactor(tmp_factors);
            int yellow_points_num = 0;
            for (int point_index = 0; point_index < candidate_x[line_index].size(); ++point_index)
            {
                if (img_HSV.at<uchar>(candidate_y[line_index][point_index], candidate_x[line_index][point_index]) > 0)
                    yellow_points_num++;
            }

            if (yellow_points_num * 3 > candidate_x[line_index].size())
                tmp_line.color = LaneLine::YELLOW;
            else
                tmp_line.color = LaneLine::WHITE;

            GetLaneLineTypeAndRange(candidate_y[line_index],tmp_line);
            tmp_line.ID = line_index;
            tmp_line.lines_factors = tmp_factors;
            lane_lines_.push_back(tmp_line);
        }//if
    }//for line_index

}//GetLaneLines

int LaneDetector::GetLaneLineTypeAndRange(std::vector<int> candidate_y, LaneLine &lane_line){
    if(candidate_y.empty()){
        std::cout<<"error in GetLaneLinesType: candidate_y is empty"<<std::endl;
        return false;
    }
    std::set<int> row_record;
    for (int point_index = 0; point_index < candidate_y.size(); ++point_index)
    {
        row_record.insert(candidate_y[point_index]);
    }
    std::set<int>::iterator tmp_it =row_record.end();
    tmp_it--;
    int candidata_y_max = *tmp_it;
    int candidata_y_min = *(row_record.begin());

    if(row_record.size() * 2 > (candidata_y_max - candidata_y_min)){
        lane_line.type = LaneLine::SOLID;
    }
    else{
        lane_line.type = LaneLine::DASHED;
    }

    lane_line.start_row = candidata_y_max;
    lane_line.end_row = candidata_y_min;
    return true;


}//GetLaneLinesType

int LaneDetector::CalculateCurvature(const std::vector<double> factors, const int point_row, double &curvature){
    if (factors.size() != 3) {
        return false;
    }//if
    curvature = 2 * point_row*factors[2] * point_row + factors[1];
    return true;
}//CalculateCurvature

int LaneDetector::GetLane(const int min_lane_width,const int max_lane_width){
    if (lane_lines_.empty()) {
        return false;
    }
    lanes_.clear();
    for (int line_index = 0; line_index < lane_lines_.size() - 1; ++line_index)
        for (int compared_line_index = line_index + 1; compared_line_index < lane_lines_.size(); ++compared_line_index) {
            int line_top_x = lane_lines_[line_index].lines_factors[0];
            int compared_line_top_x = lane_lines_[compared_line_index].lines_factors[0];
            int line_center_x = pow(img_bird_eye_.rows / 2, 2) * lane_lines_[line_index].lines_factors[2] + img_bird_eye_.rows / 2 * lane_lines_[line_index].lines_factors[1] + lane_lines_[line_index].lines_factors[0];
            int compared_line_center_x = pow(img_bird_eye_.rows / 2, 2) * lane_lines_[compared_line_index].lines_factors[2] + img_bird_eye_.rows / 2 * lane_lines_[compared_line_index].lines_factors[1] + lane_lines_[compared_line_index].lines_factors[0];
            int line_bottom_x = pow(img_bird_eye_.rows, 2) * lane_lines_[line_index].lines_factors[2] + img_bird_eye_.rows * lane_lines_[line_index].lines_factors[1] + lane_lines_[line_index].lines_factors[0];
            int compared_line_bottom_x = pow(img_bird_eye_.rows, 2) * lane_lines_[compared_line_index].lines_factors[2] + img_bird_eye_.rows* lane_lines_[compared_line_index].lines_factors[1] + lane_lines_[compared_line_index].lines_factors[0];
            Lane tmp_lane;
if (//line_top_x - compared_line_top_x > min_lane_width && line_top_x - compared_line_top_x < max_lane_width &&
    line_center_x - compared_line_center_x > min_lane_width && line_center_x - compared_line_center_x < max_lane_width &&
    line_bottom_x - compared_line_bottom_x > min_lane_width && line_bottom_x - compared_line_bottom_x < max_lane_width) {
    tmp_lane.left_line = lane_lines_[compared_line_index];
    tmp_lane.right_line = lane_lines_[line_index];
    lanes_.push_back(tmp_lane);
}//if
else if (//compared_line_top_x - line_top_x > min_lane_width && compared_line_top_x - line_top_x < max_lane_width &&
    compared_line_center_x - line_center_x > min_lane_width && compared_line_center_x - line_center_x < max_lane_width &&
    compared_line_bottom_x - line_bottom_x > min_lane_width && compared_line_bottom_x - line_bottom_x < max_lane_width) {
    tmp_lane.left_line = lane_lines_[line_index];
    tmp_lane.right_line = lane_lines_[compared_line_index];
    lanes_.push_back(tmp_lane);
}//else if
    }//for line_index

    for (int lane_index = 0; lane_index < lanes_.size(); ++lane_index) {
        if (lanes_[lane_index].left_line.type == LaneLine::DASHED ||
            lanes_[lane_index].left_line.type == LaneLine::DASHED_DASHED ||
            lanes_[lane_index].left_line.type == LaneLine::DASHED_SOLID) {
            lanes_[lane_index].left_change_type = Lane::PERMIT;
        }//if
        else {
            lanes_[lane_index].left_change_type = Lane::REJECT;
        }//else
        if (lanes_[lane_index].right_line.type == LaneLine::DASHED ||
            lanes_[lane_index].right_line.type == LaneLine::DASHED_DASHED ||
            lanes_[lane_index].right_line.type == LaneLine::DASHED_SOLID) {
            lanes_[lane_index].right_change_type = Lane::PERMIT;
        }//if
        else {
            lanes_[lane_index].right_change_type = Lane::REJECT;
        }//else
        lanes_[lane_index].center = (lanes_[lane_index].left_line.lines_factors[0] + lanes_[lane_index].right_line.lines_factors[0]) / 2;
    }// for lane_index
    return true;
}// GetLane

int LaneDetector::TrackLane(const int min_lane_width, const int max_lane_width) {
    if (lanes_.empty()) {
        for (int index = 0; index < tracking_lanes_.size(); ++index) {
            tracking_lanes_[index].score--;
        }
        std::vector<Lane>::iterator it;
        for (it = tracking_lanes_.begin(); it != tracking_lanes_.end(); ) {
            if ((*it).score < 0) {
                tracking_lanes_.erase(it);
            }
            else {
                it++;
            }
        }
        return false;
    }
    std::sort(lanes_.begin(), lanes_.end());
    std::sort(tracking_lanes_.begin(), tracking_lanes_.end());

    if (tracking_lanes_.empty()) {
        std::vector<std::vector<int>> combined_lanes_index;
        std::set<int> used_index;
        if (lanes_.size() == 1) {
            std::vector<int> lanes_index;
            lanes_index.push_back(0);
            combined_lanes_index.push_back(lanes_index);
        }
        for (int index = 0; index < lanes_.size() - 1; ++index) {
            std::vector<int> lanes_index;
            if (used_index.count(index) == 0) {
                lanes_index.push_back(index);
                for (int compared_index = index + 1; compared_index < lanes_.size(); ++compared_index) {
                    if (used_index.count(compared_index) == 0) {
                        if (lanes_[compared_index].center - lanes_[index].center > min_lane_width &&
                            lanes_[compared_index].center - lanes_[index].center < max_lane_width) {
                            lanes_index.push_back(compared_index);
                        }//if

                    }//if
                }//for compared_index
                combined_lanes_index.push_back(lanes_index);
            }//if
        }//for index

        if (combined_lanes_index.size() == 1) {
            for (int lane_index = 0; lane_index < lanes_.size(); ++lane_index) {
                tracking_lanes_.push_back(lanes_[lane_index]);
                tracking_lanes_[lane_index].score = 1;
                tracking_lanes_[lane_index].ID = lane_index;
            }//for lane_index
        }
        else {
            int max_num = 0, max_index = 0;
            for (int tmp = 0; tmp < combined_lanes_index.size(); ++tmp) {
                if (combined_lanes_index[tmp].size() > max_num) {
                    max_num = combined_lanes_index[tmp].size();
                    max_index = tmp;
                }//if
            }//for
            for (int lane_index = 0; lane_index < combined_lanes_index[max_index].size(); ++lane_index) {
                tracking_lanes_.push_back(lanes_[combined_lanes_index[max_index][lane_index]]);
                tracking_lanes_[lane_index].score = 1;
                tracking_lanes_[lane_index].ID = lane_index;
            }//for lane_index
        }
    }//if empty
    else {
        for (int index = 0; index < tracking_lanes_.size(); ++index) {
            tracking_lanes_[index].tracking = false;
        }
        int last_lanes_num = tracking_lanes_.size();
        for (int lane_index = 0; lane_index < lanes_.size(); ++lane_index) {
            std::vector<Lane>::iterator it = find(tracking_lanes_.begin(), tracking_lanes_.end(), lanes_[lane_index]);
            if (it == tracking_lanes_.end()) {
                if (lanes_[lane_index].center - (*(it - 1)).center > min_lane_width &&
                    lanes_[lane_index].center - (*(it - 1)).center < max_lane_width){
                    lanes_[lane_index].score = 1;
                    lanes_[lane_index].ID = tracking_lanes_.size();
                    tracking_lanes_.push_back(lanes_[lane_index]);

                }//if
                else if (-lanes_[lane_index].center +(*tracking_lanes_.begin()).center > min_lane_width &&
                    -lanes_[lane_index].center + (*tracking_lanes_.begin()).center < max_lane_width) {
                    for (int index = 0; index < tracking_lanes_.size(); ++index) {
                        tracking_lanes_[index].ID += 1;
                    }
                    lanes_[lane_index].score = 1;
                    lanes_[lane_index].ID = 0;
                    tracking_lanes_.push_back(lanes_[lane_index]);

                }
            }

            else {
                lanes_[lane_index].tracking = true;
                lanes_[lane_index].score = cv::min((*it).score + 1, 6);
                lanes_[lane_index].ID = (*it).ID;
                (*it) = lanes_[lane_index];
            }//else
        }//for
        for (int index = 0; index < last_lanes_num; ++index) {
            if (!tracking_lanes_[index].tracking) {
                tracking_lanes_[index].score--;
            }
        }
        std::vector<Lane>::iterator it;
        for (it = tracking_lanes_.begin(); it != tracking_lanes_.end(); ) {
            if ((*it).score < 0) {
                tracking_lanes_.erase(it);
            }
            else {
                it++;
            }
        }
    }
    return true;
}

int LaneDetector::LaneLinesShow(){
    if (img_source_.empty()) {
        return false;
    }
    if (img_display_.empty()) {
        img_display_ = cv::Mat(img_source_.size(), img_source_.type());
    }
    img_display_.setTo(0);
    std::set<int> drawID;

    for (int lane_index = 0; lane_index < tracking_lanes_.size(); ++lane_index) {
        if (tracking_lanes_[lane_index].score >= 3) {
            if (drawID.count(tracking_lanes_[lane_index].left_line.ID) == 0) {
                drawID.insert(tracking_lanes_[lane_index].left_line.ID);
                LineShow(tracking_lanes_[lane_index].left_line);
            }//
            if (drawID.count(tracking_lanes_[lane_index].right_line.ID) == 0) {
                drawID.insert(tracking_lanes_[lane_index].right_line.ID);
                LineShow(tracking_lanes_[lane_index].right_line);
            }
        }

    }
    StopLineShow();
    cv::warpPerspective(img_display_, img_display_, inverse_perspective_matrix_, img_display_.size());
    cv::addWeighted(img_source_, 1, img_display_, 1, 0, img_display_);
    cv::imshow("result", img_display_);
    cvWaitKey(1);
}

int LaneDetector::LineShow(const LaneLine line){
    if (line.lines_factors.size() != 3) {
        return false;
    }

    cv::Vec3b tmp_draw_color = cv::Vec3b(0, 0, 255);
    for (int row = 0; row < img_display_.rows; row++){

        int col = line.lines_factors[2] * row * row + line.lines_factors[1] * row + line.lines_factors[0];
        int tmp_flag = 1;
        if (stop_line_.lines_factors.size() == 3) {
            if (row < stop_line_.lines_factors[2] * col * col + stop_line_.lines_factors[1] * col + stop_line_.lines_factors[0]) {
                tmp_flag = 0;
            }
        }
        if (tmp_flag && col >= 20 && col < img_display_.cols - 20){
            for (int tmp = col - 20; tmp < col + 20; tmp++){
                img_display_.at<cv::Vec3b>(row, tmp) = tmp_draw_color;
            }//for tmp
        }//if
    }//for row

}//LineShow

int LaneDetector::StopLineShow() {
    if (stop_line_.lines_factors.size() != 3) {
        return false;
    }
    cv::Vec3b tmp_draw_color = cv::Vec3b(0, 0, 255);
    for (int col = 0; col < img_display_.cols; col++) {
        int row = stop_line_.lines_factors[2] * col * col + stop_line_.lines_factors[1] * col + stop_line_.lines_factors[0];
        if (row >= 20 && row < img_display_.rows - 20) {
            for (int tmp = row - 20; tmp < row + 20; tmp++) {
                img_display_.at<cv::Vec3b>(tmp, col) = tmp_draw_color;
            }//for tmp
        }//if
    }//for row

}//LineShow

} //namespace perception
} //namespace vecan
