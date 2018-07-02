
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
#include "opencv2/video/tracking.hpp"

#include <algorithm>
#include <numeric>
#include <set>
#include <vector>

#include "../include/lane_detector.h"
#include "../include/lane_line_fitter.h"
#include "../include/ransac.h"
namespace vecan {
namespace perception {

LaneDetector::LaneDetector(const bool show_flag, const bool debug_flag, const bool save_flag){
    show_flag_ = show_flag;
    debug_flag_ = debug_flag;
    save_flag_ = save_flag;

    stop_line_.detected_flag = false;
    vehicle_center_.x = 640;
    vehicle_center_.y = 0;
    pixel_to_ground_x_ = 1;
    pixel_to_ground_y_ = 1;
    last_lane_id = -1;


}

int LaneDetector::DetectLane(const cv::Mat frame) {
    GetBinary(frame);
    //GetLaneLines(1000);
    GetStopLine(20,50,100,3000,8);
    GetLaneLines(600);
    if (debug_flag_) {
        std::cout << lane_lines_.size() << " lines are detected" << "    ";
    }
    GetLane(100, 400);
    TrackLane(100, 400);
    if(show_flag_ == true){
        LaneLinesShow();
    }
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
    //equalizeHist(img_gray, img_binary_equalized);
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
    int diff_left, diff_right, diff_left_right;
    for (int row = 0; row < img_gray.rows; row++){
        for (int col = 0 + lane_line_width; col < img_gray.cols - lane_line_width; col++){
            int tmp_difference_threshold = difference_threshold * (1 - 0.3 * (img_gray.rows - row)/img_gray.rows - 0.3 * 2 *abs(col - img_gray.cols/2)/img_gray.cols);
            diff_left = img_gray.at<uchar>(row, col) - img_gray.at<uchar>(row, col - lane_line_width);
            diff_right = img_gray.at<uchar>(row, col) - img_gray.at<uchar>(row, col + lane_line_width);
            //diff_left_right = abs(img_gray.at<uchar>(row, col - lane_line_width) - img_gray.at<uchar>(row, col + lane_line_width));

            if (diff_left > tmp_difference_threshold && diff_right > tmp_difference_threshold ){
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

    int tmp_row_offset = 480;
    int tmp_col_offset_left = 500;
    int tmp_col_offset_right = 500;
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
    GetBinaryFromRules(img_bird_eye_gray, 12 ,30, img_binary_rules);
    GetStopLineBinary(img_bird_eye_gray,10,40);
    cv::Mat img_binary_equalized;
    GetBinaryFromEqualizedGrayscale(img_bird_eye_gray, 175, img_binary_equalized);//170
    cv::Mat tmp_test;
    DeleteBigConnectedRegion(img_binary_equalized, tmp_test, 1, 5000);
    cv::imshow("tmp_test",tmp_test);
    //    cv::Mat img_white_lanes,img_yello_lanes;
    //    cv::Scalar white_high(180,30,255),white_low(0,0,180);
    //    GetBinaryFromHSV(img_bird_eye_,white_low,white_high,img_white_lanes);
    //    cv::Scalar yello_high(34,255,255),yello_low(11,43,46);
    //    GetBinaryFromHSV(img_bird_eye_,yello_low,yello_high,img_yello_lanes);
    //    cv::Mat image_hsv;
    //    cv::addWeighted(img_white_lanes, 1, img_yello_lanes, 1, 0., image_hsv);
    //    cv::imshow("img_yello_lanes", img_yello_lanes);
    //    cv::imshow("img_white_lanes", img_white_lanes);
    //    cv::imshow("image_hsv", image_hsv);


    cv::bitwise_and(img_binary_rules, tmp_test, img_bird_eye_binary_);
    cv::bitwise_and(img_binary_stop_line_, img_binary_equalized, img_binary_stop_line_);
    //    cv::Mat gaussian_image;
    //    cv::GaussianBlur(img_bird_eye_gray,gaussian_image,cv::Size(3,3),0);
    //    cv::imshow("GaussianBlur", gaussian_image);
    //    cv::Canny(gaussian_image,gaussian_image,30,40);
    //    cv::imshow("Canny", gaussian_image);

    if (debug_flag_) {
        cv::imshow("img_bird_eye_", img_bird_eye_);
        cv::imshow("img_binary_rules", img_binary_rules);
        cv::imshow("img_binary_equalized", img_binary_equalized);
        cv::imshow("img_bird_eye_binary_", img_bird_eye_binary_);
        cv::imshow("img_binary_stop_line_", img_binary_stop_line_);
        cvWaitKey(1);
    }//if

    if (save_flag_) {
        cv::imwrite("src/image_lane_detector/result/source_img.bmp", img_source_);
        cv::imwrite("src/image_lane_detector/result/bird_eye_img.bmp", img_bird_eye_);
        cv::imwrite("src/image_lane_detector/result/binary_rules_img.bmp", img_binary_rules);
        cv::imwrite("src/image_lane_detector/result/binary_equalized_img.bmp", img_binary_equalized);
        cv::imwrite("src/image_lane_detector/result/binary_bird_eye_img.bmp", img_bird_eye_binary_);
        cv::imwrite("src/image_lane_detector/result/binary_stop_line.bmp", img_binary_stop_line_);
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
    float number_histogram = floor(img_bird_eye_binary_.cols / histogram_width);
    cv::Mat histogram = cv::Mat(1, number_histogram, CV_32S, 0.0);
    for (float number = 0; number < number_histogram; ++number){
        float tmp1 = number_histogram/2.0 -abs(number - number_histogram/2.0);
        float tmp2 = 1 + 0.4 * tmp1/number_histogram * 2.0;
        int tmp_start_row = start_row * tmp2;

        for (int row = img_bird_eye_binary_.rows - 1; row > tmp_start_row; row--){
            for (int tmp_col = number*histogram_width; tmp_col < (number +1)*histogram_width; ++tmp_col){
                if (img_bird_eye_binary_.at<uchar>(row, tmp_col) == 255)
                    histogram.at<int>(0, number) = histogram.at<int>(0, number) + 1;

            }//for tmp_col
        }//for row
    }
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
            center_points.push_back((i +0.5)*histogram_width);
        }//if
    }//for

    if(debug_flag_){
        std::cout<<center_points.size()<<" center points are found:  ";
        for(int i = 0; i < center_points.size(); ++i){
            std::cout<<center_points[i]<<"  ";
        }
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
                              const int stop_line_min_pixels,
                              const int stop_line_min_windows) {
    if (img_binary_stop_line_.empty()) {
        return -1;
    }//if

    stop_line_.line_factors.clear();
    stop_line_.detected_flag = false;
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
    int tmp_num_windows = 0;
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
            for (int col = win_x_low; col < win_x_high; col++){
                if (img_binary_stop_line_.at<uchar>(row, col) == 255){
                    window_inds_x.push_back(col);
                    window_inds_y.push_back(row);
                }
            }

        if (window_inds_x.size() > window_min_pixels){
            stop_line_inds_x.insert(stop_line_inds_x.end(), window_inds_x.begin(), window_inds_x.end());
            stop_line_inds_y.insert(stop_line_inds_y.end(), window_inds_y.begin(), window_inds_y.end());
            tmp_center = std::accumulate(window_inds_y.begin(), window_inds_y.end(), 0.0) / window_inds_y.size();
            tmp_num_windows++;
        }
    }
    vecan::Fit fit_state;

    if (stop_line_inds_x.size() > stop_line_min_pixels && tmp_num_windows >= stop_line_min_windows) {
        fit_state.polyfit(stop_line_inds_x, stop_line_inds_y, 1, true);
        fit_state.getFactor(stop_line_.line_factors);
        stop_line_.detected_flag = true;
    }
    if (debug_flag_) {
        cv::imshow("stop_line_windows", windows_img);
        cvWaitKey(1);
    }
    if (save_flag_) {
        cv::imwrite("src/image_lane_detector/result/stop_line_windows.bmp", windows_img);
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
    if (debug_flag_){
        img_windows_ = img_bird_eye_binary_.clone();
        cvtColor(img_windows_, img_windows_, cv::COLOR_GRAY2BGR);
    }//if
    std::vector<int> window_center = center_points;


    int window_height = floor(img_bird_eye_binary_.rows / number_windows);
    for (int line_index = 0; line_index < window_center.size(); line_index++) {
        std::vector<int> lane_lines_points_x;
        std::vector<int> lane_lines_points_y;
        int last_offset = 0;
        int tmp_last_window_flag = 0;
        int tmp_window_empty_flag = -1;
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
                cv::rectangle(img_windows_, cv::Point(window_x_low, window_y_low), cv::Point(window_x_high, window_y_high), cv::Scalar(0, 255, 0), 2);
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


            if (window_inds_x.size() > window_min_pixels && tmp_window_empty_flag < 10) {
                tmp_window_empty_flag = 0;
                double tmp_average = 0, tmp_variance = 0;
                Calculatevariance(window_inds_x, tmp_average, tmp_variance);
                //std::cout<<tmp_variance<<std::endl;
                if(tmp_variance < 4){
                    lane_lines_points_x.insert(lane_lines_points_x.end(), window_inds_x.begin(), window_inds_x.end());
                    lane_lines_points_y.insert(lane_lines_points_y.end(), window_inds_y.begin(), window_inds_y.end());

                    int tmp_center = std::accumulate(window_inds_x.begin(), window_inds_x.end(), 0.0) / window_inds_x.size();
                    if (tmp_last_window_flag) {
                        last_offset = tmp_center - window_center[line_index];
                    }
                    if(tmp_center * last_offset < 0){
                        window_center[line_index] = tmp_center;
                    }
                    else{
                        window_center[line_index] = tmp_center;
                    }

                    tmp_last_window_flag = 1;
                }
            }//if
            else {
                window_center[line_index] += last_offset;
                tmp_last_window_flag = 0;
                if(tmp_window_empty_flag != -1){
                    tmp_window_empty_flag++;
                }

            }
        }//for index_windows
        candidate_x.push_back(lane_lines_points_x);
        candidate_y.push_back(lane_lines_points_y);
    }//for line_index



}//function GetCandidateBySlidingWindows


int LaneDetector::GetLaneLines(const int lane_line_min_pixels){
    std::vector<int> line_center_points;
    GetLaneLineCenter(20, 100, 100, 450, line_center_points);

    std::vector< std::vector<int>> candidate_x;
    std::vector< std::vector<int>> candidate_y;
    GetCandidateBySlidingWindows(line_center_points,60,50,50, candidate_x, candidate_y);

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
    RansacCurve ransac_fitter;
    std::set<int> tmp_row_num;
    for (int line_index = 0; line_index < candidate_x.size(); ++line_index) {

        if (candidate_x[line_index].size() > lane_line_min_pixels) {
            tmp_row_num.clear();

            std::vector<double> tmp_factors;
            LaneLine tmp_line;
            tmp_line.score = (candidate_y[line_index].size() - 3000.0) / 3000.0;

            int yellow_points_num = 0;
            for (int point_index = 0; point_index < candidate_x[line_index].size(); ++point_index)
            {
                tmp_row_num.insert(candidate_y[line_index][point_index]);
                if (img_HSV.at<uchar>(candidate_y[line_index][point_index], candidate_x[line_index][point_index]) > 0)
                    yellow_points_num++;
            }
            if(tmp_row_num.size() > 70){
                if (yellow_points_num * 3 > candidate_x[line_index].size())
                    tmp_line.color = LaneLine::YELLOW;
                else
                    tmp_line.color = LaneLine::WHITE;

                GetLaneLineTypeAndRange(candidate_x[line_index], candidate_y[line_index], tmp_line);
                tmp_line.id = line_index;

                fitter.polyfit(candidate_y[line_index], candidate_x[line_index], 2, true);
                fitter.getFactor(tmp_factors);
                tmp_line.lines_factors = tmp_factors;

                //            tmp_line.ransac_lines_factors = ransac_fitter.GetBestModel(candidate_y[line_index], candidate_x[line_index]);
                //            tmp_line.lines_factors[0] = tmp_line.ransac_lines_factors[2];
                //            tmp_line.lines_factors[1] = tmp_line.ransac_lines_factors[1];
                //            tmp_line.lines_factors[2] = tmp_line.ransac_lines_factors[0];
                lane_lines_.push_back(tmp_line);

                if (debug_flag_) {
                    for(int row = 0; row < img_windows_.rows;++row){
                        //int tmp_col1 = tmp_line.ransac_lines_factors[0] * row *row + tmp_line.ransac_lines_factors[1] * row + tmp_line.ransac_lines_factors[2];
                        int tmp_col2 = tmp_line.lines_factors[2] * row *row + tmp_line.lines_factors[1] * row + tmp_line.lines_factors[0];
                        //if(tmp_col1 >= 0 && tmp_col1 < img_windows_.cols){
                        //img_windows_.at<cv::Vec3b>(row,tmp_col1) = cv::Vec3b(255,0,0);
                        //}
                        if(tmp_col2 >= 0 && tmp_col2 < img_windows_.cols){
                            img_windows_.at<cv::Vec3b>(row,tmp_col2) = cv::Vec3b(0,0,255);
                        }
                    }
                }//if
            }



        }//if
    }//for line_index

    if (debug_flag_) {
        cv::imshow("windows_img",img_windows_);
    }//if
    if (save_flag_) {
        cv::imwrite("src/image_lane_detector/result/lane_windows_img.bmp", img_windows_);
    }

}//GetLaneLines

int LaneDetector::GetLaneLineTypeAndRange(std::vector<int> candidate_x,
                                          std::vector<int> candidate_y,
                                          LaneLine &lane_line){
    if(candidate_y.empty()){
        std::cout<<"error in GetLaneLinesType: candidate_y is empty"<<std::endl;
        return false;
    }
    if(candidate_x.size() != candidate_y.size()){
        std::cout<<"error in GetLaneLinesType: "<<std::endl;
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

    if(row_record.size() * 1.5 > (candidata_y_max - candidata_y_min)){
        lane_line.type = LaneLine::SOLID;
    }
    else{
        lane_line.type = LaneLine::DASHED;
    }
    lane_line.start_row = candidata_y_max;
    lane_line.end_row = candidata_y_min;
    int tmp_end = 10000;
    int zero_num = 0;
    for(int tmp = candidata_y_max; tmp > candidata_y_min; --tmp){
        if(row_record.count(tmp) == 0){
            zero_num++;
        }
        else{
            zero_num = 0;
        }
        if(zero_num > 150)
        {
            tmp_end = tmp;
            lane_line.end_row = tmp + 100;
            break;
        }
        //std::cout<<zero_num<<std::endl;
    }

    if(tmp_end < 10000){
        std::vector<int>::iterator tmp_it_x = candidate_x.begin();
        std::vector<int>::iterator tmp_it_y = candidate_y.begin();
        for(; tmp_it_x != candidate_x.end() && tmp_it_y != candidate_y.end();){
            if(*(tmp_it_y) < tmp_end){
                candidate_x.erase(tmp_it_x);
                candidate_y.erase(tmp_it_y);
            }
            else{
                tmp_it_x++;
                tmp_it_y++;
            }
        }
    }

    if(stop_line_.detected_flag == true){
        std::vector<int>::iterator tmp_it_x = candidate_x.begin();
        std::vector<int>::iterator tmp_it_y = candidate_y.begin();
        for(; tmp_it_x != candidate_x.end() && tmp_it_y != candidate_y.end();){
            if((*tmp_it_y) <= stop_line_.line_factors[1] * (*tmp_it_x) + stop_line_.line_factors[0]){
                lane_line.end_row = (*tmp_it_y);
                break;
            }
            else{
                tmp_it_x++;
                tmp_it_y++;
            }
        }
        while(tmp_it_x != candidate_x.end() && tmp_it_y != candidate_y.end()){
            candidate_x.erase(tmp_it_x);
            candidate_y.erase(tmp_it_y);
        }
    }


    return true;
}//GetLaneLinesType

int LaneDetector::CalculateCurvature(const std::vector<double> factors,
                                     const int point_row,
                                     double &curvature){
    if (factors.size() != 3) {
        return false;
    }//if
    curvature = 2 * point_row*factors[2] * point_row + factors[1];
    return true;
}//CalculateCurvature

int LaneDetector::GetLane(const int min_lane_width,
                          const int max_lane_width){
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
        lanes_[lane_index].center = (pow(img_bird_eye_.rows, 2) * lanes_[lane_index].left_line.lines_factors[2] + img_bird_eye_.rows * lanes_[lane_index].left_line.lines_factors[1] + lanes_[lane_index].left_line.lines_factors[0] +
                pow(img_bird_eye_.rows, 2) * lanes_[lane_index].right_line.lines_factors[2] + img_bird_eye_.rows * lanes_[lane_index].right_line.lines_factors[1] + lanes_[lane_index].right_line.lines_factors[0]) / 2;

        kalman_filter_init(lanes_[lane_index]);
    }// for lane_index
    return true;
}// GetLane

int LaneDetector::TrackLane(const int min_lane_width,
                            const int max_lane_width) {
    if (lanes_.empty()) {
        for (int index = 0; index < tracking_lanes_.size(); ++index) {
            tracking_lanes_[index].tracking = false;
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
                tracking_lanes_[lane_index].id = last_lane_id--;
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
                tracking_lanes_[lane_index].id = last_lane_id--;
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
                    lanes_[lane_index].id = last_lane_id--;
                    tracking_lanes_.push_back(lanes_[lane_index]);
                }//if
                else if (-lanes_[lane_index].center +(*tracking_lanes_.begin()).center > min_lane_width &&
                         -lanes_[lane_index].center + (*tracking_lanes_.begin()).center < max_lane_width) {
                    lanes_[lane_index].score = 1;
                    lanes_[lane_index].id = last_lane_id--;;
                    tracking_lanes_.push_back(lanes_[lane_index]);
                }
            }

            else {
                lanes_[lane_index].tracking = true;
                lanes_[lane_index].score = cv::min((*it).score + 1, 6);
                lanes_[lane_index].id = (*it).id;

                (*it) = lanes_[lane_index];
                kalman_filter_update((*it), lanes_[lane_index].left_line.lines_factors,lanes_[lane_index].right_line.lines_factors);
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

    int tmp_min = 10000,tmp_min_index = -1;
    for(int index = 0; index < tracking_lanes_.size(); ++index) {
        if(abs(tracking_lanes_[index].center - vehicle_center_.x) < tmp_min){
            tmp_min =abs(vehicle_center_.x - tracking_lanes_[index].center);
            tmp_min_index = index;
        }
        if(stop_line_.detected_flag == true){
            tracking_lanes_[index].distanceToStop = (vehicle_center_.y - cv::max(tracking_lanes_[index].left_line.end_row, tracking_lanes_[index].right_line.end_row)) * pixel_to_ground_y_;
        }
        else{
            tracking_lanes_[index].distanceToStop = -1;
        }
    }
    if(tmp_min_index != -1){
        for(int index = 0; index < tracking_lanes_.size(); ++index) {
            tracking_lanes_[index].offsetIndex = index - tmp_min_index;
            tracking_lanes_[index].offset = (vehicle_center_.x - tracking_lanes_[index].center) * pixel_to_ground_x_;
            tracking_lanes_[index].dirDiff = atan(2 * img_bird_eye_.rows * tracking_lanes_[index].left_line.lines_factors[2] + tracking_lanes_[index].left_line.lines_factors[1]);
        }
    }


    if(debug_flag_){
        std::cout << tracking_lanes_.size() << " lanes detected;" << std::endl;
    }

    return true;
}//TrackLane

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
        if (tracking_lanes_[lane_index].score >= 3 && tracking_lanes_[lane_index].tracking) {
            if (drawID.count(tracking_lanes_[lane_index].left_line.id) == 0) {
                drawID.insert(tracking_lanes_[lane_index].left_line.id);
                LineShow(tracking_lanes_[lane_index].left_line);
            }//
            if (drawID.count(tracking_lanes_[lane_index].right_line.id) == 0) {
                drawID.insert(tracking_lanes_[lane_index].right_line.id);
                LineShow(tracking_lanes_[lane_index].right_line);
            }
        }
    }
    StopLineShow();
    cv::warpPerspective(img_display_, img_display_, inverse_perspective_matrix_, img_display_.size());
    cv::addWeighted(img_source_, 1, img_display_, 1, 0, img_display_);
    cv::imshow("result", img_display_);
    cvWaitKey(1);
    return true;
}//LaneLinesShow

int LaneDetector::LineShow(const LaneLine line){
    if (line.lines_factors.size() != 3) {
        std::cout<<"error in LineShow: the line factors are not good."<<std::endl;
        return false;
    }
    cv::Vec3b tmp_draw_color = cv::Vec3b(0, 0, 255);
    for (int row = img_display_.rows; row > line.end_row; row--){

        int col =  line.lines_factors[2] * row * row + line.lines_factors[1] * row + line.lines_factors[0];
        //        int tmp_flag = 1;
        //        if (stop_line_.line_factors.size() == 2) {
        //            if (row <  stop_line_.line_factors[1] * col + stop_line_.line_factors[0]) {
        //                tmp_flag = 0;
        //            }//if row
        //        }// if
        if (col >= 20 && col < img_display_.cols - 20){
            for (int tmp = col - 20; tmp < col + 20; tmp++){
                img_display_.at<cv::Vec3b>(row, tmp) = tmp_draw_color;
            }//for tmp
        }//if
    }//for row

}//LineShow

int LaneDetector::StopLineShow() {
    if (stop_line_.line_factors.size() != 2) {
        return false;
    }
    cv::Vec3b tmp_draw_color = cv::Vec3b(0, 0, 255);
    for (int col = 0; col < img_display_.cols; col++) {
        int row = stop_line_.line_factors[1] * col + stop_line_.line_factors[0];
        if (row >= 20 && row < img_display_.rows - 20) {
            for (int tmp = row - 20; tmp < row + 20; tmp++) {
                img_display_.at<cv::Vec3b>(tmp, col) = tmp_draw_color;
            }//for tmp
        }//if
    }//for row
    return true;

}//LineShow

int LaneDetector::PublishRoadMsg(local_messages::Road &road_msg){
    local_messages::Lane lane_msg;
    for(int index = 0; index < tracking_lanes_.size() ; ++index){
        if(tracking_lanes_[index].score > 3){
            lane_msg.id = tracking_lanes_[index].id;
            lane_msg.relation = local_messages::Lane::NEAR;
            lane_msg.preferred = false;
            lane_msg.offsetIndex = tracking_lanes_[index].offsetIndex;
            lane_msg.offset = tracking_lanes_[index].offset;
            lane_msg.dirDiff = tracking_lanes_[index].dirDiff;
            lane_msg.distanceToStop = tracking_lanes_[index].distanceToStop;
            lane_msg.startPointIndex = 0;
            if(index >0){
                lane_msg.leftLaneId = tracking_lanes_[index - 1].id;
            }
            if(index < tracking_lanes_.size() -1){
                lane_msg.rightLaneId = tracking_lanes_[index + 1].id;
            }
            if(tracking_lanes_[index].left_change_type == vecan::perception::Lane::PERMIT){
                lane_msg.canChangeLeft = true;
            }
            else{
                lane_msg.canChangeLeft = false;
            }
            if(tracking_lanes_[index].right_change_type == vecan::perception::Lane::PERMIT){
                lane_msg.canChangeRight = true;
            }
            else{
                lane_msg.canChangeRight = false;
            }

            geometry_msgs::Point32 tmp_point;
            int tmp_row = 0, tmp_col_left = 0, tmp_col_right = 0;
            float tmp_col = 0;
            int tmp_row_start = cv::min(tracking_lanes_[index].left_line.start_row,tracking_lanes_[index].right_line.start_row);
            int tmp_row_end = cv::max(tracking_lanes_[index].left_line.end_row,tracking_lanes_[index].right_line.end_row);
            for (tmp_row = tmp_row_start; tmp_row > tmp_row_end; tmp_row--){
                tmp_col_left = tracking_lanes_[index].left_line.lines_factors[2] * tmp_row * tmp_row + tracking_lanes_[index].left_line.lines_factors[1] * tmp_row +tracking_lanes_[index].left_line.lines_factors[0];
                tmp_col_right = tracking_lanes_[index].right_line.lines_factors[2] * tmp_row * tmp_row + tracking_lanes_[index].right_line.lines_factors[1] * tmp_row +tracking_lanes_[index].right_line.lines_factors[0];
                tmp_col = (tmp_col_left + tmp_col_right)/2.0;
                tmp_point.x = (tmp_col - vehicle_center_.x) * pixel_to_ground_x_;
                tmp_point.y = (tmp_row - vehicle_center_.y) * pixel_to_ground_y_;
                lane_msg.points.push_back(tmp_point);
            }
            road_msg.lanes.push_back(lane_msg);
        }

    }

}//PublishRoadMsg

int LaneDetector::kalman_filter_init(Lane &lane){
    /*------------------------------------------------------------------*/
    lane.kalman_filter_[0].init(3,3);
    lane.kalman_filter_[0].transitionMatrix = (cv::Mat_<float>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);  //转移矩阵A[1,1;0,1]
    cv::setIdentity(lane.kalman_filter_[0].measurementMatrix);                             //测量矩阵H
    cv::setIdentity(lane.kalman_filter_[0].processNoiseCov, cv::Scalar::all(1e-1));            //系统噪声方差矩阵Q
    cv::setIdentity(lane.kalman_filter_[0].measurementNoiseCov, cv::Scalar::all(1e-1));        //测量噪声方差矩阵R
    cv::setIdentity(lane.kalman_filter_[0].errorCovPost, cv::Scalar::all(1));                  //后验错误估计协方差矩阵P
    if(lane.left_line.lines_factors.size()!=3){
        std::cout<<"bad parameter size"<<std::endl;
        return false;
    }
    else{
        lane.kalman_filter_[0].statePost.at<float>(0,0) = lane.left_line.lines_factors[0];
        lane.kalman_filter_[0].statePost.at<float>(1,0) = lane.left_line.lines_factors[1];
        lane.kalman_filter_[0].statePost.at<float>(2,0) = lane.left_line.lines_factors[2];
    }

    /*------------------------------------------------------------------------------*/
    lane.kalman_filter_[1].init(3,3);
    lane.kalman_filter_[1].transitionMatrix = (cv::Mat_<float>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);  //转移矩阵A[1,1;0,1]
    cv::setIdentity(lane.kalman_filter_[1].measurementMatrix);                             //测量矩阵H
    cv::setIdentity(lane.kalman_filter_[1].processNoiseCov, cv::Scalar::all(1e-1));            //系统噪声方差矩阵Q
    cv::setIdentity(lane.kalman_filter_[1].measurementNoiseCov, cv::Scalar::all(1e-1));        //测量噪声方差矩阵R
    cv::setIdentity(lane.kalman_filter_[1].errorCovPost, cv::Scalar::all(1));                  //后验错误估计协方差矩阵P
    if(lane.right_line.lines_factors.size()!=3){
        std::cout<<"bad parameter size"<<std::endl;
        return false;
    }
    else{
        lane.kalman_filter_[1].statePost.at<float>(0,0) = lane.right_line.lines_factors[0];
        lane.kalman_filter_[1].statePost.at<float>(1,0) = lane.right_line.lines_factors[1];
        lane.kalman_filter_[1].statePost.at<float>(2,0) = lane.right_line.lines_factors[2];
    }

    return true;
}//kalman_filter_init

int LaneDetector::kalman_filter_update(Lane &lane, std::vector<double> left_paramters, std::vector<double> right_paramters){
    if(left_paramters.size() != 3 || right_paramters.size() != 3 || lane.left_line.lines_factors.size() != 3 || lane.right_line.lines_factors.size() != 3){
        return false;
    }
    lane.kalman_filter_[0].predict();
    cv::Mat left_measurement(3, 1, CV_32F);
    left_measurement.at<float>(0,0) = left_paramters[0];
    left_measurement.at<float>(1,0) = left_paramters[1];
    left_measurement.at<float>(2,0) = left_paramters[2];
    cv::Mat update_left_factors = lane.kalman_filter_[0].correct(left_measurement);

    lane.left_line.lines_factors[0] = update_left_factors.at<float>(0,0);
    lane.left_line.lines_factors[1] = update_left_factors.at<float>(1,0);
    lane.left_line.lines_factors[2] = update_left_factors.at<float>(2,0);

    lane.kalman_filter_[1].predict();
    cv::Mat right_measurement(3, 1, CV_32F);
    right_measurement.at<float>(0,0) = right_paramters[0];
    right_measurement.at<float>(1,0) = right_paramters[1];
    right_measurement.at<float>(2,0) = right_paramters[2];
    cv::Mat update_right_factors = lane.kalman_filter_[1].correct(right_measurement);

    lane.right_line.lines_factors[0] = update_right_factors.at<float>(0,0);
    lane.right_line.lines_factors[1] = update_right_factors.at<float>(1,0);
    lane.right_line.lines_factors[2] = update_right_factors.at<float>(2,0);

    return true;
}//kalman_filter_update

template<typename T> int LaneDetector::Calculatevariance(std::vector<T> data, double &average, double &variance){
    T tmp_sum = std::accumulate(data.begin(),data.end(),0);
    average = tmp_sum/data.size();
    T tmp_variance_sum = 0;
    for(int i =0; i < data.size(); ++i){
        tmp_variance_sum += (data[i] - average)*(data[i] - average);
    }
    variance = sqrt(tmp_variance_sum/data.size());
}//Calculatevariance

int LaneDetector::DeleteBigConnectedRegion(cv::Mat img_gray, cv::Mat &img_output, const int rectangle_num, const int max_area_threshold){
    img_output = img_gray.clone();

    cv::Mat img_tmp_area = img_gray.clone();
    std::vector<std::vector<cv::Point>> contours;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10));
    cv::dilate(img_tmp_area, img_tmp_area, element);
    cv::findContours(img_gray, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    cv::Scalar white_color = 255;

    for(int area_index = 0; area_index < contours.size(); ++area_index){
        double tmp_area = cv::contourArea(contours[area_index]);
        if(tmp_area > max_area_threshold){
            cv::drawContours(img_tmp_area, contours, area_index, white_color, CV_FILLED);
        }
    }

    cv::Mat erode_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(60, 10));
    cv::Mat dilate_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(60, 20));

    cv::erode(img_tmp_area, img_tmp_area, erode_element);
    cv::dilate(img_tmp_area, img_tmp_area, dilate_element);
    //cv::morphologyEx(img_tmp_area, img_tmp_area, cv::MORPH_OPEN, element);


    //    int tmp_image_height = img_gray.rows/rectangle_num;
    //    for(int rectangle_index = 0; rectangle_index < rectangle_num; ++rectangle_index){
    //        cv::Rect tmp_rect(0, (rectangle_index) * tmp_image_height, img_open.cols, tmp_image_height);
    //        cv::Mat tmp_roi_img;
    //        img_open(tmp_rect).copyTo(tmp_roi_img);
    //        std::vector<std::vector<cv::Point>> contours;
    //        cv::findContours(tmp_roi_img, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    //        cv::Scalar black_color = 0;
    //        for(int area_index = 0; area_index < contours.size(); ++area_index){
    //            double tmp_area = cv::contourArea(contours[area_index]);
    //            if(tmp_area > max_area_threshold){
    //                cv::drawContours(tmp_roi_img, contours, area_index, black_color, CV_FILLED);
    //            }
    //        }
    //        tmp_roi_img.copyTo(img_output(tmp_rect));
    //    }

    cv::bitwise_not(img_tmp_area,img_output);//逻辑非，求补集
    cv::bitwise_and(img_output,img_gray,img_output);//逻辑与，求交集

    return true;
}//DeleteBigConnectedRegion

} //namespace perception
} //namespace vecan


