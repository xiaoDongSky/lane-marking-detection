#include "ransac.h"

namespace vecan {
namespace perception {
RansacCurve::RansacCurve() {
        max_iterations_ = 100;
        min_inliers_distance_ = 10;
        required_inliers_rate_ = 0.9;
}//RansacCurve

RansacCurve::RansacCurve(const int required_inliers_rate, const int max_iterations, const double min_inliers_distance) {
        max_iterations_ = max_iterations;
        min_inliers_distance_ = min_inliers_distance;
        required_inliers_rate_ = required_inliers_rate;
}//RansacCurve

int RansacCurve::get_random(int min, int max) {
        return min + (int)(rand()*(max - min + 1.0) / (1.0 + RAND_MAX));
}

std::vector<double> RansacCurve::GetBestModel(const std::vector<int>& points_x, const std::vector<int>& points_y) {
        if (points_x.size() == 0 || points_x.size() != points_y.size()) {
                std::cout << "error: bad points" << std::endl;
        }
        int max_inliers_num =0;
        int min_required_inliers = required_inliers_rate_*points_x.size();
        cv::Mat left_side, right_side, solution;
        double best_a;
        double best_b;
        double best_c;
        std::vector<int> best_inliers_x;
        std::vector<int> best_inliers_y;

        for (int i = 0; i < max_iterations_; i++) {
                inliers_x_.clear();
                inliers_y_.clear();
                int num_data = points_x.size();

                int x1 = get_random(0, num_data/3.0 - 1);
                int x2 = get_random(0, num_data/3.0  - 1) + num_data/3.0;
                int x3 = get_random(0, num_data/3.0  - 1) + num_data/3.0 * 2.0;

                left_side = (cv::Mat_<double>(3, 3)
                        <<
                        points_x[x1] * points_x[x1], points_x[x1], 1,
                        points_x[x2] * points_x[x2], points_x[x2], 1,
                        points_x[x3] * points_x[x3], points_x[x3], 1);

                right_side = (cv::Mat_<double>(3, 1)
                        <<
                        points_y[x1],
                        points_y[x2],
                        points_y[x3]);

                solution = cv::Mat(3, 1, CV_64FC1);
                cv::solve(left_side, right_side, solution);

                double a = solution.at<double>(0, 0);
                double b = solution.at<double>(1, 0);
                double c = solution.at<double>(2, 0);


                for (int j = 0; j < num_data; j++) {
                        double value = a * points_x[j] * points_x[j] + b * points_x[j] + c;
                        if (points_y[j] <= (value + min_inliers_distance_) && points_y[j] >= (value - min_inliers_distance_)) {
                                inliers_x_.push_back(points_x[j]);
                                inliers_y_.push_back(points_y[j]);
                        }
                }
                int num_inliers = inliers_x_.size();
                if (num_inliers > max_inliers_num) {
                        max_inliers_num = num_inliers;
                        best_a = a;
                        best_b = b;
                        best_c = c;
                        best_inliers_x = inliers_x_;
                        best_inliers_y = inliers_y_;
                }
                if(num_inliers > min_required_inliers){
                    inliers_x_.clear();
                    inliers_y_.clear();
                    break;
                }
                inliers_x_.clear();
                inliers_y_.clear();
        }
        inliers_x_ = best_inliers_x;
        inliers_y_ = best_inliers_y;
        best_parameters_.clear();
        best_parameters_.push_back(best_a);
        best_parameters_.push_back(best_b);
        best_parameters_.push_back(best_c);

        return best_parameters_;
}//GetBestModel


} //namespace perception
} //namespace vecan
