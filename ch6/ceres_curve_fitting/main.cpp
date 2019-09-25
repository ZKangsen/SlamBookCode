#include <iostream>
#include <vector>
#include <chrono>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>

using namespace std;

// fitting y = exp(ax^2 + bx + c) + res
struct CURVE_FITTING_COST {
    CURVE_FITTING_COST(double x, double y) : x_(x), y_(y) {}

    // calculate residual
    template <typename T>
    bool operator() (const T* abc, T* residual) const {
      residual[0] = T(y_) - ceres::exp(abc[0] * T(x_) * T(x_) + abc[1] * T(x_) + abc[2]);
      return true;
    }

    const double x_;
    const double y_;
};

int main(int argc, char ** argv) {
  // init log
  google::InitGoogleLogging(argv[0]);
  google::LogToStderr();

  // init curve data
  double a = 1.0;
  double b = 2.0;
  double c = 1.0;
  int point_num = 100;          // data num
  double w_sigma = 1.0;
  cv::RNG rng;
  double abc[3] = {0, 0, 0};    // save result
  std::vector<double> x_data, y_data;

  // generate data
  LOG(INFO) << "generate curve data: ";
  for(int i = 0; i < point_num; ++i) {
    double x = i / 100.0;
    x_data.emplace_back(x);
    double y = exp(a * x * x + b * x + c) + rng.gaussian(w_sigma);
    y_data.emplace_back(y);
    LOG(INFO) << "x = " << x << " y = " << y;
  }

  // define problem
  ceres::Problem problem;
  for(int i = 0; i < point_num; ++i) {
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
            new CURVE_FITTING_COST(x_data[i], y_data[i])), nullptr, abc);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  ceres::Solve(options, &problem, &summary);
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double> >(t2 - t1);
  LOG(INFO) << "solve time cost = " << time_used.count();

  // output result
  LOG(INFO) << summary.BriefReport();
  LOG(INFO) << "estimate abc = ";
  for(auto data : abc) {
    LOG(INFO) << data << " ";
  }

  return 0;
}