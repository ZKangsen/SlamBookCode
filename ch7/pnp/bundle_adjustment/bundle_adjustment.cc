#include "bundle_adjustment.h"
#include <Eigen/Core>
#include <glog/logging.h>
#include <chrono>
#include <g2o/core/linear_solver.h>
#include <g2o/core/block_solver.h>
#include <g2o/types/sba/types_sba.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

void BundleAdjustment(const std::vector<cv::Point3f>& pts_3d,
                      const std::vector<cv::Point2f>& pts_2d,
                      const cv::Mat& K,
                      const cv::Mat& R, const cv::Mat& t) {
  // pose is 6 dim, landmark(points) is 3 dim
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block;
  Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
  Block* solver_ptr = new Block(std::unique_ptr<Block::LinearSolverType>(linearSolver));
  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<Block>(solver_ptr));
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  LOG(INFO) << "g2o init successfully";

  // add vertex(pose, points)
  g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
  Eigen::Matrix3d R_mat;
  R_mat << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
           R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
           R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
  pose->setId(0);
  pose->setEstimate(g2o::SE3Quat(R_mat,
                                 Eigen::Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0))));
  optimizer.addVertex(pose);

  int index = 1;
  for(const auto& p : pts_3d) {      // landmark(points)
    g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
    point->setId(index++);
    point->setEstimate(Eigen::Vector3d(p.x, p.y, p.z));
    point->setMarginalized(true);
    optimizer.addVertex(point);
  }

  // add camera parameters(intrinsic K)
  g2o::CameraParameters* cam_paras = new g2o::CameraParameters(K.at<double>(0, 0),
      Eigen::Vector2d(K.at<double>(0, 2), K.at<double>(1, 2)), 0.0);
  cam_paras->setId(0);
  optimizer.addParameter(cam_paras);

  // add edges(residual)
  index = 1;
  for(const auto& p : pts_2d) {
    g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
    edge->setId(index);
    edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(index)));
    edge->setVertex(1, pose);
    edge->setMeasurement(Eigen::Vector2d(p.x, p.y));
    edge->setParameterId(0, 0);
    edge->setInformation(Eigen::Matrix2d::Identity());
    optimizer.addEdge(edge);
    index++;
  }

  // time count
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  optimizer.setVerbose(true);
  optimizer.initializeOptimization();
  optimizer.optimize(100);
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  LOG(INFO) << "time used = " << time_used.count();

  LOG(INFO) << "after optimized: \n";
  LOG(INFO) << "T = \n" << Eigen::Isometry3d(pose->estimate()).matrix(); // Eigen::Isometry3d is 4x4 欧式变换矩阵
}