#include "myslam/config.h"

namespace myslam {

Config::~Config() {
  if(config_->file_.isOpened()) {
    config_->file_.release();
  }
}

void Config::SetParameterFile(const std::string& filename) {
  if(config_ == nullptr) {
    config_.reset(new Config());
  }
  config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
  if(config_->file_.isOpened() == false) {
    std::cerr << "parameter file " << filename << " doesn't exit\n";
    config_->file_.release();
    return;
  }
}

} // namespace myslam
