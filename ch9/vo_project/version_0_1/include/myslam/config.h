#ifndef MYSLAM_CONFIG_H_
#define MYSLAM_CONFIG_H_

#include "common_include.h"

namespace myslam {

class Config {
  public:
    ~Config();
    static void SetParameterFile(const std::string& filename);

    template<typename T>
    static T Get(const std::string& key) {
      return T(config_->file_[key]);
    }

  private:
    Config() {}
    cv::FileStorage file_;
    static std::shared_ptr<Config> config_;

}; // class config

std::shared_ptr<Config> Config::config_ = nullptr;

} // namespace myslam

#endif // MYSLAM_CONFIG_H_
