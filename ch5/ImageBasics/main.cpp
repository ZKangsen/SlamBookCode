#include <iostream>
#include <chrono>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

int main(int argc, char** argv) {
    // init log
    google::InitGoogleLogging(argv[0]);
    google::LogToStderr();

    // read image from argv[1]
    cv::Mat image;
    image = cv::imread(argv[1]);
    if(image.data == nullptr) {
        LOG(ERROR) << "picture not exit: "  << argv[1];
        return 0;
    }
    LOG(INFO) << "\nimage width = " << image.cols << "\n"
              << "image hight = " << image.rows << "\n"
              << "image channels = " << image.channels();
    cv::imshow("origin_image", image);
    cv::waitKey(0);

    if(image.type() != CV_8UC1 && image.type() != CV_8UC3) {
        LOG(ERROR) << "please input a color or grey-scale image";
        return 0;
    }
    // traverse image data
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for(size_t y = 0; y < image.rows; ++y) {
        for(size_t x = 0; x < image.cols; ++x) {
            unsigned char* row_ptr = image.ptr<unsigned char>(y);
            unsigned char* data_ptr = &row_ptr[x * image.channels()];
            for(size_t i = 0; i < image.channels(); ++i) {
                unsigned char data = data_ptr[i];
            }
        }
    }
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used =
        chrono::duration_cast<chrono::duration<double> >(t2 - t1);
    LOG(INFO) << "time used = " << time_used.count() << "s";

    // cv::Mat copy
    cv::Mat image_another = image;      // not copy, image_another is the ref of image
    image_another(cv::Rect(0, 0, 100, 100)).setTo(0);
    cv::imshow("image_1", image);
    cv::waitKey(0);

    // real copy use “clone()” function
    cv::Mat image_clone = image.clone();
    image_clone(cv::Rect(0, 0, 100, 100)).setTo(255);
    cv::imshow("image_2", image);
    cv::imshow("image clone", image_clone);
    cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}