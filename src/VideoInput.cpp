#include "VideoInput.h"

#include <iostream>

bool VideoInput::open(const std::string& path) {
    cap_.open(path);
    if (!cap_.isOpened()) {
        std::cerr << "打不开视频: " << path << std::endl;
        return false;
    }
    double fps = cap_.get(cv::CAP_PROP_FPS);
    if (fps > 0) fps_ = fps;
    std::cout << "视频已打开: " << path
              << " (" << cap_.get(cv::CAP_PROP_FRAME_WIDTH) << "x"
              << cap_.get(cv::CAP_PROP_FRAME_HEIGHT) << ", " << fps_ << " fps)"
              << std::endl;
    return true;
}

bool VideoInput::read(cv::Mat& frame) {
    return cap_.read(frame);
}
