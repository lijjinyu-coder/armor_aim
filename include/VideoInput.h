#ifndef ARMOR_AIM_VIDEO_INPUT_H
#define ARMOR_AIM_VIDEO_INPUT_H

#include <string>

#include <opencv2/videoio.hpp>

// 视频输入: 打开视频文件, 一帧一帧读
class VideoInput {
public:
    bool open(const std::string& path);
    bool read(cv::Mat& frame);

    double fps() const { return fps_; }

private:
    cv::VideoCapture cap_;
    double fps_ = 30.0;
};

#endif // ARMOR_AIM_VIDEO_INPUT_H
