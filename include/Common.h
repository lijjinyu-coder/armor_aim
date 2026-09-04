#ifndef ARMOR_AIM_COMMON_H
#define ARMOR_AIM_COMMON_H

#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

// 目标状态: 0 未发现, 1 检测到, 2 跟踪中, 3 短暂丢失
enum class TargetState : uint8_t {
    NO_TARGET = 0,
    DETECTED  = 1,
    TRACKING  = 2,
    LOST      = 3,
};

// 状态转成字符串, 用来显示和写日志
inline const char* stateToString(TargetState s) {
    switch (s) {
        case TargetState::NO_TARGET: return "NO_TARGET";
        case TargetState::DETECTED:  return "DETECTED";
        case TargetState::TRACKING:  return "TRACKING";
        case TargetState::LOST:      return "LOST";
    }
    return "UNKNOWN";
}

// 一根灯条
struct LightBar {
    cv::RotatedRect rect;   // 最小外接矩形(宽 <= 高)
    cv::Point2f     center; // 中心点
    float           angle;  // 长边和竖直方向的夹角(度)
    float           length; // 长边长度
    float           width;  // 短边长度
    float           area;   // 面积
};

// 一块装甲板候选(四个角点顺序: 左上、右上、右下、左下)
struct Armor {
    std::vector<cv::Point2f> four_points;
    cv::Point2f              center;
    float                    width;
    float                    height;
    float                    aspect_ratio;
    LightBar                 left_light;
    LightBar                 right_light;
};

// 位姿解算结果
struct Pose {
    double yaw_deg     = 0.0;     // 偏航角(度)
    double pitch_deg   = 0.0;     // 俯仰角(度)
    double distance_mm = 0.0;     // 距离(毫米)
    bool   valid       = false;   // 是否解算成功
};

// 一帧的视觉输出数据(打包成报文用)
struct VisionData {
    float       yaw_deg     = 0.f;
    float       pitch_deg   = 0.f;
    float       distance_mm = 0.f;
    TargetState state       = TargetState::NO_TARGET;
};

#endif // ARMOR_AIM_COMMON_H
