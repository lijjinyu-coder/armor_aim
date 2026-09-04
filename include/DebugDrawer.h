#ifndef ARMOR_AIM_DEBUG_DRAWER_H
#define ARMOR_AIM_DEBUG_DRAWER_H

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "Common.h"

// 画调试信息: 候选框、目标框、文字
class DebugDrawer {
public:
    void drawFrame(cv::Mat& frame,
                   const std::vector<Armor>& armors,
                   const Armor* target,
                   const Pose& pose,
                   double fps,
                   TargetState state);
};

#endif // ARMOR_AIM_DEBUG_DRAWER_H
