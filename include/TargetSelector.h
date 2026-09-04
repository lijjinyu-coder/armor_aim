#ifndef ARMOR_AIM_TARGET_SELECTOR_H
#define ARMOR_AIM_TARGET_SELECTOR_H

#include <vector>

#include <opencv2/core.hpp>

#include "Common.h"

// 目标选择: 从多个候选里挑一个, 顺便维护目标状态
// 状态变化:
//   NO_TARGET --发现--> DETECTED --连续发现--> TRACKING
//   TRACKING --丢失--> LOST --丢太久--> NO_TARGET
//   LOST --重新发现--> TRACKING
class TargetSelector {
public:
    explicit TargetSelector(int lost_threshold = 20);

    const Armor* select(const std::vector<Armor>& armors, int frame_w, int frame_h);

    TargetState state() const { return state_; }
    int lostCount() const { return lost_count_; }

private:
    TargetState state_          = TargetState::NO_TARGET;
    int         lost_count_     = 0;
    int         lost_threshold_ = 20;
    cv::Point2f last_center_; // 上一帧目标中心
};

#endif // ARMOR_AIM_TARGET_SELECTOR_H
