#include "TargetSelector.h"

#include <algorithm>

// 目标保持规则:
//   上一帧有目标时, 优先选离它最近的候选, 防止目标跳来跳去;
//   这一帧没检测到不马上清状态, 先算"短暂丢失", 丢够帧数才清。
TargetSelector::TargetSelector(int lost_threshold) {
    if (lost_threshold > 0) lost_threshold_ = lost_threshold;
}

const Armor* TargetSelector::select(const std::vector<Armor>& armors,
                                    int frame_w, int frame_h) {
    const Armor* chosen = nullptr;

    if (!armors.empty()) {
        // 参考点: 正在跟踪/丢失时用上一帧目标中心, 否则用画面中心
        cv::Point2f ref = last_center_;
        if (state_ != TargetState::TRACKING && state_ != TargetState::LOST) {
            ref = cv::Point2f(frame_w * 0.5f, frame_h * 0.5f);
        }
        double max_dist = cv::norm(cv::Point2f(frame_w, frame_h));

        // 打分 = 面积归一化 - 距离归一化 * 0.3, 分最高的当选
        float max_area = 0.f;
        for (const auto& a : armors) {
            max_area = std::max(max_area, a.width * a.height);
        }
        if (max_area <= 0.f) max_area = 1.f;

        size_t best_idx = 0;
        float best_score = -1e9f;
        for (size_t i = 0; i < armors.size(); ++i) {
            float area = armors[i].width * armors[i].height;
            float dist = static_cast<float>(cv::norm(armors[i].center - ref));
            float score = (area / max_area) - 0.3f * (dist / static_cast<float>(max_dist));
            if (score > best_score) {
                best_score = score;
                best_idx = i;
            }
        }
        chosen = &armors[best_idx];
    }

    // 更新状态
    if (chosen != nullptr) {
        lost_count_ = 0;
        switch (state_) {
            case TargetState::NO_TARGET: state_ = TargetState::DETECTED;  break; // 第一次发现
            case TargetState::DETECTED:  state_ = TargetState::TRACKING;  break; // 连续发现 -> 跟踪
            case TargetState::LOST:      state_ = TargetState::TRACKING;  break; // 又找到了
            case TargetState::TRACKING:  break; // 保持跟踪
        }
        last_center_ = chosen->center;
    } else {
        ++lost_count_;
        switch (state_) {
            case TargetState::NO_TARGET:
                break; // 本来就是没目标
            case TargetState::DETECTED:
            case TargetState::TRACKING:
                state_ = TargetState::LOST; // 暂时丢失, 不清状态
                break;
            case TargetState::LOST:
                if (lost_count_ >= lost_threshold_) state_ = TargetState::NO_TARGET;
                break;
        }
    }
    return chosen;
}
