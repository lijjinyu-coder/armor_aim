#include "DebugDrawer.h"

#include <iomanip>
#include <sstream>

#include <opencv2/imgproc.hpp>

// 保留两位小数
static std::string fmtDouble(double v) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << v;
    return oss.str();
}

// OpenCV 5 的 polylines 只收整数点, 先把浮点角点转成整数
static std::vector<cv::Point> toIntPts(const std::vector<cv::Point2f>& pts) {
    std::vector<cv::Point> out;
    out.reserve(pts.size());
    for (const auto& p : pts) out.emplace_back(cvRound(p.x), cvRound(p.y));
    return out;
}

void DebugDrawer::drawFrame(cv::Mat& frame,
                            const std::vector<Armor>& armors,
                            const Armor* target,
                            const Pose& pose,
                            double fps,
                            TargetState state) {
    // 1. 候选装甲板画绿框
    for (const auto& a : armors) {
        cv::polylines(frame, toIntPts(a.four_points), true, cv::Scalar(0, 200, 0), 2, cv::LINE_AA);
    }

    // 2. 选中的目标画红框 + 中心十字
    if (target != nullptr) {
        cv::polylines(frame, toIntPts(target->four_points), true, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
        cv::line(frame,
                 cv::Point(target->center.x - 10, target->center.y),
                 cv::Point(target->center.x + 10, target->center.y),
                 cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
        cv::line(frame,
                 cv::Point(target->center.x, target->center.y - 10),
                 cv::Point(target->center.x, target->center.y + 10),
                 cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    }

    // 3. 左上角显示 FPS / 状态 / 角度 / 距离
    std::vector<std::string> lines;
    lines.push_back("FPS: " + fmtDouble(fps));
    lines.push_back(std::string("State: ") + stateToString(state));
    if (target != nullptr && pose.valid) {
        lines.push_back("yaw:   " + fmtDouble(pose.yaw_deg) + " deg");
        lines.push_back("pitch: " + fmtDouble(pose.pitch_deg) + " deg");
        lines.push_back("dist:  " + fmtDouble(pose.distance_mm) + " mm");
    } else {
        lines.push_back("No target");
    }

    // 先画一块黑底, 再写白字
    const int x0 = 10, y0 = 20, dh = 22, w = 320;
    cv::rectangle(frame, cv::Rect(x0 - 5, y0 - 18, w, static_cast<int>(lines.size()) * dh + 6),
                  cv::Scalar(0, 0, 0), cv::FILLED);
    for (size_t i = 0; i < lines.size(); ++i) {
        cv::putText(frame, lines[i], cv::Point(x0, y0 + static_cast<int>(i) * dh),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255),
                    1, cv::LINE_AA);
    }
}
