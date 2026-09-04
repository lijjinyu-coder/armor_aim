#include "ArmorDetector.h"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>
#if CV_VERSION_MAJOR >= 5
#include <opencv2/geometry/2d.hpp> // OpenCV 5 里 contourArea / minAreaRect 在这个头文件
#endif

// 把矩形转成"宽 <= 高"的形式, angle 表示长边和竖直方向的夹角
static void normalizeRotatedRect(cv::RotatedRect& rect) {
    if (rect.size.width > rect.size.height) {
        rect.size = cv::Size2f(rect.size.height, rect.size.width);
        rect.angle += 90.f;
    }
    if (rect.angle >= 90.f) rect.angle -= 180.f;
    if (rect.angle < -90.f) rect.angle += 180.f;
}

static bool cmpX(const cv::Point2f& a, const cv::Point2f& b) { return a.x < b.x; }
static bool cmpY(const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; }

std::vector<Armor> ArmorDetector::detect(const cv::Mat& frame) {
    std::vector<Armor> armors;
    if (frame.empty()) return armors;

    cv::Mat binary = preprocess(frame);                    // 变成二值图
    std::vector<LightBar> bars = extractLightBars(binary); // 找出灯条
    last_bars = bars;                                      // 存下来给调试用
    armors = matchArmors(bars);                            // 灯条两两配成装甲板
    return armors;
}

cv::Mat ArmorDetector::preprocess(const cv::Mat& frame) const {
    cv::Mat binary;

    if (params_.preprocess_mode == "BGR") {
        // 用通道差分离出敌方颜色
        std::vector<cv::Mat> ch;
        cv::split(frame, ch);
        cv::Mat diff, other;
        if (params_.enemy_color == "RED") {
            cv::max(ch[0], ch[1], other);      // max(B, G)
            cv::subtract(ch[2], other, diff);  // R - max(B, G)
        } else {
            cv::max(ch[2], ch[1], other);      // max(R, G)
            cv::subtract(ch[0], other, diff);  // B - max(R, G)
        }
        cv::threshold(diff, binary, params_.bgr_diff_thresh, 255, cv::THRESH_BINARY);
    } else if (params_.preprocess_mode == "GRAY") {
        // 灯条比较亮, 直接灰度阈值
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, binary, params_.gray_thresh, 255, cv::THRESH_BINARY);
    } else {
        // 默认 HSV 阈值
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::Scalar low(params_.h_low, params_.s_low, params_.v_low);
        cv::Scalar high(params_.h_high, params_.s_high, params_.v_high);
        if (params_.h_low <= params_.h_high) {
            cv::inRange(hsv, low, high, binary);
        } else {
            // 红色在色相环两头, 拆成两段: [h_low, 180] 和 [0, h_high]
            cv::Mat b1, b2;
            cv::inRange(hsv, cv::Scalar(params_.h_low, params_.s_low, params_.v_low),
                        cv::Scalar(180, params_.s_high, params_.v_high), b1);
            cv::inRange(hsv, cv::Scalar(0, params_.s_low, params_.v_low),
                        cv::Scalar(params_.h_high, params_.s_high, params_.v_high), b2);
            cv::bitwise_or(b1, b2, binary);
        }
        // 开运算去噪, 闭运算补洞
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
    }
    return binary;
}

std::vector<LightBar> ArmorDetector::extractLightBars(const cv::Mat& binary) const {
    std::vector<LightBar> bars;

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < params_.lightbar_min_area) continue; // 太小的不要

        cv::RotatedRect rect = cv::minAreaRect(contour);
        normalizeRotatedRect(rect);

        if (rect.size.width <= 1e-3f) continue;
        double ratio = rect.size.height / rect.size.width; // 长边 / 短边
        if (ratio < params_.lightbar_min_ratio) continue;  // 太胖的不是灯条

        if (std::abs(rect.angle) > params_.lightbar_max_angle) continue; // 歪太多不要

        LightBar bar;
        bar.rect   = rect;
        bar.center = rect.center;
        bar.angle  = rect.angle;
        bar.length = rect.size.height;
        bar.width  = rect.size.width;
        bar.area   = static_cast<float>(area);
        bars.push_back(bar);
    }

    // 按 x 从小到大排, 保证后面左灯条的 x 一定小于右灯条
    std::sort(bars.begin(), bars.end(),
              [](const LightBar& a, const LightBar& b) { return a.center.x < b.center.x; });
    return bars;
}

std::vector<Armor> ArmorDetector::matchArmors(const std::vector<LightBar>& bars) const {
    std::vector<Armor> armors;

    for (size_t i = 0; i < bars.size(); ++i) {
        for (size_t j = i + 1; j < bars.size(); ++j) {
            const LightBar& l = bars[i]; // 左灯条
            const LightBar& r = bars[j]; // 右灯条

            float avg_h = (l.length + r.length) * 0.5f;
            if (avg_h <= 0.f) continue;

            // 两个灯条要差不多长
            if (std::abs(l.length - r.length) / avg_h > params_.armor_length_diff_ratio) continue;
            // 上下不能错位太多
            if (std::abs(l.center.y - r.center.y) > avg_h * params_.armor_y_offset_ratio) continue;
            // 左右灯条要"内八", 角度和接近 0
            if (std::abs(l.angle + r.angle) > params_.armor_angle_sum_max) continue;

            // 宽高比要在范围内: 宽度(两灯条中心距) / 高度(平均灯条长)
            float dist   = static_cast<float>(cv::norm(l.center - r.center));
            float aspect = dist / avg_h;
            if (aspect < params_.armor_min_ratio || aspect > params_.armor_max_ratio) continue;

            // 算四个角点, 顺序: 左上、右上、右下、左下(solvePnP 要用)
            cv::Point2f pts[4];
            l.rect.points(pts);
            std::vector<cv::Point2f> lp(pts, pts + 4);
            r.rect.points(pts);
            std::vector<cv::Point2f> rp(pts, pts + 4);

            std::sort(lp.begin(), lp.end(), cmpX);
            std::sort(rp.begin(), rp.end(), cmpX);
            // 左灯条取最右两个点, 右灯条取最左两个点(靠中间的那侧)
            std::vector<cv::Point2f> il(lp.end() - 2, lp.end());
            std::vector<cv::Point2f> ir(rp.begin(), rp.begin() + 2);
            std::sort(il.begin(), il.end(), cmpY);
            std::sort(ir.begin(), ir.end(), cmpY);

            Armor armor;
            armor.four_points  = {il[0], ir[0], ir[1], il[1]};
            armor.center       = (l.center + r.center) * 0.5f;
            armor.width        = dist;
            armor.height       = avg_h;
            armor.aspect_ratio = aspect;
            armor.left_light   = l;
            armor.right_light  = r;
            armors.push_back(armor);
        }
    }
    return armors;
}
