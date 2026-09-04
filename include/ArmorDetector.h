#ifndef ARMOR_AIM_ARMOR_DETECTOR_H
#define ARMOR_AIM_ARMOR_DETECTOR_H

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "Common.h"

// 检测装甲板: 预处理 -> 找灯条 -> 灯条两两匹配成装甲板
class ArmorDetector {
public:
    struct Params {
        std::string enemy_color      = "BLUE";
        std::string preprocess_mode  = "HSV"; // HSV / BGR / GRAY
        int h_low = 0, s_low = 0, v_low = 0;
        int h_high = 0, s_high = 0, v_high = 0; // h_low > h_high 表示红色环绕
        int gray_thresh     = 120;
        int bgr_diff_thresh = 40;

        double lightbar_min_area  = 50.0;  // 灯条最小面积
        double lightbar_min_ratio = 1.8;   // 灯条长边 / 短边
        double lightbar_max_angle = 45.0;  // 灯条最大倾斜角(度)

        double armor_min_ratio          = 1.8;  // 装甲板宽高比范围
        double armor_max_ratio          = 5.0;
        double armor_angle_sum_max      = 30.0; // 左右灯条角度和(内八判定)
        double armor_length_diff_ratio  = 0.35; // 两灯条长度差比例
        double armor_y_offset_ratio     = 0.6;  // 两灯条纵向错位比例
    };

    void setParams(const Params& p) { params_ = p; }

    std::vector<Armor> detect(const cv::Mat& frame);

    std::vector<LightBar> last_bars; // 调试用: 上一帧的灯条

private:
    cv::Mat preprocess(const cv::Mat& frame) const;
    std::vector<LightBar> extractLightBars(const cv::Mat& binary) const;
    std::vector<Armor>    matchArmors(const std::vector<LightBar>& bars) const;

    Params params_;
};

#endif // ARMOR_AIM_ARMOR_DETECTOR_H
