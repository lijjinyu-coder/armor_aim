#ifndef ARMOR_AIM_CONFIG_LOADER_H
#define ARMOR_AIM_CONFIG_LOADER_H

#include <string>

// 所有配置项
// 约定: 数值为 0 或字符串为空表示"没配置", 加载时会用内置默认值
struct AppConfig {
    // 视频输入
    std::string video_path;                    // 测试视频路径
    bool        save_video      = false;       // 是否保存结果视频
    std::string save_video_path;               // 结果视频路径(空 -> output/output.avi)
    bool        show_debug      = true;        // 是否显示窗口

    // 预处理
    std::string enemy_color     = "BLUE";
    std::string preprocess_mode = "HSV";       // HSV / BGR / GRAY
    int h_low = 0, s_low = 0, v_low = 0;
    int h_high = 0, s_high = 0, v_high = 0;    // 全 0 = 没配置
    int gray_thresh     = 120;
    int bgr_diff_thresh = 40;

    // 灯条筛选
    double lightbar_min_area  = 0; // 0 -> 默认 50
    double lightbar_min_ratio = 0; // 0 -> 默认 1.8
    double lightbar_max_angle = 0; // 0 -> 默认 45

    // 装甲板匹配
    double armor_min_ratio          = 0; // 0 -> 默认 1.8
    double armor_max_ratio          = 0; // 0 -> 默认 5.0
    double armor_angle_sum_max      = 0; // 0 -> 默认 30
    double armor_length_diff_ratio  = 0; // 0 -> 默认 0.35
    double armor_y_offset_ratio     = 0; // 0 -> 默认 0.6

    // 目标选择
    int lost_threshold = 0; // 0 -> 默认 20

    // 装甲板实际尺寸(mm)
    double armor_width_mm  = 0; // 0 -> 默认 130
    double armor_height_mm = 0; // 0 -> 默认 55
};

class ConfigLoader {
public:
    bool load(const std::string& path, AppConfig& cfg) const;

private:
    void applyDefaults(AppConfig& cfg) const;
};

#endif // ARMOR_AIM_CONFIG_LOADER_H
