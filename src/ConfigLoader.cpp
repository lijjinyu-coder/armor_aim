#include "ConfigLoader.h"

#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp> // cv::FileStorage

// 读字符串: 没有这个键就跳过
static void readStr(cv::FileStorage& fs, const char* key, std::string& out) {
    cv::FileNode n = fs[key];
    if (!n.isNone() && n.isString()) out = static_cast<std::string>(n);
}

// 读整数
static void readInt(cv::FileStorage& fs, const char* key, int& out) {
    cv::FileNode n = fs[key];
    if (!n.isNone() && n.isInt()) out = static_cast<int>(n);
}

// 读小数(整数也当成小数读)
static void readDouble(cv::FileStorage& fs, const char* key, double& out) {
    cv::FileNode n = fs[key];
    if (n.isNone()) return;
    if (n.isReal()) out = static_cast<double>(n);
    else if (n.isInt()) out = static_cast<int>(n);
}

// 读布尔(true / false 或 1 / 0)
static void readBool(cv::FileStorage& fs, const char* key, bool& out) {
    cv::FileNode n = fs[key];
    if (n.isNone()) return;
    if (n.isInt()) { out = (static_cast<int>(n) != 0); return; }
    if (n.isString()) out = (static_cast<std::string>(n) == "true");
}

bool ConfigLoader::load(const std::string& path, AppConfig& cfg) const {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "打不开配置文件: " << path << std::endl;
        return false;
    }

    // 逐个键读, 没有的键直接跳过(保持 cfg 里已有的默认值)
    readStr(fs, "video_path", cfg.video_path);
    readBool(fs, "save_video", cfg.save_video);
    readStr(fs, "save_video_path", cfg.save_video_path);
    readBool(fs, "show_debug", cfg.show_debug);

    readStr(fs, "enemy_color", cfg.enemy_color);
    readStr(fs, "preprocess_mode", cfg.preprocess_mode);
    readInt(fs, "h_low", cfg.h_low); readInt(fs, "s_low", cfg.s_low); readInt(fs, "v_low", cfg.v_low);
    readInt(fs, "h_high", cfg.h_high); readInt(fs, "s_high", cfg.s_high); readInt(fs, "v_high", cfg.v_high);
    readInt(fs, "gray_thresh", cfg.gray_thresh);
    readInt(fs, "bgr_diff_thresh", cfg.bgr_diff_thresh);

    readDouble(fs, "lightbar_min_area", cfg.lightbar_min_area);
    readDouble(fs, "lightbar_min_ratio", cfg.lightbar_min_ratio);
    readDouble(fs, "lightbar_max_angle", cfg.lightbar_max_angle);

    readDouble(fs, "armor_min_ratio", cfg.armor_min_ratio);
    readDouble(fs, "armor_max_ratio", cfg.armor_max_ratio);
    readDouble(fs, "armor_angle_sum_max", cfg.armor_angle_sum_max);
    readDouble(fs, "armor_length_diff_ratio", cfg.armor_length_diff_ratio);
    readDouble(fs, "armor_y_offset_ratio", cfg.armor_y_offset_ratio);

    readInt(fs, "lost_threshold", cfg.lost_threshold);

    readDouble(fs, "armor_width_mm", cfg.armor_width_mm);
    readDouble(fs, "armor_height_mm", cfg.armor_height_mm);

    fs.release();
    applyDefaults(cfg);
    return true;
}

void ConfigLoader::applyDefaults(AppConfig& cfg) const {
    // HSV 阈值全为 0 就当没配
    if (cfg.h_low == 0 && cfg.s_low == 0 && cfg.v_low == 0 &&
        cfg.h_high == 0 && cfg.s_high == 0 && cfg.v_high == 0) {
        if (cfg.preprocess_mode == "HSV") {
            if (cfg.enemy_color == "RED") {
                cfg.h_low = 170; cfg.s_low = 120; cfg.v_low = 80;
                cfg.h_high = 10; cfg.s_high = 255; cfg.v_high = 255; // h_low > h_high 表示环绕
            } else {
                cfg.h_low = 100; cfg.s_low = 120; cfg.v_low = 80;
                cfg.h_high = 124; cfg.s_high = 255; cfg.v_high = 255;
            }
        }
    }

    if (cfg.lightbar_min_area  <= 0) cfg.lightbar_min_area = 50;
    if (cfg.lightbar_min_ratio <= 0) cfg.lightbar_min_ratio = 1.8;
    if (cfg.lightbar_max_angle <= 0) cfg.lightbar_max_angle = 45;

    if (cfg.armor_min_ratio         <= 0) cfg.armor_min_ratio = 1.8;
    if (cfg.armor_max_ratio         <= 0) cfg.armor_max_ratio = 5.0;
    if (cfg.armor_angle_sum_max     <= 0) cfg.armor_angle_sum_max = 30;
    if (cfg.armor_length_diff_ratio <= 0) cfg.armor_length_diff_ratio = 0.35;
    if (cfg.armor_y_offset_ratio    <= 0) cfg.armor_y_offset_ratio = 0.6;

    if (cfg.lost_threshold <= 0) cfg.lost_threshold = 20;

    if (cfg.armor_width_mm  <= 0) cfg.armor_width_mm = 130;
    if (cfg.armor_height_mm <= 0) cfg.armor_height_mm = 55;
}
