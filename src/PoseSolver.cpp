#include "PoseSolver.h"

#include <cmath>

#include <opencv2/calib3d.hpp>

void PoseSolver::setParams(const Params& p) {
    params_ = p;

    // 相机内参矩阵(用的近似值, 没做标定)
    camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    camera_matrix_.at<double>(0, 0) = 800.0; // fx
    camera_matrix_.at<double>(1, 1) = 800.0; // fy
    camera_matrix_.at<double>(0, 2) = 320.0; // cx (640 的一半)
    camera_matrix_.at<double>(1, 2) = 240.0; // cy (480 的一半)

    dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F); // 没有畸变

    // 装甲板四个角的世界坐标, 顺序和 Armor::four_points 对应(左上、右上、右下、左下)
    // 原点在装甲板中心, x 向右, y 向下, z 指向相机
    double hw = p.armor_width_mm * 0.5;
    double hh = p.armor_height_mm * 0.5;
    object_points_ = {
        cv::Point3f(-hw,  hh, 0.f),
        cv::Point3f( hw,  hh, 0.f),
        cv::Point3f( hw, -hh, 0.f),
        cv::Point3f(-hw, -hh, 0.f),
    };
}

Pose PoseSolver::solve(const Armor& armor) {
    Pose pose;
    if (armor.four_points.size() != 4 || object_points_.empty()) return pose;

    // 先试 IPPE(平面四点更快更稳), 不行再换 ITERATIVE
    cv::Mat rvec, tvec;
    bool ok = cv::solvePnP(object_points_, armor.four_points, camera_matrix_,
                           dist_coeffs_, rvec, tvec, false, cv::SOLVEPNP_IPPE);
    if (!ok) {
        ok = cv::solvePnP(object_points_, armor.four_points, camera_matrix_,
                          dist_coeffs_, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    }
    if (!ok) return pose;

    double tx = tvec.at<double>(0);
    double ty = tvec.at<double>(1);
    double tz = tvec.at<double>(2);
    if (tz <= 0) return pose; // 目标在相机后面, 不算

    pose.yaw_deg     = std::atan2(tx, tz) * 180.0 / CV_PI;
    pose.pitch_deg   = -std::atan2(ty, tz) * 180.0 / CV_PI;
    pose.distance_mm = std::sqrt(tx * tx + ty * ty + tz * tz);
    pose.valid       = true;
    return pose;
}
