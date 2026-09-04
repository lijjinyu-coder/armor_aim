#ifndef ARMOR_AIM_POSE_SOLVER_H
#define ARMOR_AIM_POSE_SOLVER_H

#include <vector>

#include <opencv2/core.hpp>

#include "Common.h"

// 位姿解算: 用四个角点 + 相机内参, solvePnP 得到平移向量 t
//   yaw   = atan2(tx, tz)
//   pitch = -atan2(ty, tz)
//   dist  = |t|
// 相机内参直接用近似的 (fx=fy=800, cx=320, cy=240), 写在下面代码里
class PoseSolver {
public:
    struct Params {
        double armor_width_mm  = 130; // 装甲板宽(mm)
        double armor_height_mm = 55;  // 装甲板高(mm)
    };

    void setParams(const Params& p);

    Pose solve(const Armor& armor);

private:
    Params params_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    std::vector<cv::Point3f> object_points_; // 装甲板四个角的世界坐标(mm)
};

#endif // ARMOR_AIM_POSE_SOLVER_H
