#ifndef POSE_ESTIMATOR_HPP
#define POSE_ESTIMATOR_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include "LBFGSB.h" // L-BFGS-B optimization library for yaw estimation

// Classes
#include "ValidityFilter.hpp"

// Default armor plate dimensions (mm). Legacy real-robot values; can be overridden
// at runtime through PoseEstimator::setArmorDimensions() (simulator scene calibration).
#define LIGHTBAR_HALF_HEIGHT 54.f / 2.f
#define SMALL_ARMOR_HALF_WIDTH 134.f / 2.f
#define LARGE_ARMOR_HALF_WIDTH 225.f / 2.f

// Default camera intrinsics (legacy MindVision camera). Overridden at runtime by
// PoseEstimator::setCameraIntrinsics() so the same core algorithm can run on the
// simulator camera (see the sim launch files) without recompiling.
#define DEFAULT_CAM_FX 1019.109
#define DEFAULT_CAM_FY 1016.785
#define DEFAULT_CAM_CX 601.885
#define DEFAULT_CAM_CY 521.005
#define DEFAULT_CAM_K1 -0.1088
#define DEFAULT_CAM_K2 -0.0721
#define DEFAULT_CAM_P1 -0.000847
#define DEFAULT_CAM_P2 0.0

using Eigen::VectorXd;
using namespace LBFGSpp;

class PoseEstimator
{
public:
    PoseEstimator() {}
    ~PoseEstimator() {}

    ValidityFilter validity_filter_ = ValidityFilter();

    // Class methods
    double estimateYaw(const double yaw_guess, const std::vector<cv::Point2f> &image_points, const cv::Mat &tvec);
    void estimateTranslation(const std::vector<cv::Point2f> &image_points, bool largeArmor, cv::Mat &tvec, cv::Mat &rvec);
    bool isValid(float x, float y, float z, std::string &auto_aim_status, bool &reset_kalman);

    // Setters
    void setNumFramesToFireAfter(int num_frames_to_fire_after) { _num_frames_to_fire_after = num_frames_to_fire_after; }
    void setAllowedMissedFramesBeforeNoFire(int allowed_missed_frames_before_no_fire) { _allowed_missed_frames_before_no_fire = allowed_missed_frames_before_no_fire; }

    /**
     * @brief Override the camera intrinsics used by solvePnP / projectPoints.
     *
     * The legacy real-robot camera parameters are the defaults; the simulator camera
     * needs its own values (see the sim launch file / camera_info topic).
     * Distortion coefficients are expected in OpenCV order (k1, k2, p1, p2[, k3]).
     */
    void setCameraIntrinsics(double fx, double fy, double cx, double cy, const std::vector<double> &distortion)
    {
        CAMERA_MATRIX = (cv::Mat_<double>(3, 3) << fx, 0.0, cx,
                         0.0, fy, cy,
                         0.0, 0.0, 1.0);
        DISTORTION_COEFFS = cv::Mat(1, static_cast<int>(distortion.size()), CV_64F);
        for (size_t i = 0; i < distortion.size(); ++i)
        {
            DISTORTION_COEFFS.at<double>(0, static_cast<int>(i)) = distortion[i];
        }
    }

    /**
     * @brief Override armor plate dimensions (mm) used for the PnP object points.
     */
    void setArmorDimensions(double lightbar_half_height, double small_armor_half_width, double large_armor_half_width)
    {
        rebuildObjectPoints(lightbar_half_height, small_armor_half_width, large_armor_half_width);
    }

private:
    // Class methods
    double computeLoss(double yaw_guess, std::vector<cv::Point2f> image_points, cv::Mat tvec);
    double gradientWrtYawFinitediff(double yaw, std::vector<cv::Point2f> image_points, cv::Mat tvec);

    /// Rebuild the PnP object points from the (possibly overridden) armor dimensions.
    void rebuildObjectPoints(double lightbar_half_height, double small_armor_half_width, double large_armor_half_width)
    {
        LIGHTBAR_HALF_HEIGHT_M = lightbar_half_height;
        SMALL_ARMOR_HALF_WIDTH_M = small_armor_half_width;
        LARGE_ARMOR_HALF_WIDTH_M = large_armor_half_width;

        SMALL_ARMOR_OBJECT_POINTS = {
            {-small_armor_half_width, -lightbar_half_height, 0}, // Top Left
            {-small_armor_half_width, lightbar_half_height, 0},  // Bot Left
            {small_armor_half_width, -lightbar_half_height, 0},  // Top Right
            {small_armor_half_width, lightbar_half_height, 0}    // Bot Right
        };
        LARGE_ARMOR_OBJECT_POINTS = {
            {-large_armor_half_width, -lightbar_half_height, 0}, // Top Left
            {-large_armor_half_width, lightbar_half_height, 0},  // Bot Left
            {large_armor_half_width, -lightbar_half_height, 0},  // Top Right
            {large_armor_half_width, lightbar_half_height, 0}    // Bot Right
        };
    }

    // Class variables
    int _consecutive_tracking_frames_ctr = 0;
    int _num_frames_to_fire_after = 1;
    int _allowed_missed_frames_before_no_fire = 150;
    int _remaining_missed_frames_before_no_fire = 0; // Gets reset when we have a valid pose estimate
    std::chrono::time_point<std::chrono::system_clock> _last_fire_time;

    // Validity filter parameters
    int _lock_in_after = 2;
    float _max_distance = 10000;
    float _min_distance = 10;
    float _max_shift_distance = 150;
    int _prev_len = 5;

    /**
     * @brief Functor for the loss function and gradient computation.
     *
     * Used for the L-BFGS-B optimization library.
     *
     * @param poseEstimator Reference to the PoseEstimator instance.
     * @param image_points The ground truth image points of the detected armor.
     * @param tvec The translation vector of the detected armor.
     * @param x The input vector containing a guess for the yaw angle.
     * @return double The loss value.
     * @return VectorXd The gradient vector.
     */
    class LossAndGradient
    {
    public:
        // Constructor to accept reference to PoseEstimator instance
        LossAndGradient(PoseEstimator &poseEstimator, std::vector<cv::Point2f> image_points, cv::Mat tvec) : poseEstimator(poseEstimator), image_points(image_points), tvec(tvec) {}

        double operator()(const Eigen::VectorXd &x, Eigen::VectorXd &grad)
        {
            double yaw = x(0);
            double loss = poseEstimator.computeLoss(yaw, image_points, tvec);
            grad(0) = poseEstimator.gradientWrtYawFinitediff(yaw, image_points, tvec);
            return loss;
        }

    private:
        std::vector<cv::Point2f> image_points;
        cv::Mat tvec;
        PoseEstimator &poseEstimator;
    };

    // Camera intrinsics (runtime-overridable, defaults = legacy real-robot camera)
    cv::Mat CAMERA_MATRIX = (cv::Mat_<double>(3, 3) << DEFAULT_CAM_FX, 0, DEFAULT_CAM_CX,
                             0, DEFAULT_CAM_FY, DEFAULT_CAM_CY,
                             0, 0, 1);
    cv::Mat DISTORTION_COEFFS = (cv::Mat_<double>(1, 4) << DEFAULT_CAM_K1, DEFAULT_CAM_K2, DEFAULT_CAM_P1, DEFAULT_CAM_P2);

    // Active armor dimensions (mm); defaults from the macros above.
    double LIGHTBAR_HALF_HEIGHT_M = LIGHTBAR_HALF_HEIGHT;
    double SMALL_ARMOR_HALF_WIDTH_M = SMALL_ARMOR_HALF_WIDTH;
    double LARGE_ARMOR_HALF_WIDTH_M = LARGE_ARMOR_HALF_WIDTH;

    /* 3D object points (measured armor dimensions)
     * Coordinate system (camera is facing us):
     *        
     *                      +x (pitch)
     *               +---->
     *             / |
     * +z (roll)  L  v +y (yaw)
     *  
     */
    std::vector<cv::Point3f> SMALL_ARMOR_OBJECT_POINTS = {
        {-SMALL_ARMOR_HALF_WIDTH, -LIGHTBAR_HALF_HEIGHT, 0}, // Top Left
        {-SMALL_ARMOR_HALF_WIDTH, LIGHTBAR_HALF_HEIGHT, 0},  // Bot Left
        {SMALL_ARMOR_HALF_WIDTH, -LIGHTBAR_HALF_HEIGHT, 0},  // Top Right
        {SMALL_ARMOR_HALF_WIDTH, LIGHTBAR_HALF_HEIGHT, 0}    // Bot Right
    };

    std::vector<cv::Point3f> LARGE_ARMOR_OBJECT_POINTS = {
        {-LARGE_ARMOR_HALF_WIDTH, -LIGHTBAR_HALF_HEIGHT, 0}, // Top Left
        {-LARGE_ARMOR_HALF_WIDTH, LIGHTBAR_HALF_HEIGHT, 0},  // Bot Left
        {LARGE_ARMOR_HALF_WIDTH, -LIGHTBAR_HALF_HEIGHT, 0},  // Top Right
        {LARGE_ARMOR_HALF_WIDTH, LIGHTBAR_HALF_HEIGHT, 0}    // Bot Right
    };
};

#endif // POSE_ESTIMATOR_HPP