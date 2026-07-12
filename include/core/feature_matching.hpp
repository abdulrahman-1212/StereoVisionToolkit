#pragma once
#include "types.hpp"
#include <opencv2/features2d.hpp>
#include <vector>

namespace StereoVision::core {

/// Bundle of detected keypoints, descriptors, and matches for a stereo pair.
struct StereoMatches {
    std::vector<cv::KeyPoint> keypoints_left;
    std::vector<cv::KeyPoint> keypoints_right;
    std::vector<cv::DMatch>   matches;
    cv::Mat                   descriptors_left;
    cv::Mat                   descriptors_right;
};

/// Lightweight, edge-optimised feature matching engine using ORB.
class FeatureMatchingEngine {
public:
    struct Config {
        int   n_features      = 2000;
        float scale_factor    = 1.2f;
        int   n_levels        = 8;
        float ratio_thresh    = 0.75f;  ///< Lowe's ratio test threshold (if cross-check is disabled)
        bool  use_cross_check = true;   ///< BFMatcher cross-check (faster, ideal for edge devices)
        float max_epipolar_dy = 1.5f;   ///< Max Y-difference for rectified epipolar constraint (px)
        float max_disparity   = 250.0f; ///< Max allowed disparity (px) to filter outliers
    };

    FeatureMatchingEngine();
    explicit FeatureMatchingEngine(const Config& cfg);
    /// Detect, describe, and match features between a rectified stereo pair.
    /// Applies epipolar and disparity constraints assuming horizontally rectified images.
    StereoMatches match(const cv::Mat& img_left, const cv::Mat& img_right) const;

    /// Draw matches for visual verification.
    cv::Mat draw_matches(const StereoMatches& m, 
                         const cv::Mat& img_left, 
                         const cv::Mat& img_right,
                         int max_matches_to_draw = 150) const;

private:
    Config cfg_;
    cv::Ptr<cv::ORB> orb_;
    cv::Ptr<cv::BFMatcher> matcher_;
};

} // namespace StereoVision::core