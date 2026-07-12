#pragma once

#include <cstdint>
#include <array>
#include <string>
#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace StereoVision {

// Primitive types

using float32_t = float;
using float64_t = double;

/// Dense disparity map (float32, one pixel per stereo pair pixel)
using DisparityMap = cv::Mat1f;

/// Confidence map in [0,1] — higher is more reliable
using ConfidenceMap = cv::Mat1f;

/// Semantic label map (uint8, class IDs)
using SemanticMap = cv::Mat1b;

// Stereo camera intrinsics & extrinsics
struct CameraIntrinsics {
    float64_t fx{0.0}, fy{0.0};   ///< Focal lengths (px)
    float64_t cx{0.0}, cy{0.0};   ///< Principal point (px)
    int       width{0}, height{0};
    /// Distortion: [k1, k2, p1, p2, k3]
    std::array<float64_t, 5> dist_coeffs{};
    cv::Mat   camera_matrix() const;
    cv::Mat   dist_coeffs_mat() const;
};

struct StereoCameraConfig {
    CameraIntrinsics left;
    CameraIntrinsics right;
    Eigen::Matrix4d  T_right_left = Eigen::Matrix4d::Identity(); ///< Extrinsic transform
    float64_t        baseline_m{0.12};                           ///< Baseline in metres

    /// Derive Q matrix (OpenCV reprojection matrix) from calibration
    cv::Mat compute_Q_matrix() const;
};

// ─────────────────────────────────────────────────────────────────────────────
// Disparity algorithm selection
// ─────────────────────────────────────────────────────────────────────────────

enum class DisparityAlgorithm {
    SGBM,          ///< OpenCV Semi-Global Block Matching
    SGBM_CUDA,     ///< CUDA-accelerated SGBM (requires SVT_CUDA_ENABLED)
    RAFT_STEREO,   ///< RAFT-Stereo neural network (requires SVT_TENSORRT_ENABLED)
    HITNET,        ///< HITNet neural network
    CENSUS_FAST,   ///< Census-transform BM, optimised for edge devices
};

// ─────────────────────────────────────────────────────────────────────────────
// Occlusion handling modes
// ─────────────────────────────────────────────────────────────────────────────

enum class OcclusionMode {
    LEFT_RIGHT_CHECK,   ///< Classic LR consistency check
    CENSUS_VALIDATION,  ///< Census-based uniqueness check
    TEMPORAL_FILL,      ///< Fill from previous frames via optical flow
    INPAINTING,         ///< Image-guided inpainting (JBF)
    NONE,
};

// ─────────────────────────────────────────────────────────────────────────────
// Sensor fusion strategy
// ─────────────────────────────────────────────────────────────────────────────

enum class FusionStrategy {
    STEREO_ONLY,
    LIDAR_STEREO_KALMAN,  ///< EKF fusing sparse LiDAR + dense stereo
    RADAR_STEREO,         ///< Doppler-aware fusion for moving objects
    FULL_FUSION,          ///< LiDAR + Radar + Stereo + IMU
};

// ─────────────────────────────────────────────────────────────────────────────
// Runtime pipeline configuration
// ─────────────────────────────────────────────────────────────────────────────

struct StereoProcessingConfig {
    // Algorithm choices
    DisparityAlgorithm disparity_algo  = DisparityAlgorithm::SGBM;
    OcclusionMode      occlusion_mode  = OcclusionMode::LEFT_RIGHT_CHECK;
    FusionStrategy     fusion_strategy = FusionStrategy::STEREO_ONLY;

    // SGBM parameters
    int  min_disparity{0};
    int  num_disparities{128};
    int  block_size{5};
    int  p1_coeff{8};          ///< P1 = p1_coeff * block_size^2
    int  p2_coeff{32};         ///< P2 = p2_coeff * block_size^2
    int  disp12_max_diff{1};
    int  pre_filter_cap{63};
    int  uniqueness_ratio{15};
    int  speckle_window_size{100};
    int  speckle_range{32};

    // Edge/resource limits
    bool  half_precision{false};  ///< FP16 where possible
    int   max_threads{4};
    bool  use_gpu{false};
    float target_fps{10.0f};      ///< Throttle if exceeded

    // Temporal smoothing
    float temporal_alpha{0.3f};   ///< EMA weight for depth map smoothing
    int   temporal_window{5};

    // Output
    bool  publish_pointcloud{true};
    bool  visualise{false};

    static StereoProcessingConfig load_yaml(const std::string& path);
    void save_yaml(const std::string& path) const;
};

// ─────────────────────────────────────────────────────────────────────────────
// Per-frame result bundle
// ─────────────────────────────────────────────────────────────────────────────

struct StereoFrame {
    uint64_t      timestamp_us{0};
    cv::Mat       left_rectified;
    cv::Mat       right_rectified;
    DisparityMap  disparity;
    DisparityMap  disparity_filtered;
    ConfidenceMap confidence;
    cv::Mat       depth_m;           ///< float32 depth in metres
    cv::Mat       point_cloud_xyz;   ///< CV_32FC3, organised

    // Occlusion metadata
    cv::Mat1b     occlusion_mask;    ///< 255 = occluded, 0 = valid
    float         occlusion_ratio{0.f};

    // Diagnostics
    float         processing_ms{0.f};
    bool          is_valid{false};
};

} // namespace svt
