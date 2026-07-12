#pragma once

#include "types.hpp"
#include <opencv2/calib3d.hpp>
#include <memory>

namespace StereoVision::core {

/// Precomputes undistort/rectify maps; applies them at runtime with zero allocations.
class RectificationEngine {
public:
    explicit RectificationEngine(const StereoCameraConfig& cfg);

    /// Rectify a raw stereo pair in-place (or into separate output buffers).
    void rectify(const cv::Mat& raw_left,
                 const cv::Mat& raw_right,
                 cv::Mat&       out_left,
                 cv::Mat&       out_right) const;

    const cv::Mat& Q()              const { return Q_; }
    const cv::Rect valid_roi_left() const { return roi_left_; }
    const cv::Rect valid_roi_right()const { return roi_right_; }

    cv::Size rectified_size()       const { return rect_size_; }

private:
    void build_maps(const StereoCameraConfig& cfg);

    cv::Mat map_left_x_, map_left_y_;
    cv::Mat map_right_x_, map_right_y_;
    cv::Mat Q_;
    cv::Rect roi_left_, roi_right_;
    cv::Size rect_size_;
};

} // namespace svt::core