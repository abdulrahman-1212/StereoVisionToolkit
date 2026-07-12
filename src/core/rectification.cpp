#include "core/rectification.hpp"
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace StereoVision {

cv::Mat CameraIntrinsics::camera_matrix() const {
    return (cv::Mat_<double>(3,3) <<
        fx, 0,  cx,
        0,  fy, cy,
        0,  0,  1.0);
}

cv::Mat CameraIntrinsics::dist_coeffs_mat() const {
    return cv::Mat(1, 5, CV_64F, const_cast<double*>(dist_coeffs.data())).clone();
}

cv::Mat StereoCameraConfig::compute_Q_matrix() const {
    // Build R and T from the extrinsic matrix.
    cv::Mat R = (cv::Mat_<double>(3,3) <<
        T_right_left(0,0), T_right_left(0,1), T_right_left(0,2),
        T_right_left(1,0), T_right_left(1,1), T_right_left(1,2),
        T_right_left(2,0), T_right_left(2,1), T_right_left(2,2));
    cv::Mat t = (cv::Mat_<double>(3,1) <<
        T_right_left(0,3), T_right_left(1,3), T_right_left(2,3));

    cv::Mat K1 = left.camera_matrix(),  D1 = left.dist_coeffs_mat();
    cv::Mat K2 = right.camera_matrix(), D2 = right.dist_coeffs_mat();

    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(K1, D1, K2, D2,
                       cv::Size(left.width, left.height),
                       R, t, R1, R2, P1, P2, Q,
                       cv::CALIB_ZERO_DISPARITY, 0);
    return Q;
}

}   // namespace StereoVision

namespace StereoVision::core {

RectificationEngine::RectificationEngine(const StereoCameraConfig& cfg) {
    build_maps(cfg);
}

void RectificationEngine::build_maps(const StereoCameraConfig& cfg) {
    cv::Mat K1 = cfg.left.camera_matrix(),  D1 = cfg.left.dist_coeffs_mat();
    cv::Mat K2 = cfg.right.camera_matrix(), D2 = cfg.right.dist_coeffs_mat();

    cv::Mat R = (cv::Mat_<double>(3,3) <<
        cfg.T_right_left(0,0), cfg.T_right_left(0,1), cfg.T_right_left(0,2),
        cfg.T_right_left(1,0), cfg.T_right_left(1,1), cfg.T_right_left(1,2),
        cfg.T_right_left(2,0), cfg.T_right_left(2,1), cfg.T_right_left(2,2));
    cv::Mat t = (cv::Mat_<double>(3,1) <<
        cfg.T_right_left(0,3), cfg.T_right_left(1,3), cfg.T_right_left(2,3));

    cv::Size img_size(cfg.left.width, cfg.left.height);

    cv::Mat R1, R2, P1, P2;
    cv::stereoRectify(K1, D1, K2, D2, img_size,
                       R, t, R1, R2, P1, P2, Q_,
                       cv::CALIB_ZERO_DISPARITY, 0,
                       img_size, &roi_left_, &roi_right_);

    cv::initUndistortRectifyMap(K1, D1, R1, P1, img_size, CV_16SC2,
                                  map_left_x_, map_left_y_);
    cv::initUndistortRectifyMap(K2, D2, R2, P2, img_size, CV_16SC2,
                                  map_right_x_, map_right_y_);
    rect_size_ = img_size;
}

void RectificationEngine::rectify(const cv::Mat& raw_left,
                                   const cv::Mat& raw_right,
                                   cv::Mat&       out_left,
                                   cv::Mat&       out_right) const {
    cv::remap(raw_left,  out_left,  map_left_x_,  map_left_y_,
               cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    cv::remap(raw_right, out_right, map_right_x_, map_right_y_,
               cv::INTER_LINEAR, cv::BORDER_CONSTANT);
}

} // namespace StereoVision::core
