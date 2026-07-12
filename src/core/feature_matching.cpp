#include "core/feature_matching.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace StereoVision::core {

FeatureMatchingEngine::FeatureMatchingEngine(const Config& cfg) : cfg_(cfg) {
    orb_ = cv::ORB::create(cfg_.n_features, cfg_.scale_factor, cfg_.n_levels);
    matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, cfg_.use_cross_check);
}

StereoMatches FeatureMatchingEngine::match(const cv::Mat& img_left, const cv::Mat& img_right) const {
    StereoMatches result;
    
    if (img_left.empty() || img_right.empty()) {
        throw std::invalid_argument("Input images are empty.");
    }

    // 1. Detect and compute ORB features
    orb_->detectAndCompute(img_left,  cv::noArray(), result.keypoints_left,  result.descriptors_left);
    orb_->detectAndCompute(img_right, cv::noArray(), result.keypoints_right, result.descriptors_right);

    if (result.descriptors_left.empty() || result.descriptors_right.empty()) {
        return result; // No features found
    }

    // 2. Match descriptors
    if (cfg_.use_cross_check) {
        matcher_->match(result.descriptors_left, result.descriptors_right, result.matches);
    } else {
        std::vector<std::vector<cv::DMatch>> knn_matches;
        matcher_->knnMatch(result.descriptors_left, result.descriptors_right, knn_matches, 2);
        
        // Apply Lowe's ratio test
        for (const auto& m_pair : knn_matches) {
            if (m_pair.size() == 2 && m_pair[0].distance < cfg_.ratio_thresh * m_pair[1].distance) {
                result.matches.push_back(m_pair[0]);
            }
        }
    }

    // 3. Geometric filtering for rectified stereo pairs
    // Assumes images are horizontally rectified:
    // - Epipolar lines are horizontal (dy ~= 0)
    // - Right camera is to the right, so x_left > x_right (disparity > 0)
    std::vector<cv::DMatch> filtered;
    filtered.reserve(result.matches.size());
    
    for (const auto& m : result.matches) {
        const auto& pt1 = result.keypoints_left[m.queryIdx].pt;
        const auto& pt2 = result.keypoints_right[m.trainIdx].pt;
        
        float dy = std::abs(pt1.y - pt2.y);
        float dx = pt1.x - pt2.x; // Disparity
        
        if (dy < cfg_.max_epipolar_dy && dx > 0.0f && dx < cfg_.max_disparity) {
            filtered.push_back(m);
        }
    }
    
    result.matches = std::move(filtered);
    return result;
}

cv::Mat FeatureMatchingEngine::draw_matches(const StereoMatches& m, 
                                            const cv::Mat& img_left, 
                                            const cv::Mat& img_right,
                                            int max_matches_to_draw) const {
    cv::Mat out;
    std::vector<cv::DMatch> matches_to_draw = m.matches;
    
    // Limit the number of drawn matches for cleaner visualization
    if (max_matches_to_draw > 0 && static_cast<int>(matches_to_draw.size()) > max_matches_to_draw) {
        matches_to_draw.resize(max_matches_to_draw);
    }
    
    cv::drawMatches(img_left, m.keypoints_left, 
                    img_right, m.keypoints_right, 
                    matches_to_draw, out,
                    cv::Scalar::all(-1), cv::Scalar::all(-1),
                    std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
                    
    return out;
}

} // namespace StereoVision::core