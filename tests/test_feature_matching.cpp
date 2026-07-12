#include "core/rectification.hpp"
#include "core/feature_matching.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>

namespace fs = std::filesystem;
using namespace StereoVision;
using namespace StereoVision::core;

// Parse KITTI Odometry calibration file (calib/XX.txt)
bool parseKITTICalib(const std::string& calib_path, StereoCameraConfig& cfg) {
    std::ifstream file(calib_path);
    if (!file.is_open()) {
        std::cerr << "Failed to open calibration file: " << calib_path << std::endl;
        return false;
    }
    std::string line;
    std::vector<double> P2_data, P3_data;
    while (std::getline(file, line)) {
        if (line.substr(0, 3) == "P2:") {
            std::stringstream ss(line.substr(3));
            double val;
            while (ss >> val) P2_data.push_back(val);
        } else if (line.substr(0, 3) == "P3:") {
            std::stringstream ss(line.substr(3));
            double val;
            while (ss >> val) P3_data.push_back(val);
        }
    }
    if (P2_data.size() != 12 || P3_data.size() != 12) {
        std::cerr << "Invalid P2/P3 matrix size in calib file." << std::endl;
        return false;
    }

    cfg.left.fx = P2_data[0];  cfg.left.fy = P2_data[5];
    cfg.left.cx = P2_data[2];  cfg.left.cy = P2_data[6];
    cfg.right.fx = P3_data[0]; cfg.right.fy = P3_data[5];
    cfg.right.cx = P3_data[2]; cfg.right.cy = P3_data[6];

    cfg.left.dist_coeffs = {0, 0, 0, 0, 0};
    cfg.right.dist_coeffs = {0, 0, 0, 0, 0};

    double baseline = (P2_data[3] - P3_data[3]) / cfg.left.fx;
    cfg.T_right_left = Eigen::Matrix4d::Identity();
    cfg.T_right_left(0, 3) = -baseline; 

    cfg.left.width = 1241;  cfg.left.height = 376;
    cfg.right.width = 1241; cfg.right.height = 376;
    return true;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <kitti_odometry_root_dir>" << std::endl;
        return -1;
    }
    std::string root_dir = argv[1];
    fs::create_directories("output_matches");

    // Configure the feature matcher for edge devices
    FeatureMatchingEngine::Config match_cfg;
    match_cfg.n_features = 1500;
    match_cfg.use_cross_check = true;
    match_cfg.max_epipolar_dy = 2.0f; // Relaxed slightly for KITTI rectification noise
    
    FeatureMatchingEngine matcher(match_cfg);

    for (int scene = 0; scene <= 10; ++scene) {
        std::string scene_str = std::to_string(scene);
        if (scene_str.length() < 2) scene_str = "0" + scene_str;
        std::cout << "\n--- Processing Scene " << scene_str << " ---" << std::endl;

        // Using std::filesystem for robust path concatenation
        fs::path root = root_dir;
        fs::path calib_path = root / scene_str / "calib.txt";
        fs::path left_img_path = root / scene_str / "image_2" / "000000.png";
        fs::path right_img_path = root / scene_str / "image_3" / "000000.png";

        // Fallback paths if directory structure is flat
        if (fs::exists(root / "image_2" / "000000.png")) {
            left_img_path = root / "image_2" / "000000.png";
            right_img_path = root / "image_3" / "000000.png";
        }

        cv::Mat left_raw = cv::imread(left_img_path.string(), cv::IMREAD_GRAYSCALE);
        cv::Mat right_raw = cv::imread(right_img_path.string(), cv::IMREAD_GRAYSCALE);
        if (left_raw.empty() || right_raw.empty()) {
            std::cerr << "Failed to load images for scene " << scene_str << ". Check paths." << std::endl;
            continue;
        }

        StereoVision::StereoCameraConfig cfg;
        if (!parseKITTICalib(calib_path.string(), cfg)) continue;

        cfg.left.width = left_raw.cols;   cfg.left.height = left_raw.rows;
        cfg.right.width = right_raw.cols; cfg.right.height = right_raw.rows;

        try {
            // 1. Rectify
            RectificationEngine rect_engine(cfg);
            cv::Mat left_rect, right_rect;
            rect_engine.rectify(left_raw, right_raw, left_rect, right_rect);

            // 2. Match Features
            auto matches = matcher.match(left_rect, right_rect);
            std::cout << "Found " << matches.keypoints_left.size() << " left KPs, "
                      << matches.keypoints_right.size() << " right KPs, "
                      << matches.matches.size() << " valid stereo matches." << std::endl;

            // 3. Visualise
            cv::Mat vis = matcher.draw_matches(matches, left_rect, right_rect, 150);
            
            // Ensure image is BGR so we can draw green epipolar lines
            if (vis.channels() == 1) cv::cvtColor(vis, vis, cv::COLOR_GRAY2BGR);
            
            for (int y = 0; y < vis.rows; y += 50) {
                cv::line(vis, cv::Point(0, y), cv::Point(vis.cols, y), cv::Scalar(0, 255, 0), 1);
            }

            std::string out_path = "output_matches/scene_" + scene_str + "_matches.png";
            cv::imwrite(out_path, vis);
            std::cout << "Saved verification image to: " << out_path << std::endl;

        } catch (const std::exception& e) {
            std::cerr << "Error processing scene " << scene_str << ": " << e.what() << std::endl;
        }
    }

    std::cout << "\nAll scenes processed. Check the 'output_matches/' directory." << std::endl;
    return 0;
}
