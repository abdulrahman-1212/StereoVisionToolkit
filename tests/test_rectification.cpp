#include "core/rectification.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <sys/stat.h>
#include <filesystem>

namespace fs = std::filesystem;

using namespace StereoVision;
using namespace StereoVision::core;


// Helper to draw horizontal epipolar lines to verify rectification
void drawEpipolarLines(cv::Mat& left, cv::Mat& right, int line_gap = 50) {
    cv::cvtColor(left, left, cv::COLOR_GRAY2BGR);
    cv::cvtColor(right, right, cv::COLOR_GRAY2BGR);

    for (int y = 0; y < left.rows; y += line_gap) {
        cv::line(left, cv::Point(0, y), cv::Point(left.cols, y), cv::Scalar(0, 255, 0), 1);
        cv::line(right, cv::Point(0, y), cv::Point(right.cols, y), cv::Scalar(0, 255, 0), 1);
    }
}

// Parse KITTI Odometry calibration file (calib/XX.txt)
// KITTI provides P2 (left) and P3 (right) projection matrices.
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

    // Extract Intrinsics (K) from P2 (top-left 3x3)
    cfg.left.fx = P2_data[0];
    cfg.left.fy = P2_data[5];
    cfg.left.cx = P2_data[2];
    cfg.left.cy = P2_data[6];
    
    cfg.right.fx = P3_data[0];
    cfg.right.fy = P3_data[5];
    cfg.right.cx = P3_data[2];
    cfg.right.cy = P3_data[6];

    // KITTI Odometry images are already rectified, so R = Identity
    // and distortion D = 0.
    cfg.left.dist_coeffs = {0, 0, 0, 0, 0};
    cfg.right.dist_coeffs = {0, 0, 0, 0, 0};

    // Calculate baseline from P2 and P3
    // P2[0,3] = -fx * baseline_left, P3[0,3] = -fx * baseline_right
    // baseline = (P2[0,3] - P3[0,3]) / fx
    double baseline = (P2_data[3] - P3_data[3]) / cfg.left.fx;

    // Construct T_right_left (4x4 Transformation Matrix)
    // Since it's already rectified, R is Identity, t is [-baseline, 0, 0]
    cfg.T_right_left = Eigen::Matrix4d::Identity();
    cfg.T_right_left(0, 3) = -baseline; // X translation
    cfg.T_right_left(1, 3) = 0.0;       // Y translation
    cfg.T_right_left(2, 3) = 0.0;       // Z translation

    // Set image dimensions (KITTI odometry is typically 1241x376 or 1226x370)
    // We will read this from the actual image later, but set a default
    cfg.left.width = 1241; 
    cfg.left.height = 376;
    cfg.right.width = 1241;
    cfg.right.height = 376;

    return true;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <kitti_odometry_root_dir>" << std::endl;
        std::cerr << "Example: ./test_rectification ./kitti_dataset/" << std::endl;
        return -1;
    }
    
    std::string root_dir = argv[1];

    // Create output directory
    fs::create_directories("output");

    // Loop through scenes 00 to 10
    for (int scene = 0; scene <= 10; ++scene) {
        std::string scene_str = std::to_string(scene);
        if (scene_str.length() < 2) scene_str = "0" + scene_str;

        std::cout << "\n--- Processing Scene " << scene_str << " ---" << std::endl;

        // 1. Construct paths
        std::string calib_path = root_dir + scene_str + "/calib.txt";
        std::string left_img_path = root_dir + scene_str + "/image_2/000000.png";
        std::string right_img_path = root_dir + scene_str + "/image_3/000000.png";
        
        // Fallback paths if directory structure is flat
        if (fs::exists(root_dir + "/image_2/000000.png")) {
            left_img_path = root_dir + "/image_2/000000.png";
            right_img_path = root_dir + "/image_3/000000.png";
        }

        // 2. Load Images
        cv::Mat left_raw = cv::imread(left_img_path, cv::IMREAD_GRAYSCALE);
        cv::Mat right_raw = cv::imread(right_img_path, cv::IMREAD_GRAYSCALE);

        if (left_raw.empty() || right_raw.empty()) {
            std::cerr << "Failed to load images for scene " << scene_str << ". Check paths." << std::endl;
            continue;
        }

        // 3. Parse Calibration
        StereoVision::StereoCameraConfig cfg;
        if (!parseKITTICalib(calib_path, cfg)) continue;

        // Update image dimensions from the actual loaded image
        cfg.left.width = left_raw.cols;
        cfg.left.height = left_raw.rows;
        cfg.right.width = right_raw.cols;
        cfg.right.height = right_raw.rows;

        // 4. Initialize Rectification Engine
        try {
            RectificationEngine engine(cfg);

            // 5. Rectify
            cv::Mat left_rect, right_rect;
            engine.rectify(left_raw, right_raw, left_rect, right_rect);

            // 6. Visual Verification: Draw Epipolar Lines
            drawEpipolarLines(left_rect, right_rect, 50);

            // 7. Concatenate and Save
            cv::Mat combined;
            cv::vconcat(left_rect, right_rect, combined);
            
            std::string out_path = "output/scene_" + scene_str + "_rectified.png";
            cv::imwrite(out_path, combined);
            std::cout << "Saved verification image to: " << out_path << std::endl;

        } catch (const std::exception& e) {
            std::cerr << "Error processing scene " << scene_str << ": " << e.what() << std::endl;
        }
    }

    std::cout << "\nAll scenes processed. Check the 'output/' directory." << std::endl;
    return 0;
}