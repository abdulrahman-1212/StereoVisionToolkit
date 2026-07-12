# StereoVision Edge Toolkit

A high-performance, low-latency stereo vision toolkit optimized for edge devices, featuring robust occlusion handling and real-time depth estimation.

## Overview

**StereoVision Edge Toolkit** is designed to bring accurate 3D perception to resource-constrained environments. By leveraging hardware acceleration and zero-allocation pipelines, it enables real-time stereo processing on embedded platforms (e.g., Raspberry Pi, NVIDIA Jetson, ARM Cortex) without sacrificing accuracy. 

This project is currently in active development. The core rectification pipeline has been implemented, with stereo matching and advanced occlusion handling currently in the works.

## Key Features

- **Zero-Allocation Rectification**: Precomputes undistortion and rectification maps to ensure the runtime pipeline introduces no dynamic memory allocations.
- **Edge-Optimized Pipeline**: Tailored for low-power CPUs and embedded GPUs, utilizing OpenCV's optimized primitives.
- **Occlusion Handling**: *(In Progress)* Advanced algorithms to detect and handle occluded regions in stereo pairs, reducing noise and artifacts in depth maps.
- **Accurate 3D Reconstruction**: Provides direct access to the reprojection matrix (`Q`) and valid Regions of Interest (ROIs) for seamless integration with downstream 3D processing.

## Project Architecture

The toolkit is organized into modular components under the `StereoVision::core` namespace:

- **`RectificationEngine`**: Handles camera calibration, precomputes rectification maps, and applies them to raw stereo pairs.
- **`StereoMatcher`**: Performs pixel matching to generate disparity maps.
- **`OcclusionHandler`** *(Coming Soon)*: Filters invalid disparities and fills occluded regions.
- **`DepthEstimator`** *(Coming Soon)*: Converts disparity maps into metric 3D point clouds.

## Prerequisites

- C++17 compatible compiler
- [OpenCV](https://opencv.org/) (version 4.5 or higher recommended)
- CMake (version 3.14+)

## Building

```bash
# Clone the repository
git clone https://github.com/abdulrahman-1212/StereoVisionToolkit.git
cd stereovision-edge

# Create a build directory
mkdir build && cd build

# Configure and build
cmake ..
make -j$(nproc)
```


## Roadmap

- [ ] **Core Types & Configuration** (`types.hpp`)
- [ ] **Rectification Engine** (Zero-allocation map precomputation and application)
- [ ] **Stereo Matching** (Implementing block matching / semi-global matching optimized for edge)
- [ ] **Occlusion Handling** (Left-right consistency checks, edge-aware filtering)
- [ ] **Depth Map Generation** (Reprojection using the `Q` matrix)
- [ ] **Hardware Acceleration** (OpenCL / CUDA / NEON optimizations)
- [ ] **ROS2 Integration** (Publishing rectified images and depth clouds)

## Contributing

Contributions are welcome! If you'd like to help build out the stereo matching or occlusion handling modules, please open an issue or submit a pull request.
