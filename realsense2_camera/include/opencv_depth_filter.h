// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2022 Intel Corporation. All Rights Reserved.

#pragma once

#include <ros_sensor.h>
#include <sensor_params.h>
#include <librealsense2/rs.hpp>
#include <memory>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include "named_filter.h"

namespace realsense2_camera {
// Custom filter that wraps OpenCV depth filtering
// This filter processes depth frames using OpenCV operations
// It's designed to be compatible with the filter pipeline
class OpenCVDepthFilterWrapper : public NamedFilter
{
   public:
    OpenCVDepthFilterWrapper(std::shared_ptr<Parameters> parameters, rclcpp::Logger logger, bool is_enabled = false);
    ~OpenCVDepthFilterWrapper() = default;

    // Override Process methods to use OpenCV processing (not virtual in base, but we override behavior)
    rs2::frameset Process(rs2::frameset frameset);
    rs2::frame Process(rs2::frame frame);

   private:
    void setOpenCVParameters();
    rs2::frame processDepthFrame(rs2::depth_frame depth_frame);
    cv::Mat depthFrameToMat(rs2::depth_frame depth_frame);

    // Filter parameters
    bool _use_bilateral;
    int _bilateral_d;
    double _bilateral_sigma_color;
    double _bilateral_sigma_space;

    bool _use_median;
    int _median_kernel_size;

    bool _use_morphology;
    int _morphology_kernel_size;
    int _morphology_type;  // 0: opening, 1: closing, 2: gradient

    bool _use_gaussian;
    int _gaussian_kernel_size;
    double _gaussian_sigma_x;
    double _gaussian_sigma_y;
};
}  // namespace realsense2_camera
