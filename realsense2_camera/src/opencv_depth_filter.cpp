// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2022 Intel Corporation. All Rights Reserved.

#include "../include/opencv_depth_filter.h"
#include <librealsense2/rsutil.h>
#include <cstring>
#include <memory>
#include <sensor_msgs/image_encodings.hpp>
#include <vector>
#include "constants.h"

using namespace realsense2_camera;

// Create a minimal filter wrapper - we use a pointcloud filter as base since it's always available
// but we override its Process methods
OpenCVDepthFilterWrapper::OpenCVDepthFilterWrapper(std::shared_ptr<Parameters> parameters, rclcpp::Logger logger,
                                                   bool is_enabled)
    : NamedFilter(std::make_shared<rs2::pointcloud>(), parameters, logger, is_enabled, false)
    , _use_bilateral(true)          // Enabled by default for edge-preserving noise reduction
    , _bilateral_d(9)               // Larger d for more noticeable smoothing
    , _bilateral_sigma_color(75.0)  // Higher sigma for more smoothing
    , _bilateral_sigma_space(75.0)  // Higher sigma for more smoothing
    , _use_median(true)             // Enabled by default for salt-and-pepper noise removal
    , _median_kernel_size(7)        // Larger kernel for more noticeable effect
    , _use_morphology(true)         // Enabled by default to fill small holes
    , _morphology_kernel_size(5)
    , _morphology_type(1)   // Closing operation to fill holes
    , _use_gaussian(false)  // Disabled by default (bilateral is better for depth)
    , _gaussian_kernel_size(5)
    , _gaussian_sigma_x(1.0)
    , _gaussian_sigma_y(1.0)
{
    setOpenCVParameters();
}

void OpenCVDepthFilterWrapper::setOpenCVParameters()
{
    std::string module_name = "opencv_depth_filter";

    // NOTE: The filtered depth is published on the same topics as regular depth:
    // - /camera/depth/image_rect_raw (main depth topic - THIS IS WHERE YOU'LL SEE THE CHANGES)
    // - /camera/aligned_depth_to_color/image_raw (if align_depth is enabled)
    // - /camera/depth/color/points (pointcloud, if enabled)
    // The filter processes frames in the pipeline before publishing.
    //
    // To visualize: Use RViz2 and subscribe to /camera/depth/image_rect_raw
    // You should see: smoother depth, fewer holes, less noise, especially on edges

    // Enable parameter - use NamedFilter's parameter system
    std::string param_name = module_name + ".enable";
    _params.getParameters()->setParamT(param_name, _is_enabled,
                                       [this](const rclcpp::Parameter& param) { _is_enabled = param.as_bool(); });
    _parameters_names.push_back(param_name);

    // Bilateral filter parameters
    param_name = module_name + ".use_bilateral";
    _params.getParameters()->setParamT(param_name, _use_bilateral);
    _parameters_names.push_back(param_name);

    param_name = module_name + ".bilateral_d";
    _params.getParameters()->setParamT(param_name, _bilateral_d);
    _parameters_names.push_back(param_name);

    param_name = module_name + ".bilateral_sigma_color";
    _params.getParameters()->setParamT(param_name, _bilateral_sigma_color);
    _parameters_names.push_back(param_name);

    param_name = module_name + ".bilateral_sigma_space";
    _params.getParameters()->setParamT(param_name, _bilateral_sigma_space);
    _parameters_names.push_back(param_name);

    // Median filter parameters
    param_name = module_name + ".use_median";
    _params.getParameters()->setParamT(param_name, _use_median);
    _parameters_names.push_back(param_name);

    param_name = module_name + ".median_kernel_size";
    _params.getParameters()->setParamT(param_name, _median_kernel_size);
    _parameters_names.push_back(param_name);

    // Morphology filter parameters
    param_name = module_name + ".use_morphology";
    _params.getParameters()->setParamT(param_name, _use_morphology);
    _parameters_names.push_back(param_name);

    param_name = module_name + ".morphology_kernel_size";
    _params.getParameters()->setParamT(param_name, _morphology_kernel_size);
    _parameters_names.push_back(param_name);

    param_name = module_name + ".morphology_type";
    _params.getParameters()->setParamT(param_name, _morphology_type);
    _parameters_names.push_back(param_name);

    // Gaussian filter parameters
    param_name = module_name + ".use_gaussian";
    _params.getParameters()->setParamT(param_name, _use_gaussian);
    _parameters_names.push_back(param_name);

    param_name = module_name + ".gaussian_kernel_size";
    _params.getParameters()->setParamT(param_name, _gaussian_kernel_size);
    _parameters_names.push_back(param_name);

    param_name = module_name + ".gaussian_sigma_x";
    _params.getParameters()->setParamT(param_name, _gaussian_sigma_x);
    _parameters_names.push_back(param_name);

    param_name = module_name + ".gaussian_sigma_y";
    _params.getParameters()->setParamT(param_name, _gaussian_sigma_y);
    _parameters_names.push_back(param_name);
}

cv::Mat OpenCVDepthFilterWrapper::depthFrameToMat(rs2::depth_frame depth_frame)
{
    const int width = depth_frame.get_width();
    const int height = depth_frame.get_height();

    // Create OpenCV Mat from depth frame data
    cv::Mat depth_mat(height, width, CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

    // Clone to ensure we have our own copy
    return depth_mat.clone();
}

rs2::frame OpenCVDepthFilterWrapper::processDepthFrame(rs2::depth_frame depth_frame)
{
    if (!_is_enabled)
    {
        return depth_frame;
    }

    cv::Mat depth_mat = depthFrameToMat(depth_frame);
    cv::Mat processed = depth_mat.clone();

    // Apply filters in sequence:
    // - Bilateral: Edge-preserving smoothing (reduces noise while keeping sharp edges)
    // - Median: Removes salt-and-pepper noise (outliers)
    // - Morphology: Fills small holes (closing) or removes small objects (opening)
    // - Gaussian: General smoothing (can blur edges, use sparingly)
    if (_use_bilateral)
    {
        cv::bilateralFilter(processed, processed, _bilateral_d, _bilateral_sigma_color, _bilateral_sigma_space);
    }

    if (_use_median)
    {
        cv::medianBlur(processed, processed, _median_kernel_size);
    }

    if (_use_morphology)
    {
        cv::Mat kernel =
            cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(_morphology_kernel_size, _morphology_kernel_size));

        switch (_morphology_type)
        {
            case 0:  // Opening
                cv::morphologyEx(processed, processed, cv::MORPH_OPEN, kernel);
                break;
            case 1:  // Closing
                cv::morphologyEx(processed, processed, cv::MORPH_CLOSE, kernel);
                break;
            case 2:  // Gradient
                cv::morphologyEx(processed, processed, cv::MORPH_GRADIENT, kernel);
                break;
            default:
                break;
        }
    }

    if (_use_gaussian)
    {
        cv::GaussianBlur(processed, processed, cv::Size(_gaussian_kernel_size, _gaussian_kernel_size),
                         _gaussian_sigma_x, _gaussian_sigma_y);
    }

    // Copy processed data back to the original frame (in-place modification)
    uint16_t* p_depth_frame = reinterpret_cast<uint16_t*>(const_cast<void*>(depth_frame.get_data()));
    const int width = processed.cols;
    const int height = processed.rows;
    std::memcpy(p_depth_frame, processed.data, width * height * sizeof(uint16_t));

    // Return the original frame (now with processed data)
    return depth_frame;
}

rs2::frameset OpenCVDepthFilterWrapper::Process(rs2::frameset frameset)
{
    if (!_is_enabled)
    {
        return frameset;
    }

    rs2::depth_frame depth_frame = frameset.get_depth_frame();
    if (!depth_frame)
    {
        return frameset;
    }

    // Process the depth frame in-place (modifies the frame data directly)
    processDepthFrame(depth_frame);

    // Return the original frameset (depth frame data has been modified in-place)
    return frameset;
}

rs2::frame OpenCVDepthFilterWrapper::Process(rs2::frame frame)
{
    if (!_is_enabled)
    {
        return frame;
    }

    if (frame.is<rs2::depth_frame>())
    {
        // Process in-place and return the frame
        processDepthFrame(frame.as<rs2::depth_frame>());
        return frame;
    }

    return frame;
}
