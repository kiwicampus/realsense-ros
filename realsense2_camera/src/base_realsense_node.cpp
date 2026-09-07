// Copyright 2023 RealSense, Inc. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "../include/base_realsense_node.h"
#include "assert.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <thread>
#include <mutex>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <array>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/clock.hpp>
#include <fstream>
#include <image_publisher.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

// Header files for disabling intra-process comms for static broadcaster.
#include <rclcpp/publisher_options.hpp>
#include <tf2_ros/qos.hpp>
#include "pointcloud_filter.h"
#include "align_depth_filter.h"

using namespace realsense2_camera;

SyncedImuPublisher::SyncedImuPublisher(rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher, 
                                       std::size_t waiting_list_size):
            _publisher(imu_publisher), _pause_mode(false),
            _waiting_list_size(waiting_list_size), _is_enabled(false)
            {}

SyncedImuPublisher::~SyncedImuPublisher()
{
    try
    {
        PublishPendingMessages();
    }
    catch(...){} // Not allowed to throw from Dtor
}

void SyncedImuPublisher::Publish(sensor_msgs::msg::Imu imu_msg)
{
    std::lock_guard<std::mutex> lock_guard(_mutex);
    if (_pause_mode)
    {
        if (_pending_messages.size() >= _waiting_list_size)
        {
            throw std::runtime_error("SyncedImuPublisher inner list reached maximum size of " + std::to_string(_pending_messages.size()));
        }
        _pending_messages.push(imu_msg);
    }
    else
    {
        _publisher->publish(imu_msg);
    }
    return;
}

void SyncedImuPublisher::Pause()
{
    if (!_is_enabled) return;
    std::lock_guard<std::mutex> lock_guard(_mutex);
    _pause_mode = true;
}

void SyncedImuPublisher::Resume()
{
    std::lock_guard<std::mutex> lock_guard(_mutex);
    _pause_mode = false;
    PublishPendingMessages();
}

void SyncedImuPublisher::PublishPendingMessages()
{
    while (!_pending_messages.empty())
    {
        const sensor_msgs::msg::Imu &imu_msg = _pending_messages.front();
        _publisher->publish(imu_msg);
        _pending_messages.pop();
    }
}
size_t SyncedImuPublisher::getNumSubscribers()
{ 
    if (!_publisher) return 0;
    return _publisher->get_subscription_count();
}

BaseRealSenseNode::BaseRealSenseNode(RosNodeBase& node,
                                     rs2::device dev,
                                     std::shared_ptr<Parameters> parameters,
                                     bool use_intra_process) :
    _is_running(true),
    _node(node),
    _logger(node.get_logger()),
    _parameters(parameters),
    _dev(dev),
    _json_file_path(""),
    _depth_scale_meters(0),
    _clipping_distance(0),
    _linear_accel_cov(0),
    _angular_velocity_cov(0),
    _hold_back_imu_for_frames(false),
    _publish_tf(false),
    _tf_publish_rate(TF_PUBLISH_RATE),
    _diagnostics_period(0),
    _use_intra_process(use_intra_process),
    _is_initialized_time_base(false),
    _camera_time_base(0),
    _sync_frames(SYNC_FRAMES),
    _enable_rgbd(ENABLE_RGBD),
    _is_color_enabled(false),
    _is_depth_enabled(false),
    _is_accel_enabled(false),
    _is_gyro_enabled(false),
    _pointcloud(false),
    _imu_sync_method(imu_sync_method::NONE),
    _is_profile_changed(false),
    _is_align_depth_changed(false),
    _safety_sensor(nullptr),
    _stereo_color_publish_rate(-1.0),
    _stereo_color_frame_available(false),
    _stereo_depth_publish_rate(-1.0),
    _stereo_depth_frame_available(false),
    _stereo_pointcloud_frame_available(false),
    _previous_frame_time(0.0)
#if defined (ACCELERATE_GPU_WITH_GLSL)
    ,_app(1280, 720, "RS_GLFW_Window"),
    _accelerate_gpu_with_glsl(false),
    _is_accelerate_gpu_with_glsl_changed(false)
#endif
{
    if ( use_intra_process )
    {
        ROS_INFO("Intra-Process communication enabled");
    }

    // Kiwi added: allow static tf with intra process
    rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options;
    options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    _static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node, tf2_ros::StaticBroadcasterQoS(), options);

    initializeFormatsMaps();
    _monitor_options = {RS2_OPTION_ASIC_TEMPERATURE, RS2_OPTION_PROJECTOR_TEMPERATURE};

    // Kiwibot: TF buffer for the get_coords service, which transforms cached vertices
    // into a caller-provided frame. The TF listener subscribes with TransientLocal/Reliable
    // QoS, which is incompatible with intra-process; disable IPC on this listener so it
    // works whether or not the realsense node is run inside a composable container.
    _buffer_tf2 = std::make_unique<tf2_ros::Buffer>(_node.get_clock());
    rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> tf_sub_options;
    tf_sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    _listener_tf2 = std::make_shared<tf2_ros::TransformListener>(
        *_buffer_tf2, _node.shared_from_this(), true,
        tf2_ros::DynamicListenerQoS(), tf2_ros::StaticListenerQoS(),
        tf_sub_options, tf_sub_options);
}

BaseRealSenseNode::~BaseRealSenseNode()
{
    // Kill dynamic transform thread
    _is_running = false;
    _cv_tf.notify_one();
    if (_tf_t && _tf_t->joinable())
        _tf_t->join();

    _cv_temp.notify_one();
    _cv_mpc.notify_one();
    if (_monitoring_t && _monitoring_t->joinable())
    {
        _monitoring_t->join();
    }
    if (_monitoring_pc && _monitoring_pc->joinable())
    {
        _monitoring_pc->join();
    }
    clearParameters();
    try
    {
        for(auto&& sensor : _available_ros_sensors)
        {
            sensor->stop();
        }
    }
    catch(...){} // Not allowed to throw from Dtor
}

void BaseRealSenseNode::hardwareResetRequest()
{
    ROS_ERROR_STREAM("Performing Hardware Reset.");
    _dev.hardware_reset();
}

void BaseRealSenseNode::publishTopics()
{
    getParameters();
    setup();
    // Kiwi added virtual cam
    if (_color_virtual_cam >= 0 ){
        _virtualcam = new FakeWebcam("/dev/video" + std::to_string(_color_virtual_cam),
        _camera_info[COLOR].width, _camera_info[COLOR].height);
    }

    // Initialize stereo color publish timer if custom rate is enabled
    if (_stereo_color_publish_rate > 0.0)
    {
        ROS_INFO_STREAM("Stereo color publish rate set to " << _stereo_color_publish_rate << " Hz");
        _stereo_color_publish_timer = _node.create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / _stereo_color_publish_rate)),
            std::bind(&BaseRealSenseNode::stereoColorPublishTimerCallback, this)
        );
    }

    // Initialize stereo depth publish timer if custom rate is enabled
    if (_stereo_depth_publish_rate > 0.0)
    {
        ROS_INFO_STREAM("Stereo depth publish rate set to " << _stereo_depth_publish_rate << " Hz");
        _stereo_depth_publish_timer = _node.create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / _stereo_depth_publish_rate)),
            std::bind(&BaseRealSenseNode::stereoDepthPublishTimerCallback, this)
        );
    }

    // Initialize stereo pointcloud publish timer if custom rate is enabled
    if (_stereo_depth_publish_rate > 0.0)
    {
        ROS_INFO_STREAM("Stereo pointcloud publish rate set to " << _stereo_depth_publish_rate << " Hz");
        _stereo_pointcloud_publish_timer = _node.create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / _stereo_depth_publish_rate)),
            std::bind(&BaseRealSenseNode::stereoPointcloudPublishTimerCallback, this)
        );
    }

    ROS_INFO_STREAM("RealSense Node Is Up!");
}

void BaseRealSenseNode::initializeFormatsMaps()
{
    // from rs2_format to OpenCV format
    // https://docs.opencv.org/3.4/d1/d1b/group__core__hal__interface.html
    // https://docs.opencv.org/2.4/modules/core/doc/basic_structures.html
    // CV_<bit-depth>{U|S|F}C(<number_of_channels>)
    // where U is unsigned integer type, S is signed integer type, and F is float type.
    // For example, CV_8UC1 means a 8-bit single-channel array,
    // CV_32FC2 means a 2-channel (complex) floating-point array, and so on.
    _rs_format_to_cv_format[RS2_FORMAT_Y8] = CV_8UC1;
    _rs_format_to_cv_format[RS2_FORMAT_Y16] = CV_16UC1;
    _rs_format_to_cv_format[RS2_FORMAT_Z16] = CV_16UC1;
    _rs_format_to_cv_format[RS2_FORMAT_RGB8] = CV_8UC3;
    _rs_format_to_cv_format[RS2_FORMAT_BGR8] = CV_8UC3;
    _rs_format_to_cv_format[RS2_FORMAT_RGBA8] = CV_8UC4;
    _rs_format_to_cv_format[RS2_FORMAT_BGRA8] = CV_8UC4;
    _rs_format_to_cv_format[RS2_FORMAT_YUYV] = CV_8UC2;
    _rs_format_to_cv_format[RS2_FORMAT_UYVY] = CV_8UC2;
    // _rs_format_to_cv_format[RS2_FORMAT_M420] = not supported yet in ROS2
    _rs_format_to_cv_format[RS2_FORMAT_RAW8] = CV_8UC1;
    _rs_format_to_cv_format[RS2_FORMAT_RAW10] = CV_16UC1;
    _rs_format_to_cv_format[RS2_FORMAT_RAW16] = CV_16UC1;

    // from rs2_format to ROS2 image msg encoding (format)
    // http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html
    // http://docs.ros.org/en/jade/api/sensor_msgs/html/image__encodings_8h_source.html
    _rs_format_to_ros_format[RS2_FORMAT_Y8] = sensor_msgs::image_encodings::MONO8;
    _rs_format_to_ros_format[RS2_FORMAT_Y16] = sensor_msgs::image_encodings::MONO16;
    _rs_format_to_ros_format[RS2_FORMAT_Z16] = sensor_msgs::image_encodings::TYPE_16UC1;
    _rs_format_to_ros_format[RS2_FORMAT_RGB8] = sensor_msgs::image_encodings::RGB8;
    _rs_format_to_ros_format[RS2_FORMAT_BGR8] = sensor_msgs::image_encodings::BGR8;
    _rs_format_to_ros_format[RS2_FORMAT_RGBA8] = sensor_msgs::image_encodings::RGBA8;
    _rs_format_to_ros_format[RS2_FORMAT_BGRA8] = sensor_msgs::image_encodings::BGRA8;
    _rs_format_to_ros_format[RS2_FORMAT_YUYV] = sensor_msgs::image_encodings::YUV422_YUY2;
    _rs_format_to_ros_format[RS2_FORMAT_UYVY] = sensor_msgs::image_encodings::YUV422;
    // _rs_format_to_ros_format[RS2_FORMAT_M420] =  not supported yet in ROS2
    _rs_format_to_ros_format[RS2_FORMAT_RAW8] = sensor_msgs::image_encodings::TYPE_8UC1;
    _rs_format_to_ros_format[RS2_FORMAT_RAW10] = sensor_msgs::image_encodings::TYPE_16UC1;
    _rs_format_to_ros_format[RS2_FORMAT_RAW16] = sensor_msgs::image_encodings::TYPE_16UC1;
}

void BaseRealSenseNode::stereoColorPublishTimerCallback()
{
    std::lock_guard<std::mutex> lock(_stereo_color_frame_mutex);
    if (_stereo_color_frame_available && _latest_stereo_color_frame)
    {
        // Find the color stream publisher
        auto color_publisher_it = _image_publishers.find(COLOR);
        if (color_publisher_it != _image_publishers.end())
        {
            // Update timestamp and move the frame to avoid copying
            _latest_stereo_color_frame->header.stamp = _node.now();
            color_publisher_it->second->publish(std::move(_latest_stereo_color_frame));
        }
    }
}

void BaseRealSenseNode::stereoDepthPublishTimerCallback()
{
    std::lock_guard<std::mutex> lock(_stereo_depth_frame_mutex);
    if (_stereo_depth_frame_available && _latest_stereo_depth_frame)
    {
        // Find the depth stream publisher, the COLOR publisher is the one that is aligned to the rgb image
        auto depth_publisher_it = _depth_aligned_image_publishers.find(COLOR);
        if (depth_publisher_it != _depth_aligned_image_publishers.end())
        {
            // Update timestamp and move the frame to avoid copying
            _latest_stereo_depth_frame->header.stamp = _node.now();
            depth_publisher_it->second->publish(std::move(_latest_stereo_depth_frame));
        }
    }
}

void BaseRealSenseNode::stereoPointcloudPublishTimerCallback()
{
    std::lock_guard<std::mutex> lock(_stereo_pointcloud_frame_mutex);
    if (_stereo_pointcloud_frame_available && _latest_stereo_pointcloud_frame)
    {
        // Get the pointcloud publisher from the pc_filter
        if (_pc_filter && _pc_filter->getPointcloudPublisher())
        {
            // Update timestamp and move the frame to avoid copying
            _latest_stereo_pointcloud_frame->header.stamp = _node.now();
            _pc_filter->getPointcloudPublisher()->publish(std::move(_latest_stereo_pointcloud_frame));
        }
    }
}

void BaseRealSenseNode::setupFilters()
{
    _filters.push_back(std::make_shared<NamedFilter>(std::make_shared<rs2::decimation_filter>(), _parameters, _logger));
    _filters.push_back(std::make_shared<NamedFilter>(std::make_shared<rs2::hdr_merge>(), _parameters, _logger));
    _filters.push_back(std::make_shared<NamedFilter>(std::make_shared<rs2::sequence_id_filter>(), _parameters, _logger));
    _filters.push_back(std::make_shared<NamedFilter>(std::make_shared<rs2::disparity_transform>(), _parameters, _logger));
    _filters.push_back(std::make_shared<NamedFilter>(std::make_shared<rs2::spatial_filter>(), _parameters, _logger));
    _filters.push_back(std::make_shared<NamedFilter>(std::make_shared<rs2::temporal_filter>(), _parameters, _logger));
    _filters.push_back(std::make_shared<NamedFilter>(std::make_shared<rs2::hole_filling_filter>(), _parameters, _logger));
    _filters.push_back(std::make_shared<NamedFilter>(std::make_shared<rs2::disparity_transform>(false), _parameters, _logger));
    _filters.push_back(std::make_shared<NamedFilter>(std::make_shared<rs2::rotation_filter>(std::vector< rs2_stream >{ RS2_STREAM_DEPTH, RS2_STREAM_COLOR, RS2_STREAM_INFRARED }), _parameters, _logger));

    /* 
    update_align_depth_func is being used in the align depth filter for triggiring the thread that monitors profile
    changes (_monitoring_pc) on every disable/enable of the align depth filter. This filter enablement/disablement affects
    several topics creation/destruction, therefore, refreshing the topics is required similarly to what is done when turning on/off a sensor.
    See BaseRealSenseNode::monitoringProfileChanges() as reference.
    */ 
    std::function<void(const rclcpp::Parameter&)> update_align_depth_func = [this](const rclcpp::Parameter&){
        {
            std::lock_guard<std::mutex> lock_guard(_profile_changes_mutex);
            _is_align_depth_changed = true;
        }
        _cv_mpc.notify_one();
    };

#if defined (ACCELERATE_GPU_WITH_GLSL)
    _colorizer_filter = std::make_shared<NamedFilter>(std::make_shared<rs2::gl::colorizer>(), _parameters, _logger); 
    _pc_filter = std::make_shared<PointcloudFilter>(std::make_shared<rs2::gl::pointcloud>(), _node, _parameters, _logger);
#else
    _colorizer_filter = std::make_shared<NamedFilter>(std::make_shared<rs2::colorizer>(), _parameters, _logger);
    _pc_filter = std::make_shared<PointcloudFilter>(std::make_shared<rs2::pointcloud>(), _node, _parameters, _logger);
#endif

    // Apply PointCloud filter before applying Align-depth as it requires original depth image not aligned-depth image.
    _filters.push_back(_pc_filter);

    _align_depth_filter = std::make_shared<AlignDepthFilter>(std::make_shared<rs2::align>(RS2_STREAM_COLOR), update_align_depth_func, _parameters, _logger);
    _filters.push_back(_align_depth_filter);

    // Apply Colorizer filter after applying Align-Depth to get colorized aligned depth image.
    _filters.push_back(_colorizer_filter);
}

cv::Mat& BaseRealSenseNode::fix_depth_scale(const cv::Mat& from_image, cv::Mat& to_image)
{
    static const float meter_to_mm = 0.001f;
    if (fabs(_depth_scale_meters - meter_to_mm) < 1e-6)
    {
        to_image = from_image;
        return to_image;
    }

    if (to_image.size() != from_image.size())
    {
        to_image.create(from_image.rows, from_image.cols, from_image.type());
    }

    CV_Assert(CV_MAKETYPE(from_image.depth(),from_image.channels()) == _rs_format_to_cv_format[RS2_FORMAT_Z16]);

    int nRows = from_image.rows;
    int nCols = from_image.cols;

    if (from_image.isContinuous())
    {
        nCols *= nRows;
        nRows = 1;
    }

    int i,j;
    const uint16_t* p_from;
    uint16_t* p_to;
    for( i = 0; i < nRows; ++i)
    {
        p_from = from_image.ptr<uint16_t>(i);
        p_to = to_image.ptr<uint16_t>(i);
        for ( j = 0; j < nCols; ++j)
        {
            p_to[j] = p_from[j] * _depth_scale_meters / meter_to_mm;
        }
    }
    return to_image;
}

void BaseRealSenseNode::clip_depth(rs2::depth_frame depth_frame, float clipping_dist)
{
    uint16_t* p_depth_frame = reinterpret_cast<uint16_t*>(const_cast<void*>(depth_frame.get_data()));
    uint16_t clipping_value = static_cast<uint16_t>(clipping_dist / _depth_scale_meters);

    int width = depth_frame.get_width();
    int height = depth_frame.get_height();

    #ifdef _OPENMP
    #pragma omp parallel for schedule(dynamic) //Using OpenMP to try to parallelise the loop
    #endif
    for (int y = 0; y < height; y++)
    {
        auto depth_pixel_index = y * width;
        for (int x = 0; x < width; x++, ++depth_pixel_index)
        {
            // Check if the depth value is greater than the threashold
            if (p_depth_frame[depth_pixel_index] > clipping_value)
            {
                p_depth_frame[depth_pixel_index] = 0; //Set to invalid (<=0) value.
            }
        }
    }
}

sensor_msgs::msg::Imu BaseRealSenseNode::CreateUnitedMessage(const CimuData accel_data, const CimuData gyro_data)
{
    sensor_msgs::msg::Imu imu_msg;
    rclcpp::Time t(gyro_data.m_time_ns);  //rclcpp::Time(uint64_t nanoseconds)
    imu_msg.header.stamp = t;

    imu_msg.angular_velocity.x = gyro_data.m_data.x();
    imu_msg.angular_velocity.y = gyro_data.m_data.y();
    imu_msg.angular_velocity.z = gyro_data.m_data.z();

    imu_msg.linear_acceleration.x = accel_data.m_data.x();
    imu_msg.linear_acceleration.y = accel_data.m_data.y();
    imu_msg.linear_acceleration.z = accel_data.m_data.z();
    return imu_msg;
}

template <typename T> T lerp(const T &a, const T &b, const double t) {
  return a * (1.0 - t) + b * t;
}

void BaseRealSenseNode::FillImuData_LinearInterpolation(const CimuData imu_data, std::deque<sensor_msgs::msg::Imu>& imu_msgs)
{
    _imu_history.push_back(imu_data);
    stream_index_pair type(imu_data.m_type);
    imu_msgs.clear();

    if ((type != ACCEL) || _imu_history.size() < 3)
        return;
    
    std::deque<CimuData> gyros_data;
    CimuData accel0, accel1, crnt_imu;

    while (_imu_history.size()) 
    {
        crnt_imu = _imu_history.front();
        _imu_history.pop_front();
        if (!accel0.is_set() && crnt_imu.m_type == ACCEL) 
        {
            accel0 = crnt_imu;
        } 
        else if (accel0.is_set() && crnt_imu.m_type == ACCEL) 
        {
            accel1 = crnt_imu;
            const double dt = accel1.m_time_ns - accel0.m_time_ns;

            while (gyros_data.size())
            {
                CimuData crnt_gyro = gyros_data.front();
                gyros_data.pop_front();
                const double alpha = (crnt_gyro.m_time_ns - accel0.m_time_ns) / dt;
                CimuData crnt_accel(ACCEL, lerp(accel0.m_data, accel1.m_data, alpha), crnt_gyro.m_time_ns);
                imu_msgs.push_back(CreateUnitedMessage(crnt_accel, crnt_gyro));
            }
            accel0 = accel1;
        } 
        else if (accel0.is_set() && crnt_imu.m_time_ns >= accel0.m_time_ns && crnt_imu.m_type == GYRO)
        {
            gyros_data.push_back(crnt_imu);
        }
    }
    _imu_history.push_back(crnt_imu);
    return;
}

void BaseRealSenseNode::FillImuData_Copy(const CimuData imu_data, std::deque<sensor_msgs::msg::Imu>& imu_msgs)
{
    stream_index_pair type(imu_data.m_type);

    if (ACCEL == type)
    {
        _imu_history.clear();
        _imu_history.push_back(imu_data);
        return;
    }

    if (_imu_history.empty())
        return;

    imu_msgs.push_back(CreateUnitedMessage(_imu_history.back(), imu_data));
}

void BaseRealSenseNode::ImuMessage_AddDefaultValues(sensor_msgs::msg::Imu& imu_msg)
{
    imu_msg.header.frame_id = IMU_OPTICAL_FRAME_ID;
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 0.0;

    imu_msg.orientation_covariance = { -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    imu_msg.linear_acceleration_covariance = { _linear_accel_cov, 0.0, 0.0, 0.0, _linear_accel_cov, 0.0, 0.0, 0.0, _linear_accel_cov};
    imu_msg.angular_velocity_covariance = { _angular_velocity_cov, 0.0, 0.0, 0.0, _angular_velocity_cov, 0.0, 0.0, 0.0, _angular_velocity_cov};
}

void BaseRealSenseNode::imu_callback_sync(rs2::frame frame, imu_sync_method sync_method)
{
    std::lock_guard<std::mutex> lock(_imu_callback_mutex);

    auto stream = frame.get_profile().stream_type();
    auto stream_index = (stream == GYRO.first)?GYRO:ACCEL;
    double frame_time = frame.get_timestamp();

    // Kiwibot: collect ACCEL samples for IMU-driven stereo calibration.
    if (stream_index == ACCEL && !_imu_accel_initiated)
    {
        auto accel_reading = *(reinterpret_cast<const float3*>(frame.get_data()));
        _imu_accel_x_vector.push_back(accel_reading.x);
        _imu_accel_y_vector.push_back(accel_reading.y);
        _imu_accel_z_vector.push_back(accel_reading.z);
        if (_imu_accel_x_vector.size() > 30) _imu_accel_initiated = true;
    }

    if (_synced_imu_publisher && (0 != _synced_imu_publisher->getNumSubscribers()))
    {
        auto crnt_reading = *(reinterpret_cast<const float3*>(frame.get_data()));
        Eigen::Vector3d v(crnt_reading.x, crnt_reading.y, crnt_reading.z);
        CimuData imu_data(stream_index, v, frameSystemTimeSec(frame).nanoseconds());
        std::deque<sensor_msgs::msg::Imu> imu_msgs;
        switch (sync_method)
        {
            case imu_sync_method::COPY:
                FillImuData_Copy(imu_data, imu_msgs);
                break;
            case imu_sync_method::LINEAR_INTERPOLATION:
                FillImuData_LinearInterpolation(imu_data, imu_msgs);
                break;
            case imu_sync_method::NONE: //Cannot really be NONE. Just to avoid compilation warning.
                throw std::runtime_error("sync_method in this section can be either COPY or LINEAR_INTERPOLATION");
                break;
        }
        while (imu_msgs.size())
        {
            sensor_msgs::msg::Imu imu_msg = imu_msgs.front();
            ImuMessage_AddDefaultValues(imu_msg);
            _synced_imu_publisher->Publish(imu_msg);
            ROS_DEBUG("Publish united %s stream", rs2_stream_to_string(frame.get_profile().stream_type()));

            // kiwi Added to calculate first accel measurements
            _imu_accel_x_vector.push_back(imu_msg.linear_acceleration.x);
            _imu_accel_y_vector.push_back(imu_msg.linear_acceleration.y);
            _imu_accel_z_vector.push_back(imu_msg.linear_acceleration.z);

            if (_imu_accel_x_vector.size() > 30 )
                _imu_accel_initiated = true;

            imu_msgs.pop_front();
         }
    }
}

std::array<double, 2> BaseRealSenseNode::getImuPitchandRoll() {
    if (_imu_accel_x_vector.empty())
    {
        return {0.0, 0.0};
    }
    double accel_x = std::accumulate(_imu_accel_x_vector.begin(), _imu_accel_x_vector.end(), 0.0) / _imu_accel_x_vector.size();
    double accel_y = std::accumulate(_imu_accel_y_vector.begin(), _imu_accel_y_vector.end(), 0.0) / _imu_accel_y_vector.size();
    double accel_z = std::accumulate(_imu_accel_z_vector.begin(), _imu_accel_z_vector.end(), 0.0) / _imu_accel_z_vector.size();

    // Calculate pitch and roll with imu accel data
    // With respect to our robot 4.0, raw data: y is looking up, z forward and x to the left.
    double x_Buff = accel_z;  // corresponding to /camera/imu z
    double y_Buff = accel_x;  // corresponding to /camera/imu x
    double z_Buff = accel_y;  // corresponding to /camera/imu y

    double pitch = atan2((-x_Buff), sqrt(y_Buff * y_Buff + z_Buff * z_Buff));
    double roll =  atan2(-y_Buff, -z_Buff);    //signs were modified doing tests.

    return {pitch, roll};
}

void BaseRealSenseNode::imu_callback(rs2::frame frame)
{
    auto stream = frame.get_profile().stream_type();

    ROS_DEBUG("Frame arrived: stream: %s ; index: %d ; Timestamp Domain: %s",
                ros_stream_to_string(frame.get_profile().stream_type()).c_str(),
                frame.get_profile().stream_index(),
                rs2_timestamp_domain_to_string(frame.get_frame_timestamp_domain()));

    stream_index_pair stream_index;
    
    if(stream == GYRO.first)
    {
        stream_index = GYRO;
    }
    else if(stream == ACCEL.first)
    {
        stream_index = ACCEL;
    }
    else if(stream == MOTION.first)
    {
        stream_index = MOTION;
    }
    else
    {
        ROS_ERROR("Unknown IMU stream type.");
        return;
    }

    // Kiwibot: collect ACCEL samples for IMU-driven stereo calibration.
    if (stream_index == ACCEL && !_imu_accel_initiated)
    {
        auto accel_reading = *(reinterpret_cast<const float3*>(frame.get_data()));
        _imu_accel_x_vector.push_back(accel_reading.x);
        _imu_accel_y_vector.push_back(accel_reading.y);
        _imu_accel_z_vector.push_back(accel_reading.z);
        if (_imu_accel_x_vector.size() > 30) _imu_accel_initiated = true;
    }

    rclcpp::Time t(frameSystemTimeSec(frame));

    if(_imu_publishers.find(stream_index) == _imu_publishers.end())
    {
        ROS_DEBUG("Received IMU callback while topic does not exist");
        return;
    }

    if (0 != _imu_publishers[stream_index]->get_subscription_count())
    {
        auto imu_msg = sensor_msgs::msg::Imu();
        ImuMessage_AddDefaultValues(imu_msg);
        imu_msg.header.frame_id = OPTICAL_FRAME_ID(stream_index);

        if (MOTION == stream_index)
        {
            auto combined_motion_data = frame.as<rs2::motion_frame>().get_combined_motion_data();

            imu_msg.linear_acceleration.x = combined_motion_data.linear_acceleration.x;
            imu_msg.linear_acceleration.y = combined_motion_data.linear_acceleration.y;
            imu_msg.linear_acceleration.z = combined_motion_data.linear_acceleration.z;

            imu_msg.angular_velocity.x = combined_motion_data.angular_velocity.x;
            imu_msg.angular_velocity.y = combined_motion_data.angular_velocity.y;
            imu_msg.angular_velocity.z = combined_motion_data.angular_velocity.z;

            imu_msg.orientation.x = combined_motion_data.orientation.x;
            imu_msg.orientation.y = combined_motion_data.orientation.y;
            imu_msg.orientation.z = combined_motion_data.orientation.z;
            imu_msg.orientation.w = combined_motion_data.orientation.w;

        }
        else
        {
            auto motion_data = frame.as<rs2::motion_frame>().get_motion_data();
            if (GYRO == stream_index)
            {
                imu_msg.angular_velocity.x = motion_data.x;
                imu_msg.angular_velocity.y = motion_data.y;
                imu_msg.angular_velocity.z = motion_data.z;
            }
            else // ACCEL == stream_index
            {
                imu_msg.linear_acceleration.x = motion_data.x;
                imu_msg.linear_acceleration.y = motion_data.y;
                imu_msg.linear_acceleration.z = motion_data.z;
            }
        }

        imu_msg.header.stamp = t;
        _imu_publishers[stream_index]->publish(imu_msg);
        ROS_DEBUG("Publish %s stream", ros_stream_to_string(frame.get_profile().stream_type()).c_str());
    }
    publishMetadata(frame, t, OPTICAL_FRAME_ID(stream_index));
}

// Kiwibot: re-buffer ACCEL samples and publish (pitch, roll) as a latched Quaternion.
bool BaseRealSenseNode::calibrate_imu_cb(std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                                         std_srvs::srv::Trigger::Response::SharedPtr res)
{
    if (!_is_accel_enabled)
    {
        // Kiwibot: no ACCEL stream to calibrate from (IMU disabled) — fall back to the
        // statically configured STEREO_PITCH_ANGLE/STEREO_ROLL_ANGLE env vars.
        tf2::Quaternion q;
        q.setRPY(_cam_roll, _cam_pitch, _cam_yaw);
        geometry_msgs::msg::Quaternion q_msg = tf2::toMsg(q);
        _cam_imu_angles_publisher->publish(q_msg);

        res->success = true;
        res->message = "Camera angle was calibrated using ENV VAR.";
        return true;
    }

    _imu_accel_initiated = false;
    _imu_accel_x_vector.clear();
    _imu_accel_y_vector.clear();
    _imu_accel_z_vector.clear();

    constexpr int max_wait_ms = 2000;
    int waited_ms = 0;
    while (!_imu_accel_initiated && waited_ms < max_wait_ms)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        waited_ms += 10;
    }
    if (!_imu_accel_initiated)
    {
        res->success = false;
        res->message = "Camera calibration timed out waiting for ACCEL samples";
        return false;
    }

    const auto angles = getImuPitchandRoll();
    const double cam_pitch = angles[0];
    const double cam_roll  = angles[1];
    RCLCPP_INFO(_logger, "Calibrated pitch [deg]: %.3f, roll [deg]: %.3f",
                cam_pitch * 180.0 / M_PI, cam_roll * 180.0 / M_PI);

    tf2::Quaternion q;
    q.setRPY(cam_roll, cam_pitch, 0.0);
    geometry_msgs::msg::Quaternion q_msg = tf2::toMsg(q);
    _cam_imu_angles_publisher->publish(q_msg);

    res->success = true;
    res->message = "PITCH=" + std::to_string(cam_pitch) + " ROLL=" + std::to_string(cam_roll);
    return true;
}

// Kiwibot: hardware-reset Trigger. Kronos remaps this to /stereo/restart.
void BaseRealSenseNode::shutdown_cb(std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                                    std_srvs::srv::Trigger::Response::SharedPtr res)
{
    RCLCPP_WARN(_logger, "Hardware reset requested, resetting device");
    try
    {
        _dev.hardware_reset();
        res->success = true;
        res->message = "Realsense hardware reset issued";
    }
    catch (const std::exception& e)
    {
        res->success = false;
        res->message = std::string("Hardware reset failed: ") + e.what();
    }
}

// Kiwibot: pixel→3D coords lookup. Reports (-1,-1,-1) for stale frames, missing TF,
// out-of-bounds pixels, or pixels with non-positive depth.
void BaseRealSenseNode::get_coords_cb(realsense2_camera_srvs::srv::CoordinateReq::Request::SharedPtr req,
                                      realsense2_camera_srvs::srv::CoordinateReq::Response::SharedPtr res)
{
    constexpr double kMaxAgeSec = 3.0;
    const auto& pixels = req->pixel_requested;

    if (!_pc_filter)
    {
        res->xyz_coordinate.assign(pixels.size(), [](){ geometry_msgs::msg::Point p; p.x=-1; p.y=-1; p.z=-1; return p; }());
        ROS_WARN_STREAM("get_coords called but pointcloud filter is not initialized");
        return;
    }

    std::vector<geometry_msgs::msg::Point> raw_coords;
    std::string source_frame_id;
    rclcpp::Time stamp;
    const bool have_cache = _pc_filter->getCoordsAtPixels(pixels, raw_coords, source_frame_id, stamp);

    auto fill_invalid = [&](){
        res->xyz_coordinate.clear();
        res->xyz_coordinate.reserve(pixels.size());
        for (size_t i = 0; i < pixels.size(); ++i)
        {
            geometry_msgs::msg::Point p;
            p.x = -1.0; p.y = -1.0; p.z = -1.0;
            res->xyz_coordinate.push_back(p);
        }
    };

    if (!have_cache)
    {
        ROS_WARN_STREAM("get_coords called before any pointcloud frame has been cached");
        fill_invalid();
        return;
    }

    {
        const int64_t now_ns = _node.now().nanoseconds();
        const int64_t stamp_ns = stamp.nanoseconds();
        const int64_t max_age_ns = static_cast<int64_t>(kMaxAgeSec * 1e9);
        if ((now_ns - stamp_ns) > max_age_ns)
        {
            ROS_WARN_STREAM("get_coords: cached pointcloud is stale (>"
                            << kMaxAgeSec << "s); returning invalid points");
            fill_invalid();
            return;
        }
    }

    geometry_msgs::msg::TransformStamped transform;
    bool transform_available = true;
    try
    {
        transform = _buffer_tf2->lookupTransform(req->frame, source_frame_id, tf2::TimePointZero);
    }
    catch (const tf2::TransformException& ex)
    {
        ROS_WARN_STREAM("get_coords: TF lookup '" << source_frame_id << "' -> '" << req->frame
                                                  << "' failed: " << ex.what());
        transform_available = false;
    }

    res->xyz_coordinate.clear();
    res->xyz_coordinate.reserve(raw_coords.size());
    for (const auto& raw : raw_coords)
    {
        geometry_msgs::msg::Point out_point;
        if (raw.x < 0.0 && raw.y < 0.0 && raw.z < 0.0)
        {
            // sentinel from PointcloudFilter: invalid pixel or non-positive depth
            out_point.x = -1.0; out_point.y = -1.0; out_point.z = -1.0;
        }
        else if (!transform_available)
        {
            out_point.x = -1.0; out_point.y = -1.0; out_point.z = -1.0;
        }
        else
        {
            geometry_msgs::msg::PointStamped ps_in, ps_out;
            ps_in.header.frame_id = source_frame_id;
            ps_in.header.stamp = stamp;
            ps_in.point = raw;
            tf2::doTransform(ps_in, ps_out, transform);
            out_point = ps_out.point;
        }
        res->xyz_coordinate.push_back(out_point);
    }
}


// Kiwibot: 3D point→pixel projection using COLOR camera intrinsics. Caller's points may be
// in any TF frame; we transform them to camera_color_optical_frame and apply pinhole projection.
// Pixel.z is the depth in meters in the optical frame (so the caller can sanity-check distance).
void BaseRealSenseNode::get_pixel_cb(realsense2_camera_srvs::srv::PixelReq::Request::SharedPtr req,
                                     realsense2_camera_srvs::srv::PixelReq::Response::SharedPtr res)
{
    res->pixels.clear();
    if (req->points_requested.empty())
    {
        ROS_WARN_STREAM("get_pixel called with empty points_requested");
        return;
    }

    auto color_it = _camera_info.find(COLOR);
    if (color_it == _camera_info.end())
    {
        ROS_WARN_STREAM("get_pixel called but COLOR camera_info is not yet available");
        res->pixels.assign(req->points_requested.size(), [](){
            geometry_msgs::msg::Point p; p.x = -1; p.y = -1; p.z = -1; return p;
        }());
        return;
    }
    const auto& msg_camera_info = color_it->second;
    const std::string target_frame = OPTICAL_FRAME_ID(COLOR);

    geometry_msgs::msg::TransformStamped transform;
    bool transform_available = true;
    try
    {
        transform = _buffer_tf2->lookupTransform(target_frame,
                                                 req->points_requested.front().header.frame_id,
                                                 tf2::TimePointZero);
    }
    catch (const tf2::TransformException& ex)
    {
        ROS_WARN_STREAM("get_pixel: TF lookup '" << req->points_requested.front().header.frame_id
                                                  << "' -> '" << target_frame << "' failed: " << ex.what());
        transform_available = false;
    }

    res->pixels.reserve(req->points_requested.size());
    for (const auto& point : req->points_requested)
    {
        geometry_msgs::msg::Point pixel;
        if (!transform_available)
        {
            pixel.x = -1.0; pixel.y = -1.0; pixel.z = -1.0;
        }
        else
        {
            geometry_msgs::msg::PointStamped transformed;
            tf2::doTransform(point, transformed, transform);
            // Pinhole projection. Add a tiny epsilon to avoid division-by-zero at z=0.
            pixel.x = (msg_camera_info.k[0] * transformed.point.x) / (transformed.point.z + 1e-5)
                      + msg_camera_info.k[2];
            pixel.y = (msg_camera_info.k[4] * transformed.point.y) / (transformed.point.z + 1e-5)
                      + msg_camera_info.k[5];
            pixel.z = transformed.point.z;
        }
        res->pixels.push_back(pixel);
    }
}


void BaseRealSenseNode::frame_callback(rs2::frame frame)
{
    if (_synced_imu_publisher)
        _synced_imu_publisher->Pause();

    double frame_time = frame.get_timestamp();
    (void)frame_time;

    rclcpp::Time t(frameSystemTimeSec(frame));
    if (frame.is<rs2::frameset>())
    {
        ROS_DEBUG("Frameset arrived.");
        auto frameset = frame.as<rs2::frameset>();
        ROS_DEBUG("List of frameset before applying filters: size: %d", static_cast<int>(frameset.size()));
        for (auto it = frameset.begin(); it != frameset.end(); ++it)
        {
            auto f = (*it);
            auto stream_type = f.get_profile().stream_type();
            auto stream_index = f.get_profile().stream_index();
            auto stream_format = f.get_profile().format();
            auto stream_unique_id = f.get_profile().unique_id();

            ROS_DEBUG("Frameset contain (%s, %d, %s %d) frame. frame_number: %llu ; frame_TS: %f ; ros_TS(NSec): %lu",
                        rs2_stream_to_string(stream_type), stream_index, rs2_format_to_string(stream_format), stream_unique_id, frame.get_frame_number(), frame_time, t.nanoseconds());
        }
        // Clip depth_frame for max range:
        rs2::depth_frame original_depth_frame = frameset.get_depth_frame();
        if (original_depth_frame && _clipping_distance > 0)
        {
            clip_depth(original_depth_frame, _clipping_distance);
        }

        rs2::video_frame original_color_frame = frameset.get_color_frame();
        rs2::video_frame original_infra2_frame = frameset.get_infrared_frame(2);

        ROS_DEBUG("num_filters: %d", static_cast<int>(_filters.size()));
        for (auto filter_it : _filters)
        {
            frameset = filter_it->Process(frameset);
        }

        ROS_DEBUG("List of frameset after applying filters: size: %d", static_cast<int>(frameset.size()));
        bool sent_depth_frame(false);
        for (auto it = frameset.begin(); it != frameset.end(); ++it)
        {
            auto f = (*it);
            auto stream_type = f.get_profile().stream_type();
            auto stream_index = f.get_profile().stream_index();
            auto stream_format = f.get_profile().format();
            stream_index_pair sip{stream_type,stream_index};

            ROS_DEBUG("Frameset contain (%s, %d, %s) frame. frame_number: %llu ; frame_TS: %f ; ros_TS(NSec): %lu", 
                rs2_stream_to_string(stream_type), stream_index, rs2_format_to_string(stream_format), f.get_frame_number(), frame_time, t.nanoseconds());
            if (f.is<rs2::video_frame>())
                ROS_DEBUG_STREAM("frame: " << f.as<rs2::video_frame>().get_width() << " x " << f.as<rs2::video_frame>().get_height());

            if (f.is<rs2::labeled_points>())
            {
                publishLabeledPointCloud(f.as<rs2::labeled_points>(), t);
                publishMetadata(f, t, OPTICAL_FRAME_ID(sip));
            }
            else if (f.is<rs2::points>())
            {
                publishPointCloud(f.as<rs2::points>(), t, frameset);
            }
            else if(stream_type == RS2_STREAM_OCCUPANCY)
            {
                publishOccupancyFrame(f, t);
            }
            else
            {
                if (stream_type == RS2_STREAM_DEPTH)
                {
                    if (sent_depth_frame) continue;
                    sent_depth_frame = true;
                    if (original_color_frame && _align_depth_filter->is_enabled())
                    {
                        publishFrame(f, t, COLOR, _depth_aligned_image, _depth_aligned_info_publisher, _depth_aligned_image_publishers, false);
                        continue;
                    }
                    if (original_infra2_frame && _align_depth_filter->is_enabled())
                    {
                        publishFrame(f, t, INFRA2, _depth_aligned_image, _depth_aligned_info_publisher, _depth_aligned_image_publishers, false);
                        continue;
                    }
                }
                publishFrame(f, t, sip, _images, _info_publishers, _image_publishers);
            }
        }
        if (original_depth_frame && _align_depth_filter->is_enabled())
        {
            rs2::frame frame_to_send;
            if (_colorizer_filter->is_enabled())
                frame_to_send = _colorizer_filter->Process(original_depth_frame);
            else
                frame_to_send = original_depth_frame;
            publishFrame(frame_to_send, t, DEPTH, _images, _info_publishers, _image_publishers);

            // Publish RGBD only if rgbd enabled and both depth and color frames exist.
            // On this line we already know original_depth_frame is valid.
            if(_enable_rgbd && original_color_frame)
            {
                auto color_format = original_color_frame.get_profile().format();
                auto depth_format = original_depth_frame.get_profile().format();
                publishRGBD(_images[COLOR], color_format, _depth_aligned_image[COLOR], depth_format, t);
            }  
        }
    }
    else if (frame.is<rs2::video_frame>())
    {
        auto stream_type = frame.get_profile().stream_type();
        auto stream_index = frame.get_profile().stream_index();
        ROS_DEBUG("Single video frame arrived (%s, %d). frame_number: %llu ; frame_TS: %f ; ros_TS(NSec): %lu",
                    rs2_stream_to_string(stream_type), stream_index, frame.get_frame_number(), frame_time, t.nanoseconds());
            
        stream_index_pair sip{stream_type,stream_index};
        if(stream_type == RS2_STREAM_OCCUPANCY)
        {
            publishOccupancyFrame(frame, t);
        }
        else 
        {
            if (frame.is<rs2::depth_frame>())
            {
                if (_clipping_distance > 0)
                {
                    clip_depth(frame, _clipping_distance);
                }
            }
            publishFrame(frame, t, sip, _images, _info_publishers, _image_publishers);
        }
    }
    else if (frame.is<rs2::labeled_points>())
    {
        auto stream_type = frame.get_profile().stream_type();
        auto stream_index = frame.get_profile().stream_index();
        stream_index_pair sip{stream_type,stream_index};
        ROS_DEBUG("Single labeled point cloud frame arrived (%s, %d). frame_number: %llu ; frame_TS: %f ; ros_TS(NSec): %lu",
                    rs2_stream_to_string(stream_type), stream_index, frame.get_frame_number(), frame_time, t.nanoseconds());
        publishLabeledPointCloud(frame.as<rs2::labeled_points>(), t);
        publishMetadata(frame, t, OPTICAL_FRAME_ID(sip));
    }
    if (_synced_imu_publisher)
        _synced_imu_publisher->Resume();
} // frame_callback

void BaseRealSenseNode::multiple_message_callback(rs2::frame frame, imu_sync_method sync_method)
{
    auto stream = frame.get_profile().stream_type();
    switch (stream)
    {
        case RS2_STREAM_GYRO:
        case RS2_STREAM_ACCEL:
            if (sync_method > imu_sync_method::NONE) imu_callback_sync(frame, sync_method);
            else imu_callback(frame);
            break;
        default:
            frame_callback(frame);
    }
}

uint64_t BaseRealSenseNode::millisecondsToNanoseconds(double timestamp_ms)
{
        // modf breaks input into an integral and fractional part
        double int_part_ms, fract_part_ms;
        fract_part_ms = modf(timestamp_ms, &int_part_ms);

        //convert both parts to ns
        static constexpr uint64_t milli_to_nano = 1000000;
        uint64_t int_part_ns = static_cast<uint64_t>(int_part_ms) * milli_to_nano;
        uint64_t fract_part_ns = static_cast<uint64_t>(std::round(fract_part_ms * milli_to_nano));

        return int_part_ns + fract_part_ns;
}

rclcpp::Time BaseRealSenseNode::frameSystemTimeSec(rs2::frame frame)
{
    double timestamp_ms = frame.get_timestamp();
    if (frame.get_frame_timestamp_domain() == RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK)
    {
        std::lock_guard<std::mutex> lock(_time_base_mutex);
        if (!_is_initialized_time_base)
        {
            ROS_WARN("frame's time domain is HARDWARE_CLOCK. Timestamps may reset periodically.");
            _ros_time_base = _node.now();
            _camera_time_base = timestamp_ms;
            _previous_frame_time = timestamp_ms;
            _is_initialized_time_base = true;
        }
        else if (_previous_frame_time > timestamp_ms)
        {
            ROS_WARN("Hardware clock reset detected. Resetting ROS time base.");
            _ros_time_base = _node.now();
            _camera_time_base = timestamp_ms;
        }
        _previous_frame_time = timestamp_ms;

        double elapsed_camera_ns = (/*ms*/ timestamp_ms - /*ms*/ _camera_time_base) * 1e6;

        /*
        Fixing deprecated-declarations compilation warning.
        Duration(rcl_duration_value_t) is deprecated in favor of
        static Duration::from_nanoseconds(rcl_duration_value_t)
        starting from GALAXY.
        */
#if defined(FOXY)
        auto duration = rclcpp::Duration(elapsed_camera_ns);
#else
        auto duration = rclcpp::Duration::from_nanoseconds(elapsed_camera_ns);
#endif
        return rclcpp::Time(_ros_time_base + duration);
    }
    else
    {
        return rclcpp::Time(millisecondsToNanoseconds(timestamp_ms));
    }
}

void BaseRealSenseNode::updateProfilesStreamCalibData(const std::vector<rs2::stream_profile>& profiles)
{
    std::shared_ptr<rs2::stream_profile> left_profile;
    std::shared_ptr<rs2::stream_profile> right_profile;
    for (auto& profile : profiles)
    {
        if (profile.is<rs2::video_stream_profile>())
        {
            updateStreamCalibData(profile.as<rs2::video_stream_profile>());

            // stream index: 1=left, 2=right
            if (profile.stream_index() == 1) { left_profile = std::make_shared<rs2::stream_profile>(profile); }
            if (profile.stream_index() == 2) { right_profile = std::make_shared<rs2::stream_profile>(profile);  }
        }
    }
    if (left_profile && right_profile) {
        updateExtrinsicsCalibData(left_profile->as<rs2::video_stream_profile>(), right_profile->as<rs2::video_stream_profile>());
    }
}

void BaseRealSenseNode::updateStreamCalibData(const rs2::video_stream_profile& video_profile)
{
    stream_index_pair stream_index{video_profile.stream_type(), video_profile.stream_index()};

    rs2_intrinsics intrinsic;
    try
    {
        intrinsic = video_profile.get_intrinsics();
    }
    catch(const std::exception& ex)
    {
        // e.g. infra1/infra2 in Y16i format (calibration mode) doesn't have intrinsics.
        ROS_WARN_STREAM("No intrinsics available for this stream profile. Using zeroed intrinsics as default.");
        intrinsic = { 0, 0, 0, 0, 0, 0, RS2_DISTORTION_NONE ,{ 0,0,0,0,0 } };
    }

    _camera_info[stream_index].width = intrinsic.width;
    _camera_info[stream_index].height = intrinsic.height;
    _camera_info[stream_index].header.frame_id = OPTICAL_FRAME_ID(stream_index);

    _camera_info[stream_index].k.at(0) = intrinsic.fx;
    _camera_info[stream_index].k.at(2) = intrinsic.ppx;
    _camera_info[stream_index].k.at(4) = intrinsic.fy;
    _camera_info[stream_index].k.at(5) = intrinsic.ppy;
    _camera_info[stream_index].k.at(8) = 1;

    _camera_info[stream_index].p.at(0) = _camera_info[stream_index].k.at(0);
    _camera_info[stream_index].p.at(1) = 0;
    _camera_info[stream_index].p.at(2) = _camera_info[stream_index].k.at(2);
    _camera_info[stream_index].p.at(3) = 0;
    _camera_info[stream_index].p.at(4) = 0;
    _camera_info[stream_index].p.at(5) = _camera_info[stream_index].k.at(4);
    _camera_info[stream_index].p.at(6) = _camera_info[stream_index].k.at(5);
    _camera_info[stream_index].p.at(7) = 0;
    _camera_info[stream_index].p.at(8) = 0;
    _camera_info[stream_index].p.at(9) = 0;
    _camera_info[stream_index].p.at(10) = 1;
    _camera_info[stream_index].p.at(11) = 0;

    // set R (rotation matrix) values to identity matrix
    _camera_info[stream_index].r.at(0) = 1.0;
    _camera_info[stream_index].r.at(1) = 0.0;
    _camera_info[stream_index].r.at(2) = 0.0;
    _camera_info[stream_index].r.at(3) = 0.0;
    _camera_info[stream_index].r.at(4) = 1.0;
    _camera_info[stream_index].r.at(5) = 0.0;
    _camera_info[stream_index].r.at(6) = 0.0;
    _camera_info[stream_index].r.at(7) = 0.0;
    _camera_info[stream_index].r.at(8) = 1.0;

    int coeff_size(5);
    if (intrinsic.model == RS2_DISTORTION_KANNALA_BRANDT4)
    {
        _camera_info[stream_index].distortion_model = "equidistant";
        coeff_size = 4;
    } else {
        _camera_info[stream_index].distortion_model = "plumb_bob";
    }

    _camera_info[stream_index].d.resize(coeff_size);
    for (int i = 0; i < coeff_size; i++)
    {
        _camera_info[stream_index].d.at(i) = intrinsic.coeffs[i];
    }

    if (stream_index == DEPTH && _enable[DEPTH] && _enable[COLOR])
    {
        _camera_info[stream_index].p.at(3) = 0;     // Tx
        _camera_info[stream_index].p.at(7) = 0;     // Ty
    }
}

void BaseRealSenseNode::updateExtrinsicsCalibData(const rs2::video_stream_profile& left_video_profile, const rs2::video_stream_profile& right_video_profile)
{
    stream_index_pair left{left_video_profile.stream_type(), left_video_profile.stream_index()};
    stream_index_pair right{right_video_profile.stream_type(), right_video_profile.stream_index()};

    float fx = _camera_info[right].k.at(0);
    float fy = _camera_info[right].k.at(4);
    const auto& ex = right_video_profile.get_extrinsics_to(left_video_profile);
    _camera_info[right].header.frame_id = OPTICAL_FRAME_ID(left);
    _camera_info[right].p.at(3) = -fx * ex.translation[0] + 0.0; // Tx - avoid -0.0 values.
    _camera_info[right].p.at(7) = -fy * ex.translation[1] + 0.0; // Ty - avoid -0.0 values.
}

void BaseRealSenseNode::SetBaseStream()
{
    const std::vector<stream_index_pair> base_stream_priority = {DEPTH};
    std::set<stream_index_pair> checked_sips;
    std::map<stream_index_pair, rs2::stream_profile> available_profiles;
    for(auto&& sensor : _available_ros_sensors)
    {
        for (auto& profile : sensor->get_stream_profiles())
        {
            stream_index_pair sip(profile.stream_type(), profile.stream_index());
            if (available_profiles.find(sip) != available_profiles.end())
                continue;
            available_profiles[sip] = profile;
        }
    }
    
    std::vector<stream_index_pair>::const_iterator base_stream(base_stream_priority.begin());
    while((base_stream != base_stream_priority.end()) && (available_profiles.find(*base_stream) == available_profiles.end()))
    {
        base_stream++;
    }
    if (base_stream == base_stream_priority.end())
    {
        throw std::runtime_error("No known base_stream found for transformations.");
    }
    ROS_DEBUG_STREAM("SELECTED BASE:" << base_stream->first << ", " << base_stream->second);

    _base_profile = available_profiles[*base_stream];
}

void BaseRealSenseNode::publishPointCloud(rs2::points pc, const rclcpp::Time& t, const rs2::frameset& frameset)
{
    // Kiwibot: pointcloud cadence follows the depth-throttle setting; suppress full-rate publishes
    // (and the cache update inside PointcloudFilter::Publish) when throttling is on.
    if (!shouldPublishStream(_stereo_depth_publish_rate, _last_pointcloud_publish_ns))
    {
        return;
    }
    // Match iron's behavior: rewrite stamp to publish-time when throttling is active so consumers
    // see the cadence rather than the (stale) sensor capture time.
    const rclcpp::Time pub_t = (_stereo_depth_publish_rate > 0.0) ? _node.now() : t;
    std::string frame_id = OPTICAL_FRAME_ID(DEPTH);
    _pc_filter->Publish(pc, pub_t, frameset, frame_id, true);
}

// Kiwibot: stream throttle. Returns true if rate<=0 (no throttling) or enough time has elapsed
// since the last publish for that stream. The atomic load/store is intentionally lock-free:
// if two threads race they may both pass once, but the next publish time is set to the latest.
// For "publish at most this rate" semantics that's accurate enough.
bool BaseRealSenseNode::shouldPublishStream(double rate, std::atomic<int64_t>& last_ns)
{
    if (rate <= 0.0)
    {
        return true;
    }
    const int64_t now_ns = _node.now().nanoseconds();
    const int64_t period_ns = static_cast<int64_t>(1.0e9 / rate);
    const int64_t prev = last_ns.load(std::memory_order_relaxed);
    if (now_ns - prev < period_ns)
    {
        return false;
    }
    last_ns.store(now_ns, std::memory_order_relaxed);
    return true;
}

bool BaseRealSenseNode::shouldPublishCameraInfo(const stream_index_pair& sip)
{
    const rs2_stream stream = sip.first;
    return (stream != RS2_STREAM_SAFETY && stream != RS2_STREAM_OCCUPANCY && stream != RS2_STREAM_LABELED_POINT_CLOUD);
}

void BaseRealSenseNode::publishOccupancyFrame(rs2::frame f, const rclcpp::Time& t)
{
    if(!_occupancy_publisher || 0 == _occupancy_publisher->get_subscription_count())
        return;

    ROS_DEBUG("Publishing Occupancy GridCells Frame");

    // get frame bytes and frame metadata relevant info
    auto frame_as_uint8_arr = (uint8_t*)f.get_data();
    auto cols = static_cast<int>(f.get_frame_metadata(RS2_FRAME_METADATA_OCCUPANCY_GRID_COLUMNS)); // grid cells width
    auto rows = static_cast<int>(f.get_frame_metadata(RS2_FRAME_METADATA_OCCUPANCY_GRID_ROWS)); // grid cells height
    auto cell_size = static_cast<float>(f.get_frame_metadata(RS2_FRAME_METADATA_OCCUPANCY_CELL_SIZE) / 100.0f); // convert to meters

    // create GridCells msg and start filling it
    nav_msgs::msg::GridCells msg;
    msg.header.stamp = t;
    msg.header.frame_id = FRAME_ID(OCCUPANCY);
    msg.cell_width = cell_size;
    msg.cell_height = cell_size;

    for (auto i = 0; i < cols * rows; ++i)
    {
        // AICV algo is packing each 8 cells into one byte. Each byte include 8 bits <--> 8 cells
        // The rightest bit (LSB) inside the packed byte from AICV algo represnts the closest cell we want to work with in the grid.
        // e.g. Original Occupancy Cells: 0 0 1 1 0 0 1 0 ---> AICV packing algo ---> 01001100 (not the opposite order)
        // In this if we check if current cell is occupied.
        // Note that we start working from the most left bit, aka, the farest point of the grid.
        if ((frame_as_uint8_arr[i / 8U] & (1U << i % 8)) != 0)
        {
            // Find x,y,z positions of current index
            // Remember, in ROS CS: (X: Forward, Y: Left, Z: Up)
            geometry_msgs::msg::Point p3d;
            uint32_t row = (i / cols);
            uint32_t col = (i % cols);
            p3d.x = (cell_size * static_cast<float>(rows)) - cell_size * (static_cast<float>(row) + 0.5f);
            p3d.y = (cell_size * static_cast<float>(cols)) / 2 - cell_size * (static_cast<float>(col) + 0.5f);
            p3d.z = 0;
            msg.cells.push_back(p3d);
        }
    }
    _occupancy_publisher->publish(msg);
}

void BaseRealSenseNode::publishLabeledPointCloud(rs2::labeled_points lpc, const rclcpp::Time& t)
{
    if(!_labeled_pointcloud_publisher || 0 == _labeled_pointcloud_publisher->get_subscription_count())
        return;
    
    ROS_DEBUG("Publishing Labeled Point Cloud Frame");

    // Create the PointCloud message
    sensor_msgs::msg::PointCloud2::UniquePtr msg_pointcloud = std::make_unique<sensor_msgs::msg::PointCloud2>();

    // Define the fields of the PointCloud message
    sensor_msgs::PointCloud2Modifier modifier(*msg_pointcloud);

    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                "label", 1, sensor_msgs::msg::PointField::UINT8);
    modifier.resize(lpc.size());

    // Fill the PointCloud message with data
    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg_pointcloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg_pointcloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*msg_pointcloud, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_label(*msg_pointcloud, "label");
    const rs2::vertex* vertex = lpc.get_vertices();
    const uint8_t* label = lpc.get_labels();
    
    msg_pointcloud->width = lpc.get_width();
    msg_pointcloud->height = lpc.get_height();
    msg_pointcloud->point_step = lpc.get_bits_per_pixel() / 8;
    msg_pointcloud->row_step = msg_pointcloud->width * msg_pointcloud->point_step;
    msg_pointcloud->data.resize(msg_pointcloud->height * msg_pointcloud->row_step);

    for (size_t point_idx=0; point_idx < lpc.size(); point_idx++, vertex++, label++)
    {
        *iter_x = vertex->x;
        *iter_y = vertex->y;
        *iter_z = vertex->z;
        *iter_label = *label;
        ++iter_x; ++iter_y; ++iter_z; ++iter_label;
    }

    msg_pointcloud->header.stamp = t;
    msg_pointcloud->header.frame_id = FRAME_ID(LABELED_POINT_CLOUD);

    // Publish the PointCloud message
    _labeled_pointcloud_publisher->publish(std::move(msg_pointcloud));
}


Extrinsics BaseRealSenseNode::rsExtrinsicsToMsg(const rs2_extrinsics& extrinsics) const
{
    Extrinsics extrinsicsMsg;
    for (int i = 0; i < 9; ++i)
    {
        extrinsicsMsg.rotation[i] = extrinsics.rotation[i];
        if (i < 3)
            extrinsicsMsg.translation[i] = extrinsics.translation[i];
    }
    return extrinsicsMsg;
}

IMUInfo BaseRealSenseNode::getImuInfo(const rs2::stream_profile& profile)
{
    IMUInfo info{};
    auto sp = profile.as<rs2::motion_stream_profile>();
    rs2_motion_device_intrinsic imuIntrinsics;
    try
    {
        imuIntrinsics = sp.get_motion_intrinsics();
    }
    catch(const std::runtime_error &ex)
    {
        ROS_DEBUG_STREAM("No Motion Intrinsics available.");
        imuIntrinsics = {{{1,0,0,0},{0,1,0,0},{0,0,1,0}}, {0,0,0}, {0,0,0}};
    }

    auto index = 0;
    stream_index_pair sip(profile.stream_type(), profile.stream_index());
    info.header.frame_id = OPTICAL_FRAME_ID(sip);
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            info.data[index] = imuIntrinsics.data[i][j];
            ++index;
        }
        info.noise_variances[i] =  imuIntrinsics.noise_variances[i];
        info.bias_variances[i] = imuIntrinsics.bias_variances[i];
    }
    return info;
}

bool BaseRealSenseNode::fillROSImageMsgAndReturnStatus(
    const cv::Mat& cv_matrix_image,
    const stream_index_pair& stream,
    unsigned int width,
    unsigned int height,
    const rs2_format& stream_format,
    const rclcpp::Time& t,
    sensor_msgs::msg::Image* img_msg_ptr)
{
    if (cv_matrix_image.empty())
    {
        ROS_ERROR_STREAM("cv::Mat is empty. Ignoring this frame.");
        return false;
    }
    else if (_rs_format_to_ros_format.find(stream_format) == _rs_format_to_ros_format.end())
    {
        ROS_ERROR_STREAM("Format " << rs2_format_to_string(stream_format) << " is not supported in ROS2 image messages"
                                   << "Please try different format of this stream.");
        return false;
    }
    // Convert the CV::Mat into a ROS image message (1 copy is done here)
    cv_bridge::CvImage(std_msgs::msg::Header(), _rs_format_to_ros_format[stream_format], cv_matrix_image).toImageMsg(*img_msg_ptr);

    // Convert OpenCV Mat to ROS Image
    img_msg_ptr->header.frame_id = OPTICAL_FRAME_ID(stream);
    img_msg_ptr->header.stamp = t;
    img_msg_ptr->height = height;
    img_msg_ptr->width = width;
    img_msg_ptr->is_bigendian = false;
    img_msg_ptr->step = width * cv_matrix_image.elemSize();
    return true;
}

bool BaseRealSenseNode::fillCVMatImageAndReturnStatus(
    rs2::frame& frame,
    std::map<stream_index_pair, cv::Mat>& images,
    unsigned int width,
    unsigned int height,
    const stream_index_pair& stream)
{
    auto& image = images[stream];
    auto stream_format = frame.get_profile().format();

    if (_rs_format_to_cv_format.find(stream_format) == _rs_format_to_cv_format.end())
    {
        ROS_ERROR_STREAM("Format " << rs2_format_to_string(stream_format) << " is not supported in realsense2_camera node."
                                   << "\nPlease try different format of this stream.");
        return false;
    }
    // we try to reduce image creation as much we can, so we check if the same image structure
    // was already created before, and we fill this image next with the frame data
    // image.create() should be called once per <stream>_<profile>_<format>
    if (image.size() != cv::Size(width, height) || CV_MAKETYPE(image.depth(), image.channels()) != _rs_format_to_cv_format[stream_format])
    {
        image.create(height, width, _rs_format_to_cv_format[stream_format]);
    }

    image.data = (uint8_t*)frame.get_data();

    if (frame.is<rs2::depth_frame>())
    {
        image = fix_depth_scale(image, _depth_scaled_image[stream]);
    }

    return true;
}

void BaseRealSenseNode::publishFrame(
    rs2::frame f,
    const rclcpp::Time& frame_t,
    const stream_index_pair& stream,
    std::map<stream_index_pair, cv::Mat>& images,
    const std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr>& info_publishers,
    const std::map<stream_index_pair, std::shared_ptr<image_publisher>>& image_publishers,
    const bool is_publishMetadata)
{
    ROS_DEBUG("publishFrame(...)");

    // Kiwibot: null/empty frames can arrive briefly after a sensor stop/start cycle.
    if (!f) return;

    // Kiwibot: optional throttling. Color path is stream==COLOR with a non-depth frame.
    // Depth-aligned-to-color path is stream==COLOR with a depth frame (set up at line ~900).
    // When throttling is active for the stream, rewrite the stamp to publish-time so consumers
    // see the cadence rather than the sensor capture time (matches iron's c33effef behavior).
    rclcpp::Time t = frame_t;
    if (stream == COLOR)
    {
        const bool is_aligned_depth = f.is<rs2::depth_frame>();
        const double rate = is_aligned_depth ? _stereo_depth_publish_rate : _stereo_color_publish_rate;
        std::atomic<int64_t>& last_ns = is_aligned_depth ? _last_depth_publish_ns : _last_color_publish_ns;
        if (!shouldPublishStream(rate, last_ns)) return;
        if (rate > 0.0) t = _node.now();
    }

    unsigned int width = 0;
    unsigned int height = 0;
    auto stream_format = RS2_FORMAT_ANY;
    if (f.is<rs2::video_frame>())
    {
        auto timage = f.as<rs2::video_frame>();
        if(stream.first == RS2_STREAM_OCCUPANCY)
        {
            if (!f.supports_frame_metadata(RS2_FRAME_METADATA_OCCUPANCY_GRID_ROWS) ||
                !f.supports_frame_metadata(RS2_FRAME_METADATA_OCCUPANCY_GRID_COLUMNS))
                throw std::runtime_error("Occupancy rows / columns could not be read from frame metadata");

            width = static_cast<int>(f.get_frame_metadata(RS2_FRAME_METADATA_OCCUPANCY_GRID_COLUMNS));
            height = static_cast<int>(f.get_frame_metadata(RS2_FRAME_METADATA_OCCUPANCY_GRID_ROWS));
        }
        else
        {
            width = timage.get_width();
            height = timage.get_height();
        }
        stream_format = timage.get_profile().format();
        if (width == 0 || height == 0) return;
        if (!f.get_data()) return;
    }
    else
    {
        ROS_ERROR("f.is<rs2::video_frame>() check failed. Frame was dropped.");
        return;
    }

    // Publish stream image
    if (image_publishers.find(stream) != image_publishers.end())
    {
        auto &image_publisher = image_publishers.at(stream);
        cv::Mat image_cv_matrix;

        // if rgbd has subscribers we fetch the CV image here
        if (_rgbd_publisher && 0 != _rgbd_publisher->get_subscription_count())
        {
            if (fillCVMatImageAndReturnStatus(f, images, width, height, stream))
            {
                image_cv_matrix = images[stream];
            }
        }

        // if depth/color has subscribers, ask first if rgbd already fetched
        // the images from the frame. if not, fetch the relevant color/depth image.
        if (0 != image_publisher->get_subscription_count())
        {
            if (image_cv_matrix.empty() && fillCVMatImageAndReturnStatus(f, images, width, height, stream))
            {
                image_cv_matrix = images[stream];
            }

            // Prepare image topic to be published
            // We use UniquePtr for allow intra-process publish when subscribers of that type are available
            sensor_msgs::msg::Image::UniquePtr img_msg_ptr(new sensor_msgs::msg::Image());
            if (!img_msg_ptr)
            {
                ROS_ERROR("Sensor image message allocation failed. Frame was dropped.");
                return;
            }

            if (fillROSImageMsgAndReturnStatus(image_cv_matrix, stream, width, height, stream_format, t, img_msg_ptr.get()))
            {

                // Transfer the unique pointer ownership to the RMW
                sensor_msgs::msg::Image *msg_address = img_msg_ptr.get();
                image_publisher->publish(std::move(img_msg_ptr));

                ROS_DEBUG_STREAM(rs2_stream_to_string(f.get_profile().stream_type()) << " stream published, message address: " << std::hex << msg_address);
            }
            else
            {
                ROS_ERROR("Could not fill ROS message. Frame was dropped.");
            }
        }
    }

    // Publish stream camera info
    if(info_publishers.find(stream) != info_publishers.end())
    {
        auto& info_publisher = info_publishers.at(stream);

        // If rgbd has subscribers, get the camera info of color/detph sensors from _camera_info map.
        // We need this camera info to fill the rgbd msg, regardless if there subscribers to depth/color camera info.
        // We are not publishing this cam_info here, but will be published by rgbd publisher.
        if (_rgbd_publisher && 0 != _rgbd_publisher->get_subscription_count())
        {
            auto& cam_info = _camera_info.at(stream);

            // Fix the camera info if needed, usually only in the first time
            // when we init this object in the _camera_info map
            if (cam_info.width != width)
            {
                updateStreamCalibData(f.get_profile().as<rs2::video_stream_profile>());
            }
            cam_info.header.stamp = t;
        }

        // If depth/color camera info has subscribers get camera info from _camera_info map,
        // and publish this msg.
        if(0 != info_publisher->get_subscription_count())
        {
            auto& cam_info = _camera_info.at(stream);

            // Fix the camera info if needed, usually only in the first time
            // when we init this object in the _camera_info map
            if (cam_info.width != width)
            {
                updateStreamCalibData(f.get_profile().as<rs2::video_stream_profile>());
            }
            cam_info.header.stamp = t;
            info_publisher->publish(cam_info);
        }
    }

    // Publish stream metadata
    if (is_publishMetadata)
    {
        publishMetadata(f, t, OPTICAL_FRAME_ID(stream));
    }
}


void BaseRealSenseNode::publishRGBD(
    const cv::Mat& rgb_cv_matrix,
    const rs2_format& color_format,
    const cv::Mat& depth_cv_matrix,
    const rs2_format& depth_format,
    const rclcpp::Time& t)
{
    if (_rgbd_publisher && 0 != _rgbd_publisher->get_subscription_count())
    {
        ROS_DEBUG_STREAM("Publishing RGBD message");
        unsigned int rgb_width = rgb_cv_matrix.size().width;
        unsigned int rgb_height = rgb_cv_matrix.size().height;
        unsigned int depth_width = depth_cv_matrix.size().width;
        unsigned int depth_height = depth_cv_matrix.size().height;

        realsense2_camera_msgs::msg::RGBD::UniquePtr msg(new realsense2_camera_msgs::msg::RGBD());

        msg->rgb_camera_info = _camera_info.at(COLOR);
        msg->depth_camera_info = _camera_info.at(DEPTH);

        auto depth_stream_index_pair = DEPTH;
        if (_align_depth_filter->is_enabled())
        {
            depth_stream_index_pair = COLOR;
            msg->depth_camera_info = _camera_info.at(COLOR);
        }

        bool rgb_message_filled = fillROSImageMsgAndReturnStatus(rgb_cv_matrix, COLOR, rgb_width, rgb_height, color_format, t, &msg->rgb);
        if(!rgb_message_filled)
        {
            ROS_ERROR_STREAM("Failed to fill rgb message inside RGBD message");
            return;
        }
        
        bool depth_messages_filled = fillROSImageMsgAndReturnStatus(depth_cv_matrix, depth_stream_index_pair, depth_width, depth_height, depth_format, t, &msg->depth);
        if(!depth_messages_filled)
        {
            ROS_ERROR_STREAM("Failed to fill depth message inside RGBD message");
            return;
        }

        msg->header.frame_id = "camera_rgbd_optical_frame";
        msg->header.stamp = t;


        realsense2_camera_msgs::msg::RGBD *msg_address = msg.get();
        _rgbd_publisher->publish(std::move(msg));
        ROS_DEBUG_STREAM("rgbd stream published, message address: " << std::hex << msg_address);
    }
}

void BaseRealSenseNode::publishMetadata(rs2::frame f, const rclcpp::Time& header_time, const std::string& frame_id)
{
    stream_index_pair stream = {f.get_profile().stream_type(), f.get_profile().stream_index()};    
    if (_metadata_publishers.find(stream) != _metadata_publishers.end())
    {
        auto& md_publisher = _metadata_publishers.at(stream);
        if (0 != md_publisher->get_subscription_count())
        {
            realsense2_camera_msgs::msg::Metadata msg;
            msg.header.frame_id = frame_id;
            msg.header.stamp = header_time;
            std::stringstream json_data;
            const char* separator = ",";
            json_data << "{";
            // Add additional fields:
            json_data << "\"" << "frame_number" << "\":" << f.get_frame_number();
            json_data << separator << "\"" << "clock_domain" << "\":" << "\"" << create_graph_resource_name(rs2_timestamp_domain_to_string(f.get_frame_timestamp_domain())) << "\"";
            json_data << separator << "\"" << "frame_timestamp" << "\":" << std::fixed << f.get_timestamp();

            for (auto i = 0; i < RS2_FRAME_METADATA_COUNT; i++)
            {
                if (f.supports_frame_metadata((rs2_frame_metadata_value)i))
                {
                    rs2_frame_metadata_value mparam = (rs2_frame_metadata_value)i;
                    std::string name = create_graph_resource_name(rs2_frame_metadata_to_string(mparam));
                    if (RS2_FRAME_METADATA_FRAME_TIMESTAMP == i)
                    {
                        name = "hw_timestamp";
                    }
                    rs2_metadata_type val = f.get_frame_metadata(mparam);
                    json_data << separator << "\"" << name << "\":" << val;
                }
            }
            json_data << "}";
            msg.json_data = json_data.str();
            md_publisher->publish(msg);
        }
    }
}

void BaseRealSenseNode::startDiagnosticsUpdater()
{
    std::string serial_no = _dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    if (_diagnostics_period > 0)
    {
        ROS_INFO_STREAM("Publish diagnostics every " << _diagnostics_period << " seconds.");
        _diagnostics_updater = std::make_shared<diagnostic_updater::Updater>(&_node, _diagnostics_period);

        _diagnostics_updater->setHardwareID("realsense");

        _diagnostics_updater->add("Temperatures", [this](diagnostic_updater::DiagnosticStatusWrapper& status)
        {
            bool got_temperature(false);
            for(auto&& sensor : _available_ros_sensors)
            {
                for (rs2_option option : _monitor_options)
                {
                    try
                    {
                        if (sensor->supports(option))
                        {
                            status.add(rs2_option_to_string(option), sensor->get_option(option));
                            got_temperature = true;
                        }
                    }
                    catch(const std::exception& ex)
                    {
                        got_temperature = false;
                        ROS_WARN_STREAM("An error has occurred during monitoring: " << ex.what());
                    }
                }
                if (got_temperature) break;
            }
            status.summary(0, "OK");
        });
    }
}
