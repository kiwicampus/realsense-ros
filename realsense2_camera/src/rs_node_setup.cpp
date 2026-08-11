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
#include <image_publisher.h>
#include <fstream>
#include <sstream>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <rclcpp/qos.hpp>
#include "pointcloud_filter.h"
#include "align_depth_filter.h"

using namespace realsense2_camera;
using namespace rs2;

void BaseRealSenseNode::setup()
{
#if defined (ACCELERATE_GPU_WITH_GLSL)
    initOpenGLProcessing(_accelerate_gpu_with_glsl);
    _is_accelerate_gpu_with_glsl_changed = false;
#endif
    setDynamicParams();
    startDiagnosticsUpdater();
    setAvailableSensors();
    SetBaseStream();
    setupFilters();
    setCallbackFunctions();
    monitoringProfileChanges();
    updateSensors();
    publishServices();
}

void BaseRealSenseNode::monitoringProfileChanges()
{
    int time_interval(10000);
    std::function<void()> func = [this, time_interval](){
        std::unique_lock<std::mutex> lock(_profile_changes_mutex);
        while(_is_running) {
            _cv_mpc.wait_for(lock, std::chrono::milliseconds(time_interval),
                                               [&]{return (!_is_running || _is_profile_changed
                                                                        || _is_align_depth_changed
                                                                        #if defined (ACCELERATE_GPU_WITH_GLSL)
                                                                            || _is_accelerate_gpu_with_glsl_changed
                                                                        #endif
                                                           );});

            if (_is_running && (_is_profile_changed
                                        || _is_align_depth_changed
                                        #if defined (ACCELERATE_GPU_WITH_GLSL)
                                            || _is_accelerate_gpu_with_glsl_changed
                                        #endif
                                ))
            {
                ROS_DEBUG("Profile has changed");
                try
                {
                    updateSensors();
                }
                catch(const std::exception& e)
                {
                    ROS_ERROR_STREAM("Error updating the sensors: " << e.what());
                }
                _is_profile_changed = false;
                _is_align_depth_changed = false;

                #if defined (ACCELERATE_GPU_WITH_GLSL)
                    _is_accelerate_gpu_with_glsl_changed = false;
                #endif
            }
        }
    };
    _monitoring_pc = std::make_shared<std::thread>(func);
}

void BaseRealSenseNode::setAvailableSensors()
{
    {
        std::string msg;
        if (loadDepthPreset(msg))
            ROS_INFO_STREAM("JSON file is loaded! (" << _json_file_path << ")");
        else
            ROS_INFO_STREAM("Depth preset not applied: " << msg);
    }

    auto device_name = _dev.get_info(RS2_CAMERA_INFO_NAME);
    ROS_INFO_STREAM("Device Name: " << device_name);

    auto serial_no = _dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    ROS_INFO_STREAM("Device Serial No: " << serial_no);

    auto device_port_id = _dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT);

    ROS_INFO_STREAM("Device physical port: " << device_port_id);

    auto fw_ver = _dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
    ROS_INFO_STREAM("Device FW version: " << fw_ver);

    auto pid = _dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
    ROS_INFO_STREAM("Device Product ID: 0x" << pid);

    ROS_INFO_STREAM("Sync Mode: " << ((_sync_frames)?"On":"Off"));

    std::function<void(rs2::frame)> frame_callback_function = [this](rs2::frame frame){
        bool is_filter(_filters.end() != find_if(_filters.begin(), _filters.end(), [](std::shared_ptr<NamedFilter> f){return (f->is_enabled()); }));
        if (_sync_frames || is_filter)
            this->_asyncer.invoke(frame);
        else
            frame_callback(frame);
    };

    std::function<void(rs2::frame)> imu_callback_function = [this](rs2::frame frame){
        imu_callback(frame);
        if (_imu_sync_method != imu_sync_method::NONE)
            imu_callback_sync(frame);
    };

    std::function<void(rs2::frame)> multiple_message_callback_function = [this](rs2::frame frame){multiple_message_callback(frame, _imu_sync_method);};

    std::function<void()> update_sensor_func = [this](){
        {
            std::lock_guard<std::mutex> lock_guard(_profile_changes_mutex);
            _is_profile_changed = true;
        }
        _cv_mpc.notify_one();
    };

    std::function<void()> hardware_reset_func = [this](){hardwareResetRequest();};

    for(auto&& sensor : _dev_sensors)
    {
        const std::string module_name(rs2_to_ros(sensor.get_info(RS2_CAMERA_INFO_NAME)));
        std::unique_ptr<RosSensor> rosSensor;
        if (sensor.is<rs2::depth_sensor>() ||
            sensor.is<rs2::color_sensor>() ||
            sensor.is<rs2::safety_sensor>() ||
            sensor.is<rs2::depth_mapping_sensor>())
        {
            ROS_DEBUG_STREAM("Set " << module_name << " as VideoSensor.");
            rosSensor = std::make_unique<RosSensor>(sensor, _parameters, frame_callback_function, update_sensor_func, hardware_reset_func, _diagnostics_updater, _logger, _use_intra_process, _dev.is<playback>());
        }
        else if (sensor.is<rs2::motion_sensor>())
        {
            ROS_DEBUG_STREAM("Set " << module_name << " as ImuSensor.");
            rosSensor = std::make_unique<RosSensor>(sensor, _parameters, imu_callback_function, update_sensor_func, hardware_reset_func, _diagnostics_updater, _logger, false, _dev.is<playback>());
        }
        else
        {
            ROS_WARN_STREAM("Module Name \"" << module_name << "\" does not define a callback.");
            continue;
        }
        _available_ros_sensors.push_back(std::move(rosSensor));
    }

}

void BaseRealSenseNode::setCallbackFunctions()
{
    _asyncer.start([this](rs2::frame f)
    {
        frame_callback(f);
    });
}

void BaseRealSenseNode::stopPublishers(const std::vector<stream_profile>& profiles)
{
    for (auto& profile : profiles)
    {
        stream_index_pair sip(profile.stream_type(), profile.stream_index());
        if (profile.is<rs2::video_stream_profile>())
        {
            _image_publishers.erase(sip);
            _info_publishers.erase(sip);
            _depth_aligned_image_publishers.erase(sip);
            _depth_aligned_info_publisher.erase(sip);
            if(profile.stream_type() == RS2_STREAM_LABELED_POINT_CLOUD && _labeled_pointcloud_publisher)
            {
                _labeled_pointcloud_publisher.reset();
            }
        }
        else if (profile.is<rs2::motion_stream_profile>())
        {
            _is_accel_enabled = false;
            _is_gyro_enabled = false;
            _synced_imu_publisher.reset();
            _imu_publishers.erase(sip);
            _imu_info_publishers.erase(sip);
        }
        _metadata_publishers.erase(sip);
        _extrinsics_publishers.erase(sip);

        if (_publish_tf)
        {
            eraseTransformMsgs(sip, profile);
        }
    }
}

void BaseRealSenseNode::startPublishers(const std::vector<stream_profile>& profiles, const RosSensor& sensor)
{
    const std::string module_name(create_graph_resource_name(rs2_to_ros(sensor.get_info(RS2_CAMERA_INFO_NAME))));
    for (auto& profile : profiles)
    {
        stream_index_pair sip(profile.stream_type(), profile.stream_index());
        std::string stream_name(STREAM_NAME(sip));

        rmw_qos_profile_t qos = sensor.getQOS(sip);
        rmw_qos_profile_t info_qos = sensor.getInfoQOS(sip);

        if (profile.is<rs2::video_stream_profile>())
        {
            if(profile.stream_type() == RS2_STREAM_COLOR)
                _is_color_enabled = true;
            else if (profile.stream_type() == RS2_STREAM_DEPTH)
                _is_depth_enabled = true;

            if (profile.stream_type() == RS2_STREAM_OCCUPANCY)
            {
                // special handling for occupancy stream, since it is a topic of nav_msgs/msg/GridCells messages
                // and not a normal image publisher
                 _occupancy_publisher = _node.create_publisher<nav_msgs::msg::GridCells>("~/occupancy",
                    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos),qos));
            }
            else if(profile.stream_type() == RS2_STREAM_LABELED_POINT_CLOUD)
            {
                // special handling for labeled point cloud stream, since it is a topic of PointCloud messages
                // and not a normal image publisher
                 _labeled_pointcloud_publisher = _node.create_publisher<sensor_msgs::msg::PointCloud2>("~/labeled_point_cloud/points",
                    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos),qos));
            }
            else
            {
                std::stringstream image_raw, camera_info;
                // Depth stream is rectified, Color is unrectified
                // IR streams come in two flavors:
                //   Rectified formats  Y8 and Y8I for Left, Left & Right Luma
                //   Unrectified raw (calibration) format: Y12I
                bool rectified_image = false;
                if (profile.stream_type() == RS2_STREAM_DEPTH || profile.format() == RS2_FORMAT_Y8 || profile.format() == RS2_FORMAT_Y8I)
                    rectified_image = true;

                // adding "~/" to the topic name will add node namespace and node name to the topic
                // see "Private Namespace Substitution Character" section on https://design.ros2.org/articles/topic_and_service_names.html
                image_raw << "~/" << stream_name << "/image_" << ((rectified_image)?"rect_":"") << "raw";
                camera_info << "~/" << stream_name << "/camera_info";


                // We can use 2 types of publishers:
                // 1. Native RCL publisher (supports intra-process zero-copy communication)
                // 2. Image-transport package publisher (adds a compressed image topic if installed)

                #ifdef USE_LIFECYCLE_NODE
                // Always use `image_rcl_publisher` when lifecycle nodes are enabled
                _image_publishers[sip] = std::make_shared<image_rcl_publisher>(_node, image_raw.str(), qos);
                #else
                // 🚀 Use intra-process if enabled, otherwise use image_transport
                if (_use_intra_process)
                {
                    _image_publishers[sip] = std::make_shared<image_rcl_publisher>(_node, image_raw.str(), qos);
                }
                else
                {
                    _image_publishers[sip] = std::make_shared<image_transport_publisher>(_node, image_raw.str(), qos);
                    ROS_DEBUG_STREAM("image transport publisher was created for topic" << image_raw.str());
                }
                #endif

                // create cameraInfo publishers only for non-SC streams
                if(shouldPublishCameraInfo(sip))
                {
                    _info_publishers[sip] = _node.create_publisher<sensor_msgs::msg::CameraInfo>(camera_info.str(),
                                    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(info_qos), info_qos));
                }

                if (_align_depth_filter->is_enabled() && (sip != DEPTH) && sip.second < 2)
                {
                    std::stringstream aligned_image_raw, aligned_camera_info;
                    aligned_image_raw << "~/" << "aligned_depth_to_" << stream_name << "/image_raw";
                    aligned_camera_info << "~/" << "aligned_depth_to_" << stream_name << "/camera_info";

                    std::string aligned_stream_name = "aligned_depth_to_" + stream_name;

                    // We can use 2 types of publishers:
                    // Native RCL publisher that support intra-process zero-copy comunication
                    // image-transport package publisher that add's a commpressed image topic if the package is installed
                    #ifdef USE_LIFECYCLE_NODE
                    // Always use `image_rcl_publisher` when lifecycle nodes are enabled
                    _depth_aligned_image_publishers[sip] = std::make_shared<image_rcl_publisher>(_node, aligned_image_raw.str(), qos);
                    #else
                    // Use intra-process if enabled, otherwise use image_transport
                    if (_use_intra_process)
                    {
                        _depth_aligned_image_publishers[sip] = std::make_shared<image_rcl_publisher>(_node, aligned_image_raw.str(), qos);
                    }
                    else
                    {
                        _depth_aligned_image_publishers[sip] = std::make_shared<image_transport_publisher>(_node, aligned_image_raw.str(), qos);
                        ROS_DEBUG_STREAM("image transport publisher was created for topic " << aligned_image_raw.str());
                    }
                    #endif
                    _depth_aligned_info_publisher[sip] = _node.create_publisher<sensor_msgs::msg::CameraInfo>(aligned_camera_info.str(),
                                                      rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(info_qos), info_qos));
                }
            }
        }
        else if (profile.is<rs2::motion_stream_profile>())
        {
            if(profile.stream_type() == RS2_STREAM_ACCEL)
                _is_accel_enabled = true;
            else if (profile.stream_type() == RS2_STREAM_GYRO)
                _is_gyro_enabled = true;

            std::stringstream data_topic_name, info_topic_name;
            data_topic_name << "~/" << stream_name << "/sample";
            _imu_publishers[sip] = _node.create_publisher<sensor_msgs::msg::Imu>(data_topic_name.str(),
                rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos));
            // Publish Intrinsics:
            info_topic_name << "~/" << stream_name << "/imu_info";

            // IMU Info will have latched QoS, and it will publish its data only once during the ROS Node lifetime.
            // intra-process do not support latched QoS, so we need to disable intra-process for this topic
            rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options;
            options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
            _imu_info_publishers[sip] = _node.create_publisher<IMUInfo>(info_topic_name.str(),
                                                                        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_latched), rmw_qos_profile_latched),
                                                                        std::move(options));
            IMUInfo info_msg = getImuInfo(profile);
            _imu_info_publishers[sip]->publish(info_msg);
        }
        std::string topic_metadata("~/" + stream_name + "/metadata");
        _metadata_publishers[sip] = _node.create_publisher<realsense2_camera_msgs::msg::Metadata>(topic_metadata, 
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(info_qos), info_qos));

        if (!((rs2::stream_profile)profile==(rs2::stream_profile)_base_profile))
        {

            // intra-process do not support latched QoS, so we need to disable intra-process for this topic
            rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options;
            options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
            rmw_qos_profile_t extrinsics_qos = rmw_qos_profile_latched;

            std::string topic_extrinsics("~/extrinsics/" + create_graph_resource_name(ros_stream_to_string(_base_profile.stream_type()) + "_to_" + stream_name));
            _extrinsics_publishers[sip] = _node.create_publisher<realsense2_camera_msgs::msg::Extrinsics>(topic_extrinsics,
                rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(extrinsics_qos), extrinsics_qos), std::move(options));
        }
    }
    if (_is_accel_enabled && _is_gyro_enabled && (_imu_sync_method > imu_sync_method::NONE))
    {
        rmw_qos_profile_t qos = _use_intra_process ? qos_string_to_qos(DEFAULT_QOS) : qos_string_to_qos(HID_QOS);
        
        _synced_imu_publisher = std::make_shared<SyncedImuPublisher>(_node.create_publisher<sensor_msgs::msg::Imu>("~/imu",
                                                        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos)));
    }

    // Kiwibot: latched topic consumed by transform_frames for stereo TF calibration persisted in Firebase.
    // Latched (TransientLocal+Reliable) QoS is incompatible with intra-process comms, so disable
    // IPC on this publisher so the latch survives even when the node is run inside a composable container.
    if (!_cam_imu_angles_publisher)
    {
        rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options;
        options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
        _cam_imu_angles_publisher = _node.create_publisher<geometry_msgs::msg::Quaternion>(
            "~/camera_imu_angles",
            rclcpp::QoS(1).keep_all().transient_local().reliable(),
            options);
    }
}

void BaseRealSenseNode::startRGBDPublisherIfNeeded()
{
    _rgbd_publisher.reset();
    if(_enable_rgbd && !_rgbd_publisher)
    {
        if (_sync_frames && _is_color_enabled && _is_depth_enabled && _align_depth_filter->is_enabled())
        {
            rmw_qos_profile_t qos = _use_intra_process ? qos_string_to_qos(DEFAULT_QOS) : qos_string_to_qos(IMAGE_QOS);

            // adding "~/" to the topic name will add node namespace and node name to the topic
            // see "Private Namespace Substitution Character" section on https://design.ros2.org/articles/topic_and_service_names.html
            _rgbd_publisher = _node.create_publisher<realsense2_camera_msgs::msg::RGBD>("~/rgbd",
                rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos));
        }
        else {
            ROS_ERROR("In order to get rgbd topic enabled, "\
             "you should enable: color stream, depth stream, sync_mode and align_depth");
        }
    }
}

void BaseRealSenseNode::updateSensors()
{
    std::lock_guard<std::mutex> lock_guard(_update_sensor_mutex);
    try{
        stopRequiredSensors();

        #if defined (ACCELERATE_GPU_WITH_GLSL)
            if (_is_accelerate_gpu_with_glsl_changed)
            {
                shutdownOpenGLProcessing();

                initOpenGLProcessing(_accelerate_gpu_with_glsl);
            }
        #endif

        startUpdatedSensors();
    }
    catch(const std::exception& ex)
    {
        ROS_ERROR_STREAM(__FILE__ << ":" << __LINE__ << ":" << "An exception has been thrown: " << ex.what());
        throw;
    }
    catch(...)
    {
        ROS_ERROR_STREAM(__FILE__ << ":" << __LINE__ << ":" << "Unknown exception has occured!");
        throw;
    }
}

void BaseRealSenseNode::stopRequiredSensors()
{
    try{
        for(auto&& sensor : _available_ros_sensors)
        {
            std::string module_name(rs2_to_ros(sensor->get_info(RS2_CAMERA_INFO_NAME)));
            // if active_profiles != wanted_profiles: stop sensor.
            std::vector<stream_profile> wanted_profiles;

            bool is_profile_changed(sensor->getUpdatedProfiles(wanted_profiles));
            bool is_video_sensor = (sensor->is<rs2::depth_sensor>() || sensor->is<rs2::color_sensor>());

            // do all updates if profile has been changed, or if the align depth filter or gpu acceleration status has been changed
            // and we are on a video sensor. TODO: explore better options to monitor and update changes
            // without resetting the whole sensors and topics.
            if (is_profile_changed || (is_video_sensor && (_is_align_depth_changed
                                                                #if defined (ACCELERATE_GPU_WITH_GLSL)
                                                                    || _is_accelerate_gpu_with_glsl_changed
                                                                #endif
                                                            )))
            {
                std::vector<stream_profile> active_profiles = sensor->get_active_streams();
                if (is_profile_changed
                        #if defined (ACCELERATE_GPU_WITH_GLSL)
                            || _is_accelerate_gpu_with_glsl_changed
                        #endif
                    )
                {
                    // Start/stop sensors only if profile or gpu acceleration status was changed
                    // No need to start/stop sensors if align_depth was changed
                    ROS_INFO_STREAM("Stopping Sensor: " << module_name);
                    sensor->stop();
                }
                stopPublishers(active_profiles);
            }
        }
    }
    catch(const std::exception& ex)
    {
        ROS_ERROR_STREAM(__FILE__ << ":" << __LINE__ << ":" << "An exception has been thrown: " << ex.what());
        throw;
    }
    catch(...)
    {
        ROS_ERROR_STREAM(__FILE__ << ":" << __LINE__ << ":" << "Unknown exception has occured!");
        throw;
    }
}

void BaseRealSenseNode::startUpdatedSensors()
{
    try{
        for(auto&& sensor : _available_ros_sensors)
        {
            std::string module_name(rs2_to_ros(sensor->get_info(RS2_CAMERA_INFO_NAME)));
            // if active_profiles != wanted_profiles: stop sensor.
            std::vector<stream_profile> wanted_profiles;

            bool is_profile_changed(sensor->getUpdatedProfiles(wanted_profiles));
            bool is_video_sensor = (sensor->is<rs2::depth_sensor>() || sensor->is<rs2::color_sensor>());

            if (is_profile_changed || (is_video_sensor && (_is_align_depth_changed
                                                                #if defined (ACCELERATE_GPU_WITH_GLSL)
                                                                    || _is_accelerate_gpu_with_glsl_changed
                                                                #endif
                                                            )))
            {
                if (!wanted_profiles.empty())
                {
                    startPublishers(wanted_profiles, *sensor);
                    updateProfilesStreamCalibData(wanted_profiles);
                    if (_publish_tf)
                    {
                        std::lock_guard<std::mutex> lock_guard(_publish_tf_mutex);
                        for (auto &profile : wanted_profiles)
                        {
                            calcAndAppendTransformMsgs(profile, _base_profile);
                        }
                    }

                    if (is_profile_changed
                            #if defined (ACCELERATE_GPU_WITH_GLSL)
                                || _is_accelerate_gpu_with_glsl_changed
                            #endif
                        )
                    {
                        // Start/stop sensors only if profile or gpu acceleration was changed
                        // No need to start/stop sensors if align_depth was changed
                        ROS_INFO_STREAM("Starting Sensor: " << module_name);
                        sensor->start(wanted_profiles);
                    }

                    if (sensor->rs2::sensor::is<rs2::depth_sensor>())
                    {
                        _depth_scale_meters = sensor->as<rs2::depth_sensor>().get_depth_scale();
                    }
                }
            }
        }
        if (_publish_tf)
        {
            std::lock_guard<std::mutex> lock_guard(_publish_tf_mutex);
            publishStaticTransforms();
        }
        startRGBDPublisherIfNeeded();
    }
    catch(const std::exception& ex)
    {
        ROS_ERROR_STREAM(__FILE__ << ":" << __LINE__ << ":" << "An exception has been thrown: " << ex.what());
        throw;
    }
    catch(...)
    {
        ROS_ERROR_STREAM(__FILE__ << ":" << __LINE__ << ":" << "Unknown exception has occured!");
        throw;
    }
}

void BaseRealSenseNode::publishServices()
{
    // adding "~/" to the service name will add node namespace and node name to the service
    // see "Private Namespace Substitution Character" section on https://design.ros2.org/articles/topic_and_service_names.html
    _reset_srv = _node.create_service<std_srvs::srv::Empty>(
            "~/hw_reset",
            [&](const std_srvs::srv::Empty::Request::SharedPtr req,
                        std_srvs::srv::Empty::Response::SharedPtr res)
                        {handleHWReset(req, res);});

    // Kiwibot: triggered by webclient operator to recompute camera_imu_angles from accel.
    _calibrate_imu_srv = _node.create_service<std_srvs::srv::Trigger>(
            "~/calibrate_imu",
            [&](const std_srvs::srv::Trigger::Request::SharedPtr req,
                        std_srvs::srv::Trigger::Response::SharedPtr res)
                        {calibrate_imu_cb(req, res);});

    // Kiwibot: hardware-reset Trigger; kronos_bringup remaps this to /stereo/restart.
    _shutdown_srv = _node.create_service<std_srvs::srv::Trigger>(
            "~/shutdown",
            [&](const std_srvs::srv::Trigger::Request::SharedPtr req,
                        std_srvs::srv::Trigger::Response::SharedPtr res)
                        {shutdown_cb(req, res);});

    // Kiwibot: pixel→3D coords lookup against the latest pointcloud frame.
    _get_coords_srv = _node.create_service<realsense2_camera_srvs::srv::CoordinateReq>(
            "~/get_coords",
            [&](const realsense2_camera_srvs::srv::CoordinateReq::Request::SharedPtr req,
                        realsense2_camera_srvs::srv::CoordinateReq::Response::SharedPtr res)
                        {get_coords_cb(req, res);});

    // Kiwibot: 3D point→pixel projection using the COLOR camera intrinsics.
    _get_pixel_srv = _node.create_service<realsense2_camera_srvs::srv::PixelReq>(
            "~/get_pixel",
            [&](const realsense2_camera_srvs::srv::PixelReq::Request::SharedPtr req,
                        realsense2_camera_srvs::srv::PixelReq::Response::SharedPtr res)
                        {get_pixel_cb(req, res);});

    _device_info_srv = _node.create_service<realsense2_camera_msgs::srv::DeviceInfo>(
            "~/device_info",
            [&](const realsense2_camera_msgs::srv::DeviceInfo::Request::SharedPtr req,
                        realsense2_camera_msgs::srv::DeviceInfo::Response::SharedPtr res)
                        {getDeviceInfo(req, res);});
        // KIWI: service for shuting down node before something going wrong
    _cam_pitch = getEnv("STEREO_PITCH_ANGLE", 15.0)/57.2958; // convert to rads
    _cam_roll = getEnv("STEREO_ROLL_ANGLE", 0.0)/57.2958; // convert to rads    
    _buffer_tf2 = std::make_unique<tf2_ros::Buffer>(_node.get_clock(), tf2::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME), _node.shared_from_this());
    rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> options;
    options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    _listener_tf2 = std::make_shared<tf2_ros::TransformListener>(*_buffer_tf2, _node.shared_from_this(), true,tf2_ros::DynamicListenerQoS(),tf2_ros::StaticListenerQoS(),  options, options);
    _reload_preset_srv = _node.create_service<std_srvs::srv::Trigger>("reload_depth_preset",
                    std::bind(&BaseRealSenseNode::reload_preset_callback, this,
                              std::placeholders::_1, std::placeholders::_2));
    _get_version_srv = _node.create_service<realsense2_camera_srvs::srv::VersionReq>(
            "get_version",
            std::bind(
                &BaseRealSenseNode::get_version_cb,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
    _get_pitch_srv = _node.create_service<realsense2_camera_srvs::srv::CameraPitchReq>(
        "get_pitch",
        std::bind(
                &BaseRealSenseNode::get_pitch_cb,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    if(_safety_sensor)
    {
        publishSafetyServices();
    }

}

void BaseRealSenseNode::publishActions()
{

    using namespace std::placeholders;
    _triggered_calibration_action_server = rclcpp_action::create_server<TriggeredCalibration>(
      _node.get_node_base_interface(),
      _node.get_node_clock_interface(),
      _node.get_node_logging_interface(),
      _node.get_node_waitables_interface(),
      "~/triggered_calibration",
      std::bind(&BaseRealSenseNode::TriggeredCalibrationHandleGoal, this, _1, _2),
      std::bind(&BaseRealSenseNode::TriggeredCalibrationHandleCancel, this, _1),
      std::bind(&BaseRealSenseNode::TriggeredCalibrationHandleAccepted, this, _1));

}

void BaseRealSenseNode::handleHWReset(const std_srvs::srv::Empty::Request::SharedPtr req,
                                const std_srvs::srv::Empty::Response::SharedPtr res)
{
    (void)req;
    (void)res;
    ROS_INFO_STREAM("Reset requested through service call");
    if (_dev)
    {
        try
        {
            for(auto&& sensor : _available_ros_sensors)
            {
                std::string module_name(rs2_to_ros(sensor->get_info(RS2_CAMERA_INFO_NAME)));
                ROS_INFO_STREAM("Stopping Sensor: " << module_name);
                sensor->stop();
            }
            ROS_INFO("Resetting device...");
            _dev.hardware_reset();
        }
        catch(const std::exception& ex)
        {
            ROS_WARN_STREAM("An exception has been thrown: " << __FILE__ << ":" << __LINE__ << ":" << ex.what());
        }
    }
}

void BaseRealSenseNode::getDeviceInfo(const realsense2_camera_msgs::srv::DeviceInfo::Request::SharedPtr,
                                            realsense2_camera_msgs::srv::DeviceInfo::Response::SharedPtr res)
{
    res->device_name = _dev.supports(RS2_CAMERA_INFO_NAME) ? create_graph_resource_name(_dev.get_info(RS2_CAMERA_INFO_NAME)) : "";
    res->serial_number = _dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER) ? _dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) : "";
    res->firmware_version = _dev.supports(RS2_CAMERA_INFO_FIRMWARE_VERSION) ? _dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) : "";
    res->usb_type_descriptor = _dev.supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR) ? _dev.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR) : "";
    res->firmware_update_id = _dev.supports(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID) ? _dev.get_info(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID) : "";

    std::stringstream sensors_names;

    for(auto&& sensor : _available_ros_sensors)
    {
        sensors_names << create_graph_resource_name(rs2_to_ros(sensor->get_info(RS2_CAMERA_INFO_NAME))) << ",";
    }

    res->sensors = sensors_names.str().substr(0, sensors_names.str().size()-1);
    res->physical_port = _dev.supports(RS2_CAMERA_INFO_PHYSICAL_PORT) ? _dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT) : "";
}

void BaseRealSenseNode::CalibConfigReadService(const realsense2_camera_msgs::srv::CalibConfigRead::Request::SharedPtr req,
    realsense2_camera_msgs::srv::CalibConfigRead::Response::SharedPtr res){
    try
    {
        (void)req; // silence unused parameter warning
        res->calib_config = _dev.as<rs2::auto_calibrated_device>().get_calibration_config();
        res->success = true;
    }
    catch (const std::exception &e)
    {
        res->success = false;
        res->error_message = std::string("Exception occurred: ") + e.what();
    }
}

void BaseRealSenseNode::CalibConfigWriteService(const realsense2_camera_msgs::srv::CalibConfigWrite::Request::SharedPtr req,
    realsense2_camera_msgs::srv::CalibConfigWrite::Response::SharedPtr res){
    try
    {
        _dev.as<rs2::auto_calibrated_device>().set_calibration_config(req->calib_config);
        res->success = true;
    }
    catch (const std::exception &e)
    {
        res->success = false;
        res->error_message = std::string("Exception occurred: ") + e.what();
    }
}

// ---------------------------------------------------------------------------
// Runtime depth-register (advanced mode) support
//
// The D4 ASIC's matching and validation registers live behind the advanced-mode
// interface. They are not rs2_options, so the ROS parameter API cannot reach
// them and they are invisible to `ros2 param`. These services are the only
// runtime path: they read the current register block, overwrite just the fields
// the caller asked for, write it back, and read it out again so the caller can
// verify what actually landed. No device reset, no stream interruption.
// ---------------------------------------------------------------------------

bool BaseRealSenseNode::loadDepthPreset(std::string& msg)
{
    if (_json_file_path.empty())
    {
        msg = "json_file_path is empty";
        return false;
    }
    std::ifstream in(_json_file_path);
    if (!in.is_open())
    {
        msg = "cannot open " + _json_file_path;
        return false;
    }
    std::stringstream ss;
    ss << in.rdbuf();
    if (!_dev || !_dev.is<rs400::advanced_mode>())
    {
        msg = "device does not support advanced mode";
        return false;
    }
    auto adv = _dev.as<rs400::advanced_mode>();
    if (!adv.is_enabled())
    {
        // Not toggling it on: toggle_advanced_mode() re-enumerates the device,
        // which would pull the handle out from under the driver.
        msg = "advanced mode is disabled on this device; enable it out of band";
        return false;
    }
    try
    {
        adv.load_json(ss.str());
    }
    catch (const std::exception& e)
    {
        msg = std::string("load_json rejected the preset: ") + e.what();
        return false;
    }
    msg = "loaded " + _json_file_path;
    return true;
}

void BaseRealSenseNode::reload_preset_callback(const std_srvs::srv::Trigger::Request::SharedPtr req,
                                               std_srvs::srv::Trigger::Response::SharedPtr res)
{
    (void)req;
    std::string msg;
    res->success = loadDepthPreset(msg);
    res->message = msg;
    if (res->success)
        ROS_INFO_STREAM("JSON file is loaded! (" << _json_file_path << ")");
    else
        ROS_WARN_STREAM("Depth preset not applied: " << msg);
}

void BaseRealSenseNode::shutdown_callback(const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)
{
    (void)req;
    res->success = true;
    res->message = "Stereo Node will be killed";
    RCLCPP_WARN(_node.get_logger(), "SHUTTING DOWN NODE");
    _dev.hardware_reset();
}

bool BaseRealSenseNode::get_version_cb(realsense2_camera_srvs::srv::VersionReq::Request::SharedPtr req, realsense2_camera_srvs::srv::VersionReq::Response::SharedPtr res){
    (void) req;
    res->version=_dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
    return true;
}

bool BaseRealSenseNode::get_pitch_cb(realsense2_camera_srvs::srv::CameraPitchReq::Request::SharedPtr req, realsense2_camera_srvs::srv::CameraPitchReq::Response::SharedPtr res){
    (void) req;
    res->pitch=_cam_pitch;
    return true;
}

