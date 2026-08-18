// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2022 Intel Corporation. All Rights Reserved.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#if defined( DASHING ) || defined( ELOQUENT )
#include <image_transport/image_transport.h>
#else
#include <image_transport/image_transport.hpp>
#endif

#include <shm_ros/publisher.hpp>

namespace realsense2_camera {
class image_publisher
{
public:
    virtual void publish( sensor_msgs::msg::Image::UniquePtr image_ptr ) = 0;
    virtual size_t get_subscription_count() const = 0;
    virtual ~image_publisher() = default;
};

// Native RCL implementation of an image publisher (needed for intra-process communication)
class image_rcl_publisher : public image_publisher
{
public:
    image_rcl_publisher( rclcpp::Node & node,
                         const std::string & topic_name,
                         const rmw_qos_profile_t & qos );
    void publish( sensor_msgs::msg::Image::UniquePtr image_ptr ) override;
    size_t get_subscription_count() const override;

private:
    rclcpp::Publisher< sensor_msgs::msg::Image >::SharedPtr image_publisher_impl;
};

// image_transport implementation of an image publisher (adds a compressed image topic)
class image_transport_publisher : public image_publisher
{
public:
    image_transport_publisher( rclcpp::Node & node,
                               const std::string & topic_name,
                               const rmw_qos_profile_t & qos );
    void publish( sensor_msgs::msg::Image::UniquePtr image_ptr ) override;
    size_t get_subscription_count() const override;

private:
    std::shared_ptr< image_transport::Publisher > image_publisher_impl;
};

// Kiwi added: layers shm_ros::ImagePublisher OVER one of the publishers above, so the
// normal image topic keeps working and the frame also lands in a shared-memory segment
// announced on a sibling "<topic>_shm" topic. Ported from robot-os.
class image_shm_publisher : public image_publisher
{
public:
    image_shm_publisher( rclcpp::Node & node,
                         const std::string & topic_name,
                         const rmw_qos_profile_t & qos,
                         std::shared_ptr< image_publisher > inner );
    void publish( sensor_msgs::msg::Image::UniquePtr image_ptr ) override;
    // Counts the announcement's subscribers too, otherwise the driver skips the frame
    // when only shared-memory consumers listen.
    size_t get_subscription_count() const override;

private:
    std::shared_ptr< image_publisher > _inner;
    shm_ros::ImagePublisher _shm;
};

}  // namespace realsense2_camera
