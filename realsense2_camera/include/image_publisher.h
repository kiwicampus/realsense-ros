// Copyright 2023 Intel Corporation. All Rights Reserved.
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

#pragma once

#include "ros_node_base.h"
#include <sensor_msgs/msg/image.hpp>

#include <image_transport/image_transport.hpp>
#include <shm_ros/publisher.hpp>

#include <memory>
#include <string>

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
    image_rcl_publisher( RosNodeBase & node,
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

// Adapter: bolts shm_ros::ImagePublisher onto the vendor's image_publisher
// interface, layered OVER one of the two above so the normal image topic keeps
// working. All the shared-memory behaviour — ring, stride, write-then-announce
// ordering, error de-duplication — lives in shm_ros, not here.
class image_shm_publisher : public image_publisher
{
public:
    image_shm_publisher( RosNodeBase & node,
                         const std::string & topic_name,
                         const rmw_qos_profile_t & qos,
                         std::shared_ptr< image_publisher > inner );
    void publish( sensor_msgs::msg::Image::UniquePtr image_ptr ) override;
    // Counts the announcement's subscribers too. Without that the driver skips
    // filling the frame whenever nobody subscribes to the plain image topic, and
    // the shared-memory consumers would never see a thing.
    size_t get_subscription_count() const override;

private:
    std::shared_ptr< image_publisher > _inner;
    shm_ros::ImagePublisher _shm;
};

}  // namespace realsense2_camera
