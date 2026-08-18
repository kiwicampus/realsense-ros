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

#include <image_publisher.h>

using namespace realsense2_camera;

// --- image_rcl_publisher implementation ---
image_rcl_publisher::image_rcl_publisher( RosNodeBase & node,
                                          const std::string & topic_name,
                                          const rmw_qos_profile_t & qos )
{
    image_publisher_impl = node.create_publisher< sensor_msgs::msg::Image >(
        topic_name,
        rclcpp::QoS( rclcpp::QoSInitialization::from_rmw( qos ), qos ) );
}

void image_rcl_publisher::publish( sensor_msgs::msg::Image::UniquePtr image_ptr )
{
    image_publisher_impl->publish( std::move( image_ptr ) );
}

size_t image_rcl_publisher::get_subscription_count() const
{
    return image_publisher_impl->get_subscription_count();
}

// --- image_transport_publisher implementation ---
image_transport_publisher::image_transport_publisher( rclcpp::Node & node,
                                                      const std::string & topic_name,
                                                      const rmw_qos_profile_t & qos )
{
    image_publisher_impl = std::make_shared< image_transport::Publisher >(
        image_transport::create_publisher( &node, topic_name, qos ) );
}
void image_transport_publisher::publish( sensor_msgs::msg::Image::UniquePtr image_ptr )
{
    image_publisher_impl->publish( *image_ptr );
}

size_t image_transport_publisher::get_subscription_count() const
{
    return image_publisher_impl->getNumSubscribers();
}

// --- image_shm_publisher implementation ---
image_shm_publisher::image_shm_publisher( RosNodeBase & node,
                                          const std::string & topic_name,
                                          const rmw_qos_profile_t & qos,
                                          std::shared_ptr< image_publisher > inner )
    : _inner( std::move( inner ) )
    , _shm( node, topic_name, rclcpp::QoS( rclcpp::QoSInitialization::from_rmw( qos ), qos ) )
{
    RCLCPP_INFO_STREAM( node.get_logger(), "shm publisher announcing " << _shm.topic_name()
                                                                       << " -> /dev/shm/"
                                                                       << _shm.segment_name() );
}

void image_shm_publisher::publish( sensor_msgs::msg::Image::UniquePtr image_ptr )
{
    // Write and announce BEFORE handing the message on: the inner publisher takes
    // ownership and may move the buffer out from under us.
    if( image_ptr && !image_ptr->data.empty() )
    {
        _shm.publish( image_ptr->data.data(), image_ptr->data.size(), image_ptr->width,
                      image_ptr->height, image_ptr->encoding, image_ptr->step,
                      image_ptr->header );
    }
    _inner->publish( std::move( image_ptr ) );
}

size_t image_shm_publisher::get_subscription_count() const
{
    return _inner->get_subscription_count() + _shm.get_subscription_count();
}
