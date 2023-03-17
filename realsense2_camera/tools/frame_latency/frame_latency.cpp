// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2022 Intel Corporation. All Rights Reserved.

#include <sstream>
#include <string>
#include <frame_latency/frame_latency.h>
#include <constants.h>
// Node which receives sensor_msgs/Image messages and prints the image latency.

using namespace rs2_ros::tools::frame_latency;

FrameLatencyNode::FrameLatencyNode( const std::string & node_name,
                                    const std::string & ns,
                                    const rclcpp::NodeOptions & node_options )
    : Node( node_name, ns, node_options )
    , _logger( this->get_logger() )
{
}


FrameLatencyNode::FrameLatencyNode( const rclcpp::NodeOptions & node_options )
    : Node( "frame_latency", "/", node_options )
    , _logger( this->get_logger() )
{
    ROS_INFO_STREAM( "frame_latency node is UP!" );
    ROS_INFO_STREAM( "Intra-Process is "
                     << ( this->get_node_options().use_intra_process_comms() ? "ON" : "OFF" ) );
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE( rs2_ros::tools::frame_latency::FrameLatencyNode )
