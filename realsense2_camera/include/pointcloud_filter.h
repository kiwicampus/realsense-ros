// Copyright 2025 RealSense, Inc. All Rights Reserved.
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

#include <string>
#include <memory>
#include <atomic>
#include <librealsense2/rs.hpp>
#include <sensor_params.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ros_sensor.h>
#include "named_filter.h"

namespace realsense2_camera
{
    class PointcloudFilter : public NamedFilter
    {
        public:
            PointcloudFilter(std::shared_ptr<rs2::filter> filter, RosNodeBase& node, std::shared_ptr<Parameters> parameters, rclcpp::Logger logger, bool is_enabled=false);
        
            void setPublisher();
            void Publish(rs2::points pc, const rclcpp::Time& t, const rs2::frameset& frameset, const std::string& frame_id, bool publish_immediately = true);
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr getPointcloudPublisher() { return _pointcloud_publisher; }
            sensor_msgs::msg::PointCloud2::UniquePtr getLatestPointcloudMessage() { return std::make_unique<sensor_msgs::msg::PointCloud2>(_msg_pointcloud); }

            // KIWI: Make it public so we can get it in the get coords service call
            sensor_msgs::msg::PointCloud2 _msg_pointcloud;
            //DO NOT WRITE THIS VARIABLE, ONLY READ OPERATIONS ARE ALLOWED
            std::atomic<rs2::vertex*> _vertex;
            rs2_intrinsics _depth_intrin;

        private:
            void setParameters();

        private:
            bool _is_enabled_pc;
            RosNodeBase& _node;
            bool _allow_no_texture_points;
            bool _ordered_pc;
            std::mutex _mutex_publisher;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pointcloud_publisher;
            std::string _pointcloud_qos;

            //Toxic logs removal
            int _texture_display_logs;
            // Pointcloud subsampling and decimation filter
            int _pc_subsample_fct;
    };
}
