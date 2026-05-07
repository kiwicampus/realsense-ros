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
#include <vector>
#include <librealsense2/rs.hpp>
#include <sensor_params.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <ros_sensor.h>
#include "named_filter.h"

namespace realsense2_camera
{
    class PointcloudFilter : public NamedFilter
    {
        public:
            PointcloudFilter(std::shared_ptr<rs2::filter> filter, RosNodeBase& node, std::shared_ptr<Parameters> parameters, rclcpp::Logger logger, bool is_enabled=false);

            void setPublisher();
            void Publish(rs2::points pc, const rclcpp::Time& t, const rs2::frameset& frameset, const std::string& frame_id);

            // Kiwibot: pixel→3D lookup against the most recent depth frame.
            // Returns false if no frame has been cached yet. Pixels outside the depth image
            // and pixels with z<=0 are reported as (-1,-1,-1).
            bool getCoordsAtPixels(const std::vector<geometry_msgs::msg::Point>& pixels,
                                   std::vector<geometry_msgs::msg::Point>& out_coords,
                                   std::string& source_frame_id,
                                   rclcpp::Time& stamp);

        private:
            void setParameters();

        private:
            bool _is_enabled_pc;
            RosNodeBase& _node;
            bool _allow_no_texture_points;
            bool _ordered_pc;
            // Kiwibot: keep every Nth pixel in the published pointcloud (per-axis stride).
            // Default 1 = full resolution. Production sets STEREO_PC_SUBSAMPLE_FCT=8 → 1/64 density.
            int _pc_subsample_fct;
            std::mutex _mutex_publisher;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pointcloud_publisher;
            std::string _pointcloud_qos;

            // Cache for the get_coords service, refreshed on every Publish().
            // Holding the rs2::points object keeps librealsense's vertex buffer alive
            // (refcounted) without copying ~5 MB/frame.
            std::mutex _cache_mutex;
            rs2::points _cached_points;
            rs2_intrinsics _cached_intrinsics{};
            rclcpp::Time _cached_stamp;
            std::string _cached_frame_id;
    };
}
