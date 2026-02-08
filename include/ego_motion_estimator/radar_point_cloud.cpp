// This file is part of REVE - Radar Ego Velocity Estimator
// Copyright (C) 2021  Christopher Doer <christopher.doer@kit.edu>

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <memory>
#include <set>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <ego_motion_estimator/radar_point_cloud.h>

using namespace reve;

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(RadarPointCloudType,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, power, power)
                                  (float, vel,   vel)
                                  (float, RCS,   RCS)
                                  )

bool reve::pcl2msgToPcl(const sensor_msgs::msg::PointCloud2& pcl_msg, pcl::PointCloud<RadarPointCloudType>& scan)
{
  // TODO: add support for ti_mmwave_rospkg clound type

  std::set<std::string> fields;
  std::string fields_str = "";

  for (const auto& field : pcl_msg.fields)
  {
    fields.emplace(field.name);
    fields_str += field.name + ", ";
  }

  const auto has_field = [&fields](const std::string& name) { return fields.find(name) != fields.end(); };
  const auto pick_field = [&fields](const std::initializer_list<const char*>& names) -> std::string {
    for (const auto& name : names) {
      if (fields.find(name) != fields.end()) return name;
    }
    return "";
  };

  if (has_field("x") && has_field("y") && has_field("z") &&
      (has_field("power") || has_field("RCS")) &&
      has_field("vel"))
  {
    RCLCPP_INFO_ONCE(rclcpp::get_logger("reve"), "[pcl2msgToPcl]: Detected rio pcl format!");
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(pcl_msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, scan);

    // fix format
    for (auto& p : scan) p.range = p.getVector3fMap().norm();

    return true;
  }

  if (!has_field("x") || !has_field("y") || !has_field("z")) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("reve"),
        "[pcl2msgToPcl]: Missing x/y/z fields. Available fields: " << fields_str.substr(0, fields_str.size() - 2));
    return false;
  }

  const std::string vel_field = pick_field({"vel", "radial_velocity", "doppler_velocity", "v"});
  const std::string rcs_field = pick_field({"RCS", "radar_cross_section"});
  const std::string power_field = pick_field({"power", "signal_noise_ratio", "snr"});

  if (vel_field.empty()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("reve"),
        "[pcl2msgToPcl]: Missing doppler velocity field (expected 'vel' or 'radial_velocity'). Available fields: "
            << fields_str.substr(0, fields_str.size() - 2));
    return false;
  }

  RCLCPP_INFO_STREAM_ONCE(
      rclcpp::get_logger("reve"),
      "[pcl2msgToPcl]: Detected custom radar fields. Mapping vel='" << vel_field << "', rcs='" << rcs_field
                                                                    << "', power='" << power_field << "'.");

  scan.clear();
  scan.is_dense = pcl_msg.is_dense;
  scan.reserve(pcl_msg.width * pcl_msg.height);

  sensor_msgs::PointCloud2ConstIterator<float> msg_x(pcl_msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> msg_y(pcl_msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> msg_z(pcl_msg, "z");
  sensor_msgs::PointCloud2ConstIterator<float> msg_vel(pcl_msg, vel_field);

  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> msg_rcs;
  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> msg_power;
  if (!rcs_field.empty()) {
    msg_rcs = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(pcl_msg, rcs_field);
  }
  if (!power_field.empty()) {
    msg_power = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(pcl_msg, power_field);
  }

  const size_t total_points = pcl_msg.width * pcl_msg.height;
  for (size_t i = 0; i < total_points; ++i, ++msg_x, ++msg_y, ++msg_z, ++msg_vel) {
    RadarPointCloudType p;
    p.x = *msg_x;
    p.y = *msg_y;
    p.z = *msg_z;
    p.vel = *msg_vel;

    float power = 0.0f;
    float rcs = 0.0f;
    if (msg_power) {
      power = **msg_power;
      ++(*msg_power);
    }
    if (msg_rcs) {
      rcs = **msg_rcs;
      ++(*msg_rcs);
    }

    if (!msg_power && msg_rcs) power = rcs;
    if (!msg_rcs && msg_power) rcs = power;

    p.power = power;
    p.RCS = rcs;
    p.range = p.getVector3fMap().norm();
    scan.push_back(p);
  }

  scan.width = scan.size();
  scan.height = 1;

  return true;
}

bool reve::pclToPcl2msg(pcl::PointCloud<RadarPointCloudType> scan, sensor_msgs::msg::PointCloud2& pcl_msg)
{
  scan.height = 1;
  scan.width  = scan.size();

  pcl::PCLPointCloud2 tmp;
  pcl::toPCLPointCloud2<RadarPointCloudType>(scan, tmp);
  pcl_conversions::moveFromPCL(tmp, pcl_msg);

  return true;
}
