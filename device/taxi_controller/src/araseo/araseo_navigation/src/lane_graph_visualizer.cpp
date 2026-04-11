#include "araseo_navigation/lane_graph_visualizer.hpp"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/quaternion.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "yaml-cpp/yaml.h"

namespace araseo_navigation
{

namespace
{

geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

std::string pointLabel(const geometry_msgs::msg::Point & point)
{
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "(%.2f, %.2f)", point.x, point.y);
  return std::string(buffer);
}

}  // namespace

LaneGraphVisualizer::LaneGraphVisualizer()
: Node("lane_graph_visualizer")
{
  graph_file_ = declare_parameter<std::string>("graph_file", "");
  frame_id_ = declare_parameter<std::string>("frame_id", "odom");

  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/lane_graph", rclcpp::QoS(1).reliable().transient_local());

  if (graph_file_.empty()) {
    throw std::runtime_error("lane_graph_visualizer requires a non-empty graph_file parameter");
  }

  loadGraph();
  publishMarkers();

  republish_timer_ = create_wall_timer(
    std::chrono::seconds(1), [this]() {
      publishMarkers();
    });

  RCLCPP_INFO(
    get_logger(), "Publishing %zu lane graph lanes from %s",
    lanes_.size(), graph_file_.c_str());
}

void LaneGraphVisualizer::loadGraph()
{
  lanes_.clear();

  YAML::Node root;
  try {
    root = YAML::LoadFile(graph_file_);
  } catch (const std::exception & ex) {
    throw std::runtime_error("Failed to load lane graph file: " + std::string(ex.what()));
  }

  if (root["default_lane_width"]) {
    default_lane_width_ = root["default_lane_width"].as<double>();
  }

  const auto lanes_node = root["lanes"];
  if (!lanes_node || !lanes_node.IsSequence()) {
    throw std::runtime_error("Lane graph file must contain a top-level 'lanes' sequence");
  }

  for (const auto & lane_node : lanes_node) {
    Lane lane;
    lane.id = lane_node["id"].as<std::string>();

    const auto centerline_node = lane_node["centerline"];
    if (!centerline_node || !centerline_node.IsSequence() || centerline_node.size() < 2U) {
      throw std::runtime_error("Lane '" + lane.id + "' must contain at least two centerline points");
    }

    lane.centerline.reserve(centerline_node.size());
    for (const auto & point_node : centerline_node) {
      if (!point_node.IsSequence() || point_node.size() < 2U) {
        throw std::runtime_error(
                "Lane '" + lane.id + "' has an invalid centerline point definition");
      }

      geometry_msgs::msg::Point point;
      point.x = point_node[0].as<double>();
      point.y = point_node[1].as<double>();
      point.z = point_node.size() > 2U ? point_node[2].as<double>() : 0.0;
      lane.centerline.push_back(point);
    }

    const auto successors_node = lane_node["successors"];
    if (successors_node && successors_node.IsSequence()) {
      for (const auto & successor_node : successors_node) {
        lane.successors.push_back(successor_node.as<std::string>());
      }
    }

    lanes_.emplace(lane.id, std::move(lane));
  }

  for (const auto & [lane_id, lane] : lanes_) {
    for (const auto & successor_id : lane.successors) {
      if (lanes_.find(successor_id) == lanes_.end()) {
        throw std::runtime_error(
                "Lane '" + lane_id + "' references unknown successor '" + successor_id + "'");
      }
    }
  }
}

void LaneGraphVisualizer::publishMarkers()
{
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(makeDeleteAllMarker());

  int32_t id = 0;
  for (const auto & [lane_id, lane] : lanes_) {
    (void)lane_id;
    marker_array.markers.push_back(makeCenterlineMarker(lane, id++));
    marker_array.markers.push_back(makeWaypointMarker(lane, id++));
    marker_array.markers.push_back(makeLaneLabelMarker(lane, id++));
    marker_array.markers.push_back(makeDirectionMarker(lane, id++));
  }

  auto area_markers = makeLaneAreaMarkers(id);
  marker_array.markers.insert(
    marker_array.markers.end(), area_markers.begin(), area_markers.end());

  auto coord_markers = makeCoordinateMarkers(id);
  marker_array.markers.insert(
    marker_array.markers.end(), coord_markers.begin(), coord_markers.end());

  auto successor_markers = makeSuccessorMarkers(id);
  marker_array.markers.insert(
    marker_array.markers.end(), successor_markers.begin(), successor_markers.end());

  marker_pub_->publish(marker_array);
}

visualization_msgs::msg::Marker LaneGraphVisualizer::makeDeleteAllMarker() const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = now();
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  return marker;
}

visualization_msgs::msg::Marker LaneGraphVisualizer::makeCenterlineMarker(
  const Lane & lane, const int32_t id) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = now();
  marker.ns = "lane_centerline";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.02;
  marker.color.r = 0.98F;
  marker.color.g = 0.85F;
  marker.color.b = 0.18F;
  marker.color.a = 1.0F;
  marker.pose.orientation.w = 1.0;
  marker.points = lane.centerline;
  return marker;
}

visualization_msgs::msg::Marker LaneGraphVisualizer::makeWaypointMarker(
  const Lane & lane, const int32_t id) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = now();
  marker.ns = "lane_points";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.04;
  marker.scale.y = 0.04;
  marker.scale.z = 0.04;
  marker.color.r = 0.95F;
  marker.color.g = 0.95F;
  marker.color.b = 0.95F;
  marker.color.a = 1.0F;
  marker.pose.orientation.w = 1.0;
  marker.points = lane.centerline;
  return marker;
}

visualization_msgs::msg::Marker LaneGraphVisualizer::makeLaneLabelMarker(
  const Lane & lane, const int32_t id) const
{
  const auto label_index = lane.centerline.size() / 2U;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = now();
  marker.ns = "lane_labels";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position = lane.centerline[label_index];
  marker.pose.position.z += 0.14;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = 0.06;
  marker.color.r = 1.0F;
  marker.color.g = 1.0F;
  marker.color.b = 1.0F;
  marker.color.a = 0.95F;
  marker.text = lane.id;
  return marker;
}

visualization_msgs::msg::Marker LaneGraphVisualizer::makeDirectionMarker(
  const Lane & lane, const int32_t id) const
{
  const auto index = (lane.centerline.size() > 2U) ? (lane.centerline.size() / 2U) - 1U : 0U;
  const auto next_index = std::min(index + 1U, lane.centerline.size() - 1U);

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = now();
  marker.ns = "lane_direction";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.12;
  marker.scale.y = 0.03;
  marker.scale.z = 0.03;
  marker.color.r = 1.0F;
  marker.color.g = 0.45F;
  marker.color.b = 0.12F;
  marker.color.a = 0.95F;
  marker.pose.position = midpoint(lane.centerline[index], lane.centerline[next_index]);
  marker.pose.position.z += 0.05;
  marker.pose.orientation = createQuaternionMsgFromYaw(
    yawBetween(lane.centerline[index], lane.centerline[next_index]));
  return marker;
}

std::vector<visualization_msgs::msg::Marker> LaneGraphVisualizer::makeCoordinateMarkers(int32_t & id) const
{
  std::vector<visualization_msgs::msg::Marker> markers;
  for (const auto & [lane_id, lane] : lanes_) {
    (void)lane_id;
    for (std::size_t i = 0; i < lane.centerline.size(); ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id_;
      marker.header.stamp = now();
      marker.ns = "lane_coords";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position = lane.centerline[i];
      marker.pose.position.z += 0.08;
      marker.pose.orientation.w = 1.0;
      marker.scale.z = 0.04;
      marker.color.r = 0.95F;
      marker.color.g = 0.95F;
      marker.color.b = 0.95F;
      marker.color.a = 0.9F;
      marker.text = pointLabel(lane.centerline[i]);
      markers.push_back(std::move(marker));
    }
  }
  return markers;
}

std::vector<visualization_msgs::msg::Marker> LaneGraphVisualizer::makeLaneAreaMarkers(int32_t & id) const
{
  std::vector<visualization_msgs::msg::Marker> markers;
  for (const auto & [lane_id, lane] : lanes_) {
    (void)lane_id;
    for (std::size_t i = 1; i < lane.centerline.size(); ++i) {
      const auto & start = lane.centerline[i - 1U];
      const auto & end = lane.centerline[i];
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id_;
      marker.header.stamp = now();
      marker.ns = "lane_area";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position = midpoint(start, end);
      marker.pose.orientation = createQuaternionMsgFromYaw(yawBetween(start, end));
      marker.scale.x = distance2D(start, end);
      marker.scale.y = default_lane_width_;
      marker.scale.z = 0.01;
      marker.color.r = 0.18F;
      marker.color.g = 0.85F;
      marker.color.b = 0.62F;
      marker.color.a = 0.28F;
      markers.push_back(std::move(marker));
    }
  }
  return markers;
}

std::vector<visualization_msgs::msg::Marker> LaneGraphVisualizer::makeSuccessorMarkers(int32_t & id) const
{
  std::vector<visualization_msgs::msg::Marker> markers;
  for (const auto & [lane_id, lane] : lanes_) {
    (void)lane_id;
    const auto & from = lane.centerline.back();
    for (const auto & successor_id : lane.successors) {
      const auto successor_it = lanes_.find(successor_id);
      if (successor_it == lanes_.end()) {
        continue;
      }

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id_;
      marker.header.stamp = now();
      marker.ns = "lane_successors";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.015;
      marker.scale.y = 0.035;
      marker.scale.z = 0.05;
      marker.color.r = 1.0F;
      marker.color.g = 0.55F;
      marker.color.b = 0.20F;
      marker.color.a = 0.9F;
      auto to = successor_it->second.centerline.front();
      geometry_msgs::msg::Point lifted_from = from;
      geometry_msgs::msg::Point lifted_to = to;
      lifted_from.z += 0.03;
      lifted_to.z += 0.03;
      marker.points = {lifted_from, lifted_to};
      markers.push_back(std::move(marker));
    }
  }
  return markers;
}

geometry_msgs::msg::Point LaneGraphVisualizer::midpoint(
  const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  geometry_msgs::msg::Point point;
  point.x = (a.x + b.x) * 0.5;
  point.y = (a.y + b.y) * 0.5;
  point.z = (a.z + b.z) * 0.5;
  return point;
}

double LaneGraphVisualizer::distance2D(
  const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double LaneGraphVisualizer::yawBetween(
  const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::atan2(b.y - a.y, b.x - a.x);
}

}  // namespace araseo_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<araseo_navigation::LaneGraphVisualizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
