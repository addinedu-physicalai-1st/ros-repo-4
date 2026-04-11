#ifndef ARASEO_NAVIGATION__LANE_GRAPH_VISUALIZER_HPP_
#define ARASEO_NAVIGATION__LANE_GRAPH_VISUALIZER_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace araseo_navigation
{

class LaneGraphVisualizer : public rclcpp::Node
{
public:
  LaneGraphVisualizer();

private:
  struct Lane
  {
    std::string id;
    std::vector<geometry_msgs::msg::Point> centerline;
    std::vector<std::string> successors;
  };

  void loadGraph();
  void publishMarkers();
  visualization_msgs::msg::Marker makeDeleteAllMarker() const;
  visualization_msgs::msg::Marker makeCenterlineMarker(
    const Lane & lane, int32_t id) const;
  visualization_msgs::msg::Marker makeWaypointMarker(
    const Lane & lane, int32_t id) const;
  visualization_msgs::msg::Marker makeLaneLabelMarker(
    const Lane & lane, int32_t id) const;
  visualization_msgs::msg::Marker makeDirectionMarker(
    const Lane & lane, int32_t id) const;
  std::vector<visualization_msgs::msg::Marker> makeCoordinateMarkers(int32_t & id) const;
  std::vector<visualization_msgs::msg::Marker> makeLaneAreaMarkers(int32_t & id) const;
  std::vector<visualization_msgs::msg::Marker> makeSuccessorMarkers(int32_t & id) const;

  static geometry_msgs::msg::Point midpoint(const geometry_msgs::msg::Point & a,
    const geometry_msgs::msg::Point & b);
  static double distance2D(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b);
  static double yawBetween(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b);

  std::string graph_file_;
  std::string frame_id_;
  double default_lane_width_{0.16};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr republish_timer_;
  std::unordered_map<std::string, Lane> lanes_;
};

}  // namespace araseo_navigation

#endif  // ARASEO_NAVIGATION__LANE_GRAPH_VISUALIZER_HPP_
