#ifndef ARASEO_NAVIGATION__ROAD_DRIVING_PLANNER_HPP_
#define ARASEO_NAVIGATION__ROAD_DRIVING_PLANNER_HPP_

#include <memory>
#include <string>
#include <functional>
#include <unordered_map>
#include <vector>
#include <limits>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace araseo_navigation
{

class LaneGraphPlanner : public nav2_core::GlobalPlanner
{
public:
  LaneGraphPlanner() = default;
  ~LaneGraphPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

protected:
  struct Lane
  {
    std::string id;
    std::string kind{"road"};
    std::vector<geometry_msgs::msg::Point> centerline;
    std::vector<std::string> successors;
    double length{0.0};
    std::size_t order{0U};
  };

  struct LaneMatch
  {
    std::string lane_id;
    std::size_t waypoint_index{0U};
    double score{0.0};
  };

  struct LaneCandidate
  {
    LaneMatch match;
    double raw_score{std::numeric_limits<double>::max()};
    double penalized_score{std::numeric_limits<double>::max()};
  };

  void loadLaneGraph();
  std::vector<LaneCandidate> findLaneCandidates(
    const geometry_msgs::msg::PoseStamped & pose,
    bool prefer_forward_half) const;
  std::vector<std::string> searchLaneSequence(
    const std::string & start_lane_id,
    const std::string & goal_lane_id) const;
  double computeLaneSequenceCost(const std::vector<std::string> & lane_sequence) const;
  std::vector<geometry_msgs::msg::Point> sampleLaneCenterline(
    const Lane & lane) const;
  nav_msgs::msg::Path buildPathFromLaneSequence(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const LaneMatch & start_match,
    const LaneMatch & goal_match,
    const std::vector<std::string> & lane_sequence) const;

  static double distance2D(
    const geometry_msgs::msg::Point & a,
    const geometry_msgs::msg::Point & b);
  static double distance2D(
    const geometry_msgs::msg::Point & a,
    const geometry_msgs::msg::Pose & b);
  static double distance2D(
    const geometry_msgs::msg::Pose & a,
    const geometry_msgs::msg::Pose & b);
  static double computeLaneLength(const std::vector<geometry_msgs::msg::Point> & centerline);
  static double normalizeAngle(double angle);
  static double poseYaw(const geometry_msgs::msg::PoseStamped & pose);
  static double headingAtIndex(
    const std::vector<geometry_msgs::msg::Point> & points,
    std::size_t index);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("lane_graph_planner")};
  std::string name_;
  std::string global_frame_;
  std::string graph_file_;
  double lane_heading_weight_{0.7};
  double lane_sampling_resolution_{0.1};
  double path_resolution_{0.1};
  std::size_t lane_candidate_count_{5U};
  double connector_match_penalty_{0.15};
  double lane_sequence_cost_weight_{0.2};
  std::unordered_map<std::string, Lane> lanes_;
  std::vector<std::string> lane_order_;
};

}  // namespace araseo_navigation

#endif  // ARASEO_NAVIGATION__ROAD_DRIVING_PLANNER_HPP_
