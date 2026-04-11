#ifndef ARASEO_NAVIGATION__ROAD_DRIVING_CONTROLLER_HPP_
#define ARASEO_NAVIGATION__ROAD_DRIVING_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace araseo_navigation
{

class RoadDrivingController : public nav2_core::Controller
{
public:
  RoadDrivingController() = default;
  ~RoadDrivingController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  bool cancel() override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  void reset() override;

private:
  struct LaneDebugInfo
  {
    std::string id;
    std::string kind{"road"};
    std::vector<geometry_msgs::msg::Point> centerline;
  };

  void loadLaneGraph();
  const LaneDebugInfo * findClosestLane(
    const geometry_msgs::msg::PoseStamped & pose,
    double * best_distance = nullptr) const;
  std::size_t findClosestPoseIndex(const geometry_msgs::msg::PoseStamped & pose) const;
  std::size_t findLookaheadPoseIndex(
    const geometry_msgs::msg::PoseStamped & pose,
    std::size_t start_index,
    double lookahead_distance) const;

  static double distance2D(
    const geometry_msgs::msg::Point & a,
    const geometry_msgs::msg::Point & b);
  static double normalizeAngle(double angle);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("road_driving_controller")};

  std::string plugin_name_;
  std::string graph_file_;
  nav_msgs::msg::Path global_plan_;

  double desired_linear_vel_{0.12};
  double max_linear_vel_{0.18};
  double min_linear_vel_{0.03};
  double max_angular_vel_{0.45};
  double lookahead_dist_{0.45};
  double yaw_gain_{1.2};
  double heading_error_slowdown_threshold_{0.35};
  double goal_dist_tolerance_{0.10};
  double goal_yaw_tolerance_{0.15};
  double rotate_to_goal_angular_vel_{0.30};
  double rotate_in_place_heading_error_{0.85};
  double slow_turn_heading_error_{0.35};
  double turn_lookahead_scale_{0.5};
  double max_speed_limit_{0.18};
  std::size_t path_progress_index_{0U};
  int path_search_ahead_poses_{40};
  int path_search_behind_poses_{3};
  std::unordered_map<std::string, LaneDebugInfo> lanes_;
  rclcpp::Time last_connector_trace_time_{0, 0, RCL_ROS_TIME};
  bool cancel_requested_{false};
};

}  // namespace araseo_navigation

#endif  // ARASEO_NAVIGATION__ROAD_DRIVING_CONTROLLER_HPP_
