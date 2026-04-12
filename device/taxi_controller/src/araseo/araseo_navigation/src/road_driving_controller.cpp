#include "araseo_navigation/road_driving_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "yaml-cpp/yaml.h"

namespace araseo_navigation
{

void RoadDrivingController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  if (!node_) {
    throw nav2_core::ControllerException(
            "Failed to lock parent node in RoadDrivingController");
  }

  plugin_name_ = std::move(name);
  logger_ = node_->get_logger();
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);

  node_->declare_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  node_->declare_parameter(plugin_name_ + ".max_linear_vel", max_linear_vel_);
  node_->declare_parameter(plugin_name_ + ".min_linear_vel", min_linear_vel_);
  node_->declare_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  node_->declare_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node_->declare_parameter(plugin_name_ + ".yaw_gain", yaw_gain_);
  node_->declare_parameter(
    plugin_name_ + ".heading_error_slowdown_threshold",
    heading_error_slowdown_threshold_);
  node_->declare_parameter(plugin_name_ + ".goal_dist_tolerance", goal_dist_tolerance_);
  node_->declare_parameter(plugin_name_ + ".goal_yaw_tolerance", goal_yaw_tolerance_);
  node_->declare_parameter(
    plugin_name_ + ".rotate_to_goal_angular_vel",
    rotate_to_goal_angular_vel_);
  node_->declare_parameter(
    plugin_name_ + ".rotate_in_place_heading_error",
    rotate_in_place_heading_error_);
  node_->declare_parameter(
    plugin_name_ + ".slow_turn_heading_error",
    slow_turn_heading_error_);
  node_->declare_parameter(
    plugin_name_ + ".turn_lookahead_scale",
    turn_lookahead_scale_);
  node_->declare_parameter(
    plugin_name_ + ".path_search_ahead_poses",
    path_search_ahead_poses_);
  node_->declare_parameter(
    plugin_name_ + ".path_search_behind_poses",
    path_search_behind_poses_);
  node_->declare_parameter(plugin_name_ + ".graph_file", graph_file_);

  node_->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  node_->get_parameter(plugin_name_ + ".max_linear_vel", max_linear_vel_);
  node_->get_parameter(plugin_name_ + ".min_linear_vel", min_linear_vel_);
  node_->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  node_->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node_->get_parameter(plugin_name_ + ".yaw_gain", yaw_gain_);
  node_->get_parameter(
    plugin_name_ + ".heading_error_slowdown_threshold",
    heading_error_slowdown_threshold_);
  node_->get_parameter(plugin_name_ + ".goal_dist_tolerance", goal_dist_tolerance_);
  node_->get_parameter(plugin_name_ + ".goal_yaw_tolerance", goal_yaw_tolerance_);
  node_->get_parameter(
    plugin_name_ + ".rotate_to_goal_angular_vel",
    rotate_to_goal_angular_vel_);
  node_->get_parameter(
    plugin_name_ + ".rotate_in_place_heading_error",
    rotate_in_place_heading_error_);
  node_->get_parameter(
    plugin_name_ + ".slow_turn_heading_error",
    slow_turn_heading_error_);
  node_->get_parameter(
    plugin_name_ + ".turn_lookahead_scale",
    turn_lookahead_scale_);
  node_->get_parameter(
    plugin_name_ + ".path_search_ahead_poses",
    path_search_ahead_poses_);
  node_->get_parameter(
    plugin_name_ + ".path_search_behind_poses",
    path_search_behind_poses_);
  node_->get_parameter(plugin_name_ + ".graph_file", graph_file_);

  max_speed_limit_ = max_linear_vel_;
  path_search_ahead_poses_ = std::max(1, path_search_ahead_poses_);
  path_search_behind_poses_ = std::max(0, path_search_behind_poses_);
  loadLaneGraph();

  RCLCPP_INFO(
    logger_,
    "Configured RoadDrivingController: desired_linear_vel=%.3f, lookahead_dist=%.3f, max_angular_vel=%.3f, lane_debug=%s",
    desired_linear_vel_, lookahead_dist_, max_angular_vel_, graph_file_.empty() ? "disabled" : "enabled");
}

void RoadDrivingController::cleanup()
{
  global_plan_.poses.clear();
  lanes_.clear();
  costmap_ros_.reset();
  tf_.reset();
  node_.reset();
}

void RoadDrivingController::activate()
{
  cancel_requested_ = false;
  RCLCPP_INFO(logger_, "RoadDrivingController activated");
}

void RoadDrivingController::deactivate()
{
  RCLCPP_INFO(logger_, "RoadDrivingController deactivated");
}

void RoadDrivingController::setPlan(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    throw nav2_core::InvalidPath("RoadDrivingController received an empty path");
  }

  const auto plan_frame = !path.header.frame_id.empty() ? path.header.frame_id : path.poses.front().header.frame_id;
  if (plan_frame.empty()) {
    throw nav2_core::InvalidPath("RoadDrivingController received a path without a frame_id");
  }
  for (const auto & pose : path.poses) {
    if (!pose.header.frame_id.empty() && pose.header.frame_id != plan_frame) {
      throw nav2_core::InvalidPath("RoadDrivingController received a path with mixed pose frame_ids");
    }
  }

  global_plan_ = path;
  global_plan_.header.frame_id = plan_frame;
  for (auto & pose : global_plan_.poses) {
    pose.header.frame_id = plan_frame;
  }
  cancel_requested_ = false;
  path_progress_index_ = 0U;
}

geometry_msgs::msg::TwistStamped RoadDrivingController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist &,
  nav2_core::GoalChecker *)
{
  if (cancel_requested_) {
    throw nav2_core::ControllerException("RoadDrivingController cancelled");
  }

  if (global_plan_.poses.empty()) {
    throw nav2_core::InvalidPath("RoadDrivingController has no plan to follow");
  }

  const auto working_frame = costmap_ros_->getGlobalFrameID();
  const auto local_pose = transformPoseToFrame(pose, working_frame);
  const auto local_plan = transformPlanToFrame(working_frame);
  const auto plan_pose = transformPoseToFrame(pose, getPlanFrame());

  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = node_->now();
  command.header.frame_id = local_pose.header.frame_id;
  const auto previous_plan = std::move(global_plan_);
  global_plan_ = local_plan;

  try {
    const auto closest_index = findClosestPoseIndex(local_pose);
    path_progress_index_ = std::max(path_progress_index_, closest_index);

    const auto & goal_pose = global_plan_.poses.back();

    const double goal_distance = distance2D(local_pose.pose.position, goal_pose.pose.position);
    const double robot_yaw = tf2::getYaw(local_pose.pose.orientation);
    const double goal_yaw = tf2::getYaw(goal_pose.pose.orientation);
    const double goal_yaw_error = normalizeAngle(goal_yaw - robot_yaw);

    if (goal_distance <= goal_dist_tolerance_) {
      if (std::abs(goal_yaw_error) <= goal_yaw_tolerance_) {
        global_plan_ = previous_plan;
        return command;
      }

      command.twist.angular.z = std::clamp(
        yaw_gain_ * goal_yaw_error,
        -rotate_to_goal_angular_vel_,
        rotate_to_goal_angular_vel_);
      global_plan_ = previous_plan;
      return command;
    }

    auto lookahead_index = findLookaheadPoseIndex(
      local_pose, path_progress_index_, std::max(0.05, lookahead_dist_));
    auto target_heading = std::atan2(
      global_plan_.poses[lookahead_index].pose.position.y - local_pose.pose.position.y,
      global_plan_.poses[lookahead_index].pose.position.x - local_pose.pose.position.x);
    double heading_error = normalizeAngle(target_heading - robot_yaw);
    double abs_heading_error = std::abs(heading_error);

    if (abs_heading_error >= slow_turn_heading_error_) {
      const double shorter_lookahead = std::max(0.05, lookahead_dist_ * turn_lookahead_scale_);
      lookahead_index = findLookaheadPoseIndex(local_pose, path_progress_index_, shorter_lookahead);
      target_heading = std::atan2(
        global_plan_.poses[lookahead_index].pose.position.y - local_pose.pose.position.y,
        global_plan_.poses[lookahead_index].pose.position.x - local_pose.pose.position.x);
      heading_error = normalizeAngle(target_heading - robot_yaw);
      abs_heading_error = std::abs(heading_error);
    }

    if (abs_heading_error >= rotate_in_place_heading_error_) {
      const auto now = node_->now();
      if (!lanes_.empty() && (now - last_connector_trace_time_).seconds() >= 1.0) {
        double lane_distance = std::numeric_limits<double>::max();
        const auto * lane = findClosestLane(plan_pose, &lane_distance);
        if (lane != nullptr && lane->kind == "connector") {
          last_connector_trace_time_ = now;
          RCLCPP_WARN(
            logger_,
            "Connector trace: rotating in place near lane=%s kind=%s lane_dist=%.3f path_idx=%zu lookahead_idx=%zu heading_err=%.3f goal_dist=%.3f",
            lane->id.c_str(), lane->kind.c_str(), lane_distance, path_progress_index_,
            lookahead_index, heading_error, goal_distance);
        }
      }
      command.twist.angular.z = std::clamp(
        yaw_gain_ * heading_error,
        -max_angular_vel_,
        max_angular_vel_);
      global_plan_ = previous_plan;
      return command;
    }

    double linear_vel = std::min(desired_linear_vel_, max_speed_limit_);
    if (abs_heading_error >= slow_turn_heading_error_) {
      linear_vel = min_linear_vel_;
    } else if (abs_heading_error > heading_error_slowdown_threshold_) {
      linear_vel *= 0.5;
    }
    if (goal_distance < lookahead_dist_) {
      linear_vel *= std::clamp(goal_distance / lookahead_dist_, 0.3, 1.0);
    }
    linear_vel = std::clamp(linear_vel, min_linear_vel_, max_speed_limit_);

    command.twist.linear.x = linear_vel;
    command.twist.angular.z = std::clamp(
      yaw_gain_ * heading_error,
      -max_angular_vel_,
      max_angular_vel_);

    const auto now = node_->now();
    if (!lanes_.empty() && linear_vel <= (min_linear_vel_ + 1e-6) &&
      abs_heading_error >= slow_turn_heading_error_ &&
      (now - last_connector_trace_time_).seconds() >= 1.0)
    {
      double lane_distance = std::numeric_limits<double>::max();
      const auto * lane = findClosestLane(plan_pose, &lane_distance);
      if (lane != nullptr && lane->kind == "connector") {
        last_connector_trace_time_ = now;
        RCLCPP_WARN(
          logger_,
          "Connector trace: crawling near lane=%s kind=%s lane_dist=%.3f path_idx=%zu lookahead_idx=%zu heading_err=%.3f goal_dist=%.3f linear=%.3f angular=%.3f",
          lane->id.c_str(), lane->kind.c_str(), lane_distance, path_progress_index_,
          lookahead_index, heading_error, goal_distance,
          command.twist.linear.x, command.twist.angular.z);
      }
    }

    global_plan_ = previous_plan;
    return command;
  } catch (...) {
    global_plan_ = previous_plan;
    throw;
  }
}

bool RoadDrivingController::cancel()
{
  cancel_requested_ = true;
  return true;
}

void RoadDrivingController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    max_speed_limit_ = std::clamp(max_linear_vel_ * speed_limit / 100.0, min_linear_vel_, max_linear_vel_);
    return;
  }

  max_speed_limit_ = std::clamp(speed_limit, min_linear_vel_, max_linear_vel_);
}

void RoadDrivingController::reset()
{
  cancel_requested_ = false;
  path_progress_index_ = 0U;
}

void RoadDrivingController::loadLaneGraph()
{
  lanes_.clear();
  if (graph_file_.empty()) {
    return;
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(graph_file_);
  } catch (const YAML::Exception & ex) {
    throw nav2_core::ControllerException(
            "Failed to load controller lane graph file: " + std::string(ex.what()));
  }

  const auto lanes_node = root["lanes"];
  if (!lanes_node || !lanes_node.IsSequence()) {
    throw nav2_core::ControllerException(
            "Controller lane graph file must contain a top-level 'lanes' sequence");
  }

  for (const auto & lane_node : lanes_node) {
    LaneDebugInfo lane;
    lane.id = lane_node["id"].as<std::string>();
    lane.kind = lane_node["kind"] ? lane_node["kind"].as<std::string>() : "road";

    const auto centerline_node = lane_node["centerline"];
    if (!centerline_node || !centerline_node.IsSequence() || centerline_node.size() < 2U) {
      throw nav2_core::ControllerException(
              "Controller lane graph lane '" + lane.id + "' must contain at least two centerline points");
    }

    lane.centerline.reserve(centerline_node.size());
    for (const auto & point_node : centerline_node) {
      if (!point_node.IsSequence() || point_node.size() != 2U) {
        throw nav2_core::ControllerException(
                "Controller lane graph lane '" + lane.id + "' has an invalid centerline point definition");
      }

      geometry_msgs::msg::Point point;
      point.x = point_node[0].as<double>();
      point.y = point_node[1].as<double>();
      point.z = 0.0;
      lane.centerline.push_back(point);
    }

    lanes_.emplace(lane.id, std::move(lane));
  }
}

std::string RoadDrivingController::getPlanFrame() const
{
  if (!global_plan_.header.frame_id.empty()) {
    return global_plan_.header.frame_id;
  }
  if (!global_plan_.poses.empty()) {
    return global_plan_.poses.front().header.frame_id;
  }
  throw nav2_core::InvalidPath("RoadDrivingController has no plan frame");
}

geometry_msgs::msg::PoseStamped RoadDrivingController::transformPoseToFrame(
  const geometry_msgs::msg::PoseStamped & pose,
  const std::string & target_frame) const
{
  if (target_frame.empty()) {
    throw nav2_core::ControllerException("RoadDrivingController target frame is empty");
  }

  geometry_msgs::msg::PoseStamped transformed_pose = pose;
  if (transformed_pose.header.frame_id.empty()) {
    throw nav2_core::ControllerException("RoadDrivingController received pose without a frame_id");
  }

  if (transformed_pose.header.frame_id == target_frame) {
    return transformed_pose;
  }

  try {
    transformed_pose = tf_->transform(transformed_pose, target_frame, tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & ex) {
    throw nav2_core::ControllerException(
            "RoadDrivingController failed to transform pose from " +
            pose.header.frame_id + " to " + target_frame + ": " + ex.what());
  }

  return transformed_pose;
}

nav_msgs::msg::Path RoadDrivingController::transformPlanToFrame(const std::string & target_frame) const
{
  nav_msgs::msg::Path transformed_plan;
  transformed_plan.header = global_plan_.header;
  transformed_plan.header.frame_id = target_frame;
  transformed_plan.poses.reserve(global_plan_.poses.size());

  for (const auto & pose : global_plan_.poses) {
    transformed_plan.poses.push_back(transformPoseToFrame(pose, target_frame));
  }

  return transformed_plan;
}

const RoadDrivingController::LaneDebugInfo * RoadDrivingController::findClosestLane(
  const geometry_msgs::msg::PoseStamped & pose,
  double * best_distance) const
{
  const LaneDebugInfo * best_lane = nullptr;
  double min_distance = std::numeric_limits<double>::max();

  for (const auto & [lane_id, lane] : lanes_) {
    (void)lane_id;
    for (const auto & point : lane.centerline) {
      const double dist = distance2D(point, pose.pose.position);
      if (dist < min_distance) {
        min_distance = dist;
        best_lane = &lane;
      }
    }
  }

  if (best_distance != nullptr) {
    *best_distance = min_distance;
  }

  return best_lane;
}

std::size_t RoadDrivingController::findClosestPoseIndex(const geometry_msgs::msg::PoseStamped & pose) const
{
  std::size_t best_index = std::min(path_progress_index_, global_plan_.poses.size() - 1U);
  double best_distance = std::numeric_limits<double>::max();

  const std::size_t start_index = path_progress_index_ > static_cast<std::size_t>(path_search_behind_poses_) ?
    path_progress_index_ - static_cast<std::size_t>(path_search_behind_poses_) : 0U;
  const std::size_t end_index = std::min(
    global_plan_.poses.size(),
    path_progress_index_ + static_cast<std::size_t>(path_search_ahead_poses_) + 1U);

  for (std::size_t i = start_index; i < end_index; ++i) {
    const double dist = distance2D(pose.pose.position, global_plan_.poses[i].pose.position);
    if (dist < best_distance) {
      best_distance = dist;
      best_index = i;
    }
  }

  return best_index;
}

std::size_t RoadDrivingController::findLookaheadPoseIndex(
  const geometry_msgs::msg::PoseStamped & pose,
  const std::size_t start_index,
  const double lookahead_distance) const
{
  double accumulated_distance = 0.0;
  geometry_msgs::msg::Point previous_point = pose.pose.position;

  for (std::size_t i = start_index; i < global_plan_.poses.size(); ++i) {
    const auto & current_point = global_plan_.poses[i].pose.position;
    accumulated_distance += distance2D(previous_point, current_point);
    if (accumulated_distance >= lookahead_distance) {
      return i;
    }
    previous_point = current_point;
  }

  return global_plan_.poses.size() - 1U;
}

double RoadDrivingController::distance2D(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double RoadDrivingController::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

}  // namespace araseo_navigation

PLUGINLIB_EXPORT_CLASS(araseo_navigation::RoadDrivingController, nav2_core::Controller)
