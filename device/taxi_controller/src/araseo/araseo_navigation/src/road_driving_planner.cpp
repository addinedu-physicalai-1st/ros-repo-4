#include "araseo_navigation/road_driving_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <tuple>
#include <unordered_set>
#include <utility>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
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

geometry_msgs::msg::Point interpolatePoint(
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & end,
  const double ratio)
{
  geometry_msgs::msg::Point point;
  point.x = start.x + (end.x - start.x) * ratio;
  point.y = start.y + (end.y - start.y) * ratio;
  point.z = start.z + (end.z - start.z) * ratio;
  return point;
}

}  // namespace

void LaneGraphPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  if (!node_) {
    throw nav2_core::PlannerException("Failed to lock parent node in LaneGraphPlanner");
  }

  name_ = std::move(name);
  logger_ = node_->get_logger();
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  global_frame_ = costmap_ros_->getGlobalFrameID();

  node_->declare_parameter(name_ + ".graph_file", std::string(""));
  node_->declare_parameter(name_ + ".lane_heading_weight", lane_heading_weight_);
  node_->declare_parameter(name_ + ".lane_sampling_resolution", lane_sampling_resolution_);
  node_->declare_parameter(name_ + ".path_resolution", path_resolution_);
  node_->declare_parameter(name_ + ".lane_candidate_count", static_cast<int>(lane_candidate_count_));
  node_->declare_parameter(name_ + ".connector_match_penalty", connector_match_penalty_);
  node_->declare_parameter(name_ + ".lane_sequence_cost_weight", lane_sequence_cost_weight_);

  node_->get_parameter(name_ + ".graph_file", graph_file_);
  node_->get_parameter(name_ + ".lane_heading_weight", lane_heading_weight_);
  node_->get_parameter(name_ + ".lane_sampling_resolution", lane_sampling_resolution_);
  node_->get_parameter(name_ + ".path_resolution", path_resolution_);
  int lane_candidate_count = static_cast<int>(lane_candidate_count_);
  node_->get_parameter(name_ + ".lane_candidate_count", lane_candidate_count);
  node_->get_parameter(name_ + ".connector_match_penalty", connector_match_penalty_);
  node_->get_parameter(name_ + ".lane_sequence_cost_weight", lane_sequence_cost_weight_);
  lane_candidate_count_ = std::max(1, lane_candidate_count);

  if (graph_file_.empty()) {
    throw nav2_core::PlannerException("LaneGraphPlanner requires a non-empty graph_file parameter");
  }

  loadLaneGraph();
  RCLCPP_INFO(
    logger_,
    "Configured LaneGraphPlanner with %zu directed lanes from %s",
    lanes_.size(), graph_file_.c_str());
}

void LaneGraphPlanner::cleanup()
{
  lanes_.clear();
  lane_order_.clear();
  costmap_ros_.reset();
  tf_.reset();
  node_.reset();
}

void LaneGraphPlanner::activate()
{
  RCLCPP_INFO(logger_, "LaneGraphPlanner activated");
}

void LaneGraphPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "LaneGraphPlanner deactivated");
}

nav_msgs::msg::Path LaneGraphPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    throw nav2_core::PlannerException(
            "LaneGraphPlanner received start/goal outside global frame " + global_frame_);
  }

  if (lanes_.empty()) {
    throw nav2_core::PlannerException("Lane graph is empty");
  }

  if (cancel_checker()) {
    throw nav2_core::PlannerCancelled("LaneGraphPlanner request cancelled before search");
  }

  const auto start_candidates = findLaneCandidates(start, true);
  const auto goal_candidates = findLaneCandidates(goal, false);
  if (cancel_checker()) {
    throw nav2_core::PlannerCancelled("LaneGraphPlanner request cancelled during lane matching");
  }
  if (start_candidates.empty() || goal_candidates.empty()) {
    throw nav2_core::PlannerException("Unable to match start/goal onto lane graph");
  }

  LaneMatch start_match;
  LaneMatch goal_match;
  std::vector<std::string> lane_sequence;
  double best_total_score = std::numeric_limits<double>::max();
  double best_sequence_cost = std::numeric_limits<double>::max();
  std::size_t best_start_order = std::numeric_limits<std::size_t>::max();
  std::size_t best_goal_order = std::numeric_limits<std::size_t>::max();

  for (const auto & start_candidate : start_candidates) {
    for (const auto & goal_candidate : goal_candidates) {
      std::vector<std::string> candidate_sequence;
      try {
        candidate_sequence = searchLaneSequence(
          start_candidate.match.lane_id,
          goal_candidate.match.lane_id);
      } catch (const nav2_core::PlannerException &) {
        continue;
      }

      const double sequence_cost = computeLaneSequenceCost(candidate_sequence);
      const double total_score =
        start_candidate.penalized_score +
        goal_candidate.penalized_score +
        lane_sequence_cost_weight_ * sequence_cost;
      const auto & start_lane = lanes_.at(start_candidate.match.lane_id);
      const auto & goal_lane = lanes_.at(goal_candidate.match.lane_id);
      const bool better = total_score + 1e-6 < best_total_score ||
        (std::abs(total_score - best_total_score) <= 1e-6 &&
        (sequence_cost + 1e-6 < best_sequence_cost ||
        (std::abs(sequence_cost - best_sequence_cost) <= 1e-6 &&
        std::tie(start_lane.order, goal_lane.order) < std::tie(best_start_order, best_goal_order))));

      if (better) {
        best_total_score = total_score;
        best_sequence_cost = sequence_cost;
        best_start_order = start_lane.order;
        best_goal_order = goal_lane.order;
        start_match = start_candidate.match;
        goal_match = goal_candidate.match;
        lane_sequence = std::move(candidate_sequence);
      }
    }
  }

  if (lane_sequence.empty()) {
    throw nav2_core::PlannerException("No reachable lane pair found for start/goal poses");
  }

  auto path = buildPathFromLaneSequence(start, goal, start_match, goal_match, lane_sequence);

  if (path.poses.empty()) {
    throw nav2_core::PlannerException("LaneGraphPlanner produced an empty path");
  }

  RCLCPP_INFO(
    logger_,
    "Planned %zu poses through %zu lanes from %s to %s (total score %.3f)",
    path.poses.size(), lane_sequence.size(),
    start_match.lane_id.c_str(), goal_match.lane_id.c_str(), best_total_score);
  return path;
}

void LaneGraphPlanner::loadLaneGraph()
{
  lanes_.clear();
  lane_order_.clear();

  YAML::Node root;
  try {
    root = YAML::LoadFile(graph_file_);
  } catch (const std::exception & ex) {
    throw nav2_core::PlannerException("Failed to load lane graph file: " + std::string(ex.what()));
  }

  const auto lanes_node = root["lanes"];
  if (!lanes_node || !lanes_node.IsSequence()) {
    throw nav2_core::PlannerException("Lane graph file must contain a top-level 'lanes' sequence");
  }

  std::size_t lane_order = 0U;
  for (const auto & lane_node : lanes_node) {
    Lane lane;
    lane.id = lane_node["id"].as<std::string>();
    lane.kind = lane_node["kind"] ? lane_node["kind"].as<std::string>() : "road";
    lane.order = lane_order++;

    const auto centerline_node = lane_node["centerline"];
    if (!centerline_node || !centerline_node.IsSequence() || centerline_node.size() < 2U) {
      throw nav2_core::PlannerException("Lane '" + lane.id + "' must contain at least two centerline points");
    }

    lane.centerline.reserve(centerline_node.size());
    for (const auto & point_node : centerline_node) {
      if (!point_node.IsSequence() || point_node.size() < 2U) {
        throw nav2_core::PlannerException(
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

    lane.length = computeLaneLength(lane.centerline);
    lane_order_.push_back(lane.id);
    lanes_.emplace(lane.id, std::move(lane));
  }

  for (const auto & [lane_id, lane] : lanes_) {
    for (const auto & successor : lane.successors) {
      if (lanes_.find(successor) == lanes_.end()) {
        throw nav2_core::PlannerException(
                "Lane '" + lane_id + "' references unknown successor '" + successor + "'");
      }
    }
  }
}

std::vector<LaneGraphPlanner::LaneCandidate> LaneGraphPlanner::findLaneCandidates(
  const geometry_msgs::msg::PoseStamped & pose,
  const bool prefer_forward_half) const
{
  std::vector<LaneCandidate> candidates;

  const auto pose_heading = poseYaw(pose);
  for (const auto & lane_id : lane_order_) {
    const auto lane_it = lanes_.find(lane_id);
    if (lane_it == lanes_.end()) {
      continue;
    }

    const auto & lane = lane_it->second;
    auto sampled = sampleLaneCenterline(lane);
    if (sampled.size() < 2U) {
      continue;
    }

    std::size_t start_index = 0U;
    std::size_t end_index = sampled.size();
    if (prefer_forward_half && sampled.size() > 4U) {
      end_index = std::max<std::size_t>(2U, sampled.size() * 3U / 4U);
    } else if (!prefer_forward_half && sampled.size() > 4U) {
      start_index = sampled.size() / 4U;
    }

    LaneCandidate best_for_lane;
    best_for_lane.match.lane_id = lane_id;

    for (std::size_t i = start_index; i < end_index; ++i) {
      const auto & point = sampled[i];
      const double distance = distance2D(point, pose.pose);
      const double heading = headingAtIndex(sampled, i);
      const double heading_error = std::abs(normalizeAngle(heading - pose_heading));
      const double score = distance + lane_heading_weight_ * heading_error;
      const double penalized_score = score + (lane.kind == "connector" ? connector_match_penalty_ : 0.0);

      if (penalized_score < best_for_lane.penalized_score) {
        best_for_lane.match.waypoint_index = i;
        best_for_lane.match.score = penalized_score;
        best_for_lane.raw_score = score;
        best_for_lane.penalized_score = penalized_score;
      }
    }

    if (!best_for_lane.match.lane_id.empty() &&
      best_for_lane.penalized_score < std::numeric_limits<double>::max())
    {
      candidates.push_back(best_for_lane);
    }
  }

  if (candidates.empty()) {
    throw nav2_core::PlannerException("Unable to match pose onto any lane centerline");
  }

  std::sort(
    candidates.begin(), candidates.end(),
    [this](const LaneCandidate & a, const LaneCandidate & b) {
      if (std::abs(a.penalized_score - b.penalized_score) > 1e-6) {
        return a.penalized_score < b.penalized_score;
      }

      const auto & lane_a = lanes_.at(a.match.lane_id);
      const auto & lane_b = lanes_.at(b.match.lane_id);
      if (lane_a.kind != lane_b.kind) {
        return lane_a.kind == "road";
      }
      return lane_a.order < lane_b.order;
    });

  if (candidates.size() > lane_candidate_count_) {
    candidates.resize(lane_candidate_count_);
  }

  return candidates;
}

std::vector<std::string> LaneGraphPlanner::searchLaneSequence(
  const std::string & start_lane_id,
  const std::string & goal_lane_id) const
{
  struct QueueEntry
  {
    double cost;
    std::string lane_id;
    bool operator>(const QueueEntry & other) const { return cost > other.cost; }
  };

  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> frontier;
  std::unordered_map<std::string, double> cost_so_far;
  std::unordered_map<std::string, std::string> parent;

  frontier.push({0.0, start_lane_id});
  cost_so_far[start_lane_id] = 0.0;

  while (!frontier.empty()) {
    const auto current = frontier.top();
    frontier.pop();

    if (current.lane_id == goal_lane_id) {
      break;
    }

    const auto lane_it = lanes_.find(current.lane_id);
    if (lane_it == lanes_.end()) {
      continue;
    }

    for (const auto & successor : lane_it->second.successors) {
      const auto successor_it = lanes_.find(successor);
      if (successor_it == lanes_.end()) {
        continue;
      }

      const double candidate_cost = current.cost + successor_it->second.length;
      const auto cost_it = cost_so_far.find(successor);
      if (cost_it == cost_so_far.end() || candidate_cost < cost_it->second) {
        cost_so_far[successor] = candidate_cost;
        parent[successor] = current.lane_id;
        frontier.push({candidate_cost, successor});
      }
    }
  }

  if (cost_so_far.find(goal_lane_id) == cost_so_far.end()) {
    throw nav2_core::PlannerException(
            "No lane sequence found between '" + start_lane_id + "' and '" + goal_lane_id + "'");
  }

  std::vector<std::string> lane_sequence;
  for (std::string lane_id = goal_lane_id; !lane_id.empty();) {
    lane_sequence.push_back(lane_id);
    const auto parent_it = parent.find(lane_id);
    if (parent_it == parent.end()) {
      break;
    }
    lane_id = parent_it->second;
  }

  std::reverse(lane_sequence.begin(), lane_sequence.end());
  return lane_sequence;
}

double LaneGraphPlanner::computeLaneSequenceCost(
  const std::vector<std::string> & lane_sequence) const
{
  double cost = 0.0;
  for (const auto & lane_id : lane_sequence) {
    const auto lane_it = lanes_.find(lane_id);
    if (lane_it != lanes_.end()) {
      cost += lane_it->second.length;
    }
  }
  return cost;
}

std::vector<geometry_msgs::msg::Point> LaneGraphPlanner::sampleLaneCenterline(
  const Lane & lane) const
{
  if (lane.centerline.empty()) {
    return {};
  }

  std::vector<geometry_msgs::msg::Point> sampled;
  sampled.push_back(lane.centerline.front());

  for (std::size_t i = 1; i < lane.centerline.size(); ++i) {
    const auto & previous = lane.centerline[i - 1U];
    const auto & current = lane.centerline[i];
    const double segment_length = distance2D(previous, current);

    if (segment_length <= lane_sampling_resolution_) {
      sampled.push_back(current);
      continue;
    }

    const auto steps = static_cast<std::size_t>(std::floor(segment_length / lane_sampling_resolution_));
    for (std::size_t step = 1; step <= steps; ++step) {
      const double ratio = std::min(1.0, (step * lane_sampling_resolution_) / segment_length);
      sampled.push_back(interpolatePoint(previous, current, ratio));
    }

    if (distance2D(sampled.back(), current) > 1e-6) {
      sampled.push_back(current);
    }
  }

  return sampled;
}

nav_msgs::msg::Path LaneGraphPlanner::buildPathFromLaneSequence(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const LaneMatch & start_match,
  const LaneMatch & goal_match,
  const std::vector<std::string> & lane_sequence) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = node_->now();

  std::vector<geometry_msgs::msg::Point> stitched_points;
  stitched_points.push_back(start.pose.position);

  for (std::size_t lane_index = 0; lane_index < lane_sequence.size(); ++lane_index) {
    const auto lane_it = lanes_.find(lane_sequence[lane_index]);
    if (lane_it == lanes_.end()) {
      continue;
    }

    auto sampled = sampleLaneCenterline(lane_it->second);
    if (sampled.empty()) {
      continue;
    }

    std::size_t begin_index = 0U;
    std::size_t end_index = sampled.size() - 1U;
    if (lane_sequence.size() == 1U) {
      begin_index = std::min(start_match.waypoint_index, sampled.size() - 1U);
      end_index = std::min(goal_match.waypoint_index, sampled.size() - 1U);
      if (end_index < begin_index) {
        std::swap(begin_index, end_index);
      }
    } else {
      if (lane_index == 0U) {
        begin_index = std::min(start_match.waypoint_index, sampled.size() - 1U);
      }
      if (lane_index + 1U == lane_sequence.size()) {
        end_index = std::min(goal_match.waypoint_index, sampled.size() - 1U);
      }
    }

    for (std::size_t point_index = begin_index; point_index <= end_index; ++point_index) {
      const auto & point = sampled[point_index];
      if (distance2D(stitched_points.back(), point) > std::max(1e-4, path_resolution_ * 0.25)) {
        stitched_points.push_back(point);
      }
    }
  }

  if (distance2D(stitched_points.back(), goal.pose.position) > 1e-4) {
    stitched_points.push_back(goal.pose.position);
  }

  for (std::size_t i = 0; i < stitched_points.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = stitched_points[i];

    double yaw = 0.0;
    if (stitched_points.size() > 1U) {
      if (i + 1U < stitched_points.size()) {
        yaw = std::atan2(
          stitched_points[i + 1U].y - stitched_points[i].y,
          stitched_points[i + 1U].x - stitched_points[i].x);
      } else {
        yaw = std::atan2(
          stitched_points[i].y - stitched_points[i - 1U].y,
          stitched_points[i].x - stitched_points[i - 1U].x);
      }
    }

    pose.pose.orientation = createQuaternionMsgFromYaw(yaw);
    path.poses.push_back(std::move(pose));
  }

  if (!path.poses.empty()) {
    path.poses.front().pose.orientation = start.pose.orientation;
    path.poses.back().pose.orientation = goal.pose.orientation;
  }

  return path;
}

double LaneGraphPlanner::distance2D(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double LaneGraphPlanner::distance2D(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Pose & b)
{
  return std::hypot(a.x - b.position.x, a.y - b.position.y);
}

double LaneGraphPlanner::distance2D(
  const geometry_msgs::msg::Pose & a,
  const geometry_msgs::msg::Pose & b)
{
  return std::hypot(a.position.x - b.position.x, a.position.y - b.position.y);
}

double LaneGraphPlanner::computeLaneLength(const std::vector<geometry_msgs::msg::Point> & centerline)
{
  double length = 0.0;
  for (std::size_t i = 1; i < centerline.size(); ++i) {
    length += distance2D(centerline[i - 1U], centerline[i]);
  }
  return length;
}

double LaneGraphPlanner::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double LaneGraphPlanner::poseYaw(const geometry_msgs::msg::PoseStamped & pose)
{
  return tf2::getYaw(pose.pose.orientation);
}

double LaneGraphPlanner::headingAtIndex(
  const std::vector<geometry_msgs::msg::Point> & points,
  const std::size_t index)
{
  if (points.size() < 2U) {
    return 0.0;
  }

  if (index + 1U < points.size()) {
    return std::atan2(
      points[index + 1U].y - points[index].y,
      points[index + 1U].x - points[index].x);
  }

  return std::atan2(
    points[index].y - points[index - 1U].y,
    points[index].x - points[index - 1U].x);
}

}  // namespace araseo_navigation

PLUGINLIB_EXPORT_CLASS(araseo_navigation::LaneGraphPlanner, nav2_core::GlobalPlanner)
