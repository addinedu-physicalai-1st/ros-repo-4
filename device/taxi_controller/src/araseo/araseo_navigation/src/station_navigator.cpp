#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "yaml-cpp/yaml.h"

#include "araseo_navigation/action/navigate_to_station.hpp"

namespace araseo_navigation
{

using namespace std::chrono_literals;

namespace
{

geometry_msgs::msg::Quaternion quaternionFromYaw(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

}  // namespace

class StationNavigator : public rclcpp::Node
{
public:
  using NavigateToStation = araseo_navigation::action::NavigateToStation;
  using GoalHandleNavigateToStation = rclcpp_action::ServerGoalHandle<NavigateToStation>;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  struct Station
  {
    std::string station_id;
    std::string display_name;
    geometry_msgs::msg::PoseStamped pose;
  };

  StationNavigator()
  : Node("station_navigator")
  {
    declare_parameter("stations_file", std::string(""));
    declare_parameter("map_frame", std::string("map"));
    declare_parameter("navigation_action_name", std::string("navigate_to_pose"));
    declare_parameter("default_behavior_tree", std::string(""));

    map_frame_ = get_parameter("map_frame").as_string();
    default_behavior_tree_ = get_parameter("default_behavior_tree").as_string();
    navigation_action_name_ = get_parameter("navigation_action_name").as_string();
    const auto stations_file = get_parameter("stations_file").as_string();
    const auto use_sim_time = get_parameter("use_sim_time").as_bool();

    navigate_to_pose_client_ =
      rclcpp_action::create_client<NavigateToPose>(this, navigation_action_name_);
    action_server_ = rclcpp_action::create_server<NavigateToStation>(
      this,
      "navigate_to_station",
      std::bind(&StationNavigator::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&StationNavigator::handleCancel, this, std::placeholders::_1),
      std::bind(&StationNavigator::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Station navigator configured with action='%s', map_frame='%s', use_sim_time=%s, bt='%s'",
      navigation_action_name_.c_str(), map_frame_.c_str(), use_sim_time ? "true" : "false",
      default_behavior_tree_.empty() ? "<default>" : default_behavior_tree_.c_str());

    loadStations(stations_file);
  }

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const NavigateToStation::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal request for station_id='%s'", goal->station_id.c_str());
    if (stations_.find(goal->station_id) == stations_.end()) {
      RCLCPP_WARN(
        get_logger(), "Rejected station goal for unknown station_id='%s'",
        goal->station_id.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleNavigateToStation> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    RCLCPP_INFO(get_logger(), "Received cancellation request for station_id='%s'", goal->station_id.c_str());
    std::scoped_lock lock(active_nav_goal_mutex_);
    if (active_nav_goal_) {
      (void)navigate_to_pose_client_->async_cancel_goal(active_nav_goal_);
    }

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleNavigateToStation> goal_handle)
  {
    std::thread{std::bind(&StationNavigator::execute, this, std::placeholders::_1), goal_handle}
    .detach();
  }

  void execute(const std::shared_ptr<GoalHandleNavigateToStation> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    const auto station_it = stations_.find(goal->station_id);
    auto result = std::make_shared<NavigateToStation::Result>();
    result->station_id = goal->station_id;

    if (station_it == stations_.end()) {
      result->success = false;
      result->message = "Unknown station_id";
      goal_handle->abort(result);
      return;
    }

    const auto station = station_it->second;

    RCLCPP_INFO(
      get_logger(),
      "Sending station goal station_id='%s' to '%s' with frame='%s' pose=(%.3f, %.3f)",
      station.station_id.c_str(), navigation_action_name_.c_str(),
      station.pose.header.frame_id.c_str(), station.pose.pose.position.x, station.pose.pose.position.y);

    if (!navigate_to_pose_client_->wait_for_action_server(10s)) {
      result->success = false;
      result->message = "navigate_to_pose action server unavailable; check Nav2 bringup and use_sim_time";
      goal_handle->abort(result);
      return;
    }

    NavigateToPose::Goal nav_goal;
    nav_goal.pose = station.pose;
    nav_goal.behavior_tree = default_behavior_tree_;

    std::promise<std::shared_ptr<GoalHandleNavigateToPose>> goal_promise;
    auto goal_future = goal_promise.get_future();
    std::promise<GoalHandleNavigateToPose::WrappedResult> result_promise;
    auto nav_result_future = result_promise.get_future();

    typename rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
      [&goal_promise](const std::shared_ptr<GoalHandleNavigateToPose> nav_goal_handle) mutable {
        goal_promise.set_value(nav_goal_handle);
      };
    options.feedback_callback =
      [this, weak_goal_handle = std::weak_ptr<GoalHandleNavigateToStation>(goal_handle), station](
      GoalHandleNavigateToPose::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback)
      {
        const auto locked_goal_handle = weak_goal_handle.lock();
        if (!locked_goal_handle || !locked_goal_handle->is_active()) {
          return;
        }

        auto station_feedback = std::make_shared<NavigateToStation::Feedback>();
        station_feedback->station_id = station.station_id;
        station_feedback->display_name = station.display_name;
        station_feedback->target_pose = station.pose;
        station_feedback->navigation_time = feedback->navigation_time;
        station_feedback->estimated_time_remaining = feedback->estimated_time_remaining;
        station_feedback->number_of_recoveries = feedback->number_of_recoveries;
        station_feedback->distance_remaining = feedback->distance_remaining;
        locked_goal_handle->publish_feedback(station_feedback);

        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Navigating to station '%s': distance_remaining=%.2f m, estimated_time=%.1f s",
          station.station_id.c_str(), feedback->distance_remaining,
          rclcpp::Duration(feedback->estimated_time_remaining).seconds());
      };
    options.result_callback =
      [&result_promise](const GoalHandleNavigateToPose::WrappedResult & wrapped_result) mutable {
        result_promise.set_value(wrapped_result);
      };

    navigate_to_pose_client_->async_send_goal(nav_goal, options);
    const auto nav_goal_handle = goal_future.get();
    if (!nav_goal_handle) {
      result->success = false;
      result->message = "navigate_to_pose rejected goal";
      goal_handle->abort(result);
      return;
    }

    {
      std::scoped_lock lock(active_nav_goal_mutex_);
      active_nav_goal_ = nav_goal_handle;
    }

    const auto wrapped_result = nav_result_future.get();

    {
      std::scoped_lock lock(active_nav_goal_mutex_);
      active_nav_goal_.reset();
    }

    result->error_code = wrapped_result.result ? wrapped_result.result->error_code : 0U;
    if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
      wrapped_result.result && wrapped_result.result->error_code == NavigateToPose::Result::NONE)
    {
      RCLCPP_INFO(get_logger(), "Successfully arrived at station_id='%s'", station.station_id.c_str());
      result->success = true;
      result->message = "Arrived at station";
      goal_handle->succeed(result);
      return;
    }

    if (wrapped_result.code == rclcpp_action::ResultCode::CANCELED || goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Navigation canceled";
      RCLCPP_INFO(
        get_logger(),
        "Station navigation canceled for station_id='%s' (nav2_error_code=%u)",
        station.station_id.c_str(), result->error_code);
      goal_handle->canceled(result);
      return;
    }

    result->success = false;
    result->message = wrapped_result.result ?
      "Navigation failed: " + wrapped_result.result->error_msg +
      " (error_code=" + std::to_string(wrapped_result.result->error_code) + ")" :
      "Navigation failed without Nav2 result";
    RCLCPP_WARN(
      get_logger(),
      "Station navigation failed for station_id='%s' via action='%s': result_code=%d, nav2_error_code=%u, message='%s'",
      station.station_id.c_str(), navigation_action_name_.c_str(),
      static_cast<int>(wrapped_result.code), result->error_code, result->message.c_str());
    goal_handle->abort(result);
  }

  void loadStations(const std::string & stations_file)
  {
    if (stations_file.empty()) {
      throw std::runtime_error("stations_file parameter must not be empty");
    }

    YAML::Node root = YAML::LoadFile(stations_file);
    const auto stations_node = root["stations"];
    if (!stations_node || !stations_node.IsSequence()) {
      throw std::runtime_error("stations file must contain a top-level 'stations' sequence");
    }

    for (const auto & station_node : stations_node) {
      Station station;
      station.station_id = station_node["station_id"].as<std::string>();
      station.display_name = station_node["display_name"] ?
        station_node["display_name"].as<std::string>() : station.station_id;

      if (stations_.find(station.station_id) != stations_.end()) {
        throw std::runtime_error("duplicate station_id: " + station.station_id);
      }

      const auto pose_node = station_node["pose"];
      if (!pose_node) {
        throw std::runtime_error("station '" + station.station_id + "' is missing pose");
      }

      station.pose.header.frame_id = map_frame_;
      station.pose.pose.position.x = pose_node["x"].as<double>();
      station.pose.pose.position.y = pose_node["y"].as<double>();
      station.pose.pose.position.z = pose_node["z"] ? pose_node["z"].as<double>() : 0.0;
      station.pose.pose.orientation =
        quaternionFromYaw(pose_node["yaw"].as<double>());

      stations_.emplace(station.station_id, std::move(station));
    }

    RCLCPP_INFO(get_logger(), "Loaded %zu stations", stations_.size());
  }

  std::string map_frame_;
  std::string default_behavior_tree_;
  std::string navigation_action_name_;
  std::unordered_map<std::string, Station> stations_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
  rclcpp_action::Server<NavigateToStation>::SharedPtr action_server_;
  std::mutex active_nav_goal_mutex_;
  std::shared_ptr<GoalHandleNavigateToPose> active_nav_goal_;
};

}  // namespace araseo_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<araseo_navigation::StationNavigator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
