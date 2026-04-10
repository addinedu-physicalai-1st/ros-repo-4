#include "araseo_navigation/log_custom_bt_loaded.hpp"

#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

namespace araseo_navigation
{

LogCustomBtLoaded::LogCustomBtLoaded(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList LogCustomBtLoaded::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "message",
      "[AraseoNavigation] custom NavigateToPose BT loaded")
  };
}

BT::NodeStatus LogCustomBtLoaded::tick()
{
  if (!has_logged_) {
    const auto message = getInput<std::string>("message")
      .value_or("[AraseoNavigation] custom NavigateToPose BT loaded");
    RCLCPP_WARN(
      rclcpp::get_logger("araseo_navigation.bt"),
      "[BT:%s] %s",
      name().c_str(),
      message.c_str());
    has_logged_ = true;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace araseo_navigation

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<araseo_navigation::LogCustomBtLoaded>("LogCustomBtLoaded");
}
