#ifndef ARASEO_NAVIGATION__LOG_CUSTOM_BT_LOADED_HPP_
#define ARASEO_NAVIGATION__LOG_CUSTOM_BT_LOADED_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"

namespace araseo_navigation
{

class LogCustomBtLoaded : public BT::SyncActionNode
{
public:
  LogCustomBtLoaded(const std::string & name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  bool has_logged_{false};
};

}  // namespace araseo_navigation

#endif  // ARASEO_NAVIGATION__LOG_CUSTOM_BT_LOADED_HPP_
