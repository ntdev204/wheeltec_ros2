













#include "nav2_rviz_plugins/goal_tool.hpp"

#include <memory>
#include <string>

#include "nav2_rviz_plugins/goal_common.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/load_resource.hpp"

namespace nav2_rviz_plugins
{

GoalTool::GoalTool()
: rviz_default_plugins::tools::PoseTool()
{
  shortcut_key_ = 'g';
}

GoalTool::~GoalTool()
{
}

void GoalTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Nav2 Goal");
  setIcon(rviz_common::loadPixmap("package:
}

void
GoalTool::onPoseSet(double x, double y, double theta)
{

  GoalUpdater.setGoal(x, y, theta, context_->getFixedFrame());
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::GoalTool, rviz_common::Tool)
