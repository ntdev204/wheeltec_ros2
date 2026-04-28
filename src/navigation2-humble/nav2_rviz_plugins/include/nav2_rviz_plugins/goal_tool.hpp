













#ifndef NAV2_RVIZ_PLUGINS__GOAL_TOOL_HPP_
#define NAV2_RVIZ_PLUGINS__GOAL_TOOL_HPP_

#include <QObject>

#include <memory>

#include "rviz_default_plugins/tools/pose/pose_tool.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_common
{

class DisplayContext;

namespace properties
{
class StringProperty;
}
}

namespace nav2_rviz_plugins
{

class RVIZ_DEFAULT_PLUGINS_PUBLIC GoalTool : public rviz_default_plugins::tools::PoseTool
{
  Q_OBJECT

public:
  GoalTool();
  ~GoalTool() override;

  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double theta) override;
};

}

#endif
