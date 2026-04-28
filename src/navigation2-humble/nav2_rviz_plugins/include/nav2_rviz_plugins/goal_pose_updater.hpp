













#ifndef NAV2_RVIZ_PLUGINS__GOAL_POSE_UPDATER_HPP_
#define NAV2_RVIZ_PLUGINS__GOAL_POSE_UPDATER_HPP_

#include <QObject>

namespace nav2_rviz_plugins
{


class GoalPoseUpdater : public QObject
{
  Q_OBJECT

public:
  GoalPoseUpdater() {}
  ~GoalPoseUpdater() {}

  void setGoal(double x, double y, double theta, QString frame)
  {
    emit updateGoal(x, y, theta, frame);
  }

signals:
  void updateGoal(double x, double y, double theta, QString frame);
};

}

#endif
