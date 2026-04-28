













#ifndef NAV2_RVIZ_PLUGINS__ROS_ACTION_QEVENT_HPP_
#define NAV2_RVIZ_PLUGINS__ROS_ACTION_QEVENT_HPP_

#include <QAbstractTransition>

namespace nav2_rviz_plugins
{

enum class QActionState
{
  ACTIVE,
  INACTIVE
};


struct ROSActionQEvent : public QEvent
{
  explicit ROSActionQEvent(QActionState state)
  : QEvent(QEvent::Type(QEvent::User + 1)),
    state_(state) {}

  QActionState state_;
};


class ROSActionQTransition : public QAbstractTransition
{
public:
  explicit ROSActionQTransition(QActionState initial_status)
  : status_(initial_status)
  {}

  ~ROSActionQTransition() {}

protected:
  virtual bool eventTest(QEvent * e)
  {
    if (e->type() != QEvent::Type(QEvent::User + 1)) {
      return false;
    }
    ROSActionQEvent * action_event = static_cast<ROSActionQEvent *>(e);
    return status_ != action_event->state_;
  }

  virtual void onTransition(QEvent *) {}
  QActionState status_;
};

}

#endif
