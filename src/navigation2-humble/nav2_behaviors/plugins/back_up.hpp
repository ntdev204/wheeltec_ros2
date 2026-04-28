













#ifndef NAV2_BEHAVIORS__PLUGINS__BACK_UP_HPP_
#define NAV2_BEHAVIORS__PLUGINS__BACK_UP_HPP_

#include <memory>

#include "drive_on_heading.hpp"
#include "nav2_msgs/action/back_up.hpp"

using BackUpAction = nav2_msgs::action::BackUp;


namespace nav2_behaviors
{
class BackUp : public DriveOnHeading<nav2_msgs::action::BackUp>
{
public:
  Status onRun(const std::shared_ptr<const BackUpAction::Goal> command) override;
};
}

#endif
