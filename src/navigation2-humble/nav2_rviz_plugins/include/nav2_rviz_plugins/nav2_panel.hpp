













#ifndef NAV2_RVIZ_PLUGINS__NAV2_PANEL_HPP_
#define NAV2_RVIZ_PLUGINS__NAV2_PANEL_HPP_

#include <QtWidgets>
#include <QBasicTimer>
#undef NO_ERROR

#include <memory>
#include <string>
#include <vector>

#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_rviz_plugins/ros_action_qevent.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/panel.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav2_util/geometry_utils.hpp"

class QPushButton;

namespace nav2_rviz_plugins
{

class InitialThread;


class Nav2Panel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit Nav2Panel(QWidget * parent = 0);
  virtual ~Nav2Panel();

  void onInitialize() override;


  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void startThread();
  void onStartup();
  void onShutdown();
  void onCancel();
  void onPause();
  void onResume();
  void onAccumulatedWp();
  void onAccumulatedNTP();
  void onAccumulating();
  void onNewGoal(double x, double y, double theta, QString frame);

private:
  void loadLogFiles();
  void onCancelButtonPressed();
  void timerEvent(QTimerEvent * event) override;

  int unique_id {0};


  void startWaypointFollowing(std::vector<geometry_msgs::msg::PoseStamped> poses);
  void startNavigation(geometry_msgs::msg::PoseStamped);
  void startNavThroughPoses(std::vector<geometry_msgs::msg::PoseStamped> poses);
  using NavigationGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  using WaypointFollowerGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>;
  using NavThroughPosesGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>;


  rclcpp::Node::SharedPtr client_node_;


  std::chrono::milliseconds server_timeout_;


  QBasicTimer timer_;


  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr
    waypoint_follower_action_client_;
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr
    nav_through_poses_action_client_;


  rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>::SharedPtr
    navigation_feedback_sub_;
  rclcpp::Subscription<nav2_msgs::action::NavigateThroughPoses::Impl::FeedbackMessage>::SharedPtr
    nav_through_poses_feedback_sub_;
  rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Impl::GoalStatusMessage>::SharedPtr
    navigation_goal_status_sub_;
  rclcpp::Subscription<nav2_msgs::action::NavigateThroughPoses::Impl::GoalStatusMessage>::SharedPtr
    nav_through_poses_goal_status_sub_;


  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
  nav2_msgs::action::FollowWaypoints::Goal waypoint_follower_goal_;
  nav2_msgs::action::NavigateThroughPoses::Goal nav_through_poses_goal_;
  NavigationGoalHandle::SharedPtr navigation_goal_handle_;
  WaypointFollowerGoalHandle::SharedPtr waypoint_follower_goal_handle_;
  NavThroughPosesGoalHandle::SharedPtr nav_through_poses_goal_handle_;


  std::shared_ptr<nav2_lifecycle_manager::LifecycleManagerClient> client_nav_;
  std::shared_ptr<nav2_lifecycle_manager::LifecycleManagerClient> client_loc_;

  QPushButton * start_reset_button_{nullptr};
  QPushButton * pause_resume_button_{nullptr};
  QPushButton * navigation_mode_button_{nullptr};

  QLabel * navigation_status_indicator_{nullptr};
  QLabel * localization_status_indicator_{nullptr};
  QLabel * navigation_goal_status_indicator_{nullptr};
  QLabel * navigation_feedback_indicator_{nullptr};

  QStateMachine state_machine_;
  InitialThread * initial_thread_;

  QState * pre_initial_{nullptr};
  QState * initial_{nullptr};
  QState * idle_{nullptr};
  QState * reset_{nullptr};
  QState * paused_{nullptr};
  QState * resumed_{nullptr};




  QState * running_{nullptr};
  QState * canceled_{nullptr};


  QState * accumulating_{nullptr};
  QState * accumulated_wp_{nullptr};
  QState * accumulated_nav_through_poses_{nullptr};

  std::vector<geometry_msgs::msg::PoseStamped> acummulated_poses_;


  void updateWpNavigationMarkers();


  int getUniqueId();

  void resetUniqueId();


  static inline QString getGoalStatusLabel(
    int8_t status = action_msgs::msg::GoalStatus::STATUS_UNKNOWN);


  static inline QString getNavToPoseFeedbackLabel(
    nav2_msgs::action::NavigateToPose::Feedback msg =
    nav2_msgs::action::NavigateToPose::Feedback());
  static inline QString getNavThroughPosesFeedbackLabel(
    nav2_msgs::action::NavigateThroughPoses::Feedback =
    nav2_msgs::action::NavigateThroughPoses::Feedback());
  template<typename T>
  static inline std::string toLabel(T & msg);


  static inline std::string toString(double val, int precision = 0);


  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wp_navigation_markers_pub_;
};

class InitialThread : public QThread
{
  Q_OBJECT

public:
  using SystemStatus = nav2_lifecycle_manager::SystemStatus;

  explicit InitialThread(
    std::shared_ptr<nav2_lifecycle_manager::LifecycleManagerClient> & client_nav,
    std::shared_ptr<nav2_lifecycle_manager::LifecycleManagerClient> & client_loc)
  : client_nav_(client_nav), client_loc_(client_loc)
  {}

  void run() override
  {
    SystemStatus status_nav = SystemStatus::TIMEOUT;
    SystemStatus status_loc = SystemStatus::TIMEOUT;

    while (status_nav == SystemStatus::TIMEOUT) {
      if (status_nav == SystemStatus::TIMEOUT) {
        status_nav = client_nav_->is_active(std::chrono::seconds(1));
      }
    }


    bool tried_loc_bringup_once = false;
    while (status_loc == SystemStatus::TIMEOUT) {
      status_loc = client_loc_->is_active(std::chrono::seconds(1));
      if (tried_loc_bringup_once) {
        break;
      }
      tried_loc_bringup_once = true;
    }

    if (status_nav == SystemStatus::ACTIVE) {
      emit navigationActive();
    } else {
      emit navigationInactive();
    }

    if (status_loc == SystemStatus::ACTIVE) {
      emit localizationActive();
    } else {
      emit localizationInactive();
    }
  }

signals:
  void navigationActive();
  void navigationInactive();
  void localizationActive();
  void localizationInactive();

private:
  std::shared_ptr<nav2_lifecycle_manager::LifecycleManagerClient> client_nav_;
  std::shared_ptr<nav2_lifecycle_manager::LifecycleManagerClient> client_loc_;
};

}

#endif
