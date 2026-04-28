
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy




def main():
    rclpy.init()

    navigator = BasicNavigator()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 3.45
    initial_pose.pose.position.y = 2.15
    initial_pose.pose.orientation.z = 1.0
    initial_pose.pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = -3.0
    goal_pose.pose.position.y = -2.0
    goal_pose.pose.orientation.w = 1.0

    path = navigator.getPath(initial_pose, goal_pose)
    smoothed_path = navigator.smoothPath(path)

    navigator.followPath(smoothed_path)

    i = 0
    while not navigator.isTaskComplete():

        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated distance remaining to goal position: ' +
                  '{0:.3f}'.format(feedback.distance_to_goal) +
                  '\nCurrent speed of the robot: ' +
                  '{0:.3f}'.format(feedback.speed))

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()
