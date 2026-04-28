
from time import sleep

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
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

    navigator.assistedTeleop(time_allowance=20)
    while not navigator.isTaskComplete():
        sleep(0.2)
        pass

    navigator.lifecycleShutdown()
    exit(0)


if __name__ == '__main__':
    main()
