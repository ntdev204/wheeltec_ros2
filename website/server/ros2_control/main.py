import sys
import os
import rclpy

# Add server path to Python path so we can import app config
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from app.config import settings
from ros2_control.node import WheeltecControlNode

def main(args=None):
    # Set ROS_DOMAIN_ID to match the robot
    os.environ['ROS_DOMAIN_ID'] = str(settings.ros_domain_id)
    
    rclpy.init(args=args)
    
    zmq_ports = {
        "cmd": settings.zmq_cmd_port,
        "telemetry": settings.zmq_telemetry_port,
        "camera": settings.zmq_camera_port
    }
    
    node = WheeltecControlNode(zmq_ports)
    print(f"L2: ROS2 Control Node started on DOMAIN_ID {settings.ros_domain_id}")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
