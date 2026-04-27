#!/usr/bin/env python3
"""
Context-Aware Safety Shield (Lá chắn an toàn ngầm)
- Chặn giữa Twist_Mux và Robot Base.
- Nhận /cmd_vel_muxed từ twist_mux.
- Lắng nghe /scan (Lidar ở độ cao 20cm).
- Nếu phát hiện vật cản ở hướng di chuyển (đặc biệt khi lùi để né), dừng khẩn cấp.
- Output ra /cmd_vel để vi điều khiển thực thi.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class SafetyShieldNode(Node):
    def __init__(self):
        super().__init__('context_safety_shield')
        
        # Đọc tham số an toàn (bất đối xứng do vị trí đặt Lidar)
        self.declare_parameter('stop_dist_front', 0.50)
        self.declare_parameter('stop_dist_rear', 0.80)
        self.declare_parameter('stop_dist_side', 0.35)
        
        self.stop_front = self.get_parameter('stop_dist_front').value
        self.stop_rear = self.get_parameter('stop_dist_rear').value
        self.stop_side = self.get_parameter('stop_dist_side').value
        
        # Sub/Pub
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel_muxed', self.cmd_cb, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_cb, rclpy.qos.qos_profile_sensor_data)
        
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Trạng thái Lidar (khoảng cách vật cản gần nhất theo hướng)
        self.min_dist_front = float('inf')
        self.min_dist_rear = float('inf')
        self.min_dist_left = float('inf')
        self.min_dist_right = float('inf')

        self.get_logger().info("🛡️ Context Safety Shield Node Started! Protecting /cmd_vel...")

    def scan_cb(self, msg: LaserScan):
        # Giải thích cho user:
        # 1. Khi đi thẳng (X), robot chỉ quan tâm những vật cản nằm CÙNG LÀN ĐƯỜNG với mình.
        #    Vì vậy ta phải dùng kích thước CHIỀU NGANG (Y) để lọc các vật cản không nằm trên đường đi (nằm ngoài lề).
        # 2. Ngược lại, khi trượt ngang (Y), robot chỉ quan tâm những vật nằm trong phạm vi CHIỀU DỌC (X) của thân xe.
        y_limit_for_x = 0.35  # Bề rộng xét va chạm khi đi thẳng (X) (chiều ngang)
        x_limit_for_y = 0.50  # Bề dài xét va chạm khi trượt ngang (Y) (chiều dọc)

        min_x_front = float('inf')
        min_x_rear = float('inf')
        min_y_left = float('inf')
        min_y_right = float('inf')
        
        angle = msg.angle_min
        for r in msg.ranges:
            if not math.isinf(r) and not math.isnan(r) and r > msg.range_min:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                
                # 1. Trục X (Tiến/Lùi) - chỉ xét vật cản NẰM TRONG làn đường di chuyển của xe
                if abs(y) <= y_limit_for_x:
                    if x > 0:
                        min_x_front = min(min_x_front, x)
                    elif x < 0:
                        min_x_rear = min(min_x_rear, abs(x))
                        
                # 2. Trục Y (Trượt ngang) - chỉ xét vật cản NẰM TRONG làn trượt của xe
                if abs(x) <= x_limit_for_y:
                    if y > 0:
                        min_y_left = min(min_y_left, y)
                    elif y < 0:
                        min_y_right = min(min_y_right, abs(y))
                        
            angle += msg.angle_increment
            
        self.min_dist_front = min_x_front
        self.min_dist_rear = min_x_rear
        self.min_dist_left = min_y_left
        self.min_dist_right = min_y_right

    def cmd_cb(self, msg: Twist):
        safe_msg = Twist()
        safe_msg.linear.x = msg.linear.x
        safe_msg.linear.y = msg.linear.y
        safe_msg.angular.z = msg.angular.z
        
        emergency_stopped = False

        # Kiểm tra đâm phía trước
        if msg.linear.x > 0 and self.min_dist_front < self.stop_front:
            safe_msg.linear.x = 0.0
            emergency_stopped = True
            
        # Kiểm tra đâm phía sau (Lidar ở đầu xe nên phía sau phải cách xa 0.8m)
        if msg.linear.x < 0 and self.min_dist_rear < self.stop_rear:
            safe_msg.linear.x = 0.0
            emergency_stopped = True

        # Kiểm tra đâm 2 bên (Trái/Phải)
        if msg.linear.y > 0 and self.min_dist_left < self.stop_side:
            safe_msg.linear.y = 0.0
            emergency_stopped = True
        if msg.linear.y < 0 and self.min_dist_right < self.stop_side:
            safe_msg.linear.y = 0.0
            emergency_stopped = True

        vx_blocked = (safe_msg.linear.x == 0.0 and msg.linear.x != 0.0)
        vy_blocked = (safe_msg.linear.y == 0.0 and msg.linear.y != 0.0)

        if emergency_stopped:
            self.get_logger().warn(
                f"🛑 BLOCKED: "
                f"{'vx' if vx_blocked else ''} {'vy' if vy_blocked else ''} | "
                f"F:{self.min_dist_front:.2f}m R:{self.min_dist_rear:.2f}m "
                f"L:{self.min_dist_left:.2f}m Ri:{self.min_dist_right:.2f}m"
            )
            # Chỉ dừng angular.z khi CẢ vx VÀ vy đều bị chặn (robot hoàn toàn kẹt)
            # Nếu chỉ vy bị chặn (vật bên hông) mà vx vẫn OK → vẫn cho quay/tiến
            if vx_blocked and vy_blocked:
                safe_msg.angular.z = 0.0

        self.pub_cmd.publish(safe_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyShieldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
