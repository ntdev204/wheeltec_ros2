
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult
from rclpy.duration import Duration

from std_msgs.msg import Bool 
from std_msgs.msg import Int8 
from std_msgs.msg import UInt8
from std_msgs.msg import Float32
from turtlesim.srv import Spawn

from geometry_msgs.msg import PoseStamped

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry


import sys, select, termios, tty

import time

import json
import yaml

import math
import os

json_file='/home/wheeltec/wheeltec_ros2/src/auto_recharge_ros2/Charger_Position.json'
yaml_file='/home/wheeltec/wheeltec_ros2/src/auto_recharge_ros2/robot_info.yaml'
RESET = '\033[0m'
RED   = '\033[1;31m'
GREEN = '\033[1;32m'
YELLOW= '\033[1;33m'
BLUE  = '\033[1;34m'
PURPLE= '\033[1;35m'
CYAN  = '\033[1;36m'

PI=3.1415926535897

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty
	
settings = None
if os.name != 'nt' and sys.stdin.isatty():
	settings = list(termios.tcgetattr(sys.stdin))

def get_key(settings):
	if os.name == 'nt':
		return msvcrt.getch().decode('utf-8')
	if sys.stdin.isatty():
		tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''
	if sys.stdin.isatty():
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def print_and_fixRetract(str):
	global settings
	if sys.stdin.isatty():
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	print(str)

class AutoRecharger(Node):
	def __init__(self):
		
		super().__init__("auto_recharger")

		print_and_fixRetract('Automatic charging node start!')

		self.robot = {
        'Type':'Plus', 
        'BatteryCapacity':5000, 
        'Voltage':25, 
        'Charging':0, 
        'Charging_current':0, 
        'RED':0, 
        'Rotation_Z':0,
        'car_mode':'mini_mec'
        }

		self.nav_end_z=0
		self.start_turn = 0
		self.find_redsignal = 0

		self.red_count=0
		self.chargeflag=0
		self.last_time= self.get_clock().now()
		self.lost_red_flag=self.get_clock().now()
		self.power_lost_count=0
		self.lost_power_once=1
		self.charge_complete=0
		self.last_charge_complete=0
		self.json_data=0
		self.star_getNav_Feedback_Flag=0

		self.diff_point = 1.2

		self.diff_angle = -15

		self.nav_controller =  BasicNavigator()

		with open(json_file,'r')as fp:
			self.json_data = json.load(fp)

		self.robot_security_off_pub = self.create_publisher(Int8,'/chassis_security',   10) 

		self.Charger_marker_pub   = self.create_publisher(MarkerArray,'/goal_marker',   10) 

		self.Recharger_Flag_pub = self.create_publisher(Int8,"robot_recharge_flag",  5)

		self.Cmd_vel_pub = self.create_publisher(Twist,"/cmd_vel",  5)

		self.Voltage_sub = self.create_subscription(Float32, "PowerVoltage", self.Voltage_callback,10)

		self.Charging_Flag_sub = self.create_subscription(Bool, "robot_charging_flag",self.Charging_Flag_callback,10)

		self.Charging_Current_sub = self.create_subscription(Float32,"robot_charging_current",  self.Charging_Current_callback,10)

		self.RED_Flag_sub = self.create_subscription(UInt8,"robot_red_flag",  self.RED_Flag_callback,10)

		self.Charger_Position_Update_sub = self.create_subscription( PoseStamped,"/charger_position_update", self.Position_Update_callback,10)

		self.Odom_sub = self.create_subscription(Odometry, '/odom', self.Odom_callback,10)

		self.set_charge = self.create_client(Spawn,'/set_charge')

		self.server_set_state = None
		self.wait_server_done = None
		self.tips = """
使用下面按键使用自动回充功能.       Press below Key to AutoRecharger.
Q/q:开启自动回充.                   Q/q:Start Navigation to find charger.
E/e:停止自动回充.                   E/e:Stop find charger.
Ctrl+C/c:关闭自动回充功能并退出.    Ctrl+C/c:Quit the program.
可使用话题"charger_position_update"更新充电桩的位置.