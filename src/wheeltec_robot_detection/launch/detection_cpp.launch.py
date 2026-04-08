"""Launch file for C++ TensorRT detection node."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('wheeltec_robot_detection')

    # Model path (update this to your actual engine path)
    engine_path = os.path.join(
        os.path.expanduser('~'),
        'wheeltec_ros2/src/wheeltec_robot_detection/models/yolov8m_int8.engine'
    )

    # COCO class names
    class_names = [
        "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
        "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
        "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
        "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
        "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
        "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
        "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
        "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
        "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
        "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
    ]

    return LaunchDescription([
        Node(
            package='wheeltec_robot_detection',
            executable='detection_node_cpp',
            name='detection_node_cpp',
            parameters=[{
                'engine_path': engine_path,
                'input_h': 640,
                'input_w': 640,
                'conf_threshold': 0.5,
                'nms_threshold': 0.45,
                'class_names': class_names
            }],
            output='screen'
        ),
    ])
