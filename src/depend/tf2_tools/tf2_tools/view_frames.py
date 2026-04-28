

import argparse
import subprocess
import sys
import time

import yaml

import rclpy
from tf2_msgs.srv import FrameGraph
import tf2_py as tf2
import tf2_ros


def main():
    rclpy.init(args=sys.argv)

    args_without_ros = rclpy.utilities.remove_ros_args(sys.argv)
    parser = argparse.ArgumentParser(
        description='Create a diagram of the TF frames being broadcast over ROS')
    parser.add_argument(
        '--wait-time', '-t', type=float, default=5.0,
        help='Listen to the /tf topic for this many seconds before rendering the frame tree')
    parsed_args = parser.parse_args(args=args_without_ros[1:])

    node = rclpy.create_node('view_frames')

    buf = tf2_ros.Buffer(node=node)
    listener = tf2_ros.TransformListener(buf, node, spin_thread=False)

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    node.get_logger().info(f'Listening to tf data for {parsed_args.wait_time} seconds...')
    start_time = time.time()
    while (time.time() - start_time) < parsed_args.wait_time:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info('Generating graph in frames.pdf file...')

    cli = node.create_client(FrameGraph, 'tf2_frames')
    req = FrameGraph.Request()
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    ret = 1
    try:
        result = future.result()
        ret = 0
    except Exception as e:
        node.get_logger().error('Service call failed %r' % (e,))
    else:
        node.get_logger().info(
            'Result:'+ str(result) )
        data = yaml.safe_load(result.frame_yaml)
        with open('frames.gv', 'w') as f:
           f.write(generate_dot(data, node.get_clock().now().seconds_nanoseconds()))
        subprocess.Popen('dot -Tpdf frames.gv -o frames.pdf'.split(' ')).communicate()
    finally:
        cli.destroy()
        node.destroy_node()
        rclpy.shutdown()
        return ret

def generate_dot(data, recorded_time):
    if len(data) == 0:
        return 'digraph G { "No tf data received" }'

    dot = 'digraph G {\n'
    for el in data:
        map = data[el]
        dot += '"'+map['parent']+'" -> "'+str(el)+'"'
        dot += '[label=" '
        dot += 'Broadcaster: '+map['broadcaster']+'\\n'
        dot += 'Average rate: '+str(map['rate'])+'\\n'
        dot += 'Buffer length: '+str(map['buffer_length'])+'\\n'
        dot += 'Most recent transform: '+str(map['most_recent_transform'])+'\\n'
        dot += 'Oldest transform: '+str(map['oldest_transform'])+'\\n'
        dot += '"];\n'
        if not map['parent'] in data:
            root = map['parent']
    dot += 'edge [style=invis];\n'
    dot += ' subgraph cluster_legend { style=bold; color=black; label ="view_frames Result";\n'
    dot += '"Recorded at time: '+str(recorded_time[0]+recorded_time[1]/1e9)+'"[ shape=plaintext ] ;\n'
    dot += '}->"'+root+'";\n}'
    return dot


if __name__ == '__main__':
    sys.exit(main())
