#!/usr/bin/env python3
"""ROS regression: final approach requires a clear finite path to the goal.

Synthetic inputs on an isolated domain; no Gazebo or physical vehicle commands.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'reports/corridor_review/final_approach_regression'
OUT.mkdir(parents=True, exist_ok=True)
os.environ['ROS_DOMAIN_ID'] = '88'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_LOG_DIR'] = str(OUT/'ros_logs')

import rclpy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--case', choices=['blocked', 'clear', 'beyond_goal'])
    args = parser.parse_args()
    cases = {'blocked': (3.0, 2.0), 'clear': (100.0, 0.0), 'beyond_goal': (19.0, 10.0)}
    rclpy.init()
    node = rclpy.create_node('final_approach_regression')
    results = {}
    try:
        for name, (xs, ys) in cases.items():
            if args.case and args.case != name:
                continue
            drone = 'final_test_'+name
            front = node.create_publisher(PointCloud2, f'/model/{drone}/frontal_sonar/points', 10)
            pose = node.create_publisher(TFMessage, '/world/buoyant_tethys/dynamic_pose/info', 10)
            tf = TransformStamped(child_frame_id=drone)
            tf.transform.translation.z = 8.0
            tf.transform.rotation.w = 1.0
            cloud = PointCloud2(height=1, width=1, point_step=12, row_step=12,
                                is_dense=True,
                                fields=[PointField(name=k, offset=4*i, datatype=7, count=1)
                                        for i, k in enumerate(['x', 'y', 'z'])],
                                data=struct.pack('fff', xs, ys, 0.0))
            log_path = OUT/(name+'.log')
            with log_path.open('w') as log:
                proc = subprocess.Popen([
                    str(ROOT/'install/collision_avoidance/lib/collision_avoidance/collision_avoidance_node'),
                    '--ros-args', '-p', f'drone_name:={drone}', '-p', 'yaw_bias:=0.0',
                    '-p', 'target_x:=10.0', '-p', 'target_y:=5.0', '-p', 'target_z:=8.0',
                    '-p', 'allow_vertical_escape:=false',
                    '-p', f'csv_path:={OUT}/{name}.csv'],
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    start = time.monotonic()
                    while time.monotonic()-start < 4.0:
                        pose.publish(TFMessage(transforms=[tf]))
                        front.publish(cloud)
                        rclpy.spin_once(node, timeout_sec=.05)
                        time.sleep(.03)
                finally:
                    os.killpg(proc.pid, signal.SIGINT)
                    try:
                        proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid, signal.SIGKILL)
                        proc.wait()
            text = log_path.read_text()
            approach = '[FINAL-APPROACH-START]' in text
            assert 'nessun sonar' not in text, text
            assert '[HARD STUCK]' not in text, text
            assert approach == (name != 'blocked'), f'{name}: final approach={approach}; see {log_path}'
            if name == 'blocked':
                assert '[AVOID-MINIMAL]' in text or '[SIDE-AVOID]' in text, text
            results[name] = {'passed': True, 'final_approach': approach}
            node.destroy_publisher(front)
            node.destroy_publisher(pose)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    (OUT/'results.json').write_text(json.dumps(results, indent=2))
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
