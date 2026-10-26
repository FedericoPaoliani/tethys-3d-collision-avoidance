#!/usr/bin/env python3
"""Replay low-depth forward motion and a close obstacle, without Gazebo."""
import argparse
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'reports/corridor_review/depth_brake_regression'
OUT.mkdir(parents=True, exist_ok=True)
os.environ['ROS_DOMAIN_ID'] = '88'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_LOG_DIR'] = str(OUT/'ros_logs')
import rclpy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--binary', type=Path, default=ROOT/'install/collision_avoidance/lib/collision_avoidance/collision_avoidance_node')
    parser.add_argument('--label', default='current')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('depth_brake_regression')
    front = node.create_publisher(PointCloud2, '/model/depth_brake/frontal_sonar/points', 10)
    poses = node.create_publisher(TFMessage, '/world/buoyant_tethys/dynamic_pose/info', 10)
    commands = []
    node.create_subscription(Float64, '/model/depth_brake/joint/propeller_joint/cmd_thrust',
                             lambda msg: commands.append((time.monotonic(), msg.data)), 10)
    tf = TransformStamped(child_frame_id='depth_brake')
    tf.transform.translation.z = 4.0
    tf.transform.rotation.w = 1.0
    cloud = PointCloud2(height=1, width=1, point_step=12, row_step=12, is_dense=True,
        fields=[PointField(name=k, offset=4*i, datatype=7, count=1) for i,k in enumerate(['x','y','z'])])
    log_path = OUT/(args.label+'.log')
    with log_path.open('w') as log:
        proc = subprocess.Popen([str(args.binary.resolve()), '--ros-args',
            '-p', 'drone_name:=depth_brake', '-p', 'yaw_bias:=0.0',
            '-p', 'spawn_z:=4.0', '-p', 'target_x:=100.0', '-p', 'target_y:=0.0',
            '-p', 'target_z:=8.0', '-p', 'allow_vertical_escape:=false',
            '-p', 'depth_limit_high:=17.0', '-p', f'csv_path:={OUT}/{args.label}.csv'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            start = time.monotonic()
            while time.monotonic()-start < 8.0:
                elapsed = time.monotonic()-start
                tf.transform.translation.x = max(0.0, elapsed-3.0)
                distance = 100.0 if elapsed < 4.5 else (7.0 if elapsed < 6.0 else 2.0)
                cloud.data = struct.pack('fff', distance, 0.0, 0.0)
                poses.publish(TFMessage(transforms=[tf]))
                front.publish(cloud)
                rclpy.spin_once(node, timeout_sec=.03)
                time.sleep(.02)
            assert '[DEPTH LIMIT]' in log_path.read_text(), 'Depth recovery was not exercised'
            tail = [v for t,v in commands if t > time.monotonic()-.7]
            assert tail and all(v < 0 for v in tail), f'Depth recovery overrode front braking: {tail}; {log_path}'
            print('PASS front braking overrides depth recovery while moving forward')
        finally:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
