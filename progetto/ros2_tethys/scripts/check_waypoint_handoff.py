#!/usr/bin/env python3
"""Exercise waypoint completion with synthetic ROS poses, without Gazebo."""
import argparse
import math
import os
from pathlib import Path
import re
import signal
import struct
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'reports/corridor_review/waypoint_regression'
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
    parser.add_argument('--binary', type=Path, default=ROOT/'install/collision_avoidance/lib/collision_avoidance/collision_avoidance_node')
    parser.add_argument('--label', default='current')
    parser.add_argument('--close-side', action='store_true',
                        help='Check that a lateral return does not freeze waypoint tracking')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('waypoint_handoff_regression')
    front = node.create_publisher(PointCloud2, '/model/waypoint_test/frontal_sonar/points', 10)
    poses = node.create_publisher(TFMessage, '/world/buoyant_tethys/dynamic_pose/info', 10)
    tf = TransformStamped(child_frame_id='waypoint_test')
    tf.transform.translation.y = -50.0
    tf.transform.translation.z = 8.0
    tf.transform.rotation.z = math.sin(-math.pi/4)
    tf.transform.rotation.w = math.cos(-math.pi/4)
    cloud = PointCloud2(height=1, width=1, point_step=12, row_step=12, is_dense=True,
                        fields=[PointField(name=k, offset=4*i, datatype=7, count=1)
                                for i, k in enumerate(['x', 'y', 'z'])],
                        data=struct.pack('fff', 20.0, 0.0, 0.0))
    log_path = OUT/(args.label+'.log')
    def publish_for(seconds):
        start = time.monotonic()
        while time.monotonic()-start < seconds:
            poses.publish(TFMessage(transforms=[tf]))
            front.publish(cloud)
            rclpy.spin_once(node, timeout_sec=.03)
            time.sleep(.02)
    with log_path.open('w') as log:
        proc = subprocess.Popen([str(args.binary.resolve()), '--ros-args',
            '-p', 'drone_name:=waypoint_test', '-p', f'yaw_bias:={math.pi}',
            '-p', 'spawn_y:=-50.0', '-p', 'spawn_z:=8.0',
            '-p', 'target_x:=0.0', '-p', 'target_y:=50.0', '-p', 'target_z:=8.0',
            '-p', 'wall_wx:=15.0', '-p', 'wall_wy:=60.0',
            '-p', 'allow_vertical_escape:=false', '-p', 'depth_limit_high:=17.0',
            '-p', f'csv_path:={OUT}/{args.label}.csv'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            publish_for(3.5)
            match = re.search(r'\[LOCAL-PATH\].*wp=\(([-\d.]+),([-\d.]+)\)', log_path.read_text())
            assert match, log_path.read_text()
            wx, wy = map(float, match.groups())
            length = math.hypot(wx, wy+50.0)
            if args.close_side:
                # The observed point is outside the forward swept corridor.
                # Move right of the leg, so the fixed waypoint bearing changes.
                cloud.data = struct.pack('fff', 5.0, -2.5, 0.0)
                for step in range(1, 31):
                    tf.transform.translation.x = step/30 * 1.5
                    tf.transform.translation.y = -50.0 + step/30 * 4.0
                    publish_for(.1)
                publish_for(1.2)
                expected = math.atan2(wy-tf.transform.translation.y,
                                      wx-tf.transform.translation.x)
                matches = re.findall(r'mode=ESCAPE.*tgt_yaw=([-\d.]+)', log_path.read_text())
                assert matches, log_path.read_text()
                actual = float(matches[-1])
                assert abs(actual-expected) < .035, (
                    f'Waypoint bearing froze: expected {expected:.3f}, commanded {actual:.3f}; {log_path}')
                print('PASS waypoint tracking follows the fixed point with a close lateral return')
                return
            for step in range(1, 51):
                fraction = step/50 * (length-2.0)/length
                tf.transform.translation.x = fraction*wx
                tf.transform.translation.y = -50.0 + fraction*(wy+50.0)
                publish_for(.1)
            publish_for(.5)
            text = log_path.read_text()
            assert '[WP-REACHED]' in text, text
            assert '[WP-NEXT]' in text, 'Completed waypoint has no immediately planned successor; see '+str(log_path)
            print('PASS waypoint completion immediately plans the next observed segment')
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
