#!/usr/bin/env python3
"""ROS integration regression: stale/missing front/pose must stop propulsion.

Uses a separate ROS domain; does not start Gazebo or command a real vehicle.
"""
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

os.environ['ROS_DOMAIN_ID'] = '88'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'reports/corridor_review/input_regression'
OUT.mkdir(parents=True, exist_ok=True)
os.environ['ROS_LOG_DIR'] = str(OUT/'ros_logs')
import rclpy
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float64


def main():
    rclpy.init()
    node = rclpy.create_node('controller_inputs_regression')
    base = '/model/input_test'
    front = node.create_publisher(PointCloud2, base+'/frontal_sonar/points', 10)
    side = node.create_publisher(PointCloud2, base+'/left_sonar/points', 10)
    pose = node.create_publisher(TFMessage, '/world/buoyant_tethys/dynamic_pose/info', 10)
    commands = []
    fins = {'rudder': [], 'elevator': []}
    node.create_subscription(Float64, base+'/joint/propeller_joint/cmd_thrust',
                             lambda msg: commands.append((time.monotonic(), msg.data)), 10)
    for name, topic in [('rudder', 'vertical_fins/cmd_pos'), ('elevator', 'horizontal_fins/cmd_pos')]:
        node.create_subscription(Float64, base+'/'+topic,
                                 lambda msg, key=name: fins[key].append((time.monotonic(), msg.data)), 10)
    tf = TransformStamped(child_frame_id='input_test')
    # Nonzero depth error makes stale-feedback fin commands observable.
    tf.transform.translation.z = 10.0
    tf.transform.rotation.w = 1.0
    cloud = PointCloud2(height=1, width=1, point_step=12, row_step=12, is_dense=True,
                        fields=[PointField(name=n, offset=i*4, datatype=7, count=1)
                                for i, n in enumerate(['x', 'y', 'z'])],
                        data=struct.pack('fff', 100., 0., 0.))
    results = {}
    with (OUT/'controller.log').open('w') as log:
        process = subprocess.Popen([str(ROOT/'install/collision_avoidance/lib/collision_avoidance/collision_avoidance_node'),
                                    '--ros-args', '-p', 'drone_name:=input_test',
                                    '-p', 'target_x:=0.0', '-p', 'target_y:=100.0', '-p', 'target_z:=8.0',
                                    '-p', f'csv_path:={OUT}/metrics.csv'],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        def phase(name, seconds, send_front, send_pose, expected):
            start = time.monotonic()
            while time.monotonic()-start < seconds:
                side.publish(cloud)
                if send_front:
                    front.publish(cloud)
                if send_pose:
                    pose.publish(TFMessage(transforms=[tf]))
                rclpy.spin_once(node, timeout_sec=.05)
                time.sleep(.03)
            tail = [v for t, v in commands if t > time.monotonic()-.7]
            assert tail, f'{name}: no command received'
            assert expected(tail), f'{name}: unexpected thrust {tail}'
            fin_tail = {key: [v for t, v in values if t > time.monotonic()-.7]
                        for key, values in fins.items()}
            if name in {'side_without_front', 'front_stale_side_fresh', 'pose_stale_front_fresh'}:
                assert all(values and all(v == 0 for v in values) for values in fin_tail.values()), (name, fin_tail)
            if name == 'fresh_front_and_pose':
                assert any(abs(v) > .01 for v in fin_tail['elevator']), fin_tail
            results[name] = {'passed': True, 'thrust_tail': tail, **fin_tail}
        try:
            phase('side_without_front', 2.0, False, True, lambda a: all(v == 0 for v in a))
            phase('fresh_front_and_pose', 4.0, True, True, lambda a: max(a) > 0)
            phase('front_stale_side_fresh', 2.5, False, True, lambda a: all(v == 0 for v in a))
            phase('pose_stale_front_fresh', 2.5, True, False, lambda a: all(v == 0 for v in a))
            cloud.data = struct.pack('fff', 2., 0., 0.)
            phase('close_obstacle_no_positive_thrust', 2.5, True, True, lambda a: all(v <= 0 for v in a))
            cloud.data = struct.pack('fff', 100., 0., 0.)
            phase('blind_front_retains_close_obstacle', 1.0, True, True,
                  lambda a: all(v <= 0 for v in a))
        finally:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            node.destroy_node()
            rclpy.shutdown()
    (OUT/'results.json').write_text(json.dumps(results, indent=2))
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
