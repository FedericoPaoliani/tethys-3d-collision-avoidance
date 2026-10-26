#!/usr/bin/env python3
"""Run one isolated corridor mission; preserve executable, logs and ROS telemetry.

Source ROS and the workspace before running. Run sequentially: GPU lidar is shared.
Physical contact events and the controller's sonar proximity count stay separate.
"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import struct
import subprocess
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from ros_gz_interfaces.msg import Contacts

ROOT = Path(__file__).resolve().parents[1]


class Recorder(Node):
    def __init__(self, output):
        super().__init__('corridor_review_recorder')
        self.clock = 0.0
        self.front_seen = False
        self.sonar_samples = []
        self.last_sonar_t = -1.0
        self.initial_front_saved = False
        self.pose_seen = False
        self.last_t = -1.0
        self.commands = [0.0, 0.0, 0.0]
        self.actual_fins = [0.0, 0.0]
        self.rows = []
        self.output = output
        self.frames_saved = False
        self.contact_events = []
        self.last_contact = -100.0
        self.file = (output / 'telemetry.csv').open('w')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['sim_t', 'wall_t', 'x', 'y', 'z', 'roll', 'pitch',
                              'yaw', 'thrust', 'rudder', 'elevator', 'goal_dist',
                              'actual_rudder', 'actual_elevator'])
        self.create_subscription(Clock, '/clock', self.on_clock, 10)
        self.create_subscription(TFMessage, '/world/buoyant_tethys/dynamic_pose/info', self.on_pose, 10)
        self.create_subscription(PointCloud2, '/model/tethys/frontal_sonar/points',
                                 self.on_front, qos_profile_sensor_data)
        self.contact_messages = 0
        for name in ['base_link', 'horizontal_fins', 'vertical_fins', 'propeller',
                     'buoyancy_engine', 'acoustic_transponder']:
            self.create_subscription(Contacts, '/model/tethys/review_contacts/' + name,
                                     self.on_contacts, 10)
        for i, topic in enumerate(['joint/propeller_joint/cmd_thrust',
                                   'vertical_fins/cmd_pos', 'horizontal_fins/cmd_pos']):
            self.create_subscription(Float64, '/model/tethys/' + topic,
                                     lambda msg, idx=i: self.commands.__setitem__(idx, msg.data), 10)

    def on_clock(self, msg):
        self.clock = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_front(self, msg):
        self.front_seen = True
        if self.clock - self.last_sonar_t < .19:
            return
        offsets = {f.name: f.offset for f in msg.fields if f.datatype == 7}
        if not all(k in offsets for k in ('x', 'y', 'z')):
            return
        closest = 99.0
        points = []
        endian = '>' if msg.is_bigendian else '<'
        for row in range(msg.height):
            for col in range(msg.width):
                start = row*msg.row_step + col*msg.point_step
                x, y, z = [struct.unpack_from(endian+'f', msg.data, start+offsets[k])[0]
                           for k in ('x', 'y', 'z')]
                if not all(math.isfinite(v) for v in (x, y, z)) or x <= 0:
                    continue
                points.append([x, y, z])
                if abs(math.atan2(y, x)) < .06 and abs(math.atan2(z, math.hypot(x,y))) < .06:
                    closest = min(closest, math.sqrt(x*x+y*y+z*z))
        self.sonar_samples.append({'sim_t': self.clock, 'center_range': closest})
        if not self.initial_front_saved and self.rows and self.clock >= 1.0:
            (self.output/'initial_front_points.json').write_text(json.dumps({
                'sim_t': self.clock, 'width': msg.width, 'height': msg.height,
                'pose_sample': self.rows[-1], 'points_sensor_frame': points}))
            self.initial_front_saved = True
        self.last_sonar_t = self.clock

    def on_contacts(self, msg):
        self.contact_messages += 1
        if not msg.contacts:
            return
        if self.clock - self.last_contact > 1.0:
            self.contact_events.append({'sim_t': self.clock, 'wall_t': time.time(),
                                        'contacts': str(msg.contacts)})
        self.last_contact = self.clock

    def on_pose(self, msg):
        for tf in msg.transforms:
            q = tf.transform.rotation
            if tf.child_frame_id == 'horizontal_fins':
                self.actual_fins[1] = math.asin(max(-1.0, min(1.0, 2*(q.w*q.y-q.z*q.x))))
            elif tf.child_frame_id == 'vertical_fins':
                self.actual_fins[0] = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        if not self.frames_saved and self.clock > 5:
            (self.output/'frames.txt').write_text(str(msg))
            self.frames_saved = True
        for tf in msg.transforms:
            if tf.child_frame_id != 'tethys':
                continue
            self.pose_seen = True
            t = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            if t == 0.0:
                t = self.clock
            if t - self.last_t < 0.19:
                continue
            self.last_t = t
            p, q = tf.transform.translation, tf.transform.rotation
            roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
            pitch = math.asin(max(-1.0, min(1.0, 2*(q.w*q.y-q.z*q.x))))
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            dist = math.sqrt(p.x*p.x + (p.y-50)**2 + (p.z-8)**2)
            row = [t, time.time(), p.x, p.y, p.z, roll, pitch, yaw, *self.commands, dist]
            self.rows.append(row)
            self.writer.writerow(row + self.actual_fins)
            self.file.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', required=True, type=int)
    ap.add_argument('--duration', type=float, default=180.0)
    ap.add_argument('--label', default='baseline')
    ap.add_argument('--world', type=Path, default=ROOT/'worlds/mondoCorridoio.sdf',
                    help='Corridor world with the same spawn/goal and tank dimensions')
    ap.add_argument('--elevator', type=float, default=None,
                    help='Open-loop sign diagnostic: publish constant elevator and 35 N thrust')
    ap.add_argument('--diagnostic-thrust', type=float, default=35.0)
    ap.add_argument('--stationary', action='store_true',
                    help='Sensor diagnostic: neutral actuators, preserve all scene models')
    ap.add_argument('--wall-limit', type=float, default=600.0,
                    help='Tetto di tempo REALE per la missione [s] (default 600)')
    ap.add_argument('--stall-abort-s', type=float, default=0.0,
                    help='Scarta la missione dopo N s simulati senza avvicinarsi al goal '
                         'di almeno --stall-margin-m (0 = disattivato)')
    ap.add_argument('--stall-margin-m', type=float, default=1.0,
                    help='Avvicinamento minimo che azzera il conteggio di stallo [m]')
    ap.add_argument('--out-root', type=Path, default=ROOT / 'reports/corridor_review',
                    help='Cartella dei run (default reports/corridor_review, quella storica); '
                         'i livelli 2 e 3 usano una sottocartella per livello')
    ap.add_argument('--controller-snapshot', type=Path,
                    help='Replay controller and exact sources from a previous run directory')
    args = ap.parse_args()
    output = args.out_root / f'run_{args.run:02d}'
    output.mkdir(parents=True, exist_ok=False)
    # Snapshots contain package.xml for reproducibility, not extra workspace packages.
    (output.parent/'COLCON_IGNORE').touch()
    os.environ['IGN_PARTITION'] = f'codex_corridor_{os.getpid()}'
    # ign-transport sul loopback: una caduta della WiFi non deve interrompere il bridge
    # (vedi review_multi.py, run 229-231 di Multi3). Nessun effetto sulla simulazione.
    os.environ['IGN_IP'] = '127.0.0.1'
    os.environ['ROS_DOMAIN_ID'] = '87'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_LOG_DIR'] = str(output / 'ros_logs')
    processes, handles = [], []

    def launch(argv, name):
        h = (output / (name + '.log')).open('w')
        handles.append(h)
        proc = subprocess.Popen(argv, cwd=ROOT, stdout=h, stderr=subprocess.STDOUT,
                                start_new_session=True)
        processes.append(proc)
        return proc

    rclpy.init()
    recorder = Recorder(output)
    reason, mission_start = 'startup_failed', None
    try:
        # Observation only: attach contact sensors to every collision-bearing
        # vehicle link in a private world copy, leaving all physical parameters intact.
        tree = ET.parse(args.world)
        world = tree.getroot().find('world')
        obstacles_model = world.find("model[@name='ostacoli']")
        obstacle_count = len(obstacles_model.findall('link')) if obstacles_model is not None else 0
        if args.elevator is not None and not args.stationary:
            # Isolate the actuator response: a collision can change pitch and
            # falsely look like a reversal of the elevator's steady response.
            for model in list(world.findall('model')):
                if model.get('name') != 'tethys':
                    world.remove(model)
        if not any(p.get('name') == 'ignition::gazebo::systems::Contact'
                   for p in world.findall('plugin')):
            ET.SubElement(world, 'plugin', filename='ignition-gazebo-contact-system',
                          name='ignition::gazebo::systems::Contact')
        contact_links = []
        for link in world.find("model[@name='tethys']").findall('link'):
            collisions = link.findall('collision')
            if not collisions:
                continue
            contact_links.append(link.get('name'))
            sensor = ET.SubElement(link, 'sensor', name='review_contacts', type='contact')
            ET.SubElement(sensor, 'always_on').text = 'true'
            ET.SubElement(sensor, 'update_rate').text = '20'
            ET.SubElement(sensor, 'topic').text = '/model/tethys/review_contacts'
            contact = ET.SubElement(sensor, 'contact')
            for collision in collisions:
                ET.SubElement(contact, 'collision').text = collision.get('name')
        tree.write(output/'instrumented_world.sdf')
        config = (ROOT/'scripts/ponte_single_config.yaml').read_text()
        for link in contact_links:
            config += (f'\n- ros_topic_name: "/model/tethys/review_contacts/{link}"\n'
                       f'  gz_topic_name: "/world/buoyant_tethys/model/tethys/link/{link}/sensor/review_contacts/contact"\n'
                       '  ros_type_name: "ros_gz_interfaces/msg/Contacts"\n'
                       '  gz_type_name: "ignition.msgs.Contacts"\n  direction: GZ_TO_ROS\n')
        (output/'bridge.yaml').write_text(config)
        launch(['/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge', '--ros-args',
                '-p', f'config_file:={output}/bridge.yaml'], 'bridge')
        startup = output / 'start_gazebo.sh'
        startup.write_text((ROOT/'scripts/start_sim_corridoio_headless.sh').read_text()
                           .replace('-v 3', '-v 4')
                           .replace('~/ros2_tethys/worlds/mondoCorridoio.sdf', str(output/'instrumented_world.sdf')))
        sim = launch(['bash', str(startup)], 'gazebo')
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline and sim.poll() is None:
            rclpy.spin_once(recorder, timeout_sec=0.1)
            if recorder.front_seen and recorder.pose_seen and recorder.clock > 0:
                break
        else:
            raise RuntimeError('No clock, pose and frontal sonar within 60 wall seconds')
        snapshot = args.controller_snapshot
        executable = snapshot/'controller' if snapshot else ROOT / 'install/collision_avoidance/lib/collision_avoidance/collision_avoidance_node'
        shutil.copy2(executable, output / 'controller')
        source_dir = snapshot if snapshot else ROOT/'src/collision_avoidance/src'
        package_dir = snapshot if snapshot else ROOT/'src/collision_avoidance'
        source = source_dir/'collision_avoidance_node.cpp'
        shutil.copy2(source, output / source.name)
        shutil.copy2(args.world, output/'world.sdf')
        shutil.copy2(source_dir/'vfh_planner.cpp', output/'vfh_planner.cpp')
        shutil.copytree(package_dir/'include', output/'include')
        shutil.copy2(package_dir/'CMakeLists.txt', output/'CMakeLists.txt')
        shutil.copy2(package_dir/'package.xml', output/'package.xml')
        metadata = {'label': args.label, 'duration_limit_sim_s': args.duration,
                    'stall_abort_s': args.stall_abort_s,
                    'controller_snapshot': str(snapshot.resolve()) if snapshot else None,
                    'obstacle_count': obstacle_count,
                    'open_loop_elevator_rad': args.elevator,
                    'stationary_sensor_diagnostic': args.stationary,
                    'open_loop_thrust_n': args.diagnostic_thrust if args.elevator is not None else None,
                    'diagnostic_obstacles_removed': args.elevator is not None and not args.stationary,
                    'diagnostic_walls_removed': args.elevator is not None and not args.stationary,
                    'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
                    'planner_sha256': hashlib.sha256((output/'vfh_planner.cpp').read_bytes()).hexdigest(),
                    'binary_sha256': hashlib.sha256((output/'controller').read_bytes()).hexdigest(),
                    'world_source': str(args.world.resolve()),
                    'world_sha256': hashlib.sha256((output/'world.sdf').read_bytes()).hexdigest()}
        metadata['headers_sha256'] = {
            str(p.relative_to(output/'include')): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted((output/'include').rglob('*.hpp'))}
        (output / 'metadata.json').write_text(json.dumps(metadata, indent=2))
        spawn_pose = [float(v) for v in ET.parse(args.world).findtext(
            './/model[@name="tethys"]/pose').split()]
        params = {'use_sim_time': 'true', 'yaw_bias': '3.14159265', 'drone_name': 'tethys',
                  'world_name': 'mondoCorridoio', 'n_obstacles': str(obstacle_count),
                  'csv_path': str(output / 'controller_metrics.csv'),
                  'target_x': '0.0', 'target_y': '50.0', 'target_z': '8.0',
                  'spawn_x': str(spawn_pose[0]), 'spawn_y': str(spawn_pose[1]),
                  'spawn_z': str(spawn_pose[2]),
                  'goal_radius': '3.0', 'depth_limit_high': '17.0',
                  'allow_vertical_escape': 'false', 'wall_wx': '15.0', 'wall_wy': '60.0',
                  'inner_wx': '60.0', 'inner_wy': '60.0'}
        argv = [str(output/'controller'), '--ros-args']
        for k, v in params.items():
            argv += ['-p', f'{k}:={v}']
        mission_start = recorder.clock
        controller = launch(argv, 'controller') if args.elevator is None and not args.stationary else None
        diagnostic_pubs = []
        if args.elevator is not None or args.stationary:
            for topic in ['joint/propeller_joint/cmd_thrust', 'vertical_fins/cmd_pos', 'horizontal_fins/cmd_pos']:
                diagnostic_pubs.append(recorder.create_publisher(Float64, '/model/tethys/' + topic, 10))
        print(f'RUN {args.run} {args.label}: started at sim={mission_start:.2f}; '
              f'obstacles={obstacle_count}; world={metadata["world_sha256"][:12]}', flush=True)
        deadline = time.monotonic() + args.wall_limit
        next_progress = mission_start + 20
        reason = 'sim_timeout'
        # Stallo: la missione e' scartata se per --stall-abort-s secondi simulati
        # la distanza dal goal non migliora di almeno --stall-margin-m.
        best_goal_dist, best_goal_t = float('inf'), mission_start
        while recorder.clock - mission_start < args.duration:
            rclpy.spin_once(recorder, timeout_sec=0.05)
            if diagnostic_pubs:
                values = [0.0, 0.0, 0.0] if args.stationary else [args.diagnostic_thrust, 0.0, args.elevator]
                for pub, value in zip(diagnostic_pubs, values):
                    pub.publish(Float64(data=value))
            if controller is not None and controller.poll() is not None:
                reason = f'controller_exit_{controller.returncode}'
                break
            if sim.poll() is not None:
                reason = 'gazebo_exit'
                break
            if time.monotonic() > deadline:
                reason = 'wall_timeout'
                break
            if args.stall_abort_s > 0 and recorder.rows:
                goal_dist = recorder.rows[-1][-1]
                if goal_dist < best_goal_dist - args.stall_margin_m:
                    best_goal_dist, best_goal_t = goal_dist, recorder.clock
                elif recorder.clock - best_goal_t >= args.stall_abort_s:
                    reason = 'stall_abort'
                    print(f'RUN {args.run} scartata: {args.stall_abort_s:.0f}s simulati senza '
                          f'avvicinarsi al goal (minimo {best_goal_dist:.1f} m)', flush=True)
                    break
            if recorder.clock >= next_progress:
                row = recorder.rows[-1]
                print(f'RUN {args.run} t={recorder.clock-mission_start:.0f}s '
                      f'xyz=({row[2]:.1f},{row[3]:.1f},{row[4]:.1f}) dist={row[-1]:.1f} '
                      f'thr={row[8]:.1f}', flush=True)
                next_progress = recorder.clock + 20
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGTERM)
                    try:
                        proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid, signal.SIGKILL)
                        proc.wait(timeout=3)
        recorder.file.close()
        for h in handles:
            h.close()
        recorder.destroy_node()
        rclpy.shutdown()
    rows = [r for r in recorder.rows if mission_start is not None and r[0] >= mission_start]
    log = (output / 'controller.log').read_text() if (output / 'controller.log').exists() else ''
    gazebo_log = (output / 'gazebo.log').read_text()
    # Contact sensors emit only events. Verify advertisement + exact bridge
    # mapping before interpreting silence as zero observed physical contacts.
    contact_pipeline = all(
        f'link/{link}/sensor/review_contacts/contact' in gazebo_log
        and f'link/{link}/sensor/review_contacts/contact' in config
        for link in contact_links)
    collisions = list(re.finditer(r'\[(?:COLLISIONE|NEAR-CONTACT) #(\d+)\]', log))
    distance, slow, measured = 0.0, 0.0, 0.0
    for a, b in zip(rows, rows[1:]):
        dt = b[0] - a[0]
        step = math.dist(a[2:5], b[2:5])
        if 0 < dt < 1 and step/dt < 5:
            distance += step
            measured += dt
            if step/dt < 0.4:
                slow += dt
    goal_confirmation = re.search(r'\[GOAL\].*raggiunto! dist=([\d.]+) m', log)
    summary = {'run': args.run, 'label': args.label, 'stop_reason': reason,
               'physical_contact_events': len(recorder.contact_events) if contact_pipeline else None,
               'contact_pipeline_configured': contact_pipeline,
               'contact_messages_received': recorder.contact_messages,
               'goal': bool(re.search(r'\[GOAL\].*raggiunto', log)),
               # The recorder samples at 5 Hz; its last pose can precede the
               # controller's goal check. Preserve both measurements explicitly.
               'controller_goal_dist_m': float(goal_confirmation[1]) if goal_confirmation else None,
               'sonar_collision_events': max([int(c.group(1)) for c in collisions], default=0),
               'min_goal_dist_m': min([r[-1] for r in rows], default=None),
               'final_goal_dist_m': rows[-1][-1] if rows else None,
               'mean_speed_m_s': distance/measured if measured else None,
               'below_0_4_m_s_sim_s': slow, 'measured_sim_s': measured,
               # Wall clock of the mission itself, from the telemetry's own
               # time.time() column: it excludes Gazebo start-up and teardown,
               # unlike the run directory's file timestamps.
               'mission_wall_s': (rows[-1][1] - rows[0][1]) if len(rows) > 1 else None,
               'rt_factor': (measured / (rows[-1][1] - rows[0][1]))
                            if len(rows) > 1 and rows[-1][1] > rows[0][1] else None,
               'distance_m': distance, 'z_min': min([r[4] for r in rows], default=None),
               'z_max': max([r[4] for r in rows], default=None),
               'abs_roll_max_deg': max([abs(r[5])*180/math.pi for r in rows], default=None)}
    for tag in ['TURN-BOOST', 'TURN-SLOWDOWN', 'TURN-NOT-CLOSING', 'TURN-BRAKE-ALLOW',
                'CURVE-BRAKE', 'SIDE-AVOID', 'ALT-RECOVERY-START', 'DEPTH LIMIT']:
        summary[tag] = log.count('[' + tag + ']')
    (output / 'summary.json').write_text(json.dumps(summary, indent=2))
    (output / 'contacts.json').write_text(json.dumps(recorder.contact_events, indent=2))
    (output / 'sonar_samples.json').write_text(json.dumps(recorder.sonar_samples, indent=2))
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == '__main__':
    main()
