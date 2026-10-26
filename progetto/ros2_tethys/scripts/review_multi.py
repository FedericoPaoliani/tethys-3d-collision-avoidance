#!/usr/bin/env python3
"""Run one isolated multi-drone mission (mondoMulti3 / mondoMulti5) and preserve
executable, sources, world, logs and ROS telemetry — the N-drone counterpart of
review_corridor.py, which stays untouched so historical corridor runs remain
reproducible.

Source ROS before running. Run sequentially: GPU lidar is shared.

Differences from the corridor harness:
- one controller per drone (tethys_0 … tethys_{N-1}), spawn/target read from the SDF;
- the bridge config is built from ponte_single_config.yaml, whose fin topics map to
  the JPC default /joint/<name>/0/cmd_pos (ponte_multi_lanes.sh maps them to a topic
  no plugin subscribes to);
- stall is tracked per drone; the mission ends when every drone has either reached
  its goal, exited, or stalled;
- physical contacts are classified as drone-drone or drone-environment, and the
  minimum horizontal/3-D distance between drones is recorded;
- on teardown every process still carrying this run's IGN_PARTITION is SIGKILLed
  (review_corridor.py left orphaned `ign gazebo` servers behind);
- the controllers see a wrapper `ign` first in PATH. At its goal the node asks
  Gazebo to *remove* its model; in Fortress removing a model that carries gpu_lidar
  sensors while the others keep rendering aborts the server inside Ogre2
  (HlmsDatablock destructor, run_01). The wrapper turns that request into a
  set_pose far outside the tank, beyond sonar range: for the other drones the effect
  is the same as a removal, the controller binary stays the one tested on the corridor.
"""
import argparse
import atexit
import csv
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from ros_gz_interfaces.msg import Contacts

ROOT = Path(__file__).resolve().parents[1]
PARK_X, PARK_Y, PARK_Z = 1000.0, 1000.0, -100.0   # far outside the tank, per drone +100 m in x
IGN_WRAPPER = r'''#!/bin/bash
# Written by review_multi.py: turns the controller's model removal into a
# set_pose outside the tank (removing gpu_lidar models crashes Fortress/Ogre2).
REAL="{real}"
if [[ "$*" == *"/world/buoyant_tethys/remove"* ]]; then
    name=$(printf '%s' "$*" | grep -oP 'name: "\K[^"]+')
    idx=${{name##*_}}
    echo "$(date +%s.%N) remove $name -> set_pose" >> "{log}"
    exec "$REAL" service -s /world/buoyant_tethys/set_pose \
        --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 2000 \
        --req "name: \"$name\" position: {{x: $(( {px} + 100*idx )) y: {py} z: {pz}}}"
fi
exec "$REAL" "$@"
'''
CONTACT_LINKS = ['base_link', 'horizontal_fins', 'vertical_fins', 'propeller',
                 'buoyancy_engine', 'acoustic_transponder']


class Recorder(Node):
    def __init__(self, output, drones):
        super().__init__('multi_review_recorder')
        self.drones = drones            # {name: {'target': (x,y,z)}}
        self.clock = 0.0
        self.front_seen = set()
        self.pose_seen = set()
        self.last_t = {d: -1.0 for d in drones}
        self.last_pos = {}
        self.commands = {d: [0.0, 0.0, 0.0] for d in drones}
        self.rows = {d: [] for d in drones}
        self.contact_events = {d: [] for d in drones}
        self.last_contact = {d: -100.0 for d in drones}
        self.contact_messages = 0
        self.min_sep = {'dist_3d_m': float('inf'), 'pair': None, 'sim_t': None}
        self.frames_saved = False
        self.finished = set()           # drones at goal / exited: parked, stop recording
        # Wall-clock arrival of every input stream: a gap here means the stream
        # stopped, whatever the controller then did with stale data.
        self.arrivals = {}
        self.output = output
        self.file = (output / 'telemetry.csv').open('w')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['sim_t', 'wall_t', 'drone', 'x', 'y', 'z', 'roll', 'pitch',
                              'yaw', 'thrust', 'rudder', 'elevator', 'goal_dist'])
        # Best effort: /clock arrives at ~1 kHz, and a slow reliable Python reader can
        # back-pressure the single bridge that also feeds every controller.
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/world/buoyant_tethys/dynamic_pose/info',
                                 self.on_pose, qos_profile_sensor_data)
        for d in drones:
            self.create_subscription(PointCloud2, f'/model/{d}/frontal_sonar/points',
                                     lambda msg, d=d: (self.front_seen.add(d),
                                                       self.arrived(f'{d}/frontal_sonar')),
                                     qos_profile_sensor_data)
            for link in CONTACT_LINKS:
                self.create_subscription(Contacts, f'/model/{d}/review_contacts/{link}',
                                         lambda msg, d=d: self.on_contacts(d, msg), 10)
            for i, topic in enumerate(['joint/propeller_joint/cmd_thrust',
                                       'vertical_fins/cmd_pos', 'horizontal_fins/cmd_pos']):
                self.create_subscription(
                    Float64, f'/model/{d}/{topic}',
                    lambda msg, d=d, idx=i: self.commands[d].__setitem__(idx, msg.data), 10)

    def arrived(self, stream):
        now = time.monotonic()
        a = self.arrivals.setdefault(stream, {'n': 0, 'last': None, 'gaps': []})
        if a['last'] is not None and now - a['last'] > 0.6:
            a['gaps'].append({'sim_t': round(self.clock, 2), 'wall_gap_s': round(now - a['last'], 2),
                              'wall_end': round(time.time(), 2)})
        a['last'] = now
        a['n'] += 1

    def on_clock(self, msg):
        self.arrived('clock')
        self.clock = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_contacts(self, drone, msg):
        self.contact_messages += 1
        if not msg.contacts:
            return
        names = sorted({n for c in msg.contacts
                        for n in (c.collision1.name, c.collision2.name)})
        others = sorted({m for n in names for m in re.findall(r'(tethys_\d+)', n)} - {drone})
        if self.clock - self.last_contact[drone] > 1.0:
            self.contact_events[drone].append({
                'sim_t': self.clock, 'wall_t': time.time(), 'entities': names,
                'other_drones': others, 'drone_drone': bool(others)})
        elif others:
            # Same episode, but a drone joined it: keep the classification honest.
            ev = self.contact_events[drone][-1]
            ev['other_drones'] = sorted(set(ev['other_drones']) | set(others))
            ev['drone_drone'] = True
        self.last_contact[drone] = self.clock

    def on_pose(self, msg):
        self.arrived('pose')
        if not self.frames_saved and self.clock > 5:
            (self.output / 'frames.txt').write_text(str(msg))
            self.frames_saved = True
        updated = False
        for tf in msg.transforms:
            d = tf.child_frame_id
            if d not in self.drones or d in self.finished:
                continue
            if abs(tf.transform.translation.x) > PARK_X / 2:
                continue                # parked by the ign wrapper before exit was seen
            self.pose_seen.add(d)
            t = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            if t == 0.0:
                t = self.clock
            p, q = tf.transform.translation, tf.transform.rotation
            self.last_pos[d] = (p.x, p.y, p.z, t)
            updated = True
            if t - self.last_t[d] < 0.19:
                continue
            self.last_t[d] = t
            roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
            pitch = math.asin(max(-1.0, min(1.0, 2*(q.w*q.y-q.z*q.x))))
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            tx, ty, tz = self.drones[d]['target']
            dist = math.sqrt((p.x-tx)**2 + (p.y-ty)**2 + (p.z-tz)**2)
            row = [t, time.time(), p.x, p.y, p.z, roll, pitch, yaw, *self.commands[d], dist]
            self.rows[d].append(row)
            self.writer.writerow(row[:2] + [d] + row[2:])
        if updated:
            self.file.flush()
            # Only drones still in the scene: a drone removed at its goal keeps its
            # last pose here, but it no longer appears in fresh messages.
            fresh = {d: v for d, v in self.last_pos.items() if abs(v[3] - self.clock) < 1.0}
            for a, b in itertools.combinations(sorted(fresh), 2):
                sep = math.dist(fresh[a][:3], fresh[b][:3])
                if sep < self.min_sep['dist_3d_m']:
                    self.min_sep = {'dist_3d_m': sep, 'pair': [a, b], 'sim_t': self.clock}


def kill_partition(partition):
    """SIGKILL every process whose environment carries this run's IGN_PARTITION."""
    killed = []
    needle = f'IGN_PARTITION={partition}'.encode()
    for pid_dir in Path('/proc').iterdir():
        if not pid_dir.name.isdigit() or int(pid_dir.name) == os.getpid():
            continue
        try:
            env = (pid_dir / 'environ').read_bytes().split(b'\0')
        except (PermissionError, FileNotFoundError, ProcessLookupError):
            continue
        if needle in env:
            try:
                os.kill(int(pid_dir.name), signal.SIGKILL)
                killed.append(int(pid_dir.name))
            except ProcessLookupError:
                pass
    return killed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', required=True, type=int)
    ap.add_argument('--world', type=Path, default=ROOT/'worlds/mondoMulti5.sdf')
    ap.add_argument('--label', default='multi')
    ap.add_argument('--duration', type=float, default=900.0,
                    help='Tetto di tempo SIMULATO della missione [s]')
    ap.add_argument('--wall-limit', type=float, default=2400.0,
                    help='Tetto di tempo REALE della missione [s]')
    ap.add_argument('--stall-abort-s', type=float, default=60.0,
                    help='Un drone e\' in stallo dopo N s simulati senza avvicinarsi al '
                         'proprio goal di almeno --stall-margin-m (0 = disattivato)')
    ap.add_argument('--stall-margin-m', type=float, default=1.0)
    ap.add_argument('--no-controllers', action='store_true',
                    help='Diagnostica: droni fermi, nessun controller; dura --duration s simulati')
    ap.add_argument('--gz-pose-probe', action='store_true',
                    help='Diagnostica: ascolta la posa anche lato Gazebo (ign topic -e) e '
                         'registra in gz_pose_probe.log ogni pausa > 0,6 s del flusso')
    ap.add_argument('--min-goals', type=int, default=3,
                    help='Droni a destinazione perche\' la missione conti come riuscita')
    args = ap.parse_args()

    world_path = args.world.resolve()
    final = ROOT / 'reports/multi_review' / world_path.stem / f'run_{args.run:02d}'
    if final.exists():
        raise SystemExit(f'{final} esiste gia\'')
    final.parent.mkdir(parents=True, exist_ok=True)
    (final.parents[1] / 'COLCON_IGNORE').touch()
    # Everything is written to RAM during the mission and copied at the end. On
    # this machine each ext4 journal commit (every 5 s) can hold writers in
    # do_get_write_access for seconds; recorder and controllers, which log
    # continuously, froze with it and stopped consuming pose/sonar/clock while
    # Gazebo kept publishing (diagnostics run_101-105).
    output = Path('/dev/shm') / f'multi_review_{world_path.stem}_run_{args.run:02d}'
    if output.exists():
        shutil.rmtree(output)
    output.mkdir()

    def copy_back():
        shutil.copytree(output, final, symlinks=True)
        shutil.rmtree(output)
        print(f'risultati copiati in {final}', flush=True)
    atexit.register(copy_back)
    partition = f'multi_review_{os.getpid()}'
    os.environ['IGN_PARTITION'] = partition
    # ign-transport (Gazebo <-> bridge) otherwise binds to the WiFi address: each time
    # the WiFi dropped and the IP expired ("link timed out"), the bridge stopped
    # receiving for 14-200 s while Gazebo kept simulating (runs 229-231).
    os.environ['IGN_IP'] = '127.0.0.1'
    os.environ['ROS_DOMAIN_ID'] = '88'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_LOG_DIR'] = str(output / 'ros_logs')

    tree = ET.parse(world_path)
    world = tree.getroot().find('world')
    drone_models = sorted((m for m in world.findall('model')
                           if re.fullmatch(r'tethys_\d+', m.get('name', ''))),
                          key=lambda m: int(m.get('name').split('_')[1]))
    drones = {}
    for m in drone_models:
        name = m.get('name')
        idx = name.split('_')[1]
        target_model = world.find(f"model[@name='target_{idx}']")
        drones[name] = {
            'spawn': [float(v) for v in m.findtext('pose').split()[:3]],
            'target': tuple(float(v) for v in target_model.findtext('pose').split()[:3])}
    obstacles_model = world.find("model[@name='ostacoli']")
    obstacle_count = len(obstacles_model.findall('link')) if obstacles_model is not None else 0
    tank = re.search(r'wall_wx=([\d.]+) wall_wy=([\d.]+)', world_path.read_text())
    wall_wx, wall_wy = tank.groups() if tank else ('30.0', '60.0')
    min_goals = min(args.min_goals, len(drones))

    processes, handles = [], []

    def launch(argv, name, env=None):
        h = (output / (name + '.log')).open('w')
        handles.append(h)
        proc = subprocess.Popen(argv, cwd=ROOT, stdout=h, stderr=subprocess.STDOUT,
                                start_new_session=True, env=env)
        processes.append(proc)
        return proc

    rclpy.init()
    recorder = Recorder(output, drones)
    reason, mission_start = 'startup_failed', None
    status = {d: {'state': 'running', 'end_sim_t': None} for d in drones}
    controllers = {}
    orphans_killed = []
    try:
        # Observation only: contact sensors on every collision-bearing link.
        if not any(p.get('name') == 'ignition::gazebo::systems::Contact'
                   for p in world.findall('plugin')):
            ET.SubElement(world, 'plugin', filename='ignition-gazebo-contact-system',
                          name='ignition::gazebo::systems::Contact')
        contact_links = {}
        for m in drone_models:
            d = m.get('name')
            contact_links[d] = []
            for link in m.findall('link'):
                collisions = link.findall('collision')
                if not collisions:
                    continue
                contact_links[d].append(link.get('name'))
                sensor = ET.SubElement(link, 'sensor', name='review_contacts', type='contact')
                ET.SubElement(sensor, 'always_on').text = 'true'
                ET.SubElement(sensor, 'update_rate').text = '20'
                ET.SubElement(sensor, 'topic').text = f'/model/{d}/review_contacts'
                contact = ET.SubElement(sensor, 'contact')
                for collision in collisions:
                    ET.SubElement(contact, 'collision').text = collision.get('name')
        tree.write(output / 'instrumented_world.sdf')

        # Bridge: per-drone copy of the single-drone config (correct JPC fin topics),
        # global entries (pose, clock) once.
        single = yaml.safe_load((ROOT/'scripts/ponte_single_config.yaml').read_text())
        entries = [e for e in single if 'model/tethys/' not in e['gz_topic_name']]
        for d in drones:
            for e in single:
                if 'model/tethys/' in e['gz_topic_name']:
                    entries.append({k: (v.replace('model/tethys/', f'model/{d}/')
                                        if k.endswith('topic_name') else v)
                                    for k, v in e.items()})
            for link in contact_links[d]:
                entries.append({
                    'ros_topic_name': f'/model/{d}/review_contacts/{link}',
                    'gz_topic_name': f'/world/buoyant_tethys/model/{d}/link/{link}'
                                     '/sensor/review_contacts/contact',
                    'ros_type_name': 'ros_gz_interfaces/msg/Contacts',
                    'gz_type_name': 'ignition.msgs.Contacts', 'direction': 'GZ_TO_ROS'})
        config = yaml.safe_dump(entries, sort_keys=False)
        (output / 'bridge.yaml').write_text(config)
        launch(['/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge', '--ros-args',
                '-p', f'config_file:={output}/bridge.yaml'], 'bridge')

        startup = output / 'start_gazebo.sh'
        text = (ROOT/'scripts/start_sim_multi5_headless.sh').read_text()
        startup.write_text(text.replace('-v 3', '-v 4').replace(
            '~/ros2_tethys/worlds/mondoMulti5.sdf', str(output / 'instrumented_world.sdf')))
        sim = launch(['bash', str(startup)], 'gazebo')
        if args.gz_pose_probe:
            probe = ("import sys,time\nlast=None;n=0\nfor line in sys.stdin:\n"
                     "    now=time.monotonic();n+=1\n"
                     "    if last is not None and now-last>0.6: print(f'GAP {time.time():.2f} {now-last:.2f} lines={n}',flush=True)\n"
                     "    last=now\n")
            launch(['bash', '-c', 'ign topic -e -t /world/buoyant_tethys/dynamic_pose/info '
                    f'| python3 -u -c "{probe}"'], 'gz_pose_probe')
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline and sim.poll() is None:
            rclpy.spin_once(recorder, timeout_sec=0.1)
            if (recorder.front_seen == set(drones) and recorder.pose_seen == set(drones)
                    and recorder.clock > 0):
                break
        else:
            raise RuntimeError('No clock, pose and frontal sonar of every drone within 90 s')

        executable = ROOT/'install/collision_avoidance/lib/collision_avoidance/collision_avoidance_node'
        shutil.copy2(executable, output / 'controller')
        package_dir = ROOT/'src/collision_avoidance'
        for f in ['collision_avoidance_node.cpp', 'vfh_planner.cpp']:
            shutil.copy2(package_dir/'src'/f, output/f)
        shutil.copytree(package_dir/'include', output/'include')
        shutil.copy2(package_dir/'CMakeLists.txt', output/'CMakeLists.txt')
        shutil.copy2(package_dir/'package.xml', output/'package.xml')
        shutil.copy2(world_path, output/'world.sdf')
        sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
        metadata = {'label': args.label, 'world_source': str(world_path),
                    'world_sha256': sha(output/'world.sdf'),
                    'n_drones': len(drones), 'min_goals_for_success': min_goals,
                    'obstacle_count': obstacle_count,
                    'drones': {d: {'spawn': v['spawn'], 'target': list(v['target'])}
                               for d, v in drones.items()},
                    'duration_limit_sim_s': args.duration, 'wall_limit_s': args.wall_limit,
                    'stall_abort_s': args.stall_abort_s, 'stall_margin_m': args.stall_margin_m,
                    'source_sha256': sha(output/'collision_avoidance_node.cpp'),
                    'planner_sha256': sha(output/'vfh_planner.cpp'),
                    'binary_sha256': sha(output/'controller'),
                    'headers_sha256': {str(p.relative_to(output/'include')): sha(p)
                                       for p in sorted((output/'include').rglob('*.hpp'))}}
        (output / 'metadata.json').write_text(json.dumps(metadata, indent=2))

        wrapper_dir = output / 'ign_wrapper'
        wrapper_dir.mkdir()
        (wrapper_dir / 'ign').write_text(IGN_WRAPPER.format(
            real=shutil.which('ign'), log=output / 'parked.log',
            px=int(PARK_X), py=PARK_Y, pz=PARK_Z))
        (wrapper_dir / 'ign').chmod(0o755)
        controller_env = dict(os.environ, PATH=f'{wrapper_dir}:{os.environ["PATH"]}')

        # Parameters identical to cervello_multi_lanes.sh.
        for d, v in drones.items():
            params = {'use_sim_time': 'true', 'yaw_bias': '3.14159265', 'drone_name': d,
                      'world_name': world_path.stem, 'n_obstacles': str(obstacle_count),
                      'csv_path': str(output / f'controller_metrics_{d}.csv'),
                      'target_x': str(v['target'][0]), 'target_y': str(v['target'][1]),
                      'target_z': str(v['target'][2]),
                      'spawn_x': str(v['spawn'][0]), 'spawn_y': str(v['spawn'][1]),
                      'spawn_z': str(v['spawn'][2]),
                      'goal_radius': '3.0', 'depth_limit_high': '17.0',
                      'allow_vertical_escape': 'false', 'wall_wx': wall_wx, 'wall_wy': wall_wy}
            argv = [str(output/'controller'), '--ros-args']
            for k, val in params.items():
                argv += ['-p', f'{k}:={val}']
            if not args.no_controllers:
                controllers[d] = launch(argv, f'controller_{d}', controller_env)
        mission_start = recorder.clock
        print(f'RUN {args.run} {world_path.stem}: {len(drones)} droni, start sim='
              f'{mission_start:.2f}; obstacles={obstacle_count}; '
              f'world={metadata["world_sha256"][:12]}', flush=True)

        deadline = time.monotonic() + args.wall_limit
        next_progress = mission_start + 20
        best = {d: (float('inf'), mission_start) for d in drones}
        reason = 'sim_timeout'
        while recorder.clock - mission_start < args.duration:
            rclpy.spin_once(recorder, timeout_sec=0.05)
            for d, proc in controllers.items():
                if status[d]['state'] == 'running' and proc.poll() is not None:
                    log = (output / f'controller_{d}.log').read_text()
                    goal = bool(re.search(r'\[GOAL\].*raggiunto', log))
                    status[d] = {'state': 'goal' if goal else f'exit_{proc.returncode}',
                                 'end_sim_t': recorder.clock - mission_start}
                    recorder.finished.add(d)
                    print(f'RUN {args.run} {d}: {status[d]["state"]} a '
                          f't={status[d]["end_sim_t"]:.1f}s', flush=True)
                if (status[d]['state'] == 'running' and args.stall_abort_s > 0
                        and recorder.rows[d]):
                    gd = recorder.rows[d][-1][-1]
                    if gd < best[d][0] - args.stall_margin_m:
                        best[d] = (gd, recorder.clock)
                    elif recorder.clock - best[d][1] >= args.stall_abort_s:
                        # The drone stays in the scene with its controller running:
                        # for the others it is still a moving obstacle.
                        status[d] = {'state': 'stall', 'end_sim_t': recorder.clock - mission_start}
                        print(f'RUN {args.run} {d}: stallo a t={status[d]["end_sim_t"]:.1f}s '
                              f'(minimo {best[d][0]:.1f} m dal goal)', flush=True)
            if not args.no_controllers and all(s['state'] != 'running' for s in status.values()):
                reason = 'all_resolved'
                break
            if sim.poll() is not None:
                reason = 'gazebo_exit'
                break
            if time.monotonic() > deadline:
                reason = 'wall_timeout'
                break
            if recorder.clock >= next_progress:
                parts = []
                for d in drones:
                    if recorder.rows[d]:
                        r = recorder.rows[d][-1]
                        parts.append(f'{d}[{status[d]["state"]}] ({r[2]:.1f},{r[3]:.1f},'
                                     f'{r[4]:.1f}) d={r[-1]:.1f}')
                print(f'RUN {args.run} t={recorder.clock-mission_start:.0f}s ' + ' | '.join(parts),
                      flush=True)
                next_progress = recorder.clock + 20
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGINT)
                    proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait(timeout=3)
                except ProcessLookupError:
                    pass
        time.sleep(1)
        orphans_killed = kill_partition(partition)
        recorder.file.close()
        for h in handles:
            h.close()
        recorder.destroy_node()
        rclpy.shutdown()

    gazebo_log = (output / 'gazebo.log').read_text()
    per_drone = {}
    for d in drones:
        rows = [r for r in recorder.rows[d] if mission_start is not None and r[0] >= mission_start]
        log_path = output / f'controller_{d}.log'
        log = log_path.read_text() if log_path.exists() else ''
        pipeline = all(f'model/{d}/link/{link}/sensor/review_contacts/contact' in gazebo_log
                       for link in contact_links.get(d, [])) and bool(contact_links.get(d))
        distance, slow, measured = 0.0, 0.0, 0.0
        for a, b in zip(rows, rows[1:]):
            dt = b[0] - a[0]
            step = math.dist(a[2:5], b[2:5])
            if 0 < dt < 1 and step/dt < 5:
                distance += step
                measured += dt
                if step/dt < 0.4:
                    slow += dt
        confirmation = re.search(r'\[GOAL\].*raggiunto! dist=([\d.]+) m', log)
        events = recorder.contact_events[d]
        per_drone[d] = {
            'state': status[d]['state'], 'end_sim_t': status[d]['end_sim_t'],
            # Same rule as the corridor's --stall-abort-s: a drone declared stalled
            # does not count even if it later reaches its goal (run 212 of Multi5).
            'goal': status[d]['state'] == 'goal',
            'goal_after_stall': status[d]['state'] == 'stall'
                                and bool(re.search(r'\[GOAL\].*raggiunto', log)),
            'controller_goal_dist_m': float(confirmation[1]) if confirmation else None,
            'contact_pipeline_configured': pipeline,
            'physical_contact_events': len(events) if pipeline else None,
            'drone_drone_contact_events': sum(e['drone_drone'] for e in events) if pipeline else None,
            'min_goal_dist_m': min([r[-1] for r in rows], default=None),
            'final_goal_dist_m': rows[-1][-1] if rows else None,
            'distance_m': distance, 'measured_sim_s': measured,
            'mean_speed_m_s': distance/measured if measured else None,
            'below_0_4_m_s_sim_s': slow,
            'z_min': min([r[4] for r in rows], default=None),
            'z_max': max([r[4] for r in rows], default=None),
            'abs_roll_max_deg': max([abs(r[5])*180/math.pi for r in rows], default=None)}
        for tag in ['TURN-BOOST', 'TURN-SLOWDOWN', 'TURN-NOT-CLOSING', 'CURVE-BRAKE',
                    'SIDE-AVOID', 'DEPTH LIMIT']:
            per_drone[d][tag] = log.count('[' + tag + ']')
    all_rows = [r for rs in recorder.rows.values() for r in rs
                if mission_start is not None and r[0] >= mission_start]
    t0 = min(all_rows, key=lambda r: r[0]) if all_rows else None
    t1 = max(all_rows, key=lambda r: r[0]) if all_rows else None
    n_goals = sum(v['goal'] for v in per_drone.values())
    summary = {
        'run': args.run, 'label': args.label, 'world': world_path.stem,
        'stop_reason': reason, 'n_drones': len(drones), 'goals': n_goals,
        'min_goals_for_success': min_goals, 'success': n_goals >= min_goals,
        'physical_contact_events': (sum(v['physical_contact_events'] for v in per_drone.values())
                                    if all(v['contact_pipeline_configured'] for v in per_drone.values())
                                    else None),
        'drone_drone_contact_events': (sum(v['drone_drone_contact_events'] for v in per_drone.values())
                                       if all(v['contact_pipeline_configured'] for v in per_drone.values())
                                       else None),
        'contact_messages_received': recorder.contact_messages,
        'min_drone_separation': recorder.min_sep if recorder.min_sep['pair'] else None,
        'mission_sim_s': (recorder.clock - mission_start) if mission_start is not None else None,
        'mission_wall_s': (t1[1] - t0[1]) if t0 and t1 and t1[1] > t0[1] else None,
        'rt_factor': ((t1[0] - t0[0]) / (t1[1] - t0[1])) if t0 and t1 and t1[1] > t0[1] else None,
        'orphan_pids_killed': orphans_killed,
        'input_gaps': {k: {'messages': v['n'], 'gaps_over_0_6_s': len(v['gaps']),
                           'total_gap_wall_s': round(sum(g['wall_gap_s'] for g in v['gaps']), 2),
                           'gaps': v['gaps'][:30]}
                       for k, v in sorted(recorder.arrivals.items())},
        'parked_at_goal': ((output / 'parked.log').read_text().splitlines()
                           if (output / 'parked.log').exists() else []),
        'drones': per_drone}
    (output / 'summary.json').write_text(json.dumps(summary, indent=2))
    (output / 'contacts.json').write_text(json.dumps(recorder.contact_events, indent=2))
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == '__main__':
    main()
