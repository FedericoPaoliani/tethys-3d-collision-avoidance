#!/usr/bin/env python3
"""Regression checks for the active and generated Fortress vehicle configurations."""
from pathlib import Path
import argparse
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def check_goal_visibility(path, drone_names=('tethys',)):
    """I marker sferici dei goal sono visual senza collisione, ma i gpu_lidar li vedono
    lo stesso: vanno esclusi dai sonar di OGNI drone, non solo del primo."""
    root = ET.parse(path).getroot()
    assert tuple(map(int, root.get('version').split('.'))) >= (1, 9), path
    world = root.find('world')
    markers = [m for m in world.findall('model') if (m.get('name') or '').startswith('target_')]
    assert len(markers) == len(drone_names), (path, len(markers), len(drone_names))
    flags = []
    for marker in markers:
        assert not marker.findall('.//collision'), (path, marker.get('name'))
        marker_flags = [int(v.findtext('visibility_flags', '4294967295'))
                        for v in marker.findall('.//visual')]
        assert marker_flags and all(marker_flags), (path, marker.get('name'))  # Visibile nella GUI.
        flags += marker_flags
    for drone in drone_names:
        sensors = world.findall(f"model[@name='{drone}']/link/sensor[@type='gpu_lidar']")
        assert len(sensors) == 5, (path, drone, len(sensors))
        for sensor in sensors:
            mask = int(sensor.findtext('lidar/visibility_mask', '4294967295'))
            assert all(mask & flag == 0 for flag in flags), (path, drone, sensor.get('name'))
            for model in world.findall('model'):
                if model in markers:
                    continue
                for visual in model.findall('.//visual'):
                    assert mask & int(visual.findtext('visibility_flags', '4294967295')), (
                        path, drone, model.get('name'))
    print(f'PASS {Path(path).name}: {len(markers)} marker del goal esclusi da '
          f'{5 * len(drone_names)} sonar; altri visual conservati')


def check(path, width, model_name='tethys'):
    world = ET.parse(path).getroot().find('world')
    model = world.find(f"model[@name='{model_name}']")
    assert model is not None, (path, model_name)
    assert float(world.findtext("plugin[@name='ignition::gazebo::systems::Buoyancy']/uniform_fluid_density")) == 1025, path
    ballast = model.find("joint[@name='drop_weight_joint']")
    assert ballast is not None and ballast.findtext('parent') == 'base_link' and ballast.findtext('child') == 'drop_weight', path
    # Every link must belong to the rigid/articulated vehicle graph. The old
    # flat model silently let its ballast fall independently under gravity.
    edges = [(j.findtext('parent'), j.findtext('child')) for j in model.findall('joint')]
    attached = {'base_link'}
    for _ in model.findall('link'):
        for parent, child in edges:
            if parent in attached:
                attached.add(child)
    assert attached == {link.get('name') for link in model.findall('link')}, path
    mass = sum(float(link.findtext('inertial/mass')) for link in model.findall('link'))
    volume = 0.0
    for size in model.findall('link/collision/geometry/box/size'):
        x, y, z = map(float, size.text.split())
        volume += x*y*z
    assert abs(mass - 1025*volume) < 1e-5, (path, mass, volume)
    assert world.find("plugin[@name='ignition::gazebo::systems::Contact']") is not None, path
    for system in ['Hydrodynamics', 'BuoyancyEngine']:
        plugins = [p for p in model.findall('plugin') if p.get('name').endswith('::'+system)]
        assert len(plugins) == 1, (path, system)
        assert not any(p.get('name', '').endswith('::'+system) for p in model.findall('link/plugin')), path
    hydro = next(p for p in model.findall('plugin') if p.get('name').endswith('::Hydrodynamics'))
    assert hydro.findtext('disable_coriolis') == 'true', path
    assert hydro.find('enable_coriolis') is None, path
    elevator = model.find("joint[@name='horizontal_fins_joint']/axis/xyz")
    assert list(map(float, elevator.text.split())) == [0, 1, 0], path
    lift = next(p for p in model.findall('plugin') if p.findtext('link_name') == 'horizontal_fins')
    assert list(map(float, lift.findtext('upward').split())) == [0, 0, 1], path
    engine = next(p for p in model.findall('plugin') if p.get('name').endswith('::BuoyancyEngine'))
    assert float(engine.findtext('min_volume')) <= float(engine.findtext('neutral_volume')) <= float(engine.findtext('max_volume')), path
    assert float(engine.findtext('default_volume')) == float(engine.findtext('neutral_volume')), path
    front = model.find("link[@name='base_link']/sensor[@name='frontal_sonar']")
    assert front.findtext('lidar/scan/horizontal/samples') == str(width), path
    assert front.findtext('lidar/scan/vertical/samples') == '15', path
    assert float(front.findtext('update_rate')) == 5, path
    observed = 0
    for link in model.findall('link'):
        names = {c.get('name') for c in link.findall('collision')}
        if not names:
            continue
        sensor = link.find("sensor[@name='collision_contacts']")
        assert sensor is not None, (path, link.get('name'))
        assert {c.text for c in sensor.findall('contact/collision')} == names, path
        observed += 1
    assert observed == 6, path
    print(f'PASS {Path(path).name} [{model_name}]: model plugins, elevator plane, buoyancy volume, frontal sonar, six contact links')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--corridor-only', action='store_true',
                        help='Check only the current single-drone corridor, without generating worlds')
    args = parser.parse_args()
    check_goal_visibility(ROOT/'worlds/mondoCorridoio.sdf')
    if args.corridor_only:
        check(ROOT/'worlds/mondoCorridoio.sdf', 30)
        return
    # Mondi definitivi della campagna (worlds/): 3 livelli a 1 drone e 3 a 5 droni.
    for name in ('mondoCorridoio.sdf', 'mondoCorridoio_L2.sdf', 'mondoCorridoio_L3.sdf'):
        check_goal_visibility(ROOT/'worlds'/name)
        check(ROOT/'worlds'/name, 30)
    # Mondi a corsie parallele: ogni drone porta la propria copia flat del modello,
    # quindi i tre difetti storici vanno verificati su tutti, non solo sul primo.
    for name in ('mondoMulti5.sdf', 'mondoMulti5_L2.sdf', 'mondoMulti5_L3.sdf'):
        check_goal_visibility(ROOT/'worlds'/name, [f'tethys_{i}' for i in range(5)])
        for i in range(5):
            check(ROOT/'worlds'/name, 30, model_name=f'tethys_{i}')
    with tempfile.TemporaryDirectory(prefix='tethys-config-') as tmp:
        single = Path(tmp)/'generated_single.sdf'
        subprocess.run(['python3', str(ROOT/'scripts/generate_world.py'), '7', '--n-drones', '1',
                        '--n-obstacles', '2', '--only-vert', '--parallel-lanes', '--output', str(single)], check=True)
        check(single, 30)


if __name__ == '__main__':
    main()
