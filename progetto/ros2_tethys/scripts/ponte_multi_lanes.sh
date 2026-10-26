#!/bin/bash
# Bridge ROS 2 <-> Gazebo per i mondi a corsie parallele mondoMulti3 / mondoMulti5.
# Uso: ./ros2_tethys/scripts/ponte_multi_lanes.sh [n_droni]   (default: 5)
#
# Va avviato PRIMA dei nodi: senza bridge i nodi restano a spinta zero
# finche' non arriva il primo frame sonar.
#
# La config e' generata da ponte_single_config.yaml, replicando per ogni drone le
# voci /model/tethys/ e lasciando posa e clock una volta sola. Le pinne JPC in
# Fortress ascoltano /model/<nome>/joint/<giunto>/0/cmd_pos: la vecchia sintassi
# "topic@tipo@tipo" usava lo stesso nome dai due lati e le mandava su
# /model/<nome>/vertical_fins/cmd_pos, dove nessun plugin ascolta — le pinne dei
# droni multi restavano ferme.
source /opt/ros/humble/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
N_DRONES="${1:-5}"
CONFIG="/tmp/ponte_multi${N_DRONES}_config.yaml"

python3 - "$SCRIPT_DIR/ponte_single_config.yaml" "$N_DRONES" "$CONFIG" <<'PY'
import sys
import yaml
single = yaml.safe_load(open(sys.argv[1]))
n = int(sys.argv[2])
entries = [e for e in single if 'model/tethys/' not in e['gz_topic_name']]
for i in range(n):
    for e in single:
        if 'model/tethys/' in e['gz_topic_name']:
            entries.append({k: (v.replace('model/tethys/', f'model/tethys_{i}/')
                                if k.endswith('topic_name') else v) for k, v in e.items()})
open(sys.argv[3], 'w').write(yaml.safe_dump(entries, sort_keys=False))
PY

echo "Avvio bridge multi-drone ($N_DRONES droni: tethys_0 ... tethys_$((N_DRONES - 1)), config: $CONFIG)..."

ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -p config_file:="$CONFIG"
