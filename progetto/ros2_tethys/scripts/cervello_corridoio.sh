#!/bin/bash
# Lancia il nodo collision_avoidance per singolo drone (tethys) in mondoCorridoio.sdf.
# Uso: ./ros2_tethys/scripts/cervello_corridoio.sh [n_ostacoli]
#   $1  n_obstacles  (default: conteggio dal mondo SDF)

source /opt/ros/humble/setup.bash
source ~/ros2_tethys/install/setup.bash

pkill -f "collision_avoidance_node" 2>/dev/null
sleep 2

SDF="${MONDO:-$HOME/ros2_tethys/worlds/mondoCorridoio.sdf}"   # MONDO sceglie un altro mondo

if [ ! -f "$SDF" ]; then
    echo "ERRORE: $SDF non trovato. Genera con: python3 scripts/generate_world.py 80 --n-drones 1 --n-obstacles 80 --parallel-lanes --spawn-z-min 10 --spawn-z-max 10 --output ~/ros2_tethys/mondoCorridoio.sdf"
    exit 1
fi

if [ "$#" -gt 0 ]; then
    N_OBS="$1"
else
    N_OBS=$(python3 - "$SDF" <<'PY'
import sys
import xml.etree.ElementTree as ET
world = ET.parse(sys.argv[1]).getroot().find('world')
obstacles = world.find("model[@name='ostacoli']")
print(len(obstacles.findall('link')) if obstacles is not None else 0)
PY
    )
fi

# Leggi spawn dal commento SDF
SPAWN_RAW=$(grep -oP '(?<=Spawn: \()([^)]+)' "$SDF" | head -1)
SX=$(echo "$SPAWN_RAW" | cut -d',' -f1 | tr -d ' ')
SY=$(echo "$SPAWN_RAW" | cut -d',' -f2 | tr -d ' ')
SZ=$(echo "$SPAWN_RAW" | cut -d',' -f3 | tr -d ' ')

# Leggi target dalla posa del modello target_0
TARGET_RAW=$(grep -A3 'model name="target_0"' "$SDF" | grep -oP '(?<=<pose>)[^<]+' | head -1)
TX=$(echo "$TARGET_RAW" | awk '{print $1}')
TY=$(echo "$TARGET_RAW" | awk '{print $2}')
TZ=$(echo "$TARGET_RAW" | awk '{print $3}')

# Leggi dimensioni vasca
WALL_WX=$(grep -oP '(?<=wall_wx=)[0-9.]+' "$SDF" | head -1)
WALL_WY=$(grep -oP '(?<=wall_wy=)[0-9.]+' "$SDF" | head -1)
WALL_WX="${WALL_WX:-79.75}"
WALL_WY="${WALL_WY:-79.75}"

echo "Single-drone: spawn=(${SX},${SY},${SZ}) target=(${TX},${TY},${TZ}) wall_wy=${WALL_WY} n_obs=${N_OBS}"

ros2 run collision_avoidance collision_avoidance_node \
    --ros-args \
    -p use_sim_time:=true \
    -p yaw_bias:=3.14159265 \
    -p drone_name:="tethys" \
    -p world_name:="mondoCorridoio" \
    -p n_obstacles:=${N_OBS} \
    -p csv_path:="$HOME/ros2_tethys/docs/risultati_corridoio.csv" \
    -p target_x:="${TX}" \
    -p target_y:="${TY}" \
    -p target_z:="${TZ}" \
    -p spawn_x:="${SX}" \
    -p spawn_y:="${SY}" \
    -p spawn_z:="${SZ}" \
    -p goal_radius:=3.0 \
    -p depth_limit_high:=17.0 \
    -p allow_vertical_escape:=false \
    -p wall_wx:="${WALL_WX}" \
    -p wall_wy:="${WALL_WY}" \
    -p inner_wx:=60.0 \
    -p inner_wy:=60.0 \
    > "/tmp/drone_corridoio.log" 2>&1 &

PID=$!
echo "Nodo avviato (PID=$PID) — log in /tmp/drone_corridoio.log"
echo "Per seguire in tempo reale: tail -f /tmp/drone_corridoio.log"
wait "$PID"
echo "Missione completata. Risultati in ~/ros2_tethys/docs/risultati_corridoio.csv"
