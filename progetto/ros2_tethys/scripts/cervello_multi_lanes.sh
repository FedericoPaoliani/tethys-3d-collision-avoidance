#!/bin/bash
# Lancia un nodo collision_avoidance per ogni drone dei mondi a corsie parallele.
# Uso: ./ros2_tethys/scripts/cervello_multi_lanes.sh [3|5] [n_ostacoli]
#   $1  numero di droni  → usa worlds/mondoMulti${N}.sdf (nel progetto esiste solo mondoMulti5)
#   $2  n_obstacles      → default: conteggio dei link dentro il modello "ostacoli"
#
# Spawn, target e semi-larghezze della vasca sono letti dai commenti dell'SDF
# generati da generate_world.py: non vanno tenuti allineati a mano.

source /opt/ros/humble/setup.bash
source ~/ros2_tethys/install/setup.bash

N_DRONES="${1:-5}"
WORLD_NAME="mondoMulti${N_DRONES}"
SDF="$HOME/ros2_tethys/worlds/${WORLD_NAME}.sdf"

if [ ! -f "$SDF" ]; then
    echo "ERRORE: $SDF non trovato. Rigenera con:"
    echo "  python3 scripts/generate_world.py 9 --n-drones ${N_DRONES} --lane-slots 5 \\"
    echo "      --only-vert --parallel-lanes --tank-half-x 30 --tank-half-y 60 --tank-height 20 \\"
    echo "      --spawn-z-min 8 --spawn-z-max 8 --spawn-y -50 --target-y 50 \\"
    echo "      --obs-z-center 8 --obs-height-min 8 --obs-height-max 14 \\"
    echo "      --n-obstacles 14 --path-obstacles 8 --path-corridor-w 5.0 --output $SDF"
    exit 1
fi

# Uccidi eventuali nodi residui e attendi sempre il cleanup DDS
pkill -f "collision_avoidance_node" 2>/dev/null
sleep 3  # 3s per DDS discovery (8s causava affondamento al fondale con RT≈47%)

# Spawn/target di ogni drone dai commenti "<!-- Drone i spawn: (...) target: (...) -->"
mapfile -t DRONE_LINES < <(grep -oP '(?<=Drone )\d+[^\n]*' "$SDF")
N_FOUND=${#DRONE_LINES[@]}
if [ "$N_FOUND" -ne "$N_DRONES" ]; then
    echo "ERRORE: $SDF contiene $N_FOUND droni, attesi $N_DRONES."
    exit 1
fi

# Numero di ostacoli: argomento esplicito oppure conteggio dal modello "ostacoli"
if [ "$#" -gt 1 ]; then
    N_OBS="$2"
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

# Semi-larghezze vasca dal commento "<!-- Tank: wall_wx=.. wall_wy=.. -->"
WALL_WX=$(grep -oP '(?<=wall_wx=)[0-9.]+' "$SDF" | head -1)
WALL_WY=$(grep -oP '(?<=wall_wy=)[0-9.]+' "$SDF" | head -1)
WALL_WX="${WALL_WX:-30.00}"
WALL_WY="${WALL_WY:-60.00}"

# Vasca alta 20 m con pilastri che arrivano a ~16 m: sopra un ostacolo non resta
# spazio utile, quindi la fuga verticale e' disattivata come nel corridoio.
DEPTH_LIMIT_HIGH=17.0
ALLOW_VERTICAL_ESCAPE=false

declare -a SPAWNS TARGETS
for line in "${DRONE_LINES[@]}"; do
    i=$(echo "$line" | grep -oP '^\d+')
    SPAWNS[$i]=$(echo "$line" | grep -oP '(?<=spawn: \()([^)]+)')
    TARGETS[$i]=$(echo "$line" | grep -oP '(?<=target: \()([^)]+)')
done

echo "Mondo '$WORLD_NAME': $N_DRONES droni, $N_OBS ostacoli, wall_wx=$WALL_WX wall_wy=$WALL_WY"

PIDS=()
for i in $(seq 0 $((N_DRONES - 1))); do
    SX=$(echo "${SPAWNS[$i]}"  | cut -d',' -f1 | tr -d ' ')
    SY=$(echo "${SPAWNS[$i]}"  | cut -d',' -f2 | tr -d ' ')
    SZ=$(echo "${SPAWNS[$i]}"  | cut -d',' -f3 | tr -d ' ')
    TX=$(echo "${TARGETS[$i]}" | cut -d',' -f1 | tr -d ' ')
    TY=$(echo "${TARGETS[$i]}" | cut -d',' -f2 | tr -d ' ')
    TZ=$(echo "${TARGETS[$i]}" | cut -d',' -f3 | tr -d ' ')

    CSV="$HOME/ros2_tethys/docs/risultati_multi${N_DRONES}_drone${i}.csv"
    LOG="/tmp/drone_multi${N_DRONES}_${i}.log"
    echo "  tethys_${i}: spawn=(${SX},${SY},${SZ}) → target=(${TX},${TY},${TZ})   log=${LOG}"

    ros2 run collision_avoidance collision_avoidance_node \
        --ros-args \
        -p use_sim_time:=true \
        -p yaw_bias:=3.14159265 \
        -p drone_name:="tethys_${i}" \
        -p world_name:="${WORLD_NAME}" \
        -p n_obstacles:=${N_OBS} \
        -p csv_path:="${CSV}" \
        -p target_x:="${TX}" \
        -p target_y:="${TY}" \
        -p target_z:="${TZ}" \
        -p spawn_x:="${SX}" \
        -p spawn_y:="${SY}" \
        -p spawn_z:="${SZ}" \
        -p goal_radius:=3.0 \
        -p depth_limit_high:=${DEPTH_LIMIT_HIGH} \
        -p allow_vertical_escape:=${ALLOW_VERTICAL_ESCAPE} \
        -p wall_wx:="${WALL_WX}" \
        -p wall_wy:="${WALL_WY}" \
        > "${LOG}" 2>&1 &
    PIDS+=($!)
done

echo "Attendo che tutti raggiungano il target (Ctrl+C per interrompere)..."
for pid in "${PIDS[@]}"; do
    wait "$pid" 2>/dev/null
done

echo "Missione completata. Risultati in ~/ros2_tethys/docs/risultati_multi${N_DRONES}_drone*.csv"
