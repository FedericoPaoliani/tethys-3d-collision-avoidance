#!/bin/bash
# Campagna di missioni a 1 drone con review_corridor.py, con lo stesso protocollo dei run
# 64-268 del Livello 1: nessun limite di tempo pratico (900 s simulati) e prova scartata dopo
# 60 s simulati senza avvicinarsi al goal di almeno 1 m. Si ferma a N successi.
# Sequenziale per forza: il gpu_lidar e' condiviso.
# Uso: ./scripts/prove_corridoio.sh MONDO.sdf [primo_run] [n_run_max] [successi_obiettivo]
source /opt/ros/humble/setup.bash
source "$HOME/ros2_tethys/install/setup.bash"
set -u
cd "$HOME/ros2_tethys"

WORLD="${1:?serve il file del mondo}"
FIRST="${2:-201}"
COUNT="${3:-400}"
TARGET="${4:-10}"
STEM="$(basename "$WORLD" .sdf)"
# Il Livello 1 (mondoCorridoio.sdf) resta nella cartella storica; gli altri livelli hanno la loro.
if [ "$STEM" = "mondoCorridoio" ]; then DIR="reports/corridor_review"; else DIR="reports/corridor_review/$STEM"; fi
PROGRESS="$DIR/progress.log"
mkdir -p "$DIR"

successi() {
    python3 - "$DIR" "$(sha256sum "$WORLD" | cut -d' ' -f1)" <<'PY'
import json, glob, sys
n = 0
for f in glob.glob(sys.argv[1] + '/run_*/summary.json'):
    s = json.load(open(f))
    m = json.load(open(f.replace('summary.json', 'metadata.json')))
    if s['run'] >= 201 and m['world_sha256'] == sys.argv[2] and s['goal']:
        n += 1
print(n)
PY
}

echo "stallo: ${STALLO:-60} s  mondo: $WORLD $(sha256sum "$WORLD" | cut -c1-12)  binario: $(sha256sum install/collision_avoidance/lib/collision_avoidance/collision_avoidance_node | cut -c1-12)  obiettivo: $TARGET successi" | tee -a "$PROGRESS"

for i in $(seq "$FIRST" $((FIRST + COUNT - 1))); do
    if [ "$(successi)" -ge "$TARGET" ]; then
        echo "$(date +%H:%M:%S)  raggiunti $TARGET successi: fine" | tee -a "$PROGRESS"
        break
    fi
    if [ -e "$DIR/run_$(printf '%02d' $i)" ]; then continue; fi
    orfani=$(pgrep -f "^ign gazebo" || true)
    if [ -n "$orfani" ]; then
        echo "$(date +%H:%M:%S)  orfani ign gazebo: $orfani -> SIGKILL" | tee -a "$PROGRESS"
        kill -9 $orfani 2>/dev/null; sleep 2
    fi
    printf '%s  run_%02d  avvio  load=%s\n' "$(date +%H:%M:%S)" "$i" "$(cut -d' ' -f1 /proc/loadavg)" | tee -a "$PROGRESS"
    TMPLOG="/dev/shm/${STEM}_run_$(printf '%02d' $i)_driver.log"
    python3 scripts/review_corridor.py --run "$i" --world "$WORLD" --out-root "$DIR" \
        --duration 900 --stall-abort-s "${STALLO:-60}" --label "${STEM}_livello" > "$TMPLOG" 2>&1
    mv "$TMPLOG" "$DIR/"
    esito=$(python3 - "$DIR/run_$(printf '%02d' $i)/summary.json" <<'PY'
import json, sys
from pathlib import Path
p = Path(sys.argv[1])
if not p.exists():
    print("summary assente"); raise SystemExit
d = json.load(p.open())
print(f"{'OK' if d['goal'] else 'KO'} urti={d['physical_contact_events']} t_sim={d['measured_sim_s']:.0f}s "
      f"dist={d['distance_m']:.1f}m min_goal={d['min_goal_dist_m']:.1f}m rt={d['rt_factor'] or 0:.3f} stop={d['stop_reason']}")
PY
)
    printf '%s  run_%02d  %s\n' "$(date +%H:%M:%S)" "$i" "$esito" | tee -a "$PROGRESS"
done
