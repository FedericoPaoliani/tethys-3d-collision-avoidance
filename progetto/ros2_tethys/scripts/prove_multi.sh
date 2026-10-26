#!/bin/bash
# Campagna di missioni multi-drone con l'harness ripetibile review_multi.py.
# Sequenziali per forza: il gpu_lidar è condiviso e due istanze si strozzano a vicenda.
# Uso: ./scripts/prove_multi.sh [n_droni 3|5] [primo_run] [n_run] [successi_obiettivo] [droni_al_goal]
#   droni_al_goal: quanti droni devono arrivare perche' la prova riesca (default: tutti)
#   successi_obiettivo: se indicato, il driver si ferma appena i run riusciti con
#   numero >= 201 arrivano a quel valore (n_run resta il tetto massimo).
source /opt/ros/humble/setup.bash
source "$HOME/ros2_tethys/install/setup.bash"
set -u
cd "$HOME/ros2_tethys"

N_DRONES="${1:-3}"
FIRST="${2:-201}"
COUNT="${3:-20}"
TARGET="${4:-0}"   # 0 = nessun obiettivo; prova_livello.sh passa SUCCESSI (10)
MIN_GOALS="${5:-$N_DRONES}"
# MONDO (variabile d'ambiente) sceglie un mondo diverso, es. un livello di difficolta':
#   MONDO=mondoMulti5_L2.sdf ./scripts/prove_multi.sh 5 201 300 20 5
# La cartella dei risultati prende il nome del mondo (come in review_multi.py).
WORLD="${MONDO:-mondoMulti${N_DRONES}.sdf}"
DIR="reports/multi_review/$(basename "$WORLD" .sdf)"
PROGRESS="$DIR/progress.log"
mkdir -p "$DIR"

echo "criterio: $MIN_GOALS/$N_DRONES droni al goal  mondo: $WORLD $(sha256sum "$WORLD" | cut -c1-12)  binario: $(sha256sum install/collision_avoidance/lib/collision_avoidance/collision_avoidance_node | cut -c1-12)" | tee -a "$PROGRESS"

successi() {
    python3 - "$DIR" <<'PY'
import json, glob, sys
print(sum(json.load(open(f))['success'] for f in glob.glob(sys.argv[1] + '/run_[2-9][0-9][0-9]/summary.json')
          + glob.glob(sys.argv[1] + '/run_[0-9][0-9][0-9][0-9]/summary.json')))
PY
}

for i in $(seq "$FIRST" $((FIRST + COUNT - 1))); do
    if [ "$TARGET" -gt 0 ] && [ "$(successi)" -ge "$TARGET" ]; then
        echo "$(date +%H:%M:%S)  raggiunti $TARGET successi: fine" | tee -a "$PROGRESS"
        break
    fi
    # Run gia' eseguito: review_multi.py rifiuterebbe la cartella esistente, ma il log del
    # driver (mv qui sotto) sovrascriverebbe quello originale (successo il 3 ott 2026, run 201-297).
    if [ -e "$DIR/run_$(printf '%02d' $i)" ]; then continue; fi
    # Gazebo orfani degradano le prove successive (CLAUDE.md): SIGTERM viene ignorato.
    # Pattern ancorato all'inizio della riga di comando: "ign gazebo" senza ancora
    # colpisce anche le shell che contengono quella stringa, compresa quella che ha
    # lanciato questo driver.
    orfani=$(pgrep -f "^ign gazebo" || true)
    if [ -n "$orfani" ]; then
        echo "$(date +%H:%M:%S)  orfani ign gazebo: $orfani -> SIGKILL" | tee -a "$PROGRESS"
        kill -9 $orfani 2>/dev/null; sleep 2
    fi
    printf '%s  run_%02d  avvio  load=%s\n' "$(date +%H:%M:%S)" "$i" "$(cut -d' ' -f1 /proc/loadavg)" | tee -a "$PROGRESS"
    # Anche lo stdout dell'harness va in RAM: le scritture su ext4 possono restare
    # bloccate per secondi a ogni commit del journal (vedi review_multi.py).
    TMPLOG="/dev/shm/multi${N_DRONES}_run_$(printf '%02d' $i)_driver.log"
    python3 scripts/review_multi.py --run "$i" --world "$WORLD" --label "multi${N_DRONES}_campagna" \
        --min-goals "$MIN_GOALS" --stall-abort-s "${STALLO:-60}" > "$TMPLOG" 2>&1
    mv "$TMPLOG" "$DIR/"
    esito=$(python3 - "$DIR/run_$(printf '%02d' $i)/summary.json" <<'PY'
import json, sys
from pathlib import Path
p = Path(sys.argv[1])
if not p.exists():
    print("summary assente"); raise SystemExit
d = json.load(p.open())
parts = []
for n, v in d['drones'].items():
    t = f"{v['end_sim_t']:.0f}s" if v['end_sim_t'] is not None else '-'
    parts.append(f"{n[-1]}:{v['state']}@{t} min={v['min_goal_dist_m']:.1f}")
sep = d['min_drone_separation']
print(f"{'OK' if d['success'] else 'KO'} goal={d['goals']}/{d['n_drones']} "
      f"urti={d['physical_contact_events']} (fra droni {d['drone_drone_contact_events']}) "
      f"sep_min={sep['dist_3d_m']:.1f}m rt={d['rt_factor'] or 0:.3f} stop={d['stop_reason']} "
      f"buchi_posa={d['input_gaps'].get('pose', {}).get('gaps_over_0_6_s')} | "
      + ' '.join(parts))
PY
)
    printf '%s  run_%02d  %s\n' "$(date +%H:%M:%S)" "$i" "$esito" | tee -a "$PROGRESS"
done
