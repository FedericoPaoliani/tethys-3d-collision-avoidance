#!/bin/bash
# Una missione reale nel mondo demo con registrazione dei frame delle telecamere.
# Uso: esegui_prova.sh {anteprima|finale} N_RUN DURATA_SIM [telecamere,separate,da,virgola]
# Avvia scripts/review_corridor.py (Gazebo headless, bridge, nodo collision_avoidance,
# telemetria) e, nella stessa IGN_PARTITION, l'iscritto ai topic delle telecamere.
set -e
MODO="$1"; RUN="$2"; DURATA="$3"; CAMERE="${4:-}"
DEMO="$HOME/ros2_tethys/presentazione_tesi"
FRAME="/dev/shm/tethys_demo/${MODO}_run${RUN}"
MONDO="$DEMO/mondi/demo_${MODO}_run${RUN}.sdf"
rm -rf "$FRAME"; mkdir -p "$FRAME"
python3 "$DEMO/strumenti/genera_mondo_demo.py" "$MODO" "$MONDO" "$FRAME" $CAMERE
TOPICS=$(grep -oP '(?<=<topic>)/presentazione/[^<]+' "$MONDO")
for t in $TOPICS; do mkdir -p "$FRAME/$(basename "$t")"; done
source /opt/ros/humble/setup.bash
source "$HOME/ros2_tethys/install/setup.bash"
cd "$HOME/ros2_tethys"
python3 scripts/review_corridor.py --run "$RUN" --label "demo_video_${MODO}" --world "$MONDO" \
    --out-root "$DEMO/prove" --duration "$DURATA" --wall-limit 2400 &
HARNESS=$!
# review_corridor.py usa IGN_PARTITION=codex_corridor_<pid> e IGN_IP=127.0.0.1
IGN_PARTITION="codex_corridor_${HARNESS}" IGN_IP=127.0.0.1 \
    "$DEMO/strumenti/abbonato/build/abbonato_telecamere" "$FRAME/tempi_frame.csv" $TOPICS &
ABB=$!
wait "$HARNESS" || true
kill "$ABB" 2>/dev/null || true
wait "$ABB" 2>/dev/null || true
echo "FINE: frame in $FRAME"
