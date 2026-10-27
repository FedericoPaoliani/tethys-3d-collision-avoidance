#!/bin/bash
# Un tentativo con la configurazione finale, registrato dall'inizio alla fine dalla telecamera
# simulata del mondo video (stesso metodo dei video precedenti).
# Uso: esegui_tentativo.sh {1drone|5droni} MONDO_SORGENTE.sdf N_RUN [HZ=10] [DURATA_SIM=900]
# - mondo: copia del definitivo + modello telecamera_video senza geometria (genera_mondo_video.py);
# - harness delle campagne (review_corridor.py / review_multi.py): Gazebo headless, bridge,
#   nodo/i collision_avoidance, telemetria, stallo 60 s, criterio di successo del progetto;
# - iscritto al topic della telecamera nella stessa IGN_PARTITION (in Fortress la camera viene
#   resa solo se il topic ha un iscritto); frame PNG in /dev/shm/video_successi/<scenario>_runNN.
# Output harness: log/<scenario>/run_NN.
set -e
MODO="$1"; SORG="$2"; RUN="$3"; HZ="${4:-10}"; DURATA="${5:-900}"
V="$HOME/video_successi_livello1"
STEM="succ_$(basename "$SORG" .sdf)"
RR=$(printf '%02d' "$RUN")
FRAME="/dev/shm/video_successi/${STEM}_run${RR}"
MONDO="$V/log/mondi/${STEM}.sdf"
[ -e "$FRAME" ] && { echo "esiste già $FRAME"; exit 1; }
mkdir -p "$FRAME/frame" "$V/log/mondi"
python3 "$V/script/genera_mondo_video.py" "$MODO" "$SORG" "$MONDO" "$FRAME/frame" "$HZ" >/dev/null
source /opt/ros/humble/setup.bash
source "$HOME/ros2_tethys/install/setup.bash"
cd "$HOME/ros2_tethys"
if [ "$MODO" = 1drone ]; then
    python3 scripts/review_corridor.py --run "$RUN" --world "$MONDO" --out-root "$V/log/$STEM" \
        --duration "$DURATA" --stall-abort-s 60 --wall-limit 9000 --label "$STEM" &
    HARNESS=$!; PART="codex_corridor_${HARNESS}"
else
    python3 scripts/review_multi.py --run "$RUN" --world "$MONDO" --label "$STEM" \
        --duration "$DURATA" --stall-abort-s 60 --min-goals 5 --wall-limit 9000 &
    HARNESS=$!; PART="multi_review_${HARNESS}"
fi
sleep 3
IGN_PARTITION="$PART" IGN_IP=127.0.0.1 \
    "$V/script/abbonato/build/abbonato_telecamere" "$FRAME/tempi_frame.csv" /video/camera_video &
ABB=$!
RC=0; wait "$HARNESS" || RC=$?
kill "$ABB" 2>/dev/null || true
wait "$ABB" 2>/dev/null || true
if [ "$MODO" != 1drone ]; then
    # review_multi.py copia l'output in ~/ros2_tethys/reports/multi_review/<mondo>/: lo spostiamo
    mkdir -p "$V/log/$STEM"
    mv "$HOME/ros2_tethys/reports/multi_review/$STEM/run_$RR" "$V/log/$STEM/"
    rmdir "$HOME/ros2_tethys/reports/multi_review/$STEM" 2>/dev/null || true
fi
echo "FINE rc=$RC frame=$FRAME"
