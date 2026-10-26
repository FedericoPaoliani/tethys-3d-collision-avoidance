#!/bin/bash
# Una missione con la configurazione finale, registrata dalla telecamera del mondo video.
# Uso: esegui_video.sh {1drone|5droni} MONDO_SORGENTE.sdf N_RUN HZ [DURATA_SIM=900]
# Avvia l'harness delle campagne (review_corridor.py o review_multi.py: Gazebo headless,
# bridge, nodo/i collision_avoidance, telemetria, stallo 60 s) sul mondo con telecamera e,
# nella stessa IGN_PARTITION, l'iscritto al topic della telecamera (in Fortress una camera
# viene resa solo se il suo topic ha un iscritto). I frame PNG vanno in /dev/shm.
set -e
MODO="$1"; SORG="$2"; RUN="$3"; HZ="$4"; DURATA="${5:-900}"
V="$HOME/video_simulazioni_finali"
STEM="video_$(basename "$SORG" .sdf)"
FRAME="/dev/shm/video_tethys/${STEM}_run${RUN}"
MONDO="$V/mondi_video/${STEM}.sdf"
rm -rf "$FRAME"; mkdir -p "$FRAME/frame"
python3 "$V/script/genera_mondo_video.py" "$MODO" "$SORG" "$MONDO" "$FRAME/frame" "$HZ"
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
wait "$HARNESS" || true
kill "$ABB" 2>/dev/null || true
wait "$ABB" 2>/dev/null || true
echo "FINE: frame in $FRAME"
