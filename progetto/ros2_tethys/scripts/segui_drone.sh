#!/bin/bash
# Attiva la telecamera "Follow" della GUI di Gazebo sul drone indicato e le applica
# un offset fisso, senza dover fare ogni volta tasto destro → Follow.
#
# Uso:  ./segui_drone.sh [nome_drone] [x] [y] [z]
# Va lanciato IN BACKGROUND subito prima di "ign gazebo": aspetta che la GUI esponga
# i servizi /gui/follow e /gui/follow/offset, poi li chiama. Messo dopo "ign gazebo"
# non partirebbe finché la simulazione non viene chiusa.
#
# In Fortress il plugin CameraTracking espone follow e offset solo come servizi:
# non esiste un equivalente dichiarativo nella gui.config, quindi questa attesa
# attiva è l'unica strada. L'offset è nel sistema di riferimento del modello.

DRONE="${1:-tethys}"
OX="${2:-4}"
OY="${3:-0}"
OZ="${4:-0.7}"
GUI_TIMEOUT_S="${SEGUI_GUI_TIMEOUT_S:-90}"    # attesa massima per la GUI
FOLLOW_TRIES="${SEGUI_FOLLOW_TRIES:-40}"      # tentativi di aggancio al modello

# 1. attesa che la GUI esista — con scadenza: senza, se Gazebo non parte (o è
#    headless) questo ciclo resterebbe vivo per sempre come processo orfano.
deadline=$(( $(date +%s) + GUI_TIMEOUT_S ))
until ign service -l 2>/dev/null | grep -q '/gui/follow/offset'; do
    if [ "$(date +%s)" -ge "$deadline" ]; then
        echo "[CAMERA] GUI non disponibile entro ${GUI_TIMEOUT_S}s: Follow non impostato." >&2
        exit 1
    fi
    sleep 0.5
done

# 2. il servizio compare PRIMA che la scena contenga il modello: finché l'entità non
#    esiste il Follow risponde false ("Unable to follow target"). Si riprova invece di
#    affidarsi a una pausa fissa, che su mondi lenti da caricare non basta — il mondo
#    marino ha una mesh da 153 MB.
for i in $(seq 1 "$FOLLOW_TRIES"); do
    reply=$(ign service -s /gui/follow \
                --reqtype ignition.msgs.StringMsg \
                --reptype ignition.msgs.Boolean \
                --timeout 2000 \
                --req "data: \"$DRONE\"" 2>/dev/null)
    case "$reply" in *true*) break ;; esac
    if [ "$i" -eq "$FOLLOW_TRIES" ]; then
        echo "[CAMERA] '$DRONE' non trovato nella scena: Follow non attivato." >&2
        echo "[CAMERA] Controlla il nome del modello nell'SDF (single/corridoio/labirinto/marino: tethys; multi: tethys_0)." >&2
        exit 1
    fi
    sleep 0.5
done

# 3. l'offset si applica solo a Follow già attivo, quindi va sempre dopo.
ign service -s /gui/follow/offset \
    --reqtype ignition.msgs.Vector3d \
    --reptype ignition.msgs.Boolean \
    --timeout 2000 \
    --req "x: $OX, y: $OY, z: $OZ" >/dev/null 2>&1

echo "[CAMERA] Follow su '$DRONE' con offset ($OX, $OY, $OZ)."
