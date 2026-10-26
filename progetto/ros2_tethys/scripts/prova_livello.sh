#!/bin/bash
# Esegue la campagna di un livello di difficolta' (configurazione in scripts/livelli.py).
# Uso: ./scripts/prova_livello.sh SCENARIO LIVELLO [primo_run] [n_run_max] [successi_obiettivo]
#   SCENARIO: 1 (1 drone, vasca 30x120x20) oppure 5 (5 droni, vasca 60x120x20)
#   LIVELLO : 1, 2 o 3
# Esempi:
#   ./scripts/prova_livello.sh 1 2        # 1 drone,  35 ostacoli, fino a 10 successi
#   ./scripts/prova_livello.sh 5 3        # 5 droni,  60 ostacoli, fino a 10 successi 5/5
# Il numero di successi di default e' SUCCESSI in scripts/livelli.py.
# Criterio di successo: tutti i droni al proprio goal prima dello stallo (60 s simulati senza
# avvicinarsi di 1 m, uguale per tutti i livelli: vedi STALLO in scripts/livelli.py).
set -u
cd "$HOME/ros2_tethys"
SCEN="${1:?scenario 1 o 5}"
LIV="${2:?livello 1, 2 o 3}"
FIRST="${3:-201}"
COUNT="${4:-400}"
TARGET="${5:-$(python3 scripts/livelli.py successi)}"
MONDO="$(python3 scripts/livelli.py mondo "$SCEN" "$LIV")" || exit 1
export STALLO="$(python3 scripts/livelli.py stallo "$SCEN" "$LIV")"
[ -f "$MONDO" ] || { echo "manca $MONDO: eseguire python3 scripts/livelli.py genera"; exit 1; }
echo "Scenario $SCEN, Livello $LIV: $MONDO (stallo $STALLO s)"
if [ "$SCEN" = "1" ]; then
    exec ./scripts/prove_corridoio.sh "$MONDO" "$FIRST" "$COUNT" "$TARGET"
else
    MONDO="$MONDO" exec ./scripts/prove_multi.sh "$SCEN" "$FIRST" "$COUNT" "$TARGET" "$SCEN"
fi
