# Sequenza del controllo

Il diagramma descrive le responsabilità principali, non una traccia temporale di una specifica esecuzione. Callback e timer sono distinti; le protezioni possono produrre ritorni anticipati e non ogni tick invoca tutti i metodi del planner.

```mermaid
sequenceDiagram
    participant B as Bridge ROS-Gazebo
    participant N as CollisionAvoidanceNode
    participant H as SphHist
    participant M as ObstacleMemory
    participant P as VFHPlanner
    participant D as DepthTrim
    Note over B,N: Callback asincrone rispetto al timer: ordine illustrativo
    B->>N: PointCloud2 dei cinque sonar
    N->>H: sonar_callback: aggiorna raw_[id]
    N->>M: observe: punti in coordinate world
    B->>N: TFMessage della posa
    N->>N: pose_callback: aggiorna state_
    loop Ogni 200 ms sul clock del nodo
        N->>N: control_loop / build_merged
        N->>H: smooth(merged_, sm_)
        alt Dati mancanti, vecchi o warmup
            N->>B: stop_actuators: comandi di arresto
        else Dati utilizzabili
            N->>N: aggiorna metriche e verifica goal
            alt Goal raggiunto
                N->>B: stop_actuators
                N->>N: write_csv e arresto del nodo
            else Navigazione
                N->>P: clearance, pareti e ricerca direzione quando necessaria
                P-->>N: distanze e candidati di fuga
                N->>N: gestisce mode_, waypoint e protezioni
                opt Controllo quota in cmd
                    N->>D: update(error, vz, pitch, speed, enabled, dt)
                    D-->>N: correzione integrale
                end
                opt Protezione retromarcia
                    N->>M: clearance lungo la direzione posteriore
                    M-->>N: distanza disponibile
                end
                N->>B: cmd: spinta e posizione pinne
            end
        end
    end
    Note over N: Timer separato: publish_metrics ogni 5 secondi reali
```

## Percezione e dati

1. `sonar_callback` converte i punti e aggiorna l'istogramma del sensore, i punti osservati e la memoria degli ostacoli.
2. `pose_callback` aggiorna `VehicleState` e le stime cinematiche.
3. `build_merged` ignora i sonar scaduti, somma le densità, prende le distanze minime e prepara i punti orizzontali rispetto alla posa attuale.
4. `SphHist::smooth` costruisce `sm_`, letto dal planner attraverso un riferimento const.

La griglia è 36 × 18 settori, con risoluzione angolare di 10°. Ogni cella conserva densità `h` e distanza minima `min_r`; i punti orizzontali mantengono informazioni geometriche più precise della sola quantizzazione.

## Pianificazione e attuazione

`VFHPlanner` valuta ingombro frontale, clearance dei corridoi, pareti, archi dei waypoint e vie di fuga. `CollisionAvoidanceNode` decide quale ramo usare, mantiene waypoint e stato e costruisce i comandi. `cmd` applica ulteriori protezioni, controllo quota, limiti e filtri prima di pubblicare.

`DepthTrim` conserva una correzione integrale limitata: integra soltanto se abilitato, velocità superiore a 0,4 m/s e intervallo valido, con anti-windup. `ObstacleMemory` conserva osservazioni per 30 s e uno snapshot durante il recupero; quest'ultimo viene liberato quando termina il recupero. Le funzioni libere di `motion_safety.hpp` supportano arresto e retromarcia: non sono classi.

La fuga verticale esiste nel codice ma è disabilitata nei sei scenari finali. Non va dedotta la sua attivazione dalla sola presenza del metodo `find_vertical_escape`.

Fonti: [sensori](../../progetto/ros2_tethys/src/collision_avoidance/src/sensors.cpp), [controllo](../../progetto/ros2_tethys/src/collision_avoidance/src/collision_avoidance_node.cpp), [attuazione](../../progetto/ros2_tethys/src/collision_avoidance/src/motion_control.cpp), [planner](../../progetto/ros2_tethys/src/collision_avoidance/src/vfh_planner.cpp). [Mermaid modificabile](sequenza.mmd).
