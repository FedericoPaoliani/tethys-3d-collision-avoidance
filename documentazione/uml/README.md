# UML del codice di collision avoidance

Diagramma ricavato dagli header e dal `CMakeLists.txt` del pacchetto `collision_avoidance`, nel sorgente presente al commit `54a9355` (documentazione aggiornata il 27 ottobre 2026). Questa aggiunta modifica soltanto la documentazione, non il controllo o le simulazioni.

## Percorso di lettura

1. [Architettura e interfacce ROS 2](architettura.md): componenti, topic, tipi e tempi.
2. Diagramma delle classi qui sotto: proprietà dei dati e responsabilità.
3. [Sequenza del controllo](sequenza.md): ricezione, fusione, pianificazione e attuazione.
4. [Stati e transizioni](stati.md): sintesi e indice di tutte le assegnazioni a `mode_`.
5. [Header completi con riferimenti](riferimenti.md): dichiarazioni effettive per approfondire attributi e firme.

I diagrammi delle classi, di sequenza e degli stati usano notazione UML resa con Mermaid. L'architettura è uno schema funzionale complementare. I diagrammi comportamentali sintetizzano il codice: non sostituiscono le condizioni complete dei numerosi rami di sicurezza. Nessuna modifica al codice o ai parametri delle simulazioni.

## Quale versione è pubblicata

Il codice principale è **già suddiviso in più file**. Sono inoltre presenti classi e strutture distinte per pianificazione, istogramma, stato, memoria degli ostacoli e correzione di quota.

La suddivisione in file non corrisponde però sempre a una classe per file: `sensors.cpp`, `navigation.cpp`, `motion_control.cpp` e `metrics.cpp` definiscono metodi di **CollisionAvoidanceNode**, dichiarati nello stesso header. Non esistono classi chiamate `Sensors`, `Navigation`, `MotionControl` o `Metrics` nella versione esaminata.

La copia monolitica in `progetto/sorgente_campagne_c7fd221cdb10/` è un archivio della versione delle campagne; non è quella selezionata dal `CMakeLists.txt` del pacchetto principale.

## Diagramma delle classi

Il diagramma mostra le responsabilità principali. Le firme sono abbreviate: tipi ROS, qualificatori, riferimenti dei parametri, valori predefiniti e numerosi membri di controllo sono omessi per leggibilità. Il codice rimane la fonte delle firme complete. In `SphHist`, gli indici, le trasformazioni e `smooth` sono funzioni statiche.

```mermaid
classDiagram
    direction TB
    class ROSNode["rclcpp::Node"]
    class CollisionAvoidanceNode {
        +CollisionAvoidanceNode()
        -raw_ : SphHist[5]
        -merged_ : SphHist
        -sm_ : SphHist
        -state_ : VehicleState
        -planner_ : VFHPlanner
        -obstacle_memory_ : ObstacleMemory
        -depth_trim_ : DepthTrim
        -mode_ : Mode
        -local_wp_active_ : bool
        -vertical_escape_ : bool
        -goal_active_ : bool
        -goal_reached_ : bool
        -stabilize_active_ : bool
        -alt_recovery_active_ : bool
        -circum_active_ : bool
        -control_loop()
        -sonar_callback(msg, id)
        -pose_callback(msg)
        -build_merged()
        -preferred_dir() double
        -create_local_waypoint(yaw_world, clearance)
        -cmd(thr, az, el)
        -stop_actuators()
        -create_local_waypoint_throttled(yaw_world, clearance, t)
        -steer_limit(fwd_min) double
        -lookahead_preferred_dir(fwd_min) double
        -publish_metrics()
        -write_csv()
    }
    class VFHPlanner {
        -sm_ : const SphHist reference
        -state_ : const VehicleState reference
        -wall_wx_ : double
        -wall_wy_ : double
        -inner_wx_ : double
        -inner_wy_ : double
        -compute_az_min(az_min)
        +set_walls(wx, wy)
        +set_inner_walls(wx, wy)
        +inner_wall_dist_in_dir(world_yaw) double
        +wall_dist_in_dir(world_yaw) double
        +wall_fwd_dist() double
        +local_waypoint_length(bearing, clearance) double
        +forward_blocked() bool
        +forward_min_range() double
        +corridor_clearance(candidate_az) double
        +find_escape(best_az, preferred_az) bool
        +find_lookahead_escape(best_az, preferred_az, path_len_out) bool
        +find_vertical_escape(new_depth, el_span_out, range_thresh) bool
        +waypoint_arc_free(bearing, length) bool
    }
    class SphHist {
        <<struct>>
        +h : double grid 36x18
        +min_r : double grid 36x18
        +horizontal_points : vector
        +last_time : double
        +received : bool
        +horizontal_points_valid : bool
        +az_center(k) double
        +el_center(l) double
        +reset()
        +az_idx(az) int
        +el_idx(el) int
        +smooth(raw, out)
        +to_body(xs, ys, zs, id, xb, yb, zb)
    }
    class VehicleState {
        <<struct>>
        +x : double
        +y : double
        +z : double
        +pitch : double
        +roll : double
        +yaw : double
        +yaw_rate : double
        +vz : double
    }
    class ObstacleMemory {
        -points_ : deque of Point
        -recovery_points_ : deque of Point
        -recovery_active_ : bool
        +observe(x, y, z, t)
        +begin_recovery(t)
        +end_recovery()
        +clearance(x, y, z, yaw, t) double
    }
    class DepthTrim {
        -integral_ : double
        +update(error, vz, pitch, speed, enabled, dt) double
    }
    class Mode {
        <<enumeration>>
        CRUISE
        ESCAPE
        BRAKE
    }
    ROSNode <|-- CollisionAvoidanceNode
    CollisionAvoidanceNode "1" *-- "1" VFHPlanner : planner_
    CollisionAvoidanceNode "1" *-- "7" SphHist : raw_ x5 + merged_ + sm_
    CollisionAvoidanceNode "1" *-- "1" VehicleState : state_
    CollisionAvoidanceNode "1" *-- "1" ObstacleMemory : obstacle_memory_
    CollisionAvoidanceNode "1" *-- "1" DepthTrim : depth_trim_
    CollisionAvoidanceNode ..> Mode : mode_ - enum annidato
    VFHPlanner --> "1" SphHist : sm_ - riferimento const
    VFHPlanner --> "1" VehicleState : state_ - riferimento const
    class MemoryPoint["ObstacleMemory::Point"] {
        <<struct>>
        +x : double
        +y : double
        +z : double
        +t : double
    }
    class SensorId {
        <<enumeration>>
        FRONT
        LEFT
        RIGHT
        BOTTOM
        TOP
        N_SENSORS
    }
    ObstacleMemory "1" *-- "0..*" MemoryPoint : points_ e recovery_points_
    CollisionAvoidanceNode ..> SensorId : selezione sonar
```

### Come leggere le relazioni

- Triangolo verso `rclcpp::Node`: ereditarietà; `CollisionAvoidanceNode` è un nodo ROS 2.
- Rombo pieno: composizione; il nodo possiede quei membri per valore.
- Freccia semplice dal planner: riferimenti non proprietari e `const`; il planner legge `sm_` e `state_`, posseduti dal nodo.
- I sette `SphHist` sono cinque istogrammi dei sensori, uno fuso e uno smussato. Non sono sette classi diverse.
- `Mode` è un enum privato annidato in `CollisionAvoidanceNode`, non una classe di controllo autonoma.

`ObstacleMemory` contiene anche il tipo privato annidato `Point` con `x`, `y`, `z`, `t`, usato dalle due code. È rappresentato nel disegno come tipo annidato privato; i suoi campi sono pubblici perché dichiarato con `struct`.

## Suddivisione effettiva nei file

Tutti i percorsi della tabella sono relativi a [collision_avoidance](../../progetto/ros2_tethys/src/collision_avoidance/).

| File | Tipo o responsabilità |
|---|---|
| `src/main.cpp` | Inizializza ROS 2, crea il nodo e avvia `rclcpp::spin` |
| `include/collision_avoidance/collision_avoidance_node.hpp` | Dichiarazione di `CollisionAvoidanceNode`, membri ROS e stato del controllo |
| `src/collision_avoidance_node.cpp` | Costruttore e ciclo di controllo della stessa classe |
| `src/sensors.cpp` | Callback sonar e posa, fusione degli istogrammi della stessa classe |
| `src/navigation.cpp` | Direzioni preferite e waypoint della stessa classe |
| `src/motion_control.cpp` | Comandi al propulsore e alle pinne della stessa classe |
| `src/metrics.cpp` | Pubblicazione delle metriche e scrittura CSV della stessa classe |
| `include/collision_avoidance/vfh_planner.hpp`, `src/vfh_planner.cpp` | Classe separata `VFHPlanner` |
| `include/collision_avoidance/sph_hist.hpp` | Struct `SphHist` e funzioni per l'istogramma |
| `include/collision_avoidance/vehicle_state.hpp` | Struct `VehicleState` |
| `include/collision_avoidance/obstacle_memory.hpp` | Classe `ObstacleMemory` |
| `include/collision_avoidance/depth_controller.hpp` | Classe `DepthTrim` |
| `include/collision_avoidance/motion_safety.hpp` | Funzioni libere inline, non una classe |

Le funzioni libere di `motion_safety.hpp` sono `rear_stopping_distance`, `recovery_reverse_can_resume` e `recovery_thrust`. Non è stata inventata una classe UML per rappresentarle.

## Collegamento con le sei simulazioni

La struttura C++ è comune ai sei scenari. Nel caso multi-drone gli script avviano un nodo per veicolo; cambiano istanza, topic e parametri, non la struttura delle classi. I mondi SDF definiscono ambiente, sensori e dinamica e non sono classi C++ del nodo.

La funzione `find_vertical_escape` esiste nel planner, ma nei sei scenari finali l'opzione `allow_vertical_escape` è disabilitata. Il diagramma descrive il codice disponibile, non afferma che ogni funzionalità sia attiva in ogni prova.

## Sorgenti del diagramma

- [Mermaid modificabile](classi.mmd)
- [Header del nodo](../../progetto/ros2_tethys/src/collision_avoidance/include/collision_avoidance/collision_avoidance_node.hpp)
- [Header del planner](../../progetto/ros2_tethys/src/collision_avoidance/include/collision_avoidance/vfh_planner.hpp)
- [CMakeLists del pacchetto principale](../../progetto/ros2_tethys/src/collision_avoidance/CMakeLists.txt)

GitHub visualizza il blocco Mermaid direttamente in questa pagina.
