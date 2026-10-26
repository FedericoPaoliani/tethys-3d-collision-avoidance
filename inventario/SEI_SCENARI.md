# I sei scenari finali

Percorsi relativi a `progetto/ros2_tethys/`. Ogni scenario è un **file di mondo distinto**: tutti
sono stati generati una volta da `scripts/livelli.py` (che chiama `scripts/generate_world.py`)
e poi usati così come sono. Lo SHA-256 di ogni mondo copiato coincide con il `world_sha256`
registrato nel `metadata.json` di **tutti** i run della serie corrispondente (controllato il
26 ottobre 2026 su 2039 run). Tutti i run finali usano il binario `fee8566e2489`.

| Scenario | Mondo (SHA-256) | Ostacoli | Generazione (`scripts/livelli.py`) | Run della serie (dati in `reports/`) |
|---|---|---|---|---|
| 1 drone, Livello 1 | `worlds/mondoCorridoio.sdf` (`c20af7c59aa0`) | 30 (24 casuali + 6 di percorso) | seed 219401, raggio ×1,0 | `corridor_review/run_*`: 139 run con questo mondo; il foglio conta fino al 10° successo (62 tentativi) |
| 1 drone, Livello 2 | `worlds/mondoCorridoio_L2.sdf` (`68b6dd96b887`) | 35 (28 + 7) | seed 219430, raggio ×0,7 | `corridor_review/mondoCorridoio_L2/run_*`: 170 run |
| 1 drone, Livello 3 | `worlds/mondoCorridoio_L3.sdf` (`6cf683405427`) | 40 (32 + 8) | seed 219576, raggio ×0,85 con le posizioni generate a ×0,7 (`POSIZIONI_DA`) | `corridor_review/mondoCorridoio_L3/run_*`: 215 run |
| 5 droni, Livello 1 | `worlds/mondoMulti5.sdf` (`4ad20a45d2cc`) | 40 (26 + 14) | seed 282075, raggio ×1,0 | `multi_review/mondoMulti5/run_*`: 329 run; il foglio conta fino al 10° successo (213 tentativi) |
| 5 droni, Livello 2 | `worlds/mondoMulti5_L2.sdf` (`b4a3ec1463db`) | 50 (33 + 17) | seed 282093, raggio ×1,0 | `multi_review/mondoMulti5_L2/run_*`: 168 run |
| 5 droni, Livello 3 | `worlds/mondoMulti5_L3.sdf` (`29431aedc0a9`) | 60 (39 + 21) | seed 282175, raggio ×0,9; droni spostati (`DRONI`: x = −24/−12/0/12/24, z = 7/9,5 alternati, `sposta_droni.py`); ostacoli distanziati (`DISTANZIA`: centri ≥ 9 m, pareti ≥ 3 m, `distanzia_ostacoli.py`) | `multi_review/mondoMulti5_L3/run_*`: 1018 run |

Vasche: 1 drone 30×120×20 m, spawn (0, −50, 8) → goal (0, 50, 8); 5 droni 60×120×20 m, corsie
x = −20/−10/0/10/20 (L3: vedi sopra), da y = −50 a y = +50. Spawn, goal e semi-larghezze della vasca
sono scritti nei commenti del file SDF e letti da lì dagli script.

## Script, parametri e comandi

Non esistono file launch né file YAML di parametri del nodo: tutto passa dagli script.

| Elemento | 1 drone | 5 droni |
|---|---|---|
| Harness di un run (Gazebo headless + bridge + nodo + telemetria) | `scripts/review_corridor.py` (modello di avvio: `scripts/start_sim_corridoio_headless.sh`) | `scripts/review_multi.py` (modello: `scripts/start_sim_multi5_headless.sh`) |
| Driver della campagna | `scripts/prove_corridoio.sh` | `scripts/prove_multi.sh` |
| Bridge ROS 2–Gazebo | `scripts/ponte_single_config.yaml` (via `ponte_single.sh` o l'harness) | YAML generato da `scripts/ponte_multi_lanes.sh` a partire da `ponte_single_config.yaml` |
| Parametri del nodo | scritti nell'harness e in `scripts/cervello_corridoio.sh`: `use_sim_time:=true`, `yaw_bias:=3.14159265`, `goal_radius:=3.0`, `depth_limit_high:=17.0`, `allow_vertical_escape:=false`, `wall_wx/wall_wy` dal commento `Tank:` dell'SDF, spawn/target dall'SDF | uguali, per ogni `tethys_0…4` (`review_multi.py` righe ~399–410, `cervello_multi_lanes.sh`) |
| Parametri della campagna | `scripts/livelli.py`: mondo, `SUCCESSI = 10`, stallo 60 s | idem |
| Costanti dell'algoritmo | `src/collision_avoidance/include/collision_avoidance/vfh_planner.hpp` e `collision_avoidance_node.hpp` | idem |

### Comandi di avvio (ricavati dai file, NON eseguiti in questa raccolta)

Dalla radice del workspace (`~/ros2_tethys`), dopo la compilazione:

| Scenario | Campagna completa (fino al 10° successo) | Un singolo run |
|---|---|---|
| 1 drone L1 | `./scripts/prova_livello.sh 1 1` | `python3 scripts/review_corridor.py --run N --world worlds/mondoCorridoio.sdf --duration 900 --stall-abort-s 60` |
| 1 drone L2 | `./scripts/prova_livello.sh 1 2` | idem con `worlds/mondoCorridoio_L2.sdf` |
| 1 drone L3 | `./scripts/prova_livello.sh 1 3` | idem con `worlds/mondoCorridoio_L3.sdf` |
| 5 droni L1 | `./scripts/prova_livello.sh 5 1` | `python3 scripts/review_multi.py --run N --world mondoMulti5.sdf --min-goals 5 --stall-abort-s 60` |
| 5 droni L2 | `./scripts/prova_livello.sh 5 2` | idem con `mondoMulti5_L2.sdf` |
| 5 droni L3 | `./scripts/prova_livello.sh 5 3` | idem con `mondoMulti5_L3.sdf` |

`prova_livello.sh SCENARIO LIVELLO [primo_run=201] [n_run_max=400] [successi=10]` legge il mondo da
`livelli.py` e chiama `prove_corridoio.sh` o `prove_multi.sh` con le righe sopra. I run delle serie
L2/L3 e 5 droni L1 sono stati prodotti con questi driver (`progress.log` di ogni serie).
⚠ I run del **1 drone L1** (23–24 set, run 44–99 e 200–268) sono precedenti a `prova_livello.sh`:
furono eseguiti con driver storici (non copiati) e lo stesso protocollo di `review_corridor.py`
(`--duration 900 --stall-abort-s 60`, i run 44–63 con un limite fisso di 300 s).

Avvio manuale con interfaccia grafica (3 terminali):

```bash
MONDO=$HOME/ros2_tethys/worlds/mondoCorridoio_L3.sdf ./scripts/start_sim_corridoio.sh   # Gazebo in pausa: premere ▶
./scripts/ponte_single.sh
MONDO=$HOME/ros2_tethys/worlds/mondoCorridoio_L3.sdf ./scripts/cervello_corridoio.sh
# 5 droni (solo Livello 1: cervello_multi_lanes.sh usa sempre worlds/mondoMulti5.sdf)
./scripts/start_sim_multi5.sh ; ./scripts/ponte_multi5.sh ; ./scripts/cervello_multi5.sh
```

Rigenerare i mondi e controllarli: `python3 scripts/livelli.py elenco | genera | verifica`
(`genera` rigenera L2/L3 e per L1 controlla solo la riproducibilità).
Foglio dei risultati: `python3 scripts/foglio_risultati.py > reports/RISULTATI_TEST.md`.

## Risorse di ogni scenario (verificate sulla copia)

Tutti e sei i mondi richiamano solo:
- `model://tethys/meshes/tethys.dae`, `model://tethys/materials/scripts/`,
  `model://tethys/materials/textures/` → `src/lrauv_description/models/tethys/` (presenti);
- le 4 texture `Tethys_{Albedo,Normal,Metalness,Roughness}.png` con percorso **assoluto**
  `/home/federico/ros2_tethys/src/lrauv_description/models/tethys/materials/textures/` (presenti
  nella copia allo stesso percorso relativo; vedi `DA_VERIFICARE.md`);
- plugin di sistema di Ignition Gazebo Fortress (thruster, joint-position-controller, lift-drag,
  hydrodynamics, buoyancy-engine, buoyancy, contact, physics, sensors, scene-broadcaster,
  user-commands): installati con Gazebo, nessun plugin personalizzato.

Gli ostacoli sono cilindri primitivi dentro il modello `ostacoli` di ciascun SDF: non richiedono
modelli esterni. Il drone è il "flat model" scritto per intero nel mondo (link, giunti, 5 sonar
`gpu_lidar`, 6 sensori di contatto, attuatori), ricavato da `models/tethys/model.sdf`.
