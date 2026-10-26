# Da verificare: dipendenze, percorsi assoluti, dubbi, limiti di riproducibilità

**Nessuna simulazione e nessuna compilazione è stata eseguita per preparare questa raccolta.**
Non è quindi verificato che il progetto, così copiato, compili o giri su un'altra macchina.

## 1. Stato dei comandi

| Comando | Stato |
|---|---|
| `colcon build --packages-select collision_avoidance` | **verificato dai log** sul PC originale: build riuscita l'8 ottobre 2026 (`log/build_2026-10-08_01-57-14`, non copiato) |
| `colcon test --packages-select collision_avoidance` (`vfh_planner_regression`) | **verificato dai log**: 1 test su 1 superato l'8 ottobre 2026 |
| `colcon build` di `lrauv_description` | ultima build riuscita il 12 e 20 maggio 2026 (con tutti i modelli); **non provata con la sola cartella `models/tethys` di questa raccolta** |
| `./scripts/prova_livello.sh S L`, `review_corridor.py`, `review_multi.py` | **usati davvero**: hanno prodotto i dati delle serie (`progress.log`, `metadata.json`). Non rieseguiti qui |
| Avvio manuale con GUI (`start_sim_*`, `ponte_*`, `cervello_*`) | ricavato da script e documentazione |
| `livelli.py genera/verifica`, `foglio_risultati.py` | ricavati dai file; il foglio è stato rigenerato l'ultima volta il 7 ottobre 2026 |

## 2. Percorsi assoluti

I file **non sono stati modificati**: cambiare un mondo ne cambia lo SHA-256, che gli script usano
per riconoscere i run di ogni livello.

| File | Riferimento | Conseguenza |
|---|---|---|
| `worlds/*.sdf` (tutti e 6) | texture `/home/federico/ros2_tethys/src/lrauv_description/models/tethys/materials/textures/Tethys_*.png` | su un'altra macchina il drone appare senza texture finché il workspace non è in `/home/federico/ros2_tethys` o il percorso non viene corretto. I sonar e la fisica non usano le texture (dedotto, non provato) |
| `scripts/*.sh` | `$HOME/ros2_tethys/...`, `~/ros2_tethys/install/setup.bash`, `/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins` | il workspace deve stare in `~/ros2_tethys` |
| `scripts/review_corridor.py`, `review_multi.py` | sostituiscono come testo `~/ros2_tethys/worlds/<mondo>.sdf` dentro gli script headless | idem |
| `scripts/foglio_risultati.py`, `scripts/blocchi.py` | `/home/federico/ros2_tethys/reports` scritto per esteso | da correggere se cambia utente o percorso |
| `scripts/start_sim_*.sh` | `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` include `install/lrauv_gazebo_plugins/lib` | il pacchetto non è incluso: la cartella mancante è innocua perché i mondi non ne usano i plugin (dedotto) |
| `presentazione_tesi/mondi/mondo_demo_video.sdf`, `prove/run_07/*.sdf` | plugin `/home/federico/ros2_tethys/presentazione_tesi/strumenti/telecamera_plugin/build/libSeguiDroneCamera.so` | serve compilare `strumenti/telecamera_plugin` con CMake prima di rigenerare il video |
| `reports/**/metadata.json`, log di `run_07` | percorsi del PC originale e di `/dev/shm` | registrazioni storiche, da lasciare così |

## 3. Limiti di riproducibilità

- **Binario vs sorgente.** Tutti i run finali usano il binario `fee8566e2489`
  (`progetto/ros2_tethys/binari/`, ELF x86-64), compilato dal sorgente monolitico
  `c7fd221cdb10` (`progetto/sorgente_campagne_c7fd221cdb10/`). Il pacchetto in `src/` è la
  versione dell'8 ottobre, divisa in più file senza cambiare il codice: secondo le note tecniche originali (non pubblicate integralmente) è
  equivalente (stesso assembly `-O0` per le 660 funzioni della classe), ma produce un binario con
  hash diverso (`26a72beaf1a4`). Header, `vfh_planner.cpp` e `package.xml` sono identici byte per
  byte a quelli delle campagne (verificato). Per ricompilare esattamente la versione delle campagne:
  sostituire `src/collision_avoidance_node.cpp` e `CMakeLists.txt` con quelli di
  `sorgente_campagne_c7fd221cdb10/`: il `CMakeLists.txt` delle campagne compila solo
  `collision_avoidance_node.cpp` (che contiene anche `main`) e `vfh_planner.cpp`, quindi
  `sensors.cpp`, `navigation.cpp`, `motion_control.cpp`, `metrics.cpp`, `main.cpp` vengono
  ignorati (verificato confrontando i due `CMakeLists.txt`; compilazione non provata).
- **Risultati non deterministici.** Lo stesso run ripetuto dà esiti diversi (il `gpu_lidar` dipende
  dai tempi di rendering) e il tasso di successo dipende dal carico della macchina (`rt_factor`).
  Una nuova campagna non riprodurrà gli stessi numeri, ma solo la stessa procedura.
- **Foglio dei risultati.** `reports/` contiene solo `summary.json` e `metadata.json` (più
  `progress.log` e `.cache_blocchi.json`). Bastano per tentativi, successi, tempi e distanze. La
  colonna "Collisioni o blocchi davanti a un ostacolo" viene da `scripts/blocchi.py`, che legge
  `telemetry.csv` di ogni run oppure la cache. La chiave della cache contiene il percorso assoluto
  del run e la data di modifica di `summary.json` al nanosecondo, che exFAT/NTFS non conservano.
  Quindi **`foglio_risultati.py` così com'è fallirà** cercando `telemetry.csv`. Per rigenerarlo
  servono i run completi, che restano su `~/ros2_tethys/reports/` (25 GB) e nella raccolta
  completa `/home/federico/Tethys_da_trasferire`. Il foglio finale è comunque copiato in
  `risultati_finali/RISULTATI_TEST.md`.
- **Grafici.** I 6 PNG in `risultati_finali/grafici/` (1–6 ottobre) non hanno uno script
  generatore sul disco: sono stati prodotti con comandi una tantum. Non è noto se e quali siano
  usati nella tesi o nelle slide.
- **Pacchetto `lrauv_description` ridotto.** Contiene solo `models/tethys` con i file richiamati.
  `model.sdf` è la versione modificata localmente l'11 settembre; `model.sdf.in` è quella upstream,
  da cui CMake genera un `model.sdf` diverso nella cartella di build (non usato dagli script, che
  leggono `src/.../models/tethys/model.sdf`).

## 4. Materiale non trovato

- **Tesi definitiva in PDF**: non trovata (solo frontespizio e modelli Sapienza sulla Scrivania).
- **Relazione finale del tirocinio**: non trovata (solo i report tecnici di luglio in
  `~/ros2_tethys/docs/`, esclusi perché non finali).
- **Presentazione finale** (`.pptx`/`.pdf`): non trovata; c'è solo il modello Sapienza vuoto.
  Il materiale per le slide incluso è il video e i 4 fotogrammi.

## 5. Componenti di terzi e licenze

- `src/lrauv_description` viene da **osrf/lrauv** (Open Source Robotics Foundation / MBARI,
  intestazioni Apache License 2.0 nei sorgenti del progetto upstream; il file `LICENSE` upstream è stato aggiunto nella copia pubblicata). Il modello 3D del Tethys (`model.config`: Mabel Zhang, Open Robotics) è anche su
  Gazebo Fuel ("MBARI Tethys LRAUV"). Commit di origine upstream **non noto**: confronto con upstream e attribuzioni documentati in `THIRD_PARTY_NOTICES.md`.
- `src/collision_avoidance/package.xml`: descrizione e licenza "TODO", e-mail `federico@todo.todo`.

## 6. Windows e chiavetta

- Nessun collegamento simbolico, nessun nome non valido su Windows (verificato).
- exFAT non conserva i permessi di esecuzione: i 24 file eseguibili su Ubuntu sono elencati in
  `PERMESSI_ESEGUIBILI.txt`, con il comando per ripristinarli. Gli script hanno fine riga LF: non
  convertirli in CRLF (attenzione a `core.autocrlf` di git su Windows).
- I file `COLCON_IGNORE` sono vuoti di proposito: dicono a colcon di non cercare pacchetti in
  `presentazione_tesi/`.

## Nota sulla pubblicazione

Le note conversazionali originali non sono pubblicate. La guida tecnica pubblica è `documentazione/PROGETTO.md`. La verifica del codice e dei risultati qui riportata resta quella della raccolta su Ubuntu, salvo i controlli statici documentati in `documentazione/PUBBLICAZIONE.md`.
