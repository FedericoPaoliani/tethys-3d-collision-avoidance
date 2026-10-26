# Esclusi e motivazioni

Nulla è stato cancellato o modificato sul PC Ubuntu: gli esclusi restano dove sono. Una copia più
ampia (25 GB) si trova in `/home/federico/Tethys_da_trasferire` sul PC; **sulla chiavetta** la
cartella `Tethys_da_trasferire` è una copia **parziale e interrotta** (circa 0,8 GB) di quella
raccolta: non è completa né verificata, va ignorata o cancellata a mano.

| Categoria esclusa | Dove si trova | Motivo |
|---|---|---|
| `build/`, `install/`, `log/` | `~/ros2_tethys/` | rigenerabili con colcon. Controllato il 26 ottobre: contengono solo file generati (moc/qrc di Qt, protobuf, `model.sdf` generati da `model.sdf.in`, binari compilati), nessuna risorsa personalizzata assente dai sorgenti |
| `.git/` | `~/ros2_tethys/.git` (220 MB) | storia del repository; HEAD registrato in `AMBIENTE.md` |
| `.cache/`, `__pycache__/`, `compile_commands.json` (link a `build/`), `.clangd`, `.vscode/` | workspace | cache e configurazioni dell'editor con percorsi locali |
| Run completi delle campagne (telemetria, log, copie di binario/sorgenti/mondo per run) | `~/ros2_tethys/reports/**/run_*` (25 GB, ~111 000 file) | troppo grandi; copiati solo `summary.json` e `metadata.json` dei run delle sei serie, non sufficienti a rigenerare tutte le metriche del foglio. Vedi `DA_VERIFICARE.md` per cosa richiede i dati completi |
| Serie archiviate, interrotte, storiche e test a 3 droni (scartato) | `reports/**/archivio_*`, `interrotti/`, `mondoMulti3_review/`, `mondoMulti5_review/`, `multi_review/scartati_3_droni/`, `reports/TEST_3_DRONI_SCARTATO.md` | non fanno parte dei sei scenari finali (mondi diversi o criteri diversi) |
| Log dei driver (`*_driver.log`) e `driver_campagna*.log` | cartelle delle serie | log grezzi; l'esito di ogni run è in `progress.log` e `summary.json`, copiati |
| `scripts/storico/` | workspace | driver e analisi una tantum delle campagne (cambio obiettivo, pause, code fra livelli, tabelle del L1) |
| `scripts/__pycache__/` | workspace | cache Python |
| Pacchetti `lrauv_gazebo_plugins`, `lrauv_system_tests` | `~/ros2_tethys/src/` | codice upstream (osrf/lrauv) non usato dai sei mondi né dal nodo; scaricabile da https://github.com/osrf/lrauv |
| Modelli `nathan_benderson_park`, `tethys_equipped`, `tethys_0…4`; mesh `base.dae`, `propeller.dae` | `src/lrauv_description/models/` | non richiamati dai mondi né da `generate_world.py` (la funzione che usa `tethys_equipped` e scrive `tethys_N` non viene mai chiamata) |
| Mondi vecchi, `.bak`, varianti abbandonate | storia git, `reports/` | non finali |
| Report e CSV di luglio–settembre | `~/ros2_tethys/docs/` | primi esperimenti su mondi `mondoRandom`, labirinto e singolo: non finali |
| Frame della telecamera (1517 PNG, 77 MB), copia del binario e sorgenti di `prove/run_07`, prove `run_01…06` e `*.out` | `presentazione_tesi/prove/` | materiale intermedio del video; tenuti i dati e i log di `run_07` da cui è stato montato il video finale |
| `presentazione_tesi/strumenti/*/build/` | workspace | build CMake degli strumenti video, rigenerabili |
| Registrazioni dello schermo (16 `.webm`) | `~/Video/Video/` | registrazioni intermedie di settembre, non il video finale |
| Schermate | `~/Immagini/Schermate/` | non identificate come finali |
| Modelli, articoli, tesi di altri autori, modelli Sapienza, zip Fuel | Scrivania, Scaricati | non necessari al progetto finale |
| ROS bag | — | nessuna trovata |
| Credenziali, chiavi, dati personali | — | non cercati fuori dal progetto e non copiati; scansione della copia: 0 segreti |
