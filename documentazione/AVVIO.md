# Compilazione e avvio su Ubuntu

Questi comandi sono ricavati dagli script archiviati. La pubblicazione è stata preparata su Windows: compilazione, test C++ e simulazioni non sono stati rieseguiti.

## Ambiente

Ubuntu 22.04, ROS 2 Humble, Ignition Gazebo Fortress e strumenti colcon. Le versioni della macchina originale sono in [AMBIENTE.md](../inventario/AMBIENTE.md). I plugin usati dai sei mondi sono quelli di sistema di Fortress.

## Posizione del workspace

Gli script assumono `~/ros2_tethys`. Copiare **il contenuto della cartella `progetto/ros2_tethys`** in un nuovo workspace con quel nome, senza sovrascrivere un workspace già esistente. Conservare la copia archiviata del repository separata dalle nuove prove, perché gli harness scrivono in `reports/`.

I mondi contengono inoltre percorsi delle texture sotto `/home/federico/ros2_tethys`. Con un utente diverso, predisporre i percorsi corretti in una copia dei mondi destinata alle nuove prove. **Qualsiasi modifica cambia lo SHA-256**: non confrontare automaticamente quei nuovi run con gli hash delle campagne archiviate.

## Compilazione

Dopo aver installato le dipendenze indicate nell'ambiente:

```bash
cd ~/ros2_tethys
source /opt/ros/humble/setup.bash
colcon build --packages-select collision_avoidance lrauv_description
source install/setup.bash
colcon test --packages-select collision_avoidance
colcon test-result --verbose
```

Il pacchetto `lrauv_description` usa `colcon.pkg` e CMake. La raccolta contiene solo il modello Tethys: la compilazione di questa selezione ridotta è ancora da verificare. Gli script di avvio usano anche le risorse in `src/lrauv_description/models`.

Git conserva il bit eseguibile degli script. Se si usa uno ZIP o una chiavetta exFAT, dalla radice della raccolta si possono ripristinare i permessi indicati in [PERMESSI_ESEGUIBILI.txt](../inventario/PERMESSI_ESEGUIBILI.txt). Le texture non richiedono il bit eseguibile.

## Una prova

Dalla radice del workspace, scegliere un numero di run non già presente; `900001` è soltanto un esempio:

```bash
python3 scripts/review_corridor.py --run 900001   --world worlds/mondoCorridoio_L3.sdf --duration 900 --stall-abort-s 60

python3 scripts/review_multi.py --run 900001   --world mondoMulti5_L3.sdf --min-goals 5 --stall-abort-s 60
```

Per gli altri livelli cambiare il nome del mondo secondo la [tabella degli scenari](../inventario/SEI_SCENARI.md).

## Una campagna

```bash
# scenario, livello, primo run, massimo tentativi, successi obiettivo
./scripts/prova_livello.sh 1 1 900001 400 10
./scripts/prova_livello.sh 5 3 900001 1200 10
```

Il driver ha un limite massimo di tentativi: può terminare prima di raggiungere dieci successi. In particolare, il default di 400 non basta a riprodurre una serie lunga quanto quella storica del livello 3 con cinque droni (1018 tentativi). Non avviare una nuova serie sugli identificativi dei dati archiviati.

Sul PC originale è stato osservato un problema di rendering Ogre2/GLX in assenza di monitor. È una limitazione di quell'ambiente, non una proprietà generale di Gazebo.

## Versione esatta delle campagne

Il sorgente monolitico e il CMakeLists delle campagne sono in `progetto/sorgente_campagne_c7fd221cdb10/`. Per confrontarlo con la versione suddivisa, usare una copia separata del workspace. Il binario Linux x86-64 archiviato è mantenuto per la provenienza degli esperimenti; non è un eseguibile Windows né un binario portabile garantito.
