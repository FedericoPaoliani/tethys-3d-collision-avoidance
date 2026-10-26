# Funzionamento del sistema

## Flusso dei dati

Sonar simulati → punti nello spazio del veicolo → istogramma sferico → valutazione dei corridoi → waypoint locale → controllo di pinne e propulsore → movimento in Gazebo.

ROS 2 collega il nodo di collision avoidance ai sensori e agli attuatori attraverso il bridge. Gazebo calcola dinamica e osservazioni simulate.

## Percezione e pianificazione

Il sistema usa cinque sonar simulati (`gpu_lidar`), rivolti in avanti, lateralmente e nelle direzioni superiore e inferiore. L'istogramma ha 36 settori di azimuth e 18 di elevazione. Le osservazioni vengono riportate in un riferimento comune e gli ostacoli vicini ricevono maggiore peso.

Il planner locale valuta la percorribilità di corridoi e l'ingresso nelle manovre. I waypoint locali stabilizzano la deviazione rispetto a una scelta istantanea continuamente variabile. La rappresentazione sferica si ispira al VFH, ma la logica completa comprende adattamenti per il veicolo subacqueo.

## Controllo e più veicoli

Il controllo distingue crociera (`CRUISE`), manovra evasiva (`ESCAPE`) e frenata (`BRAKE`), con protezioni per quota, stabilità e retromarcia. I comandi agiscono su pinne e propulsore. Ogni drone usa un'istanza del controllo e tratta gli altri come ostacoli; non c'è un coordinamento esplicito del gruppo.

I sei scenari finali usano `allow_vertical_escape:=false`. Una rappresentazione tridimensionale non implica quindi che ogni manovra verticale sia attiva nelle campagne pubblicate.

## Organizzazione del sorgente

Nel workspace `progetto/ros2_tethys/`:

- `src/collision_avoidance/include/`: dichiarazioni, strutture e parametri del planner e del nodo;
- `src/collision_avoidance/src/vfh_planner.cpp`: planner e ricerca dei corridoi;
- `src/collision_avoidance/src/`: nodo, sensori, navigazione, controllo del movimento e metriche;
- `src/collision_avoidance/test/vfh_planner_test.cpp`: regressioni del planner;
- `scripts/`: avvio, bridge, generazione dei mondi, conduzione delle prove e analisi;
- `worlds/`: sei mondi finali;
- `src/lrauv_description/`: risorse del modello Tethys;
- `reports/`: riepiloghi e metadati selezionati dei run;
- `presentazione_tesi/`: scenario e strumenti dedicati al video.

## Versioni delle campagne

Il binario archiviato `fee8566e2489` è associato, nei metadati, al sorgente monolitico `c7fd221cdb10`, conservato in `progetto/sorgente_campagne_c7fd221cdb10/`. Il pacchetto principale è una successiva suddivisione del nodo in più file. Le note di trasferimento riportano verifiche pregresse di equivalenza, non ripetute in questa pubblicazione: non bisogna confondere gli hash dei due binari.

Vedi [riproducibilità](RIPRODUCIBILITA.md), [sei scenari](../inventario/SEI_SCENARI.md) e [risultati](../risultati_finali/RISULTATI_TEST.md).
