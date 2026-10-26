# Video delle simulazioni — livello 1

Due nuove esecuzioni dimostrative del 26 ottobre 2026, con uno e cinque droni. Non sono registrazioni storiche delle campagne della tesi e non vengono aggiunte ai loro conteggi.

| Scenario | Video | Riproduzione | Esito dai log |
|---|---|---|---|
| 1 drone, livello 1, 30 ostacoli | [MP4, 57,3 s](finali/tethys_1drone_livello1.mp4) | 4× | Stallo dopo 189,3 s simulati; distanza minima dal goal 42,3 m; zero contatti registrati |
| 5 droni, livello 1, 40 ostacoli | [MP4, 81,7 s](finali/tethys_5droni_livello1.mp4) | 3× | Quattro droni su cinque al goal; drone 1 in stallo; zero contatti registrati |

Entrambe le prove sono **non riuscite secondo il criterio del progetto**: tutti i droni devono raggiungere il target. I video mostrano anche i limiti del planner, senza presentarli come successi.

## Visualizzazione e verifica

MP4 H.264, 1920×1080, 30 fps, senza audio. Integrità verificata contro gli SHA-256 del manifest; tutti i fotogrammi decodificati nella preparazione della pubblicazione. I cartelli occupano 4 s iniziali e 6 s finali; il filmato è accelerato rispetto al tempo simulato, con velocità uniforme e senza tagli secondo i metadati di montaggio.

La camera segue il singolo drone; per cinque droni è fissa e panoramica con etichette. I veicoli sono piccoli nella panoramica e i goal sono vicini al bordo superiore.

## Provenienza e differenze dalle campagne

- [Manifest originale](manifest.csv): comandi, versioni, durate, hash e riferimenti alle esecuzioni sul PC Ubuntu.
- [Log del singolo drone](log/video_mondoCorridoio/run_01/): riepilogo, telemetria e log della prova.
- [Log dei cinque droni](log/multi_video_mondoMulti5/run_01/): riepilogo, telemetria e log della prova.
- [Mondi video](mondi_video/): copie dei mondi di livello 1 con telecamera aggiunta; gli originali dei sei scenari restano distinti.
- Il manifest identifica il codice Ubuntu con commit `ee1345e` e binario `26a72beaf1a4`; le campagne archiviate usavano un binario diverso. L'equivalenza dei binari non è stata verificata in questa pubblicazione.
- Il rendering riduce il real-time factor misurato a circa 0,518 e 0,271. Queste nuove prove non costituiscono una riproduzione identica dei risultati storici.

## Strumenti e selezione dei file

[Sorgenti di registrazione e montaggio](script/) conservati come forniti. Usano percorsi Ubuntu come `~/video_simulazioni_finali` e `~/ros2_tethys`: richiedono preparazione dell'ambiente, compilazione dei componenti C++ e adattamento dei percorsi. Non sono stati eseguiti su Windows. Il produttore segnala un problema locale di compilazione dell'abbonato legato a libsodium; i binari della sua macchina non sono inclusi.

Sono esclusi i campioni preliminari, le cache, i binari, i frame grezzi e le copie ripetute del sorgente del controller. Sono inclusi i due MP4 finali, le anteprime, i metadati, i log delle due esecuzioni, i mondi video e gli strumenti sorgente. Il manifest originale descrive l'esecuzione su Ubuntu e può riferirsi anche a file non selezionati per questa pubblicazione.
