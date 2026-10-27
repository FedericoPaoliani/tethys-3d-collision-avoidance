# Livello 1: due prove riuscite

Nuove esecuzioni del 27 ottobre 2026, separate dalle campagne della tesi. Sono video selezionati di successi: non rappresentano il tasso di successo del sistema.

| Scenario | Prova scelta | Tentativi | Video | Esito |
|---|---|---|---|---|
| Un drone, 30 ostacoli | run_17 | 17: 16 stalli e 1 successo | [60,6 s, 3×](finali/tethys_1drone_livello1_successo.mp4) | 1/1 al goal, 151,7 s simulati |
| Cinque droni, 40 ostacoli | run_02 | 2: 1 fallimento e 1 successo | [83,5 s, 2×](finali/tethys_5droni_livello1_successo.mp4) | 5/5 al goal, 146,9 s simulati |

I riepiloghi delle due prove confermano il successo e zero contatti registrati. Nel caso multi-drone l'harness sposta fuori vasca i veicoli dopo l'arrivo al goal: la loro scomparsa è il comportamento di gestione della prova.

## Montaggio e verifica

Filmati già montati sulla macchina Ubuntu, pubblicati senza ricodifica: MP4 H.264, 1920×1080, 30 fps, senza audio. Le registrazioni sorgenti sono 1280×720, ingrandite nel montaggio. Velocità uniforme rispetto al tempo simulato, missione completa senza tagli secondo i metadati, cartelli iniziali di 4 s e finali di 6 s. SHA-256 verificati contro il manifest e decodifica completa eseguita prima della pubblicazione; controllati fotogrammi iniziali, intermedi e finali.

## Log e provenienza

- [Tutti i 19 tentativi](log/tentativi.csv), con summary e metadata di ogni prova nelle rispettive cartelle.
- [Prova riuscita con un drone](log/succ_mondoCorridoio/run_17/): log e telemetria.
- [Prova riuscita con cinque droni](log/succ_mondoMulti5/run_02/): log e telemetria.
- [Manifest originale](manifest.csv): versioni, comandi, hash e parametri.
- [Mondi con telecamera](log/mondi/) e [strumenti sorgente](script/).

Il manifest identifica il codice Ubuntu con commit `ee1345e` e binario `26a72beaf1a4`. I mondi derivano da quelli definitivi con una telecamera aggiunta. Il real-time factor delle prove scelte è circa 0,933 e 0,699; la registrazione modifica il carico della macchina. Dal tentativo 10 con un drone la risoluzione di cattura è stata ridotta da 1080p a 720p. Queste prove non aggiornano i totali storici della tesi.

## Materiale selezionato

Esclusi frame grezzi, montaggi precedenti, campioni tecnici, cache, binari e copie ripetute del controller. Gli originali restano sulla pennetta. Il manifest contiene anche riferimenti a questi originali non pubblicati. Gli script mantengono i percorsi del PC Ubuntu e richiedono ambiente ROS/Gazebo e compilazione dei componenti; non sono stati eseguiti su Windows. Il produttore segnala un problema locale di compilazione dell'abbonato relativo a libsodium.
