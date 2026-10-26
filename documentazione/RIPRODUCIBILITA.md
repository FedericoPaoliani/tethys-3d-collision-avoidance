# Risultati e limiti di riproducibilità

## Che cosa è incluso

Sono conservati sei mondi definitivi, sorgenti, risorse del modello, script, riepiloghi e metadati di 2039 run, foglio finale, grafici e un video dimostrativo. Il riepilogo sperimentale considera 1846 tentativi fino al decimo successo per serie. I run ulteriori nell'archivio non vanno aggiunti automaticamente alle percentuali.

## Dati esclusi

Le telemetrie complete delle campagne occupavano circa 25 GB e non sono incluse. `scripts/blocchi.py` usa quelle telemetrie, oppure una cache che dipende da percorsi assoluti e timestamp. La cache copiata non è portabile: il foglio completo **non può essere rigenerato in modo affidabile dai soli summary e metadata**. Per rieseguire quell'analisi servono i dati grezzi originali. I numeri pubblicati restano quelli del foglio fornito, senza ricostruzioni o stime.

## Definizioni

- Successo: tutti i veicoli raggiungono il target prima della dichiarazione di stallo.
- Stallo: 60 secondi simulati senza il progresso richiesto di 1 m verso il target.
- Collisioni fisiche: contatti registrati dai sensori.
- Collisioni o blocchi: almeno un contatto, oppure un veicolo fallito che rimane entro 2 m da un ostacolo/parete nell'intervallo finale usato dall'analisi.

Le medie temporali comprendono prove riuscite e fallite. Un tempo minore non dimostra da solo una navigazione migliore.

## Configurazioni e portabilità

Fra i livelli cambiano anche posizioni e raggi degli ostacoli; il livello 3 con cinque droni usa inoltre corsie e quote diverse. Le dimensioni delle vasche differiscono fra uno e cinque droni. I risultati non isolano unicamente l'effetto del numero di ostacoli.

I mondi e gli script contengono percorsi del PC Ubuntu originale. I file SDF e i sorgenti scientifici sono stati mantenuti byte per byte, per non alterare gli hash storici. Adattare percorsi o mondi per una nuova macchina crea una nuova configurazione da registrare come tale.

Il sorgente delle campagne e la versione suddivisa in più file sono entrambi inclusi e identificati. La compilazione non è stata ripetuta in questa pubblicazione. La non determinazione dei sensori e del rendering può produrre esiti diversi ripetendo una prova.

## Video e grafici

Il video usa uno scenario semplificato con un solo pilastro ed è riprodotto a 2×. Non è una ripresa di uno dei sei mondi completi. I grafici sono quelli trovati nella raccolta; per alcuni non è disponibile lo script originale di generazione.

I dettagli di trasferimento sono conservati in [DA_VERIFICARE.md](../inventario/DA_VERIFICARE.md). Le note sono evidenza storica del lavoro sul PC originale, non prove rieseguite su questa macchina.
