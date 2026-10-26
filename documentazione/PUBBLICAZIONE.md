# Preparazione della raccolta per GitHub

## Origine

Selezione `Tethys_finale` trasferita dal PC Ubuntu tramite USB. I 4201 file elencati nell'inventario originario sono stati verificati con SHA-256 senza discrepanze. Sulla chiavetta erano presenti inoltre i report dell'inventario stesso.

## Interventi nella copia pubblica

- README riscritto come pagina di presentazione con video, risultati e collegamenti.
- Appunti conversazionali `CLAUDE.md` esclusi; creata una guida tecnica pubblica distinta.
- Corretto nel foglio risultati il testo che attribuiva le differenze tra livelli al solo numero di ostacoli. **Tutte le righe numeriche sono mantenute.**
- Chiarito che i riepiloghi non permettono di rigenerare la metrica dei blocchi senza telemetria completa.
- Aggiornati i riferimenti delle note di trasferimento alla documentazione pubblica.
- Aggiunta la licenza upstream delle risorse Tethys e il confronto di provenienza.
- Regole Git aggiornate per includere i riepiloghi selezionati, preservare esattamente i byte e conservare i permessi eseguibili degli script.

Sorgenti scientifici, sei mondi, dati JSON, video e immagini restano invariati. Gli hash dei file pubblicati sono in `inventario/FILE_PUBBLICATI.csv`; per i file modificati è indicato anche lo SHA-256 della copia USB, quando disponibile.

## Verifiche

Integrità della copia, controllo di pattern comuni di credenziali, parsing JSON/XML, analisi sintattica Python, corrispondenza degli hash dei sei mondi con i metadati e controllo dei collegamenti locali della documentazione principale. Compilazione ROS 2, test C++ e simulazioni non eseguiti su Windows.

## Materiali ancora da aggiungere

Tesi definitiva in PDF, relazione finale del tirocinio e presentazione finale non erano nella raccolta USB. Non sono state ricostruite o sostituite con bozze.

## Aggiunta della tesi definitiva

Aggiunto `tesi/L31_paoliani_tirocinio.pdf`, indicato e fornito dall’autore come versione definitiva. Il documento di 70 pagine è pubblicato senza modifiche. Aggiornati README e inventario; restano da aggiungere le slide finali.
