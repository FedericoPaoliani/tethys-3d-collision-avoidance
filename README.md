# Collision avoidance 3D per Tethys

Progetto di tesi e tirocinio di Federico Paoliani dedicato
alla navigazione e all’evitamento degli ostacoli per il
veicolo subacqueo autonomo Tethys.

## Obiettivo

Sviluppare e valutare in simulazione un sistema che permetta
al veicolo di raggiungere un target evitando gli ostacoli
percepiti dai sonar.

## Metodo

Il sistema si ispira al Vector Field Histogram (VFH),
adattandone il principio a una rappresentazione
tridimensionale degli ostacoli mediante un istogramma sferico.

La pipeline comprende:
- acquisizione ed elaborazione dei dati dei sonar;
- rappresentazione degli ostacoli;
- pianificazione locale e scelta della direzione;
- controllo di pinne e propulsore;
- raccolta e analisi dei risultati.

## Tecnologie

- ROS 2 per i componenti software e la comunicazione.
- Gazebo per la simulazione del veicolo e dell’ambiente.
- C++ per il software di navigazione e controllo.

## Materiale del progetto

Il repository è in allestimento. Verranno aggiunti:
- codice sorgente e istruzioni di esecuzione;
- documentazione della tesi e del tirocinio;
- presentazione;
- risultati sperimentali e grafici;
- video delle simulazioni.

## Validazione

Il sistema è stato valutato in simulazione.
I risultati e i limiti saranno documentati insieme
alle configurazioni degli esperimenti.

## Riferimento principale

J. Borenstein e Y. Koren,
“The Vector Field Histogram—Fast Obstacle Avoidance
for Mobile Robots”, IEEE Transactions on Robotics
and Automation, 1991.
