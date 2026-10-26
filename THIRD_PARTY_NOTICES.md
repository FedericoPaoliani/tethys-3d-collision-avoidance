# Componenti di terzi e licenze

## Modello e descrizione Tethys

Le risorse in `progetto/ros2_tethys/src/lrauv_description/` derivano dal progetto [osrf/lrauv](https://github.com/osrf/lrauv), sviluppato con il contributo di Open Robotics / Open Source Robotics Foundation e MBARI. Le attribuzioni presenti nei file sono conservate. Il file `model.config` contiene ulteriori informazioni sull'autore del modello.

È inclusa la [licenza Apache 2.0 upstream](progetto/ros2_tethys/src/lrauv_description/LICENSE). Il modello e gli SDF del progetto includono adattamenti locali per sensori, attuatori e scenari sperimentali; non sono presentati come una distribuzione upstream non modificata.

Riferimento usato per il confronto: commit `8cf77021c2e6b6727592f1d952c97349823ecc94` del repository ufficiale. Questo è il riferimento consultato durante la preparazione, **non** il commit di origine della copia storica, che non è noto.

I file scientifici originali sono mantenuti senza riscritture per preservare gli hash delle campagne. Le modifiche locali e le differenze rilevate sono elencate in [MODIFICHE_LOCALI.md](progetto/ros2_tethys/src/lrauv_description/MODIFICHE_LOCALI.md).

## Codice e materiali dell'autore

Il pacchetto `collision_avoidance` è presentato come contributo di Federico Paoliani. Il suo `package.xml` originale contiene ancora un campo licenza da definire. Non viene applicata automaticamente la licenza del modello al codice dell'autore, ai risultati o al video. La scelta di una licenza per questi materiali resta da definire.

## Articolo di riferimento

Borenstein e Koren, 1991, è citato come riferimento scientifico. Il PDF dell'articolo non è ripubblicato in questo repository.
