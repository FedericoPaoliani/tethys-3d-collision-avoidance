# Video dimostrativo per la presentazione della tesi

**File:** `Manovra_Collision_Avoidance_Tethys.mp4`: 19,9 s, 1920×1080, 30 fps, H.264 High@4.1
(yuv420p), MP4 con `faststart`, senza audio, 1,9 MB. Verificato con GStreamer (`qtdemux`),
con la decodifica `libavcodec` di tutti i 598 frame e con la riproduzione in Chrome.

> **Da dichiarare nella slide: riproduzione a velocità 2×.** Il video copre 39,4 s di tempo
> simulato (da t=3,5 s a t=42,9 s della missione), con un frame simulato ogni due.

## Cosa mostra

È una missione reale e non modificata del nodo `collision_avoidance` (VFH 3D), nel mondo del
Livello 1 a 1 drone ridotto a **un solo pilastro**. La telecamera è un sensore Gazebo che
riprende la simulazione mentre gira.

| Video | Tempo simulato | Fase | Cosa succede (telemetria e log del controller, run_07) |
|---|---|---|---|
| 0 – 3,6 s | 3,5 – 10,7 s | **Avvicinamento** | CRUISE verso il goal (0, 50, 8), rotta dritta x=0, pilastro davanti a 26–35 m |
| 3,6 – 12,5 s | 10,7 – 28,5 s | **Deviazione** | A y=−41 m il corridoio verso il goal risulta bloccato a 24,7 m: lookahead `[LH] pref=0° best=10°`, waypoint locale (−2,1; −29,3), passaggio CRUISE → ESCAPE. Il timone gira a sinistra e il drone si sposta a x≈−3,3 m |
| ≈12,5 s | 28,5 s | **Superamento** | Distanza minima dalla superficie del pilastro: **2,37 m**, nessun contatto |
| 12,5 – 19,9 s | 28,5 – 42,9 s | **Ripresa** | Il waypoint successivo è scelto nella direzione del goal (`best=0°`). Da t≈34 s (video 15,2 s) timone ≈0 e prua entro 3° dalla direzione del goal |

Velocità media durante la manovra: 1,46 m/s. Nessuna frenata (`BRAKE`) e nessun contatto fisico
(`summary.json`: `physical_contact_events: 0`).

### Per la discussione: la deviazione non è programmata

La sequenza di decisioni del planner, dal log `prove/run_07/controller.log`:

```
[LH] pref=0° best=10° score=52.7 clr=86.4                 direzione del goal bloccata: sceglie 10° a sinistra
[LOCAL-PATH] yaw=100° len=12.0 wp=(-2.1,-29.3) clr=86.4    waypoint locale fisso nello spazio
[AVOID-MINIMAL] corridoio bloccato clr=24.7 m, az=10°      → ESCAPE
[WP-REACHED] wp=(-2.1,-29.3)  →  [LH] pref=-19° best=-10°  secondo waypoint (-2,6; -19,4)
[WP-REACHED] wp=(-2.6,-19.4)  →  [LH] pref=-7°  best=-5°   affianca il pilastro (-2,7; -9,8)
[WP-REACHED] wp=(-2.7,-9.8)   →  [LH] pref=3°   best=0°    pilastro superato: di nuovo verso il goal
```

Il pilastro non è noto al controller: viene percepito dal sonar frontale (35 m, 30×15 raggi)
e dai punti accumulati nell'istogramma sferico. Il lookahead (`find_lookahead_escape()` in
`src/collision_avoidance/src/vfh_planner.cpp`) valuta le direzioni da −100° a +100° ogni 5°.
Scarta quelle con un corridoio libero (semi-larghezza 1,5 m più i margini) più corto di 11 m e
quelle con l'arco d'ingresso ostruito. Fra le rimaste sceglie il punteggio
`2·min(clearance, 30) − penalità per lo scarto dalla direzione del goal`: qui vince 10° a
sinistra, con punteggio 52,7. La stessa
decisione (`pref=0° best=10° score=52.7`) e la stessa traiettoria (virata a y≈−41 m,
x minima −3,3/−3,9 m, 2,3–2,4 m dal pilastro) si sono ripetute in **tutte e 7 le missioni**
eseguite (`prove/run_01`…`run_07`).

### Fotogrammi statici (per le slide, senza perdita)

`fotogrammi/Fase_1_avvicinamento_…png`, `Fase_2_deviazione_…`, `Fase_3_superamento_…`,
`Fase_4_ripresa_…` (1920×1080, presi dai frame originali della telecamera).

## Differenze rispetto al Livello 1 (`worlds/mondoCorridoio.sdf`, non modificato)

Il mondo demo è `mondi/mondo_demo_video.sdf`, generato da `strumenti/genera_mondo_demo.py`:

- **un solo pilastro** (`p0`) al posto dei 30: centro (0,5; −15), raggio 1,40 m, alto 12 m
  (i pilastri del L1 hanno raggio 0,8–1,5 m e altezza 11–16 m), sul fondale, sulla rotta
  spawn → goal, decentrato di 0,5 m;
- il modello **`telecamera_presentazione`**: statico, **senza geometria** (invisibile ai sonar,
  nessun effetto sulla fisica), con un sensore `camera` 1920×1080 a 30 Hz in tempo simulato.
  Un plugin (`strumenti/telecamera_plugin/SeguiDroneCamera.cc`) lo sposta solo lungo l'asse
  del corridoio, dietro al drone, con un filtro del primo ordine (τ=1 s). L'orientamento è
  fisso, quindi la virata si vede come rotazione del drone rispetto alle pareti dritte.

Tutto il resto è invariato: drone, sonar, vasca 30×120×20, spawn (0, −50, 8), goal (0, 50, 8),
fisica e parametri del nodo (quelli dell'harness `scripts/review_corridor.py`, come nelle
campagne). Il controller usato è il binario installato `26a72beaf1a4`, cioè la build dopo la
divisione in file dell'8 ottobre, equivalente a `fee8566e2489` (vedi le note tecniche originali (non pubblicate integralmente)). Il codice
del controller non è stato toccato.

## Come è stato prodotto

1. `strumenti/esegui_prova.sh finale 7 50 alto_centrale_v2` genera il mondo, avvia
   `scripts/review_corridor.py`, che lancia Gazebo headless, il bridge e il nodo con telemetria,
   e avvia nella stessa `IGN_PARTITION` `strumenti/abbonato/` (in Fortress una telecamera viene
   resa solo se il suo topic ha un iscritto; l'iscritto registra il tempo simulato di ogni frame).
2. I frame PNG salvati dal sensore (`prove/run_07/frame_telecamera/`, 1517 frame da 0 a 50 s)
   sono stati selezionati con `prove/run_07/frame_video.csv`: finestra 3,5–43 s, uno ogni due.
3. `strumenti/codificatore/build/codifica_h264` produce l'H.264 con libx264 via `libavcodec`
   (CRF 16, keyframe ogni secondo). `strumenti/codificatore/mux_mp4.py` crea l'MP4 con
   `mp4mux` di GStreamer e lo verifica con `qtdemux` (`--verifica`).

Le prove `run_01`–`run_06` sono anteprime per scegliere l'inquadratura: la manovra è la stessa,
cambiano solo le telecamere. `run_01` non ha immagini, mancava l'iscritto. `run_05` usa
un'inquadratura obliqua scartata e `run_06` la versione centrata, troppo vicina al bordo inferiore.
Il fattore di tempo reale scende a 0,14–0,29 per il rendering a 1080p. Il video non ne risente
perché è in tempo simulato, e la traiettoria coincide con quella delle anteprime a rt 0,97.

## ⚠ Problema di sistema trovato (non risolto, nessuna modifica fatta)

`dpkg -V` segnala due librerie di sistema **corrotte sul disco** (checksum diverso dal pacchetto):

- `/usr/lib/x86_64-linux-gnu/libavformat.so.58.76.100` (pacchetto `libavformat58`): va in
  `SIGILL` dentro `av_guess_format`. Rompe anche il registratore video della GUI di Gazebo e
  qualsiasi programma FFmpeg che scriva MP4;
- `/usr/lib/x86_64-linux-gnu/libsodium.so.23.3.0` (`libsodium23`): sezione `.dynstr` non valida.
  È la causa dell'errore `crypto_secretbox_open` all'`import cv2`.

Per questo il video è stato codificato senza `libavformat`. Per ripristinarle:
`sudo apt install --reinstall libavformat58 libsodium23`. Due file corrotti senza un motivo
apparente possono anche indicare un problema del disco: conviene controllare
`sudo dmesg | grep -i -E "ext4|I/O error"` e lo stato SMART.
