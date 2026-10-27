#!/usr/bin/env python3
"""Ripete i tentativi finché ogni scenario ha un successo verificato (1 drone L1, poi 5 droni L1).

- Ogni tentativo: script/esegui_tentativo.sh (harness delle campagne + telecamera simulata).
- Esito: SOLO da summary.json dell'harness, con i criteri del progetto:
    1 drone: summary['goal'] == True (goal dichiarato dal controller prima dello stallo);
    5 droni: summary['success'] == True (5/5 al goal, ognuno prima del proprio stallo;
             un arrivo dopo lo stallo non conta) + controllo per drone.
- Fallimento: si cancellano SOLO i frame del tentativo (in /dev/shm) e la copia del binario
  `controller` nella cartella del run (identico, hash in metadata.json); restano summary,
  metadata, telemetria e log.
- Successo: i frame originali vanno in originali_successi/<scenario>_runNN/; si passa allo
  scenario successivo.
- Limiti: --max-tentativi (100) e --max-ore (6) complessivi; 3 errori tecnici consecutivi -> stop.
- Stato in log/stato_campagna.json e log/tentativi.csv dopo ogni tentativo: rilanciando lo
  script si riprende dal punto lasciato (i numeri di run già usati vengono saltati).
"""
import argparse, csv, datetime, json, os, shutil, subprocess, sys, time
from pathlib import Path

V = Path.home() / 'video_successi_livello1'
WS = Path.home() / 'ros2_tethys'
SCENARI = [('1drone', WS / 'worlds/mondoCorridoio.sdf', 'succ_mondoCorridoio'),
           ('5droni', WS / 'worlds/mondoMulti5.sdf', 'succ_mondoMulti5')]
ap = argparse.ArgumentParser()
ap.add_argument('--max-tentativi', type=int, default=100)
ap.add_argument('--max-ore', type=float, default=6.0)
ap.add_argument('--hz', type=int, default=10)
args = ap.parse_args()

STATO = V / 'log/stato_campagna.json'
TENT = V / 'log/tentativi.csv'
stato = json.load(open(STATO)) if STATO.exists() else {'secondi_usati': 0.0, 'tentativi': 0, 'successi': {}}
CAMPI = ['scenario', 'run', 'inizio', 'fine', 'durata_reale_s', 'esito', 'dettaglio', 'stop_reason',
         'sim_s', 'rt_factor', 'contatti', 'frame', 'cartella_log']
if not TENT.exists():
    with open(TENT, 'w', newline='') as f:
        csv.writer(f).writerow(CAMPI)


def salva_stato():
    STATO.write_text(json.dumps(stato, indent=1))


def esito(modo, run_dir):
    s = json.load(open(run_dir / 'summary.json'))
    if modo == '1drone':
        ok = bool(s.get('goal'))
        det = (f"goal a {s.get('measured_sim_s', 0):.1f} s" if ok else
               f"{s.get('stop_reason')}, min dal goal {s.get('min_goal_dist_m', 0):.1f} m")
        return ok, det, s, s.get('measured_sim_s'), s.get('physical_contact_events')
    d = s.get('drones', {})
    ok = bool(s.get('success')) and len(d) == 5 and all(v.get('goal') and not v.get('goal_after_stall')
                                                         for v in d.values())
    det = f"{s.get('goals')}/5 al goal; " + '; '.join(
        f"{k[-1]}={'goal' if v.get('goal') else v.get('state')}" for k, v in sorted(d.items()))
    return ok, det, s, s.get('mission_sim_s'), s.get('physical_contact_events')


errori_tecnici = 0
t_inizio_sessione = time.time()
for modo, mondo, stem in SCENARI:
    if stem in stato['successi']:
        continue
    run = 1
    while True:
        usati = stato['secondi_usati'] + (time.time() - t_inizio_sessione)
        if stato['tentativi'] >= args.max_tentativi or usati >= args.max_ore * 3600:
            stato['interrotta'] = f"limite raggiunto: {stato['tentativi']} tentativi, {usati/3600:.2f} h"
            stato['secondi_usati'] = usati
            salva_stato()
            print(stato['interrotta'], flush=True)
            sys.exit(2)
        rr = f'{run:02d}'
        if (V / 'log' / stem / f'run_{rr}').exists() or Path(f'/dev/shm/video_successi/{stem}_run{rr}').exists():
            run += 1
            continue
        inizio = datetime.datetime.now()
        print(f'{inizio:%H:%M:%S} {stem} run_{rr} avvio', flush=True)
        out = V / 'log' / f'esecuzione_{stem}_run{rr}.out'
        with open(out, 'w') as f:
            subprocess.run([str(V / 'script/esegui_tentativo.sh'), modo, str(mondo), str(run), str(args.hz)],
                           stdout=f, stderr=subprocess.STDOUT, cwd=str(Path.home()))
        fine = datetime.datetime.now()
        stato['tentativi'] += 1
        run_dir = V / 'log' / stem / f'run_{rr}'
        frame_dir = Path(f'/dev/shm/video_successi/{stem}_run{rr}')
        nframe = len(list((frame_dir / 'frame').glob('*.png'))) if frame_dir.exists() else 0
        try:
            ok, det, s, sim, cont = esito(modo, run_dir)
            errori_tecnici = 0
            tecnico = False
        except Exception as e:                          # summary mancante o illeggibile
            ok, det, s, sim, cont, tecnico = False, f'ERRORE TECNICO: {e}', {}, None, None, True
        if not tecnico and nframe < 10:
            tecnico, det = True, det + f'; ERRORE TECNICO: solo {nframe} frame della telecamera'
        if tecnico:
            errori_tecnici += 1
        riga = dict(scenario=stem, run=rr, inizio=f'{inizio:%Y-%m-%d %H:%M:%S}', fine=f'{fine:%H:%M:%S}',
                    durata_reale_s=f'{(fine-inizio).total_seconds():.0f}',
                    esito='SUCCESSO' if ok else ('ERRORE_TECNICO' if tecnico else 'fallito'),
                    dettaglio=det, stop_reason=s.get('stop_reason'), sim_s=sim,
                    rt_factor=f"{s.get('rt_factor', 0):.3f}" if s else '', contatti=cont, frame=nframe,
                    cartella_log=str(run_dir.relative_to(V)) if run_dir.exists() else '')
        with open(TENT, 'a', newline='') as f:
            csv.DictWriter(f, fieldnames=CAMPI).writerow(riga)
        print(f"{fine:%H:%M:%S} {stem} run_{rr} {riga['esito']}: {det} (rt {riga['rt_factor']}, frame {nframe})",
              flush=True)
        if ok:
            dest = V / 'originali_successi' / f'{stem}_run{rr}'
            dest.parent.mkdir(exist_ok=True)
            shutil.move(str(frame_dir), str(dest))
            stato['successi'][stem] = rr
        else:
            if frame_dir.exists():
                shutil.rmtree(frame_dir)                 # solo i frame di questo tentativo
            b = run_dir / 'controller'
            if b.is_file():
                b.unlink()                               # copia del binario (hash in metadata.json)
        stato['secondi_usati'] = stato['secondi_usati'] + (time.time() - t_inizio_sessione)
        t_inizio_sessione = time.time()
        salva_stato()
        if errori_tecnici >= 3:
            stato['interrotta'] = '3 errori tecnici consecutivi: diagnosi necessaria'
            salva_stato()
            print(stato['interrotta'], flush=True)
            sys.exit(3)
        if ok:
            break
        run += 1
print('CAMPAGNA COMPLETATA', stato['successi'], flush=True)
