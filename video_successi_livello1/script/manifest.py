#!/usr/bin/env python3
"""Scrive manifest.csv dai log dei tentativi riusciti e dai video finali (nessun valore a mano)."""
import csv, hashlib, json, os
from pathlib import Path

V = Path.home() / 'video_successi_livello1'
tent = list(csv.DictReader(open(V / 'log/tentativi.csv')))
righe = []
for nome, stem, mondo_orig, cmd in [
        ('tethys_1drone_livello1_successo', 'succ_mondoCorridoio', 'worlds/mondoCorridoio.sdf (c20af7c59aa0)',
         'script/esegui_tentativo.sh 1drone ~/ros2_tethys/worlds/mondoCorridoio.sdf 17 10 -> '
         'scripts/review_corridor.py --run 17 --duration 900 --stall-abort-s 60 --wall-limit 9000'),
        ('tethys_5droni_livello1_successo', 'succ_mondoMulti5', 'worlds/mondoMulti5.sdf (4ad20a45d2cc)',
         'script/esegui_tentativo.sh 5droni ~/ros2_tethys/worlds/mondoMulti5.sdf 2 10 -> '
         'scripts/review_multi.py --run 2 --duration 900 --stall-abort-s 60 --min-goals 5 --wall-limit 9000')]:
    mine = [t for t in tent if t['scenario'] == stem]
    ok = [t for t in mine if t['esito'] == 'SUCCESSO'][0]
    run = V / 'log' / stem / f"run_{ok['run']}"
    s, m = json.load(open(run / 'summary.json')), json.load(open(run / 'metadata.json'))
    info = json.load(open(V / 'finali' / f'{nome}.json'))
    mp4 = V / 'finali' / f'{nome}.mp4'
    righe.append({
        'video': f'finali/{nome}.mp4',
        'scenario': stem.replace('succ_', ''),
        'mondo': f"{mondo_orig} + telecamera senza geometria -> log/mondi/{stem}.sdf ({m['world_sha256'][:12]})",
        'codice': f"commit ee1345e (master), nessuna modifica ai file tracciati; binario installato "
                  f"{m['binary_sha256'][:12]}, sorgente {m['source_sha256'][:12]}",
        'comando': cmd,
        'parametri': 'yaw_bias=pi, goal_radius=3, depth_limit_high=17, allow_vertical_escape=false (harness); '
                     'stallo 60 s / 1 m; seed del mondo nel commento SDF; esecuzione senza seed (non deterministica)',
        'tentativi_scenario': len(mine),
        'esiti_tentativi': '; '.join(f"run_{t['run']}={t['esito']}" for t in mine),
        'prova_scelta': f"run_{ok['run']} ({ok['inizio']})",
        'esito_verificato': ok['dettaglio'] + f"; contatti {s.get('physical_contact_events')}",
        'log': f"log/{stem}/run_{ok['run']}/ (summary.json, metadata.json, telemetry.csv, controller*.log, gazebo.log)",
        'originale': f"originali_successi/{stem}_run{ok['run']}/ ({info['frame_sorgente']} PNG 1280x720 a 10 Hz di tempo simulato)",
        'rt_factor': f"{s['rt_factor']:.3f}",
        'durata_simulata_s': f"{info['sim_fine_s']:.1f}",
        'durata_video_s': f"{info['durata_video_s']:.1f}",
        'velocita': f"{info['velocita']}x rispetto al tempo simulato, uniforme; cartelli fermi 4 s + 6 s",
        'tagli': info['tagli'],
        'dimensione_byte': mp4.stat().st_size,
        'sha256': hashlib.sha256(mp4.read_bytes()).hexdigest(),
    })
with open(V / 'manifest.csv', 'w', newline='', encoding='utf-8') as f:
    w = csv.DictWriter(f, fieldnames=list(righe[0]))
    w.writeheader(); w.writerows(righe)
for r in righe:
    print(r['video'], r['tentativi_scenario'], r['prova_scelta'], r['durata_video_s'], r['dimensione_byte'])
