#!/usr/bin/env python3
"""Indicatore "collisioni o blocchi davanti a un ostacolo" per il foglio dei risultati.

Un tentativo e' contato se almeno un drone:
- ha un contatto fisico registrato (physical_contact_events > 0, compresi gli urti fra droni), oppure
- non arriva al goal e nell'ultimo minuto prima dello stallo (o della fine della missione) ha lo
  scafo a meno di SOGLIA_M da un pilastro o da una parete della vasca.

Geometria approssimata dello scafo: segmento di +-SCAFO_MEZZA_LUNGHEZZA lungo lo yaw con raggio
SCAFO_RAGGIO; pilastri presi dal modello `ostacoli` del world.sdf salvato nel run, pareti dal
commento `Tank: wall_wx=... wall_wy=...`. Legge solo i file dei run; i risultati sono memorizzati
in reports/.cache_blocchi.json (chiave: cartella del run + mtime di summary.json).
"""
import csv
import json
import math
import os
import re
from pathlib import Path

SOGLIA_M = 2.0
FINESTRA_S = 60.0
SCAFO_MEZZA_LUNGHEZZA = 1.2
SCAFO_RAGGIO = 0.16
CACHE = Path('/home/federico/ros2_tethys/reports/.cache_blocchi.json')

_mondi = {}


def _mondo(world_sdf):
    if world_sdf not in _mondi:
        s = open(world_sdf).read()
        m = s[s.find('<model name="ostacoli"'):]
        m = m[:m.find('</model>')]
        obs = [(float(x), float(y), float(r)) for x, y, r in re.findall(
            r'<link name="[op]\d+">\s*<pose>([-\d.]+) ([-\d.]+) [^<]*</pose>\s*'
            r'<collision name="c"><geometry><cylinder><radius>([\d.]+)', m)]
        w = re.search(r'Tank: wall_wx=([\d.]+) wall_wy=([\d.]+)', s)
        _mondi[world_sdf] = (obs, float(w.group(1)), float(w.group(2)))
    return _mondi[world_sdf]


def _distanza(x, y, yaw, obs, wx, wy):
    ux, uy = math.cos(yaw), math.sin(yaw)
    h = SCAFO_MEZZA_LUNGHEZZA
    best = min(min(wx - abs(px), wy - abs(py)) for px, py in
               ((x + h * ux, y + h * uy), (x - h * ux, y - h * uy)))
    for ox, oy, r in obs:
        t = max(-h, min(h, (ox - x) * ux + (oy - y) * uy))
        best = min(best, math.hypot(ox - (x + t * ux), oy - (y + t * uy)) - r)
    return best - SCAFO_RAGGIO


def _bloccato(righe, fine, obs, wx, wy):
    return any(_distanza(x, y, yaw, obs, wx, wy) < SOGLIA_M
               for t, x, y, yaw in righe if fine - FINESTRA_S <= t <= fine)


def _calcola(run_dir):
    run_dir = Path(run_dir)
    s = json.load(open(run_dir / 'summary.json'))
    if s['physical_contact_events'] > 0:
        return True
    obs, wx, wy = _mondo(str(run_dir / 'world.sdf'))
    if 'drones' not in s:                      # 1 drone (review_corridor.py)
        if s['goal']:
            return False
        righe = [(float(r['sim_t']), float(r['x']), float(r['y']), float(r['yaw']))
                 for r in csv.DictReader(open(run_dir / 'telemetry.csv'))]
        return bool(righe) and _bloccato(righe, righe[-1][0], obs, wx, wy)
    falliti = {n: v['end_sim_t'] for n, v in s['drones'].items() if v['state'] != 'goal'}
    if not falliti:
        return False
    righe = {n: [] for n in falliti}
    for r in csv.DictReader(open(run_dir / 'telemetry.csv')):
        if r['drone'] in righe:
            righe[r['drone']].append((float(r['sim_t']), float(r['x']), float(r['y']), float(r['yaw'])))
    for n, fine in falliti.items():
        rr = righe[n]
        if rr and _bloccato(rr, fine if fine is not None else rr[-1][0], obs, wx, wy):
            return True
    return False


def conta(run_dirs):
    """Numero di tentativi con collisione o blocco davanti a un ostacolo."""
    try:
        cache = json.load(open(CACHE))
    except (OSError, ValueError):
        cache = {}
    n, cambiata = 0, False
    for d in run_dirs:
        chiave = f'{d}|{os.path.getmtime(Path(d) / "summary.json")}|{SOGLIA_M}'
        if chiave not in cache:
            cache[chiave] = _calcola(d)
            cambiata = True
        n += cache[chiave]
    if cambiata:
        json.dump(cache, open(CACHE, 'w'))
    return n
