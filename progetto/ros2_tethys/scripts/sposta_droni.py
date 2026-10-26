#!/usr/bin/env python3
"""Sposta spawn e goal dei droni di un mondo multi (x della corsia e quota), ostacoli invariati.

Modifica la posa dei modelli tethys_i e target_i e i commenti "Drone i spawn: ... target: ..."
che leggono gli script manuali e distanzia_ostacoli.py. L'harness (review_multi.py) legge
spawn e target dalle pose dei modelli. La y di spawn e target resta quella del mondo.

Uso: python3 scripts/sposta_droni.py IN.sdf OUT.sdf --x -24 -12 0 12 24 --z 7 9.5 7 9.5 7
Chiesto dall'utente il 2 ottobre 2026 per il Livello 3 a 5 droni (vedi scripts/livelli.py).
"""
import argparse
import re
from pathlib import Path


def _nuova_posa(testo, modello, x, z):
    m = re.search(rf'(<model name="{modello}">\s*(?:<static>true</static>\s*)?<pose>)([^<]+)(</pose>)', testo)
    if not m:
        raise SystemExit(f'modello {modello} non trovato')
    v = m.group(2).split()
    v[0], v[2] = f'{x:.2f}', f'{z:.2f}'
    return testo[:m.start(2)] + ' '.join(v) + testo[m.end(2):], float(v[1])


def sposta(inp, out, xs, zs):
    testo = Path(inp).read_text()
    for i, (x, z) in enumerate(zip(xs, zs)):
        testo, sy = _nuova_posa(testo, f'tethys_{i}', x, z)
        testo, ty = _nuova_posa(testo, f'target_{i}', x, z)
        testo, n = re.subn(rf'<!-- Drone {i} spawn: \([^)]*\)\s+target: \([^)]*\) -->',
                           f'<!-- Drone {i} spawn: ({x:.2f}, {sy:.2f}, {z:.2f})  '
                           f'target: ({x:.2f}, {ty:.2f}, {z:.2f}) -->', testo)
        if n != 1:
            raise SystemExit(f'commento del drone {i} non trovato')
    Path(out).write_text(testo)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('inp')
    ap.add_argument('out')
    ap.add_argument('--x', type=float, nargs='+', required=True)
    ap.add_argument('--z', type=float, nargs='+', required=True)
    a = ap.parse_args()
    if len(a.x) != len(a.z):
        raise SystemExit('--x e --z devono avere lo stesso numero di valori')
    sposta(a.inp, a.out, a.x, a.z)
