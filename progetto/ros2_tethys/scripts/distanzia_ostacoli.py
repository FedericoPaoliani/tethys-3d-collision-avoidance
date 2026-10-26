#!/usr/bin/env python3
"""Allontana fra loro gli ostacoli di un mondo senza cambiarne numero, raggio e altezza.

Rilassamento deterministico: a ogni iterazione le coppie con centri a meno di SEP metri
vengono spinte via simmetricamente; poi ogni ostacolo e' riportato dentro la vasca (centro
ad almeno WALL metri dalle pareti) e fuori dai cerchi di SPAWN_CLEAR metri attorno a spawn
e target. Le posizioni di partenza sono quelle del mondo d'ingresso, quindi la
disposizione generale resta riconoscibile. Stesso ingresso -> stesso risultato.

Uso: python3 scripts/distanzia_ostacoli.py IN.sdf OUT.sdf --sep 9 [--wall 3] [--spawn-clear 12]
Chiesto dall'utente il 2 ottobre 2026 per il Livello 3 a 5 droni (vedi scripts/livelli.py).
"""
import argparse
import math
import re
from pathlib import Path


def leggi(testo):
    i = testo.index('<model name="ostacoli">')
    j = testo.index('</model>', i)
    pose = [(m.start(1), m.end(1), *map(float, m.group(1).split()[:2]))
            for m in re.finditer(r'<link name="[op]\d+">\s*<pose>([^<]+)</pose>', testo[i:j])]
    return i, pose


def punti_fissi(testo):
    """Spawn e target dai commenti generati da generate_world.py."""
    pts = []
    for m in re.finditer(r'spawn: \(([-\d.]+), ([-\d.]+),[^)]*\)\s+target: \(([-\d.]+), ([-\d.]+),', testo):
        sx, sy, tx, ty = map(float, m.groups())
        pts += [(sx, sy), (tx, ty)]
    return pts


def rilassa(xy, half_x, half_y, sep, wall, fissi, spawn_clear, iterazioni=4000):
    xy = [list(p) for p in xy]
    for _ in range(iterazioni):
        mosso = False
        for a in range(len(xy)):
            for b in range(a + 1, len(xy)):
                dx, dy = xy[b][0] - xy[a][0], xy[b][1] - xy[a][1]
                d = math.hypot(dx, dy)
                if d < sep:
                    if d < 1e-9:
                        dx, dy, d = 1.0, 0.0, 1.0
                    k = (sep - d) / 2 / d
                    xy[a][0] -= dx * k; xy[a][1] -= dy * k
                    xy[b][0] += dx * k; xy[b][1] += dy * k
                    mosso = True
        for p in xy:
            for fx, fy in fissi:
                dx, dy = p[0] - fx, p[1] - fy
                d = math.hypot(dx, dy)
                if d < spawn_clear:
                    if d < 1e-9:
                        dx, dy, d = 0.0, 1.0, 1.0
                    p[0] = fx + dx / d * spawn_clear; p[1] = fy + dy / d * spawn_clear
                    mosso = True
            p[0] = max(-half_x + wall, min(half_x - wall, p[0]))
            p[1] = max(-half_y + wall, min(half_y - wall, p[1]))
        if not mosso:
            break
    return [tuple(p) for p in xy]


def distanzia(inp, out, sep, wall=3.0, spawn_clear=12.0):
    testo = Path(inp).read_text()
    hx = float(re.search(r'wall_wx=([\d.]+)', testo).group(1))
    hy = float(re.search(r'wall_wy=([\d.]+)', testo).group(1))
    base, pose = leggi(testo)
    nuovi = rilassa([(x, y) for _, _, x, y in pose], hx, hy, sep, wall, punti_fissi(testo), spawn_clear)
    pezzi, prec = [], 0
    for (s, e, _, _), (x, y) in zip(pose, nuovi):
        vecchia = testo[base + s:base + e].split()
        pezzi += [testo[prec:base + s], ' '.join([f'{x:.2f}', f'{y:.2f}', *vecchia[2:]])]
        prec = base + e
    pezzi.append(testo[prec:])
    nuovo = ''.join(pezzi)
    nuovo = nuovo.replace(' -->\n', f' — distanziati: centri >= {sep:g} m, pareti >= {wall:g} m -->\n', 1)
    Path(out).write_text(nuovo)
    return nuovi


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('inp')
    ap.add_argument('out')
    ap.add_argument('--sep', type=float, required=True)
    ap.add_argument('--wall', type=float, default=3.0)
    ap.add_argument('--spawn-clear', type=float, default=12.0)
    a = ap.parse_args()
    distanzia(a.inp, a.out, a.sep, a.wall, a.spawn_clear)
