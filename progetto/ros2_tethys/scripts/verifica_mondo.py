#!/usr/bin/env python3
"""Verifica le protezioni del generatore su un mondo generato (nessuna dipendenza da ROS).

Uso: python3 scripts/verifica_mondo.py MONDO.sdf [--wall-clear 6] [--min-sep 7] [--spawn-clear 12]
Esce con codice 1 se una verifica fallisce.

Controlla, sui soli link oN/pN del modello `ostacoli`:
- ogni ostacolo interamente dentro la vasca (pareti lette dal commento `Tank:`);
- centro a >= wall_clear dalle pareti laterali e di fondo (WALL_CLEAR del generatore);
- distanza XY >= spawn_clear da ogni spawn e da ogni target (SPAWN_CLEAR = 12 m);
- separazione fra centri >= min_sep;
- navigabilita': in ogni fascia di 10 m lungo Y resta almeno un varco >= 4 m
  (COLLISION_WIDTH = 2 m per lato del nodo);
- collo di bottiglia (--min-collo, per ogni drone): larghezza della strettoia sul percorso
  spawn->target che resta piu' lontano da ostacoli e pareti (cammino maximin su griglia a
  0,25 m). A differenza del controllo per fasce vede anche le barriere sfalsate fra due fasce.
"""
import argparse
import heapq
import itertools
import math
import re
import sys
import xml.etree.ElementTree as ET

ap = argparse.ArgumentParser()
ap.add_argument('world')
ap.add_argument('--wall-clear', type=float, default=6.0)
ap.add_argument('--min-sep', type=float, default=7.0)
ap.add_argument('--spawn-clear', type=float, default=12.0)
ap.add_argument('--min-gap', type=float, default=4.0)
ap.add_argument('--min-collo', type=float, default=0.0,
                help='larghezza minima del collo di bottiglia spawn->target [m] (0 = solo misura)')
a = ap.parse_args()

text = open(a.world).read()
hx, hy = (float(v) for v in re.search(r'wall_wx=([\d.]+) wall_wy=([\d.]+)', text).groups())
world = ET.fromstring(text).find('world')
obs = []
for link in world.find("model[@name='ostacoli']").findall('link'):
    x, y = (float(v) for v in link.findtext('pose').split()[:2])
    obs.append((link.get('name'), x, y, float(link.find('.//cylinder/radius').text)))
points, starts, goals = [], {}, {}
for m in world.findall('model'):
    nm = re.fullmatch(r'(tethys|target)(_\d+)?', m.get('name', ''))
    if nm:
        p = tuple(float(v) for v in m.findtext('pose').split()[:2])
        points.append(p)
        (starts if nm.group(1) == 'tethys' else goals)[nm.group(2) or ''] = p


def collo(start, goal, res=0.25):
    """Larghezza (2 x clearance) della strettoia sul cammino maximin start->goal."""
    nx, ny = int(2 * hx / res) + 1, int(2 * hy / res) + 1
    xs = [-hx + i * res for i in range(nx)]
    ys = [-hy + j * res for j in range(ny)]
    clear = [[min(hx - abs(x), hy - abs(y)) for x in xs] for y in ys]
    for _, ox, oy, r in obs:
        j0, j1 = max(0, int((oy - r - 8 + hy) / res)), min(ny, int((oy + r + 8 + hy) / res) + 1)
        i0, i1 = max(0, int((ox - r - 8 + hx) / res)), min(nx, int((ox + r + 8 + hx) / res) + 1)
        for j in range(j0, j1):
            row, y = clear[j], ys[j]
            for i in range(i0, i1):
                d = math.hypot(xs[i] - ox, y - oy) - r
                if d < row[i]:
                    row[i] = d
    si = (round((start[1] + hy) / res), round((start[0] + hx) / res))
    gi = (round((goal[1] + hy) / res), round((goal[0] + hx) / res))
    best = {si: clear[si[0]][si[1]]}
    pq = [(-best[si], si)]
    while pq:
        v, (j, i) = heapq.heappop(pq)
        v = -v
        if (j, i) == gi:
            return 2 * v
        if v < best.get((j, i), -1e9):
            continue
        for dj, di in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            a, b = j + dj, i + di
            if 0 <= a < ny and 0 <= b < nx:
                nv = min(v, clear[a][b])
                if nv > best.get((a, b), -1e9):
                    best[(a, b)] = nv
                    heapq.heappush(pq, (-nv, (a, b)))
    return 0.0

dentro = min(min(hx - abs(x) - r, hy - abs(y) - r) for _, x, y, r in obs)
parete = min((min(hx - abs(x), hy - abs(y)), n) for n, x, y, _ in obs)
spawn = min((math.hypot(x - px, y - py), n) for n, x, y, _ in obs for px, py in points)
sep = min((math.hypot(p[1] - q[1], p[2] - q[2]), p[0], q[0]) for p, q in itertools.combinations(obs, 2))
varco = 1e9
for y0 in range(-int(hy), int(hy), 10):
    iv = sorted((x - r, x + r) for _, x, y, r in obs if y + r > y0 and y - r < y0 + 10)
    best, cur = 0.0, -hx
    for lo, hi in iv + [(hx, hx)]:
        best, cur = max(best, lo - cur), max(cur, hi)
    varco = min(varco, best)

# Abbinamento in ordine: nel corridoio il drone e' `tethys` e il target `target_0`.
colli = [collo(starts[ks], goals[kg]) for ks, kg in zip(sorted(starts), sorted(goals))]
esiti = [
    (f'{len(obs)} ostacoli ({sum(n.startswith("o") for n, *_ in obs)} casuali + '
     f'{sum(n.startswith("p") for n, *_ in obs)} di percorso), vasca {2*hx:g}x{2*hy:g} m', True),
    (f'tutti dentro la vasca (margine minimo {dentro:.2f} m)', dentro > 0),
    (f'centro-parete minimo {parete[0]:.2f} m ({parete[1]}) >= {a.wall_clear}', parete[0] >= a.wall_clear),
    (f'ostacolo-spawn/target minimo {spawn[0]:.2f} m ({spawn[1]}) >= {a.spawn_clear}', spawn[0] >= a.spawn_clear),
    (f'separazione minima {sep[0]:.2f} m ({sep[1]}-{sep[2]}) >= {a.min_sep}', sep[0] >= a.min_sep),
    (f'varco piu stretto per fascia {varco:.1f} m >= {a.min_gap}', varco >= a.min_gap),
    (f'collo di bottiglia spawn->target {min(colli):.1f} m (per drone: '
     f'{" / ".join(f"{c:.1f}" for c in colli)}) >= {a.min_collo}', min(colli) >= a.min_collo),
]
for msg, ok in esiti:
    print(('OK   ' if ok else 'FAIL ') + msg)
sys.exit(0 if all(ok for _, ok in esiti) else 1)
