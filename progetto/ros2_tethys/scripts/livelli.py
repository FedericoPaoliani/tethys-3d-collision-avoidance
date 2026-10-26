#!/usr/bin/env python3
"""Configurazione unica dei livelli di difficolta' della campagna sperimentale.

Due scenari (1 drone, 5 droni), tre livelli ciascuno. Fra un livello e l'altro cambia
SOLO il numero di ostacoli (stessa quota di ostacoli di percorso del Livello 1);
vasca, spawn, target, drone, sonar e controller restano identici.

Uso:
  python3 scripts/livelli.py elenco                 # tabella dei livelli
  python3 scripts/livelli.py mondo 5 2              # nome del file del mondo (per i driver)
  python3 scripts/livelli.py stallo 5 2             # soglia di stallo del livello [s]
  python3 scripts/livelli.py successi               # successi che chiudono una campagna
  python3 scripts/livelli.py genera [5 [2]]         # genera i mondi L2/L3; per L1 verifica soltanto
                                                    # che il file esistente sia riproducibile byte per byte
  python3 scripts/livelli.py verifica [5 [2]]       # controlla le protezioni con verifica_mondo.py

Scelta dei mondi (30 settembre 2026, rivista lo stesso giorno). La prima versione dei
livelli usava come navigabilita' un varco di 4 m per fascia: nel corridoio L2 il drone si
bloccava sempre 10 m dopo lo spawn davanti a una barriera sfalsata fra due fasce (collo di
bottiglia reale 4,7 m). Criterio attuale per i livelli 2 e 3, fissato prima dei test:
- dentro la vasca, distanza nominale dalle pareti, distanza da spawn/target >= Livello 1;
- collo di bottiglia spawn->target >= 6 m per ogni drone (corridoio del pianificatore
  ~3,8 m + oltre 1 m di margine per lato), misurato da verifica_mondo.py;
- primo seed valido a partire dal default.
Nel corridoio (vasca 30 m) con 35-40 ostacoli un collo di 6 m non esiste con il raggio
standard (0,8-1,8 m): su richiesta dell'utente il raggio e' scalato a 0,7 (0,56-1,26 m) nei
livelli 2 e 3 del corridoio; Multi5 mantiene il raggio standard. Il 1° ottobre 2026 il
corridoio L3 e' stato portato a raggio 0,85 con le stesse posizioni (POSIZIONI_DA): il collo
di bottiglia scende sotto 6 m, scelta esplicita dell'utente.
"""
import filecmp
import re
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MIN_COLLO = 6.0   # collo di bottiglia spawn->target minimo per i livelli 2 e 3 [m]
# Secondi simulati senza avvicinarsi al goal dopo cui un drone e' in stallo. Uguale per tutti
# i livelli (i Livelli 1 sono stati misurati a 60 s): scelta dell'utente del 30 settembre,
# dopo una breve prova a 90 s archiviata.
STALLO = {1: 60, 2: 60, 3: 60}
# Successi che chiudono la campagna di un livello (portato da 20 a 10 il 30 settembre: con
# piu' ostacoli 20 successi richiederebbero troppe simulazioni). Anche i Livelli 1 gia'
# raccolti sono riassunti fino al tentativo del 10° successo (scripts/foglio_risultati.py).
SUCCESSI = 10
# Livelli le cui posizioni vengono dal generatore a un'altra scala del raggio: il mondo e' generato
# a quella scala e poi i raggi sono portati alla scala del livello senza spostare gli ostacoli.
# Corridoio L3 (1° ottobre 2026, scelta dell'utente): stesse posizioni del L3 a raggio 0,7 (33
# tentativi, 10 successi, archiviati) e raggio 0,85, a meta' fra Livello 1 (1,0) e 0,7.
# Multi5 L3 (3 ottobre 2026, scelta dell'utente dopo 1 successo su 257): stesse posizioni del
# mondo a raggio 1,0 (generato a 1,0, distanziato e con i droni spostati) e raggio 0,9.
POSIZIONI_DA = {(1, 3): 0.7, (5, 3): 1.0}
# Livelli i cui ostacoli, dopo la generazione, sono allontanati fra loro con
# scripts/distanzia_ostacoli.py: (distanza minima fra i centri, distanza minima centro-parete) [m].
# Multi5 L3 (2 ottobre 2026, scelta dell'utente): con 60 ostacoli il generatore satura la vasca
# (centri a ~7,4 m, varco mediano 4,8 m, 0 successi su 117); stessi ostacoli distanziati a 9 m,
# fino a 3 m dalle pareti. Run precedenti in mondoMulti5_L3/archivio_non_distanziato/.
DISTANZIA = {(5, 3): (9.0, 3.0)}
# Livelli con spawn e goal dei droni spostati (scripts/sposta_droni.py) prima di DISTANZIA:
# (x delle corsie, quota di ciascun drone) [m]. Multi5 L3 (2 ottobre 2026, scelta dell'utente):
# con le corsie a 10 m i droni 2 e 3 si urtavano in 10 run su 12 a t~18 s nel varco o35-o36;
# corsie a 12 m e quote alternate 7 / 9,5 m (sotto i pilastri piu' bassi, 11 m, e sopra il
# DEPTH LIMIT di 5 m). Run precedenti in mondoMulti5_L3/archivio_distanziato_corsie10/.
# Spostamenti puntuali di singoli ostacoli applicati per ultimi, dopo DISTANZIA: {nome: (dx, dy)} [m].
# Multi5 L3 (3 ottobre 2026, scelta dell'utente): o1 -2 m in x apre la tasca o1-o21-o31 dove il
# drone 0 si fermava a t~94 s (stallo a ~71 m dal goal in 9 run su 13 con raggio 0,9).
# 3 ottobre ~20:20: l'utente ha chiesto di rimettere o1 nella posizione originale (con o1 spostato:
# 6 successi su 23, serie archiviata in mondoMulti5_L3/archivio_raggio090_o1spostato/).
SPOSTA_OSTACOLI = {}
DRONI = {(5, 3): ([-24.0, -12.0, 0.0, 12.0, 24.0], [7.0, 9.5, 7.0, 9.5, 7.0])}

COMUNI = ['--only-vert', '--parallel-lanes', '--tank-height', '20', '--tank-half-y', '60',
          '--spawn-z-min', '8', '--spawn-z-max', '8', '--spawn-y', '-50', '--target-y', '50',
          '--obs-z-center', '8', '--obs-height-min', '11', '--obs-height-max', '16',
          '--obs-base-z', '0', '--spread', '12', '--min-sep', '7.0']

SCENARI = {
    1: {'nome': '1 drone', 'vasca': '30x120x20 m', 'world_number': '7',
        'args': ['--n-drones', '1', '--tank-half-x', '15', '--path-corridor-w', '6.0',
                 '--wall-clear', '4.0'],
        'wall_clear': 4.0, 'spawn_clear': 9.07,   # 9,07 = valore reale del Livello 1
        # (casuali, di percorso, seed, file, scala del raggio)
        'livelli': {1: (24, 6, 219401, 'worlds/mondoCorridoio.sdf', 1.0),
                    2: (28, 7, 219430, 'worlds/mondoCorridoio_L2.sdf', 0.7),
                    3: (32, 8, 219576, 'worlds/mondoCorridoio_L3.sdf', 0.85)}},
    5: {'nome': '5 droni', 'vasca': '60x120x20 m', 'world_number': '9',
        'args': ['--n-drones', '5', '--tank-half-x', '30', '--path-corridor-w', '5.0',
                 '--lane-slots', '5'],
        'wall_clear': 6.0, 'spawn_clear': 12.0,
        'livelli': {1: (26, 14, 282075, 'worlds/mondoMulti5.sdf', 1.0),
                    2: (33, 17, 282093, 'worlds/mondoMulti5_L2.sdf', 1.0),
                    3: (39, 21, 282175, 'worlds/mondoMulti5_L3.sdf', 0.9)}},
}


def selezione(argv):
    scen = [int(argv[0])] if argv else list(SCENARI)
    liv = [int(argv[1])] if len(argv) > 1 else [1, 2, 3]
    return [(s, l) for s in scen for l in liv]


def riscala_raggi(sdf, fattore):
    """Moltiplica per `fattore` il raggio dei cilindri del modello `ostacoli` (posizioni invariate)."""
    t = Path(sdf).read_text()
    i = t.index('<model name="ostacoli">')
    j = t.index('</model>', i)
    corpo = re.sub(r'<radius>([0-9.]+)</radius>',
                   lambda m: f'<radius>{float(m.group(1)) * fattore:.2f}</radius>', t[i:j])
    Path(sdf).write_text(t[:i] + corpo + t[j:])


def genera_in(s, l, dest):
    sc = SCENARI[s]
    n_rand, n_path, seed, _, scala = sc['livelli'][l]
    # Livelli con posizioni generate a un'altra scala e raggi riscalati dopo (vedi POSIZIONI_DA).
    scala_pos = POSIZIONI_DA.get((s, l), scala)
    cmd = [sys.executable, str(ROOT / 'scripts/generate_world.py'), sc['world_number'], *COMUNI,
           *sc['args'], '--n-obstacles', str(n_rand), '--path-obstacles', str(n_path),
           '--seed', str(seed), '--output', str(dest)]
    if scala_pos != 1.0:
        cmd[-2:-2] = ['--obs-radius-scale', str(scala_pos)]
    subprocess.run(cmd, check=True, capture_output=True)
    if scala_pos != scala:
        riscala_raggi(dest, scala / scala_pos)
    if (s, l) in DRONI:
        from sposta_droni import sposta
        sposta(dest, dest, *DRONI[(s, l)])
    if (s, l) in DISTANZIA:
        from distanzia_ostacoli import distanzia
        sep, parete = DISTANZIA[(s, l)]
        # 0,1 m oltre spawn_clear: verifica_mondo.py confronta con >= e i decimali sono arrotondati
        distanzia(dest, dest, sep, parete, sc['spawn_clear'] + 0.1)
    for nome, (dx, dy) in SPOSTA_OSTACOLI.get((s, l), {}).items():
        testo = Path(dest).read_text()
        m = re.search(rf'(<link name="{nome}">\s*<pose>)([^<]+)(</pose>)', testo)
        v = m.group(2).split()
        v[0], v[1] = f'{float(v[0]) + dx:.2f}', f'{float(v[1]) + dy:.2f}'
        Path(dest).write_text(testo[:m.start(2)] + ' '.join(v) + testo[m.end(2):])


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ('elenco', 'mondo', 'stallo', 'successi', 'genera', 'verifica'):
        print(__doc__)
        sys.exit(2)
    azione, argv = sys.argv[1], sys.argv[2:]
    if azione == 'mondo':
        print(SCENARI[int(argv[0])]['livelli'][int(argv[1])][3])
        return
    if azione == 'stallo':
        print(STALLO[int(argv[1])])
        return
    if azione == 'successi':
        print(SUCCESSI)
        return
    if azione == 'elenco':
        for s, sc in SCENARI.items():
            for l, (nr, npth, seed, f, scala) in sc['livelli'].items():
                print(f"{sc['nome']:8} vasca {sc['vasca']}  Livello {l}: {nr + npth:2} ostacoli "
                      f"({nr} casuali + {npth} di percorso)  raggio x{scala:g}  stallo {STALLO[l]} s  "
                      f"seed {seed}  {f}")
        return
    esito = 0
    for s, l in selezione(argv):
        sc = SCENARI[s]
        f = ROOT / sc['livelli'][l][3]
        if azione == 'genera':
            with tempfile.TemporaryDirectory() as tmp:
                nuovo = Path(tmp) / f.name
                genera_in(s, l, nuovo)
                if f.exists() and filecmp.cmp(nuovo, f, shallow=False):
                    print(f'{f.name}: gia\' presente e identico')
                elif l == 1:
                    # Il Livello 1 e' il mondo dei dati reali: mai sovrascriverlo.
                    print(f'{f.name}: ATTENZIONE, la rigenerazione non coincide; file NON toccato')
                    esito = 1
                elif f.exists():
                    print(f'{f.name}: esiste ed e\' diverso; file NON toccato (rimuoverlo a mano se voluto)')
                    esito = 1
                else:
                    f.write_bytes(nuovo.read_bytes())
                    print(f'{f.name}: generato')
        else:
            print(f'== {f.name} ({sc["nome"]}, Livello {l})')
            r = subprocess.run([sys.executable, str(ROOT / 'scripts/verifica_mondo.py'), str(f),
                                '--wall-clear', str(DISTANZIA[(s, l)][1] if (s, l) in DISTANZIA else sc['wall_clear']),
                                '--spawn-clear', str(sc['spawn_clear']), '--min-sep', '0',
                                '--min-collo', '0' if l == 1 or (s, l) in POSIZIONI_DA else str(MIN_COLLO)])
            esito |= r.returncode
    sys.exit(esito)


if __name__ == '__main__':
    main()
