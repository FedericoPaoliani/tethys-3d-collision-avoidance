#!/usr/bin/env python3
"""Genera il foglio dei risultati della campagna sperimentale definitiva.

Struttura (decisa il 30 settembre 2026): 2 scenari x 3 livelli di difficolta'
(configurazione dei mondi in scripts/livelli.py). Il test a 3 droni e' escluso dalla
campagna principale e va in un file separato.

Uso:
  python3 scripts/foglio_risultati.py > reports/RISULTATI_TEST.md
  python3 scripts/foglio_risultati.py --scartati > reports/TEST_3_DRONI_SCARTATO.md

Legge solo summary.json/metadata.json dei run e non modifica nessun dato. Ogni livello
accetta solo i run il cui metadata.json riporta l'hash del proprio mondo, quindi i
risultati di livelli diversi non possono mescolarsi. Tentativi validi del Livello 1:
- 1 drone: run 44-99 e 200-268 di reports/corridor_review (il run_100 e' stato sovrascritto:
  nessun dato, escluso);
- 5 droni: run 301-630 di reports/multi_review/mondoMulti5 (il 560, interrotto, e' in
  interrotti/; la campagna a criterio 3/5 e' in criterio_3su5_interrotta/).
Livelli 2 e 3: run >= 201 delle rispettive cartelle.

Arresto al 10° successo (SUCCESSI in scripts/livelli.py, deciso il 30 settembre 2026): ogni
livello e' riassunto fino al tentativo in cui arriva il 10° successo, in ordine di numero di
run; i tentativi successivi restano su disco ma non entrano nel foglio. Nei Livelli 1, raccolti
quando l'obiettivo era 20, il 10° successo e' al tentativo 62 (run_205) a 1 drone e al
tentativo 213 (run_513) a 5 droni.
"""
import glob
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from livelli import SUCCESSI  # noqa: E402
import blocchi  # noqa: E402

ROOT = Path('/home/federico/ros2_tethys/reports')


def _valido(p, sha, runs, stallo):
    s = json.load(open(p))
    m = json.load(open(Path(p).with_name('metadata.json')))
    # I run a 1 drone precedenti al 30 settembre non registrano stall_abort_s: sono tutti a 60 s.
    ok_stallo = stallo is None or m.get('stall_abort_s', 60) == stallo
    return s if (m['world_sha256'].startswith(sha) and runs(s['run']) and ok_stallo) else None


def singolo(cartella, sha, runs=lambda n: n >= 201, stallo=None):
    """Run a 1 drone (review_corridor.py)."""
    found = [(s, Path(p).parent) for p in glob.glob(str(ROOT / cartella / 'run_*/summary.json'))
             if (s := _valido(p, sha, runs, stallo))]
    return [(s['goal'], s['physical_contact_events'], s['measured_sim_s'], s['mission_wall_s'],
             f"{s['distance_m']:.1f}", d) for s, d in sorted(found, key=lambda x: x[0]['run'])]


def multi(cartella, sha, runs=lambda n: n >= 201, stallo=None):
    """Run multi-drone (review_multi.py); distanza per drone."""
    found = [(s, Path(p).parent) for p in glob.glob(str(ROOT / cartella / 'run_*/summary.json'))
             if (s := _valido(p, sha, runs, stallo))]
    return [(s['success'], s['physical_contact_events'], s['mission_sim_s'], s['mission_wall_s'],
             ' / '.join(f"{v['distance_m']:.1f}" for v in s['drones'].values()), d)
            for s, d in sorted(found, key=lambda x: x[0]['run'])]


def tronca(rows, successi=SUCCESSI):
    """Tentativi fino a quello del successo n. `successi` compreso (tutti se non ancora raggiunto)."""
    ok = 0
    for i, r in enumerate(rows, 1):
        ok += bool(r[0])
        if ok == successi:
            return rows[:i]
    return rows


def sezione(titolo, rows, unita, livello='##'):
    out = [f'{livello} {titolo}', '',
           f'| Prova | Collisioni | Tempo simulato (s) | Tempo reale (s) | Distanza {unita} |',
           '|------:|----------:|-------------------:|----------------:|------:|']
    for i, (ok, urti, sim, wall, dist, _) in enumerate(rows, 1):
        out.append(f"| {f'{i} ✓' if ok else i} | {urti} | {sim:.1f} | {wall:.1f} | {dist} |")
    n, ok = len(rows), sum(r[0] for r in rows)
    perc = f'{100 * ok / n:.1f}%' if n else 'n.d. (nessuna prova ancora eseguita)'
    out += ['', f'{livello}# Riepilogo', '', f'Tentativi totali: {n}  ', f'Successi: {ok}  ',
            f'Fallimenti: {n - ok}  ', f'Percentuale di successo: {perc}', '']
    # Controllo di coerenza: righe di tabella = tentativi dichiarati nel riepilogo.
    righe = sum(1 for l in out if l.startswith('| ') and not l.startswith('| Prova'))
    assert righe == n, (titolo, righe, n)
    return out


def principale():
    uno = [
        ('Livello 1 – 30 ostacoli', singolo('corridor_review', 'c20af7c59aa0',
                                            lambda n: 44 <= n <= 99 or 200 <= n <= 268)),
        ('Livello 2 – 35 ostacoli', singolo('corridor_review/mondoCorridoio_L2', '68b6dd96b887', stallo=60)),
        ('Livello 3 – 40 ostacoli', singolo('corridor_review/mondoCorridoio_L3', '6cf683405427', stallo=60)),
    ]
    cinque = [
        ('Livello 1 – 40 ostacoli', multi('multi_review/mondoMulti5', '4ad20a45d2cc',
                                          lambda n: 301 <= n <= 630)),
        ('Livello 2 – 50 ostacoli', multi('multi_review/mondoMulti5_L2', 'b4a3ec1463db', stallo=60)),
        ('Livello 3 – 60 ostacoli', multi('multi_review/mondoMulti5_L3', '29431aedc0a9', stallo=60)),
    ]
    # Tentativi validi su disco dei Livelli 1 (raccolti con obiettivo 20) e punto di arresto.
    assert (len(uno[0][1]), len(cinque[0][1])) == (125, 329), (len(uno[0][1]), len(cinque[0][1]))
    uno = [(t, tronca(r)) for t, r in uno]
    cinque = [(t, tronca(r)) for t, r in cinque]
    assert (len(uno[0][1]), len(cinque[0][1])) == (62, 213), (len(uno[0][1]), len(cinque[0][1]))
    doc = ['# RISULTATI TEST', '',
           'Campagna definitiva: 2 scenari × 3 livelli di difficoltà. Fra un livello e l\'altro cambia',
           'solo il numero di ostacoli (stessa vasca, stessi spawn e target, stesso drone e controller).',
           'Successo: tutti i droni al proprio goal prima dello stallo (60 s simulati senza avvicinarsi',
           'al goal di almeno 1 m, uguale per tutti i livelli). Prove riuscite',
           'contrassegnate con ✓. Nei test a 5 droni la distanza è per drone (ordine 0 / 1 / 2 / 3 / 4),',
           'il tempo si ferma quando ogni drone è arrivato o in stallo e un urto fra due droni è contato',
           'una volta per ciascun drone.', '',
           f'Ogni campagna si ferma al {SUCCESSI}° successo: i tentativi successivi non sono riportati.',
           'Il Livello 3 a 5 droni è stato portato per un periodo a 5 successi e poi riportato a 10 (4 ottobre 2026).',
           'I Livelli 1 erano stati proseguiti fino a 20 successi (125 tentativi a 1 drone, 329 a 5',
           'droni); qui sono riassunti fino al tentativo del 10° successo: n. 62 (run_205) a 1 drone,',
           'n. 213 (run_513) a 5 droni. I dati dei tentativi successivi restano su disco.', '']
    # Riepilogo di tutte le serie in cima al foglio (le tabelle complete sono molto lunghe).
    doc += ['## Riepilogo generale', '',
            '| Scenario | Livello | Ostacoli | Tentativi | Successi | Percentuale '
            '| Collisioni o blocchi davanti a un ostacolo (tentativi) | Collisioni o blocchi (%) '
            '| Tempo simulato medio (s) | Tempo reale medio (s) |',
            '|---|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for scenario, serie in (('1 drone', uno), ('5 droni', cinque)):
        for titolo, rows in serie:
            livello, ostacoli = titolo.split(' – ')
            n, ok = len(rows), sum(bool(r[0]) for r in rows)
            cb = blocchi.conta([r[5] for r in rows])
            perc = f'{100 * ok / n:.1f}%' if n else 'n.d.'
            media = f'{100 * cb / n:.1f}%' if n else 'n.d.'
            sim = f'{sum(r[2] for r in rows) / n:.1f}' if n else 'n.d.'
            wall = f'{sum(r[3] for r in rows) / n:.1f}' if n else 'n.d.'
            doc.append(f"| {scenario} | {livello} | {ostacoli.split()[0]} | {n} | {ok} | {perc} "
                       f"| {cb} | {media} | {sim} | {wall} |")
    doc += ['', 'Collisioni o blocchi davanti a un ostacolo = tentativi in cui almeno un drone ha avuto un',
            'contatto fisico (anche con un altro drone) oppure non è arrivato al goal ed è rimasto,',
            f'nell\'ultimo minuto prima dello stallo, con lo scafo a meno di {blocchi.SOGLIA_M:g} m da un pilastro o',
            'da una parete. La colonna "Collisioni" delle tabelle sotto conta invece solo i contatti fisici',
            '(a 5 droni somma su tutti i droni; un urto fra due droni conta una volta per ciascuno).',
            'Tempi medi = media delle colonne "Tempo simulato" e "Tempo',
            'reale" su tutti i tentativi della serie (riusciti e falliti).', '']
    doc += ['# 1 DRONE', '', 'Vasca 30×120×20 m (mondoCorridoio, mondoCorridoio_L2, mondoCorridoio_L3). Nei livelli',
           '2 il raggio degli ostacoli è ridotto al 70% (0,56–1,26 m invece di 0,8–1,8 m), nel livello 3',
           'all\'85% (0,69–1,52 m).', '']
    for titolo, rows in uno:
        doc += sezione(titolo, rows, '(m)')
    doc += ['# 5 DRONI', '', 'Vasca 60×120×20 m (mondoMulti5, mondoMulti5_L2, mondoMulti5_L3). Raggio standard nei livelli 1 e 2;',
           'nel livello 3 raggio al 90% (0,72–1,60 m), ostacoli distanziati (centri ≥ 9 m, fino a 3 m dalle pareti) e droni',
           'su corsie a 12 m (x = −24…+24) a quote alternate 7 / 9,5 m invece che a 10 m e tutti a 8 m.', '']
    for titolo, rows in cinque:
        doc += sezione(titolo, rows, 'per drone (m)')
    return '\n'.join(doc)


def scartati():
    vecchio = multi('multi_review/scartati_3_droni/mondoMulti3_60x120_40ost', '462b9827756e')
    nuovo = multi('multi_review/scartati_3_droni/mondoMulti3_45x120_35ost', '284655d05f27')
    assert (len(vecchio), len(nuovo)) == (30, 59), (len(vecchio), len(nuovo))
    doc = ['# TEST A 3 DRONI — PRELIMINARE / SCARTATO', '',
           'Escluso dalla campagna sperimentale principale (decisione del 30 settembre 2026): lo',
           'scenario non è dimensionato in modo coerente con gli altri e non va usato nel confronto',
           'né nelle statistiche finali. Dati reali conservati per completezza; distanza per drone',
           '(ordine 0 / 1 / 2). Dati in `reports/multi_review/scartati_3_droni/`.', '']
    doc += sezione('Vasca 60×120×20 m – 40 ostacoli', vecchio, 'per drone (m)')
    doc += sezione('Vasca 45×120×20 m – 35 ostacoli', nuovo, 'per drone (m)')
    return '\n'.join(doc)


if __name__ == '__main__':
    print(scartati() if '--scartati' in sys.argv else principale())
