#!/usr/bin/env python3
"""Monta il video di una missione dai frame della telecamera Gazebo.

- Velocità di riproduzione UNIFORME (nessun taglio): un fattore intero scelto perché la parte
  simulata stia entro la durata massima; indicato sempre in sovrimpressione.
- Ogni fotogramma mostra il tempo simulato del frame sorgente.
- Cartello iniziale (scenario) e finale (esito): fotogrammi fermi, dichiarati come tali.
- Tutti i numeri vengono da summary.json e telemetry.csv della stessa esecuzione.
- 5 droni: etichetta con il numero di ogni drone, proiettando la sua posizione registrata in
  telemetria con il modello della telecamera fissa (stessi parametri di genera_mondo_video.py).

Uso: monta_video.py {1drone|5droni} CARTELLA_FRAME CARTELLA_RUN USCITA.mp4 TITOLO MAX_S HZ NOTA
NOTA: riga sul tentativo selezionato (es. "tentativo 4 di 4, selezionato perché riuscito").
"""
import bisect, csv, json, math, os, re, subprocess, sys, tempfile
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

modo, frame_dir, run_dir, uscita, titolo, max_s, hz, nota = sys.argv[1:9]
frame_dir, run_dir, uscita = Path(frame_dir), Path(run_dir), Path(uscita)
max_s, hz = float(max_s), float(hz)
FPS, W, H = 30, 1920, 1080
SCRIPT = Path(__file__).resolve().parent
F = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
FB = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
f_t, f_m, f_s, f_big = (ImageFont.truetype(FB, 40), ImageFont.truetype(F, 32),
                       ImageFont.truetype(F, 26), ImageFont.truetype(FB, 64))

s = json.load(open(run_dir / 'summary.json'))
meta = json.load(open(run_dir / 'metadata.json'))

# --- frame e tempi simulati (indice nel nome del file; tempo = (indice+1)/hz) --------------
frames = sorted(((int(re.search(r'_(\d+)\.png$', p.name).group(1)), p) for p in frame_dir.glob('*.png')))
tempi = [(i + 1) / hz for i, _ in frames]
if (frame_dir.parent / 'tempi_frame.csv').exists():   # tempi registrati dall'iscritto
    righe = list(csv.DictReader(open(frame_dir.parent / 'tempi_frame.csv')))
    if righe:
        reg = {int(r['indice']): float(r['sim_t']) for r in righe}
        tempi = [reg.get(i, t) for (i, _), t in zip(frames, tempi)]

if modo == '1drone':
    fine_sim = float(s.get('measured_sim_s') or tempi[-1])
else:
    fine_sim = float(s.get('mission_sim_s') or tempi[-1])
fine_sim = min(fine_sim, tempi[-1])
velocita = max(1, math.ceil(fine_sim / (max_s - 10)))   # 10 s di cartelli fermi

# --- telemetria (per le etichette dei droni) --------------------------------------------
tel = {}
for r in csv.DictReader(open(run_dir / 'telemetry.csv')):
    d = r.get('drone', 'tethys')
    tel.setdefault(d, []).append((float(r['sim_t']), float(r['x']), float(r['y']), float(r['z'])))


def posizione(d, t):
    v = tel[d]
    k = bisect.bisect_left([a[0] for a in v], t)
    return v[min(k, len(v) - 1)][1:]


# telecamera fissa 5 droni: pos (0,-80,80), yaw 90°, pitch 51° verso il basso, hfov 1,56
C = (0.0, -80.0, 80.0); PITCH = math.radians(51.0); HFOV = 1.56
FX = (W / 2) / math.tan(HFOV / 2)


def proietta(x, y, z):
    dx, dy, dz = x - C[0], y - C[1], z - C[2]
    # assi camera: avanti = (0, cos p, -sin p), destra = (1,0,0), su = (0, sin p, cos p)
    av = dy * math.cos(PITCH) - dz * math.sin(PITCH)
    su = dy * math.sin(PITCH) + dz * math.cos(PITCH)
    if av <= 0.1:
        return None
    return W / 2 + FX * dx / av, H / 2 - FX * su / av


# --- esito (solo dai log) -----------------------------------------------------------------
def esito_righe():
    if modo == '1drone':
        ok = bool(s.get('goal'))
        r = [('SUCCESSO VERIFICATO: 1/1 al goal' if ok else 'GOAL NON RAGGIUNTO'),
             f"motivo di fine: {s.get('stop_reason')}",
             f"tempo simulato all'arrivo: {float(s.get('measured_sim_s', 0)):.1f} s   percorso: {float(s.get('distance_m', 0)):.1f} m",
             (f"goal dichiarato dal controller a {float(s['controller_goal_dist_m']):.2f} m dal goal   "
              if s.get('controller_goal_dist_m') is not None else
              f"distanza finale dal goal: {float(s.get('final_goal_dist_m', 0)):.1f} m   ") +
             f"contatti fisici: {s.get('physical_contact_events')}"]
        return ok, r
    ok = bool(s.get('success'))
    r = [('SUCCESSO VERIFICATO: 5/5 al goal' if ok else f"NON RIUSCITA: {s.get('goals')}/5 al goal"),
         f"motivo di fine: {s.get('stop_reason')}   tempo simulato all'ultimo arrivo: {float(s.get('mission_sim_s', 0)):.1f} s",
         f"contatti fisici: {s.get('physical_contact_events')} (fra droni: {s.get('drone_drone_contact_events')})"]
    for d, v in sorted(s['drones'].items()):
        st = 'goal' if v.get('goal') else v.get('state')
        extra = ' (dopo lo stallo: non conta)' if v.get('goal_after_stall') else ''
        t = v.get('end_sim_t')
        if v.get('goal'):
            r.append(f"drone {d[-1]}: goal dichiarato dal controller a {float(t):.1f} s simulati, "
                     f"a {float(v['controller_goal_dist_m']):.2f} m dal goal")
        else:
            r.append(f"drone {d[-1]}: {st}{extra}" + (f" a {float(t):.0f} s" if t else '') +
                     f", minimo dal goal {float(v.get('min_goal_dist_m') or 0):.1f} m")
    return ok, r


def riquadro(dr, xy, testo, font, pad=12, alpha=170):
    x, y = xy
    b = dr.textbbox((x, y), testo, font=font)
    dr.rectangle((b[0] - pad, b[1] - pad, b[2] + pad, b[3] + pad), fill=(0, 0, 0, alpha))
    dr.text((x, y), testo, font=font, fill=(255, 255, 255, 255))


def base(img):
    if img.size != (W, H):
        img = img.resize((W, H), Image.LANCZOS)     # sensore a 1280x720 -> 1920x1080
    return img.convert('RGBA')


def sovrimpressione(img, t):
    o = Image.new('RGBA', (W, H), (0, 0, 0, 0)); dr = ImageDraw.Draw(o)
    y1, y2 = (28, 98) if modo == '1drone' else (H - 215, H - 145)   # 5 droni: in alto ci sono i goal
    riquadro(dr, (30, y1), titolo, f_t)
    riquadro(dr, (30, y2), f"Velocità {velocita}× rispetto al tempo simulato  ·  tempo simulato {t:6.1f} s", f_m)
    riquadro(dr, (30, H - 60), f'{nota}. Nuova esecuzione, separata dalle campagne della tesi; '
             'configurazione finale, codice ee1345e.', f_s, pad=8)
    if modo == '5droni':
        for d, v in sorted(meta.get('drones', {}).items()):
            g = proietta(*v['target'])
            if g:
                dr.ellipse((g[0] - 9, g[1] - 9, g[0] + 9, g[1] + 9), outline=(255, 80, 80, 255), width=3)
                dr.text((g[0] + 12, g[1] - 16), f'G{d[-1]}', font=f_s, fill=(255, 120, 120, 255))
        for d in sorted(tel):
            info = s.get('drones', {}).get(d, {})
            if info.get('goal') and info.get('end_sim_t') is not None and t >= float(info['end_sim_t']):
                continue                     # arrivato: l'harness lo sposta fuori vasca
            x, y, z = posizione(d, t)
            if abs(x) > 31 or abs(y) > 61:
                continue                     # arrivato e spostato fuori vasca dall'harness
            p = proietta(x, y, z)
            if p:
                riquadro(dr, (p[0] - 10, p[1] - 70), d[-1], f_m, pad=6)
                dr.line((p[0], p[1] - 30, p[0], p[1] - 8), fill=(255, 255, 255, 230), width=3)
    return Image.alpha_composite(base(img), o).convert('RGB')


def cartello(img, righe, intestazione):
    o = Image.new('RGBA', (W, H), (0, 0, 0, 165)); dr = ImageDraw.Draw(o)
    y = 300
    fb = f_big
    for size in (64, 56, 48, 42, 36):
        fb = ImageFont.truetype(FB, size)
        if dr.textlength(intestazione, font=fb) <= W - 240:
            break
    dr.text((120, y), intestazione, font=fb, fill=(255, 255, 255, 255)); y += 110
    for r in righe:
        dr.text((120, y), r, font=f_m, fill=(235, 235, 235, 255)); y += 52
    return Image.alpha_composite(base(img), o).convert('RGB')


tmp = Path(tempfile.mkdtemp(prefix='montaggio_', dir=str(SCRIPT.parent / 'grezzi')))
elenco = []


def salva(img, volte=1):
    p = tmp / f'{len(elenco):06d}.png'
    img.save(p, compress_level=1)
    elenco.extend([str(p)] * volte)          # i cartelli fermi riusano lo stesso file


primo = Image.open(frames[0][1]).convert('RGB')
intro = [f"Mondo: {Path(meta.get('world_source', '')).name or 'n.d.'} (copia con telecamera)",
         f"Droni: {meta.get('n_drones', 1)}   ostacoli: {meta.get('obstacle_count', 'n.d.')}",
         'Successo = ogni drone nella sfera di 3 m del goal prima dello stallo (60 s simulati',
         'senza avvicinarsi di 1 m). Nuova esecuzione, non una prova storica della tesi.',
         f"{nota}: video SELEZIONATO, non mostra il tasso di successo.",
         f"Velocità {velocita}× rispetto al tempo simulato, senza tagli (fotogrammi della",
         'telecamera di Gazebo presi in tempo simulato).',
         '(cartello fermo: 4 s)']
c = cartello(primo, intro, titolo)
salva(c, 4 * FPS)

# corpo: un fotogramma ogni velocita/FPS secondi simulati, frame sorgente più vicino
k, t = 0, tempi[0]
ultimo = primo
while t <= fine_sim + 1e-6:
    j = min(bisect.bisect_left(tempi, t), len(tempi) - 1)
    if j > 0 and abs(tempi[j - 1] - t) < abs(tempi[j] - t):
        j -= 1
    ultimo = Image.open(frames[j][1]).convert('RGB')
    salva(sovrimpressione(ultimo, tempi[j]))
    k += 1
    t = tempi[0] + k * velocita / FPS

ok, righe = esito_righe()
c = cartello(sovrimpressione(ultimo, min(t, fine_sim)), righe + ['(cartello fermo: 6 s; valori da summary.json)'],
             'Esito verificato dai log')
salva(c, 6 * FPS)

lista = tmp / 'elenco.txt'
lista.write_text('\n'.join(elenco) + '\n')
pkt = tmp / 'video.pkt'
subprocess.run([str(SCRIPT / 'codificatore/build/codifica_h264'), str(lista), str(pkt), str(FPS), '12000000'], check=True)
uscita.parent.mkdir(parents=True, exist_ok=True)
subprocess.run(['python3', str(SCRIPT / 'codificatore/mux_mp4.py'), str(pkt), str(uscita), str(FPS), str(W), str(H)], check=True)
subprocess.run(['python3', str(SCRIPT / 'codificatore/mux_mp4.py'), '--verifica', str(uscita), str(pkt)], check=True)
info = dict(video=str(uscita), fotogrammi=len(elenco), durata_video_s=len(elenco) / FPS, velocita=velocita,
            sim_inizio_s=tempi[0], sim_fine_s=fine_sim, frame_sorgente=len(frames), esito_ok=ok,
            cartello_iniziale_s=4, cartello_finale_s=6, tagli='nessuno')
json.dump(info, open(uscita.with_suffix('.json'), 'w'), indent=1)
print(json.dumps(info))
for p in set(elenco):
    os.unlink(p)
pkt.unlink(); lista.unlink(); tmp.rmdir()
