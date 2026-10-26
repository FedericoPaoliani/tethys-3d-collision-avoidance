#!/usr/bin/env python3
"""Mondo dimostrativo per il video della tesi: COPIA di worlds/mondoCorridoio.sdf (Livello 1).

Differenze rispetto al Livello 1 (il file originale non viene modificato):
  * il modello `ostacoli` contiene un solo pilastro, sul percorso spawn -> goal;
  * si aggiunge `telecamera_presentazione`: un modello statico senza geometria (invisibile
    ai sonar, nessun effetto sulla fisica) con una o più telecamere Gazebo che salvano i
    frame PNG, mosso lungo l'asse del corridoio dal plugin SeguiDroneCamera.
Drone, sonar, vasca, fisica e controller restano quelli del Livello 1.

Uso: genera_mondo_demo.py {anteprima|finale} USCITA.sdf CARTELLA_FRAME
"""
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SORGENTE = ROOT / 'worlds/mondoCorridoio.sdf'
PLUGIN = Path(__file__).resolve().parent / 'telecamera_plugin/build/libSeguiDroneCamera.so'

# Pilastro unico: sul fondale, sulla rotta x=0 (leggermente decentrato), 35 m dopo lo spawn.
# altezza 12 m (i pilastri del L1 sono alti 11-16 m): la telecamera, a z=19,5, ci passa
# sopra a 7,5 m e la cima non copre l'inquadratura.
PILASTRO = dict(x=0.5, y=-15.0, raggio=1.40, altezza=12.0)

# Carrello: segue il drone lungo y (offset), x e z fissi.
# z=17: sopra la cima del pilastro (15 m), il carrello sull'asse x=0 non lo attraversa.
CARRELLO = dict(offset_y=-10.0, x=0.0, z=17.0, tau=1.0)


def guarda(dx, dy, dz):
    """rpy di una telecamera (asse ottico = +X) orientata lungo (dx, dy, dz)."""
    return 0.0, math.atan2(-dz, math.hypot(dx, dy)), math.atan2(dy, dx)


# Telecamere: posizione relativa al carrello, punto guardato (relativo al carrello), fov.
# Il drone devia a sinistra (x<0) del pilastro: le telecamere stanno a sinistra, così il
# pilastro resta sempre oltre il drone e non lo copre.
TELECAMERE = {
    # prova 1: sull'asse del corridoio (scartata: il pilastro copre il drone al sorpasso)
    'dietro_alta': dict(pos=(0, 0, 0), mira=(0, 18, -8.7)),
    'obliqua_dx': dict(pos=(7, 2, -3), mira=(-1, 16, -8.7)),
    'obliqua_sx': dict(pos=(-7, 2, -3), mira=(1, 16, -8.7)),
    'campo_largo': dict(pos=(0, -4, 2), mira=(0, 20, -10.7)),
    # prova 2: tre quarti posteriore sinistra
    'tq_sx': dict(pos=(-7, 2, -2), mira=(0, 20, -8.7)),
    'tq_sx_alta': dict(pos=(-5, 0, 0), mira=(0, 22, -8.7)),
    'tq_sx_vicina': dict(pos=(-8, 5, -3), mira=(0, 18, -8.7)),
    'tq_sx_larga': dict(pos=(-9, -2, 1), mira=(0, 22, -9), fov=1.4),
    # prova 3: inseguimento dall'alto (z=19,5, sotto la superficie), grandangolo
    'alto_sx': dict(pos=(-5, 4, 2.5), mira=(-1, 22, -9.2), fov=1.5),
    'alto_centro_sx': dict(pos=(-3, 2, 2.5), mira=(0, 20, -9.2), fov=1.5),
    'alto_ripido': dict(pos=(-4, 6, 2.5), mira=(-1.5, 18, -9.2), fov=1.6),
    'alto_tq_sx': dict(pos=(-8, 1, 2.5), mira=(0, 22, -9.2), fov=1.3),
    # prova 4 (finale): parallela al corridoio e centrata sulla traiettoria (x=-1,5),
    # così una rotta dritta appare verticale e la virata come rotazione del drone.
    'alto_centrale': dict(pos=(-1.5, 4, 2.5), mira=(-1.5, 20, -9.2), fov=1.5),
    # come alto_centrale ma 3 m più indietro e inclinata a 38°: il drone resta nel quadro
    # anche da fermo (alto_centrale lo taglia sul bordo inferiore). Usata per il video.
    'alto_centrale_v2': dict(pos=(-1.5, 1, 2.5), mira=(-1.5, 16, -9.2), fov=1.5),
}

MODI = {
    'anteprima': dict(w=640, h=360, hz=5, camere=None),
    'finale': dict(w=1920, h=1080, hz=30, camere=None),  # camere scelte da riga di comando
}


def main():
    modo, uscita, frame_dir = sys.argv[1], Path(sys.argv[2]), Path(sys.argv[3])
    cfg = MODI[modo]
    camere = cfg['camere'] or sys.argv[4].split(',')

    tree = ET.parse(SORGENTE)
    world = tree.getroot().find('world')

    ostacoli = world.find("model[@name='ostacoli']")
    for link in list(ostacoli.findall('link')):
        ostacoli.remove(link)
    p = PILASTRO
    link = ET.SubElement(ostacoli, 'link', name='p0')
    ET.SubElement(link, 'pose').text = f"{p['x']} {p['y']} {p['altezza']/2} 0 0 0"
    for tag, name in (('collision', 'c'), ('visual', 'v')):
        el = ET.SubElement(link, tag, name=name)
        cyl = ET.SubElement(ET.SubElement(el, 'geometry'), 'cylinder')
        ET.SubElement(cyl, 'radius').text = str(p['raggio'])
        ET.SubElement(cyl, 'length').text = str(p['altezza'])
        if tag == 'visual':
            # stesso stile dei pilastri del generatore (tinta + 40% emissive)
            mat = ET.SubElement(el, 'material')
            ET.SubElement(mat, 'ambient').text = '0.950 0.300 0.080 1'
            ET.SubElement(mat, 'diffuse').text = '0.950 0.300 0.080 1'
            ET.SubElement(mat, 'specular').text = '0.25 0.25 0.25 1'
            ET.SubElement(mat, 'emissive').text = '0.380 0.120 0.032 1'

    c = CARRELLO
    rig = ET.SubElement(world, 'model', name='telecamera_presentazione')
    ET.SubElement(rig, 'static').text = 'true'
    ET.SubElement(rig, 'pose').text = f"{c['x']} {-50 + c['offset_y']} {c['z']} 0 0 0"
    rig_link = ET.SubElement(rig, 'link', name='link')
    for nome in camere:
        t = TELECAMERE[nome]
        (px, py, pz), (mx, my, mz) = t['pos'], t['mira']
        r, pi, ya = guarda(mx - px, my - py, mz - pz)
        s = ET.SubElement(rig_link, 'sensor', name=nome, type='camera')
        ET.SubElement(s, 'pose').text = f'{px} {py} {pz} {r:.4f} {pi:.4f} {ya:.4f}'
        ET.SubElement(s, 'always_on').text = 'true'
        ET.SubElement(s, 'update_rate').text = str(cfg['hz'])
        ET.SubElement(s, 'topic').text = f'/presentazione/{nome}'
        cam = ET.SubElement(s, 'camera')
        ET.SubElement(cam, 'horizontal_fov').text = str(t.get('fov', 1.15))
        img = ET.SubElement(cam, 'image')
        ET.SubElement(img, 'width').text = str(cfg['w'])
        ET.SubElement(img, 'height').text = str(cfg['h'])
        ET.SubElement(img, 'format').text = 'R8G8B8'
        clip = ET.SubElement(cam, 'clip')
        ET.SubElement(clip, 'near').text = '0.1'
        ET.SubElement(clip, 'far').text = '250'
        ET.SubElement(cam, 'anti_aliasing').text = '4'
        save = ET.SubElement(cam, 'save', enabled='true')
        ET.SubElement(save, 'path').text = str(frame_dir / nome)
    plug = ET.SubElement(rig, 'plugin', filename=str(PLUGIN), name='demo::SeguiDroneCamera')
    for k in ('offset_y', 'x', 'z', 'tau'):
        ET.SubElement(plug, k).text = str(c[k])
    ET.SubElement(plug, 'target').text = 'tethys'

    uscita.parent.mkdir(parents=True, exist_ok=True)
    testo = ET.tostring(tree.getroot(), encoding='unicode')
    # I commenti di testa (Spawn:, Tank:) servono agli script cervello_*: si riportano.
    testa = ''.join(l for l in SORGENTE.read_text().splitlines(True)[:6]
                    if l.startswith(('<!-- Spawn:', '<!-- Tank:')))
    testa += (f"<!-- DEMO VIDEO TESI: copia di mondoCorridoio.sdf con 1 pilastro "
              f"(x={p['x']}, y={p['y']}, r={p['raggio']}) e telecamere {','.join(camere)} -->\n")
    uscita.write_text('<?xml version="1.0"?>\n' + testa + testo + '\n')
    print(uscita)


if __name__ == '__main__':
    main()
