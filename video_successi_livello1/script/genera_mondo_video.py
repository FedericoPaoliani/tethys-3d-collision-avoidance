#!/usr/bin/env python3
"""Mondo per il video: COPIA di un mondo definitivo con in più una telecamera.

Il file originale in worlds/ non viene modificato. L'unica aggiunta è il modello
`telecamera_video`: statico, SENZA geometria (non ha collisioni e non ha visual, quindi i
sonar non lo vedono e la fisica non cambia), con un sensore `camera` che salva i frame PNG.
Drone, ostacoli, sonar, vasca, fisica e plugin restano identici.

Uso: genera_mondo_video.py {1drone|5droni} SORGENTE.sdf USCITA.sdf CARTELLA_FRAME HZ
"""
import math
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

PLUGIN = Path(__file__).resolve().parent / 'telecamera_plugin/build/libTelecameraVideo.so'


def guarda(dx, dy, dz):
    """rpy di una telecamera (asse ottico = +X) orientata lungo (dx, dy, dz)."""
    return 0.0, math.atan2(-dz, math.hypot(dx, dy)), math.atan2(dy, dx)


# 1 drone: carrello che insegue il drone (filtro tau 1,5 s), 3 m a sinistra, 8 m dietro, a
# quota 26 m (sopra la superficie semitrasparente a 20 m e la cima dei pilastri, al massimo 16 m:
# nessun pilastro copre il quadro da vicino). Orientamento fisso verso +y, inclinato di ~52°: il drone
# sta nella parte bassa del quadro e gli ostacoli davanti restano visibili.
UN_DRONE = dict(rig=dict(offset_x=-3.0, offset_y=-8.0, z=26.0, tau=1.5, target='tethys'),
                mira=(3.0, 14.0, -18.0), fov=1.40)
# 5 droni: telecamera fissa sopra la superficie (0, -80, 80), dietro la linea di partenza, inclinata
# di 51°: inquadra tutta la vasca 60×120 m, partenze (y=-50) e goal (y=+50) con margine.
CINQUE = dict(pos=(0.0, -80.0, 80.0), mira_ang=51.0, fov=1.56)


def main():
    modo, sorgente, uscita, frame_dir, hz = sys.argv[1:6]
    sorgente, uscita, frame_dir = Path(sorgente), Path(uscita), Path(frame_dir)
    tree = ET.parse(sorgente)
    world = tree.getroot().find('world')

    rig = ET.SubElement(world, 'model', name='telecamera_video')
    ET.SubElement(rig, 'static').text = 'true'
    link = ET.SubElement(rig, 'link', name='link')
    s = ET.SubElement(link, 'sensor', name='camera_video', type='camera')
    if modo == '1drone':
        c = UN_DRONE
        r = c['rig']
        ET.SubElement(rig, 'pose').text = f"{r['offset_x']} {-50 + r['offset_y']} {r['z']} 0 0 0"
        ro, pi, ya = guarda(*c['mira'])
        fov = c['fov']
    else:
        c = CINQUE
        x, y, z = c['pos']
        ET.SubElement(rig, 'pose').text = f'{x} {y} {z} 0 0 0'
        ro, pi, ya = 0.0, math.radians(c['mira_ang']), math.pi / 2
        fov = c['fov']
    ET.SubElement(s, 'pose').text = f'0 0 0 {ro:.4f} {pi:.4f} {ya:.4f}'
    ET.SubElement(s, 'always_on').text = 'true'
    ET.SubElement(s, 'update_rate').text = str(hz)
    ET.SubElement(s, 'topic').text = '/video/camera_video'
    cam = ET.SubElement(s, 'camera')
    ET.SubElement(cam, 'horizontal_fov').text = str(fov)
    img = ET.SubElement(cam, 'image')
    # risoluzione del sensore (default 1280x720 dal tentativo 10 della campagna a 1 drone, per
    # ridurre il carico di rendering; il montaggio ingrandisce a 1920x1080)
    ET.SubElement(img, 'width').text = os.environ.get('VIDEO_W', '1280')
    ET.SubElement(img, 'height').text = os.environ.get('VIDEO_H', '720')
    ET.SubElement(img, 'format').text = 'R8G8B8'
    clip = ET.SubElement(cam, 'clip')
    ET.SubElement(clip, 'near').text = '0.1'
    ET.SubElement(clip, 'far').text = '300'
    ET.SubElement(cam, 'anti_aliasing').text = '4'
    save = ET.SubElement(cam, 'save', enabled='true')
    ET.SubElement(save, 'path').text = str(frame_dir)
    if modo == '1drone':
        plug = ET.SubElement(rig, 'plugin', filename=str(PLUGIN), name='video::TelecameraVideo')
        for k, v in UN_DRONE['rig'].items():
            ET.SubElement(plug, k).text = str(v)

    # ElementTree perde i commenti: quelli di testa (Spawn:, Drone N spawn:, Tank:) servono
    # agli script e vengono riportati.
    testa = ''.join(l for l in sorgente.read_text().splitlines(True)[:12]
                    if l.startswith('<!--'))
    testa += f'<!-- VIDEO: copia di {sorgente.name} con il solo modello telecamera_video in più -->\n'
    uscita.parent.mkdir(parents=True, exist_ok=True)
    uscita.write_text('<?xml version="1.0" ?>\n' + testa + ET.tostring(tree.getroot(), encoding='unicode') + '\n')
    print(uscita)


if __name__ == '__main__':
    main()
