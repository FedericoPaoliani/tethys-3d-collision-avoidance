#!/usr/bin/env python3
"""
generate_world.py — Generatore di mondi SDF casuali per Tethys LRAUV (120×120×35 m).

2 tipi di ostacoli:
  cyl_vert  : cilindro verticale (pilastro floor-to-ceiling)
  cyl_horiz : cilindro orizzontale (trave)

Spawn del drone casuale (x, y, z) con clearance 8 m da ogni ostacolo.
Seed deterministico: N * 31337 + 42 (riproducibile).

Uso:
  python3 generate_world.py 50 --n-drones 3 --n-obstacles 25 --parallel-lanes --output mondoMulti5.sdf
  python3 generate_world.py 8 --output mondo.sdf
  python3 generate_world.py 8 --seed 12345
  python3 generate_world.py 8 --preview                → senza drone
"""

import argparse
import math
import random
import re
import sys
from pathlib import Path

WORKSPACE_ROOT = Path(__file__).resolve().parent.parent
MODELS_DIR = WORKSPACE_ROOT / "src" / "lrauv_description" / "models"

# ── Dimensioni vasca ──────────────────────────────────────────────────────────
TANK_HALF    = 80.0   # pareti est/ovest a ±80 m (X)
TANK_HALF_Y  = 80.0   # pareti nord/sud a ±80 m (Y) — può essere > TANK_HALF per vasca rettangolare
TANK_HEIGHT  = 35.0   # altezza vasca [m]
TANK_Z_MIN   = 2.5
TANK_Z_MAX   = TANK_HEIGHT - 1.5   # = 33.5 m (ma i droni operano fino a 17 m)
WALL_CLEAR   = 6.0    # clearance minima da ogni parete
OBS_Z_MIN    = 3.0    # quota minima centro ostacolo
OBS_Z_MAX    = 15.0   # quota massima centro ostacolo
OBS_RADIUS_SCALE = 1.0  # fattore sul raggio dei cilindri verticali (--obs-radius-scale)
MIN_SEP      = 9.0    # distanza minima XY tra centri (passaggio ≥6 m con r=1.5)
SPAWN_CLEAR  = 12.0   # clearance minima XY tra spawn e qualsiasi ostacolo
SPAWN_Z_MIN  = 10.0   # quota spawn
SPAWN_Z_MAX  = 10.0   # quota spawn (fissa a z=10)

OBS_HEIGHT_MIN = 10.0  # altezza minima cyl_vert
OBS_HEIGHT_MAX = 18.0  # altezza massima cyl_vert
OBS_Z_CENTER   = 10.0  # quota centro pilastro (banda operativa drone z=10m)
OBS_BASE_CLEAR = 2.0   # stacco MINIMO della base del pilastro dal fondale [m]
OBS_BASE_Z     = None  # se impostata, ancora la base di OGNI pilastro a questa quota

# Vecchia palette a 8 tinte, ciclica: con 26 ostacoli si ripeteva tre volte e, peggio,
# le tinte calde sparivano del tutto. Lo <scene><ambient> dei mondi e' 0.0 0.6 0.7, cioe'
# con il rosso a zero: il termine ambientale annulla la componente rossa di ogni materiale
# e marrone, arancio, rosso, giallo e viola collassano tutti sullo stesso verde-azzurro.
# Sostituita da _obstacle_color(), che assegna una tinta diversa a ogni ostacolo e aggiunge
# una componente emissiva, l'unica che sopravvive a un ambiente colorato.
GOLDEN_ANGLE = 0.6180339887498949   # separa al massimo le tinte di indici consecutivi
EMISSIVE_FRACTION = 0.40            # 0 = solo luce riflessa (tinte illeggibili sott'acqua)


def _obstacle_color(idx: int) -> str:
    """Tinta distinta per l'ostacolo idx, come stringa SDF "r g b 1"."""
    import colorsys
    h = (idx * GOLDEN_ANGLE) % 1.0
    # saturazione e valore alternati: due ostacoli con tinte vicine restano comunque
    # distinguibili per chiarezza.
    sat = 0.90 if idx % 2 == 0 else 0.65
    val = 0.95 if idx % 3 else 0.72
    r, g, b = colorsys.hsv_to_rgb(h, sat, val)
    return f"{r:.3f} {g:.3f} {b:.3f} 1"

ALL_SHAPES = ['cyl_vert', 'cyl_horiz']


def _make_tank_template():
    SX   = TANK_HALF   * 2   # larghezza X
    SY   = TANK_HALF_Y * 2   # lunghezza Y (può essere > SX per vasca rettangolare)
    WX   = TANK_HALF
    WY   = TANK_HALF_Y
    H    = TANK_HEIGHT
    Hmid = H / 2.0
    Hceil = H + 0.25
    Smax = max(SX, SY)        # fondale/soffitto coprono l'intera pianta
    return f"""\
    <!-- Fondale -->
    <model name="fondale">
      <static>true</static>
      <pose>0 0 -0.25 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{SX} {SY} 0.5</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>{SX} {SY} 0.5</size></box></geometry>
          <material><ambient>0.5 0.4 0.3 1</ambient><diffuse>0.5 0.4 0.3 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Vasca {SX}×{SY}×{H} m -->
    <model name="vasca">
      <static>true</static>
      <link name="walls">
        <collision name="n_c"><pose>0 {WY} {Hmid} 0 0 0</pose><geometry><box><size>{SX} 0.5 {H}</size></box></geometry></collision>
        <visual name="n_v"><pose>0 {WY} {Hmid} 0 0 0</pose><geometry><box><size>{SX} 0.5 {H}</size></box></geometry><material><ambient>0 0.2 1 0.30</ambient><diffuse>0 0.2 1 0.30</diffuse></material></visual>
        <collision name="s_c"><pose>0 -{WY} {Hmid} 0 0 0</pose><geometry><box><size>{SX} 0.5 {H}</size></box></geometry></collision>
        <visual name="s_v"><pose>0 -{WY} {Hmid} 0 0 0</pose><geometry><box><size>{SX} 0.5 {H}</size></box></geometry><material><ambient>0 0.2 1 0.30</ambient><diffuse>0 0.2 1 0.30</diffuse></material></visual>
        <collision name="e_c"><pose>{WX} 0 {Hmid} 0 0 0</pose><geometry><box><size>0.5 {SY} {H}</size></box></geometry></collision>
        <visual name="e_v"><pose>{WX} 0 {Hmid} 0 0 0</pose><geometry><box><size>0.5 {SY} {H}</size></box></geometry><material><ambient>0 0.2 1 0.30</ambient><diffuse>0 0.2 1 0.30</diffuse></material></visual>
        <collision name="w_c"><pose>-{WX} 0 {Hmid} 0 0 0</pose><geometry><box><size>0.5 {SY} {H}</size></box></geometry></collision>
        <visual name="w_v"><pose>-{WX} 0 {Hmid} 0 0 0</pose><geometry><box><size>0.5 {SY} {H}</size></box></geometry><material><ambient>0 0.2 1 0.30</ambient><diffuse>0 0.2 1 0.30</diffuse></material></visual>
      </link>
    </model>

    <model name="superficie_acqua">
      <static>true</static>
      <pose>0 0 {H} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>{SX} {SY} 0.05</size></box></geometry>
          <material><ambient>0 0.4 0.8 0.20</ambient><diffuse>0 0.4 0.8 0.20</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="soffitto">
      <static>true</static>
      <link name="link">
        <pose>0 0 {Hceil} 0 0 0</pose>
        <collision name="collision"><geometry><box><size>{SX} {SY} 0.5</size></box></geometry></collision>
      </link>
    </model>"""


# Nota: TANK_TEMPLATE NON viene calcolato a livello modulo perché TANK_HEIGHT può essere
# aggiornato da CLI dopo l'import. _make_tank_template() viene chiamata dentro generate_world().


def get_obstacle_count(world_number: int) -> int:
    group = (world_number - 3) // 5
    return 10 + group * 5


def _make_target_marker(idx, x, y, z):
    """Sfera rossa semi-trasparente (visual only, no collision) al target del drone idx."""
    return f"""    <model name="target_{idx}">
      <static>true</static>
      <pose>{x:.2f} {y:.2f} {z:.2f} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <!-- Layer dedicato al bersaglio: i gpu_lidar lo escludono dalla propria
               visibility_mask, altrimenti la sfera del goal viene restituita come
               un ostacolo e il drone schiva il punto in cui deve arrivare. -->
          <visibility_flags>2</visibility_flags>
          <geometry><sphere><radius>3.0</radius></sphere></geometry>
          <material>
            <ambient>1.0 0.0 0.0 0.25</ambient>
            <diffuse>1.0 0.0 0.0 0.25</diffuse>
            <specular>0.5 0.0 0.0 0.25</specular>
          </material>
        </visual>
      </link>
    </model>"""


def _rand_spawn(rng):
    """Genera una posizione di spawn casuale con clearance dalle pareti."""
    lox = -TANK_HALF   + WALL_CLEAR + 2.0
    hix =  TANK_HALF   - WALL_CLEAR - 2.0
    loy = -TANK_HALF_Y + WALL_CLEAR + 2.0
    hiy =  TANK_HALF_Y - WALL_CLEAR - 2.0
    x = rng.uniform(lox, hix)
    y = rng.uniform(loy, hiy)
    z = rng.uniform(SPAWN_Z_MIN, SPAWN_Z_MAX)
    return x, y, z


def rand_pos(rng, half_size, placed_xy=None, spawn_xys=None, max_tries=60, spread_k=1):
    """Genera posizione XY casuale con:
    - clearance MIN_SEP da tutti gli ostacoli già piazzati
    - clearance SPAWN_CLEAR da tutti gli spawn/target dei droni

    spread_k > 1 attiva il campionamento "best candidate": invece di accettare il
    primo punto valido si estraggono fino a spread_k candidati validi e si tiene
    quello più lontano da tutto il resto. Il risultato è una distribuzione visibilmente
    uniforme invece che a grappoli — con il puro rifiuto casuale i grappoli sono la
    norma, non l'eccezione. Con spread_k=1 il comportamento è quello di prima.
    """
    lox = -TANK_HALF   + WALL_CLEAR + half_size
    hix =  TANK_HALF   - WALL_CLEAR - half_size
    loy = -TANK_HALF_Y + WALL_CLEAR + half_size
    hiy =  TANK_HALF_Y - WALL_CLEAR - half_size
    if lox >= hix:
        lox = -TANK_HALF   + WALL_CLEAR
        hix =  TANK_HALF   - WALL_CLEAR
    if loy >= hiy:
        loy = -TANK_HALF_Y + WALL_CLEAR
        hiy =  TANK_HALF_Y - WALL_CLEAR
    ref = list(placed_xy or []) + list(spawn_xys or [])
    best, best_d, found = None, -1.0, 0
    for _ in range(max_tries):
        x = rng.uniform(lox, hix)
        y = rng.uniform(loy, hiy)
        # clearance dagli altri ostacoli
        if placed_xy and any(math.hypot(x - px, y - py) < MIN_SEP for px, py in placed_xy):
            continue
        # clearance da tutti gli spawn e target dei droni
        if spawn_xys and any(math.hypot(x - sx, y - sy) < SPAWN_CLEAR for sx, sy in spawn_xys):
            continue
        if spread_k <= 1:
            return x, y
        d = min((math.hypot(x - px, y - py) for px, py in ref), default=float('inf'))
        if d > best_d:
            best, best_d = (x, y), d
        found += 1
        if found >= spread_k:
            break
    if best is not None:
        return best
    # Fallback anti-stallo. Con spread attivo non si accetta il primo punto a caso:
    # ignorare MIN_SEP qui è ciò che produce coppie di ostacoli quasi sovrapposte,
    # che a colpo d'occhio sembrano un ostacolo solo e lasciano vuoto il resto.
    if spread_k > 1 and ref:
        cands = [(rng.uniform(lox, hix), rng.uniform(loy, hiy)) for _ in range(max_tries * 4)]
        return max(cands, key=lambda c: min(math.hypot(c[0] - px, c[1] - py) for px, py in ref))
    return rng.uniform(lox, hix), rng.uniform(loy, hiy)


def rand_pos_in_corridor(rng, spawns, targets, corridor_half_w=8.0,
                         placed_xy=None, spawn_xys=None, max_tries=120, spread_k=1):
    """Piazza un ostacolo dentro il corridoio spawn→target di un drone casuale."""
    for _ in range(max_tries):
        idx = rng.randint(0, len(spawns) - 1)
        sx, sy, _ = spawns[idx]
        tx, ty, _ = targets[idx]
        t   = rng.uniform(0.12, 0.88)          # evita vicinanza a spawn/target
        cx  = sx + t * (tx - sx)
        cy  = sy + t * (ty - sy)
        off = rng.uniform(-corridor_half_w, corridor_half_w)
        x   = cx + off
        y   = cy + rng.uniform(-2.0, 2.0)
        # clamp dentro vasca con clearance
        x = max(-TANK_HALF   + WALL_CLEAR + 1, min(TANK_HALF   - WALL_CLEAR - 1, x))
        y = max(-TANK_HALF_Y + WALL_CLEAR + 1, min(TANK_HALF_Y - WALL_CLEAR - 1, y))
        if placed_xy and any(math.hypot(x-px, y-py) < MIN_SEP for px, py in placed_xy):
            continue
        if spawn_xys and any(math.hypot(x-sx2, y-sy2) < SPAWN_CLEAR for sx2, sy2 in spawn_xys):
            continue
        if placed_xy is not None:
            placed_xy.append((x, y))
        return x, y
    # Fallback anti-stallo. Restituire sempre il punto medio del percorso accatastava
    # tutti gli ostacoli di percorso nello stesso punto quando MIN_SEP era troppo grande
    # per la larghezza disponibile: si sceglie invece un punto a caso lungo la rotta.
    ref = list(placed_xy or []) + list(spawn_xys or [])
    def _on_route():
        i = rng.randint(0, len(spawns) - 1)
        sx, sy, _ = spawns[i]
        tx, ty, _ = targets[i]
        t = rng.uniform(0.15, 0.85)
        return (sx + t * (tx - sx) + rng.uniform(-corridor_half_w, corridor_half_w),
                sy + t * (ty - sy))
    if spread_k > 1 and ref:
        cands = [_on_route() for _ in range(max_tries)]
        best = max(cands, key=lambda c: min(math.hypot(c[0] - px, c[1] - py) for px, py in ref))
        if placed_xy is not None:
            placed_xy.append(best)
        return best
    sx, sy, _ = spawns[0]
    tx, ty, _ = targets[0]
    t = rng.uniform(0.15, 0.85)
    return (sx + t * (tx - sx) + rng.uniform(-corridor_half_w, corridor_half_w),
            sy + t * (ty - sy))


def _seg_point_dist_3d(px, py, pz, ax, ay, az, bx, by, bz):
    """Distanza minima 3D dal punto P al segmento AB."""
    dx, dy, dz = bx - ax, by - ay, bz - az
    len2 = dx*dx + dy*dy + dz*dz
    if len2 < 1e-10:
        return math.sqrt((px-ax)**2 + (py-ay)**2 + (pz-az)**2)
    t = max(0.0, min(1.0, ((px-ax)*dx + (py-ay)*dy + (pz-az)*dz) / len2))
    return math.sqrt((px-ax-t*dx)**2 + (py-ay-t*dy)**2 + (pz-az-t*dz)**2)


def _geom_cyl(r, l):
    return f"<cylinder><radius>{r:.2f}</radius><length>{l:.2f}</length></cylinder>"


def _material(color: str) -> str:
    """Materiale di un ostacolo.

    Il solo ambient+diffuse non basta: sott'acqua la luce direzionale arriva radente
    sui fianchi dei cilindri verticali, quindi il colore visibile e' dominato dal
    termine ambientale = ambiente della scena x ambient del materiale. Con l'ambiente
    a rosso zero ogni tinta calda si spegne. L'emissive non dipende dalle luci e
    mantiene la tinta leggibile; abbassare EMISSIVE_FRACTION per un effetto meno acceso.
    """
    r, g, b = [float(v) for v in color.split()[:3]]
    e = EMISSIVE_FRACTION
    return (f"<ambient>{color}</ambient><diffuse>{color}</diffuse>"
            f"<specular>0.25 0.25 0.25 1</specular>"
            f"<emissive>{r*e:.3f} {g*e:.3f} {b*e:.3f} 1</emissive>")


def _link(name, pose_str, geom_str, color):
    return f"""
      <link name="{name}">
        <pose>{pose_str}</pose>
        <collision name="c"><geometry>{geom_str}</geometry></collision>
        <visual name="v">
          <geometry>{geom_str}</geometry>
          <material>{_material(color)}</material>
        </visual>
      </link>"""


def _clamp_z(z, lo, hi):
    return max(lo, min(hi, z))


def generate_obstacle(rng, idx, shape, z_center=None, placed_xy=None, spawn_xy=None,
                      occupied_xyz=None, spread_k=1):
    """Genera un ostacolo SDF del tipo specificato.

    z_center:     quota suggerita (campionamento stratificato).
    placed_xy:    lista mutabile di (x,y) già piazzati (ostacoli precedenti).
    spawn_xy:     (x, y) spawn singolo drone (per mondi a 1 drone).
    occupied_xyz: lista di (x,y,z) di spawn+target di TUTTI i droni — usata da cyl_horiz
                  per controllare la clearance 3D sull'intera lunghezza del segmento.
    """
    color = _obstacle_color(idx)
    name  = f"o{idx}"

    if shape == 'cyl_vert':
        # OBS_RADIUS_SCALE scala il raggio dopo l'estrazione: la sequenza casuale resta la stessa.
        radius = rng.uniform(0.8, 1.8) * OBS_RADIUS_SCALE
        height = rng.uniform(OBS_HEIGHT_MIN, OBS_HEIGHT_MAX)
        # spawn_xy non usato (rimosso): ora si usa occupied_xyz convertito in XY list.
        spawn_xys = [(p[0], p[1]) for p in occupied_xyz] if occupied_xyz else None
        x, y   = rand_pos(rng, radius, placed_xy, spawn_xys, spread_k=spread_k)
        if placed_xy is not None:
            placed_xy.append((x, y))
        # Centra il pilastro alla quota operativa del drone (non floor-to-ceiling).
        # Questo lascia fondale (0→z_base) e soffitto (z_top→TANK_HEIGHT) sempre liberi:
        # DEPTH_LIMIT fondale/soffitto possono funzionare senza ostacoli che bloccano l'escape.
        half_h = height / 2.0
        # Con OBS_BASE_Z la base e' ancorata: tenere il centro fisso a OBS_Z_CENTER
        # lascia sospesi i pilastri piu' bassi, che e' esattamente l'effetto da evitare.
        z = (OBS_BASE_Z + half_h) if OBS_BASE_Z is not None else OBS_Z_CENTER
        z = min(z, TANK_Z_MAX - half_h)   # testa del pilastro sotto il soffitto
        if OBS_BASE_Z is None:
            z = max(z, half_h + OBS_BASE_CLEAR)   # base del pilastro sopra il fondale
        pose   = f"{x:.2f} {y:.2f} {z:.2f} 0.0000 0.0000 0.0000"
        return _link(name, pose, _geom_cyl(radius, height), color)

    elif shape == 'cyl_horiz':
        radius = rng.uniform(0.4, 0.8)
        length = rng.uniform(15.0, 40.0)
        half   = length / 2.0

        # Per il centro: usa l'intera area (il check sulle estremità filtra fuori-vasca)
        lo_x = -TANK_HALF   + WALL_CLEAR
        hi_x =  TANK_HALF   - WALL_CLEAR
        lo_y = -TANK_HALF_Y + WALL_CLEAR
        hi_y =  TANK_HALF_Y - WALL_CLEAR
        lo_z, hi_z = OBS_Z_MIN + radius, OBS_Z_MAX - radius
        # Calcola z_beam prima del loop per il controllo 3D
        z_beam = _clamp_z(z_center, lo_z, hi_z) if z_center is not None \
                 else (lo_z + hi_z) / 2.0

        x = y = yaw = None
        for _ in range(200):
            cx  = rng.uniform(lo_x, hi_x)
            cy  = rng.uniform(lo_y, hi_y)
            yw  = rng.uniform(0, math.pi)

            ax3 = cx - half * math.cos(yw); ay3 = cy - half * math.sin(yw)
            bx3 = cx + half * math.cos(yw); by3 = cy + half * math.sin(yw)

            # Entrambe le estremità devono stare dentro la vasca (con margine)
            wall_m = WALL_CLEAR
            if (min(ax3, bx3) < -TANK_HALF   + wall_m or
                    max(ax3, bx3) > TANK_HALF   - wall_m or
                    min(ay3, by3) < -TANK_HALF_Y + wall_m or
                    max(ay3, by3) > TANK_HALF_Y  - wall_m):
                continue

            # clearance centro-centro dagli ostacoli già piazzati
            if placed_xy and any(math.hypot(cx - px, cy - py) < MIN_SEP
                                 for px, py in placed_xy):
                continue

            # clearance 3D del SEGMENTO della trave da spawn e target
            if occupied_xyz:
                if any(_seg_point_dist_3d(sx, sy, sz,
                                         ax3, ay3, z_beam,
                                         bx3, by3, z_beam) < radius + SPAWN_CLEAR
                       for sx, sy, sz in occupied_xyz):
                    continue

            x, y, yaw = cx, cy, yw
            break

        if x is None:  # fallback anti-stallo: trave corta che entra sicuramente
            short = min(length, (TANK_HALF - WALL_CLEAR) * 1.2)
            half  = short / 2.0
            cx = rng.uniform(-TANK_HALF + WALL_CLEAR + half, TANK_HALF - WALL_CLEAR - half)
            cy = rng.uniform(-TANK_HALF + WALL_CLEAR + half, TANK_HALF - WALL_CLEAR - half)
            x, y, yaw = cx, cy, rng.uniform(0, math.pi)
            length = short

        if placed_xy is not None:
            placed_xy.append((x, y))
        z = _clamp_z(z_center, lo_z, hi_z) if z_center is not None else rng.uniform(lo_z, hi_z)
        pose = f"{x:.2f} {y:.2f} {z:.2f} 0.0000 1.5708 {yaw:.4f}"
        return _link(name, pose, _geom_cyl(radius, length), color)

    raise ValueError(f"Tipo sconosciuto: {shape}")


def _rand_positions(rng, n, existing_xy=None, label="pos",
                    z_min=SPAWN_Z_MIN, z_max=SPAWN_Z_MAX):
    """Genera n posizioni XY casuali con clearance SPAWN_CLEAR tra loro e da existing_xy."""
    positions = list(existing_xy) if existing_xy else []
    result = []
    for i in range(n):
        lo = -TANK_HALF + WALL_CLEAR + 2.0
        hi =  TANK_HALF - WALL_CLEAR - 2.0
        for _ in range(200):
            x = rng.uniform(lo, hi)
            y = rng.uniform(lo, hi)
            z = rng.uniform(z_min, z_max)
            if not any(math.hypot(x - px, y - py) < SPAWN_CLEAR for px, py in positions):
                positions.append((x, y))
                result.append((x, y, z))
                break
        else:
            x = rng.uniform(lo, hi)
            y = rng.uniform(lo, hi)
            z = rng.uniform(z_min, z_max)
            positions.append((x, y))
            result.append((x, y, z))
    return result


def _add_contact_sensors(model_xml: str) -> str:
    """Observe physical contacts on all collision-bearing links, using Fortress topics."""
    def add_sensor(match):
        link = match.group()
        names = re.findall(r'<collision\s+name="([^"]+)"', link)
        if not names or 'name="collision_contacts"' in link:
            return link
        collisions = ''.join(f'<collision>{name}</collision>' for name in names)
        sensor = ('<sensor name="collision_contacts" type="contact">'
                  '<always_on>true</always_on><update_rate>20</update_rate>'
                  f'<contact>{collisions}</contact></sensor>\n')
        return link.replace('</link>', sensor + '</link>')
    return re.sub(r'<link\s+name="[^"]+".*?</link>', add_sensor, model_xml, flags=re.S)


def _make_drone_flat(name: str, sx: float, sy: float, sz: float,
                     tethys_sdf: str) -> str:
    """Genera il blocco <model> completamente flat (nessun <include>) per il world SDF.

    Combina tethys/model.sdf (links, joints, 5 sonar) con i plugin essenziali di
    tethys_equipped (Thruster, JPC×2, LiftDrag×2, Hydrodynamics, BuoyancyEngine)
    in un unico XML senza nessun <include> o <experimental:params>.

    Questo garantisce che propeller_joint, vertical_fins_joint e horizontal_fins_joint
    siano già presenti nell'ECM quando i plugin Configure() vengono chiamati — il
    problema che causava il Thruster a non sottoscriversi in Fortress.

    La rotazione -90° attorno a Z (heredata da tethys_equipped) viene incorporata
    nella pose del modello: (sx, sy, sz, 0, 0, -π/2).
    """
    # Estrai inner content da tethys/model.sdf (tra <model name="tethys"> e </model>)
    start = tethys_sdf.index('<model name="tethys">') + len('<model name="tethys">')
    end   = tethys_sdf.rfind('</model>')
    inner = tethys_sdf[start:end]
    inner = _add_contact_sensors(inner)
    # The flat model must retain the ballast mass; otherwise it free-falls.
    inner += '''<joint name="drop_weight_joint" type="fixed">
      <parent>base_link</parent><child>drop_weight</child>
    </joint>'''

    # Sostituisci topic sonar hardcoded (/model/tethys/… → /model/name/…)
    inner = inner.replace('/model/tethys/', f'/model/{name}/')

    # I cinque gpu_lidar vedono tutto tranne il layer del bersaglio.
    inner = inner.replace('<lidar>', '<lidar>\n            <visibility_mask>4294967293</visibility_mask>')

    # Disabilita visualizzazione sonar (visualize=true disegna i raggi GPU nella GUI → enormemente
    # costoso con 10 droni × 5 sonar = 50 sensori; causa RT factor ~0.003)
    inner = inner.replace('<visualize>true</visualize>', '<visualize>false</visualize>')

    # Keep the frontal sensor at full resolution for obstacle detection.
    frontal_original = re.search(r'<sensor name="frontal_sonar".*?</sensor>', inner, re.S).group()
    frontal_original = frontal_original.replace('<update_rate>10</update_rate>', '<update_rate>5</update_rate>')

    # Riduce update rate sonar da 10 Hz a 2 Hz.
    # 1 Hz causava collisioni: il drone percorre ~2m sim tra aggiornamenti sonar → BRAKE_DIST=5m
    # lasciava solo 2 update prima della collisione. A 2 Hz il drone percorre 1m/update → 5 update
    # di margine → frenata affidabile. RT stimato ~26% (da ~52% a 1 Hz), accettabile con use_sim_time.
    inner = inner.replace('<update_rate>10</update_rate>', '<update_rate>2</update_rate>')

    # Riduce risoluzione sonar (~10× meno raggi GPU → RT factor stimato ~77% invece di ~8%)
    # Ordine critico: prima i valori univoci piccoli, poi i grandi che li generano.
    inner = inner.replace('<samples>15</samples>', '<samples>8</samples>')   # frontale vert 15→8
    inner = inner.replace('<samples>30</samples>', '<samples>15</samples>')  # frontale horiz 30→15
    inner = inner.replace('<samples>10</samples>', '<samples>5</samples>')   # lat vert + top 10→5
    inner = inner.replace('<samples>20</samples>', '<samples>10</samples>')  # lat horiz 20→10

    # propeller_joint: nessuna modifica necessaria. La pose di spawn del modello è
    # (sx, sy, sz, 0, 0, -π/2) → asse propulsore in model -X → thrust=world+Y.
    # Il nodo VFH usa yaw_bias=π (non π/2!) per ottenere state_.yaw=π/2 a spawn yaw=-π/2:
    #   state_.yaw = gazebo_yaw + yaw_bias = -π/2 + π = π/2 ✓ (direzione mondo +Y).
    # Con yaw_bias=π il wander pertuba intorno a π/2 → droni si muovono verso y=target. ✓

    inner = re.sub(r'<sensor name="frontal_sonar".*?</sensor>',
                   lambda _: frontal_original, inner, count=1, flags=re.S)

    # Elevator rotates about model Y; its LiftDrag plane must be XZ.
    inner = re.sub(r'(<joint name="horizontal_fins_joint".*?<xyz>).*?(</xyz>)',
                   r'\g<1>0 1 0\2', inner, count=1, flags=re.S)

    # Converti path relativi mesh/materiali in URI model:// risolti da IGN_GAZEBO_RESOURCE_PATH
    inner = inner.replace('<uri>meshes/tethys.dae</uri>',
                          '<uri>model://tethys/meshes/tethys.dae</uri>')
    inner = inner.replace('<uri>materials/scripts/</uri>',
                          '<uri>model://tethys/materials/scripts/</uri>')
    inner = inner.replace('<uri>materials/textures/</uri>',
                          '<uri>model://tethys/materials/textures/</uri>')

    # Fix path PBR texture (<albedo_map>, <normal_map>, <metalness_map>, <roughness_map>):
    # nel flat model, il SDF non è nel modello tethys, quindi i path relativi vengono
    # risolti rispetto alla directory del world file invece che del modello.
    # Usiamo path assoluto derivato da MODELS_DIR (già usato nel resto dello script).
    tethys_textures = str(MODELS_DIR / "tethys" / "materials" / "textures") + "/"
    inner = inner.replace('materials/textures/Tethys_', tethys_textures + 'Tethys_')

    # Plugin a livello modello (Hydrodynamics + BuoyancyEngine)
    dynamics_plugins = f"""\
      <plugin
        filename="libignition-gazebo-hydrodynamics-system.so"
        name="ignition::gazebo::systems::Hydrodynamics">
        <link_name>base_link</link_name>
        <disable_coriolis>true</disable_coriolis>
        <xDotU>-4.876161</xDotU>
        <yDotV>-126.324739</yDotV>
        <zDotW>-126.324739</zDotW>
        <kDotP>0</kDotP>
        <mDotQ>-33.46</mDotQ>
        <nDotR>-33.46</nDotR>
        <xUabsU>-6.2282</xUabsU>
        <xU>0</xU>
        <yVabsV>-601.27</yVabsV>
        <yV>0</yV>
        <zWabsW>-400.00</zWabsW>
        <zW>-30</zW>
        <kPabsP>-200.0</kPabsP>
        <kP>-30.0</kP>
        <mQabsQ>-600</mQabsQ>
        <mQ>-100</mQ>
        <nRabsR>-200.0</nRabsR>
        <nR>0</nR>
      </plugin>

      <plugin filename="libignition-gazebo-buoyancy-engine-system.so"
        name="ignition::gazebo::systems::BuoyancyEngine">
        <link_name>buoyancy_engine</link_name>
        <fluid_density>1025</fluid_density>
        <min_volume>0.000080</min_volume>
        <neutral_volume>0.0034</neutral_volume>
        <default_volume>0.0034</default_volume>
        <max_volume>0.004</max_volume>
        <max_inflation_rate>0.000003</max_inflation_rate>
      </plugin>
"""
    # Gazebo system plugins are model systems. Plugins inside <link> are
    # ignored: without hydrodynamics the configured damping is never applied.

    # Plugin a livello modello: Thruster, JPC×2, LiftDrag×2
    model_plugins = f"""
    <plugin
      filename="libignition-gazebo-thruster-system.so"
      name="ignition::gazebo::systems::Thruster">
      <joint_name>propeller_joint</joint_name>
      <thrust_coefficient>0.004312328425753156</thrust_coefficient>
      <fluid_density>1025</fluid_density>
      <propeller_diameter>0.2</propeller_diameter>
      <velocity_control>true</velocity_control>
    </plugin>

    <plugin
      filename="libignition-gazebo-joint-position-controller-system.so"
      name="ignition::gazebo::systems::JointPositionController">
      <joint_name>vertical_fins_joint</joint_name>
      <p_gain>10.0</p_gain>
      <i_gain>0.0</i_gain>
      <d_gain>0.5</d_gain>
    </plugin>

    <plugin
      filename="libignition-gazebo-joint-position-controller-system.so"
      name="ignition::gazebo::systems::JointPositionController">
      <joint_name>horizontal_fins_joint</joint_name>
      <p_gain>10.0</p_gain>
      <i_gain>0.0</i_gain>
      <d_gain>0.5</d_gain>
    </plugin>

    <plugin
      filename="libignition-gazebo-lift-drag-system.so"
      name="ignition::gazebo::systems::LiftDrag">
      <air_density>1025</air_density>
      <cla>60.0</cla>
      <cla_stall>-50.0</cla_stall>
      <cda>0.2</cda>
      <cda_stall>0.03</cda_stall>
      <alpha_stall>0.17</alpha_stall>
      <a0>0</a0>
      <area>0.08</area>
      <upward>0 1 0</upward>
      <forward>1 0 0</forward>
      <link_name>vertical_fins</link_name>
      <cp>0 0 0</cp>
    </plugin>

    <plugin
      filename="libignition-gazebo-lift-drag-system.so"
      name="ignition::gazebo::systems::LiftDrag">
      <air_density>1025</air_density>
      <cla>12.0</cla>
      <cla_stall>-1.1</cla_stall>
      <cda>0.2</cda>
      <cda_stall>0.03</cda_stall>
      <alpha_stall>0.40</alpha_stall>
      <a0>0</a0>
      <area>0.05</area>
      <upward>0 0 1</upward>
      <forward>1 0 0</forward>
      <link_name>horizontal_fins</link_name>
      <cp>0 0 0</cp>
    </plugin>
"""
    return (f'    <model name="{name}">\n'
            f'      <pose>{sx:.2f} {sy:.2f} {sz:.2f} 0 0 -1.5707963</pose>\n'
            f'{inner}{dynamics_plugins}'
            f'{model_plugins}'
            f'    </model>')


def generate_drone_models(n_drones: int):
    """Genera una directory modello per-drone in MODELS_DIR/tethys_N/.

    Ogni modello è una copia di tethys_equipped/model.sdf con:
      - <model name="tethys"> → <model name="tethys_N">
      - /model/tethys/        → /model/tethys_N/   (plugin topic e sensor topic)

    Questo garantisce che i plugin Thruster/JPC usino i topic corretti senza
    che Ignition Fortress debba rinominare i topic (cosa che non fa).
    """
    equipped_sdf_path = MODELS_DIR / "tethys_equipped" / "model.sdf"
    equipped_sdf = equipped_sdf_path.read_text()

    for i in range(n_drones):
        name = f"tethys_{i}"
        drone_dir = MODELS_DIR / name
        drone_dir.mkdir(exist_ok=True)

        # Sostituisci nome modello e tutti i topic /model/tethys/ → /model/tethys_N/
        drone_sdf = equipped_sdf.replace('<model name="tethys">', f'<model name="{name}">')
        drone_sdf = drone_sdf.replace('/model/tethys/', f'/model/{name}/')

        # Inietta gli override topic sonar DENTRO l'experimental:params interno
        # (che sovrascrive i topic hardcoded in tethys/model.sdf).
        # Questo elimina la necessità di experimental:params nel world include,
        # che in Fortress sembra interferire con il caricamento dei plugin modello.
        sonar_overrides = (
            f'\n        <sensor element_id="base_link::frontal_sonar" action="modify">'
            f'<topic>/model/{name}/frontal_sonar</topic></sensor>'
            f'\n        <sensor element_id="base_link::bottom_sonar"  action="modify">'
            f'<topic>/model/{name}/bottom_sonar</topic></sensor>'
            f'\n        <sensor element_id="base_link::left_sonar"    action="modify">'
            f'<topic>/model/{name}/left_sonar</topic></sensor>'
            f'\n        <sensor element_id="base_link::right_sonar"   action="modify">'
            f'<topic>/model/{name}/right_sonar</topic></sensor>'
            f'\n        <sensor element_id="base_link::top_sonar"     action="modify">'
            f'<topic>/model/{name}/top_sonar</topic></sensor>'
        )
        inner_end = '      </experimental:params>\n    </include>'
        drone_sdf = drone_sdf.replace(inner_end, sonar_overrides + '\n' + inner_end, 1)

        (drone_dir / "model.sdf").write_text(drone_sdf)
        (drone_dir / "model.config").write_text(f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>Tethys LRAUV drone {i} (generato automaticamente)</description>
</model>
""")

    print(f"Modelli per-drone generati in {MODELS_DIR}: tethys_0 … tethys_{n_drones-1}",
          file=sys.stderr)


def _parallel_lanes(n_drones: int, z: float = 10.0,
                    spawn_y: float = -20.0, target_y: float = 20.0,
                    lane_slots=None, lane_span=None):
    """Genera spawn e target in corsie parallele N-S (percorsi non si incrociano).

    n_drones droni vanno da spawn_y a target_y, equidistribuiti in X.

    lane_slots fissa la griglia di corsie indipendentemente dal numero di droni:
    con lane_slots=5 e n_drones=3 i droni occupano le corsie 0, 2, 4. Le corsie
    non occupate restano comunque riservate nel piazzamento degli ostacoli, così
    due mondi generati con lo stesso seed e la stessa griglia hanno un campo
    ostacoli identico e i run a 3 e a 5 droni restano confrontabili.

    lane_span fissa la distanza in X fra le due corsie esterne [m]. Senza, il passo
    resta quello storico (40 m di apertura, al massimo 15 m fra corsie), che non
    dipende dalla larghezza della vasca: in una vasca stretta le corsie esterne
    finirebbero sotto la soglia di frenata di parete del nodo (8 m).

    Ritorna (spawns, targets, lane_spawns, lane_targets): le prime due liste sono
    i droni effettivi, le altre due l'intera griglia di corsie.
    """
    slots = lane_slots if lane_slots is not None else n_drones
    if slots < n_drones:
        raise ValueError(f"--lane-slots ({slots}) deve essere >= --n-drones ({n_drones})")

    if slots == 1:
        lane_xs = [0.0]
    else:
        step    = (lane_span / (slots - 1) if lane_span is not None
                   else min(40.0 / (slots - 1), 15.0))
        x0      = -step * (slots - 1) / 2.0
        lane_xs = [x0 + i * step for i in range(slots)]

    lane_spawns  = [(x, spawn_y,  z) for x in lane_xs]
    lane_targets = [(x, target_y, z) for x in lane_xs]

    if n_drones == slots:
        used = list(range(slots))
    elif n_drones == 1:
        used = [slots // 2]
    else:
        # corsie equidistanti fra quelle disponibili, estremi sempre inclusi
        used = [round(i * (slots - 1) / (n_drones - 1)) for i in range(n_drones)]

    spawns  = [lane_spawns[i]  for i in used]
    targets = [lane_targets[i] for i in used]
    return spawns, targets, lane_spawns, lane_targets


def generate_world(world_number, seed=None, preview=False, n_drones=1,
                   n_obs_override=None, spawn_z_min=None, spawn_z_max=None,
                   only_vert=False, parallel_lanes=False,
                   spawn_y=None, target_y=None, path_obstacles=0,
                   path_corridor_w=7.0, lane_slots=None, spread_k=1, lane_span=None):
    if seed is None:
        seed = world_number * 31337 + 42
    rng   = random.Random(seed)
    n_obs = n_obs_override if n_obs_override is not None else get_obstacle_count(world_number)

    sz_min = spawn_z_min if spawn_z_min is not None else SPAWN_Z_MIN
    sz_max = spawn_z_max if spawn_z_max is not None else SPAWN_Z_MAX

    if parallel_lanes:
        lane_z  = (sz_min + sz_max) / 2.0
        spw_y   = spawn_y  if spawn_y  is not None else -20.0
        tgt_y   = target_y if target_y is not None else 20.0
        spawns, targets, lane_spawns, lane_targets = _parallel_lanes(
            n_drones, z=lane_z, spawn_y=spw_y, target_y=tgt_y, lane_slots=lane_slots,
            lane_span=lane_span)
        # Gli ostacoli tengono conto di TUTTE le corsie della griglia, non solo di
        # quelle occupate: è questo che rende identico il campo ostacoli fra il
        # mondo a 3 droni e quello a 5 generati con lo stesso seed.
        all_positions = list(lane_spawns) + list(lane_targets)
    else:
        # Genera n_drones spawn + n_drones target (tutti con mutual clearance)
        all_positions = _rand_positions(rng, n_drones * 2, z_min=sz_min, z_max=sz_max)
        spawns  = all_positions[:n_drones]
        targets = all_positions[n_drones:]
        lane_spawns, lane_targets = spawns, targets

    # Raccoglie tutte le posizioni 3D occupate (spawn + target)
    occupied_xyz = list(all_positions)           # (x,y,z) — usato dal check 3D cyl_horiz
    occupied_xy  = [(p[0], p[1]) for p in all_positions]  # solo XY — per placed_xy

    # Forma della lista ostacoli
    if only_vert:
        shape_list = ['cyl_vert'] * n_obs
    else:
        # Bilanciamento 50/50 tra cyl_vert e cyl_horiz, ordine rimescolato
        n_types    = len(ALL_SHAPES)
        base       = n_obs // n_types
        extra      = n_obs % n_types
        shape_list = ALL_SHAPES[:extra] * (base + 1) + ALL_SHAPES[extra:] * base
        rng.shuffle(shape_list)

    # Campionamento stratificato in quota per i cilindri orizzontali
    z_band    = (OBS_Z_MAX - OBS_Z_MIN) / n_obs
    z_centers = [rng.uniform(OBS_Z_MIN + i * z_band, OBS_Z_MIN + (i + 1) * z_band)
                 for i in range(n_obs)]
    rng.shuffle(z_centers)

    placed_xy: list = list(occupied_xy)
    obstacles = [generate_obstacle(rng, i, shape_list[i], z_centers[i], placed_xy, None,
                                   occupied_xyz, spread_k=spread_k)
                 for i in range(n_obs)]

    # Ostacoli nel percorso (path_obstacles): cilindri verticali concentrati nei corridoi
    spawn_xys_all = [(p[0], p[1]) for p in occupied_xyz]
    for j in range(path_obstacles):
        idx     = n_obs + j
        px, py  = rand_pos_in_corridor(rng, lane_spawns, lane_targets,
                                       corridor_half_w=path_corridor_w,
                                       placed_xy=placed_xy,
                                       spawn_xys=spawn_xys_all,
                                       spread_k=spread_k)
        radius  = rng.uniform(0.8, 1.5) * OBS_RADIUS_SCALE
        height  = rng.uniform(OBS_HEIGHT_MIN, OBS_HEIGHT_MAX)
        half_h  = height / 2.0
        z_p     = (OBS_BASE_Z + half_h) if OBS_BASE_Z is not None else OBS_Z_CENTER
        z_p     = min(z_p, TANK_Z_MAX - half_h)
        if OBS_BASE_Z is None:
            z_p = max(z_p, half_h + OBS_BASE_CLEAR)
        color   = _obstacle_color(idx)
        pose    = f"{px:.2f} {py:.2f} {z_p:.2f} 0.0000 0.0000 0.0000"
        obstacles.append(_link(f"p{j}", pose, _geom_cyl(radius, height), color))

    obs_sdf = "\n".join(obstacles)

    if preview:
        drone_includes = ""
    else:
        tethys_sdf_path = MODELS_DIR / "tethys" / "model.sdf"
        tethys_sdf_text = tethys_sdf_path.read_text()
        if n_drones == 1:
            # Usa flat model anche per singolo drone: stesso aggiro del bug Thruster
            # con <topic> esplicito che produce topic malformati (tethysmodel/...).
            drone_includes = _make_drone_flat("tethys", spawns[0][0], spawns[0][1], spawns[0][2], tethys_sdf_text) + "\n"
        else:
            parts = []
            for i, (sx, sy, sz) in enumerate(spawns):
                n = f"tethys_{i}"
                # Modello completamente flat: nessun <include>, tutti i link/joint/plugin
                # espliciti → Thruster/JPC/LiftDrag trovano propeller_joint nell'ECM.
                parts.append(_make_drone_flat(n, sx, sy, sz, tethys_sdf_text))
            drone_includes = "\n\n".join(parts) + "\n"

    # Commento spawn/target per ogni drone
    spawn_comments = "\n".join(
        f"<!-- Drone {i} spawn: ({s[0]:.2f}, {s[1]:.2f}, {s[2]:.2f})  target: ({t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f}) -->"
        for i, (s, t) in enumerate(zip(spawns, targets))
    ) if n_drones > 1 else f"<!-- Spawn: ({spawns[0][0]:.2f}, {spawns[0][1]:.2f}, {spawns[0][2]:.2f}) -->"
    # Commento dimensioni vasca (letto da cervello_multi5.sh per wall_wx/wy)
    spawn_comments += f"\n<!-- Tank: wall_wx={TANK_HALF:.2f} wall_wy={TANK_HALF_Y:.2f} -->"

    # Sfere rosse semi-trasparenti ai target
    target_markers = "\n".join(
        _make_target_marker(i, t[0], t[1], t[2])
        for i, t in enumerate(targets)
    ) if n_drones > 1 else _make_target_marker(0, targets[0][0], targets[0][1], targets[0][2])

    SX = int(TANK_HALF   * 2)
    SY = int(TANK_HALF_Y * 2)
    obs_type_label = "cyl_vert" if only_vert else "cyl_vert+cyl_horiz"
    n_total = n_obs + path_obstacles
    tank_template  = _make_tank_template()  # chiamato qui per usare TANK_HEIGHT aggiornato da CLI
    return f"""<?xml version="1.0" ?>
<!-- Mondo {world_number} — {n_total} ostacoli ({n_obs} rand+{path_obstacles} percorso, {obs_type_label}) — seed={seed} — vasca {SX}×{SY}×{int(TANK_HEIGHT)} m -->
{spawn_comments}
<sdf version="1.9">
  <world name="buoyant_tethys">
    <scene>
      <ambient>0.0 0.6 0.7</ambient>
      <background>0.0 0.4 0.5</background>
    </scene>

    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="ignition-gazebo-physics-system"           name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-contact-system" name="ignition::gazebo::systems::Contact"/>
    <plugin filename="ignition-gazebo-user-commands-system"     name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system"           name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="ignition-gazebo-buoyancy-system"          name="ignition::gazebo::systems::Buoyancy">
      <uniform_fluid_density>1025</uniform_fluid_density>
    </plugin>

    <light type="directional" name="sole_mondo">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 50 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

{drone_includes}
{tank_template}

    <!-- Target markers: sfere rosse semi-trasparenti (visual only) -->
{target_markers}

    <!-- Ostacoli: {n_obs} ({obs_type_label}) — seed {seed} -->
    <model name="ostacoli">
      <static>true</static>
{obs_sdf}
    </model>

  </world>
</sdf>
""", spawns, targets


def main():
    parser = argparse.ArgumentParser(description="Genera mondo SDF per Tethys (cyl_vert + cyl_horiz, spawn random)")
    parser.add_argument("world_number", type=int, help="Numero del mondo (≥3)")
    parser.add_argument("--seed",        type=int,   default=None)
    parser.add_argument("--output",      type=str,   default=None)
    parser.add_argument("--preview",     action="store_true")
    parser.add_argument("--n-drones",    type=int,   default=1,    dest="n_drones",
                        help="Numero di droni (default: 1)")
    parser.add_argument("--n-obstacles", type=int,   default=None, dest="n_obstacles",
                        help="Numero ostacoli (override formula world_number)")
    parser.add_argument("--spawn-z-min", type=float, default=None, dest="spawn_z_min",
                        help="Quota minima spawn/target [m] (default: SPAWN_Z_MIN=4)")
    parser.add_argument("--spawn-z-max", type=float, default=None, dest="spawn_z_max",
                        help="Quota massima spawn/target [m] (default: SPAWN_Z_MAX=12)")
    parser.add_argument("--only-vert",      action="store_true", dest="only_vert",
                        help="Usa solo cyl_vert (nessun cyl_horiz)")
    parser.add_argument("--parallel-lanes", action="store_true", dest="parallel_lanes",
                        help="Spawn/target in corsie parallele N-S (no incrocio percorsi)")
    parser.add_argument("--tank-height",    type=float, default=None, dest="tank_height",
                        help="Altezza vasca [m] (default: 35.0)")
    parser.add_argument("--obs-height-min", type=float, default=None, dest="obs_height_min",
                        help="Altezza minima ostacoli cyl_vert [m] (default: 8.0)")
    parser.add_argument("--obs-height-max", type=float, default=None, dest="obs_height_max",
                        help="Altezza massima ostacoli cyl_vert [m] (default: 12.0)")
    parser.add_argument("--obs-z-center",   type=float, default=None, dest="obs_z_center",
                        help="Quota centrale ostacoli cyl_vert [m] (default: 10.0)")
    parser.add_argument("--min-sep",       type=float, default=None, dest="min_sep",
                        help="Distanza minima XY fra centri ostacoli [m] (default: 9.0). "
                             "Valori alti diradano il campo.")
    parser.add_argument("--tank-half-x",   type=float, default=None, dest="tank_half_x",
                        help="Semi-larghezza vasca in X [m] (default: 80.0). "
                             "Valori bassi creano un corridoio stretto.")
    parser.add_argument("--tank-half-y",   type=float, default=None, dest="tank_half_y",
                        help="Semi-larghezza vasca in Y [m] (default: 80.0 = vasca quadrata). "
                             "Valori > 80 creano vasca rettangolare più lunga in N-S.")
    parser.add_argument("--spawn-y",       type=float, default=None, dest="spawn_y",
                        help="Coordinata Y degli spawn in --parallel-lanes (default: -20.0)")
    parser.add_argument("--target-y",      type=float, default=None, dest="target_y",
                        help="Coordinata Y dei target in --parallel-lanes (default: 20.0)")
    parser.add_argument("--path-obstacles", type=int,  default=0,    dest="path_obstacles",
                        help="N ostacoli aggiuntivi concentrati nel corridoio spawn→target")
    parser.add_argument("--obs-base-z",    type=float, default=None, dest="obs_base_z",
                        help="Ancora la base di OGNI pilastro a questa quota [m]: con 0 tutti "
                             "poggiano sul fondale, indipendentemente dall'altezza. Senza questa "
                             "opzione il centro resta a --obs-z-center e i pilastri piu' bassi "
                             "restano sospesi.")
    parser.add_argument("--obs-base-clear", type=float, default=None, dest="obs_base_clear",
                        help="Stacco minimo della base dal fondale [m] (default: 2.0), usato solo "
                             "quando --obs-base-z non e' impostata. Teneva libera una fascia sul "
                             "fondale per la fuga verticale, che nel corridoio e' disattivata.")
    parser.add_argument("--spread",        type=int,   default=1,    dest="spread_k",
                        help="Candidati valutati per ogni ostacolo casuale (default: 1 = "
                             "primo punto valido). Valori 8–16 distribuiscono gli ostacoli "
                             "in modo uniforme invece che a grappoli.")
    parser.add_argument("--wall-clear",    type=float, default=None, dest="wall_clear",
                        help="Distanza minima fra ostacoli e pareti [m] (default: 6.0). "
                             "In un corridoio stretto il default confina tutti gli ostacoli "
                             "nella fascia centrale.")
    parser.add_argument("--lane-slots",    type=int,   default=None, dest="lane_slots",
                        help="N corsie della griglia in --parallel-lanes (default: --n-drones). "
                             "Con un valore > --n-drones i droni occupano corsie equidistanti e "
                             "le corsie vuote restano riservate: due mondi con lo stesso seed e la "
                             "stessa griglia hanno campo ostacoli identico.")
    parser.add_argument("--obs-radius-scale", type=float, default=None, dest="obs_radius_scale",
                        help="Fattore sul raggio dei cilindri verticali (default 1: raggio 0,8-1,8 m "
                             "casuali, 0,8-1,5 m di percorso).")
    parser.add_argument("--lane-span",     type=float, default=None, dest="lane_span",
                        help="Distanza in X fra le corsie esterne in --parallel-lanes [m] "
                             "(default: passo storico, 40 m di apertura e al massimo 15 m fra "
                             "corsie, indipendente dalla vasca).")
    parser.add_argument("--path-corridor-w", type=float, default=7.0, dest="path_corridor_w",
                        help="Semi-larghezza corridoio per --path-obstacles [m] (default: 7.0)")
    args = parser.parse_args()

    # Override costanti globali da CLI
    if args.tank_height is not None:
        global TANK_HEIGHT, TANK_Z_MAX
        TANK_HEIGHT = args.tank_height
        TANK_Z_MAX  = TANK_HEIGHT - 1.5
    if args.obs_height_min is not None:
        global OBS_HEIGHT_MIN
        OBS_HEIGHT_MIN = args.obs_height_min
    if args.obs_height_max is not None:
        global OBS_HEIGHT_MAX
        OBS_HEIGHT_MAX = args.obs_height_max
    if args.obs_z_center is not None:
        global OBS_Z_CENTER
        OBS_Z_CENTER = args.obs_z_center
    if args.min_sep is not None:
        global MIN_SEP
        MIN_SEP = args.min_sep
    if args.tank_half_x is not None:
        global TANK_HALF
        TANK_HALF = args.tank_half_x
    if args.tank_half_y is not None:
        global TANK_HALF_Y
        TANK_HALF_Y = args.tank_half_y
    if args.wall_clear is not None:
        global WALL_CLEAR
        WALL_CLEAR = args.wall_clear
    if args.obs_base_clear is not None:
        global OBS_BASE_CLEAR
        OBS_BASE_CLEAR = args.obs_base_clear
    if args.obs_base_z is not None:
        global OBS_BASE_Z
        OBS_BASE_Z = args.obs_base_z
    if args.obs_radius_scale is not None:
        global OBS_RADIUS_SCALE
        OBS_RADIUS_SCALE = args.obs_radius_scale

    if args.world_number < 3:
        print("Errore: world_number deve essere ≥3", file=sys.stderr)
        sys.exit(1)

    sdf, spawns, targets = generate_world(
        args.world_number, args.seed, args.preview, args.n_drones,
        n_obs_override=args.n_obstacles,
        spawn_z_min=args.spawn_z_min,
        spawn_z_max=args.spawn_z_max,
        only_vert=args.only_vert,
        parallel_lanes=args.parallel_lanes,
        spawn_y=args.spawn_y,
        target_y=args.target_y,
        path_obstacles=args.path_obstacles,
        path_corridor_w=args.path_corridor_w,
        lane_slots=args.lane_slots,
        spread_k=args.spread_k,
        lane_span=args.lane_span,
    )
    n_obs = args.n_obstacles if args.n_obstacles is not None else get_obstacle_count(args.world_number)
    group = (args.world_number - 3) // 5 + 1

    if args.output:
        Path(args.output).write_text(sdf)
        obs_label = "cyl_vert" if args.only_vert else "cyl_vert+cyl_horiz"
        print(f"Mondo {args.world_number}: {n_obs} ostacoli ({obs_label}, gruppo {group}) → {args.output}")
        if args.n_drones > 1:
            for i, (s, t) in enumerate(zip(spawns, targets)):
                print(f"  Drone {i}: spawn ({s[0]:.2f},{s[1]:.2f},{s[2]:.2f}) target ({t[0]:.2f},{t[1]:.2f},{t[2]:.2f})")
    else:
        print(sdf)


if __name__ == "__main__":
    main()
