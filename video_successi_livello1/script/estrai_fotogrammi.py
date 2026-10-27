#!/usr/bin/env python3
"""Controllo di un MP4 finale: demux con qtdemux (GStreamer), decodifica completa con
libavcodec (codifica_h264 --decodifica) e salvataggio di un PNG ogni PASSO fotogrammi.
Sostituisce ffprobe/ffmpeg, non installati (libavformat corrotta su questa macchina).

Uso: estrai_fotogrammi.py VIDEO.mp4 CARTELLA PASSO
Stampa caps (codec, profilo, risoluzione, fps), numero di campioni, durata.
"""
import struct, subprocess, sys
from pathlib import Path
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst  # noqa: E402

mp4, out, passo = sys.argv[1], Path(sys.argv[2]), sys.argv[3]
out.mkdir(parents=True, exist_ok=True)
Gst.init(None)
pipe = Gst.parse_launch(f'filesrc location="{mp4}" ! qtdemux ! appsink name=sink sync=false')
sink = pipe.get_by_name('sink')
pipe.set_state(Gst.State.PLAYING)
campioni, caps = [], None
while True:
    s = sink.emit('try-pull-sample', 10 * Gst.SECOND)
    if s is None:
        break
    caps = s.get_caps()
    b = s.get_buffer()
    ok, info = b.map(Gst.MapFlags.READ)
    campioni.append((b.pts, not b.has_flags(Gst.BufferFlags.DELTA_UNIT), bytes(info.data)))
    b.unmap(info)
pipe.set_state(Gst.State.NULL)
st = caps.get_structure(0)
ok, cd = st.get_value('codec_data').map(Gst.MapFlags.READ)
avcc = bytes(cd.data)
# avcC -> SPS/PPS in Annex B
sc = b'\x00\x00\x00\x01'
p, extra = 6, b''
nsps = avcc[5] & 0x1f
for _ in range(nsps):
    (n,) = struct.unpack_from('>H', avcc, p); extra += sc + avcc[p + 2:p + 2 + n]; p += 2 + n
npps = avcc[p]; p += 1
for _ in range(npps):
    (n,) = struct.unpack_from('>H', avcc, p); extra += sc + avcc[p + 2:p + 2 + n]; p += 2 + n


def annexb(d):
    r, i = b'', 0
    while i < len(d):
        (n,) = struct.unpack_from('>I', d, i); r += sc + d[i + 4:i + 4 + n]; i += 4 + n
    return r


pkt = out / 'flusso.pkt'
with open(pkt, 'wb') as f:
    f.write(b'PKT1' + struct.pack('<I', len(extra)) + extra)
    for k, (pts, key, d) in enumerate(campioni):
        a = annexb(d)
        f.write(struct.pack('<IBq', len(a), int(key), k) + a)
print(st.to_string())
print(f'campioni {len(campioni)}, durata {campioni[-1][0] / Gst.SECOND + 1/30:.2f} s')
subprocess.run([str(Path(__file__).resolve().parent / 'codificatore/build/codifica_h264'),
                '--decodifica', str(pkt), str(out), passo], check=True)
pkt.unlink()
