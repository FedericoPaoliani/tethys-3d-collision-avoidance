#!/usr/bin/env python3
"""Mette in un contenitore MP4 i pacchetti H.264 prodotti da codifica_h264 (formato .pkt).

Usa mp4mux di GStreamer (gst-plugins-good) al posto di libavformat, corrotto su questa
macchina. Converte i pacchetti da Annex B (start code) ad AVC (lunghezza a 4 byte) e
costruisce il record avcC da SPS/PPS, come richiesto da MP4.

Uso:
  mux_mp4.py FILE.pkt USCITA.mp4 FPS LARGHEZZA ALTEZZA
  mux_mp4.py --verifica FILE.mp4 FILE.pkt   rilegge l'MP4 con qtdemux e confronta i pacchetti
"""
import struct
import sys

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstApp', '1.0')
from gi.repository import Gst, GLib  # noqa: E402


def leggi_pkt(path):
    data = open(path, 'rb').read()
    assert data[:4] == b'PKT1', 'formato .pkt non riconosciuto'
    (ext,) = struct.unpack_from('<I', data, 4)
    extradata = data[8:8 + ext]
    off, pacchetti = 8 + ext, []
    while off < len(data):
        size, key, pts = struct.unpack_from('<IBq', data, off)
        off += 13
        pacchetti.append((pts, bool(key), data[off:off + size]))
        off += size
    return extradata, pacchetti


def nal_annexb(buf):
    """Separa le unità NAL di un buffer Annex B."""
    out, i, n = [], 0, len(buf)
    starts = []
    while i < n - 3:
        if buf[i] == 0 and buf[i + 1] == 0 and (buf[i + 2] == 1 or (buf[i + 2] == 0 and buf[i + 3] == 1)):
            sc = 3 if buf[i + 2] == 1 else 4
            starts.append((i, i + sc))
            i += sc
        else:
            i += 1
    for k, (_, s) in enumerate(starts):
        e = starts[k + 1][0] if k + 1 < len(starts) else n
        nal = buf[s:e]
        while nal and nal[-1] == 0 and k + 1 < len(starts):
            nal = nal[:-1]
        if nal:
            out.append(nal)
    return out


def avcc(nals):
    sps = [x for x in nals if x[0] & 0x1F == 7]
    pps = [x for x in nals if x[0] & 0x1F == 8]
    assert sps and pps, 'SPS/PPS assenti'
    b = bytes([1, sps[0][1], sps[0][2], sps[0][3], 0xFF, 0xE0 | len(sps)])
    for s in sps:
        b += struct.pack('>H', len(s)) + s
    b += bytes([len(pps)])
    for p in pps:
        b += struct.pack('>H', len(p)) + p
    return b


def in_avc(buf):
    # SPS/PPS/SEI restano nel record avcC; nei campioni solo le slice e le SEI.
    return b''.join(struct.pack('>I', len(n)) + n for n in nal_annexb(buf)
                    if n[0] & 0x1F not in (7, 8, 9))


def mux(pkt_path, uscita, fps, w, h):
    extradata, pacchetti = leggi_pkt(pkt_path)
    codec_data = avcc(nal_annexb(extradata) or nal_annexb(pacchetti[0][2]))
    Gst.init(None)
    pipe = Gst.parse_launch(
        f'mp4mux name=mux faststart=true ! filesink location="{uscita}" '
        'appsrc name=src format=time is-live=false ! mux.video_0')
    src = pipe.get_by_name('src')
    caps = Gst.Caps.from_string(
        f'video/x-h264,stream-format=avc,alignment=au,width={w},height={h},'
        f'framerate={fps}/1,pixel-aspect-ratio=1/1')
    s = caps.get_structure(0).copy()
    s.set_value('codec_data', Gst.Buffer.new_wrapped(codec_data))
    caps = Gst.Caps.new_empty()
    caps.append_structure(s)
    src.set_property('caps', caps)
    pipe.set_state(Gst.State.PLAYING)
    dur = Gst.SECOND // fps
    for pts, key, data in pacchetti:
        buf = Gst.Buffer.new_wrapped(in_avc(data))
        buf.pts = buf.dts = pts * Gst.SECOND // fps
        buf.duration = dur
        if not key:
            buf.set_flags(Gst.BufferFlags.DELTA_UNIT)
        assert src.emit('push-buffer', buf) == Gst.FlowReturn.OK
    src.emit('end-of-stream')
    msg = pipe.get_bus().timed_pop_filtered(60 * Gst.SECOND,
                                            Gst.MessageType.EOS | Gst.MessageType.ERROR)
    pipe.set_state(Gst.State.NULL)
    if msg is None or msg.type == Gst.MessageType.ERROR:
        print('errore GStreamer:', msg.parse_error() if msg else 'timeout')
        return 1
    print(f'{len(pacchetti)} campioni -> {uscita}')
    return 0


def verifica(mp4, pkt_path):
    """Demux con qtdemux: confronta campione per campione con i pacchetti originali."""
    _, pacchetti = leggi_pkt(pkt_path)
    Gst.init(None)
    pipe = Gst.parse_launch(f'filesrc location="{mp4}" ! qtdemux ! appsink name=sink sync=false')
    sink = pipe.get_by_name('sink')
    pipe.set_state(Gst.State.PLAYING)
    letti, caps = [], None
    while True:
        sample = sink.emit('try-pull-sample', 10 * Gst.SECOND)
        if sample is None:
            break
        caps = sample.get_caps().to_string()
        b = sample.get_buffer()
        ok, info = b.map(Gst.MapFlags.READ)
        letti.append((b.pts, bytes(info.data), not b.has_flags(Gst.BufferFlags.DELTA_UNIT)))
        b.unmap(info)
    pipe.set_state(Gst.State.NULL)
    uguali = sum(1 for (_, d, k), (_, key, orig) in zip(letti, pacchetti) if d == in_avc(orig) and k == key)
    print('caps:', caps)
    print(f'campioni letti {len(letti)} / scritti {len(pacchetti)}; identici {uguali}; '
          f'ultimo pts {letti[-1][0] / Gst.SECOND:.3f} s; keyframe {sum(k for *_, k in letti)}')
    return 0 if uguali == len(pacchetti) == len(letti) else 1


if __name__ == '__main__':
    if sys.argv[1] == '--verifica':
        sys.exit(verifica(sys.argv[2], sys.argv[3]))
    sys.exit(mux(sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5])))
