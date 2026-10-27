// Codifica frame PNG in H.264 (libx264 via libavcodec, yuv420p, senza B-frame) e scrive i
// pacchetti in un file semplice, poi messi in MP4 da mux_mp4.py con GStreamer (mp4mux).
// Non usa libavformat: su questa macchina /usr/lib/x86_64-linux-gnu/libavformat.so.58.76.100
// risulta corrotto (dpkg -V) e va in SIGILL.
//
// Uso:
//   codifica_h264 ELENCO.txt USCITA.pkt FPS MAXRATE      codifica (un PNG per riga; CRF 16,
//                                                        MAXRATE = tetto VBV in bit/s)
//   codifica_h264 --decodifica FILE.pkt CARTELLA PASSO    decodifica e salva un PNG ogni PASSO
// Formato .pkt: "PKT1", u32 len extradata, extradata, poi per pacchetto:
//   u32 dimensione, u8 keyframe, i64 pts, dati (Annex B).
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}
#include <ignition/common/Image.hh>

namespace ic = ignition::common;

static void scrivi_u32(FILE *f, uint32_t v) { fwrite(&v, 4, 1, f); }

static int scarica(AVCodecContext *ctx, AVPacket *pkt, FILE *out, size_t &n)
{
  int r;
  while ((r = avcodec_receive_packet(ctx, pkt)) == 0)
  {
    scrivi_u32(out, pkt->size);
    uint8_t key = (pkt->flags & AV_PKT_FLAG_KEY) ? 1 : 0;
    fwrite(&key, 1, 1, out);
    int64_t pts = pkt->pts;
    fwrite(&pts, 8, 1, out);
    fwrite(pkt->data, 1, pkt->size, out);
    av_packet_unref(pkt);
    ++n;
  }
  return (r == AVERROR(EAGAIN) || r == AVERROR_EOF) ? 0 : r;
}

static int codifica(const char *elenco_path, const char *uscita, int fps, int64_t bitrate)
{
  std::ifstream elenco(elenco_path);
  std::vector<std::string> frame;
  for (std::string r; std::getline(elenco, r);)
    if (!r.empty()) frame.push_back(r);
  if (frame.empty()) { std::cerr << "elenco vuoto\n"; return 1; }
  ic::Image primo(frame.front());
  const int w = primo.Width(), h = primo.Height();

  const AVCodec *codec = avcodec_find_encoder_by_name("libx264");
  if (!codec) { std::cerr << "libx264 non disponibile in libavcodec\n"; return 1; }
  AVCodecContext *ctx = avcodec_alloc_context3(codec);
  ctx->width = w;
  ctx->height = h;
  ctx->time_base = AVRational{1, fps};
  ctx->framerate = AVRational{fps, 1};
  ctx->pix_fmt = AV_PIX_FMT_YUV420P;
  ctx->gop_size = fps;          // un keyframe al secondo: ricerca agevole in PowerPoint
  ctx->max_b_frames = 0;        // pts == dts: mux semplice
  // Qualità costante (CRF) con tetto VBV: livello 4.1, compatibile con PowerPoint.
  ctx->rc_max_rate = bitrate;
  ctx->rc_buffer_size = bitrate * 2;
  ctx->level = 41;
  ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
  ctx->colorspace = AVCOL_SPC_BT709;
  ctx->color_primaries = AVCOL_PRI_BT709;
  ctx->color_trc = AVCOL_TRC_BT709;
  ctx->color_range = AVCOL_RANGE_MPEG;
  av_opt_set(ctx->priv_data, "preset", "slow", 0);
  av_opt_set(ctx->priv_data, "crf", "16", 0);
  av_opt_set(ctx->priv_data, "profile", "high", 0);
  if (avcodec_open2(ctx, codec, nullptr) < 0) { std::cerr << "avcodec_open2 fallito\n"; return 1; }

  FILE *out = std::fopen(uscita, "wb");
  fwrite("PKT1", 1, 4, out);
  scrivi_u32(out, ctx->extradata_size);
  fwrite(ctx->extradata, 1, ctx->extradata_size, out);

  SwsContext *sws = sws_getContext(w, h, AV_PIX_FMT_RGB24, w, h, AV_PIX_FMT_YUV420P,
                                   SWS_BICUBIC, nullptr, nullptr, nullptr);
  const int *bt709 = sws_getCoefficients(SWS_CS_ITU709);
  sws_setColorspaceDetails(sws, bt709, 1, bt709, 0, 0, 1 << 16, 1 << 16);
  AVFrame *yuv = av_frame_alloc();
  yuv->format = AV_PIX_FMT_YUV420P;
  yuv->width = w;
  yuv->height = h;
  av_frame_get_buffer(yuv, 0);
  AVPacket *pkt = av_packet_alloc();
  size_t npkt = 0;

  for (size_t k = 0; k < frame.size(); ++k)
  {
    ic::Image img(frame[k]);
    if (static_cast<int>(img.Width()) != w || static_cast<int>(img.Height()) != h)
    { std::cerr << "dimensioni diverse: " << frame[k] << "\n"; return 1; }
    unsigned char *rgb = nullptr;
    unsigned int size = 0;
    img.RGBData(&rgb, size);
    const uint8_t *src[1] = {rgb};
    const int stride[1] = {3 * w};
    av_frame_make_writable(yuv);
    sws_scale(sws, src, stride, 0, h, yuv->data, yuv->linesize);
    delete[] rgb;
    yuv->pts = static_cast<int64_t>(k);
    if (avcodec_send_frame(ctx, yuv) < 0 || scarica(ctx, pkt, out, npkt) < 0)
    { std::cerr << "errore di codifica al frame " << k << "\n"; return 1; }
  }
  avcodec_send_frame(ctx, nullptr);
  scarica(ctx, pkt, out, npkt);
  std::fclose(out);
  std::cout << "frame " << frame.size() << ", pacchetti " << npkt << ", " << w << "x" << h
            << " @ " << fps << " fps -> " << uscita << "\n";
  sws_freeContext(sws);
  av_frame_free(&yuv);
  av_packet_free(&pkt);
  avcodec_free_context(&ctx);
  return npkt == frame.size() ? 0 : 1;
}

// Decodifica un flusso di pacchetti (formato .pkt) e salva un PNG ogni `passo` frame.
static int decodifica(const char *pkt_path, const std::string &dir, int passo)
{
  FILE *f = std::fopen(pkt_path, "rb");
  char magic[4];
  uint32_t ext = 0;
  if (!f || fread(magic, 1, 4, f) != 4 || fread(&ext, 4, 1, f) != 1) return 1;
  std::vector<uint8_t> extradata(ext + AV_INPUT_BUFFER_PADDING_SIZE, 0);
  if (fread(extradata.data(), 1, ext, f) != ext) return 1;
  const AVCodec *codec = avcodec_find_decoder(AV_CODEC_ID_H264);
  AVCodecContext *ctx = avcodec_alloc_context3(codec);
  ctx->extradata = static_cast<uint8_t *>(av_mallocz(ext + AV_INPUT_BUFFER_PADDING_SIZE));
  memcpy(ctx->extradata, extradata.data(), ext);
  ctx->extradata_size = ext;
  if (avcodec_open2(ctx, codec, nullptr) < 0) return 1;
  AVPacket *pkt = av_packet_alloc();
  AVFrame *fr = av_frame_alloc();
  SwsContext *sws = nullptr;
  int n = 0, w = 0, h = 0;
  auto ricevi = [&]() {
    while (avcodec_receive_frame(ctx, fr) == 0)
    {
      w = fr->width; h = fr->height;
      if (n % passo == 0)
      {
        if (!sws)
          sws = sws_getContext(w, h, static_cast<AVPixelFormat>(fr->format), w, h,
                               AV_PIX_FMT_RGB24, SWS_BICUBIC, nullptr, nullptr, nullptr);
        std::vector<uint8_t> rgb(static_cast<size_t>(w) * h * 3);
        uint8_t *dst[1] = {rgb.data()};
        int ls[1] = {3 * w};
        sws_scale(sws, fr->data, fr->linesize, 0, h, dst, ls);
        ic::Image img;
        img.SetFromData(rgb.data(), w, h, ic::Image::RGB_INT8);
        img.SavePNG(dir + "/frame_" + std::to_string(n) + ".png");
      }
      ++n;
    }
  };
  for (;;)
  {
    uint32_t size; uint8_t key; int64_t pts;
    if (fread(&size, 4, 1, f) != 1 || fread(&key, 1, 1, f) != 1 || fread(&pts, 8, 1, f) != 1) break;
    av_new_packet(pkt, size);
    if (fread(pkt->data, 1, size, f) != size) break;
    pkt->pts = pts;
    if (avcodec_send_packet(ctx, pkt) < 0) { std::cerr << "pacchetto non valido\n"; return 1; }
    av_packet_unref(pkt);
    ricevi();
  }
  avcodec_send_packet(ctx, nullptr);
  ricevi();
  std::cout << "decodificati " << n << " frame " << w << "x" << h << "\n";
  return n > 0 ? 0 : 1;
}

int main(int argc, char **argv)
{
  if (argc == 5 && std::string(argv[1]) == "--decodifica")
    return decodifica(argv[2], argv[3], std::stoi(argv[4]));
  if (argc != 5)
  {
    std::cerr << "uso: codifica_h264 ELENCO.txt USCITA.pkt FPS BITRATE\n"
                 "     codifica_h264 --decodifica FILE.pkt CARTELLA PASSO\n";
    return 2;
  }
  return codifica(argv[1], argv[2], std::stoi(argv[3]), std::stoll(argv[4]));
}
