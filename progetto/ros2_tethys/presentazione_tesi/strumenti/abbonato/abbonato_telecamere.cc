// Iscritto ai topic delle telecamere del mondo demo. In Fortress il sistema Sensors rende
// una telecamera solo se il suo topic ha almeno un iscritto: questo programma lo fornisce e
// registra, per ogni frame ricevuto, il tempo simulato dell'header (per allineare i PNG
// salvati dal sensore con la telemetria della missione). Non salva immagini.
//
// Uso: abbonato_telecamere USCITA.csv TOPIC [TOPIC...]
#include <atomic>
#include <csignal>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>

#include <ignition/msgs/image.pb.h>
#include <ignition/transport/Node.hh>

static std::atomic<bool> fine{false};
static std::mutex m;
static FILE *out = nullptr;

int main(int argc, char **argv)
{
  if (argc < 3) { std::fprintf(stderr, "uso: abbonato_telecamere USCITA.csv TOPIC...\n"); return 2; }
  out = std::fopen(argv[1], "w");
  std::fprintf(out, "topic,indice,sim_t,larghezza,altezza\n");
  std::signal(SIGINT, [](int) { fine = true; });
  std::signal(SIGTERM, [](int) { fine = true; });
  ignition::transport::Node node;
  for (int i = 2; i < argc; ++i)
  {
    std::string topic = argv[i];
    auto conteggio = std::make_shared<long>(0);
    std::function<void(const ignition::msgs::Image &)> cb =
      [topic, conteggio](const ignition::msgs::Image &msg)
      {
        const double t = msg.header().stamp().sec() + msg.header().stamp().nsec() * 1e-9;
        std::lock_guard<std::mutex> lk(m);
        std::fprintf(out, "%s,%ld,%.4f,%u,%u\n", topic.c_str(), (*conteggio)++, t,
                     msg.width(), msg.height());
      };
    if (!node.Subscribe(topic, cb)) { std::fprintf(stderr, "iscrizione fallita: %s\n", argv[i]); return 1; }
  }
  while (!fine)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::lock_guard<std::mutex> lk(m);
    std::fflush(out);
  }
  std::fclose(out);
  return 0;
}
