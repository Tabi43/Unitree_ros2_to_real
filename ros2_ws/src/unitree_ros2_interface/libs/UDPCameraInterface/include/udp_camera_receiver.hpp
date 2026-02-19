#ifndef UDP_CAMERA_RECEIVER_HPP
#define UDP_CAMERA_RECEIVER_HPP

#include <cstdint>

#pragma pack(push, 1)
struct UdpImageChunkHeader
{
  uint32_t magic;        // 'UIMG' = 0x55494D47
  uint16_t version;      // 1
  uint16_t stream_id;    // 0=left,1=right,...
  uint64_t frame_id;     // incrementale
  uint32_t chunk_idx;    // 0..chunk_count-1
  uint32_t chunk_count;  // totale chunk
  uint32_t payload_size; // bytes payload in questo datagramma
  uint32_t jpeg_size;    // bytes totali JPEG
  uint32_t width;        // px
  uint32_t height;       // px
  uint64_t stamp_ns;     // steady clock ns
};
#pragma pack(pop)

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>  // cv::Mat

struct UdpCameraReceiverConfig {
  std::string bind_ip = "0.0.0.0";
  int bind_port = 5000;
  uint16_t expected_stream_id = 0;

  int frame_timeout_ms = 80;   // scarta frame incompleto dopo questo tempo
  int max_inflight = 16;       // max frame in ricomposizione

  bool store_jpeg = false;     // se vuoi anche i bytes JPEG oltre al Mat decodificato
};

struct ReceivedCameraFrame
{
  uint64_t frame_id = 0;
  uint64_t stamp_ns = 0;
  uint32_t width = 0;
  uint32_t height = 0;

  cv::Mat bgr;                 // immagine decodificata (BGR)
  std::vector<uint8_t> jpeg;   // opzionale (store_jpeg=true)
};

enum class CameraReadMode
{
  NewOnly,      // ritorna false se l'ultimo frame è già stato letto
  LatestAlways  // ritorna sempre l'ultimo frame disponibile (anche se uguale)
};

class UdpCameraReceiver {
public:
  explicit UdpCameraReceiver(UdpCameraReceiverConfig cfg);
  ~UdpCameraReceiver();

  bool start();       // apre socket e avvia thread
  void stop();        // ferma thread e chiude socket

  // Lettura "zero-copy" (consigliata): ti passa uno shared_ptr stabile al frame.
  // - NewOnly: false se non c'è un frame nuovo rispetto all'ultima read() di QUESTO oggetto.
  bool read(std::shared_ptr<const ReceivedCameraFrame>& out,
            CameraReadMode mode = CameraReadMode::NewOnly);

  // Lettura che copia (o clona) in cv::Mat.
  // clone=true: copia profonda (sicura per modifiche del caller); clone=false: shallow copy (più veloce).
  bool read(cv::Mat& out_bgr,
            CameraReadMode mode = CameraReadMode::NewOnly,
            bool clone = false);

  // Attende un frame nuovo (rispetto all'ultima read()) fino a timeout.
  bool waitForNew(std::shared_ptr<const ReceivedCameraFrame>& out,
                  std::chrono::milliseconds timeout);

  uint64_t lastPublishedFrameId() const;

private:
  void rxLoop();
  bool openSocket();

private:
  UdpCameraReceiverConfig cfg_;

  int sock_ = -1;
  std::atomic<bool> running_{false};
  std::thread* rx_thread_ = nullptr;

  // "latest frame" pubblicato dal thread rx
  std::shared_ptr<const ReceivedCameraFrame> latest_;  // accesso atomico via atomic_load/store
  std::atomic<uint64_t> latest_id_{0};

  // per modalità NewOnly: ultimo frame_id letto da questo oggetto
  uint64_t last_read_id_ = static_cast<uint64_t>(-1);
  std::mutex read_mtx_;  // protegge last_read_id_ per chiamate concorrenti a read()

  // notify per waitForNew
  std::mutex cv_mtx_;
  std::condition_variable cv_;
};

#endif  // UDP_CAMERA_RECEIVER_HPP
