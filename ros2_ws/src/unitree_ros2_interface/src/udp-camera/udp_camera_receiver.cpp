#include "unitree_ros2_interface/udp-camera/udp_camera_receiver.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <unordered_map>
#include <vector>

#include <opencv2/imgcodecs.hpp>  // cv::imdecode

namespace {

static inline uint64_t ntohll_u64(uint64_t x) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  return (static_cast<uint64_t>(ntohl(static_cast<uint32_t>(x & 0xFFFFFFFFULL))) << 32) |
         (static_cast<uint64_t>(ntohl(static_cast<uint32_t>(x >> 32))));
#else
  return x;
#endif
}

struct FrameAssembly {
  uint64_t frame_id = 0;
  uint16_t stream_id = 0;

  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t chunk_count = 0;
  uint32_t jpeg_size = 0;
  uint64_t stamp_ns = 0;

  std::vector<std::vector<uint8_t>> chunks;
  std::vector<uint8_t> received; // 0/1
  uint32_t received_count = 0;

  std::chrono::steady_clock::time_point first_seen;

  bool complete() const { return chunk_count > 0 && received_count == chunk_count; }
};

} // namespace

UdpCameraReceiver::UdpCameraReceiver(UdpCameraReceiverConfig cfg) : cfg_(std::move(cfg)) {}
UdpCameraReceiver::~UdpCameraReceiver() { stop(); }

bool UdpCameraReceiver::openSocket() {
  sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_ < 0) {
    std::cerr << "socket() failed\n";
    return false;
  }

  int reuse = 1;
  ::setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  // Large recv buffer to absorb chunk bursts without kernel drops.
  int rcvbuf = 4 * 1024 * 1024;
  ::setsockopt(sock_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons((uint16_t)cfg_.bind_port);

  if (::inet_pton(AF_INET, cfg_.bind_ip.c_str(), &addr.sin_addr) != 1) {
    std::cerr << "inet_pton() failed for bind_ip=" << cfg_.bind_ip << "\n";
    return false;
  }

  if (::bind(sock_, (sockaddr*)&addr, sizeof(addr)) < 0) {
    std::cerr << "bind() failed\n";
    return false;
  }

  return true;
}

bool UdpCameraReceiver::start()
{
  if (running_.load()) return true;
  if (!openSocket()) {
    if (sock_ >= 0) { ::close(sock_); sock_ = -1; }
    return false;
  }

  running_.store(true);
  rx_thread_     = new std::thread(&UdpCameraReceiver::rxLoop, this);
  decode_thread_ = new std::thread(&UdpCameraReceiver::decodeLoop, this);
  return true;
}

void UdpCameraReceiver::stop()
{
  running_.store(false);

  // Wake decode thread so it can exit.
  decode_cv_.notify_all();

  if (rx_thread_) {
    rx_thread_->join();
    delete rx_thread_;
    rx_thread_ = nullptr;
  }

  if (decode_thread_) {
    decode_thread_->join();
    delete decode_thread_;
    decode_thread_ = nullptr;
  }

  if (sock_ >= 0) {
    ::close(sock_);
    sock_ = -1;
  }
}

uint64_t UdpCameraReceiver::lastPublishedFrameId() const
{
  return latest_id_.load(std::memory_order_relaxed);
}

bool UdpCameraReceiver::read(std::shared_ptr<const ReceivedCameraFrame>& out, CameraReadMode mode)
{
  auto cur = std::atomic_load(&latest_);
  if (!cur) return false;

  std::lock_guard<std::mutex> lk(read_mtx_);
  if (mode == CameraReadMode::NewOnly && cur->frame_id == last_read_id_) {
    return false;
  }

  last_read_id_ = cur->frame_id;
  out = std::move(cur);
  return true;
}

bool UdpCameraReceiver::read(cv::Mat& out_bgr, CameraReadMode mode, bool clone)
{
  std::shared_ptr<const ReceivedCameraFrame> f;
  if (!read(f, mode)) return false;

  out_bgr = clone ? f->bgr.clone() : f->bgr;
  return true;
}

bool UdpCameraReceiver::waitForNew(std::shared_ptr<const ReceivedCameraFrame>& out,
                                   std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;

  std::unique_lock<std::mutex> lk(cv_mtx_);
  while (true) {
    lk.unlock();
    if (read(out, CameraReadMode::NewOnly)) return true;
    lk.lock();

    if (cv_.wait_until(lk, deadline) == std::cv_status::timeout) {
      lk.unlock();
      return read(out, CameraReadMode::NewOnly);
    }
  }
}

void UdpCameraReceiver::rxLoop()
{
  std::unordered_map<uint64_t, FrameAssembly> inflight;
  inflight.reserve((size_t)std::max(4, cfg_.max_inflight));

  std::vector<uint8_t> buf(65536);

  while (running_.load()) {
    pollfd pfd{};
    pfd.fd = sock_;
    pfd.events = POLLIN;

    const int pr = ::poll(&pfd, 1, 10);
    if (pr > 0 && (pfd.revents & POLLIN)) {
      const ssize_t n = ::recvfrom(sock_, buf.data(), buf.size(), 0, nullptr, nullptr);
      if (n <= 0) continue;
      if ((size_t)n < sizeof(UdpImageChunkHeader)) continue;

      const auto* h = reinterpret_cast<const UdpImageChunkHeader*>(buf.data());
      const uint32_t magic = ntohl(h->magic);
      if (magic != 0x55494D47u) continue;

      const uint16_t version = ntohs(h->version);
      if (version != 1) continue;

      const uint16_t stream_id = ntohs(h->stream_id);
      if (stream_id != cfg_.expected_stream_id) continue;

      const uint64_t frame_id = ntohll_u64(h->frame_id);
      const uint32_t chunk_idx = ntohl(h->chunk_idx);
      const uint32_t chunk_count = ntohl(h->chunk_count);
      const uint32_t payload_size = ntohl(h->payload_size);
      const uint32_t jpeg_size = ntohl(h->jpeg_size);
      const uint32_t width = ntohl(h->width);
      const uint32_t height = ntohl(h->height);
      const uint64_t stamp_ns = ntohll_u64(h->stamp_ns);

      if (chunk_count == 0 || chunk_idx >= chunk_count) continue;
      if (jpeg_size == 0 || jpeg_size > 50u * 1024u * 1024u) continue;  // guard against malformed packets
      if (sizeof(UdpImageChunkHeader) + payload_size > (size_t)n) continue;

      // limita inflight: rimuovi il frame con frame_id più basso (oldest)
      if ((int)inflight.size() > cfg_.max_inflight) {
        auto oldest = std::min_element(inflight.begin(), inflight.end(),
          [](const auto& a, const auto& b){ return a.first < b.first; });
        inflight.erase(oldest);
      }

      auto it = inflight.find(frame_id);
      if (it == inflight.end()) {
        FrameAssembly fa;
        fa.frame_id = frame_id;
        fa.stream_id = stream_id;
        fa.width = width;
        fa.height = height;
        fa.chunk_count = chunk_count;
        fa.jpeg_size = jpeg_size;
        fa.stamp_ns = stamp_ns;
        fa.chunks.resize(chunk_count);
        fa.received.assign(chunk_count, 0);
        fa.first_seen = std::chrono::steady_clock::now();
        it = inflight.emplace(frame_id, std::move(fa)).first;
      } else {
        // consistency check
        if (it->second.chunk_count != chunk_count || it->second.jpeg_size != jpeg_size) {
          inflight.erase(it);
          continue;
        }
      }

      FrameAssembly& fa = it->second;
      if (!fa.received[chunk_idx]) {
        fa.chunks[chunk_idx].assign(buf.begin() + sizeof(UdpImageChunkHeader),
                                    buf.begin() + sizeof(UdpImageChunkHeader) + payload_size);
        fa.received[chunk_idx] = 1;
        fa.received_count++;
      }

      if (fa.complete()) {
        // rebuild jpeg
        std::vector<uint8_t> jpeg;
        jpeg.reserve(fa.jpeg_size);
        for (uint32_t i = 0; i < fa.chunk_count; ++i) {
          const auto& c = fa.chunks[i];
          jpeg.insert(jpeg.end(), c.begin(), c.end());
        }

        // Validate reassembled size before decode.
        if (jpeg.size() != fa.jpeg_size) {
          std::cerr << "WARN: reassembled jpeg size " << jpeg.size()
                    << " != declared " << fa.jpeg_size << " (frame " << fa.frame_id << ")\n";
          inflight.erase(it);
          continue;
        }

        // Hand off JPEG to the decode thread (latest-wins: replaces any
        // pending frame so the decoder always works on the freshest data).
        {
          auto af = std::make_unique<AssembledFrame>();
          af->frame_id = fa.frame_id;
          af->stamp_ns = fa.stamp_ns;
          af->width    = fa.width;
          af->height   = fa.height;
          af->jpeg     = std::move(jpeg);

          std::lock_guard<std::mutex> lk(decode_mtx_);
          pending_decode_ = std::move(af);
        }
        decode_cv_.notify_one();

        inflight.erase(it);
      }
    }

    // cleanup timeout frame incompleti
    const auto now = std::chrono::steady_clock::now();
    for (auto it = inflight.begin(); it != inflight.end(); ) {
      const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - it->second.first_seen).count();
      if (age_ms > cfg_.frame_timeout_ms) it = inflight.erase(it);
      else ++it;
    }
  }
}

void UdpCameraReceiver::decodeLoop()
{
  while (running_.load()) {
    std::unique_ptr<AssembledFrame> af;
    {
      std::unique_lock<std::mutex> lk(decode_mtx_);
      decode_cv_.wait_for(lk, std::chrono::milliseconds(10), [&]{
        return pending_decode_ != nullptr || !running_.load();
      });
      if (!running_.load()) break;
      if (!pending_decode_) continue;
      af = std::move(pending_decode_);
    }

    cv::Mat img = cv::imdecode(af->jpeg, cv::IMREAD_COLOR);
    if (!img.empty()) {
      auto fr = std::make_shared<ReceivedCameraFrame>();
      fr->frame_id    = af->frame_id;
      fr->stamp_ns    = af->stamp_ns;
      fr->width       = af->width;
      fr->height      = af->height;
      fr->received_at = std::chrono::steady_clock::now();
      fr->bgr         = img;

      if (cfg_.store_jpeg) fr->jpeg = std::move(af->jpeg);

      std::shared_ptr<const ReceivedCameraFrame> fr_const = fr;
      std::atomic_store(&latest_, fr_const);
      latest_id_.store(fr->frame_id, std::memory_order_relaxed);

      cv_.notify_all();
    }
  }
}
