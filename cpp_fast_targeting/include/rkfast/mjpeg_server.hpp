#pragma once

// Lightweight MJPEG-over-HTTP server for remote debug viewing.
//
// The board is headless and reached over SSH, so cv::imshow is not visible.
// This server streams the annotated debug overlay (the same image DrawDebug
// produces) as multipart/x-mixed-replace JPEG. Point a browser on the host
// machine at http://<board-ip>:<port>/ to watch the live feed and the
// detected-rectangle state.
//
// Design notes:
//  - JPEG encoding only happens while at least one client is connected, so the
//    vision pipeline pays zero cost when nobody is watching.
//  - PushFrame never blocks on the network: it encodes the latest frame under a
//    short lock and wakes per-client sender threads. Slow clients simply miss
//    intermediate frames (latest-frame-wins), they never stall the main loop.

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

namespace rkfast {

class MjpegServer {
 public:
  MjpegServer() = default;
  ~MjpegServer();

  MjpegServer(const MjpegServer&) = delete;
  MjpegServer& operator=(const MjpegServer&) = delete;

  // Bind to 0.0.0.0:port and start accepting clients. Returns false and fills
  // *error on failure (e.g. port already in use).
  bool Start(int port, int jpeg_quality, std::string* error);

  // Encode and publish the newest overlay frame. No-op (and no encode cost)
  // when no client is connected. Safe to call every loop iteration.
  void PushFrame(const cv::Mat& frame);

  // Stop the acceptor, close all client connections, join threads. Idempotent.
  void Stop();

  bool running() const { return running_.load(std::memory_order_relaxed); }
  int client_count() const {
    return client_count_.load(std::memory_order_relaxed);
  }

 private:
  void AcceptLoop();
  void ServeClient(int client_fd);

  int listen_fd_ = -1;
  int port_ = 0;
  int jpeg_quality_ = 70;

  std::atomic<bool> running_{false};
  std::atomic<int> client_count_{0};
  std::thread accept_thread_;

  std::mutex clients_mutex_;
  std::vector<std::thread> client_threads_;

  // Latest encoded JPEG, guarded by frame_mutex_. frame_seq_ increments on each
  // new frame; client threads wait on frame_cv_ for it to change.
  std::mutex frame_mutex_;
  std::condition_variable frame_cv_;
  std::vector<uint8_t> jpeg_buffer_;
  uint64_t frame_seq_ = 0;
};

}  // namespace rkfast
