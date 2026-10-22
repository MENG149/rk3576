#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "rkfast/config.hpp"
#include "rkfast/types.hpp"

namespace rkfast {

class CameraSource {
 public:
  explicit CameraSource(const AppConfig& config);
  ~CameraSource();

  bool Open(std::string* error);
  void Close();

  bool ReadLatest(cv::Mat* frame, double* timestamp, uint64_t* seq) const;
  bool WaitLatest(uint64_t after_seq, FramePacket* packet,
                  std::chrono::milliseconds timeout) const;

 private:
  void CaptureLoop();

  AppConfig config_;
  mutable std::mutex mutex_;
  mutable std::condition_variable frame_cv_;
  cv::VideoCapture cap_;
  cv::Mat latest_;
  double timestamp_ = 0.0;
  uint64_t seq_ = 0;
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

}  // namespace rkfast
