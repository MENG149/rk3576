#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "rkfast/config.hpp"

namespace rkfast {

class CameraSource {
 public:
  explicit CameraSource(const AppConfig& config);
  ~CameraSource();

  bool Open(std::string* error);
  void Close();

  bool ReadLatest(cv::Mat* frame, double* timestamp, uint64_t* seq) const;

 private:
  void CaptureLoop();

  AppConfig config_;
  mutable std::mutex mutex_;
  cv::VideoCapture cap_;
  cv::Mat latest_;
  double timestamp_ = 0.0;
  uint64_t seq_ = 0;
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

}  // namespace rkfast
