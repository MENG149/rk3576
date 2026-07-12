#include "rkfast/camera_source.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <iostream>
#include <thread>

#include <opencv2/videoio.hpp>

#include "rkfast/thread_affinity.hpp"
#include "rkfast/types.hpp"

namespace rkfast {

namespace {

bool IsDigits(const std::string& value) {
  return !value.empty() &&
         std::all_of(value.begin(), value.end(),
                     [](unsigned char ch) { return std::isdigit(ch) != 0; });
}

}  // namespace

CameraSource::CameraSource(const AppConfig& config) : config_(config) {}

CameraSource::~CameraSource() { Close(); }

bool CameraSource::Open(std::string* error) {
  const int api = config_.use_v4l2 ? cv::CAP_V4L2 : cv::CAP_ANY;
  if (IsDigits(config_.camera)) {
    cap_.open(std::stoi(config_.camera), api);
  } else {
    cap_.open(config_.camera, api);
  }
  if (!cap_.isOpened() && api != cv::CAP_ANY) {
    if (IsDigits(config_.camera)) {
      cap_.open(std::stoi(config_.camera), cv::CAP_ANY);
    } else {
      cap_.open(config_.camera, cv::CAP_ANY);
    }
  }
  if (!cap_.isOpened()) {
    if (error != nullptr) {
      *error = "could not open camera: " + config_.camera;
    }
    return false;
  }

  if (config_.fourcc.size() >= 4) {
    cap_.set(cv::CAP_PROP_FOURCC,
             cv::VideoWriter::fourcc(config_.fourcc[0], config_.fourcc[1],
                                     config_.fourcc[2], config_.fourcc[3]));
  }
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, config_.width);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, config_.height);
  cap_.set(cv::CAP_PROP_FPS, config_.fps);
  cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

  stop_.store(false);
  thread_ = std::thread(&CameraSource::CaptureLoop, this);
  return true;
}

void CameraSource::Close() {
  stop_.store(true);
  if (thread_.joinable()) {
    thread_.join();
  }
  if (cap_.isOpened()) {
    cap_.release();
  }
}

bool CameraSource::ReadLatest(cv::Mat* frame, double* timestamp, uint64_t* seq) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (latest_.empty()) {
    return false;
  }
  if (frame != nullptr) {
    *frame = latest_;
  }
  if (timestamp != nullptr) {
    *timestamp = timestamp_;
  }
  if (seq != nullptr) {
    *seq = seq_;
  }
  return true;
}

void CameraSource::CaptureLoop() {
  std::string affinity_error;
  if (!config_.camera_affinity.empty() &&
      !BindCurrentThread(config_.camera_affinity, &affinity_error)) {
    std::cerr << "camera affinity warning: " << affinity_error << "\n";
  }
  while (!stop_.load()) {
    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
      // Reduced sleep for higher FPS
      std::this_thread::yield();
      continue;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = std::move(frame);
    timestamp_ = NowSeconds();
    ++seq_;
  }
}

}  // namespace rkfast
