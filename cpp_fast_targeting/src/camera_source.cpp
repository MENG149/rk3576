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

std::string FourccToString(int fourcc) {
  std::string value(4, '?');
  for (int i = 0; i < 4; ++i) {
    const unsigned char ch =
        static_cast<unsigned char>((fourcc >> (i * 8)) & 0xff);
    if (std::isprint(ch) != 0) {
      value[static_cast<size_t>(i)] = static_cast<char>(ch);
    }
  }
  return value;
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
  // >=2 is required to reach the sensor's full rate: with a single buffer the
  // V4L2 driver stalls capture until userspace returns it (~51fps ceiling on
  // this board). The capture thread keeps only the newest frame, so the deeper
  // queue costs no added latency. See AppConfig::capture_buffers.
  cap_.set(cv::CAP_PROP_BUFFERSIZE, std::max(1, config_.capture_buffers));

  const int actual_width = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
  const int actual_height = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
  const double actual_fps = cap_.get(cv::CAP_PROP_FPS);
  const int actual_fourcc = static_cast<int>(cap_.get(cv::CAP_PROP_FOURCC));
  std::cout << "camera actual width=" << actual_width
            << " height=" << actual_height << " fps=" << actual_fps
            << " fourcc=" << FourccToString(actual_fourcc) << "\n";

  stop_.store(false);
  thread_ = std::thread(&CameraSource::CaptureLoop, this);
  return true;
}

void CameraSource::Close() {
  stop_.store(true);
  frame_cv_.notify_all();
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

bool CameraSource::WaitLatest(uint64_t after_seq, FramePacket* packet,
                              std::chrono::milliseconds timeout) const {
  if (packet == nullptr) {
    return false;
  }

  std::unique_lock<std::mutex> lock(mutex_);
  const bool ready = frame_cv_.wait_for(lock, timeout, [this, after_seq]() {
    return stop_.load() || (!latest_.empty() && seq_ != after_seq);
  });
  if (!ready || stop_.load() || latest_.empty() || seq_ == after_seq) {
    return false;
  }

  packet->frame = latest_;
  packet->capture_done_ts = timestamp_;
  packet->seq = seq_;
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
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      continue;
    }
    const double capture_done_ts = NowSeconds();
    if (stop_.load()) {
      break;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_ = std::move(frame);
      timestamp_ = capture_done_ts;
      ++seq_;
    }
    frame_cv_.notify_all();
  }
}

}  // namespace rkfast
