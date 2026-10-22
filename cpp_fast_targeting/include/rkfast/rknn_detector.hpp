#pragma once

#include <atomic>
#include <array>
#include <cstddef>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include <rknn_api.h>

#include "rkfast/config.hpp"
#include "rkfast/rga_preprocess.hpp"
#include "rkfast/types.hpp"

namespace rkfast {

class RknnDetector {
 public:
  explicit RknnDetector(const AppConfig& config);
  ~RknnDetector();
  RknnDetector(const RknnDetector&) = delete;
  RknnDetector& operator=(const RknnDetector&) = delete;
  RknnDetector(RknnDetector&&) = delete;
  RknnDetector& operator=(RknnDetector&&) = delete;

  bool Open(std::string* error);
  void Close();
  bool Warmup(int count, std::string* error);

  std::optional<Detection> Infer(
      const cv::Mat& frame,
      const std::optional<Box>& prior,
      InferTimings* timings,
      std::string* error);

 private:
  struct PreprocessMeta {
    float scale = 1.0f;
    int pad_x = 0;
    int pad_y = 0;
    int frame_w = 0;
    int frame_h = 0;
  };

  struct OutputMatrix {
    int rows = 0;
    int cols = 0;
    std::vector<float> data;
  };

  bool LoadModel(std::string* error);
  bool QueryTensors(std::string* error);
  bool SetupZeroCopyInput(std::string* error);
  const uint8_t* InputData() const;
  size_t InputBytes() const;
  void Preprocess(const cv::Mat& frame, PreprocessMeta* meta);
  OutputMatrix ToMatrix(const rknn_tensor_attr& attr, const rknn_output& output) const;
  std::vector<Detection> DecodeOutputs(
      const std::vector<OutputMatrix>& matrices,
      const PreprocessMeta& meta) const;
  std::optional<Detection> ChooseDetection(
      const std::vector<Detection>& detections,
      const std::optional<Box>& prior,
      int frame_w,
      int frame_h) const;

  AppConfig config_;
  rknn_context context_ = 0;
  std::vector<uint8_t> model_data_;
  rknn_input_output_num io_num_{};
  rknn_tensor_attr input_attr_{};
  rknn_tensor_attr zero_copy_input_attr_{};
  rknn_tensor_mem* input_mem_ = nullptr;
  std::vector<rknn_tensor_attr> output_attrs_;
  int input_width_ = 0;
  int input_height_ = 0;
  int input_channels_ = 0;
  cv::Mat input_rgb_;
  cv::Mat resized_bgr_;
  std::vector<uint8_t> input_chw_;
  std::array<cv::Mat, 3> input_chw_views_{};
  bool input_is_nchw_ = false;
  bool using_zero_copy_input_ = false;
  bool opened_ = false;
  RgaPreprocessor rga_preprocessor_;
};

struct WorkerStats {
  uint64_t submitted = 0;
  uint64_t overwritten = 0;
  uint64_t started = 0;
  uint64_t completed = 0;
};

class DetectorWorker {
 public:
  DetectorWorker(std::unique_ptr<RknnDetector> detector, std::string affinity = {});
  ~DetectorWorker();

  void Start();
  void Stop();

  bool Submit(const FramePacket& packet, const std::optional<Box>& prior);
  bool Busy() const;
  InferResult LatestResult() const;
  WorkerStats Stats() const;

 private:
  void Run();

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::thread thread_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> started_{false};
  bool busy_ = false;
  bool pending_ = false;
  FramePacket pending_packet_;
  std::optional<Box> pending_prior_;
  InferResult latest_result_;
  double smoothed_infer_fps_ = 0.0;
  double smoothed_actual_fps_ = 0.0;
  double completion_window_start_ = 0.0;
  uint64_t completion_window_count_ = 0;
  uint64_t sequence_ = 0;
  WorkerStats stats_;
  std::unique_ptr<RknnDetector> detector_;
  std::string affinity_;
};

}  // namespace rkfast
