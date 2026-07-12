#pragma once

#include <optional>
#include <string>

#include <opencv2/core.hpp>

#include "rkfast/config.hpp"
#include "rkfast/types.hpp"

namespace rkfast {

class RectTracker {
 public:
  explicit RectTracker(const AppConfig& config);

  void Update(const Detection& detection, double timestamp, const cv::Size& frame_size,
              const std::string& source);
  void Update(const RefinedRect& rect, double timestamp, const cv::Size& frame_size,
              const std::string& source);
  void Predict(double timestamp, const cv::Size& frame_size);
  void MarkDetectMiss(double timestamp, const cv::Size& frame_size);

  std::optional<Box> PriorBox() const;
  RefineHints Hints() const;
  TrackSnapshot Snapshot() const;

 private:
  void ApplyMeasurement(const Box& box, const cv::Point2f& center, float conf, int count,
                        double timestamp, const cv::Size& frame_size, const std::string& source);
  void UpdateCorners(const std::array<cv::Point2f, 4>& corners);
  double StepDt(double timestamp);
  void Clamp(const cv::Size& frame_size);

  AppConfig config_;
  bool initialized_ = false;
  double last_timestamp_ = 0.0;
  float x_ = 0.0f;
  float y_ = 0.0f;
  float vx_ = 0.0f;
  float vy_ = 0.0f;
  float w_ = 0.0f;
  float h_ = 0.0f;
  int misses_ = 1000;
  float conf_ = 0.0f;
  int count_ = 0;
  std::string state_ = "SEARCHING";
  std::string source_ = "none";
  std::array<cv::Point2f, 4> corners_{};
  bool has_corners_ = false;
};

}  // namespace rkfast
