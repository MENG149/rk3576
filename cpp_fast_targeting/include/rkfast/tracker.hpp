#pragma once

#include <optional>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>

#include "rkfast/config.hpp"
#include "rkfast/types.hpp"

namespace rkfast {

class RectTracker {
 public:
  explicit RectTracker(const AppConfig& config);

  bool Update(const Detection& detection, double timestamp, const cv::Size& frame_size,
              const std::string& source, uint64_t measurement_frame_seq = 0,
              uint64_t query_frame_seq = 0,
              double measurement_timestamp = -1.0);
  bool Update(const RefinedRect& rect, double timestamp, const cv::Size& frame_size,
              const std::string& source, uint64_t measurement_frame_seq = 0,
              uint64_t query_frame_seq = 0,
              double measurement_timestamp = -1.0);
  bool Reacquire(const Detection& detection, double measurement_timestamp,
                 double query_timestamp, const cv::Size& frame_size,
                 uint64_t measurement_frame_seq, uint64_t query_frame_seq);
  void Predict(double timestamp, const cv::Size& frame_size, uint64_t frame_seq = 0);
  void MarkDetectMiss(double timestamp, const cv::Size& frame_size,
                      uint64_t frame_seq = 0);

  std::optional<Box> PriorBox() const;
  Box ProjectBox(const Box& box, double source_timestamp, double target_timestamp,
                 const cv::Size& frame_size) const;
  RefineHints Hints() const;
  TrackSnapshot Snapshot() const;

 private:
  bool ApplyMeasurement(const Box& box, const cv::Point2f& center, float conf, int count,
                        double timestamp, const cv::Size& frame_size,
                        const std::string& source,
                        uint64_t measurement_frame_seq,
                        uint64_t query_frame_seq,
                        double measurement_timestamp);
  void InitializeFilter(const Box& box, const cv::Point2f& center, double timestamp,
                        const cv::Size& frame_size, const std::string& source,
                        float conf, int count,
                        uint64_t measurement_frame_seq,
                        uint64_t query_frame_seq,
                        double measurement_timestamp);
  bool PredictTo(double timestamp, const cv::Size& frame_size);
  void ConfigureTransition(double dt);
  void ConfigureMeasurementNoise(const std::string& source);
  // Result of the decoupled innovation gate: center and size are judged
  // independently so a bad size (motion blur) cannot reject a good center.
  struct GateResult {
    bool center_ok = false;
    bool size_ok = false;
  };
  GateResult PassInnovationGate(const cv::Mat& measurement) const;
  void ClampState(const cv::Size& frame_size);
  void TransformCorners(float old_x, float old_y, float old_w, float old_h,
                        float new_x, float new_y, float new_w, float new_h);
  bool UpdateCorners(const std::array<cv::Point2f, 4>& corners);
  float State(int index) const;
  float Width() const;
  float Height() const;
  bool IsFresh(double now) const;

  AppConfig config_;
  cv::KalmanFilter kalman_;
  bool initialized_ = false;
  double state_timestamp_ = 0.0;
  double last_measurement_timestamp_ = 0.0;
  double acquisition_started_timestamp_ = 0.0;
  double last_corner_timestamp_ = 0.0;
  uint64_t query_frame_seq_ = 0;
  uint64_t last_measurement_frame_seq_ = 0;
  int consecutive_measurements_ = 0;
  int consecutive_gate_rejections_ = 0;
  int misses_ = 1000;
  float conf_ = 0.0f;
  float measurement_conf_ = 0.0f;
  int count_ = 0;
  std::string state_ = "SEARCHING";
  std::string source_ = "none";
  std::array<cv::Point2f, 4> corners_{};
  bool has_corners_ = false;
  cv::Size frame_size_;
};

}  // namespace rkfast
