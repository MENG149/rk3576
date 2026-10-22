#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

#include <opencv2/core.hpp>

namespace rkfast {

struct Box {
  float x1 = 0.0f;
  float y1 = 0.0f;
  float x2 = 0.0f;
  float y2 = 0.0f;
};

struct Detection {
  Box box;
  cv::Point2f center{0.0f, 0.0f};
  float conf = 0.0f;
  int class_id = 0;
  int candidate_count = 0;
};

struct RefinedRect {
  Box box;
  cv::Point2f center{0.0f, 0.0f};
  std::array<cv::Point2f, 4> corners{};
  float score = 0.0f;
  // Centroid of red pixels inside the refined quad (image coords), when the
  // real target's red concentric rings / center dot are visible. This is the
  // physical bullseye and the preferred aim point; empty on the plain
  // black-frame dataset, so downstream aim falls back to the frame center.
  std::optional<cv::Point2f> red_center;
  int red_pixels = 0;  // red pixel count backing red_center (0 = none)
};

// A camera frame and the monotonic timestamp recorded immediately after the
// backend returned it. The timestamp is deliberately not called exposure_ts:
// OpenCV does not expose a reliable sensor exposure timestamp on this path.
struct FramePacket {
  cv::Mat frame;
  double capture_done_ts = 0.0;
  uint64_t seq = 0;
};

// Optional context passed from the tracker into the refiner so candidate
// scoring and ROI sizing can adapt to motion and the prior estimate.
struct RefineHints {
  std::optional<Box> prior_box;       // previous tracked box (image coords)
  float prior_aspect = 0.0f;          // previous short/long edge ratio, 0 = unknown
  float speed = 0.0f;                 // center speed magnitude in px/s
};

struct TrackSnapshot {
  bool active = false;
  Box box;
  cv::Point2f center{0.0f, 0.0f};
  cv::Point2f predict_center{0.0f, 0.0f};  // lead-compensated aim point
  float vx = 0.0f;                          // px/s
  float vy = 0.0f;                          // px/s
  std::array<cv::Point2f, 4> corners{};     // smoothed corners (image coords)
  bool has_corners = false;
  float conf = 0.0f;
  int count = 0;
  int misses = 0;
  std::string state = "SEARCHING";
  std::string source = "none";
  uint64_t query_frame_seq = 0;
  uint64_t last_measurement_frame_seq = 0;
  double state_ts = 0.0;
  double last_measurement_ts = 0.0;
  double publish_ts = 0.0;
  double state_age_ms = 0.0;
  double measurement_age_ms = 0.0;
  bool predicted_only = false;
};

struct InferTimings {
  uint32_t input_bytes = 0;
  uint64_t input_checksum = 0;
  double preprocess_ms = 0.0;
  double inputs_set_ms = 0.0;
  double rknn_ms = 0.0;
  double outputs_get_ms = 0.0;
  double postprocess_ms = 0.0;
  double select_ms = 0.0;
  double outputs_release_ms = 0.0;
  double total_ms = 0.0;
};

struct InferResult {
  uint64_t seq = 0;
  uint64_t source_frame_seq = 0;
  std::optional<Detection> detection;
  // Processing-capacity FPS: reciprocal of one detector invocation.
  double infer_fps = 0.0;
  // Actual completion cadence, including scheduling gaps.
  double actual_fps = 0.0;
  double source_capture_ts = 0.0;
  double infer_start_ts = 0.0;
  double infer_done_ts = 0.0;
  // Compatibility alias for infer_done_ts.
  double timestamp = 0.0;
  InferTimings timings;
};

struct VisualEstimate {
  uint64_t query_frame_seq = 0;
  uint64_t last_measurement_frame_seq = 0;
  double state_ts = 0.0;
  double last_measurement_ts = 0.0;
  double publish_ts = 0.0;
  double state_age_ms = 0.0;
  double measurement_age_ms = 0.0;
  std::string state = "SEARCHING";
  std::string source = "none";
  Box box;
  std::array<cv::Point2f, 4> corners{};
  bool has_corners = false;
  bool has_valid_homography = false;
  cv::Point2f target_center_px{0.0f, 0.0f};
  cv::Matx33f image_to_target = cv::Matx33f::eye();
  cv::Matx33f target_to_image = cv::Matx33f::eye();
  float vx_px_s = 0.0f;
  float vy_px_s = 0.0f;
  float confidence = 0.0f;
};

inline double NowSeconds() {
  using Clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

inline float ClampFloat(float value, float low, float high) {
  return std::max(low, std::min(high, value));
}

inline int ClampInt(int value, int low, int high) {
  return std::max(low, std::min(high, value));
}

inline Box ClipBox(const Box& box, int frame_w, int frame_h) {
  Box clipped;
  clipped.x1 = ClampFloat(box.x1, 0.0f, static_cast<float>(std::max(0, frame_w - 1)));
  clipped.y1 = ClampFloat(box.y1, 0.0f, static_cast<float>(std::max(0, frame_h - 1)));
  clipped.x2 = ClampFloat(box.x2, 0.0f, static_cast<float>(std::max(0, frame_w - 1)));
  clipped.y2 = ClampFloat(box.y2, 0.0f, static_cast<float>(std::max(0, frame_h - 1)));
  if (clipped.x2 < clipped.x1) {
    std::swap(clipped.x1, clipped.x2);
  }
  if (clipped.y2 < clipped.y1) {
    std::swap(clipped.y1, clipped.y2);
  }
  return clipped;
}

inline float BoxWidth(const Box& box) { return std::max(1.0f, box.x2 - box.x1); }

inline float BoxHeight(const Box& box) { return std::max(1.0f, box.y2 - box.y1); }

inline float BoxArea(const Box& box) { return BoxWidth(box) * BoxHeight(box); }

inline cv::Point2f BoxCenter(const Box& box) {
  return cv::Point2f((box.x1 + box.x2) * 0.5f, (box.y1 + box.y2) * 0.5f);
}

inline float BoxIou(const Box& a, const Box& b) {
  const float ix1 = std::max(a.x1, b.x1);
  const float iy1 = std::max(a.y1, b.y1);
  const float ix2 = std::min(a.x2, b.x2);
  const float iy2 = std::min(a.y2, b.y2);
  const float iw = std::max(0.0f, ix2 - ix1);
  const float ih = std::max(0.0f, iy2 - iy1);
  const float inter = iw * ih;
  if (inter <= 0.0f) {
    return 0.0f;
  }
  const float uni = BoxArea(a) + BoxArea(b) - inter;
  return inter / std::max(uni, 1e-6f);
}

inline double SmoothValue(double previous, double current, double alpha) {
  return previous <= 0.0 ? current : previous * (1.0 - alpha) + current * alpha;
}

// Centered pixel offset: origin at frame center, y grows downward.
inline cv::Point2f CenterOffset(const cv::Point2f& p, int frame_w, int frame_h) {
  return cv::Point2f(p.x - frame_w * 0.5f, p.y - frame_h * 0.5f);
}

// Normalized offset in [-1, 1] for both axes (origin at frame center).
inline cv::Point2f NormalizedOffset(const cv::Point2f& p, int frame_w,
                                    int frame_h) {
  const float hx = std::max(1.0f, frame_w * 0.5f);
  const float hy = std::max(1.0f, frame_h * 0.5f);
  const cv::Point2f off = CenterOffset(p, frame_w, frame_h);
  return cv::Point2f(ClampFloat(off.x / hx, -1.0f, 1.0f),
                     ClampFloat(off.y / hy, -1.0f, 1.0f));
}

}  // namespace rkfast
