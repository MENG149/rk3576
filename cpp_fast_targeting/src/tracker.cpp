#include "rkfast/tracker.hpp"

#include <algorithm>
#include <cmath>

namespace rkfast {

RectTracker::RectTracker(const AppConfig& config) : config_(config) {}

void RectTracker::Update(const Detection& detection, double timestamp,
                         const cv::Size& frame_size, const std::string& source) {
  ApplyMeasurement(detection.box, detection.center, detection.conf,
                   std::max(1, detection.candidate_count), timestamp, frame_size,
                   source);
  has_corners_ = false;  // detection has no corners; drop stale smoothing
}

void RectTracker::Update(const RefinedRect& rect, double timestamp,
                         const cv::Size& frame_size, const std::string& source) {
  ApplyMeasurement(rect.box, rect.center, rect.score, 1, timestamp, frame_size,
                   source);
  UpdateCorners(rect.corners);
}

void RectTracker::Predict(double timestamp, const cv::Size& frame_size) {
  if (!initialized_) {
    state_ = "SEARCHING";
    source_ = "none";
    return;
  }
  const double dt = StepDt(timestamp);
  x_ += vx_ * static_cast<float>(dt);
  y_ += vy_ * static_cast<float>(dt);
  Clamp(frame_size);
  if (misses_ > 0) {
    state_ = "LOST";
    source_ = "predict";
    conf_ *= 0.97f;
    // Decay velocity so a stopped target does not let the estimate fly away.
    vx_ *= config_.predict_decay;
    vy_ *= config_.predict_decay;
  }
}

void RectTracker::MarkDetectMiss(double timestamp, const cv::Size& frame_size) {
  if (!initialized_) {
    state_ = "SEARCHING";
    source_ = "none";
    return;
  }
  ++misses_;
  Predict(timestamp, frame_size);
  if (misses_ > config_.max_predict_misses) {
    state_ = "SEARCHING";
    source_ = "none";
    conf_ = 0.0f;
    count_ = 0;
  }
}

std::optional<Box> RectTracker::PriorBox() const {
  if (!initialized_ || misses_ > config_.max_predict_misses) {
    return std::nullopt;
  }
  return Box{x_ - w_ * 0.5f, y_ - h_ * 0.5f, x_ + w_ * 0.5f, y_ + h_ * 0.5f};
}

RefineHints RectTracker::Hints() const {
  RefineHints hints;
  hints.prior_box = PriorBox();
  if (initialized_) {
    const float lo = std::min(w_, h_);
    const float hi = std::max(1.0f, std::max(w_, h_));
    hints.prior_aspect = lo / hi;
    hints.speed = std::hypot(vx_, vy_);
  }
  return hints;
}

TrackSnapshot RectTracker::Snapshot() const {
  TrackSnapshot snapshot;
  snapshot.active = initialized_ && misses_ <= config_.max_predict_misses;
  snapshot.misses = misses_;
  snapshot.state = snapshot.active ? state_ : "SEARCHING";
  snapshot.source = snapshot.active ? source_ : "none";
  snapshot.conf = snapshot.active ? conf_ : 0.0f;
  snapshot.count = snapshot.active ? count_ : 0;
  if (snapshot.active) {
    snapshot.center = cv::Point2f(x_, y_);
    snapshot.vx = vx_;
    snapshot.vy = vy_;
    const float lead = config_.lead_time;
    snapshot.predict_center = cv::Point2f(x_ + vx_ * lead, y_ + vy_ * lead);
    snapshot.box = Box{x_ - w_ * 0.5f, y_ - h_ * 0.5f, x_ + w_ * 0.5f,
                       y_ + h_ * 0.5f};
    snapshot.has_corners = has_corners_;
    snapshot.corners = corners_;
  }
  return snapshot;
}

void RectTracker::ApplyMeasurement(const Box& box, const cv::Point2f& center,
                                   float conf, int count, double timestamp,
                                   const cv::Size& frame_size,
                                   const std::string& source) {
  const float new_w = BoxWidth(box);
  const float new_h = BoxHeight(box);
  const double dt = StepDt(timestamp);

  if (!initialized_ || misses_ > config_.max_predict_misses) {
    x_ = center.x;
    y_ = center.y;
    vx_ = 0.0f;
    vy_ = 0.0f;
    w_ = new_w;
    h_ = new_h;
    initialized_ = true;
  } else {
    const float px = x_ + vx_ * static_cast<float>(dt);
    const float py = y_ + vy_ * static_cast<float>(dt);
    const float rx = center.x - px;
    const float ry = center.y - py;

    x_ = px + config_.track_alpha * rx;
    y_ = py + config_.track_alpha * ry;
    const float dt_safe = static_cast<float>(std::max(1e-3, dt));
    vx_ += config_.track_beta * rx / dt_safe;
    vy_ += config_.track_beta * ry / dt_safe;
    w_ = (1.0f - config_.track_box_alpha) * w_ + config_.track_box_alpha * new_w;
    h_ = (1.0f - config_.track_box_alpha) * h_ + config_.track_box_alpha * new_h;
  }

  Clamp(frame_size);
  misses_ = 0;
  conf_ = conf;
  count_ = count;
  state_ = "LOCKED";
  source_ = source;
}

void RectTracker::UpdateCorners(const std::array<cv::Point2f, 4>& corners) {
  const float a = ClampFloat(config_.corner_alpha, 0.0f, 1.0f);
  if (!has_corners_) {
    corners_ = corners;
    has_corners_ = true;
    return;
  }
  // Reset smoothing if any corner jumped too far (re-acquired target).
  float max_jump = 0.0f;
  for (size_t i = 0; i < 4; ++i) {
    max_jump = std::max(max_jump,
                        static_cast<float>(cv::norm(corners[i] - corners_[i])));
  }
  if (max_jump > config_.corner_reset_dist) {
    corners_ = corners;
    return;
  }
  for (size_t i = 0; i < 4; ++i) {
    corners_[i] = corners_[i] * (1.0f - a) + corners[i] * a;
  }
}

double RectTracker::StepDt(double timestamp) {
  if (last_timestamp_ <= 0.0) {
    last_timestamp_ = timestamp;
    return 1.0 / 60.0;
  }
  const double dt = std::clamp(timestamp - last_timestamp_, 1e-3, 0.25);
  last_timestamp_ = timestamp;
  return dt;
}

void RectTracker::Clamp(const cv::Size& frame_size) {
  x_ = ClampFloat(x_, 0.0f, static_cast<float>(std::max(0, frame_size.width - 1)));
  y_ = ClampFloat(y_, 0.0f, static_cast<float>(std::max(0, frame_size.height - 1)));
  w_ = ClampFloat(w_, 1.0f, static_cast<float>(std::max(1, frame_size.width)));
  h_ = ClampFloat(h_, 1.0f, static_cast<float>(std::max(1, frame_size.height)));
}

}  // namespace rkfast
