#include "rkfast/tracker.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace rkfast {
namespace {

constexpr float kMinExtent = 2.0f;
constexpr double kTimeEpsilon = 1e-6;

bool Finite(float value) { return std::isfinite(value); }

bool ValidBox(const Box& box) {
  return Finite(box.x1) && Finite(box.y1) && Finite(box.x2) &&
         Finite(box.y2) && box.x2 > box.x1 && box.y2 > box.y1;
}

bool IsRknn(const std::string& source) {
  return source.find("rknn") != std::string::npos;
}

bool IsRoi(const std::string& source) {
  return source.find("roi") != std::string::npos;
}

}  // namespace

RectTracker::RectTracker(const AppConfig& config)
    : config_(config), kalman_(6, 4, 0, CV_32F) {
  cv::setIdentity(kalman_.transitionMatrix);
  kalman_.measurementMatrix = cv::Mat::zeros(4, 6, CV_32F);
  kalman_.measurementMatrix.at<float>(0, 0) = 1.0f;
  kalman_.measurementMatrix.at<float>(1, 1) = 1.0f;
  kalman_.measurementMatrix.at<float>(2, 4) = 1.0f;
  kalman_.measurementMatrix.at<float>(3, 5) = 1.0f;
  cv::setIdentity(kalman_.processNoiseCov);
  cv::setIdentity(kalman_.measurementNoiseCov);
  cv::setIdentity(kalman_.errorCovPost);
  kalman_.statePost = cv::Mat::zeros(6, 1, CV_32F);
}

bool RectTracker::Update(const Detection& detection, double timestamp,
                         const cv::Size& frame_size, const std::string& source,
                         uint64_t measurement_frame_seq,
                         uint64_t query_frame_seq,
                         double measurement_timestamp) {
  return ApplyMeasurement(detection.box, detection.center, detection.conf,
                          std::max(1, detection.candidate_count), timestamp,
                          frame_size, source, measurement_frame_seq,
                          query_frame_seq, measurement_timestamp);
}

bool RectTracker::Update(const RefinedRect& rect, double timestamp,
                         const cv::Size& frame_size, const std::string& source,
                         uint64_t measurement_frame_seq,
                         uint64_t query_frame_seq,
                         double measurement_timestamp) {
  for (const auto& corner : rect.corners) {
    if (!Finite(corner.x) || !Finite(corner.y)) {
      return false;
    }
  }
  const bool accepted =
      ApplyMeasurement(rect.box, rect.center, rect.score, 1, timestamp,
                       frame_size, source, measurement_frame_seq,
                       query_frame_seq, measurement_timestamp);
  if (accepted && UpdateCorners(rect.corners)) {
    last_corner_timestamp_ = timestamp;
  }
  return accepted;
}

bool RectTracker::Reacquire(const Detection& detection,
                            double measurement_timestamp,
                            double query_timestamp,
                            const cv::Size& frame_size,
                            uint64_t measurement_frame_seq,
                            uint64_t query_frame_seq) {
  if (!ValidBox(detection.box) || !Finite(detection.center.x) ||
      !Finite(detection.center.y) || !std::isfinite(measurement_timestamp) ||
      !std::isfinite(query_timestamp) ||
      query_timestamp + kTimeEpsilon < measurement_timestamp ||
      query_timestamp - measurement_timestamp > config_.prediction_timeout) {
    return false;
  }
  InitializeFilter(detection.box, detection.center, measurement_timestamp,
                   frame_size, "rknn", detection.conf,
                   std::max(1, detection.candidate_count),
                   measurement_frame_seq, measurement_frame_seq,
                   measurement_timestamp);
  if (!PredictTo(query_timestamp, frame_size)) {
    return false;
  }
  query_frame_seq_ = query_frame_seq;
  acquisition_started_timestamp_ = query_timestamp;
  const double age = query_timestamp - measurement_timestamp;
  const double tau = std::max(1e-3, config_.confidence_decay_tau);
  conf_ = measurement_conf_ * static_cast<float>(std::exp(-age / tau));
  misses_ = age > kTimeEpsilon ? 1 : 0;
  state_ = "ACQUIRING";
  source_ = "rknn";
  return true;
}

void RectTracker::Predict(double timestamp, const cv::Size& frame_size,
                          uint64_t frame_seq) {
  if (!initialized_) {
    state_ = "SEARCHING";
    source_ = "none";
    query_frame_seq_ = frame_seq;
    return;
  }
  if (!std::isfinite(timestamp) ||
      timestamp + kTimeEpsilon < state_timestamp_) {
    return;
  }

  const bool new_query = frame_seq == 0 || frame_seq != query_frame_seq_;
  const double prev_state_ts = state_timestamp_;
  PredictTo(timestamp, frame_size);
  query_frame_seq_ = frame_seq;
  if (new_query) {
    ++misses_;
  }

  // Coasting on prediction (a genuinely dropped measurement): bleed off the
  // velocity estimate so a stale constant-velocity extrapolation does not fly
  // past the target and overshoot. Without this the box keeps translating at
  // the last measured speed through every dropped frame; when the target
  // reappears the prediction has run off and the innovation gate rejects it,
  // turning a brief dropout into a LOST. predict_decay in (0,1); 1 disables.
  // misses_ == 1 is the ordinary per-frame increment right after a fresh
  // Update, so only decay from the second consecutive miss onward.
  if (config_.predict_decay > 0.0f && config_.predict_decay < 1.0f &&
      new_query && misses_ >= 2) {
    kalman_.statePost.at<float>(2) *= config_.predict_decay;
    kalman_.statePost.at<float>(3) *= config_.predict_decay;
  }

  // Coasting on prediction (a genuinely dropped measurement): shrink the box
  // toward its center so the extrapolated rectangle keeps hugging the target
  // rather than freezing at a possibly motion-inflated size. misses_ == 1 is
  // the ordinary per-frame increment right after a fresh Update, so only shrink
  // from the second consecutive miss onward.
  if (config_.predict_shrink_rate > 0.0f && misses_ >= 2) {
    const double dt = timestamp - prev_state_ts;
    if (dt > kTimeEpsilon) {
      const float old_w = Width();
      const float old_h = Height();
      const float decay =
          static_cast<float>(config_.predict_shrink_rate * dt);
      kalman_.statePost.at<float>(4) -= decay;
      kalman_.statePost.at<float>(5) -= decay;
      ClampState(frame_size);
      if (has_corners_) {
        TransformCorners(State(0), State(1), old_w, old_h, State(0), State(1),
                         Width(), Height());
      }
    }
  }

  const double age =
      std::max(0.0, state_timestamp_ - last_measurement_timestamp_);
  if (age > config_.prediction_timeout) {
    state_ = "LOST";
    source_ = "none";
    conf_ = 0.0f;
    count_ = 0;
    has_corners_ = false;
    return;
  }

  state_ = consecutive_measurements_ < 2 ? "ACQUIRING" : "PREDICT_ONLY";
  source_ = "predict";
  const double tau = std::max(1e-3, config_.confidence_decay_tau);
  conf_ = measurement_conf_ * static_cast<float>(std::exp(-age / tau));
  if (has_corners_ &&
      state_timestamp_ - last_corner_timestamp_ > config_.homography_timeout) {
    has_corners_ = false;
  }
}

void RectTracker::MarkDetectMiss(double timestamp, const cv::Size& frame_size,
                                 uint64_t frame_seq) {
  Predict(timestamp, frame_size, frame_seq);
}

std::optional<Box> RectTracker::PriorBox() const {
  if (!initialized_ || state_ == "LOST" || state_ == "SEARCHING" ||
      !IsFresh(state_timestamp_)) {
    return std::nullopt;
  }
  const float w = Width();
  const float h = Height();
  return ClipBox(Box{State(0) - w * 0.5f, State(1) - h * 0.5f,
                     State(0) + w * 0.5f, State(1) + h * 0.5f},
                 frame_size_.width, frame_size_.height);
}

Box RectTracker::ProjectBox(const Box& box, double source_timestamp,
                            double target_timestamp,
                            const cv::Size& frame_size) const {
  if (!ValidBox(box) || !std::isfinite(source_timestamp) ||
      !std::isfinite(target_timestamp) || target_timestamp <= source_timestamp ||
      !initialized_) {
    return ClipBox(box, frame_size.width, frame_size.height);
  }
  const double dt = std::min(target_timestamp - source_timestamp,
                             std::max(0.0, config_.prediction_timeout));
  const float dx = State(2) * static_cast<float>(dt);
  const float dy = State(3) * static_cast<float>(dt);
  return ClipBox(Box{box.x1 + dx, box.y1 + dy, box.x2 + dx, box.y2 + dy},
                 frame_size.width, frame_size.height);
}

RefineHints RectTracker::Hints() const {
  RefineHints hints;
  hints.prior_box = PriorBox();
  if (initialized_) {
    const float w = Width();
    const float h = Height();
    hints.prior_aspect =
        std::min(w, h) / std::max(1.0f, std::max(w, h));
    hints.speed = std::hypot(State(2), State(3));
  }
  return hints;
}

TrackSnapshot RectTracker::Snapshot() const {
  TrackSnapshot out;
  out.query_frame_seq = query_frame_seq_;
  out.last_measurement_frame_seq = last_measurement_frame_seq_;
  out.state_ts = state_timestamp_;
  out.last_measurement_ts = last_measurement_timestamp_;
  out.publish_ts = NowSeconds();
  if (initialized_) {
    out.state_age_ms =
        std::max(0.0, out.publish_ts - state_timestamp_) * 1000.0;
    out.measurement_age_ms =
        std::max(0.0, out.publish_ts - last_measurement_timestamp_) * 1000.0;
  }
  out.active =
      initialized_ && state_ != "SEARCHING" && state_ != "LOST";
  out.misses = misses_;
  out.state = initialized_ ? state_ : "SEARCHING";
  out.source = out.active ? source_ : "none";
  out.predicted_only =
      initialized_ &&
      state_timestamp_ - last_measurement_timestamp_ > kTimeEpsilon;
  out.conf = out.active ? conf_ : 0.0f;
  out.count = out.active ? count_ : 0;
  if (!out.active) {
    return out;
  }

  const float w = Width();
  const float h = Height();
  out.center = cv::Point2f(State(0), State(1));
  out.vx = State(2);
  out.vy = State(3);
  out.predict_center =
      cv::Point2f(State(0) + State(2) * config_.lead_time,
                  State(1) + State(3) * config_.lead_time);
  out.predict_center.x =
      ClampFloat(out.predict_center.x, 0.0f,
                 static_cast<float>(std::max(0, frame_size_.width - 1)));
  out.predict_center.y =
      ClampFloat(out.predict_center.y, 0.0f,
                 static_cast<float>(std::max(0, frame_size_.height - 1)));
  out.box = ClipBox(Box{State(0) - w * 0.5f, State(1) - h * 0.5f,
                        State(0) + w * 0.5f, State(1) + h * 0.5f},
                    frame_size_.width, frame_size_.height);
  out.has_corners =
      has_corners_ &&
      state_timestamp_ - last_corner_timestamp_ <= config_.homography_timeout;
  if (out.has_corners) {
    out.corners = corners_;
  }
  return out;
}

bool RectTracker::ApplyMeasurement(const Box& box, const cv::Point2f& center,
                                   float conf, int count, double timestamp,
                                   const cv::Size& frame_size,
                                   const std::string& source,
                                   uint64_t measurement_frame_seq,
                                   uint64_t query_frame_seq,
                                   double measurement_timestamp) {
  if (measurement_timestamp < 0.0) {
    measurement_timestamp = timestamp;
  }
  if (query_frame_seq == 0) {
    query_frame_seq = measurement_frame_seq;
  }
  if (!ValidBox(box) || !Finite(center.x) || !Finite(center.y) ||
      !std::isfinite(timestamp) || frame_size.width <= 1 ||
      frame_size.height <= 1 || !std::isfinite(measurement_timestamp) ||
      measurement_timestamp > timestamp + kTimeEpsilon ||
      timestamp - measurement_timestamp > config_.prediction_timeout) {
    return false;
  }
  if (initialized_ && timestamp + kTimeEpsilon < state_timestamp_) {
    return false;
  }

  const bool anchor = IsRknn(source);
  if (!initialized_ || state_ == "LOST" || state_ == "SEARCHING" ||
      !IsFresh(timestamp)) {
    if (!anchor) {
      return false;
    }
    InitializeFilter(box, center, timestamp, frame_size, source, conf, count,
                     measurement_frame_seq, query_frame_seq,
                     measurement_timestamp);
    return true;
  }

  if (consecutive_measurements_ < 2 &&
      timestamp - acquisition_started_timestamp_ > config_.acquire_timeout) {
    if (!anchor) {
      state_ = "LOST";
      source_ = "none";
      conf_ = 0.0f;
      return false;
    }
    InitializeFilter(box, center, timestamp, frame_size, source, conf, count,
                     measurement_frame_seq, query_frame_seq,
                     measurement_timestamp);
    return true;
  }

  if (!PredictTo(timestamp, frame_size)) {
    return false;
  }
  ConfigureMeasurementNoise(source);
  cv::Mat measurement(4, 1, CV_32F);
  measurement.at<float>(0) = center.x;
  measurement.at<float>(1) = center.y;
  measurement.at<float>(2) =
      std::log(std::max(kMinExtent, BoxWidth(box)));
  measurement.at<float>(3) =
      std::log(std::max(kMinExtent, BoxHeight(box)));

  const GateResult gate = PassInnovationGate(measurement);
  if (!gate.center_ok) {
    // A genuine center outlier. Normally reject and coast. But if the filter
    // has been rejecting for a while it has probably diverged (bad velocity /
    // stale state), so let a fresh ANCHOR (RKNN) measurement force-re-seed it
    // rather than coast all the way into LOST.
    ++consecutive_gate_rejections_;
    if (anchor && config_.gate_reset_rejections > 0 &&
        consecutive_gate_rejections_ >= config_.gate_reset_rejections) {
      InitializeFilter(box, center, timestamp, frame_size, source, conf, count,
                       measurement_frame_seq, query_frame_seq,
                       measurement_timestamp);
      return true;
    }
    return false;
  }
  consecutive_gate_rejections_ = 0;
  if (!gate.size_ok) {
    // Center is trustworthy but the box size is not (motion blur / partial
    // occlusion inflated it). Correct the center as usual, but massively
    // distrust the size sample so the established size is held instead of
    // snapping to the bad value. Center correction is unaffected: the noise
    // blocks are independent.
    kalman_.measurementNoiseCov.at<float>(2, 2) *= 1.0e6f;
    kalman_.measurementNoiseCov.at<float>(3, 3) *= 1.0e6f;
  }

  const float old_x = State(0);
  const float old_y = State(1);
  const float old_w = Width();
  const float old_h = Height();
  kalman_.correct(measurement);
  ClampState(frame_size);
  if (has_corners_) {
    TransformCorners(old_x, old_y, old_w, old_h, State(0), State(1), Width(),
                     Height());
  }

  frame_size_ = frame_size;
  state_timestamp_ = timestamp;
  last_measurement_timestamp_ = measurement_timestamp;
  last_measurement_frame_seq_ = measurement_frame_seq;
  query_frame_seq_ = query_frame_seq;
  misses_ = 0;
  measurement_conf_ = ClampFloat(conf, 0.0f, 1.0f);
  conf_ = measurement_conf_;
  count_ = std::max(1, count);
  source_ = source;
  if (IsRoi(source)) {
    ++consecutive_measurements_;
  }
  state_ = consecutive_measurements_ >= 2 ? "TRACKING" : "ACQUIRING";
  return true;
}

void RectTracker::InitializeFilter(const Box& box, const cv::Point2f& center,
                                   double timestamp,
                                   const cv::Size& frame_size,
                                   const std::string& source, float conf,
                                   int count,
                                   uint64_t measurement_frame_seq,
                                   uint64_t query_frame_seq,
                                   double measurement_timestamp) {
  cv::setIdentity(kalman_.transitionMatrix);
  kalman_.statePost = cv::Mat::zeros(6, 1, CV_32F);
  kalman_.statePost.at<float>(0) = center.x;
  kalman_.statePost.at<float>(1) = center.y;
  kalman_.statePost.at<float>(4) =
      std::log(std::max(kMinExtent, BoxWidth(box)));
  kalman_.statePost.at<float>(5) =
      std::log(std::max(kMinExtent, BoxHeight(box)));
  kalman_.statePre = kalman_.statePost.clone();
  kalman_.errorCovPost = cv::Mat::zeros(6, 6, CV_32F);
  kalman_.errorCovPost.at<float>(0, 0) = 25.0f;
  kalman_.errorCovPost.at<float>(1, 1) = 25.0f;
  kalman_.errorCovPost.at<float>(2, 2) = 250000.0f;
  kalman_.errorCovPost.at<float>(3, 3) = 250000.0f;
  kalman_.errorCovPost.at<float>(4, 4) = 0.05f;
  kalman_.errorCovPost.at<float>(5, 5) = 0.05f;
  kalman_.errorCovPre = kalman_.errorCovPost.clone();
  ConfigureMeasurementNoise(source);

  initialized_ = true;
  frame_size_ = frame_size;
  state_timestamp_ = timestamp;
  last_measurement_timestamp_ = measurement_timestamp;
  acquisition_started_timestamp_ = timestamp;
  last_measurement_frame_seq_ = measurement_frame_seq;
  query_frame_seq_ =
      query_frame_seq == 0 ? measurement_frame_seq : query_frame_seq;
  consecutive_measurements_ = IsRoi(source) ? 1 : 0;
  consecutive_gate_rejections_ = 0;
  misses_ = 0;
  measurement_conf_ = ClampFloat(conf, 0.0f, 1.0f);
  conf_ = measurement_conf_;
  count_ = std::max(1, count);
  state_ = "ACQUIRING";
  source_ = source;
  has_corners_ = false;
  last_corner_timestamp_ = 0.0;
  ClampState(frame_size);
}

bool RectTracker::PredictTo(double timestamp, const cv::Size& frame_size) {
  if (!initialized_ || !std::isfinite(timestamp) ||
      timestamp + kTimeEpsilon < state_timestamp_) {
    return false;
  }
  frame_size_ = frame_size;
  const double raw_dt = timestamp - state_timestamp_;
  if (raw_dt <= kTimeEpsilon) {
    return true;
  }

  const float old_x = State(0);
  const float old_y = State(1);
  const float old_w = Width();
  const float old_h = Height();
  const double dt =
      std::min(raw_dt, std::max(0.01, config_.prediction_timeout));
  ConfigureTransition(dt);
  kalman_.predict();
  ClampState(frame_size);
  state_timestamp_ = timestamp;
  if (has_corners_) {
    TransformCorners(old_x, old_y, old_w, old_h, State(0), State(1), Width(),
                     Height());
  }
  return true;
}

void RectTracker::ConfigureTransition(double dt) {
  cv::setIdentity(kalman_.transitionMatrix);
  kalman_.transitionMatrix.at<float>(0, 2) = static_cast<float>(dt);
  kalman_.transitionMatrix.at<float>(1, 3) = static_cast<float>(dt);
  kalman_.processNoiseCov = cv::Mat::zeros(6, 6, CV_32F);

  const double sigma = std::max(1.0f, config_.kalman_process_noise);
  const double q = sigma * sigma;
  const double dt2 = dt * dt;
  const float q_pos = static_cast<float>(0.25 * dt2 * dt2 * q);
  const float q_cross = static_cast<float>(0.5 * dt2 * dt * q);
  const float q_vel = static_cast<float>(dt2 * q);
  for (const auto& pair :
       {std::pair<int, int>{0, 2}, std::pair<int, int>{1, 3}}) {
    kalman_.processNoiseCov.at<float>(pair.first, pair.first) = q_pos;
    kalman_.processNoiseCov.at<float>(pair.first, pair.second) = q_cross;
    kalman_.processNoiseCov.at<float>(pair.second, pair.first) = q_cross;
    kalman_.processNoiseCov.at<float>(pair.second, pair.second) = q_vel;
  }
  const float size_sigma =
      std::max(1e-4f, config_.kalman_size_noise * 0.25f);
  const float size_q = size_sigma * size_sigma * static_cast<float>(dt);
  kalman_.processNoiseCov.at<float>(4, 4) = size_q;
  kalman_.processNoiseCov.at<float>(5, 5) = size_q;
}

void RectTracker::ConfigureMeasurementNoise(const std::string& source) {
  kalman_.measurementNoiseCov = cv::Mat::zeros(4, 4, CV_32F);
  const bool roi = IsRoi(source);
  const float center_sigma =
      std::max(0.1f, roi ? config_.kalman_roi_noise
                         : config_.kalman_rknn_noise);
  float size_sigma =
      std::max(1e-4f, config_.kalman_size_noise * (roi ? 1.0f : 2.0f));
  // Motion blur inflates the measured box; the faster the target moves the less
  // we trust its reported SIZE, so the established (true) size is held instead
  // of growing. Center tracking is unaffected. Scales from 1x (still) up to
  // size_motion_noise_gain (at/above motion_speed_ref px/s).
  if (config_.size_motion_noise_gain > 1.0f && config_.motion_speed_ref > 1.0f) {
    const float speed = std::hypot(State(2), State(3));
    const float motion =
        ClampFloat(speed / config_.motion_speed_ref, 0.0f, 1.0f);
    size_sigma *= 1.0f + motion * (config_.size_motion_noise_gain - 1.0f);
  }
  kalman_.measurementNoiseCov.at<float>(0, 0) =
      center_sigma * center_sigma;
  kalman_.measurementNoiseCov.at<float>(1, 1) =
      center_sigma * center_sigma;
  kalman_.measurementNoiseCov.at<float>(2, 2) = size_sigma * size_sigma;
  kalman_.measurementNoiseCov.at<float>(3, 3) = size_sigma * size_sigma;
}

RectTracker::GateResult RectTracker::PassInnovationGate(
    const cv::Mat& measurement) const {
  GateResult result;
  const cv::Mat innovation =
      measurement - kalman_.measurementMatrix * kalman_.statePost;
  const cv::Mat covariance =
      kalman_.measurementMatrix * kalman_.errorCovPost *
          kalman_.measurementMatrix.t() +
      kalman_.measurementNoiseCov;

  // Gate center [0,1] and size [2,3] as two INDEPENDENT 2-DoF chi-square tests.
  // The measurement blocks are uncorrelated (block-diagonal cov), so slicing
  // the 2x2 diagonal sub-blocks is exact, not an approximation. This is the key
  // motion-robustness fix: a motion-blur-inflated size no longer drags the
  // joint distance over threshold and rejects a perfectly good center.
  const auto block_ok = [&](int i, double threshold) -> bool {
    const cv::Mat cov_block = covariance(cv::Rect(i, i, 2, 2)).clone();
    const cv::Mat inn_block = innovation(cv::Rect(0, i, 1, 2)).clone();
    cv::Mat solved;
    if (!cv::solve(cov_block, inn_block, solved, cv::DECOMP_CHOLESKY)) {
      return false;
    }
    const double distance = inn_block.dot(solved);
    return std::isfinite(distance) && distance <= threshold;
  };

  result.center_ok = block_ok(0, config_.kalman_center_gate);
  result.size_ok = block_ok(2, config_.kalman_size_gate);
  return result;
}

void RectTracker::ClampState(const cv::Size& frame_size) {
  const float max_x =
      static_cast<float>(std::max(1, frame_size.width - 1));
  const float max_y =
      static_cast<float>(std::max(1, frame_size.height - 1));
  kalman_.statePost.at<float>(0) = ClampFloat(State(0), 0.0f, max_x);
  kalman_.statePost.at<float>(1) = ClampFloat(State(1), 0.0f, max_y);
  const float max_speed = std::max(1.0f, config_.max_track_speed);
  kalman_.statePost.at<float>(2) =
      ClampFloat(State(2), -max_speed, max_speed);
  kalman_.statePost.at<float>(3) =
      ClampFloat(State(3), -max_speed, max_speed);
  kalman_.statePost.at<float>(4) =
      ClampFloat(State(4), std::log(kMinExtent),
                 std::log(static_cast<float>(std::max(2, frame_size.width))));
  kalman_.statePost.at<float>(5) =
      ClampFloat(State(5), std::log(kMinExtent),
                 std::log(static_cast<float>(std::max(2, frame_size.height))));
}

void RectTracker::TransformCorners(float old_x, float old_y, float old_w,
                                   float old_h, float new_x, float new_y,
                                   float new_w, float new_h) {
  if (!has_corners_) {
    return;
  }
  const float sx = new_w / std::max(kMinExtent, old_w);
  const float sy = new_h / std::max(kMinExtent, old_h);
  for (auto& corner : corners_) {
    corner.x = new_x + (corner.x - old_x) * sx;
    corner.y = new_y + (corner.y - old_y) * sy;
  }
}

bool RectTracker::UpdateCorners(
    const std::array<cv::Point2f, 4>& corners) {
  for (const auto& corner : corners) {
    if (!Finite(corner.x) || !Finite(corner.y)) {
      return false;
    }
  }
  const float alpha = ClampFloat(config_.corner_alpha, 0.0f, 1.0f);
  if (!has_corners_) {
    corners_ = corners;
    has_corners_ = true;
    return true;
  }
  float max_jump = 0.0f;
  for (size_t i = 0; i < corners.size(); ++i) {
    max_jump = std::max(
        max_jump, static_cast<float>(cv::norm(corners[i] - corners_[i])));
  }
  if (max_jump > config_.corner_reset_dist) {
    corners_ = corners;
    return true;
  }
  for (size_t i = 0; i < corners.size(); ++i) {
    corners_[i] = corners_[i] * (1.0f - alpha) + corners[i] * alpha;
  }
  return true;
}

float RectTracker::State(int index) const {
  return kalman_.statePost.at<float>(index);
}

float RectTracker::Width() const { return std::exp(State(4)); }

float RectTracker::Height() const { return std::exp(State(5)); }

bool RectTracker::IsFresh(double now) const {
  return initialized_ && std::isfinite(now) &&
         now + kTimeEpsilon >= last_measurement_timestamp_ &&
         now - last_measurement_timestamp_ <= config_.prediction_timeout;
}

}  // namespace rkfast
