#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "rkfast/aim_writer.hpp"
#include "rkfast/camera_source.hpp"
#include "rkfast/config.hpp"
#include "rkfast/mjpeg_server.hpp"
#include "rkfast/rect_refiner.hpp"
#include "rkfast/rknn_detector.hpp"
#include "rkfast/target_plane.hpp"
#include "rkfast/thread_affinity.hpp"
#include "rkfast/tracker.hpp"
#include "rkfast/types.hpp"

namespace rkfast {
namespace {

constexpr double kMaxRknnAgeSeconds = 0.050;

VisualEstimate BuildVisualEstimate(const TrackSnapshot& track,
                                   const TargetPlane& plane) {
  VisualEstimate out;
  out.query_frame_seq = track.query_frame_seq;
  out.last_measurement_frame_seq = track.last_measurement_frame_seq;
  out.state_ts = track.state_ts;
  out.last_measurement_ts = track.last_measurement_ts;
  out.publish_ts = NowSeconds();
  out.state_age_ms = track.state_age_ms;
  out.measurement_age_ms = track.measurement_age_ms;
  out.state = track.state;
  out.source = track.source;
  out.box = track.box;
  out.corners = track.corners;
  out.has_corners = track.has_corners;
  out.vx_px_s = track.vx;
  out.vy_px_s = track.vy;
  out.confidence = track.conf;
  if (track.active && plane.Valid()) {
    const auto center = plane.TargetToImage(cv::Point2f(0.0f, 0.0f));
    if (center.has_value()) {
      out.has_valid_homography = true;
      out.target_center_px = *center;
      out.image_to_target = plane.ImageToTargetMatrix();
      out.target_to_image = plane.TargetToImageMatrix();
    }
  }
  return out;
}

cv::Mat DrawDebug(const cv::Mat& frame, const TrackSnapshot& track,
                  const VisualEstimate& estimate, double frame_fps,
                  double infer_capacity, double detector_hz,
                  const std::optional<cv::Point2f>& aim_center,
                  bool aim_is_red) {
  cv::Mat overlay = frame.clone();
  const cv::Point image_center(overlay.cols / 2, overlay.rows / 2);
  cv::line(overlay, image_center - cv::Point(18, 0),
           image_center + cv::Point(18, 0), cv::Scalar(80, 80, 80), 1);
  cv::line(overlay, image_center - cv::Point(0, 18),
           image_center + cv::Point(0, 18), cv::Scalar(80, 80, 80), 1);

  if (track.active) {
    const cv::Rect rect(static_cast<int>(std::round(track.box.x1)),
                        static_cast<int>(std::round(track.box.y1)),
                        static_cast<int>(std::round(BoxWidth(track.box))),
                        static_cast<int>(std::round(BoxHeight(track.box))));
    cv::rectangle(overlay, rect, cv::Scalar(0, 180, 255), 2);
    if (track.has_corners) {
      for (int i = 0; i < 4; ++i) {
        cv::line(overlay, track.corners[i], track.corners[(i + 1) % 4],
                 cv::Scalar(0, 255, 0), 2);
      }
    }
    cv::drawMarker(overlay, track.center, cv::Scalar(0, 0, 255),
                   cv::MARKER_CROSS, 14, 2);
    cv::drawMarker(overlay, track.predict_center, cv::Scalar(255, 0, 255),
                   cv::MARKER_TILTED_CROSS, 16, 2);
    if (estimate.has_valid_homography) {
      cv::drawMarker(overlay, estimate.target_center_px,
                     cv::Scalar(255, 255, 0), cv::MARKER_CROSS, 20, 2);
    }
    // Final aim point: red ring = snapped to the physical bullseye, cyan =
    // geometric/frame-center fallback.
    if (aim_center.has_value()) {
      cv::circle(overlay, *aim_center, 7,
                 aim_is_red ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 200, 0),
                 2);
    }
  }

  std::ostringstream status;
  status << track.state << " " << track.source
         << " conf=" << cv::format("%.2f", track.conf)
         << " age=" << cv::format("%.1f", estimate.measurement_age_ms)
         << "ms";
  cv::putText(overlay, status.str(), cv::Point(12, 28),
              cv::FONT_HERSHEY_SIMPLEX, 0.65,
              track.active ? cv::Scalar(0, 180, 255)
                           : cv::Scalar(0, 0, 255),
              2);

  std::ostringstream rates;
  rates << "frame=" << cv::format("%.1f", frame_fps)
        << " infer-cap=" << cv::format("%.1f", infer_capacity)
        << " detect=" << cv::format("%.1f", detector_hz);
  cv::putText(overlay, rates.str(), cv::Point(12, 56),
              cv::FONT_HERSHEY_SIMPLEX, 0.60, cv::Scalar(255, 255, 255), 2);
  return overlay;
}

}  // namespace
}  // namespace rkfast

int main(int argc, char** argv) {
  using namespace rkfast;

  AppConfig config;
  if (!ParseCommandLine(argc, argv, &config)) {
    return argc > 1 ? 1 : 0;
  }
  cv::setNumThreads(std::max(1, config.opencv_threads));
  cv::setUseOptimized(true);

  std::string affinity_error;
  if (!config.main_affinity.empty() &&
      !BindCurrentThread(config.main_affinity, &affinity_error)) {
    std::cerr << "main affinity warning: " << affinity_error << std::endl;
  }

  std::string error;
  CameraSource camera(config);
  if (!camera.Open(&error)) {
    std::cerr << error << std::endl;
    return 1;
  }
  auto detector = std::make_unique<RknnDetector>(config);
  if (!detector->Open(&error) ||
      !detector->Warmup(config.warmup_runs, &error)) {
    std::cerr << error << std::endl;
    return 1;
  }

  DetectorWorker worker(std::move(detector), config.detector_affinity);
  worker.Start();
  RectRefiner refiner(config);
  RectTracker tracker(config);
  TargetPlane target_plane(config.target_width_cm, config.target_height_cm);
  AimWriter writer(config);
  if (!writer.Open(&error)) {
    std::cerr << error << std::endl;
    return 1;
  }

  MjpegServer mjpeg;
  if (config.mjpeg_port > 0) {
    if (!mjpeg.Start(config.mjpeg_port, config.mjpeg_quality, &error)) {
      std::cerr << error << std::endl;
      return 1;
    }
    std::cout << "mjpeg debug stream on http://<board-ip>:" << config.mjpeg_port
              << "/" << std::endl;
  }

  double frame_fps = 0.0;
  double infer_capacity = 0.0;
  double detector_hz = 0.0;
  double refine_ms = 0.0;
  double loop_ms = 0.0;
  double result_age_ms = 0.0;
  double last_loop = NowSeconds();
  double last_stats = last_loop;
  uint64_t last_frame_seq = 0;
  uint64_t last_result_seq = 0;
  uint64_t skipped_camera_frames = 0;
  int frames_until_detection = 0;
  InferTimings latest_timings;
  double last_window_draw = 0.0;

  // Red bullseye aim memory. The physical bullseye offset (target cm) is a
  // pose-invariant quantity, so smoothing it in cm and re-projecting through
  // the current homography each frame keeps the aim point stable AND riding the
  // target's motion/perspective. Image-space EMA is a fallback for frames with
  // no valid homography.
  std::optional<cv::Point2f> red_offset_cm;
  std::optional<cv::Point2f> red_center_px_ema;
  double red_cm_ts = 0.0;
  double red_px_ts = 0.0;
  const auto in_frame = [](const cv::Point2f& p, const cv::Mat& f) {
    return std::isfinite(p.x) && std::isfinite(p.y) && p.x >= 0.0f &&
           p.y >= 0.0f && p.x <= static_cast<float>(f.cols - 1) &&
           p.y <= static_cast<float>(f.rows - 1);
  };

  std::cout << "running vision pipeline";
  if (config.show_window) {
    std::cout << "; press q to exit";
  }
  std::cout << std::endl;

  while (true) {
    FramePacket packet;
    if (!camera.WaitLatest(last_frame_seq, &packet,
                           std::chrono::milliseconds(100)) ||
        packet.frame.empty()) {
      continue;
    }
    if (last_frame_seq != 0 && packet.seq > last_frame_seq + 1) {
      skipped_camera_frames += packet.seq - last_frame_seq - 1;
    }
    last_frame_seq = packet.seq;
    const double loop_start = NowSeconds();

    // The current-frame prediction is the ROI prior. A valid ROI measurement
    // below immediately corrects it at the same capture timestamp.
    tracker.Predict(packet.capture_done_ts, packet.frame.size(), packet.seq);

    const InferResult result = worker.LatestResult();
    const bool new_result =
        result.seq != 0 && result.seq != last_result_seq;
    std::optional<Detection> current_detection;
    std::optional<Detection> source_detection;
    uint64_t detection_source_seq = 0;
    double detection_source_timestamp = 0.0;
    if (new_result) {
      last_result_seq = result.seq;
      infer_capacity = result.infer_fps;
      detector_hz = result.actual_fps;
      latest_timings = result.timings;
      result_age_ms =
          std::max(0.0, packet.capture_done_ts - result.source_capture_ts) *
          1000.0;
      const bool valid_timestamp =
          result.source_frame_seq != 0 && result.source_capture_ts > 0.0 &&
          result.source_capture_ts <= packet.capture_done_ts + 1e-6;
      if (result.detection.has_value() && valid_timestamp &&
          result_age_ms <= kMaxRknnAgeSeconds * 1000.0) {
        source_detection = result.detection;
        detection_source_seq = result.source_frame_seq;
        detection_source_timestamp = result.source_capture_ts;
        Detection projected = *result.detection;
        projected.box =
            tracker.ProjectBox(projected.box, result.source_capture_ts,
                               packet.capture_done_ts, packet.frame.size());
        projected.center = BoxCenter(projected.box);
        current_detection = projected;
      }
    }

    std::optional<Box> guide_box;
    if (current_detection.has_value()) {
      guide_box = current_detection->box;
    } else {
      guide_box = tracker.PriorBox();
    }

    std::optional<RefinedRect> refined;
    const double refine_start = NowSeconds();
    if (guide_box.has_value()) {
      const TrackSnapshot prior = tracker.Snapshot();
      const std::optional<cv::Point2f> reference_center =
          prior.active ? std::optional<cv::Point2f>(prior.center)
                       : std::nullopt;
      refined = refiner.Refine(packet.frame, *guide_box, reference_center,
                               tracker.Hints());
    }
    refine_ms =
        SmoothValue(refine_ms, (NowSeconds() - refine_start) * 1000.0, 0.10);

    bool accepted = false;
    const bool had_active_track = tracker.Snapshot().active;
    if (refined.has_value()) {
      accepted = tracker.Update(
          *refined, packet.capture_done_ts, packet.frame.size(),
          current_detection.has_value() ? "roi+rknn" : "roi", packet.seq,
          packet.seq, packet.capture_done_ts);
    }
    if (!accepted && current_detection.has_value()) {
      if (!had_active_track && source_detection.has_value()) {
        accepted = tracker.Reacquire(
            *source_detection, detection_source_timestamp,
            packet.capture_done_ts, packet.frame.size(),
            detection_source_seq, packet.seq);
      } else {
        accepted = tracker.Update(
            *current_detection, packet.capture_done_ts, packet.frame.size(),
            "rknn", detection_source_seq, packet.seq,
            detection_source_timestamp);
      }
    }

    const TrackSnapshot track = tracker.Snapshot();
    if (track.active && track.state != "ACQUIRING" && track.has_corners) {
      std::string plane_error;
      target_plane.Update(track.corners, packet.frame.size(), track.state_ts,
                          &plane_error);
    } else {
      target_plane.Invalidate();
    }
    const VisualEstimate estimate = BuildVisualEstimate(track, target_plane);

    const bool need_detection =
        !track.active || track.predicted_only || frames_until_detection <= 0;
    if (need_detection) {
      worker.Submit(packet, tracker.PriorBox());
      frames_until_detection =
          track.active ? config.detect_interval_locked : 0;
    } else {
      --frames_until_detection;
    }

    // Update the red-bullseye memory from this frame's refinement. Prefer the
    // cm-space offset (pose invariant); always keep an image-space EMA too.
    const double now_ts = packet.capture_done_ts;
    if (config.aim_red_center && refined.has_value() &&
        refined->red_center.has_value()) {
      const cv::Point2f rc = *refined->red_center;
      const float a = ClampFloat(config.red_center_alpha, 0.05f, 1.0f);
      if (target_plane.Valid()) {
        const auto cm = target_plane.ImageToTargetCm(rc);
        if (cm.has_value() && std::isfinite(cm->x) && std::isfinite(cm->y)) {
          red_offset_cm = red_offset_cm.has_value()
                              ? *red_offset_cm * (1.0f - a) + *cm * a
                              : *cm;
          red_cm_ts = now_ts;
        }
      }
      red_center_px_ema = red_center_px_ema.has_value()
                              ? *red_center_px_ema * (1.0f - a) + rc * a
                              : rc;
      red_px_ts = now_ts;
    }

    // Aim priority: (1) red bullseye re-projected through the current
    // homography, (2) red bullseye in image space when no homography this
    // frame, (3) perspective-corrected frame center, else AimWriter falls back
    // to the bbox center. The in-frame guard rejects wild extrapolations.
    std::optional<cv::Point2f> aim_center;
    bool aim_is_red = false;
    const double timeout = std::max(0.0, config.red_center_timeout);
    if (config.aim_red_center && red_offset_cm.has_value() &&
        estimate.has_valid_homography && now_ts - red_cm_ts <= timeout) {
      const auto px = target_plane.TargetToImage(*red_offset_cm);
      if (px.has_value() && in_frame(*px, packet.frame)) {
        aim_center = px;
        aim_is_red = true;
      }
    }
    if (!aim_center.has_value() && config.aim_red_center &&
        red_center_px_ema.has_value() && now_ts - red_px_ts <= timeout &&
        in_frame(*red_center_px_ema, packet.frame)) {
      aim_center = red_center_px_ema;
      aim_is_red = true;
    }
    if (!aim_center.has_value() && config.aim_use_projective_center &&
        estimate.has_valid_homography) {
      const cv::Point2f& c = estimate.target_center_px;
      if (in_frame(c, packet.frame)) {
        aim_center = c;
      }
    }
    writer.Send(track, packet.frame.cols, packet.frame.rows, aim_center);
    const double now = NowSeconds();
    frame_fps =
        SmoothValue(frame_fps, 1.0 / std::max(1e-6, now - last_loop), 0.10);
    last_loop = now;
    loop_ms = SmoothValue(loop_ms, (now - loop_start) * 1000.0, 0.10);

    if (config.stats_interval > 0.0 &&
        now - last_stats >= config.stats_interval) {
      const WorkerStats worker_stats = worker.Stats();
      std::ostringstream stats;
      stats << "stats frame_fps=" << cv::format("%.1f", frame_fps)
            << " infer_capacity=" << cv::format("%.1f", infer_capacity)
            << " detector_hz=" << cv::format("%.1f", detector_hz)
            << " refine=" << cv::format("%.2f", refine_ms)
            << "ms loop=" << cv::format("%.2f", loop_ms)
            << "ms rknn=" << cv::format("%.2f", latest_timings.rknn_ms)
            << "ms total=" << cv::format("%.2f", latest_timings.total_ms)
            << "ms result_age=" << cv::format("%.1f", result_age_ms)
            << "ms measurement_age="
            << cv::format("%.1f", estimate.measurement_age_ms)
            << "ms submitted=" << worker_stats.submitted
            << " overwritten=" << worker_stats.overwritten
            << " started=" << worker_stats.started
            << " completed=" << worker_stats.completed
            << " camera_skipped=" << skipped_camera_frames
            << " state=" << track.state << " source=" << track.source
            << " plane=" << (estimate.has_valid_homography ? "valid" : "none")
            << " aim=" << (aim_is_red ? "red"
                           : (aim_center.has_value() ? "frame" : "bbox"))
            << " conf=" << cv::format("%.2f", track.conf);
      std::cout << stats.str() << std::endl;
      last_stats = now;
    }

    const bool stream_active =
        config.mjpeg_port > 0 && mjpeg.client_count() > 0;
    // cv::imshow (Qt5 software-rendered backend on this board) costs several
    // ms/frame for the clone + drawing + paint -- redrawing every camera
    // frame is what turns a ~99fps no-window run into a jittery 60-80fps one.
    // Throttling the window to window_interval keeps tracking/serial at full
    // camera rate while the debug view stays smooth to the eye.
    const bool window_due =
        config.show_window &&
        (now - last_window_draw >= config.window_interval);
    if (window_due || stream_active) {
      cv::Mat overlay = DrawDebug(packet.frame, track, estimate, frame_fps,
                                  infer_capacity, detector_hz, aim_center,
                                  aim_is_red);
      if (stream_active) {
        mjpeg.PushFrame(overlay);
      }
      if (window_due) {
        cv::imshow("rkfast_target", overlay);
        last_window_draw = now;
      }
    }
    if (config.show_window) {
      const int key = cv::waitKey(1);
      if (key == 27 || key == 'q' || key == 'Q') {
        break;
      }
    }
  }

  mjpeg.Stop();
  writer.Close();
  worker.Stop();
  camera.Close();
  cv::destroyAllWindows();
  return 0;
}
