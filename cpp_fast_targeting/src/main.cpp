#include <chrono>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "rkfast/aim_writer.hpp"
#include "rkfast/camera_source.hpp"
#include "rkfast/config.hpp"
#include "rkfast/rect_refiner.hpp"
#include "rkfast/rknn_detector.hpp"
#include "rkfast/thread_affinity.hpp"
#include "rkfast/tracker.hpp"
#include "rkfast/types.hpp"

namespace rkfast {

namespace {

cv::Mat DrawDebug(const cv::Mat& frame, const TrackSnapshot& snapshot,
                  double frame_fps, double infer_fps) {
  cv::Mat overlay = frame.clone();
  const int cx0 = overlay.cols / 2;
  const int cy0 = overlay.rows / 2;
  cv::line(overlay, cv::Point(cx0 - 18, cy0), cv::Point(cx0 + 18, cy0),
           cv::Scalar(80, 80, 80), 1);
  cv::line(overlay, cv::Point(cx0, cy0 - 18), cv::Point(cx0, cy0 + 18),
           cv::Scalar(80, 80, 80), 1);

  if (snapshot.active) {
    const cv::Rect rect(static_cast<int>(std::round(snapshot.box.x1)),
                        static_cast<int>(std::round(snapshot.box.y1)),
                        static_cast<int>(std::round(BoxWidth(snapshot.box))),
                        static_cast<int>(std::round(BoxHeight(snapshot.box))));
    cv::rectangle(overlay, rect, cv::Scalar(0, 180, 255), 2);
    if (snapshot.has_corners) {
      for (int i = 0; i < 4; ++i) {
        cv::line(overlay, snapshot.corners[i], snapshot.corners[(i + 1) % 4],
                 cv::Scalar(0, 255, 0), 2);
      }
    }
    cv::drawMarker(overlay,
                   cv::Point(static_cast<int>(std::round(snapshot.center.x)),
                             static_cast<int>(std::round(snapshot.center.y))),
                   cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 14, 2);
    // Predicted (lead-compensated) aim point.
    cv::drawMarker(overlay,
                   cv::Point(static_cast<int>(std::round(snapshot.predict_center.x)),
                             static_cast<int>(std::round(snapshot.predict_center.y))),
                   cv::Scalar(255, 0, 255), cv::MARKER_TILTED_CROSS, 16, 2);
    std::ostringstream status;
    status << snapshot.state << " conf=" << cv::format("%.2f", snapshot.conf);
    cv::putText(overlay, status.str(), cv::Point(12, 28),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 180, 255), 2);
  } else {
    cv::putText(overlay, "SEARCHING", cv::Point(12, 28),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
  }
  std::ostringstream fps_text;
  fps_text << "frame=" << cv::format("%.1f", frame_fps)
           << " infer=" << cv::format("%.1f", infer_fps);
  cv::putText(overlay, fps_text.str(), cv::Point(12, 56),
              cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 255, 255), 2);
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
    std::cerr << "main affinity warning: " << affinity_error << "\n";
  }

  std::string error;
  CameraSource camera(config);
  if (!camera.Open(&error)) {
    std::cerr << error << "\n";
    return 1;
  }

  auto detector = std::make_unique<RknnDetector>(config);
  if (!detector->Open(&error)) {
    std::cerr << error << "\n";
    return 1;
  }
  if (!detector->Warmup(config.warmup_runs, &error)) {
    std::cerr << error << "\n";
    return 1;
  }

  DetectorWorker worker(std::move(detector), config.detector_affinity);
  worker.Start();

  RectRefiner refiner(config);
  RectTracker tracker(config);
  AimWriter writer(config);
  if (!writer.Open(&error)) {
    std::cerr << error << "\n";
  }

  double frame_fps = 0.0;
  double infer_fps = 0.0;
  double last_loop = NowSeconds();
  double last_stats = last_loop;
  uint64_t last_frame_seq = 0;
  uint64_t last_result_seq = 0;
  int detect_skip = 0;
  InferTimings latest_timings;

  std::cout << "running; press q in the debug window to exit\n";
  while (true) {
    cv::Mat frame;
    double frame_ts = 0.0;
    uint64_t frame_seq = 0;
    if (!camera.ReadLatest(&frame, &frame_ts, &frame_seq) || frame.empty()) {
      // Removed sleep for maximum FPS
      continue;
    }
    if (frame_seq == last_frame_seq) {
      // Removed sleep for maximum FPS
      continue;
    }
    last_frame_seq = frame_seq;

    const auto result = worker.LatestResult();
    bool new_result = result.seq != 0 && result.seq != last_result_seq;
    std::optional<Detection> new_detection;
    if (new_result) {
      last_result_seq = result.seq;
      new_detection = result.detection;
      infer_fps = result.infer_fps;
      latest_timings = result.timings;
      detect_skip = tracker.Snapshot().active ? config.detect_interval_locked : 0;
    }

    std::optional<Box> guide_box = new_detection.has_value()
                                       ? std::optional<Box>(new_detection->box)
                                       : tracker.PriorBox();
    std::optional<RefinedRect> refined;
    if (guide_box.has_value()) {
      auto snapshot = tracker.Snapshot();
      const std::optional<cv::Point2f> ref_center =
          snapshot.active ? std::optional<cv::Point2f>(snapshot.center)
                          : std::nullopt;
      refined = refiner.Refine(frame, *guide_box, ref_center, tracker.Hints());
    }

    bool updated = false;
    if (refined.has_value()) {
      tracker.Update(*refined, frame_ts, frame.size(),
                     new_detection.has_value() ? "roi+rknn" : "roi");
      updated = true;
    } else if (new_detection.has_value()) {
      tracker.Update(*new_detection, result.timestamp, frame.size(), "rknn");
      updated = true;
    } else if (new_result) {
      tracker.MarkDetectMiss(result.timestamp, frame.size());
      updated = true;
    }

    if (!updated) {
      tracker.Predict(frame_ts, frame.size());
    }

    const TrackSnapshot snapshot = tracker.Snapshot();
    if (!worker.Busy()) {
      const bool need_detection =
          !snapshot.active || snapshot.misses > 0 || detect_skip <= 0;
      if (need_detection) {
        worker.Submit(frame, tracker.PriorBox());
      } else {
        --detect_skip;
      }
    }

    writer.Send(snapshot, frame.cols, frame.rows);

    const double now = NowSeconds();
    frame_fps = SmoothValue(frame_fps, 1.0 / std::max(1e-6, now - last_loop), 0.10);
    last_loop = now;

    if (config.stats_interval > 0.0 && now - last_stats >= config.stats_interval) {
      std::ostringstream stats;
      stats << "stats frame_fps=" << cv::format("%.1f", frame_fps)
            << " infer_fps=" << cv::format("%.1f", infer_fps)
            << " pre=" << cv::format("%.1f", latest_timings.preprocess_ms)
            << "ms set=" << cv::format("%.1f", latest_timings.inputs_set_ms)
            << "ms rknn=" << cv::format("%.1f", latest_timings.rknn_ms)
            << "ms get=" << cv::format("%.1f", latest_timings.outputs_get_ms)
            << "ms post=" << cv::format("%.1f", latest_timings.postprocess_ms)
            << "ms select=" << cv::format("%.1f", latest_timings.select_ms)
            << "ms release=" << cv::format("%.1f", latest_timings.outputs_release_ms)
            << "ms total=" << cv::format("%.1f", latest_timings.total_ms)
            << "ms state=" << snapshot.state
            << " conf=" << cv::format("%.2f", snapshot.conf);
      std::cout << stats.str() << "\n";
      last_stats = now;
    }

    if (config.show_window) {
      cv::imshow("rkfast_target", DrawDebug(frame, snapshot, frame_fps, infer_fps));
      const int key = cv::waitKey(1);
      if (key == 27 || key == 'q' || key == 'Q') {
        break;
      }
    }
  }

  writer.Close();
  worker.Stop();
  camera.Close();
  cv::destroyAllWindows();
  return 0;
}
