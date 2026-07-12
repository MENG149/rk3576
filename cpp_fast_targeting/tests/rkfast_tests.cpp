// Host-side unit tests for rkfast logic that does not require the NPU.
// Build: see cpp_fast_targeting/tests/CMakeLists.txt (target rkfast_tests).
#include <array>
#include <cmath>
#include <cstdio>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "rkfast/config.hpp"
#include "rkfast/rect_refiner.hpp"
#include "rkfast/tracker.hpp"
#include "rkfast/types.hpp"

namespace {

int g_failures = 0;

void Check(bool cond, const std::string& name) {
  if (cond) {
    std::printf("  [ok] %s\n", name.c_str());
  } else {
    std::printf("  [FAIL] %s\n", name.c_str());
    ++g_failures;
  }
}

bool Near(float a, float b, float eps = 1e-3f) { return std::abs(a - b) <= eps; }

void TestCoordinateMapping() {
  std::printf("CoordinateMapping\n");
  const int w = 640, h = 480;
  const cv::Point2f center(w * 0.5f, h * 0.5f);
  const cv::Point2f off = rkfast::CenterOffset(center, w, h);
  Check(Near(off.x, 0.0f) && Near(off.y, 0.0f), "center maps to (0,0)");

  const cv::Point2f n_tl = rkfast::NormalizedOffset(cv::Point2f(0, 0), w, h);
  Check(Near(n_tl.x, -1.0f) && Near(n_tl.y, -1.0f), "top-left maps to (-1,-1)");

  const cv::Point2f n_br = rkfast::NormalizedOffset(cv::Point2f(w, h), w, h);
  Check(Near(n_br.x, 1.0f) && Near(n_br.y, 1.0f), "bottom-right maps to (1,1)");

  // y grows downward: a point below center has positive ny.
  const cv::Point2f below(w * 0.5f, h * 0.75f);
  const cv::Point2f n_below = rkfast::NormalizedOffset(below, w, h);
  Check(n_below.y > 0.0f, "below-center has positive ny (y down)");
}

void TestTrackerPrediction() {
  std::printf("TrackerPrediction\n");
  rkfast::AppConfig config;
  config.lead_time = 0.1f;          // 100 ms look-ahead
  config.track_alpha = 1.0f;        // trust measurement fully for a clean test
  config.track_beta = 1.0f;
  config.max_predict_misses = 3;
  config.predict_decay = 0.5f;
  rkfast::RectTracker tracker(config);

  const cv::Size size(640, 480);
  // Constant-velocity target moving +100 px/s in x.
  auto make_det = [](float cx, float cy) {
    rkfast::Detection d;
    d.box = rkfast::Box{cx - 10, cy - 10, cx + 10, cy + 10};
    d.center = cv::Point2f(cx, cy);
    d.conf = 0.9f;
    d.candidate_count = 1;
    return d;
  };

  double t = 0.0;
  tracker.Update(make_det(100, 240), t, size, "rknn");
  t += 0.1;
  tracker.Update(make_det(110, 240), t, size, "rknn");
  t += 0.1;
  tracker.Update(make_det(120, 240), t, size, "rknn");

  auto snap = tracker.Snapshot();
  Check(snap.active, "tracker active after measurements");
  Check(snap.vx > 50.0f, "estimated vx is positive and sizeable");
  // predict_center should lead current center by ~vx*lead_time.
  Check(snap.predict_center.x > snap.center.x, "predict leads current x");
  Check(Near(snap.predict_center.x, snap.center.x + snap.vx * config.lead_time, 1.0f),
        "predict_center matches center + v*lead");

  // Miss handling: velocity should decay across extrapolation steps.
  const float vx_before = snap.vx;
  t += 0.1;
  tracker.MarkDetectMiss(t, size);
  auto snap_miss = tracker.Snapshot();
  Check(std::abs(snap_miss.vx) < std::abs(vx_before) + 1e-3f,
        "velocity does not grow during extrapolation");
}

void TestRefinerSelectsRectangle() {
  std::printf("RefinerSelectsRectangle\n");
  rkfast::AppConfig config;
  config.roi_min_size = 16;
  config.refine_min_area_ratio = 0.02f;
  rkfast::RectRefiner refiner(config);

  // White canvas with a black filled rectangle in the middle.
  cv::Mat frame(240, 320, CV_8UC3, cv::Scalar(255, 255, 255));
  const cv::Rect target(120, 90, 80, 60);
  cv::rectangle(frame, target, cv::Scalar(0, 0, 0), cv::FILLED);

  rkfast::Box guide{static_cast<float>(target.x - 8),
                    static_cast<float>(target.y - 8),
                    static_cast<float>(target.x + target.width + 8),
                    static_cast<float>(target.y + target.height + 8)};

  auto refined = refiner.Refine(frame, guide);
  Check(refined.has_value(), "refiner found a rectangle");
  if (refined.has_value()) {
    const float cx = refined->center.x;
    const float cy = refined->center.y;
    Check(std::abs(cx - (target.x + target.width * 0.5f)) < 6.0f,
          "refined center x near true center");
    Check(std::abs(cy - (target.y + target.height * 0.5f)) < 6.0f,
          "refined center y near true center");
  }
}

void TestRefinerCornersAreFrameCoordinates() {
  std::printf("RefinerCornersAreFrameCoordinates\n");
  rkfast::AppConfig config;
  config.roi_min_size = 16;
  config.refine_min_area_ratio = 0.02f;
  rkfast::RectRefiner refiner(config);

  // Put the rectangle far from the origin so a corner-offset bug (ROI-local
  // coordinates leaking into frame-space output) is unmistakable.
  cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
  const cv::Rect target(420, 300, 100, 70);
  cv::rectangle(frame, target, cv::Scalar(0, 0, 0), cv::FILLED);

  rkfast::Box guide{static_cast<float>(target.x - 8),
                    static_cast<float>(target.y - 8),
                    static_cast<float>(target.x + target.width + 8),
                    static_cast<float>(target.y + target.height + 8)};

  auto refined = refiner.Refine(frame, guide);
  Check(refined.has_value(), "refiner found a rectangle far from origin");
  if (!refined.has_value()) {
    return;
  }
  for (int i = 0; i < 4; ++i) {
    const cv::Point2f& c = refined->corners[i];
    const bool inside_frame_region = c.x > 300.0f && c.y > 200.0f;
    Check(inside_frame_region,
          "corner " + std::to_string(i) + " is in frame coords, not ROI-local");
  }
  Check(std::abs(refined->center.x - (target.x + target.width * 0.5f)) < 6.0f,
        "refined center x near true center (far-from-origin case)");
  Check(std::abs(refined->center.y - (target.y + target.height * 0.5f)) < 6.0f,
        "refined center y near true center (far-from-origin case)");
}

}  // namespace

int main() {
  TestCoordinateMapping();
  TestTrackerPrediction();
  TestRefinerSelectsRectangle();
  TestRefinerCornersAreFrameCoordinates();
  if (g_failures == 0) {
    std::printf("\nALL TESTS PASSED\n");
    return 0;
  }
  std::printf("\n%d TEST(S) FAILED\n", g_failures);
  return 1;
}
