// Host-side unit tests for rkfast logic that does not require the NPU.
// Build: see cpp_fast_targeting/tests/CMakeLists.txt (target rkfast_tests).
#include <array>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "rkfast/config.hpp"
#include "rkfast/rect_refiner.hpp"
#include "rkfast/target_plane.hpp"
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

void TestGateDecouplesCenterFromSize() {
  std::printf("GateDecouplesCenterFromSize\n");
  // Regression for motion-time lock loss: motion blur inflates the measured
  // box while the center stays correct. The old joint 4-DoF gate let the huge
  // size innovation reject the whole measurement; the decoupled gate must keep
  // the center lock and simply hold the size.
  rkfast::AppConfig config;
  rkfast::RectTracker tracker(config);
  const cv::Size size(640, 480);

  auto make_det = [](float cx, float cy, float half) {
    rkfast::Detection d;
    d.box = rkfast::Box{cx - half, cy - half, cx + half, cy + half};
    d.center = cv::Point2f(cx, cy);
    d.conf = 0.9f;
    d.candidate_count = 1;
    return d;
  };

  double t = 0.0;
  // Establish a stable ~20px box moving +100 px/s.
  tracker.Update(make_det(200, 240, 10), t, size, "rknn");
  for (int i = 0; i < 3; ++i) {
    t += 0.02;
    tracker.Update(make_det(200 + (i + 1) * 2, 240, 10), t, size, "roi");
  }
  Check(tracker.Snapshot().active, "track active before blur frame");
  const float w_before = rkfast::BoxWidth(tracker.Snapshot().box);

  // Motion-blur frame: center still correct, box smeared ~5x wider.
  t += 0.02;
  const bool accepted =
      tracker.Update(make_det(208, 240, 50), t, size, "roi");
  const auto snap = tracker.Snapshot();
  Check(accepted, "size-inflated measurement is still accepted (center gate)");
  Check(snap.active, "track stays active through the blur frame");
  Check(std::abs(snap.center.x - 208.0f) < 8.0f,
        "center still tracks despite bad size");
  const float w_after = rkfast::BoxWidth(snap.box);
  Check(w_after < w_before * 2.5f,
        "box size is held, not snapped to the smeared 5x width");
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

void TestRefinerProjectiveCenter() {
  std::printf("RefinerProjectiveCenter\n");
  // A foreshortened (trapezoidal) view is the case where the integer AABB
  // center and the true projective center (diagonal intersection) diverge.
  // Narrow top, wide bottom -> AABB center sits lower (toward the wide edge)
  // than the diagonal intersection. This regression proves the sub-pixel
  // corners now drive the tracked center instead of the boundingRect center.
  rkfast::AppConfig config;
  config.roi_min_size = 16;
  config.refine_min_area_ratio = 0.02f;
  config.refine_min_score = 0.0f;
  rkfast::RectRefiner refiner(config);

  cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
  // TL,TR,BR,BL of a symmetric trapezoid.
  std::vector<cv::Point> quad{
      {260, 150}, {380, 150}, {460, 400}, {180, 400}};
  cv::fillConvexPoly(frame, quad, cv::Scalar(0, 0, 0));

  // AABB is x[180,460] y[150,400] -> AABB center (320, 275).
  // Diagonals intersect at (320, 225): 50px above the AABB center.
  const float aabb_cy = 275.0f;
  const float diag_cy = 225.0f;

  rkfast::Box guide{170.0f, 140.0f, 470.0f, 410.0f};
  auto refined = refiner.Refine(frame, guide);
  Check(refined.has_value(), "refiner found the trapezoid");
  if (!refined.has_value()) {
    return;
  }
  const float cx = refined->center.x;
  const float cy = refined->center.y;
  Check(std::abs(cx - 320.0f) < 15.0f, "projective center x near 320");
  Check(std::abs(cy - diag_cy) < std::abs(cy - aabb_cy),
        "center is the diagonal intersection, not the AABB center");
  Check(cy < 260.0f, "center pulled toward true projective center (not AABB)");
}

void TestRefinerSmallRoiNoCrash() {
  std::printf("RefinerSmallRoiNoCrash\n");
  // Regression: a small/distant target yields a ROI < 15px, and the old
  // fixed cornerSubPix win(5,5) asserts src.cols >= win.width*2+5 == 15,
  // aborting the whole process with cv::Exception. Force a 14x14 ROI and
  // confirm Refine() returns normally instead of throwing.
  rkfast::AppConfig config;
  config.roi_min_size = 10;         // allow ROIs down to 10px
  config.roi_expand_ratio = 0.0f;   // no padding -> ROI == guide box size
  config.motion_roi_gain = 1.0f;
  config.refine_min_area_ratio = 0.02f;
  config.refine_min_score = 0.0f;   // don't reject on score, we test crashing
  rkfast::RectRefiner refiner(config);

  cv::Mat frame(120, 120, CV_8UC3, cv::Scalar(255, 255, 255));
  const cv::Rect target(50, 50, 10, 10);  // small black square with margin
  cv::rectangle(frame, target, cv::Scalar(0, 0, 0), cv::FILLED);

  // Guide box 14px wide/tall -> clipped ROI is 14x14 (< old 15px minimum).
  rkfast::Box guide{48.0f, 48.0f, 62.0f, 62.0f};

  bool threw = false;
  std::optional<rkfast::RefinedRect> refined;
  try {
    refined = refiner.Refine(frame, guide);
  } catch (const cv::Exception&) {
    threw = true;
  }
  Check(!threw, "Refine does not throw on a sub-15px ROI");
  Check(refined.has_value(), "Refine still finds the rectangle in a 14px ROI");
}

void TestTargetPlaneMapping() {
  std::printf("TargetPlaneMapping");
  rkfast::TargetPlane plane(21.0f, 29.7f);
  const std::array<cv::Point2f, 4> corners{
      cv::Point2f(100.0f, 60.0f), cv::Point2f(310.0f, 60.0f),
      cv::Point2f(310.0f, 357.0f), cv::Point2f(100.0f, 357.0f)};
  std::string error;
  Check(plane.Update(corners, cv::Size(640, 480), 1.0, &error),
        "valid rectangle creates homography");
  const auto target_center =
      plane.ImageToTargetCm(cv::Point2f(205.0f, 208.5f));
  Check(target_center.has_value(), "image center maps to target plane");
  if (target_center.has_value()) {
    Check(Near(target_center->x, 0.0f, 0.05f) &&
              Near(target_center->y, 0.0f, 0.05f),
          "target center maps to (0,0) cm");
  }
  const auto image_center = plane.TargetToImage(cv::Point2f(0.0f, 0.0f));
  Check(image_center.has_value(), "target center maps back to image");
  if (image_center.has_value()) {
    Check(Near(image_center->x, 205.0f, 0.1f) &&
              Near(image_center->y, 208.5f, 0.1f),
          "homography round trip preserves center");
  }
}

}  // namespace

int main() {
  TestCoordinateMapping();
  TestTrackerPrediction();
  TestGateDecouplesCenterFromSize();
  TestRefinerSelectsRectangle();
  TestRefinerCornersAreFrameCoordinates();
  TestRefinerProjectiveCenter();
  TestRefinerSmallRoiNoCrash();
  TestTargetPlaneMapping();
  if (g_failures == 0) {
    std::printf("\nALL TESTS PASSED\n");
    return 0;
  }
  std::printf("\n%d TEST(S) FAILED\n", g_failures);
  return 1;
}
