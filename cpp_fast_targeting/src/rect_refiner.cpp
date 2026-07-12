#include "rkfast/rect_refiner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "rkfast/types.hpp"

namespace rkfast {

RectRefiner::RectRefiner(const AppConfig& config) : config_(config) {}

std::optional<RefinedRect> RectRefiner::Refine(
    const cv::Mat& frame_bgr, const Box& guide_box,
    const std::optional<cv::Point2f>& reference_center,
    const RefineHints& hints) const {
  if (frame_bgr.empty()) {
    return std::nullopt;
  }

  // Motion-adaptive ROI: faster targets need a larger search margin.
  float motion = 0.0f;
  if (config_.motion_speed_ref > 1.0f) {
    motion = ClampFloat(hints.speed / config_.motion_speed_ref, 0.0f, 1.0f);
  }
  const float expand =
      config_.roi_expand_ratio * (1.0f + motion * (config_.motion_roi_gain - 1.0f));

  const float guide_w = BoxWidth(guide_box);
  const float guide_h = BoxHeight(guide_box);
  const float pad_x = guide_w * expand;
  const float pad_y = guide_h * expand;
  const Box roi_box = ClipBox(
      Box{guide_box.x1 - pad_x, guide_box.y1 - pad_y, guide_box.x2 + pad_x,
          guide_box.y2 + pad_y},
      frame_bgr.cols, frame_bgr.rows);

  const int x = static_cast<int>(std::floor(roi_box.x1));
  const int y = static_cast<int>(std::floor(roi_box.y1));
  const int w =
      static_cast<int>(std::ceil(roi_box.x2) - std::floor(roi_box.x1));
  const int h =
      static_cast<int>(std::ceil(roi_box.y2) - std::floor(roi_box.y1));
  if (w < config_.roi_min_size || h < config_.roi_min_size) {
    return std::nullopt;
  }

  const cv::Rect roi(x, y, std::min(w, frame_bgr.cols - x),
                     std::min(h, frame_bgr.rows - y));
  if (roi.width < config_.roi_min_size || roi.height < config_.roi_min_size) {
    return std::nullopt;
  }

  const cv::Point2f target_center =
      reference_center.has_value() ? *reference_center : BoxCenter(guide_box);
  const cv::Mat roi_bgr = frame_bgr(roi);

  cv::Mat gray;
  cv::cvtColor(roi_bgr, gray, cv::COLOR_BGR2GRAY);
  // Lighter blur under fast motion to avoid over-eroding a smeared edge.
  const int blur_k = motion > 0.5f ? 3 : 5;
  cv::GaussianBlur(gray, gray, cv::Size(blur_k, blur_k), 0.0);

  cv::Mat binary;
  cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
  const int close_k = motion > 0.5f ? 3 : 5;
  cv::morphologyEx(binary, binary, cv::MORPH_OPEN,
                   cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
  cv::morphologyEx(
      binary, binary, cv::MORPH_CLOSE,
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(close_k, close_k)));

  auto result = SearchContours(binary, gray, roi, target_center, hints);
  if (!result.has_value()) {
    cv::Mat edge;
    cv::Canny(gray, edge, 60.0, 160.0);
    cv::dilate(edge, edge,
               cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    result = SearchContours(edge, gray, roi, target_center, hints);
  }
  if (result.has_value() && result->score < config_.refine_min_score) {
    return std::nullopt;
  }
  return result;
}

std::optional<RefinedRect> RectRefiner::SearchContours(
    const cv::Mat& binary, const cv::Mat& gray, const cv::Rect& roi,
    const cv::Point2f& target_center, const RefineHints& hints) const {
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
  const float min_area =
      static_cast<float>(roi.area()) * config_.refine_min_area_ratio;

  std::optional<RefinedRect> best;
  float best_score = -1.0f;

  for (const auto& contour : contours) {
    const double contour_area = cv::contourArea(contour);
    if (contour_area < min_area) {
      continue;
    }
    const double perimeter = cv::arcLength(contour, true);
    if (perimeter < 20.0) {
      continue;
    }

    std::vector<cv::Point> approx;
    cv::approxPolyDP(contour, approx, 0.02 * perimeter, true);
    if (approx.size() != 4 || !cv::isContourConvex(approx)) {
      continue;
    }

    const cv::Rect rect = cv::boundingRect(approx);
    if (rect.width < config_.roi_min_size / 2 ||
        rect.height < config_.roi_min_size / 2) {
      continue;
    }

    // approx is in ROI-local pixel coordinates (contours were found on the
    // cropped ROI); offset back to full-frame coordinates before ordering.
    std::vector<cv::Point> approx_frame = approx;
    for (auto& pt : approx_frame) {
      pt.x += roi.x;
      pt.y += roi.y;
    }
    auto corners = OrderCorners(approx_frame);
    RefineCornersSubPix(gray, roi, &corners);

    // Right-angle constraint: reject quads whose corners deviate far from 90.
    const float right_angle_score = RightAngleScore(corners);
    if (right_angle_score < 0.3f) {
      continue;
    }

    const float extent =
        static_cast<float>(contour_area) /
        std::max(1.0f, static_cast<float>(rect.area()));
    const cv::Point2f center(roi.x + rect.x + rect.width * 0.5f,
                             roi.y + rect.y + rect.height * 0.5f);
    const float dist = static_cast<float>(cv::norm(center - target_center));
    const float dist_norm = dist / std::max(1.0f, static_cast<float>(roi.width + roi.height));
    const float center_score = std::max(0.0f, 1.0f - dist_norm);
    const float area_score =
        std::min(1.0f, static_cast<float>(contour_area) /
                           std::max(1.0f, static_cast<float>(roi.area()) * 0.35f));

    // Aspect score: closeness to prior aspect when known, else self-squareness.
    const float aspect = std::min(rect.width, rect.height) /
                         std::max(1.0f, static_cast<float>(std::max(rect.width, rect.height)));
    float aspect_score = aspect;
    if (hints.prior_aspect > 0.0f) {
      aspect_score = std::max(0.0f, 1.0f - std::abs(aspect - hints.prior_aspect));
    }

    // Prior-box IOU: prefer candidates near last tracked box.
    float iou_score = 0.0f;
    if (hints.prior_box.has_value()) {
      const Box cand{static_cast<float>(roi.x + rect.x),
                     static_cast<float>(roi.y + rect.y),
                     static_cast<float>(roi.x + rect.x + rect.width),
                     static_cast<float>(roi.y + rect.y + rect.height)};
      iou_score = BoxIou(cand, *hints.prior_box);
    }

    const float iw = config_.prior_iou_weight;
    const float score = (1.0f - iw) * (area_score * 0.40f + extent * 0.25f +
                                       center_score * 0.13f + aspect_score * 0.12f +
                                       right_angle_score * 0.10f) +
                        iw * iou_score;
    if (score <= best_score) {
      continue;
    }

    RefinedRect candidate;
    candidate.box = Box{static_cast<float>(roi.x + rect.x),
                        static_cast<float>(roi.y + rect.y),
                        static_cast<float>(roi.x + rect.x + rect.width),
                        static_cast<float>(roi.y + rect.y + rect.height)};
    candidate.center = center;
    candidate.corners = corners;
    candidate.score = score;
    best = candidate;
    best_score = score;
  }

  return best;
}

float RectRefiner::RightAngleScore(const std::array<cv::Point2f, 4>& corners) {
  float worst = 1.0f;  // 1.0 = perfect right angle, 0.0 = degenerate
  for (int i = 0; i < 4; ++i) {
    const cv::Point2f& prev = corners[(i + 3) % 4];
    const cv::Point2f& cur = corners[i];
    const cv::Point2f& next = corners[(i + 1) % 4];
    const cv::Point2f a = prev - cur;
    const cv::Point2f b = next - cur;
    const float na = static_cast<float>(cv::norm(a));
    const float nb = static_cast<float>(cv::norm(b));
    if (na < 1e-3f || nb < 1e-3f) {
      return 0.0f;
    }
    const float cos_angle = (a.x * b.x + a.y * b.y) / (na * nb);
    // 1 - |cos| peaks at 90 degrees (cos = 0), drops toward 0 or 180.
    worst = std::min(worst, 1.0f - std::abs(cos_angle));
  }
  return worst;
}

void RectRefiner::RefineCornersSubPix(const cv::Mat& gray, const cv::Rect& roi,
                                      std::array<cv::Point2f, 4>* corners) {
  // cornerSubPix needs ROI-local coordinates since `gray` is the cropped ROI.
  std::vector<cv::Point2f> local(corners->begin(), corners->end());
  for (auto& pt : local) {
    pt.x -= static_cast<float>(roi.x);
    pt.y -= static_cast<float>(roi.y);
  }
  const cv::Size win(5, 5);
  const cv::Size zero_zone(-1, -1);
  const cv::TermCriteria criteria(
      cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.01);
  cv::cornerSubPix(gray, local, win, zero_zone, criteria);
  for (size_t i = 0; i < 4; ++i) {
    (*corners)[i] = cv::Point2f(local[i].x + static_cast<float>(roi.x),
                                local[i].y + static_cast<float>(roi.y));
  }
}

std::array<cv::Point2f, 4> RectRefiner::OrderCorners(
    const std::vector<cv::Point>& polygon) {
  std::array<cv::Point2f, 4> corners{};
  std::array<cv::Point2f, 4> pts{};
  for (size_t i = 0; i < 4; ++i) {
    pts[i] = cv::Point2f(static_cast<float>(polygon[i].x),
                         static_cast<float>(polygon[i].y));
  }

  auto sum_cmp = [](const cv::Point2f& a, const cv::Point2f& b) {
    return (a.x + a.y) < (b.x + b.y);
  };
  auto diff_cmp = [](const cv::Point2f& a, const cv::Point2f& b) {
    return (a.y - a.x) < (b.y - b.x);
  };

  corners[0] = *std::min_element(pts.begin(), pts.end(), sum_cmp);
  corners[2] = *std::max_element(pts.begin(), pts.end(), sum_cmp);
  corners[1] = *std::min_element(pts.begin(), pts.end(), diff_cmp);
  corners[3] = *std::max_element(pts.begin(), pts.end(), diff_cmp);
  return corners;
}

}  // namespace rkfast
