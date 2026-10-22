#include "rkfast/target_plane.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace rkfast {
namespace {

bool FinitePoint(const cv::Point2f& point) {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

float Cross(const cv::Point2f& a, const cv::Point2f& b,
            const cv::Point2f& c) {
  return (b.x - a.x) * (c.y - b.y) -
         (b.y - a.y) * (c.x - b.x);
}

float SignedArea2(const std::array<cv::Point2f, 4>& points) {
  float area2 = 0.0f;
  for (size_t i = 0; i < points.size(); ++i) {
    const auto& a = points[i];
    const auto& b = points[(i + 1) % points.size()];
    area2 += a.x * b.y - b.x * a.y;
  }
  return area2;
}

std::array<cv::Point2f, 4> OrderCorners(
    const std::array<cv::Point2f, 4>& input) {
  cv::Point2f center(0.0f, 0.0f);
  for (const auto& point : input) {
    center += point;
  }
  center *= 0.25f;

  std::array<cv::Point2f, 4> ordered = input;
  std::sort(ordered.begin(), ordered.end(),
            [&center](const cv::Point2f& a, const cv::Point2f& b) {
              return std::atan2(a.y - center.y, a.x - center.x) <
                     std::atan2(b.y - center.y, b.x - center.x);
            });
  const auto start = std::min_element(
      ordered.begin(), ordered.end(),
      [](const cv::Point2f& a, const cv::Point2f& b) {
        return a.x + a.y < b.x + b.y;
      });
  std::rotate(ordered.begin(), start, ordered.end());
  if (SignedArea2(ordered) < 0.0f) {
    std::swap(ordered[1], ordered[3]);
  }
  return ordered;
}

bool FiniteMatrix(const cv::Matx33f& matrix) {
  for (float value : matrix.val) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

cv::Matx33f ToMatx33f(const cv::Mat& matrix) {
  cv::Mat as_float;
  matrix.convertTo(as_float, CV_32F);
  cv::Matx33f result;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      result(row, col) = as_float.at<float>(row, col);
    }
  }
  return result;
}

void SetError(std::string* error, const std::string& detail) {
  if (error != nullptr) {
    *error = "target plane update failed: " + detail;
  }
}

std::optional<cv::Point2f> TransformPoint(const cv::Point2f& point,
                                          const cv::Matx33f& matrix) {
  if (!FinitePoint(point) || !FiniteMatrix(matrix)) {
    return std::nullopt;
  }
  const float denominator =
      matrix(2, 0) * point.x + matrix(2, 1) * point.y + matrix(2, 2);
  if (!std::isfinite(denominator) || std::abs(denominator) < 1e-7f) {
    return std::nullopt;
  }
  cv::Point2f output(
      (matrix(0, 0) * point.x + matrix(0, 1) * point.y + matrix(0, 2)) /
          denominator,
      (matrix(1, 0) * point.x + matrix(1, 1) * point.y + matrix(1, 2)) /
          denominator);
  return FinitePoint(output) ? std::optional<cv::Point2f>(output)
                             : std::nullopt;
}

}  // namespace

TargetPlane::TargetPlane(float width_cm, float height_cm)
    : width_cm_(width_cm), height_cm_(height_cm) {}

bool TargetPlane::Update(const std::array<cv::Point2f, 4>& input,
                         const cv::Size& image_size, double timestamp,
                         std::string* error) {
  Invalidate();
  if (!(width_cm_ > 0.0f) || !(height_cm_ > 0.0f) ||
      image_size.width <= 1 || image_size.height <= 1 ||
      !std::isfinite(timestamp)) {
    SetError(error, "invalid target, image, or timestamp");
    return false;
  }
  for (const auto& point : input) {
    if (!FinitePoint(point)) {
      SetError(error, "corner is not finite");
      return false;
    }
  }

  const auto ordered = OrderCorners(input);
  float cross_sign = 0.0f;
  float min_edge = std::numeric_limits<float>::max();
  for (size_t i = 0; i < ordered.size(); ++i) {
    const float cross =
        Cross(ordered[i], ordered[(i + 1) % 4], ordered[(i + 2) % 4]);
    if (std::abs(cross) < 1e-3f ||
        (cross_sign != 0.0f && cross * cross_sign <= 0.0f)) {
      SetError(error, "corners are not a strict convex quadrilateral");
      return false;
    }
    cross_sign = cross;
    min_edge =
        std::min(min_edge,
                 static_cast<float>(cv::norm(ordered[(i + 1) % 4] -
                                             ordered[i])));
  }

  const float area = std::abs(SignedArea2(ordered)) * 0.5f;
  const float image_area =
      static_cast<float>(image_size.width) * image_size.height;
  if (area < std::max(16.0f, image_area * 1e-4f)) {
    SetError(error, "quadrilateral area is too small");
    return false;
  }
  const float image_diagonal =
      std::hypot(static_cast<float>(image_size.width),
                 static_cast<float>(image_size.height));
  if (min_edge < std::max(2.0f, image_diagonal * 0.002f)) {
    SetError(error, "quadrilateral edge is too short");
    return false;
  }

  const float observed_width =
      0.5f * (static_cast<float>(cv::norm(ordered[1] - ordered[0])) +
              static_cast<float>(cv::norm(ordered[2] - ordered[3])));
  const float observed_height =
      0.5f * (static_cast<float>(cv::norm(ordered[3] - ordered[0])) +
              static_cast<float>(cv::norm(ordered[2] - ordered[1])));
  const float observed_aspect =
      observed_width / std::max(1e-3f, observed_height);
  const float portrait_aspect = width_cm_ / height_cm_;
  const float landscape_aspect = height_cm_ / width_cm_;
  const bool use_landscape =
      orientation_locked_
          ? landscape_
          :
        std::abs(std::log(observed_aspect / landscape_aspect)) <
                std::abs(std::log(observed_aspect / portrait_aspect));
  const float expected_aspect =
      use_landscape ? landscape_aspect : portrait_aspect;
  const float aspect_error = observed_aspect / expected_aspect;
  if (aspect_error < 0.35f || aspect_error > 2.85f) {
    SetError(error, "observed aspect is inconsistent with target");
    return false;
  }

  std::array<cv::Point2f, 4> target;
  const float half_w = width_cm_ * 0.5f;
  const float half_h = height_cm_ * 0.5f;
  if (!use_landscape) {
    target = {cv::Point2f(-half_w, half_h),
              cv::Point2f(half_w, half_h),
              cv::Point2f(half_w, -half_h),
              cv::Point2f(-half_w, -half_h)};
  } else {
    // Clockwise 90-degree mounting. Orientation stays locked to avoid
    // alternating centimeter axes when perspective changes near a corner.
    target = {cv::Point2f(-half_w, -half_h),
              cv::Point2f(-half_w, half_h),
              cv::Point2f(half_w, half_h),
              cv::Point2f(half_w, -half_h)};
  }

  try {
    const std::vector<cv::Point2f> image_points(ordered.begin(), ordered.end());
    const std::vector<cv::Point2f> target_points(target.begin(), target.end());
    const cv::Mat image_to_target =
        cv::getPerspectiveTransform(image_points, target_points);
    const cv::Mat target_to_image =
        cv::getPerspectiveTransform(target_points, image_points);
    const cv::Matx33f i2t = ToMatx33f(image_to_target);
    const cv::Matx33f t2i = ToMatx33f(target_to_image);
    if (!FiniteMatrix(i2t) || !FiniteMatrix(t2i) ||
        std::abs(cv::determinant(image_to_target)) < 1e-12 ||
        std::abs(cv::determinant(target_to_image)) < 1e-12) {
      SetError(error, "homography is singular or non-finite");
      return false;
    }
    for (size_t i = 0; i < ordered.size(); ++i) {
      const auto reprojected = TransformPoint(target[i], t2i);
      if (!reprojected.has_value() ||
          cv::norm(*reprojected - ordered[i]) > 2.0) {
        SetError(error, "homography reprojection error is too large");
        return false;
      }
    }
    corners_ = ordered;
    image_to_target_ = i2t;
    target_to_image_ = t2i;
    if (!orientation_locked_) {
      landscape_ = use_landscape;
      orientation_locked_ = true;
    }
  } catch (const cv::Exception&) {
    SetError(error, "OpenCV rejected the homography");
    return false;
  }

  timestamp_ = timestamp;
  valid_ = true;
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

void TargetPlane::Invalidate() {
  valid_ = false;
  timestamp_ = 0.0;
  image_to_target_ = cv::Matx33f::eye();
  target_to_image_ = cv::Matx33f::eye();
}

std::optional<cv::Point2f> TargetPlane::ImageToTargetCm(
    const cv::Point2f& image_point) const {
  return valid_ ? TransformPoint(image_point, image_to_target_) : std::nullopt;
}

std::optional<cv::Point2f> TargetPlane::TargetToImage(
    const cv::Point2f& target_point_cm) const {
  return valid_ ? TransformPoint(target_point_cm, target_to_image_)
                : std::nullopt;
}

}  // namespace rkfast
