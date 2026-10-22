#pragma once

#include <array>
#include <optional>
#include <string>

#include <opencv2/core.hpp>

namespace rkfast {

class TargetPlane {
 public:
  explicit TargetPlane(float width_cm = 21.0f, float height_cm = 29.7f);

  bool Update(const std::array<cv::Point2f, 4>& corners,
              const cv::Size& image_size, double timestamp,
              std::string* error = nullptr);
  void Invalidate();

  bool Valid() const { return valid_; }
  double Timestamp() const { return timestamp_; }
  const std::array<cv::Point2f, 4>& Corners() const { return corners_; }
  const cv::Matx33f& ImageToTargetMatrix() const { return image_to_target_; }
  const cv::Matx33f& TargetToImageMatrix() const { return target_to_image_; }

  std::optional<cv::Point2f> ImageToTargetCm(
      const cv::Point2f& image_point) const;
  std::optional<cv::Point2f> TargetToImage(
      const cv::Point2f& target_point_cm) const;

 private:
  float width_cm_;
  float height_cm_;
  bool valid_ = false;
  bool orientation_locked_ = false;
  bool landscape_ = false;
  double timestamp_ = 0.0;
  std::array<cv::Point2f, 4> corners_{};
  cv::Matx33f image_to_target_ = cv::Matx33f::eye();
  cv::Matx33f target_to_image_ = cv::Matx33f::eye();
};

}  // namespace rkfast
