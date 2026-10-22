#pragma once

#include <optional>

#include <opencv2/core.hpp>

#include "rkfast/config.hpp"
#include "rkfast/types.hpp"

namespace rkfast {

class RectRefiner {
 public:
  explicit RectRefiner(const AppConfig& config);

  std::optional<RefinedRect> Refine(
      const cv::Mat& frame_bgr,
      const Box& guide_box,
      const std::optional<cv::Point2f>& reference_center = std::nullopt,
      const RefineHints& hints = {}) const;

 private:
  std::optional<RefinedRect> SearchContours(
      const cv::Mat& binary,
      const cv::Mat& gray,
      const cv::Rect& roi,
      const cv::Point2f& target_center,
      const RefineHints& hints) const;
  // Centroid of red pixels inside the refined quad (full-frame coords). Scans
  // only the HSV of the already-cropped ROI, masked to the quad interior.
  std::optional<cv::Point2f> DetectRedCenter(
      const cv::Mat& roi_bgr, const cv::Rect& roi,
      const std::array<cv::Point2f, 4>& corners, int* red_pixels) const;
  static std::array<cv::Point2f, 4> OrderCorners(const std::vector<cv::Point>& polygon);
  static float RightAngleScore(const std::array<cv::Point2f, 4>& corners);
  static void RefineCornersSubPix(const cv::Mat& gray, const cv::Rect& roi,
                                  std::array<cv::Point2f, 4>* corners);

  AppConfig config_;
};

}  // namespace rkfast
