#pragma once

#include <optional>
#include <string>

#include <opencv2/core.hpp>

#include "rkfast/config.hpp"
#include "rkfast/types.hpp"

namespace rkfast {

class AimWriter {
 public:
  explicit AimWriter(const AppConfig& config);
  ~AimWriter();

  bool Open(std::string* error);
  void Close();
  // aim_center, when set, overrides the axis-aligned bbox center as the point
  // to transmit (e.g. the perspective-corrected target-plane center). Velocity
  // still comes from the tracker. When nullopt the snapshot bbox center is used.
  void Send(const TrackSnapshot& snapshot, int frame_w, int frame_h,
            const std::optional<cv::Point2f>& aim_center = std::nullopt);

 private:
  static std::string FormatAim(const TrackSnapshot& snapshot,
                               const cv::Point2f& aim_center, int frame_w,
                               int frame_h);
  // Build the 12-byte erweiyuntai vision packet into out[12]. Center/velocity
  // are scaled from the capture frame into the receiver's frame size.
  void BuildPacket(const TrackSnapshot& snapshot, const cv::Point2f& aim_center,
                   int frame_w, int frame_h, uint8_t out[12]) const;
  bool ConfigureSerial(int fd, std::string* error) const;

  AppConfig config_;
  int fd_ = -1;
  double last_stdout_ = 0.0;
  double last_serial_ = 0.0;
};

}  // namespace rkfast
