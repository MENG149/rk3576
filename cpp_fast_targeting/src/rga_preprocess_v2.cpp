#include "rkfast/rga_preprocess.hpp"

#include <cstring>
#include <iostream>

namespace rkfast {

RgaPreprocessor::RgaPreprocessor() {}

RgaPreprocessor::~RgaPreprocessor() { Release(); }

bool RgaPreprocessor::Init(int input_width, int input_height,
                           std::string* error) {
#ifdef RKFAST_USE_RGA
  input_width_ = input_width;
  input_height_ = input_height;

  // Pre-allocate letterbox buffer (RGB format for RKNN)
  letterbox_buffer_ = cv::Mat(input_height_, input_width_, CV_8UC3,
                              cv::Scalar(114, 114, 114));

  // Test RGA availability by checking device
  if (access("/dev/rga", F_OK) != 0 &&
      access("/dev/dri/card0", F_OK) != 0) {
    if (error != nullptr) {
      *error = "RGA device not found (/dev/rga or /dev/dri/card0)";
    }
    available_ = false;
    return false;
  }

  // Try to query RGA capabilities
  int ret = querystring(RGA_VENDOR);
  if (ret < 0) {
    // RGA API v1 - try c_RkRgaInit
    ret = c_RkRgaInit();
    if (ret != IM_STATUS_SUCCESS && ret != 0) {
      if (error != nullptr) {
        *error = "RGA init failed: " + std::to_string(ret);
      }
      available_ = false;
      return false;
    }
  }

  available_ = true;
  std::cout << "RGA hardware acceleration enabled\n";
  return true;
#else
  if (error != nullptr) {
    *error = "RGA support not compiled (define RKFAST_USE_RGA)";
  }
  available_ = false;
  return false;
#endif
}

void RgaPreprocessor::Release() {
#ifdef RKFAST_USE_RGA
  if (available_) {
    c_RkRgaDeInit();
    available_ = false;
  }
#endif
  letterbox_buffer_.release();
}

bool RgaPreprocessor::Process(const cv::Mat& bgr_frame, cv::Mat* rgb_output,
                              float* scale, int* pad_x, int* pad_y,
                              std::string* error) {
#ifdef RKFAST_USE_RGA
  if (!available_ || bgr_frame.empty()) {
    if (error != nullptr) {
      *error = "RGA not available or empty frame";
    }
    return false;
  }

  const int frame_w = bgr_frame.cols;
  const int frame_h = bgr_frame.rows;

  // Calculate letterbox parameters
  const float scale_val = std::min(
      static_cast<float>(input_width_) / std::max(1, frame_w),
      static_cast<float>(input_height_) / std::max(1, frame_h));
  const int new_w = std::max(1, static_cast<int>(std::round(frame_w * scale_val)));
  const int new_h = std::max(1, static_cast<int>(std::round(frame_h * scale_val)));
  const int pad_x_val = (input_width_ - new_w) / 2;
  const int pad_y_val = (input_height_ - new_h) / 2;

  // Reset letterbox to gray
  letterbox_buffer_.setTo(cv::Scalar(114, 114, 114));

  // Try RGA API v2 (im2d)
  rga_buffer_t src;
  rga_buffer_t dst;

  // Setup source buffer
  src = wrapbuffer_virtualaddr(
      const_cast<void*>(static_cast<const void*>(bgr_frame.data)),
      frame_w, frame_h, RK_FORMAT_BGR_888, frame_w * 3);

  // Setup destination buffer (ROI in letterbox)
  cv::Mat dst_roi = letterbox_buffer_(cv::Rect(pad_x_val, pad_y_val, new_w, new_h));
  dst = wrapbuffer_virtualaddr(
      dst_roi.data, new_w, new_h, RK_FORMAT_RGB_888, new_w * 3);

  // RGA resize + color convert
  IM_STATUS status = imresize_t(src, dst, scale_val, scale_val, INTER_LINEAR, 1);

  if (status != IM_STATUS_SUCCESS) {
    // Fallback: try separate operations
    // 1. Resize only
    cv::Mat resized_bgr;
    cv::resize(bgr_frame, resized_bgr, cv::Size(new_w, new_h),
               0, 0, scale_val < 1.0f ? cv::INTER_AREA : cv::INTER_LINEAR);

    // 2. Try RGA color convert only
    rga_buffer_t src2 = wrapbuffer_virtualaddr(
        resized_bgr.data, new_w, new_h, RK_FORMAT_BGR_888, new_w * 3);

    status = imcvtcolor(src2, dst, src2.format, dst.format, IM_COLOR_SPACE_DEFAULT, 1);

    if (status != IM_STATUS_SUCCESS) {
      if (error != nullptr) {
        *error = "RGA operation failed: " + std::to_string(status);
      }
      return false;
    }
  }

  // Output results
  if (rgb_output != nullptr) {
    *rgb_output = letterbox_buffer_;
  }
  if (scale != nullptr) {
    *scale = scale_val;
  }
  if (pad_x != nullptr) {
    *pad_x = pad_x_val;
  }
  if (pad_y != nullptr) {
    *pad_y = pad_y_val;
  }

  return true;
#else
  if (error != nullptr) {
    *error = "RGA not compiled";
  }
  return false;
#endif
}

}  // namespace rkfast
