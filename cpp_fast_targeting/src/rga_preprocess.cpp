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

  // Test RGA availability
  int ret = c_RkRgaInit();
  // Note: Some RGA versions return 0 for success instead of IM_STATUS_SUCCESS
  if (ret != IM_STATUS_SUCCESS && ret != 0) {
    if (error != nullptr) {
      *error = "RGA init failed: " + std::to_string(ret);
    }
    available_ = false;
    return false;
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

  // Setup RGA buffers (using wstride and hstride parameters)
  rga_buffer_t src = wrapbuffer_virtualaddr(
      const_cast<void*>(static_cast<const void*>(bgr_frame.data)),
      frame_w, frame_h, RK_FORMAT_BGR_888, frame_w, frame_h);

  rga_buffer_t dst = wrapbuffer_virtualaddr(
      letterbox_buffer_(cv::Rect(pad_x_val, pad_y_val, new_w, new_h)).data,
      new_w, new_h, RK_FORMAT_RGB_888, new_w, new_h);

  // RGA resize + BGR2RGB in one operation (sync=1 for blocking)
  IM_STATUS status = imresize_t(src, dst, scale_val, scale_val, INTER_LINEAR, 1);
  if (status != IM_STATUS_SUCCESS) {
    // Fallback: use RGA for format conversion only
    status = imcvtcolor(src, dst, src.format, dst.format);
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
