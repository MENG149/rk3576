#ifndef RKFAST_RGA_PREPROCESS_HPP_
#define RKFAST_RGA_PREPROCESS_HPP_

#include <opencv2/core.hpp>
#include <optional>
#include <string>

#ifdef RKFAST_USE_RGA
#include <rga/RgaApi.h>
#include <rga/im2d.h>
#endif

namespace rkfast {

class RgaPreprocessor {
 public:
  RgaPreprocessor();
  ~RgaPreprocessor();

  bool Init(int input_width, int input_height, std::string* error = nullptr);
  void Release();

  // RGA-accelerated preprocessing: resize + letterbox + BGR2RGB
  bool Process(const cv::Mat& bgr_frame, cv::Mat* rgb_output,
               float* scale, int* pad_x, int* pad_y,
               std::string* error = nullptr);

  bool IsAvailable() const { return available_; }

 private:
  bool available_ = false;
  int input_width_ = 0;
  int input_height_ = 0;
  cv::Mat letterbox_buffer_;

#ifdef RKFAST_USE_RGA
  rga_buffer_t src_buf_{};
  rga_buffer_t dst_buf_{};
#endif
};

}  // namespace rkfast

#endif  // RKFAST_RGA_PREPROCESS_HPP_
