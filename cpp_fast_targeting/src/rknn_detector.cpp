#include "rkfast/rknn_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <optional>
#include <sstream>
#include <thread>

#include <opencv2/imgproc.hpp>

#include "rkfast/thread_affinity.hpp"

namespace rkfast {

namespace {

uint32_t TensorElementCount(const rknn_tensor_attr& attr) {
  uint32_t count = 1;
  for (uint32_t i = 0; i < attr.n_dims; ++i) {
    const uint32_t dim = attr.dims[i] == 0 ? 1 : attr.dims[i];
    count *= dim;
  }
  return std::max<uint32_t>(count, 1);
}

uint64_t Fnv1a64(const uint8_t* data, size_t size) {
  uint64_t hash = 1469598103934665603ULL;
  for (size_t i = 0; i < size; ++i) {
    hash ^= data[i];
    hash *= 1099511628211ULL;
  }
  return hash;
}

uint16_t FloatToHalf(float value) {
  uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  const uint32_t sign = (bits >> 16) & 0x8000u;
  int32_t exponent = static_cast<int32_t>((bits >> 23) & 0xffu) - 127 + 15;
  uint32_t mantissa = bits & 0x007fffffu;
  if (exponent <= 0) {
    return static_cast<uint16_t>(sign);
  }
  if (exponent >= 31) {
    return static_cast<uint16_t>(sign | 0x7c00u);
  }
  mantissa += 0x00000fffu + ((mantissa >> 13) & 1u);
  if ((mantissa & 0x00800000u) != 0) {
    mantissa = 0;
    ++exponent;
    if (exponent >= 31) {
      return static_cast<uint16_t>(sign | 0x7c00u);
    }
  }
  return static_cast<uint16_t>(
      sign | (static_cast<uint32_t>(exponent) << 10) | (mantissa >> 13));
}

const std::array<uint16_t, 256>& NormalizedFp16Lut() {
  static const std::array<uint16_t, 256> lut = [] {
    std::array<uint16_t, 256> values{};
    for (size_t i = 0; i < values.size(); ++i) {
      values[i] = FloatToHalf(static_cast<float>(i) / 255.0f);
    }
    return values;
  }();
  return lut;
}

#ifdef RKFAST_RKNN_HAS_CORE_MASK
std::optional<rknn_core_mask> ResolveCoreMask(const std::string& name,
                                              std::string* error) {
  if (name.empty() || name == "default") {
    return std::nullopt;
  }
  if (name == "auto") {
    return RKNN_NPU_CORE_AUTO;
  }
  if (name == "0") {
    return RKNN_NPU_CORE_0;
  }
  if (name == "1") {
    return RKNN_NPU_CORE_1;
  }
  if (name == "2") {
    return RKNN_NPU_CORE_2;
  }
  if (name == "01") {
    return RKNN_NPU_CORE_0_1;
  }
  if (name == "012") {
    return RKNN_NPU_CORE_0_1_2;
  }
  if (name == "all") {
    return RKNN_NPU_CORE_ALL;
  }
  if (error != nullptr) {
    *error = "unsupported RKNN core mask: " + name;
  }
  return std::nullopt;
}
#endif

float HalfToFloat(uint16_t value) {
  const uint32_t sign = (value & 0x8000u) << 16;
  uint32_t exponent = (value >> 10) & 0x1fu;
  uint32_t mantissa = value & 0x03ffu;

  if (exponent == 0) {
    if (mantissa == 0) {
      uint32_t bits = sign;
      float out;
      std::memcpy(&out, &bits, sizeof(out));
      return out;
    }
    while ((mantissa & 0x0400u) == 0) {
      mantissa <<= 1;
      --exponent;
    }
    ++exponent;
    mantissa &= ~0x0400u;
  } else if (exponent == 31) {
    uint32_t bits = sign | 0x7f800000u | (mantissa << 13);
    float out;
    std::memcpy(&out, &bits, sizeof(out));
    return out;
  }

  exponent = exponent + (127 - 15);
  const uint32_t bits = sign | (exponent << 23) | (mantissa << 13);
  float out;
  std::memcpy(&out, &bits, sizeof(out));
  return out;
}

}  // namespace

RknnDetector::RknnDetector(const AppConfig& config) : config_(config) {}

RknnDetector::~RknnDetector() { Close(); }

bool RknnDetector::Open(std::string* error) {
  if (config_.rknn_input_mode != "auto" &&
      config_.rknn_input_mode != "legacy" &&
      config_.rknn_input_mode != "zero-copy") {
    if (error != nullptr) {
      *error = "unsupported RKNN input mode: " + config_.rknn_input_mode;
    }
    return false;
  }
  if (!LoadModel(error)) {
    return false;
  }
  int ret = rknn_init(&context_, model_data_.data(),
                      static_cast<uint32_t>(model_data_.size()), 0, nullptr);
  if (ret != RKNN_SUCC) {
    if (error != nullptr) {
      *error = "rknn_init failed: " + std::to_string(ret);
    }
    return false;
  }
#ifdef RKFAST_RKNN_HAS_CORE_MASK
  const auto core_mask = ResolveCoreMask(config_.core_mask, error);
  if (!config_.core_mask.empty() && config_.core_mask != "default") {
    if (!core_mask.has_value()) {
      Close();
      return false;
    }
    ret = rknn_set_core_mask(context_, *core_mask);
    if (ret != RKNN_SUCC) {
      if (error != nullptr) {
        *error = "rknn_set_core_mask(" + config_.core_mask +
                 ") failed: " + std::to_string(ret);
      }
      Close();
      return false;
    }
    std::cout << "RKNN core_mask=" << config_.core_mask << "\n";
  } else {
    std::cout << "RKNN core_mask=default\n";
  }
#else
  std::cout << "RKNN core_mask=default (API not available in this runtime)\n";
#endif
  if (!QueryTensors(error)) {
    Close();
    return false;
  }
  input_rgb_ = cv::Mat(input_height_, input_width_, CV_8UC3,
                       cv::Scalar(114, 114, 114));
  if (input_is_nchw_) {
    const size_t plane = static_cast<size_t>(input_width_) * input_height_;
    input_chw_.resize(plane * 3);
    input_chw_views_[0] = cv::Mat(input_height_, input_width_, CV_8UC1,
                                  input_chw_.data() + plane * 0);
    input_chw_views_[1] = cv::Mat(input_height_, input_width_, CV_8UC1,
                                  input_chw_.data() + plane * 1);
    input_chw_views_[2] = cv::Mat(input_height_, input_width_, CV_8UC1,
                                  input_chw_.data() + plane * 2);
  }

  if (config_.rknn_input_mode == "legacy") {
    std::cout << "RKNN input_mode=legacy\n";
  } else {
    std::string zero_copy_error;
    if (!SetupZeroCopyInput(&zero_copy_error)) {
      if (config_.rknn_input_mode == "zero-copy") {
        if (error != nullptr) {
          *error = zero_copy_error;
        }
        Close();
        return false;
      }
      std::cerr << "RKNN zero-copy input unavailable (" << zero_copy_error
                << "), falling back to legacy\n";
      std::cout << "RKNN input_mode=legacy\n";
    }
  }

  // Try to initialize RGA hardware acceleration
  std::string rga_error;
  if (rga_preprocessor_.Init(input_width_, input_height_, &rga_error)) {
    std::cout << "RGA hardware acceleration initialized\n";
  } else {
    std::cout << "RGA init failed (" << rga_error << "), using OpenCV fallback\n";
  }

  opened_ = true;
  return true;
}

void RknnDetector::Close() {
  if (input_mem_ != nullptr && context_ != 0) {
    (void)rknn_destroy_mem(context_, input_mem_);
    input_mem_ = nullptr;
  }
  using_zero_copy_input_ = false;
  if (context_ != 0) {
    rknn_destroy(context_);
    context_ = 0;
  }
  opened_ = false;
}

bool RknnDetector::Warmup(int count, std::string* error) {
  cv::Mat dummy(input_height_, input_width_, CV_8UC3,
                cv::Scalar(114, 114, 114));
  for (int i = 0; i < count; ++i) {
    InferTimings timings;
    if (!Infer(dummy, std::nullopt, &timings, error).has_value() && error != nullptr &&
        !error->empty()) {
      return false;
    }
  }
  return true;
}

std::optional<Detection> RknnDetector::Infer(const cv::Mat& frame,
                                             const std::optional<Box>& prior,
                                             InferTimings* timings,
                                             std::string* error) {
  if (!opened_) {
    if (error != nullptr) {
      *error = "detector is not open";
    }
    return std::nullopt;
  }
  const double start = NowSeconds();
  PreprocessMeta meta;

  const double t0 = NowSeconds();
  Preprocess(frame, &meta);
  const double t1 = NowSeconds();

  const uint8_t* input_data = InputData();
  const size_t input_bytes = InputBytes();
  if (input_data == nullptr || input_bytes == 0 ||
      input_bytes > std::numeric_limits<uint32_t>::max()) {
    if (error != nullptr) {
      *error = "invalid preprocessed RKNN input buffer";
    }
    return std::nullopt;
  }

  const double ts0 = NowSeconds();
  int ret = RKNN_SUCC;
  if (using_zero_copy_input_) {
    const size_t native_bytes = input_bytes * sizeof(uint16_t);
    if (input_mem_ == nullptr || input_mem_->virt_addr == nullptr ||
        input_mem_->size < native_bytes) {
      if (error != nullptr) {
        *error = "RKNN zero-copy input buffer is invalid";
      }
      return std::nullopt;
    }
    const auto& lut = NormalizedFp16Lut();
    auto* native_input = static_cast<uint16_t*>(input_mem_->virt_addr);
    for (size_t i = 0; i < input_bytes; ++i) {
      native_input[i] = lut[input_data[i]];
    }
    ret = rknn_mem_sync(context_, input_mem_, RKNN_MEMORY_SYNC_TO_DEVICE);
  } else {
    rknn_input input{};
    input.index = 0;
    input.type = RKNN_TENSOR_UINT8;
    input.size = static_cast<uint32_t>(input_bytes);
    input.fmt = input_is_nchw_ ? RKNN_TENSOR_NCHW : RKNN_TENSOR_NHWC;
    input.pass_through = 0;
    input.buf = const_cast<uint8_t*>(input_data);
    ret = rknn_inputs_set(context_, 1, &input);
  }
  const double ts1 = NowSeconds();
  if (ret != RKNN_SUCC) {
    if (error != nullptr) {
      *error = std::string(using_zero_copy_input_ ? "rknn_mem_sync(input)"
                                                  : "rknn_inputs_set") +
               " failed: " + std::to_string(ret);
    }
    return std::nullopt;
  }

  const double t2 = NowSeconds();
  ret = rknn_run(context_, nullptr);
  const double t3 = NowSeconds();
  if (ret != RKNN_SUCC) {
    if (error != nullptr) {
      *error = "rknn_run failed: " + std::to_string(ret);
    }
    return std::nullopt;
  }

  std::vector<rknn_output> outputs(io_num_.n_output);
  for (uint32_t i = 0; i < io_num_.n_output; ++i) {
    outputs[i].want_float = 0;
    outputs[i].is_prealloc = 0;
    // rknn_api.h documents index as the output selector for rknn_outputs_get.
    // Leaving it 0 makes every request read output 0 and misassigns attrs the
    // moment a multi-head / keypoint model is loaded. Harmless for the current
    // single-output model, required for any future multi-output export.
    outputs[i].index = i;
  }

  const double tg0 = NowSeconds();
  ret = rknn_outputs_get(context_, io_num_.n_output, outputs.data(), nullptr);
  const double t4 = NowSeconds();
  if (ret != RKNN_SUCC) {
    if (error != nullptr) {
      *error = "rknn_outputs_get failed: " + std::to_string(ret);
    }
    return std::nullopt;
  }

  std::vector<OutputMatrix> matrices;
  matrices.reserve(outputs.size());
  for (uint32_t i = 0; i < io_num_.n_output; ++i) {
    matrices.push_back(ToMatrix(output_attrs_[i], outputs[i]));
  }
  const auto detections = DecodeOutputs(matrices, meta);
  const double t5 = NowSeconds();
  const auto selected =
      ChooseDetection(detections, prior, frame.cols, frame.rows);
  const double t6 = NowSeconds();

  const double tr0 = NowSeconds();
  (void)rknn_outputs_release(context_, io_num_.n_output, outputs.data());
  const double tr1 = NowSeconds();

  if (timings != nullptr) {
    timings->input_bytes = static_cast<uint32_t>(input_bytes);
    timings->input_checksum =
        config_.verify_input_checksum ? Fnv1a64(input_data, input_bytes) : 0;
    timings->preprocess_ms = (t1 - t0) * 1000.0;
    timings->inputs_set_ms = (ts1 - ts0) * 1000.0;
    timings->rknn_ms = (t3 - t2) * 1000.0;
    timings->outputs_get_ms = (t4 - tg0) * 1000.0;
    timings->postprocess_ms = (t5 - t4) * 1000.0;
    timings->select_ms = (t6 - t5) * 1000.0;
    timings->outputs_release_ms = (tr1 - tr0) * 1000.0;
    timings->total_ms = (NowSeconds() - start) * 1000.0;
  }
  return selected;
}

bool RknnDetector::LoadModel(std::string* error) {
  std::ifstream ifs(config_.model_path, std::ios::binary);
  if (!ifs) {
    if (error != nullptr) {
      *error = "could not open model: " + config_.model_path;
    }
    return false;
  }
  model_data_ = std::vector<uint8_t>(std::istreambuf_iterator<char>(ifs),
                                     std::istreambuf_iterator<char>());
  if (model_data_.empty()) {
    if (error != nullptr) {
      *error = "model file is empty: " + config_.model_path;
    }
    return false;
  }
  return true;
}

bool RknnDetector::QueryTensors(std::string* error) {
  int ret = rknn_query(context_, RKNN_QUERY_IN_OUT_NUM, &io_num_,
                       sizeof(io_num_));
  if (ret != RKNN_SUCC) {
    if (error != nullptr) {
      *error = "RKNN_QUERY_IN_OUT_NUM failed: " + std::to_string(ret);
    }
    return false;
  }
  if (io_num_.n_input != 1) {
    if (error != nullptr) {
      *error = "expected exactly one input tensor";
    }
    return false;
  }

  std::memset(&input_attr_, 0, sizeof(input_attr_));
  input_attr_.index = 0;
  ret = rknn_query(context_, RKNN_QUERY_INPUT_ATTR, &input_attr_,
                   sizeof(input_attr_));
  if (ret != RKNN_SUCC) {
    if (error != nullptr) {
      *error = "RKNN_QUERY_INPUT_ATTR failed: " + std::to_string(ret);
    }
    return false;
  }
  input_is_nchw_ = input_attr_.fmt == RKNN_TENSOR_NCHW;

  if (input_attr_.n_dims < 4) {
    if (error != nullptr) {
      *error = "unexpected RKNN input dims count: " +
               std::to_string(input_attr_.n_dims);
    }
    return false;
  }
  if (input_is_nchw_) {
    input_channels_ = static_cast<int>(input_attr_.dims[1]);
    input_height_ = static_cast<int>(input_attr_.dims[2]);
    input_width_ = static_cast<int>(input_attr_.dims[3]);
  } else {
    input_height_ = static_cast<int>(input_attr_.dims[1]);
    input_width_ = static_cast<int>(input_attr_.dims[2]);
    input_channels_ = static_cast<int>(input_attr_.dims[3]);
  }
  if (input_width_ <= 0 || input_height_ <= 0 || input_channels_ != 3) {
    if (error != nullptr) {
      *error = "unexpected RKNN input shape";
    }
    return false;
  }
  if (config_.input_size != input_width_ || config_.input_size != input_height_) {
    std::cout << "RKNN model input is " << input_width_ << "x" << input_height_
              << ", overriding requested --input-size " << config_.input_size
              << "\n";
  }

  output_attrs_.resize(io_num_.n_output);
  for (uint32_t i = 0; i < io_num_.n_output; ++i) {
    std::memset(&output_attrs_[i], 0, sizeof(rknn_tensor_attr));
    output_attrs_[i].index = i;
    ret = rknn_query(context_, RKNN_QUERY_OUTPUT_ATTR, &output_attrs_[i],
                     sizeof(rknn_tensor_attr));
    if (ret != RKNN_SUCC) {
      if (error != nullptr) {
        *error = "RKNN_QUERY_OUTPUT_ATTR failed: " + std::to_string(ret);
      }
      return false;
    }
  }
  return true;
}

bool RknnDetector::SetupZeroCopyInput(std::string* error) {
  const size_t input_bytes = InputBytes();
  if (input_bytes == 0 ||
      input_bytes > std::numeric_limits<uint32_t>::max()) {
    if (error != nullptr) {
      *error = "invalid logical input size for zero-copy";
    }
    return false;
  }

  std::memset(&zero_copy_input_attr_, 0, sizeof(zero_copy_input_attr_));
  zero_copy_input_attr_.index = 0;
  int ret = rknn_query(context_, RKNN_QUERY_NATIVE_INPUT_ATTR,
                       &zero_copy_input_attr_,
                       sizeof(zero_copy_input_attr_));
  if (ret != RKNN_SUCC) {
    if (error != nullptr) {
      *error = "RKNN_QUERY_NATIVE_INPUT_ATTR failed: " +
               std::to_string(ret);
    }
    return false;
  }
  const rknn_tensor_format expected_fmt =
      input_is_nchw_ ? RKNN_TENSOR_NCHW : RKNN_TENSOR_NHWC;
  if (zero_copy_input_attr_.type != RKNN_TENSOR_FLOAT16 ||
      zero_copy_input_attr_.fmt != expected_fmt) {
    if (error != nullptr) {
      *error = "native zero-copy input is not matching-layout FLOAT16";
    }
    return false;
  }

  const size_t native_bytes = input_bytes * sizeof(uint16_t);
  const uint32_t capacity = std::max(
      static_cast<uint32_t>(native_bytes),
      std::max(zero_copy_input_attr_.size,
               zero_copy_input_attr_.size_with_stride));
  input_mem_ = rknn_create_mem(context_, capacity);
  if (input_mem_ == nullptr || input_mem_->virt_addr == nullptr) {
    if (error != nullptr) {
      *error = "rknn_create_mem(input) failed";
    }
    input_mem_ = nullptr;
    return false;
  }

  zero_copy_input_attr_.pass_through = 1;
  ret = rknn_set_io_mem(context_, input_mem_, &zero_copy_input_attr_);
  if (ret != RKNN_SUCC) {
    if (error != nullptr) {
      *error = "rknn_set_io_mem(input) failed: " + std::to_string(ret);
    }
    (void)rknn_destroy_mem(context_, input_mem_);
    input_mem_ = nullptr;
    return false;
  }

  using_zero_copy_input_ = true;
  std::cout << "RKNN input_mode=zero-copy logical_bytes=" << input_bytes
            << " memory_bytes=" << input_mem_->size
            << " native_size=" << zero_copy_input_attr_.size
            << " native_stride_size=" << zero_copy_input_attr_.size_with_stride
            << " native_type=" << zero_copy_input_attr_.type
            << " native_fmt=" << zero_copy_input_attr_.fmt
            << " w_stride=" << zero_copy_input_attr_.w_stride << "\n";
  return true;
}

const uint8_t* RknnDetector::InputData() const {
  return input_is_nchw_ ? input_chw_.data() : input_rgb_.data;
}

size_t RknnDetector::InputBytes() const {
  return input_is_nchw_ ? input_chw_.size()
                        : input_rgb_.total() * input_rgb_.elemSize();
}

void RknnDetector::Preprocess(const cv::Mat& frame, PreprocessMeta* meta) {
  const int frame_w = frame.cols;
  const int frame_h = frame.rows;

  // Try RGA hardware acceleration first
  if (rga_preprocessor_.IsAvailable()) {
    float scale = 1.0f;
    int pad_x = 0;
    int pad_y = 0;
    std::string rga_error;

    if (rga_preprocessor_.Process(frame, &input_rgb_, &scale, &pad_x, &pad_y, &rga_error)) {
      // RGA success - convert to NCHW if needed
      if (input_is_nchw_) {
        cv::split(input_rgb_, input_chw_views_);
      }

      if (meta != nullptr) {
        meta->scale = scale;
        meta->pad_x = pad_x;
        meta->pad_y = pad_y;
        meta->frame_w = frame_w;
        meta->frame_h = frame_h;
      }
      return;
    }
    // RGA failed, fall through to OpenCV
  }

  // OpenCV fallback path
  const float scale = std::min(
      static_cast<float>(input_width_) / std::max(1, frame_w),
      static_cast<float>(input_height_) / std::max(1, frame_h));
  const int new_w = std::max(1, static_cast<int>(std::round(frame_w * scale)));
  const int new_h = std::max(1, static_cast<int>(std::round(frame_h * scale)));
  const int pad_x = (input_width_ - new_w) / 2;
  const int pad_y = (input_height_ - new_h) / 2;

  input_rgb_.setTo(cv::Scalar(114, 114, 114));
  if (new_w == frame_w && new_h == frame_h) {
    cv::cvtColor(frame,
                 input_rgb_(cv::Rect(pad_x, pad_y, new_w, new_h)),
                 cv::COLOR_BGR2RGB);
  } else {
    cv::resize(frame, resized_bgr_, cv::Size(new_w, new_h),
               0.0, 0.0, scale < 1.0f ? cv::INTER_AREA : cv::INTER_LINEAR);
    cv::cvtColor(resized_bgr_,
                 input_rgb_(cv::Rect(pad_x, pad_y, new_w, new_h)),
                 cv::COLOR_BGR2RGB);
  }

  if (input_is_nchw_) {
    cv::split(input_rgb_, input_chw_views_);
  }

  if (meta != nullptr) {
    meta->scale = scale;
    meta->pad_x = pad_x;
    meta->pad_y = pad_y;
    meta->frame_w = frame_w;
    meta->frame_h = frame_h;
  }
}

RknnDetector::OutputMatrix RknnDetector::ToMatrix(const rknn_tensor_attr& attr,
                                                  const rknn_output& output) const {
  OutputMatrix matrix;
  const uint32_t count = TensorElementCount(attr);
  if (count == 0 || output.buf == nullptr) {
    return matrix;
  }

  std::vector<int> dims;
  dims.reserve(attr.n_dims);
  for (uint32_t i = 0; i < attr.n_dims; ++i) {
    const int dim = static_cast<int>(attr.dims[i]);
    if (dim > 1) {
      dims.push_back(dim);
    }
  }

  if (dims.empty()) {
    matrix.rows = 1;
    matrix.cols = static_cast<int>(count);
  } else if (dims.size() == 1) {
    matrix.rows = 1;
    matrix.cols = dims[0];
  } else {
    matrix.rows = 1;
    for (size_t i = 0; i + 1 < dims.size(); ++i) {
      matrix.rows *= dims[i];
    }
    matrix.cols = dims.back();
  }

  if (matrix.rows <= 0 || matrix.cols <= 0 ||
      static_cast<uint32_t>(matrix.rows * matrix.cols) != count) {
    matrix.rows = 1;
    matrix.cols = static_cast<int>(count);
  }

  matrix.data.resize(count);
  switch (attr.type) {
    case RKNN_TENSOR_FLOAT32: {
      const float* src = static_cast<const float*>(output.buf);
      std::copy(src, src + count, matrix.data.begin());
      break;
    }
    case RKNN_TENSOR_FLOAT16: {
      const uint16_t* src = static_cast<const uint16_t*>(output.buf);
      for (uint32_t i = 0; i < count; ++i) {
        matrix.data[i] = HalfToFloat(src[i]);
      }
      break;
    }
    case RKNN_TENSOR_INT8: {
      const int8_t* src = static_cast<const int8_t*>(output.buf);
      for (uint32_t i = 0; i < count; ++i) {
        matrix.data[i] = (static_cast<int>(src[i]) - attr.zp) * attr.scale;
      }
      break;
    }
    case RKNN_TENSOR_UINT8: {
      const uint8_t* src = static_cast<const uint8_t*>(output.buf);
      for (uint32_t i = 0; i < count; ++i) {
        matrix.data[i] = (static_cast<int>(src[i]) - attr.zp) * attr.scale;
      }
      break;
    }
    default:
      matrix.data.assign(count, 0.0f);
      break;
  }

  if (matrix.rows <= 256 && matrix.rows < matrix.cols) {
    std::vector<float> transposed(count);
    for (int r = 0; r < matrix.rows; ++r) {
      for (int c = 0; c < matrix.cols; ++c) {
        transposed[c * matrix.rows + r] = matrix.data[r * matrix.cols + c];
      }
    }
    std::swap(matrix.rows, matrix.cols);
    matrix.data.swap(transposed);
  }
  return matrix;
}

std::vector<Detection> RknnDetector::DecodeOutputs(
    const std::vector<OutputMatrix>& matrices, const PreprocessMeta& meta) const {
  int width = 0;
  size_t total_rows = 0;
  for (const auto& matrix : matrices) {
    if (matrix.rows <= 0 || matrix.cols < 5) {
      continue;
    }
    if (width == 0) {
      width = matrix.cols;
    }
    if (matrix.cols == width) {
      total_rows += static_cast<size_t>(matrix.rows);
    }
  }
  if (width < 5 || total_rows == 0) {
    return {};
  }

  std::vector<float> pred(total_rows * static_cast<size_t>(width));
  size_t row_offset = 0;
  for (const auto& matrix : matrices) {
    if (matrix.cols != width || matrix.rows <= 0) {
      continue;
    }
    std::copy(matrix.data.begin(), matrix.data.end(),
              pred.begin() + row_offset * static_cast<size_t>(width));
    row_offset += static_cast<size_t>(matrix.rows);
  }

  std::vector<Detection> detections;
  detections.reserve(static_cast<size_t>(config_.max_det) * 4);

  for (size_t row = 0; row < total_rows; ++row) {
    const float* item = pred.data() + row * static_cast<size_t>(width);
    const float cx0 = item[0];
    const float cy0 = item[1];
    const float bw0 = item[2];
    const float bh0 = item[3];
    if (!std::isfinite(cx0) || !std::isfinite(cy0) || !std::isfinite(bw0) ||
        !std::isfinite(bh0)) {
      continue;
    }

    float cx = cx0;
    float cy = cy0;
    float bw = bw0;
    float bh = bh0;
    if (std::max(std::max(std::fabs(cx), std::fabs(cy)),
                 std::max(std::fabs(bw), std::fabs(bh))) <= 2.0f) {
      cx *= static_cast<float>(input_width_);
      cy *= static_cast<float>(input_height_);
      bw *= static_cast<float>(input_width_);
      bh *= static_cast<float>(input_height_);
    }

    const int rest = width - 4;
    int chosen_class = -1;
    float score = 0.0f;
    if (config_.class_count > 0 && rest == config_.class_count) {
      if (config_.target_class >= 0 && config_.target_class < rest) {
        chosen_class = config_.target_class;
        score = item[4 + config_.target_class];
      } else {
        for (int i = 0; i < rest; ++i) {
          if (item[4 + i] > score) {
            score = item[4 + i];
            chosen_class = i;
          }
        }
      }
    } else if (config_.class_count > 0 && rest >= config_.class_count + 1) {
      const float objectness = item[4];
      if (config_.target_class >= 0 && config_.target_class < config_.class_count) {
        chosen_class = config_.target_class;
        score = objectness * item[5 + config_.target_class];
      } else {
        for (int i = 0; i < config_.class_count; ++i) {
          const float cls_score = objectness * item[5 + i];
          if (cls_score > score) {
            score = cls_score;
            chosen_class = i;
          }
        }
      }
    } else {
      continue;
    }

    // A NaN score passes "score < conf_threshold" (any comparison with NaN is
    // false), then poisons the std::sort comparator below (strict-weak-order
    // violation -> UB) and can be emitted as a bogus detection. Reject it here.
    if (chosen_class < 0 || !std::isfinite(score) ||
        score < config_.conf_threshold) {
      continue;
    }

    // Non-positive width/height must be rejected BEFORE ClipBox: ClipBox swaps
    // x2<x1 / y2<y1 into a superficially-valid box, silently turning a garbage
    // negative-size prediction into a plausible-looking target.
    if (!(bw > 0.0f) || !(bh > 0.0f)) {
      continue;
    }

    Box box;
    box.x1 = (cx - bw * 0.5f - meta.pad_x) / std::max(meta.scale, 1e-6f);
    box.y1 = (cy - bh * 0.5f - meta.pad_y) / std::max(meta.scale, 1e-6f);
    box.x2 = (cx + bw * 0.5f - meta.pad_x) / std::max(meta.scale, 1e-6f);
    box.y2 = (cy + bh * 0.5f - meta.pad_y) / std::max(meta.scale, 1e-6f);
    if (!std::isfinite(box.x1) || !std::isfinite(box.y1) ||
        !std::isfinite(box.x2) || !std::isfinite(box.y2)) {
      continue;
    }
    box = ClipBox(box, meta.frame_w, meta.frame_h);

    if (BoxWidth(box) < config_.min_box_size ||
        BoxHeight(box) < config_.min_box_size) {
      continue;
    }

    Detection det;
    det.box = box;
    det.center = BoxCenter(box);
    det.conf = score;
    det.class_id = chosen_class;
    det.candidate_count = 1;
    detections.push_back(det);
  }

  std::sort(detections.begin(), detections.end(),
            [](const Detection& a, const Detection& b) {
              return a.conf > b.conf;
            });

  // Keep up to max(max_det, candidate_pool) post-NMS candidates so the
  // downstream selector/association has real alternatives even when only one
  // final target is emitted (max_det=1). Selection to a single target happens
  // later in ChooseDetection / the tracker, not by starving NMS here.
  const int pool_cap =
      std::max(1, std::max(config_.max_det, config_.candidate_pool));
  std::vector<Detection> kept;
  kept.reserve(std::min<size_t>(static_cast<size_t>(pool_cap),
                                detections.size()));
  for (const auto& det : detections) {
    bool keep = true;
    for (const auto& prev : kept) {
      if (BoxIou(det.box, prev.box) > config_.iou_threshold) {
        keep = false;
        break;
      }
    }
    if (!keep) {
      continue;
    }
    auto det_copy = det;
    det_copy.candidate_count = static_cast<int>(detections.size());
    kept.push_back(det_copy);
    if (static_cast<int>(kept.size()) >= pool_cap) {
      break;
    }
  }
  return kept;
}

std::optional<Detection> RknnDetector::ChooseDetection(
    const std::vector<Detection>& detections, const std::optional<Box>& prior,
    int frame_w, int frame_h) const {
  if (detections.empty()) {
    return std::nullopt;
  }
  if (!prior.has_value()) {
    return detections.front();
  }

  const cv::Point2f prior_center = BoxCenter(*prior);
  const float diag =
      std::max(1.0f, static_cast<float>(std::hypot(frame_w, frame_h)));
  std::optional<Detection> best;
  float best_score = -std::numeric_limits<float>::infinity();
  for (const auto& det : detections) {
    const float dist = static_cast<float>(cv::norm(det.center - prior_center)) /
                       diag;
    float score = det.conf;
    score += 0.35f * std::max(0.0f, 1.0f - dist * 3.0f);
    score += 0.30f * BoxIou(det.box, *prior);
    if (!best.has_value() || score > best_score) {
      best = det;
      best_score = score;
    }
  }
  return best;
}

DetectorWorker::DetectorWorker(std::unique_ptr<RknnDetector> detector,
                               std::string affinity)
    : detector_(std::move(detector)), affinity_(std::move(affinity)) {}

DetectorWorker::~DetectorWorker() { Stop(); }

void DetectorWorker::Start() {
  if (started_.exchange(true)) {
    return;
  }
  stop_.store(false);
  thread_ = std::thread(&DetectorWorker::Run, this);
}

void DetectorWorker::Stop() {
  stop_.store(true);
  condition_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    busy_ = false;
    pending_ = false;
    pending_packet_ = FramePacket{};
    pending_prior_.reset();
  }
  started_.store(false);
}

bool DetectorWorker::Submit(const FramePacket& packet,
                            const std::optional<Box>& prior) {
  if (packet.frame.empty() || stop_.load() || !started_.load()) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stop_.load()) {
      return false;
    }
    ++stats_.submitted;
    if (pending_) {
      ++stats_.overwritten;
    }
    pending_packet_ = packet;
    pending_prior_ = prior;
    pending_ = true;
  }
  condition_.notify_one();
  return true;
}

bool DetectorWorker::Busy() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return busy_ || pending_;
}

InferResult DetectorWorker::LatestResult() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_result_;
}

WorkerStats DetectorWorker::Stats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

void DetectorWorker::Run() {
  std::string affinity_error;
  if (!affinity_.empty() && !BindCurrentThread(affinity_, &affinity_error)) {
    std::cerr << "detector affinity warning: " << affinity_error << "\n";
  }
  while (true) {
    FramePacket packet;
    std::optional<Box> prior;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [this] { return stop_.load() || pending_; });
      if (stop_.load()) {
        break;
      }
      pending_ = false;
      busy_ = true;
      ++stats_.started;
      packet = std::move(pending_packet_);
      prior = std::move(pending_prior_);
      pending_packet_ = FramePacket{};
      pending_prior_.reset();
    }

    const double start = NowSeconds();
    InferTimings timings;
    std::string error;
    auto detection = detector_->Infer(packet.frame, prior, &timings, &error);
    const double end = NowSeconds();
    const double inst_fps = 1.0 / std::max(1e-6, end - start);
    smoothed_infer_fps_ = SmoothValue(smoothed_infer_fps_, inst_fps, 0.15);
    if (completion_window_start_ <= 0.0) {
      completion_window_start_ = end;
      completion_window_count_ = 1;
    } else {
      ++completion_window_count_;
      const double window_seconds = end - completion_window_start_;
      if (window_seconds >= 0.25) {
        const double actual_fps =
            static_cast<double>(completion_window_count_ - 1) /
            window_seconds;
        smoothed_actual_fps_ =
            SmoothValue(smoothed_actual_fps_, actual_fps, 0.50);
        completion_window_start_ = end;
        completion_window_count_ = 1;
      }
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++sequence_;
      latest_result_.seq = sequence_;
      latest_result_.source_frame_seq = packet.seq;
      latest_result_.source_capture_ts = packet.capture_done_ts;
      latest_result_.infer_start_ts = start;
      latest_result_.infer_done_ts = end;
      latest_result_.detection = detection;
      latest_result_.infer_fps = smoothed_infer_fps_;
      latest_result_.actual_fps = smoothed_actual_fps_;
      latest_result_.timestamp = end;
      latest_result_.timings = timings;
      busy_ = false;
      ++stats_.completed;
    }

    if (!error.empty()) {
      std::cerr << "RKNN inference error: " << error << "\n";
    }
  }
}

}  // namespace rkfast
