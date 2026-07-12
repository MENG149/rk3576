#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "rkfast/config.hpp"
#include "rkfast/rknn_detector.hpp"
#include "rkfast/types.hpp"

namespace {

struct Options {
  rkfast::AppConfig config;
  std::string images;
  int limit = 0;
};

bool NeedValue(int argc, char** argv, int* index, std::string* value) {
  if (*index + 1 >= argc) {
    return false;
  }
  *value = argv[++(*index)];
  return true;
}

bool ParseOptions(int argc, char** argv, Options* options) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    std::string value;
    if (arg == "--model" && NeedValue(argc, argv, &i, &value)) {
      options->config.model_path = value;
    } else if (arg == "--images" && NeedValue(argc, argv, &i, &value)) {
      options->images = value;
    } else if (arg == "--limit" && NeedValue(argc, argv, &i, &value)) {
      options->limit = std::stoi(value);
    } else if (arg == "--input-size" && NeedValue(argc, argv, &i, &value)) {
      options->config.input_size = std::stoi(value);
    } else if (arg == "--class-count" && NeedValue(argc, argv, &i, &value)) {
      options->config.class_count = std::stoi(value);
    } else if (arg == "--target-class" &&
               NeedValue(argc, argv, &i, &value)) {
      options->config.target_class = std::stoi(value);
    } else if (arg == "--conf" && NeedValue(argc, argv, &i, &value)) {
      options->config.conf_threshold = std::stof(value);
    } else if (arg == "--core-mask" && NeedValue(argc, argv, &i, &value)) {
      options->config.core_mask = value;
    } else {
      std::cerr << "unknown or incomplete option: " << arg << "\n";
      return false;
    }
  }
  return !options->images.empty();
}

std::string Hex(uint64_t value) {
  std::ostringstream out;
  out << std::hex << std::setw(16) << std::setfill('0') << value;
  return out.str();
}

std::string DetectionJson(const std::optional<rkfast::Detection>& detection) {
  if (!detection.has_value()) {
    return "null";
  }
  std::ostringstream out;
  out << std::setprecision(9)
      << "{class:" << detection->class_id
      << ",conf:" << detection->conf
      << ",box:[" << detection->box.x1 << "," << detection->box.y1
      << "," << detection->box.x2 << "," << detection->box.y2 << "]}";
  return out.str();
}

std::vector<std::filesystem::path> CollectImages(const std::string& directory) {
  std::vector<std::filesystem::path> images;
  std::error_code error;
  for (const auto& entry :
       std::filesystem::directory_iterator(directory, error)) {
    if (error || !entry.is_regular_file()) {
      continue;
    }
    std::string ext = entry.path().extension().string();
    std::transform(
        ext.begin(), ext.end(), ext.begin(),
        [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    if (ext == ".jpg" || ext == ".jpeg" || ext == ".png") {
      images.push_back(entry.path());
    }
  }
  std::sort(images.begin(), images.end());
  return images;
}

std::string CompareResult(const std::optional<rkfast::Detection>& legacy,
                          const rkfast::InferTimings& legacy_timings,
                          const std::optional<rkfast::Detection>& zero_copy,
                          const rkfast::InferTimings& zero_timings) {
  if (legacy_timings.input_bytes != zero_timings.input_bytes) {
    return "input byte count differs";
  }
  if (legacy_timings.input_checksum != zero_timings.input_checksum) {
    return "input checksum differs";
  }
  if (legacy.has_value() != zero_copy.has_value()) {
    return "detection presence differs";
  }
  if (!legacy.has_value()) {
    return {};
  }
  if (legacy->class_id != zero_copy->class_id) {
    return "class id differs";
  }
  if (std::fabs(legacy->conf - zero_copy->conf) > 1e-4f) {
    return "confidence differs";
  }
  const float legacy_box[] = {legacy->box.x1, legacy->box.y1,
                              legacy->box.x2, legacy->box.y2};
  const float zero_box[] = {zero_copy->box.x1, zero_copy->box.y1,
                            zero_copy->box.x2, zero_copy->box.y2};
  for (int i = 0; i < 4; ++i) {
    if (std::fabs(legacy_box[i] - zero_box[i]) > 0.1f) {
      return "box coordinate differs";
    }
  }
  return {};
}

}  // namespace

int main(int argc, char** argv) {
  using namespace rkfast;

  Options options;
  if (!ParseOptions(argc, argv, &options)) {
    std::cerr << "usage: input_mode_compare --model PATH --images DIR "
                 "[--limit N] [--conf F] [--core-mask 01]\n";
    return 1;
  }

  auto images = CollectImages(options.images);
  if (images.empty()) {
    std::cerr << "no images found in: " << options.images << "\n";
    return 1;
  }
  if (options.limit > 0 &&
      static_cast<size_t>(options.limit) < images.size()) {
    images.resize(static_cast<size_t>(options.limit));
  }

  AppConfig legacy_config = options.config;
  legacy_config.rknn_input_mode = "legacy";
  legacy_config.verify_input_checksum = true;
  AppConfig zero_config = options.config;
  zero_config.rknn_input_mode = "zero-copy";
  zero_config.verify_input_checksum = true;

  RknnDetector legacy(legacy_config);
  RknnDetector zero_copy(zero_config);
  std::string error;
  if (!legacy.Open(&error)) {
    std::cerr << "legacy open failed: " << error << "\n";
    return 1;
  }
  error.clear();
  if (!zero_copy.Open(&error)) {
    std::cerr << "zero-copy open failed: " << error << "\n";
    return 1;
  }
  if (!legacy.Warmup(1, &error)) {
    std::cerr << "legacy warmup failed: " << error << "\n";
    return 1;
  }
  error.clear();
  if (!zero_copy.Warmup(1, &error)) {
    std::cerr << "zero-copy warmup failed: " << error << "\n";
    return 1;
  }

  int compared = 0;
  int mismatches = 0;
  for (const auto& image_path : images) {
    const cv::Mat frame = cv::imread(image_path.string());
    if (frame.empty()) {
      std::cerr << "could not read image: " << image_path.string() << "\n";
      ++mismatches;
      continue;
    }

    InferTimings legacy_timings;
    InferTimings zero_timings;
    error.clear();
    const auto legacy_detection =
        legacy.Infer(frame, std::nullopt, &legacy_timings, &error);
    const std::string legacy_error = error;
    error.clear();
    const auto zero_detection =
        zero_copy.Infer(frame, std::nullopt, &zero_timings, &error);
    const std::string zero_error = error;

    std::string reason;
    if (!legacy_error.empty() || !zero_error.empty()) {
      reason = "inference error";
    } else {
      reason = CompareResult(legacy_detection, legacy_timings,
                             zero_detection, zero_timings);
    }
    const bool match = reason.empty();
    ++compared;
    if (!match) {
      ++mismatches;
    }
    std::cout << "RESULT\t" << image_path.filename().string()
              << "\t" << (match ? "match" : "mismatch")
              << "\t" << reason
              << "\t" << legacy_timings.input_bytes
              << "\t" << Hex(legacy_timings.input_checksum)
              << "\t" << Hex(zero_timings.input_checksum)
              << "\t" << DetectionJson(legacy_detection)
              << "\t" << DetectionJson(zero_detection) << "\n";
  }

  std::cout << "SUMMARY\timages=" << compared
            << "\tmismatches=" << mismatches << "\n";
  return mismatches == 0 && compared == static_cast<int>(images.size()) ? 0 : 2;
}
