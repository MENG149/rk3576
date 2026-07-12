// Standalone tool: run the real RknnDetector + RectRefiner pipeline against a
// single static image and print the result. Verifies the production code
// path end to end without needing a live camera or a moving target.
//
// Usage: image_probe --model PATH --image PATH [--input-size N]
//                     [--class-count N] [--target-class N] [--conf F]
#include <iostream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "rkfast/config.hpp"
#include "rkfast/rect_refiner.hpp"
#include "rkfast/rknn_detector.hpp"
#include "rkfast/types.hpp"

int main(int argc, char** argv) {
  using namespace rkfast;

  AppConfig config;
  std::string image_path;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&](int& i) { return std::string(argv[++i]); };
    if (arg == "--model") config.model_path = next(i);
    else if (arg == "--image") image_path = next(i);
    else if (arg == "--input-size") config.input_size = std::stoi(next(i));
    else if (arg == "--class-count") config.class_count = std::stoi(next(i));
    else if (arg == "--target-class") config.target_class = std::stoi(next(i));
    else if (arg == "--conf") config.conf_threshold = std::stof(next(i));
  }
  if (image_path.empty()) {
    std::cerr << "usage: image_probe --model PATH --image PATH\n";
    return 1;
  }

  cv::Mat frame = cv::imread(image_path);
  if (frame.empty()) {
    std::cerr << "could not read image: " << image_path << "\n";
    return 1;
  }
  std::cout << "image: " << image_path << " (" << frame.cols << "x" << frame.rows << ")\n";

  RknnDetector detector(config);
  std::string error;
  if (!detector.Open(&error)) {
    std::cerr << "detector open failed: " << error << "\n";
    return 1;
  }

  InferTimings timings;
  auto detection = detector.Infer(frame, std::nullopt, &timings, &error);
  if (!error.empty()) {
    std::cerr << "infer error: " << error << "\n";
  }
  if (!detection.has_value()) {
    std::cout << "RKNN: no detection above conf=" << config.conf_threshold << "\n";
    return 0;
  }
  std::cout << "RKNN detection: conf=" << detection->conf
            << " box=(" << detection->box.x1 << "," << detection->box.y1 << ","
            << detection->box.x2 << "," << detection->box.y2 << ")"
            << " center=(" << detection->center.x << "," << detection->center.y << ")\n";

  RectRefiner refiner(config);
  auto refined = refiner.Refine(frame, detection->box);
  if (!refined.has_value()) {
    std::cout << "OpenCV refine: no quad accepted (falls back to RKNN box)\n";
    return 0;
  }
  std::cout << "OpenCV refine: score=" << refined->score
            << " center=(" << refined->center.x << "," << refined->center.y << ")\n";
  for (int i = 0; i < 4; ++i) {
    std::cout << "  corner[" << i << "] = (" << refined->corners[i].x << ","
              << refined->corners[i].y << ")\n";
  }
  return 0;
}
