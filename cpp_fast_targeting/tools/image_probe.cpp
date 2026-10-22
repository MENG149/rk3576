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
  std::optional<Box> forced_box;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&](int& i) { return std::string(argv[++i]); };
    if (arg == "--model") config.model_path = next(i);
    else if (arg == "--image") image_path = next(i);
    else if (arg == "--input-size") config.input_size = std::stoi(next(i));
    else if (arg == "--class-count") config.class_count = std::stoi(next(i));
    else if (arg == "--target-class") config.target_class = std::stoi(next(i));
    else if (arg == "--conf") config.conf_threshold = std::stof(next(i));
    else if (arg == "--box") {
      // x1,y1,x2,y2 -- skip RKNN, drive the refiner directly. Lets us validate
      // the OpenCV refine + red-center path independent of the model.
      std::string v = next(i);
      float b[4]; int n = 0; size_t pos = 0;
      while (n < 4 && pos < v.size()) {
        size_t comma = v.find(',', pos);
        b[n++] = std::stof(v.substr(pos, comma - pos));
        if (comma == std::string::npos) break;
        pos = comma + 1;
      }
      if (n == 4) forced_box = rkfast::Box{b[0], b[1], b[2], b[3]};
    }
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

  Box guide_box;
  std::string error;
  if (forced_box.has_value()) {
    guide_box = *forced_box;
    std::cout << "guide box (forced): (" << guide_box.x1 << "," << guide_box.y1
              << "," << guide_box.x2 << "," << guide_box.y2 << ")\n";
  } else {
    RknnDetector detector(config);
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
    guide_box = detection->box;
    std::cout << "RKNN detection: conf=" << detection->conf
              << " box=(" << guide_box.x1 << "," << guide_box.y1 << ","
              << guide_box.x2 << "," << guide_box.y2 << ")"
              << " center=(" << detection->center.x << "," << detection->center.y << ")\n";
  }

  RectRefiner refiner(config);
  auto refined = refiner.Refine(frame, guide_box);
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
  if (refined->red_center.has_value()) {
    std::cout << "  RED bullseye: center=(" << refined->red_center->x << ","
              << refined->red_center->y << ") pixels=" << refined->red_pixels
              << "\n";
  } else {
    std::cout << "  RED bullseye: none (frame-center aim)\n";
  }
  return 0;
}
