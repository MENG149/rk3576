#include "rkfast/config.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

namespace rkfast {

namespace {

bool NeedValue(int argc, char** argv, int* index, std::string* value) {
  if (*index + 1 >= argc) {
    std::cerr << "Missing value for " << argv[*index] << "\n";
    return false;
  }
  *value = argv[++(*index)];
  return true;
}

}  // namespace

void PrintUsage(const char* argv0) {
  std::cout
      << "Usage: " << argv0 << " [options]\n"
      << "  --model PATH\n"
      << "  --camera DEVICE_OR_INDEX\n"
      << "  --width N --height N --fps N --fourcc MJPG\n"
      << "  --input-size N --class-count N --target-class N\n"
      << "  --conf F --iou F --max-det N --min-box-size F\n"
      << "  --core-mask default|auto|0|1|01|all\n"
      << "  --rknn-input-mode auto|legacy|zero-copy\n"
      << "  --detect-interval-locked N --max-predict-misses N\n"
      << "  --lead-time SEC --predict-decay F\n"
      << "  --corner-alpha F --corner-reset-dist PX --prior-iou-weight F\n"
      << "  --motion-roi-gain F --motion-speed-ref PX_PER_SEC\n"
      << "  --main-affinity LIST --camera-affinity LIST --detector-affinity LIST\n"
      << "  --serial-port PATH --baudrate N\n"
      << "  --window | --no-window\n";
}

bool ParseCommandLine(int argc, char** argv, AppConfig* config) {
  if (config == nullptr) {
    return false;
  }
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    std::string value;
    if (arg == "--help" || arg == "-h") {
      PrintUsage(argv[0]);
      return false;
    }
    if (arg == "--model") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->model_path = value;
    } else if (arg == "--camera") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->camera = value;
    } else if (arg == "--camera-api") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->use_v4l2 = (value == "v4l2");
    } else if (arg == "--width") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->width = std::stoi(value);
    } else if (arg == "--height") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->height = std::stoi(value);
    } else if (arg == "--fps") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->fps = std::stoi(value);
    } else if (arg == "--fourcc") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->fourcc = value;
    } else if (arg == "--input-size") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->input_size = std::stoi(value);
    } else if (arg == "--target-class") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->target_class = std::stoi(value);
    } else if (arg == "--class-count") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->class_count = std::stoi(value);
    } else if (arg == "--conf") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->conf_threshold = std::stof(value);
    } else if (arg == "--iou") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->iou_threshold = std::stof(value);
    } else if (arg == "--max-det") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->max_det = std::stoi(value);
    } else if (arg == "--min-box-size") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->min_box_size = std::stof(value);
    } else if (arg == "--warmup") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->warmup_runs = std::stoi(value);
    } else if (arg == "--core-mask") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      if (value != "default" && value != "auto" && value != "0" &&
          value != "1" && value != "01" && value != "all") {
        std::cerr << "Invalid --core-mask: " << value << "\n";
        PrintUsage(argv[0]);
        return false;
      }
      config->core_mask = value;
    } else if (arg == "--rknn-input-mode") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      if (value != "auto" && value != "legacy" && value != "zero-copy") {
        std::cerr << "Invalid --rknn-input-mode: " << value << "\n";
        PrintUsage(argv[0]);
        return false;
      }
      config->rknn_input_mode = value;
    } else if (arg == "--detect-interval-locked") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->detect_interval_locked = std::stoi(value);
    } else if (arg == "--max-predict-misses") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->max_predict_misses = std::stoi(value);
    } else if (arg == "--track-alpha") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->track_alpha = std::stof(value);
    } else if (arg == "--track-beta") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->track_beta = std::stof(value);
    } else if (arg == "--track-box-alpha") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->track_box_alpha = std::stof(value);
    } else if (arg == "--roi-expand-ratio") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->roi_expand_ratio = std::stof(value);
    } else if (arg == "--roi-min-size") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->roi_min_size = std::stoi(value);
    } else if (arg == "--refine-min-area-ratio") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->refine_min_area_ratio = std::stof(value);
    } else if (arg == "--refine-min-score") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->refine_min_score = std::stof(value);
    } else if (arg == "--lead-time") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->lead_time = std::stof(value);
    } else if (arg == "--predict-decay") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->predict_decay = std::stof(value);
    } else if (arg == "--corner-alpha") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->corner_alpha = std::stof(value);
    } else if (arg == "--corner-reset-dist") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->corner_reset_dist = std::stof(value);
    } else if (arg == "--prior-iou-weight") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->prior_iou_weight = std::stof(value);
    } else if (arg == "--motion-roi-gain") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->motion_roi_gain = std::stof(value);
    } else if (arg == "--motion-speed-ref") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->motion_speed_ref = std::stof(value);
    } else if (arg == "--serial-port") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->serial_port = value;
    } else if (arg == "--baudrate") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->baudrate = std::stoi(value);
    } else if (arg == "--stdout-interval") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->stdout_interval = std::stod(value);
    } else if (arg == "--serial-interval") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->serial_interval = std::stod(value);
    } else if (arg == "--stats-interval") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->stats_interval = std::stod(value);
    } else if (arg == "--opencv-threads") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->opencv_threads = std::stoi(value);
    } else if (arg == "--main-affinity") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->main_affinity = value;
    } else if (arg == "--camera-affinity") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->camera_affinity = value;
    } else if (arg == "--detector-affinity") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->detector_affinity = value;
    } else if (arg == "--window") {
      config->show_window = true;
    } else if (arg == "--no-window") {
      config->show_window = false;
    } else {
      std::cerr << "Unknown option: " << arg << "\n";
      PrintUsage(argv[0]);
      return false;
    }
  }
  return true;
}

}  // namespace rkfast
