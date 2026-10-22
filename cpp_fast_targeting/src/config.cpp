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
      << "  --capture-buffers N (V4L2 queue depth; >=2 unlocks full fps)\n"
      << "  --input-size N --class-count N --target-class N\n"
      << "  --conf F --iou F --max-det N --candidate-pool N --min-box-size F\n"
      << "  --core-mask default|auto|0|1|01|all\n"
      << "  --rknn-input-mode auto|legacy|zero-copy\n"
      << "  --detect-interval-locked N --max-predict-misses N\n"
      << "  --lead-time SEC --predict-decay F\n"
      << "  --kalman-process-noise F --kalman-roi-noise F --kalman-rknn-noise F\n"
      << "  --kalman-size-noise F --size-motion-noise-gain F --predict-shrink-rate F\n"
      << "  --kalman-gate F (deprecated) --kalman-center-gate F --kalman-size-gate F\n"
      << "  --gate-reset-rejections N --max-track-speed PX_PER_SEC\n"
      << "  --acquire-timeout SEC --prediction-timeout SEC --homography-timeout SEC\n"
      << "  --target-width-cm F --target-height-cm F\n"
      << "  --aim-red-center | --aim-frame-center\n"
      << "  --red-hue-lo N --red-hue-hi N --red-sat-min N --red-val-min N\n"
      << "  --red-min-pixels N --red-center-alpha F --red-center-timeout SEC\n"
      << "  --corner-alpha F --corner-reset-dist PX --prior-iou-weight F\n"
      << "  --motion-roi-gain F --motion-speed-ref PX_PER_SEC\n"
      << "  --main-affinity LIST --camera-affinity LIST --detector-affinity LIST\n"
      << "  --serial-port PATH --baudrate N\n"
      << "  --serial-packet | --serial-ascii\n"
      << "  --aim-projective-center | --aim-box-center\n"
      << "  --packet-frame-width N --packet-frame-height N\n"
      << "  --window | --no-window\n"
      << "  --window-interval SEC (redraw throttle, default 1/30s)\n"
      << "  --mjpeg-port N (0=off) --mjpeg-quality 1-100\n";
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
    } else if (arg == "--capture-buffers") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->capture_buffers = std::stoi(value);
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
    } else if (arg == "--candidate-pool") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->candidate_pool = std::stoi(value);
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
    } else if (arg == "--kalman-process-noise") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->kalman_process_noise = std::stof(value);
    } else if (arg == "--kalman-roi-noise") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->kalman_roi_noise = std::stof(value);
    } else if (arg == "--kalman-rknn-noise") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->kalman_rknn_noise = std::stof(value);
    } else if (arg == "--kalman-size-noise") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->kalman_size_noise = std::stof(value);
    } else if (arg == "--size-motion-noise-gain") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->size_motion_noise_gain = std::stof(value);
    } else if (arg == "--predict-shrink-rate") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->predict_shrink_rate = std::stof(value);
    } else if (arg == "--kalman-gate") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->kalman_gate_threshold = std::stof(value);
    } else if (arg == "--kalman-center-gate") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->kalman_center_gate = std::stof(value);
    } else if (arg == "--kalman-size-gate") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->kalman_size_gate = std::stof(value);
    } else if (arg == "--gate-reset-rejections") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->gate_reset_rejections = std::stoi(value);
    } else if (arg == "--max-track-speed") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->max_track_speed = std::stof(value);
    } else if (arg == "--acquire-timeout") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->acquire_timeout = std::stod(value);
    } else if (arg == "--prediction-timeout") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->prediction_timeout = std::stod(value);
    } else if (arg == "--homography-timeout") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->homography_timeout = std::stod(value);
    } else if (arg == "--confidence-decay-tau") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->confidence_decay_tau = std::stod(value);
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
    } else if (arg == "--target-width-cm") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->target_width_cm = std::stof(value);
    } else if (arg == "--target-height-cm") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->target_height_cm = std::stof(value);
    } else if (arg == "--aim-red-center") {
      config->aim_red_center = true;
    } else if (arg == "--aim-frame-center") {
      config->aim_red_center = false;
    } else if (arg == "--red-hue-lo") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->red_hue_lo = std::stoi(value);
    } else if (arg == "--red-hue-hi") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->red_hue_hi = std::stoi(value);
    } else if (arg == "--red-sat-min") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->red_sat_min = std::stoi(value);
    } else if (arg == "--red-val-min") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->red_val_min = std::stoi(value);
    } else if (arg == "--red-min-pixels") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->red_min_pixels = std::stoi(value);
    } else if (arg == "--red-center-alpha") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->red_center_alpha = std::stof(value);
    } else if (arg == "--red-center-timeout") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->red_center_timeout = std::stod(value);
    } else if (arg == "--serial-port") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->serial_port = value;
    } else if (arg == "--serial-packet") {
      config->serial_packet = true;
    } else if (arg == "--serial-ascii") {
      config->serial_packet = false;
    } else if (arg == "--aim-projective-center") {
      config->aim_use_projective_center = true;
    } else if (arg == "--aim-box-center") {
      config->aim_use_projective_center = false;
    } else if (arg == "--packet-frame-width") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->packet_frame_width = std::stoi(value);
    } else if (arg == "--packet-frame-height") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->packet_frame_height = std::stoi(value);
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
    } else if (arg == "--window-interval") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->window_interval = std::stod(value);
    } else if (arg == "--mjpeg-port") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->mjpeg_port = std::stoi(value);
    } else if (arg == "--mjpeg-quality") {
      if (!NeedValue(argc, argv, &i, &value)) return false;
      config->mjpeg_quality = std::stoi(value);
    } else {
      std::cerr << "Unknown option: " << arg << "\n";
      PrintUsage(argv[0]);
      return false;
    }
  }
  if (config->kalman_process_noise <= 0.0f || config->kalman_roi_noise <= 0.0f ||
      config->kalman_rknn_noise <= 0.0f || config->kalman_size_noise <= 0.0f ||
      config->kalman_gate_threshold <= 0.0f ||
      config->kalman_center_gate <= 0.0f || config->kalman_size_gate <= 0.0f ||
      config->max_track_speed <= 0.0f ||
      config->acquire_timeout <= 0.0 || config->prediction_timeout <= 0.0 ||
      config->homography_timeout <= 0.0 || config->confidence_decay_tau <= 0.0 ||
      config->target_width_cm <= 0.0f ||
      config->target_height_cm <= 0.0f) {
    std::cerr << "Kalman, timeout, speed, and target-size options must be positive\n";
    return false;
  }
  if (config->capture_buffers < 1) {
    std::cerr << "--capture-buffers must be >= 1\n";
    return false;
  }
  return true;
}

}  // namespace rkfast
