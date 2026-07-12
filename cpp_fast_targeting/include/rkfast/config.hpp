#pragma once

#include <string>

namespace rkfast {

struct AppConfig {
  // FP16 is the working model: the INT8 export was verified (2026-07-12) to
  // produce score=0.0 on every real frame, so it must not be used as default.
  std::string model_path = "../../models/best_n_stage1_512_fp16.rknn";
  std::string camera = "/dev/video73";
  bool use_v4l2 = true;
  int width = 1280;
  int height = 720;
  int fps = 30;
  std::string fourcc = "MJPG";

  int input_size = 512;
  int target_class = 0;
  int class_count = 1;
  float conf_threshold = 0.25f;
  float iou_threshold = 0.50f;
  int max_det = 1;
  float min_box_size = 6.0f;

  int warmup_runs = 3;
  // Dual NPU core measured ~2x faster than single-core default on this board
  // (2026-07-12: frame_fps 13.7->27.5, rknn 19.7ms->16ms). No accuracy cost.
  std::string core_mask = "01";
  // Prefer native FP16 zero-copy; auto falls back to legacy when unsupported.
  std::string rknn_input_mode = "auto";
  bool verify_input_checksum = false;
  // No FPS pressure: run NPU detection every frame instead of skipping while
  // locked, so a missed/rotated rectangle is re-detected immediately.
  int detect_interval_locked = 0;
  int max_predict_misses = 3;
  float track_alpha = 0.55f;
  float track_beta = 0.20f;
  float track_box_alpha = 0.45f;

  // Velocity prediction.
  float lead_time = 0.05f;       // seconds of look-ahead for aiming
  float predict_decay = 0.9f;    // per-miss velocity decay while extrapolating

  float roi_expand_ratio = 0.24f;
  int roi_min_size = 24;
  float refine_min_area_ratio = 0.05f;
  float refine_min_score = 0.45f;  // reject weak OpenCV quads, fall back to the NPU box

  // Rectangle stability.
  float corner_alpha = 0.5f;        // EMA factor for refined corners
  float corner_reset_dist = 40.0f;  // px corner jump that resets smoothing
  float prior_iou_weight = 0.20f;   // candidate score weight for prior-box IOU
  float motion_roi_gain = 1.5f;     // max ROI expand multiplier at high speed
  float motion_speed_ref = 300.0f;  // px/s where motion gain saturates

  bool show_window = false;
  double stats_interval = 1.0;
  int opencv_threads = 1;
  std::string main_affinity;
  std::string camera_affinity;
  std::string detector_affinity;

  std::string serial_port;
  int baudrate = 115200;
  double stdout_interval = 0.20;
  double serial_interval = 0.02;
};

bool ParseCommandLine(int argc, char** argv, AppConfig* config);
void PrintUsage(const char* argv0);

}  // namespace rkfast
