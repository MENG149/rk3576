#pragma once

#include <string>

namespace rkfast {

struct AppConfig {
  // FP16 is the working model: the INT8 export was verified (2026-07-12) to
  // produce score=0.0 on every real frame, so it must not be used as default.
  std::string model_path = "../../models/best_n_stage1_512_fp16.rknn";
  std::string camera = "/dev/video73";
  bool use_v4l2 = true;
  // HF868-2 (OV2710, 2.7mm/135 deg HFOV) replaces the old 1280x720@27.5fps
  // camera. Its datasheet only hits 120fps in MJPG at 640x480; YUY2 caps at
  // 30fps at this resolution, and 720p/1080p MJPG modes cap at 30-60fps, so
  // 640x480 MJPG is the only mode that delivers the fast, low-motion-blur
  // frames the feedforward velocity estimate needs.
  int width = 640;
  int height = 480;
  int fps = 120;
  std::string fourcc = "MJPG";
  // V4L2 driver-side capture queue depth (CAP_PROP_BUFFERSIZE). Measured
  // 2026-07-17 on this board+HF868-2: buffersize=1 serializes capture
  // (driver must wait for userspace to return the single buffer before
  // grabbing the next frame) and pins grab() at ~51fps -- exactly half the
  // sensor's rate. buffersize>=2 lets the driver DMA the next frame while the
  // capture thread drains the previous one, unlocking ~99fps. >=2 saturates;
  // 3 gives a small scheduling-jitter margin. The dedicated capture thread
  // still keeps only the newest frame, so extra buffers do NOT add latency.
  int capture_buffers = 3;

  int input_size = 512;
  int target_class = 0;
  int class_count = 1;
  float conf_threshold = 0.25f;
  float iou_threshold = 0.50f;
  int max_det = 1;
  // How many post-NMS candidates to keep for prior/track association. max_det
  // bounds the FINAL emitted targets; candidate_pool bounds what the selector
  // (ChooseDetection / future track association) may choose among. With the
  // old code these were the same value, so max_det=1 meant only the single
  // highest-conf box survived NMS and prior-based selection could never switch
  // to a nearer/higher-IoU candidate under clutter. Effective pool is
  // max(max_det, candidate_pool).
  int candidate_pool = 10;
  // Halved from 6.0 for the 640x480 camera (was tuned at 1280x720; width
  // ratio 640/1280 = 0.5). Re-tune from this starting point, not from 6.0.
  float min_box_size = 3.0f;

  int warmup_runs = 3;
  // Dual NPU core measured ~2x faster than single-core default on this board
  // (2026-07-12: frame_fps 13.7->27.5, rknn 19.7ms->16ms). No accuracy cost.
  std::string core_mask = "01";
  // Prefer native FP16 zero-copy; auto falls back to legacy when unsupported.
  std::string rknn_input_mode = "auto";
  bool verify_input_checksum = false;
  // RKNN inference optimized to ~73fps capacity (~13.7ms/inference,
  // 2026-07-15), still short of the 120fps/8.3ms camera but far closer than
  // the earlier ~16-19ms. Once locked, skip 1 frame (60Hz submission) so
  // RKNN anchor corrections run at double the old 30Hz cadence while
  // staying comfortably under the 73fps ceiling (~18% headroom for
  // scheduling/preprocessing jitter) instead of oversubscribing every frame
  // and relying purely on DetectorWorker's overwrite-pending semantics.
  // ROI refine + Kalman still carry the frame-to-frame tracking and
  // feedforward velocity between anchor corrections.
  int detect_interval_locked = 1;
  int max_predict_misses = 3;
  float track_alpha = 0.55f;
  float track_beta = 0.20f;
  float track_box_alpha = 0.45f;

  // Variable-dt constant-velocity Kalman filter. Noise values are standard
  // deviations in pixel-domain units unless noted otherwise.
  // Halved from 600.0 for the 640x480 camera (was tuned at 1280x720; width
  // ratio 640/1280 = 0.5): the same real-world motion now spans fewer
  // pixels, so apparent acceleration in px/s^2 shrinks proportionally.
  float kalman_process_noise = 300.0f;  // apparent acceleration, px/s^2
  // Absolute per-measurement noise floors, not tied to resolution scale;
  // re-measure from motion-video logs on the new camera rather than scaling.
  float kalman_roi_noise = 2.0f;        // current-frame ROI center, px
  float kalman_rknn_noise = 8.0f;       // age-projected RKNN center, px
  float kalman_size_noise = 0.04f;      // log(width/height)
  // Distrust box-SIZE measurements while the target moves fast: motion blur
  // inflates the detected box, but a rigid rectangle's size only changes with
  // scale (approach/recede), not translation. 1.0 disables; higher = hold the
  // established size more firmly so the box stops growing during motion.
  float size_motion_noise_gain = 4.0f;
  // While coasting on Kalman prediction during dropped frames, shrink the box
  // by exp(-rate * seconds_coasted) so the extrapolated box hugs the target
  // instead of keeping a possibly-inflated size. 0 disables. Frame-rate safe.
  float predict_shrink_rate = 1.2f;
  float kalman_gate_threshold = 13.28f; // DEPRECATED joint 4-DoF gate; kept for
                                        // CLI back-compat, no longer used by the
                                        // decoupled gate below.
  // Decoupled innovation gate. Center [cx,cy] and size [log w,log h] are gated
  // independently so a motion-blur-inflated SIZE innovation can never veto an
  // otherwise-good CENTER measurement -- the dominant motion-time lock-loss
  // cause. Both are 2-DoF chi-square distances. 11.83 ~= 99.7% (a touch
  // generous, tolerating maneuver/acceleration the constant-velocity model
  // cannot predict); 9.21 = 99% for size.
  float kalman_center_gate = 11.83f;   // 2 DoF, center innovation
  float kalman_size_gate = 9.21f;      // 2 DoF, size innovation
  // Diverged-filter recovery: after this many CONSECUTIVE center-gate
  // rejections, the next anchor (RKNN) measurement force-re-seeds the filter
  // instead of coasting into LOST. 0 disables (never force-reseed).
  int gate_reset_rejections = 6;
  // Halved from 4000.0 for the 640x480 camera (width ratio 640/1280 = 0.5).
  float max_track_speed = 2000.0f;      // px/s safety clamp
  double acquire_timeout = 0.10;        // seconds
  double prediction_timeout = 0.10;     // seconds since accepted measurement
  double homography_timeout = 0.05;     // seconds since reliable corners
  double confidence_decay_tau = 0.15;   // seconds

  // Velocity prediction.
  float lead_time = 0.05f;       // seconds of look-ahead for aiming
  float predict_decay = 0.9f;    // per-miss velocity decay while extrapolating

  float roi_expand_ratio = 0.24f;
  // Halved from 24 for the 640x480 camera (width ratio 640/1280 = 0.5).
  int roi_min_size = 12;
  float refine_min_area_ratio = 0.05f;
  float refine_min_score = 0.45f;  // reject weak OpenCV quads, fall back to the NPU box

  // Rectangle stability.
  float corner_alpha = 0.5f;        // EMA factor for refined corners
  // Halved from 40.0 for the 640x480 camera (width ratio 640/1280 = 0.5).
  float corner_reset_dist = 20.0f;  // px corner jump that resets smoothing
  float prior_iou_weight = 0.20f;   // candidate score weight for prior-box IOU
  float motion_roi_gain = 1.5f;     // max ROI expand multiplier at high speed
  // Halved from 300.0 for the 640x480 camera (width ratio 640/1280 = 0.5).
  float motion_speed_ref = 150.0f;  // px/s where motion gain saturates

  float target_width_cm = 21.0f;
  float target_height_cm = 29.7f;

  // Red bullseye (concentric rings + center dot on the real target). When
  // enabled, the refiner looks for red pixels INSIDE the refined black frame
  // and returns their centroid as the physical target center; the aim loop
  // prefers it over the geometric frame center. Frame-rate safe: the red scan
  // runs only inside the already-cropped refine ROI. On the plain black-frame
  // dataset no red is found and behaviour is unchanged.
  bool aim_red_center = true;
  // HSV gates for red. Hue wraps, so red is [0,red_hue_lo] U [red_hue_hi,179].
  int red_hue_lo = 10;       // upper bound of the low-hue red band
  int red_hue_hi = 160;      // lower bound of the high-hue red band
  int red_sat_min = 80;      // reject pale/desaturated pixels (white paper)
  int red_val_min = 60;      // reject dark pixels (the black frame itself)
  // Minimum red pixel count inside the quad to trust a bullseye detection.
  int red_min_pixels = 12;
  // EMA factor for the smoothed red aim point (higher = snappier, less stable).
  float red_center_alpha = 0.5f;
  // Drop the smoothed red center if not re-seen within this window (seconds).
  double red_center_timeout = 0.12;

  bool show_window = false;
  // Throttles cv::imshow redraws (clone + drawing + Qt-backend paint), not the
  // vision/tracking/serial pipeline. Measured on this board: with show_window
  // on, redrawing every camera frame added ~3-6ms/frame of Qt5 CPU-rendering
  // overhead (this OpenCV build has no GPU-accelerated GUI backend) and made
  // frame_fps drop from the ~99fps no-window ceiling to a jittery 60-80fps.
  // 30Hz is smooth to the eye and frees the rest of the frame budget for
  // tracking + serial output, which still run at full camera rate.
  double window_interval = 1.0 / 30.0;
  // Remote MJPEG debug stream. Enabled when mjpeg_port > 0: open
  // http://<board-ip>:<port>/ from the host to watch the annotated feed.
  int mjpeg_port = 0;
  int mjpeg_quality = 70;
  double stats_interval = 1.0;
  int opencv_threads = 1;
  std::string main_affinity;
  std::string camera_affinity;
  std::string detector_affinity;

  std::string serial_port;
  int baudrate = 115200;
  double stdout_interval = 0.20;
  double serial_interval = 0.02;

  // Serial output framing. The erweiyuntai (STM32F1) gimbal expects a fixed
  // 12-byte binary packet, not the ASCII $AIM line:
  //   AA 08 xHi xLo yHi yLo vxHi vxLo vyHi vyLo checksum 55
  //   checksum = (sum of bytes[1..9]) & 0xFF ; coords/velocity big-endian.
  // The receiver's frame is packet_frame_width x packet_frame_height and it
  // rejects any coord >= those bounds, so the capture-resolution center and
  // velocity are scaled into that space before packing. stdout stays ASCII.
  bool serial_packet = true;
  // Must equal FRAME_WIDTH/FRAME_HEIGHT in erweiyuntai's version.h, which
  // is now 640x480 to match this project's native camera resolution (no
  // more 800x480 K230 legacy scaling -- see BuildPacket). If width/height
  // above ever changes, update both sides together.
  int packet_frame_width = 640;
  int packet_frame_height = 480;

  // Use the perspective-corrected target-plane center (homography projection of
  // the target origin) as the aim point instead of the axis-aligned bbox
  // center. Under an oblique view the AABB center is NOT the physical target
  // center, so this removes a systematic accuracy bias. Falls back to the bbox
  // center automatically whenever the homography is invalid or the projected
  // point is non-finite / outside the frame, so it is safe to leave on.
  // Toggle with --aim-projective-center / --aim-box-center for A/B replay.
  bool aim_use_projective_center = true;
};

bool ParseCommandLine(int argc, char** argv, AppConfig* config);
void PrintUsage(const char* argv0);

}  // namespace rkfast
