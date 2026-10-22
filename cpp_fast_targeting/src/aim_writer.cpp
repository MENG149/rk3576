#include "rkfast/aim_writer.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <sstream>

#include <opencv2/core.hpp>

#include "rkfast/types.hpp"

namespace rkfast {

namespace {

speed_t ToBaud(int baudrate) {
  switch (baudrate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
    default:
      return B115200;
  }
}

}  // namespace

AimWriter::AimWriter(const AppConfig& config) : config_(config) {}

AimWriter::~AimWriter() { Close(); }

bool AimWriter::Open(std::string* error) {
  if (config_.serial_port.empty()) {
    return true;
  }
  fd_ = open(config_.serial_port.c_str(), O_WRONLY | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    if (error != nullptr) {
      *error = "open serial failed: " + config_.serial_port + " : " +
               std::strerror(errno);
    }
    return false;
  }
  if (!ConfigureSerial(fd_, error)) {
    fd_ = -1;
    return false;
  }
  return true;
}

void AimWriter::Close() {
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
}

void AimWriter::Send(const TrackSnapshot& snapshot, int frame_w, int frame_h,
                     const std::optional<cv::Point2f>& aim_center) {
  // Only override with the projected center when it is finite; otherwise fall
  // back to the tracker's bbox center so a bad homography can never inject NaN.
  cv::Point2f center = snapshot.center;
  if (aim_center.has_value() && std::isfinite(aim_center->x) &&
      std::isfinite(aim_center->y)) {
    center = *aim_center;
  }

  const double now = NowSeconds();
  if (fd_ >= 0) {
    if (config_.serial_interval > 0.0 &&
        now - last_serial_ < config_.serial_interval) {
      return;
    }
    if (config_.serial_packet) {
      uint8_t packet[12];
      BuildPacket(snapshot, center, frame_w, frame_h, packet);
      (void)!write(fd_, packet, sizeof(packet));
    } else {
      const std::string line =
          FormatAim(snapshot, center, frame_w, frame_h) + "\n";
      (void)!write(fd_, line.data(), line.size());
    }
    last_serial_ = now;
    return;
  }

  if (now - last_stdout_ >= config_.stdout_interval) {
    std::cout << FormatAim(snapshot, center, frame_w, frame_h) << std::endl;
    last_stdout_ = now;
  }
}

void AimWriter::BuildPacket(const TrackSnapshot& snapshot,
                            const cv::Point2f& aim_center, int frame_w,
                            int frame_h, uint8_t out[12]) const {
  const int rx_w = std::max(2, config_.packet_frame_width);
  const int rx_h = std::max(2, config_.packet_frame_height);

  uint16_t x = 0;
  uint16_t y = 0;
  int16_t vx = 0;
  int16_t vy = 0;
  if (snapshot.active && frame_w > 0 && frame_h > 0) {
    const float sx = static_cast<float>(rx_w) / static_cast<float>(frame_w);
    const float sy = static_cast<float>(rx_h) / static_cast<float>(frame_h);
    // Clamp to [1, rx-2] so a valid target never reads as 0 (stop) or as the
    // receiver's out-of-bounds reject threshold (>= rx_w / rx_h).
    const auto clampi = [](float v, int lo, int hi) {
      const int iv = static_cast<int>(std::lround(v));
      return static_cast<uint16_t>(iv < lo ? lo : (iv > hi ? hi : iv));
    };
    const auto clamp16 = [](float v) {
      const long iv = std::lround(v);
      const long c = iv < -32768 ? -32768 : (iv > 32767 ? 32767 : iv);
      return static_cast<int16_t>(c);
    };
    x = clampi(aim_center.x * sx, 1, rx_w - 2);
    y = clampi(aim_center.y * sy, 1, rx_h - 2);
    vx = clamp16(snapshot.vx * sx);
    vy = clamp16(snapshot.vy * sy);
  }

  out[0] = 0xAAu;
  out[1] = 0x08u;
  out[2] = static_cast<uint8_t>(x >> 8);
  out[3] = static_cast<uint8_t>(x & 0xFFu);
  out[4] = static_cast<uint8_t>(y >> 8);
  out[5] = static_cast<uint8_t>(y & 0xFFu);
  out[6] = static_cast<uint8_t>(static_cast<uint16_t>(vx) >> 8);
  out[7] = static_cast<uint8_t>(static_cast<uint16_t>(vx) & 0xFFu);
  out[8] = static_cast<uint8_t>(static_cast<uint16_t>(vy) >> 8);
  out[9] = static_cast<uint8_t>(static_cast<uint16_t>(vy) & 0xFFu);
  uint8_t sum = 0u;
  for (int i = 1; i <= 9; ++i) {
    sum = static_cast<uint8_t>(sum + out[i]);
  }
  out[10] = sum;
  out[11] = 0x55u;
}

std::string AimWriter::FormatAim(const TrackSnapshot& snapshot,
                                 const cv::Point2f& aim_center, int frame_w,
                                 int frame_h) {
  const int found = snapshot.active ? 1 : 0;
  const float cx = snapshot.active ? aim_center.x : 0.0f;
  const float cy = snapshot.active ? aim_center.y : 0.0f;
  const cv::Point2f off =
      snapshot.active ? CenterOffset(aim_center, frame_w, frame_h)
                      : cv::Point2f(0.0f, 0.0f);
  const cv::Point2f norm =
      snapshot.active ? NormalizedOffset(aim_center, frame_w, frame_h)
                      : cv::Point2f(0.0f, 0.0f);
  const float px = snapshot.active ? snapshot.predict_center.x : 0.0f;
  const float py = snapshot.active ? snapshot.predict_center.y : 0.0f;
  const cv::Point2f pnorm =
      snapshot.active ? NormalizedOffset(snapshot.predict_center, frame_w, frame_h)
                      : cv::Point2f(0.0f, 0.0f);
  const float vx = snapshot.active ? snapshot.vx : 0.0f;
  const float vy = snapshot.active ? snapshot.vy : 0.0f;
  char buffer[320];
  std::snprintf(buffer, sizeof(buffer),
                "$AIM,%d,%s,%.1f,%.1f,%.1f,%.1f,%.3f,%.3f,%.1f,%.1f,%.3f,%.3f,"
                "%.1f,%.1f,%.3f,%s",
                found, snapshot.state.c_str(), cx, cy, off.x, off.y, norm.x,
                norm.y, px, py, pnorm.x, pnorm.y, vx, vy, snapshot.conf,
                snapshot.source.c_str());
  return std::string(buffer);
}

bool AimWriter::ConfigureSerial(int fd, std::string* error) const {
  termios tty{};
  if (tcgetattr(fd, &tty) != 0) {
    if (error != nullptr) {
      *error = "tcgetattr failed: " + std::string(std::strerror(errno));
    }
    close(fd);
    return false;
  }

  cfsetospeed(&tty, ToBaud(config_.baudrate));
  cfsetispeed(&tty, ToBaud(config_.baudrate));
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag &= ~IGNBRK;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    if (error != nullptr) {
      *error = "tcsetattr failed: " + std::string(std::strerror(errno));
    }
    close(fd);
    return false;
  }
  return true;
}

}  // namespace rkfast
