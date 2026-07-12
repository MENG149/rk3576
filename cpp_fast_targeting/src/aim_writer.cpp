#include "rkfast/aim_writer.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
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

void AimWriter::Send(const TrackSnapshot& snapshot, int frame_w, int frame_h) {
  const std::string payload = FormatAim(snapshot, frame_w, frame_h);
  const double now = NowSeconds();
  if (fd_ >= 0) {
    if (config_.serial_interval > 0.0 &&
        now - last_serial_ < config_.serial_interval) {
      return;
    }
    const std::string line = payload + "\n";
    (void)!write(fd_, line.data(), line.size());
    last_serial_ = now;
    return;
  }

  if (now - last_stdout_ >= config_.stdout_interval) {
    std::cout << payload << std::endl;
    last_stdout_ = now;
  }
}

std::string AimWriter::FormatAim(const TrackSnapshot& snapshot, int frame_w,
                                 int frame_h) {
  const int found = snapshot.active ? 1 : 0;
  const float cx = snapshot.active ? snapshot.center.x : 0.0f;
  const float cy = snapshot.active ? snapshot.center.y : 0.0f;
  const cv::Point2f off =
      snapshot.active ? CenterOffset(snapshot.center, frame_w, frame_h)
                      : cv::Point2f(0.0f, 0.0f);
  const cv::Point2f norm =
      snapshot.active ? NormalizedOffset(snapshot.center, frame_w, frame_h)
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
