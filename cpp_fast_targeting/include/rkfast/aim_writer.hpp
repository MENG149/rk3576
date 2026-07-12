#pragma once

#include <string>

#include "rkfast/config.hpp"
#include "rkfast/types.hpp"

namespace rkfast {

class AimWriter {
 public:
  explicit AimWriter(const AppConfig& config);
  ~AimWriter();

  bool Open(std::string* error);
  void Close();
  void Send(const TrackSnapshot& snapshot, int frame_w, int frame_h);

 private:
  static std::string FormatAim(const TrackSnapshot& snapshot, int frame_w, int frame_h);
  bool ConfigureSerial(int fd, std::string* error) const;

  AppConfig config_;
  int fd_ = -1;
  double last_stdout_ = 0.0;
  double last_serial_ = 0.0;
};

}  // namespace rkfast
