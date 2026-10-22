#include "rkfast/mjpeg_server.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>

#include <opencv2/imgcodecs.hpp>

namespace rkfast {
namespace {

// Write all bytes to fd, retrying on partial writes. Returns false if the peer
// closed or errored (broken pipe on client disconnect is expected, not fatal).
bool WriteAll(int fd, const void* data, size_t len) {
  const auto* p = static_cast<const uint8_t*>(data);
  size_t sent = 0;
  while (sent < len) {
    const ssize_t n = ::send(fd, p + sent, len - sent, MSG_NOSIGNAL);
    if (n > 0) {
      sent += static_cast<size_t>(n);
      continue;
    }
    if (n < 0 && (errno == EINTR)) {
      continue;
    }
    return false;
  }
  return true;
}

constexpr char kStreamHeader[] =
    "HTTP/1.0 200 OK\r\n"
    "Connection: close\r\n"
    "Cache-Control: no-cache, private\r\n"
    "Pragma: no-cache\r\n"
    "Access-Control-Allow-Origin: *\r\n"
    "Content-Type: multipart/x-mixed-replace; boundary=rkfastframe\r\n"
    "\r\n";

}  // namespace

MjpegServer::~MjpegServer() { Stop(); }

bool MjpegServer::Start(int port, int jpeg_quality, std::string* error) {
  const auto fail = [&](const std::string& msg) {
    if (error != nullptr) *error = "mjpeg_server: " + msg;
    if (listen_fd_ >= 0) {
      ::close(listen_fd_);
      listen_fd_ = -1;
    }
    return false;
  };

  port_ = port;
  jpeg_quality_ = jpeg_quality;

  listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (listen_fd_ < 0) {
    return fail(std::string("socket() failed: ") + std::strerror(errno));
  }
  int one = 1;
  ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) <
      0) {
    return fail("bind() to port " + std::to_string(port) +
                " failed: " + std::strerror(errno));
  }
  if (::listen(listen_fd_, 4) < 0) {
    return fail(std::string("listen() failed: ") + std::strerror(errno));
  }

  running_.store(true, std::memory_order_relaxed);
  accept_thread_ = std::thread(&MjpegServer::AcceptLoop, this);
  return true;
}

void MjpegServer::AcceptLoop() {
  while (running_.load(std::memory_order_relaxed)) {
    sockaddr_in peer{};
    socklen_t peer_len = sizeof(peer);
    const int client_fd =
        ::accept(listen_fd_, reinterpret_cast<sockaddr*>(&peer), &peer_len);
    if (client_fd < 0) {
      if (!running_.load(std::memory_order_relaxed)) break;
      if (errno == EINTR) continue;
      break;
    }
    int one = 1;
    ::setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    std::lock_guard<std::mutex> lock(clients_mutex_);
    client_threads_.emplace_back(&MjpegServer::ServeClient, this, client_fd);
  }
}

void MjpegServer::ServeClient(int client_fd) {
  client_count_.fetch_add(1, std::memory_order_relaxed);

  // Drain the HTTP request line/headers. We serve the same MJPEG stream on any
  // path, so the request content does not matter; we just need to consume it so
  // the browser is happy before we start replying.
  char scratch[1024];
  ::recv(client_fd, scratch, sizeof(scratch), MSG_DONTWAIT);

  bool ok = WriteAll(client_fd, kStreamHeader, sizeof(kStreamHeader) - 1);

  uint64_t last_sent_seq = 0;
  std::vector<uint8_t> local;
  while (ok && running_.load(std::memory_order_relaxed)) {
    {
      std::unique_lock<std::mutex> lock(frame_mutex_);
      frame_cv_.wait_for(lock, std::chrono::milliseconds(500), [&] {
        return frame_seq_ != last_sent_seq ||
               !running_.load(std::memory_order_relaxed);
      });
      if (!running_.load(std::memory_order_relaxed)) break;
      if (frame_seq_ == last_sent_seq || jpeg_buffer_.empty()) {
        continue;  // spurious wake or no frame yet
      }
      last_sent_seq = frame_seq_;
      local = jpeg_buffer_;  // copy out, release lock before network I/O
    }

    char part_header[128];
    const int header_len = std::snprintf(
        part_header, sizeof(part_header),
        "--rkfastframe\r\nContent-Type: image/jpeg\r\n"
        "Content-Length: %zu\r\n\r\n",
        local.size());
    ok = header_len > 0 &&
         WriteAll(client_fd, part_header, static_cast<size_t>(header_len)) &&
         WriteAll(client_fd, local.data(), local.size()) &&
         WriteAll(client_fd, "\r\n", 2);
  }

  ::close(client_fd);
  client_count_.fetch_sub(1, std::memory_order_relaxed);
}

void MjpegServer::PushFrame(const cv::Mat& frame) {
  // Skip all encoding work when nobody is watching.
  if (client_count_.load(std::memory_order_relaxed) == 0 || frame.empty()) {
    return;
  }
  std::vector<uint8_t> encoded;
  const std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
  if (!cv::imencode(".jpg", frame, encoded, params)) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    jpeg_buffer_.swap(encoded);
    ++frame_seq_;
  }
  frame_cv_.notify_all();
}

void MjpegServer::Stop() {
  if (!running_.exchange(false)) {
    return;  // already stopped
  }
  frame_cv_.notify_all();

  // Unblock accept() by shutting down and closing the listening socket.
  if (listen_fd_ >= 0) {
    ::shutdown(listen_fd_, SHUT_RDWR);
    ::close(listen_fd_);
    listen_fd_ = -1;
  }
  if (accept_thread_.joinable()) {
    accept_thread_.join();
  }

  std::vector<std::thread> threads;
  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    threads.swap(client_threads_);
  }
  for (auto& t : threads) {
    if (t.joinable()) t.join();
  }
}

}  // namespace rkfast
