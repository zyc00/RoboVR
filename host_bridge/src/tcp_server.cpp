#include "quest3_teleop/tcp_server.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

namespace quest3_teleop {
namespace {

constexpr std::uint32_t kMaxPayloadSize = 16 * 1024 * 1024;

bool readExact(int fd, void* data, std::size_t size) {
  auto* bytes = static_cast<std::uint8_t*>(data);
  std::size_t total = 0;
  while (total < size) {
    const ssize_t n = ::recv(fd, bytes + total, size - total, 0);
    if (n == 0) {
      return false;
    }
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    total += static_cast<std::size_t>(n);
  }
  return true;
}

bool writeExact(int fd, const void* data, std::size_t size) {
  const auto* bytes = static_cast<const std::uint8_t*>(data);
  std::size_t total = 0;
  while (total < size) {
    const ssize_t n = ::send(fd, bytes + total, size - total, MSG_NOSIGNAL);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    total += static_cast<std::size_t>(n);
  }
  return true;
}

void closeFd(int& fd) {
  if (fd >= 0) {
    ::shutdown(fd, SHUT_RDWR);
    ::close(fd);
    fd = -1;
  }
}

}  // namespace

std::int64_t monotonicTimeNs() {
  timespec ts{};
  ::clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<std::int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

TcpServer::TcpServer(std::uint16_t port) : port_(port) {}

TcpServer::~TcpServer() {
  stop();
}

bool TcpServer::start() {
  if (running_.exchange(true)) {
    return true;
  }

  listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (listen_fd_ < 0) {
    running_ = false;
    emitStatus("socket() failed");
    return false;
  }

  int yes = 1;
  ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(port_);

  if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    running_ = false;
    emitStatus("bind() failed");
    closeFd(listen_fd_);
    return false;
  }

  if (::listen(listen_fd_, 1) < 0) {
    running_ = false;
    emitStatus("listen() failed");
    closeFd(listen_fd_);
    return false;
  }

  worker_ = std::thread(&TcpServer::acceptLoop, this);
  emitStatus("listening");
  return true;
}

void TcpServer::stop() {
  if (!running_.exchange(false)) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(send_mutex_);
    closeFd(client_fd_);
  }
  closeFd(listen_fd_);
  if (worker_.joinable()) {
    worker_.join();
  }
}

bool TcpServer::send(PacketType type, std::string_view payload) {
  std::lock_guard<std::mutex> lock(send_mutex_);
  if (client_fd_ < 0) {
    return false;
  }
  PacketHeader header{};
  header.type = static_cast<std::uint16_t>(type);
  header.seq = send_seq_.fetch_add(1);
  header.timestamp_ns = monotonicTimeNs();
  header.payload_size = static_cast<std::uint32_t>(payload.size());

  const auto header_bytes = packHeader(header);
  return writeExact(client_fd_, header_bytes.data(), header_bytes.size()) &&
         writeExact(client_fd_, payload.data(), payload.size());
}

void TcpServer::setPacketCallback(PacketCallback callback) {
  packet_callback_ = std::move(callback);
}

void TcpServer::setStatusCallback(StatusCallback callback) {
  status_callback_ = std::move(callback);
}

void TcpServer::acceptLoop() {
  while (running_) {
    sockaddr_in client_addr{};
    socklen_t client_len = sizeof(client_addr);
    int fd = ::accept(listen_fd_, reinterpret_cast<sockaddr*>(&client_addr), &client_len);
    if (fd < 0) {
      if (running_) {
        emitStatus("accept() failed");
      }
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(send_mutex_);
      if (client_fd_ >= 0) {
        closeFd(client_fd_);
      }
      int one = 1;
      ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
      client_fd_ = fd;
    }
    emitStatus("client connected");
    clientLoop(fd);
    {
      std::lock_guard<std::mutex> lock(send_mutex_);
      if (client_fd_ == fd) {
        closeFd(client_fd_);
      }
    }
    emitStatus("client disconnected");
  }
}

void TcpServer::clientLoop(int client_fd) {
  while (running_) {
    std::array<std::uint8_t, kPacketHeaderSize> header_bytes{};
    if (!readExact(client_fd, header_bytes.data(), header_bytes.size())) {
      return;
    }

    PacketHeader header = unpackHeader(header_bytes.data());
    if (header.magic != kProtocolMagic || header.version != kProtocolVersion ||
        !isKnownPacketType(header.type) || header.payload_size > kMaxPayloadSize) {
      emitStatus("invalid packet header");
      return;
    }

    std::string payload(header.payload_size, '\0');
    if (!payload.empty() && !readExact(client_fd, payload.data(), payload.size())) {
      return;
    }

    if (packet_callback_) {
      packet_callback_(ReceivedPacket{header, std::move(payload)});
    }
  }
}

void TcpServer::emitStatus(std::string_view message) const {
  if (status_callback_) {
    status_callback_(message);
  }
}

}  // namespace quest3_teleop
