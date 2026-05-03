#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "quest3_teleop_protocol.h"

namespace quest3_teleop {

struct ReceivedPacket {
  PacketHeader header;
  std::string payload;
};

class TcpServer {
 public:
  using PacketCallback = std::function<void(const ReceivedPacket&)>;
  using StatusCallback = std::function<void(std::string_view)>;

  explicit TcpServer(std::uint16_t port);
  ~TcpServer();

  TcpServer(const TcpServer&) = delete;
  TcpServer& operator=(const TcpServer&) = delete;

  bool start();
  void stop();
  bool send(PacketType type, std::string_view payload);

  void setPacketCallback(PacketCallback callback);
  void setStatusCallback(StatusCallback callback);

 private:
  void acceptLoop();
  void clientLoop(int client_fd);
  void emitStatus(std::string_view message) const;

  std::uint16_t port_;
  std::atomic<bool> running_{false};
  std::atomic<std::uint64_t> send_seq_{1};
  int listen_fd_ = -1;
  int client_fd_ = -1;
  mutable std::mutex send_mutex_;
  std::thread worker_;
  PacketCallback packet_callback_;
  StatusCallback status_callback_;
};

std::int64_t monotonicTimeNs();

}  // namespace quest3_teleop
