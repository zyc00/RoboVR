#include "transport.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <cerrno>

namespace robovr {

std::int64_t monotonicTimeNs() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<std::int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

bool readExact(int fd, void* data, std::size_t size) {
  auto* bytes = static_cast<std::uint8_t*>(data);
  std::size_t total = 0;
  while (total < size) {
    const ssize_t n = recv(fd, bytes + total, size - total, 0);
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
    const ssize_t n = send(fd, bytes + total, size - total, MSG_NOSIGNAL);
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

bool sendPacket(int fd,
                quest3_teleop::PacketType type,
                std::uint64_t seq,
                std::string_view payload) {
  const quest3_teleop::PacketHeader header =
      quest3_teleop::makePacketHeader(type, seq, monotonicTimeNs(), payload.size());
  const auto header_bytes = quest3_teleop::packHeader(header);
  return writeExact(fd, header_bytes.data(), header_bytes.size()) &&
         writeExact(fd, payload.data(), payload.size());
}

int connectTcp(const char* host, std::uint16_t port, bool tcp_no_delay) {
  int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    return -1;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (inet_pton(AF_INET, host, &addr.sin_addr) != 1) {
    close(fd);
    return -1;
  }

  if (connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    close(fd);
    return -1;
  }
  if (tcp_no_delay) {
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
  }
  return fd;
}

bool hasReadableData(int fd) {
  fd_set read_set;
  FD_ZERO(&read_set);
  FD_SET(fd, &read_set);
  timeval timeout{};
  return select(fd + 1, &read_set, nullptr, nullptr, &timeout) > 0 && FD_ISSET(fd, &read_set);
}

}  // namespace robovr
