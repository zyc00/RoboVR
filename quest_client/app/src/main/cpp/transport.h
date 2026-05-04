#pragma once

#include <cstdint>
#include <cstddef>
#include <string_view>

#include "quest3_teleop_protocol.h"

namespace robovr {

std::int64_t monotonicTimeNs();

bool readExact(int fd, void* data, std::size_t size);
bool writeExact(int fd, const void* data, std::size_t size);
bool sendPacket(int fd,
                quest3_teleop::PacketType type,
                std::uint64_t seq,
                std::string_view payload);
int connectTcp(const char* host, std::uint16_t port, bool tcp_no_delay = true);
bool hasReadableData(int fd);

}  // namespace robovr
