#pragma once

#include <array>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <string>
#include <string_view>

namespace quest3_teleop {

constexpr std::uint32_t kProtocolMagic = 0x50543351u; // "Q3TP" little-endian.
constexpr std::uint16_t kProtocolVersion = 1;
constexpr std::size_t kPacketHeaderSize = 32;
constexpr std::size_t kVideoFrameHeaderSize = 40;

enum class PacketType : std::uint16_t {
  kHello = 1,
  kConfig = 2,
  kVideo = 3,
  kPose = 4,
  kStats = 5,
  kHeartbeat = 6,
  kGoodbye = 7,
  kError = 8,
};

struct PacketHeader {
  std::uint32_t magic = kProtocolMagic;
  std::uint16_t version = kProtocolVersion;
  std::uint16_t type = 0;
  std::uint64_t seq = 0;
  std::int64_t timestamp_ns = 0;
  std::uint32_t payload_size = 0;
  std::uint32_t crc32 = 0;
};

static_assert(sizeof(PacketHeader) == kPacketHeaderSize);

enum class VideoCodec : std::uint16_t {
  kRawRgba = 1,
  kH264AnnexB = 2,
  kH265AnnexB = 3,
  kAv1AnnexB = 4,
};

enum VideoFrameFlags : std::uint16_t {
  kVideoFrameKeyframe = 1u << 0u,
};

struct VideoFrameHeader {
  std::uint16_t codec = 0;
  std::uint16_t flags = 0;
  std::uint32_t stream_id = 0;
  std::uint64_t frame_index = 0;
  std::int64_t capture_time_ns = 0;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t left_payload_size = 0;
  std::uint32_t right_payload_size = 0;
};

static_assert(sizeof(VideoFrameHeader) == kVideoFrameHeaderSize);

inline std::array<std::uint8_t, kPacketHeaderSize> packHeader(const PacketHeader& header) {
  std::array<std::uint8_t, kPacketHeaderSize> bytes{};
  std::memcpy(bytes.data(), &header, sizeof(header));
  return bytes;
}

inline PacketHeader makePacketHeader(PacketType type,
                                     std::uint64_t seq,
                                     std::int64_t timestamp_ns,
                                     std::size_t payload_size) {
  PacketHeader header{};
  header.type = static_cast<std::uint16_t>(type);
  header.seq = seq;
  header.timestamp_ns = timestamp_ns;
  header.payload_size = static_cast<std::uint32_t>(payload_size);
  return header;
}

inline PacketHeader unpackHeader(const std::uint8_t* data) {
  PacketHeader header{};
  std::memcpy(&header, data, sizeof(header));
  return header;
}

inline std::array<std::uint8_t, kVideoFrameHeaderSize> packVideoFrameHeader(
    const VideoFrameHeader& header) {
  std::array<std::uint8_t, kVideoFrameHeaderSize> bytes{};
  std::memcpy(bytes.data(), &header, sizeof(header));
  return bytes;
}

inline VideoFrameHeader unpackVideoFrameHeader(const std::uint8_t* data) {
  VideoFrameHeader header{};
  std::memcpy(&header, data, sizeof(header));
  return header;
}

inline std::string_view packetTypeName(PacketType type) {
  switch (type) {
    case PacketType::kHello:
      return "HELLO";
    case PacketType::kConfig:
      return "CONFIG";
    case PacketType::kVideo:
      return "VIDEO";
    case PacketType::kPose:
      return "POSE";
    case PacketType::kStats:
      return "STATS";
    case PacketType::kHeartbeat:
      return "HEARTBEAT";
    case PacketType::kGoodbye:
      return "GOODBYE";
    case PacketType::kError:
      return "ERROR";
  }
  return "UNKNOWN";
}

inline bool isKnownPacketType(std::uint16_t type) {
  switch (static_cast<PacketType>(type)) {
    case PacketType::kHello:
    case PacketType::kConfig:
    case PacketType::kVideo:
    case PacketType::kPose:
    case PacketType::kStats:
    case PacketType::kHeartbeat:
    case PacketType::kGoodbye:
    case PacketType::kError:
      return true;
  }
  return false;
}

inline std::string_view findKeyValue(std::string_view payload, std::string_view key) {
  std::size_t start = 0;
  while (start < payload.size()) {
    std::size_t end = payload.find_first_of(";\n", start);
    const std::string_view field = payload.substr(
        start, end == std::string_view::npos ? std::string_view::npos : end - start);
    const std::size_t equal = field.find('=');
    if (equal != std::string_view::npos && field.substr(0, equal) == key) {
      return field.substr(equal + 1);
    }
    if (end == std::string_view::npos) {
      break;
    }
    start = end + 1;
  }
  return {};
}

inline float parseFloatValue(std::string_view value, float fallback) {
  if (value.empty()) {
    return fallback;
  }
  std::string text(value);
  char* end = nullptr;
  const float parsed = std::strtof(text.c_str(), &end);
  if (end == text.c_str()) {
    return fallback;
  }
  return parsed;
}

}  // namespace quest3_teleop
