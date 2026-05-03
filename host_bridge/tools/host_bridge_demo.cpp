#include <csignal>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <vector>

#include "quest3_teleop/tcp_server.h"

namespace {

std::atomic<bool> g_running{true};

std::mutex g_video_stats_mutex;
std::unordered_map<std::uint64_t, std::int64_t> g_video_send_time_ns;
std::uint64_t g_video_ack_count = 0;
double g_video_ack_latency_sum_ms = 0.0;
std::uint64_t g_latest_video_ack_frame = 0;
bool g_has_video_ack = false;

struct H264TestStream {
  std::vector<std::string> access_units;
  std::vector<bool> keyframes;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t fps = 30;
};

void handleSignal(int) {
  g_running = false;
}

std::unordered_map<std::string, std::string> parseKeyValuePayload(std::string_view payload) {
  std::unordered_map<std::string, std::string> values;
  std::size_t start = 0;
  while (start < payload.size()) {
    const std::size_t end = payload.find(';', start);
    const std::string_view field = payload.substr(
        start, end == std::string_view::npos ? std::string_view::npos : end - start);
    const std::size_t equal = field.find('=');
    if (equal != std::string_view::npos) {
      values.emplace(std::string(field.substr(0, equal)), std::string(field.substr(equal + 1)));
    }
    if (end == std::string_view::npos) {
      break;
    }
    start = end + 1;
  }
  return values;
}

std::string getValue(const std::unordered_map<std::string, std::string>& values,
                     const std::string& key,
                     std::string_view fallback = "?") {
  const auto it = values.find(key);
  if (it == values.end()) {
    return std::string(fallback);
  }
  return it->second;
}

std::string formatPoseSummary(std::string_view payload) {
  const auto values = parseKeyValuePayload(payload);
  std::ostringstream out;
  out << std::fixed << std::setprecision(3)
      << "head=(" << getValue(values, "head_px") << ","
      << getValue(values, "head_py") << ","
      << getValue(values, "head_pz") << ") "
      << "ipd_m=" << getValue(values, "ipd_m")
      << " Lfov=(" << getValue(values, "left_fov_left") << ","
      << getValue(values, "left_fov_right") << ","
      << getValue(values, "left_fov_up") << ","
      << getValue(values, "left_fov_down") << ")"
      << " Rfov=(" << getValue(values, "right_fov_left") << ","
      << getValue(values, "right_fov_right") << ","
      << getValue(values, "right_fov_up") << ","
      << getValue(values, "right_fov_down") << ") "
      << "L{grip=" << getValue(values, "left_grip_flags")
      << " aim=" << getValue(values, "left_aim_flags")
      << " trig=" << getValue(values, "left_trigger")
      << " sqz=" << getValue(values, "left_squeeze")
      << " X=" << getValue(values, "button_x")
      << " Y=" << getValue(values, "button_y")
      << "} "
      << "R{grip=" << getValue(values, "right_grip_flags")
      << " aim=" << getValue(values, "right_aim_flags")
      << " trig=" << getValue(values, "right_trigger")
      << " sqz=" << getValue(values, "right_squeeze")
      << " A=" << getValue(values, "button_a")
      << " B=" << getValue(values, "button_b")
      << "}";
  return out.str();
}

void handleStatsPayload(std::string_view payload) {
  const auto values = parseKeyValuePayload(payload);
  if (getValue(values, "kind", "") != "video_ack") {
    std::cout << "rx stats payload=\"" << payload << "\"\n";
    return;
  }

  const auto frame_it = values.find("frame_index");
  if (frame_it == values.end()) {
    return;
  }
  const std::uint64_t frame_index = std::strtoull(frame_it->second.c_str(), nullptr, 10);
  const std::int64_t now_ns = quest3_teleop::monotonicTimeNs();
  double host_tx_to_ack_ms = 0.0;
  bool found_send_time = false;
  {
    std::lock_guard<std::mutex> lock(g_video_stats_mutex);
    const auto send_it = g_video_send_time_ns.find(frame_index);
    if (send_it != g_video_send_time_ns.end()) {
      host_tx_to_ack_ms = static_cast<double>(now_ns - send_it->second) / 1.0e6;
      g_video_send_time_ns.erase(send_it);
      found_send_time = true;
      ++g_video_ack_count;
      g_video_ack_latency_sum_ms += host_tx_to_ack_ms;
      g_latest_video_ack_frame = frame_index;
      g_has_video_ack = true;
    }
    for (auto it = g_video_send_time_ns.begin(); it != g_video_send_time_ns.end();) {
      if (it->first + 300 < frame_index) {
        it = g_video_send_time_ns.erase(it);
      } else {
        ++it;
      }
    }
  }

  if (!found_send_time) {
    return;
  }
  if (g_video_ack_count % 30 == 1) {
    const double avg_ms = g_video_ack_latency_sum_ms / static_cast<double>(g_video_ack_count);
    std::cout << "rx video_ack frame=" << frame_index
              << " host_tx_to_ack_ms=" << std::fixed << std::setprecision(2) << host_tx_to_ack_ms
              << " avg_ms=" << avg_ms
              << " quest_receive_to_publish_ms=" << getValue(values, "quest_receive_to_publish_ms", "?")
              << " surface=" << getValue(values, "surface", "?") << '\n';
  }
}

std::string makeVideoTestPatternPayload(std::uint64_t frame_index) {
  constexpr int width = 128;
  constexpr int height = 128;
  constexpr std::size_t eye_payload_size = width * height * 4;

  quest3_teleop::VideoFrameHeader header{};
  header.codec = static_cast<std::uint16_t>(quest3_teleop::VideoCodec::kRawRgba);
  header.flags = quest3_teleop::kVideoFrameKeyframe;
  header.stream_id = 0;
  header.frame_index = frame_index;
  header.capture_time_ns = quest3_teleop::monotonicTimeNs();
  header.width = width;
  header.height = height;
  header.left_payload_size = eye_payload_size;
  header.right_payload_size = eye_payload_size;

  std::string payload(quest3_teleop::kVideoFrameHeaderSize + eye_payload_size * 2, '\0');
  const auto header_bytes = quest3_teleop::packVideoFrameHeader(header);
  std::memcpy(payload.data(), header_bytes.data(), header_bytes.size());
  auto* pixels = reinterpret_cast<unsigned char*>(payload.data() + header_bytes.size());

  auto fill_eye = [&](int eye, unsigned char* out) {
    const int moving_bar = static_cast<int>((frame_index * 5 + eye * 37) % width);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const bool checker = ((x / 16) + (y / 16) + eye) % 2 == 0;
        const bool bar = std::abs(x - moving_bar) < 4;
        const std::size_t index = static_cast<std::size_t>(y * width + x) * 4;
        if (eye == 0) {
          out[index + 0] = bar ? 255 : (checker ? 32 : 16);
          out[index + 1] = checker ? 220 : 72;
          out[index + 2] = bar ? 24 : 245;
        } else {
          out[index + 0] = checker ? 245 : 96;
          out[index + 1] = bar ? 255 : (checker ? 72 : 24);
          out[index + 2] = checker ? 32 : 220;
        }
        out[index + 3] = 255;
      }
    }
  };

  fill_eye(0, pixels);
  fill_eye(1, pixels + eye_payload_size);
  return payload;
}

std::optional<H264TestStream> loadH264TestStream(const char* path,
                                                 std::uint32_t width,
                                                 std::uint32_t height) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    return std::nullopt;
  }
  std::string bytes((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  if (bytes.empty()) {
    return std::nullopt;
  }

  auto start_code_size = [](std::string_view data, std::size_t offset) -> std::size_t {
    if (offset + 3 <= data.size() &&
        static_cast<unsigned char>(data[offset]) == 0x00 &&
        static_cast<unsigned char>(data[offset + 1]) == 0x00 &&
        static_cast<unsigned char>(data[offset + 2]) == 0x01) {
      return 3;
    }
    if (offset + 4 <= data.size() &&
        static_cast<unsigned char>(data[offset]) == 0x00 &&
        static_cast<unsigned char>(data[offset + 1]) == 0x00 &&
        static_cast<unsigned char>(data[offset + 2]) == 0x00 &&
        static_cast<unsigned char>(data[offset + 3]) == 0x01) {
      return 4;
    }
    return 0;
  };
  auto find_start_code = [&](std::size_t offset) -> std::size_t {
    for (std::size_t i = offset; i + 3 <= bytes.size(); ++i) {
      if (start_code_size(bytes, i) != 0) {
        return i;
      }
    }
    return std::string::npos;
  };
  auto read_first_mb_in_slice = [](std::string_view nal) -> std::optional<std::uint32_t> {
    if (nal.empty()) {
      return std::nullopt;
    }
    std::vector<std::uint8_t> rbsp;
    rbsp.reserve(std::min<std::size_t>(nal.size(), 32));
    for (std::size_t i = 1; i < nal.size() && rbsp.size() < 32; ++i) {
      if (i + 2 < nal.size() &&
          static_cast<std::uint8_t>(nal[i]) == 0x00 &&
          static_cast<std::uint8_t>(nal[i + 1]) == 0x00 &&
          static_cast<std::uint8_t>(nal[i + 2]) == 0x03) {
        rbsp.push_back(0x00);
        rbsp.push_back(0x00);
        i += 2;
      } else {
        rbsp.push_back(static_cast<std::uint8_t>(nal[i]));
      }
    }

    std::size_t bit_offset = 0;
    std::uint32_t leading_zero_bits = 0;
    while (bit_offset < rbsp.size() * 8) {
      const std::uint8_t byte = rbsp[bit_offset / 8];
      const bool bit = ((byte >> (7 - (bit_offset % 8))) & 0x01) != 0;
      ++bit_offset;
      if (bit) {
        break;
      }
      ++leading_zero_bits;
      if (leading_zero_bits > 31) {
        return std::nullopt;
      }
    }
    std::uint32_t suffix = 0;
    for (std::uint32_t i = 0; i < leading_zero_bits; ++i) {
      if (bit_offset >= rbsp.size() * 8) {
        return std::nullopt;
      }
      const std::uint8_t byte = rbsp[bit_offset / 8];
      const bool bit = ((byte >> (7 - (bit_offset % 8))) & 0x01) != 0;
      suffix = (suffix << 1) | (bit ? 1u : 0u);
      ++bit_offset;
    }
    return ((1u << leading_zero_bits) - 1u) + suffix;
  };

  std::vector<std::string> access_units;
  std::vector<bool> keyframes;
  std::string current_access_unit;
  bool current_has_vcl = false;
  bool current_is_keyframe = false;
  std::size_t nal_start = find_start_code(0);
  while (nal_start != std::string::npos) {
    const std::size_t next_nal = find_start_code(nal_start + start_code_size(bytes, nal_start));
    const std::size_t nal_end = next_nal == std::string::npos ? bytes.size() : next_nal;
    const std::size_t header_offset = nal_start + start_code_size(bytes, nal_start);
    if (header_offset < nal_end) {
      const std::uint8_t nal_type = static_cast<std::uint8_t>(bytes[header_offset]) & 0x1f;
      const std::string_view nal(bytes.data() + nal_start, nal_end - nal_start);
      const bool is_vcl = nal_type == 1 || nal_type == 5;
      if (is_vcl) {
        const std::string_view nal_body(bytes.data() + header_offset, nal_end - header_offset);
        const std::optional<std::uint32_t> first_mb_in_slice = read_first_mb_in_slice(nal_body);
        if (current_has_vcl && first_mb_in_slice.value_or(0) == 0) {
          access_units.push_back(std::move(current_access_unit));
          keyframes.push_back(current_is_keyframe);
          current_access_unit.clear();
          current_has_vcl = false;
          current_is_keyframe = false;
        }
        current_has_vcl = true;
        current_is_keyframe = current_is_keyframe || nal_type == 5;
      } else if (nal_type == 9 && !current_access_unit.empty()) {
        access_units.push_back(std::move(current_access_unit));
        keyframes.push_back(current_is_keyframe);
        current_access_unit.clear();
        current_has_vcl = false;
        current_is_keyframe = false;
      }
      current_access_unit.append(nal);
    }
    nal_start = next_nal;
  }
  if (!current_access_unit.empty()) {
    access_units.push_back(std::move(current_access_unit));
    keyframes.push_back(current_is_keyframe);
  }

  if (access_units.empty()) {
    access_units.push_back(std::move(bytes));
    keyframes.push_back(true);
  }
  return H264TestStream{std::move(access_units), std::move(keyframes), width, height, 30};
}

std::string makeH264TestPayload(const H264TestStream& stream, std::uint64_t frame_index) {
  const std::size_t access_unit_index = frame_index % stream.access_units.size();
  const std::string& access_unit = stream.access_units[access_unit_index];
  quest3_teleop::VideoFrameHeader header{};
  header.codec = static_cast<std::uint16_t>(quest3_teleop::VideoCodec::kH264AnnexB);
  header.flags = stream.keyframes[access_unit_index] ? quest3_teleop::kVideoFrameKeyframe : 0;
  header.stream_id = 0;
  header.frame_index = frame_index;
  header.capture_time_ns = quest3_teleop::monotonicTimeNs();
  header.width = stream.width;
  header.height = stream.height;
  header.left_payload_size = static_cast<std::uint32_t>(access_unit.size());
  header.right_payload_size = static_cast<std::uint32_t>(access_unit.size());

  std::string payload(quest3_teleop::kVideoFrameHeaderSize + access_unit.size() * 2, '\0');
  const auto header_bytes = quest3_teleop::packVideoFrameHeader(header);
  std::memcpy(payload.data(), header_bytes.data(), header_bytes.size());
  char* output = payload.data() + header_bytes.size();
  std::memcpy(output, access_unit.data(), access_unit.size());
  std::memcpy(output + access_unit.size(), access_unit.data(), access_unit.size());
  return payload;
}

}  // namespace

int main(int argc, char** argv) {
  std::uint16_t port = 7777;
  if (argc >= 2) {
    port = static_cast<std::uint16_t>(std::atoi(argv[1]));
  }
  std::optional<H264TestStream> h264_stream;
  if ((argc == 5 || argc == 6) && std::string_view(argv[2]) == "--h264") {
    const auto width = static_cast<std::uint32_t>(std::atoi(argv[4]));
    const auto height = argc == 6 ? static_cast<std::uint32_t>(std::atoi(argv[5])) : width;
    h264_stream = loadH264TestStream(argv[3], width, height);
    if (!h264_stream.has_value()) {
      std::cerr << "failed to load H.264 Annex-B stream: " << argv[3] << '\n';
      return 1;
    }
    std::cout << "loaded H.264 Annex-B test stream: " << argv[3]
              << " access_units=" << h264_stream->access_units.size()
              << " size=" << h264_stream->width << "x" << h264_stream->height << '\n';
  } else if (argc != 1 && argc != 2) {
    std::cerr << "usage: " << argv[0] << " [port] [--h264 annexb.h264 width [height]]\n";
    return 1;
  }

  std::signal(SIGINT, handleSignal);
  std::signal(SIGTERM, handleSignal);

  quest3_teleop::TcpServer server(port);
  server.setStatusCallback([port](std::string_view message) {
    std::cout << "[host:" << port << "] " << message << '\n';
  });
  server.setPacketCallback([&server, &h264_stream](const quest3_teleop::ReceivedPacket& packet) {
    const auto type = static_cast<quest3_teleop::PacketType>(packet.header.type);
    static std::uint64_t heartbeat_count = 0;
    static std::uint64_t pose_count = 0;
    static std::uint64_t video_frame = 0;

    if (type == quest3_teleop::PacketType::kHeartbeat) {
      ++heartbeat_count;
      if (heartbeat_count % 20 == 1) {
        std::cout << "rx heartbeat seq=" << packet.header.seq << " count=" << heartbeat_count << '\n';
      }
      server.send(quest3_teleop::PacketType::kHeartbeat, "host_alive=1");
      return;
    }

    if (type == quest3_teleop::PacketType::kPose) {
      ++pose_count;
      if (!h264_stream.has_value() && pose_count % 10 == 1) {
        const std::string video_payload = makeVideoTestPatternPayload(video_frame);
        server.send(quest3_teleop::PacketType::kVideo, video_payload);
        if (video_frame % 30 == 0) {
          std::cout << "tx raw video frame=" << video_frame << '\n';
        }
        ++video_frame;
      }
      if (pose_count % 30 == 1) {
        std::cout << "rx pose seq=" << packet.header.seq << " count=" << pose_count;
        if (!packet.payload.empty()) {
          std::cout << " " << formatPoseSummary(packet.payload);
        }
        std::cout << '\n';
      }
      return;
    }

    if (type == quest3_teleop::PacketType::kStats) {
      handleStatsPayload(packet.payload);
      return;
    }

    std::cout << "rx seq=" << packet.header.seq << " type="
              << quest3_teleop::packetTypeName(type) << " size="
              << packet.header.payload_size;
    if (!packet.payload.empty()) {
      std::cout << " payload=\"" << packet.payload << "\"";
    }
    std::cout << '\n';

    if (type == quest3_teleop::PacketType::kHello) {
      server.send(quest3_teleop::PacketType::kHello, "host=robovr;protocol=1");
    }
  });

  if (!server.start()) {
    std::cerr << "failed to start host bridge on port " << port << '\n';
    return 1;
  }

  std::thread video_thread;
  if (h264_stream.has_value()) {
    video_thread = std::thread([&server, &h264_stream] {
      std::uint64_t video_frame = 0;
      const auto frame_interval = std::chrono::nanoseconds(1000000000LL / h264_stream->fps);
      auto next_frame_time = std::chrono::steady_clock::now();
      bool started_streaming = false;
      while (g_running) {
        const auto now = std::chrono::steady_clock::now();
        if (started_streaming && now > next_frame_time + frame_interval) {
          const auto missed_frames = static_cast<std::uint64_t>((now - next_frame_time) / frame_interval);
          video_frame += missed_frames;
          next_frame_time += frame_interval * missed_frames;
        }
        {
          std::lock_guard<std::mutex> lock(g_video_stats_mutex);
          if (g_has_video_ack && video_frame > g_latest_video_ack_frame + 3) {
            video_frame = g_latest_video_ack_frame + 3;
            next_frame_time = std::chrono::steady_clock::now();
          }
        }
        const std::string video_payload = makeH264TestPayload(*h264_stream, video_frame);
        if (server.send(quest3_teleop::PacketType::kVideo, video_payload)) {
          if (!started_streaming) {
            started_streaming = true;
            next_frame_time = std::chrono::steady_clock::now();
          }
          {
            std::lock_guard<std::mutex> lock(g_video_stats_mutex);
            g_video_send_time_ns[video_frame] = quest3_teleop::monotonicTimeNs();
          }
          if (video_frame % 30 == 0) {
            std::cout << "tx h264 video frame=" << video_frame << '\n';
          }
          ++video_frame;
          next_frame_time += frame_interval;
          std::this_thread::sleep_until(next_frame_time);
        } else {
          started_streaming = false;
          next_frame_time = std::chrono::steady_clock::now();
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
      }
    });
  }

  while (g_running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (video_thread.joinable()) {
    video_thread.join();
  }
  server.stop();
  return 0;
}
