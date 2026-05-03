#include <android/log.h>
#include <android_native_app_glue.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <array>
#include <chrono>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>

#include "openxr_app.h"
#include "quest3_teleop_protocol.h"
#include "video_decoder.h"

namespace {

constexpr char kLogTag[] = "RoboVR";

std::atomic<bool> g_transport_running{false};
std::thread g_transport_thread;
std::mutex g_pose_mutex;
std::optional<HeadPoseSample> g_latest_pose;
VideoDecoderRouter g_video_decoder;

void logInfo(const char* message) {
  __android_log_print(ANDROID_LOG_INFO, kLogTag, "%s", message);
}

std::int64_t nowNs() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<std::int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
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

bool sendPacket(int fd, quest3_teleop::PacketType type, std::uint64_t seq, const std::string& payload) {
  quest3_teleop::PacketHeader header{};
  header.type = static_cast<std::uint16_t>(type);
  header.seq = seq;
  header.timestamp_ns = nowNs();
  header.payload_size = static_cast<std::uint32_t>(payload.size());
  const auto header_bytes = quest3_teleop::packHeader(header);
  return writeExact(fd, header_bytes.data(), header_bytes.size()) &&
         writeExact(fd, payload.data(), payload.size());
}

int connectToHost() {
  int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    return -1;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(7777);
  if (inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr) != 1) {
    close(fd);
    return -1;
  }

  if (connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    close(fd);
    return -1;
  }
  int one = 1;
  setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
  return fd;
}

bool hasReadableData(int fd) {
  fd_set read_set;
  FD_ZERO(&read_set);
  FD_SET(fd, &read_set);
  timeval timeout{};
  return select(fd + 1, &read_set, nullptr, nullptr, &timeout) > 0 && FD_ISSET(fd, &read_set);
}

std::string_view findValue(std::string_view payload, std::string_view key) {
  std::size_t start = 0;
  while (start < payload.size()) {
    const std::size_t end = payload.find(';', start);
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

float parseFloat(std::string_view value, float fallback) {
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

bool sendVideoDecodeAcks(int fd, std::uint64_t* seq) {
  for (const VideoDecodeAck& ack : g_video_decoder.takeAcks()) {
    char payload[256];
    std::snprintf(payload, sizeof(payload),
                  "kind=video_ack;frame_index=%llu;quest_receive_to_publish_ms=%.3f;surface=%d",
                  static_cast<unsigned long long>(ack.frame_index),
                  ack.quest_receive_to_publish_ms,
                  ack.surface_output ? 1 : 0);
    if (!sendPacket(fd, quest3_teleop::PacketType::kStats, (*seq)++, payload)) {
      return false;
    }
  }
  return true;
}

void handleVideoPayload(const std::string& payload) {
  if (g_video_decoder.handlePayload(payload)) {
    return;
  }

  StereoClearColors colors;
  colors.left[0] = parseFloat(findValue(payload, "left_r"), colors.left[0]);
  colors.left[1] = parseFloat(findValue(payload, "left_g"), colors.left[1]);
  colors.left[2] = parseFloat(findValue(payload, "left_b"), colors.left[2]);
  colors.left[3] = 1.0f;
  colors.right[0] = parseFloat(findValue(payload, "right_r"), colors.right[0]);
  colors.right[1] = parseFloat(findValue(payload, "right_g"), colors.right[1]);
  colors.right[2] = parseFloat(findValue(payload, "right_b"), colors.right[2]);
  colors.right[3] = 1.0f;
  publishStereoClearColors(colors);

  static std::uint64_t video_count = 0;
  if (video_count % 30 == 0) {
    const std::string frame(findValue(payload, "frame"));
    __android_log_print(ANDROID_LOG_INFO, kLogTag, "rx video frame=%s count=%llu",
                        frame.empty() ? "?" : frame.c_str(),
                        static_cast<unsigned long long>(video_count));
  }
  ++video_count;
}

bool receiveHostPackets(int fd, std::uint64_t* seq) {
  while (hasReadableData(fd)) {
    std::array<std::uint8_t, quest3_teleop::kPacketHeaderSize> header_bytes{};
    if (!readExact(fd, header_bytes.data(), header_bytes.size())) {
      return false;
    }
    const quest3_teleop::PacketHeader header = quest3_teleop::unpackHeader(header_bytes.data());
    if (header.magic != quest3_teleop::kProtocolMagic ||
        header.version != quest3_teleop::kProtocolVersion ||
        !quest3_teleop::isKnownPacketType(header.type) ||
        header.payload_size > 1024 * 1024) {
      return false;
    }

    std::string payload(header.payload_size, '\0');
    if (!payload.empty() && !readExact(fd, payload.data(), payload.size())) {
      return false;
    }

    const auto type = static_cast<quest3_teleop::PacketType>(header.type);
    if (type == quest3_teleop::PacketType::kVideo) {
      handleVideoPayload(payload);
      if (!sendVideoDecodeAcks(fd, seq)) {
        return false;
      }
    }
  }
  return true;
}

void transportLoop() {
  std::uint64_t seq = 1;
  auto last_heartbeat = std::chrono::steady_clock::now() - std::chrono::seconds(1);
  while (g_transport_running) {
    int fd = connectToHost();
    if (fd < 0) {
      logInfo("host connection failed; retrying");
      std::this_thread::sleep_for(std::chrono::seconds(1));
      continue;
    }

    logInfo("connected to host bridge");
    sendPacket(fd, quest3_teleop::PacketType::kHello, seq++, "client=quest;protocol=1");
    last_heartbeat = std::chrono::steady_clock::now() - std::chrono::seconds(1);

    while (g_transport_running) {
      if (!receiveHostPackets(fd, &seq)) {
        break;
      }
      if (!sendVideoDecodeAcks(fd, &seq)) {
        break;
      }

      const auto now = std::chrono::steady_clock::now();
      if (now - last_heartbeat >= std::chrono::milliseconds(500)) {
        if (!sendPacket(fd, quest3_teleop::PacketType::kHeartbeat, seq++, "quest_alive=1")) {
          break;
        }
        last_heartbeat = now;
      }

      std::optional<HeadPoseSample> pose;
      {
        std::lock_guard<std::mutex> lock(g_pose_mutex);
        pose = g_latest_pose;
        g_latest_pose.reset();
      }
      if (pose.has_value()) {
        char payload[4096];
        const auto& h = pose->pose;
        const auto& le = pose->left_eye_pose;
        const auto& re = pose->right_eye_pose;
        const auto& lf = pose->left_eye_fov;
        const auto& rf = pose->right_eye_fov;
        const auto& lg = pose->left_grip_pose;
        const auto& rg = pose->right_grip_pose;
        const auto& la = pose->left_aim_pose;
        const auto& ra = pose->right_aim_pose;
        std::snprintf(
            payload, sizeof(payload),
            "predicted_display_time_ns=%lld;view_state_flags=%u;"
            "head_px=%.6f;head_py=%.6f;head_pz=%.6f;head_qx=%.6f;head_qy=%.6f;head_qz=%.6f;head_qw=%.6f;"
            "eye_view_count=%u;ipd_m=%.6f;"
            "left_eye_px=%.6f;left_eye_py=%.6f;left_eye_pz=%.6f;left_eye_qx=%.6f;left_eye_qy=%.6f;left_eye_qz=%.6f;left_eye_qw=%.6f;"
            "right_eye_px=%.6f;right_eye_py=%.6f;right_eye_pz=%.6f;right_eye_qx=%.6f;right_eye_qy=%.6f;right_eye_qz=%.6f;right_eye_qw=%.6f;"
            "left_fov_left=%.6f;left_fov_right=%.6f;left_fov_up=%.6f;left_fov_down=%.6f;"
            "right_fov_left=%.6f;right_fov_right=%.6f;right_fov_up=%.6f;right_fov_down=%.6f;"
            "left_grip_flags=%llu;left_grip_px=%.6f;left_grip_py=%.6f;left_grip_pz=%.6f;left_grip_qx=%.6f;left_grip_qy=%.6f;left_grip_qz=%.6f;left_grip_qw=%.6f;"
            "right_grip_flags=%llu;right_grip_px=%.6f;right_grip_py=%.6f;right_grip_pz=%.6f;right_grip_qx=%.6f;right_grip_qy=%.6f;right_grip_qz=%.6f;right_grip_qw=%.6f;"
            "left_aim_flags=%llu;left_aim_px=%.6f;left_aim_py=%.6f;left_aim_pz=%.6f;left_aim_qx=%.6f;left_aim_qy=%.6f;left_aim_qz=%.6f;left_aim_qw=%.6f;"
            "right_aim_flags=%llu;right_aim_px=%.6f;right_aim_py=%.6f;right_aim_pz=%.6f;right_aim_qx=%.6f;right_aim_qy=%.6f;right_aim_qz=%.6f;right_aim_qw=%.6f;"
            "left_trigger=%.3f;right_trigger=%.3f;left_squeeze=%.3f;right_squeeze=%.3f;"
            "button_a=%d;button_b=%d;button_x=%d;button_y=%d",
            static_cast<long long>(pose->predicted_display_time_ns),
            static_cast<unsigned int>(pose->view_state_flags),
            h.position.x, h.position.y, h.position.z,
            h.orientation.x, h.orientation.y, h.orientation.z, h.orientation.w,
            static_cast<unsigned int>(pose->eye_view_count),
            pose->ipd_m,
            le.position.x, le.position.y, le.position.z,
            le.orientation.x, le.orientation.y, le.orientation.z, le.orientation.w,
            re.position.x, re.position.y, re.position.z,
            re.orientation.x, re.orientation.y, re.orientation.z, re.orientation.w,
            lf.angleLeft, lf.angleRight, lf.angleUp, lf.angleDown,
            rf.angleLeft, rf.angleRight, rf.angleUp, rf.angleDown,
            static_cast<unsigned long long>(pose->left_grip_flags),
            lg.position.x, lg.position.y, lg.position.z,
            lg.orientation.x, lg.orientation.y, lg.orientation.z, lg.orientation.w,
            static_cast<unsigned long long>(pose->right_grip_flags),
            rg.position.x, rg.position.y, rg.position.z,
            rg.orientation.x, rg.orientation.y, rg.orientation.z, rg.orientation.w,
            static_cast<unsigned long long>(pose->left_aim_flags),
            la.position.x, la.position.y, la.position.z,
            la.orientation.x, la.orientation.y, la.orientation.z, la.orientation.w,
            static_cast<unsigned long long>(pose->right_aim_flags),
            ra.position.x, ra.position.y, ra.position.z,
            ra.orientation.x, ra.orientation.y, ra.orientation.z, ra.orientation.w,
            pose->left_trigger, pose->right_trigger,
            pose->left_squeeze, pose->right_squeeze,
            pose->button_a ? 1 : 0, pose->button_b ? 1 : 0,
            pose->button_x ? 1 : 0, pose->button_y ? 1 : 0);
        if (!sendPacket(fd, quest3_teleop::PacketType::kPose, seq++, payload)) {
          break;
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    close(fd);
    logInfo("host bridge disconnected");
  }
}

void startTransport() {
  if (g_transport_running.exchange(true)) {
    return;
  }
  g_transport_thread = std::thread(transportLoop);
}

void stopTransport() {
  if (!g_transport_running.exchange(false)) {
    return;
  }
  if (g_transport_thread.joinable()) {
    g_transport_thread.join();
  }
}

void handleCmd(android_app*, int32_t cmd) {
  switch (cmd) {
    case APP_CMD_RESUME:
      setOpenXrResumed(true);
      break;
    case APP_CMD_PAUSE:
      setOpenXrResumed(false);
      break;
    default:
      break;
  }
}

}  // namespace

void publishHeadPose(const HeadPoseSample& sample) {
  std::lock_guard<std::mutex> lock(g_pose_mutex);
  g_latest_pose = sample;
}

void android_main(android_app* app) {
  app_dummy();
  app->onAppCmd = handleCmd;

  JNIEnv* env = nullptr;
  app->activity->vm->AttachCurrentThread(&env, nullptr);

  logInfo("NativeActivity started");
  startTransport();
  startOpenXr(app->activity->vm, app->activity->clazz);

  while (app->destroyRequested == 0) {
    int events = 0;
    android_poll_source* source = nullptr;
    const int ident = ALooper_pollOnce(50, nullptr, &events, reinterpret_cast<void**>(&source));
    if (source != nullptr) {
      source->process(app, source);
    }
    if (ident == ALOOPER_POLL_ERROR) {
      break;
    }
  }

  stopOpenXr();
  stopTransport();
  app->activity->vm->DetachCurrentThread();
  logInfo("NativeActivity stopped");
}
