#include <android/log.h>
#include <android_native_app_glue.h>
#include <unistd.h>

#include <atomic>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>

#include "openxr_app.h"
#include "quest3_teleop_protocol.h"
#include "transport.h"
#include "video_decoder.h"

namespace {

constexpr char kLogTag[] = "RoboVR";
constexpr std::uint32_t kMaxHostPayloadSize = 1024 * 1024;

std::atomic<bool> g_transport_running{false};
std::thread g_transport_thread;
std::mutex g_pose_mutex;
std::optional<HeadPoseSample> g_latest_pose;
VideoDecoderRouter g_video_decoder;

void logInfo(const char* message) {
  __android_log_print(ANDROID_LOG_INFO, kLogTag, "%s", message);
}

std::optional<HeadPoseSample> takeLatestPose() {
  std::lock_guard<std::mutex> lock(g_pose_mutex);
  std::optional<HeadPoseSample> pose = g_latest_pose;
  g_latest_pose.reset();
  return pose;
}

std::string formatPosePayload(const HeadPoseSample& sample) {
  char payload[4096];
  const auto& h = sample.pose;
  const auto& le = sample.left_eye_pose;
  const auto& re = sample.right_eye_pose;
  const auto& lf = sample.left_eye_fov;
  const auto& rf = sample.right_eye_fov;
  const auto& lg = sample.left_grip_pose;
  const auto& rg = sample.right_grip_pose;
  const auto& la = sample.left_aim_pose;
  const auto& ra = sample.right_aim_pose;
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
      static_cast<long long>(sample.predicted_display_time_ns),
      static_cast<unsigned int>(sample.view_state_flags),
      h.position.x, h.position.y, h.position.z,
      h.orientation.x, h.orientation.y, h.orientation.z, h.orientation.w,
      static_cast<unsigned int>(sample.eye_view_count),
      sample.ipd_m,
      le.position.x, le.position.y, le.position.z,
      le.orientation.x, le.orientation.y, le.orientation.z, le.orientation.w,
      re.position.x, re.position.y, re.position.z,
      re.orientation.x, re.orientation.y, re.orientation.z, re.orientation.w,
      lf.angleLeft, lf.angleRight, lf.angleUp, lf.angleDown,
      rf.angleLeft, rf.angleRight, rf.angleUp, rf.angleDown,
      static_cast<unsigned long long>(sample.left_grip_flags),
      lg.position.x, lg.position.y, lg.position.z,
      lg.orientation.x, lg.orientation.y, lg.orientation.z, lg.orientation.w,
      static_cast<unsigned long long>(sample.right_grip_flags),
      rg.position.x, rg.position.y, rg.position.z,
      rg.orientation.x, rg.orientation.y, rg.orientation.z, rg.orientation.w,
      static_cast<unsigned long long>(sample.left_aim_flags),
      la.position.x, la.position.y, la.position.z,
      la.orientation.x, la.orientation.y, la.orientation.z, la.orientation.w,
      static_cast<unsigned long long>(sample.right_aim_flags),
      ra.position.x, ra.position.y, ra.position.z,
      ra.orientation.x, ra.orientation.y, ra.orientation.z, ra.orientation.w,
      sample.left_trigger, sample.right_trigger,
      sample.left_squeeze, sample.right_squeeze,
      sample.button_a ? 1 : 0, sample.button_b ? 1 : 0,
      sample.button_x ? 1 : 0, sample.button_y ? 1 : 0);
  return payload;
}

float payloadFloat(const std::string& payload, std::string_view key, float fallback) {
  return quest3_teleop::parseFloatValue(quest3_teleop::findKeyValue(payload, key), fallback);
}

void applyColorPayload(const std::string& payload, StereoClearColors* colors) {
  colors->left[0] = payloadFloat(payload, "left_r", colors->left[0]);
  colors->left[1] = payloadFloat(payload, "left_g", colors->left[1]);
  colors->left[2] = payloadFloat(payload, "left_b", colors->left[2]);
  colors->left[3] = 1.0f;
  colors->right[0] = payloadFloat(payload, "right_r", colors->right[0]);
  colors->right[1] = payloadFloat(payload, "right_g", colors->right[1]);
  colors->right[2] = payloadFloat(payload, "right_b", colors->right[2]);
  colors->right[3] = 1.0f;
}

bool sendVideoDecodeAcks(int fd, std::uint64_t* seq) {
  for (const VideoDecodeAck& ack : g_video_decoder.takeAcks()) {
    char payload[256];
    std::snprintf(payload, sizeof(payload),
                  "kind=video_ack;frame_index=%llu;quest_receive_to_publish_ms=%.3f;surface=%d",
                  static_cast<unsigned long long>(ack.frame_index),
                  ack.quest_receive_to_publish_ms,
                  ack.surface_output ? 1 : 0);
    if (!robovr::sendPacket(fd, quest3_teleop::PacketType::kStats, (*seq)++, payload)) {
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
  applyColorPayload(payload, &colors);
  publishStereoClearColors(colors);

  static std::uint64_t video_count = 0;
  if (video_count % 30 == 0) {
    const std::string frame(quest3_teleop::findKeyValue(payload, "frame"));
    __android_log_print(ANDROID_LOG_INFO, kLogTag, "rx video frame=%s count=%llu",
                        frame.empty() ? "?" : frame.c_str(),
                        static_cast<unsigned long long>(video_count));
  }
  ++video_count;
}

bool receiveHostPackets(int fd, std::uint64_t* seq) {
  while (robovr::hasReadableData(fd)) {
    std::array<std::uint8_t, quest3_teleop::kPacketHeaderSize> header_bytes{};
    if (!robovr::readExact(fd, header_bytes.data(), header_bytes.size())) {
      return false;
    }
    const quest3_teleop::PacketHeader header = quest3_teleop::unpackHeader(header_bytes.data());
    if (header.magic != quest3_teleop::kProtocolMagic ||
        header.version != quest3_teleop::kProtocolVersion ||
        !quest3_teleop::isKnownPacketType(header.type) ||
        header.payload_size > kMaxHostPayloadSize) {
      return false;
    }

    std::string payload(header.payload_size, '\0');
    if (!payload.empty() && !robovr::readExact(fd, payload.data(), payload.size())) {
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
    int fd = robovr::connectTcp("127.0.0.1", 7777);
    if (fd < 0) {
      logInfo("host connection failed; retrying");
      std::this_thread::sleep_for(std::chrono::seconds(1));
      continue;
    }

    logInfo("connected to host bridge");
    robovr::sendPacket(fd, quest3_teleop::PacketType::kHello, seq++, "client=quest;protocol=1");
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
        if (!robovr::sendPacket(fd, quest3_teleop::PacketType::kHeartbeat, seq++, "quest_alive=1")) {
          break;
        }
        last_heartbeat = now;
      }

      std::optional<HeadPoseSample> pose = takeLatestPose();
      if (pose.has_value()) {
        const std::string payload = formatPosePayload(*pose);
        if (!robovr::sendPacket(fd, quest3_teleop::PacketType::kPose, seq++, payload)) {
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
