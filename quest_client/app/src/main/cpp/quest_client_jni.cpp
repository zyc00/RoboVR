#include <arpa/inet.h>
#include <android/log.h>
#include <jni.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>

#include "openxr_app.h"
#include "quest3_teleop_protocol.h"

namespace {

constexpr char kLogTag[] = "RoboVR";

std::atomic<bool> g_running{false};
std::thread g_thread;

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

int connectToHost(const std::string& host, int port) {
  int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    return -1;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<std::uint16_t>(port));
  if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
    close(fd);
    return -1;
  }

  if (connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    close(fd);
    return -1;
  }
  return fd;
}

void clientLoop(std::string host, int port) {
  std::uint64_t seq = 1;
  while (g_running) {
    int fd = connectToHost(host, port);
    if (fd < 0) {
      logInfo("host connection failed; retrying");
      std::this_thread::sleep_for(std::chrono::seconds(1));
      continue;
    }

    logInfo("connected to host bridge");
    sendPacket(fd, quest3_teleop::PacketType::kHello, seq++, "client=quest;protocol=1");

    while (g_running) {
      const bool ok = sendPacket(fd, quest3_teleop::PacketType::kHeartbeat, seq++, "quest_alive=1");
      if (!ok) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    close(fd);
    logInfo("host bridge disconnected");
  }
}

}  // namespace

extern "C" JNIEXPORT void JNICALL
Java_com_yuchen_robovr_MainActivity_nativeStart(JNIEnv* env, jclass, jobject activity, jstring host, jint port) {
  const char* host_chars = env->GetStringUTFChars(host, nullptr);
  std::string host_string = host_chars != nullptr ? host_chars : "127.0.0.1";
  if (host_chars != nullptr) {
    env->ReleaseStringUTFChars(host, host_chars);
  }

  JavaVM* vm = nullptr;
  env->GetJavaVM(&vm);
  startOpenXr(vm, activity);

  if (g_running.exchange(true)) {
    return;
  }
  g_thread = std::thread(clientLoop, std::move(host_string), static_cast<int>(port));
}

extern "C" JNIEXPORT void JNICALL
Java_com_yuchen_robovr_MainActivity_nativeStop(JNIEnv*, jclass) {
  if (!g_running.exchange(false)) {
    return;
  }
  if (g_thread.joinable()) {
    g_thread.join();
  }
  stopOpenXr();
}

extern "C" JNIEXPORT void JNICALL
Java_com_yuchen_robovr_MainActivity_nativeOnResume(JNIEnv*, jclass) {
  setOpenXrResumed(true);
}

extern "C" JNIEXPORT void JNICALL
Java_com_yuchen_robovr_MainActivity_nativeOnPause(JNIEnv*, jclass) {
  setOpenXrResumed(false);
}
