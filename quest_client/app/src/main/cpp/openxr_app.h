#pragma once

#include <cstdint>
#include <vector>

#include <android/native_window.h>
#include <jni.h>
#include <openxr/openxr.h>

void startOpenXr(JavaVM* vm, jobject activity);
void stopOpenXr();
void setOpenXrResumed(bool resumed);

struct StereoClearColors {
  float left[4] = {0.0f, 0.34f, 0.62f, 1.0f};
  float right[4] = {0.65f, 0.18f, 0.05f, 1.0f};
};

struct StereoVideoFrame {
  std::uint32_t frame_index = 0;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::vector<std::uint8_t> left_rgba;
  std::vector<std::uint8_t> right_rgba;
};

struct HeadPoseSample {
  std::int64_t predicted_display_time_ns = 0;
  XrPosef pose{};
  XrViewStateFlags view_state_flags = 0;
  std::uint32_t eye_view_count = 0;
  XrPosef left_eye_pose{};
  XrPosef right_eye_pose{};
  XrFovf left_eye_fov{};
  XrFovf right_eye_fov{};
  float ipd_m = 0.0f;
  XrPosef left_grip_pose{};
  XrPosef right_grip_pose{};
  XrPosef left_aim_pose{};
  XrPosef right_aim_pose{};
  XrSpaceLocationFlags left_grip_flags = 0;
  XrSpaceLocationFlags right_grip_flags = 0;
  XrSpaceLocationFlags left_aim_flags = 0;
  XrSpaceLocationFlags right_aim_flags = 0;
  float left_trigger = 0.0f;
  float right_trigger = 0.0f;
  float left_squeeze = 0.0f;
  float right_squeeze = 0.0f;
  bool button_a = false;
  bool button_b = false;
  bool button_x = false;
  bool button_y = false;
};

struct StereoCodecSurfaces {
  ANativeWindow* left_window = nullptr;
  ANativeWindow* right_window = nullptr;
  std::uint32_t generation = 0;
};

void publishHeadPose(const HeadPoseSample& sample);
void publishStereoClearColors(const StereoClearColors& colors);
void publishStereoVideoFrame(StereoVideoFrame frame);
bool acquireStereoCodecSurfaces(StereoCodecSurfaces* surfaces);
void publishStereoCodecFrame(std::uint32_t frame_index);
