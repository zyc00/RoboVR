#include "openxr_app.h"

#include <EGL/egl.h>
#include <GLES3/gl3.h>
#include <android/native_window.h>
#include <android/native_window_jni.h>
#include <android/log.h>
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

constexpr char kLogTag[] = "RoboVR";
constexpr GLenum kTextureExternalOes = 0x8D65;
constexpr float kRequestedDisplayRefreshRateHz = 72.0f;

#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, kLogTag, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN, kLogTag, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, kLogTag, __VA_ARGS__)

std::atomic<bool> g_running{false};
std::atomic<bool> g_resumed{false};
std::thread g_thread;
JavaVM* g_vm = nullptr;
jobject g_activity = nullptr;
std::mutex g_video_mutex;
StereoClearColors g_stereo_colors;
std::optional<StereoVideoFrame> g_stereo_frame;
std::mutex g_surface_mutex;
StereoCodecSurfaces g_codec_surfaces;
std::uint32_t g_codec_surface_generation = 0;
std::atomic<std::uint32_t> g_latest_codec_frame{UINT32_MAX};

bool xrOk(XrResult result, const char* call) {
  if (XR_SUCCEEDED(result)) {
    return true;
  }
  LOGE("%s failed: XrResult=%d", call, result);
  return false;
}

bool isInstanceExtensionSupported(const char* extension_name) {
  uint32_t extension_count = 0;
  if (XR_FAILED(xrEnumerateInstanceExtensionProperties(nullptr, 0, &extension_count, nullptr))) {
    return false;
  }
  std::vector<XrExtensionProperties> extensions(
      extension_count, XrExtensionProperties{XR_TYPE_EXTENSION_PROPERTIES});
  if (XR_FAILED(xrEnumerateInstanceExtensionProperties(
          nullptr, extension_count, &extension_count, extensions.data()))) {
    return false;
  }
  for (const XrExtensionProperties& extension : extensions) {
    if (std::strcmp(extension.extensionName, extension_name) == 0) {
      return true;
    }
  }
  return false;
}

struct Swapchain {
  XrSwapchain handle = XR_NULL_HANDLE;
  int32_t width = 0;
  int32_t height = 0;
  std::vector<XrSwapchainImageOpenGLESKHR> images;
};

struct ControllerState {
  XrPosef grip_pose{};
  XrPosef aim_pose{};
  XrSpaceLocationFlags grip_flags = 0;
  XrSpaceLocationFlags aim_flags = 0;
  float trigger = 0.0f;
  float squeeze = 0.0f;
  bool primary_button = false;
  bool secondary_button = false;
};

class OpenXrApp {
 public:
  OpenXrApp(JavaVM* vm, jobject activity) : vm_(vm), activity_(activity) {}

  void run() {
    JNIEnv* env = nullptr;
    if (vm_->AttachCurrentThread(&env, nullptr) != JNI_OK) {
      LOGE("failed to attach OpenXR thread to JVM");
      return;
    }

    if (!initialize(env)) {
      cleanup();
      vm_->DetachCurrentThread();
      return;
    }

    LOGI("OpenXR initialized");
    while (g_running && !exit_render_loop_) {
      pollEvents();
      if (!session_running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        continue;
      }
      renderFrame();
    }

    cleanup();
    vm_->DetachCurrentThread();
    LOGI("OpenXR stopped");
  }

 private:
  bool initialize(JNIEnv* env) {
    PFN_xrInitializeLoaderKHR initialize_loader = nullptr;
    const XrResult loader_proc_result =
        xrGetInstanceProcAddr(XR_NULL_HANDLE, "xrInitializeLoaderKHR",
                              reinterpret_cast<PFN_xrVoidFunction*>(&initialize_loader));
    if (XR_SUCCEEDED(loader_proc_result) && initialize_loader != nullptr) {
      XrLoaderInitInfoAndroidKHR loader_init{XR_TYPE_LOADER_INIT_INFO_ANDROID_KHR};
      loader_init.applicationVM = vm_;
      loader_init.applicationContext = activity_;
      xrOk(initialize_loader(reinterpret_cast<const XrLoaderInitInfoBaseHeaderKHR*>(&loader_init)),
           "xrInitializeLoaderKHR");
    }

    std::vector<const char*> extensions = {
        XR_KHR_ANDROID_CREATE_INSTANCE_EXTENSION_NAME,
        XR_KHR_OPENGL_ES_ENABLE_EXTENSION_NAME,
    };
    display_refresh_rate_extension_enabled_ =
        isInstanceExtensionSupported(XR_FB_DISPLAY_REFRESH_RATE_EXTENSION_NAME);
    if (display_refresh_rate_extension_enabled_) {
      extensions.push_back(XR_FB_DISPLAY_REFRESH_RATE_EXTENSION_NAME);
      LOGI("OpenXR extension enabled: %s", XR_FB_DISPLAY_REFRESH_RATE_EXTENSION_NAME);
    } else {
      LOGW("OpenXR extension unavailable: %s", XR_FB_DISPLAY_REFRESH_RATE_EXTENSION_NAME);
    }

    XrInstanceCreateInfoAndroidKHR android_info{XR_TYPE_INSTANCE_CREATE_INFO_ANDROID_KHR};
    android_info.applicationVM = vm_;
    android_info.applicationActivity = activity_;

    XrInstanceCreateInfo create_info{XR_TYPE_INSTANCE_CREATE_INFO};
    create_info.next = &android_info;
    std::strncpy(create_info.applicationInfo.applicationName, "RoboVR", XR_MAX_APPLICATION_NAME_SIZE - 1);
    create_info.applicationInfo.applicationVersion = 1;
    std::strncpy(create_info.applicationInfo.engineName, "RoboVRNative", XR_MAX_ENGINE_NAME_SIZE - 1);
    create_info.applicationInfo.engineVersion = 1;
    create_info.applicationInfo.apiVersion = XR_API_VERSION_1_0;
    create_info.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
    create_info.enabledExtensionNames = extensions.data();

    if (!xrOk(xrCreateInstance(&create_info, &instance_), "xrCreateInstance")) {
      return false;
    }

    XrInstanceProperties props{XR_TYPE_INSTANCE_PROPERTIES};
    if (xrOk(xrGetInstanceProperties(instance_, &props), "xrGetInstanceProperties")) {
      LOGI("OpenXR runtime: %s", props.runtimeName);
    }

    XrSystemGetInfo system_info{XR_TYPE_SYSTEM_GET_INFO};
    system_info.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    if (!xrOk(xrGetSystem(instance_, &system_info, &system_id_), "xrGetSystem")) {
      return false;
    }

    if (!createEglContext()) {
      return false;
    }

    PFN_xrGetOpenGLESGraphicsRequirementsKHR get_gles_requirements = nullptr;
    if (!xrOk(xrGetInstanceProcAddr(instance_, "xrGetOpenGLESGraphicsRequirementsKHR",
                                    reinterpret_cast<PFN_xrVoidFunction*>(&get_gles_requirements)),
              "xrGetOpenGLESGraphicsRequirementsKHR proc")) {
      return false;
    }
    XrGraphicsRequirementsOpenGLESKHR gles_requirements{XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_ES_KHR};
    if (!xrOk(get_gles_requirements(instance_, system_id_, &gles_requirements),
              "xrGetOpenGLESGraphicsRequirementsKHR")) {
      return false;
    }

    XrGraphicsBindingOpenGLESAndroidKHR graphics_binding{XR_TYPE_GRAPHICS_BINDING_OPENGL_ES_ANDROID_KHR};
    graphics_binding.display = egl_display_;
    graphics_binding.config = egl_config_;
    graphics_binding.context = egl_context_;

    XrSessionCreateInfo session_info{XR_TYPE_SESSION_CREATE_INFO};
    session_info.next = &graphics_binding;
    session_info.systemId = system_id_;
    if (!xrOk(xrCreateSession(instance_, &session_info, &session_), "xrCreateSession")) {
      return false;
    }
    initializeDisplayRefreshRateExtension();

    XrReferenceSpaceCreateInfo space_info{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    space_info.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
    space_info.poseInReferenceSpace.orientation.w = 1.0f;
    if (!xrOk(xrCreateReferenceSpace(session_, &space_info, &space_), "xrCreateReferenceSpace")) {
      return false;
    }

    if (!createActions()) {
      return false;
    }

    if (!createSwapchains()) {
      return false;
    }

    glGenFramebuffers(1, &framebuffer_);
    if (framebuffer_ == 0) {
      return false;
    }
    glGenTextures(2, video_textures_.data());
    if (!createVideoProgram() || !createExternalVideoProgram() || !createWaitingProgram()) {
      return false;
    }
    if (!createCodecSurfaceResources(env)) {
      LOGW("codec SurfaceTexture path unavailable; using CPU video fallback");
    }
    return true;
  }

  bool createEglContext() {
    egl_display_ = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (egl_display_ == EGL_NO_DISPLAY) {
      LOGE("eglGetDisplay failed");
      return false;
    }
    if (eglInitialize(egl_display_, nullptr, nullptr) != EGL_TRUE) {
      LOGE("eglInitialize failed");
      return false;
    }

    const EGLint config_attribs[] = {
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT,
        EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
        EGL_RED_SIZE, 8,
        EGL_GREEN_SIZE, 8,
        EGL_BLUE_SIZE, 8,
        EGL_ALPHA_SIZE, 8,
        EGL_DEPTH_SIZE, 0,
        EGL_NONE,
    };
    EGLint num_configs = 0;
    if (eglChooseConfig(egl_display_, config_attribs, &egl_config_, 1, &num_configs) != EGL_TRUE ||
        num_configs < 1) {
      LOGE("eglChooseConfig failed");
      return false;
    }

    const EGLint context_attribs[] = {EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE};
    egl_context_ = eglCreateContext(egl_display_, egl_config_, EGL_NO_CONTEXT, context_attribs);
    if (egl_context_ == EGL_NO_CONTEXT) {
      LOGE("eglCreateContext failed");
      return false;
    }

    const EGLint surface_attribs[] = {EGL_WIDTH, 16, EGL_HEIGHT, 16, EGL_NONE};
    egl_surface_ = eglCreatePbufferSurface(egl_display_, egl_config_, surface_attribs);
    if (egl_surface_ == EGL_NO_SURFACE) {
      LOGE("eglCreatePbufferSurface failed");
      return false;
    }
    if (eglMakeCurrent(egl_display_, egl_surface_, egl_surface_, egl_context_) != EGL_TRUE) {
      LOGE("eglMakeCurrent failed");
      return false;
    }

    LOGI("GLES renderer: %s", reinterpret_cast<const char*>(glGetString(GL_RENDERER)));
    return true;
  }

  bool stringToPath(const char* path_string, XrPath* path) {
    return xrOk(xrStringToPath(instance_, path_string, path), path_string);
  }

  bool createActions() {
    if (!stringToPath("/user/hand/left", &hand_paths_[0]) ||
        !stringToPath("/user/hand/right", &hand_paths_[1])) {
      return false;
    }

    XrActionSetCreateInfo action_set_info{XR_TYPE_ACTION_SET_CREATE_INFO};
    std::strncpy(action_set_info.actionSetName, "teleop", XR_MAX_ACTION_SET_NAME_SIZE - 1);
    std::strncpy(action_set_info.localizedActionSetName, "Teleop", XR_MAX_LOCALIZED_ACTION_SET_NAME_SIZE - 1);
    action_set_info.priority = 0;
    if (!xrOk(xrCreateActionSet(instance_, &action_set_info, &action_set_), "xrCreateActionSet")) {
      return false;
    }

    if (!createPoseAction("grip_pose", "Grip Pose", &grip_pose_action_) ||
        !createPoseAction("aim_pose", "Aim Pose", &aim_pose_action_) ||
        !createFloatAction("trigger", "Trigger", &trigger_action_) ||
        !createFloatAction("squeeze", "Squeeze", &squeeze_action_) ||
        !createBoolAction("primary_button", "Primary Button", &primary_button_action_) ||
        !createBoolAction("secondary_button", "Secondary Button", &secondary_button_action_)) {
      return false;
    }

    XrPath touch_profile = XR_NULL_PATH;
    if (!stringToPath("/interaction_profiles/oculus/touch_controller", &touch_profile)) {
      return false;
    }

    std::vector<XrActionSuggestedBinding> bindings;
    addBinding(bindings, grip_pose_action_, "/user/hand/left/input/grip/pose");
    addBinding(bindings, grip_pose_action_, "/user/hand/right/input/grip/pose");
    addBinding(bindings, aim_pose_action_, "/user/hand/left/input/aim/pose");
    addBinding(bindings, aim_pose_action_, "/user/hand/right/input/aim/pose");
    addBinding(bindings, trigger_action_, "/user/hand/left/input/trigger/value");
    addBinding(bindings, trigger_action_, "/user/hand/right/input/trigger/value");
    addBinding(bindings, squeeze_action_, "/user/hand/left/input/squeeze/value");
    addBinding(bindings, squeeze_action_, "/user/hand/right/input/squeeze/value");
    addBinding(bindings, primary_button_action_, "/user/hand/left/input/x/click");
    addBinding(bindings, primary_button_action_, "/user/hand/right/input/a/click");
    addBinding(bindings, secondary_button_action_, "/user/hand/left/input/y/click");
    addBinding(bindings, secondary_button_action_, "/user/hand/right/input/b/click");

    XrInteractionProfileSuggestedBinding suggested{XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
    suggested.interactionProfile = touch_profile;
    suggested.suggestedBindings = bindings.data();
    suggested.countSuggestedBindings = static_cast<uint32_t>(bindings.size());
    if (!xrOk(xrSuggestInteractionProfileBindings(instance_, &suggested),
              "xrSuggestInteractionProfileBindings")) {
      return false;
    }

    XrActionSpaceCreateInfo action_space_info{XR_TYPE_ACTION_SPACE_CREATE_INFO};
    action_space_info.poseInActionSpace.orientation.w = 1.0f;
    action_space_info.action = grip_pose_action_;
    action_space_info.subactionPath = hand_paths_[0];
    if (!xrOk(xrCreateActionSpace(session_, &action_space_info, &left_grip_space_),
              "xrCreateActionSpace left grip")) {
      return false;
    }
    action_space_info.subactionPath = hand_paths_[1];
    if (!xrOk(xrCreateActionSpace(session_, &action_space_info, &right_grip_space_),
              "xrCreateActionSpace right grip")) {
      return false;
    }
    action_space_info.action = aim_pose_action_;
    action_space_info.subactionPath = hand_paths_[0];
    if (!xrOk(xrCreateActionSpace(session_, &action_space_info, &left_aim_space_),
              "xrCreateActionSpace left aim")) {
      return false;
    }
    action_space_info.subactionPath = hand_paths_[1];
    if (!xrOk(xrCreateActionSpace(session_, &action_space_info, &right_aim_space_),
              "xrCreateActionSpace right aim")) {
      return false;
    }

    XrSessionActionSetsAttachInfo attach_info{XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO};
    attach_info.countActionSets = 1;
    attach_info.actionSets = &action_set_;
    if (!xrOk(xrAttachSessionActionSets(session_, &attach_info), "xrAttachSessionActionSets")) {
      return false;
    }

    LOGI("OpenXR controller actions initialized");
    return true;
  }

  bool createPoseAction(const char* name, const char* localized_name, XrAction* action) {
    return createAction(name, localized_name, XR_ACTION_TYPE_POSE_INPUT, action);
  }

  bool createFloatAction(const char* name, const char* localized_name, XrAction* action) {
    return createAction(name, localized_name, XR_ACTION_TYPE_FLOAT_INPUT, action);
  }

  bool createBoolAction(const char* name, const char* localized_name, XrAction* action) {
    return createAction(name, localized_name, XR_ACTION_TYPE_BOOLEAN_INPUT, action);
  }

  bool createAction(const char* name, const char* localized_name, XrActionType type, XrAction* action) {
    XrActionCreateInfo info{XR_TYPE_ACTION_CREATE_INFO};
    info.actionType = type;
    std::strncpy(info.actionName, name, XR_MAX_ACTION_NAME_SIZE - 1);
    std::strncpy(info.localizedActionName, localized_name, XR_MAX_LOCALIZED_ACTION_NAME_SIZE - 1);
    info.countSubactionPaths = static_cast<uint32_t>(hand_paths_.size());
    info.subactionPaths = hand_paths_.data();
    return xrOk(xrCreateAction(action_set_, &info, action), name);
  }

  void addBinding(std::vector<XrActionSuggestedBinding>& bindings, XrAction action, const char* path_string) {
    XrPath path = XR_NULL_PATH;
    if (XR_SUCCEEDED(xrStringToPath(instance_, path_string, &path))) {
      bindings.push_back({action, path});
    } else {
      LOGW("failed to resolve binding path: %s", path_string);
    }
  }

  bool createSwapchains() {
    uint32_t view_count = 0;
    if (!xrOk(xrEnumerateViewConfigurationViews(instance_, system_id_,
                                                XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 0, &view_count, nullptr),
              "xrEnumerateViewConfigurationViews count") ||
        view_count == 0) {
      return false;
    }

    view_configs_.resize(view_count, {XR_TYPE_VIEW_CONFIGURATION_VIEW});
    views_.resize(view_count, {XR_TYPE_VIEW});
    projection_views_.resize(view_count, {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW});
    if (!xrOk(xrEnumerateViewConfigurationViews(instance_, system_id_,
                                                XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, view_count, &view_count,
                                                view_configs_.data()),
              "xrEnumerateViewConfigurationViews")) {
      return false;
    }

    uint32_t format_count = 0;
    if (!xrOk(xrEnumerateSwapchainFormats(session_, 0, &format_count, nullptr),
              "xrEnumerateSwapchainFormats count") ||
        format_count == 0) {
      return false;
    }
    std::vector<int64_t> formats(format_count);
    if (!xrOk(xrEnumerateSwapchainFormats(session_, format_count, &format_count, formats.data()),
              "xrEnumerateSwapchainFormats")) {
      return false;
    }

    const std::array<int64_t, 3> preferred = {
        GL_SRGB8_ALPHA8,
        GL_RGBA8,
        GL_RGB10_A2,
    };
    color_format_ = formats.front();
    for (const int64_t candidate : preferred) {
      if (std::find(formats.begin(), formats.end(), candidate) != formats.end()) {
        color_format_ = candidate;
        break;
      }
    }

    swapchains_.resize(view_count);
    for (uint32_t i = 0; i < view_count; ++i) {
      auto& swapchain = swapchains_[i];
      swapchain.width = static_cast<int32_t>(view_configs_[i].recommendedImageRectWidth);
      swapchain.height = static_cast<int32_t>(view_configs_[i].recommendedImageRectHeight);

      XrSwapchainCreateInfo swapchain_info{XR_TYPE_SWAPCHAIN_CREATE_INFO};
      swapchain_info.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT;
      swapchain_info.format = color_format_;
      swapchain_info.sampleCount = view_configs_[i].recommendedSwapchainSampleCount;
      swapchain_info.width = view_configs_[i].recommendedImageRectWidth;
      swapchain_info.height = view_configs_[i].recommendedImageRectHeight;
      swapchain_info.faceCount = 1;
      swapchain_info.arraySize = 1;
      swapchain_info.mipCount = 1;
      if (!xrOk(xrCreateSwapchain(session_, &swapchain_info, &swapchain.handle), "xrCreateSwapchain")) {
        return false;
      }

      uint32_t image_count = 0;
      if (!xrOk(xrEnumerateSwapchainImages(swapchain.handle, 0, &image_count, nullptr),
                "xrEnumerateSwapchainImages count")) {
        return false;
      }
      swapchain.images.resize(image_count, {XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_ES_KHR});
      if (!xrOk(xrEnumerateSwapchainImages(
                    swapchain.handle, image_count, &image_count,
                    reinterpret_cast<XrSwapchainImageBaseHeader*>(swapchain.images.data())),
                "xrEnumerateSwapchainImages")) {
        return false;
      }
    }

    LOGI("created %zu OpenXR swapchains, format=%lld", swapchains_.size(), static_cast<long long>(color_format_));
    return true;
  }

  void pollEvents() {
    XrEventDataBuffer event{XR_TYPE_EVENT_DATA_BUFFER};
    while (xrPollEvent(instance_, &event) == XR_SUCCESS) {
      switch (event.type) {
        case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED: {
          const auto* changed = reinterpret_cast<const XrEventDataSessionStateChanged*>(&event);
          session_state_ = changed->state;
          LOGI("OpenXR session state=%d", session_state_);
          if (session_state_ == XR_SESSION_STATE_READY) {
            XrSessionBeginInfo begin_info{XR_TYPE_SESSION_BEGIN_INFO};
            begin_info.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
            session_running_ = xrOk(xrBeginSession(session_, &begin_info), "xrBeginSession");
            if (session_running_) {
              requestDisplayRefreshRate(kRequestedDisplayRefreshRateHz);
            }
          } else if (session_state_ == XR_SESSION_STATE_STOPPING) {
            session_running_ = false;
            xrOk(xrEndSession(session_), "xrEndSession");
          } else if (session_state_ == XR_SESSION_STATE_EXITING ||
                     session_state_ == XR_SESSION_STATE_LOSS_PENDING) {
            exit_render_loop_ = true;
          }
          break;
        }
        case XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING:
          exit_render_loop_ = true;
          break;
        case XR_TYPE_EVENT_DATA_DISPLAY_REFRESH_RATE_CHANGED_FB: {
          const auto* changed =
              reinterpret_cast<const XrEventDataDisplayRefreshRateChangedFB*>(&event);
          LOGI("OpenXR display refresh changed: %.1f -> %.1f Hz",
               changed->fromDisplayRefreshRate, changed->toDisplayRefreshRate);
          break;
        }
        default:
          break;
      }
      event = {XR_TYPE_EVENT_DATA_BUFFER};
    }
  }

  void renderFrame() {
    XrFrameWaitInfo wait_info{XR_TYPE_FRAME_WAIT_INFO};
    XrFrameState frame_state{XR_TYPE_FRAME_STATE};
    if (!xrOk(xrWaitFrame(session_, &wait_info, &frame_state), "xrWaitFrame")) {
      return;
    }

    XrFrameBeginInfo begin_info{XR_TYPE_FRAME_BEGIN_INFO};
    if (!xrOk(xrBeginFrame(session_, &begin_info), "xrBeginFrame")) {
      return;
    }

    std::array<XrCompositionLayerBaseHeader*, 1> layers{};
    uint32_t layer_count = 0;
    XrCompositionLayerProjection projection_layer{XR_TYPE_COMPOSITION_LAYER_PROJECTION};
    projection_layer.space = space_;

    if (frame_state.shouldRender == XR_TRUE) {
      const auto left_controller = locateController(0, frame_state.predictedDisplayTime);
      const auto right_controller = locateController(1, frame_state.predictedDisplayTime);

      XrViewState view_state{XR_TYPE_VIEW_STATE};
      uint32_t view_count = 0;
      XrViewLocateInfo locate_info{XR_TYPE_VIEW_LOCATE_INFO};
      locate_info.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
      locate_info.displayTime = frame_state.predictedDisplayTime;
      locate_info.space = space_;
      if (xrOk(xrLocateViews(session_, &locate_info, &view_state, static_cast<uint32_t>(views_.size()), &view_count,
                             views_.data()),
               "xrLocateViews")) {
        if (view_count > 0) {
          HeadPoseSample sample{};
          sample.predicted_display_time_ns = frame_state.predictedDisplayTime;
          sample.view_state_flags = view_state.viewStateFlags;
          sample.eye_view_count = view_count;
          sample.pose = views_[0].pose;
          sample.left_eye_pose = views_[0].pose;
          sample.left_eye_fov = views_[0].fov;
          if (view_count >= 2) {
            sample.right_eye_pose = views_[1].pose;
            sample.right_eye_fov = views_[1].fov;
            sample.pose.position.x = 0.5f * (views_[0].pose.position.x + views_[1].pose.position.x);
            sample.pose.position.y = 0.5f * (views_[0].pose.position.y + views_[1].pose.position.y);
            sample.pose.position.z = 0.5f * (views_[0].pose.position.z + views_[1].pose.position.z);
            const float dx = views_[1].pose.position.x - views_[0].pose.position.x;
            const float dy = views_[1].pose.position.y - views_[0].pose.position.y;
            const float dz = views_[1].pose.position.z - views_[0].pose.position.z;
            sample.ipd_m = std::sqrt(dx * dx + dy * dy + dz * dz);
          }
          sample.left_grip_pose = left_controller.grip_pose;
          sample.right_grip_pose = right_controller.grip_pose;
          sample.left_aim_pose = left_controller.aim_pose;
          sample.right_aim_pose = right_controller.aim_pose;
          sample.left_grip_flags = left_controller.grip_flags;
          sample.right_grip_flags = right_controller.grip_flags;
          sample.left_aim_flags = left_controller.aim_flags;
          sample.right_aim_flags = right_controller.aim_flags;
          sample.left_trigger = left_controller.trigger;
          sample.right_trigger = right_controller.trigger;
          sample.left_squeeze = left_controller.squeeze;
          sample.right_squeeze = right_controller.squeeze;
          sample.button_x = left_controller.primary_button;
          sample.button_y = left_controller.secondary_button;
          sample.button_a = right_controller.primary_button;
          sample.button_b = right_controller.secondary_button;
          publishHeadPose(sample);
        }
        for (uint32_t i = 0; i < view_count; ++i) {
          renderView(i);
          projection_views_[i].pose = views_[i].pose;
          projection_views_[i].fov = views_[i].fov;
          projection_views_[i].subImage.swapchain = swapchains_[i].handle;
          projection_views_[i].subImage.imageRect.offset = {0, 0};
          projection_views_[i].subImage.imageRect.extent = {swapchains_[i].width, swapchains_[i].height};
        }
        projection_layer.viewCount = view_count;
        projection_layer.views = projection_views_.data();
        layers[0] = reinterpret_cast<XrCompositionLayerBaseHeader*>(&projection_layer);
        layer_count = 1;
      }
    }

    XrFrameEndInfo end_info{XR_TYPE_FRAME_END_INFO};
    end_info.displayTime = frame_state.predictedDisplayTime;
    end_info.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    end_info.layerCount = layer_count;
    end_info.layers = layers.data();
    xrOk(xrEndFrame(session_, &end_info), "xrEndFrame");
  }

  ControllerState locateController(uint32_t hand_index, XrTime display_time) {
    ControllerState state{};
    syncActions();
    const XrPath hand_path = hand_paths_[hand_index];

    XrActionStateGetInfo get_info{XR_TYPE_ACTION_STATE_GET_INFO};
    get_info.subactionPath = hand_path;

    get_info.action = trigger_action_;
    XrActionStateFloat trigger{XR_TYPE_ACTION_STATE_FLOAT};
    if (XR_SUCCEEDED(xrGetActionStateFloat(session_, &get_info, &trigger)) && trigger.isActive == XR_TRUE) {
      state.trigger = trigger.currentState;
    }

    get_info.action = squeeze_action_;
    XrActionStateFloat squeeze{XR_TYPE_ACTION_STATE_FLOAT};
    if (XR_SUCCEEDED(xrGetActionStateFloat(session_, &get_info, &squeeze)) && squeeze.isActive == XR_TRUE) {
      state.squeeze = squeeze.currentState;
    }

    get_info.action = primary_button_action_;
    XrActionStateBoolean primary{XR_TYPE_ACTION_STATE_BOOLEAN};
    if (XR_SUCCEEDED(xrGetActionStateBoolean(session_, &get_info, &primary)) && primary.isActive == XR_TRUE) {
      state.primary_button = primary.currentState == XR_TRUE;
    }

    get_info.action = secondary_button_action_;
    XrActionStateBoolean secondary{XR_TYPE_ACTION_STATE_BOOLEAN};
    if (XR_SUCCEEDED(xrGetActionStateBoolean(session_, &get_info, &secondary)) && secondary.isActive == XR_TRUE) {
      state.secondary_button = secondary.currentState == XR_TRUE;
    }

    locateSpace(hand_index == 0 ? left_grip_space_ : right_grip_space_, display_time, &state.grip_pose, &state.grip_flags);
    locateSpace(hand_index == 0 ? left_aim_space_ : right_aim_space_, display_time, &state.aim_pose, &state.aim_flags);
    return state;
  }

  void syncActions() {
    XrActiveActionSet active_set{};
    active_set.actionSet = action_set_;
    XrActionsSyncInfo sync_info{XR_TYPE_ACTIONS_SYNC_INFO};
    sync_info.countActiveActionSets = 1;
    sync_info.activeActionSets = &active_set;
    xrOk(xrSyncActions(session_, &sync_info), "xrSyncActions");
  }

  void locateSpace(XrSpace target_space, XrTime display_time, XrPosef* pose, XrSpaceLocationFlags* flags) {
    XrSpaceLocation location{XR_TYPE_SPACE_LOCATION};
    if (XR_SUCCEEDED(xrLocateSpace(target_space, space_, display_time, &location))) {
      *pose = location.pose;
      *flags = location.locationFlags;
    }
  }

  void renderView(uint32_t view_index) {
    auto& swapchain = swapchains_[view_index];
    uint32_t image_index = 0;
    XrSwapchainImageAcquireInfo acquire_info{XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
    if (!xrOk(xrAcquireSwapchainImage(swapchain.handle, &acquire_info, &image_index), "xrAcquireSwapchainImage")) {
      return;
    }

    XrSwapchainImageWaitInfo wait_info{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
    wait_info.timeout = XR_INFINITE_DURATION;
    if (!xrOk(xrWaitSwapchainImage(swapchain.handle, &wait_info), "xrWaitSwapchainImage")) {
      return;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, swapchain.images[image_index].image, 0);
    glViewport(0, 0, swapchain.width, swapchain.height);
    current_viewport_width_ = swapchain.width;
    current_viewport_height_ = swapchain.height;
    if (uploadLatestVideoFrame(view_index)) {
      drawVideoTexture(video_textures_[view_index]);
    } else if (drawLatestCodecSurfaceFrame(view_index)) {
      // Drawn directly from MediaCodec's SurfaceTexture.
    } else {
      drawWaitingOverlay();
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    XrSwapchainImageReleaseInfo release_info{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
    xrOk(xrReleaseSwapchainImage(swapchain.handle, &release_info), "xrReleaseSwapchainImage");
  }

  bool createVideoProgram() {
    const char* vertex_source = R"(#version 300 es
precision mediump float;
out vec2 v_uv;
void main() {
  vec2 positions[3] = vec2[3](vec2(-1.0, -1.0), vec2(3.0, -1.0), vec2(-1.0, 3.0));
  vec2 position = positions[gl_VertexID];
  v_uv = position * 0.5 + 0.5;
  gl_Position = vec4(position, 0.0, 1.0);
})";
    const char* fragment_source = R"(#version 300 es
precision mediump float;
in vec2 v_uv;
uniform sampler2D u_texture;
out vec4 out_color;
void main() {
  out_color = texture(u_texture, v_uv);
})";

    const GLuint vertex_shader = compileShader(GL_VERTEX_SHADER, vertex_source);
    const GLuint fragment_shader = compileShader(GL_FRAGMENT_SHADER, fragment_source);
    if (vertex_shader == 0 || fragment_shader == 0) {
      glDeleteShader(vertex_shader);
      glDeleteShader(fragment_shader);
      return false;
    }

    video_program_ = glCreateProgram();
    glAttachShader(video_program_, vertex_shader);
    glAttachShader(video_program_, fragment_shader);
    glLinkProgram(video_program_);
    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);

    GLint linked = GL_FALSE;
    glGetProgramiv(video_program_, GL_LINK_STATUS, &linked);
    if (linked != GL_TRUE) {
      LOGE("video shader link failed");
      glDeleteProgram(video_program_);
      video_program_ = 0;
      return false;
    }
    video_texture_uniform_ = glGetUniformLocation(video_program_, "u_texture");
    return true;
  }

  bool createExternalVideoProgram() {
    const char* vertex_source = R"(#version 300 es
precision mediump float;
out vec2 v_uv;
void main() {
  vec2 positions[3] = vec2[3](vec2(-1.0, -1.0), vec2(3.0, -1.0), vec2(-1.0, 3.0));
  vec2 position = positions[gl_VertexID];
  v_uv = position * 0.5 + 0.5;
  gl_Position = vec4(position, 0.0, 1.0);
})";
    const char* fragment_source = R"(#version 300 es
#extension GL_OES_EGL_image_external_essl3 : require
precision mediump float;
in vec2 v_uv;
uniform samplerExternalOES u_texture;
out vec4 out_color;
void main() {
  out_color = texture(u_texture, v_uv);
})";

    const GLuint vertex_shader = compileShader(GL_VERTEX_SHADER, vertex_source);
    const GLuint fragment_shader = compileShader(GL_FRAGMENT_SHADER, fragment_source);
    if (vertex_shader == 0 || fragment_shader == 0) {
      glDeleteShader(vertex_shader);
      glDeleteShader(fragment_shader);
      return false;
    }

    external_video_program_ = glCreateProgram();
    glAttachShader(external_video_program_, vertex_shader);
    glAttachShader(external_video_program_, fragment_shader);
    glLinkProgram(external_video_program_);
    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);

    GLint linked = GL_FALSE;
    glGetProgramiv(external_video_program_, GL_LINK_STATUS, &linked);
    if (linked != GL_TRUE) {
      LOGE("external video shader link failed");
      glDeleteProgram(external_video_program_);
      external_video_program_ = 0;
      return false;
    }
    external_video_texture_uniform_ = glGetUniformLocation(external_video_program_, "u_texture");
    return true;
  }

  bool createWaitingProgram() {
    const char* vertex_source = R"(#version 300 es
precision mediump float;
void main() {
  vec2 positions[3] = vec2[3](vec2(-1.0, -1.0), vec2(3.0, -1.0), vec2(-1.0, 3.0));
  gl_Position = vec4(positions[gl_VertexID], 0.0, 1.0);
})";
    const char* fragment_source = R"(#version 300 es
precision mediump float;
uniform vec2 u_viewport;
out vec4 out_color;
void main() {
  vec2 p = gl_FragCoord.xy;
  vec2 uv = p / max(u_viewport, vec2(1.0));
  vec2 d = abs(uv - vec2(0.5));
  float border = step(uv.x, 0.04) + step(0.96, uv.x) + step(uv.y, 0.04) + step(0.96, uv.y);
  float cross = step(d.x, 0.006) * step(d.y, 0.24) + step(d.y, 0.006) * step(d.x, 0.24);
  float center = step(length(d), 0.035);
  float ink = clamp(border + cross + center, 0.0, 1.0);
  vec3 bg = vec3(0.015, 0.018, 0.022);
  vec3 fg = vec3(0.50, 0.74, 0.95);
  out_color = vec4(mix(bg, fg, ink), 1.0);
})";

    const GLuint vertex_shader = compileShader(GL_VERTEX_SHADER, vertex_source);
    const GLuint fragment_shader = compileShader(GL_FRAGMENT_SHADER, fragment_source);
    if (vertex_shader == 0 || fragment_shader == 0) {
      glDeleteShader(vertex_shader);
      glDeleteShader(fragment_shader);
      return false;
    }

    waiting_program_ = glCreateProgram();
    glAttachShader(waiting_program_, vertex_shader);
    glAttachShader(waiting_program_, fragment_shader);
    glLinkProgram(waiting_program_);
    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);

    GLint linked = GL_FALSE;
    glGetProgramiv(waiting_program_, GL_LINK_STATUS, &linked);
    if (linked != GL_TRUE) {
      LOGE("waiting shader link failed");
      glDeleteProgram(waiting_program_);
      waiting_program_ = 0;
      return false;
    }
    waiting_viewport_uniform_ = glGetUniformLocation(waiting_program_, "u_viewport");
    return true;
  }

  GLuint compileShader(GLenum type, const char* source) {
    const GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);
    GLint compiled = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
    if (compiled != GL_TRUE) {
      LOGE("video shader compile failed");
      glDeleteShader(shader);
      return 0;
    }
    return shader;
  }

  bool createCodecSurfaceResources(JNIEnv* env) {
    jclass activity_class = env->GetObjectClass(activity_);
    jmethodID get_class_loader = env->GetMethodID(activity_class, "getClassLoader", "()Ljava/lang/ClassLoader;");
    jobject class_loader = env->CallObjectMethod(activity_, get_class_loader);
    jclass class_loader_class = env->FindClass("java/lang/ClassLoader");
    jmethodID load_class = env->GetMethodID(class_loader_class, "loadClass", "(Ljava/lang/String;)Ljava/lang/Class;");
    jstring bridge_name = env->NewStringUTF("com.yuchen.robovr.VideoSurfaceBridge");
    jclass local_bridge_class = reinterpret_cast<jclass>(
        env->CallObjectMethod(class_loader, load_class, bridge_name));
    env->DeleteLocalRef(bridge_name);
    env->DeleteLocalRef(class_loader_class);
    env->DeleteLocalRef(class_loader);
    env->DeleteLocalRef(activity_class);
    if (local_bridge_class == nullptr) {
      env->ExceptionClear();
      LOGE("VideoSurfaceBridge class not found");
      return false;
    }
    bridge_class_ = reinterpret_cast<jclass>(env->NewGlobalRef(local_bridge_class));
    env->DeleteLocalRef(local_bridge_class);
    create_surface_texture_method_ = env->GetStaticMethodID(
        bridge_class_, "createSurfaceTexture", "(I)Landroid/graphics/SurfaceTexture;");
    create_surface_method_ = env->GetStaticMethodID(
        bridge_class_, "createSurface", "(Landroid/graphics/SurfaceTexture;)Landroid/view/Surface;");
    update_tex_image_method_ = env->GetStaticMethodID(
        bridge_class_, "updateTexImage", "(Landroid/graphics/SurfaceTexture;)V");
    release_surface_method_ = env->GetStaticMethodID(
        bridge_class_, "releaseSurface", "(Landroid/view/Surface;)V");
    release_surface_texture_method_ = env->GetStaticMethodID(
        bridge_class_, "releaseSurfaceTexture", "(Landroid/graphics/SurfaceTexture;)V");
    if (create_surface_texture_method_ == nullptr || create_surface_method_ == nullptr ||
        update_tex_image_method_ == nullptr || release_surface_method_ == nullptr ||
        release_surface_texture_method_ == nullptr) {
      LOGE("VideoSurfaceBridge method lookup failed");
      return false;
    }

    glGenTextures(static_cast<GLsizei>(codec_textures_.size()), codec_textures_.data());
    for (std::size_t i = 0; i < codec_textures_.size(); ++i) {
      glBindTexture(kTextureExternalOes, codec_textures_[i]);
      glTexParameteri(kTextureExternalOes, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri(kTextureExternalOes, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      glTexParameteri(kTextureExternalOes, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameteri(kTextureExternalOes, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

      jobject local_texture = env->CallStaticObjectMethod(
          bridge_class_, create_surface_texture_method_, static_cast<jint>(codec_textures_[i]));
      if (env->ExceptionCheck() || local_texture == nullptr) {
        env->ExceptionClear();
        LOGE("SurfaceTexture creation failed");
        return false;
      }
      surface_textures_[i] = env->NewGlobalRef(local_texture);
      env->DeleteLocalRef(local_texture);

      jobject local_surface = env->CallStaticObjectMethod(
          bridge_class_, create_surface_method_, surface_textures_[i]);
      if (env->ExceptionCheck() || local_surface == nullptr) {
        env->ExceptionClear();
        LOGE("Surface creation failed");
        return false;
      }
      surfaces_[i] = env->NewGlobalRef(local_surface);
      env->DeleteLocalRef(local_surface);

      codec_windows_[i] = ANativeWindow_fromSurface(env, surfaces_[i]);
      if (codec_windows_[i] == nullptr) {
        LOGE("ANativeWindow_fromSurface failed");
        return false;
      }
    }

    {
      std::lock_guard<std::mutex> lock(g_surface_mutex);
      g_codec_surfaces.left_window = codec_windows_[0];
      g_codec_surfaces.right_window = codec_windows_[1];
      g_codec_surfaces.generation = ++g_codec_surface_generation;
    }
    LOGI("created codec SurfaceTexture resources generation=%u", g_codec_surface_generation);
    return true;
  }

  bool drawLatestCodecSurfaceFrame(uint32_t view_index) {
    const std::uint32_t frame_index = g_latest_codec_frame.load();
    if (frame_index == UINT32_MAX || external_video_program_ == 0 || view_index >= codec_textures_.size()) {
      return false;
    }
    JNIEnv* env = nullptr;
    if (vm_->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK ||
        env == nullptr || surface_textures_[view_index] == nullptr) {
      return false;
    }
    env->CallStaticVoidMethod(bridge_class_, update_tex_image_method_, surface_textures_[view_index]);
    if (env->ExceptionCheck()) {
      env->ExceptionClear();
      return false;
    }
    drawExternalVideoTexture(codec_textures_[view_index]);
    return true;
  }

  bool uploadLatestVideoFrame(uint32_t view_index) {
    std::optional<StereoVideoFrame> frame;
    {
      std::lock_guard<std::mutex> lock(g_video_mutex);
      frame = g_stereo_frame;
    }
    if (!frame.has_value() || frame->width == 0 || frame->height == 0) {
      return false;
    }

    const std::uint32_t eye_frame_index = uploaded_video_frame_[view_index];
    if (eye_frame_index == frame->frame_index) {
      return true;
    }

    const auto& pixels = view_index == 0 ? frame->left_rgba : frame->right_rgba;
    if (pixels.size() != static_cast<std::size_t>(frame->width) * frame->height * 4) {
      return false;
    }

    glBindTexture(GL_TEXTURE_2D, video_textures_[view_index]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, static_cast<GLsizei>(frame->width),
                 static_cast<GLsizei>(frame->height), 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
    uploaded_video_frame_[view_index] = frame->frame_index;
    return true;
  }

  void drawVideoTexture(GLuint texture) {
    glDisable(GL_DEPTH_TEST);
    glUseProgram(video_program_);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture);
    glUniform1i(video_texture_uniform_, 0);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindTexture(GL_TEXTURE_2D, 0);
    glUseProgram(0);
  }

  void drawExternalVideoTexture(GLuint texture) {
    glDisable(GL_DEPTH_TEST);
    glUseProgram(external_video_program_);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(kTextureExternalOes, texture);
    glUniform1i(external_video_texture_uniform_, 0);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindTexture(kTextureExternalOes, 0);
    glUseProgram(0);
  }

  void drawWaitingOverlay() {
    glDisable(GL_DEPTH_TEST);
    glUseProgram(waiting_program_);
    glUniform2f(
        waiting_viewport_uniform_,
        static_cast<float>(std::max(1, current_viewport_width_)),
        static_cast<float>(std::max(1, current_viewport_height_)));
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glUseProgram(0);
  }

  void cleanup() {
    {
      std::lock_guard<std::mutex> lock(g_surface_mutex);
      g_codec_surfaces = {};
      ++g_codec_surface_generation;
    }
    JNIEnv* env = nullptr;
    if (vm_ != nullptr && vm_->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) == JNI_OK && env != nullptr) {
      for (jobject& surface : surfaces_) {
        if (surface != nullptr) {
          env->CallStaticVoidMethod(bridge_class_, release_surface_method_, surface);
          env->DeleteGlobalRef(surface);
          surface = nullptr;
        }
      }
      for (jobject& surface_texture : surface_textures_) {
        if (surface_texture != nullptr) {
          env->CallStaticVoidMethod(bridge_class_, release_surface_texture_method_, surface_texture);
          env->DeleteGlobalRef(surface_texture);
          surface_texture = nullptr;
        }
      }
      if (bridge_class_ != nullptr) {
        env->DeleteGlobalRef(bridge_class_);
        bridge_class_ = nullptr;
      }
    }
    for (ANativeWindow*& window : codec_windows_) {
      if (window != nullptr) {
        ANativeWindow_release(window);
        window = nullptr;
      }
    }
    if (external_video_program_ != 0) {
      glDeleteProgram(external_video_program_);
      external_video_program_ = 0;
    }
    if (waiting_program_ != 0) {
      glDeleteProgram(waiting_program_);
      waiting_program_ = 0;
    }
    if (video_program_ != 0) {
      glDeleteProgram(video_program_);
      video_program_ = 0;
    }
    if (codec_textures_[0] != 0 || codec_textures_[1] != 0) {
      glDeleteTextures(static_cast<GLsizei>(codec_textures_.size()), codec_textures_.data());
      codec_textures_ = {};
    }
    if (video_textures_[0] != 0 || video_textures_[1] != 0) {
      glDeleteTextures(static_cast<GLsizei>(video_textures_.size()), video_textures_.data());
      video_textures_ = {};
    }
    if (framebuffer_ != 0) {
      glDeleteFramebuffers(1, &framebuffer_);
      framebuffer_ = 0;
    }
    for (auto& swapchain : swapchains_) {
      if (swapchain.handle != XR_NULL_HANDLE) {
        xrDestroySwapchain(swapchain.handle);
        swapchain.handle = XR_NULL_HANDLE;
      }
    }
    if (space_ != XR_NULL_HANDLE) {
      xrDestroySpace(space_);
      space_ = XR_NULL_HANDLE;
    }
    destroySpace(left_grip_space_);
    destroySpace(right_grip_space_);
    destroySpace(left_aim_space_);
    destroySpace(right_aim_space_);
    destroyAction(grip_pose_action_);
    destroyAction(aim_pose_action_);
    destroyAction(trigger_action_);
    destroyAction(squeeze_action_);
    destroyAction(primary_button_action_);
    destroyAction(secondary_button_action_);
    if (action_set_ != XR_NULL_HANDLE) {
      xrDestroyActionSet(action_set_);
      action_set_ = XR_NULL_HANDLE;
    }
    if (session_ != XR_NULL_HANDLE) {
      xrDestroySession(session_);
      session_ = XR_NULL_HANDLE;
    }
    if (instance_ != XR_NULL_HANDLE) {
      xrDestroyInstance(instance_);
      instance_ = XR_NULL_HANDLE;
    }
    if (egl_display_ != EGL_NO_DISPLAY) {
      eglMakeCurrent(egl_display_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
      if (egl_surface_ != EGL_NO_SURFACE) {
        eglDestroySurface(egl_display_, egl_surface_);
      }
      if (egl_context_ != EGL_NO_CONTEXT) {
        eglDestroyContext(egl_display_, egl_context_);
      }
      eglTerminate(egl_display_);
    }
  }

  void destroySpace(XrSpace& target_space) {
    if (target_space != XR_NULL_HANDLE) {
      xrDestroySpace(target_space);
      target_space = XR_NULL_HANDLE;
    }
  }

  void destroyAction(XrAction& action) {
    if (action != XR_NULL_HANDLE) {
      xrDestroyAction(action);
      action = XR_NULL_HANDLE;
    }
  }

  void initializeDisplayRefreshRateExtension() {
    if (!display_refresh_rate_extension_enabled_) {
      return;
    }
    if (!xrOk(xrGetInstanceProcAddr(
                  instance_, "xrEnumerateDisplayRefreshRatesFB",
                  reinterpret_cast<PFN_xrVoidFunction*>(&xr_enumerate_display_refresh_rates_)),
              "xrEnumerateDisplayRefreshRatesFB proc")) {
      display_refresh_rate_extension_enabled_ = false;
      return;
    }
    if (!xrOk(xrGetInstanceProcAddr(
                  instance_, "xrRequestDisplayRefreshRateFB",
                  reinterpret_cast<PFN_xrVoidFunction*>(&xr_request_display_refresh_rate_)),
              "xrRequestDisplayRefreshRateFB proc")) {
      display_refresh_rate_extension_enabled_ = false;
      return;
    }

    uint32_t rate_count = 0;
    if (!xrOk(xr_enumerate_display_refresh_rates_(session_, 0, &rate_count, nullptr),
              "xrEnumerateDisplayRefreshRatesFB count")) {
      return;
    }
    supported_display_refresh_rates_.resize(rate_count);
    if (rate_count > 0 &&
        !xrOk(xr_enumerate_display_refresh_rates_(
                  session_, rate_count, &rate_count, supported_display_refresh_rates_.data()),
              "xrEnumerateDisplayRefreshRatesFB")) {
      supported_display_refresh_rates_.clear();
      return;
    }
    std::string message = "OpenXR supported display refresh rates:";
    for (float rate : supported_display_refresh_rates_) {
      char buffer[32];
      std::snprintf(buffer, sizeof(buffer), " %.1f", rate);
      message += buffer;
    }
    LOGI("%s", message.c_str());
  }

  void requestDisplayRefreshRate(float requested_hz) {
    if (!display_refresh_rate_extension_enabled_ || xr_request_display_refresh_rate_ == nullptr ||
        display_refresh_rate_requested_) {
      return;
    }
    float selected_hz = requested_hz;
    if (!supported_display_refresh_rates_.empty()) {
      selected_hz = supported_display_refresh_rates_.front();
      float best_error = std::abs(selected_hz - requested_hz);
      for (float rate : supported_display_refresh_rates_) {
        const float error = std::abs(rate - requested_hz);
        if (error < best_error) {
          selected_hz = rate;
          best_error = error;
        }
      }
    }
    if (xrOk(xr_request_display_refresh_rate_(session_, selected_hz),
             "xrRequestDisplayRefreshRateFB")) {
      display_refresh_rate_requested_ = true;
      LOGI("requested OpenXR display refresh rate %.1f Hz", selected_hz);
    }
  }

  JavaVM* vm_ = nullptr;
  jobject activity_ = nullptr;
  XrInstance instance_ = XR_NULL_HANDLE;
  XrSystemId system_id_ = XR_NULL_SYSTEM_ID;
  XrSession session_ = XR_NULL_HANDLE;
  XrSpace space_ = XR_NULL_HANDLE;
  std::array<XrPath, 2> hand_paths_{XR_NULL_PATH, XR_NULL_PATH};
  XrActionSet action_set_ = XR_NULL_HANDLE;
  XrAction grip_pose_action_ = XR_NULL_HANDLE;
  XrAction aim_pose_action_ = XR_NULL_HANDLE;
  XrAction trigger_action_ = XR_NULL_HANDLE;
  XrAction squeeze_action_ = XR_NULL_HANDLE;
  XrAction primary_button_action_ = XR_NULL_HANDLE;
  XrAction secondary_button_action_ = XR_NULL_HANDLE;
  XrSpace left_grip_space_ = XR_NULL_HANDLE;
  XrSpace right_grip_space_ = XR_NULL_HANDLE;
  XrSpace left_aim_space_ = XR_NULL_HANDLE;
  XrSpace right_aim_space_ = XR_NULL_HANDLE;
  XrSessionState session_state_ = XR_SESSION_STATE_UNKNOWN;
  bool session_running_ = false;
  bool exit_render_loop_ = false;
  bool display_refresh_rate_extension_enabled_ = false;
  bool display_refresh_rate_requested_ = false;
  PFN_xrEnumerateDisplayRefreshRatesFB xr_enumerate_display_refresh_rates_ = nullptr;
  PFN_xrRequestDisplayRefreshRateFB xr_request_display_refresh_rate_ = nullptr;
  std::vector<float> supported_display_refresh_rates_;

  EGLDisplay egl_display_ = EGL_NO_DISPLAY;
  EGLConfig egl_config_ = nullptr;
  EGLContext egl_context_ = EGL_NO_CONTEXT;
  EGLSurface egl_surface_ = EGL_NO_SURFACE;
  GLuint framebuffer_ = 0;
  GLuint video_program_ = 0;
  GLint video_texture_uniform_ = -1;
  std::array<GLuint, 2> video_textures_{0, 0};
  GLuint external_video_program_ = 0;
  GLint external_video_texture_uniform_ = -1;
  GLuint waiting_program_ = 0;
  GLint waiting_viewport_uniform_ = -1;
  int32_t current_viewport_width_ = 1;
  int32_t current_viewport_height_ = 1;
  std::array<GLuint, 2> codec_textures_{0, 0};
  std::array<std::uint32_t, 2> uploaded_video_frame_{UINT32_MAX, UINT32_MAX};
  int64_t color_format_ = 0;
  jclass bridge_class_ = nullptr;
  jmethodID create_surface_texture_method_ = nullptr;
  jmethodID create_surface_method_ = nullptr;
  jmethodID update_tex_image_method_ = nullptr;
  jmethodID release_surface_method_ = nullptr;
  jmethodID release_surface_texture_method_ = nullptr;
  std::array<jobject, 2> surface_textures_{nullptr, nullptr};
  std::array<jobject, 2> surfaces_{nullptr, nullptr};
  std::array<ANativeWindow*, 2> codec_windows_{nullptr, nullptr};

  std::vector<XrViewConfigurationView> view_configs_;
  std::vector<XrView> views_;
  std::vector<XrCompositionLayerProjectionView> projection_views_;
  std::vector<Swapchain> swapchains_;
};

}  // namespace

void startOpenXr(JavaVM* vm, jobject activity) {
  if (g_running.exchange(true)) {
    return;
  }
  g_vm = vm;
  JNIEnv* env = nullptr;
  if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) == JNI_OK) {
    g_activity = env->NewGlobalRef(activity);
  }
  g_thread = std::thread([] {
    OpenXrApp app(g_vm, g_activity);
    app.run();
  });
}

void stopOpenXr() {
  if (!g_running.exchange(false)) {
    return;
  }
  if (g_thread.joinable()) {
    g_thread.join();
  }
  if (g_vm != nullptr && g_activity != nullptr) {
    JNIEnv* env = nullptr;
    if (g_vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) == JNI_OK) {
      env->DeleteGlobalRef(g_activity);
    }
  }
  g_activity = nullptr;
}

void setOpenXrResumed(bool resumed) {
  g_resumed = resumed;
}

void publishStereoClearColors(const StereoClearColors& colors) {
  std::lock_guard<std::mutex> lock(g_video_mutex);
  g_stereo_colors = colors;
}

void publishStereoVideoFrame(StereoVideoFrame frame) {
  std::lock_guard<std::mutex> lock(g_video_mutex);
  g_stereo_frame = std::move(frame);
}

bool acquireStereoCodecSurfaces(StereoCodecSurfaces* surfaces) {
  if (surfaces == nullptr) {
    return false;
  }
  std::lock_guard<std::mutex> lock(g_surface_mutex);
  if (g_codec_surfaces.left_window == nullptr || g_codec_surfaces.right_window == nullptr) {
    return false;
  }
  *surfaces = g_codec_surfaces;
  ANativeWindow_acquire(surfaces->left_window);
  ANativeWindow_acquire(surfaces->right_window);
  return true;
}

void publishStereoCodecFrame(std::uint32_t frame_index) {
  g_latest_codec_frame.store(frame_index);
  std::lock_guard<std::mutex> lock(g_video_mutex);
  g_stereo_frame.reset();
}
