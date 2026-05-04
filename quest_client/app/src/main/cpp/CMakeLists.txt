cmake_minimum_required(VERSION 3.22.1)

project(robovr_quest_client LANGUAGES C CXX)

set(BUILD_API_LAYERS OFF CACHE BOOL "" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_CONFORMANCE_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_SDK_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_ALL_EXTENSIONS OFF CACHE BOOL "" FORCE)
add_subdirectory(
  ../../../../../external/OpenXR-SDK-Source
  ${CMAKE_CURRENT_BINARY_DIR}/openxr
)

add_library(robovr_quest_client SHARED
  native_main.cpp
  openxr_app.cpp
  transport.cpp
  video_decoder.cpp
  ${ANDROID_NDK}/sources/android/native_app_glue/android_native_app_glue.c
)

target_include_directories(robovr_quest_client PRIVATE
  ../../../../../proto
  ${ANDROID_NDK}/sources/android/native_app_glue
)

target_compile_features(robovr_quest_client PRIVATE cxx_std_20)
target_compile_definitions(robovr_quest_client PRIVATE
  XR_USE_PLATFORM_ANDROID
  XR_USE_GRAPHICS_API_OPENGL_ES
  XR_USE_PLATFORM_EGL
)

target_link_libraries(robovr_quest_client
  OpenXR::openxr_loader
  android
  EGL
  GLESv3
  log
  mediandk
)
