#include "video_decoder.h"

#include <android/log.h>
#include <android/native_window.h>
#include <media/NdkMediaCodec.h>
#include <media/NdkMediaFormat.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <memory>
#include <optional>
#include <string_view>
#include <utility>

namespace {

constexpr char kLogTag[] = "RoboVR";

bool checkedRawRgbaSize(const quest3_teleop::VideoFrameHeader& header, std::size_t* eye_bytes) {
  const std::uint64_t bytes = static_cast<std::uint64_t>(header.width) * header.height * 4u;
  if (header.width == 0 || header.height == 0 || bytes > 64u * 1024u * 1024u) {
    return false;
  }
  if (header.left_payload_size != bytes || header.right_payload_size != bytes) {
    return false;
  }
  *eye_bytes = static_cast<std::size_t>(bytes);
  return true;
}

void logUnsupportedOnce(quest3_teleop::VideoCodec codec) {
  static bool logged_hevc = false;
  static bool logged_av1 = false;
  if (codec == quest3_teleop::VideoCodec::kH265AnnexB && !logged_hevc) {
    __android_log_print(ANDROID_LOG_WARN, kLogTag, "H265AnnexB decoder is not implemented yet");
    logged_hevc = true;
  } else if (codec == quest3_teleop::VideoCodec::kAv1AnnexB && !logged_av1) {
    __android_log_print(ANDROID_LOG_WARN, kLogTag, "Av1AnnexB decoder is not implemented yet");
    logged_av1 = true;
  }
}

struct MediaFormatDeleter {
  void operator()(AMediaFormat* format) const {
    if (format != nullptr) {
      AMediaFormat_delete(format);
    }
  }
};

using MediaFormatPtr = std::unique_ptr<AMediaFormat, MediaFormatDeleter>;

AMediaCodec* createH264Decoder(std::uint32_t width, std::uint32_t height, ANativeWindow* output_window) {
  AMediaCodec* codec = AMediaCodec_createDecoderByType("video/avc");
  if (codec == nullptr) {
    __android_log_print(ANDROID_LOG_ERROR, kLogTag, "AMediaCodec_createDecoderByType(video/avc) failed");
    return nullptr;
  }

  MediaFormatPtr format(AMediaFormat_new());
  if (!format) {
    AMediaCodec_delete(codec);
    return nullptr;
  }
  AMediaFormat_setString(format.get(), AMEDIAFORMAT_KEY_MIME, "video/avc");
  AMediaFormat_setInt32(format.get(), AMEDIAFORMAT_KEY_WIDTH, static_cast<std::int32_t>(width));
  AMediaFormat_setInt32(format.get(), AMEDIAFORMAT_KEY_HEIGHT, static_cast<std::int32_t>(height));
  AMediaFormat_setInt32(format.get(), AMEDIAFORMAT_KEY_MAX_INPUT_SIZE,
                        static_cast<std::int32_t>(std::max<std::uint32_t>(width * height, 1024 * 1024)));
  AMediaFormat_setInt32(format.get(), AMEDIAFORMAT_KEY_PRIORITY, 0);
  AMediaFormat_setInt32(format.get(), AMEDIAFORMAT_KEY_LATENCY, 1);

  media_status_t status = AMediaCodec_configure(codec, format.get(), output_window, nullptr, 0);
  if (status != AMEDIA_OK) {
    __android_log_print(ANDROID_LOG_ERROR, kLogTag, "AMediaCodec_configure(video/avc) failed: %d", status);
    AMediaCodec_delete(codec);
    return nullptr;
  }

  status = AMediaCodec_start(codec);
  if (status != AMEDIA_OK) {
    __android_log_print(ANDROID_LOG_ERROR, kLogTag, "AMediaCodec_start(video/avc) failed: %d", status);
    AMediaCodec_delete(codec);
    return nullptr;
  }

  char* name = nullptr;
  if (AMediaCodec_getName(codec, &name) == AMEDIA_OK && name != nullptr) {
    __android_log_print(ANDROID_LOG_INFO, kLogTag, "H.264 decoder started: %s %ux%u surface=%d",
                        name, width, height, output_window != nullptr ? 1 : 0);
    AMediaCodec_releaseName(codec, name);
  } else {
    __android_log_print(ANDROID_LOG_INFO, kLogTag, "H.264 decoder started: %ux%u surface=%d",
                        width, height, output_window != nullptr ? 1 : 0);
  }
  return codec;
}

std::uint8_t clampByte(int value) {
  return static_cast<std::uint8_t>(std::clamp(value, 0, 255));
}

void yuvToRgb(std::uint8_t y, std::uint8_t u, std::uint8_t v,
              std::uint8_t* r, std::uint8_t* g, std::uint8_t* b) {
  const int c = static_cast<int>(y) - 16;
  const int d = static_cast<int>(u) - 128;
  const int e = static_cast<int>(v) - 128;
  *r = clampByte((298 * c + 409 * e + 128) >> 8);
  *g = clampByte((298 * c - 100 * d - 208 * e + 128) >> 8);
  *b = clampByte((298 * c + 516 * d + 128) >> 8);
}

std::int64_t nowNs() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
}

}  // namespace

bool RawRgbaDecoder::decode(const quest3_teleop::VideoFrameHeader& header,
                            const std::uint8_t* left_payload,
                            const std::uint8_t* right_payload) {
  std::size_t eye_bytes = 0;
  if (!checkedRawRgbaSize(header, &eye_bytes)) {
    return false;
  }

  StereoVideoFrame frame;
  frame.frame_index = static_cast<std::uint32_t>(header.frame_index);
  frame.width = header.width;
  frame.height = header.height;
  frame.left_rgba.assign(left_payload, left_payload + eye_bytes);
  frame.right_rgba.assign(right_payload, right_payload + eye_bytes);
  publishStereoVideoFrame(std::move(frame));

  static std::uint64_t raw_video_count = 0;
  if (raw_video_count % 30 == 0) {
    __android_log_print(ANDROID_LOG_INFO, kLogTag,
                        "rx binary raw_rgba frame=%llu count=%llu %ux%u",
                        static_cast<unsigned long long>(header.frame_index),
                        static_cast<unsigned long long>(raw_video_count),
                        header.width, header.height);
  }
  ++raw_video_count;
  return true;
}

H264AnnexBDecoder::~H264AnnexBDecoder() {
  stop();
}

bool H264AnnexBDecoder::decode(const quest3_teleop::VideoFrameHeader& header,
                               const std::uint8_t* left_payload,
                               const std::uint8_t* right_payload) {
  if (has_published_frame_ && header.frame_index + 2 < last_published_frame_index_) {
    __android_log_print(ANDROID_LOG_INFO, kLogTag,
                        "H.264 stream frame index reset: incoming=%llu last_published=%llu; restarting decoder",
                        static_cast<unsigned long long>(header.frame_index),
                        static_cast<unsigned long long>(last_published_frame_index_));
    stop();
  }

  ++received_frames_;
  const std::int64_t receive_time_ns = nowNs();
  if (!ensureStarted(header)) {
    return false;
  }

  const bool keyframe = (header.flags & quest3_teleop::kVideoFrameKeyframe) != 0;
  const bool left_queued = queueAccessUnit(left_codec_, left_payload, header.left_payload_size, header.frame_index, keyframe);
  const bool right_queued = queueAccessUnit(right_codec_, right_payload, header.right_payload_size, header.frame_index, keyframe);
  if (left_queued && right_queued) {
    ++queued_frames_;
    receive_time_by_frame_[header.frame_index] = receive_time_ns;
  } else {
    ++dropped_input_frames_;
  }

  if (surface_output_) {
    std::vector<std::uint64_t> left_indices;
    std::vector<std::uint64_t> right_indices;
    if (drainSurfaceOutput(left_codec_, &left_indices)) {
      for (const std::uint64_t frame_index : left_indices) {
        surface_left_ready_[frame_index] = true;
      }
    }
    if (drainSurfaceOutput(right_codec_, &right_indices)) {
      for (const std::uint64_t frame_index : right_indices) {
        surface_right_ready_[frame_index] = true;
      }
    }

    std::optional<std::uint64_t> ready_frame_index;
    for (const auto& [frame_index, ready] : surface_left_ready_) {
      if (!ready || (has_published_frame_ && frame_index <= last_published_frame_index_)) {
        continue;
      }
      const auto right_it = surface_right_ready_.find(frame_index);
      if (right_it == surface_right_ready_.end() || !right_it->second) {
        continue;
      }
      if (!ready_frame_index.has_value() || frame_index > *ready_frame_index) {
        ready_frame_index = frame_index;
      }
    }

    if (ready_frame_index.has_value()) {
      const std::uint64_t decoded_frame_index = *ready_frame_index;
      ++decoded_frames_;
      const auto receive_it = receive_time_by_frame_.find(decoded_frame_index);
      if (receive_it != receive_time_by_frame_.end()) {
        latest_latency_ms_ = static_cast<double>(nowNs() - receive_it->second) / 1.0e6;
        latency_sum_ms_ += latest_latency_ms_;
      }
      publishStereoCodecFrame(static_cast<std::uint32_t>(decoded_frame_index));
      pending_acks_.push_back(VideoDecodeAck{decoded_frame_index, latest_latency_ms_, true});
      last_published_frame_index_ = decoded_frame_index;
      has_published_frame_ = true;
      for (auto it = surface_left_ready_.begin(); it != surface_left_ready_.end();) {
        it = it->first <= decoded_frame_index ? surface_left_ready_.erase(it) : std::next(it);
      }
      for (auto it = surface_right_ready_.begin(); it != surface_right_ready_.end();) {
        it = it->first <= decoded_frame_index ? surface_right_ready_.erase(it) : std::next(it);
      }
      for (auto it = receive_time_by_frame_.begin(); it != receive_time_by_frame_.end();) {
        it = it->first <= decoded_frame_index ? receive_time_by_frame_.erase(it) : std::next(it);
      }
    }

    const std::int64_t stats_now = nowNs();
    if (last_stats_time_ns_ == 0) {
      last_stats_time_ns_ = stats_now;
    } else if (stats_now - last_stats_time_ns_ >= 1000000000LL) {
      const double avg_latency = decoded_frames_ > 0
          ? latency_sum_ms_ / static_cast<double>(decoded_frames_)
          : 0.0;
      __android_log_print(ANDROID_LOG_INFO, kLogTag,
                          "h264 surface stats rx=%llu queued=%llu decoded=%llu dropped_input=%llu latency_ms=%.2f avg_ms=%.2f",
                          static_cast<unsigned long long>(received_frames_),
                          static_cast<unsigned long long>(queued_frames_),
                          static_cast<unsigned long long>(decoded_frames_),
                          static_cast<unsigned long long>(dropped_input_frames_),
                          latest_latency_ms_, avg_latency);
      last_stats_time_ns_ = stats_now;
    }
    return left_queued && right_queued;
  }

  std::vector<DecodedEyeFrame> left_frames;
  std::vector<DecodedEyeFrame> right_frames;
  const bool left_ready = drainOutput(left_codec_, &left_frames);
  const bool right_ready = drainOutput(right_codec_, &right_frames);
  if (left_ready) {
    for (auto& frame : left_frames) {
      if (frame.valid) {
        decoded_left_by_frame_[frame.frame_index] = std::move(frame.rgba);
      }
    }
  }
  if (right_ready) {
    for (auto& frame : right_frames) {
      if (frame.valid) {
        decoded_right_by_frame_[frame.frame_index] = std::move(frame.rgba);
      }
    }
  }

  const std::size_t expected_rgba_size = static_cast<std::size_t>(width_) * height_ * 4;
  std::optional<std::uint64_t> ready_frame_index;
  for (const auto& [frame_index, left_rgba] : decoded_left_by_frame_) {
    if (has_published_frame_ && frame_index <= last_published_frame_index_) {
      continue;
    }
    const auto right_it = decoded_right_by_frame_.find(frame_index);
    if (right_it == decoded_right_by_frame_.end()) {
      continue;
    }
    if (left_rgba.size() != expected_rgba_size || right_it->second.size() != expected_rgba_size) {
      continue;
    }
    if (!ready_frame_index.has_value() || frame_index < *ready_frame_index) {
      ready_frame_index = frame_index;
    }
  }

  if (ready_frame_index.has_value()) {
    const std::uint64_t decoded_frame_index = *ready_frame_index;
    auto left_it = decoded_left_by_frame_.find(decoded_frame_index);
    auto right_it = decoded_right_by_frame_.find(decoded_frame_index);
    ++decoded_frames_;
    const auto receive_it = receive_time_by_frame_.find(decoded_frame_index);
    if (receive_it != receive_time_by_frame_.end()) {
      latest_latency_ms_ = static_cast<double>(nowNs() - receive_it->second) / 1.0e6;
      latency_sum_ms_ += latest_latency_ms_;
    }
    StereoVideoFrame frame;
    frame.frame_index = static_cast<std::uint32_t>(decoded_frame_index);
    frame.width = width_;
    frame.height = height_;
    frame.left_rgba = left_it->second;
    frame.right_rgba = right_it->second;
    publishStereoVideoFrame(std::move(frame));
    pending_acks_.push_back(VideoDecodeAck{decoded_frame_index, latest_latency_ms_, false});
    last_published_frame_index_ = decoded_frame_index;
    has_published_frame_ = true;
    decoded_left_by_frame_.erase(left_it);
    decoded_right_by_frame_.erase(right_it);
    for (auto it = decoded_left_by_frame_.begin(); it != decoded_left_by_frame_.end();) {
      if (it->first <= decoded_frame_index) {
        it = decoded_left_by_frame_.erase(it);
      } else {
        ++it;
      }
    }
    for (auto it = decoded_right_by_frame_.begin(); it != decoded_right_by_frame_.end();) {
      if (it->first <= decoded_frame_index) {
        it = decoded_right_by_frame_.erase(it);
      } else {
        ++it;
      }
    }
    for (auto it = receive_time_by_frame_.begin(); it != receive_time_by_frame_.end();) {
      if (it->first <= decoded_frame_index) {
        it = receive_time_by_frame_.erase(it);
      } else {
        ++it;
      }
    }
  }

  const std::int64_t stats_now = nowNs();
  if (last_stats_time_ns_ == 0) {
    last_stats_time_ns_ = stats_now;
  } else if (stats_now - last_stats_time_ns_ >= 1000000000LL) {
    const double avg_latency = decoded_frames_ > 0
        ? latency_sum_ms_ / static_cast<double>(decoded_frames_)
        : 0.0;
    __android_log_print(ANDROID_LOG_INFO, kLogTag,
                        "h264 stats rx=%llu queued=%llu decoded=%llu dropped_input=%llu latency_ms=%.2f avg_ms=%.2f",
                        static_cast<unsigned long long>(received_frames_),
                        static_cast<unsigned long long>(queued_frames_),
                        static_cast<unsigned long long>(decoded_frames_),
                        static_cast<unsigned long long>(dropped_input_frames_),
                        latest_latency_ms_, avg_latency);
    last_stats_time_ns_ = stats_now;
  }
  return left_queued && right_queued;
}

std::vector<VideoDecodeAck> H264AnnexBDecoder::takeAcks() {
  std::vector<VideoDecodeAck> acks;
  acks.swap(pending_acks_);
  return acks;
}

bool H264AnnexBDecoder::ensureStarted(const quest3_teleop::VideoFrameHeader& header) {
  if (header.width == 0 || header.height == 0 ||
      header.left_payload_size == 0 || header.right_payload_size == 0) {
    return false;
  }
  if (left_codec_ != nullptr && right_codec_ != nullptr &&
      width_ == header.width && height_ == header.height) {
    return true;
  }

  stop();
  width_ = header.width;
  height_ = header.height;
  StereoCodecSurfaces surfaces;
  if (!acquireStereoCodecSurfaces(&surfaces)) {
    static std::uint64_t wait_log_count = 0;
    if (wait_log_count % 30 == 0) {
      __android_log_print(ANDROID_LOG_INFO, kLogTag, "waiting for H.264 Surface before starting decoder");
    }
    ++wait_log_count;
    return false;
  }
  left_window_ = surfaces.left_window;
  right_window_ = surfaces.right_window;
  surface_generation_ = surfaces.generation;
  surface_output_ = true;
  left_codec_ = createH264Decoder(width_, height_, left_window_);
  right_codec_ = createH264Decoder(width_, height_, right_window_);
  if (left_codec_ == nullptr || right_codec_ == nullptr) {
    stop();
    return false;
  }
  __android_log_print(ANDROID_LOG_INFO, kLogTag, "H.264 output path: %s",
                      surface_output_ ? "surface" : "cpu");
  return true;
}

bool H264AnnexBDecoder::queueAccessUnit(AMediaCodec* codec,
                                        const std::uint8_t* payload,
                                        std::size_t payload_size,
                                        std::uint64_t frame_index,
                                        bool) {
  if (codec == nullptr || payload == nullptr || payload_size == 0) {
    return false;
  }

  const ssize_t input_index = AMediaCodec_dequeueInputBuffer(codec, 0);
  if (input_index == AMEDIACODEC_INFO_TRY_AGAIN_LATER) {
    return false;
  }
  if (input_index < 0) {
    __android_log_print(ANDROID_LOG_WARN, kLogTag, "H.264 dequeue input failed: %zd", input_index);
    return false;
  }

  std::size_t input_size = 0;
  std::uint8_t* input = AMediaCodec_getInputBuffer(codec, static_cast<std::size_t>(input_index), &input_size);
  if (input == nullptr || input_size < payload_size) {
    __android_log_print(ANDROID_LOG_WARN, kLogTag, "H.264 input buffer too small: have=%zu need=%zu",
                        input_size, payload_size);
    return false;
  }

  std::memcpy(input, payload, payload_size);
  const media_status_t status = AMediaCodec_queueInputBuffer(
      codec, static_cast<std::size_t>(input_index), 0, payload_size, frame_index, 0);
  if (status != AMEDIA_OK) {
    __android_log_print(ANDROID_LOG_WARN, kLogTag, "H.264 queue input failed: %d", status);
    return false;
  }
  return true;
}

bool H264AnnexBDecoder::drainOutput(AMediaCodec* codec, std::vector<DecodedEyeFrame>* frames) {
  if (codec == nullptr || frames == nullptr || width_ == 0 || height_ == 0) {
    return false;
  }

  bool produced_frame = false;
  for (int iteration = 0; iteration < 4; ++iteration) {
    AMediaCodecBufferInfo info{};
    const ssize_t output_index = AMediaCodec_dequeueOutputBuffer(codec, &info, 0);
    if (output_index == AMEDIACODEC_INFO_TRY_AGAIN_LATER) {
      break;
    }
    if (output_index == AMEDIACODEC_INFO_OUTPUT_FORMAT_CHANGED) {
      updateOutputFormat(codec);
      continue;
    }
    if (output_index < 0) {
      __android_log_print(ANDROID_LOG_WARN, kLogTag, "H.264 dequeue output failed: %zd", output_index);
      break;
    }

    std::size_t output_size = 0;
    std::uint8_t* output = AMediaCodec_getOutputBuffer(codec, static_cast<std::size_t>(output_index), &output_size);
    if (output != nullptr && info.size > 0) {
      const std::int32_t stride = output_stride_ > 0 ? output_stride_ : static_cast<std::int32_t>(width_);
      const std::int32_t slice_height = output_slice_height_ > 0
          ? output_slice_height_
          : static_cast<std::int32_t>(height_);
      const std::size_t y_plane_size = static_cast<std::size_t>(stride) * slice_height;
      const std::size_t rgba_pixels = static_cast<std::size_t>(width_) * height_;
      if (output_size >= static_cast<std::size_t>(info.offset) + y_plane_size) {
        DecodedEyeFrame frame;
        const auto* frame_base = output + info.offset;
        const auto* y_plane = frame_base;
        const auto* uv_plane = frame_base + y_plane_size;
        frame.rgba.resize(rgba_pixels * 4);

        for (std::uint32_t y = 0; y < height_; ++y) {
          for (std::uint32_t x = 0; x < width_; ++x) {
            const std::size_t out_index = (static_cast<std::size_t>(y) * width_ + x) * 4;
            const std::uint8_t y_value = y_plane[static_cast<std::size_t>(y) * stride + x];
            std::uint8_t u_value = 128;
            std::uint8_t v_value = 128;

            if (output_color_format_ == 21) {
              const std::size_t uv_index = static_cast<std::size_t>(y / 2) * stride + (x / 2) * 2;
              if (info.offset + y_plane_size + uv_index + 1 < output_size) {
                u_value = uv_plane[uv_index + 0];
                v_value = uv_plane[uv_index + 1];
              }
            } else if (output_color_format_ == 19) {
              const std::size_t chroma_stride = static_cast<std::size_t>(stride) / 2;
              const std::size_t chroma_slice_height = static_cast<std::size_t>(slice_height) / 2;
              const auto* u_plane = frame_base + y_plane_size;
              const auto* v_plane = u_plane + chroma_stride * chroma_slice_height;
              const std::size_t uv_index = static_cast<std::size_t>(y / 2) * chroma_stride + (x / 2);
              if (info.offset + y_plane_size + chroma_stride * chroma_slice_height + uv_index < output_size) {
                u_value = u_plane[uv_index];
                v_value = v_plane[uv_index];
              }
            }

            yuvToRgb(y_value, u_value, v_value,
                     &frame.rgba[out_index + 0],
                     &frame.rgba[out_index + 1],
                     &frame.rgba[out_index + 2]);
            frame.rgba[out_index + 3] = 255;
          }
        }
        frame.frame_index = static_cast<std::uint64_t>(std::max<std::int64_t>(info.presentationTimeUs, 0));
        frame.valid = true;
        frames->push_back(std::move(frame));
        produced_frame = true;
      } else {
        __android_log_print(ANDROID_LOG_WARN, kLogTag,
                            "H.264 output buffer too small for luma: out=%zu offset=%d need=%zu",
                            output_size, info.offset, y_plane_size);
      }
    }
    AMediaCodec_releaseOutputBuffer(codec, static_cast<std::size_t>(output_index), false);
  }
  return produced_frame;
}

bool H264AnnexBDecoder::drainSurfaceOutput(AMediaCodec* codec, std::vector<std::uint64_t>* frame_indices) {
  if (codec == nullptr || frame_indices == nullptr) {
    return false;
  }

  bool produced_frame = false;
  struct PendingOutput {
    ssize_t index = -1;
    std::uint64_t frame_index = 0;
    bool render = false;
  };
  std::vector<PendingOutput> outputs;
  outputs.reserve(8);

  for (int iteration = 0; iteration < 32; ++iteration) {
    AMediaCodecBufferInfo info{};
    const ssize_t output_index = AMediaCodec_dequeueOutputBuffer(codec, &info, 0);
    if (output_index == AMEDIACODEC_INFO_TRY_AGAIN_LATER) {
      break;
    }
    if (output_index == AMEDIACODEC_INFO_OUTPUT_FORMAT_CHANGED) {
      updateOutputFormat(codec);
      continue;
    }
    if (output_index < 0) {
      __android_log_print(ANDROID_LOG_WARN, kLogTag, "H.264 surface dequeue output failed: %zd", output_index);
      break;
    }

    const std::uint64_t frame_index =
        static_cast<std::uint64_t>(std::max<std::int64_t>(info.presentationTimeUs, 0));
    outputs.push_back(PendingOutput{output_index, frame_index, info.size > 0});
  }

  std::optional<std::uint64_t> latest_render_frame;
  ssize_t latest_render_index = -1;
  for (const PendingOutput& output : outputs) {
    if (output.render && (!latest_render_frame.has_value() || output.frame_index > *latest_render_frame)) {
      latest_render_frame = output.frame_index;
      latest_render_index = output.index;
    }
  }

  for (const PendingOutput& output : outputs) {
    const bool render = output.index == latest_render_index;
    AMediaCodec_releaseOutputBuffer(codec, static_cast<std::size_t>(output.index), render);
    if (render) {
      frame_indices->push_back(output.frame_index);
      produced_frame = true;
    }
  }
  return produced_frame;
}

void H264AnnexBDecoder::updateOutputFormat(AMediaCodec* codec) {
  MediaFormatPtr output_format(AMediaCodec_getOutputFormat(codec));
  if (!output_format) {
    return;
  }
  AMediaFormat_getInt32(output_format.get(), AMEDIAFORMAT_KEY_STRIDE, &output_stride_);
  AMediaFormat_getInt32(output_format.get(), AMEDIAFORMAT_KEY_SLICE_HEIGHT, &output_slice_height_);
  AMediaFormat_getInt32(output_format.get(), AMEDIAFORMAT_KEY_COLOR_FORMAT, &output_color_format_);
  __android_log_print(ANDROID_LOG_INFO, kLogTag, "H.264 output format changed: %s",
                      AMediaFormat_toString(output_format.get()));
}

void H264AnnexBDecoder::resetStreamState() {
  decoded_left_by_frame_.clear();
  decoded_right_by_frame_.clear();
  surface_left_ready_.clear();
  surface_right_ready_.clear();
  receive_time_by_frame_.clear();
  received_frames_ = 0;
  queued_frames_ = 0;
  decoded_frames_ = 0;
  dropped_input_frames_ = 0;
  last_published_frame_index_ = 0;
  last_stats_time_ns_ = 0;
  latency_sum_ms_ = 0.0;
  latest_latency_ms_ = 0.0;
  has_published_frame_ = false;
  pending_acks_.clear();
}

void H264AnnexBDecoder::stop() {
  auto release_codec = [](AMediaCodec*& codec) {
    if (codec != nullptr) {
      AMediaCodec_stop(codec);
      AMediaCodec_delete(codec);
      codec = nullptr;
    }
  };
  release_codec(left_codec_);
  release_codec(right_codec_);
  if (left_window_ != nullptr) {
    ANativeWindow_release(left_window_);
    left_window_ = nullptr;
  }
  if (right_window_ != nullptr) {
    ANativeWindow_release(right_window_);
    right_window_ = nullptr;
  }
  surface_output_ = false;
  surface_generation_ = 0;
  width_ = 0;
  height_ = 0;
  output_stride_ = 0;
  output_slice_height_ = 0;
  output_color_format_ = 0;
  resetStreamState();
}

bool VideoDecoderRouter::handlePayload(std::string_view payload) {
  if (payload.size() < quest3_teleop::kVideoFrameHeaderSize) {
    return false;
  }

  const auto* bytes = reinterpret_cast<const std::uint8_t*>(payload.data());
  const quest3_teleop::VideoFrameHeader header = quest3_teleop::unpackVideoFrameHeader(bytes);
  const std::size_t left_size = header.left_payload_size;
  const std::size_t right_size = header.right_payload_size;
  const std::size_t expected_size = quest3_teleop::kVideoFrameHeaderSize + left_size + right_size;
  if (header.width == 0 || header.height == 0 || payload.size() != expected_size) {
    return false;
  }

  const auto* left_payload = bytes + quest3_teleop::kVideoFrameHeaderSize;
  const auto* right_payload = left_payload + left_size;
  switch (static_cast<quest3_teleop::VideoCodec>(header.codec)) {
    case quest3_teleop::VideoCodec::kRawRgba:
      return raw_rgba_.decode(header, left_payload, right_payload);
    case quest3_teleop::VideoCodec::kH264AnnexB:
      return h264_.decode(header, left_payload, right_payload);
    case quest3_teleop::VideoCodec::kH265AnnexB:
    case quest3_teleop::VideoCodec::kAv1AnnexB:
      logUnsupportedOnce(static_cast<quest3_teleop::VideoCodec>(header.codec));
      return false;
  }
  return false;
}

std::vector<VideoDecodeAck> VideoDecoderRouter::takeAcks() {
  return h264_.takeAcks();
}
