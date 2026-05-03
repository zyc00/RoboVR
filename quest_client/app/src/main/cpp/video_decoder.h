#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "openxr_app.h"
#include "quest3_teleop_protocol.h"

struct AMediaCodec;
struct ANativeWindow;

struct VideoDecodeAck {
  std::uint64_t frame_index = 0;
  double quest_receive_to_publish_ms = 0.0;
  bool surface_output = false;
};

class VideoDecoder {
 public:
  virtual ~VideoDecoder() = default;
  virtual bool decode(const quest3_teleop::VideoFrameHeader& header,
                      const std::uint8_t* left_payload,
                      const std::uint8_t* right_payload) = 0;
};

class RawRgbaDecoder final : public VideoDecoder {
 public:
  bool decode(const quest3_teleop::VideoFrameHeader& header,
              const std::uint8_t* left_payload,
              const std::uint8_t* right_payload) override;
};

class H264AnnexBDecoder final : public VideoDecoder {
 public:
  ~H264AnnexBDecoder() override;

  bool decode(const quest3_teleop::VideoFrameHeader& header,
              const std::uint8_t* left_payload,
              const std::uint8_t* right_payload) override;
  std::vector<VideoDecodeAck> takeAcks();

 private:
  struct DecodedEyeFrame {
    std::vector<std::uint8_t> rgba;
    std::uint64_t frame_index = 0;
    bool valid = false;
  };

  bool ensureStarted(const quest3_teleop::VideoFrameHeader& header);
  bool queueAccessUnit(AMediaCodec* codec,
                       const std::uint8_t* payload,
                       std::size_t payload_size,
                       std::uint64_t frame_index,
                       bool keyframe);
  bool drainOutput(AMediaCodec* codec, std::vector<DecodedEyeFrame>* frames);
  bool drainSurfaceOutput(AMediaCodec* codec, std::vector<std::uint64_t>* frame_indices);
  void updateOutputFormat(AMediaCodec* codec);
  void resetStreamState();
  void stop();

  AMediaCodec* left_codec_ = nullptr;
  AMediaCodec* right_codec_ = nullptr;
  ANativeWindow* left_window_ = nullptr;
  ANativeWindow* right_window_ = nullptr;
  std::uint32_t surface_generation_ = 0;
  std::uint32_t width_ = 0;
  std::uint32_t height_ = 0;
  bool surface_output_ = false;
  std::int32_t output_stride_ = 0;
  std::int32_t output_slice_height_ = 0;
  std::int32_t output_color_format_ = 0;
  std::unordered_map<std::uint64_t, std::vector<std::uint8_t>> decoded_left_by_frame_;
  std::unordered_map<std::uint64_t, std::vector<std::uint8_t>> decoded_right_by_frame_;
  std::unordered_map<std::uint64_t, bool> surface_left_ready_;
  std::unordered_map<std::uint64_t, bool> surface_right_ready_;
  std::unordered_map<std::uint64_t, std::int64_t> receive_time_by_frame_;
  std::uint64_t received_frames_ = 0;
  std::uint64_t queued_frames_ = 0;
  std::uint64_t decoded_frames_ = 0;
  std::uint64_t dropped_input_frames_ = 0;
  std::uint64_t last_published_frame_index_ = 0;
  std::int64_t last_stats_time_ns_ = 0;
  double latency_sum_ms_ = 0.0;
  double latest_latency_ms_ = 0.0;
  bool has_published_frame_ = false;
  std::vector<VideoDecodeAck> pending_acks_;
};

class VideoDecoderRouter {
 public:
  bool handlePayload(std::string_view payload);
  std::vector<VideoDecodeAck> takeAcks();

 private:
  RawRgbaDecoder raw_rgba_;
  H264AnnexBDecoder h264_;
};
