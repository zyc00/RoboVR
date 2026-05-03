# Protocol

All packets begin with a fixed 32-byte little-endian header:

```cpp
struct PacketHeader {
  uint32_t magic;        // "Q3TP" as 0x50543351 on little-endian hosts
  uint16_t version;      // currently 1
  uint16_t type;         // HELLO, CONFIG, VIDEO, POSE, STATS, HEARTBEAT
  uint64_t seq;
  int64_t timestamp_ns;  // monotonic clock at sender
  uint32_t payload_size;
  uint32_t crc32;        // reserved, currently 0
};
```

MVP packet types:

- `HELLO`: version and role negotiation.
- `HEARTBEAT`: connection liveness.
- `POSE`: head/controller pose samples.
- `STATS`: timing and queue metrics.
- `VIDEO`: binary video frame payload.

`VIDEO` payloads begin with a fixed little-endian `VideoFrameHeader`, followed by
left-eye bytes and then right-eye bytes:

```cpp
enum class VideoCodec : uint16_t {
  kRawRgba = 1,
  kH264AnnexB = 2,
  kH265AnnexB = 3,
  kAv1AnnexB = 4,
};

struct VideoFrameHeader {
  uint16_t codec;
  uint16_t flags;              // bit 0: keyframe
  uint32_t stream_id;
  uint64_t frame_index;
  int64_t capture_time_ns;
  uint32_t width;
  uint32_t height;
  uint32_t left_payload_size;
  uint32_t right_payload_size;
};
```

For `kRawRgba`, each eye payload is tightly packed RGBA8 with
`width * height * 4` bytes. H.264/HEVC/AV1 will use the same envelope with
codec-specific access units in each eye payload.
