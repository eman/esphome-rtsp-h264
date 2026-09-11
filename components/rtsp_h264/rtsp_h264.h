#pragma once

// RTSP server for hardware-encoded H.264 on the ESP32-P4.
//
// Two tasks and a queue. The camera's capture task calls on_raw_frame() at
// the sensor's rate; frames are thinned to the configured rate, run through
// the hardware H.264 encoder, and the resulting access units are copied into
// a queue. The server task owns every socket: it accepts RTSP clients, answers
// their requests, and packetises each access unit into RTP (RFC 6184) for
// every client that is PLAYing - interleaved on the RTSP connection, or as
// UDP unicast. Nothing here runs on ESPHome's main loop.

#include "esphome/core/defines.h"

#ifdef USE_ESP32_VARIANT_ESP32P4

#include <atomic>
#include <string>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <lwip/sockets.h>

#include "esp_h264_enc_param_hw.h"
#include "esp_h264_enc_single.h"

#include "esphome/components/p4_csi_camera/p4_csi_camera.h"
#include "esphome/core/component.h"

namespace esphome {
namespace rtsp_h264 {

/// One encoded access unit on its way from the encoder to the server task.
struct EncodedFrame {
  uint8_t *data;
  size_t len;
  int64_t timestamp_us;
  bool keyframe;
};

enum class Transport : uint8_t { NONE, TCP, UDP };
enum class ClientState : uint8_t { INIT, READY, PLAYING };

struct Client {
  int fd{-1};
  std::string rx;
  uint32_t session{0};
  ClientState state{ClientState::INIT};
  Transport transport{Transport::NONE};
  uint8_t rtp_channel{0};
  struct sockaddr_in udp_addr {};
  uint16_t seq{0};
  uint32_t ssrc{0};
  uint32_t last_activity_ms{0};
  /// Nothing is sent until a keyframe has gone by, so a decoder never starts
  /// on a P-frame.
  bool waiting_for_keyframe{true};
  uint32_t packets_sent{0};
};

class RtspH264 : public Component, public p4_csi_camera::RawVideoSink {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  void set_camera(p4_csi_camera::P4CsiCamera *camera) { this->camera_ = camera; }
  void set_port(uint16_t port) { this->port_ = port; }
  void set_bitrate(uint32_t bps) { this->bitrate_ = bps; }
  void set_framerate(float fps);
  void set_gop(uint8_t gop) { this->gop_ = gop; }
  void set_qp(uint8_t qp_min, uint8_t qp_max) {
    this->qp_min_ = qp_min;
    this->qp_max_ = qp_max;
  }
  void set_max_clients(uint8_t n) { this->max_clients_ = n; }

  // p4_csi_camera::RawVideoSink, on the capture task.
  void on_raw_frame(const p4_csi_camera::RawFrame &frame) override;

 protected:
  // Encoder, capture-task side.
  bool ensure_encoder_(uint16_t width, uint16_t height, uint32_t fourcc);
  void destroy_encoder_();
  void note_parameter_sets_(const uint8_t *data, size_t len);

  // Server task.
  static void server_task_(void *arg);
  void server_loop_();
  bool open_listener_();
  void accept_clients_();
  void service_client_(Client &client);
  void handle_request_(Client &client, const std::string &request);
  void send_response_(Client &client, int code, const char *reason, const std::string &cseq,
                      const std::string &headers, const std::string &body);
  std::string describe_sdp_(int fd);
  bool parse_transport_(Client &client, const std::string &transport);
  void broadcast_(const EncodedFrame &frame);
  bool send_packet_(Client &client, const uint8_t *rtp, size_t len);
  bool flush_tcp_(Client &client);
  void drop_client_(Client &client);
  void update_streaming_state_();
  void set_streaming_(bool on);

  p4_csi_camera::P4CsiCamera *camera_{nullptr};
  uint16_t port_{8554};
  uint32_t bitrate_{2000000};
  uint8_t fps_{15};
  int64_t frame_interval_us_{66667};
  uint8_t gop_{30};
  uint8_t qp_min_{18};
  uint8_t qp_max_{45};
  uint8_t max_clients_{2};

  // Encoder state (capture task only, except the atomics).
  esp_h264_enc_handle_t encoder_{nullptr};
  esp_h264_enc_param_hw_handle_t encoder_param_{nullptr};
  uint16_t enc_width_{0};
  uint16_t enc_height_{0};
  uint8_t *out_buf_{nullptr};
  size_t out_len_{0};
  int64_t last_encoded_us_{0};
  std::atomic<bool> force_idr_{false};
  std::atomic<bool> encoder_wanted_{false};
  std::atomic<uint32_t> frames_encoded_{0};
  std::atomic<uint32_t> frames_dropped_{0};
  std::atomic<uint32_t> encode_failures_{0};
  std::atomic<uint32_t> encode_us_total_{0};

  // SPS/PPS, written by the capture task and read by the server task.
  portMUX_TYPE ps_lock_ = portMUX_INITIALIZER_UNLOCKED;
  uint8_t sps_[128] = {};
  uint8_t pps_[64] = {};
  size_t sps_len_{0};
  size_t pps_len_{0};
  std::atomic<bool> parameter_sets_ready_{false};

  // Server state (server task only).
  QueueHandle_t frame_queue_{nullptr};
  TaskHandle_t task_{nullptr};
  int listen_fd_{-1};
  int udp_fd_{-1};
  uint16_t udp_port_{0};
  std::vector<Client> clients_;
  bool streaming_{false};
  uint32_t last_client_ms_{0};
  std::vector<uint8_t> tcp_batch_;
  uint32_t last_stats_ms_{0};
  uint32_t bytes_sent_{0};
};

}  // namespace rtsp_h264
}  // namespace esphome

#endif  // USE_ESP32_VARIANT_ESP32P4
