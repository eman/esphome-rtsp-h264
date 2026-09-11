#include "rtsp_h264.h"

#ifdef USE_ESP32_VARIANT_ESP32P4

#include <algorithm>
#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <lwip/netdb.h>

// No extern "C" guards of its own.
extern "C" {
#include "esp_h264_alloc.h"
}
#include "esp_h264_enc_single_hw.h"
#include "esp_heap_caps.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "linux/videodev2.h"

#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

// The hardware encoder insists on internal RAM for its reference-frame buffer:
// 135 KB, contiguous, for 1080p. On a panel that also runs a display, Wi-Fi
// over ESP-Hosted and ESPHome's API, internal RAM is the scarce one - this
// board had 150 KB free with an 84 KB largest block when a viewer connected -
// and taking 135 KB of it would starve the network stack. So the encoder's
// allocator is wrapped (-Wl,--wrap in __init__.py): internal RAM first, as
// upstream wants, then PSRAM. Only the encoder's DMA ever touches the
// reference frame, so there is no CPU cache to keep coherent, and the frame
// data itself already streams through PSRAM.
extern "C" void *__real_esp_h264_aligned_malloc(uint32_t alignment, uint32_t n, uint32_t size, uint32_t *actual_size,
                                                uint32_t caps);
extern "C" void *__wrap_esp_h264_aligned_malloc(uint32_t alignment, uint32_t n, uint32_t size, uint32_t *actual_size,
                                                uint32_t caps) {
  void *p = __real_esp_h264_aligned_malloc(alignment, n, size, actual_size, caps);
  if (p != nullptr || !(caps & MALLOC_CAP_INTERNAL))
    return p;
  ESP_LOGW("rtsp_h264", "encoder wanted %u KB of internal RAM (largest free block %u KB); using PSRAM",
           (unsigned) ((n * size) / 1024), (unsigned) (heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024));
  // DMA into PSRAM wants cache-line alignment and a cache-line multiple.
  const uint32_t align = alignment < 128 ? 128 : alignment;
  const uint32_t bytes = ((n * size) + align - 1) / align * align;
  p = heap_caps_aligned_alloc(align, bytes, (caps & ~MALLOC_CAP_INTERNAL) | MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (p != nullptr && actual_size != nullptr)
    *actual_size = bytes;
  return p;
}

namespace esphome {
namespace rtsp_h264 {

static const char *const TAG = "rtsp_h264";

// RTP payload type for dynamic H.264, as advertised in the SDP.
static const uint8_t RTP_PAYLOAD_TYPE = 96;
// Largest RTP packet, header included. Fits an Ethernet MTU with room for
// IP/UDP, and is what every RTSP client expects to reassemble.
static const size_t RTP_MAX_PACKET = 1400;
static const size_t RTP_HEADER = 12;
// Encoded frames queued between the encoder and the server task.
static const UBaseType_t FRAME_QUEUE_DEPTH = 3;
// A client that has said nothing for this long is gone.
static const uint32_t CLIENT_TIMEOUT_MS = 120000;
// How long the camera keeps streaming after the last client leaves, so a
// viewer that reconnects does not wait for the pipeline to come up again.
static const uint32_t STREAM_LINGER_MS = 10000;
// How long DESCRIBE waits for the first keyframe to yield SPS and PPS.
static const uint32_t PARAMETER_SET_WAIT_MS = 3000;
// One socket write may block this long before the client is judged stuck.
static const int SEND_TIMEOUT_MS = 2000;
// TCP writes are batched into this much before a send().
static const size_t TCP_BATCH = 8192;

/* ---------------- configuration ---------------- */

void RtspH264::set_framerate(float fps) {
  if (fps < 1.0f)
    fps = 1.0f;
  if (fps > 30.0f)
    fps = 30.0f;
  this->fps_ = (uint8_t) (fps + 0.5f);
  this->frame_interval_us_ = (int64_t) (1000000.0f / fps);
}

void RtspH264::setup() {
  if (this->camera_ == nullptr) {
    this->mark_failed();
    return;
  }
  this->frame_queue_ = xQueueCreate(FRAME_QUEUE_DEPTH, sizeof(EncodedFrame));
  if (this->frame_queue_ == nullptr) {
    ESP_LOGE(TAG, "could not create the frame queue");
    this->mark_failed();
    return;
  }
  this->tcp_batch_.reserve(TCP_BATCH);
  this->camera_->add_raw_sink(this);
  ESP_LOGI(TAG, "at setup: internal RAM free %u KB, largest block %u KB",
           (unsigned) (heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
           (unsigned) (heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024));

  // The server has its own task: socket calls block, and ESPHome's main loop
  // is watchdogged. Core 1, alongside the capture task and away from LVGL.
  if (xTaskCreatePinnedToCore(RtspH264::server_task_, "rtsp", 8192, this, 4, &this->task_, 1) != pdPASS) {
    ESP_LOGE(TAG, "could not start the server task");
    this->task_ = nullptr;
    this->mark_failed();
  }
}

void RtspH264::dump_config() {
  ESP_LOGCONFIG(TAG, "RTSP H.264 server:");
  ESP_LOGCONFIG(TAG, "  URL: rtsp://<this device>:%u/stream", this->port_);
  ESP_LOGCONFIG(TAG, "  Bit rate: %u kbit/s, %u fps, keyframe every %u frames, QP %u..%u",
                (unsigned) (this->bitrate_ / 1000), this->fps_, this->gop_, this->qp_min_, this->qp_max_);
  ESP_LOGCONFIG(TAG, "  Max clients: %u", this->max_clients_);
}

/* ---------------- encoder (capture task) ---------------- */

bool RtspH264::ensure_encoder_(uint16_t width, uint16_t height, uint32_t fourcc) {
  if (this->encoder_ != nullptr && this->enc_width_ == width && this->enc_height_ == height)
    return true;
  this->destroy_encoder_();

  if (fourcc != V4L2_PIX_FMT_YUV420) {
    ESP_LOGE(TAG, "the hardware encoder needs YUV 4:2:0 frames; the camera delivers '%c%c%c%c'", (char) (fourcc & 0xFF),
             (char) ((fourcc >> 8) & 0xFF), (char) ((fourcc >> 16) & 0xFF), (char) ((fourcc >> 24) & 0xFF));
    return false;
  }

  esp_h264_enc_cfg_hw_t cfg = {};
  // What the ESP32-P4 ISP writes for V4L2_PIX_FMT_YUV420, and what its
  // encoder reads: odd lines U Y Y, even lines V Y Y.
  cfg.pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY;
  cfg.gop = this->gop_;
  cfg.fps = this->fps_;
  cfg.res.width = width;
  cfg.res.height = height;
  cfg.rc.bitrate = this->bitrate_;
  cfg.rc.qp_min = this->qp_min_;
  cfg.rc.qp_max = this->qp_max_;

  const size_t internal_before = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  esp_h264_err_t err = esp_h264_enc_hw_new(&cfg, &this->encoder_);
  if (err != ESP_H264_ERR_OK) {
    // The encoder wants its reference-frame and deblocking buffers in
    // internal RAM, and says only "no memory" when it cannot have them.
    if (this->encode_failures_ % 30 == 0)
      ESP_LOGE(TAG, "esp_h264_enc_hw_new failed (%d); internal RAM free %u KB, largest block %u KB", (int) err,
               (unsigned) (heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
               (unsigned) (heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024));
    this->encoder_ = nullptr;
    return false;
  }
  ESP_LOGI(TAG, "encoder took %u KB of internal RAM; %u KB free, largest block %u KB",
           (unsigned) ((internal_before - heap_caps_get_free_size(MALLOC_CAP_INTERNAL)) / 1024),
           (unsigned) (heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
           (unsigned) (heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024));
  err = esp_h264_enc_open(this->encoder_);
  if (err != ESP_H264_ERR_OK) {
    ESP_LOGE(TAG, "esp_h264_enc_open failed (%d)", (int) err);
    esp_h264_enc_del(this->encoder_);
    this->encoder_ = nullptr;
    return false;
  }
  if (esp_h264_enc_hw_get_param_hd(this->encoder_, &this->encoder_param_) != ESP_H264_ERR_OK)
    this->encoder_param_ = nullptr;

  // The encoder may write up to one uncompressed frame's worth, with the
  // dimensions rounded up to whole macroblocks.
  const size_t aw = (width + 15) & ~15u, ah = (height + 15) & ~15u;
  const size_t need = aw * ah * 3 / 2;
  if (this->out_buf_ == nullptr || this->out_len_ < need) {
    if (this->out_buf_ != nullptr)
      heap_caps_free(this->out_buf_);
    uint32_t actual = 0;
    this->out_buf_ = (uint8_t *) esp_h264_aligned_calloc(16, 1, need, &actual, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    this->out_len_ = actual;
    if (this->out_buf_ == nullptr) {
      ESP_LOGE(TAG, "no memory for the encoder's output buffer (%u bytes)", (unsigned) need);
      this->destroy_encoder_();
      return false;
    }
  }

  this->enc_width_ = width;
  this->enc_height_ = height;
  this->parameter_sets_ready_ = false;
  this->force_idr_ = true;
  ESP_LOGI(TAG, "H.264 encoder up: %ux%u, %u kbit/s, %u fps, GOP %u", width, height,
           (unsigned) (this->bitrate_ / 1000), this->fps_, this->gop_);
  return true;
}

void RtspH264::destroy_encoder_() {
  if (this->encoder_ != nullptr) {
    esp_h264_enc_close(this->encoder_);
    esp_h264_enc_del(this->encoder_);
    this->encoder_ = nullptr;
    this->encoder_param_ = nullptr;
    ESP_LOGI(TAG, "H.264 encoder released");
  }
  this->enc_width_ = 0;
  this->enc_height_ = 0;
}

/// Walks the Annex-B byte stream for SPS (type 7) and PPS (type 8) NAL units
/// and keeps a copy of each for the SDP.
void RtspH264::note_parameter_sets_(const uint8_t *data, size_t len) {
  size_t i = 0;
  while (i + 4 <= len) {
    // Start code: 00 00 01 or 00 00 00 01.
    if (!(data[i] == 0 && data[i + 1] == 0 && (data[i + 2] == 1 || (data[i + 2] == 0 && i + 4 < len && data[i + 3] == 1)))) {
      i++;
      continue;
    }
    const size_t nal_start = i + (data[i + 2] == 1 ? 3 : 4);
    // Find the next start code.
    size_t nal_end = len;
    for (size_t j = nal_start; j + 3 <= len; j++) {
      if (data[j] == 0 && data[j + 1] == 0 && (data[j + 2] == 1 || (j + 4 <= len && data[j + 2] == 0 && data[j + 3] == 1))) {
        nal_end = j;
        break;
      }
    }
    if (nal_start < nal_end) {
      const uint8_t type = data[nal_start] & 0x1F;
      const size_t n = nal_end - nal_start;
      if (type == 7 && n <= sizeof(this->sps_)) {
        portENTER_CRITICAL(&this->ps_lock_);
        memcpy(this->sps_, data + nal_start, n);
        this->sps_len_ = n;
        portEXIT_CRITICAL(&this->ps_lock_);
      } else if (type == 8 && n <= sizeof(this->pps_)) {
        portENTER_CRITICAL(&this->ps_lock_);
        memcpy(this->pps_, data + nal_start, n);
        this->pps_len_ = n;
        portEXIT_CRITICAL(&this->ps_lock_);
      }
    }
    i = nal_end;
  }
  if (this->sps_len_ > 0 && this->pps_len_ > 0)
    this->parameter_sets_ready_ = true;
}

void RtspH264::on_raw_frame(const p4_csi_camera::RawFrame &frame) {
  if (!this->encoder_wanted_) {
    // Nobody is watching. Let the encoder go the first time a frame arrives
    // after the last client left; the camera stops delivering soon after.
    if (this->encoder_ != nullptr)
      this->destroy_encoder_();
    return;
  }

  // Thin the sensor's 30 fps to the configured rate. A little slack so that
  // jitter in frame arrival does not skip a frame that is due.
  if (frame.timestamp_us - this->last_encoded_us_ < this->frame_interval_us_ - 3000)
    return;

  if (!this->ensure_encoder_(frame.width, frame.height, frame.fourcc)) {
    this->encode_failures_++;
    return;
  }

  if (this->force_idr_.exchange(false) && this->encoder_param_ != nullptr)
    esp_h264_enc_force_idr(&this->encoder_param_->base);

  esp_h264_enc_in_frame_t in = {};
  in.raw_data.buffer = const_cast<uint8_t *>(frame.data);
  in.raw_data.len = frame.len;
  in.pts = (uint32_t) (frame.timestamp_us / 1000);
  esp_h264_enc_out_frame_t out = {};
  out.raw_data.buffer = this->out_buf_;
  out.raw_data.len = this->out_len_;

  const int64_t t0 = esp_timer_get_time();
  const esp_h264_err_t err = esp_h264_enc_process(this->encoder_, &in, &out);
  this->encode_us_total_ += (uint32_t) (esp_timer_get_time() - t0);
  if (err != ESP_H264_ERR_OK) {
    this->encode_failures_++;
    if (this->encode_failures_ % 30 == 1)
      ESP_LOGW(TAG, "esp_h264_enc_process failed (%d), %u failures so far", (int) err,
               (unsigned) this->encode_failures_.load());
    return;
  }
  this->last_encoded_us_ = frame.timestamp_us;
  this->frames_encoded_++;

  const bool keyframe = out.frame_type == ESP_H264_FRAME_TYPE_IDR || out.frame_type == ESP_H264_FRAME_TYPE_I;
  if (out.frame_type == ESP_H264_FRAME_TYPE_IDR)
    this->note_parameter_sets_(out.raw_data.buffer, out.length);

  // A copy the server task can send at its own pace.
  EncodedFrame ef{};
  ef.len = out.length;
  ef.data = (uint8_t *) heap_caps_malloc(ef.len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (ef.data == nullptr) {
    this->frames_dropped_++;
    return;
  }
  memcpy(ef.data, out.raw_data.buffer, ef.len);
  ef.timestamp_us = frame.timestamp_us;
  ef.keyframe = keyframe;
  if (xQueueSend(this->frame_queue_, &ef, 0) != pdTRUE) {
    // The server task is behind: the network is slower than the encoder. A
    // dropped frame breaks every P-frame that follows until the next
    // keyframe, so ask for one now rather than leave the viewer with
    // garbage for up to a GOP.
    heap_caps_free(ef.data);
    this->frames_dropped_++;
    this->force_idr_ = true;
  }
}

/* ---------------- server task ---------------- */

void RtspH264::server_task_(void *arg) { static_cast<RtspH264 *>(arg)->server_loop_(); }

bool RtspH264::open_listener_() {
  this->listen_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (this->listen_fd_ < 0) {
    ESP_LOGE(TAG, "socket() failed (errno %d)", errno);
    return false;
  }
  int one = 1;
  setsockopt(this->listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  struct sockaddr_in addr = {};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(this->port_);
  if (bind(this->listen_fd_, (struct sockaddr *) &addr, sizeof(addr)) != 0 || listen(this->listen_fd_, 2) != 0) {
    ESP_LOGE(TAG, "could not listen on port %u (errno %d)", this->port_, errno);
    close(this->listen_fd_);
    this->listen_fd_ = -1;
    return false;
  }
  fcntl(this->listen_fd_, F_SETFL, fcntl(this->listen_fd_, F_GETFL, 0) | O_NONBLOCK);

  // One UDP socket serves every UDP client; RTCP on the port above it is
  // accepted and ignored.
  this->udp_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (this->udp_fd_ >= 0) {
    struct sockaddr_in uaddr = {};
    uaddr.sin_family = AF_INET;
    uaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    this->udp_port_ = (uint16_t) (this->port_ + 2);
    uaddr.sin_port = htons(this->udp_port_);
    if (bind(this->udp_fd_, (struct sockaddr *) &uaddr, sizeof(uaddr)) != 0) {
      ESP_LOGW(TAG, "could not bind the RTP/UDP socket (errno %d); UDP transport disabled", errno);
      close(this->udp_fd_);
      this->udp_fd_ = -1;
    } else {
      fcntl(this->udp_fd_, F_SETFL, fcntl(this->udp_fd_, F_GETFL, 0) | O_NONBLOCK);
    }
  }
  ESP_LOGI(TAG, "listening on rtsp://0.0.0.0:%u/stream", this->port_);
  return true;
}

void RtspH264::server_loop_() {
  // The network stack is up once the socket binds; try until it does.
  while (!this->open_listener_())
    vTaskDelay(pdMS_TO_TICKS(2000));

  for (;;) {
    EncodedFrame frame{};
    if (xQueueReceive(this->frame_queue_, &frame, pdMS_TO_TICKS(20)) == pdTRUE) {
      this->broadcast_(frame);
      heap_caps_free(frame.data);
    }

    this->accept_clients_();
    for (size_t i = 0; i < this->clients_.size();) {
      this->service_client_(this->clients_[i]);
      if (this->clients_[i].fd < 0) {
        this->clients_.erase(this->clients_.begin() + i);
      } else {
        i++;
      }
    }
    this->update_streaming_state_();

    const uint32_t now = millis();
    if (this->streaming_ && now - this->last_stats_ms_ > 10000) {
      this->last_stats_ms_ = now;
      const uint32_t encoded = this->frames_encoded_.exchange(0);
      const uint32_t us = this->encode_us_total_.exchange(0);
      ESP_LOGD(TAG, "%u clients; last 10 s: %u frames encoded (%u ms each), %u dropped, %u encode failures, %u KB sent",
               (unsigned) this->clients_.size(), (unsigned) encoded, (unsigned) (encoded ? us / encoded / 1000 : 0),
               (unsigned) this->frames_dropped_.exchange(0), (unsigned) this->encode_failures_.load(),
               (unsigned) (this->bytes_sent_ / 1024));
      this->bytes_sent_ = 0;
    }
  }
}

void RtspH264::set_streaming_(bool on) {
  if (on == this->streaming_)
    return;
  this->streaming_ = on;
  this->encoder_wanted_ = on;
  this->camera_->set_raw_streaming(this, on);
  ESP_LOGI(TAG, "camera stream %s", on ? "requested" : "released");
}

void RtspH264::update_streaming_state_() {
  if (!this->clients_.empty()) {
    this->last_client_ms_ = millis();
    this->set_streaming_(true);
  } else if (this->streaming_ && millis() - this->last_client_ms_ > STREAM_LINGER_MS) {
    this->set_streaming_(false);
  }
}

void RtspH264::accept_clients_() {
  struct sockaddr_in addr = {};
  socklen_t len = sizeof(addr);
  const int fd = accept(this->listen_fd_, (struct sockaddr *) &addr, &len);
  if (fd < 0)
    return;
  if (this->clients_.size() >= this->max_clients_) {
    ESP_LOGW(TAG, "refusing a client: %u already connected", (unsigned) this->clients_.size());
    close(fd);
    return;
  }
  // Reads are polled without blocking; writes may block briefly, and a client
  // that cannot take a frame in that time is dropped rather than stalling
  // the others.
  struct timeval tv = {};
  tv.tv_sec = SEND_TIMEOUT_MS / 1000;
  tv.tv_usec = (SEND_TIMEOUT_MS % 1000) * 1000;
  setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  int one = 1;
  setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

  Client client;
  client.fd = fd;
  client.session = esp_random();
  client.ssrc = esp_random();
  client.seq = (uint16_t) esp_random();
  client.last_activity_ms = millis();
  this->clients_.push_back(client);
  char ip[16];
  inet_ntoa_r(addr.sin_addr, ip, sizeof(ip));
  ESP_LOGI(TAG, "client %s connected", ip);
}

void RtspH264::drop_client_(Client &client) {
  if (client.fd >= 0) {
    close(client.fd);
    client.fd = -1;
  }
  client.state = ClientState::INIT;
}

void RtspH264::service_client_(Client &client) {
  char buf[512];
  const int n = recv(client.fd, buf, sizeof(buf), MSG_DONTWAIT);
  if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
    ESP_LOGI(TAG, "client disconnected");
    this->drop_client_(client);
    return;
  }
  if (n > 0) {
    client.rx.append(buf, (size_t) n);
    client.last_activity_ms = millis();
    if (client.rx.size() > 8192) {
      ESP_LOGW(TAG, "client sent 8 KB without a complete request; dropping it");
      this->drop_client_(client);
      return;
    }
  }
  if (millis() - client.last_activity_ms > CLIENT_TIMEOUT_MS) {
    ESP_LOGI(TAG, "client timed out");
    this->drop_client_(client);
    return;
  }

  // Interleaved RTCP or anything else binary from the client: discard it.
  while (!client.rx.empty() && client.rx[0] == '$') {
    if (client.rx.size() < 4)
      return;
    const size_t len = ((uint8_t) client.rx[2] << 8) | (uint8_t) client.rx[3];
    if (client.rx.size() < 4 + len)
      return;
    client.rx.erase(0, 4 + len);
  }

  // One or more complete requests.
  for (;;) {
    const size_t end = client.rx.find("\r\n\r\n");
    if (end == std::string::npos)
      return;
    // A body (Content-Length) is possible on SET_PARAMETER; wait for it.
    size_t body = 0;
    const size_t cl = client.rx.find("Content-Length:");
    if (cl != std::string::npos && cl < end)
      body = (size_t) atoi(client.rx.c_str() + cl + 15);
    if (client.rx.size() < end + 4 + body)
      return;
    const std::string request = client.rx.substr(0, end + 4);
    client.rx.erase(0, end + 4 + body);
    this->handle_request_(client, request);
    if (client.fd < 0)
      return;
  }
}

static std::string header_value(const std::string &request, const char *name) {
  const std::string key = std::string("\r\n") + name + ":";
  size_t pos = request.find(key);
  if (pos == std::string::npos) {
    // Case-insensitive fallback for the common headers clients vary on.
    std::string lower = request;
    for (auto &c : lower)
      c = (char) tolower(c);
    std::string lkey = key;
    for (auto &c : lkey)
      c = (char) tolower(c);
    pos = lower.find(lkey);
    if (pos == std::string::npos)
      return "";
  }
  pos += key.size();
  const size_t end = request.find("\r\n", pos);
  std::string v = request.substr(pos, end == std::string::npos ? std::string::npos : end - pos);
  while (!v.empty() && v.front() == ' ')
    v.erase(0, 1);
  while (!v.empty() && (v.back() == ' ' || v.back() == '\r'))
    v.pop_back();
  return v;
}

void RtspH264::send_response_(Client &client, int code, const char *reason, const std::string &cseq,
                              const std::string &headers, const std::string &body) {
  std::string r = "RTSP/1.0 " + std::to_string(code) + " " + reason + "\r\n";
  r += "CSeq: " + cseq + "\r\n";
  r += "Server: ESPHome rtsp_h264\r\n";
  r += headers;
  if (!body.empty()) {
    r += "Content-Type: application/sdp\r\n";
    r += "Content-Length: " + std::to_string(body.size()) + "\r\n";
  }
  r += "\r\n";
  r += body;
  if (send(client.fd, r.data(), r.size(), 0) != (int) r.size())
    this->drop_client_(client);
}

std::string RtspH264::describe_sdp_(int fd) {
  // Our own address, for the origin line.
  struct sockaddr_in local = {};
  socklen_t len = sizeof(local);
  char ip[16] = "0.0.0.0";
  if (getsockname(fd, (struct sockaddr *) &local, &len) == 0)
    inet_ntoa_r(local.sin_addr, ip, sizeof(ip));

  std::string fmtp = "a=fmtp:96 packetization-mode=1";
  uint8_t sps[sizeof(this->sps_)], pps[sizeof(this->pps_)];
  size_t sps_len, pps_len;
  portENTER_CRITICAL(&this->ps_lock_);
  sps_len = this->sps_len_;
  pps_len = this->pps_len_;
  memcpy(sps, this->sps_, sps_len);
  memcpy(pps, this->pps_, pps_len);
  portEXIT_CRITICAL(&this->ps_lock_);
  if (sps_len >= 4 && pps_len > 0) {
    char profile[8];
    snprintf(profile, sizeof(profile), "%02X%02X%02X", sps[1], sps[2], sps[3]);
    fmtp += ";profile-level-id=";
    fmtp += profile;
    fmtp += ";sprop-parameter-sets=" + base64_encode(sps, sps_len) + "," + base64_encode(pps, pps_len);
  }

  std::string sdp;
  sdp += "v=0\r\n";
  sdp += "o=- " + std::to_string(esp_random()) + " 1 IN IP4 " + ip + "\r\n";
  sdp += "s=ESPHome camera\r\n";
  sdp += "c=IN IP4 0.0.0.0\r\n";
  sdp += "t=0 0\r\n";
  sdp += "a=control:*\r\n";
  sdp += "m=video 0 RTP/AVP 96\r\n";
  sdp += "a=rtpmap:96 H264/90000\r\n";
  sdp += fmtp + "\r\n";
  sdp += "a=framerate:" + std::to_string(this->fps_) + "\r\n";
  sdp += "a=control:track0\r\n";
  return sdp;
}

bool RtspH264::parse_transport_(Client &client, const std::string &transport) {
  if (transport.find("multicast") != std::string::npos)
    return false;
  if (transport.find("RTP/AVP/TCP") != std::string::npos) {
    client.transport = Transport::TCP;
    client.rtp_channel = 0;
    const size_t p = transport.find("interleaved=");
    if (p != std::string::npos)
      client.rtp_channel = (uint8_t) atoi(transport.c_str() + p + 12);
    return true;
  }
  if (transport.find("RTP/AVP") != std::string::npos) {
    if (this->udp_fd_ < 0)
      return false;
    const size_t p = transport.find("client_port=");
    if (p == std::string::npos)
      return false;
    client.transport = Transport::UDP;
    struct sockaddr_in peer = {};
    socklen_t len = sizeof(peer);
    if (getpeername(client.fd, (struct sockaddr *) &peer, &len) != 0)
      return false;
    client.udp_addr = peer;
    client.udp_addr.sin_port = htons((uint16_t) atoi(transport.c_str() + p + 12));
    return true;
  }
  return false;
}

void RtspH264::handle_request_(Client &client, const std::string &request) {
  const size_t sp1 = request.find(' ');
  const size_t sp2 = sp1 == std::string::npos ? std::string::npos : request.find(' ', sp1 + 1);
  if (sp1 == std::string::npos || sp2 == std::string::npos) {
    this->send_response_(client, 400, "Bad Request", "0", "", "");
    return;
  }
  const std::string method = request.substr(0, sp1);
  const std::string url = request.substr(sp1 + 1, sp2 - sp1 - 1);
  std::string cseq = header_value(request, "CSeq");
  if (cseq.empty())
    cseq = "0";
  ESP_LOGD(TAG, "%s %s", method.c_str(), url.c_str());

  const std::string session_hdr = "Session: " + str_sprintf("%08X", (unsigned) client.session) + ";timeout=60\r\n";

  if (method == "OPTIONS") {
    this->send_response_(client, 200, "OK", cseq,
                         "Public: OPTIONS, DESCRIBE, SETUP, PLAY, PAUSE, TEARDOWN, GET_PARAMETER, SET_PARAMETER\r\n", "");
  } else if (method == "DESCRIBE") {
    // Start the camera and the encoder now, so the SDP can carry the
    // parameter sets: a keyframe is needed for that, and one is forced.
    this->set_streaming_(true);
    this->force_idr_ = true;
    const uint32_t start = millis();
    while (!this->parameter_sets_ready_ && millis() - start < PARAMETER_SET_WAIT_MS)
      vTaskDelay(pdMS_TO_TICKS(50));
    if (!this->parameter_sets_ready_)
      ESP_LOGW(TAG, "no keyframe within %u ms; describing without parameter sets", (unsigned) PARAMETER_SET_WAIT_MS);
    const std::string sdp = this->describe_sdp_(client.fd);
    this->send_response_(client, 200, "OK", cseq, "Content-Base: " + url + "/\r\n", sdp);
  } else if (method == "SETUP") {
    const std::string transport = header_value(request, "Transport");
    if (!this->parse_transport_(client, transport)) {
      this->send_response_(client, 461, "Unsupported Transport", cseq, "", "");
      return;
    }
    std::string th;
    if (client.transport == Transport::TCP) {
      th = str_sprintf("Transport: RTP/AVP/TCP;unicast;interleaved=%u-%u;ssrc=%08X\r\n", client.rtp_channel,
                       client.rtp_channel + 1, (unsigned) client.ssrc);
    } else {
      th = str_sprintf("Transport: RTP/AVP;unicast;client_port=%u-%u;server_port=%u-%u;ssrc=%08X\r\n",
                       ntohs(client.udp_addr.sin_port), ntohs(client.udp_addr.sin_port) + 1, this->udp_port_,
                       this->udp_port_ + 1, (unsigned) client.ssrc);
    }
    client.state = ClientState::READY;
    this->send_response_(client, 200, "OK", cseq, th + session_hdr, "");
  } else if (method == "PLAY") {
    if (client.state == ClientState::INIT) {
      this->send_response_(client, 455, "Method Not Valid in This State", cseq, "", "");
      return;
    }
    client.state = ClientState::PLAYING;
    client.waiting_for_keyframe = true;
    this->force_idr_ = true;
    const std::string info = "RTP-Info: url=" + url + "/track0;seq=" + std::to_string(client.seq) + "\r\n";
    this->send_response_(client, 200, "OK", cseq, session_hdr + "Range: npt=0.000-\r\n" + info, "");
    ESP_LOGI(TAG, "client playing over %s", client.transport == Transport::TCP ? "TCP" : "UDP");
  } else if (method == "PAUSE") {
    if (client.state == ClientState::PLAYING)
      client.state = ClientState::READY;
    this->send_response_(client, 200, "OK", cseq, session_hdr, "");
  } else if (method == "TEARDOWN") {
    this->send_response_(client, 200, "OK", cseq, session_hdr, "");
    this->drop_client_(client);
  } else if (method == "GET_PARAMETER" || method == "SET_PARAMETER") {
    this->send_response_(client, 200, "OK", cseq, session_hdr, "");
  } else {
    this->send_response_(client, 501, "Not Implemented", cseq, "", "");
  }
}

/* ---------------- RTP ---------------- */

bool RtspH264::flush_tcp_(Client &client) {
  if (this->tcp_batch_.empty())
    return true;
  size_t off = 0;
  while (off < this->tcp_batch_.size()) {
    const int n = send(client.fd, this->tcp_batch_.data() + off, this->tcp_batch_.size() - off, 0);
    if (n <= 0) {
      this->tcp_batch_.clear();
      return false;
    }
    off += (size_t) n;
  }
  this->bytes_sent_ += this->tcp_batch_.size();
  this->tcp_batch_.clear();
  return true;
}

bool RtspH264::send_packet_(Client &client, const uint8_t *rtp, size_t len) {
  client.packets_sent++;
  if (client.transport == Transport::TCP) {
    if (this->tcp_batch_.size() + 4 + len > TCP_BATCH && !this->flush_tcp_(client))
      return false;
    const uint8_t hdr[4] = {'$', client.rtp_channel, (uint8_t) (len >> 8), (uint8_t) (len & 0xFF)};
    this->tcp_batch_.insert(this->tcp_batch_.end(), hdr, hdr + 4);
    this->tcp_batch_.insert(this->tcp_batch_.end(), rtp, rtp + len);
    return true;
  }
  const int n = sendto(this->udp_fd_, rtp, len, 0, (struct sockaddr *) &client.udp_addr, sizeof(client.udp_addr));
  if (n == (int) len)
    this->bytes_sent_ += len;
  // A UDP send that fails is a dropped packet, not a dropped client.
  return true;
}

void RtspH264::broadcast_(const EncodedFrame &frame) {
  bool anyone = false;
  for (auto &c : this->clients_)
    anyone |= c.state == ClientState::PLAYING;
  if (!anyone)
    return;

  // Split the access unit into NAL units on Annex-B start codes.
  struct Nal {
    const uint8_t *p;
    size_t n;
  };
  Nal nals[16];
  size_t nal_count = 0;
  const uint8_t *d = frame.data;
  const size_t len = frame.len;
  size_t i = 0;
  while (i + 3 <= len && nal_count < 16) {
    if (!(d[i] == 0 && d[i + 1] == 0 && (d[i + 2] == 1 || (i + 4 <= len && d[i + 2] == 0 && d[i + 3] == 1)))) {
      i++;
      continue;
    }
    const size_t start = i + (d[i + 2] == 1 ? 3 : 4);
    size_t end = len;
    for (size_t j = start; j + 3 <= len; j++) {
      if (d[j] == 0 && d[j + 1] == 0 && (d[j + 2] == 1 || (j + 4 <= len && d[j + 2] == 0 && d[j + 3] == 1))) {
        end = j;
        break;
      }
    }
    if (start < end)
      nals[nal_count++] = Nal{d + start, end - start};
    i = end;
  }
  if (nal_count == 0)
    return;

  const uint32_t rtp_ts = (uint32_t) ((frame.timestamp_us * 9) / 100);  // 90 kHz
  uint8_t pkt[RTP_MAX_PACKET];

  for (auto &client : this->clients_) {
    if (client.state != ClientState::PLAYING)
      continue;
    if (client.waiting_for_keyframe) {
      if (!frame.keyframe)
        continue;
      client.waiting_for_keyframe = false;
    }
    bool ok = true;
    for (size_t k = 0; k < nal_count && ok; k++) {
      const Nal &nal = nals[k];
      const bool last_nal = k + 1 == nal_count;
      auto header = [&](bool marker) {
        pkt[0] = 0x80;
        pkt[1] = (uint8_t) (RTP_PAYLOAD_TYPE | (marker ? 0x80 : 0));
        pkt[2] = (uint8_t) (client.seq >> 8);
        pkt[3] = (uint8_t) (client.seq & 0xFF);
        client.seq++;
        pkt[4] = (uint8_t) (rtp_ts >> 24);
        pkt[5] = (uint8_t) (rtp_ts >> 16);
        pkt[6] = (uint8_t) (rtp_ts >> 8);
        pkt[7] = (uint8_t) rtp_ts;
        pkt[8] = (uint8_t) (client.ssrc >> 24);
        pkt[9] = (uint8_t) (client.ssrc >> 16);
        pkt[10] = (uint8_t) (client.ssrc >> 8);
        pkt[11] = (uint8_t) client.ssrc;
      };
      if (nal.n <= RTP_MAX_PACKET - RTP_HEADER) {
        // Single NAL unit packet.
        header(last_nal);
        memcpy(pkt + RTP_HEADER, nal.p, nal.n);
        ok = this->send_packet_(client, pkt, RTP_HEADER + nal.n);
      } else {
        // Fragmentation units, type A (RFC 6184 section 5.8).
        const uint8_t indicator = (uint8_t) ((nal.p[0] & 0xE0) | 28);
        const uint8_t type = (uint8_t) (nal.p[0] & 0x1F);
        size_t off = 1;
        const size_t chunk = RTP_MAX_PACKET - RTP_HEADER - 2;
        while (off < nal.n && ok) {
          const size_t n = std::min(chunk, nal.n - off);
          const bool first = off == 1;
          const bool last = off + n == nal.n;
          header(last && last_nal);
          pkt[RTP_HEADER] = indicator;
          pkt[RTP_HEADER + 1] = (uint8_t) ((first ? 0x80 : 0) | (last ? 0x40 : 0) | type);
          memcpy(pkt + RTP_HEADER + 2, nal.p + off, n);
          ok = this->send_packet_(client, pkt, RTP_HEADER + 2 + n);
          off += n;
        }
      }
    }
    if (ok && client.transport == Transport::TCP)
      ok = this->flush_tcp_(client);
    if (!ok) {
      ESP_LOGW(TAG, "client could not keep up; dropping it");
      this->drop_client_(client);
      this->force_idr_ = true;
    }
  }
}

}  // namespace rtsp_h264
}  // namespace esphome

#endif  // USE_ESP32_VARIANT_ESP32P4
