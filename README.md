# esphome-rtsp-h264

An [ESPHome](https://esphome.io) external component that serves a camera as an
RTSP stream of hardware-encoded H.264, on the ESP32-P4.

Home Assistant's generic camera, go2rtc, ffmpeg and VLC all play it:

```
rtsp://<device>:8554/stream
```

It takes raw frames from a [`p4_csi_camera`](https://github.com/eman/esphome-p4-camera),
runs them through the ESP32-P4's H.264 encoder (Espressif's `esp_h264`), and
packetises the result as RTP over TCP (interleaved on the RTSP connection) or
UDP unicast, per RFC 6184. Nothing runs on ESPHome's main loop: the encoder
runs on the camera's capture task and the server on its own.

Measured on a Guition JC1060P470 (ESP32-P4, ESP-Hosted Wi-Fi): 1920×1080 at a
steady 11 fps, about 1 Mbit/s at the default settings, with an LVGL display
running on the same chip. The encoder sets that ceiling, not the network: the
same stream over a 100 Mbit wire delivers the same frame rate and uses a
quarter of what the Wi-Fi link carries on a good evening.

## Configuration

```yaml
external_components:
  - source: github://eman/esphome-p4-camera
  - source: github://eman/esphome-rtsp-h264

p4_csi_camera:
  id: cam
  name: Camera
  i2c_id: bus_a

rtsp_h264:
  camera: cam
  port: 8554             # default
  bitrate: 2000000       # bits per second, default 2 Mbit/s
  framerate: 15 fps      # default; the sensor runs at 30 and the rest are skipped
  gop: 30                # frames between keyframes, default two seconds' worth
  qp_min: 18             # quantiser range the rate control may use
  qp_max: 45
  max_clients: 2
```

The stream path is fixed at `/stream`. Both RTP transports are supported; TCP
is what Home Assistant and go2rtc use by default, and is the one to prefer
over Wi-Fi.

### Home Assistant

Add a **Generic Camera** integration with the stream source
`rtsp://<device>.local:8554/stream`, or hand the URL to go2rtc. The
`p4_csi_camera` entity remains the still-image camera over the native API;
this is the video path.

## How it works, and what it needs

- **Frames** arrive as YUV 4:2:0 from the ISP, which is the only input the
  hardware encoder accepts. The camera component captures in that format and
  converts for its JPEG stills itself.
- **The encoder runs on the capture task**, so a frame that is being encoded
  holds a sensor buffer. The camera runs four of them for that reason; with
  two, exposure and encoding serialise and the rate collapses to one over
  their sum. In dim light the camera may also slow the sensor itself for a
  longer exposure.
- **Internal RAM, and why you may not want to win it.** The encoder wants a
  135 KB contiguous internal-RAM reference frame at 1080p. On a board that
  also runs a display, Wi-Fi and ESPHome's API that is the scarce resource, so
  the component falls back to PSRAM rather than refuse to start. The log line
  `encoder took N KB of internal RAM` says which case you are in, and the
  difference is real: on one scene, one minute apart, 84 ms per 1080p frame
  with the reference in PSRAM against 40 ms with it internal.

  That is not the whole story. The RAM has to come from somewhere, and on this
  panel it came from LVGL: ESPHome puts a draw buffer of a quarter frame or
  smaller in internal RAM, and raising `buffer_size` to 50 % moves it out.
  Doing that halved the encode time and *lowered* the delivered frame rate,
  11.3 a second to 8, because LVGL then drew through the same PSRAM the sensor
  and the encoder were already saturating. Measure the frame rate, not the
  encode time.
- **New viewers** get a keyframe within a frame or two: `PLAY` forces an IDR.
  `DESCRIBE` starts the camera and waits up to three seconds for the first
  keyframe so the SDP carries the parameter sets; clients that arrive before
  that get an SDP without them, which every decoder handles in-band.
- **Slow clients** are dropped rather than allowed to stall the others: a
  socket write that blocks for two seconds ends that client.

## Requirements

ESP32-P4, ESP-IDF 5.5, ESPHome 2026.8 or later, and a `p4_csi_camera`. The
`espressif/esp_h264` component (1.3.x) is added to the build automatically.

## Licence

MIT.
