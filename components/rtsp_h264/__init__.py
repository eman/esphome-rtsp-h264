"""RTSP server streaming hardware-encoded H.264 from an ESP32-P4 camera.

Takes raw frames from a `p4_csi_camera`, encodes them with the ESP32-P4's
hardware H.264 encoder (Espressif's `esp_h264`), and serves them over RTSP
with RTP over TCP (interleaved) or UDP unicast. Home Assistant's generic
camera, go2rtc, ffmpeg and VLC all consume it.

    rtsp_h264:
      camera: cam
      port: 8554
      bitrate: 2000000
      framerate: 15 fps

The stream is at rtsp://<device>:<port>/stream.
"""

import esphome.codegen as cg
from esphome.components import esp32
from esphome.components.p4_csi_camera import P4CsiCamera
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_PORT

CODEOWNERS = ["@eman"]
DEPENDENCIES = ["network", "p4_csi_camera"]

CONF_CAMERA = "camera"
CONF_BITRATE = "bitrate"
CONF_FRAMERATE = "framerate"
CONF_GOP = "gop"
CONF_QP_MIN = "qp_min"
CONF_QP_MAX = "qp_max"
CONF_MAX_CLIENTS = "max_clients"

rtsp_h264_ns = cg.esphome_ns.namespace("rtsp_h264")
RtspH264 = rtsp_h264_ns.class_("RtspH264", cg.Component)


def _p4_only(config):
    variant = esp32.get_esp32_variant()
    if variant != esp32.const.VARIANT_ESP32P4:
        raise cv.Invalid(f"rtsp_h264 needs the ESP32-P4's hardware H.264 encoder; this is an {variant}")
    return config


def _qp_order(config):
    if config[CONF_QP_MIN] > config[CONF_QP_MAX]:
        raise cv.Invalid("qp_min must not exceed qp_max")
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(RtspH264),
            cv.Required(CONF_CAMERA): cv.use_id(P4CsiCamera),
            cv.Optional(CONF_PORT, default=8554): cv.port,
            # Target bit rate in bits per second. 1080p at 15 fps looks good
            # around 2 Mbit/s; the ESP-Hosted Wi-Fi link comfortably carries 4.
            cv.Optional(CONF_BITRATE, default=2_000_000): cv.int_range(min=100_000, max=25_000_000),
            # Frames encoded per second; the sensor runs at 30 and the rest are
            # skipped. Fewer frames means more bits for each one.
            cv.Optional(CONF_FRAMERATE, default="15 fps"): cv.framerate,
            # Frames between keyframes. Defaults to two seconds' worth: a new
            # viewer waits at most that long for a picture.
            cv.Optional(CONF_GOP): cv.int_range(min=1, max=255),
            # Quantiser range the rate control may use; a wide range lets it
            # actually hold the bit rate.
            cv.Optional(CONF_QP_MIN, default=18): cv.int_range(min=0, max=51),
            cv.Optional(CONF_QP_MAX, default=45): cv.int_range(min=0, max=51),
            cv.Optional(CONF_MAX_CLIENTS, default=2): cv.int_range(min=1, max=4),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _p4_only,
    _qp_order,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_camera(await cg.get_variable(config[CONF_CAMERA])))
    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_bitrate(config[CONF_BITRATE]))
    fps = config[CONF_FRAMERATE]
    cg.add(var.set_framerate(fps))
    gop = config.get(CONF_GOP, max(1, min(255, int(round(fps * 2)))))
    cg.add(var.set_gop(gop))
    cg.add(var.set_qp(config[CONF_QP_MIN], config[CONF_QP_MAX]))
    cg.add(var.set_max_clients(config[CONF_MAX_CLIENTS]))

    # The hardware encoder. esp_video already depends on this at the same
    # version, so nothing new is downloaded when the camera is present.
    esp32.add_idf_component(name="espressif/esp_h264", ref="1.3.8")
    # Let the encoder's internal-RAM allocations fall back to PSRAM; see the
    # wrapper at the top of rtsp_h264.cpp.
    cg.add_build_flag("-Wl,--wrap=esp_h264_aligned_malloc")
