

#include "rc_input.hpp"
#include <drivers/drv_hrt.h>
#include <math.h>
#include <mathlib/mathlib.h>
#include <parameters/param.h>
#include <stdio.h>
#include <uORB/topics/input_rc.h>

const char *const UavcanRCInputBridge::NAME = "rc_input";

UavcanRCInputBridge::UavcanRCInputBridge(uavcan::INode &node)
    : UavcanSensorBridgeBase("uavcan_rc_input", ORB_ID(input_rc)),
      _sub_rc_input(node) {}

int UavcanRCInputBridge::init() {

  int res = _sub_rc_input.start(
      RCInputCbBinder(this, &UavcanRCInputBridge::rc_input_sub_cb));

  if (res < 0) {
    printf("failed to start uavcan rc_input sub: %d\n", res);
    return res;
  }

  return 0;
}

void UavcanRCInputBridge::rc_input_sub_cb(
    const uavcan::ReceivedDataStructure<dronecan::sensors::rc::RCInput> &msg) {
  input_rc_s rc{};
  rc.timestamp = hrt_absolute_time();

  // Frame counters
  _total_frames++;
  rc.rc_total_frame_count = _total_frames;

  // Failsafe detection
  bool failsafe =
      (msg.status & dronecan::sensors::rc::RCInput::STATUS_FAILSAFE);
  bool valid_quality =
      (msg.status & dronecan::sensors::rc::RCInput::STATUS_QUALITY_VALID);

  // Link quality
  rc.link_quality = valid_quality ? (int8_t)((msg.quality * 100) / 255) : -1;

  // Channel values
  uint8_t channel_count = msg.rcin.size();
  if (channel_count > input_rc_s::RC_INPUT_MAX_CHANNELS) {
    channel_count = input_rc_s::RC_INPUT_MAX_CHANNELS;
  }
  rc.channel_count = channel_count;
  for (uint8_t i = 0; i < channel_count; ++i) {
    uint16_t pwm = 1000 + ((msg.rcin[i] * 1000) / 4095);
    rc.values[i] = math::constrain<uint16_t>(pwm, 1000, 2000);
  }

  // Failsafe and lost
  rc.rc_failsafe = failsafe;
  hrt_abstime now = hrt_absolute_time();
  bool rc_lost =
      ((now - _last_frame_time) > VALID_FRAME_TIMEOUT_US) || failsafe;
  rc.rc_lost = rc_lost;
  if (rc_lost) {
    _lost_frames++;
  }
  rc.rc_lost_frame_count = _lost_frames;

  rc.timestamp_last_signal = _last_frame_time;
  rc.input_source = input_rc_s::RC_INPUT_SOURCE_UAVCAN;
  rc.rc_ppm_frame_length = 0;

  // RSSI
  rc.rssi = rc.link_quality;
  rc.rssi_dbm =
      rc.link_quality > 0 ? (-100.0f + (rc.link_quality * 0.5f)) : NAN;

  // publish using bridge
  publish(msg.getSrcNodeID().get(), &rc);
}

int UavcanRCInputBridge::init_driver(uavcan_bridge::Channel *channel) {
  // Not used for RC input
  return PX4_OK;
}
