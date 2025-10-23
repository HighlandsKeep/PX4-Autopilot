#pragma once

#include "sensor_bridge.hpp"
#include <dronecan/sensors/rc/RCInput.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/input_rc.h>
#include <uavcan/uavcan.hpp>

class UavcanRCInputBridge : public UavcanSensorBridgeBase {
public:
  static const char *const NAME;

  UavcanRCInputBridge(uavcan::INode &node);

  const char *get_name() const override { return NAME; }

  int init() override;

private:
  int init_driver(uavcan_bridge::Channel *channel) override;

  void rc_input_sub_cb(
      const uavcan::ReceivedDataStructure<dronecan::sensors::rc::RCInput> &msg);

  typedef uavcan::MethodBinder<UavcanRCInputBridge *,
                               void (UavcanRCInputBridge::*)(
                                   const uavcan::ReceivedDataStructure<
                                       dronecan::sensors::rc::RCInput> &)>
      RCInputCbBinder;

  uavcan::Subscriber<dronecan::sensors::rc::RCInput, RCInputCbBinder>
      _sub_rc_input;

  // Frame tracking
  uint16_t _total_frames{0};
  uint16_t _lost_frames{0};

  // Timeout tracking
  hrt_abstime _last_frame_time{0};
  static constexpr uint64_t VALID_FRAME_TIMEOUT_US = 500000; // 0.5s
};
