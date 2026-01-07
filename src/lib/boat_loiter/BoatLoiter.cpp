/****************************************************************************
 *
 *   Copyright (c) 2025 Highlands Keep. All rights reserved.
 *
 *   This software is proprietary and confidential.
 *   Unauthorized copying or distribution is strictly prohibited.
 *
 ****************************************************************************/

#include "BoatLoiter.hpp"

using namespace matrix;
using namespace time_literals;

BoatLoiter::BoatLoiter(ModuleParams *parent) : ModuleParams(parent) {
  updateParams();
  _rover_position_setpoint_pub.advertise();
}

void BoatLoiter::updateParams() { ModuleParams::updateParams(); }

void BoatLoiter::loiterControl() {
  vehicle_local_position_s vehicle_local_position{};

  if (_vehicle_local_position_sub.copy(&vehicle_local_position)) {
    // Always check for position setpoint updates
    position_setpoint_triplet_s position_setpoint_triplet{};
    bool setpoint_valid = false;

    if (_position_setpoint_triplet_sub.copy(&position_setpoint_triplet)) {
      // If we have a valid current setpoint, use it as the loiter center
      if (position_setpoint_triplet.current.valid &&
          PX4_ISFINITE(position_setpoint_triplet.current.lat) &&
          PX4_ISFINITE(position_setpoint_triplet.current.lon)) {

        // Convert setpoint to local NED coordinates
        MapProjection global_ned_proj_ref{};
        if (!global_ned_proj_ref.isInitialized() ||
            (global_ned_proj_ref.getProjectionReferenceTimestamp() !=
             vehicle_local_position.ref_timestamp)) {
          global_ned_proj_ref.initReference(vehicle_local_position.ref_lat,
                                           vehicle_local_position.ref_lon,
                                           vehicle_local_position.ref_timestamp);
        }

        Vector2f target_ned;
        global_ned_proj_ref.project(position_setpoint_triplet.current.lat,
                                   position_setpoint_triplet.current.lon,
                                   target_ned(0), target_ned(1));

        // Check if loiter center changed significantly (new target)
        const bool center_changed = !PX4_ISFINITE(_loiter_center_ned(0)) ||
                                    (target_ned - _loiter_center_ned).norm() > 1.0f;

        // Update loiter center to the setpoint position
        _loiter_center_ned = target_ned;
        setpoint_valid = true;
        _loiter_initialized = true;

        // Print entry message when entering loiter with new target
        if (center_changed) {
          PX4_INFO("Boat entering LOITER mode");
          _loiter_entry_printed = true;
        }
      }
    }

    // Initialize loiter center on first call if no valid setpoint
    if (!setpoint_valid && (!_loiter_initialized || !PX4_ISFINITE(_loiter_center_ned(0)))) {
      PX4_INFO("Boat entering LOITER mode");
      Vector2f current_pos(vehicle_local_position.x, vehicle_local_position.y);
      Vector2f current_vel(vehicle_local_position.vx,
                           vehicle_local_position.vy);
      _loiter_center_ned = calculateStoppingLocation(current_pos, current_vel);
      _loiter_initialized = true;
      _loiter_entry_printed = true;
    }

    const float loiter_radius = math::max(_param_loit_radius.get(), 0.1f);
    const float distance_to_center = distanceFromLoiterCenter();

    rover_position_setpoint_s rover_position_setpoint{};
    rover_position_setpoint.timestamp = hrt_absolute_time();

    // Within loiter radius - just drift
    if (distance_to_center <= loiter_radius) {
      // Set position to current location (drift)
      rover_position_setpoint.position_ned[0] = vehicle_local_position.x;
      rover_position_setpoint.position_ned[1] = vehicle_local_position.y;
      rover_position_setpoint.cruising_speed = 0.0f;
      rover_position_setpoint.arrival_speed = 0.0f;
      rover_position_setpoint.yaw = NAN; // No specific yaw required

    } else {
      // Outside loiter radius - drive back towards center
      const float distance_beyond_radius = distance_to_center - loiter_radius;
      const float bearing_to_center = bearingToLoiterCenter();
      const float current_yaw = vehicle_local_position.heading;

      // Calculate target speed
      // speed = 0.5 m/s * distance_beyond_radius * speed_gain, clamped to max
      // speed
      float base_speed =
          0.5f * distance_beyond_radius * _param_loit_speed_gain.get();
      float desired_speed = math::min(base_speed, _param_ro_speed_limit.get());

      // Determine heading based on LOIT_TYPE
      float target_yaw = bearing_to_center;
      bool drive_forward = true;

      switch (_param_loit_type.get()) {
      case 0: // Forward or reverse (whichever requires less rotation)
        drive_forward = shouldDriveForward(bearing_to_center, current_yaw);

        if (!drive_forward) {
          target_yaw = wrap_pi(bearing_to_center + M_PI_F);
        }

        break;

      case 1: // Always face bow towards target
        drive_forward = true;
        target_yaw = bearing_to_center;
        break;

      case 2: // Always face stern towards target (drive backward)
        drive_forward = false;
        target_yaw = wrap_pi(bearing_to_center + M_PI_F);
        break;

      default:
        drive_forward = shouldDriveForward(bearing_to_center, current_yaw);

        if (!drive_forward) {
          target_yaw = wrap_pi(bearing_to_center + M_PI_F);
        }

        break;
      }

      // Reduce speed based on heading error (like ArduPilot)
      // 45deg error → 75% speed, 90deg error → 50% speed
      float yaw_error = wrap_pi(target_yaw - current_yaw);
      float yaw_error_ratio =
          1.0f -
          math::constrain(fabsf(yaw_error) / (M_PI_F / 2.0f), 0.0f, 1.0f) *
              0.5f;
      desired_speed *= yaw_error_ratio;

      // Set position setpoint to loiter center
      rover_position_setpoint.position_ned[0] = _loiter_center_ned(0);
      rover_position_setpoint.position_ned[1] = _loiter_center_ned(1);
      rover_position_setpoint.cruising_speed =
          drive_forward ? desired_speed : -desired_speed;
      rover_position_setpoint.arrival_speed = 0.0f;
      rover_position_setpoint.yaw = target_yaw;
    }

    rover_position_setpoint.start_ned[0] = vehicle_local_position.x;
    rover_position_setpoint.start_ned[1] = vehicle_local_position.y;

    _rover_position_setpoint_pub.publish(rover_position_setpoint);
  }
}

float BoatLoiter::distanceFromLoiterCenter() {
  vehicle_local_position_s vehicle_local_position{};

  if (_vehicle_local_position_sub.copy(&vehicle_local_position) &&
      PX4_ISFINITE(_loiter_center_ned(0))) {
    const Vector2f current_pos(vehicle_local_position.x,
                               vehicle_local_position.y);
    return (current_pos - _loiter_center_ned).norm();
  }

  return 0.0f;
}

float BoatLoiter::bearingToLoiterCenter() {
  vehicle_local_position_s vehicle_local_position{};

  if (_vehicle_local_position_sub.copy(&vehicle_local_position) &&
      PX4_ISFINITE(_loiter_center_ned(0))) {
    const Vector2f current_pos(vehicle_local_position.x,
                               vehicle_local_position.y);
    const Vector2f vec_to_center = _loiter_center_ned - current_pos;
    return atan2f(vec_to_center(1), vec_to_center(0));
  }

  return 0.0f;
}

bool BoatLoiter::shouldDriveForward(float bearing_to_target,
                                    float current_yaw) const {
  // Calculate angle difference between current yaw and bearing to target
  float yaw_error_forward = wrap_pi(bearing_to_target - current_yaw);
  float yaw_error_reverse = wrap_pi((bearing_to_target + M_PI_F) - current_yaw);

  // Choose forward if it requires less rotation
  return fabsf(yaw_error_forward) < fabsf(yaw_error_reverse);
}

Vector2f BoatLoiter::calculateStoppingLocation(const Vector2f &current_pos,
                                               const Vector2f &current_vel) {
  // Calculate stopping distance based on current velocity
  // Assumes exponential deceleration with time constant
  const float velocity_magnitude = current_vel.norm();

  // If velocity is very low, just use current position
  if (velocity_magnitude < 0.1f) {
    return current_pos;
  }

  // Estimate stopping distance using simplified physics
  // Assuming drag-based deceleration: v(t) = v0 * e^(-t/tau)
  // Stopping distance ≈ v0 * tau (where tau is time constant)
  // For boats, tau is typically 2-5 seconds depending on hull design
  const float time_constant = 3.0f; // seconds - tunable parameter
  const float stopping_distance = velocity_magnitude * time_constant;

  // Calculate unit velocity vector
  Vector2f velocity_direction = current_vel / velocity_magnitude;

  // Project stopping location along velocity vector
  Vector2f stopping_location =
      current_pos + velocity_direction * stopping_distance;

  return stopping_location;
}
