/****************************************************************************
 *
 *   Copyright (c) 2025 Highlands Keep. All rights reserved.
 *
 *   This software is proprietary and confidential.
 *   Unauthorized copying or distribution is strictly prohibited.
 *
 ****************************************************************************/

#pragma once

#include <lib/pure_pursuit/PurePursuit.hpp>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/rover_position_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>

/**
 * @brief Boat loiter mode for holding position in current
 *
 * This mode allows boats to maintain position while drifting:
 * - Vehicle drifts within LOIT_RADIUS
 * - When beyond radius, drives back towards target
 * - Speed scales with distance from loiter edge
 * - Can drive forward or reverse depending on LOIT_TYPE
 */
class BoatLoiter : public ModuleParams {
public:
  /**
   * @brief Construct a new Boat Loiter object
   * @param parent Parent module for parameter management
   */
  BoatLoiter(ModuleParams *parent);

  ~BoatLoiter() = default;

  /**
   * @brief Update loiter control
   * Calculates desired position, yaw, and speed to maintain loiter
   */
  void loiterControl();

  /**
   * @brief Update parameters
   */
  void updateParams() override;

protected:
  /**
   * @brief Calculate distance from the loiter center
   * @return Distance in meters
   */
  float distanceFromLoiterCenter();

  /**
   * @brief Calculate bearing to the loiter center
   * @return Bearing in radians
   */
  float bearingToLoiterCenter();

  /**
   * @brief Determine if vehicle should drive forward or reverse
   * @param bearing_to_target Bearing to loiter center (rad)
   * @param current_yaw Current vehicle yaw (rad)
   * @return True if forward, false if reverse
   */
  bool shouldDriveForward(float bearing_to_target, float current_yaw) const;

  /**
   * @brief Calculate optimal stopping location based on current velocity
   * @param current_pos Current position (NED)
   * @param current_vel Current velocity (NED)
   * @return Predicted stopping position (NED)
   */
  matrix::Vector2f
  calculateStoppingLocation(const matrix::Vector2f &current_pos,
                            const matrix::Vector2f &current_vel);

  uORB::Subscription _vehicle_local_position_sub{
      ORB_ID(vehicle_local_position)};
  uORB::Publication<rover_position_setpoint_s> _rover_position_setpoint_pub{
      ORB_ID(rover_position_setpoint)};

  matrix::Vector2f _loiter_center_ned{NAN, NAN}; ///< Loiter center in NED frame
  bool _loiter_initialized{false}; ///< True when loiter center is set

  DEFINE_PARAMETERS(
      (ParamFloat<px4::params::LOIT_RADIUS>)_param_loit_radius,
      (ParamFloat<px4::params::LOIT_SPEED_GAIN>)_param_loit_speed_gain,
      (ParamInt<px4::params::LOIT_TYPE>)_param_loit_type,
      (ParamFloat<px4::params::RO_SPEED_LIM>)_param_ro_speed_limit)
};
