/****************************************************************************
 *
 *   Copyright (c) 2025 Highlands Keep. All rights reserved.
 *
 *   This software is proprietary and confidential.
 *   Unauthorized copying or distribution is strictly prohibited.
 *
 ****************************************************************************/

/**
 * @file boat_loiter_params.c
 *
 * Parameters for boat loiter mode.
 *
 * @author Andrew Gregg, Highlands Keep
 */

/**
 * Loiter radius for boats
 *
 * Defines the radius around the loiter center point within which the boat
 * is allowed to drift. When the boat moves outside this radius, it will
 * actively drive back towards the center.
 *
 * @unit m
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @increment 0.1
 * @group Boat Loiter
 */
PARAM_DEFINE_FLOAT(LOIT_RADIUS, 2.0f);

/**
 * Loiter speed gain for boats
 *
 * Controls how aggressively the boat drives back when outside the loiter radius.
 * Higher values result in faster return speeds. The actual speed is calculated as:
 * Speed = 0.5 m/s * distance_beyond_radius * LOIT_SPEED_GAIN (clamped to max speed).
 *
 * @min 0.1
 * @max 5.0
 * @decimal 1
 * @increment 0.1
 * @group Boat Loiter
 */
PARAM_DEFINE_FLOAT(LOIT_SPEED_GAIN, 0.5f);

/**
 * Loiter type for boats
 *
 * Determines the heading behavior when driving back to the loiter center:
 * 0: Forward or reverse (whichever requires less rotation)
 * 1: Always drive forward (bow towards target)
 * 2: Always drive reverse (stern towards target)
 *
 * @value 0 Forward or Reverse
 * @value 1 Always Forward
 * @value 2 Always Reverse
 * @group Boat Loiter
 */
PARAM_DEFINE_INT32(LOIT_TYPE, 0);
