/****************************************************************************
 *
 *   Copyright (c) 2025 Highlands Keep. All rights reserved.
 *
 *   This software is proprietary and confidential.
 *   Unauthorized copying or distribution is strictly prohibited.
 *
 ****************************************************************************/

#pragma once

#include <lib/boat_loiter/BoatLoiter.hpp>

/**
 * @brief Boat loiter mode wrapper for vector thrust boats
 */
class VectorThrustLoiterMode : public BoatLoiter {
public:
  /**
   * @brief Construct a new Boat Loiter Mode object
   * @param parent Parent module for parameter management
   */
  VectorThrustLoiterMode(ModuleParams *parent) : BoatLoiter(parent) {}

  ~VectorThrustLoiterMode() = default;
};
