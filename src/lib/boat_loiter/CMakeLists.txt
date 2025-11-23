############################################################################
#
#   Copyright (c) 2025 Highlands Keep. All rights reserved.
#
#   This software is proprietary and confidential.
#   Unauthorized copying or distribution is strictly prohibited.
#
############################################################################

px4_add_library(boat_loiter
	BoatLoiter.cpp
	BoatLoiter.hpp
)

target_link_libraries(boat_loiter PUBLIC px4_platform)
