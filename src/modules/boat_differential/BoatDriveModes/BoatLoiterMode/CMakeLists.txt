############################################################################
#
#   Copyright (c) 2025 Highlands Keep. All rights reserved.
#
#   This software is proprietary and confidential.
#   Unauthorized copying or distribution is strictly prohibited.
#
############################################################################

px4_add_library(BoatLoiterMode
	BoatLoiterMode.cpp
)

target_link_libraries(BoatLoiterMode
	PUBLIC
		boat_loiter
)
