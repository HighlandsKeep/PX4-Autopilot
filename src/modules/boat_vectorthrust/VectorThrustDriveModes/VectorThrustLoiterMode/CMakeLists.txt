############################################################################
#
#   Copyright (c) 2025 Highlands Keep. All rights reserved.
#
#   This software is proprietary and confidential.
#   Unauthorized copying or distribution is strictly prohibited.
#
############################################################################

px4_add_library(VectorThrustLoiterMode
	VectorThrustLoiterMode.cpp
)

target_link_libraries(VectorThrustLoiterMode
	PUBLIC
		boat_loiter
)
