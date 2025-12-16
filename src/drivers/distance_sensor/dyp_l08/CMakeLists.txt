############################################################################
#
#   Copyright (c) 2025 Highlands Keep. All rights reserved.
#
#   This software is proprietary and confidential.
#   Unauthorized copying or distribution is strictly prohibited.
#
############################################################################
px4_add_module(
	MODULE drivers__dyp_l08
	MAIN dyp_l08
	SRCS
		DYP_L08.cpp
		DYP_L08.hpp
		dyp_l08_main.cpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		drivers_rangefinder
	)
