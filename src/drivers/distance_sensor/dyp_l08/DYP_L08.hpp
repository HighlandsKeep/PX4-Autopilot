/****************************************************************************
 *
 *   Copyright (c) 2025 Highlands Keep. All rights reserved.
 *
 *   This software is proprietary and confidential.
 *   Unauthorized copying or distribution is strictly prohibited.
 *
 ****************************************************************************/

/**
 * @file DYP_L08.hpp
 * @author Andrew Gregg
 *
 * Driver for the DYP-L08 ultrasonic underwater rangefinder.
 */

#pragma once

#include <termios.h>

#include <drivers/drv_hrt.h>
#include <drivers/rangefinder/PX4Rangefinder.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

using namespace time_literals;

/* Configuration Constants */
static constexpr uint32_t DYP_L08_MEASURE_INTERVAL{25_ms};	// 25ms minimum trigger interval per ICD

/* Frame format */
static constexpr unsigned char START_FRAME{0xFF};

/**
 * Frame format definition per ICD:
 *   1B    1B         1B            1B
 * | 0xFF | Data_H | Data_L | Checksum |
 *
 * Checksum = (0xFF + Data_H + Data_L) & 0xFF (low byte of sum)
 * Distance = (Data_H * 256 + Data_L) in millimeters
 * Invalid reading = 0xFFFF (no valid echo detected)
 */
static constexpr uint8_t FRAME_LENGTH{4};
static constexpr uint8_t DATA_H_POS{1};
static constexpr uint8_t DATA_L_POS{2};
static constexpr uint8_t CHECKSUM_POS{3};

static constexpr float MAX_DISTANCE_M{10.0f};	// 10 meters maximum distance in mm

class DYP_L08 : public px4::ScheduledWorkItem
{
public:
	/**
	 * Default Constructor
	 * @param port The serial port to open for communicating with the sensor.
	 * @param rotation The sensor rotation relative to the vehicle body.
	 */
	DYP_L08(const char *port, uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);

	/** Virtual destructor */
	virtual ~DYP_L08() override;

	/**
	 * Method : init()
	 * This method initializes the general driver for a range finder sensor.
	 */
	int init();

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_info();

private:

	enum class PARSE_STATE {
		WAITING_FRAME = 0,
		DATA_H,
		DATA_L,
		CHECKSUM
	};

	/**
	 * Reads data from serial UART and places it into a buffer.
	 */
	int collect();

	/**
	 * Parse incoming byte and extract distance measurement
	 * @param check_byte The byte to parse
	 * @param data_h High byte of distance data
	 * @param data_l Low byte of distance data
	 * @param state Current parser state
	 * @param distance Parsed distance in 0.1mm units
	 * @return PX4_OK if frame is complete and valid, PX4_ERROR otherwise
	 */
	int data_parser(const uint8_t check_byte, uint8_t &data_h, uint8_t &data_l,
			PARSE_STATE &state, int &distance);

	/**
	 * Opens and configures the UART serial communications port.
	 * @param speed The baudrate (speed) to configure the serial UART port.
	 */
	int open_serial_port(const speed_t speed = B115200);

	/**
	 * Perform a reading cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void Run() override;

	/**
	 * Initialise the automatic measurement state machine and start it.
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void start();

	/**
	 * Stops the automatic measurement state machine.
	 */
	void stop();


	PX4Rangefinder	_px4_rangefinder;

	char _port[20] {};

	int _file_descriptor{-1};

	static constexpr size_t LINEBUF_SIZE = 32;  // Buffer size for serial data
	uint8_t _linebuf[LINEBUF_SIZE] {};

	uint8_t _data_h{0};
	uint8_t _data_l{0};

	PARSE_STATE _parse_state{PARSE_STATE::WAITING_FRAME};

	unsigned _consecutive_fail_count{0};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
};
