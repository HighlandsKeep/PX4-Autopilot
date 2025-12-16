/****************************************************************************
 *
 *   Copyright (c) 2025 Highlands Keep. All rights reserved.
 *
 *   This software is proprietary and confidential.
 *   Unauthorized copying or distribution is strictly prohibited.
 *
 ****************************************************************************/

/**
 * @file DYP_L08.cpp
 * @author Andrew Gregg
 *
 * Driver for the DYP-L08 ultrasonic underwater rangefinder.
 *
 * The sensor outputs UART data at 115200 baud with the following format:
 * - Frame header: 0xFF
 * - Data high byte
 * - Data low byte
 * - Checksum: low byte of (0xFF + Data_H + Data_L)
 * - Distance in millimeters: Data_H * 256 + Data_L
 */

#include "DYP_L08.hpp"

#include <fcntl.h>

#include <lib/drivers/device/Device.hpp>

DYP_L08::DYP_L08(const char *port, uint8_t rotation) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_px4_rangefinder(0, rotation)
{
	// Store the port name.
	strncpy(_port, port, sizeof(_port) - 1);

	// Enforce null termination.
	_port[sizeof(_port) - 1] = '\0';

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

	_px4_rangefinder.set_device_id(device_id.devid);

	// DYP-L08 specifications for underwater operation
	// Per ICD: minimum range is typically 0.05m (5cm), maximum 10m
	// Note: Actual specifications may vary - adjust based on sensor datasheet
	_px4_rangefinder.set_min_distance(0.05f);	// 5cm minimum range
	_px4_rangefinder.set_max_distance(MAX_DISTANCE_M);	// 10m maximum range
	_px4_rangefinder.set_fov(0.26f);		// ~15 degrees cone angle (typical for ultrasonic)
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_DYP_L08);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND);
}

DYP_L08::~DYP_L08()
{
	// Ensure we are truly inactive and serial port is closed
	stop();

	// Free performance counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
DYP_L08::collect()
{
	perf_begin(_sample_perf);

	int distance_raw = -1;

	bool frame_valid = false;

	// Send trigger pulse - sensor requires low pulse on RX line to trigger measurement
	// Per ICD: "When the trigger input lead 'RX' receives a low pulse signal,
	// the module will be triggered to work" (T1 >= 25ms minimum interval)
	// We trigger by sending a null byte
	uint8_t trigger = 0x00;

	if (::write(_file_descriptor, &trigger, 1) != 1) {
		PX4_DEBUG("Failed to send trigger pulse");
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	// Small delay to allow sensor to respond (per ICD: T1 >= 25ms, T2 >= 20ms)
	px4_usleep(1000);  // 1ms delay before reading

	// Read from the sensor UART buffer.
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	int bytes_read = ::read(_file_descriptor, &_linebuf[0], sizeof(_linebuf));

	if (bytes_read > 0) {
		// Process each byte
		for (int i = 0; i < bytes_read; i++) {
			if (data_parser(_linebuf[i], _data_h, _data_l, _parse_state, distance_raw) == PX4_OK) {
				frame_valid = true;
				break;  // Frame complete
			}
		}

	} else if (bytes_read == 0) {
		// No data available yet
		perf_end(_sample_perf);
		return -EAGAIN;

	} else if (bytes_read < 0) {
		if (errno == EAGAIN || errno == EINTR) {
			// Try again later
			perf_end(_sample_perf);
			return -EAGAIN;
		}

		// Actual read error
		PX4_ERR("read error: %d, errno: %d", bytes_read, errno);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	if (!frame_valid) {
		_consecutive_fail_count++;

		// Only report errors after sensor initialization period (2 seconds)
		if (hrt_absolute_time() > 2_s && _consecutive_fail_count >= 10) {
			PX4_DEBUG("No valid frame, consecutive failures: %u", _consecutive_fail_count);
		}

		perf_end(_sample_perf);
		return -EAGAIN;
	}

	// Check for invalid reading (sensor returns 0xFFFF = 65535 when no valid target detected)
	int8_t signal_quality;
	float current_distance;

	if (distance_raw == 0xFFFF || distance_raw > uint16_t(MAX_DISTANCE_M * 1000)) {
		// No valid target detected - mark as invalid
		// Per uORB message spec: signal_quality=0 means invalid signal
		signal_quality = 0;
		current_distance = -1.0f;  // Use -1.0 to indicate out of range

	} else {
		// Valid reading - 100% signal quality
		// Per ICD: distance value is in millimeters
		signal_quality = 100;
		current_distance = static_cast<float>(distance_raw) / 1000.0f;
	}

	// Publish distance measurement with signal quality
	_px4_rangefinder.update(timestamp_sample, current_distance, signal_quality);

	// Reset consecutive failure counter on successful read
	_consecutive_fail_count = 0;

	perf_end(_sample_perf);

	return PX4_OK;
}

int
DYP_L08::data_parser(const uint8_t check_byte, uint8_t &data_h, uint8_t &data_l,
		     PARSE_STATE &state, int &distance)
{
	switch (state) {
	case PARSE_STATE::WAITING_FRAME:
		if (check_byte == START_FRAME) {
			state = PARSE_STATE::DATA_H;
		}

		break;

	case PARSE_STATE::DATA_H:
		data_h = check_byte;
		state = PARSE_STATE::DATA_L;
		break;

	case PARSE_STATE::DATA_L:
		data_l = check_byte;
		state = PARSE_STATE::CHECKSUM;
		break;

	case PARSE_STATE::CHECKSUM: {
			// Calculate expected checksum: low byte of (START_FRAME + data_h + data_l)
			uint8_t expected_checksum = (START_FRAME + data_h + data_l) & 0xFF;

			// Reset state for next frame
			state = PARSE_STATE::WAITING_FRAME;

			if (check_byte == expected_checksum) {
				// Valid frame - calculate distance
				distance = (data_h << 8) | data_l;
				return PX4_OK;
			} else {
				// Checksum failed
				perf_count(_comms_errors);
			}

			break;
		}
	}

	return PX4_ERROR;
}

int
DYP_L08::init()
{
	start();

	return PX4_OK;
}

int
DYP_L08::open_serial_port(const speed_t speed)
{
	// File descriptor initialized?
	if (_file_descriptor > 0) {
		return PX4_OK;
	}

	// Configure port flags for read/write, non-controlling, non-blocking.
	int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

	// Open the serial port.
	_file_descriptor = ::open(_port, flags);

	if (_file_descriptor < 0) {
		PX4_ERR("open failed (%i)", errno);
		return PX4_ERROR;
	}

	termios uart_config = {};

	// Store the current port configuration attributes.
	tcgetattr(_file_descriptor, &uart_config);

	// Clear ONLCR flag (which appends a CR for every LF).
	uart_config.c_oflag &= ~ONLCR;

	// No parity, one stop bit (8N1 configuration per ICD).
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	// Set the input baud rate in the uart_config struct.
	int termios_state = cfsetispeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d ISPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Set the output baud rate in the uart_config struct.
	termios_state = cfsetospeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d OSPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Apply the modified port attributes.
	termios_state = tcsetattr(_file_descriptor, TCSANOW, &uart_config);

	if (termios_state < 0) {
		PX4_ERR("baud %d ATTR", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	PX4_INFO("successfully opened UART port %s", _port);
	return PX4_OK;
}

void
DYP_L08::print_info()
{
	PX4_INFO("DYP-L08 Ultrasonic Underwater Rangefinder");
	PX4_INFO("  Port: %s", _port);
	PX4_INFO("  Range: %.2f - %.1f m", 0.05, (double)MAX_DISTANCE_M);
	PX4_INFO("  Consecutive failures: %u", _consecutive_fail_count);

	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

void
DYP_L08::Run()
{
	// Ensure the serial port is open
	if (open_serial_port() != PX4_OK) {
		PX4_ERR("Failed to open serial port");
		return;
	}

	// Perform data collection
	collect();
}

void
DYP_L08::start()
{
	// Schedule the driver at regular intervals.
	ScheduleOnInterval(DYP_L08_MEASURE_INTERVAL);

	PX4_INFO("driver started");
}

void
DYP_L08::stop()
{
	// Clear the work queue schedule.
	ScheduleClear();

	// Ensure the serial port is closed.
	::close(_file_descriptor);
}
