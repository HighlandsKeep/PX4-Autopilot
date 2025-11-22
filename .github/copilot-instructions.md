# PX4 Autopilot Copilot Instructions

## Project Overview

PX4 is a professional-grade open source drone autopilot. This is a real-time embedded system running on NuttX RTOS (for hardware) or Linux/macOS (for simulation), with the main flight control code in `src/modules/`.

## Core Architecture

### uORB Messaging System
All inter-module communication uses **uORB** (Micro Object Request Broker), an asynchronous pub/sub messaging API:
- Message definitions are in `msg/*.msg` files (CamelCase, e.g., `VehicleAttitude.msg`)
- Generated headers use snake_case (e.g., `#include <uORB/topics/vehicle_attitude.h>`)
- Reference topics using `ORB_ID(topic_name)` macro
- **All messages MUST include `uint64_t timestamp` field** for logging
- Use `uORB::Publication<msg_type>` to publish, `uORB::Subscription<msg_type>` to subscribe
- Check `docs/en/middleware/uorb.md` for details

### Module Pattern
Modules inherit from `ModuleBase<T>` and run either as:
1. **Work Queue Tasks** (preferred, less RAM): Inherit from `ModuleBase<T>` and `px4::ScheduledWorkItem`
   - Schedule with `ScheduleOnInterval()`, `ScheduleDelayed()`, or `ScheduleNow()`
   - Implement `void Run()` override for periodic work
   - Example: `src/examples/work_item/`
2. **Standalone Tasks**: Inherit only from `ModuleBase<T>`
   - Implement `void run()` override with its own loop
   - Higher memory usage but more control

Both require implementing:
- `static int task_spawn(int argc, char *argv[])`
- `static T *instantiate(int argc, char *argv[])`
- `static int custom_command(int argc, char *argv[])`
- `static int print_usage(const char *reason = nullptr)`

## Build System

### CMake Structure
- Uses custom `px4_add_module()` function in CMakeLists.txt files (see `cmake/px4_add_module.cmake`)
- Build targets format: `<vendor>_<board>_<config>` (e.g., `px4_fmu-v5_default`)
- All module CMakeLists must include dependencies like `uorb_headers` for uORB access

### Building & Testing
```bash
# SITL (Software-In-The-Loop) - primary development target
make px4_sitl_default                    # Build for simulation
make px4_sitl_default sitl_gazebo-classic # Build + launch Gazebo Classic
make px4_sitl_default sitl_gz            # Build + launch Gazebo (new)

# Hardware targets
make <vendor>_<board>_<config>           # Build for specific board
make <vendor>_<board>_<config> upload    # Build and flash

# Testing
make tests                               # Run all tests
make px4_sitl test                      # Run SITL tests
```

View all board targets: `find boards -name '*.px4board' | sed 's|boards/||; s|\.px4board||; s|/|_|g'`

### Makefile Conventions
- Never use quotes around CMake variables in conditionals: `if(${VAR} STREQUAL "value")` not `if("${VAR}" ...)`
- Hard tabs (width 8) for indentation in C/C++/CMake/Kconfig (`.editorconfig`)
- Spaces for YAML files (2 spaces)
- Max line length: 120 chars (C/C++), 80 chars (shell scripts)

## Code Organization

### Directory Structure
- `src/modules/` - Main flight control modules (attitude control, position control, etc.)
- `src/drivers/` - Hardware drivers
- `src/lib/` - Reusable libraries (math, control algorithms, etc.)
- `platforms/` - OS-specific code (NuttX, POSIX, QURT)
- `boards/` - Board-specific configurations organized by vendor
- `msg/` - uORB message definitions
- `ROMFS/` - Root filesystem for embedded targets

### Platform Abstraction
Code must work across NuttX (embedded), POSIX (simulation), and QURT (Snapdragon). Use `px4_` prefixed functions for platform-agnostic operations (e.g., `px4_usleep()`, `px4_open()`)

## Development Workflow

### Adding a New Module
1. Create directory in appropriate location (`src/modules/`, `src/drivers/`, etc.)
2. Add `CMakeLists.txt` with `px4_add_module(MODULE module_name ...)`
3. Inherit from `ModuleBase<YourClass>` (and optionally `ScheduledWorkItem`)
4. Implement required static methods and `Run()`/`run()` override
5. Add to board config's `default.cmake` to enable it

### Adding uORB Messages
1. Create `msg/YourMessage.msg` with CamelCase name
2. Add to `msg/CMakeLists.txt` in the `set(msg_files ...)` list
3. Include generated header: `#include <uORB/topics/your_message.h>`
4. Reference with `ORB_ID(your_message)`

### Flight Modes & Controllers
- Flight modes are in `src/modules/flight_mode_manager/`
- Attitude control: `src/modules/mc_att_control/` (multicopter), `src/modules/fw_att_control/` (fixed wing)
- Position control: `src/modules/mc_pos_control/`, etc.
- VTOL logic: `src/modules/vtol_att_control/`

## Testing & Debugging

### SITL Development
SITL is the primary development environment. Use these workflows:
- `make px4_sitl_default` builds without launching simulator
- Launch PX4 shell: `./build/px4_sitl_default/bin/px4` (then use `commander`, `logger`, etc.)
- View live uORB topics: `uorb top`, `listener <topic_name>`
- Check parameters: `param show`, `param set <name> <value>`

### Common Tools
- `mavlink shell` - Connect to PX4 shell over MAVLink
- `logger` module - Log uORB topics to SD card
- Flight logs: Upload to `logs.px4.io` (Flight Review) for analysis

## Conventions & Standards

### Coding Style
- Follow `.clang-tidy` rules (many checks enabled, see file)
- Use C++14 standard
- Hard tabs for indentation (not spaces) in C/C++ files
- Include guards in headers: `#pragma once` is acceptable
- Module documentation: Use `PRINT_MODULE_*` macros for command-line help

### Parameter Naming
Parameters use uppercase with underscores (e.g., `MC_ROLL_P`). Define in module using `PARAM_DEFINE_*` macros.

### Git Workflow
- Main branch: `main`
- Follow GitHub flow: branch → PR → merge
- Commit messages: Descriptive with issue references (`Fixes #123`)
- Include test flight logs in PRs when relevant (upload to logs.px4.io)

## Key Files to Reference

- `platforms/common/include/px4_platform_common/module.h` - ModuleBase template
- `platforms/common/include/px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp` - Work queue pattern
- `platforms/common/uORB/` - uORB implementation
- `src/examples/work_item/` - Work queue module example
- `cmake/px4_add_module.cmake` - Module build system
- `CMakeLists.txt` - Root CMake file with coding standards comments
