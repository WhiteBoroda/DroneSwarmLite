cmake_minimum_required(VERSION 3.16)

# vcpkg integration - ВАЖНО: до project()!
if(DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
            CACHE STRING "")
    message(STATUS "Using vcpkg toolchain: ${CMAKE_TOOLCHAIN_FILE}")
endif()

# Project configuration
project(UkrainianSwarmControl
        VERSION 1.0.0
        DESCRIPTION "Ukrainian Drone Swarm Control System - Слава Україні!"
        LANGUAGES CXX C
)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -Wextra -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -DNDEBUG")

# Add custom flags for Ukrainian build
add_definitions(-DUKRAINE_BUILD=1)
add_definitions(-DSLAVA_UKRAINI=1)

# Find required packages через vcpkg
find_package(yaml-cpp CONFIG REQUIRED)
find_package(OpenSSL REQUIRED)

# Optional packages
find_package(nlohmann_json CONFIG REQUIRED)

# Check if packages found
if(yaml-cpp_FOUND)
    message(STATUS "✅ yaml-cpp found: ${yaml-cpp_VERSION}")
else()
    message(FATAL_ERROR "❌ yaml-cpp not found! Install: vcpkg install yaml-cpp:x64-windows")
endif()

if(OpenSSL_FOUND)
    message(STATUS "✅ OpenSSL found: ${OPENSSL_VERSION}")
else()
    message(FATAL_ERROR "❌ OpenSSL not found! Install: vcpkg install openssl:x64-windows")
endif()

# Include directories
include_directories(
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/src
)

# Source files
set(SWARM_CONTROL_SOURCES
        # Core implementation files
        src/main.cpp
        src/MeshProtocol.cpp
        src/DistributedPositioning.cpp
        src/AutonomousDroneAgent.cpp
        src/UWBManager.cpp
        src/ConfigManager.cpp
        src/CryptoManager.cpp

        # Utility and helper files
        src/SwarmTypes.cpp
)

set(SWARM_CONTROL_HEADERS
        include/SwarmTypes.h
        include/MeshProtocol.h
        include/DistributedPositioning.h
        include/AutonomousDroneAgent.h
        include/UWBManager.h
        include/ConfigManager.h
        include/CryptoManager.h
)

# Create main executable
add_executable(swarm_control
        ${SWARM_CONTROL_SOURCES}
        ${SWARM_CONTROL_HEADERS}
)

# Link libraries - ВАЖНО: используй новый синтаксис
target_link_libraries(swarm_control
        PRIVATE
        yaml-cpp::yaml-cpp
        OpenSSL::SSL
        OpenSSL::Crypto
        nlohmann_json::nlohmann_json
)

# Windows specific libraries
if(WIN32)
    target_link_libraries(swarm_control PRIVATE ws2_32 wsock32)
endif()

# Installation
install(TARGETS swarm_control
        RUNTIME DESTINATION bin
)

install(FILES config/swarm_config.yaml
        DESTINATION bin/config/
        COMPONENT config
)

# Print build configuration
message(STATUS "")
message(STATUS "Ukrainian Swarm Control System - Build Configuration")
message(STATUS "========================================================")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
if(DEFINED CMAKE_TOOLCHAIN_FILE)
    message(STATUS "vcpkg toolchain: ${CMAKE_TOOLCHAIN_FILE}")
endif()
message(STATUS "")
message(STATUS "Dependencies:")
message(STATUS "  yaml-cpp: ${yaml-cpp_VERSION}")
message(STATUS "  OpenSSL: ${OPENSSL_VERSION}")
message(STATUS "")
message(STATUS "Слава Україні! 💛💙")
message(STATUS "========================================================")
message(STATUS "")

# Check for hardware-specific libraries (may need custom installation)
find_path(LORA_INCLUDE_DIR
        NAMES sx126x.h sx127x.h
        PATHS /usr/local/include/lora /usr/include/lora
        HINTS ENV LORA_INCLUDE_PATH
)

find_library(LORA_LIBRARY
        NAMES lora sx126x sx127x
        PATHS /usr/local/lib /usr/lib
        HINTS ENV LORA_LIB_PATH
)

# UWB library (DW1000/DW3000)
find_path(UWB_INCLUDE_DIR
        NAMES dw1000.h dw3000.h
        PATHS /usr/local/include/uwb /usr/include/uwb
        HINTS ENV UWB_INCLUDE_PATH
)

find_library(UWB_LIBRARY
        NAMES uwb dw1000 dw3000
        PATHS /usr/local/lib /usr/lib
        HINTS ENV UWB_LIB_PATH
)

# Video/Camera libraries
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(GSTREAMER gstreamer-1.0)
    pkg_check_modules(GSTREAMER_APP gstreamer-app-1.0)
    pkg_check_modules(V4L2 libv4l2)
endif()

# Include directories
include_directories(
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/src
)

# Add conditional includes if libraries are found
if(LORA_INCLUDE_DIR)
    include_directories(${LORA_INCLUDE_DIR})
    message(STATUS "LoRa support: ENABLED")
    add_definitions(-DLORA_SUPPORT=1)
else()
    message(WARNING "LoRa support: DISABLED (libraries not found)")
    add_definitions(-DLORA_SUPPORT=0)
endif()

if(UWB_INCLUDE_DIR)
    include_directories(${UWB_INCLUDE_DIR})
    message(STATUS "UWB support: ENABLED")
    add_definitions(-DUWB_SUPPORT=1)
else()
    message(WARNING "UWB support: DISABLED (libraries not found)")
    add_definitions(-DUWB_SUPPORT=0)
endif()

if(GSTREAMER_FOUND)
    include_directories(${GSTREAMER_INCLUDE_DIRS})
    message(STATUS "GStreamer video support: ENABLED")
    add_definitions(-DVIDEO_SUPPORT=1)
else()
    message(WARNING "Video support: DISABLED (GStreamer not found)")
    add_definitions(-DVIDEO_SUPPORT=0)
endif()

# Source files
set(SWARM_CONTROL_SOURCES
        # Core implementation files (to be created)
        src/Drone.cpp
        src/SwarmManager.cpp
        src/CommunicationManager.cpp
        src/FlightController.cpp
        src/UWBManager.cpp
        src/VideoTransmitter.cpp
        src/FormationController.cpp

        # Utility and helper files
        src/SwarmTypes.cpp
        src/ConfigManager.cpp
        src/Logger.cpp
        src/MathUtils.cpp
        src/HardwareInterface.cpp

        # Protocol implementations
        src/LoRaProtocol.cpp
        src/ELRSProtocol.cpp
        src/UWBProtocol.cpp
        src/VideoProtocol.cpp

        # Mission and planning
        src/MissionPlanner.cpp
        src/PathPlanning.cpp
        src/NavigationController.cpp

        # Safety and monitoring
        src/SafetyMonitor.cpp
        src/FailsafeManager.cpp
        src/BatteryMonitor.cpp

        # Ground station interface
        src/GroundStationInterface.cpp
        src/TelemetryManager.cpp
)

set(SWARM_CONTROL_HEADERS
        include/SwarmTypes.h
        include/Drone.h
        include/SwarmManager.h
        include/CommunicationManager.h
        include/FlightController.h
        include/UWBManager.h
        include/VideoTransmitter.h
        include/FormationController.h
        include/ConfigManager.h
        include/Logger.h
        include/MathUtils.h
        include/HardwareInterface.h
        src/CryptoManager.cpp
)

# Create main executable
add_executable(swarm_control
        main.cpp
        ${SWARM_CONTROL_SOURCES}
        ${SWARM_CONTROL_HEADERS}
)

# Link libraries
target_link_libraries(swarm_control
        ${CMAKE_THREAD_LIBS_INIT}
        yaml-cpp
        ${RT_LIBRARY}
        m  # Math library
)

# Conditional library linking
if(LORA_LIBRARY)
    target_link_libraries(swarm_control ${LORA_LIBRARY})
endif()

if(UWB_LIBRARY)
    target_link_libraries(swarm_control ${UWB_LIBRARY})
endif()

if(GSTREAMER_FOUND)
    target_link_libraries(swarm_control
            ${GSTREAMER_LIBRARIES}
            ${GSTREAMER_APP_LIBRARIES}
    )
endif()

if(V4L2_FOUND)
    target_link_libraries(swarm_control ${V4L2_LIBRARIES})
endif()

# Installation
install(TARGETS swarm_control
        RUNTIME DESTINATION bin
)

install(FILES swarm_config.yaml
        DESTINATION /etc/swarm_control/
        COMPONENT config
)

install(DIRECTORY scripts/
        DESTINATION /usr/local/bin/
        FILES_MATCHING PATTERN "*.sh"
        PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
        GROUP_EXECUTE GROUP_READ
        WORLD_EXECUTE WORLD_READ
)

# Create systemd service files
configure_file(
        ${CMAKE_SOURCE_DIR}/scripts/swarm_control.service.in
        ${CMAKE_BINARY_DIR}/swarm_control.service
        @ONLY
)

install(FILES ${CMAKE_BINARY_DIR}/swarm_control.service
        DESTINATION /etc/systemd/system/
        COMPONENT service
)

# Testing (optional)
option(BUILD_TESTS "Build test suite" OFF)
if(BUILD_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)

    add_subdirectory(tests)

    # Unit tests
    add_executable(swarm_tests
            tests/test_formation_controller.cpp
            tests/test_communication.cpp
            tests/test_uwb_positioning.cpp
            tests/test_swarm_manager.cpp
            # Add test source files
            ${SWARM_CONTROL_SOURCES}
    )

    target_link_libraries(swarm_tests
            ${CMAKE_THREAD_LIBS_INIT}
            yaml-cpp
            ${RT_LIBRARY}
            GTest::gtest
            GTest::gtest_main
            m
    )

    add_test(NAME SwarmControlTests COMMAND swarm_tests)
endif()

# Documentation
option(BUILD_DOCS "Build documentation" OFF)
if(BUILD_DOCS)
    find_package(Doxygen)
    if(DOXYGEN_FOUND)
        set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
        set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

        configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

        add_custom_target(docs ALL
                COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
                WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                COMMENT "Generating API documentation with Doxygen"
                VERBATIM
        )
    endif()
endif()

# Development tools
option(ENABLE_CLANG_TIDY "Enable clang-tidy" OFF)
if(ENABLE_CLANG_TIDY)
    find_program(CLANG_TIDY_EXE NAMES "clang-tidy")
    if(CLANG_TIDY_EXE)
        set_target_properties(swarm_control PROPERTIES
                CXX_CLANG_TIDY "${CLANG_TIDY_EXE};-checks=-*,readability-*,performance-*,modernize-*"
        )
    endif()
endif()

# Print build configuration
message(STATUS "")
message(STATUS "Ukrainian Swarm Control System - Build Configuration")
message(STATUS "========================================================")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "")
message(STATUS "Features:")
message(STATUS "  LoRa Support: ${LORA_SUPPORT}")
message(STATUS "  UWB Support: ${UWB_SUPPORT}")
message(STATUS "  Video Support: ${VIDEO_SUPPORT}")
message(STATUS "  Build Tests: ${BUILD_TESTS}")
message(STATUS "  Build Docs: ${BUILD_DOCS}")
message(STATUS "")
message(STATUS "Слава Україні! 💛💙")
message(STATUS "========================================================")
message(STATUS "")

# Custom targets for development
add_custom_target(format
        COMMAND find ${CMAKE_SOURCE_DIR} -name '*.cpp' -o -name '*.h' | xargs clang-format -i
        COMMENT "Formatting source code"
)

add_custom_target(clean-all
        COMMAND ${CMAKE_BUILD_TOOL} clean
        COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/CMakeFiles
        COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/CMakeCache.txt
        COMMENT "Deep clean build directory"
)