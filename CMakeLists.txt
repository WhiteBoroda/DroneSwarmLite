# CMakeLists.txt
# Система управления роем дронов для защиты Украины
# 🇺🇦 Slava Ukraini! 🇺🇦

cmake_minimum_required(VERSION 3.16)
project(SwarmControlSystem
        VERSION 2.0.0
        DESCRIPTION "Ukrainian Drone Swarm Control System with Hot Config Reload"
        LANGUAGES CXX)

# C++ стандарт и флаги компиляции
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Требуем поддержку filesystem для ConfigWatcher
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lstdc++fs")

# Оптимизация для продакшена
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -DDEBUG -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=native")

# vcpkg integration
if(DEFINED CMAKE_TOOLCHAIN_FILE)
    message(STATUS "🔧 Using vcpkg toolchain: ${CMAKE_TOOLCHAIN_FILE}")
endif()

# Find required packages
find_package(yaml-cpp CONFIG REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

# Проверяем что все зависимости найдены
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

# Source files - ОБНОВЛЕНО: добавлен ConfigWatcher
set(SWARM_CONTROL_SOURCES
        # Core implementation files
        src/main.cpp
        src/MeshProtocol.cpp
        src/DistributedPositioning.cpp
        src/AutonomousDroneAgent.cpp
        src/CommunicationManager.cpp
        src/ConfigWatcher.cpp                # ← ДОБАВЛЕНО!
        src/UWBManager.cpp
        src/UWBCryptoSync.cpp
        src/LoRaSystem.cpp
        src/ConfigManager.cpp
        src/CryptoManager.cpp

        # Utility and helper files
        src/SwarmTypes.cpp
        src/AutonomousDroneAgent.cpp
)

set(SWARM_CONTROL_HEADERS
        include/SwarmTypes.h
        include/CommunicationManager.h
        include/ConfigWatcher.h             # ← ДОБАВЛЕНО!
        include/LoRaSystem.h
        include/UWBCryptoSync.h
        include/MeshProtocol.h
        include/DistributedPositioning.h
        include/AutonomousDroneAgent.h
        include/UWBManager.h
        include/ConfigManager.h
        include/CryptoManager.h
        firmware/drone_firmware/include/UWBManager_ESP32.h
        firmware/drone_firmware/src/UWBManager_ESP32.cpp
        src/AutonomousDroneAgent.cpp
        include/UWBUtilities.h
        include/FormationController.h
        include/PowerManager.h
        include/SafetyManager.h
        firmware/drone_firmware/src/AerialUWBManager_ESP32.cpp
        firmware/drone_firmware/include/AerialUWBManager_ESP32.h
        src/AerialUWBManager.cpp
        include/AerialUWBManager.h
        include/AerialCoordinateSystem.h
        include/HighSpeedRanging.h
        include/ConfigHandlers.h
)

# Create main executable
add_executable(swarm_control
        ${SWARM_CONTROL_SOURCES}
        ${SWARM_CONTROL_HEADERS}
)

# Link libraries - ВАЖНО: используй новый синтаксис + filesystem
target_link_libraries(swarm_control
        PRIVATE
        yaml-cpp::yaml-cpp
        OpenSSL::SSL
        OpenSSL::Crypto
        nlohmann_json::nlohmann_json
)

# Platform specific filesystem library
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS "9.0")
    target_link_libraries(swarm_control PRIVATE stdc++fs)
endif()

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
message(STATUS "🔥 NEW FEATURES:")
message(STATUS "  ✅ Hot configuration reload support")
message(STATUS "  ✅ Real-time frequency changes")
message(STATUS "  ✅ Dynamic power adjustments")
message(STATUS "  ✅ Mesh networking controls")
message(STATUS "")
message(STATUS "🇺🇦 Slava Ukraini! HEROIAM SLAVA! 🇺🇦")