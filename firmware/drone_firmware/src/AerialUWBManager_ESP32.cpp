//=============================================================================
// ✅ ESP32 AERIAL UWB IMPLEMENTATION
// High-speed drone positioning with dynamic TAG/ANCHOR switching
// 🇺🇦 Slava Ukraini! 🇺🇦
//=============================================================================

// firmware/drone_firmware/src/AerialUWBManager_ESP32.cpp

#include "../include/AerialUWBManager_ESP32.h"
#include "../include/HardwareConfig.h"
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <math.h>

namespace AerialUWB_ESP32 {

//=============================================================================
// ✅ AERIAL UWB CONFIGURATION FOR HIGH-SPEED DRONES
//=============================================================================

// Optimized configuration for fast-moving drones (100 km/h)
    static const AerialUWBConfig_ESP32 AERIAL_CONFIG = {
            .fast_ranging_rate_hz = 200,         // 200 Hz for high-speed tracking
            .max_velocity_ms = 27.78f,           // 100 km/h maximum
            .doppler_compensation_enabled = true,
            .motion_prediction_enabled = true,
            .anchor_switch_interval_ms = 30000,  // Switch anchors every 30 seconds
            .min_anchor_stability = 0.8f,        // High stability requirement
            .ranging_timeout_us = 5000,          // 5ms timeout for fast ranging
            .position_prediction_ms = 100,       // 100ms position prediction
            .kalman_filter_enabled = true,
            .formation_sync_enabled = true
    };

//=============================================================================
// ✅ DYNAMIC ANCHOR/TAG SWITCHING
//=============================================================================

    bool AerialUWBManager_ESP32::switchToAnchorMode() {
        if (current_aerial_mode_ == AerialMode_ESP32::ANCHOR) {
            return true; // Already in anchor mode
        }

        UWB_DEBUG("🔄 Switching to ANCHOR mode");

        // Stop tag operations
        if (ranging_task_handle_ != nullptr) {
            vTaskSuspend(ranging_task_handle_);
        }

        // Configure hardware for anchor mode
        if (!configureForAnchorMode()) {
            UWB_ERROR("Failed to configure hardware for anchor mode");
            return false;
        }

        // Start anchor beacon transmission
        if (!startAnchorBeacon()) {
            UWB_ERROR("Failed to start anchor beacon");
            return false;
        }

        current_aerial_mode_ = AerialMode_ESP32::ANCHOR;
        anchor_start_time_ = esp_timer_get_time();

        // Update network about role change
        broadcastRoleChange(AerialMode_ESP32::ANCHOR);

        UWB_INFO("✅ Successfully switched to ANCHOR mode");
        return true;
    }

    bool AerialUWBManager_ESP32::switchToTagMode() {
        if (current_aerial_mode_ == AerialMode_ESP32::TAG) {
            return true; // Already in tag mode
        }

        UWB_DEBUG("🔄 Switching to TAG mode");

        // Stop anchor operations
        stopAnchorBeacon();

        // Configure hardware for tag mode
        if (!configureForTagMode()) {
            UWB_ERROR("Failed to configure hardware for tag mode");
            return false;
        }

        // Resume/start ranging operations
        if (ranging_task_handle_ != nullptr) {
            vTaskResume(ranging_task_handle_);
        } else {
            startRangingTask();
        }

        current_aerial_mode_ = AerialMode_ESP32::TAG;

        // Update network about role change
        broadcastRoleChange(AerialMode_ESP32::TAG);

        UWB_INFO("✅ Successfully switched to TAG mode");
        return true;
    }

    bool AerialUWBManager_ESP32::shouldSwitchRole() {
        uint64_t current_time = esp_timer_get_time();

        // Don't switch too frequently
        if (current_time - last_role_switch_ < CONFIG_MIN_ROLE_SWITCH_INTERVAL_US) {
            return false;
        }

        // Check if we're moving too fast to be a good anchor
        if (current_aerial_mode_ == AerialMode_ESP32::ANCHOR) {
            float current_speed = calculateCurrentSpeed();
            if (current_speed > AERIAL_CONFIG.max_velocity_ms * 0.3f) { // 30% of max speed
                UWB_DEBUG_F("Speed too high for anchor: %.2f m/s", current_speed);
                return true; // Switch to tag
            }

            // Check if we've been anchor long enough
            if (current_time - anchor_start_time_ > AERIAL_CONFIG.anchor_switch_interval_ms * 1000) {
                return true; // Time to rotate
            }
        }

        // Check if network needs more anchors
        if (current_aerial_mode_ == AerialMode_ESP32::TAG) {
            if (active_anchor_count_ < MIN_ANCHORS_NEEDED && canServeAsAnchor()) {
                return true; // Need to become anchor
            }
        }

        return false;
    }

//=============================================================================
// ✅ HIGH-SPEED RANGING WITH MOTION COMPENSATION
//=============================================================================

    bool AerialUWBManager_ESP32::performHighSpeedRanging(uint8_t target_id) {
        FastRangingFrame_ESP32 ranging_frame = {};

        // Prepare fast ranging frame
        ranging_frame.frame_control = 0x41;
        ranging_frame.sequence_number = ++current_sequence_;
        ranging_frame.sender_id = my_drone_id_;
        ranging_frame.target_id = target_id;
        ranging_frame.message_type = UWBMessageType_ESP32::FAST_RANGING_REQUEST;

        // Add current position and velocity for motion compensation
        ranging_frame.sender_position = current_position_;
        ranging_frame.sender_velocity = current_velocity_;
        ranging_frame.timestamp_tx = esp_timer_get_time();

        // Set frame-specific parameters for high-speed ranging
        ranging_frame.ranging_timeout_us = AERIAL_CONFIG.ranging_timeout_us;
        ranging_frame.expected_response_delay_us = 1000; // 1ms expected delay

        // Calculate CRC
        ranging_frame.crc = calculateCRC16((uint8_t*)&ranging_frame,
                                           sizeof(ranging_frame) - sizeof(ranging_frame.crc));

        // Transmit ranging request
        if (!transmitFrame(ranging_frame)) {
            UWB_ERROR_F("Failed to transmit fast ranging request to node %d", target_id);
            return false;
        }

        // Wait for response with motion compensation
        FastRangingFrame_ESP32 response_frame;
        uint64_t tx_timestamp = ranging_frame.timestamp_tx;

        if (!waitForFastRangingResponse(target_id, response_frame,
                                        AERIAL_CONFIG.ranging_timeout_us)) {
            return false;
        }

        // Calculate motion-compensated distance
        MotionCompensatedMeasurement_ESP32 measurement;
        if (!calculateMotionCompensatedDistance(ranging_frame, response_frame, measurement)) {
            return false;
        }

        // Store measurement
        addMotionCompensatedMeasurement(measurement);

        UWB_DEBUG_F("High-speed ranging to node %d: %.2f m (velocity: %.2f m/s)",
                    target_id, measurement.distance_m, measurement.relative_velocity_ms);

        return true;
    }

    bool AerialUWBManager_ESP32::calculateMotionCompensatedDistance(
            const FastRangingFrame_ESP32& request,
            const FastRangingFrame_ESP32& response,
            MotionCompensatedMeasurement_ESP32& measurement) {

        // Calculate raw time-of-flight
        uint64_t round_trip_time = response.timestamp_rx - request.timestamp_tx;
        uint64_t response_processing_time = response.timestamp_tx - response.timestamp_rx;
        uint64_t time_of_flight = (round_trip_time - response_processing_time) / 2;

        // Convert to distance
        double raw_distance = (double)time_of_flight * UWB_TIME_TO_DISTANCE_FACTOR;

        // Calculate relative velocity for Doppler compensation
        Position3D_ESP32 relative_position = {
                response.sender_position.x - request.sender_position.x,
                response.sender_position.y - request.sender_position.y,
                response.sender_position.z - request.sender_position.z
        };

        Position3D_ESP32 relative_velocity = {
                response.sender_velocity.x - request.sender_velocity.x,
                response.sender_velocity.y - request.sender_velocity.y,
                response.sender_velocity.z - request.sender_velocity.z
        };

        // Calculate radial velocity (along line-of-sight)
        double distance_3d = sqrt(relative_position.x * relative_position.x +
                                  relative_position.y * relative_position.y +
                                  relative_position.z * relative_position.z);

        if (distance_3d > 0.1) { // Avoid division by zero
            double radial_velocity = (relative_position.x * relative_velocity.x +
                                      relative_position.y * relative_velocity.y +
                                      relative_position.z * relative_velocity.z) / distance_3d;

            // Doppler shift correction
            double doppler_factor = 1.0 + (radial_velocity / SPEED_OF_LIGHT);
            double corrected_distance = raw_distance / doppler_factor;

            // Fill measurement structure
            measurement.target_id = response.sender_id;
            measurement.distance_m = corrected_distance;
            measurement.relative_velocity_ms = radial_velocity;
            measurement.doppler_shift_hz = (radial_velocity * UWB_CENTER_FREQUENCY) / SPEED_OF_LIGHT;
            measurement.timestamp_us = esp_timer_get_time();
            measurement.accuracy_m = estimateRangingAccuracy(round_trip_time, radial_velocity);
            measurement.motion_compensated = true;

            return true;
        }

        return false;
    }

//=============================================================================
// ✅ MOTION PREDICTION AND KALMAN FILTERING
//=============================================================================

    bool AerialUWBManager_ESP32::updateMotionFilter(const Position3D_ESP32& measured_position) {
        uint64_t current_time_us = esp_timer_get_time();
        double dt = (current_time_us - last_filter_update_us_) / 1000000.0; // Convert to seconds

        if (dt > 1.0) { // Reset if too much time has passed
            initializeMotionFilter(measured_position);
            last_filter_update_us_ = current_time_us;
            return true;
        }

        // Kalman filter prediction step
        predictMotionState(dt);

        // Kalman filter update step
        updateMotionState(measured_position);

        // Extract velocity and acceleration estimates
        current_velocity_.x = motion_state_[3];
        current_velocity_.y = motion_state_[4];
        current_velocity_.z = motion_state_[5];

        current_acceleration_.x = motion_state_[6];
        current_acceleration_.y = motion_state_[7];
        current_acceleration_.z = motion_state_[8];

        last_filter_update_us_ = current_time_us;

        return true;
    }

    Position3D_ESP32 AerialUWBManager_ESP32::predictPositionAtTime(uint64_t future_time_us) {
        uint64_t current_time_us = esp_timer_get_time();
        double dt = (future_time_us - current_time_us) / 1000000.0; // Convert to seconds

        // Simple kinematic prediction: x = x0 + v*t + 0.5*a*t^2
        Position3D_ESP32 predicted_position;

        predicted_position.x = current_position_.x +
                               current_velocity_.x * dt +
                               0.5 * current_acceleration_.x * dt * dt;

        predicted_position.y = current_position_.y +
                               current_velocity_.y * dt +
                               0.5 * current_acceleration_.y * dt * dt;

        predicted_position.z = current_position_.z +
                               current_velocity_.z * dt +
                               0.5 * current_acceleration_.z * dt * dt;

        // Estimate prediction accuracy based on time horizon
        predicted_position.accuracy_meters = current_position_.accuracy_meters * (1.0f + dt);
        predicted_position.timestamp_us = future_time_us;

        return predicted_position;
    }

//=============================================================================
// ✅ FORMATION SYNCHRONIZATION
//=============================================================================

    bool AerialUWBManager_ESP32::synchronizeWithFormation() {
        if (formation_leader_id_ == 0) {
            return false; // No formation leader set
        }

        // Request position update from formation leader
        FormationSyncFrame_ESP32 sync_request = {};
        sync_request.frame_control = 0x41;
        sync_request.sequence_number = ++current_sequence_;
        sync_request.sender_id = my_drone_id_;
        sync_request.target_id = formation_leader_id_;
        sync_request.message_type = UWBMessageType_ESP32::FORMATION_SYNC_REQUEST;
        sync_request.timestamp_tx = esp_timer_get_time();
        sync_request.current_position = current_position_;
        sync_request.current_velocity = current_velocity_;

        if (!transmitFrame(sync_request)) {
            return false;
        }

        // Wait for formation leader response
        FormationSyncFrame_ESP32 sync_response;
        if (!waitForFormationSyncResponse(formation_leader_id_, sync_response, 10000)) {
            return false;
        }

        // Update formation reference
        formation_center_ = sync_response.formation_center;
        formation_velocity_ = sync_response.formation_velocity;

        // Adjust our coordinate system based on formation
        adjustCoordinateSystem(sync_response);

        UWB_DEBUG("✅ Formation synchronization completed");
        return true;
    }

    bool AerialUWBManager_ESP32::broadcastFormationPosition() {
        if (current_aerial_mode_ != AerialMode_ESP32::FORMATION_LEADER) {
            return false;
        }

        FormationSyncFrame_ESP32 position_broadcast = {};
        position_broadcast.frame_control = 0x41;
        position_broadcast.sequence_number = ++current_sequence_;
        position_broadcast.sender_id = my_drone_id_;
        position_broadcast.target_id = 0xFFFF; // Broadcast
        position_broadcast.message_type = UWBMessageType_ESP32::FORMATION_POSITION_BROADCAST;
        position_broadcast.timestamp_tx = esp_timer_get_time();
        position_broadcast.formation_center = formation_center_;
        position_broadcast.formation_velocity = formation_velocity_;
        position_broadcast.formation_accuracy = calculateFormationAccuracy();

        return transmitFrame(position_broadcast);
    }

//=============================================================================
// ✅ FREERTOS TASKS FOR AERIAL OPERATIONS
//=============================================================================

    void AerialUWBManager_ESP32::aerialManagementTask(void* parameter) {
        AerialUWBManager_ESP32* uwb = static_cast<AerialUWBManager_ESP32*>(parameter);

        UWB_INFO("🚁 Aerial management task started");

        TickType_t last_wake_time = xTaskGetTickCount();
        const TickType_t task_period = pdMS_TO_TICKS(100); // 100ms cycle

        while (uwb->running_) {
            UWB_FEED_WATCHDOG();

            // Check if role switching is needed
            if (uwb->shouldSwitchRole()) {
                if (uwb->current_aerial_mode_ == AerialMode_ESP32::ANCHOR) {
                    uwb->switchToTagMode();
                } else if (uwb->canServeAsAnchor()) {
                    uwb->switchToAnchorMode();
                }
            }

            // Update motion state
            uwb->updateMotionTracking();

            // Synchronize with formation if in formation mode
            if (uwb->formation_leader_id_ != 0 &&
                uwb->current_aerial_mode_ != AerialMode_ESP32::FORMATION_LEADER) {
                uwb->synchronizeWithFormation();
            }

            // Broadcast formation position if we're the leader
            if (uwb->current_aerial_mode_ == AerialMode_ESP32::FORMATION_LEADER) {
                uwb->broadcastFormationPosition();
            }

            // Cleanup old measurements
            uwb->cleanupOldMeasurements();

            vTaskDelayUntil(&last_wake_time, task_period);
        }

        UWB_INFO("🚁 Aerial management task stopped");
        vTaskDelete(nullptr);
    }

    void AerialUWBManager_ESP32::highSpeedRangingTask(void* parameter) {
        AerialUWBManager_ESP32* uwb = static_cast<AerialUWBManager_ESP32*>(parameter);

        UWB_INFO("⚡ High-speed ranging task started");

        TickType_t last_wake_time = xTaskGetTickCount();
        const TickType_t task_period = pdMS_TO_TICKS(1000 / AERIAL_CONFIG.fast_ranging_rate_hz);

        while (uwb->running_) {
            UWB_FEED_WATCHDOG();

            // Only perform ranging if we're in tag mode or dynamic mode
            if (uwb->current_aerial_mode_ == AerialMode_ESP32::TAG ||
                uwb->current_aerial_mode_ == AerialMode_ESP32::DYNAMIC) {

                // Range to all known active anchors
                for (uint8_t i = 0; i < uwb->known_nodes_count_; i++) {
                    if (uwb->known_nodes_[i].is_anchor && uwb->known_nodes_[i].is_active) {
                        uwb->performHighSpeedRanging(uwb->known_nodes_[i].node_id);

                        // Small delay between ranging attempts
                        vTaskDelay(pdMS_TO_TICKS(2));
                    }
                }

                // Update position based on latest measurements
                uwb->updateAerialPosition();
            }

            vTaskDelayUntil(&last_wake_time, task_period);
        }

        UWB_INFO("⚡ High-speed ranging task stopped");
        vTaskDelete(nullptr);
    }

//=============================================================================
// ✅ HARDWARE CONFIGURATION FOR AERIAL OPERATIONS
//=============================================================================

    bool AerialUWBManager_ESP32::configureForAerialOperations() {
        UWB_DEBUG("⚙️ Configuring UWB for aerial operations");

        // Set high data rate for fast ranging
        if (!setDataRate(6800)) { // 6.8 Mbps for fast frames
            return false;
        }

        // Set appropriate preamble length for aerial range
        if (!setPreambleLength(64)) { // Shorter preamble for speed
            return false;
        }

        // Configure timing for aerial distances (up to 2km)
        if (!setReceiveTimeout(2000)) { // 2000 microseconds
            return false;
        }

        // Enable frame filtering for aerial frames
        if (!enableFrameFiltering(true)) {
            return false;
        }

        // Set maximum TX power for aerial range
        if (!setTxPower(33)) { // Maximum power
            return false;
        }

        UWB_INFO("✅ UWB configured for aerial operations");
        return true;
    }

} // namespace AerialUWB_ESP32
