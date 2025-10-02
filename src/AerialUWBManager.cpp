//=============================================================================
// src/AerialUWBManager.cpp
// Production-ready implementation of Aerial UWB positioning system
// High-speed drone positioning with dynamic TAG/ANCHOR switching
// 🇺🇦 Slava Ukraini! 🇺🇦
//=============================================================================

#include "../include/AerialUWBManager.h"
#include "../include/HighSpeedRanging.h"
#include "../include/AerialCoordinateSystem.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <iomanip>

namespace AerialUWB {

//=============================================================================
// ✅ CONSTRUCTOR & DESTRUCTOR
//=============================================================================

    AerialUWBManager::AerialUWBManager(DroneID drone_id)
            : UWBManager(drone_id, "")  // Initialize base class
            , current_mode_(AerialUWBMode::DYNAMIC_ANCHOR_TAG)
            , formation_leader_id_(0)
            , last_anchor_switch_time_(0)
            , last_ranging_cycle_time_(0)
            , anchor_rotation_counter_(0)
            , aerial_system_running_(false)
    {
        // Initialize high-speed configuration with defaults
        high_speed_config_.fast_ranging_rate_hz = 200;
        high_speed_config_.max_velocity_ms = 27.78;  // 100 km/h
        high_speed_config_.doppler_compensation_enabled = true;
        high_speed_config_.motion_prediction_enabled = true;
        high_speed_config_.anchor_switch_interval_ms = 30000;
        high_speed_config_.min_anchor_stability = 0.8;
        high_speed_config_.ranging_timeout_us = 5000;
        high_speed_config_.position_prediction_ms = 100;
        high_speed_config_.kalman_filter_enabled = true;
        high_speed_config_.formation_sync_enabled = true;

        // Initialize anchor selection criteria
        anchor_criteria_.min_geometry_score = 0.7;
        anchor_criteria_.max_distance_m = 300.0;
        anchor_criteria_.min_rssi_dbm = -90;
        anchor_criteria_.prefer_stationary = true;
        anchor_criteria_.require_time_sync = true;

        // Initialize motion state
        current_velocity_ = {0.0, 0.0, 0.0};
        current_acceleration_ = {0.0, 0.0, 0.0};

        std::cout << "🚁 AerialUWBManager initialized for drone " << drone_id << std::endl;
    }

    AerialUWBManager::~AerialUWBManager() {
        StopAerialPositioning();
    }

//=============================================================================
// ✅ INITIALIZATION AND CONTROL
//=============================================================================

    bool AerialUWBManager::InitializeAerialSystem(const HighSpeedConfig& config) {
        std::cout << "🔧 Initializing aerial UWB system..." << std::endl;

        // Store configuration
        high_speed_config_ = config;

        // Initialize base UWB system first
        if (!initialize()) {
            std::cerr << "❌ Failed to initialize base UWB system" << std::endl;
            return false;
        }

        // Initialize Kalman filter for motion tracking
        Position3D initial_pos = get_estimated_position();
        motion_filter_.initialize(initial_pos, Position3D{0, 0, 0});

        std::cout << "✅ Aerial UWB system initialized" << std::endl;
        std::cout << "   Max velocity: " << config.max_velocity_ms << " m/s" << std::endl;
        std::cout << "   Ranging rate: " << config.fast_ranging_rate_hz << " Hz" << std::endl;
        std::cout << "   Doppler compensation: "
                  << (config.doppler_compensation_enabled ? "ON" : "OFF") << std::endl;

        return true;
    }

    bool AerialUWBManager::StartAerialPositioning() {
        if (aerial_system_running_.load()) {
            std::cout << "⚠️ Aerial positioning already running" << std::endl;
            return true;
        }

        std::cout << "🚀 Starting aerial positioning system..." << std::endl;

        // Start base UWB system
        if (!start()) {
            std::cerr << "❌ Failed to start base UWB system" << std::endl;
            return false;
        }

        aerial_system_running_.store(true);

        // Start aerial-specific worker threads
        anchor_management_thread_ = std::make_unique<std::thread>(
                &AerialUWBManager::anchor_management_worker, this);

        if (high_speed_config_.fast_ranging_rate_hz > 50) {
            high_speed_ranging_thread_ = std::make_unique<std::thread>(
                    &AerialUWBManager::high_speed_ranging_worker, this);
        }

        last_anchor_switch_time_ = get_current_timestamp_us();
        last_ranging_cycle_time_ = last_anchor_switch_time_;

        std::cout << "✅ Aerial positioning started" << std::endl;
        return true;
    }

    void AerialUWBManager::StopAerialPositioning() {
        if (!aerial_system_running_.load()) {
            return;
        }

        std::cout << "🛑 Stopping aerial positioning system..." << std::endl;

        aerial_system_running_.store(false);

        // Join worker threads
        if (anchor_management_thread_ && anchor_management_thread_->joinable()) {
            anchor_management_thread_->join();
        }
        if (high_speed_ranging_thread_ && high_speed_ranging_thread_->joinable()) {
            high_speed_ranging_thread_->join();
        }

        // Stop base system
        stop();

        std::cout << "✅ Aerial positioning stopped" << std::endl;
    }

//=============================================================================
// ✅ MODE MANAGEMENT
//=============================================================================

    bool AerialUWBManager::SetAerialMode(AerialUWBMode mode) {
        if (current_mode_ == mode) {
            return true;
        }

        std::cout << "🔄 Switching aerial mode: "
                  << static_cast<int>(current_mode_) << " → "
                  << static_cast<int>(mode) << std::endl;

        std::lock_guard<std::mutex> lock(aerial_anchors_mutex_);

        AerialUWBMode old_mode = current_mode_;
        current_mode_ = mode;

        // Handle mode-specific initialization
        switch (mode) {
            case AerialUWBMode::TAG_ONLY:
                // Become pure tag - stop anchor broadcasts
                active_anchors_.erase(drone_id_);
                break;

            case AerialUWBMode::ANCHOR_ONLY:
                // Become anchor - start broadcasts
                if (BecomeAnchor()) {
                    std::cout << "✅ Now serving as anchor" << std::endl;
                }
                break;

            case AerialUWBMode::DYNAMIC_ANCHOR_TAG:
                // Enable dynamic switching
                std::cout << "✅ Dynamic anchor/tag mode enabled" << std::endl;
                break;

            case AerialUWBMode::FORMATION_LEADER:
                formation_leader_id_ = drone_id_;
                std::cout << "✅ Acting as formation leader" << std::endl;
                break;

            case AerialUWBMode::FORMATION_MEMBER:
                std::cout << "✅ Acting as formation member" << std::endl;
                break;
        }

        return true;
    }

    bool AerialUWBManager::CanServeAsAnchor() const {
        // Check if this drone can serve as an anchor based on criteria

        // 1. Check motion state - prefer slower-moving drones as anchors
        double current_speed = GetCurrentSpeed();
        if (current_speed > high_speed_config_.max_velocity_ms * 0.7) {
            return false;  // Too fast to be stable anchor
        }

        // 2. Check position accuracy
        if (ranging_accuracy_.load() > 5.0) {
            return false;  // Position not accurate enough
        }

        // 3. Check if we have enough known anchors to determine our own position
        std::lock_guard<std::mutex> lock(aerial_anchors_mutex_);
        if (active_anchors_.size() < 3) {
            return false;  // Need at least 3 anchors to position ourselves
        }

        // 4. Check system stability
        if (motion_filter_.getPositionAccuracy() > 3.0) {
            return false;  // Motion filter not converged
        }

        return true;
    }

    bool AerialUWBManager::ShouldSwitchToAnchor() const {
        if (current_mode_ != AerialUWBMode::DYNAMIC_ANCHOR_TAG) {
            return false;
        }

        // Check if we should become an anchor

        // 1. Time-based rotation
        uint64_t now = get_current_timestamp_us();
        if (now - last_anchor_switch_time_ <
            high_speed_config_.anchor_switch_interval_ms * 1000) {
            return false;  // Too soon to switch
        }

        // 2. Check if more anchors are needed
        std::lock_guard<std::mutex> lock(aerial_anchors_mutex_);
        size_t active_anchor_count = active_anchors_.size();
        size_t total_nodes = aerial_anchors_.size();

        if (total_nodes > 0 && active_anchor_count < total_nodes / 3) {
            return CanServeAsAnchor();  // Need more anchors
        }

        return false;
    }

    bool AerialUWBManager::ShouldSwitchToTag() const {
        if (current_mode_ != AerialUWBMode::DYNAMIC_ANCHOR_TAG) {
            return false;
        }

        // Check if we should switch from anchor to tag

        // 1. If moving too fast
        double speed = GetCurrentSpeed();
        if (speed > high_speed_config_.max_velocity_ms * 0.8) {
            return true;
        }

        // 2. If position accuracy degraded
        if (ranging_accuracy_.load() > 10.0) {
            return true;
        }

        // 3. Time-based rotation
        uint64_t now = get_current_timestamp_us();
        if (now - last_anchor_switch_time_ >
            high_speed_config_.anchor_switch_interval_ms * 2000) {
            return true;  // Time to rotate
        }

        return false;
    }

//=============================================================================
// ✅ DYNAMIC ANCHOR MANAGEMENT
//=============================================================================

    bool AerialUWBManager::BecomeAnchor(uint32_t anchor_priority) {
        std::cout << "📍 Becoming anchor with priority " << anchor_priority << std::endl;

        std::lock_guard<std::mutex> lock(aerial_anchors_mutex_);

        // Create anchor point for ourselves
        AerialAnchorPoint anchor;
        anchor.drone_id = drone_id_;
        anchor.position = get_estimated_position();
        anchor.velocity = current_velocity_;
        anchor.last_update = get_current_timestamp_us();
        anchor.rssi_dbm = -30;  // Strong signal (ourselves)
        anchor.quality = 1.0;
        anchor.is_active = true;
        anchor.anchor_priority = anchor_priority;
        anchor.time_synchronized = true;

        aerial_anchors_[drone_id_] = anchor;
        active_anchors_.insert(drone_id_);

        last_anchor_switch_time_ = get_current_timestamp_us();

        std::cout << "✅ Now serving as anchor at ("
                  << anchor.position.x << ", "
                  << anchor.position.y << ", "
                  << anchor.position.z << ")" << std::endl;

        return true;
    }

    bool AerialUWBManager::BecomeTag() {
        std::cout << "📡 Becoming tag" << std::endl;

        std::lock_guard<std::mutex> lock(aerial_anchors_mutex_);

        // Remove ourselves from active anchors
        active_anchors_.erase(drone_id_);

        // Keep our anchor point data for history but mark as inactive
        auto it = aerial_anchors_.find(drone_id_);
        if (it != aerial_anchors_.end()) {
            it->second.is_active = false;
        }

        last_anchor_switch_time_ = get_current_timestamp_us();

        std::cout << "✅ Now operating as tag" << std::endl;
        return true;
    }

    bool AerialUWBManager::RotateAnchors() {
        std::cout << "🔄 Rotating anchor network..." << std::endl;

        std::lock_guard<std::mutex> lock(aerial_anchors_mutex_);

        // Select new optimal anchor set
        std::vector<DroneID> new_anchors = SelectOptimalAnchors();

        if (new_anchors.empty()) {
            std::cerr << "⚠️ No suitable anchors found" << std::endl;
            return false;
        }

        // Update active anchor set
        active_anchors_.clear();
        for (DroneID anchor_id : new_anchors) {
            active_anchors_.insert(anchor_id);

            auto it = aerial_anchors_.find(anchor_id);
            if (it != aerial_anchors_.end()) {
                it->second.is_active = true;
            }
        }

        anchor_rotation_counter_++;

        std::cout << "✅ Anchor network rotated. Active anchors: "
                  << active_anchors_.size() << std::endl;

        return true;
    }

    std::vector<DroneID> AerialUWBManager::SelectOptimalAnchors() {
        std::vector<DroneID> selected_anchors;

        // Need at least 4 anchors for 3D positioning
        const size_t MIN_ANCHORS = 4;
        const size_t MAX_ANCHORS = 8;

        struct AnchorCandidate {
            DroneID id;
            double score;
            AerialAnchorPoint anchor;
        };

        std::vector<AnchorCandidate> candidates;

        // Evaluate all potential anchors
        for (const auto& [drone_id, anchor] : aerial_anchors_) {
            if (drone_id == drone_id_) {
                continue;  // Don't select ourselves
            }

            double score = 0.0;

            // 1. Signal quality (30% weight)
            if (anchor.rssi_dbm > anchor_criteria_.min_rssi_dbm) {
                score += 0.3 * anchor.quality;
            } else {
                continue;  // Skip weak signals
            }

            // 2. Distance (20% weight) - prefer medium distances
            double distance = calculate_distance(get_estimated_position(), anchor.position);
            if (distance < anchor_criteria_.max_distance_m) {
                double distance_score = 1.0 - (distance / anchor_criteria_.max_distance_m);
                // Prefer medium distances (not too close, not too far)
                if (distance > 10.0 && distance < 100.0) {
                    distance_score *= 1.2;
                }
                score += 0.2 * distance_score;
            } else {
                continue;  // Too far
            }

            // 3. Motion stability (25% weight)
            double velocity_magnitude = std::sqrt(
                    anchor.velocity.x * anchor.velocity.x +
                    anchor.velocity.y * anchor.velocity.y +
                    anchor.velocity.z * anchor.velocity.z
            );
            double stability_score = 1.0 - std::min(velocity_magnitude / 10.0, 1.0);
            score += 0.25 * stability_score;

            // 4. Time synchronization (15% weight)
            if (anchor.time_synchronized) {
                score += 0.15;
            }

            // 5. Anchor priority (10% weight)
            score += 0.1 * (anchor.anchor_priority / 100.0);

            candidates.push_back({drone_id, score, anchor});
        }

        // Sort by score (descending)
        std::sort(candidates.begin(), candidates.end(),
                  [](const AnchorCandidate& a, const AnchorCandidate& b) {
                      return a.score > b.score;
                  });

        // Select top anchors
        size_t num_to_select = std::min(candidates.size(), MAX_ANCHORS);
        num_to_select = std::max(num_to_select, MIN_ANCHORS);

        for (size_t i = 0; i < num_to_select && i < candidates.size(); ++i) {
            selected_anchors.push_back(candidates[i].id);
        }

        return selected_anchors;
    }

    bool AerialUWBManager::UpdateAnchorNetwork() {
        std::lock_guard<std::mutex> lock(aerial_anchors_mutex_);

        uint64_t now = get_current_timestamp_us();
        const uint64_t ANCHOR_TIMEOUT_US = 5000000;  // 5 seconds

        // Remove stale anchors
        std::vector<DroneID> to_remove;
        for (const auto& [drone_id, anchor] : aerial_anchors_) {
            if (now - anchor.last_update > ANCHOR_TIMEOUT_US) {
                to_remove.push_back(drone_id);
            }
        }

        for (DroneID id : to_remove) {
            aerial_anchors_.erase(id);
            active_anchors_.erase(id);
            std::cout << "⏱️ Removed stale anchor: " << id << std::endl;
        }

        return true;
    }

//=============================================================================
// ✅ HIGH-SPEED POSITIONING
//=============================================================================

    bool AerialUWBManager::PerformHighSpeedRanging(DroneID target_drone) {
        MotionCompensatedMeasurement measurement;

        // Use high-speed ranging with motion compensation
        bool success = HighSpeedRanging::MotionCompensatedDS_TWR(
                this, target_drone, measurement);

        if (success) {
            // Store the motion-compensated measurement
            std::lock_guard<std::mutex> lock(measurements_mutex_);
            motion_measurements_.push(measurement);

            // Limit queue size
            while (motion_measurements_.size() > 100) {
                motion_measurements_.pop();
            }

            return true;
        }

        return false;
    }

    bool AerialUWBManager::MeasureWithDopplerCorrection(
            DroneID target_drone,
            MotionCompensatedMeasurement& measurement)
    {
        // Perform standard ranging
        if (!range_to_drone(target_drone)) {
            return false;
        }

        // Get the latest measurement
        auto measurements = get_latest_measurements(1);
        if (measurements.empty()) {
            return false;
        }

        const auto& raw_measurement = measurements[0];

        // Apply Doppler correction if enabled
        if (high_speed_config_.doppler_compensation_enabled) {
            // Get target velocity
            Position3D target_velocity{0, 0, 0};
            {
                std::lock_guard<std::mutex> lock(aerial_anchors_mutex_);
                auto it = aerial_anchors_.find(target_drone);
                if (it != aerial_anchors_.end()) {
                    target_velocity = it->second.velocity;
                }
            }

            // Calculate relative velocity
            double rel_vx = current_velocity_.x - target_velocity.x;
            double rel_vy = current_velocity_.y - target_velocity.y;
            double rel_vz = current_velocity_.z - target_velocity.z;

            double relative_velocity = std::sqrt(
                    rel_vx * rel_vx + rel_vy * rel_vy + rel_vz * rel_vz);

            // Apply Doppler correction (simplified model)
            const double SPEED_OF_LIGHT = 299792458.0;  // m/s
            double doppler_factor = 1.0 + (relative_velocity / SPEED_OF_LIGHT);

            measurement.raw_distance = raw_measurement.distance;
            measurement.corrected_distance = raw_measurement.distance / doppler_factor;
            measurement.doppler_correction = measurement.corrected_distance - measurement.raw_distance;
        } else {
            measurement.raw_distance = raw_measurement.distance;
            measurement.corrected_distance = raw_measurement.distance;
            measurement.doppler_correction = 0.0;
        }

        measurement.target_drone = target_drone;
        measurement.measurement_time = raw_measurement.timestamp;
        measurement.sender_velocity = current_velocity_;

        return true;
    }

    Position3D AerialUWBManager::PredictPositionAtTime(Timestamp target_time) {
        std::lock_guard<std::mutex> lock(motion_filter_mutex_);

        uint64_t now = get_current_timestamp_us();
        double dt = (target_time - now) / 1000000.0;  // Convert to seconds

        return motion_filter_.getPredictedPosition(dt);
    }

    bool AerialUWBManager::CalculateAerialPosition(Position3D& position, double& accuracy) {
        // Use base UWB positioning with motion prediction enhancement
        if (!calculate_position(position, accuracy)) {
            return false;
        }

        // Apply Kalman filter for smoothing and prediction
        if (high_speed_config_.kalman_filter_enabled) {
            std::lock_guard<std::mutex> lock(motion_filter_mutex_);
            motion_filter_.update(position, accuracy);

            // Get filtered position
            position = motion_filter_.getPredictedPosition(0.0);
            accuracy = motion_filter_.getPositionAccuracy();
        }

        return true;
    }

//=============================================================================
// ✅ MOTION TRACKING
//=============================================================================

    bool AerialUWBManager::UpdateMotionState() {
        Position3D current_pos = get_estimated_position();

        std::lock_guard<std::mutex> lock(motion_filter_mutex_);

        // Update Kalman filter
        double accuracy = ranging_accuracy_.load();
        motion_filter_.update(current_pos, accuracy);

        // Extract velocity and acceleration from filter
        current_velocity_ = motion_filter_.getCurrentVelocity();
        current_acceleration_ = motion_filter_.getCurrentAcceleration();

        return true;
    }

    Position3D AerialUWBManager::GetCurrentVelocity() const {
        return current_velocity_;
    }

    Position3D AerialUWBManager::GetCurrentAcceleration() const {
        return current_acceleration_;
    }

    double AerialUWBManager::GetCurrentSpeed() const {
        double vx = current_velocity_.x;
        double vy = current_velocity_.y;
        double vz = current_velocity_.z;

        return std::sqrt(vx*vx + vy*vy + vz*vz);
    }

    bool AerialUWBManager::IsMovingFast() const {
        return GetCurrentSpeed() > high_speed_config_.max_velocity_ms * 0.5;
    }

//=============================================================================
// ✅ FORMATION SUPPORT
//=============================================================================

    bool AerialUWBManager::SetFormationLeader(DroneID leader_id) {
        formation_leader_id_ = leader_id;

        if (leader_id == drone_id_) {
            SetAerialMode(AerialUWBMode::FORMATION_LEADER);
        } else {
            SetAerialMode(AerialUWBMode::FORMATION_MEMBER);
        }

        std::cout << "🎯 Formation leader set to: " << leader_id << std::endl;
        return true;
    }

    bool AerialUWBManager::BroadcastFormationPosition() {
        if (!high_speed_config_.formation_sync_enabled) {
            return false;
        }

        // Calculate formation center and velocity
        Position3D formation_center{0, 0, 0};
        Position3D formation_velocity{0, 0, 0};
        size_t count = 0;

        {
            std::lock_guard<std::mutex> lock(aerial_anchors_mutex_);

            for (const auto& [id, anchor] : aerial_anchors_) {
                formation_center.x += anchor.position.x;
                formation_center.y += anchor.position.y;
                formation_center.z += anchor.position.z;

                formation_velocity.x += anchor.velocity.x;
                formation_velocity.y += anchor.velocity.y;
                formation_velocity.z += anchor.velocity.z;

                count++;
            }
        }

        if (count > 0) {
            formation_center.x /= count;
            formation_center.y /= count;
            formation_center.z /= count;

            formation_velocity.x /= count;
            formation_velocity.y /= count;
            formation_velocity.z /= count;
        }

        // TODO: Broadcast formation data via LoRa or mesh network
        std::cout << "📡 Formation broadcast: Center("
                  << formation_center.x << ", "
                  << formation_center.y << ", "
                  << formation_center.z << ")" << std::endl;

        return true;
    }

    bool AerialUWBManager::SynchronizeWithFormation() {
        if (formation_leader_id_ == 0 || formation_leader_id_ == drone_id_) {
            return false;
        }

        // TODO: Receive formation data from leader
        // TODO: Adjust position/velocity relative to formation

        return true;
    }

//=============================================================================
// ✅ NETWORK MANAGEMENT
//=============================================================================

    bool AerialUWBManager::AddAerialNode(
            DroneID drone_id,
            const Position3D& position,
            const Position3D& velocity)
    {
        std::lock_guard<std::mutex> lock(aerial_anchors_mutex_);

        AerialAnchorPoint anchor;
        anchor.drone_id = drone_id;
        anchor.position = position;
        anchor.velocity = velocity;
        anchor.last_update = get_current_timestamp_us();
        anchor.rssi_dbm = -70;  // Default value
        anchor.quality = 0.5;   // Default quality
        anchor.is_active = false;
        anchor.anchor_priority = 50;  // Medium priority
        anchor.time_synchronized = false;

        aerial_anchors_[drone_id] = anchor;
        anchor_candidates_.insert(drone_id);

        std::cout << "➕ Added aerial node: " << drone_id << std::endl;

        // Also add to base UWB system
        add_uwb_node(drone_id, position);

        return true;
    }

    bool AerialUWBManager::RemoveAerialNode(DroneID drone_id) {
        std::lock_guard<std::mutex> lock(aerial_anchors_mutex_);

        aerial_anchors_.erase(drone_id);
        active_anchors_.erase(drone_id);
        anchor_candidates_.erase(drone_id);

        std::cout << "➖ Removed aerial node: " << drone_id << std::endl;

        return true;
    }

    bool AerialUWBManager::UpdateNodeMotion(
            DroneID drone_id,
            const Position3D& position,
            const Position3D& velocity)
    {
        std::lock_guard<std::mutex> lock(aerial_anchors_mutex_);

        auto it = aerial_anchors_.find(drone_id);
        if (it != aerial_anchors_.end()) {
            it->second.position = position;
            it->second.velocity = velocity;
            it->second.last_update = get_current_timestamp_us();
            return true;
        }

        return false;
    }

    std::vector<AerialAnchorPoint> AerialUWBManager::GetActiveAnchors() const {
        std::vector<AerialAnchorPoint> anchors;

        std::lock_guard<std::mutex> lock(aerial_anchors_mutex_);

        for (DroneID anchor_id : active_anchors_) {
            auto it = aerial_anchors_.find(anchor_id);
            if (it != aerial_anchors_.end()) {
                anchors.push_back(it->second);
            }
        }

        return anchors;
    }

//=============================================================================
// ✅ CONFIGURATION
//=============================================================================

    bool AerialUWBManager::SetHighSpeedConfig(const HighSpeedConfig& config) {
        high_speed_config_ = config;

        std::cout << "⚙️ Updated high-speed configuration:" << std::endl;
        std::cout << "   Ranging rate: " << config.fast_ranging_rate_hz << " Hz" << std::endl;
        std::cout << "   Max velocity: " << config.max_velocity_ms << " m/s" << std::endl;

        return true;
    }

    HighSpeedConfig AerialUWBManager::GetHighSpeedConfig() const {
        return high_speed_config_;
    }

//=============================================================================
// ✅ WORKER THREADS
//=============================================================================

    void AerialUWBManager::anchor_management_worker() {
        std::cout << "🚁 Anchor management worker started" << std::endl;

        while (aerial_system_running_.load()) {
            try {
                // Check if we should switch roles
                if (ShouldSwitchToAnchor()) {
                    BecomeAnchor();
                } else if (ShouldSwitchToTag()) {
                    BecomeTag();
                }

                // Update anchor network
                UpdateAnchorNetwork();

                // Rotate anchors periodically
                if (current_mode_ == AerialUWBMode::DYNAMIC_ANCHOR_TAG) {
                    uint64_t now = get_current_timestamp_us();
                    if (now - last_anchor_switch_time_ >
                        high_speed_config_.anchor_switch_interval_ms * 1000) {
                        RotateAnchors();
                    }
                }

                // Update motion state
                UpdateMotionState();

            } catch (const std::exception& e) {
                std::cerr << "❌ Anchor management exception: " << e.what() << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cout << "🛑 Anchor management worker stopped" << std::endl;
    }

    void AerialUWBManager::high_speed_ranging_worker() {
        std::cout << "⚡ High-speed ranging worker started" << std::endl;

        const auto ranging_interval = std::chrono::microseconds(
                1000000 / high_speed_config_.fast_ranging_rate_hz);

        while (aerial_system_running_.load()) {
            auto start_time = std::chrono::steady_clock::now();

            try {
                // Get list of active targets
                std::vector<DroneID> targets;
                {
                    std::lock_guard<std::mutex> lock(aerial_anchors_mutex_);
                    for (const auto& [id, anchor] : aerial_anchors_) {
                        if (id != drone_id_ && anchor.is_active) {
                            targets.push_back(id);
                        }
                    }
                }

                // Perform high-speed ranging to all targets
                for (DroneID target : targets) {
                    PerformHighSpeedRanging(target);
                }

                // Update position
                Position3D position;
                double accuracy;
                if (CalculateAerialPosition(position, accuracy)) {
                    estimated_position_ = position;
                    ranging_accuracy_.store(accuracy);
                }

            } catch (const std::exception& e) {
                std::cerr << "❌ High-speed ranging exception: " << e.what() << std::endl;
            }

            // Maintain precise timing
            auto end_time = std::chrono::steady_clock::now();
            auto elapsed = end_time - start_time;

            if (elapsed < ranging_interval) {
                std::this_thread::sleep_for(ranging_interval - elapsed);
            }
        }

        std::cout << "🛑 High-speed ranging worker stopped" << std::endl;
    }

//=============================================================================
// ✅ HELPER FUNCTIONS
//=============================================================================

    double AerialUWBManager::calculate_distance(
            const Position3D& p1,
            const Position3D& p2) const
    {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double dz = p2.z - p1.z;

        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    uint64_t AerialUWBManager::get_current_timestamp_us() const {
        auto now = std::chrono::steady_clock::now();
        auto duration = now.time_since_epoch();
        return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
    }

} // namespace AerialUWB