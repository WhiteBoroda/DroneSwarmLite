#pragma once
#include "UWBManager.h"
#include <unordered_set>
#include <queue>

namespace AerialUWB {

// Enhanced UWB modes for aerial operations
    enum class AerialUWBMode {
        FULL_ANCHOR = 0,        // Pure anchor - stationary reference
        FULL_TAG,               // Pure tag - only measures, doesn't provide reference
        DYNAMIC_ANCHOR_TAG,     // Switches between anchor and tag (DEFAULT for drones)
        MOBILE_ANCHOR,          // Mobile anchor - provides reference while moving
        FORMATION_LEADER,       // Primary reference for formation
        FORMATION_FOLLOWER      // Follows formation leader
    };

// High-speed positioning configuration
    struct HighSpeedConfig {
        double max_velocity_ms = 27.78;         // 100 km/h in m/s
        double prediction_window_ms = 100;      // Position prediction window
        double doppler_compensation_hz = 1000;  // Max Doppler shift compensation
        uint16_t fast_ranging_rate_hz = 200;    // Fast ranging for moving targets
        double motion_threshold_ms = 5.0;       // Motion detection threshold
        bool enable_kalman_filter = true;       // Enable motion prediction
        bool enable_doppler_correction = true;  // Enable Doppler shift correction
    };

// Dynamic anchor selection criteria
    struct AnchorSelectionCriteria {
        double min_stability_score = 0.7;       // Minimum stability to be anchor
        double min_position_accuracy = 2.0;     // Minimum accuracy in meters
        double max_velocity_for_anchor = 5.0;   // Max velocity to serve as anchor (m/s)
        uint32_t min_anchor_time_ms = 10000;    // Minimum time as anchor
        uint32_t anchor_rotation_interval_ms = 60000; // Anchor rotation interval
        uint8_t min_anchors_in_network = 3;     // Minimum anchors needed
        uint8_t max_anchors_in_network = 6;     // Maximum anchors allowed
    };

// Motion-compensated measurement
    struct MotionCompensatedMeasurement {
        DroneID target_drone;
        double distance_m;
        double relative_velocity_ms;        // Radial velocity towards/away
        double doppler_shift_hz;           // Measured Doppler shift
        Position3D predicted_position;      // Predicted position at measurement time
        Timestamp measurement_time;
        double accuracy_m;
        bool doppler_corrected;

        MotionCompensatedMeasurement() {
            target_drone = 0;
            distance_m = 0.0;
            relative_velocity_ms = 0.0;
            doppler_shift_hz = 0.0;
            accuracy_m = 999.0;
            measurement_time = std::chrono::steady_clock::now();
            doppler_corrected = false;
        }
    };

// Aerial formation anchor point
    struct AerialAnchorPoint {
        DroneID anchor_drone_id;
        Position3D position;
        Position3D velocity;               // Current velocity vector
        double stability_score;            // How stable this anchor is (0.0-1.0)
        double position_accuracy;          // Position accuracy in meters
        AerialUWBMode current_mode;        // Current operating mode
        Timestamp last_position_update;
        Timestamp anchor_start_time;       // When this drone became anchor
        bool is_formation_leader;
        uint32_t anchor_priority;          // Priority for anchor selection

        AerialAnchorPoint() {
            anchor_drone_id = 0;
            stability_score = 0.0;
            position_accuracy = 999.0;
            current_mode = AerialUWBMode::DYNAMIC_ANCHOR_TAG;
            last_position_update = std::chrono::steady_clock::now();
            anchor_start_time = last_position_update;
            is_formation_leader = false;
            anchor_priority = 0;
        }
    };

// High-speed Kalman filter for position prediction
    class HighSpeedKalmanFilter {
    private:
        // State vector: [x, y, z, vx, vy, vz, ax, ay, az]
        static const int STATE_SIZE = 9;
        static const int MEASUREMENT_SIZE = 3;

        double state[STATE_SIZE];           // Position, velocity, acceleration
        double covariance[STATE_SIZE][STATE_SIZE]; // State covariance matrix
        double process_noise_q[STATE_SIZE]; // Process noise
        double measurement_noise_r[MEASUREMENT_SIZE]; // Measurement noise
        Timestamp last_update;
        bool initialized;

    public:
        HighSpeedKalmanFilter();

        void initialize(const Position3D& initial_position, const Position3D& initial_velocity);
        void predict(double dt_seconds);
        void update(const Position3D& measured_position, double measurement_accuracy);
        Position3D getPredictedPosition(double future_dt_seconds) const;
        Position3D getCurrentVelocity() const;
        Position3D getCurrentAcceleration() const;
        double getPositionAccuracy() const;
        void reset();
    };

//=============================================================================
// ✅ AERIAL UWB MANAGER - ENHANCED FOR HIGH-SPEED OPERATIONS
//=============================================================================

    class AerialUWBManager : public UWBManager {
    private:
        // Aerial-specific configuration
        AerialUWBMode current_mode_;
        HighSpeedConfig high_speed_config_;
        AnchorSelectionCriteria anchor_criteria_;

        // Dynamic anchor management
        std::unordered_map<DroneID, AerialAnchorPoint> aerial_anchors_;
        std::unordered_set<DroneID> active_anchors_;
        std::unordered_set<DroneID> anchor_candidates_;
        DroneID formation_leader_id_;

        // Motion tracking and prediction
        HighSpeedKalmanFilter motion_filter_;
        std::queue<MotionCompensatedMeasurement> motion_measurements_;
        Position3D current_velocity_;
        Position3D current_acceleration_;

        // Timing and synchronization
        uint64_t last_anchor_switch_time_;
        uint64_t last_ranging_cycle_time_;
        uint32_t anchor_rotation_counter_;

        // Thread management
        std::unique_ptr<std::thread> anchor_management_thread_;
        std::unique_ptr<std::thread> high_speed_ranging_thread_;
        std::atomic<bool> aerial_system_running_;

        // Mutexes for thread safety
        mutable std::mutex aerial_anchors_mutex_;
        mutable std::mutex motion_filter_mutex_;
        mutable std::mutex measurements_mutex_;

    public:
        AerialUWBManager(DroneID drone_id);
        ~AerialUWBManager();

        // Initialization and control
        bool InitializeAerialSystem(const HighSpeedConfig& config);
        bool StartAerialPositioning();
        void StopAerialPositioning();

        // Mode management
        bool SetAerialMode(AerialUWBMode mode);
        AerialUWBMode GetCurrentMode() const { return current_mode_; }
        bool CanServeAsAnchor() const;
        bool ShouldSwitchToAnchor() const;
        bool ShouldSwitchToTag() const;

        // Dynamic anchor management
        bool BecomeAnchor(uint32_t anchor_priority = 0);
        bool BecomeTag();
        bool RotateAnchors();
        std::vector<DroneID> SelectOptimalAnchors();
        bool UpdateAnchorNetwork();

        // High-speed positioning
        bool PerformHighSpeedRanging(DroneID target_drone);
        bool MeasureWithDopplerCorrection(DroneID target_drone, MotionCompensatedMeasurement& measurement);
        Position3D PredictPositionAtTime(Timestamp target_time);
        bool CalculateAerialPosition(Position3D& position, double& accuracy);

        // Motion tracking
        bool UpdateMotionState();
        Position3D GetCurrentVelocity() const;
        Position3D GetCurrentAcceleration() const;
        double GetCurrentSpeed() const;
        bool IsMovingFast() const;

        // Formation support
        bool SetFormationLeader(DroneID leader_id);
        DroneID GetFormationLeader() const { return formation_leader_id_; }
        bool BroadcastFormationPosition();
        bool SynchronizeWithFormation();

        // Network management
        bool AddAerialNode(DroneID drone_id, const Position3D& position, const Position3D& velocity);
        bool RemoveAerialNode(DroneID drone_id);
        bool UpdateNodeMotion(DroneID drone_id, const Position3D& position, const Position3D& velocity);
        std::vector<AerialAnchorPoint> GetActiveAnchors() const;

        // Configuration
        bool SetHighSpeedConfig(const HighSpeedConfig& config);
        HighSpeedConfig GetHighSpeedConfig() const { return high_speed_config_; }
        bool SetAnchorCriteria(const AnchorSelectionCriteria& criteria);

        // Statistics and monitoring
        double GetAerialPositioningAccuracy() const;
        double GetRangingSuccessRate() const;
        uint32_t GetActiveAnchorCount() const;
        std::string GetAerialStatusReport() const;

    private:
        // Internal methods
        void anchorManagementLoop();
        void highSpeedRangingLoop();

        double calculateStabilityScore(DroneID drone_id) const;
        double calculateAnchorPriority(DroneID drone_id) const;
        bool isValidAnchorCandidate(DroneID drone_id) const;

        bool performDopplerCorrection(MotionCompensatedMeasurement& measurement);
        double calculateDopplerShift(const Position3D& relative_velocity) const;
        bool compensateForMotion(MotionCompensatedMeasurement& measurement);

        bool broadcastAnchorStatus();
        bool sendAnchorSwitchRequest(DroneID target_drone, AerialUWBMode requested_mode);
        bool handleAnchorSwitchRequest(DroneID requester_drone, AerialUWBMode requested_mode);

        void logAerialEvent(const std::string& event, const std::string& details = "") const;
    };

//=============================================================================
// ✅ HIGH-SPEED RANGING PROTOCOLS
//=============================================================================

// Fast ranging frame for high-speed operations
    struct FastRangingFrame {
        uint8_t frame_type;                 // Frame type identifier
        DroneID sender_id;                  // Sender drone ID
        DroneID target_id;                  // Target drone ID
        uint64_t timestamp_tx;              // Transmission timestamp
        uint64_t timestamp_rx;              // Reception timestamp (for response)
        Position3D sender_position;         // Sender position
        Position3D sender_velocity;         // Sender velocity
        uint16_t sequence_number;           // Sequence number
        uint8_t ranging_mode;               // Ranging mode (SS-TWR, DS-TWR, etc.)
        uint16_t expected_response_delay;   // Expected response delay in microseconds
        uint16_t crc;                       // CRC checksum

        FastRangingFrame() {
            frame_type = 0x42;  // Fast ranging frame type
            sender_id = 0;
            target_id = 0;
            timestamp_tx = 0;
            timestamp_rx = 0;
            sequence_number = 0;
            ranging_mode = 2;   // DS-TWR by default
            expected_response_delay = 1000; // 1ms default
            crc = 0;
        }
    } __attribute__((packed));

// High-speed ranging algorithms
    class HighSpeedRanging {
    public:
        // Fast Single-Sided TWR for high-speed targets
        static bool FastSS_TWR(AerialUWBManager* uwb_manager, DroneID target_drone,
                               MotionCompensatedMeasurement& result);

        // Motion-compensated Double-Sided TWR
        static bool MotionCompensatedDS_TWR(AerialUWBManager* uwb_manager, DroneID target_drone,
                                            MotionCompensatedMeasurement& result);

        // Parallel ranging for multiple targets
        static bool ParallelRanging(AerialUWBManager* uwb_manager,
                                    const std::vector<DroneID>& target_drones,
                                    std::vector<MotionCompensatedMeasurement>& results);

        // Formation-optimized ranging
        static bool FormationRanging(AerialUWBManager* uwb_manager,
                                     const std::vector<DroneID>& formation_drones,
                                     std::vector<MotionCompensatedMeasurement>& results);

    private:
        static double calculateClockDriftCorrection(uint64_t tx_time, uint64_t rx_time);
        static bool compensateForRelativeMotion(const Position3D& sender_pos,
                                                const Position3D& sender_vel,
                                                const Position3D& target_pos,
                                                const Position3D& target_vel,
                                                double& distance_correction);
    };

//=============================================================================
// ✅ AERIAL COORDINATE SYSTEM
//=============================================================================

// 3D aerial coordinate system without ground references
    class AerialCoordinateSystem {
    private:
        Position3D formation_center_;        // Formation geometric center
        Position3D formation_velocity_;      // Formation average velocity
        DroneID primary_anchor_;            // Primary reference anchor
        std::vector<DroneID> reference_anchors_; // Additional reference anchors
        double coordinate_system_accuracy_; // System accuracy estimate
        Timestamp last_system_update_;

    public:
        AerialCoordinateSystem();

        // Coordinate system management
        bool EstablishAerialCoordinateSystem(const std::vector<AerialAnchorPoint>& anchors);
        bool UpdateCoordinateSystem(const std::vector<MotionCompensatedMeasurement>& measurements);
        bool ValidateCoordinateSystem() const;

        // Coordinate transformations
        Position3D TransformToGlobal(const Position3D& local_position) const;
        Position3D TransformToLocal(const Position3D& global_position) const;
        bool GetRelativePosition(DroneID from_drone, DroneID to_drone, Position3D& relative_pos) const;

        // Formation tracking
        Position3D GetFormationCenter() const { return formation_center_; }
        Position3D GetFormationVelocity() const { return formation_velocity_; }
        double GetSystemAccuracy() const { return coordinate_system_accuracy_; }

        // Anchor management
        bool SetPrimaryAnchor(DroneID anchor_id);
        DroneID GetPrimaryAnchor() const { return primary_anchor_; }
        std::vector<DroneID> GetReferenceAnchors() const { return reference_anchors_; }
    };

} // namespace AerialUWB