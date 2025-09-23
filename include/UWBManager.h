// include/UWBManager.h
// Заголовочний файл для UWB менеджера з криптографією та синхронізацією
// Production система позиціонування для дронів-камікадзе
// 🇺🇦 Slava Ukraini! Death to moscow occupants! 🇺🇦

#pragma once

#include "SwarmTypes.h"
#include "UWBCryptoSync.h"
#include <memory>
#include <vector>
#include <atomic>
#include <mutex>
#include <thread>
#include <deque>
#include <functional>
#include <chrono>

namespace SwarmControl {

// Forward declarations
    class UWBCrypto;
    class UWBTimeSync;
    using DroneID = uint16_t;
    using Timestamp = std::chrono::steady_clock::time_point;

// UWB Configuration
    struct UWBConfig {
        uint8_t channel;            // UWB channel (1-7, except 6)
        uint8_t preamble_code;      // Preamble code (1-24)
        uint8_t data_rate;          // 0=110k, 1=850k, 2=6.8M
        uint16_t preamble_length;   // Preamble length
        uint8_t pac_size;           // Preamble acquisition chunk size
        uint8_t tx_power;           // TX power (0-33)
        bool smart_power;           // Smart power control
        uint16_t antenna_delay_tx;  // TX antenna delay
        uint16_t antenna_delay_rx;  // RX antenna delay

        UWBConfig() {
            channel = 5;
            preamble_code = 9;
            data_rate = 2;
            preamble_length = 128;
            pac_size = 8;
            tx_power = 33;            // Maximum power for combat range
            smart_power = true;
            antenna_delay_tx = 16456;
            antenna_delay_rx = 16456;
        }
    };

// 3D Position
    struct Position3D {
        double x, y, z;

        Position3D() : x(0.0), y(0.0), z(0.0) {}
        Position3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

        double distance_to(const Position3D& other) const {
            double dx = x - other.x;
            double dy = y - other.y;
            double dz = z - other.z;
            return std::sqrt(dx*dx + dy*dy + dz*dz);
        }
    };

// UWB Ranging modes
    enum class UWBRangingMode {
        TWR,        // Two-Way Ranging
        DS_TWR,     // Double-Sided Two-Way Ranging
        SS_TWR,     // Single-Sided Two-Way Ranging
        TDOA        // Time Difference of Arrival
    };

// UWB measurement structure
    struct UWBMeasurement {
        DroneID target_drone;
        double distance;
        double accuracy;
        double signal_power;
        uint64_t timestamp_ns;
        Timestamp timestamp;
        bool valid;

        UWBMeasurement() {
            target_drone = 0;
            distance = 0.0;
            accuracy = 999.0;
            signal_power = -100.0;
            timestamp_ns = 0;
            timestamp = std::chrono::steady_clock::now();
            valid = false;
        }
    };

// UWB node information
    struct UWBNode {
        DroneID drone_id;
        Position3D position;
        double last_distance;
        double position_accuracy;
        Timestamp last_measurement;
        Timestamp last_position_update;
        bool position_valid;
        bool is_anchor;
        double signal_strength;

        UWBNode() {
            drone_id = 0;
            last_distance = 0.0;
            position_accuracy = 999.0;
            last_measurement = std::chrono::steady_clock::now();
            last_position_update = last_measurement;
            position_valid = false;
            is_anchor = false;
            signal_strength = -100.0;
        }
    };

// Trilateration result
    struct TrilaterationResult {
        Position3D position;
        double accuracy;
        double confidence;
        bool valid;

        TrilaterationResult() {
            accuracy = 999.0;
            confidence = 0.0;
            valid = false;
        }
    };

// Network statistics
    struct UWBNetworkStats {
        uint32_t packets_sent;
        uint32_t packets_received;
        uint32_t packets_lost;
        uint32_t ranging_requests;
        uint32_t ranging_responses;
        uint32_t successful_ranges;
        double average_accuracy;
        double network_coverage;
        Timestamp last_full_update;

        UWBNetworkStats() {
            memset(this, 0, sizeof(UWBNetworkStats));
            last_full_update = std::chrono::steady_clock::now();
        }
    };

// Main UWB Manager class
    class UWBManager {
    private:
        // Core components
        DroneID drone_id_;
        std::string config_path_;
        uint16_t uwb_address_;

        // Hardware interface
        class DW1000Interface;
        std::unique_ptr<DW1000Interface> uwb_hardware_;

        // Crypto and sync components
        std::unique_ptr<UWBCrypto> crypto_engine_;
        std::unique_ptr<UWBTimeSync> time_sync_;

        // Configuration
        UWBConfig current_config_;
        mutable std::mutex config_mutex_;

        // State management
        std::atomic<bool> running_;
        std::atomic<bool> is_master_clock_;
        std::atomic<double> clock_offset_;
        std::atomic<bool> network_synchronized_;

        // Positioning
        UWBRangingMode ranging_mode_;
        Position3D estimated_position_;
        std::atomic<double> ranging_accuracy_;
        std::atomic<double> measurement_rate_;

        // Network management
        std::map<DroneID, UWBNode> uwb_nodes_;
        std::mutex nodes_mutex_;

        // Measurement storage
        std::deque<UWBMeasurement> measurement_buffer_;
        mutable std::mutex measurements_mutex_;
        static constexpr size_t MAX_MEASUREMENT_HISTORY = 1000;

        // Statistics
        UWBNetworkStats network_stats_;

        // Worker threads
        std::thread ranging_thread_;
        std::thread position_thread_;
        std::thread sync_thread_;

        // Timeouts and intervals
        static constexpr auto NODE_TIMEOUT = std::chrono::seconds(5);
        static constexpr auto RANGING_INTERVAL = std::chrono::milliseconds(100);
        static constexpr auto POSITION_CALC_INTERVAL = std::chrono::milliseconds(50);

    public:
        // Constructor/Destructor
        explicit UWBManager(DroneID drone_id, const std::string& config_path = "");
        ~UWBManager();

        // Core lifecycle
        bool initialize();
        bool start();
        bool stop();
        bool is_running() const { return running_.load(); }

        // Crypto and time sync initialization
        bool initialize_crypto(const std::string& shared_secret);
        bool initialize_time_sync(bool is_master = false);

        // Ranging operations
        bool range_to_drone(DroneID target_drone, double& distance, double& accuracy);
        bool broadcast_range_request();
        std::vector<UWBMeasurement> get_latest_measurements(size_t count = 10) const;
        bool start_continuous_ranging();
        bool stop_continuous_ranging();

        // Position calculation
        bool calculate_position(Position3D& position, double& accuracy);
        bool update_position_from_measurements();
        Position3D get_estimated_position() const { return estimated_position_; }
        double get_position_accuracy() const { return ranging_accuracy_.load(); }

        // Network management
        bool add_uwb_node(DroneID drone_id, const Position3D& initial_position = Position3D());
        bool remove_uwb_node(DroneID drone_id);
        std::vector<DroneID> get_detected_nodes() const;
        std::map<DroneID, UWBNode> get_all_nodes() const;
        bool update_node_position(DroneID drone_id, const Position3D& position);

        // Network synchronization
        bool is_network_synchronized() const;
        double get_clock_offset() const;
        bool set_master_clock(bool is_master);
        bool synchronize_with_network();

        // Configuration
        bool set_uwb_config(const UWBConfig& config);
        UWBConfig get_uwb_config() const;
        bool reload_configuration();

        // Calibration
        bool calibrate_ranging();
        bool calibrate_antenna_delays();
        bool save_calibration_data();
        bool load_calibration_data();

        // Monitoring and diagnostics
        UWBNetworkStats get_network_statistics() const;
        double get_measurement_rate() const { return measurement_rate_.load(); }
        bool perform_self_test();
        std::string get_status_report() const;

        // Data export and logging
        bool export_measurements(const std::string& filename) const;
        bool log_uwb_data() const;

        // Encrypted communication
        bool send_encrypted_packet(DroneID target_drone, const uint8_t* data, size_t data_len);
        bool receive_encrypted_packet(uint8_t* data, size_t& data_len, DroneID& sender_drone);

        // Power management - hot reload support
        bool SetTxPower(uint8_t power_level);  // 0-33 range
        uint8_t GetTxPower() const;
        bool SetRxGain(uint8_t gain_level);
        uint8_t GetRxGain() const;

// Update rate control - hot reload support
        bool SetUpdateRate(uint16_t rate_hz);  // 1-1000 Hz
        uint16_t GetUpdateRate() const;
        bool SetMeasurementInterval(uint32_t interval_ms);
        uint32_t GetMeasurementInterval() const;

// Ranging configuration - hot reload support
        bool SetRangingMode(UWBRangingMode mode);
        UWBRangingMode GetRangingMode() const;
        bool SetMaxRangingDistance(double max_distance_m);
        double GetMaxRangingDistance() const;

// Accuracy and filtering - hot reload support
        bool SetPositionAccuracyThreshold(double threshold_m);
        double GetPositionAccuracyThreshold() const;
        bool EnableOutlierRejection(bool enable);
        bool IsOutlierRejectionEnabled() const;

// Channel and timing - hot reload support
        bool SetChannel(uint8_t channel);  // 1-7 except 6
        uint8_t GetChannel() const;
        bool SetPreambleLength(uint16_t length);
        uint16_t GetPreambleLength() const;

// Antenna configuration - hot reload support
        bool SetAntennaDelay(uint16_t tx_delay, uint16_t rx_delay);
        bool GetAntennaDelay(uint16_t& tx_delay, uint16_t& rx_delay) const;

// Calibration and diagnostics - hot reload support
        bool TriggerSelfCalibration();
        bool RunDiagnostics();
        bool ResetToDefaults();

    private:
        // Internal worker methods
        void ranging_worker();
        void position_calculation_worker();
        void time_sync_worker();

        // Trilateration algorithms
        bool calculate_trilateration_2d(const std::vector<std::pair<DroneID, double>>& measurements,
                                        TrilaterationResult& result);
        bool calculate_trilateration_3d(const std::vector<std::pair<DroneID, double>>& measurements,
                                        TrilaterationResult& result);
        bool calculate_trilateration_3d_anchors(const std::vector<Position3D>& anchor_positions,
                                                const std::vector<double>& distances,
                                                TrilaterationResult& result);
        bool calculate_trilateration_2d_from_anchors(const std::vector<Position3D>& anchor_positions,
                                                     const std::vector<double>& distances,
                                                     TrilaterationResult& result);

        // Advanced algorithms
        Position3D calculate_bancroft_solution(const std::vector<Position3D>& anchors,
                                               const std::vector<double>& distances);
        double calculate_gdop(const std::vector<Position3D>& anchors, const Position3D& position);

        // Utility methods
        double calculate_distance_3d(const Position3D& pos1, const Position3D& pos2) const;
        double calculate_measurement_quality(uint64_t round_trip_time, uint64_t response_time) const;
        double calculate_measurement_quality_for_node(DroneID drone_id) const;
        bool store_measurement(const UWBMeasurement& measurement);
        bool load_uwb_configuration();
        uint64_t get_synchronized_timestamp();
        bool clean_stale_measurements();
        bool validate_node_measurements();
        bool update_network_statistics();
        bool synchronize_network_time();
    };

} // namespace SwarmControl