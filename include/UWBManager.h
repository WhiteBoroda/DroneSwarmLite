#pragma once

#include "SwarmTypes.h"
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <unordered_map>

namespace SwarmControl {

// UWB ranging methods
    enum class UWBRangingMode : uint8_t {
        TWR,        // Two-Way Ranging
        DS_TWR,     // Double-Sided Two-Way Ranging
        SDS_TWR,    // Symmetric Double-Sided Two-Way Ranging
        TDoA        // Time Difference of Arrival
    };

// UWB configuration parameters
    struct UWBConfig {
        uint8_t channel;           // UWB channel (1, 2, 3, 4, 5, 7)
        uint8_t preamble_code;     // Preamble code (1-24)
        uint8_t data_rate;         // Data rate (110k, 850k, 6.8M)
        uint16_t preamble_length;  // Preamble length
        uint8_t pac_size;          // Preamble acquisition chunk size
        bool smart_power;          // Smart transmit power control
        uint8_t tx_power;          // Transmit power setting

        UWBConfig() : channel(5), preamble_code(9), data_rate(2),
                      preamble_length(128), pac_size(8), smart_power(true), tx_power(15) {}
    };

// UWB anchor information (for absolute positioning if available)
    struct UWBAnchor {
        uint16_t anchor_id;
        Position3D known_position;
        bool active;
        double clock_offset;
        Timestamp last_seen;
        int8_t signal_strength;
    };

// UWB node information for other drones
    struct UWBNode {
        DroneID drone_id;
        uint16_t uwb_address;
        Position3D estimated_position;
        bool position_valid;
        Timestamp last_measurement;
        double measurement_quality;
        std::deque<UWBMeasurement> measurement_history;
    };

// Trilateration result
    struct TrilaterationResult {
        Position3D position;
        double accuracy;         // meters - estimated accuracy
        double confidence;       // 0-1, confidence in result
        size_t measurements_used;
        bool valid;

        TrilaterationResult() : accuracy(999.0), confidence(0.0), measurements_used(0), valid(false) {}
    };

// UWB network statistics
    struct UWBNetworkStats {
        size_t total_nodes;
        size_t active_nodes;
        size_t total_measurements;
        double average_range_accuracy;
        double measurement_rate;    // Hz
        uint32_t failed_measurements;
        double network_sync_quality;
        Timestamp last_full_update;
    };

    class UWBManager {
    public:
        explicit UWBManager(DroneID drone_id, const std::string& config_path);
        ~UWBManager();

        // Core lifecycle
        bool initialize();
        bool start();
        void stop();
        void reset();

        // Network management
        bool join_uwb_network();
        bool leave_uwb_network();
        bool discover_nodes();
        std::vector<UWBNode> get_visible_nodes() const;
        bool add_node(DroneID drone_id, uint16_t uwb_address);
        bool remove_node(DroneID drone_id);

        // Ranging operations
        bool range_to_node(DroneID target_drone);
        bool range_to_all_nodes();
        std::vector<UWBMeasurement> get_latest_measurements() const;
        UWBMeasurement get_measurement_to_drone(DroneID drone_id) const;

        // Position calculation
        bool update_relative_positions();
        Position3D get_relative_position(DroneID reference_drone) const;
        Position3D get_estimated_position() const;
        TrilaterationResult calculate_position_trilateration();

        // Anchor-based positioning (if anchors available)
        bool add_anchor(const UWBAnchor& anchor);
        bool remove_anchor(uint16_t anchor_id);
        std::vector<UWBAnchor> get_anchors() const;
        bool calculate_absolute_position();

        // Network synchronization
        bool synchronize_network_time();
        bool is_network_synchronized() const;
        double get_clock_offset() const;
        bool set_master_clock(bool is_master);

        // Configuration and calibration
        bool set_uwb_config(const UWBConfig& config);
        UWBConfig get_uwb_config() const;
        bool calibrate_ranging();
        bool perform_antenna_delay_calibration();

        // Quality monitoring
        double get_ranging_accuracy() const;
        double get_measurement_rate() const;
        bool is_measurement_quality_good() const;
        UWBNetworkStats get_network_statistics() const;

        // Formation support
        bool monitor_formation_distances();
        std::vector<double> get_formation_distances() const;
        bool detect_formation_violations();
        bool calculate_formation_center() const;

        // Collision avoidance support
        std::vector<DroneID> get_nearby_drones(double max_distance) const;
        bool is_collision_risk(double safety_distance = 3.0) const;
        DroneID get_closest_drone() const;
        double get_distance_to_closest_drone() const;

        // Data export and logging
        bool export_measurements(const std::string& filename) const;
        bool log_uwb_data() const;

    private:
        DroneID drone_id_;
        std::string config_path_;
        uint16_t uwb_address_;

        // UWB hardware interface
        class DW1000Interface; // DecaWave DW1000 UWB module
        std::unique_ptr<DW1000Interface> uwb_hardware_;

        // Configuration
        UWBConfig current_config_;
        mutable std::mutex config_mutex_;

        // Network nodes management
        std::unordered_map<DroneID, UWBNode> uwb_nodes_;
        std::vector<UWBAnchor> uwb_anchors_;
        mutable std::timed_mutex nodes_mutex_;

        // Measurements storage
        std::deque<UWBMeasurement> measurement_buffer_;
        mutable std::mutex measurements_mutex_;
        static constexpr size_t MAX_MEASUREMENT_HISTORY = 1000;



        // Position tracking
        mutable std::mutex position_mutex_;
        Position3D estimated_position_;
        TrilaterationResult last_trilateration_;
        Timestamp last_position_update_;

        // Network synchronization
        std::atomic<bool> is_master_clock_;
        std::atomic<double> clock_offset_;
        std::atomic<bool> network_synchronized_;
        Timestamp last_sync_time_;

        // Threading
        std::thread ranging_thread_;
        std::thread position_calculation_thread_;
        std::thread sync_thread_;
        std::atomic<bool> running_;

        std::condition_variable ranging_cv_;
        std::condition_variable position_cv_;
        std::mutex ranging_mutex_;
        std::mutex position_calc_mutex_;

        // Statistics and monitoring
        UWBNetworkStats network_stats_;
        mutable std::mutex stats_mutex_;

        // Ranging parameters
        UWBRangingMode ranging_mode_;
        std::atomic<double> ranging_accuracy_;
        std::atomic<double> measurement_rate_;

        // Timing constants
        static constexpr Duration RANGING_PERIOD = std::chrono::milliseconds(100);        // 10Hz
        static constexpr Duration POSITION_UPDATE_PERIOD = std::chrono::milliseconds(50); // 20Hz
        static constexpr Duration SYNC_PERIOD = std::chrono::seconds(10);                 // 0.1Hz
        static constexpr Duration NODE_TIMEOUT = std::chrono::seconds(5);

        // Private methods - Main loops
        void ranging_loop();
        void position_calculation_loop();
        void synchronization_loop();

        // Ranging implementation
        bool perform_two_way_ranging(DroneID target_drone);
        bool perform_double_sided_twr(DroneID target_drone);
        bool perform_symmetric_double_sided_twr(DroneID target_drone);
        bool perform_tdoa_measurement();

        // Message handling for UWB protocol
        bool send_ranging_request(DroneID target_drone);
        bool send_ranging_response(DroneID requesting_drone, uint64_t request_timestamp);
        bool process_ranging_message(const std::vector<uint8_t>& message);

        // Position calculation algorithms
        bool calculate_trilateration_2d(const std::vector<UWBMeasurement>& measurements,
                                        TrilaterationResult& result);
        bool calculate_trilateration_3d(const std::vector<UWBMeasurement>& measurements,
                                        TrilaterationResult& result);
        bool calculate_multilateration(const std::vector<UWBMeasurement>& measurements,
                                       TrilaterationResult& result);

        // Geometric calculations
        bool solve_trilateration_equations(const std::vector<Position3D>& positions,
                                           const std::vector<double>& distances,
                                           Position3D& result);
        double calculate_position_error(const Position3D& estimated_pos,
                                        const std::vector<UWBMeasurement>& measurements);

        // Network management helpers
        bool discover_uwb_devices();
        bool validate_node_measurements();
        bool clean_stale_measurements();
        bool update_node_positions();

        // Synchronization helpers
        bool send_sync_beacon();
        bool process_sync_message(const std::vector<uint8_t>& message);
        bool calculate_clock_drift();
        uint64_t get_synchronized_timestamp();

        // Hardware abstraction
        bool initialize_uwb_hardware();
        bool configure_uwb_parameters();
        bool set_uwb_channel(uint8_t channel);
        bool set_transmit_power(uint8_t power_level);
        bool read_uwb_timestamp(uint64_t& timestamp);

        // Calibration procedures
        bool perform_distance_calibration();
        bool calibrate_antenna_delays();
        bool measure_crystal_offset();
        bool validate_calibration_results();

        // Quality assessment
        bool assess_measurement_quality(const UWBMeasurement& measurement);
        double calculate_measurement_variance();
        bool detect_multipath_interference();
        bool filter_outlier_measurements();

        // Formation monitoring helpers
        bool check_formation_geometry();
        std::vector<double> calculate_inter_drone_distances();
        bool detect_formation_drift();
        Position3D calculate_formation_centroid();

        // Collision detection algorithms
        bool predict_collision_trajectory(DroneID other_drone, double time_horizon);
        double calculate_closest_approach_distance(DroneID other_drone);
        bool is_on_collision_course(DroneID other_drone, double safety_margin);

        // Data management
        bool store_measurement(const UWBMeasurement& measurement);
        void cleanup_old_measurements();
        bool export_measurement_data(const std::string& filename);

        // Error handling and recovery
        bool handle_ranging_failure(DroneID target_drone);
        bool attempt_hardware_recovery();
        bool switch_to_backup_ranging_mode();
        bool report_uwb_error(const std::string& error_description);

        // Utility methods
        double calculate_distance_3d(const Position3D& pos1, const Position3D& pos2) const;
        bool is_position_reasonable(const Position3D& position);
        double calculate_measurement_age(const UWBMeasurement& measurement);
        bool is_measurement_fresh(const UWBMeasurement& measurement);
        double calculate_measurement_quality(uint64_t round_trip_time, uint64_t response_time) const;
        double calculate_measurement_quality_for_node(DroneID drone_id) const;
        void update_network_statistics();

// Advanced trilateration methods
        bool calculate_trilateration_3d_anchors(const std::vector<Position3D>& anchor_positions,
                                                const std::vector<double>& distances,
                                                TrilaterationResult& result);
        bool calculate_trilateration_2d_from_anchors(const std::vector<Position3D>& anchor_positions,
                                                     const std::vector<double>& distances,
                                                     TrilaterationResult& result);


        // Configuration helpers
        bool load_uwb_configuration();
        bool validate_uwb_parameters();
        bool apply_configuration_changes();
        bool save_calibration_data();

        // Logging and diagnostics
        void log_uwb_event(const std::string& event, const std::string& details = "");
        void log_measurement_data(const UWBMeasurement& measurement);
        void log_position_calculation(const TrilaterationResult& result);
        bool generate_uwb_diagnostics_report();

    };

} // namespace SwarmControl//

