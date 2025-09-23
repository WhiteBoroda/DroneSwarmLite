#pragma once

#include "SwarmTypes.h"
#include "CommunicationManager.h"
#include "FlightController.h"
#include "UWBManager.h"
#include "VideoTransmitter.h"

#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <deque>

namespace SwarmControl {

    class Drone : public std::enable_shared_from_this<Drone> {
    public:
        explicit Drone(DroneId id, const std::string& config_path);
        ~Drone();

        // Core lifecycle
        bool initialize();
        bool start();
        void stop();
        void emergency_stop();
        void self_destruct();

        // Status and identification
        DroneId get_id() const { return drone_id_; }
        DroneStatus get_status() const;
        DroneRole get_role() const;
        bool is_leader() const { return get_role() == DroneRole::LEADER; }
        bool is_ready() const;

        // Position and movement
        Position3D get_position() const;
        Position3D get_relative_position(DroneId reference_drone) const;
        Velocity3D get_velocity() const;
        Attitude get_attitude() const;

        bool set_target_position(const Position3D& target);
        bool set_velocity(const Velocity3D& velocity);
        bool move_to_formation_position(const Position3D& formation_pos);

        // Communication
        bool send_telemetry();
        bool receive_command(const std::string& command_data);
        bool broadcast_status();

        CommunicationStatus get_comm_status() const;
        int8_t get_rssi_lora() const;
        int8_t get_rssi_video() const;

        // Formation management
        bool set_formation_target(const Position3D& target);
        Position3D get_formation_target() const;
        double get_formation_error() const;
        bool is_in_formation() const;

        // Leadership and following
        bool promote_to_leader();
        bool demote_from_leader();
        bool set_leader(DroneId leader_id);
        DroneId get_leader_id() const;
        bool follow_leader();

        // UWB positioning
        bool update_uwb_measurements();
        std::vector<UWBMeasurement> get_uwb_measurements() const;
        bool calculate_relative_positions();

        // Video streaming
        bool start_video_stream(VideoChannel channel = VideoChannel::BAND_F_1);
        bool stop_video_stream();
        bool set_video_channel(VideoChannel channel);
        bool set_video_power(uint16_t power_mw);
        VideoStreamConfig get_video_config() const;

        // Battery and hardware monitoring
        double get_battery_voltage() const;
        uint8_t get_battery_percentage() const;
        double get_cpu_temperature() const;
        bool is_battery_low() const;

        // Mission execution
        bool load_mission(const Mission& mission);
        bool start_mission();
        bool abort_mission();
        MissionPhase get_mission_phase() const;

        // Terminal guidance mode
        bool enter_terminal_guidance();
        bool is_in_terminal_guidance() const;

        // Error handling
        std::vector<ErrorReport> get_recent_errors() const;
        void report_error(SwarmError error, const std::string& description, uint8_t severity = 128);
        void clear_errors();

        // Telemetry and logging
        TelemetryData get_telemetry() const;
        bool log_telemetry();

        // Configuration
        bool reload_config();
        bool update_config(const std::string& key, const std::string& value);

    private:
        // Core data
        DroneId drone_id_;
        std::atomic<DroneStatus> status_;
        std::atomic<DroneRole> role_;
        DroneId leader_id_;

        // Position and movement
        mutable std::mutex position_mutex_;
        Position3D current_position_;
        Position3D target_position_;
        Position3D formation_target_;
        Velocity3D current_velocity_;
        Attitude current_attitude_;

        // Formation control
        std::atomic<bool> in_formation_;
        double formation_error_;
        Timestamp last_formation_update_;

        // Hardware interfaces
        std::unique_ptr<FlightController> flight_controller_;
        std::unique_ptr<CommunicationManager> comm_manager_;
        std::unique_ptr<UWBManager> uwb_manager_;
        std::unique_ptr<VideoTransmitter> video_transmitter_;

        // Mission data
        Mission current_mission_;
        std::atomic<MissionPhase> mission_phase_;
        size_t current_waypoint_index_;

        // Threading and synchronization
        std::thread main_loop_thread_;
        std::thread telemetry_thread_;
        std::thread uwb_thread_;
        std::atomic<bool> running_;
        std::condition_variable main_loop_cv_;
        std::mutex main_loop_mutex_;

        // Communication buffers
        mutable std::mutex telemetry_mutex_;
        TelemetryData latest_telemetry_;
        std::deque<TelemetryData> telemetry_history_;

        // UWB measurements
        mutable std::mutex uwb_mutex_;
        std::vector<UWBMeasurement> uwb_measurements_;
        Timestamp last_uwb_update_;

        // Error tracking
        mutable std::mutex error_mutex_;
        std::deque<ErrorReport> error_history_;
        static constexpr size_t MAX_ERROR_HISTORY = 100;

        // Configuration
        std::string config_path_;
        mutable std::mutex config_mutex_;

        // Battery monitoring
        std::atomic<double> battery_voltage_;
        std::atomic<uint8_t> battery_percentage_;
        std::atomic<double> cpu_temperature_;

        // Private methods
        void main_loop();
        void telemetry_loop();
        void uwb_loop();

        bool update_position();
        bool update_telemetry();
        bool check_formation_compliance();
        bool execute_formation_correction();
        bool handle_leader_loss();

        // Flight control helpers
        bool arm_drone();
        bool disarm_drone();
        bool takeoff_to_altitude(double altitude);
        bool land_drone();

        // Communication helpers
        bool establish_ground_link();
        bool establish_mesh_network();
        bool adapt_communication_power();
        bool handle_frequency_hopping();

        // Formation control helpers
        Position3D calculate_formation_correction();
        bool apply_wind_compensation();
        double calculate_pid_correction(double error, double dt);

        // Safety and failsafe
        bool check_safety_conditions();
        bool execute_failsafe_procedure();
        bool check_collision_avoidance();

        // Terminal guidance
        bool initialize_terminal_guidance();
        bool execute_terminal_approach();

        // Utility methods
        bool validate_position(const Position3D& pos) const;
        bool validate_velocity(const Velocity3D& vel) const;
        double calculate_distance_to_leader() const;
        bool is_communication_active() const;

        // Configuration helpers
        bool load_config_file();
        bool parse_hardware_config();
        bool initialize_hardware_interfaces();

        // Logging
        void log_debug(const std::string& message) const;
        void log_info(const std::string& message) const;
        void log_warning(const std::string& message) const;
        void log_error(const std::string& message) const;
    };

} // namespace SwarmControl//
// Created by yv on 22.09.2025.
//

#ifndef DRONESWARMLITE_DRONE_H
#define DRONESWARMLITE_DRONE_H

#endif //DRONESWARMLITE_DRONE_H
