#pragma once

#include "SwarmTypes.h"
#include "Drone.h"
#include "FormationController.h"
#include "CommunicationManager.h"
#include "MissionPlanner.h"

#include <memory>
#include <unordered_map>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

namespace SwarmControl {

    class SwarmManager {
    public:
        explicit SwarmManager(const std::string& config_path);
        ~SwarmManager();

        // Core lifecycle
        bool initialize();
        bool start();
        void stop();
        void emergency_stop_all();

        // Drone management
        bool add_drone(DronePtr drone);
        bool remove_drone(DroneId drone_id);
        DronePtr get_drone(DroneId drone_id) const;
        std::vector<DronePtr> get_all_drones() const;
        size_t get_drone_count() const;

        // Drone pairing and discovery
        bool start_pairing_mode(Duration timeout = std::chrono::minutes(5));
        bool stop_pairing_mode();
        std::vector<DroneId> discover_available_drones();
        bool pair_drone(DroneId drone_id);
        bool unpair_drone(DroneId drone_id);

        // Leadership management
        bool set_leader(DroneId drone_id);
        DroneId get_current_leader() const;
        bool promote_backup_leader();
        std::vector<DroneId> get_leader_candidates() const;
        bool handle_leader_loss();

        // Formation control
        bool set_formation(FormationType formation_type);
        FormationType get_current_formation() const;
        bool transition_to_formation(FormationType new_formation, Duration transition_time = std::chrono::seconds(10));
        bool is_formation_stable() const;
        double get_formation_error() const;

        // Mission management
        bool load_mission(const Mission& mission);
        bool start_mission();
        bool pause_mission();
        bool resume_mission();
        bool abort_mission();
        MissionPhase get_mission_phase() const;
        double get_mission_progress() const;

        // Swarm status and monitoring
        bool are_all_drones_ready() const;
        bool are_all_drones_in_formation() const;
        std::vector<DroneStatus> get_all_statuses() const;
        std::vector<TelemetryData> get_swarm_telemetry() const;

        // Communication management
        bool establish_swarm_network();
        bool adapt_communication_parameters();
        CommunicationStatus get_swarm_comm_status() const;
        bool handle_frequency_interference();

        // Video stream management
        bool set_video_source(DroneId drone_id);
        DroneId get_current_video_source() const;
        bool switch_video_channel(VideoChannel channel);
        std::vector<VideoStreamConfig> get_available_video_streams() const;

        // Ground station interface
        bool connect_ground_station();
        bool disconnect_ground_station();
        bool send_status_to_ground();
        bool process_ground_command(const std::string& command);

        // Emergency and failsafe procedures
        bool execute_emergency_procedure(SwarmError error);
        bool initiate_self_destruct_sequence();
        bool abort_self_destruct();

        // Swarm intelligence and coordination
        bool update_swarm_positions();
        bool synchronize_swarm_clocks();
        bool distribute_mission_updates();
        bool handle_collision_avoidance();

        // Environmental adaptation
        bool adapt_to_wind_conditions();
        bool adjust_formation_for_terrain();
        bool handle_signal_jamming();

        // Diagnostics and maintenance
        std::vector<ErrorReport> get_swarm_errors() const;
        bool run_swarm_diagnostics();
        bool calibrate_uwb_network();
        bool test_communication_links();

        // Configuration management
        bool reload_configuration();
        bool update_swarm_parameters(const std::string& parameter_group);
        bool save_current_configuration();

        // Statistics and logging
        struct SwarmStatistics {
            size_t total_drones;
            size_t active_drones;
            size_t drones_in_formation;
            double average_formation_error;
            double swarm_velocity;
            Position3D swarm_center;
            Duration mission_elapsed_time;
            uint32_t total_commands_sent;
            uint32_t total_telemetry_received;
            double average_battery_level;
            CommunicationStatus overall_comm_status;
        };

        SwarmStatistics get_swarm_statistics() const;
        bool log_swarm_state();
        bool export_mission_log(const std::string& filename);

    private:
        // Core data structures
        std::unordered_map<DroneId, DronePtr> drones_;
        mutable std::shared_mutex drones_mutex_;

        DroneId current_leader_id_;
        std::vector<DroneId> backup_leaders_;
        mutable std::mutex leadership_mutex_;

        // Formation management
        std::unique_ptr<FormationController> formation_controller_;
        FormationType current_formation_;
        std::atomic<bool> formation_transition_active_;

        // Mission state
        Mission current_mission_;
        std::atomic<MissionPhase> mission_phase_;
        size_t current_waypoint_index_;
        Timestamp mission_start_time_;
        mutable std::mutex mission_mutex_;

        // Communication and networking
        std::unique_ptr<CommunicationProtocol> comm_protocol_;
        std::atomic<bool> ground_station_connected_;
        DroneId video_source_drone_id_;

        // Threading
        std::thread main_coordination_thread_;
        std::thread telemetry_aggregation_thread_;
        std::thread formation_monitoring_thread_;
        std::thread communication_thread_;
        std::atomic<bool> running_;

        // Synchronization
        std::condition_variable coordination_cv_;
        std::mutex coordination_mutex_;

        // Configuration
        std::string config_path_;
        mutable std::mutex config_mutex_;

        // Statistics and monitoring
        SwarmStatistics current_stats_;
        mutable std::mutex stats_mutex_;

        // Error handling
        std::deque<ErrorReport> swarm_error_history_;
        mutable std::mutex error_mutex_;
        static constexpr size_t MAX_SWARM_ERROR_HISTORY = 500;

        // Pairing state
        std::atomic<bool> pairing_mode_active_;
        std::set<DroneId> discovered_drones_;
        mutable std::mutex pairing_mutex_;

        // Private methods - Main loops
        void coordination_loop();
        void telemetry_aggregation_loop();
        void formation_monitoring_loop();
        void communication_loop();

        // Drone management helpers
        bool validate_drone_id(DroneId drone_id) const;
        bool is_drone_responsive(DroneId drone_id) const;
        bool sync_drone_clocks();

        // Leadership management helpers
        DroneId select_best_leader() const;
        bool notify_leadership_change(DroneId new_leader, DroneId old_leader);
        bool validate_leader_capabilities(DroneId drone_id) const;

        // Formation control helpers
        bool calculate_formation_positions(FormationType formation);
        bool assign_formation_positions();
        bool monitor_formation_integrity();
        bool correct_formation_deviations();

        // Mission execution helpers
        bool advance_to_next_waypoint();
        bool validate_mission_parameters(const Mission& mission) const;
        bool distribute_waypoint_updates();

        // Communication helpers
        bool establish_mesh_network();
        bool monitor_communication_quality();
        bool handle_communication_failures();
        bool optimize_network_topology();

        // Emergency procedures
        bool execute_formation_break();
        bool coordinate_emergency_landing();
        bool handle_multiple_drone_failures();

        // Utility methods
        Position3D calculate_swarm_centroid() const;
        double calculate_swarm_spread() const;
        bool are_drones_safe_distance() const;

        // Configuration helpers
        bool load_swarm_configuration();
        bool validate_configuration() const;
        bool apply_configuration_changes();

        // Logging and diagnostics
        void log_swarm_event(const std::string& event, const std::string& details = "");
        bool write_telemetry_log();
        bool generate_diagnostics_report();
    };

} // namespace SwarmControl//
// Created by yv on 22.09.2025.
//

#ifndef DRONESWARMLITE_SWARMMANAGER_H
#define DRONESWARMLITE_SWARMMANAGER_H

#endif //DRONESWARMLITE_SWARMMANAGER_H
