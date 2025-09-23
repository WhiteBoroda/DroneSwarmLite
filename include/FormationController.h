#pragma once

#include "SwarmTypes.h"
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <unordered_map>
#include <algorithm>

namespace SwarmControl {

// Formation states
    enum class FormationState : uint8_t {
        DISBANDED,        // No formation active
        ASSEMBLING,       // Moving to formation positions
        FORMED,           // In stable formation
        TRANSITIONING,    // Changing formation type
        ADJUSTING,        // Minor position adjustments
        BROKEN,           // Formation integrity lost
        EMERGENCY_SCATTER // Emergency dispersion
    };

// Formation control algorithms
    enum class FormationAlgorithm : uint8_t {
        LEADER_FOLLOWER,  // Follow designated leader
        BEHAVIORAL,       // Flocking/swarming behavior
        VIRTUAL_STRUCTURE, // Maintain virtual geometry
        CONSENSUS,        // Distributed consensus
        GRAPH_BASED      // Graph theory approach
    };

// Formation metrics for quality assessment
    struct FormationMetrics {
        double average_position_error;    // meters
        double maximum_position_error;    // meters
        double formation_cohesion;        // 0-1, how tight the formation is
        double formation_stability;       // 0-1, stability over time
        double leader_following_accuracy; // meters
        size_t drones_in_formation;       // count of drones properly positioned
        double formation_velocity_sync;   // how well velocities are matched

        FormationMetrics() : average_position_error(999.0), maximum_position_error(999.0),
                             formation_cohesion(0.0), formation_stability(0.0),
                             leader_following_accuracy(999.0), drones_in_formation(0),
                             formation_velocity_sync(0.0) {}
    };

// Individual drone formation assignment
    struct DroneFormationAssignment {
        DroneId drone_id;
        Position3D target_position;      // Relative to formation center/leader
        Position3D current_position;     // Current position
        Velocity3D target_velocity;      // Desired velocity
        uint8_t formation_priority;      // 0-255, for conflict resolution
        double position_tolerance;       // Acceptable deviation in meters
        bool position_achieved;          // True if within tolerance
        Timestamp last_update;

        DroneFormationAssignment() : drone_id(0), position_tolerance(2.0),
                                     position_achieved(false), formation_priority(128) {}
    };

// Formation transition parameters
    struct FormationTransition {
        FormationType source_formation;
        FormationType target_formation;
        Duration transition_duration;
        double transition_progress;      // 0.0-1.0
        std::vector<Position3D> intermediate_positions;
        bool smooth_transition;          // Use smooth interpolation
        Timestamp transition_start;

        FormationTransition() : source_formation(FormationType::WEDGE),
                                target_formation(FormationType::LINE),
                                transition_duration(std::chrono::seconds(10)),
                                transition_progress(0.0), smooth_transition(true) {}
    };

    class FormationController {
    public:
        explicit FormationController(const std::string& config_path);
        ~FormationController();

        // Core lifecycle
        bool initialize();
        bool start();
        void stop();
        void reset();

        // Formation management
        bool set_target_formation(FormationType formation_type);
        FormationType get_current_formation() const;
        FormationState get_formation_state() const;
        bool assemble_formation();
        bool disband_formation();

        // Drone registration and management
        bool register_drone(DroneId drone_id, const Position3D& current_pos);
        bool unregister_drone(DroneId drone_id);
        bool update_drone_position(DroneId drone_id, const Position3D& position);
        bool update_drone_velocity(DroneId drone_id, const Velocity3D& velocity);
        std::vector<DroneId> get_registered_drones() const;

        // Leader management
        bool set_formation_leader(DroneId leader_id);
        DroneId get_formation_leader() const;
        bool update_leader_position(const Position3D& leader_pos);
        bool update_leader_velocity(const Velocity3D& leader_vel);
        bool handle_leader_change(DroneId new_leader_id);

        // Position assignment and calculation
        bool calculate_formation_positions();
        Position3D get_target_position(DroneId drone_id) const;
        Velocity3D get_target_velocity(DroneId drone_id) const;
        bool assign_formation_positions();
        bool optimize_drone_assignments();

        // Formation transitions
        bool transition_to_formation(FormationType new_formation, Duration duration = std::chrono::seconds(10));
        bool is_transition_active() const;
        double get_transition_progress() const;
        bool abort_transition();

        // Formation monitoring and control
        bool monitor_formation_integrity();
        bool is_formation_stable() const;
        FormationMetrics get_formation_metrics() const;
        bool correct_formation_deviations();

        // Dynamic formation adjustments
        bool adjust_formation_scale(double scale_factor);
        bool rotate_formation(double angle_radians);
        bool translate_formation(const Position3D& offset);
        bool adapt_formation_to_environment();

        // Collision avoidance within formation
        bool enable_collision_avoidance();
        bool disable_collision_avoidance();
        bool check_collision_risks();
        std::vector<std::pair<DroneId, DroneId>> detect_collision_pairs();

        // Environmental adaptation
        bool adapt_to_wind(const Velocity3D& wind_velocity);
        bool adjust_for_terrain_constraints();
        bool handle_obstacle_avoidance();
        bool modify_formation_for_conditions();

        // Formation algorithms
        bool set_formation_algorithm(FormationAlgorithm algorithm);
        FormationAlgorithm get_current_algorithm() const;
        bool apply_leader_follower_control();
        bool apply_behavioral_control();
        bool apply_virtual_structure_control();

        // Quality assessment
        double calculate_formation_error() const;
        bool assess_formation_quality();
        bool validate_formation_integrity();
        std::vector<DroneId> identify_formation_violators() const;

        // Configuration and parameters
        bool load_formation_definitions();
        bool save_formation_configuration();
        bool set_formation_parameters(const std::string& param_name, double value);
        double get_formation_parameter(const std::string& param_name) const;

        // Statistics and logging
        bool log_formation_data();
        bool export_formation_history(const std::string& filename);
        std::vector<std::string> get_formation_events() const;

    private:
        std::string config_path_;

        // Formation state management
        std::atomic<FormationType> current_formation_type_;
        std::atomic<FormationState> formation_state_;
        std::atomic<FormationAlgorithm> current_algorithm_;

        // Drone management
        std::unordered_map<DroneId, DroneFormationAssignment> drone_assignments_;
        mutable std::shared_mutex assignments_mutex_;

        DroneId formation_leader_id_;
        Position3D leader_position_;
        Velocity3D leader_velocity_;
        mutable std::mutex leader_mutex_;

        // Formation definitions
        std::unordered_map<FormationType, FormationDefinition> formation_definitions_;
        mutable std::mutex definitions_mutex_;

        // Formation transition management
        FormationTransition active_transition_;
        std::atomic<bool> transition_active_;
        mutable std::mutex transition_mutex_;

        // Formation metrics and monitoring
        FormationMetrics current_metrics_;
        mutable std::mutex metrics_mutex_;

        // Threading
        std::thread formation_control_thread_;
        std::thread monitoring_thread_;
        std::thread transition_thread_;
        std::atomic<bool> running_;

        std::condition_variable control_cv_;
        std::condition_variable monitoring_cv_;
        std::mutex control_mutex_;
        std::mutex monitoring_mutex_;

        // Control parameters
        struct FormationControlParams {
            double position_tolerance;        // meters
            double velocity_matching_gain;    // control gain for velocity matching
            double cohesion_strength;        // strength of cohesive forces
            double separation_strength;      // strength of separation forces
            double alignment_strength;       // strength of alignment forces
            double leader_following_gain;    // gain for leader following
            double max_formation_speed;      // m/s
            double collision_avoidance_distance; // meters

            FormationControlParams() : position_tolerance(2.0), velocity_matching_gain(0.5),
                                       cohesion_strength(1.0), separation_strength(2.0),
                                       alignment_strength(0.8), leader_following_gain(1.2),
                                       max_formation_speed(20.0), collision_avoidance_distance(5.0) {}
        } control_params_;

        // Environmental factors
        Velocity3D wind_velocity_;
        std::vector<Position3D> obstacles_;
        mutable std::mutex environment_mutex_;

        // Collision avoidance
        std::atomic<bool> collision_avoidance_enabled_;
        double safety_distance_;

        // Statistics and history
        std::deque<FormationMetrics> metrics_history_;
        std::deque<std::string> formation_events_;
        mutable std::mutex history_mutex_;
        static constexpr size_t MAX_HISTORY_SIZE = 1000;

        // Timing constants
        static constexpr Duration CONTROL_UPDATE_PERIOD = std::chrono::milliseconds(50);  // 20Hz
        static constexpr Duration MONITORING_PERIOD = std::chrono::milliseconds(100);     // 10Hz
        static constexpr Duration TRANSITION_UPDATE_PERIOD = std::chrono::milliseconds(100); // 10Hz

        // Private methods - Main loops
        void formation_control_loop();
        void monitoring_loop();
        void transition_loop();

        // Formation control algorithms
        bool execute_leader_follower_control();
        bool execute_behavioral_control();
        bool execute_virtual_structure_control();
        bool execute_consensus_control();

        // Position calculation helpers
        Position3D calculate_leader_relative_position(DroneId drone_id) const;
        Position3D calculate_formation_centroid() const;
        Velocity3D calculate_average_velocity() const;

        // Formation geometry calculations
        std::vector<Position3D> generate_wedge_positions(size_t num_drones, double spacing) const;
        std::vector<Position3D> generate_line_positions(size_t num_drones, double spacing) const;
        std::vector<Position3D> generate_square_positions(size_t num_drones, double side_length) const;
        std::vector<Position3D> generate_circle_positions(size_t num_drones, double radius) const;
        std::vector<Position3D> generate_diamond_positions(size_t num_drones, double spacing) const;

        // Assignment optimization
        bool optimize_position_assignments();
        double calculate_assignment_cost(const std::vector<Position3D>& positions) const;
        bool solve_assignment_problem();

        // Transition management
        bool calculate_transition_waypoints();
        Position3D interpolate_position(const Position3D& start, const Position3D& end, double progress) const;
        bool update_transition_progress();
        bool validate_transition_feasibility();

        // Formation quality assessment
        double calculate_position_errors() const;
        double calculate_velocity_synchronization() const;
        double calculate_formation_cohesion() const;
        double calculate_formation_stability() const;

        // Collision detection and avoidance
        bool detect_potential_collisions();
        Position3D calculate_avoidance_vector(DroneId drone1, DroneId drone2) const;
        bool apply_separation_forces();

        // Environmental adaptation
        bool calculate_wind_compensation();
        bool adjust_positions_for_obstacles();
        bool modify_formation_spacing();

        // Utility methods
        double calculate_distance_between_drones(DroneId drone1, DroneId drone2) const;
        double calculate_formation_span() const;
        bool is_drone_in_formation_bounds(DroneId drone_id) const;
        Position3D limit_position_change(const Position3D& current, const Position3D& target, double max_change) const;

        // Configuration helpers
        bool load_formation_parameters();
        bool validate_formation_definition(const FormationDefinition& definition) const;
        bool create_default_formations();

        // Error handling
        bool handle_formation_breakdown();
        bool attempt_formation_recovery();
        bool scatter_formation_emergency();

        // Logging and diagnostics
        void log_formation_event(const std::string& event, const std::string& details = "");
        void update_formation_statistics();
        bool generate_formation_report();
    };

} // namespace SwarmControl//
// Created by yv on 22.09.2025.
//

#ifndef DRONESWARMLITE_FORMATIONCONTROLLER_H
#define DRONESWARMLITE_FORMATIONCONTROLLER_H

#endif //DRONESWARMLITE_FORMATIONCONTROLLER_H
