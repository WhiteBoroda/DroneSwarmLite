// include/AutonomousDroneAgent.h
// Автономный агент дрона для распределенной системы
// 🇺🇦 Slava Ukraini! 🇺🇦

#pragma once

#include "SwarmTypes.h"
#include "MeshProtocol.h"
#include "DistributedPositioning.h"
#include <vector>
#include <memory>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>
#include <functional>

namespace SwarmSystem {

// Forward declarations
    namespace MeshNetwork {
        class SwarmMeshProtocol;
    }
    namespace DistributedPositioning {
        class DistributedPositionTracker;
    }

// Mission execution states
    enum class MissionState {
        IDLE = 0,
        RECEIVED_COMMAND,
        EXECUTING_COMMAND,
        FORMATION_FLIGHT,
        LOST_COMMUNICATION,
        AUTONOMOUS_MODE,
        EMERGENCY_MODE,
        TARGETING_MODE,
        SELF_SUFFICIENT,
        MISSION_COMPLETE
    };

// Autonomous decision making
    enum class DecisionType {
        CONTINUE_MISSION,
        CHANGE_FORMATION,
        ELECT_NEW_LEADER,
        EMERGENCY_SCATTER,
        RETURN_TO_BASE,
        SELF_DESTRUCT,
        WAIT_FOR_ORDERS
    };

// Obstacle avoidance
    struct ObstacleInfo {
        Position3D position;
        Position3D velocity;      // If moving obstacle
        double radius;           // Size of obstacle
        double threat_level;     // 0.0-1.0
        uint32_t detected_time;  // When first detected
        bool is_moving;

        ObstacleInfo() : radius(1.0), threat_level(0.5), detected_time(0), is_moving(false) {}
    };

// Navigation waypoint with constraints
    struct NavigationWaypoint {
        Position3D position;
        double target_speed;     // m/s
        double loiter_time;      // seconds to stay at waypoint
        double arrival_radius;   // meters - how close to get
        bool is_critical;        // Mission-critical waypoint

        NavigationWaypoint() : target_speed(10.0), loiter_time(0.0),
                               arrival_radius(5.0), is_critical(false) {}
    };

// Autonomous behavior configuration
    struct AutonomyConfig {
        // Communication timeouts
        Duration leader_timeout;          // Time before assuming leader is lost
        Duration network_timeout;         // Time before going fully autonomous

        // Decision making
        double confidence_threshold;      // Minimum confidence for decisions
        uint32_t consensus_timeout_ms;    // Time to wait for consensus

        // Navigation
        double max_speed;                // m/s
        double formation_tolerance;      // meters
        double collision_avoidance_distance; // meters

        // Emergency behavior
        bool enable_self_destruct;
        Duration emergency_timeout;

        AutonomyConfig() : leader_timeout(std::chrono::seconds(30)),
                           network_timeout(std::chrono::minutes(2)),
                           confidence_threshold(0.7),
                           consensus_timeout_ms(10000),
                           max_speed(25.0),
                           formation_tolerance(3.0),
                           collision_avoidance_distance(10.0),
                           enable_self_destruct(true),
                           emergency_timeout(std::chrono::minutes(5)) {}
    };

// Decision record for logging and analysis
    struct DecisionRecord {
        uint32_t timestamp;
        DecisionType decision;
        double confidence;
        std::string reasoning;
        std::vector<DroneID> involved_drones;
        bool was_successful;

        DecisionRecord() : timestamp(0), decision(DecisionType::CONTINUE_MISSION),
                           confidence(0.0), was_successful(false) {}
    };

// Main autonomous drone agent class
    class AutonomousDroneAgent {
    public:
        explicit AutonomousDroneAgent(DroneID my_id);
        ~AutonomousDroneAgent();

        // Lifecycle management
        bool Initialize(std::shared_ptr<MeshNetwork::SwarmMeshProtocol> mesh_protocol,
                        std::shared_ptr<DistributedPositioning::DistributedPositionTracker> position_tracker);
        bool Start();
        void Stop();
        void Update();  // Main update loop - call frequently

        // Command processing
        bool ReceiveCommand(const DistributedCommand& command);
        bool ExecuteCommand(const DistributedCommand& command);
        bool IsCommandExecuting() const;
        DistributedCommand GetCurrentCommand() const;

        // Mission state
        MissionState GetMissionState() const;
        bool SetMissionState(MissionState new_state);
        double GetMissionProgress() const;

        // Formation flying
        bool JoinFormation(FormationType formation_type, const std::vector<DroneID>& formation_members);
        bool LeaveFormation();
        bool IsInFormation() const;
        Position3D GetFormationTargetPosition() const;

        // Leadership and coordination
        bool BecomeLeader();
        bool StepDownFromLeadership();
        bool IsLeader() const;
        DroneID GetCurrentLeader() const;
        bool ProposeNewLeader(DroneID proposed_leader);

        // Autonomous decision making
        DecisionType AnalyzeSituation();
        bool MakeAutonomousDecision();
        double CalculateDecisionConfidence(DecisionType decision) const;
        std::vector<DecisionRecord> GetDecisionHistory() const;

        // Emergency procedures
        bool HandleEmergencySituation();
        bool InitiateEmergencyScatter();
        bool ExecuteSelfDestruct();
        bool IsInEmergencyMode() const;

        // Navigation and obstacle avoidance
        bool NavigateToPosition(const Position3D& target, double target_speed = 15.0);
        bool FollowWaypoints(const std::vector<NavigationWaypoint>& waypoints);
        bool DetectObstacles();
        bool AvoidObstacles();
        std::vector<ObstacleInfo> GetDetectedObstacles() const;

        // Communication management
        bool IsNetworkConnected() const;
        bool IsCommunicatingWithLeader() const;
        Duration GetTimeSinceLastContact() const;
        bool BroadcastStatus();

        // Targeting mode (terminal guidance)
        bool EnterTargetingMode();
        bool IsInTargetingMode() const;
        bool DisconnectFromNetwork();  // For terminal guidance

        // Configuration and parameters
        bool SetAutonomyConfig(const AutonomyConfig& config);
        AutonomyConfig GetAutonomyConfig() const;
        bool EnableAutonomousMode(bool enable);
        bool IsAutonomousModeEnabled() const;

        // Status and diagnostics
        TelemetryData GetTelemetryData() const;
        double GetSystemHealth() const;
        std::vector<SystemEvent> GetRecentEvents() const;
        bool RunSelfDiagnostics();

    private:
        DroneID my_id_;
        std::atomic<bool> running_;

        // Core subsystems
        std::shared_ptr<MeshNetwork::SwarmMeshProtocol> mesh_protocol_;
        std::shared_ptr<DistributedPositioning::DistributedPositionTracker> position_tracker_;

        // Mission state management
        std::atomic<MissionState> mission_state_;
        std::atomic<DroneRole> current_role_;
        DistributedCommand current_command_;
        std::priority_queue<DistributedCommand> command_queue_;
        mutable std::mutex command_mutex_;

        // Formation state
        std::atomic<bool> in_formation_;
        FormationType current_formation_;
        std::vector<DroneID> formation_members_;
        Position3D formation_target_position_;
        mutable std::mutex formation_mutex_;

        // Leadership tracking
        std::atomic<DroneID> current_leader_;
        uint32_t last_leader_contact_;
        std::atomic<bool> is_leader_;

        // Navigation state
        std::vector<NavigationWaypoint> current_waypoints_;
        size_t current_waypoint_index_;
        Position3D navigation_target_;
        mutable std::mutex navigation_mutex_;

        // Obstacle avoidance
        std::vector<ObstacleInfo> detected_obstacles_;
        mutable std::mutex obstacles_mutex_;

        // Decision making
        std::vector<DecisionRecord> decision_history_;
        uint32_t last_decision_time_;
        mutable std::mutex decisions_mutex_;

        // Configuration
        AutonomyConfig autonomy_config_;
        std::atomic<bool> autonomous_mode_enabled_;

        // Threading
        std::thread main_thread_;
        std::thread navigation_thread_;
        std::thread monitoring_thread_;

        // Timing and synchronization
        std::mutex state_mutex_;
        uint32_t last_update_time_;
        uint32_t last_telemetry_broadcast_;
        uint32_t last_obstacle_scan_;

        // Private methods - Command execution
        bool ExecuteFormationCommand(const DistributedCommand& command);
        bool ExecuteMovementCommand(const DistributedCommand& command);
        bool ExecuteTargetingCommand(const DistributedCommand& command);
        bool ExecuteEmergencyCommand(const DistributedCommand& command);

        // Private methods - Formation management
        void UpdateFormationPosition();
        bool CalculateFormationTarget();
        bool MaintainFormationPosition();
        double CalculateFormationError() const;

        // Private methods - Leadership
        void MonitorLeaderCommunication();
        bool EvaluateLeadershipCandidates();
        DroneID SelectBestLeader(const std::vector<DroneID>& candidates);
        void HandleLeadershipTransition(DroneID new_leader);

        // Private methods - Decision making
        double EvaluateMissionContinuation() const;
        double EvaluateFormationChange() const;
        double EvaluateLeaderElection() const;
        double EvaluateEmergencyActions() const;
        bool HasSufficientInformation() const;

        // Private methods - Navigation
        void NavigationLoop();
        bool UpdateNavigationTarget();
        bool IsWaypointReached(const NavigationWaypoint& waypoint) const;
        Position3D CalculateAvoidanceVector() const;

        // Private methods - Obstacle detection
        void ScanForObstacles();
        bool IsCollisionThreat(const ObstacleInfo& obstacle) const;
        double CalculateCollisionTime(const ObstacleInfo& obstacle) const;
        void UpdateObstacleList();

        // Private methods - Network monitoring
        void MonitorNetworkConnectivity();
        bool IsNetworkHealthy() const;
        void HandleCommunicationLoss();
        void HandleNetworkRecovery();

        // Private methods - Emergency procedures
        void MonitorSystemHealth();
        bool DetectEmergencyConditions();
        void ExecuteEmergencyProcedure(const std::string& emergency_type);

        // Private methods - Message handling
        void HandleMeshMessage(const MeshNetwork::MeshMessage& message, DroneID from_neighbor);
        void HandleCommandMessage(const MeshNetwork::MeshMessage& message);
        void HandleLeadershipMessage(const MeshNetwork::MeshMessage& message);
        void HandleEmergencyMessage(const MeshNetwork::MeshMessage& message);

        // Private methods - Utilities
        uint32_t GetCurrentTime() const;
        bool IsTimeout(uint32_t timestamp, Duration timeout) const;
        double CalculateDistance(const Position3D& p1, const Position3D& p2) const;
        void LogEvent(SystemEvent event, const std::string& description);
        void LogDecision(DecisionType decision, double confidence, const std::string& reasoning);

        // Private methods - Hardware interface
        bool SendFlightCommand(const Position3D& target, double speed);
        bool GetCurrentPosition(Position3D& position) const;
        bool GetCurrentVelocity(Velocity3D& velocity) const;
        bool IsHardwareHealthy() const;
    };

// Factory function
    std::shared_ptr<AutonomousDroneAgent> CreateAutonomousDroneAgent(DroneID drone_id);

// Utility functions for autonomous operations
    namespace AutonomyUtils {
        // Decision making helpers
        double CalculateMissionRisk(const DistributedCommand& command,
                                    const std::vector<ObstacleInfo>& obstacles);
        double CalculateFormationStability(const std::vector<DroneID>& formation_members,
                                           const std::vector<Position3D>& positions);
        DroneID SelectLeaderByConsensus(const std::vector<DroneID>& candidates,
                                        const std::vector<TelemetryData>& telemetry_data);

        // Navigation helpers
        std::vector<NavigationWaypoint> PlanPath(const Position3D& start, const Position3D& end,
                                                 const std::vector<ObstacleInfo>& obstacles);
        Position3D CalculateAvoidanceManeuver(const Position3D& current_pos,
                                              const Position3D& target_pos,
                                              const std::vector<ObstacleInfo>& obstacles);

        // Formation helpers
        std::vector<Position3D> CalculateFormationPositions(FormationType formation,
                                                            const Position3D& center,
                                                            size_t num_drones,
                                                            double spacing);
        double OptimizeFormationSpacing(const std::vector<Position3D>& drone_positions);

        // Communication helpers
        std::vector<uint8_t> SerializeAutonomyStatus(MissionState state,
                                                     const AutonomyConfig& config);
        bool DeserializeAutonomyStatus(const std::vector<uint8_t>& data,
                                       MissionState& state,
                                       AutonomyConfig& config);
    }

} // namespace SwarmSystem