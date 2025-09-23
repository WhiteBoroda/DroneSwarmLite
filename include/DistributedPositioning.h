// include/DistributedPositioning.h
// Система распределенного позиционирования без GPS
// 🇺🇦 Slava Ukraini! 🇺🇦

#pragma once

#include "SwarmTypes.h"
#include "MeshProtocol.h"
#include <unordered_map>
#include <vector>
#include <memory>
#include <set>
#include <mutex>
#include <deque>

namespace DistributedPositioning {

// Forward declarations
    namespace MeshNetwork {
        class SwarmMeshProtocol;
    }

// Types of coordinate system anchors
    enum class AnchorType {
        NONE = 0,
        GEOMETRIC_CENTER,       // Center of drone formation
        MOST_STABLE_DRONE,     // Most stable drone as reference
        EXTERNAL_REFERENCE,    // Known external landmark
        CONSENSUS_ANCHOR,      // Consensus-based virtual anchor
        DYNAMIC_LEADER         // Current mission leader
    };

// Coordinate system anchor point
    struct AnchorPoint {
        Position3D position;            // Anchor position in local coordinates
        AnchorType type;               // Type of anchor
        DroneID anchor_drone_id;       // ID of drone serving as anchor (if applicable)
        double stability_score;        // Stability metric (0.0-1.0)
        uint32_t established_time;     // When anchor was established (ms)
        uint32_t last_update;          // Last update time (ms)
        bool is_valid;                 // Whether anchor is currently valid

        // Quality metrics
        double position_accuracy;      // Estimated position accuracy (meters)
        double velocity_stability;     // Velocity stability metric (0.0-1.0)
        double reference_confidence;   // Confidence in this reference (0.0-1.0)

        AnchorPoint();
    };

// Position history for stability analysis
    struct PositionHistory {
        std::deque<Position3D> positions;
        std::deque<uint32_t> timestamps;
        static const size_t MAX_HISTORY = 50;

        void AddPosition(const Position3D& pos, uint32_t time);
        double GetStabilityScore() const;
        double GetAverageVelocity() const;
        Position3D GetPredictedPosition(uint32_t future_time) const;
        void Clear();
    };

// Relative position measurement between drones
    struct RelativePositionMeasurement {
        DroneID reference_drone;       // Drone used as reference
        DroneID measured_drone;        // Drone being measured
        Position3D relative_position;  // Position relative to reference
        double accuracy;               // Measurement accuracy (meters)
        uint32_t timestamp;           // Measurement time (ms)
        double confidence;            // Confidence in measurement (0.0-1.0)

        // Source of measurement (UWB, visual, etc.)
        enum Source {
            UWB_RANGING,
            VISUAL_TRACKING,
            DEAD_RECKONING,
            FORMATION_ESTIMATE
        } source;

        RelativePositionMeasurement();
    };

// Consensus voting for anchor selection
    struct AnchorVote {
        DroneID voting_drone;
        DroneID proposed_anchor;
        AnchorType proposed_type;
        double confidence_score;
        uint32_t timestamp;

        AnchorVote() : voting_drone(INVALID_DRONE_ID), proposed_anchor(INVALID_DRONE_ID),
                       proposed_type(AnchorType::NONE), confidence_score(0.0), timestamp(0) {}
    };

// Formation tracking information
    struct FormationTracker {
        FormationType current_formation;
        Position3D formation_center;
        double formation_scale;        // Scale factor from nominal formation
        double formation_rotation;     // Rotation angle (radians)
        std::unordered_map<DroneID, Position3D> target_positions;
        std::unordered_map<DroneID, Position3D> actual_positions;
        double formation_error;        // RMS error from target positions

        FormationTracker() : current_formation(FormationType::NONE), formation_scale(1.0),
                             formation_rotation(0.0), formation_error(999.0) {}
    };

// Main distributed position tracker class
    class DistributedPositionTracker {
    public:
        explicit DistributedPositionTracker(DroneID my_id,
                                            std::shared_ptr<MeshNetwork::SwarmMeshProtocol> mesh);
        ~DistributedPositionTracker();

        // Lifecycle management
        bool Initialize();
        bool Start();
        void Stop();
        void Update();  // Call periodically

        // Position updates
        bool UpdateMyPosition(const Position3D& measured_position, double accuracy = 1.0);
        bool ProcessPositionUpdate(DroneID drone_id, const Position3D& position,
                                   double accuracy, uint32_t timestamp);
        bool AddRelativeMeasurement(const RelativePositionMeasurement& measurement);

        // Position queries
        Position3D GetAbsolutePosition(DroneID drone_id) const;
        Position3D GetRelativePosition(DroneID drone_id, DroneID reference_drone) const;
        Position3D GetMyPosition() const;
        bool IsPositionKnown(DroneID drone_id) const;
        double GetPositionAccuracy(DroneID drone_id) const;

        // Anchor management
        bool EstablishAnchor(AnchorType type, DroneID anchor_drone = INVALID_DRONE_ID);
        bool ChangeAnchor(const AnchorPoint& new_anchor);
        AnchorPoint GetCurrentAnchor() const;
        bool IsAnchorValid() const;
        std::vector<DroneID> GetAnchorCandidates() const;

        // Consensus and voting
        bool ProposeAnchor(DroneID proposed_anchor, AnchorType type, double confidence);
        void ProcessAnchorVote(const AnchorVote& vote);
        bool ExecuteAnchorConsensus();
        double GetAnchorConsensusScore(DroneID proposed_anchor) const;

        // Formation tracking
        bool SetFormationDefinition(const FormationDefinition& formation);
        bool UpdateFormationCenter(const Position3D& center);
        FormationTracker GetFormationStatus() const;
        bool IsFormationStable() const;
        double CalculateFormationError() const;

        // Network synchronization
        bool SynchronizeWithNetwork();
        bool BroadcastPositionUpdate();
        void ProcessNetworkPositionUpdate(DroneID source_drone, const std::vector<uint8_t>& data);

        // Coordinate system management
        bool TransformToLocalCoordinates(const Position3D& global_pos, Position3D& local_pos) const;
        bool TransformToGlobalCoordinates(const Position3D& local_pos, Position3D& global_pos) const;
        bool RecalibrateCoordinateSystem();

        // Quality and diagnostics
        double GetOverallPositionQuality() const;
        double GetNetworkSyncQuality() const;
        std::vector<DroneID> GetLostDrones() const;
        size_t GetTrackedDroneCount() const;

        // Configuration
        void SetPositionTimeout(Duration timeout);
        void SetAnchorTimeout(Duration timeout);
        void SetMinimumAccuracy(double min_accuracy);
        void SetConsensusThreshold(double threshold);

    private:
        DroneID my_id_;
        bool running_;

        // Current coordinate system anchor
        AnchorPoint current_anchor_;
        mutable std::mutex anchor_mutex_;

        // Drone positions (in current coordinate system)
        std::unordered_map<DroneID, Position3D> absolute_positions_;
        std::unordered_map<DroneID, double> position_accuracies_;
        std::unordered_map<DroneID, uint32_t> position_timestamps_;
        mutable std::mutex positions_mutex_;

        // Position histories for stability analysis
        std::unordered_map<DroneID, PositionHistory> position_histories_;
        mutable std::mutex histories_mutex_;

        // Relative measurements between drones
        std::vector<RelativePositionMeasurement> relative_measurements_;
        mutable std::mutex measurements_mutex_;

        // Formation tracking
        FormationTracker formation_tracker_;
        FormationDefinition formation_definition_;
        mutable std::mutex formation_mutex_;

        // Anchor consensus system
        std::vector<AnchorVote> anchor_votes_;
        std::unordered_map<DroneID, double> anchor_candidate_scores_;
        mutable std::mutex consensus_mutex_;

        // Network communication
        std::shared_ptr<MeshNetwork::SwarmMeshProtocol> mesh_protocol_;

        // Configuration parameters
        Duration position_timeout_;
        Duration anchor_timeout_;
        double minimum_accuracy_;
        double consensus_threshold_;

        // Timing
        uint32_t last_position_broadcast_;
        uint32_t last_anchor_evaluation_;
        uint32_t last_consensus_check_;

        // Private methods - Anchor management
        void EvaluateAnchorCandidates();
        double CalculateAnchorStability(DroneID candidate_drone) const;
        bool ValidateAnchorTransition(const AnchorPoint& new_anchor) const;
        void BroadcastAnchorChange(const AnchorPoint& new_anchor);

        // Private methods - Position estimation
        bool EstimatePositionFromRelativeMeasurements(DroneID target_drone, Position3D& estimated_pos);
        void UpdatePositionEstimates();
        void CleanupOldMeasurements();
        bool TriangulatePosition(DroneID target_drone, const std::vector<RelativePositionMeasurement>& measurements,
                                 Position3D& result);

        // Private methods - Formation tracking
        void UpdateFormationTracking();
        bool CalculateFormationGeometry();
        void EstimateMissingPositions();
        Position3D CalculateFormationCenter() const;

        // Private methods - Coordinate transformations
        void RecalculateCoordinateSystem();
        bool ApplyCoordinateTransform(const Position3D& old_anchor, const Position3D& new_anchor);

        // Private methods - Network synchronization
        std::vector<uint8_t> SerializePositionData() const;
        bool DeserializePositionData(const std::vector<uint8_t>& data, DroneID& source_drone,
                                     std::unordered_map<DroneID, Position3D>& positions);
        void ProcessNetworkSync(DroneID source_drone, const std::unordered_map<DroneID, Position3D>& positions);

        // Private methods - Quality assessment
        double CalculatePositionQuality(DroneID drone_id) const;
        double CalculateNetworkConsistency() const;
        void UpdateQualityMetrics();

        // Private methods - Utilities
        uint32_t GetCurrentTime() const;
        bool IsTimeoutExpired(uint32_t timestamp, Duration timeout) const;
        double CalculateDistance3D(const Position3D& p1, const Position3D& p2) const;
        Position3D InterpolatePosition(const Position3D& p1, const Position3D& p2, double factor) const;

        // Message handlers for mesh network
        void HandlePositionUpdateMessage(const ::MeshNetwork::MeshMessage& message, DroneID from_neighbor);
        void HandleAnchorAnnouncement(const ::MeshNetwork::MeshMessage& message, DroneID from_neighbor);
        void HandleConsensusVote(const ::MeshNetwork::MeshMessage& message, DroneID from_neighbor);
    };

// Utility functions for coordinate systems and transformations
    namespace PositioningUtils {
        // Coordinate system transformations
        Position3D TransformPosition(const Position3D& pos, const Position3D& old_origin,
                                     const Position3D& new_origin, double rotation = 0.0);
        double CalculateRotation(const Position3D& old_ref, const Position3D& new_ref);

        // Formation geometry calculations
        std::vector<Position3D> GenerateFormationPositions(FormationType type, size_t num_drones,
                                                           double spacing);
        Position3D CalculateFormationCenter(const std::vector<Position3D>& positions);
        double CalculateFormationSpread(const std::vector<Position3D>& positions,
                                        const Position3D& center);

        // Quality metrics
        double CalculatePositionVariance(const std::vector<Position3D>& positions);
        double CalculateConsistencyScore(const std::unordered_map<DroneID, Position3D>& positions1,
                                         const std::unordered_map<DroneID, Position3D>& positions2);

        // Serialization helpers
        std::vector<uint8_t> SerializePosition(const Position3D& pos);
        Position3D DeserializePosition(const std::vector<uint8_t>& data, size_t& offset);
        std::vector<uint8_t> SerializeAnchorPoint(const AnchorPoint& anchor);
        AnchorPoint DeserializeAnchorPoint(const std::vector<uint8_t>& data);
    }

} // namespace DistributedPositioning