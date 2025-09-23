// include/SwarmTypes.h
// Базовые типы данных для распределенной системы управления роем
// 🇺🇦 Slava Ukraini! 🇺🇦

#pragma once

#include <vector>
#include <string>
#include <chrono>
#include <cstdint>

// Базовые типы
using DroneID = uint16_t;
using Timestamp = std::chrono::steady_clock::time_point;
using Duration = std::chrono::milliseconds;

// Constants
constexpr DroneID INVALID_DRONE_ID = 0xFFFF;
constexpr DroneID BROADCAST_ID = 0x0000;

// 3D Position and movement structures
struct Position3D {
    double x, y, z;  // meters

    Position3D() : x(0), y(0), z(0) {}
    Position3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Position3D operator+(const Position3D& other) const {
        return Position3D(x + other.x, y + other.y, z + other.z);
    }

    Position3D operator-(const Position3D& other) const {
        return Position3D(x - other.x, y - other.y, z - other.z);
    }

    double distance(const Position3D& other) const;
    double magnitude() const;
};

struct Velocity3D {
    double vx, vy, vz;  // m/s

    Velocity3D() : vx(0), vy(0), vz(0) {}
    Velocity3D(double vx_, double vy_, double vz_) : vx(vx_), vy(vy_), vz(vz_) {}

    double magnitude() const;
};

struct Attitude3D {
    double roll, pitch, yaw;  // radians

    Attitude3D() : roll(0), pitch(0), yaw(0) {}
    Attitude3D(double r, double p, double y) : roll(r), pitch(p), yaw(y) {}
};

// Drone states for distributed system
enum class DroneState {
    OFFLINE = 0,
    INITIALIZING,
    READY,
    AUTONOMOUS_FLIGHT,
    IN_FORMATION,
    LOST_COMMUNICATION,
    EMERGENCY,
    TARGETING_MODE,
    SELF_DESTRUCT
};

enum class DroneRole {
    FOLLOWER = 0,
    TEMPORARY_LEADER,
    ANCHOR_POINT,
    SCOUT,
    DESIGNATED_TARGET
};

// Distributed command structure
struct DistributedCommand {
    enum Type {
        FORM_FORMATION = 1,
        CHANGE_FORMATION,
        MOVE_TO_POSITION,
        FOLLOW_WAYPOINTS,
        DESIGNATE_TARGET,
        EMERGENCY_SCATTER,
        SELF_DESTRUCT,
        RETURN_TO_BASE
    };

    Type command_type;
    DroneID target_drone;        // 0xFFFF = all drones
    Position3D target_position;
    std::vector<Position3D> waypoints;
    uint8_t priority;            // 0 = highest, 255 = lowest
    uint32_t sequence_number;
    Timestamp issued_time;
    Duration timeout;

    // Command-specific parameters
    union {
        struct {
            uint8_t formation_type;  // 0=wedge, 1=line, 2=square, etc.
            double spacing;
            double altitude;
        } formation_params;

        struct {
            double approach_speed;
            double terminal_distance;
        } targeting_params;
    };

    DistributedCommand() : command_type(FORM_FORMATION), target_drone(BROADCAST_ID),
                           priority(128), sequence_number(0), timeout(std::chrono::minutes(10)) {}
};

// Formation definitions for distributed system
enum class FormationType {
    NONE = 0,
    WEDGE,
    LINE,
    SQUARE,
    CIRCLE,
    DIAMOND,
    CUSTOM
};

struct FormationPosition {
    Position3D relative_position;  // relative to formation center
    DroneRole preferred_role;
    uint8_t priority;

    FormationPosition() : preferred_role(DroneRole::FOLLOWER), priority(128) {}
};

struct FormationDefinition {
    FormationType type;
    std::vector<FormationPosition> positions;
    double default_spacing;       // meters
    double vertical_separation;   // meters between altitude layers
    std::string description;

    FormationDefinition() : type(FormationType::NONE), default_spacing(10.0),
                            vertical_separation(5.0) {}
};

// Communication and mesh network types
struct MeshNodeInfo {
    DroneID node_id;
    Position3D last_known_position;
    Timestamp last_seen;
    int8_t signal_strength;      // dBm
    uint8_t hop_count;           // hops from this node
    double reliability_score;    // 0.0-1.0
    bool is_reachable;

    MeshNodeInfo() : node_id(INVALID_DRONE_ID), last_seen(std::chrono::steady_clock::now()),
                     signal_strength(-100), hop_count(255), reliability_score(0.0),
                     is_reachable(false) {}
};

// Hardware status and telemetry
struct HardwareStatus {
    double battery_voltage;      // V
    uint8_t battery_percentage;  // 0-100%
    double cpu_temperature;      // °C
    bool flight_controller_ok;
    bool lora_ok;
    bool uwb_ok;
    bool video_tx_ok;

    HardwareStatus() : battery_voltage(0), battery_percentage(0), cpu_temperature(0),
                       flight_controller_ok(false), lora_ok(false),
                       uwb_ok(false), video_tx_ok(false) {}
};

struct TelemetryData {
    DroneID drone_id;
    Timestamp timestamp;
    DroneState state;
    DroneRole role;
    Position3D position;
    Velocity3D velocity;
    Attitude3D attitude;
    HardwareStatus hardware;

    // Formation data
    Position3D formation_target;
    double formation_error;      // meters deviation

    // Communication metrics
    int8_t mesh_rssi;           // dBm
    uint8_t mesh_packet_loss;   // percentage
    uint8_t mesh_neighbors;     // count

    TelemetryData() : drone_id(INVALID_DRONE_ID), timestamp(std::chrono::steady_clock::now()),
                      state(DroneState::OFFLINE), role(DroneRole::FOLLOWER),
                      formation_error(999.0), mesh_rssi(-100),
                      mesh_packet_loss(100), mesh_neighbors(0) {}
};

// Error and event reporting
enum class SystemEvent {
    DRONE_ONLINE = 1,
    DRONE_OFFLINE,
    FORMATION_ESTABLISHED,
    FORMATION_BROKEN,
    COMMUNICATION_LOST,
    COMMUNICATION_RESTORED,
    ANCHOR_CHANGED,
    COMMAND_RECEIVED,
    COMMAND_COMPLETED,
    EMERGENCY_TRIGGERED,
    TARGET_DESIGNATED,
    MISSION_COMPLETED
};

struct EventReport {
    SystemEvent event_type;
    DroneID source_drone;
    std::string description;
    Timestamp timestamp;
    uint8_t severity;            // 0=info, 128=warning, 255=critical

    EventReport() : event_type(SystemEvent::DRONE_ONLINE), source_drone(INVALID_DRONE_ID),
                    timestamp(std::chrono::steady_clock::now()), severity(0) {}
};

// UWB positioning data
struct UWBMeasurement {
    DroneID target_drone;
    double distance;             // meters
    double angle;                // radians (if available)
    Timestamp timestamp;
    double accuracy;             // estimated accuracy in meters

    UWBMeasurement() : target_drone(INVALID_DRONE_ID), distance(0), angle(0),
                       timestamp(std::chrono::steady_clock::now()), accuracy(999) {}
};

// Video streaming
enum class VideoChannel {
    CHANNEL_1 = 5740,   // MHz
    CHANNEL_2 = 5760,
    CHANNEL_3 = 5780,
    CHANNEL_4 = 5800,
    CHANNEL_5 = 5820,
    CHANNEL_6 = 5840,
    CHANNEL_7 = 5860,
    CHANNEL_8 = 5880
};

struct VideoStreamConfig {
    VideoChannel channel;
    uint16_t power_mw;           // 25-2500mW
    bool active;
    DroneID streaming_drone;

    VideoStreamConfig() : channel(VideoChannel::CHANNEL_1), power_mw(200),
                          active(false), streaming_drone(INVALID_DRONE_ID) {}
};

// Collections and containers
using DronePositions = std::vector<std::pair<DroneID, Position3D>>;
using TelemetryBuffer = std::vector<TelemetryData>;
using UWBMeasurements = std::vector<UWBMeasurement>;
using EventLog = std::vector<EventReport>;

// Utility constants
namespace Config {
    constexpr double MAX_COMMUNICATION_RANGE = 10000.0;  // meters
    constexpr double MAX_UWB_RANGE = 1000.0;            // meters
    constexpr double MIN_FORMATION_SPACING = 3.0;       // meters
    constexpr double MAX_FORMATION_SPACING = 100.0;     // meters
    constexpr double MAX_FLIGHT_ALTITUDE = 500.0;       // meters
    constexpr double MAX_FLIGHT_SPEED = 30.0;           // m/s
    constexpr Duration COMMUNICATION_TIMEOUT = std::chrono::seconds(10);
    constexpr Duration HEARTBEAT_INTERVAL = std::chrono::seconds(1);
    constexpr Duration TELEMETRY_INTERVAL = std::chrono::seconds(2);
}

// Helper functions
namespace Utils {
    double calculateDistance3D(const Position3D& p1, const Position3D& p2);
    double calculateBearing(const Position3D& from, const Position3D& to);
    Position3D interpolatePosition(const Position3D& start, const Position3D& end, double factor);
    bool isValidDroneID(DroneID id);
    std::string droneStateToString(DroneState state);
    std::string droneRoleToString(DroneRole role);
    uint32_t getCurrentTimestamp();  // milliseconds since epoch
}