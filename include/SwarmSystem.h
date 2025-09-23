#pragma once

#include <vector>
#include <memory>
#include <atomic>
#include <chrono>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Dense>

namespace SwarmSystem {

// Базові типи даних
    using DroneID = uint32_t;
    using Timestamp = std::chrono::milliseconds;
    using Position3D = Eigen::Vector3d;
    using Velocity3D = Eigen::Vector3d;

// Енумерації
    enum class DroneRole {
        LEADER,
        FOLLOWER,
        CANDIDATE_LEADER,
        LOST_COMMUNICATION,
        SELF_DESTRUCT
    };

    enum class FormationType {
        WEDGE,
        LINE_HORIZONTAL,
        LINE_VERTICAL,
        SQUARE,
        CUSTOM
    };

    enum class CommunicationStatus {
        EXCELLENT,
        GOOD,
        POOR,
        CRITICAL,
        LOST
    };

    enum class MissionPhase {
        PAIRING,
        TAKEOFF,
        FORMATION_BUILD,
        FOLLOW_LEADER,
        TARGET_APPROACH,
        TERMINAL_GUIDANCE,
        EMERGENCY
    };

// Додаткові структури для OSD та команд рою
    struct OSDCommand {
        enum Type {
            ACTIVATE_TARGETING = 1,
            SWITCH_VIDEO_SOURCE = 2,
            MANUAL_LEADER_SELECT = 3,
            TARGET_LOCK = 4,
            TARGET_ENGAGE = 5
        };

        Type command;
        DroneID target_drone;
        DroneID video_source_drone;
        Position3D target_coordinates;
        std::chrono::high_resolution_clock::time_point timestamp;

        OSDCommand() : command(ACTIVATE_TARGETING), target_drone(0), video_source_drone(0) {}
    };

    struct SwarmCommand {
        enum Type {
            HOVER_IN_PLACE = 1,
            FOLLOW_LEADER = 2,
            MAINTAIN_FORMATION = 3,
            EMERGENCY_STOP = 4,
            RETURN_TO_FORMATION = 5
        };

        enum Priority {
            LOW = 1,
            NORMAL = 2,
            HIGH = 3,
            URGENT = 4
        };

        Type command_type;
        DroneID sender_id;
        Priority priority;
        uint64_t timestamp;
        Position3D target_position;  // Опціональна ціль
        double parameter;            // Опціональний параметр

        SwarmCommand() : command_type(HOVER_IN_PLACE), sender_id(0),
                         priority(NORMAL), timestamp(0), parameter(0.0) {}
    };

// Розширюємо enum для LeadershipMessage
    struct LeadershipMessage {
        enum Type {
            HEARTBEAT = 1,
            ELECTION_REQUEST = 2,
            ELECTION_RESPONSE = 3,
            LEADER_TRANSFER = 4,
            LEADER_CONFIRMATION = 5,
            ATTACK_MODE_ENGAGED = 6,
            URGENT_LEADER_TRANSFER = 7  // Нове: швидка передача
        };

        Type message_type;
        DroneID sender_id;
        DroneID target_leader_id;
        double leadership_score;
        uint64_t timestamp;

        // Дані для оцінки лідерства
        double battery_level;
        double signal_strength;
        double system_health_score;
        double position_accuracy;
        bool is_attack_capable;

        LeadershipMessage() {
            message_type = HEARTBEAT;
            sender_id = 0;
            target_leader_id = 0;
            leadership_score = 0.0;
            timestamp = 0;
            battery_level = 0.0;
            signal_strength = 0.0;
            system_health_score = 0.0;
            position_accuracy = 0.0;
            is_attack_capable = true;
        }
    };
    DroneID id;
    Position3D position;
    Position3D relative_position;  // відносно лідера
    Velocity3D velocity;
    double battery_level;
    double temperature;
    CommunicationStatus comm_status;
    DroneRole role;
    Timestamp last_update;
    bool is_healthy;

    DroneState() : id(0), battery_level(100.0), temperature(25.0),
                   comm_status(CommunicationStatus::LOST),
                   role(DroneRole::FOLLOWER), is_healthy(false) {}
};

struct FormationPattern {
    FormationType type;
    std::vector<Position3D> positions;  // відносні позиції
    double spacing;
    double altitude;

    FormationPattern() : type(FormationType::WEDGE), spacing(15.0), altitude(100.0) {}
};

struct CommunicationConfig {
    std::vector<uint32_t> lora_frequencies;
    uint32_t current_frequency;
    int8_t power_level;
    bool mesh_enabled;
    double rssi_threshold;
    double noise_threshold;

    CommunicationConfig() : current_frequency(868100000), power_level(10),
                            mesh_enabled(false), rssi_threshold(-100.0),
                            noise_threshold(-90.0) {}
};

// Базовий клас конфігурації
class ConfigManager {
private:
    YAML::Node config_;
    std::string config_path_;

public:
    explicit ConfigManager(const std::string& config_path);

    bool LoadConfig();
    bool ReloadConfig();

    template<typename T>
    T GetValue(const std::string& key, const T& default_value = T{}) const;

    template<typename T>
    bool SetValue(const std::string& key, const T& value);

    bool SaveConfig() const;

    // Спеціалізовані методи
    std::vector<uint32_t> GetLoRaFrequencies() const;
    FormationPattern GetFormation(FormationType type) const;
    CommunicationConfig GetCommConfig() const;
};

// Абстрактний базовий клас для всіх підсистем
class SwarmSubsystem {
protected:
    std::shared_ptr<ConfigManager> config_;
    std::atomic<bool> is_running_;
    std::atomic<bool> is_healthy_;

public:
    explicit SwarmSubsystem(std::shared_ptr<ConfigManager> config)
            : config_(config), is_running_(false), is_healthy_(false) {}

    virtual ~SwarmSubsystem() = default;

    virtual bool Initialize() = 0;
    virtual bool Start() = 0;
    virtual bool Stop() = 0;
    virtual bool IsHealthy() const { return is_healthy_.load(); }
    virtual bool IsRunning() const { return is_running_.load(); }
    virtual void Update() = 0;
};

// Система зв'язку
class CommunicationSystem : public SwarmSubsystem {
private:
    CommunicationConfig comm_config_;
    std::atomic<int> current_rssi_;
    std::atomic<bool> jamming_detected_;

    // Приватні методи
    bool InitializeLoRa();
    bool InitializeMesh();
    void MonitorSignalQuality();
    void HandleFrequencyHopping();
    bool DetectJamming();

public:
    explicit CommunicationSystem(std::shared_ptr<ConfigManager> config);

    bool Initialize() override;
    bool Start() override;
    bool Stop() override;
    void Update() override;

    // Методи зв'язку
    bool SendMessage(DroneID target, const std::vector<uint8_t>& data);
    bool BroadcastMessage(const std::vector<uint8_t>& data);
    bool ReceiveMessage(std::vector<uint8_t>& data, DroneID& sender);

    // Управління частотами та потужністю
    bool ChangeFrequency(uint32_t new_frequency);
    bool AdjustPower(int8_t new_power);
    void EnableMesh(bool enable);

    // Моніторинг
    int GetRSSI() const { return current_rssi_.load(); }
    bool IsJammingDetected() const { return jamming_detected_.load(); }
    CommunicationStatus GetStatus() const;
};

// Система позиціонування (UWB)
class PositioningSystem : public SwarmSubsystem {
private:
    std::unordered_map<DroneID, Position3D> drone_positions_;
    Position3D reference_position_;  // позиція лідера як референс
    double position_accuracy_;

    bool InitializeUWB();
    void UpdateRelativePositions();
    bool ValidatePosition(const Position3D& pos);

public:
    explicit PositioningSystem(std::shared_ptr<ConfigManager> config);

    bool Initialize() override;
    bool Start() override;
    bool Stop() override;
    void Update() override;

    // Методи позиціонування
    Position3D GetPosition(DroneID drone_id) const;
    Position3D GetRelativePosition(DroneID drone_id, DroneID reference_id) const;
    bool SetReferencePosition(DroneID reference_drone);
    double GetPositionAccuracy() const { return position_accuracy_; }

    // Валідація формації
    bool IsInFormation(const std::vector<DroneID>& swarm,
                       const FormationPattern& pattern,
                       double tolerance) const;
};

// Система управління формацією
class FormationController : public SwarmSubsystem {
private:
    FormationPattern current_formation_;
    DroneID leader_id_;
    std::vector<DroneID> swarm_members_;
    std::shared_ptr<PositioningSystem> positioning_;

    Position3D CalculateTargetPosition(DroneID drone_id) const;
    Velocity3D CalculateCorrectionVector(DroneID drone_id, const Position3D& target) const;
    bool ValidateFormation() const;

public:
    FormationController(std::shared_ptr<ConfigManager> config,
                        std::shared_ptr<PositioningSystem> positioning);

    bool Initialize() override;
    bool Start() override;
    bool Stop() override;
    void Update() override;

    // Управління формацією
    bool SetFormation(FormationType type);
    bool SetLeader(DroneID new_leader);
    bool AddDrone(DroneID drone_id);
    bool RemoveDrone(DroneID drone_id);

    // Отримання команд управління
    std::vector<std::pair<DroneID, Velocity3D>> GetControlCommands() const;

    // Getters
    FormationType GetCurrentFormationType() const { return current_formation_.type; }
    DroneID GetLeader() const { return leader_id_; }
    const std::vector<DroneID>& GetSwarmMembers() const { return swarm_members_; }
};

// Система управління лідерством
class LeadershipManager : public SwarmSubsystem {
private:
    DroneID current_leader_;
    std::vector<DroneID> candidates_;
    std::unordered_map<DroneID, DroneState> drone_states_;
    Timestamp last_leader_heartbeat_;

    DroneID ElectNewLeader();
    double CalculateLeadershipScore(DroneID drone_id) const;
    bool ValidateLeaderCandidate(DroneID drone_id) const;

public:
    explicit LeadershipManager(std::shared_ptr<ConfigManager> config);

    bool Initialize() override;
    bool Start() override;
    bool Stop() override;
    void Update() override;

    // Управління лідерством
    bool SetLeader(DroneID drone_id);
    DroneID GetCurrentLeader() const { return current_leader_; }
    bool ProcessHeartbeat(DroneID drone_id);
    bool IsLeaderHealthy() const;

    // Управління станом дронів
    bool UpdateDroneState(const DroneState& state);
    DroneState GetDroneState(DroneID drone_id) const;
    std::vector<DroneID> GetHealthyDrones() const;
};

// Головний клас системи управління роем
class SwarmManager {
private:
    std::shared_ptr<ConfigManager> config_;
    std::shared_ptr<CommunicationSystem> comm_system_;
    std::shared_ptr<PositioningSystem> positioning_system_;
    std::shared_ptr<FormationController> formation_controller_;
    std::shared_ptr<LeadershipManager> leadership_manager_;

    DroneID my_drone_id_;
    MissionPhase current_phase_;
    std::atomic<bool> emergency_mode_;

    // Приватні методи
    bool InitializeSubsystems();
    void RunMainLoop();
    void HandleEmergency();
    void ProcessCommands();
    void UpdateTelemetry();

public:
    explicit SwarmManager(const std::string& config_path, DroneID drone_id);
    ~SwarmManager();

    // Основні методи життєвого циклу
    bool Initialize();
    bool Start();
    bool Stop();
    bool IsHealthy() const;

    // Управління місією
    bool StartMission(FormationType formation);
    bool StopMission();
    bool EmergencyStop();
    MissionPhase GetCurrentPhase() const { return current_phase_; }

    // Інтерфейс для оператора
    bool PairDrone(DroneID drone_id);
    bool SetVideoSource(DroneID drone_id);
    bool ChangeFormation(FormationType new_formation);

    // Getters для моніторингу
    std::vector<DroneState> GetSwarmStatus() const;
    CommunicationStatus GetCommunicationStatus() const;
    FormationType getCurrentFormation() const;
};

} // namespace SwarmSystem//
// Created by yv on 22.09.2025.
//

#ifndef DRONESWARMLITE_SWARMSYSTEM_H
#define DRONESWARMLITE_SWARMSYSTEM_H

#endif //DRONESWARMLITE_SWARMSYSTEM_H
