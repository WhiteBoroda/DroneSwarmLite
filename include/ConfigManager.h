// include/ConfigManager.h
// Менеджер конфигурации для системы управления роем
// 🇺🇦 Slava Ukraini! 🇺🇦

#pragma once

#include "SwarmTypes.h"
#include <string>
#include <vector>
#include <memory>
#include <yaml-cpp/yaml.h>

namespace SwarmSystem {

// Power configuration for combat operations
    struct PowerConfig {
        int lora_min_power;        // dBm
        int lora_max_power;        // dBm - no regulatory limits in combat!
        int lora_adaptive_step;    // dBm step for adaptive power

        int video_min_power;       // mW
        int video_max_power;       // mW - use maximum available power
        int video_default_power;   // mW

        int uwb_power_setting;     // UWB power level (0-33)

        PowerConfig() : lora_min_power(0), lora_max_power(30), lora_adaptive_step(2),
                        video_min_power(25), video_max_power(2500), video_default_power(200),
                        uwb_power_setting(15) {}
    };

// Formation configuration
    struct FormationConfig {
        FormationType type;
        std::vector<Position3D> positions;  // Relative positions in formation
        double default_spacing;             // meters between drones
        double vertical_separation;         // meters between altitude layers
        double formation_speed;             // m/s - formation flight speed
        double position_tolerance;          // meters - acceptable position error

        // Combat-specific parameters
        bool evasion_capability;           // Can perform evasive maneuvers
        bool attack_formation;             // Optimized for attack approach

        FormationConfig() : type(FormationType::WEDGE), default_spacing(10.0),
                            vertical_separation(0.0), formation_speed(15.0),
                            position_tolerance(3.0), evasion_capability(true),
                            attack_formation(false) {}
    };

// Communication configuration
    struct CommunicationConfig {
        std::vector<uint32_t> lora_frequencies;  // Available frequencies (Hz)
        uint32_t current_frequency;              // Currently active frequency
        int lora_power_level;                    // Current power level (dBm)
        int video_power_level;                   // Video transmitter power (mW)

        // Network settings
        bool mesh_enabled;                       // Enable mesh networking
        bool frequency_hopping_enabled;          // Enable adaptive frequency hopping
        int hop_interval;                        // ms between frequency changes

        // Signal quality thresholds
        double rssi_threshold;                   // dBm - minimum acceptable RSSI
        double noise_threshold;                  // dBm - noise floor threshold
        double interference_threshold;           // dBm - interference detection

        // Security settings
        bool encryption_enabled;                 // Enable message encryption
        std::string shared_secret;               // Pre-shared key for encryption

        // Timing parameters
        int communication_timeout;               // ms - communication timeout
        int heartbeat_interval;                  // ms - heartbeat interval
        int retry_count;                        // maximum retransmissions

        CommunicationConfig() : current_frequency(868100000), lora_power_level(14),
                                video_power_level(200), mesh_enabled(true),
                                frequency_hopping_enabled(true), hop_interval(10000),
                                rssi_threshold(-90.0), noise_threshold(-85.0),
                                interference_threshold(-80.0), encryption_enabled(true),
                                shared_secret("SlavaUkraini2024!"), communication_timeout(5000),
                                heartbeat_interval(1000), retry_count(3) {}
    };

// Mission configuration
    struct MissionConfig {
        int max_drones;                         // Maximum drones in swarm
        int mission_timeout;                    // seconds - total mission timeout
        std::string failsafe_mode;              // "self_destruct", "return_home", etc.
        int emergency_timeout;                  // seconds - emergency action timeout

        // Flight parameters
        double max_altitude;                    // meters - maximum flight altitude
        double max_speed;                       // m/s - maximum flight speed
        double formation_altitude;              // meters - default formation altitude
        double safety_altitude;                 // meters - minimum safe altitude

        // Targeting parameters
        double terminal_guidance_distance;      // meters - when to switch to terminal guidance
        double approach_speed;                  // m/s - target approach speed
        double final_approach_distance;         // meters - final approach distance

        // Combat-specific settings
        bool enable_self_destruct;              // Enable self-destruct capability
        bool operator_override;                 // Allow operator override (usually false in combat)
        bool auto_evasion;                      // Enable automatic evasion maneuvers

        MissionConfig() : max_drones(10), mission_timeout(1800), failsafe_mode("self_destruct"),
                          emergency_timeout(300), max_altitude(300.0), max_speed(30.0),
                          formation_altitude(100.0), safety_altitude(50.0),
                          terminal_guidance_distance(50.0), approach_speed(10.0),
                          final_approach_distance(100.0), enable_self_destruct(true),
                          operator_override(false), auto_evasion(true) {}
    };

// Main configuration manager class
    class ConfigManager {
    public:
        explicit ConfigManager(const std::string& config_path);
        ~ConfigManager();

        // Configuration file management
        bool LoadConfig();
        bool ReloadConfig();
        bool SaveConfig() const;
        bool IsConfigLoaded() const { return config_loaded_; }

        // Generic value access with type safety
        template<typename T>
        T GetValue(const std::string& key, const T& default_value = T{}) const;

        template<typename T>
        bool SetValue(const std::string& key, const T& value);

        // Specialized configuration getters
        std::vector<uint32_t> GetLoRaFrequencies() const;
        std::vector<uint32_t> GetCombatLoRaFrequencies() const; // Combat-optimized frequencies
        PowerConfig GetPowerConfig() const;
        FormationConfig GetFormationConfig(FormationType type) const;
        CommunicationConfig GetCommunicationConfig() const;
        MissionConfig GetMissionConfig() const;

        // Configuration validation and diagnostics
        bool ValidateConfiguration() const;
        void PrintConfigurationSummary() const;
        bool CreateDefaultConfig();

        // Runtime configuration updates
        bool UpdateLoRaFrequency(uint32_t new_frequency);
        bool UpdatePowerLevel(int new_power_level);
        bool UpdateFormationSpacing(double new_spacing);

    private:
        std::string config_path_;
        YAML::Node config_;
        bool config_loaded_;
        std::time_t last_modified_time_;

        // Private helper methods
        YAML::Node NavigateToNode(const std::string& key) const;
        FormationConfig CreateDefaultFormation(FormationType type) const;
        bool HasConfigurationChanged() const;
        void UpdateLastModified();

        // Template method implementations (defined in source file)
        // Note: Template specializations are explicitly defined for:
        // int, double, std::string, bool
    };

// Template specializations (declared here, implemented in .cpp)
    template<> int ConfigManager::GetValue<int>(const std::string& key, const int& default_value) const;
    template<> double ConfigManager::GetValue<double>(const std::string& key, const double& default_value) const;
    template<> std::string ConfigManager::GetValue<std::string>(const std::string& key, const std::string& default_value) const;
    template<> bool ConfigManager::GetValue<bool>(const std::string& key, const bool& default_value) const;

// Utility functions for configuration management
    namespace ConfigUtils {
        bool IsValidLoRaFrequency(uint32_t frequency);
        bool IsValidPowerLevel(int power_dbm, int min_power = -10, int max_power = 30);
        std::string FormatFrequency(uint32_t frequency_hz);
        std::vector<uint32_t> ParseFrequencyList(const std::string& frequency_string);
        bool BackupConfigFile(const std::string& config_path);
        bool RestoreConfigFile(const std::string& config_path);
    }

} // namespace SwarmSystem