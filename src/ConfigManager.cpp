// src/ConfigManager.cpp
// Менеджер конфигурации для системы управления роем
// 🇺🇦 Slava Ukraini! 🇺🇦

#include "../include/ConfigManager.h"
#include "../include/SwarmTypes.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>
#include "C:/git/vcpkg/installed/x64-windows/include/yaml-cpp/yaml.h"


namespace SwarmSystem {

    ConfigManager::ConfigManager(const std::string& config_path)
            : config_path_(config_path)
            , config_loaded_(false)
            , last_modified_time_(0) {

        std::cout << "📋 Initializing ConfigManager with: " << config_path_ << std::endl;
        LoadConfig();
    }

    ConfigManager::~ConfigManager() {
        std::cout << "📋 ConfigManager destroyed" << std::endl;
    }

    bool ConfigManager::LoadConfig() {
        std::cout << "📥 Loading configuration from: " << config_path_ << std::endl;

        try {
            // Check if file exists and is readable
            std::ifstream config_file(config_path_);
            if (!config_file.good()) {
                std::cerr << "❌ Configuration file not found or not readable: " << config_path_ << std::endl;
                return CreateDefaultConfig();
            }
            config_file.close();

            // Load YAML configuration
            config_ = YAML::LoadFile(config_path_);

            // Validate configuration
            if (!ValidateConfiguration()) {
                std::cerr << "❌ Configuration validation failed!" << std::endl;
                return false;
            }

            config_loaded_ = true;
            UpdateLastModified();

            std::cout << "✅ Configuration loaded and validated successfully" << std::endl;
            PrintConfigurationSummary();
            return true;

        } catch (const YAML::Exception& e) {
            std::cerr << "❌ YAML parsing error: " << e.what() << std::endl;
            std::cerr << "   Creating default configuration..." << std::endl;
            return CreateDefaultConfig();
        } catch (const std::exception& e) {
            std::cerr << "❌ Error loading configuration: " << e.what() << std::endl;
            return false;
        }
    }

    bool ConfigManager::ReloadConfig() {
        std::cout << "🔄 Reloading configuration..." << std::endl;

        // Check if file was modified since last load
        if (!HasConfigurationChanged()) {
            std::cout << "ℹ️ Configuration unchanged, skipping reload" << std::endl;
            return true;
        }

        return LoadConfig();
    }

    bool ConfigManager::SaveConfig() const {
        if (!config_loaded_) {
            std::cerr << "❌ Cannot save: configuration not loaded" << std::endl;
            return false;
        }

        std::cout << "💾 Saving configuration to: " << config_path_ << std::endl;

        try {
            // Create backup of existing config
            std::string backup_path = config_path_ + ".backup";
            std::ifstream src(config_path_, std::ios::binary);
            std::ofstream dst(backup_path, std::ios::binary);
            dst << src.rdbuf();
            src.close();
            dst.close();

            // Write new configuration
            std::ofstream fout(config_path_);
            fout << "# Swarm Control System Configuration" << std::endl;
            fout << "# 🇺🇦 Generated automatically - Slava Ukraini! 🇺🇦" << std::endl;
            fout << std::endl;
            fout << config_;
            fout.close();

            std::cout << "✅ Configuration saved successfully" << std::endl;
            return true;

        } catch (const std::exception& e) {
            std::cerr << "❌ Error saving configuration: " << e.what() << std::endl;
            return false;
        }
    }

// Template specializations for type-safe access
    template<>
    int ConfigManager::GetValue<int>(const std::string& key, const int& default_value) const {
        if (!config_loaded_) {
            std::cerr << "⚠️ Config not loaded, returning default for key: " << key << std::endl;
            return default_value;
        }

        try {
            YAML::Node node = NavigateToNode(key);
            if (!node || node.IsNull()) {
                return default_value;
            }

            return node.as<int>();

        } catch (const YAML::Exception& e) {
            std::cerr << "⚠️ Error reading int key '" << key << "': " << e.what() << std::endl;
            return default_value;
        }
    }

    template<>
    double ConfigManager::GetValue<double>(const std::string& key, const double& default_value) const {
        if (!config_loaded_) {
            return default_value;
        }

        try {
            YAML::Node node = NavigateToNode(key);
            if (!node || node.IsNull()) {
                return default_value;
            }

            return node.as<double>();

        } catch (const YAML::Exception& e) {
            std::cerr << "⚠️ Error reading double key '" << key << "': " << e.what() << std::endl;
            return default_value;
        }
    }

    template<>
    std::string ConfigManager::GetValue<std::string>(const std::string& key, const std::string& default_value) const {
        if (!config_loaded_) {
            return default_value;
        }

        try {
            YAML::Node node = NavigateToNode(key);
            if (!node || node.IsNull()) {
                return default_value;
            }

            return node.as<std::string>();

        } catch (const YAML::Exception& e) {
            std::cerr << "⚠️ Error reading string key '" << key << "': " << e.what() << std::endl;
            return default_value;
        }
    }

    template<>
    bool ConfigManager::GetValue<bool>(const std::string& key, const bool& default_value) const {
        if (!config_loaded_) {
            return default_value;
        }

        try {
            YAML::Node node = NavigateToNode(key);
            if (!node || node.IsNull()) {
                return default_value;
            }

            return node.as<bool>();

        } catch (const YAML::Exception& e) {
            std::cerr << "⚠️ Error reading bool key '" << key << "': " << e.what() << std::endl;
            return default_value;
        }
    }

// Specialized methods for combat configuration
    std::vector<uint32_t> ConfigManager::GetLoRaFrequencies() const {
        std::vector<uint32_t> frequencies;

        try {
            auto freq_node = config_["lora"]["primary_frequencies"];
            if (freq_node && freq_node.IsSequence()) {
                for (const auto& freq : freq_node) {
                    uint32_t frequency = freq.as<uint32_t>();
                    frequencies.push_back(frequency);
                }
            }

            // Add backup frequencies if list is empty
            if (frequencies.empty()) {
                std::cout << "⚠️ No LoRa frequencies configured, using combat defaults" << std::endl;
                frequencies = GetCombatLoRaFrequencies();
            }

            std::cout << "📡 Loaded " << frequencies.size() << " LoRa frequencies" << std::endl;

        } catch (const YAML::Exception& e) {
            std::cerr << "❌ Error reading LoRa frequencies: " << e.what() << std::endl;
            std::cout << "🛡️ Using combat-optimized frequencies" << std::endl;
            frequencies = GetCombatLoRaFrequencies();
        }

        return frequencies;
    }

    std::vector<uint32_t> ConfigManager::GetCombatLoRaFrequencies() const {
        // Combat-optimized frequency list - spread across different bands
        // On the front lines, we use whatever works best!
        return {
                // 433 MHz ISM band - good propagation, lower power consumption
                433050000, 433075000, 433100000, 433125000, 433150000,

                // 868 MHz European ISM - good balance of range and bandwidth
                868000000, 868100000, 868300000, 868500000, 868800000,

                // 915 MHz ISM - higher bandwidth, shorter range
                915000000, 915200000, 915400000, 915600000, 915800000,

                // 2.4 GHz - backup frequencies if lower bands jammed
                2400000000, 2420000000, 2440000000, 2460000000, 2480000000
        };
    }

    PowerConfig ConfigManager::GetPowerConfig() const {
        PowerConfig power_config;

        try {
            // On the front - use maximum available power for reliability
            power_config.lora_min_power = GetValue<int>("lora.power_levels.min_power", 0);
            power_config.lora_max_power = GetValue<int>("lora.power_levels.max_power", 34); // 34 dBm = 2.5W
            power_config.lora_adaptive_step = GetValue<int>("lora.power_levels.adaptive_step", 3);

            // Video transmitter - use maximum power when needed
            power_config.video_min_power = GetValue<int>("video.power_levels.min_power", 25);    // 25mW
            power_config.video_max_power = GetValue<int>("video.power_levels.max_power", 2500);  // 2.5W
            power_config.video_default_power = GetValue<int>("video.power_levels.default_power", 200);

            // UWB power settings
            power_config.uwb_power_setting = GetValue<int>("uwb.tx_power", 20); // Max UWB power

            std::cout << "⚡ Combat power config: LoRa " << power_config.lora_min_power
                      << "-" << power_config.lora_max_power << "dBm, Video "
                      << power_config.video_min_power << "-" << power_config.video_max_power << "mW" << std::endl;

        } catch (const std::exception& e) {
            std::cerr << "❌ Error reading power config: " << e.what() << std::endl;

            // Combat defaults - maximum power for survival
            power_config.lora_min_power = 0;
            power_config.lora_max_power = 34;      // 2.5 Watt
            power_config.lora_adaptive_step = 3;
            power_config.video_min_power = 25;
            power_config.video_max_power = 2500;   // 2.5 Watts
            power_config.video_default_power = 500;
            power_config.uwb_power_setting = 20;
        }

        return power_config;
    }

    FormationConfig ConfigManager::GetFormationConfig(FormationType type) const {
        FormationConfig formation_config;
        formation_config.type = type;

        try {
            std::string formation_key;
            switch (type) {
                case FormationType::WEDGE:
                    formation_key = "wedge";
                    break;
                case FormationType::LINE:
                    formation_key = "line";
                    break;
                case FormationType::SQUARE:
                    formation_key = "square";
                    break;
                case FormationType::CIRCLE:
                    formation_key = "circle";
                    break;
                case FormationType::DIAMOND:
                    formation_key = "diamond";
                    break;
                default:
                    formation_key = "wedge";
                    break;
            }

            auto formations_node = config_["formations"];
            auto formation_node = formations_node[formation_key];

            if (!formation_node) {
                std::cout << "⚠️ Formation '" << formation_key << "' not found, creating default" << std::endl;
                return CreateDefaultFormation(type);
            }

            // Read formation positions
            if (formation_node["positions"] && formation_node["positions"].IsSequence()) {
                for (const auto& pos_node : formation_node["positions"]) {
                    if (pos_node.IsSequence() && pos_node.size() >= 3) {
                        Position3D pos(
                                pos_node[0].as<double>(),
                                pos_node[1].as<double>(),
                                pos_node[2].as<double>()
                        );
                        formation_config.positions.push_back(pos);
                    }
                }
            }

            // Read formation parameters
            formation_config.default_spacing = GetValue<double>("formations." + formation_key + ".spacing", 10.0);
            formation_config.vertical_separation = GetValue<double>("formations." + formation_key + ".vertical_separation", 0.0);
            formation_config.formation_speed = GetValue<double>("formations." + formation_key + ".speed", 15.0);
            formation_config.position_tolerance = GetValue<double>("formations." + formation_key + ".tolerance", 3.0);

            // Combat-specific parameters
            formation_config.evasion_capability = GetValue<bool>("formations." + formation_key + ".evasion_capable", true);
            formation_config.attack_formation = GetValue<bool>("formations." + formation_key + ".attack_formation", false);

            std::cout << "🛫 Loaded formation '" << formation_key << "' with "
                      << formation_config.positions.size() << " positions" << std::endl;

        } catch (const std::exception& e) {
            std::cerr << "❌ Error reading formation config: " << e.what() << std::endl;
            formation_config = CreateDefaultFormation(type);
        }

        return formation_config;
    }

    CommunicationConfig ConfigManager::GetCommunicationConfig() const {
        CommunicationConfig comm_config;

        try {
            // Load LoRa frequencies
            comm_config.lora_frequencies = GetLoRaFrequencies();
            comm_config.current_frequency = comm_config.lora_frequencies.empty() ?
                                            868100000 : comm_config.lora_frequencies[0];

            // Power configuration
            auto power_config = GetPowerConfig();
            comm_config.lora_power_level = power_config.lora_max_power; // Start with max power for reliability
            comm_config.video_power_level = power_config.video_default_power;

            // Network settings
            comm_config.mesh_enabled = GetValue<bool>("lora.mesh_config.enable_mesh", true);
            comm_config.frequency_hopping_enabled = GetValue<bool>("lora.frequency_hopping.enable", true);
            comm_config.hop_interval = GetValue<int>("lora.frequency_hopping.hop_interval", 10000);

            // Thresholds for combat conditions
            comm_config.rssi_threshold = GetValue<double>("lora.frequency_hopping.noise_threshold", -85.0);
            comm_config.noise_threshold = GetValue<double>("lora.frequency_hopping.noise_threshold", -80.0);
            comm_config.interference_threshold = GetValue<double>("lora.frequency_hopping.interference_threshold", -75.0);

            // Encryption settings
            comm_config.encryption_enabled = GetValue<bool>("security.enable_encryption", true);
            comm_config.shared_secret = GetValue<std::string>("security.shared_secret", "SlavaUkraini2024!");

            // Combat-specific timeouts (shorter for responsiveness)
            comm_config.communication_timeout = GetValue<int>("communication.timeout_ms", 5000);
            comm_config.heartbeat_interval = GetValue<int>("communication.heartbeat_interval_ms", 1000);
            comm_config.retry_count = GetValue<int>("communication.max_retries", 5);

            std::cout << "📡 Communication config loaded: " << comm_config.lora_frequencies.size()
                      << " frequencies, mesh=" << (comm_config.mesh_enabled ? "ON" : "OFF")
                      << ", encryption=" << (comm_config.encryption_enabled ? "ON" : "OFF") << std::endl;

        } catch (const std::exception& e) {
            std::cerr << "❌ Error reading communication config: " << e.what() << std::endl;

            // Combat defaults
            comm_config.lora_frequencies = GetCombatLoRaFrequencies();
            comm_config.current_frequency = 868100000;
            comm_config.lora_power_level = 20;  // High power
            comm_config.video_power_level = 1000; // 1W video
            comm_config.mesh_enabled = true;
            comm_config.frequency_hopping_enabled = true;
            comm_config.hop_interval = 5000;    // Fast hopping
            comm_config.rssi_threshold = -85.0;
            comm_config.noise_threshold = -80.0;
            comm_config.interference_threshold = -75.0;
            comm_config.encryption_enabled = true;
            comm_config.shared_secret = "SlavaUkraini2024!";
            comm_config.communication_timeout = 3000;
            comm_config.heartbeat_interval = 500;
            comm_config.retry_count = 8;
        }

        return comm_config;
    }

    MissionConfig ConfigManager::GetMissionConfig() const {
        MissionConfig mission_config;

        try {
            // Combat mission parameters
            mission_config.max_drones = GetValue<int>("system.max_drones_mvp", 10);
            mission_config.mission_timeout = GetValue<int>("mission.mission_timeout", 1800); // 30 minutes
            mission_config.failsafe_mode = GetValue<std::string>("failsafe.action", "self_destruct");
            mission_config.emergency_timeout = GetValue<int>("failsafe.emergency_timeout", 300); // 5 minutes

            // Flight parameters
            mission_config.max_altitude = GetValue<double>("flight.max_altitude", 300.0);
            mission_config.max_speed = GetValue<double>("flight.max_speed", 30.0);
            mission_config.formation_altitude = GetValue<double>("flight.formation_height", 100.0);
            mission_config.safety_altitude = GetValue<double>("flight.safety_height", 50.0);

            // Targeting parameters
            mission_config.terminal_guidance_distance = GetValue<double>("mission.target_approach.terminal_guidance_distance", 50.0);
            mission_config.approach_speed = GetValue<double>("mission.target_approach.approach_speed", 10.0);
            mission_config.final_approach_distance = GetValue<double>("mission.target_approach.final_approach_distance", 100.0);

            // Combat-specific settings
            mission_config.enable_self_destruct = GetValue<bool>("failsafe.enable_self_destruct", true);
            mission_config.operator_override = GetValue<bool>("operator.manual_override", false); // No override in combat
            mission_config.auto_evasion = GetValue<bool>("mission.enable_auto_evasion", true);

            std::cout << "🎯 Mission config: max_drones=" << mission_config.max_drones
                      << ", timeout=" << mission_config.mission_timeout << "s"
                      << ", self_destruct=" << (mission_config.enable_self_destruct ? "ENABLED" : "DISABLED") << std::endl;

        } catch (const std::exception& e) {
            std::cerr << "❌ Error reading mission config: " << e.what() << std::endl;

            // Combat defaults - aggressive settings for effectiveness
            mission_config.max_drones = 10;
            mission_config.mission_timeout = 1200;  // 20 minutes
            mission_config.failsafe_mode = "self_destruct";
            mission_config.emergency_timeout = 180; // 3 minutes
            mission_config.max_altitude = 300.0;
            mission_config.max_speed = 35.0;        // Fast approach
            mission_config.formation_altitude = 80.0;
            mission_config.safety_altitude = 30.0;
            mission_config.terminal_guidance_distance = 30.0;
            mission_config.approach_speed = 15.0;
            mission_config.final_approach_distance = 75.0;
            mission_config.enable_self_destruct = true;
            mission_config.operator_override = false;
            mission_config.auto_evasion = true;
        }

        return mission_config;
    }

// Private helper methods
    YAML::Node ConfigManager::NavigateToNode(const std::string& key) const {
        std::istringstream ss(key);
        std::string item;
        YAML::Node node = config_;

        while (std::getline(ss, item, '.')) {
            if (!node[item]) {
                return YAML::Node(); // Return null node
            }
            node = node[item];
        }

        return node;
    }

    bool ConfigManager::ValidateConfiguration() const {
        std::cout << "🔍 Validating configuration..." << std::endl;

        bool is_valid = true;

        try {
            // Validate system section
            if (!config_["system"]) {
                std::cerr << "❌ Missing required 'system' section" << std::endl;
                is_valid = false;
            }

            // Validate LoRa section
            if (!config_["lora"]) {
                std::cerr << "❌ Missing required 'lora' section" << std::endl;
                is_valid = false;
            } else {
                auto frequencies = GetLoRaFrequencies();
                if (frequencies.empty()) {
                    std::cerr << "❌ No LoRa frequencies configured" << std::endl;
                    is_valid = false;
                }

                // For combat, we allow any frequency that hardware supports
                for (auto freq : frequencies) {
                    if (freq < 100000000 || freq > 6000000000) { // 100MHz to 6GHz - hardware limits
                        std::cerr << "⚠️ Warning: Frequency " << freq << " Hz may exceed hardware limits" << std::endl;
                    }
                }
            }

            // Validate formations
            if (!config_["formations"]) {
                std::cerr << "❌ Missing required 'formations' section" << std::endl;
                is_valid = false;
            }

            // Validate power levels for combat use
            auto power_config = GetPowerConfig();
            if (power_config.lora_max_power < 10) {
                std::cout << "⚠️ Warning: Low LoRa power may reduce range in combat conditions" << std::endl;
            }

            // Validate mission parameters
            int max_drones = GetValue<int>("system.max_drones_mvp", 0);
            if (max_drones <= 0 || max_drones > 100) {
                std::cerr << "❌ Invalid max_drones value: " << max_drones << std::endl;
                is_valid = false;
            }

            if (is_valid) {
                std::cout << "✅ Configuration validation passed" << std::endl;
            } else {
                std::cout << "❌ Configuration validation failed" << std::endl;
            }

        } catch (const std::exception& e) {
            std::cerr << "❌ Exception during validation: " << e.what() << std::endl;
            is_valid = false;
        }

        return is_valid;
    }

    bool ConfigManager::CreateDefaultConfig() {
        std::cout << "🏗️ Creating default combat configuration..." << std::endl;

        try {
            // Create default YAML configuration optimized for combat
            config_["system"]["version"] = "1.0.0-COMBAT";
            config_["system"]["max_drones_mvp"] = 10;
            config_["system"]["operator_location"] = "ukraine";

            // LoRa configuration - combat optimized
            auto combat_frequencies = GetCombatLoRaFrequencies();
            for (size_t i = 0; i < combat_frequencies.size(); ++i) {
                config_["lora"]["primary_frequencies"][i] = combat_frequencies[i];
            }

            config_["lora"]["power_levels"]["min_power"] = 0;
            config_["lora"]["power_levels"]["max_power"] = 30;  // 1W
            config_["lora"]["power_levels"]["adaptive_step"] = 3;

            config_["lora"]["mesh_config"]["enable_mesh"] = true;
            config_["lora"]["mesh_config"]["hop_limit"] = 5;

            config_["lora"]["frequency_hopping"]["enable"] = true;
            config_["lora"]["frequency_hopping"]["hop_interval"] = 5000;
            config_["lora"]["frequency_hopping"]["noise_threshold"] = -80;

            // Combat formations
            config_["formations"]["wedge"]["spacing"] = 10.0;
            config_["formations"]["wedge"]["positions"][0] = std::vector<double>{0, 0, 0};
            config_["formations"]["wedge"]["positions"][1] = std::vector<double>{-10, -10, 0};
            config_["formations"]["wedge"]["positions"][2] = std::vector<double>{10, -10, 0};
            config_["formations"]["wedge"]["positions"][3] = std::vector<double>{-15, -20, 0};
            config_["formations"]["wedge"]["positions"][4] = std::vector<double>{15, -20, 0};

            // Security settings
            config_["security"]["enable_encryption"] = true;
            config_["security"]["shared_secret"] = "SlavaUkraini2024!";

            // Mission parameters
            config_["mission"]["mission_timeout"] = 1800;
            config_["mission"]["target_approach"]["final_approach_distance"] = 100.0;
            config_["mission"]["target_approach"]["approach_speed"] = 15.0;
            config_["mission"]["target_approach"]["terminal_guidance_distance"] = 50.0;

            // Failsafe - critical for combat
            config_["failsafe"]["action"] = "self_destruct";
            config_["failsafe"]["emergency_timeout"] = 300;
            config_["failsafe"]["enable_self_destruct"] = true;

            // Flight parameters
            config_["flight"]["max_speed"] = 30.0;
            config_["flight"]["formation_height"] = 100.0;
            config_["flight"]["safety_height"] = 50.0;
            config_["flight"]["max_altitude"] = 300.0;

            // Video settings
            config_["video"]["power_levels"]["min_power"] = 25;
            config_["video"]["power_levels"]["max_power"] = 2500;
            config_["video"]["power_levels"]["default_power"] = 500;

            config_loaded_ = true;

            // Save the default configuration
            if (SaveConfig()) {
                std::cout << "✅ Default combat configuration created and saved" << std::endl;
                return true;
            } else {
                std::cout << "⚠️ Default configuration created but not saved to disk" << std::endl;
                return true; // Still usable in memory
            }

        } catch (const std::exception& e) {
            std::cerr << "❌ Error creating default configuration: " << e.what() << std::endl;
            return false;
        }
    }

    FormationConfig ConfigManager::CreateDefaultFormation(FormationType type) const {
        FormationConfig formation_config;
        formation_config.type = type;
        formation_config.default_spacing = 10.0;
        formation_config.vertical_separation = 0.0;
        formation_config.formation_speed = 15.0;
        formation_config.position_tolerance = 3.0;
        formation_config.evasion_capability = true;
        formation_config.attack_formation = (type == FormationType::WEDGE);

        // Create positions based on formation type
        switch (type) {
            case FormationType::WEDGE:
                formation_config.positions = {
                        Position3D(0, 0, 0),      // Leader
                        Position3D(-10, -10, 0),  // Left wing
                        Position3D(10, -10, 0),   // Right wing
                        Position3D(-15, -20, 0),  // Left rear
                        Position3D(15, -20, 0),   // Right rear
                        Position3D(-20, -30, 0),  // Left far
                        Position3D(20, -30, 0),   // Right far
                };
                break;

            case FormationType::LINE:
                formation_config.positions = {
                        Position3D(0, 0, 0),      // Center
                        Position3D(0, -10, 0),    // Left
                        Position3D(0, 10, 0),     // Right
                        Position3D(0, -20, 0),    // Far left
                        Position3D(0, 20, 0),     // Far right
                };
                break;

            case FormationType::SQUARE:
                formation_config.positions = {
                        Position3D(0, 0, 0),      // Center
                        Position3D(-10, -10, 0),  // Front left
                        Position3D(10, -10, 0),   // Front right
                        Position3D(-10, 10, 0),   // Rear left
                        Position3D(10, 10, 0),    // Rear right
                };
                break;

            default:
                // Default to wedge
                formation_config.positions = {
                        Position3D(0, 0, 0),
                        Position3D(-10, -10, 0),
                        Position3D(10, -10, 0),
                };
                break;
        }

        return formation_config;
    }

    void ConfigManager::PrintConfigurationSummary() const {
        if (!config_loaded_) {
            std::cout << "⚠️ Configuration not loaded" << std::endl;
            return;
        }

        std::cout << "\n📋 === CONFIGURATION SUMMARY ===" << std::endl;
        std::cout << "🎯 System version: " << GetValue<std::string>("system.version", "unknown") << std::endl;
        std::cout << "🚁 Max drones: " << GetValue<int>("system.max_drones_mvp", 0) << std::endl;

        auto frequencies = GetLoRaFrequencies();
        std::cout << "📡 LoRa frequencies: " << frequencies.size() << " configured" << std::endl;

        auto power_config = GetPowerConfig();
        std::cout << "⚡ Power range: " << power_config.lora_min_power << "-"
                  << power_config.lora_max_power << " dBm" << std::endl;

        std::cout << "🔒 Encryption: " << (GetValue<bool>("security.enable_encryption", false) ? "ENABLED" : "DISABLED") << std::endl;
        std::cout << "💥 Self-destruct: " << (GetValue<bool>("failsafe.enable_self_destruct", false) ? "ARMED" : "DISABLED") << std::endl;
        std::cout << "🇺🇦 SLAVA UKRAINI! 🇺🇦" << std::endl;
        std::cout << "================================\n" << std::endl;
    }

    bool ConfigManager::HasConfigurationChanged() const {
        // Check file modification time
        // TODO: Implement file system check for modification time
        return true; // For now, always assume changed
    }

    void ConfigManager::UpdateLastModified() {
        last_modified_time_ = std::time(nullptr);
    }

} // namespace SwarmSystem