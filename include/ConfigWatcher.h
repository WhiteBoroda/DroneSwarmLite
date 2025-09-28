// include/ConfigWatcher.h
// Система горячего обновления конфигурации без перезапуска
// Критически важно для боевых дронов - нельзя терять время на рестарт
// 🇺🇦 Slava Ukraini! 🇺🇦

#pragma once

#include "ConfigManager.h"
#include "SwarmTypes.h"
#include <thread>
#include <atomic>
#include <functional>
#include <filesystem>
#include <chrono>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <vector>

namespace SwarmSystem {

// Forward declarations
    namespace SwarmControl {
        class CommunicationManager;
    }
    class CommunicationManager;
    class UWBManager;
    class FormationController;
    class PowerManager;
    class SafetyManager;

// Callback types for configuration changes
    using ConfigChangeCallback = std::function<bool(const std::string& section, const std::string& key, const std::string& old_value, const std::string& new_value)>;
    using SystemRestartCallback = std::function<void(const std::string& reason)>;

// Configuration change severity
    enum class ConfigChangeSeverity {
        HOT_RELOAD = 0,        // Can apply immediately without restart
        COMPONENT_RESTART,     // Requires restart of specific component
        SYSTEM_RESTART,        // Requires full system restart
        MISSION_CRITICAL       // Requires mission abort
    };

// Configuration change descriptor
    struct ConfigChange {
        std::string section;
        std::string key;
        std::string old_value;
        std::string new_value;
        ConfigChangeSeverity severity;
        std::string description;
        std::chrono::steady_clock::time_point timestamp;

        ConfigChange();
    };

// Base class for configuration section handlers
    class ConfigSectionHandler {
    public:
        virtual ~ConfigSectionHandler() = default;
        virtual bool CanHandleHotReload(const std::string& key) const = 0;
        virtual bool ApplyConfigChange(const std::string& key, const std::string& new_value) = 0;
        virtual ConfigChangeSeverity GetChangeSeverity(const std::string& key) const = 0;
        virtual std::string GetSectionName() const = 0;
    };

// LoRa configuration handler
    class LoRaConfigHandler : public ConfigSectionHandler {
    private:
        std::weak_ptr<SwarmControl::CommunicationManager> comm_manager_;

        bool updateFrequencyList(const std::string& freq_list,
                                 std::shared_ptr<SwarmControl::CommunicationManager> comm_mgr);

    public:
        explicit LoRaConfigHandler(std::shared_ptr<SwarmControl::CommunicationManager> comm_mgr);

        bool CanHandleHotReload(const std::string& key) const override;
        bool ApplyConfigChange(const std::string& key, const std::string& new_value) override;
        ConfigChangeSeverity GetChangeSeverity(const std::string& key) const override;
        std::string GetSectionName() const override;
    };

// UWB configuration handler
    class UWBConfigHandler : public ConfigSectionHandler {
    private:
        std::weak_ptr<UWBManager> uwb_manager_;

    public:
        explicit UWBConfigHandler(std::shared_ptr<UWBManager> uwb_mgr);

        bool CanHandleHotReload(const std::string& key) const override;
        bool ApplyConfigChange(const std::string& key, const std::string& new_value) override;
        ConfigChangeSeverity GetChangeSeverity(const std::string& key) const override;
        std::string GetSectionName() const override;
    };

// Security configuration handler
    class SecurityConfigHandler : public ConfigSectionHandler {
    private:
        std::weak_ptr<CryptoManager> crypto_manager_;

        bool updateLogLevel(const std::string& level);

    public:
        explicit SecurityConfigHandler(std::shared_ptr<CryptoManager> crypto_mgr);

        bool CanHandleHotReload(const std::string& key) const override;
        bool ApplyConfigChange(const std::string& key, const std::string& new_value) override;
        ConfigChangeSeverity GetChangeSeverity(const std::string& key) const override;
        std::string GetSectionName() const override;
    };

// Main configuration watcher class
    class ConfigWatcher {
    private:
        std::shared_ptr<ConfigManager> config_manager_;
        std::string config_file_path_;
        std::atomic<bool> watching_;
        std::thread watch_thread_;

        std::shared_ptr<CommunicationManager> communication_manager_;
        std::shared_ptr<UWBManager> uwb_manager_;
        std::shared_ptr<FormationController> formation_controller_;
        std::shared_ptr<PowerManager> power_manager_;
        std::shared_ptr<SafetyManager> safety_manager_;

        // File monitoring
        std::filesystem::file_time_type last_write_time_;
        std::chrono::milliseconds poll_interval_;

        // Change handlers
        std::unordered_map<std::string, std::unique_ptr<ConfigSectionHandler>> section_handlers_;
        ConfigChangeCallback global_change_callback_;
        SystemRestartCallback restart_callback_;

        // Change tracking
        std::vector<ConfigChange> pending_changes_;
        mutable std::mutex changes_mutex_;

        // Configuration snapshots for diff detection
        std::unordered_map<std::string, std::string> config_snapshot_;

        // Private methods
        void watchLoop();
        bool hasFileChanged();
        void updateLastWriteTime();
        bool processConfigurationChanges();
        std::vector<ConfigChange> detectConfigChanges();
        std::unordered_map<std::string, std::string> createCurrentSnapshot();
        void createConfigSnapshot();
        ConfigChangeSeverity determineChangeSeverity(const ConfigChange& change);
        std::string generateChangeDescription(const ConfigChange& change);
        bool applyConfigChange(const ConfigChange& change);
        bool applyLoRaConfigChange(const ConfigChange& change);
        bool applyELRSConfigChange(const ConfigChange& change);
        bool applyUWBConfigChange(const ConfigChange& change);
        bool applyMeshConfigChange(const ConfigChange& change);
        bool applyFormationConfigChange(const ConfigChange& change);
        bool applyPowerConfigChange(const ConfigChange& change);
        bool applySafetyConfigChange(const ConfigChange& change);
        std::vector<uint32_t> parseFrequencyList(const std::string& freq_string);
        UWBRangingMode parseRangingMode(const std::string& mode_string);
        ConfigChangeSeverity determineChangeSeverity(const ConfigChange& change);
        std::string generateChangeDescription(const ConfigChange& change);
        std::function<void(const std::string&)> restart_callback_;

    public:
        explicit ConfigWatcher(std::shared_ptr<ConfigManager> config_mgr,
                               const std::string& config_path,
                               std::chrono::milliseconds poll_interval = std::chrono::milliseconds(1000));
        ~ConfigWatcher();

        // Lifecycle management
        bool start();
        void stop();

        // Handler registration
        void registerSectionHandler(std::unique_ptr<ConfigSectionHandler> handler);
        void setGlobalChangeCallback(ConfigChangeCallback callback);
        void setSystemRestartCallback(SystemRestartCallback callback);

        // Manual reload trigger
        bool triggerReload();

        // Status and diagnostics
        std::vector<ConfigChange> getPendingChanges() const;
        bool isWatching() const;
        std::chrono::system_clock::time_point getLastUpdateTime() const;

        void registerCommunicationManager(std::shared_ptr<CommunicationManager> comm_mgr);
        void registerUWBManager(std::shared_ptr<UWBManager> uwb_mgr);
        void registerFormationController(std::shared_ptr<FormationController> form_ctrl);
        void registerPowerManager(std::shared_ptr<PowerManager> power_mgr);
        void registerSafetyManager(std::shared_ptr<SafetyManager> safety_mgr);
        void registerRestartCallback(std::function<void(const std::string&)> callback);
    };

} // namespace SwarmSystem