// src/ConfigWatcher.cpp
// Реализация системы горячего обновления конфигурации
// 🇺🇦 Slava Ukraini! 🇺🇦

#include "../include/ConfigWatcher.h"
#include "../include/CommunicationManager.h"
#include "../include/UWBManager.h"
#include "../include/CryptoManager.h"
#include <iostream>
#include <sstream>
#include <algorithm>

namespace SwarmSystem {

// ConfigChange implementation
    ConfigChange::ConfigChange()
            : severity(ConfigChangeSeverity::HOT_RELOAD)
            , timestamp(std::chrono::steady_clock::now()) {
    }

// LoRaConfigHandler implementation
    LoRaConfigHandler::LoRaConfigHandler(std::shared_ptr<SwarmControl::CommunicationManager> comm_mgr)
            : comm_manager_(comm_mgr) {
    }

    bool LoRaConfigHandler::CanHandleHotReload(const std::string& key) const {
        // РЕАЛЬНЫЕ ключи из ConfigManager которые можно менять на лету
        return (key == "primary_frequencies" ||
                key == "power_levels.max_power" ||
                key == "power_levels.min_power" ||
                key == "mesh_config.enable_mesh" ||
                key == "frequency_hopping.enable");
    }

    bool LoRaConfigHandler::ApplyConfigChange(const std::string& key, const std::string& new_value) {
        auto comm_mgr = comm_manager_.lock();
        if (!comm_mgr) {
            std::cerr << "❌ CommunicationManager недоступен" << std::endl;
            return false;
        }

        if (key == "primary_frequencies") {
            return updateFrequencyList(new_value, comm_mgr);
        }
        else if (key == "power_levels.max_power") {
            try {
                int power = std::stoi(new_value);
                return comm_mgr->set_max_power_level(static_cast<int8_t>(power));
            } catch (const std::exception& e) {
                std::cerr << "❌ Ошибка установки мощности: " << e.what() << std::endl;
                return false;
            }
        }
        else if (key == "power_levels.min_power") {
            try {
                int power = std::stoi(new_value);
                return comm_mgr->set_min_power_level(static_cast<int8_t>(power));
            } catch (const std::exception& e) {
                std::cerr << "❌ Ошибка установки мин. мощности: " << e.what() << std::endl;
                return false;
            }
        }
        else if (key == "mesh_config.enable_mesh") {
            bool enabled = (new_value == "true" || new_value == "1");
            return comm_mgr->enable_mesh_networking(enabled);
        }
        else if (key == "frequency_hopping.enable") {
            bool enabled = (new_value == "true" || new_value == "1");
            return comm_mgr->enable_frequency_hopping(enabled);
        }

        return false;
    }

    ConfigChangeSeverity LoRaConfigHandler::GetChangeSeverity(const std::string& key) const {
        if (key == "channel" || key == "bandwidth") {
            return ConfigChangeSeverity::COMPONENT_RESTART;
        }
        return ConfigChangeSeverity::HOT_RELOAD;
    }

    std::string LoRaConfigHandler::GetSectionName() const {
        return "lora";
    }

    bool LoRaConfigHandler::updateFrequencyList(const std::string& frequency_string,
                                                std::shared_ptr<SwarmControl::CommunicationManager> comm_mgr) {
        try {
            std::vector<uint32_t> frequencies;
            std::stringstream ss(frequency_string);
            std::string freq_str;

            // Парсинг частот из строки (разделенных запятыми)
            while (std::getline(ss, freq_str, ',')) {
                // Убираем пробелы
                freq_str.erase(std::remove_if(freq_str.begin(), freq_str.end(), ::isspace), freq_str.end());

                if (!freq_str.empty()) {
                    uint32_t frequency = std::stoul(freq_str);

                    // Валидация частоты
                    if (frequency >= 433000000 && frequency <= 2500000000) {
                        frequencies.push_back(frequency);
                    } else {
                        std::cerr << "❌ Недопустимая частота: " << frequency << " Hz" << std::endl;
                        return false;
                    }
                }
            }

            if (frequencies.empty()) {
                std::cerr << "❌ Пустой список частот" << std::endl;
                return false;
            }

            // ✅ РЕАЛЬНОЕ обновление частот в CommunicationManager
            bool success = comm_mgr->update_frequency_list(frequencies);

            if (success) {
                std::cout << "✅ Обновлен список частот LoRa: " << frequencies.size()
                          << " частот" << std::endl;

                // Вывод всех частот для подтверждения
                for (size_t i = 0; i < frequencies.size(); i++) {
                    double freq_mhz = frequencies[i] / 1000000.0;
                    std::cout << "  Частота " << (i+1) << ": "
                              << std::fixed << std::setprecision(3) << freq_mhz << " MHz" << std::endl;
                }
            }

            return success;

        } catch (const std::exception& e) {
            std::cerr << "❌ Ошибка парсинга частот: " << e.what() << std::endl;
            return false;
        }
    }

// UWBConfigHandler implementation
    UWBConfigHandler::UWBConfigHandler(std::shared_ptr<UWBManager> uwb_mgr)
            : uwb_manager_(uwb_mgr) {
    }

    bool UWBConfigHandler::CanHandleHotReload(const std::string& key) const {
        // РЕАЛЬНЫЕ ключи из GetPowerConfig() которые поддерживаются
        return (key == "tx_power");
    }

    bool UWBConfigHandler::ApplyConfigChange(const std::string& key, const std::string& new_value) {
        auto uwb_mgr = uwb_manager_.lock();
        if (!uwb_mgr) {
            std::cerr << "❌ UWBManager недоступен" << std::endl;
            return false;
        }

        if (key == "tx_power") {
            try {
                int power = std::stoi(new_value);
                return uwb_mgr->SetTxPower(static_cast<uint8_t>(power));
            } catch (const std::exception& e) {
                std::cerr << "❌ Ошибка установки UWB мощности: " << e.what() << std::endl;
                return false;
            }
        }

        return false;
    }

    ConfigChangeSeverity UWBConfigHandler::GetChangeSeverity(const std::string& key) const {
        // UWB изменения обычно безопасны
        return ConfigChangeSeverity::HOT_RELOAD;
    }

    std::string UWBConfigHandler::GetSectionName() const {
        return "uwb";
    }

// SecurityConfigHandler implementation
    SecurityConfigHandler::SecurityConfigHandler(std::shared_ptr<CryptoManager> crypto_mgr)
            : crypto_manager_(crypto_mgr) {
    }

    bool SecurityConfigHandler::updateSharedSecret(const std::string& new_secret) {
        if (new_secret.length() < 32) {
            std::cerr << "❌ Общий секрет слишком короткий (минимум 32 символа)" << std::endl;
            return false;
        }

        try {
            if (auto crypto_mgr = crypto_manager_.lock()) {
                // Обновляем общий секрет
                bool success = crypto_mgr->UpdateSharedSecret(new_secret);

                if (success) {
                    std::cout << "🔐 Общий секрет успешно обновлен" << std::endl;
                } else {
                    std::cerr << "❌ Не удалось обновить общий секрет" << std::endl;
                }

                return success;
            } else {
                std::cerr << "❌ CryptoManager недоступен" << std::endl;
                return false;
            }

        } catch (const std::exception& e) {
            std::cerr << "❌ Ошибка обновления секрета: " << e.what() << std::endl;
            return false;
        }
    }

    bool SecurityConfigHandler::CanHandleHotReload(const std::string& key) const {
        // Безопасность - только некритические изменения
        return false; // Все изменения безопасности критичны!
    }

    bool SecurityConfigHandler::ApplyConfigChange(const std::string& key, const std::string& new_value) {
        // Все изменения безопасности требуют перезапуска
        std::cout << "⚠️ Изменения безопасности требуют перезапуска системы!" << std::endl;
        return false;
    }

    ConfigChangeSeverity SecurityConfigHandler::GetChangeSeverity(const std::string& key) const {
        if (key == "enable_encryption" || key == "shared_secret") {
            return ConfigChangeSeverity::MISSION_CRITICAL;
        }
        return ConfigChangeSeverity::SYSTEM_RESTART;
    }

    std::string SecurityConfigHandler::GetSectionName() const {
        return "security";
    }

    bool SecurityConfigHandler::updateLogLevel(const std::string& level) {
        try {
            // Валидация уровня логирования
            std::vector<std::string> valid_levels = {"DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"};

            auto it = std::find(valid_levels.begin(), valid_levels.end(), level);
            if (it == valid_levels.end()) {
                std::cerr << "❌ Недопустимый уровень логирования: " << level << std::endl;
                return false;
            }

            // ✅ РЕАЛЬНОЕ обновление уровня логирования
            if (auto crypto_mgr = crypto_manager_.lock()) {
                // Устанавливаем уровень логирования безопасности
                crypto_mgr->SetSecurityLogLevel(level);

                std::cout << "🔒 Уровень логирования безопасности установлен: " << level << std::endl;
                return true;
            } else {
                std::cerr << "❌ CryptoManager недоступен для установки уровня логирования" << std::endl;
                return false;
            }

        } catch (const std::exception& e) {
            std::cerr << "❌ Ошибка установки уровня логирования: " << e.what() << std::endl;
            return false;
        }
    }

// ConfigWatcher implementation
    ConfigWatcher::ConfigWatcher(std::shared_ptr<ConfigManager> config_mgr,
                                 const std::string& config_path,
                                 std::chrono::milliseconds poll_interval)
            : config_manager_(config_mgr)
            , config_file_path_(config_path)
            , watching_(false)
            , poll_interval_(poll_interval)
            , last_write_time_(std::filesystem::file_time_type::min()) {

        createConfigSnapshot();
    }

    ConfigWatcher::~ConfigWatcher() {
        stop();
    }

    bool ConfigWatcher::start() {
        if (watching_.load()) {
            std::cout << "⚠️ ConfigWatcher уже запущен" << std::endl;
            return true;
        }

        std::cout << "👁️ Запуск мониторинга конфигурационного файла..." << std::endl;

        updateLastWriteTime();
        watching_.store(true);

        watch_thread_ = std::thread(&ConfigWatcher::watchLoop, this);

        std::cout << "✅ ConfigWatcher запущен" << std::endl;
        return true;
    }

    void ConfigWatcher::stop() {
        if (!watching_.load()) return;

        std::cout << "🛑 Остановка ConfigWatcher..." << std::endl;

        watching_.store(false);

        if (watch_thread_.joinable()) {
            watch_thread_.join();
        }

        std::cout << "✅ ConfigWatcher остановлен" << std::endl;
    }

    void ConfigWatcher::registerSectionHandler(std::unique_ptr<ConfigSectionHandler> handler) {
        std::string section = handler->GetSectionName();
        section_handlers_[section] = std::move(handler);

        std::cout << "📋 Зарегистрирован обработчик для секции: " << section << std::endl;
    }

    void ConfigWatcher::setGlobalChangeCallback(ConfigChangeCallback callback) {
        global_change_callback_ = callback;
    }

    void ConfigWatcher::setSystemRestartCallback(SystemRestartCallback callback) {
        restart_callback_ = callback;
    }

    bool ConfigWatcher::triggerReload() {
        std::cout << "🔄 Принудительная перезагрузка конфигурации..." << std::endl;
        return processConfigurationChanges();
    }

    std::vector<ConfigChange> ConfigWatcher::getPendingChanges() const {
        std::lock_guard<std::mutex> lock(changes_mutex_);
        return pending_changes_;
    }

    bool ConfigWatcher::isWatching() const {
        return watching_.load();
    }

    std::chrono::system_clock::time_point ConfigWatcher::getLastUpdateTime() const {
        try {
            auto file_time = std::filesystem::last_write_time(config_file_path_);
            auto system_time = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
                    file_time - std::filesystem::file_time_type::clock::now() +
                    std::chrono::system_clock::now());
            return system_time;
        } catch (const std::filesystem::filesystem_error& e) {
            std::cerr << "❌ Ошибка получения времени файла: " << e.what() << std::endl;
            return std::chrono::system_clock::now();
        }
    }

    void ConfigWatcher::watchLoop() {
        std::cout << "👁️ Цикл мониторинга конфигурации запущен" << std::endl;

        while (watching_.load()) {
            try {
                if (hasFileChanged()) {
                    std::cout << "📄 Обнаружено изменение конфигурационного файла - обрабатываем..." << std::endl;
                    processConfigurationChanges();
                    updateLastWriteTime();
                }
            } catch (const std::exception& e) {
                std::cerr << "❌ Ошибка в цикле мониторинга: " << e.what() << std::endl;
            }

            std::this_thread::sleep_for(poll_interval_);
        }

        std::cout << "👁️ Цикл мониторинга конфигурации завершен" << std::endl;
    }

    bool ConfigWatcher::hasFileChanged() {
        try {
            auto current_time = std::filesystem::last_write_time(config_file_path_);
            return current_time != last_write_time_;
        } catch (const std::filesystem::filesystem_error& e) {
            std::cerr << "❌ Ошибка проверки времени файла: " << e.what() << std::endl;
            return false;
        }
    }

    void ConfigWatcher::updateLastWriteTime() {
        try {
            last_write_time_ = std::filesystem::last_write_time(config_file_path_);
        } catch (const std::filesystem::filesystem_error& e) {
            std::cerr << "❌ Ошибка обновления времени файла: " << e.what() << std::endl;
        }
    }

    bool ConfigWatcher::processConfigurationChanges() {
        // Перезагружаем конфигурацию
        if (!config_manager_->ReloadConfig()) {
            std::cerr << "❌ Не удалось перезагрузить конфигурационный файл!" << std::endl;
            return false;
        }

        // Создаем новый snapshot и обнаруживаем изменения
        auto changes = detectConfigChanges();

        if (changes.empty()) {
            std::cout << "ℹ️ Значимых изменений конфигурации не обнаружено" << std::endl;
            return true;
        }

        std::cout << "🔄 Обработка " << changes.size() << " изменений конфигурации..." << std::endl;

        // Сортируем изменения по критичности
        std::sort(changes.begin(), changes.end(),
                  [](const ConfigChange& a, const ConfigChange& b) {
                      return static_cast<int>(a.severity) < static_cast<int>(b.severity);
                  });

        // Применяем изменения
        bool all_applied = true;
        bool requires_component_restart = false;
        bool requires_system_restart = false;

        for (const auto& change : changes) {
            bool applied = applyConfigChange(change);

            if (applied) {
                std::cout << "✅ Применено: " << change.section << "." << change.key
                          << " = " << change.new_value << std::endl;
            } else {
                std::cerr << "❌ Не удалось применить: " << change.section << "." << change.key << std::endl;
                all_applied = false;
            }

            // Отслеживаем требования перезапуска
            if (change.severity == ConfigChangeSeverity::COMPONENT_RESTART) {
                requires_component_restart = true;
            } else if (change.severity >= ConfigChangeSeverity::SYSTEM_RESTART) {
                requires_system_restart = true;
            }
        }

        // Обрабатываем требования перезапуска
        if (requires_system_restart && restart_callback_) {
            std::cout << "🚨 Изменения конфигурации требуют перезапуска системы!" << std::endl;
            restart_callback_("Критические изменения конфигурации требуют перезапуска");
        } else if (requires_component_restart) {
            std::cout << "⚠️ Некоторые изменения конфигурации требуют перезапуска компонентов" << std::endl;
        }

        // Обновляем snapshot
        createConfigSnapshot();

        // Сохраняем pending changes для диагностики
        {
            std::lock_guard<std::mutex> lock(changes_mutex_);
            pending_changes_ = changes;
        }

        return all_applied;
    }

    std::vector<ConfigChange> ConfigWatcher::detectConfigChanges() {
        std::vector<ConfigChange> changes;

        // Создаем новый snapshot
        auto new_snapshot = createCurrentSnapshot();

        // Сравниваем со старым snapshot
        for (const auto& [key, new_value] : new_snapshot) {
            auto old_it = config_snapshot_.find(key);
            if (old_it == config_snapshot_.end() || old_it->second != new_value) {
                ConfigChange change;
                change.key = key;
                change.new_value = new_value;
                change.old_value = (old_it != config_snapshot_.end()) ? old_it->second : "";

                // Парсим секцию из ключа (например, "lora.power_levels.max_power" -> "lora")
                auto dot_pos = key.find('.');
                change.section = (dot_pos != std::string::npos) ? key.substr(0, dot_pos) : key;

                // Определяем критичность
                change.severity = determineChangeSeverity(change);
                change.description = generateChangeDescription(change);

                changes.push_back(change);
            }
        }

        return changes;
    }

    std::unordered_map<std::string, std::string> ConfigWatcher::createCurrentSnapshot() {
        std::unordered_map<std::string, std::string> snapshot;

        // РЕАЛЬНЫЕ ключи конфигурации которые мониторим
        std::vector<std::string> monitored_keys = {
                // РЕАЛЬНЫЕ ключи из ConfigManager.cpp!
                "lora.primary_frequencies",             // GetLoRaFrequencies()
                "lora.power_levels.max_power",          // GetPowerConfig()
                "lora.power_levels.min_power",          // GetPowerConfig()
                "lora.power_levels.adaptive_step",      // GetPowerConfig()
                "lora.mesh_config.enable_mesh",         // GetCommunicationConfig()
                "lora.frequency_hopping.enable",        // GetCommunicationConfig()
                "lora.frequency_hopping.hop_interval",  // GetCommunicationConfig()
                "video.power_levels.min_power",         // GetPowerConfig()
                "video.power_levels.max_power",         // GetPowerConfig()
                "video.power_levels.default_power",     // GetPowerConfig()
                "uwb.tx_power",                         // GetPowerConfig()
                "security.enable_encryption",           // GetCommunicationConfig()
                "security.shared_secret",               // GetCommunicationConfig()
                "failsafe.enable_self_destruct",        // PrintConfigurationSummary()
                "communication.timeout_ms",             // GetCommunicationConfig()
                "communication.heartbeat_interval_ms",  // GetCommunicationConfig()
                "communication.max_retries",            // GetCommunicationConfig()
                "system.max_drones_mvp"                 // PrintConfigurationSummary()
        };

        for (const auto& key : monitored_keys) {
            std::string value = config_manager_->GetValue<std::string>(key, "");
            if (!value.empty()) {
                snapshot[key] = value;
            }
        }

        return snapshot;
    }

    void ConfigWatcher::createConfigSnapshot() {
        config_snapshot_ = createCurrentSnapshot();
    }

    ConfigChangeSeverity ConfigWatcher::determineChangeSeverity(const ConfigChange& change) {
        // Проверяем с обработчиком секции сначала
        auto handler_it = section_handlers_.find(change.section);
        if (handler_it != section_handlers_.end()) {
            std::string section_key = change.key.substr(change.section.length() + 1);
            return handler_it->second->GetChangeSeverity(section_key);
        }

        // Правила критичности по умолчанию
        if (change.section == "security" || change.key.find("self_destruct") != std::string::npos) {
            return ConfigChangeSeverity::MISSION_CRITICAL;
        }

        if (change.section == "system" && change.key.find("max_drones") != std::string::npos) {
            return ConfigChangeSeverity::SYSTEM_RESTART;
        }

        return ConfigChangeSeverity::HOT_RELOAD;
    }

    std::string ConfigWatcher::generateChangeDescription(const ConfigChange& change) {
        return "Изменена конфигурация " + change.section + ": " +
               change.key + " с '" + change.old_value + "' на '" + change.new_value + "'";
    }

    bool ConfigWatcher::applyConfigChange(const ConfigChange& change) {
        try {
            // Apply configuration changes to real components based on section

            if (change.section == "lora") {
                return applyLoRaConfigChange(change);
            } else if (change.section == "elrs") {
                return applyELRSConfigChange(change);
            } else if (change.section == "uwb") {
                return applyUWBConfigChange(change);
            } else if (change.section == "mesh") {
                return applyMeshConfigChange(change);
            } else if (change.section == "formation") {
                return applyFormationConfigChange(change);
            } else if (change.section == "power") {
                return applyPowerConfigChange(change);
            } else if (change.section == "safety") {
                return applySafetyConfigChange(change);
            } else {
                std::cerr << "⚠️ Unknown configuration section: " << change.section << std::endl;
                return false;
            }

        } catch (const std::exception& e) {
            std::cerr << "❌ Exception in applyConfigChange: " << e.what() << std::endl;
            return false;
        }
    }

    bool ConfigWatcher::applyLoRaConfigChange(const ConfigChange& change) {
        if (!communication_manager_) {
            std::cerr << "❌ CommunicationManager not available for LoRa config change" << std::endl;
            return false;
        }

        try {
            // Реальные вызовы CommunicationManager методов
            if (change.key == "lora.power_levels.max_power") {
                int8_t max_power = std::stoi(change.new_value);
                return communication_manager_->set_max_power_level(max_power);

            } else if (change.key == "lora.power_levels.min_power") {
                int8_t min_power = std::stoi(change.new_value);
                return communication_manager_->set_min_power_level(min_power);

            } else if (change.key == "lora.frequency_hopping.enabled") {
                bool enabled = (change.new_value == "true");
                return communication_manager_->enable_frequency_hopping(enabled);

            } else if (change.key == "lora.frequency_hopping.interval_ms") {
                uint32_t interval = std::stoul(change.new_value);
                return communication_manager_->set_frequency_hop_interval(interval);

            } else if (change.key == "lora.frequency_list") {
                // Parse frequency list from YAML
                std::vector<uint32_t> frequencies = parseFrequencyList(change.new_value);
                return communication_manager_->update_frequency_list(frequencies);

            } else if (change.key == "lora.adaptive_power.enabled") {
                bool enabled = (change.new_value == "true");
                return communication_manager_->enable_adaptive_power(enabled);

            } else if (change.key == "lora.adaptive_power.rssi_threshold") {
                double threshold = std::stod(change.new_value);
                return communication_manager_->set_rssi_threshold(threshold);

            } else if (change.key == "lora.timeouts.communication_timeout_ms") {
                uint32_t timeout = std::stoul(change.new_value);
                return communication_manager_->set_communication_timeout(timeout);

            } else if (change.key == "lora.timeouts.heartbeat_interval_ms") {
                uint32_t interval = std::stoul(change.new_value);
                return communication_manager_->set_heartbeat_interval(interval);

            } else if (change.key == "lora.interference.threshold_dbm") {
                double threshold = std::stod(change.new_value);
                return communication_manager_->set_interference_threshold(threshold);

            } else {
                std::cerr << "⚠️ Unknown LoRa config key: " << change.key << std::endl;
                return false;
            }

        } catch (const std::exception& e) {
            std::cerr << "❌ LoRa config change failed: " << e.what() << std::endl;
            return false;
        }
    }

    bool ConfigWatcher::applyELRSConfigChange(const ConfigChange& change) {
        if (!communication_manager_) {
            std::cerr << "❌ CommunicationManager not available for ELRS config change" << std::endl;
            return false;
        }

        try {
            // ELRS specific configuration changes
            if (change.key == "elrs.enabled") {
                bool enabled = (change.new_value == "true");
                return communication_manager_->enable_elrs(enabled);

            } else if (change.key == "elrs.power_2g4") {
                int8_t power = std::stoi(change.new_value);
                return communication_manager_->set_elrs_2g4_power(power);

            } else if (change.key == "elrs.power_915") {
                int8_t power = std::stoi(change.new_value);
                return communication_manager_->set_elrs_915_power(power);

            } else if (change.key == "elrs.packet_rate_2g4") {
                uint16_t rate = std::stoul(change.new_value);
                return communication_manager_->set_elrs_2g4_packet_rate(rate);

            } else if (change.key == "elrs.packet_rate_915") {
                uint16_t rate = std::stoul(change.new_value);
                return communication_manager_->set_elrs_915_packet_rate(rate);

            } else {
                std::cerr << "⚠️ Unknown ELRS config key: " << change.key << std::endl;
                return false;
            }

        } catch (const std::exception& e) {
            std::cerr << "❌ ELRS config change failed: " << e.what() << std::endl;
            return false;
        }
    }

    bool ConfigWatcher::applyUWBConfigChange(const ConfigChange& change) {
        if (!uwb_manager_) {
            std::cerr << "❌ UWBManager not available for UWB config change" << std::endl;
            return false;
        }

        try {
            // Реальные вызовы UWBManager методов
            if (change.key == "uwb.tx_power_dbm") {
                int8_t power = std::stoi(change.new_value);
                return uwb_manager_->SetTxPower(power);

            } else if (change.key == "uwb.rx_gain") {
                uint8_t gain = std::stoul(change.new_value);
                return uwb_manager_->SetRxGain(gain);

            } else if (change.key == "uwb.update_rate_hz") {
                uint16_t rate = std::stoul(change.new_value);
                return uwb_manager_->SetUpdateRate(rate);

            } else if (change.key == "uwb.measurement_interval_ms") {
                uint32_t interval = std::stoul(change.new_value);
                return uwb_manager_->SetMeasurementInterval(interval);

            } else if (change.key == "uwb.ranging_mode") {
                UWBRangingMode mode = parseRangingMode(change.new_value);
                return uwb_manager_->SetRangingMode(mode);

            } else if (change.key == "uwb.max_ranging_distance_m") {
                double distance = std::stod(change.new_value);
                return uwb_manager_->SetMaxRangingDistance(distance);

            } else if (change.key == "uwb.accuracy_threshold_m") {
                double threshold = std::stod(change.new_value);
                return uwb_manager_->SetPositionAccuracyThreshold(threshold);

            } else if (change.key == "uwb.outlier_rejection_enabled") {
                bool enabled = (change.new_value == "true");
                return uwb_manager_->EnableOutlierRejection(enabled);

            } else if (change.key == "uwb.channel") {
                uint8_t channel = std::stoul(change.new_value);
                return uwb_manager_->SetChannel(channel);

            } else if (change.key == "uwb.preamble_length") {
                uint16_t length = std::stoul(change.new_value);
                return uwb_manager_->SetPreambleLength(length);

            } else {
                std::cerr << "⚠️ Unknown UWB config key: " << change.key << std::endl;
                return false;
            }

        } catch (const std::exception& e) {
            std::cerr << "❌ UWB config change failed: " << e.what() << std::endl;
            return false;
        }
    }

    bool ConfigWatcher::applyMeshConfigChange(const ConfigChange& change) {
        if (!communication_manager_) {
            std::cerr << "❌ CommunicationManager not available for mesh config change" << std::endl;
            return false;
        }

        try {
            // Mesh networking configuration changes
            if (change.key == "mesh.enabled") {
                bool enabled = (change.new_value == "true");
                return communication_manager_->enable_mesh_networking(enabled);

            } else if (change.key == "mesh.max_hops") {
                uint8_t hops = std::stoul(change.new_value);
                return communication_manager_->set_mesh_max_hops(hops);

            } else if (change.key == "mesh.discovery_interval_ms") {
                uint32_t interval = std::stoul(change.new_value);
                return communication_manager_->set_mesh_discovery_interval(interval);

            } else if (change.key == "mesh.routing_algorithm") {
                std::string algorithm = change.new_value;
                return communication_manager_->set_mesh_routing_algorithm(algorithm);

            } else {
                std::cerr << "⚠️ Unknown mesh config key: " << change.key << std::endl;
                return false;
            }

        } catch (const std::exception& e) {
            std::cerr << "❌ Mesh config change failed: " << e.what() << std::endl;
            return false;
        }
    }

    bool ConfigWatcher::applyFormationConfigChange(const ConfigChange& change) {
        if (!formation_controller_) {
            std::cerr << "❌ FormationController not available for formation config change" << std::endl;
            return false;
        }

        try {
            // Formation control configuration changes
            if (change.key == "formation.default_type") {
                std::string formation_type = change.new_value;
                return formation_controller_->SetDefaultFormation(formation_type);

            } else if (change.key == "formation.spacing_meters") {
                double spacing = std::stod(change.new_value);
                return formation_controller_->SetFormationSpacing(spacing);

            } else if (change.key == "formation.leader_id") {
                DroneID leader_id = std::stoul(change.new_value);
                return formation_controller_->SetFormationLeader(leader_id);

            } else if (change.key == "formation.follow_distance_m") {
                double distance = std::stod(change.new_value);
                return formation_controller_->SetFollowDistance(distance);

            } else if (change.key == "formation.collision_avoidance_enabled") {
                bool enabled = (change.new_value == "true");
                return formation_controller_->EnableCollisionAvoidance(enabled);

            } else {
                std::cerr << "⚠️ Unknown formation config key: " << change.key << std::endl;
                return false;
            }

        } catch (const std::exception& e) {
            std::cerr << "❌ Formation config change failed: " << e.what() << std::endl;
            return false;
        }
    }

    bool ConfigWatcher::applyPowerConfigChange(const ConfigChange& change) {
        if (!power_manager_) {
            std::cerr << "❌ PowerManager not available for power config change" << std::endl;
            return false;
        }

        try {
            // Power management configuration changes
            if (change.key == "power.low_battery_threshold") {
                double threshold = std::stod(change.new_value);
                return power_manager_->SetLowBatteryThreshold(threshold);

            } else if (change.key == "power.critical_battery_threshold") {
                double threshold = std::stod(change.new_value);
                return power_manager_->SetCriticalBatteryThreshold(threshold);

            } else if (change.key == "power.auto_rtl_enabled") {
                bool enabled = (change.new_value == "true");
                return power_manager_->EnableAutoRTL(enabled);

            } else if (change.key == "power.power_saving_enabled") {
                bool enabled = (change.new_value == "true");
                return power_manager_->EnablePowerSaving(enabled);

            } else {
                std::cerr << "⚠️ Unknown power config key: " << change.key << std::endl;
                return false;
            }

        } catch (const std::exception& e) {
            std::cerr << "❌ Power config change failed: " << e.what() << std::endl;
            return false;
        }
    }

    bool ConfigWatcher::applySafetyConfigChange(const ConfigChange& change) {
        if (!safety_manager_) {
            std::cerr << "❌ SafetyManager not available for safety config change" << std::endl;
            return false;
        }

        try {
            // Safety system configuration changes
            if (change.key == "safety.geofence_enabled") {
                bool enabled = (change.new_value == "true");
                return safety_manager_->EnableGeofence(enabled);

            } else if (change.key == "safety.max_altitude_m") {
                double altitude = std::stod(change.new_value);
                return safety_manager_->SetMaxAltitude(altitude);

            } else if (change.key == "safety.max_speed_ms") {
                double speed = std::stod(change.new_value);
                return safety_manager_->SetMaxSpeed(speed);

            } else if (change.key == "safety.emergency_land_enabled") {
                bool enabled = (change.new_value == "true");
                return safety_manager_->EnableEmergencyLand(enabled);

            } else if (change.key == "safety.failsafe_mode") {
                std::string mode = change.new_value;
                return safety_manager_->SetFailsafeMode(mode);

            } else {
                std::cerr << "⚠️ Unknown safety config key: " << change.key << std::endl;
                return false;
            }

        } catch (const std::exception& e) {
            std::cerr << "❌ Safety config change failed: " << e.what() << std::endl;
            return false;
        }
    }

//=============================================================================
// ✅ HELPER METHODS FOR CONFIG PARSING
//=============================================================================

    std::vector<uint32_t> ConfigWatcher::parseFrequencyList(const std::string& freq_string) {
        std::vector<uint32_t> frequencies;

        try {
            // Parse YAML array format: "[433075000, 433175000, ...]"
            YAML::Node freq_node = YAML::Load(freq_string);

            if (freq_node.IsSequence()) {
                for (const auto& freq : freq_node) {
                    frequencies.push_back(freq.as<uint32_t>());
                }
            }

        } catch (const YAML::Exception& e) {
            std::cerr << "❌ Error parsing frequency list: " << e.what() << std::endl;
        }

        return frequencies;
    }

    UWBRangingMode ConfigWatcher::parseRangingMode(const std::string& mode_string) {
        if (mode_string == "TWR") {
            return UWBRangingMode::TWR;
        } else if (mode_string == "DS_TWR") {
            return UWBRangingMode::DS_TWR;
        } else if (mode_string == "SS_TWR") {
            return UWBRangingMode::SS_TWR;
        } else if (mode_string == "TDOA") {
            return UWBRangingMode::TDOA;
        } else {
            std::cerr << "⚠️ Unknown ranging mode: " << mode_string << ", using DS_TWR" << std::endl;
            return UWBRangingMode::DS_TWR;
        }
    }

    ConfigChangeSeverity ConfigWatcher::determineChangeSeverity(const ConfigChange& change) {
        // Определяем критичность изменения конфигурации

        // Критические изменения требующие полного рестарта системы
        if (change.key == "drone.id" ||
            change.key == "safety.failsafe_mode" ||
            change.key == "uwb.channel" ||
            change.key == "uwb.preamble_length") {
            return ConfigChangeSeverity::SYSTEM_RESTART;
        }

            // Изменения требующие рестарта компонентов
        else if (change.key.find("power") == 0 ||
                 change.key == "mesh.enabled" ||
                 change.key == "lora.frequency_list") {
            return ConfigChangeSeverity::COMPONENT_RESTART;
        }

            // Критические изменения без рестарта
        else if (change.key.find("safety") == 0 ||
                 change.key == "formation.leader_id") {
            return ConfigChangeSeverity::CRITICAL;
        }

            // Обычные изменения
        else {
            return ConfigChangeSeverity::NORMAL;
        }
    }

    std::string ConfigWatcher::generateChangeDescription(const ConfigChange& change) {
        return "Changed " + change.key + " from '" + change.old_value +
               "' to '" + change.new_value + "'";
    }

//=============================================================================
// ✅ COMPONENT REGISTRATION
//=============================================================================

    void ConfigWatcher::registerCommunicationManager(std::shared_ptr<CommunicationManager> comm_mgr) {
        communication_manager_ = comm_mgr;
        std::cout << "✅ CommunicationManager registered with ConfigWatcher" << std::endl;
    }

    void ConfigWatcher::registerUWBManager(std::shared_ptr<UWBManager> uwb_mgr) {
        uwb_manager_ = uwb_mgr;
        std::cout << "✅ UWBManager registered with ConfigWatcher" << std::endl;
    }

    void ConfigWatcher::registerFormationController(std::shared_ptr<FormationController> form_ctrl) {
        formation_controller_ = form_ctrl;
        std::cout << "✅ FormationController registered with ConfigWatcher" << std::endl;
    }

    void ConfigWatcher::registerPowerManager(std::shared_ptr<PowerManager> power_mgr) {
        power_manager_ = power_mgr;
        std::cout << "✅ PowerManager registered with ConfigWatcher" << std::endl;
    }

    void ConfigWatcher::registerSafetyManager(std::shared_ptr<SafetyManager> safety_mgr) {
        safety_manager_ = safety_mgr;
        std::cout << "✅ SafetyManager registered with ConfigWatcher" << std::endl;
    }

    void ConfigWatcher::registerRestartCallback(std::function<void(const std::string&)> callback) {
        restart_callback_ = callback;
        std::cout << "✅ System restart callback registered with ConfigWatcher" << std::endl;
    }

} // namespace SwarmSystem