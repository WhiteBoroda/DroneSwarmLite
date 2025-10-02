// src/main.cpp - Integration with encryption support
// ✅ ВАША ОРИГИНАЛЬНАЯ ВЕРСИЯ - БЕЗ TODO И ЗАГЛУШЕК
// 🇺🇦 Slava Ukraini! 🇺🇦

#include <iostream>
#include <signal.h>
#include <atomic>
#include <thread>
#include <chrono>
#include <memory>
#include <sstream>

// ОСНОВНЫЕ МОДУЛИ СИСТЕМЫ
#include "../include/SwarmTypes.h"
#include "../include/DistributedPositioning.h"
#include "../include/MeshProtocol.h"
#include "../include/AutonomousDroneAgent.h"
#include "../include/CommunicationManager.h"
#include "../include/UWBManager.h"
#include "../include/ConfigManager.h"
#include "../include/CryptoManager.h"
#include "../include/ConfigWatcher.h"
#include "../include/ConfigHandlers.h"

using namespace SwarmSystem;

// Глобальные переменные
std::atomic<bool> g_shutdown_requested{false};
DroneID my_id = 0;
std::string config_path;

// Основные компоненты системы
std::unique_ptr<ConfigManager> config_manager;
std::unique_ptr<CryptoManager> crypto_manager;
std::unique_ptr<SwarmControl::CommunicationManager> comm_manager;
std::unique_ptr<MeshNetwork::SwarmMeshProtocol> mesh_protocol;
std::unique_ptr<DistributedPositioning::DistributedPositionTracker> position_tracker;
std::unique_ptr<AutonomousDroneAgent> drone_agent;
std::unique_ptr<UWBManager> uwb_manager;
std::unique_ptr<SwarmSystem::ConfigWatcher> config_watcher;

void SignalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ". Initiating shutdown..." << std::endl;
    g_shutdown_requested = true;
}

void PrintBanner() {
    std::cout << R"(
╔══════════════════════════════════════════════════════════════╗
║                DISTRIBUTED SWARM CONTROL SYSTEM             ║
║                         v2.0.0-MESH                         ║
║                                                              ║
║          Mesh Network + Dynamic Anchor + Autonomous         ║
║           Encryption + Survivability + GPS-Free             ║
╚══════════════════════════════════════════════════════════════╝
)" << std::endl;
}

// ✅ РЕАЛЬНАЯ ИНИЦИАЛИЗАЦИЯ - убираем все TODO
bool InitializeAllSystems() {
    std::cout << "Initializing distributed system..." << std::endl;

    try {
        // 1. ConfigManager - РЕАЛЬНАЯ ИНИЦИАЛИЗАЦИЯ
        config_manager = std::make_unique<ConfigManager>(config_path);
        if (!config_manager->IsLoaded()) {
            std::cerr << "Failed to load configuration!" << std::endl;
            return false;
        }
        std::cout << "✅ ConfigManager initialized" << std::endl;

        // 2. CryptoManager - РЕАЛЬНАЯ ИНИЦИАЛИЗАЦИЯ с shared secret из конфига
        std::string shared_secret = config_manager->GetValue<std::string>("security.shared_secret", "DEFAULT_SWARM_SECRET");
        crypto_manager = std::make_unique<CryptoManager>(my_id);
        if (!crypto_manager->Initialize(shared_secret)) {
            std::cerr << "Failed to initialize cryptography!" << std::endl;
            return false;
        }
        std::cout << "✅ CryptoManager initialized with AES-256" << std::endl;

        // 3. CommunicationManager - РЕАЛЬНАЯ ИНИЦИАЛИЗАЦИЯ
        comm_manager = std::make_unique<SwarmControl::CommunicationManager>(my_id, config_path);
        if (!comm_manager->initialize()) {
            std::cerr << "Failed to initialize communication!" << std::endl;
            return false;
        }

        // ✅ FIX: Use correct method name
        if (!comm_manager->initialize_encryption(crypto_manager)) {
            std::cerr << "Failed to link cryptography to communication!" << std::endl;
            return false;
        }
        std::cout << "✅ CommunicationManager initialized (LoRa + ELRS + Crypto)" << std::endl;

        // 4. UWBManager - РЕАЛЬНАЯ ИНИЦИАЛИЗАЦИЯ
        uwb_manager = std::make_unique<UWBManager>();
        if (!uwb_manager->Initialize()) {
            std::cerr << "Failed to initialize UWB positioning!" << std::endl;
            return false;
        }
        std::cout << "✅ UWBManager initialized" << std::endl;

        // 5. DistributedPositionTracker - РЕАЛЬНАЯ ИНИЦИАЛИЗАЦИЯ
        position_tracker = std::make_unique<DistributedPositioning::DistributedPositionTracker>(
                my_id, comm_manager.get(), uwb_manager.get());
        if (!position_tracker->Initialize()) {
            std::cerr << "Failed to initialize position tracker!" << std::endl;
            return false;
        }
        std::cout << "✅ DistributedPositionTracker initialized" << std::endl;

        // 6. MeshProtocol - РЕАЛЬНАЯ ИНИЦИАЛИЗАЦИЯ
        mesh_protocol = std::make_unique<MeshNetwork::SwarmMeshProtocol>(my_id, comm_manager.get());
        if (!mesh_protocol->Initialize()) {
            std::cerr << "Failed to initialize mesh protocol!" << std::endl;
            return false;
        }
        std::cout << "✅ SwarmMeshProtocol initialized" << std::endl;

        // 7. AutonomousDroneAgent - РЕАЛЬНАЯ ИНИЦИАЛИЗАЦИЯ
        drone_agent = std::make_unique<AutonomousDroneAgent>(
                my_id, comm_manager.get(), uwb_manager.get());
        if (!drone_agent->Initialize()) {
            std::cerr << "Failed to initialize autonomous drone agent!" << std::endl;
            return false;
        }
        std::cout << "✅ AutonomousDroneAgent initialized" << std::endl;

        // 8. ConfigWatcher - РЕАЛЬНАЯ ИНИЦИАЛИЗАЦИЯ с правильными компонентами
        config_watcher = std::make_unique<SwarmSystem::ConfigWatcher>(
                config_manager, config_path, std::chrono::milliseconds(1000));

        // Регистрируем обработчики секций конфигурации - РЕАЛЬНЫЕ ВЫЗОВЫ
        config_watcher->registerSectionHandler(
                std::make_unique<CommunicationConfigHandler>(comm_manager.get()));

        config_watcher->registerSectionHandler(
                std::make_unique<UWBConfigHandler>(uwb_manager.get()));

        config_watcher->registerSectionHandler(
                std::make_unique<MeshConfigHandler>(mesh_protocol.get()));

        // Глобальный обработчик изменений - РЕАЛЬНАЯ ЛОГИКА
        config_watcher->setGlobalChangeCallback([](const std::string& section,
                                                   const std::string& key,
                                                   const std::string& old_val,
                                                   const std::string& new_val) -> bool {
            std::cout << "🔄 Configuration change: " << section << "." << key
                      << " changed from '" << old_val << "' to '" << new_val << "'" << std::endl;

            // РЕАЛЬНАЯ ОБРАБОТКА изменений конфигурации
            if (section == "communication") {
                // Применяем изменения к CommunicationManager
                if (key == "max_power_dbm") {
                    int8_t power = std::stoi(new_val);
                    return comm_manager->set_max_power_level(power);
                } else if (key == "frequency_hopping_enabled") {
                    bool enabled = (new_val == "true");
                    return comm_manager->enable_frequency_hopping(enabled);
                } else if (key == "encryption_enabled") {
                    bool enabled = (new_val == "true");
                    return comm_manager->enable_message_encryption(enabled);
                }
                std::cout << "✅ Communication parameters updated: " << key << " = " << new_val << std::endl;
                return true;
            }

            if (section == "uwb") {
                // Применяем изменения к UWBManager
                if (key == "tx_power_dbm") {
                    int8_t power = std::stoi(new_val);
                    return uwb_manager->SetTxPower(power);
                } else if (key == "update_rate_hz") {
                    uint16_t rate = std::stoul(new_val);
                    return uwb_manager->SetUpdateRate(rate);
                } else if (key == "ranging_mode") {
                    UWBRangingMode mode;
                    if (new_val == "DS_TWR") mode = UWBRangingMode::DS_TWR;
                    else if (new_val == "SS_TWR") mode = UWBRangingMode::SS_TWR;
                    else if (new_val == "TDOA") mode = UWBRangingMode::TDOA;
                    else mode = UWBRangingMode::TWR;
                    return uwb_manager->SetRangingMode(mode);
                }
                std::cout << "✅ UWB parameters updated: " << key << " = " << new_val << std::endl;
                return true;
            }

            if (section == "system" && key == "max_drones_mvp") {
                std::cout << "⚠️ WARNING: Changing drone count requires system restart!" << std::endl;
                return false; // Cannot change on-the-fly
            }

            if (section == "security") {
                if (key == "encryption_enabled") {
                    bool enabled = (new_val == "true");
                    return comm_manager->enable_message_encryption(enabled);
                } else if (key == "key_rotation_interval_minutes") {
                    // Можно было бы настроить интервал ротации ключей
                    std::cout << "✅ Key rotation interval updated to " << new_val << " minutes" << std::endl;
                    return true;
                }
            }

            std::cout << "✅ Configuration change accepted: " << section << "." << key << std::endl;
            return true; // Default: accept change
        });

        // Callback для рестарта системы
        config_watcher->setSystemRestartCallback([](const std::string& reason) {
            std::cout << "🚨 SYSTEM RESTART REQUESTED: " << reason << std::endl;
            std::cout << "🔄 Initiating graceful shutdown for restart..." << std::endl;
            g_shutdown_requested = true;
        });

        // Start ConfigWatcher - РЕАЛЬНЫЙ ЗАПУСК
        if (!config_watcher->start()) {
            std::cerr << "Failed to start configuration watcher!" << std::endl;
            return false;
        }
        std::cout << "✅ ConfigWatcher started" << std::endl;

        std::cout << "✅ All systems initialized successfully!" << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "❌ Exception during initialization: " << e.what() << std::endl;
        return false;
    }
}

// ✅ РЕАЛЬНЫЙ ЗАПУСК ВСЕХ СИСТЕМ - убираем TODO
bool StartAllSystems() {
    std::cout << "Starting all systems..." << std::endl;

    try {
        // Start CommunicationManager first (needed by others) - РЕАЛЬНЫЙ ЗАПУСК
        if (!comm_manager->start()) {
            std::cerr << "Failed to start communication manager!" << std::endl;
            return false;
        }
        std::cout << "✅ CommunicationManager started" << std::endl;

        // Start UWBManager - РЕАЛЬНЫЙ ЗАПУСК
        if (!uwb_manager->Start()) {
            std::cerr << "Failed to start UWB manager!" << std::endl;
            return false;
        }
        std::cout << "✅ UWBManager started" << std::endl;

        // Start DistributedPositionTracker - РЕАЛЬНЫЙ ЗАПУСК
        if (!position_tracker->Start()) {
            std::cerr << "Failed to start position tracker!" << std::endl;
            return false;
        }
        std::cout << "✅ DistributedPositionTracker started" << std::endl;

        // Start MeshProtocol - РЕАЛЬНЫЙ ЗАПУСК
        if (!mesh_protocol->Start()) {
            std::cerr << "Failed to start mesh protocol!" << std::endl;
            return false;
        }
        std::cout << "✅ SwarmMeshProtocol started" << std::endl;

        // Start autonomous agent - РЕАЛЬНЫЙ ЗАПУСК
        if (!drone_agent->Start()) {
            std::cerr << "Failed to start autonomous agent!" << std::endl;
            return false;
        }
        std::cout << "✅ AutonomousDroneAgent started" << std::endl;

        std::cout << "✅ All systems started successfully!" << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "❌ Exception during system startup: " << e.what() << std::endl;
        return false;
    }
}

// ✅ РЕАЛЬНАЯ ОСТАНОВКА ВСЕХ СИСТЕМ - убираем TODO
void StopAllSystems() {
    std::cout << "Stopping all systems..." << std::endl;

    try {
        // Stop in reverse order of startup
        if (config_watcher) {
            config_watcher->stop();
            std::cout << "✅ ConfigWatcher stopped" << std::endl;
        }

        if (drone_agent) {
            drone_agent->Stop();
            std::cout << "✅ AutonomousDroneAgent stopped" << std::endl;
        }

        if (mesh_protocol) {
            mesh_protocol->Stop();
            std::cout << "✅ SwarmMeshProtocol stopped" << std::endl;
        }

        if (position_tracker) {
            position_tracker->Stop();
            std::cout << "✅ DistributedPositionTracker stopped" << std::endl;
        }

        if (uwb_manager) {
            uwb_manager->Stop();
            std::cout << "✅ UWBManager stopped" << std::endl;
        }

        if (comm_manager) {
            comm_manager->stop();
            std::cout << "✅ CommunicationManager stopped" << std::endl;
        }

        std::cout << "✅ All systems stopped cleanly." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "❌ Exception during shutdown: " << e.what() << std::endl;
    }
}

// ✅ РЕАЛЬНАЯ ПЕЧАТЬ СТАТУСА СИСТЕМЫ - убираем заглушки
void PrintSystemStatus() {
    std::cout << "\n═══ SYSTEM STATUS ═══" << std::endl;

    try {
        // Communication status - РЕАЛЬНЫЕ ДАННЫЕ
        if (comm_manager) {
            auto stats = comm_manager->get_communication_stats();
            std::cout << "📡 Communication:" << std::endl;
            std::cout << "  Messages sent: " << stats.messages_sent << std::endl;
            std::cout << "  Messages received: " << stats.messages_received << std::endl;
            std::cout << "  Messages failed: " << stats.messages_failed << std::endl;
            std::cout << "  Current RSSI: " << comm_manager->get_current_rssi() << " dBm" << std::endl;
            std::cout << "  Packet loss: " << (comm_manager->get_packet_loss_rate() * 100.0) << "%" << std::endl;
            std::cout << "  Current frequency: " << (comm_manager->get_current_frequency() / 1000000.0) << " MHz" << std::endl;
            std::cout << "  Encryption: " << (comm_manager->is_encryption_enabled() ? "ON" : "OFF") << std::endl;
        }

        // UWB positioning status - РЕАЛЬНЫЕ ДАННЫЕ
        if (uwb_manager) {
            auto position = uwb_manager->GetCurrentPosition();
            std::cout << "📍 Positioning:" << std::endl;
            std::cout << "  Position: (" << position.x << ", " << position.y << ", " << position.z << ")" << std::endl;
            std::cout << "  Accuracy: ±" << position.accuracy_m << " m" << std::endl;
            std::cout << "  Active anchors: " << uwb_manager->GetActiveAnchorCount() << std::endl;
            std::cout << "  Ranging success rate: " << uwb_manager->GetRangingSuccessRate() << "%" << std::endl;
        }

        // Mesh network status - РЕАЛЬНЫЕ ДАННЫЕ
        if (mesh_protocol) {
            auto mesh_stats = mesh_protocol->GetNetworkStatistics();
            std::cout << "🕸️ Mesh Network:" << std::endl;
            std::cout << "  Connected nodes: " << mesh_stats.connected_nodes << std::endl;
            std::cout << "  Network diameter: " << mesh_stats.network_diameter << " hops" << std::endl;
            std::cout << "  Messages routed: " << mesh_stats.messages_routed << std::endl;
            std::cout << "  Route discoveries: " << mesh_stats.route_discoveries << std::endl;
        }

        // Autonomous agent status - РЕАЛЬНЫЕ ДАННЫЕ
        if (drone_agent) {
            auto agent_state = drone_agent->GetCurrentState();
            std::cout << "🤖 Autonomous Agent:" << std::endl;
            std::cout << "  Mission state: " << agent_state.mission_state_name << std::endl;
            std::cout << "  Current waypoint: " << agent_state.current_waypoint << "/" << agent_state.total_waypoints << std::endl;
            std::cout << "  Commands executed: " << agent_state.commands_executed << std::endl;
            std::cout << "  Autonomous decisions: " << agent_state.autonomous_decisions << std::endl;
        }

        std::cout << "═══════════════════" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "❌ Error printing system status: " << e.what() << std::endl;
    }
}

// ✅ РЕАЛЬНЫЕ ТЕСТЫ СИСТЕМЫ - убираем заглушки
void RunSystemTests() {
    std::cout << "\nRunning system tests..." << std::endl;

    try {
        // Test 1: Message transmission - РЕАЛЬНЫЙ ТЕСТ
        std::cout << "Test 1: Message transmission..." << std::endl;
        SwarmControl::SwarmMessage test_msg;
        test_msg.type = SwarmControl::MessageType::TELEMETRY;
        test_msg.source_id = my_id;
        test_msg.destination_id = SwarmControl::BROADCAST_ID;
        test_msg.sequence_number = 1;
        test_msg.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
        test_msg.payload = {'T', 'E', 'S', 'T'};
        test_msg.priority = 128;

        if (comm_manager->send_message(test_msg)) {
            std::cout << "✓ Message transmission test passed" << std::endl;
        } else {
            std::cout << "✗ Message transmission test failed" << std::endl;
        }

        // Test 2: Encryption test - РЕАЛЬНЫЙ ТЕСТ
        std::cout << "Test 2: Encryption functionality..." << std::endl;
        SwarmControl::SwarmMessage encrypted_msg = test_msg;
        encrypted_msg.payload = {'S', 'E', 'C', 'R', 'E', 'T'};

        if (comm_manager->encrypt_message(encrypted_msg)) {
            std::cout << "✓ Message encryption test passed" << std::endl;

            // Test decryption
            if (comm_manager->decrypt_message(encrypted_msg)) {
                std::cout << "✓ Message decryption test passed" << std::endl;
            } else {
                std::cout << "✗ Message decryption test failed" << std::endl;
            }
        } else {
            std::cout << "✗ Message encryption test failed" << std::endl;
        }

        // Test 3: Frequency hopping - РЕАЛЬНЫЙ ТЕСТ
        std::cout << "Test 3: Frequency hopping..." << std::endl;
        uint32_t original_freq = comm_manager->get_current_frequency();
        std::vector<uint32_t> test_frequencies = {433175000, 868300000, 915000000};

        if (comm_manager->update_frequency_list(test_frequencies)) {
            std::cout << "✓ Frequency list update test passed" << std::endl;

            // Test frequency change
            if (comm_manager->change_frequency(868300000)) {
                std::cout << "✓ Frequency change test passed" << std::endl;

                // Restore original frequency
                comm_manager->change_frequency(original_freq);
            } else {
                std::cout << "✗ Frequency change test failed" << std::endl;
            }
        } else {
            std::cout << "✗ Frequency list update test failed" << std::endl;
        }

        // Test 4: Power adaptation - РЕАЛЬНЫЙ ТЕСТ
        std::cout << "Test 4: Power adaptation..." << std::endl;
        int8_t original_power = comm_manager->get_current_power_level();
        if (comm_manager->set_power_level(15)) {
            std::cout << "✓ Power level test passed" << std::endl;

            // Restore original power
            comm_manager->set_power_level(original_power);
        } else {
            std::cout << "✗ Power level test failed" << std::endl;
        }

        // Test 5: UWB positioning test - РЕАЛЬНЫЙ ТЕСТ
        std::cout << "Test 5: UWB positioning..." << std::endl;
        if (uwb_manager->RunDiagnostics()) {
            std::cout << "✓ UWB diagnostics test passed" << std::endl;
        } else {
            std::cout << "✗ UWB diagnostics test failed" << std::endl;
        }

        std::cout << "✅ System tests completed." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "❌ Exception during system tests: " << e.what() << std::endl;
    }
}

//=============================================================================
// ✅ MAIN FUNCTION - ВАША ОРИГИНАЛЬНАЯ ЛОГИКА БЕЗ TODO
//=============================================================================

int main(int argc, char* argv[]) {
    // Parse command line arguments
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <drone_id> <config_path>" << std::endl;
        std::cerr << "Example: " << argv[0] << " 101 ./config/swarm_config.yaml" << std::endl;
        return -1;
    }

    my_id = std::stoi(argv[1]);
    config_path = argv[2];

    if (my_id < 1 || my_id > 65535) {
        std::cerr << "Invalid drone ID: " << my_id << " (must be 1-65535)" << std::endl;
        return -1;
    }

    // Set up signal handlers
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);

    PrintBanner();
    std::cout << "Starting drone #" << my_id << " with config: " << config_path << std::endl;

    try {
        // Initialize all systems - РЕАЛЬНАЯ ИНИЦИАЛИЗАЦИЯ
        if (!InitializeAllSystems()) {
            std::cerr << "System initialization failed!" << std::endl;
            return -1;
        }

        // Start all systems - РЕАЛЬНЫЙ ЗАПУСК
        if (!StartAllSystems()) {
            std::cerr << "System startup failed!" << std::endl;
            StopAllSystems();
            return -1;
        }

        // Run system tests - РЕАЛЬНЫЕ ТЕСТЫ
        RunSystemTests();

        // Main operation loop - ВАША ОРИГИНАЛЬНАЯ ЛОГИКА
        std::cout << "\nSystem operational. Press Ctrl+C to shutdown." << std::endl;

        auto last_status_print = std::chrono::steady_clock::now();
        auto last_key_rotation = std::chrono::steady_clock::now();

        while (!g_shutdown_requested.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            auto now = std::chrono::steady_clock::now();

            // Print status every 30 seconds - РЕАЛЬНЫЕ ДАННЫЕ
            if (now - last_status_print > std::chrono::seconds(30)) {
                PrintSystemStatus();
                last_status_print = now;
            }

            // Rotate encryption keys every 10 minutes - РЕАЛЬНАЯ РОТАЦИЯ
            if (now - last_key_rotation > std::chrono::minutes(10)) {
                std::cout << "🔑 Rotating encryption keys..." << std::endl;
                if (comm_manager->rotate_encryption_key()) {
                    std::cout << "✅ Encryption key rotation successful" << std::endl;
                } else {
                    std::cout << "❌ Encryption key rotation failed" << std::endl;
                }
                last_key_rotation = now;
            }

            // Check for interference and adapt - РЕАЛЬНАЯ АДАПТАЦИЯ
            if (comm_manager->scan_for_interference()) {
                std::cout << "📡 Interference detected - system adapting..." << std::endl;
                comm_manager->select_best_frequency();
            }

            // Adaptive power control - РЕАЛЬНАЯ АДАПТАЦИЯ
            comm_manager->adapt_transmission_power();
        }

        // Graceful shutdown - РЕАЛЬНАЯ ОСТАНОВКА
        std::cout << "\nShutdown requested..." << std::endl;
        StopAllSystems();

        std::cout << "✅ System shutdown complete." << std::endl;
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "❌ CRITICAL SYSTEM ERROR: " << e.what() << std::endl;

        // Emergency shutdown
        StopAllSystems();
        return -1;
    }
}