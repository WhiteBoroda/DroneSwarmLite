// src/main.cpp - Integration with encryption support
//

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

// Инициализация всех компонентов
bool InitializeAllSystems() {
    std::cout << "Initializing distributed system..." << std::endl;

    // 1. ConfigManager
    config_manager = std::make_unique<ConfigManager>(config_path);
    if (!config_manager->IsLoaded()) {
        std::cerr << "Failed to load configuration!" << std::endl;
        return false;
    }

    // 2. CryptoManager - инициализируем с shared secret из конфига
    std::string shared_secret = config_manager->GetValue<std::string>("security.shared_secret", "DEFAULT_SWARM_SECRET");
    crypto_manager = std::make_unique<CryptoManager>(my_id);
    if (!crypto_manager->Initialize(shared_secret)) {
        std::cerr << "Failed to initialize cryptography!" << std::endl;
        return false;
    }

    // 3. CommunicationManager
    comm_manager = std::make_unique<SwarmControl::CommunicationManager>(my_id, config_path);
    if (!comm_manager->initialize()) {
        std::cerr << "Failed to initialize communication!" << std::endl;
        return false;
    }

    // 4. INTEGRATION: Connect encryption to communication
    if (!comm_manager->initialize_encryption(crypto_manager)) {
        std::cerr << "Failed to initialize communication encryption!" << std::endl;
        return false;
    }

    // 5. Enable encryption by default
    bool encryption_enabled = config_manager->GetValue<bool>("security.enable_encryption", true);
    if (!comm_manager->enable_message_encryption(encryption_enabled)) {
        std::cerr << "Failed to enable message encryption!" << std::endl;
        return false;
    }

    std::cout << "Encryption status: " << (encryption_enabled ? "ENABLED" : "DISABLED") << std::endl;

    // 6. UWBManager
    uwb_manager = std::make_unique<UWBManager>(my_id);
    if (!uwb_manager->Initialize()) {
        std::cerr << "Failed to initialize UWB!" << std::endl;
        return false;
    }

    // 7. MeshProtocol
    mesh_protocol = std::make_unique<MeshNetwork::SwarmMeshProtocol>(my_id);
    if (!mesh_protocol->Initialize()) {
        std::cerr << "Failed to initialize Mesh network!" << std::endl;
        return false;
    }

    // 8. DistributedPositioning
    position_tracker = std::make_unique<DistributedPositioning::DistributedPositionTracker>(
            my_id, mesh_protocol);

    // 9. AutonomousDroneAgent
    drone_agent = std::make_unique<AutonomousDroneAgent>(my_id);
    if (!drone_agent->Initialize(mesh_protocol, position_tracker)) {
        std::cerr << "Failed to initialize autonomous agent!" << std::endl;
        return false;
    }

    // 10. ConfigWatcher для hot-reload
    config_watcher = std::make_unique<SwarmSystem::ConfigWatcher>(config_manager, config_path);

    // LoRa configuration handler
    auto lora_handler = std::make_unique<SwarmSystem::LoRaConfigHandler>(comm_manager);
    config_watcher->registerSectionHandler(std::move(lora_handler));

    // UWB configuration handler
    auto uwb_handler = std::make_unique<SwarmSystem::UWBConfigHandler>(uwb_manager);
    config_watcher->registerSectionHandler(std::move(uwb_handler));

    // Security configuration handler
    auto security_handler = std::make_unique<SwarmSystem::SecurityConfigHandler>(crypto_manager);
    config_watcher->registerSectionHandler(std::move(security_handler));

    // Global callback для изменений не покрытых специальными handlers
    config_watcher->setGlobalChangeCallback([](const std::string& section, const std::string& key,
                                               const std::string& old_val, const std::string& new_val) -> bool {
        std::cout << "Global config change: " << section << "." << key
                  << " changed from '" << old_val << "' to '" << new_val << "'" << std::endl;

        // Handle specific global changes that aren't covered by handlers
        if (section == "communication") {
            std::cout << "Communication parameters updated: " << key << " = " << new_val << std::endl;
            return true; // Assume applied successfully
        }

        if (section == "system" && key == "max_drones_mvp") {
            std::cout << "WARNING: Changing drone count requires system restart!" << std::endl;
            return false; // Cannot change on-the-fly
        }

        return true; // Default: accept change
    });

    // Start ConfigWatcher
    if (!config_watcher->start()) {
        std::cerr << "Failed to start configuration watcher!" << std::endl;
        return false;
    }

    std::cout << "All systems initialized successfully!" << std::endl;
    return true;
}

bool StartAllSystems() {
    std::cout << "Starting all systems..." << std::endl;

    // Start CommunicationManager first (needed by others)
    if (!comm_manager->start()) {
        std::cerr << "Failed to start communication manager!" << std::endl;
        return false;
    }

    // Start MeshProtocol
    if (!mesh_protocol->Start()) {
        std::cerr << "Failed to start mesh protocol!" << std::endl;
        return false;
    }

    // Start autonomous agent
    if (!drone_agent->Start()) {
        std::cerr << "Failed to start autonomous agent!" << std::endl;
        return false;
    }

    std::cout << "All systems started successfully!" << std::endl;
    return true;
}

void StopAllSystems() {
    std::cout << "Stopping all systems..." << std::endl;

    if (config_watcher) {
        config_watcher->stop();
    }

    if (drone_agent) {
        drone_agent->Stop();
    }

    if (mesh_protocol) {
        mesh_protocol->Stop();
    }

    if (comm_manager) {
        comm_manager->stop();
    }

    std::cout << "All systems stopped." << std::endl;
}

void PrintSystemStatus() {
    if (!comm_manager) return;

    auto stats = comm_manager->get_communication_stats();

    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "SYSTEM STATUS" << std::endl;
    std::cout << std::string(60, '=') << std::endl;

    std::cout << "Drone ID: " << my_id << std::endl;
    std::cout << "Frequency: " << (comm_manager->get_current_frequency() / 1000000.0) << " MHz" << std::endl;
    std::cout << "RSSI: " << static_cast<int>(comm_manager->get_current_rssi()) << " dBm" << std::endl;
    std::cout << "Link Quality: " << (comm_manager->get_link_quality() * 100) << "%" << std::endl;
    std::cout << "Messages Sent: " << stats.messages_sent << std::endl;
    std::cout << "Messages Received: " << stats.messages_received << std::endl;
    std::cout << "Messages Failed: " << stats.messages_failed << std::endl;
    std::cout << "Frequency Hops: " << stats.frequency_hops << std::endl;
    std::cout << "Packet Loss: " << (stats.packet_loss_rate * 100) << "%" << std::endl;

    // Encryption status
    std::cout << "Encryption: " << (comm_manager->is_encryption_enabled() ? "ENABLED" : "DISABLED") << std::endl;

    // Crypto stats
    if (crypto_manager) {
        auto crypto_stats = crypto_manager->GetStatistics();
        std::cout << "Messages Encrypted: " << crypto_stats.messages_encrypted << std::endl;
        std::cout << "Messages Decrypted: " << crypto_stats.messages_decrypted << std::endl;
        std::cout << "Auth Failures: " << crypto_stats.authentication_failures << std::endl;
        std::cout << "Replay Attacks Blocked: " << crypto_stats.replay_attacks_detected << std::endl;
    }

    std::cout << std::string(60, '=') << std::endl;
}

void RunSystemTests() {
    std::cout << "\nRunning system tests..." << std::endl;

    // Test 1: Basic message transmission
    std::cout << "Test 1: Basic message transmission..." << std::endl;
    SwarmControl::SwarmMessage test_msg;
    test_msg.type = SwarmControl::MessageType::HEARTBEAT;
    test_msg.source_id = my_id;
    test_msg.destination_id = 0xFFFF; // Broadcast
    test_msg.payload = {'T', 'E', 'S', 'T'};
    test_msg.priority = 128;

    if (comm_manager->send_message(test_msg)) {
        std::cout << "✓ Message transmission test passed" << std::endl;
    } else {
        std::cout << "✗ Message transmission test failed" << std::endl;
    }

    // Test 2: Encryption test
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

    // Test 3: Frequency hopping
    std::cout << "Test 3: Frequency hopping..." << std::endl;
    uint32_t original_freq = comm_manager->get_current_frequency();
    std::vector<uint32_t> test_frequencies = {433175000, 868300000, 915000000};

    if (comm_manager->update_frequency_list(test_frequencies)) {
        std::cout << "✓ Frequency list update test passed" << std::endl;

        // Test frequency change
        if (comm_manager->change_frequency(868300000)) {
            std::cout << "✓ Frequency change test passed" << std::endl;
        } else {
            std::cout << "✗ Frequency change test failed" << std::endl;
        }
    } else {
        std::cout << "✗ Frequency list update test failed" << std::endl;
    }

    // Test 4: Power adaptation
    std::cout << "Test 4: Power adaptation..." << std::endl;
    if (comm_manager->set_power_level(15)) {
        std::cout << "✓ Power level test passed" << std::endl;
    } else {
        std::cout << "✗ Power level test failed" << std::endl;
    }

    std::cout << "System tests completed." << std::endl;
}

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

    // Initialize all systems
    if (!InitializeAllSystems()) {
        std::cerr << "System initialization failed!" << std::endl;
        return -1;
    }

    // Start all systems
    if (!StartAllSystems()) {
        std::cerr << "System startup failed!" << std::endl;
        StopAllSystems();
        return -1;
    }

    // Run system tests
    RunSystemTests();

    // Main operation loop
    std::cout << "\nSystem operational. Press Ctrl+C to shutdown." << std::endl;

    auto last_status_print = std::chrono::steady_clock::now();
    auto last_key_rotation = std::chrono::steady_clock::now();

    while (!g_shutdown_requested.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        auto now = std::chrono::steady_clock::now();

        // Print status every 30 seconds
        if (now - last_status_print > std::chrono::seconds(30)) {
            PrintSystemStatus();
            last_status_print = now;
        }

        // Rotate encryption keys every 10 minutes
        if (now - last_key_rotation > std::chrono::minutes(10)) {
            std::cout << "Rotating encryption keys..." << std::endl;
            if (comm_manager->rotate_encryption_key()) {
                std::cout << "Encryption key rotation successful" << std::endl;
            } else {
                std::cout << "Encryption key rotation failed" << std::endl;
            }
            last_key_rotation = now;
        }

        // Check for interference and adapt
        if (comm_manager->detect_interference()) {
            std::cout << "Interference detected - system adapting..." << std::endl;
        }

        // Adaptive power control
        comm_manager->adapt_transmission_power();
    }

    // Graceful shutdown
    std::cout << "\nShutdown requested..." << std::endl;
    StopAllSystems();

    std::cout << "System shutdown complete." << std::endl;
    return 0;
}