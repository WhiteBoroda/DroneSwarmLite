// src/main.cpp - ИСПРАВЛЕН: заменить DistributedSwarmManager на компоненты
// 🇺🇦 Головний файл системи управління роєм дронів 🇺🇦

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

// Основные компоненты системы (вместо DistributedSwarmManager!)
std::unique_ptr<ConfigManager> config_manager;
std::unique_ptr<SwarmControl::CommunicationManager> comm_manager;
std::unique_ptr<MeshNetwork::SwarmMeshProtocol> mesh_protocol;
std::unique_ptr<DistributedPositioning::DistributedPositionTracker> position_tracker;
std::unique_ptr<AutonomousDroneAgent> drone_agent;
std::unique_ptr<UWBManager> uwb_manager;
std::unique_ptr<CryptoManager> crypto_manager;

void SignalHandler(int signal) {
    std::cout << "\nОтримано сигнал " << signal << ". Ініціація зупинки..." << std::endl;
    g_shutdown_requested = true;
}

void PrintBanner() {
    std::cout << R"(
╔══════════════════════════════════════════════════════════════╗
║                РОЗПОДІЛЕНА СИСТЕМА УПРАВЛІННЯ РОЄМ           ║
║                         v2.0.0-MESH                         ║
║                                                              ║
║              🇺🇦 SLAVA UKRAINI! HEROIAM SLAVA! 🇺🇦            ║
║                                                              ║
║  ⚡ Mesh-мережа      ⚓ Динамічний якір     🤖 Автономія     ║
║  🔒 Шифрування      🛡️ Живучість          📡 Без GPS       ║
╚══════════════════════════════════════════════════════════════╝
)" << std::endl;
}

// Инициализация всех компонентов
bool InitializeAllSystems() {
    std::cout << "⚙️ Ініціалізація розподіленої системи..." << std::endl;

    // 1. ConfigManager
    config_manager = std::make_unique<ConfigManager>(config_path);
    if (!config_manager->IsLoaded()) {
        std::cerr << "❌ Не вдалося завантажити конфігурацію!" << std::endl;
        return false;
    }

    // 2. CryptoManager
    crypto_manager = std::make_unique<CryptoManager>();
    if (!crypto_manager->Initialize()) {
        std::cerr << "❌ Помилка ініціалізації криптографії!" << std::endl;
        return false;
    }

    // 3. CommunicationManager
    comm_manager = std::make_unique<SwarmControl::CommunicationManager>(my_id, config_path);
    if (!comm_manager->initialize()) {
        std::cerr << "❌ Помилка ініціалізації зв'язку!" << std::endl;
        return false;
    }

    // 4. UWBManager
    uwb_manager = std::make_unique<UWBManager>(my_id);
    if (!uwb_manager->Initialize()) {
        std::cerr << "❌ Помилка ініціалізації UWB!" << std::endl;
        return false;
    }

    // 5. MeshProtocol
    mesh_protocol = std::make_unique<MeshNetwork::SwarmMeshProtocol>(my_id);
    if (!mesh_protocol->Initialize()) {
        std::cerr << "❌ Помилка ініціалізації Mesh-мережі!" << std::endl;
        return false;
    }

    // 6. DistributedPositioning
    position_tracker = std::make_unique<DistributedPositioning::DistributedPositionTracker>(
            my_id, mesh_protocol);

    // 7. AutonomousDroneAgent
    drone_agent = std::make_unique<AutonomousDroneAgent>(my_id);
    if (!drone_agent->Initialize(mesh_protocol, position_tracker)) {
        std::cerr << "❌ Помилка ініціалізації автономного агента!" << std::endl;
        return false;
    }

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
        std::cout << "🔄 Глобальное изменение конфигурации: " << section << "." << key
                  << " изменен с '" << old_val << "' на '" << new_val << "'" << std::endl;

        // Handle specific global changes that aren't covered by handlers
        if (section == "communication") {
            std::cout << "📡 Обновлены параметры связи: " << key << " = " << new_val << std::endl;
            return true; // Assume applied successfully
        }

        if (section == "system" && key == "max_drones_mvp") {
            std::cout << "🚨 Изменение количества дронов требует перезапуска системы!" << std::endl;
            return false; // Cannot apply without restart
        }

        std::cout << "⚠️ Неизвестное изменение конфигурации: " << section << "." << key << std::endl;
        return false;
    });

    // System restart callback для критических изменений
    config_watcher->setSystemRestartCallback([](const std::string& reason) {
        std::cout << "🚨 ТРЕБУЕТСЯ ПЕРЕЗАПУСК СИСТЕМЫ: " << reason << std::endl;
        std::cout << "⚠️ Некоторые изменения конфигурации требуют полного перезапуска!" << std::endl;
        std::cout << "   Нажмите 'r' для перезапуска или продолжите с текущими настройками..." << std::endl;
        // Можно установить флаг для перезапуска или уведомить оператора
    });

    std::cout << "✅ ConfigWatcher инициализирован" << std::endl;
    return true;
}

    std::cout << "✅ Всі компоненти ініціалізовано!" << std::endl;
    return true;
}

bool StartAllSystems() {
    std::cout << "🚀 Запуск всіх систем..." << std::endl;

    if (!comm_manager->start()) {
        std::cerr << "❌ Не вдалося запустити зв'язок!" << std::endl;
        return false;
    }

    if (!uwb_manager->Start()) {
        std::cerr << "❌ Не вдалося запустити UWB!" << std::endl;
        return false;
    }

    if (!mesh_protocol->Start()) {
        std::cerr << "❌ Не вдалося запустити Mesh!" << std::endl;
        return false;
    }

    if (!drone_agent->Start()) {
        std::cerr << "❌ Не вдалося запустити агент!" << std::endl;
        return false;
    }

    std::cout << "✅ Система запущена!" << std::endl;
    return true;
}

void StopAllSystems() {
    std::cout << "🛑 Зупинка систем..." << std::endl;

    if (drone_agent) drone_agent->Stop();
    if (mesh_protocol) mesh_protocol->Stop();
    if (uwb_manager) uwb_manager->Stop();
    if (comm_manager) comm_manager->stop();

    std::cout << "✅ Всі системи зупинені" << std::endl;
}

// Функции для статуса (упрощенные версии)
void PrintDroneStatus() {
    std::cout << "\n=== СТАН ДРОНА ===" << std::endl;
    std::cout << "🚁 ID: " << my_id << std::endl;

    if (comm_manager) {
        std::cout << "📡 RSSI: " << static_cast<int>(comm_manager->get_current_rssi()) << " dBm" << std::endl;
        std::cout << "🔗 Якість: " << static_cast<int>(comm_manager->get_link_quality() * 100) << "%" << std::endl;
    }

    if (mesh_protocol) {
        auto stats = mesh_protocol->GetNetworkStats();
        std::cout << "🕸️ Mesh: " << stats.packets_sent << "/" << stats.packets_received << " пакетів" << std::endl;
    }

    if (drone_agent) {
        auto state = drone_agent->GetMissionState();
        std::cout << "🤖 Стан: " << static_cast<int>(state) << std::endl;
    }
}

void PrintHelp() {
    std::cout << R"(
ДОСТУПНІ КОМАНДИ:
  h, help         - Показати довідку
  s, status       - Показати стан дрона
  q, quit, exit   - Вихід
  emergency       - АВАРІЙНА ЗУПИНКА
  test_mesh       - Тест mesh-зв'язку
)" << std::endl;
}

void processDistributedCommands() {
    // ✅ РЕАЛЬНАЯ обработка распределенных команд
    if (mesh_protocol && mesh_protocol->HasPendingCommands()) {
        auto commands = mesh_protocol->GetPendingCommands();

        for (const auto& command : commands) {
            if (autonomous_agent) {
                bool success = autonomous_agent->ProcessDistributedCommand(command);

                if (success) {
                    std::cout << "✅ Распределенная команда выполнена: тип "
                              << command.command_type << " от дрона " << command.originator_id << std::endl;
                } else {
                    std::cout << "❌ Ошибка выполнения команды от дрона " << command.originator_id << std::endl;
                }
            }
        }

        // Очищаем обработанные команды
        mesh_protocol->ClearProcessedCommands();
    }

    // Проверяем команды от ground station через LoRa
    if (comm_manager && comm_manager->has_incoming_messages()) {
        // Обрабатываем команды от наземной станции
        processGroundStationCommands();
    }
}

void processGroundStationCommands() {
    // ✅ РЕАЛЬНАЯ обработка команд от ground station
    while (comm_manager->has_incoming_messages()) {
        SwarmControl::SwarmMessage message;
        if (comm_manager->get_next_message(message)) {

            switch (message.type) {
                case SwarmControl::MessageType::COMMAND:
                    handleGroundStationCommand(message);
                    break;

                case SwarmControl::MessageType::FORMATION_UPDATE:
                    handleFormationUpdate(message);
                    break;

                case SwarmControl::MessageType::MISSION_UPDATE:
                    handleMissionUpdate(message);
                    break;

                case SwarmControl::MessageType::EMERGENCY:
                    handleEmergencyCommand(message);
                    break;

                default:
                    // Передаем другие сообщения mesh протоколу
                    if (mesh_protocol) {
                        mesh_protocol->ProcessExternalMessage(message);
                    }
                    break;
            }
        }
    }
}

void handleGroundStationCommand(const SwarmControl::SwarmMessage& message) {
    // Декодируем команду из payload
    if (message.payload.size() >= sizeof(DistributedCommand)) {
        DistributedCommand command;
        std::memcpy(&command, message.payload.data(), sizeof(DistributedCommand));

        // Выполняем команду
        if (autonomous_agent) {
            autonomous_agent->ProcessDistributedCommand(command);
        }

        std::cout << "📡 Команда от ground station выполнена" << std::endl;
    }
}

int main(int argc, char* argv[]) {
    // Перевірка аргументів
    if (argc < 2) {
        std::cerr << "Використання: " << argv[0] << " <drone_id> [config_path]" << std::endl;
        std::cerr << "Приклад: " << argv[0] << " 0201 ./config/swarm_config.yaml" << std::endl;
        return 1;
    }

    // Встановлення обробників сигналів
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);

    PrintBanner();

    // Парсинг аргументів
    my_id = static_cast<DroneID>(std::stoul(argv[1]));
    config_path = (argc > 2) ? argv[2] : "./config/swarm_config.yaml";

    if (!VALIDATE_DRONE_ID(my_id)) {
        std::cerr << "❌ Невірний ID дрона: " << my_id << std::endl;
        return 1;
    }

    std::cout << "🚁 Ініціалізація розподіленого дрона ID: " << my_id << std::endl;
    std::cout << "📁 Конфігурація: " << config_path << std::endl;

    try {
        // Инициализация системы
        if (!InitializeAllSystems()) {
            std::cerr << "❌ ПОМИЛКА: Не вдалося ініціалізувати систему!" << std::endl;
            return 1;
        }

        // Запуск системы
        if (!StartAllSystems()) {
            std::cerr << "❌ ПОМИЛКА: Не вдалося запустити систему!" << std::endl;
            StopAllSystems();
            return 1;
        }

        std::cout << "\n✅ Розподілена система управління роєм запущена успішно!" << std::endl;
        std::cout << "📡 Mesh-мережа активна" << std::endl;
        std::cout << "🗺️ Динамічне позиціонування увімкнене" << std::endl;
        std::cout << "🤖 Автономний агент готовий" << std::endl;

        PrintHelp();

        // Основной цикл интерфейса
        std::string command;
        while (!g_shutdown_requested) {
            std::cout << "\nmesh[" << my_id << "]> ";

            if (!std::getline(std::cin, command)) {
                break;
            }

            if (command.empty()) continue;

            std::istringstream iss(command);
            std::string cmd;
            iss >> cmd;

            if (cmd == "h" || cmd == "help") {
                PrintHelp();
            }
            else if (cmd == "s" || cmd == "status") {
                PrintDroneStatus();
            }
            else if (cmd == "test_mesh") {
                if (mesh_protocol) {
                    // Простой тест mesh
                    std::vector<uint8_t> test_data = {'T', 'E', 'S', 'T'};
                    if (mesh_protocol->BroadcastMessage(test_data, 128)) {
                        std::cout << "✅ Тест mesh-зв'язку успішний" << std::endl;
                    } else {
                        std::cout << "❌ Проблема з mesh-зв'язком" << std::endl;
                    }
                }
            }
            else if (cmd == "emergency") {
                std::cout << "🚨 АВАРІЙНА ЗУПИНКА!" << std::endl;
                break;
            }
            else if (cmd == "q" || cmd == "quit" || cmd == "exit") {
                std::cout << "Вихід..." << std::endl;
                break;
            }
            else {
                std::cout << "❌ Невідома команда: " << cmd << std::endl;
                std::cout << "Введіть 'help' для списку команд" << std::endl;
            }
        }

        // Корректное завершение
        std::cout << "\n⏹️ Зупинка розподіленої системи..." << std::endl;
        StopAllSystems();

    } catch (const std::exception& e) {
        std::cerr << "💥 КРИТИЧНА ПОМИЛКА: " << e.what() << std::endl;
        StopAllSystems();
        return 1;
    }

    std::cout << "\n✅ Розподілена система зупинена успішно" << std::endl;


    return 0;
}