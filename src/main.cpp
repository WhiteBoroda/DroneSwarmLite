#include <iostream>
#include <signal.h>
#include <atomic>
#include <thread>
#include <chrono>
#include "../include/SwarmTypes.h"
#include "../include/DistributedPositioning.h"
#include "../include/MeshProtocol.h"
#include "../include/DistributedPositioning.h"

// 🇺🇦 Головний файл системи управління роєм дронів 🇺🇦

// Глобальна змінна для обробки сигналів
std::atomic<bool> g_shutdown_requested{false};

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

void PrintDroneStatus(const std::vector<SwarmSystem::DroneState>& swarm_status) {
    std::cout << "\n=== СТАН РОЗПОДІЛЕНОГО РОЮ ===" << std::endl;
    std::cout << "┌──────┬────────────┬─────────────┬──────────┬────────────┬──────────────┐" << std::endl;
    std::cout << "│ ID   │ Роль       │ Батарея (%) │ Зв'язок  │ Позиція    │ Автономія    │" << std::endl;
    std::cout << "├──────┼────────────┼─────────────┼──────────┼────────────┼──────────────┤" << std::endl;

    for (const auto& drone : swarm_status) {
        std::string role_str = SwarmSystem::SwarmUtils::DroneRoleToString(drone.role);
        std::string comm_str = SwarmSystem::SwarmUtils::CommunicationStatusToString(drone.comm_status);
        std::string autonomy_str = drone.is_autonomous ? "АВТОНОМНИЙ" : "КЕРОВАНИЙ";

        printf("│ %04d │ %-10s │ %8.1f    │ %-8s │ %6.1f,%4.1f │ %-12s │\n",
               drone.id, role_str.c_str(), drone.battery_level, comm_str.c_str(),
               drone.relative_position.x(), drone.relative_position.y(), autonomy_str.c_str());
    }

    std::cout << "└──────┴────────────┴─────────────┴──────────┴────────────┴──────────────┘" << std::endl;
}

void PrintMeshStatus(const MeshNetwork::NetworkStats& mesh_stats) {
    std::cout << "\n=== СТАН MESH-МЕРЕЖІ ===" << std::endl;
    std::cout << "📊 Пакетів надіслано: " << mesh_stats.packets_sent << std::endl;
    std::cout << "📨 Пакетів отримано:  " << mesh_stats.packets_received << std::endl;
    std::cout << "🔄 Пакетів переслано: " << mesh_stats.packets_forwarded << std::endl;
    std::cout << "❌ Пакетів втрачено:  " << mesh_stats.packets_dropped << std::endl;
    std::cout << "🔍 Пошуків маршрутів: " << mesh_stats.route_discoveries << std::endl;
    std::cout << "⏱️  Середня затримка:  " << mesh_stats.average_latency << " мс" << std::endl;
}

void PrintPositioningStatus(const DistributedPositioning::DistributedPositioningSystem& pos_system) {
    std::cout << "\n=== СТАН ПОЗИЦІОНУВАННЯ ===" << std::endl;

    auto anchor = pos_system.GetCurrentAnchor();
    std::cout << "⚓ Тип якоря: ";
    switch (anchor.type) {
        case DistributedPositioning::AnchorType::GEOMETRIC_CENTER:
            std::cout << "ГЕОМЕТРИЧНИЙ ЦЕНТР"; break;
        case DistributedPositioning::AnchorType::MOST_STABLE_DRONE:
            std::cout << "НАЙСТАБІЛЬНІШИЙ ДРОН (" << anchor.anchor_drone_id << ")"; break;
        case DistributedPositioning::AnchorType::EXTERNAL_REFERENCE:
            std::cout << "ЗОВНІШНІЙ РЕФЕРЕНС"; break;
        default:
            std::cout << "НЕВИЗНАЧЕНО"; break;
    }
    std::cout << std::endl;

    std::cout << "📍 Позиція якоря: (" << anchor.position.x() << ", "
              << anchor.position.y() << ", " << anchor.position.z() << ")" << std::endl;
    std::cout << "💪 Стабільність: " << (anchor.stability_score * 100) << "%" << std::endl;
    std::cout << "🎯 Точність: ±" << pos_system.GetPositioningAccuracy() << " м" << std::endl;
    std::cout << "👥 Дронів відстежується: " << pos_system.GetTrackedSwarmSize() << std::endl;
}

void PrintHelp() {
    std::cout << R"(
ДОСТУПНІ КОМАНДИ РОЗПОДІЛЕНОЇ СИСТЕМИ:
  h, help         - Показати цю довідку
  s, status       - Показати стан рою та mesh-мережі
  mesh            - Детальна інформація про mesh-топологію
  pos, position   - Стан системи позиціонування
  autonomous      - Перевести рій в автономний режим

УПРАВЛІННЯ МІСІЄЮ:
  takeoff         - Почати місію (зліт)
  formation <type>- Змінити формацію (1=клин, 2=лінія, 3=квадрат)
  waypoint <x> <y>- Рух до точки
  search <x> <y>  - Пошук в зоні
  patrol <x> <y>  - Патрулювання навколо точки

ВІДЕО ТА ЗВ'ЯЗОК:
  video <drone_id>- Перемкнути відео на дрон
  frequency       - Змінити частоту зв'язку
  power <level>   - Змінити потужність передавача

АВАРІЙНІ КОМАНДИ:
  emergency       - АВАРІЙНА ЗУПИНКА
  self_destruct   - Самоліквідація (тільки в критичних ситуаціях)

НАЛАГОДЖЕННЯ:
  simulate <count>- Симуляція N дронів
  test_mesh       - Тест mesh-зв'язку
  anchor_vote     - Голосування за новий якір

ПРИКЛАДИ:
  formation 1         # Клин
  waypoint 100 200    # Рух до точки (100, 200)
  search 500 300      # Пошук в зоні 500x300
  video 0202          # Відео з дрона 0202
  autonomous          # Повна автономія рою
)" << std::endl;
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

    // Парсинг ID дрона
    SwarmSystem::DroneID my_id = static_cast<SwarmSystem::DroneID>(std::stoul(argv[1]));
    std::string config_path = (argc > 2) ? argv[2] : "./config/swarm_config.yaml";

    if (!VALIDATE_DRONE_ID(my_id)) {
        std::cerr << "❌ Невірний ID дрона: " << my_id << std::endl;
        return 1;
    }

    std::cout << "🚁 Ініціалізація розподіленого дрона ID: " << my_id << std::endl;
    std::cout << "📁 Конфігурація: " << config_path << std::endl;

    try {
        // Створення розподіленої системи управління роєм
        SwarmSystem::DistributedSwarmManager swarm_manager(config_path, my_id);

        std::cout << "⚙️  Ініціалізація системи..." << std::endl;
        if (!swarm_manager.Initialize()) {
            std::cerr << "❌ ПОМИЛКА: Не вдалося ініціалізувати систему!" << std::endl;
            return 1;
        }

        std::cout << "🚀 Запуск системи..." << std::endl;
        if (!swarm_manager.Start()) {
            std::cerr << "❌ ПОМИЛКА: Не вдалося запустити систему!" << std::endl;
            return 1;
        }

        std::cout << "\n✅ Розподілена система управління роєм запущена успішно!" << std::endl;
        std::cout << "📡 Mesh-мережа активна" << std::endl;
        std::cout << "🗺️ Динамічне позиціонування увімкнене" << std::endl;
        std::cout << "🤖 Автономні агенти готові" << std::endl;

        auto current_phase = swarm_manager.GetCurrentPhase();
        std::cout << "🎯 Фаза місії: " << static_cast<int>(current_phase) << std::endl;

        PrintHelp();

        // Головний цикл інтерфейсу користувача
        std::string command;
        bool mission_started = false;

        while (!g_shutdown_requested) {
            std::cout << "\nmesh[" << my_id << "]> ";

            if (!std::getline(std::cin, command)) {
                break; // EOF або помилка вводу
            }

            if (command.empty()) continue;

            // Парсинг команди
            std::istringstream iss(command);
            std::string cmd;
            iss >> cmd;

            if (cmd == "h" || cmd == "help") {
                PrintHelp();
            }
            else if (cmd == "s" || cmd == "status") {
                auto swarm_status = swarm_manager.GetSwarmStatus();
                PrintDroneStatus(swarm_status);

                std::cout << "\n📊 Додаткова інформація:" << std::endl;
                std::cout << "  Фаза місії: " << static_cast<int>(swarm_manager.GetCurrentPhase()) << std::endl;
                std::cout << "  Стан зв'язку: " << SwarmSystem::SwarmUtils::CommunicationStatusToString(
                        swarm_manager.GetCommunicationStatus()) << std::endl;
                std::cout << "  Розмір рою: " << swarm_status.size() << " дронів" << std::endl;

                // Показуємо статистику системи
                auto stats = swarm_manager.GetSystemStats();
                std::cout << "  Час роботи: " << (stats.uptime_ms / 1000) << " сек" << std::endl;
                std::cout << "  Повідомлень: " << stats.messages_sent << " надіслано, "
                          << stats.messages_received << " отримано" << std::endl;
            }
            else if (cmd == "mesh") {
                auto mesh_stats = swarm_manager.GetMeshStats();
                PrintMeshStatus(mesh_stats);

                auto connected_drones = swarm_manager.GetConnectedDrones();
                std::cout << "\n🕸️ Підключені дрони (" << connected_drones.size() << "):" << std::endl;
                for (auto drone_id : connected_drones) {
                    std::cout << "  📡 " << drone_id;
                    if (drone_id == my_id) std::cout << " (Я)";
                    std::cout << std::endl;
                }
            }
            else if (cmd == "pos" || cmd == "position") {
                auto& pos_system = swarm_manager.GetPositioningSystem();
                PrintPositioningStatus(pos_system);

                auto all_positions = pos_system.GetAllDronePositions();
                std::cout << "\n📍 Позиції дронів:" << std::endl;
                for (const auto& [drone_id, position] : all_positions) {
                    std::cout << "  " << drone_id << ": ("
                              << position.x() << ", " << position.y() << ", " << position.z() << ")";
                    if (drone_id == my_id) std::cout << " ⬅️ Я";
                    std::cout << std::endl;
                }
            }
            else if (cmd == "autonomous") {
                if (swarm_manager.EnableFullAutonomy()) {
                    std::cout << "✅ Рій переведено в повністю автономний режим" << std::endl;
                    std::cout << "🤖 Дрони продовжать місію самостійно" << std::endl;
                } else {
                    std::cout << "❌ Помилка переходу в автономний режим" << std::endl;
                }
            }
            else if (cmd == "takeoff") {
                if (!mission_started) {
                    if (swarm_manager.StartMission(SwarmSystem::FormationType::WEDGE)) {
                        std::cout << "✅ Місія розпочата! Зліт в формації клин..." << std::endl;
                        mission_started = true;
                    } else {
                        std::cout << "❌ Помилка початку місії" << std::endl;
                    }
                } else {
                    std::cout << "⚠️  Місія вже розпочата" << std::endl;
                }
            }
            else if (cmd == "formation") {
                int formation_type;
                if (iss >> formation_type) {
                    SwarmSystem::FormationType formation;
                    switch (formation_type) {
                        case 1: formation = SwarmSystem::FormationType::WEDGE; break;
                        case 2: formation = SwarmSystem::FormationType::LINE_HORIZONTAL; break;
                        case 3: formation = SwarmSystem::FormationType::SQUARE; break;
                        case 4: formation = SwarmSystem::FormationType::DIAMOND; break;
                        case 5: formation = SwarmSystem::FormationType::CIRCLE; break;
                        default:
                            std::cout << "❌ Невідома формація. Використовуйте 1-5" << std::endl;
                            continue;
                    }

                    if (swarm_manager.ChangeFormation(formation)) {
                        std::cout << "✅ Формація змінена на "
                                  << SwarmSystem::SwarmUtils::FormationTypeToString(formation) << std::endl;
                    } else {
                        std::cout << "❌ Помилка зміни формації" << std::endl;
                    }
                } else {
                    std::cout << "❌ Вкажіть номер формації (1-5)" << std::endl;
                }
            }
            else if (cmd == "waypoint") {
                double x, y;
                if (iss >> x >> y) {
                    SwarmSystem::Position3D target(x, y, 100.0); // Висота 100м за замовчуванням
                    if (swarm_manager.SendWaypointCommand(target)) {
                        std::cout << "✅ Команда руху до точки (" << x << ", " << y << ") надіслана рою" << std::endl;
                    } else {
                        std::cout << "❌ Помилка відправки команди" << std::endl;
                    }
                } else {
                    std::cout << "❌ Вкажіть координати: waypoint <x> <y>" << std::endl;
                }
            }
            else if (cmd == "search") {
                double x, y;
                if (iss >> x >> y) {
                    SwarmSystem::Position3D center(x, y, 80.0);
                    if (swarm_manager.SendSearchCommand(center, 200.0, 200.0)) {
                        std::cout << "✅ Команда пошуку в зоні (" << x << ", " << y << ") надіслана рою" << std::endl;
                    } else {
                        std::cout << "❌ Помилка відправки команди пошуку" << std::endl;
                    }
                } else {
                    std::cout << "❌ Вкажіть центр зони: search <x> <y>" << std::endl;
                }
            }
            else if (cmd == "patrol") {
                double x, y;
                if (iss >> x >> y) {
                    SwarmSystem::Position3D center(x, y, 100.0);
                    if (swarm_manager.SendPatrolCommand(center, 150.0)) {
                        std::cout << "✅ Команда патрулювання навколо (" << x << ", " << y << ") надіслана рою" << std::endl;
                    } else {
                        std::cout << "❌ Помилка відправки команди патрулювання" << std::endl;
                    }
                } else {
                    std::cout << "❌ Вкажіть центр патрулювання: patrol <x> <y>" << std::endl;
                }
            }
            else if (cmd == "video") {
                SwarmSystem::DroneID video_source;
                if (iss >> video_source) {
                    if (swarm_manager.SetVideoSource(video_source)) {
                        std::cout << "✅ Відео перемкнуто на дрон " << video_source << std::endl;
                    } else {
                        std::cout << "❌ Помилка перемикання відео" << std::endl;
                    }
                } else {
                    std::cout << "❌ Вкажіть ID дрона для відео" << std::endl;
                }
            }
            else if (cmd == "anchor_vote") {
                if (swarm_manager.InitiateAnchorVoting()) {
                    std::cout << "✅ Ініційовано голосування за новий якір" << std::endl;
                } else {
                    std::cout << "❌ Помилка ініціації голосування" << std::endl;
                }
            }
            else if (cmd == "test_mesh") {
                if (swarm_manager.TestMeshConnectivity()) {
                    std::cout << "✅ Тест mesh-зв'язку пройшов успішно" << std::endl;
                } else {
                    std::cout << "❌ Проблеми з mesh-зв'язком" << std::endl;
                }
            }
            else if (cmd == "simulate") {
                int count;
                if (iss >> count && count > 0 && count <= 20) {
                    std::cout << "🎮 Запуск симуляції " << count << " віртуальних дронів..." << std::endl;
                    swarm_manager.StartSimulation(count);
                } else {
                    std::cout << "❌ Вкажіть кількість дронів (1-20)" << std::endl;
                }
            }
            else if (cmd == "emergency") {
                std::cout << "🚨 АВАРІЙНА ЗУПИНКА АКТИВОВАНА!" << std::endl;
                swarm_manager.EmergencyStop();
                std::cout << "⏸️  Всі дрони переведені в режим зависання" << std::endl;
                break;
            }
            else if (cmd == "self_destruct") {
                std::cout << "⚠️  УВАГА: Ініціація самоліквідації!" << std::endl;
                std::cout << "Підтвердіть командою: YES_DESTROY_ALL" << std::endl;

                std::string confirmation;
                std::cin >> confirmation;
                if (confirmation == "YES_DESTROY_ALL") {
                    std::cout << "💥 САМОЛІКВІДАЦІЯ АКТИВОВАНА!" << std::endl;
                    swarm_manager.InitiateSelfDestruct();
                    return 0;
                } else {
                    std::cout << "❌ Самоліквідацію скасовано" << std::endl;
                }
            }
            else if (cmd == "q" || cmd == "quit" || cmd == "exit") {
                std::cout << "Вихід з програми..." << std::endl;
                break;
            }
            else {
                std::cout << "❌ Невідома команда: " << cmd << std::endl;
                std::cout << "Введіть 'help' для списку команд" << std::endl;
            }
        }

        // Коректна зупинка системи
        std::cout << "\n⏹️  Зупинка розподіленої системи управління роєм..." << std::endl;
        swarm_manager.Stop();

    }
    catch (const std::exception& e) {
        std::cerr << "💥 КРИТИЧНА ПОМИЛКА: " << e.what() << std::endl;
        return 1;
    }
    catch (...) {
        std::cerr << "💥 НЕВІДОМА КРИТИЧНА ПОМИЛКА!" << std::endl;
        return 1;
    }

    std::cout << "\n✅ Розподілена система управління роєм зупинена успішно" << std::endl;
    std::cout << "📊 Статистика сесії збережена" << std::endl;
    std::cout << "\n🇺🇦 SLAVA UKRAINI! HEROIAM SLAVA! 🇺🇦" << std::endl;

    return 0;
}