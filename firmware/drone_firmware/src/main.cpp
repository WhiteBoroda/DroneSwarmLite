//=============================================================================
// firmware/drone_firmware/src/main.cpp
// 🇺🇦 ESP32 прошивка автономного дрона з UWB позиціонуванням 🇺🇦
//=============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <LoRa.h>
#include <EEPROM.h>
#include <esp_task_wdt.h>

// Core system includes
#include "DroneSystem.h"
#include "HardwareConfig.h"
#include "AutonomousDroneAgent.h"
#include "UWBManager_ESP32.h"  // ✅ UWB MANAGER
#include "SwarmPairing.h"      // ✅ SWARM PAIRING SYSTEM

//=============================================================================
// ✅ ГЛОБАЛЬНІ ОБ'ЄКТИ СИСТЕМИ
//=============================================================================

// 🧠 Автономний агент дрона
std::unique_ptr<AutonomousDroneAgent> autonomous_agent;
DistributedCommandProtocol command_protocol;
DeadReckoningNavigator navigator;
ObstacleAvoidance obstacle_avoidance;
LocalEnvironmentMap environment_map;

// Апаратні компоненти
DroneController drone_controller;
CommunicationNode comm_node;
SensorManager sensor_manager;
FlightControl flight_control;
VideoTransmitter video_tx;
PositioningSensor uwb_sensor;
TargetingSystemInterface targeting_interface;

// ✅ UWB POSITIONING SYSTEM (НОВИЙ)
UWBManager_ESP32* uwb_manager = nullptr;
LoRaModule lora_module;

// ✅ SWARM PAIRING SYSTEM (НОВИЙ)
SwarmPairingManager* pairing_manager = nullptr;

//=============================================================================
// ✅ СТАН ДРОНА
//=============================================================================

DroneState current_state;
DroneID my_drone_id;
bool emergency_mode = false;
bool targeting_mode = false;
bool autonomous_mode = false;
bool operator_connected = false;
bool uwb_positioning_active = false;  // ✅ UWB FLAG

// Управління місією
DistributedCommand current_mission;
std::vector<Position3D> mission_waypoints;
size_t current_waypoint = 0;

// Таймери (ВСІІ ІСНУЮЧІ)
unsigned long last_operator_contact = 0;
unsigned long last_swarm_contact = 0;
unsigned long last_mission_update = 0;
unsigned long last_navigation_update = 0;
unsigned long last_heartbeat = 0;
unsigned long last_telemetry = 0;
unsigned long last_position_update = 0;
unsigned long last_formation_check = 0;
unsigned long last_targeting_check = 0;
unsigned long last_uwb_update = 0;          // ✅ UWB TIMING
unsigned long last_position_broadcast = 0;  // ✅ POSITION SHARING

//=============================================================================
// ✅ UWB CONFIGURATION (з EEPROM) - НОВИЙ
//=============================================================================

struct UWBConfig {
    uint16_t anchor_positions[4][3];  // 4 anchors, X/Y/Z coordinates
    uint8_t tx_power;
    uint16_t antenna_delay;
    bool enabled;
} uwb_config;

//=============================================================================
// ✅ ПРОТОТИПИ ФУНКЦІЙ (ВСІ ІСНУЮЧІ + НОВІ UWB)
//=============================================================================

// Ініціалізація
bool loadDroneID();
bool saveDroneID(uint16_t id);
bool initializeHardware();
bool initializeCommunication();
bool initializeSensors();
bool initializeAutonomousSystems();
bool initializeUWBPositioning();  // ✅ UWB INIT (НОВИЙ)

// Основні цикли
void handleAutonomousOperation(unsigned long current_time);
void handleAutonomousMode(unsigned long current_time);
void handleEmergencyMode();
void handleTargetingMode();

// UWB операції (НОВІ)
void updateUWBPositioning(unsigned long current_time);
void broadcastPositionToSwarm(unsigned long current_time);
void processUWBMeasurements();

// Обробка повідомлень (ІСНУЮЧІ)
void processIncomingMessages();
void onLoRaReceive(int packet_size);
void handleSwarmMessage(const SwarmMessage& message, DroneID sender_id);
void handleDistributedCommand(const SwarmMessage& message, DroneID sender_id);
void handleTargetingActivation(const SwarmMessage& message);
void handlePositionUpdate(const SwarmMessage& message, DroneID sender_id);
void handleFormationCommand(const SwarmMessage& message);
void handleLeaderTransfer(const SwarmMessage& message);
void handleVideoSwitch(const SwarmMessage& message);
void handleHeartbeat(const SwarmMessage& message, DroneID sender_id);
void handleSwarmStatus(const SwarmMessage& message, DroneID sender_id);
void handleOperatorReconnection(const SwarmMessage& message, DroneID sender_id);
void handleTemporaryLeaderAnnouncement(const SwarmMessage& message, DroneID sender_id);
void handleCoordinationRequest(const SwarmMessage& message, DroneID sender_id);
void handleStandardSwarmMessage(const SwarmMessage& message, DroneID sender_id);

// Оновлення систем (ІСНУЮЧІ)
void updateSensors(unsigned long current_time);
void updateNavigation(unsigned long current_time);
void updateMissionExecution(unsigned long current_time);
void updateCommunication(unsigned long current_time);
void updateVideoTransmission(unsigned long current_time);
void updateFlightControl(unsigned long current_time);
void checkSystemHealth(unsigned long current_time);

// Автономні місії (ІСНУЮЧІ)
void continueCurrentMission();
void executeWaypointMission();
void executeSearchMission();
void executePatrolMission();
void coordinateWithSwarm();
void analyzeSwarmCoordination();
void updateEnvironmentalAwareness();
void switchToHoldingPattern();
void switchToPatrolMode();

// Утиліти (ІСНУЮЧІ)
float readBatteryVoltage();
void sendHeartbeat();
void sendReadySignal();
void sendEmergencySignal();
void sendFinalMessage();
void notifySwarmOperatorLost();
void acknowledgeCommand(DroneID sender_id, uint32_t command_id);
DroneID selectLocalLeader();
DroneID findBestTemporaryLeader(const std::vector<SwarmStatusMessage>& swarm_status);
void becomeTemporaryLeader();
void followTemporaryLeader(DroneID leader_id);
void broadcastSwarmStatus(const SwarmStatusMessage& status);
void sendFlightCommand(const Position3D& target);
double calculateDistance(const Position3D& a, const Position3D& b);
bool checkSelfHealth();
void processFlightControllerCommands();

// Serial interface (ІСНУЮЧІ)
void serialEvent();
void printDroneStatus();

//=============================================================================
// ✅ SETUP - ПОВНА ІНІЦІАЛІЗАЦІЯ
//=============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);

    // Watchdog timer
    esp_task_wdt_init(30, true);
    esp_task_wdt_add(NULL);

    Serial.println("🧠 Ініціалізація автономного дрона ESP32 з UWB 🧠");
    Serial.println("🇺🇦 SLAVA UKRAINI! HEROIAM SLAVA! 🇺🇦");

    // ✅ НОВИЙ КОД: Перевірка наявності ID
    if (!loadDroneID()) {
        Serial.println("\n⚠️ ID дрона не знайдено!");
        Serial.println("🤝 Вхід в AUTO-DISCOVERY MODE...\n");

        // Ініціалізуємо базове обладнання
        if (!initializeHardware()) {
            Serial.println("❌ Критична помилка обладнання!");
            while(1) {
                blinkLED(LED_STATUS, 3, 100);
                delay(1000);
                esp_task_wdt_reset();
            }
        }

        // Створюємо pairing manager
        pairing_manager = new SwarmPairingManager();

        // Запускаємо auto-discovery
        if (pairing_manager->EnterAutoDiscoveryMode()) {
            Serial.println("\n✅ PAIRING ЗАВЕРШЕНИЙ!");
            Serial.println("🔄 Перезавантаження через 3 секунди...");

            delete pairing_manager;
            pairing_manager = nullptr;

            delay(3000);
            ESP.restart();
        } else {
            Serial.println("\n❌ PAIRING НЕ ВДАВСЯ!");
            Serial.println("🔄 Повторна спроба через 5 секунд...");

            delete pairing_manager;
            pairing_manager = nullptr;

            delay(5000);
            ESP.restart();
        }

        // Не повинні потрапити сюди
        while(1) delay(1000);
    }

    Serial.printf("✅ ID дрона: %04d\n", my_drone_id);

    // Ініціалізація всіх підсистем
    if (!initializeHardware()) {
        Serial.println("❌ Критична помилка ініціалізації обладнання!");
        emergency_mode = true;
        return;
    }

    if (!initializeCommunication()) {
        Serial.println("❌ Критична помилка ініціалізації зв'язку!");
        emergency_mode = true;
        return;
    }

    if (!initializeSensors()) {
        Serial.println("❌ Критична помилка ініціалізації сенсорів!");
        emergency_mode = true;
        return;
    }

    // ✅ ІНІЦІАЛІЗАЦІЯ UWB POSITIONING (НОВИЙ)
    if (!initializeUWBPositioning()) {
        Serial.println("⚠️ UWB позиціонування недоступне, використовую dead reckoning");
        uwb_positioning_active = false;
    } else {
        Serial.println("✅ UWB позиціонування активне!");
        uwb_positioning_active = true;
    }

    // Ініціалізація автономних систем
    if (!initializeAutonomousSystems()) {
        Serial.println("❌ Помилка ініціалізації автономних систем!");
        emergency_mode = true;
        return;
    }

    Serial.println("✅✅✅ Автономний дрон готовий до роботи! ✅✅✅");
    sendReadySignal();
}

//=============================================================================
// ✅ ІНІЦІАЛІЗАЦІЯ UWB POSITIONING (НОВИЙ КОД)
//=============================================================================

bool initializeUWBPositioning() {
    Serial.println("📡 Ініціалізація UWB позиціонування...");

    // Завантажити UWB config з EEPROM
    EEPROM.get(100, uwb_config);  // Offset 100 для UWB config

    if (!uwb_config.enabled) {
        Serial.println("⚠️ UWB вимкнено в конфігурації");
        return false;
    }

    // Створити UWB Manager
    uwb_manager = new UWBManager_ESP32(my_drone_id);

    if (!uwb_manager) {
        Serial.println("❌ Помилка створення UWB Manager");
        return false;
    }

    // Ініціалізувати UWB hardware
    if (!uwb_manager->initialize()) {
        Serial.println("❌ Помилка ініціалізації UWB hardware");
        delete uwb_manager;
        uwb_manager = nullptr;
        return false;
    }

    // Завантажити позиції якорів з EEPROM
    for (int i = 0; i < 4; i++) {
        Position3D anchor_pos(
                uwb_config.anchor_positions[i][0] / 1000.0,  // мм -> м
                uwb_config.anchor_positions[i][1] / 1000.0,
                uwb_config.anchor_positions[i][2] / 1000.0
        );

        uwb_manager->addKnownAnchor(100 + i, anchor_pos);  // Anchor IDs: 100-103
        Serial.printf("  Якір %d: (%.2f, %.2f, %.2f)\n",
                      100 + i, anchor_pos.x, anchor_pos.y, anchor_pos.z);
    }

    // Налаштувати UWB параметри
    uwb_manager->setTxPower(uwb_config.tx_power);
    uwb_manager->setAntennaDelay(uwb_config.antenna_delay);

    // Запустити UWB positioning
    if (!uwb_manager->start()) {
        Serial.println("❌ Помилка запуску UWB positioning");
        delete uwb_manager;
        uwb_manager = nullptr;
        return false;
    }

    Serial.println("✅ UWB позиціонування ініціалізовано");
    return true;
}

//=============================================================================
// ✅ ІНІЦІАЛІЗАЦІЯ АВТОНОМНИХ СИСТЕМ (ІСНУЮЧИЙ КОД)
//=============================================================================

bool initializeAutonomousSystems() {
    Serial.println("🧠 Ініціалізація систем автономії...");

    // Створення автономного агента
    autonomous_agent = std::make_unique<AutonomousDroneAgent>(my_drone_id);

    // Ініціалізація навігатора
    Position3D initial_pos(0, 0, 0);
    if (!navigator.Initialize(initial_pos)) {
        Serial.println("❌ Помилка ініціалізації навігатора");
        return false;
    }

    // Встановлення початкового режиму
    autonomous_mode = false;
    operator_connected = false;

    Serial.println("✅ Системи автономії ініціалізовані");
    return true;
}

//=============================================================================
// ✅ MAIN LOOP - ІНТЕГРАЦІЯ ВСІХ СИСТЕМ
//=============================================================================

void loop() {
    unsigned long current_time = millis();

    // Критична перевірка аварійного режиму
    if (emergency_mode) {
        handleEmergencyMode();
        return;
    }

    // Перевірка режиму наведення на ціль
    if (targeting_mode) {
        handleTargetingMode();
        return;
    }

    // ✅ ОНОВЛЕННЯ UWB ПОЗИЦІОНУВАННЯ (НОВИЙ КОД)
    if (uwb_positioning_active && uwb_manager) {
        updateUWBPositioning(current_time);
        broadcastPositionToSwarm(current_time);
    }

    // Основна логіка автономії (ІСНУЮЧИЙ КОД)
    handleAutonomousOperation(current_time);

    // Основний цикл роботи дрона (ІСНУЮЧИЙ КОД)
    processIncomingMessages();
    updateSensors(current_time);
    updateNavigation(current_time);
    updateMissionExecution(current_time);
    updateCommunication(current_time);
    updateVideoTransmission(current_time);
    updateFlightControl(current_time);
    checkSystemHealth(current_time);

    // Короткий sleep для економії енергії
    delay(10); // 100Hz основний цикл
}

//=============================================================================
// ✅ UWB POSITIONING UPDATE (НОВИЙ КОД)
//=============================================================================

void updateUWBPositioning(unsigned long current_time) {
    // Оновлення UWB кожні 100ms (10Hz)
    if (current_time - last_uwb_update >= 100) {
        last_uwb_update = current_time;

        // Виконати ranging з якорями
        uwb_manager->performRangingCycle();

        // Обробити виміри та розрахувати позицію
        processUWBMeasurements();
    }
}

void processUWBMeasurements() {
    Position3D uwb_position;
    double accuracy;

    // Отримати позицію з UWB
    if (uwb_manager->getPosition(uwb_position, accuracy)) {
        // Оновити навігатор з UWB позицією
        navigator.SetPosition(uwb_position.x, uwb_position.y, uwb_position.z);

        // Якщо точність добра, використовуємо UWB як primary source
        if (accuracy < 1.0) {  // < 1 метр
            navigator.SetPositionAccuracy(accuracy);

            // Debug вивід
            static unsigned long last_print = 0;
            if (millis() - last_print > 1000) {
                Serial.printf("📡 UWB Position: (%.2f, %.2f, %.2f) ±%.2fm\n",
                              uwb_position.x, uwb_position.y, uwb_position.z, accuracy);
                last_print = millis();
            }
        }
    }
}

void broadcastPositionToSwarm(unsigned long current_time) {
    // Broadcast позицію іншим дронам кожні 500ms
    if (current_time - last_position_broadcast >= 500) {
        last_position_broadcast = current_time;

        Position3D my_position;
        double accuracy;

        if (uwb_manager->getPosition(my_position, accuracy)) {
            // Створити повідомлення з позицією
            SwarmMessage pos_msg;
            pos_msg.type = MSG_POSITION_UPDATE;
            pos_msg.sender_id = my_drone_id;
            pos_msg.timestamp = current_time;
            pos_msg.position_x = static_cast<int32_t>(my_position.x * 1000);  // м -> мм
            pos_msg.position_y = static_cast<int32_t>(my_position.y * 1000);
            pos_msg.position_z = static_cast<int32_t>(my_position.z * 1000);
            pos_msg.accuracy = static_cast<uint16_t>(accuracy * 1000);

            // Broadcast через LoRa
            comm_node.BroadcastMessage(pos_msg);
        }
    }
}

//=============================================================================
// ✅ АВТОНОМНІ ОПЕРАЦІЇ (ІСНУЮЧИЙ КОД)
//=============================================================================

void handleAutonomousOperation(unsigned long current_time) {
    // Перевірка зв'язку з оператором
    if (current_time - last_operator_contact > OPERATOR_TIMEOUT) {
        if (operator_connected) {
            Serial.println("⚠️ Втрата зв'язку з оператором - перехід в автономний режим");
            operator_connected = false;
            autonomous_mode = true;
            notifySwarmOperatorLost();
        }
    }

    // Логіка автономного режиму
    if (autonomous_mode) {
        handleAutonomousMode(current_time);
    }
}

void handleAutonomousMode(unsigned long current_time) {
    static unsigned long last_autonomous_decision = 0;

    if (current_time - last_autonomous_decision > 1000) { // 1Hz прийняття рішень
        last_autonomous_decision = current_time;

        // 1. Продовжуємо поточну місію якщо вона є
        if (current_mission.command_type != DistributedCommand::MOVE_TO_WAYPOINT ||
            !mission_waypoints.empty()) {
            continueCurrentMission();
        }
            // 2. Якщо немає місії - координуємось з роєм
        else {
            coordinateWithSwarm();
        }

        // 3. Постійно оновлюємо локальну карту та уникаємо перешкод
        updateEnvironmentalAwareness();
    }
}

//=============================================================================
// ✅ ВИКОНАННЯ АВТОНОМНИХ МІСІЙ (ІСНУЮЧИЙ КОД)
//=============================================================================

void continueCurrentMission() {
    Serial.println("🤖 Продовження автономного виконання місії");

    switch (current_mission.command_type) {
        case DistributedCommand::MOVE_TO_WAYPOINT:
            executeWaypointMission();
            break;

        case DistributedCommand::SEARCH_PATTERN:
            executeSearchMission();
            break;

        case DistributedCommand::LOITER_AREA:
            executePatrolMission();
            break;

        case DistributedCommand::ATTACK_TARGET:
            Serial.println("⚠️ Атака потребує підтвердження оператора - переходжу в патрулювання");
            switchToPatrolMode();
            break;

        default:
            Serial.println("⚠️ Невідомий тип місії - переходжу в режим очікування");
            switchToHoldingPattern();
            break;
    }
}

void executeWaypointMission() {
    if (current_waypoint < mission_waypoints.size()) {
        Position3D target = mission_waypoints[current_waypoint];
        Position3D current_pos = navigator.GetEstimatedPosition();

        double distance = calculateDistance(current_pos, target);

        if (distance < 5.0) {
            current_waypoint++;
            Serial.printf("✅ Досягнуто waypoint %d з %d\n",
                          current_waypoint, mission_waypoints.size());

            if (current_waypoint >= mission_waypoints.size()) {
                Serial.println("✅ Місія з waypoint завершена");
                switchToHoldingPattern();
            }
        } else {
            auto safe_path = obstacle_avoidance.PlanSafePath(current_pos, target);
            if (!safe_path.empty()) {
                sendFlightCommand(safe_path[0]);
            }
        }
    }
}

void executeSearchMission() {
    static int search_pattern_step = 0;
    static Position3D search_center = current_mission.area_center;
    static double search_width = 100.0;

    Position3D current_pos = navigator.GetEstimatedPosition();
    Position3D next_search_point;

    next_search_point.x = search_center.x + (search_pattern_step % 2 ? search_width : -search_width);
    next_search_point.y = search_center.y + (search_pattern_step * 20);
    next_search_point.z = current_mission.altitude;

    double distance = calculateDistance(current_pos, next_search_point);
    if (distance < 10.0) {
        search_pattern_step++;
        Serial.printf("🔍 Пошуковий крок %d завершено\n", search_pattern_step);
    } else {
        sendFlightCommand(next_search_point);
    }

    if (search_pattern_step > 50) {
        Serial.println("🔍 Пошук завершено - перехід в патрулювання");
        switchToPatrolMode();
    }
}

void executePatrolMission() {
    static double patrol_angle = 0.0;
    Position3D patrol_center = current_mission.area_center;
    double patrol_radius = current_mission.area_radius;

    Position3D patrol_point;
    patrol_point.x = patrol_center.x + patrol_radius * cos(patrol_angle);
    patrol_point.y = patrol_center.y + patrol_radius * sin(patrol_angle);
    patrol_point.z = current_mission.altitude;

    sendFlightCommand(patrol_point);

    patrol_angle += 0.05;
    if (patrol_angle > 2 * PI) {
        patrol_angle = 0.0;
        Serial.println("🛡️ Один круг патрулювання завершено");
    }
}

void coordinateWithSwarm() {
    Serial.println("🤝 Координація з роєм без оператора");

    SwarmStatusMessage status_msg;
    status_msg.sender_id = my_drone_id;
    status_msg.position = navigator.GetEstimatedPosition();
    status_msg.battery_level = readBatteryVoltage();
    status_msg.current_mission = current_mission;
    status_msg.is_autonomous = true;
    status_msg.can_lead = (readBatteryVoltage() > 23.0);

    broadcastSwarmStatus(status_msg);
    analyzeSwarmCoordination();
}

void analyzeSwarmCoordination() {
    std::vector<SwarmStatusMessage> swarm_status;
    DroneID best_leader = findBestTemporaryLeader(swarm_status);

    if (best_leader == my_drone_id) {
        Serial.println("👑 Я тимчасовий лідер рою");
        becomeTemporaryLeader();
    } else if (best_leader != 0) {
        Serial.printf("👥 Слідую за тимчасовим лідером %04d\n", best_leader);
        followTemporaryLeader(best_leader);
    }
}

void switchToHoldingPattern() {
    Serial.println("⏸️ Перехід в режим очікування");
    current_mission.command_type = DistributedCommand::LOITER_AREA;
    current_mission.area_center = navigator.GetEstimatedPosition();
    current_mission.area_radius = 50.0;
}

void switchToPatrolMode() {
    current_mission.command_type = DistributedCommand::LOITER_AREA;
    current_mission.area_center = navigator.GetEstimatedPosition();
    current_mission.area_radius = 100.0;
}

void updateEnvironmentalAwareness() {
    environment_map.Update(navigator.GetCurrentPosition());
}

//=============================================================================
// ✅ ОБРОБКА ПОВІДОМЛЕНЬ (ІСНУЮЧИЙ КОД)
//=============================================================================

void processIncomingMessages() {
    if (comm_node.HasPendingMessages()) {
        SwarmMessage message;
        DroneID sender_id;

        if (comm_node.ReceiveMessage(message, sender_id)) {
            handleSwarmMessage(message, sender_id);
        }
    }

    if (Serial2.available()) {
        processFlightControllerCommands();
    }

    if (digitalRead(EMERGENCY_BUTTON) == LOW) {
        emergency_mode = true;
        Serial.println("🚨 АВАРІЙНА КНОПКА НАТИСНУТА!");
    }
}

void onLoRaReceive(int packet_size) {
    if (targeting_mode) return;
    if (packet_size == 0) return;

    uint8_t buffer[256];
    int i = 0;
    while (LoRa.available() && i < sizeof(buffer)) {
        buffer[i++] = LoRa.read();
    }

    SwarmMessage msg;
    if (!comm_node.DecryptMessage(buffer, i, msg)) {
        return;
    }

    if (msg.sender_id == GROUND_STATION_ID) {
        last_operator_contact = millis();
        operator_connected = true;
    } else {
        last_swarm_contact = millis();
    }

    handleSwarmMessage(msg, msg.sender_id);
}

void handleSwarmMessage(const SwarmMessage& message, DroneID sender_id) {
    last_swarm_contact = millis();

    switch (message.type) {
        case MSG_DISTRIBUTED_COMMAND:
            handleDistributedCommand(message, sender_id);
            break;

        case MSG_SWARM_STATUS:
            handleSwarmStatus(message, sender_id);
            break;

        case MSG_OPERATOR_RECONNECTED:
            handleOperatorReconnection(message, sender_id);
            break;

        case MSG_TEMPORARY_LEADER:
            handleTemporaryLeaderAnnouncement(message, sender_id);
            break;

        case MSG_COORDINATION_REQUEST:
            handleCoordinationRequest(message, sender_id);
            break;

        case MSG_HEARTBEAT:
            handleHeartbeat(message, sender_id);
            break;

        case MSG_FORMATION_COMMAND:
            handleFormationCommand(message);
            break;

        case MSG_LEADER_TRANSFER:
            handleLeaderTransfer(message);
            break;

        case MSG_VIDEO_SWITCH:
            handleVideoSwitch(message);
            break;

        case MSG_EMERGENCY_STOP:
            emergency_mode = true;
            Serial.println("🚨 ОТРИМАНО КОМАНДУ АВАРІЙНОЇ ЗУПИНКИ!");
            break;

        case MSG_TARGETING_ACTIVATE:
            handleTargetingActivation(message);
            break;

        case MSG_POSITION_UPDATE:
            handlePositionUpdate(message, sender_id);
            break;

        default:
            Serial.printf("⚠️ Невідомий тип повідомлення: %d\n", message.type);
            break;
    }
}

void handleDistributedCommand(const SwarmMessage& message, DroneID sender_id) {
    DistributedCommand* cmd = (DistributedCommand*)message.payload;

    Serial.printf("📋 Отримано розподілену команду від дрона %04d\n", sender_id);
    current_mission = *cmd;

    if (cmd->command_type == DistributedCommand::MOVE_TO_WAYPOINT) {
        mission_waypoints.clear();
        mission_waypoints.push_back(cmd->target_position);
        current_waypoint = 0;
    }

    if (!operator_connected) {
        autonomous_mode = true;
        Serial.println("🤖 Починаю автономне виконання команди");
    }

    acknowledgeCommand(sender_id, cmd->command_id);
}

void handleTargetingActivation(const SwarmMessage& message) {
    if (message.target_drone_id == my_drone_id) {
        Serial.println("🎯 АКТИВАЦІЯ СИСТЕМИ ДОВЕДЕННЯ!");

        targeting_mode = true;
        comm_node.DisableCommunication();
        video_tx.DisableTransmission();
        targeting_interface.ActivateTerminalGuidance();

        // Зупинка UWB для економії енергії
        if (uwb_manager) {
            uwb_manager->stop();
            uwb_positioning_active = false;
        }

        digitalWrite(LED_TARGETING, HIGH);
        Serial.println("📡 ЗВ'ЯЗОК ВІДКЛЮЧЕНО - АВТОНОМНИЙ РЕЖИМ");
    }
}

void handlePositionUpdate(const SwarmMessage& message, DroneID sender_id) {
    if (sender_id != my_drone_id) {
        Position3D other_pos(
                message.position_x / 1000.0,
                message.position_y / 1000.0,
                message.position_z / 1000.0
        );
        environment_map.UpdateDronePosition(sender_id, other_pos);
    }
}

// Stub implementations для інших handlers
void handleHeartbeat(const SwarmMessage& message, DroneID sender_id) {}
void handleFormationCommand(const SwarmMessage& message) {}
void handleLeaderTransfer(const SwarmMessage& message) {}
void handleVideoSwitch(const SwarmMessage& message) {}
void handleSwarmStatus(const SwarmMessage& message, DroneID sender_id) {}
void handleOperatorReconnection(const SwarmMessage& message, DroneID sender_id) {
    operator_connected = true;
    autonomous_mode = false;
}
void handleTemporaryLeaderAnnouncement(const SwarmMessage& message, DroneID sender_id) {}
void handleCoordinationRequest(const SwarmMessage& message, DroneID sender_id) {}
void handleStandardSwarmMessage(const SwarmMessage& message, DroneID sender_id) {}

//=============================================================================
// ✅ ОНОВЛЕННЯ СИСТЕМ (ІСНУЮЧИЙ КОД)
//=============================================================================

void updateSensors(unsigned long current_time) {
    if (current_time - last_position_update > 50) {
        uwb_sensor.UpdatePosition();
        last_position_update = current_time;
    }

    if (current_time - last_telemetry > 100) {
        current_state.battery_voltage = readBatteryVoltage();
        current_state.temperature = sensor_manager.GetTemperature();
        current_state.altitude = sensor_manager.GetAltitude();
        last_telemetry = current_time;
    }
}

void updateNavigation(unsigned long current_time) {
    if (current_time - last_navigation_update > 50) {
        if (!uwb_positioning_active) {
            SensorFusion sensor_data;
            sensor_data.imu_position = sensor_manager.GetIMUPosition();
            sensor_data.uwb_position = uwb_sensor.GetPosition();
            sensor_data.barometer_altitude = sensor_manager.GetAltitude();

            navigator.IntegrateSensorData(sensor_data);
            navigator.UpdatePosition(0.05);
        }

        environment_map.UpdateFromSensors();
        last_navigation_update = current_time;
    }
}

void updateMissionExecution(unsigned long current_time) {
    if (current_time - last_mission_update > 200) {
        if (autonomous_mode && autonomous_agent) {
            autonomous_agent->ExecuteCurrentMission();
        }
        last_mission_update = current_time;
    }
}

void updateCommunication(unsigned long current_time) {
    if (targeting_mode) return;

    if (current_time - last_heartbeat > 500) {
        sendHeartbeat();
        last_heartbeat = current_time;
    }

    int rssi = LoRa.rssi();
    if (rssi < RSSI_THRESHOLD) {
        int current_power = LoRa.getTxPower();
        if (current_power < MAX_LORA_POWER) {
            LoRa.setTxPower(current_power + 1);
            Serial.printf("📡 Збільшення потужності LoRa: %d dBm\n", current_power + 1);
        }
    }

    digitalWrite(LED_COMMUNICATION, (current_time / 500) % 2);
}

void updateVideoTransmission(unsigned long current_time) {
    if (video_tx.IsActive()) {
        video_tx.Update();
    }
}

void updateFlightControl(unsigned long current_time) {
    if (targeting_mode) return;

    if (current_time - last_formation_check > 200) {
        flight_control.UpdateFormationPosition();
        last_formation_check = current_time;
    }

    flight_control.SendControlCommands();
}

void checkSystemHealth(unsigned long current_time) {
    static unsigned long last_health_check = 0;

    if (current_time - last_health_check > 1000) {
        bool system_healthy = true;

        float battery_voltage = readBatteryVoltage();
        if (battery_voltage < BATTERY_CRITICAL_VOLTAGE) {
            Serial.printf("🚨 КРИТИЧНА БАТАРЕЯ: %.2fV\n", battery_voltage);
            emergency_mode = true;
            system_healthy = false;
        }

        float temperature = sensor_manager.GetTemperature();
        if (temperature > MAX_OPERATING_TEMPERATURE) {
            Serial.printf("🚨 ПЕРЕГРІВ: %.1f°C\n", temperature);
            emergency_mode = true;
            system_healthy = false;
        }

        digitalWrite(LED_STATUS, system_healthy ? HIGH : LOW);
        last_health_check = current_time;
    }
}

//=============================================================================
// ✅ АВАРІЙНИЙ РЕЖИМ ТА TARGETING MODE (ІСНУЮЧИЙ КОД)
//=============================================================================

void handleEmergencyMode() {
    static bool emergency_initialized = false;

    if (!emergency_initialized) {
        Serial.println("🚨 ВХІД В АВАРІЙНИЙ РЕЖИМ!");
        LoRa.setTxPower(MAX_LORA_POWER);
        sendEmergencySignal();
        flight_control.EmergencyHover();

        if (uwb_manager) {
            uwb_manager->stop();
            uwb_positioning_active = false;
        }

        emergency_initialized = true;
    }

    unsigned long current_time = millis();
    bool led_state = (current_time / 100) % 2;
    digitalWrite(LED_STATUS, led_state);
    digitalWrite(LED_COMMUNICATION, led_state);
    digitalWrite(LED_TARGETING, led_state);

    static unsigned long emergency_start = millis();
    if (current_time - emergency_start > EMERGENCY_TIMEOUT) {
        handleSelfDestruct();
    }
}

void handleSelfDestruct() {
    Serial.println("💥 ІНІЦІАЦІЯ САМОЛІКВІДАЦІЇ");
    sendFinalMessage();

    if (uwb_manager) {
        uwb_manager->stop();
        delete uwb_manager;
        uwb_manager = nullptr;
    }

    comm_node.Shutdown();
    video_tx.Shutdown();
    sensor_manager.Shutdown();

    for (int i = 10; i > 0; i--) {
        Serial.printf("💥 Самоліквідація через %d секунд\n", i);
        delay(1000);
    }

    Serial.println("💥 СИСТЕМА ЗНИЩЕНА");
    esp_deep_sleep_start();
}

void handleTargetingMode() {
    static unsigned long last_targeting_check = 0;
    unsigned long current_time = millis();

    if (current_time - last_targeting_check > 100) {
        TargetingStatus status = targeting_interface.GetStatus();

        switch (status) {
            case TARGETING_SEARCHING:
                digitalWrite(LED_TARGETING, (current_time / 200) % 2);
                break;

            case TARGETING_LOCKED:
                digitalWrite(LED_TARGETING, HIGH);
                break;

            case TARGETING_TERMINAL:
                digitalWrite(LED_TARGETING, HIGH);
                break;

            case TARGETING_COMPLETED:
                handleSelfDestruct();
                break;

            case TARGETING_ERROR:
                emergency_mode = true;
                break;
        }

        last_targeting_check = current_time;
    }
}

//=============================================================================
// ✅ ІНІЦІАЛІЗАЦІЯ HARDWARE (ІСНУЮЧИЙ КОД)
//=============================================================================

bool loadDroneID() {
    EEPROM.begin(512);

    uint32_t magic = 0;
    EEPROM.get(0, magic);

    if (magic != 0xDEADBEEF) {
        Serial.println("EEPROM не ініціалізований");
        return false;
    }

    EEPROM.get(4, my_drone_id);

    if (my_drone_id == 0 || my_drone_id > 9999) {
        Serial.printf("Невірний ID дрона: %d\n", my_drone_id);
        return false;
    }

    return true;
}

bool initializeHardware() {
    Serial.println("🔧 Ініціалізація обладнання...");

    pinMode(LED_STATUS, OUTPUT);
    pinMode(LED_COMMUNICATION, OUTPUT);
    pinMode(LED_TARGETING, OUTPUT);
    pinMode(EMERGENCY_BUTTON, INPUT_PULLUP);

    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_LORA_PIN);
    Wire.begin(SDA_PIN, SCL_PIN);
    Serial2.begin(115200, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN);

    Serial.println("✅ Обладнання ініціалізовано");
    return true;
}

bool initializeCommunication() {
    Serial.println("📡 Ініціалізація зв'язку...");

    LoRa.setPins(SS_LORA_PIN, RST_LORA_PIN, DIO0_LORA_PIN);

    if (!LoRa.begin(LORA_FREQUENCY)) {
        Serial.println("❌ Помилка ініціалізації LoRa");
        return false;
    }

    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    LoRa.setSignalBandwidth(LORA_BANDWIDTH);
    LoRa.setCodingRate4(LORA_CODING_RATE);
    LoRa.setTxPower(LORA_TX_POWER);
    LoRa.enableCrc();

    LoRa.onReceive(onLoRaReceive);
    LoRa.receive();

    if (!comm_node.InitializeCrypto()) {
        Serial.println("❌ Помилка ініціалізації шифрування");
        return false;
    }

    Serial.println("✅ Зв'язок ініціалізовано");
    return true;
}

bool initializeSensors() {
    Serial.println("📊 Ініціалізація сенсорів...");

    if (!uwb_sensor.Initialize()) {
        Serial.println("❌ Помилка ініціалізації UWB");
        return false;
    }

    if (!sensor_manager.InitializeIMU()) {
        Serial.println("❌ Помилка ініціалізації IMU");
        return false;
    }

    if (!sensor_manager.InitializeBarometer()) {
        Serial.println("❌ Помилка ініціалізації барометра");
        return false;
    }

    analogReadResolution(12);

    Serial.println("✅ Сенсори ініціалізовані");
    return true;
}

//=============================================================================
// ✅ УТИЛІТИ (ІСНУЮЧИЙ КОД)
//=============================================================================

float readBatteryVoltage() {
    int raw = analogRead(BATTERY_VOLTAGE_PIN);
    return (raw / 4095.0) * BATTERY_VOLTAGE_DIVIDER * 3.3;
}

void sendHeartbeat() {
    SwarmMessage heartbeat;
    heartbeat.type = MSG_HEARTBEAT;
    heartbeat.sender_id = my_drone_id;
    heartbeat.timestamp = millis();
    heartbeat.battery_level = readBatteryVoltage();
    heartbeat.signal_strength = LoRa.rssi();

    comm_node.BroadcastMessage(heartbeat);
}

void sendReadySignal() {
    SwarmMessage ready;
    ready.type = MSG_DRONE_READY;
    ready.sender_id = my_drone_id;
    ready.timestamp = millis();

    comm_node.BroadcastMessage(ready);
    Serial.println("📡 Сигнал готовності надіслано");
}

void sendEmergencySignal() {
    SwarmMessage emergency;
    emergency.type = MSG_EMERGENCY_SIGNAL;
    emergency.sender_id = my_drone_id;
    emergency.timestamp = millis();

    for (int i = 0; i < 5; i++) {
        comm_node.BroadcastMessage(emergency);
        delay(100);
    }

    Serial.println("🚨 Сигнал лиха надіслано");
}

void sendFinalMessage() {
    SwarmMessage final_msg;
    final_msg.type = MSG_FINAL_TRANSMISSION;
    final_msg.sender_id = my_drone_id;
    final_msg.timestamp = millis();

    comm_node.BroadcastMessage(final_msg);
    Serial.println("📡 Останнє повідомлення надіслано");
}

void notifySwarmOperatorLost() {
    SwarmMessage notify;
    notify.type = MSG_OPERATOR_LOST;
    notify.sender_id = my_drone_id;
    notify.timestamp = millis();

    comm_node.BroadcastMessage(notify);
}

void acknowledgeCommand(DroneID sender_id, uint32_t command_id) {
    // Stub implementation
}

DroneID selectLocalLeader() {
    std::vector<SwarmDroneInfo> nearby_drones = comm_node.GetNearbyDrones();

    if (nearby_drones.empty()) {
        return my_drone_id;
    }

    DroneID best = my_drone_id;
    double best_score = 0;

    for (const auto& drone : nearby_drones) {
        if (drone.last_contact_time > 0) {
            double score = drone.battery_level * 0.5 +
                           (drone.signal_strength + 120) * 0.3 +
                           (drone.is_stable ? 20 : 0);

            if (score > best_score) {
                best_score = score;
                best = drone.sender_id;
            }
        }
    }

    double my_score = readBatteryVoltage() * 0.5 +
                      (LoRa.rssi() + 120) * 0.3 +
                      (checkSelfHealth() ? 20 : 0);

    if (my_score > best_score) {
        return my_drone_id;
    }

    return best;
}

DroneID findBestTemporaryLeader(const std::vector<SwarmStatusMessage>& swarm_status) {
    return my_drone_id;
}

void becomeTemporaryLeader() {}
void followTemporaryLeader(DroneID leader_id) {}
void broadcastSwarmStatus(const SwarmStatusMessage& status) {}
void sendFlightCommand(const Position3D& target) {}

double calculateDistance(const Position3D& a, const Position3D& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

bool checkSelfHealth() {
    return (readBatteryVoltage() > BATTERY_LOW_VOLTAGE);
}

void processFlightControllerCommands() {
    // Stub implementation
}

//=============================================================================
// ✅ SERIAL INTERFACE (ІСНУЮЧИЙ КОД)
//=============================================================================

void serialEvent() {
    while (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command.startsWith("set_drone_id ")) {
            DroneID new_id = command.substring(13).toInt();
            if (new_id > 0 && new_id <= 9999) {
                EEPROM.put(0, 0xDEADBEEF);
                EEPROM.put(4, new_id);
                EEPROM.commit();
                Serial.printf("✅ ID дрона встановлено: %04d\n", new_id);
                Serial.println("🔄 Перезавантаження через 3 секунди...");
                delay(3000);
                ESP.restart();
            } else {
                Serial.println("❌ Невірний ID дрона (1-9999)");
            }
        }
        else if (command == "status") {
            printDroneStatus();
        }
        else if (command == "emergency") {
            emergency_mode = true;
        }
        else if (command == "restart") {
            ESP.restart();
        }
    }
}

void printDroneStatus() {
    Serial.println("\n=== СТАН ДРОНА ===");
    Serial.printf("ID: %04d\n", my_drone_id);
    Serial.printf("Батарея: %.2fV\n", readBatteryVoltage());
    Serial.printf("Температура: %.1f°C\n", sensor_manager.GetTemperature());
    Serial.printf("RSSI: %d dBm\n", LoRa.rssi());
    Serial.printf("Режим: %s\n", targeting_mode ? "НАВЕДЕННЯ" : (emergency_mode ? "АВАРІЙНИЙ" : "НОРМАЛЬНИЙ"));
    Serial.printf("UWB: %s\n", uwb_positioning_active ? "АКТИВНИЙ" : "ВИМКНЕНО");
    Serial.println("==================\n");
}

// ✅ LED Blink Helper (НОВИЙ)
void blinkLED(int pin, int times, int delay_ms) {
    for (int i = 0; i < times; i++) {
        digitalWrite(pin, HIGH);
        delay(delay_ms);
        digitalWrite(pin, LOW);
        delay(delay_ms);
    }
}
