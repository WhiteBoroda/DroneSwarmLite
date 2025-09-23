#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <LoRa.h>
#include <EEPROM.h>
#include "DroneSystem.h"
#include "HardwareConfig.h"
#include "AutonomousDroneAgent.h"

// 🧠 НОВА КОНЦЕПЦІЯ: Автономний агент дрона
std::unique_ptr<AutonomousDroneAgent> autonomous_agent;
DistributedCommandProtocol command_protocol;
DeadReckoningNavigator navigator;
ObstacleAvoidance obstacle_avoidance;
LocalEnvironmentMap environment_map;

// Глобальні об'єкти системи дрона
DroneController drone_controller;
CommunicationNode comm_node;
SensorManager sensor_manager;
FlightControl flight_control;
VideoTransmitter video_tx;
PositioningSensor uwb_sensor;
TargetingSystemInterface targeting_interface;

// Стан дрона
DroneState current_state;
DroneID my_drone_id;
bool emergency_mode = false;
bool targeting_mode = false;
bool autonomous_mode = false;      // НОВИЙ режим автономії
bool operator_connected = false;   // Чи підключений оператор

// Останні команди та місії
DistributedCommand current_mission;
std::vector<Position3D> mission_waypoints;
size_t current_waypoint = 0;

// Таймери
unsigned long last_operator_contact = 0;
unsigned long last_swarm_contact = 0;
unsigned long last_mission_update = 0;
unsigned long last_navigation_update = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("🧠 Ініціалізація автономного дрона ESP32 🧠");

    // Читання ID дрона з EEPROM
    if (!loadDroneID()) {
        Serial.println("❌ Помилка: ID дрона не налаштований!");
        while(1) delay(1000);
    }

    Serial.printf("✅ ID дрона: %04d\n", my_drone_id);

    // Ініціалізація підсистем
    if (!initializeHardware() || !initializeCommunication() || !initializeSensors()) {
        emergency_mode = true;
        return;
    }

    // Ініціалізація автономних систем
    if (!initializeAutonomousSystems()) {
        Serial.println("❌ Помилка ініціалізації автономних систем!");
        emergency_mode = true;
        return;
    }

    Serial.println("✅ Автономний дрон готовий до роботи!");
    sendReadySignal();
}

bool initializeAutonomousSystems() {
    Serial.println("🧠 Ініціалізація систем автономії...");

    // Створення автономного агента
    autonomous_agent = std::make_unique<AutonomousDroneAgent>(my_drone_id);

    // Ініціалізація навігатора
    Position3D initial_pos(0, 0, 0); // Стартова позиція
    if (!navigator.Initialize(initial_pos)) {
        Serial.println("❌ Помилка ініціалізації навігатора");
        return false;
    }

    // Ініціалізація системи уникнення перешкод
    // obstacle_avoidance вже ініціалізований конструктором

    // Ініціалізація локальної карти
    // environment_map вже ініціалізований конструктором

    // Встановлення початкового режиму
    autonomous_mode = false;  // Спочатку чекаємо оператора
    operator_connected = false;

    Serial.println("✅ Системи автономії ініціалізовані");
    return true;
}

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

    // НОВИЙ: Основна логіка автономії
    handleAutonomousOperation(current_time);

    // Основний цикл роботи дрона
    processIncomingMessages();
    updateSensors(current_time);
    updateNavigation(current_time);
    updateMissionExecution(current_time);
    updateCommunication(current_time);
    updateVideoTransmission(current_time);
    checkSystemHealth(current_time);

    // Короткий sleep для економії енергії
    delay(10); // 100Hz основний цикл
}

void handleAutonomousOperation(unsigned long current_time) {
    // Перевірка зв'язку з оператором
    if (current_time - last_operator_contact > OPERATOR_TIMEOUT) {
        if (operator_connected) {
            Serial.println("⚠️ Втрата зв'язку з оператором - перехід в автономний режим");
            operator_connected = false;
            autonomous_mode = true;

            // Повідомляємо рой про втрату оператора
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

        // 1. Продовжуємо поточну місію якщо вона є
        if (current_mission.command_type != DistributedCommand::MOVE_TO_WAYPOINT ||
            !mission_waypoints.empty()) {

            continueCurrentMission();
        }
            // 2. Якщо немає місії - координуємось з роем
        else {
            coordinateWithSwarm();
        }

        // 3. Постійно оновлюємо локальну карту та уникаємо перешкод
        updateEnvironmentalAwareness();

        last_autonomous_decision = current_time;
    }
}

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
            // Атака може бути виконана тільки з командою оператора
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

        if (distance < 5.0) { // Досягли точки з точністю 5 метрів
            current_waypoint++;
            Serial.printf("✅ Досягнуто waypoint %d з %d\n",
                          current_waypoint, mission_waypoints.size());

            if (current_waypoint >= mission_waypoints.size()) {
                Serial.println("✅ Місія з waypoint завершена - перехід в режим очікування");
                switchToHoldingPattern();
            }
        } else {
            // Планування безпечного шляху до цілі
            auto safe_path = obstacle_avoidance.PlanSafePath(current_pos, target);
            if (!safe_path.empty()) {
                sendFlightCommand(safe_path[0]); // Летимо до першої безпечної точки
            }
        }
    }
}

void executeSearchMission() {
    // Автономний пошуковий патерн
    static int search_pattern_step = 0;
    static Position3D search_center = current_mission.area_center;
    static double search_width = 100.0; // метрів

    Position3D current_pos = navigator.GetEstimatedPosition();

    // Створюємо зигзагоподібний патерн пошуку
    Position3D next_search_point;
    next_search_point.x = search_center.x + (search_pattern_step % 2 ? search_width : -search_width);
    next_search_point.y = search_center.y + (search_pattern_step * 20); // Крок 20м
    next_search_point.z = current_mission.altitude;

    double distance = calculateDistance(current_pos, next_search_point);
    if (distance < 10.0) {
        search_pattern_step++;
        Serial.printf("🔍 Пошуковий крок %d завершено\n", search_pattern_step);
    } else {
        sendFlightCommand(next_search_point);
    }

    // Обмеження пошуку
    if (search_pattern_step > 50) { // Після 50 кроків переходимо в патрулювання
        Serial.println("🔍 Пошук завершено - перехід в патрулювання");
        switchToPatrolMode();
    }
}

void executePatrolMission() {
    static double patrol_angle = 0.0;
    Position3D patrol_center = current_mission.area_center;
    double patrol_radius = current_mission.area_radius;

    // Кругове патрулювання
    Position3D patrol_point;
    patrol_point.x = patrol_center.x + patrol_radius * cos(patrol_angle);
    patrol_point.y = patrol_center.y + patrol_radius * sin(patrol_angle);
    patrol_point.z = current_mission.altitude;

    sendFlightCommand(patrol_point);

    patrol_angle += 0.05; // Поступове збільшення кута
    if (patrol_angle > 2 * PI) {
        patrol_angle = 0.0;
        Serial.println("🛡️ Один круг патрулювання завершено");
    }
}

void coordinateWithSwarm() {
    Serial.println("🤝 Координація з роем без оператора");

    // Відправляємо свій стан іншим дронам
    SwarmStatusMessage status_msg;
    status_msg.sender_id = my_drone_id;
    status_msg.position = navigator.GetEstimatedPosition();
    status_msg.battery_level = readBatteryVoltage();
    status_msg.current_mission = current_mission;
    status_msg.is_autonomous = true;
    status_msg.can_lead = (readBatteryVoltage() > 23.0); // Можу бути лідером якщо батарея >23V

    broadcastSwarmStatus(status_msg);

    // Аналізуємо стан інших дронів і приймаємо колективне рішення
    analyzeSwarmCoordination();
}

void analyzeSwarmCoordination() {
    // Проста логіка координації рою:
    // 1. Якщо є дрон з вищим зарядом батареї - він стає тимчасовим лідером
    // 2. Всі дрони продовжують останню отриману місію
    // 3. При виявленні цілі - повідомляємо всьому рою

    static std::vector<SwarmStatusMessage> swarm_status;

    // Знаходимо найкращого кандидата в лідери
    DroneID best_leader = findBestTemporaryLeader(swarm_status);

    if (best_leader == my_drone_id) {
        Serial.println("👑 Я тимчасовий лідер рою");
        becomeTemporaryLeader();
    } else if (best_leader != 0) {
        Serial.printf("👥 Слідую за тимчасовим лідером %04d\n", best_leader);
        followTemporaryLeader(best_leader);
    }
}

void updateNavigation(unsigned long current_time) {
    if (current_time - last_navigation_update > 50) { // 20Hz оновлення навігації

        // Оновлюємо оцінку позиції на основі сенсорів
        SensorFusion sensor_data;
        sensor_data.imu_position = sensor_manager.GetIMUPosition();
        sensor_data.uwb_position = uwb_sensor.GetPosition();
        sensor_data.barometer_altitude = sensor_manager.GetAltitude();

        navigator.IntegrateSensorData(sensor_data);
        navigator.UpdatePosition(0.05); // dt = 50ms

        // Оновлюємо локальну карту
        environment_map.UpdateFromSensors();

        last_navigation_update = current_time;
    }
}

void updateMissionExecution(unsigned long current_time) {
    if (current_time - last_mission_update > 200) { // 5Hz оновлення місії

        if (autonomous_mode && autonomous_agent) {
            autonomous_agent->ExecuteCurrentMission();
        }

        last_mission_update = current_time;
    }
}

void handleSwarmMessage(const SwarmMessage& message, DroneID sender_id) {
    // Оновлення часу останнього контакту
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

        default:
            // Обробка стандартних повідомлень
            handleStandardSwarmMessage(message, sender_id);
            break;
    }
}

void handleDistributedCommand(const SwarmMessage& message, DroneID sender_id) {
    DistributedCommand* cmd = (DistributedCommand*)message.payload;

    Serial.printf("📋 Отримано розподілену команду від дрона %04d\n", sender_id);
    Serial.printf("   Тип: %d, Ціль: (%.1f, %.1f, %.1f)\n",
                  cmd->command_type, cmd->target_position.x,
                  cmd->target_position.y, cmd->target_position.z);

    // Зберігаємо команду як поточну місію
    current_mission = *cmd;

    // Якщо це команда з waypoint - створюємо траєкторію
    if (cmd->command_type == DistributedCommand::MOVE_TO_WAYPOINT) {
        mission_waypoints.clear();
        mission_waypoints.push_back(cmd->target_position);
        current_waypoint = 0;
    }

    // Якщо оператор втрачений - починаємо автономне виконання
    if (!operator_connected) {
        autonomous_mode = true;
        Serial.println("🤖 Починаю автономне виконання команди");
    }

    // Підтверджуємо отримання команди
    acknowledgeCommand(sender_id, cmd->command_id);
}

void switchToHoldingPattern() {
    Serial.println("⏸️ Перехід в режим очікування");

    // Створюємо місію кругового польоту на поточній позиції
    current_mission.command_type = DistributedCommand::LOITER_AREA;
    current_mission.area_center = navigator.GetEstimatedPosition();
    current_mission.area_radius = 50.0; // 50 метрів
    current_mission.altitude = current_mission.area_center.z;

    autonomous_mode = true;
}

void switchToPatrolMode() {
    Serial.println("🛡️ Перехід в режим патрулювання");

    current_mission.command_type = DistributedCommand::LOITER_AREA;
    current_mission.area_center = navigator.GetEstimatedPosition();
    current_mission.area_radius = 200.0; // 200 метрів радіус патрулювання
    current_mission.altitude = 100.0; // Стандартна висота патрулювання

    autonomous_mode = true;
}

void notifySwarmOperatorLost() {
    SwarmMessage notification;
    notification.type = MSG_OPERATOR_LOST;
    notification.sender_id = my_drone_id;
    notification.timestamp = millis();

    comm_node.BroadcastMessage(notification);
    Serial.println("📡 Повідомлено рой про втрату оператора");
}

// Допоміжні функції
double calculateDistance(const Position3D& a, const Position3D& b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}

void sendFlightCommand(const Position3D& target) {
    // Відправка команди на політний контролер
    FlightCommand cmd;
    cmd.target_position = target;
    cmd.speed = current_mission.speed;
    cmd.timestamp = millis();

    flight_control.ExecuteCommand(cmd);
}

DroneID findBestTemporaryLeader(const std::vector<SwarmStatusMessage>& swarm) {
    DroneID best = 0;
    double best_score = -1.0;

    for (const auto& drone : swarm) {
        if (drone.can_lead && drone.is_autonomous) {
            // Рахуємо оцінку лідерства: батарея + стабільність + позиція
            double score = drone.battery_level * 0.5 +
                           (drone.signal_strength + 120) * 0.3 +
                           (drone.is_healthy ? 20 : 0);

            if (score > best_score) {
                best_score = score;
                best = drone.sender_id;
            }
        }
    }

    // Порівнюємо з собою
    double my_score = readBatteryVoltage() * 0.5 +
                      (LoRa.rssi() + 120) * 0.3 +
                      (checkSelfHealth() ? 20 : 0);

    if (my_score > best_score) {
        return my_drone_id;
    }

    return best;
}

bool loadDroneID() {
    EEPROM.begin(512);

    // Перевірка магічного числа
    uint32_t magic = 0;
    EEPROM.get(0, magic);

    if (magic != 0xDEADBEEF) {
        Serial.println("EEPROM не ініціалізований");
        return false;
    }

    // Читання ID дрона
    EEPROM.get(4, my_drone_id);

    if (my_drone_id == 0 || my_drone_id > 9999) {
        Serial.printf("Невірний ID дрона: %d\n", my_drone_id);
        return false;
    }

    return true;
}

bool initializeHardware() {
    Serial.println("🔧 Ініціалізація обладнання...");

    // GPIO налаштування
    pinMode(LED_STATUS, OUTPUT);
    pinMode(LED_COMMUNICATION, OUTPUT);
    pinMode(LED_TARGETING, OUTPUT);
    pinMode(EMERGENCY_BUTTON, INPUT_PULLUP);

    // SPI для LoRa та UWB
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_LORA_PIN);

    // I2C для сенсорів
    Wire.begin(SDA_PIN, SCL_PIN);

    // UART для зв'язку з політним контролером
    Serial2.begin(115200, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN);

    Serial.println("✅ Обладнання ініціалізовано");
    return true;
}

bool initializeCommunication() {
    Serial.println("📡 Ініціалізація зв'язку...");

    // LoRa модуль
    LoRa.setPins(SS_LORA_PIN, RST_LORA_PIN, DIO0_LORA_PIN);

    if (!LoRa.begin(LORA_FREQUENCY)) {
        Serial.println("❌ Помилка ініціалізації LoRa");
        return false;
    }

    // Налаштування LoRa
    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    LoRa.setSignalBandwidth(LORA_BANDWIDTH);
    LoRa.setCodingRate4(LORA_CODING_RATE);
    LoRa.setTxPower(LORA_TX_POWER);
    LoRa.enableCrc();

    // Callback для прийому повідомлень
    LoRa.onReceive(onLoRaReceive);
    LoRa.receive();

    // Ініціалізація шифрування
    if (!comm_node.InitializeCrypto()) {
        Serial.println("❌ Помилка ініціалізації шифрування");
        return false;
    }

    Serial.println("✅ Зв'язок ініціалізовано");
    return true;
}

bool initializeSensors() {
    Serial.println("📊 Ініціалізація сенсорів...");

    // UWB модуль для позиціонування
    if (!uwb_sensor.Initialize()) {
        Serial.println("❌ Помилка ініціалізації UWB");
        return false;
    }

    // IMU сенсор
    if (!sensor_manager.InitializeIMU()) {
        Serial.println("❌ Помилка ініціалізації IMU");
        return false;
    }

    // Барометр
    if (!sensor_manager.InitializeBarometer()) {
        Serial.println("❌ Помилка ініціалізації барометра");
        return false;
    }

    // Сенсор батареї
    analogReadResolution(12); // 12-bit ADC

    Serial.println("✅ Сенсори ініціалізовані");
    return true;
}

void processIncomingMessages() {
    // Обробка повідомлень LoRa
    if (comm_node.HasPendingMessages()) {
        SwarmMessage message;
        DroneID sender_id;

        if (comm_node.ReceiveMessage(message, sender_id)) {
            handleSwarmMessage(message, sender_id);
        }
    }

    // Обробка команд від політного контролера
    if (Serial2.available()) {
        processFlightControllerCommands();
    }

    // Перевірка аварійної кнопки
    if (digitalRead(EMERGENCY_BUTTON) == LOW) {
        emergency_mode = true;
        Serial.println("🚨 АВАРІЙНА КНОПКА НАТИСНУТА!");
    }
}

void handleSwarmMessage(const SwarmMessage& message, DroneID sender_id) {
    switch (message.type) {
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

void handleTargetingActivation(const SwarmMessage& message) {
    if (message.target_drone_id == my_drone_id) {
        Serial.println("🎯 АКТИВАЦІЯ СИСТЕМИ ДОВЕДЕННЯ!");

        // Передаємо керування системі доведення
        targeting_mode = true;

        // Відключаємо всі зв'язки (крім системи доведення)
        comm_node.DisableCommunication();
        video_tx.DisableTransmission();

        // Активуємо систему доведення через UART/OSD
        targeting_interface.ActivateTerminalGuidance();

        // Світлодіод режиму наведення
        digitalWrite(LED_TARGETING, HIGH);

        Serial.println("📡 ЗВ'ЯЗОК ВІДКЛЮЧЕНО - АВТОНОМНИЙ РЕЖИМ");
    }
}

void handleTargetingMode() {
    // В режимі наведення дрон повністю автономний
    // Всі команди надходять тільки від системи доведення

    static unsigned long last_targeting_check = 0;
    unsigned long current_time = millis();

    if (current_time - last_targeting_check > 100) { // 10Hz
        // Перевіряємо стан системи доведення
        TargetingStatus status = targeting_interface.GetStatus();

        switch (status) {
            case TARGETING_SEARCHING:
                // Пошук цілі
                digitalWrite(LED_TARGETING, (current_time / 200) % 2); // Мигання
                break;

            case TARGETING_LOCKED:
                // Ціль зафіксована
                digitalWrite(LED_TARGETING, HIGH);
                break;

            case TARGETING_TERMINAL:
                // Фінальне наведення
                digitalWrite(LED_TARGETING, HIGH);
                // Тут вже ніякого керування - тільки фізика
                break;

            case TARGETING_COMPLETED:
                // Місія завершена (самоліквідація)
                handleSelfDestruct();
                break;

            case TARGETING_ERROR:
                // Помилка системи доведення
                emergency_mode = true;
                break;
        }

        last_targeting_check = current_time;
    }
}

void updateSensors(unsigned long current_time) {
    // Оновлення показників сенсорів
    if (current_time - last_position_update > 50) { // 20Hz для UWB
        uwb_sensor.UpdatePosition();
        last_position_update = current_time;
    }

    // Оновлення телеметрії
    if (current_time - last_telemetry > 100) { // 10Hz
        current_state.battery_voltage = readBatteryVoltage();
        current_state.temperature = sensor_manager.GetTemperature();
        current_state.altitude = sensor_manager.GetAltitude();

        last_telemetry = current_time;
    }
}

void updateCommunication(unsigned long current_time) {
    if (targeting_mode) return; // Зв'язок відключений в режимі наведення

    // Відправка heartbeat
    if (current_time - last_heartbeat > 500) { // 2Hz
        sendHeartbeat();
        last_heartbeat = current_time;
    }

    // Перевірка якості сигналу та адаптація
    int rssi = LoRa.rssi();
    if (rssi < RSSI_THRESHOLD) {
        // Збільшуємо потужність
        int current_power = LoRa.getTxPower();
        if (current_power < MAX_LORA_POWER) {
            LoRa.setTxPower(current_power + 1);
            Serial.printf("📡 Збільшення потужності LoRa: %d dBm\n", current_power + 1);
        }
    }

    // Мигання світлодіода зв'язку
    digitalWrite(LED_COMMUNICATION, (current_time / 500) % 2);
}

void updateFlightControl(unsigned long current_time) {
    if (targeting_mode) return; // Політ контролює система доведення

    // Оновлення формації
    if (current_time - last_formation_check > 200) { // 5Hz
        flight_control.UpdateFormationPosition();
        last_formation_check = current_time;
    }

    // Відправка команд на політний контролер
    flight_control.SendControlCommands();
}

void checkSystemHealth(unsigned long current_time) {
    static unsigned long last_health_check = 0;

    if (current_time - last_health_check > 1000) { // 1Hz
        bool system_healthy = true;

        // Перевірка батареї
        float battery_voltage = readBatteryVoltage();
        if (battery_voltage < BATTERY_CRITICAL_VOLTAGE) {
            Serial.printf("🚨 КРИТИЧНА БАТАРЕЯ: %.2fV\n", battery_voltage);
            emergency_mode = true;
            system_healthy = false;
        }

        // Перевірка температури
        float temperature = sensor_manager.GetTemperature();
        if (temperature > MAX_OPERATING_TEMPERATURE) {
            Serial.printf("🚨 ПЕРЕГРІВ: %.1f°C\n", temperature);
            emergency_mode = true;
            system_healthy = false;
        }

        // Статус світлодіода
        digitalWrite(LED_STATUS, system_healthy ? HIGH : LOW);

        last_health_check = current_time;
    }
}

void handleEmergencyMode() {
    static bool emergency_initialized = false;

    if (!emergency_initialized) {
        Serial.println("🚨 ВХІД В АВАРІЙНИЙ РЕЖИМ!");

        // Максимальна потужність сигналу лиха
        LoRa.setTxPower(MAX_LORA_POWER);

        // Відправка сигналу лиха
        sendEmergencySignal();

        // Зависання на поточній позиції
        flight_control.EmergencyHover();

        emergency_initialized = true;
    }

    // Швидке мигання всіх світлодіодів
    unsigned long current_time = millis();
    bool led_state = (current_time / 100) % 2;
    digitalWrite(LED_STATUS, led_state);
    digitalWrite(LED_COMMUNICATION, led_state);
    digitalWrite(LED_TARGETING, led_state);

    // Таймер до самоліквідації
    static unsigned long emergency_start = millis();
    if (current_time - emergency_start > EMERGENCY_TIMEOUT) {
        handleSelfDestruct();
    }
}

void handleSelfDestruct() {
    Serial.println("💥 ІНІЦІАЦІЯ САМОЛІКВІДАЦІЇ");

    // Відправка останнього повідомлення
    sendFinalMessage();

    // Відключення всіх систем
    comm_node.Shutdown();
    video_tx.Shutdown();
    sensor_manager.Shutdown();

    // Активація системи самознищення
    // УВАГА: Тут має бути код для реального пристрою!
    // Поки що тільки симуляція
    for (int i = 10; i > 0; i--) {
        Serial.printf("💥 Самоліквідація через %d секунд\n", i);
        delay(1000);
    }

    Serial.println("💥 СИСТЕМА ЗНИЩЕНА");

    // Повна зупинка ESP32
    esp_deep_sleep_start();
}

// Допоміжні функції
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

// Callback для LoRa
void onLoRaReceive(int packetSize) {
    if (targeting_mode) return; // Ігноруємо в режимі наведення
    comm_node.ProcessIncomingPacket(packetSize);
}

// Обробка команд з серійного порту (для налаштування)
void serialEvent() {
    while (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command.startsWith("set_drone_id ")) {
            DroneID new_id = command.substring(13).toInt();
            if (new_id > 0 && new_id <= 9999) {
                EEPROM.put(0, 0xDEADBEEF); // Magic number
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
    Serial.println("==================\n");
}