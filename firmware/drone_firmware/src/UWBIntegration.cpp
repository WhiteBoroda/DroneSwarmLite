//=============================================================================
// firmware/drone_firmware/src/UWBIntegration.cpp
// Допоміжні функції для інтеграції UWB з іншими системами дрона
// 🇺🇦 Slava Ukraini! 🇺🇦
//=============================================================================

#include "UWBIntegration.h"
#include "AerialUWBManager_ESP32.h"
#include "LoRaModule.h"
#include "DroneController.h"
#include "FlightControl.h"

using namespace AerialUWB_ESP32;

//=============================================================================
// ✅ ІНТЕГРАЦІЯ З LORA ДЛЯ ОБМІНУ ПОЗИЦІЯМИ
//=============================================================================

// Структура пакету позиції для LoRa
struct __attribute__((packed)) LoRaPositionPacket {
    uint8_t packet_type;        // 0x01 = Position Update
    uint16_t sender_id;
    uint32_t timestamp_ms;
    float position_x;
    float position_y;
    float position_z;
    float velocity_x;
    float velocity_y;
    float velocity_z;
    float accuracy;
    uint8_t mode;               // AerialMode
    uint16_t crc;
};

bool sendPositionViaLoRa(AerialUWBManager_ESP32* uwb_manager, LoRaModule* lora) {
    if (!uwb_manager || !lora) {
        return false;
    }

    // Підготовка пакету
    LoRaPositionPacket packet;
    packet.packet_type = 0x01;
    packet.sender_id = uwb_manager->getDroneId();
    packet.timestamp_ms = millis();

    Position3D_ESP32 pos = uwb_manager->getCurrentPosition();
    packet.position_x = pos.x;
    packet.position_y = pos.y;
    packet.position_z = pos.z;
    packet.accuracy = pos.accuracy_meters;

    // Отримуємо швидкість з motion tracking
    Position3D_ESP32 vel = uwb_manager->getCurrentVelocity();
    packet.velocity_x = vel.x;
    packet.velocity_y = vel.y;
    packet.velocity_z = vel.z;

    packet.mode = (uint8_t)uwb_manager->getCurrentAerialMode();

    // Розрахунок CRC
    packet.crc = calculateCRC16((uint8_t*)&packet, sizeof(packet) - 2);

    // Відправка через LoRa
    return lora->sendPacket((uint8_t*)&packet, sizeof(packet));
}

bool receivePositionViaLoRa(AerialUWBManager_ESP32* uwb_manager,
                            LoRaModule* lora,
                            LoRaPositionPacket& packet) {
    if (!uwb_manager || !lora) {
        return false;
    }

    // Перевірка наявності пакету
    if (!lora->available()) {
        return false;
    }

    // Отримання пакету
    uint8_t buffer[sizeof(LoRaPositionPacket)];
    int bytes_read = lora->receivePacket(buffer, sizeof(buffer));

    if (bytes_read != sizeof(LoRaPositionPacket)) {
        return false;
    }

    memcpy(&packet, buffer, sizeof(packet));

    // Перевірка CRC
    uint16_t calculated_crc = calculateCRC16((uint8_t*)&packet, sizeof(packet) - 2);
    if (calculated_crc != packet.crc) {
        return false;
    }

    // Перевірка типу пакету
    if (packet.packet_type != 0x01) {
        return false;
    }

    // Додаємо інформацію про дрон до UWB системи
    Position3D_ESP32 pos;
    pos.x = packet.position_x;
    pos.y = packet.position_y;
    pos.z = packet.position_z;
    pos.accuracy_meters = packet.accuracy;
    pos.timestamp_us = packet.timestamp_ms * 1000;

    Position3D_ESP32 vel;
    vel.x = packet.velocity_x;
    vel.y = packet.velocity_y;
    vel.z = packet.velocity_z;

    uwb_manager->addOrUpdateKnownNode(packet.sender_id, pos);

    return true;
}

//=============================================================================
// ✅ ІНТЕГРАЦІЯ З FLIGHT CONTROLLER
//=============================================================================

bool sendPositionToFlightController(AerialUWBManager_ESP32* uwb_manager,
                                    FlightControl* flight_control) {
    if (!uwb_manager || !flight_control) {
        return false;
    }

    Position3D_ESP32 pos = uwb_manager->getCurrentPosition();

    if (!pos.isValid()) {
        return false;
    }

    // Конвертація до формату flight controller
    FlightPosition flight_pos;
    flight_pos.x_meters = pos.x;
    flight_pos.y_meters = pos.y;
    flight_pos.altitude_meters = pos.z;
    flight_pos.accuracy_meters = pos.accuracy_meters;
    flight_pos.timestamp_us = pos.timestamp_us;

    // Отримуємо швидкість
    Position3D_ESP32 vel = uwb_manager->getCurrentVelocity();
    flight_pos.velocity_x = vel.x;
    flight_pos.velocity_y = vel.y;
    flight_pos.velocity_z = vel.z;

    // Відправка до autopilot
    return flight_control->updatePositionEstimate(flight_pos);
}

bool getTargetPositionForNavigation(AerialUWBManager_ESP32* uwb_manager,
                                    uint16_t target_drone_id,
                                    Position3D_ESP32& target_position) {
    if (!uwb_manager) {
        return false;
    }

    // Отримуємо позицію цільового дрона
    if (!uwb_manager->getNodePosition(target_drone_id, target_position)) {
        return false;
    }

    // Перевірка валідності
    return target_position.isValid();
}

//=============================================================================
// ✅ ФОРМАЦІЙНИЙ ПОЛІТ - HELPER FUNCTIONS
//=============================================================================

struct FormationPosition {
    float offset_x;  // Відносно лідера
    float offset_y;
    float offset_z;
    float spacing;   // Мінімальна відстань між дронами
};

bool calculateFormationPosition(AerialUWBManager_ESP32* uwb_manager,
                                FormationPosition formation_config,
                                Position3D_ESP32& target_position) {
    if (!uwb_manager) {
        return false;
    }

    // Отримуємо ID лідера формації
    uint8_t leader_id = uwb_manager->getFormationLeader();
    if (leader_id == 0) {
        return false;  // Немає лідера
    }

    // Отримуємо позицію лідера
    Position3D_ESP32 leader_pos;
    if (!uwb_manager->getNodePosition(leader_id, leader_pos)) {
        return false;
    }

    // Розраховуємо цільову позицію з offset'ом
    target_position.x = leader_pos.x + formation_config.offset_x;
    target_position.y = leader_pos.y + formation_config.offset_y;
    target_position.z = leader_pos.z + formation_config.offset_z;

    // Перевірка collision avoidance
    Position3D_ESP32 my_pos = uwb_manager->getCurrentPosition();
    float distance = calculateDistance3D(my_pos, target_position);

    if (distance < formation_config.spacing) {
        // Занадто близько, коригуємо позицію
        return false;
    }

    return true;
}

bool maintainFormationSpacing(AerialUWBManager_ESP32* uwb_manager,
                              float min_spacing,
                              Position3D_ESP32& corrected_position) {
    if (!uwb_manager) {
        return false;
    }

    Position3D_ESP32 my_pos = uwb_manager->getCurrentPosition();
    uint8_t node_count = uwb_manager->getKnownNodesCount();

    bool collision_risk = false;

    // Перевірка відстані до всіх інших дронів
    for (uint8_t i = 0; i < node_count; i++) {
        uint16_t other_id = uwb_manager->getKnownNodeId(i);

        Position3D_ESP32 other_pos;
        if (!uwb_manager->getNodePosition(other_id, other_pos)) {
            continue;
        }

        float distance = calculateDistance3D(my_pos, other_pos);

        if (distance < min_spacing) {
            collision_risk = true;

            // Вектор від іншого дрона до нас
            float dx = my_pos.x - other_pos.x;
            float dy = my_pos.y - other_pos.y;
            float dz = my_pos.z - other_pos.z;

            // Нормалізація
            float mag = sqrtf(dx*dx + dy*dy + dz*dz);
            if (mag > 0.01f) {
                dx /= mag;
                dy /= mag;
                dz /= mag;

                // Коригуємо позицію для уникнення зіткнення
                float correction = min_spacing - distance;
                corrected_position.x = my_pos.x + dx * correction;
                corrected_position.y = my_pos.y + dy * correction;
                corrected_position.z = my_pos.z + dz * correction;
            }
        }
    }

    return collision_risk;
}

//=============================================================================
// ✅ COLLISION AVOIDANCE З UWB
//=============================================================================

struct CollisionThreat {
    uint16_t drone_id;
    float distance;
    float time_to_collision;  // секунди
    Position3D_ESP32 relative_velocity;
    bool critical;            // < 5 метрів
};

bool detectCollisionThreats(AerialUWBManager_ESP32* uwb_manager,
                            std::vector<CollisionThreat>& threats,
                            float warning_distance = 20.0f,
                            float critical_distance = 5.0f) {
    if (!uwb_manager) {
        return false;
    }

    threats.clear();

    Position3D_ESP32 my_pos = uwb_manager->getCurrentPosition();
    Position3D_ESP32 my_vel = uwb_manager->getCurrentVelocity();
    uint8_t node_count = uwb_manager->getKnownNodesCount();

    for (uint8_t i = 0; i < node_count; i++) {
        uint16_t other_id = uwb_manager->getKnownNodeId(i);

        Position3D_ESP32 other_pos, other_vel;
        if (!uwb_manager->getNodePosition(other_id, other_pos)) {
            continue;
        }
        if (!uwb_manager->getNodeVelocity(other_id, other_vel)) {
            continue;
        }

        // Поточна відстань
        float distance = calculateDistance3D(my_pos, other_pos);

        if (distance > warning_distance) {
            continue;  // Занадто далеко
        }

        // Відносна швидкість
        Position3D_ESP32 rel_vel;
        rel_vel.x = other_vel.x - my_vel.x;
        rel_vel.y = other_vel.y - my_vel.y;
        rel_vel.z = other_vel.z - my_vel.z;

        // Вектор між дронами
        float dx = other_pos.x - my_pos.x;
        float dy = other_pos.y - my_pos.y;
        float dz = other_pos.z - my_pos.z;

        // Проекція швидкості на вектор між дронами
        float closing_rate = (rel_vel.x * dx + rel_vel.y * dy + rel_vel.z * dz) / distance;

        if (closing_rate <= 0) {
            continue;  // Віддаляємося
        }

        // Час до зіткнення
        float time_to_collision = distance / closing_rate;

        if (time_to_collision > 10.0f) {
            continue;  // Більше 10 секунд - не критично
        }

        // Додаємо threat
        CollisionThreat threat;
        threat.drone_id = other_id;
        threat.distance = distance;
        threat.time_to_collision = time_to_collision;
        threat.relative_velocity = rel_vel;
        threat.critical = (distance < critical_distance);

        threats.push_back(threat);
    }

    // Сортуємо за часом до зіткнення
    std::sort(threats.begin(), threats.end(),
              [](const CollisionThreat& a, const CollisionThreat& b) {
                  return a.time_to_collision < b.time_to_collision;
              });

    return !threats.empty();
}

Position3D_ESP32 calculateAvoidanceVector(const std::vector<CollisionThreat>& threats,
                                          Position3D_ESP32 current_velocity) {
    Position3D_ESP32 avoidance_vector{0, 0, 0};

    for (const auto& threat : threats) {
        // Більша вага для критичних загроз
        float weight = threat.critical ? 2.0f : 1.0f;
        weight /= (threat.time_to_collision + 0.1f);  // Більша вага для близьких загроз

        // Вектор уникнення - перпендикулярний до відносної швидкості
        avoidance_vector.x -= threat.relative_velocity.x * weight;
        avoidance_vector.y -= threat.relative_velocity.y * weight;
        avoidance_vector.z -= threat.relative_velocity.z * weight;
    }

    // Нормалізація
    float mag = sqrtf(avoidance_vector.x * avoidance_vector.x +
                      avoidance_vector.y * avoidance_vector.y +
                      avoidance_vector.z * avoidance_vector.z);

    if (mag > 0.01f) {
        avoidance_vector.x /= mag;
        avoidance_vector.y /= mag;
        avoidance_vector.z /= mag;

        // Масштабуємо до розумної швидкості уникнення
        float avoidance_speed = 3.0f;  // 3 м/с
        avoidance_vector.x *= avoidance_speed;
        avoidance_vector.y *= avoidance_speed;
        avoidance_vector.z *= avoidance_speed;
    }

    return avoidance_vector;
}

//=============================================================================
// ✅ UTILITY FUNCTIONS
//=============================================================================

float calculateDistance3D(const Position3D_ESP32& p1, const Position3D_ESP32& p2) {
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    float dz = p2.z - p1.z;
    return sqrtf(dx*dx + dy*dy + dz*dz);
}

uint16_t calculateCRC16(const uint8_t* data, uint16_t length) {
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }

    return crc;
}

//=============================================================================
// ✅ ДІАГНОСТИКА ТА МОНІТОРИНГ
//=============================================================================

void printFormationStatus(AerialUWBManager_ESP32* uwb_manager) {
    if (!uwb_manager) {
        return;
    }

    Serial.println("\n╔══════════════════════════════════════════════════════════╗");
    Serial.println("║  СТАТУС ФОРМАЦІЇ                                         ║");
    Serial.println("╚══════════════════════════════════════════════════════════╝");

    uint8_t leader_id = uwb_manager->getFormationLeader();
    if (leader_id == 0) {
        Serial.println("⚠️ Формація не активна");
        return;
    }

    Serial.printf("Лідер формації: Drone %d\n", leader_id);

    Position3D_ESP32 formation_center = uwb_manager->getFormationCenter();
    Serial.printf("Центр формації: (%.2f, %.2f, %.2f)\n",
                  formation_center.x, formation_center.y, formation_center.z);

    Position3D_ESP32 formation_velocity = uwb_manager->getFormationVelocity();
    float formation_speed = sqrtf(formation_velocity.x * formation_velocity.x +
                                  formation_velocity.y * formation_velocity.y +
                                  formation_velocity.z * formation_velocity.z);
    Serial.printf("Швидкість формації: %.2f м/с (%.1f км/год)\n",
                  formation_speed, formation_speed * 3.6);

    // Список членів формації
    uint8_t node_count = uwb_manager->getKnownNodesCount();
    Serial.printf("\nЧленів формації: %d\n", node_count);

    for (uint8_t i = 0; i < node_count; i++) {
        uint16_t node_id = uwb_manager->getKnownNodeId(i);
        Position3D_ESP32 pos;

        if (uwb_manager->getNodePosition(node_id, pos)) {
            float distance_from_center = calculateDistance3D(pos, formation_center);
            Serial.printf("  Drone %d: відстань від центру %.2f м\n",
                          node_id, distance_from_center);
        }
    }

    Serial.println();
}

void printCollisionWarnings(const std::vector<CollisionThreat>& threats) {
    if (threats.empty()) {
        return;
    }

    Serial.println("\n⚠️ ПОПЕРЕДЖЕННЯ ПРО ЗІТКНЕННЯ:");

    for (const auto& threat : threats) {
        if (threat.critical) {
            Serial.print("🚨 КРИТИЧНО! ");
        } else {
            Serial.print("⚠️  ");
        }

        Serial.printf("Drone %d: %.2f m, %.1f секунд до зіткнення\n",
                      threat.drone_id, threat.distance, threat.time_to_collision);
    }

    Serial.println();
}

//=============================================================================
// 🇺🇦 Slava Ukraini! 🇺🇦
//=============================================================================