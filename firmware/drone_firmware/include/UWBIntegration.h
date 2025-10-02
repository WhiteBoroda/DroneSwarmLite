//=============================================================================
// firmware/drone_firmware/include/UWBIntegration.h
// Header file для інтеграції UWB з іншими системами
//=============================================================================

#ifndef UWB_INTEGRATION_H
#define UWB_INTEGRATION_H

#include <Arduino.h>
#include <vector>
#include "AerialUWBManager_ESP32.h"

// Forward declarations
class LoRaModule;
class FlightControl;
class DroneController;

//=============================================================================
// ✅ СТРУКТУРИ ДАНИХ
//=============================================================================

// LoRa пакет позиції
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

// Позиція для flight controller
struct FlightPosition {
    float x_meters;
    float y_meters;
    float altitude_meters;
    float velocity_x;
    float velocity_y;
    float velocity_z;
    float accuracy_meters;
    uint64_t timestamp_us;
};

// Конфігурація формації
struct FormationPosition {
    float offset_x;  // Відносно лідера
    float offset_y;
    float offset_z;
    float spacing;   // Мінімальна відстань між дронами
};

// Інформація про загрозу зіткнення
struct CollisionThreat {
    uint16_t drone_id;
    float distance;
    float time_to_collision;  // секунди
    AerialUWB_ESP32::Position3D_ESP32 relative_velocity;
    bool critical;            // < 5 метрів
};

//=============================================================================
// ✅ LORA ІНТЕГРАЦІЯ
//=============================================================================

/**
 * Відправити позицію через LoRa
 * @param uwb_manager Вказівник на UWB менеджер
 * @param lora Вказівник на LoRa модуль
 * @return true якщо успішно
 */
bool sendPositionViaLoRa (AerialUWB_ESP32::AerialUWBManager_ESP32* uwb_manager,LoRaModule* lora);

/**
 * Отримати позицію через LoRa
 * @param uwb_manager Вказівник на UWB менеджер
 * @param lora Вказівник на LoRa модуль
 * @param packet Пакет для заповнення
 * @return true якщо пакет отримано
 */
bool receivePositionViaLoRa(AerialUWB_ESP32::AerialUWBManager_ESP32* uwb_manager, LoRaModule* lora,
                            LoRaPositionPacket& packet);

//=============================================================================
// ✅ FLIGHT CONTROLLER ІНТЕГРАЦІЯ
//=============================================================================

/**
 * Відправити позицію до flight controller
 * @param uwb_manager Вказівник на UWB менеджер
 * @param flight_control Вказівник на flight control
 * @return true якщо успішно
 */
bool sendPositionToFlightController(AerialUWB_ESP32::AerialUWBManager_ESP32* uwb_manager,
                                    FlightControl* flight_control);

/**
 * Отримати позицію цільового дрона для навігації
 * @param uwb_manager Вказівник на UWB менеджер
 * @param target_drone_id ID цільового дрона
 * @param target_position Результуюча позиція
 * @return true якщо позиція отримана
 */
bool getTargetPositionForNavigation(AerialUWB_ESP32::AerialUWBManager_ESP32* uwb_manager,
                                    uint16_t target_drone_id,
                                    AerialUWB_ESP32::Position3D_ESP32& target_position);

//=============================================================================
// ✅ ФОРМАЦІЙНИЙ ПОЛІТ
//=============================================================================

/**
 * Розрахувати позицію в формації
 * @param uwb_manager Вказівник на UWB менеджер
 * @param formation_config Конфігурація формації
 * @param target_position Результуюча цільова позиція
 * @return true якщо успішно
 */
bool calculateFormationPosition(AerialUWB_ESP32::AerialUWBManager_ESP32* uwb_manager,
                                FormationPosition formation_config,
                                AerialUWB_ESP32::Position3D_ESP32& target_position);

/**
 * Підтримувати відстань в формації
 * @param uwb_manager Вказівник на UWB менеджер
 * @param min_spacing Мінімальна відстань
 * @param corrected_position Скоригована позиція
 * @return true якщо є ризик зіткнення
 */
bool maintainFormationSpacing(AerialUWB_ESP32::AerialUWBManager_ESP32* uwb_manager,
                              float min_spacing,
                              AerialUWB_ESP32::Position3D_ESP32& corrected_position);

//=============================================================================
// ✅ COLLISION AVOIDANCE
//=============================================================================

/**
 * Виявити загрози зіткнення
 * @param uwb_manager Вказівник на UWB менеджер
 * @param threats Вектор для заповнення загрозами
 * @param warning_distance Відстань попередження (м)
 * @param critical_distance Критична відстань (м)
 * @return true якщо є загрози
 */
bool detectCollisionThreats(AerialUWB_ESP32::AerialUWBManager_ESP32* uwb_manager,
                            std::vector<CollisionThreat>& threats,
                            float warning_distance = 20.0f,
                            float critical_distance = 5.0f);

/**
 * Розрахувати вектор уникнення
 * @param threats Вектор загроз
 * @param current_velocity Поточна швидкість
 * @return Вектор швидкості уникнення
 */
AerialUWB_ESP32::Position3D_ESP32 calculateAvoidanceVector(
        const std::vector<CollisionThreat>& threats,
        AerialUWB_ESP32::Position3D_ESP32 current_velocity);

//=============================================================================
// ✅ UTILITY FUNCTIONS
//=============================================================================

/**
 * Розрахувати 3D відстань між двома точками
 * @param p1 Перша точка
 * @param p2 Друга точка
 * @return Відстань в метрах
 */
float calculateDistance3D(const AerialUWB_ESP32::Position3D_ESP32& p1,
                          const AerialUWB_ESP32::Position3D_ESP32& p2);

/**
 * Розрахувати CRC16 checksum
 * @param data Дані
 * @param length Довжина даних
 * @return CRC16 checksum
 */
uint16_t calculateCRC16(const uint8_t* data, uint16_t length);

//=============================================================================
// ✅ ДІАГНОСТИКА
//=============================================================================

/**
 * Вивести статус формації
 * @param uwb_manager Вказівник на UWB менеджер
 */
void printFormationStatus(AerialUWB_ESP32::AerialUWBManager_ESP32* uwb_manager);

/**
 * Вивести попередження про зіткнення
 * @param threats Вектор загроз
 */
void printCollisionWarnings(const std::vector<CollisionThreat>& threats);

//=============================================================================
// ✅ МАКРОСИ ДЛЯ ЗРУЧНОСТІ
//=============================================================================

// Конвертація км/год в м/с
#define KMH_TO_MS(kmh) ((kmh) / 3.6f)

// Конвертація м/с в км/год
#define MS_TO_KMH(ms) ((ms) * 3.6f)

// Перевірка валідності позиції
#define IS_POSITION_VALID(pos) \
    ((pos).accuracy_meters > 0.0f && (pos).accuracy_meters < 100.0f)

// Відстань в межах допустимого
#define IS_DISTANCE_SAFE(distance, min_safe) ((distance) >= (min_safe))

#endif // UWB_INTEGRATION_H

//=============================================================================
// 🇺🇦 Slava Ukraini! Death to russian invaders! 🇺🇦
//=============================================================================