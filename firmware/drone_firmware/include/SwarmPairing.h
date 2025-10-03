//=============================================================================
// firmware/drone_firmware/include/SwarmPairing.h
// 🤝 Swarm Auto-Discovery and Pairing System
// Автоматичне виявлення та ініціалізація рою дронів
//=============================================================================

#ifndef SWARM_PAIRING_H
#define SWARM_PAIRING_H

#include <Arduino.h>
#include <vector>
#include <algorithm>
#include "HardwareConfig.h"
#include "PairingCrypto.h"

//=============================================================================
// ✅ КОНСТАНТИ PAIRING ПРОТОКОЛУ
//=============================================================================

namespace PairingConfig {
    // Частоти для discovery (окрема від робочої)
    constexpr uint32_t DISCOVERY_FREQUENCY = 868500000;  // 868.5 MHz

    // Таймінги discovery
    constexpr uint32_t DISCOVERY_DURATION_MS = 15000;    // 15 секунд пошук
    constexpr uint32_t BEACON_INTERVAL_MS = 500;         // Beacon кожні 500ms
    constexpr uint32_t ID_ASSIGNMENT_TIMEOUT_MS = 30000; // 30 сек на отримання ID
    constexpr uint32_t CONFIRMATION_TIMEOUT_MS = 3000;   // 3 сек на підтвердження

    // Retry параметри
    constexpr uint8_t MAX_RETRIES = 3;
    constexpr uint32_t RETRY_DELAY_MS = 100;

    // Діапазон ID для рою
    constexpr uint16_t SWARM_ID_START = 101;
    constexpr uint16_t SWARM_ID_END = 199;

    // LED для індикації
    constexpr int LED_DISCOVERY_PIN = LED_COMMUNICATION;
}

//=============================================================================
// ✅ ТИПИ ПОВІДОМЛЕНЬ DISCOVERY ПРОТОКОЛУ
//=============================================================================

enum class DiscoveryMessageType : uint8_t {
    BEACON = 0xD1,              // Discovery beacon
    ID_ASSIGNMENT = 0xD2,       // Призначення ID
    PAIRING_COMPLETE = 0xD3,    // Pairing завершений
    ID_CONFIRMATION = 0xD4      // Підтвердження отримання ID
};

//=============================================================================
// ✅ СТРУКТУРИ ПОВІДОМЛЕНЬ
//=============================================================================

// Discovery Beacon (broadcast всіма дронами)
struct DiscoveryBeacon {
    uint8_t message_type;       // 0xD1
    uint64_t mac_address;       // Унікальний MAC ESP32
    uint8_t battery_percent;    // Рівень батареї (0-100)
    uint16_t uptime_seconds;    // Час роботи в секундах
    uint32_t checksum;          // XOR checksum
} __attribute__((packed));

// ID Assignment (від координатора до дрона)
struct IDAssignmentMessage {
    uint8_t message_type;       // 0xD2
    uint64_t coordinator_mac;   // MAC координатора
    uint64_t target_mac;        // MAC дрона-отримувача
    uint16_t assigned_id;       // Призначений Drone ID
    uint32_t timestamp;         // Timestamp призначення
    uint32_t checksum;
} __attribute__((packed));

// Pairing Complete (broadcast від координатора)
struct PairingCompleteMessage {
    uint8_t message_type;       // 0xD3
    uint64_t coordinator_mac;   // MAC координатора
    uint8_t total_drones;       // Загальна кількість дронів у рої
    uint32_t timestamp;
    uint32_t checksum;
} __attribute__((packed));

// ID Confirmation (від дрона до координатора)
struct IDConfirmationMessage {
    uint8_t message_type;       // 0xD4
    uint64_t my_mac;            // MAC дрона
    uint16_t my_id;             // Отриманий ID
    uint64_t coordinator_mac;   // MAC координатора
    uint32_t checksum;
} __attribute__((packed));

//=============================================================================
// ✅ СТРУКТУРА ВИЯВЛЕНОГО ДРОНА
//=============================================================================

struct DiscoveredDrone {
    uint64_t mac_address;       // MAC адреса
    int8_t rssi;                // Сила сигналу
    uint8_t battery_percent;    // Рівень батареї
    unsigned long last_seen;    // Час останнього контакту
    bool is_paired;             // Чи отримав ID
    uint16_t assigned_id;       // Призначений ID (якщо є)

    DiscoveredDrone() : mac_address(0), rssi(-120), battery_percent(0),
                        last_seen(0), is_paired(false), assigned_id(0) {}
};

//=============================================================================
// ✅ ТИПИ ФОРМАЦІЙ
//=============================================================================

enum class FormationType {
    LINE,           // Лінія
    V_SHAPE,        // V-подібна
    CIRCLE,         // Коло
    WEDGE,          // Клин
    SQUARE,         // Квадрат
    DIAMOND         // Ромб
};

//=============================================================================
// ✅ КЛАС SWARM PAIRING MANAGER
//=============================================================================

class SwarmPairingManager {
private:
    // Стан системи
    uint64_t my_mac_address_;
    uint16_t my_drone_id_;
    bool is_coordinator_;
    bool pairing_complete_;
    bool pairing_button_pressed_;
    unsigned long pairing_button_press_time_;

    static constexpr uint32_t BUTTON_DEBOUNCE_MS = 50;
    static constexpr uint32_t LONG_PRESS_MS = 2000;  // 2 секунди для long press

    // Список виявлених дронів
    std::vector<DiscoveredDrone> discovered_drones_;

    // Coordinator tracking
    uint64_t coordinator_mac_;

    // Статистика
    uint32_t beacons_sent_;
    uint32_t beacons_received_;
    uint32_t discovery_start_time_;
    // ✅ ДОДАТИ crypto manager
    PairingCryptoManager* crypto_manager_;

    // Координатор зберігає public keys всіх дронів
    std::vector<DronePublicKey> drone_public_keys_;

public:
    SwarmPairingManager();
    ~SwarmPairingManager();

    //=========================================================================
    // ✅ ОСНОВНІ МЕТОДИ AUTO-DISCOVERY
    //=========================================================================

    // Головна функція входу в auto-discovery режим
    bool EnterAutoDiscoveryMode();

    // Фази discovery протоколу
    bool RunDiscoveryPhase();           // ФАЗА 1: Пошук сусідів
    bool EnableCrypto();

    // ✅ ДОДАТИ key exchange фазу
    bool RunKeyExchangePhase();
    bool ElectCoordinator();            // ФАЗА 2: Вибір координатора
    bool AssignDroneIDs();              // ФАЗА 3: Роздача ID (координатор)
    bool WaitForIDAssignment();         // ФАЗА 3: Очікування ID (follower)

    //=========================================================================
    // ✅ BROADCAST ТА ОБРОБКА ПОВІДОМЛЕНЬ
    //=========================================================================

    void BroadcastDiscoveryBeacon();
    void SendIDAssignment(uint64_t target_mac, uint16_t assigned_id);
    void SendIDConfirmation(uint64_t coordinator_mac);
    bool WaitForPairingButton(uint32_t timeout_ms = 0);
    void BroadcastPairingComplete();

    // Обробники вхідних повідомлень
    void HandleIncomingMessage(uint8_t* data, int length, int rssi);
    void HandleDiscoveryBeacon(const DiscoveryBeacon* beacon, int rssi);
    void HandleIDAssignment(const IDAssignmentMessage* msg);
    void HandlePairingComplete(const PairingCompleteMessage* msg);
    void HandleIDConfirmation(const IDConfirmationMessage* msg);

    //=========================================================================
    // ✅ COORDINATOR ELECTION
    //=========================================================================

    double CalculateCoordinatorScore(uint64_t mac, uint8_t battery, int8_t rssi);
    uint64_t GetBestCoordinatorMac();
    bool AmICoordinator() const { return is_coordinator_; }

    //=========================================================================
    // ✅ UTILITIES
    //=========================================================================

    uint64_t GetMyMacAddress() const { return my_mac_address_; }
    uint16_t GetMyDroneID() const { return my_drone_id_; }
    void SetMyDroneID(uint16_t id) { my_drone_id_ = id; }

    size_t GetDiscoveredDronesCount() const { return discovered_drones_.size(); }
    const std::vector<DiscoveredDrone>& GetDiscoveredDrones() const {
        return discovered_drones_;
    }

    bool IsPairingComplete() const { return pairing_complete_; }
    bool IsPairingButtonPressed();

    // Діагностика
    void PrintDiscoveryStatus();
    void PrintPairingResults();

    //=========================================================================
    // ✅ FORMATION HELPERS
    //=========================================================================

    Position3D CalculateFormationPosition(
            uint16_t drone_id,
            FormationType type,
            int total_drones,
            float spacing = 5.0f
    );

    const char* GetFormationName(FormationType type);
};

//=============================================================================
// ✅ ГЛОБАЛЬНІ HELPER ФУНКЦІЇ
//=============================================================================

// Отримання MAC адреси ESP32
uint64_t GetESP32MacAddress();

// Ініціалізація LoRa для discovery
bool InitializeLoRaForDiscovery();

// Перевірка checksums
uint32_t CalculateChecksum(uint64_t val1, uint64_t val2 = 0, uint32_t val3 = 0);
bool VerifyChecksum(const void* message, size_t size);

// LED індикація для різних фаз
void SetDiscoveryLED(bool fast_blink);  // Швидке/повільне мигання

// EEPROM операції для збереження pairing info
bool SavePairingInfo(uint16_t drone_id, uint64_t coordinator_mac);
bool LoadPairingInfo(uint16_t& drone_id, uint64_t& coordinator_mac);
void ClearPairingInfo();

//=============================================================================
// ✅ CALLBACK ДЛЯ LORA INTERRUPT
//=============================================================================

// Callback який викликається при отриманні LoRa пакету в discovery mode
void OnDiscoveryMessageReceived(int packet_size);

// Глобальний екземпляр для interrupt handler
extern SwarmPairingManager* g_pairing_manager;

#endif // SWARM_PAIRING_H