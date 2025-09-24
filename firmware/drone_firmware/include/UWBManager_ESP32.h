//
// firmware/drone_firmware/include/UWBManager_ESP32.h
// 🇺🇦 UWB Manager для ESP32 - Real-time позиционування дронів 🇺🇦
//

#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <esp_timer.h>
#include <atomic>
#include "HardwareConfig.h"

// UWB Hardware constants (DW1000/DW3000)
#define DW_DEVICE_ID_REG        0x00
#define DW_SYS_CFG_REG          0x04
#define DW_TX_FCTRL_REG         0x08
#define DW_TX_BUFFER_REG        0x09
#define DW_RX_FWTO_REG          0x0C
#define DW_SYS_CTRL_REG         0x0D
#define DW_SYS_STATUS_REG       0x0F
#define DW_RX_FINFO_REG         0x10
#define DW_RX_BUFFER_REG        0x11
#define DW_RX_FQUAL_REG         0x12
#define DW_TX_TIME_REG          0x17
#define DW_RX_TIME_REG          0x15

// UWB Commands
#define DW_TXSTRT               0x00000002UL
#define DW_TXDLYS               0x00000001UL
#define DW_TRXOFF               0x00000040UL
#define DW_RXENAB               0x00000100UL

// UWB Configuration
#define UWB_DATA_RATE           0x00    // 110 kbps
#define UWB_PREAMBLE_LENGTH     0x01    // 1024 symbols
#define UWB_CHANNEL             5       // Channel 5 (6.5 GHz)
#define UWB_PCODE               9       // Preamble code 9

// ESP32 UWB Configuration
namespace UWBConfig_ESP32 {
    constexpr uint32_t SPI_FREQUENCY = 8000000;    // 8 MHz SPI
    constexpr uint16_t MAX_FRAME_SIZE = 127;       // Max UWB frame size
    constexpr uint16_t MEASUREMENT_BUFFER_SIZE = 32; // Circular buffer
    constexpr uint32_t RANGING_TIMEOUT_US = 10000;  // 10ms timeout
    constexpr uint32_t POSITION_UPDATE_INTERVAL_MS = 50; // 20 Hz updates

    // Power management
    constexpr int8_t TX_POWER_MAX = 33;            // Max TX power (dBm)
    constexpr int8_t TX_POWER_DEFAULT = 20;        // Default TX power
    constexpr uint16_t ANTENNA_DELAY = 16456;      // Default antenna delay
}

// UWB Message types for ESP32
enum class UWBMessageType_ESP32 : uint8_t {
    RANGING_REQUEST = 0x01,
    RANGING_RESPONSE = 0x02,
    RANGING_FINAL = 0x03,
    POSITION_BEACON = 0x04,
    SYNC_MESSAGE = 0x05,
    EMERGENCY_BEACON = 0xFF
};

// UWB Frame structure for ESP32 (optimized for memory)
struct UWBFrame_ESP32 {
    uint8_t frame_control;
    uint8_t sequence_number;
    uint16_t sender_id;
    uint16_t receiver_id;
    UWBMessageType_ESP32 message_type;
    uint8_t payload_length;
    uint8_t payload[32];          // Reduced payload for ESP32 memory
    uint16_t checksum;

    // Calculate size dynamically
    uint8_t getFrameSize() const {
        return sizeof(UWBFrame_ESP32) - sizeof(payload) + payload_length;
    }
} __attribute__((packed));

// UWB Measurement structure (memory optimized)
struct UWBMeasurement_ESP32 {
    uint16_t target_id;
    float distance_meters;
    int16_t rssi_dbm;
    uint8_t quality;              // 0-100
    uint32_t timestamp_us;

    bool isValid() const {
        return (distance_meters > 0.1f && distance_meters < 1000.0f &&
                quality > 10 && rssi_dbm > -120);
    }
} __attribute__((packed));

// Position estimate (3D coordinates)
struct Position3D_ESP32 {
    float x, y, z;                // Meters
    float accuracy_meters;        // Position accuracy
    uint32_t timestamp_us;

    Position3D_ESP32() : x(0), y(0), z(0), accuracy_meters(999.0f), timestamp_us(0) {}
    Position3D_ESP32(float _x, float _y, float _z) : x(_x), y(_y), z(_z),
                                                     accuracy_meters(5.0f), timestamp_us(esp_timer_get_time()) {}

    float distanceTo(const Position3D_ESP32& other) const {
        float dx = x - other.x;
        float dy = y - other.y;
        float dz = z - other.z;
        return sqrt(dx*dx + dy*dy + dz*dz);
    }

    bool isValid() const {
        return (accuracy_meters < 10.0f);
    }
} __attribute__((packed));

// UWB Network node information
struct UWBNode_ESP32 {
    uint16_t node_id;
    Position3D_ESP32 position;
    uint32_t last_seen_us;
    uint8_t signal_quality;
    bool is_anchor;               // True if this is a fixed anchor

    bool isActive() const {
        return (esp_timer_get_time() - last_seen_us) < 5000000; // 5 seconds
    }
} __attribute__((packed));

// UWB Statistics (memory efficient)
struct UWBStats_ESP32 {
    std::atomic<uint32_t> measurements_made{0};
    std::atomic<uint32_t> successful_ranges{0};
    std::atomic<uint32_t> failed_ranges{0};
    std::atomic<uint32_t> position_updates{0};
    std::atomic<uint16_t> active_nodes{0};
    float average_accuracy_meters{999.0f};
    int16_t average_rssi_dbm{-120};
};

// Forward declarations
class LoRaModule;
class DroneController;

//=============================================================================
// ✅ MAIN UWB MANAGER CLASS FOR ESP32
//=============================================================================
class UWBManager_ESP32 : public HardwareComponent {
private:
    // Hardware interfaces
    SPIClass* uwb_spi_;
    uint16_t my_drone_id_;
    bool initialized_;
    bool running_;

    // FreeRTOS handles
    TaskHandle_t ranging_task_handle_;
    TaskHandle_t position_task_handle_;
    QueueHandle_t measurement_queue_;
    SemaphoreHandle_t spi_mutex_;
    esp_timer_handle_t ranging_timer_;

    // UWB Hardware state
    volatile bool tx_complete_;
    volatile bool rx_complete_;
    volatile bool rx_timeout_;
    volatile bool rx_error_;

    // Configuration
    int8_t tx_power_dbm_;
    uint16_t antenna_delay_;
    uint8_t current_sequence_;

    // Network state (memory optimized)
    UWBNode_ESP32 known_nodes_[8];     // Max 8 nodes for ESP32 memory
    uint8_t known_nodes_count_;
    Position3D_ESP32 current_position_;
    Position3D_ESP32 anchor_positions_[4]; // Max 4 anchors
    uint8_t anchor_count_;

    // Measurement buffers (circular)
    UWBMeasurement_ESP32 measurement_buffer_[UWBConfig_ESP32::MEASUREMENT_BUFFER_SIZE];
    volatile uint8_t measurement_head_;
    volatile uint8_t measurement_tail_;

    // Statistics
    UWBStats_ESP32 stats_;

    // Integration with other systems
    LoRaModule* lora_module_;
    DroneController* drone_controller_;

public:
    explicit UWBManager_ESP32(uint16_t drone_id);
    virtual ~UWBManager_ESP32();

    // HardwareComponent interface
    bool initialize() override;
    bool start() override;
    void stop() override;
    bool isHealthy() const override;
    const char* getName() const override { return "UWB_ESP32"; }

    //=========================================================================
    // ✅ CORE UWB FUNCTIONALITY
    //=========================================================================

    // Basic UWB operations
    bool performTWRRanging(uint16_t target_id, float& distance_meters);
    bool performDS_TWRRanging(uint16_t target_id, float& distance_meters,
                              float& quality_estimate);
    bool broadcastPositionBeacon();
    bool scanForNodes(uint32_t scan_duration_ms = 1000);

    // Position estimation
    bool updatePosition();
    Position3D_ESP32 getCurrentPosition() const { return current_position_; }
    float getPositionAccuracy() const { return current_position_.accuracy_meters; }
    bool setAnchorPosition(uint16_t anchor_id, const Position3D_ESP32& position);

    // Network management
    uint8_t getActiveNodeCount() const;
    bool getNodePosition(uint16_t node_id, Position3D_ESP32& position) const;
    bool isNodeActive(uint16_t node_id) const;
    void cleanInactiveNodes();

    // Configuration
    bool setTxPower(int8_t power_dbm);
    int8_t getTxPower() const { return tx_power_dbm_; }
    bool setAntennaDelay(uint16_t delay);
    uint16_t getAntennaDelay() const { return antenna_delay_; }

    // Statistics and monitoring
    UWBStats_ESP32 getStatistics() const { return stats_; }
    void resetStatistics();
    float getRangingSuccessRate() const;

    //=========================================================================
    // ✅ INTEGRATION WITH DRONE SYSTEMS
    //=========================================================================

    // LoRa integration for position sharing
    void setLoRaModule(LoRaModule* lora) { lora_module_ = lora; }
    bool sendPositionViaLoRa();
    bool receivePositionViaLoRa();

    // Flight controller integration
    void setDroneController(DroneController* controller) { drone_controller_ = controller; }
    bool notifyPositionUpdate();

    // Emergency functions
    bool sendEmergencyBeacon();
    bool isEmergencyBeaconDetected() const;

    //=========================================================================
    // ✅ INTERRUPT HANDLERS (must be static for ESP32)
    //=========================================================================
    static void IRAM_ATTR uwb_interrupt_handler();
    void handleUWBInterrupt();

private:
    //=========================================================================
    // ✅ FREERTOS TASKS
    //=========================================================================
    static void ranging_task(void* parameter);
    static void position_estimation_task(void* parameter);
    static void ranging_timer_callback(void* arg);

    //=========================================================================
    // ✅ LOW-LEVEL UWB HARDWARE CONTROL
    //=========================================================================

    // SPI Communication
    bool spiWriteRegister(uint8_t reg, const uint8_t* data, uint16_t length);
    bool spiReadRegister(uint8_t reg, uint8_t* data, uint16_t length);
    bool spiWrite32(uint8_t reg, uint32_t value);
    uint32_t spiRead32(uint8_t reg);

    // Hardware initialization
    bool resetUWBChip();
    bool configureUWBChip();
    bool calibrateUWBChip();
    bool setUWBChannel(uint8_t channel);
    bool setDataRate(uint8_t rate);

    // Frame transmission/reception
    bool transmitFrame(const UWBFrame_ESP32& frame);
    bool receiveFrame(UWBFrame_ESP32& frame, uint32_t timeout_us);
    bool waitForTxComplete(uint32_t timeout_us);
    bool waitForRxComplete(uint32_t timeout_us);

    // Timestamp functions
    uint64_t getTxTimestamp();
    uint64_t getRxTimestamp();
    uint64_t getCurrentUWBTime();

    //=========================================================================
    // ✅ POSITIONING ALGORITHMS (optimized for ESP32)
    //=========================================================================

    // Trilateration
    bool trilateratePosition(const UWBMeasurement_ESP32* measurements,
                             uint8_t count, Position3D_ESP32& position);
    bool multilateration(const UWBMeasurement_ESP32* measurements,
                         uint8_t count, Position3D_ESP32& position);

    // Kalman filter (simplified for ESP32)
    bool updatePositionFilter(const Position3D_ESP32& measured_position);

    //=========================================================================
    // ✅ UTILITY FUNCTIONS
    //=========================================================================

    // Buffer management
    bool addMeasurement(const UWBMeasurement_ESP32& measurement);
    bool getMeasurements(UWBMeasurement_ESP32* buffer, uint8_t max_count,
                         uint8_t& actual_count);

    // Node management
    UWBNode_ESP32* findNode(uint16_t node_id);
    bool addOrUpdateNode(uint16_t node_id, const Position3D_ESP32& position);

    // Checksum and validation
    uint16_t calculateChecksum(const uint8_t* data, uint16_t length);
    bool validateFrame(const UWBFrame_ESP32& frame);

    // Debug and diagnostics
    void printUWBStatus();
    void printNodeList();
    bool runUWBDiagnostics();

    //=========================================================================
    // ✅ MEMORY MANAGEMENT FOR ESP32
    //=========================================================================
    void* allocateAligned(size_t size, size_t alignment = 4);
    void freeAligned(void* ptr);
    bool checkHeapHealth() const;

    // Constants for ESP32 optimization
    static constexpr uint16_t SPI_BUFFER_SIZE = 256;
    static constexpr uint8_t MAX_RETRIES = 3;
    static constexpr uint32_t TASK_STACK_SIZE = 4096;
    static constexpr UBaseType_t TASK_PRIORITY = 2;
    static constexpr uint32_t QUEUE_SIZE = 16;
};

//=============================================================================
// ✅ GLOBAL FUNCTIONS FOR INTEGRATION
//=============================================================================

// Initialization helper for main.cpp integration
bool initializeUWBSystem(uint16_t drone_id);
UWBManager_ESP32* getUWBManager();

// Position sharing via LoRa (integration with existing comm system)
bool sharePositionWithSwarm(const Position3D_ESP32& position);
bool receiveSwarmPositions();

// Emergency UWB functions
bool broadcastUWBEmergency();
bool checkForUWBEmergencies();

// Diagnostic functions
void printUWBDiagnostics();
bool testUWBCommunication();

//=============================================================================
// ✅ CONFIGURATION MACROS
//=============================================================================

// Debug output control
#ifdef DEBUG_UWB
#define UWB_DEBUG(x) Serial.print("[UWB] "); Serial.println(x)
    #define UWB_DEBUG_F(format, ...) Serial.printf("[UWB] " format "\n", ##__VA_ARGS__)
#else
#define UWB_DEBUG(x)
#define UWB_DEBUG_F(format, ...)
#endif

// Memory usage tracking
#ifdef TRACK_UWB_MEMORY
#define UWB_CHECK_HEAP() \
        do { \
            if (ESP.getFreeHeap() < 15000) { \
                UWB_DEBUG_F("LOW HEAP: %d bytes", ESP.getFreeHeap()); \
            } \
        } while(0)
#else
#define UWB_CHECK_HEAP()
#endif

// Task watchdog integration
#define UWB_FEED_WATCHDOG() \
    do { \
        if (esp_task_wdt_status(NULL) == ESP_OK) { \
            esp_task_wdt_reset(); \
        } \
    } while(0)