// firmware/drone_firmware/include/HardwareConfig.h
// Конфигурация железа ESP32 для дрона


#pragma once

#include <Arduino.h>
#include <stdint.h>

// Hardware version identification
#define HARDWARE_VERSION_MAJOR 2
#define HARDWARE_VERSION_MINOR 1
#define FIRMWARE_VERSION_MAJOR 1
#define FIRMWARE_VERSION_MINOR 0

// ESP32 GPIO pin definitions
namespace HardwarePins {
    // LoRa SX1276/SX1281 pins
    constexpr int LORA_SCK = 18;
    constexpr int LORA_MISO = 19;
    constexpr int LORA_MOSI = 23;
    constexpr int LORA_SS = 5;
    constexpr int LORA_RST = 14;
    constexpr int LORA_DIO0 = 2;
    constexpr int LORA_DIO1 = 15;

    // ELRS 2.4GHz module (SX1281 + ESP8285)
    constexpr int ELRS_24_TX = 17;
    constexpr int ELRS_24_RX = 16;
    constexpr int ELRS_24_RST = 4;

    // ELRS 915MHz module (SX1276 + ESP8285)
    constexpr int ELRS_915_TX = 25;
    constexpr int ELRS_915_RX = 26;
    constexpr int ELRS_915_RST = 27;

    // UWB DW1000/DW3000 pins
    constexpr int UWB_SCK = 32;
    constexpr int UWB_MISO = 33;
    constexpr int UWB_MOSI = 21;
    constexpr int UWB_SS = 22;
    constexpr int UWB_RST = 12;
    constexpr int UWB_IRQ = 13;

    // Flight Controller UART (Mamba F722)
    constexpr int FC_TX = 1;   // ESP32 TX -> FC RX
    constexpr int FC_RX = 3;   // ESP32 RX <- FC TX
    constexpr int FC_RST = 0;  // Flight controller reset

    // Video transmitter control (VTX 5.8G)
    constexpr int VTX_TX = 9;
    constexpr int VTX_RX = 10;
    constexpr int VTX_POWER_PIN = 35;  // ADC pin for power monitoring

    // Status LEDs
    constexpr int LED_STATUS = 2;      // Built-in LED
    constexpr int LED_COMM = 34;       // Communication status LED
    constexpr int LED_ERROR = 36;      // Error status LED

    // System control
    constexpr int SYSTEM_ENABLE = 39;  // Master system enable
    constexpr int EMERGENCY_STOP = 38; // Emergency stop input

    // Analog inputs
    constexpr int BATTERY_VOLTAGE = 35;
    constexpr int CURRENT_SENSOR = 37;
    constexpr int RSSI_ANALOG = 34;

    constexpr int PAIRING_BUTTON = 26;      // Кнопка START PAIRING
    constexpr int PAIRING_LED = LED_COMM;   // LED індикація pairing
}

// Hardware timing constants (microseconds)
namespace Timing {
    constexpr uint32_t MAIN_LOOP_INTERVAL = 20000;      // 50Hz main loop
    constexpr uint32_t TELEMETRY_INTERVAL = 200000;     // 5Hz telemetry
    constexpr uint32_t HEARTBEAT_INTERVAL = 1000000;    // 1Hz heartbeat
    constexpr uint32_t POSITION_UPDATE_INTERVAL = 100000; // 10Hz position
    constexpr uint32_t FORMATION_CHECK_INTERVAL = 50000;  // 20Hz formation
    constexpr uint32_t SENSOR_READ_INTERVAL = 10000;      // 100Hz sensors
    constexpr uint32_t COMMUNICATION_TIMEOUT = 30000000;  // 30s comm timeout
    constexpr uint32_t EMERGENCY_TIMEOUT = 300000000;     // 5min emergency timeout
}

// Communication configuration
namespace CommConfig {
    // LoRa primary frequencies (Hz)
    constexpr uint32_t LORA_FREQ_1 = 868100000;  // 868.1 MHz
    constexpr uint32_t LORA_FREQ_2 = 868300000;  // 868.3 MHz
    constexpr uint32_t LORA_FREQ_3 = 868500000;  // 868.5 MHz
    constexpr uint32_t LORA_FREQ_4 = 915000000;  // 915 MHz

    // LoRa parameters
    constexpr uint8_t LORA_SPREADING_FACTOR = 7;
    constexpr uint8_t LORA_BANDWIDTH = 125;      // kHz
    constexpr uint8_t LORA_CODING_RATE = 5;
    constexpr int8_t LORA_TX_POWER = 14;         // dBm
    constexpr uint16_t LORA_PREAMBLE_LENGTH = 8;
    constexpr bool LORA_CRC_ENABLED = true;

    // ELRS baud rates
    constexpr uint32_t ELRS_24_BAUDRATE = 420000;
    constexpr uint32_t ELRS_915_BAUDRATE = 115200;

    // Flight controller UART
    constexpr uint32_t FC_BAUDRATE = 115200;
    constexpr uint32_t VTX_BAUDRATE = 9600;
}

// UWB configuration
namespace UWBConfig {
    constexpr uint8_t UWB_CHANNEL = 5;           // UWB channel (1-7)
    constexpr uint8_t UWB_PREAMBLE_CODE = 9;     // Preamble code (1-24)
    constexpr uint8_t UWB_DATA_RATE = 2;         // 0=110k, 1=850k, 2=6.8M
    constexpr uint16_t UWB_PREAMBLE_LENGTH = 128;
    constexpr uint8_t UWB_PAC_SIZE = 8;
    constexpr uint8_t UWB_TX_POWER = 15;         // Power setting (0-33)
    constexpr bool UWB_SMART_POWER = true;
}

// System limits and constants
namespace SystemLimits {
    constexpr uint16_t MAX_PACKET_SIZE = 250;    // bytes
    constexpr uint8_t MAX_RETRIES = 3;
    constexpr uint16_t MAX_DRONES_IN_SWARM = 999;
    constexpr double MIN_BATTERY_VOLTAGE = 14.8; // V (4S LiPo)
    constexpr double MAX_BATTERY_VOLTAGE = 16.8; // V (4S LiPo)
    constexpr uint8_t CRITICAL_BATTERY_PERCENT = 20;
    constexpr double MAX_CPU_TEMPERATURE = 85.0; // °C
}

// Drone ID storage in EEPROM
namespace EEPROMLayout {
    constexpr int DRONE_ID_ADDRESS = 0;
    constexpr int MAGIC_NUMBER_ADDRESS = 2;
    constexpr uint16_t EEPROM_MAGIC = 0xUA24;    // Ukraine 2024
    constexpr int CONFIG_START_ADDRESS = 8;
    constexpr int CALIBRATION_DATA_ADDRESS = 64;
}

// Hardware component classes forward declarations
class DroneController;
class CommunicationNode;
class SensorManager;
class FlightControl;
class VideoTransmitter;
class PositioningSensor;
class TargetingSystemInterface;

// System state structure
struct DroneState {
    enum State {
        OFFLINE = 0,
        INITIALIZING,
        READY,
        AUTONOMOUS_FLIGHT,
        IN_FORMATION,
        LOST_COMMUNICATION,
        EMERGENCY,
        TARGETING_MODE,
        SELF_DESTRUCT
    };

    State current_state;
    uint16_t drone_id;
    bool emergency_mode;
    bool targeting_mode;
    uint32_t state_change_time;

    DroneState() : current_state(OFFLINE), drone_id(0),
                   emergency_mode(false), targeting_mode(false),
                   state_change_time(0) {}
};

// Hardware initialization functions
bool loadDroneID();
bool saveDroneID(uint16_t id);
bool initializeHardware();
bool initializeCommunication();
bool initializeSensors();
bool checkHardwareHealth();

// System control functions
void setSystemState(DroneState::State new_state);
DroneState::State getSystemState();
void handleEmergencyStop();
void handleSelfDestruct();

// Power management
double readBatteryVoltage();
uint8_t calculateBatteryPercentage(double voltage);
double readCurrentConsumption();
bool isBatteryLow();
bool isBatteryCritical();

// Temperature monitoring
double readCPUTemperature();
bool isCPUOverheating();

// LED control
void setStatusLED(bool on);
void setCommLED(bool on);
void setErrorLED(bool on);
void blinkLED(int pin, int times, int delay_ms = 200);

// Diagnostic functions
bool runSystemDiagnostics();
void printSystemInfo();
void printHardwareStatus();

// Utility macros
#define SLAVA_UKRAINI() Serial.println("🇺🇦 SLAVA UKRAINI! 🇺🇦")
#define DEBUG_PRINT(x) Serial.println(x)
#define DEBUG_PRINTF(format, ...) Serial.printf(format, ##__VA_ARGS__)
#define ERROR_PRINT(x) Serial.print("ERROR: "); Serial.println(x)
#define WARNING_PRINT(x) Serial.print("WARNING: "); Serial.println(x)

// Memory management
#define CHECK_HEAP() \
    do { \
        if (ESP.getFreeHeap() < 10000) { \
            ERROR_PRINT("Low heap memory: " + String(ESP.getFreeHeap())); \
        } \
    } while(0)

// Watchdog timer
#define FEED_WATCHDOG() esp_task_wdt_reset()

// Hardware abstraction layer classes

// Base hardware component class
class HardwareComponent {
public:
    virtual bool initialize() = 0;
    virtual bool start() = 0;
    virtual void stop() = 0;
    virtual bool isHealthy() const = 0;
    virtual const char* getName() const = 0;
    virtual ~HardwareComponent() = default;
};

// LoRa communication module
class LoRaModule : public HardwareComponent {
private:
    bool initialized_;
    uint32_t current_frequency_;
    int8_t current_power_;

public:
    LoRaModule() : initialized_(false), current_frequency_(CommConfig::LORA_FREQ_1),
                   current_power_(CommConfig::LORA_TX_POWER) {}

    bool initialize() override;
    bool start() override;
    void stop() override;
    bool isHealthy() const override;
    const char* getName() const override { return "LoRa"; }

    // LoRa specific methods
    bool setFrequency(uint32_t frequency);
    bool setPower(int8_t power);
    bool sendPacket(const uint8_t* data, size_t length);
    bool receivePacket(uint8_t* buffer, size_t& length, int timeout_ms = 1000);
    int getRSSI() const;
    float getSNR() const;
    bool performFrequencyHop();
};

// UWB positioning module
class UWBModule : public HardwareComponent {
private:
    bool initialized_;
    uint16_t device_id_;

public:
    UWBModule() : initialized_(false), device_id_(0) {}

    bool initialize() override;
    bool start() override;
    void stop() override;
    bool isHealthy() const override;
    const char* getName() const override { return "UWB"; }

    // UWB specific methods
    bool performRanging(uint16_t target_id, double& distance, double& accuracy);
    bool setDeviceID(uint16_t id);
    uint16_t getDeviceID() const { return device_id_; }
    bool calibrateAntenna();
    bool startListening();
    bool stopListening();
};

// Flight controller interface
class FlightControllerInterface : public HardwareComponent {
private:
    bool initialized_;
    bool armed_;

public:
    FlightControllerInterface() : initialized_(false), armed_(false) {}

    bool initialize() override;
    bool start() override;
    void stop() override;
    bool isHealthy() const override;
    const char* getName() const override { return "FlightController"; }

    // Flight control methods
    bool arm();
    bool disarm();
    bool isArmed() const { return armed_; }
    bool sendCommand(const char* command);
    bool readTelemetry(char* buffer, size_t buffer_size);
    bool setMode(const char* mode);
    bool emergencyStop();
};

// Video transmitter control
class VideoTransmitterControl : public HardwareComponent {
private:
    bool initialized_;
    uint16_t current_frequency_;
    uint16_t current_power_;
    bool transmitting_;

public:
    VideoTransmitterControl() : initialized_(false), current_frequency_(5740),
                                current_power_(200), transmitting_(false) {}

    bool initialize() override;
    bool start() override;
    void stop() override;
    bool isHealthy() const override;
    const char* getName() const override { return "VideoTX"; }

    // VTX specific methods
    bool setChannel(uint16_t frequency_mhz);
    bool setPower(uint16_t power_mw);
    bool enableTransmission();
    bool disableTransmission();
    bool isTransmitting() const { return transmitting_; }
    uint16_t getCurrentFrequency() const { return current_frequency_; }
    uint16_t getCurrentPower() const { return current_power_; }
};

// System integration functions
namespace SystemIntegration {
    // Hardware manager - controls all components
    class HardwareManager {
    private:
        std::vector<std::unique_ptr<HardwareComponent>> components_;
        bool system_healthy_;

    public:
        HardwareManager();
        ~HardwareManager();

        bool initializeAll();
        bool startAll();
        void stopAll();
        bool isSystemHealthy() const { return system_healthy_; }

        void addComponent(std::unique_ptr<HardwareComponent> component);
        HardwareComponent* getComponent(const char* name);
        std::vector<const char*> getFailedComponents() const;

        void runHealthCheck();
        void printStatus() const;
    };

    // Global hardware manager instance
    extern HardwareManager hardware_manager;

    // Convenience functions
    bool initializeSystemHardware();
    void shutdownSystemHardware();
    bool isSystemReady();
    void updateSystemStatus();
}

// Task priorities for FreeRTOS
namespace TaskPriorities {
    constexpr int CRITICAL_TASK = 5;    // Emergency handling
    constexpr int HIGH_TASK = 4;        // Communication, flight control
    constexpr int NORMAL_TASK = 3;      // Main logic, positioning
    constexpr int LOW_TASK = 2;         // Telemetry, diagnostics
    constexpr int BACKGROUND_TASK = 1;  // Maintenance, cleanup
}

// Stack sizes for tasks (bytes)
namespace StackSizes {
    constexpr int MAIN_TASK = 8192;
    constexpr int COMM_TASK = 4096;
    constexpr int SENSOR_TASK = 2048;
    constexpr int TELEMETRY_TASK = 2048;
    constexpr int MONITORING_TASK = 1024;
}

// Interrupt service routines
extern "C" {
void IRAM_ATTR lora_dio0_isr();
void IRAM_ATTR uwb_irq_isr();
void IRAM_ATTR emergency_stop_isr();
}

// Global hardware instances (defined in main.cpp)
extern LoRaModule lora_module;
extern UWBModule uwb_module;
extern FlightControllerInterface flight_controller;
extern VideoTransmitterControl video_transmitter;

// Configuration validation
namespace ConfigValidation {
    bool validatePinAssignments();
    bool validateTimingConfiguration();
    bool validateCommunicationSettings();
    bool validateSystemLimits();
    void printConfigurationSummary();
}

// Hardware testing functions
namespace HardwareTesting {
    bool testLoRaCommunication();
    bool testUWBRanging();
    bool testFlightControllerLink();
    bool testVideoTransmitter();
    bool testAllSensors();
    bool runFullSystemTest();
    void printTestResults();
}

// Emergency procedures
namespace EmergencyProcedures {
    void executeEmergencyLanding();
    void activateSelfDestruct();
    void enableEmergencyBeacon();
    void cutAllPower();
    void sendEmergencyBroadcast();
}

#endif // HARDWARE_CONFIG_H