// include/LoRaSystem.h
// LoRa система для ELRS SX1281/SX1276 модулів
// Військова система зв'язку для дронів-камікадзе проти москалів
// 🇺🇦 Slava Ukraini! Death to russian occupants! 🇺🇦

#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <atomic>
#include <mutex>
#include <thread>
#include <queue>
#include <condition_variable>
#include <memory>
#include <chrono>
#include <functional>

namespace SwarmControl {

// Forward declarations
    struct CombatLoRaMessage;

// LoRa frequency bands
    enum class LoRaFrequencyBand {
        BAND_433MHz,    // 433 MHz ISM band
        BAND_868MHz,    // 868 MHz EU ISM band
        BAND_915MHz,    // 915 MHz US ISM band
        BAND_2_4GHz     // 2.4 GHz ISM band (via ELRS)
    };

// Message priority levels
    enum class MessagePriority : uint8_t {
        LOW = 64,
        NORMAL = 128,
        HIGH = 192,
        CRITICAL = 255
    };

// Communication protocols
    enum class CommProtocol : uint8_t {
        ELRS_2_4GHz = 1,    // 2.4GHz ELRS via SX1281
        ELRS_915MHz = 2,    // 915MHz ELRS via SX1276
        AUTO_SELECT = 3     // Automatic protocol selection
    };

// Signal quality metrics
    struct SignalQuality {
        int rssi_dbm;           // Received Signal Strength Indicator
        float snr_db;           // Signal to Noise Ratio
        double packet_loss;     // Packet loss percentage (0.0-1.0)
        uint32_t error_count;   // Total error count
        std::chrono::steady_clock::time_point last_update;

        SignalQuality() {
            rssi_dbm = -120;
            snr_db = -10.0f;
            packet_loss = 1.0;
            error_count = 0;
            last_update = std::chrono::steady_clock::now();
        }
    };

// LoRa message structure for application layer
    struct LoRaMessage {
        uint16_t sender_id;
        uint16_t target_id;         // 0xFFFF = broadcast
        uint8_t message_type;
        MessagePriority priority;
        std::vector<uint8_t> payload;
        uint64_t timestamp_ns;
        uint8_t hop_count;

        LoRaMessage() {
            sender_id = 0;
            target_id = 0xFFFF;
            message_type = 0;
            priority = MessagePriority::NORMAL;
            timestamp_ns = 0;
            hop_count = 0;
        }
    };

// Communication statistics
    struct LoRaStatistics {
        uint32_t messages_sent;
        uint32_t messages_received;
        uint32_t messages_forwarded;
        uint32_t messages_dropped;
        uint32_t crc_errors;
        uint32_t encryption_errors;
        uint32_t frequency_changes;
        double average_rssi;
        double packet_loss_rate;
        std::chrono::steady_clock::time_point start_time;
        std::chrono::steady_clock::time_point last_activity;

        LoRaStatistics() {
            memset(this, 0, sizeof(LoRaStatistics));
            start_time = std::chrono::steady_clock::now();
            last_activity = start_time;
        }
    };

// Frequency hopping configuration
    struct FrequencyHoppingConfig {
        bool enabled;
        uint32_t hop_interval_ms;
        uint32_t dwell_time_ms;
        std::vector<uint32_t> frequency_list;
        bool randomize_sequence;
        uint32_t anti_jam_threshold;

        FrequencyHoppingConfig() {
            enabled = true;
            hop_interval_ms = 5000;     // 5 seconds
            dwell_time_ms = 100;        // 100ms minimum dwell
            randomize_sequence = true;
            anti_jam_threshold = 5;     // Failed transmissions before hop
        }
    };

// Mesh routing information
    struct MeshRouteInfo {
        uint16_t destination_id;
        uint16_t next_hop_id;
        uint8_t hop_count;
        double link_quality;
        std::chrono::steady_clock::time_point last_used;
        bool is_valid;

        MeshRouteInfo() {
            destination_id = 0;
            next_hop_id = 0;
            hop_count = 255;
            link_quality = 0.0;
            last_used = std::chrono::steady_clock::now();
            is_valid = false;
        }
    };

// Main LoRa System class
    class LoRaSystem {
    public:
        // Constructor/Destructor
        explicit LoRaSystem(uint16_t drone_id, const std::string& config_path = "");
        ~LoRaSystem();

        // Core lifecycle
        bool initialize();
        bool start();
        bool stop();
        bool is_running() const { return running_.load(); }

        // Message transmission
        bool send_message(const LoRaMessage& message, CommProtocol protocol = CommProtocol::AUTO_SELECT);
        bool send_message_reliable(const LoRaMessage& message, uint8_t max_retries = 3);
        bool broadcast_message(const LoRaMessage& message);
        bool send_emergency_message(const LoRaMessage& message);

        // Message reception
        bool receive_message(LoRaMessage& message, uint32_t timeout_ms = 100);
        bool set_message_handler(uint8_t message_type, std::function<void(const LoRaMessage&)> handler);
        bool remove_message_handler(uint8_t message_type);

        // Frequency management
        bool set_frequency(uint32_t frequency, LoRaFrequencyBand band);
        bool enable_frequency_hopping(const FrequencyHoppingConfig& config);
        bool disable_frequency_hopping();
        bool perform_emergency_frequency_hop();
        std::vector<uint32_t> get_available_frequencies(LoRaFrequencyBand band) const;

        // Power management
        bool set_tx_power(int8_t power_dbm, LoRaFrequencyBand band);
        bool enable_adaptive_power(bool enable);
        int8_t get_optimal_power(uint16_t target_drone_id) const;

        // Signal quality and monitoring
        SignalQuality get_signal_quality(LoRaFrequencyBand band) const;
        bool is_jamming_detected() const { return jamming_detected_.load(); }
        bool run_signal_test(LoRaFrequencyBand band, uint32_t duration_ms);

        // Mesh networking
        bool enable_mesh_routing(bool enable);
        bool add_mesh_route(const MeshRouteInfo& route);
        bool remove_mesh_route(uint16_t destination_id);
        std::vector<MeshRouteInfo> get_mesh_routes() const;
        bool discover_mesh_neighbors();

        // Security and encryption
        bool enable_encryption(bool enable);
        bool set_encryption_key(const std::string& key);
        bool rotate_encryption_key();

        // Statistics and diagnostics
        LoRaStatistics get_statistics() const;
        bool perform_self_test();
        std::string get_status_report() const;
        bool export_diagnostics(const std::string& filename) const;

        // Configuration
        bool reload_configuration();
        bool set_drone_id(uint16_t drone_id);
        bool set_call_sign(const std::string& call_sign);

    private:
        // Hardware interfaces
        class SX1281Interface;  // 2.4GHz ELRS module
        class SX1276Interface;  // 915MHz ELRS module

        std::unique_ptr<SX1281Interface> sx1281_interface_;
        std::unique_ptr<SX1276Interface> sx1276_interface_;

        // Core identification
        uint16_t drone_id_;
        std::string config_path_;
        std::string call_sign_;

        // Threading
        std::thread tx_thread_;
        std::thread rx_thread_;
        std::thread frequency_hopping_thread_;
        std::thread monitoring_thread_;
        std::atomic<bool> running_;

        // Message queues
        std::queue<LoRaMessage> outgoing_queue_;
        std::queue<LoRaMessage> incoming_queue_;
        std::queue<LoRaMessage> priority_queue_;
        std::mutex outgoing_mutex_;
        std::mutex incoming_mutex_;
        std::mutex priority_mutex_;
        std::condition_variable tx_cv_;
        std::condition_variable rx_cv_;

        // Message handlers
        std::map<uint8_t, std::function<void(const LoRaMessage&)>> message_handlers_;
        std::mutex handlers_mutex_;

        // Frequency management
        std::vector<uint32_t> frequency_list_;
        std::atomic<size_t> current_frequency_index_;
        FrequencyHoppingConfig hopping_config_;
        mutable std::mutex frequency_mutex_;

        // Power management
        std::atomic<int8_t> current_power_level_;
        bool adaptive_power_enabled_;
        std::map<uint16_t, int8_t> power_table_; // Per-drone power levels
        mutable std::mutex power_mutex_;

        // Signal monitoring
        std::atomic<bool> jamming_detected_;
        SignalQuality signal_quality_2_4ghz_;
        SignalQuality signal_quality_915mhz_;
        mutable std::mutex signal_mutex_;

        // Mesh networking
        bool mesh_enabled_;
        std::map<uint16_t, MeshRouteInfo> mesh_routes_;
        mutable std::mutex mesh_mutex_;

        // Security
        std::atomic<bool> encryption_enabled_;
        uint8_t encryption_key_[32];  // AES-256 key
        uint8_t hmac_key_[32];        // HMAC key
        std::atomic<uint32_t> sequence_number_;
        mutable std::mutex crypto_mutex_;

        // Statistics
        LoRaStatistics stats_;
        mutable std::mutex stats_mutex_;

        // Worker thread methods
        void transmission_worker();
        void reception_worker();
        void frequency_hopping_worker();
        void monitoring_worker();

        // Protocol implementation
        bool transmit_via_sx1281(const CombatLoRaMessage& message);
        bool transmit_via_sx1276(const CombatLoRaMessage& message);
        bool receive_via_sx1281(CombatLoRaMessage& message, uint32_t timeout_ms);
        bool receive_via_sx1276(CombatLoRaMessage& message, uint32_t timeout_ms);

        // Message processing
        bool encode_message(const LoRaMessage& app_message, CombatLoRaMessage& lora_message);
        bool decode_message(const CombatLoRaMessage& lora_message, LoRaMessage& app_message);
        bool encrypt_payload(const uint8_t* plaintext, size_t plaintext_len,
                             uint8_t* ciphertext, size_t& ciphertext_len);
        bool decrypt_payload(const uint8_t* ciphertext, size_t ciphertext_len,
                             uint8_t* plaintext, size_t& plaintext_len);

        // Frequency management helpers
        bool select_best_frequency(LoRaFrequencyBand band, uint32_t& frequency);
        bool test_frequency_quality(uint32_t frequency, LoRaFrequencyBand band);
        bool detect_jamming(LoRaFrequencyBand band);
        void perform_frequency_hop();

        // Mesh routing helpers
        bool forward_mesh_message(const LoRaMessage& message);
        bool find_mesh_route(uint16_t destination_id, uint16_t& next_hop_id);
        bool update_mesh_route_quality(uint16_t destination_id, double quality);
        void clean_stale_mesh_routes();

        // Signal quality helpers
        void update_signal_quality(LoRaFrequencyBand band, int rssi, float snr);
        double calculate_link_quality(int rssi, float snr, double packet_loss);
        bool adapt_transmission_parameters();

        // Utility methods
        uint16_t calculate_crc16(const uint8_t* data, size_t length);
        bool validate_message_integrity(const CombatLoRaMessage& message);
        uint64_t get_precise_timestamp();
        bool load_configuration();
        bool initialize_encryption();
        void log_event(const std::string& event, const std::string& details = "");

        // Constants
        static constexpr size_t MAX_PAYLOAD_SIZE = 200;
        static constexpr uint32_t DEFAULT_TIMEOUT_MS = 1000;
        static constexpr uint8_t MAX_HOP_COUNT = 10;
        static constexpr double MIN_LINK_QUALITY = 0.3;
        static constexpr auto FREQUENCY_HOP_INTERVAL = std::chrono::seconds(5);
        static constexpr auto SIGNAL_MONITORING_INTERVAL = std::chrono::milliseconds(100);
    };

} // namespace SwarmControl
public:
// Constructor/Destructor
explicit ProductionLoRaSystem(uint16_t drone_id, const std::string& config_path = "");
~ProductionLoRaSystem();

// Core lifecycle
bool initialize();
bool start();
bool stop();
bool is_running() const { return running_.load(); }

// Message transmission
bool send_message(const LoRaMessage& message, CommProtocol protocol = CommProtocol::AUTO_SELECT);
bool send_message_reliable(const LoRaMessage& message, uint8_t max_retries = 3);
bool broadcast_message(const LoRaMessage& message);
bool send_emergency_message(const LoRaMessage& message);

// Message reception
bool receive_message(LoRaMessage& message, uint32_t timeout_ms = 100);
bool set_message_handler(uint8_t message_type, std::function<void(const LoRaMessage&)> handler);
bool remove_message_handler(uint8_t message_type);

// Frequency management
bool set_frequency(uint32_t frequency, LoRaFrequencyBand band);
bool enable_frequency_hopping(const FrequencyHoppingConfig& config);
bool disable_frequency_hopping();
bool perform_emergency_frequency_hop();
std::vector<uint32_t> get_available_frequencies(LoRaFrequencyBand band) const;

// Power management
bool set_tx_power(int8_t power_dbm, LoRaFrequencyBand band);
bool enable_adaptive_power(bool enable);
int8_t get_optimal_power(uint16_t target_drone_id) const;

// Signal quality and monitoring
SignalQuality get_signal_quality(LoRaFrequencyBand band) const;
bool is_jamming_detected() const { return jamming_detected_.load(); }
bool run_signal_test(LoRaFrequencyBand band, uint32_t duration_ms);

// Mesh networking
bool enable_mesh_routing(bool enable);
bool add_mesh_route(const MeshRouteInfo& route);
bool remove_mesh_route(uint16_t destination_id);
std::vector<MeshRouteInfo> get_mesh_routes() const;
bool discover_mesh_neighbors();

// Security and encryption
bool enable_encryption(bool enable);
bool set_encryption_key(const std::string& key);
bool rotate_encryption_key();

// Statistics and diagnostics
LoRaStatistics get_statistics() const;
bool perform_self_test();
std::string get_status_report() const;
bool export_diagnostics(const std::string& filename) const;

// Configuration
bool reload_configuration();
bool set_drone_id(uint16_t drone_id);
bool set_call_sign(const std::string& call_sign);

private:
// Hardware interfaces
class SX1281Interface;  // 2.4GHz ELRS module
class SX1276Interface;  // 915MHz ELRS module

std::unique_ptr<SX1281Interface> sx1281_interface_;
std::unique_ptr<SX1276Interface> sx1276_interface_;

// Core identification
uint16_t drone_id_;
std::string config_path_;
std::string call_sign_;

// Threading
std::thread tx_thread_;
std::thread rx_thread_;
std::thread frequency_hopping_thread_;
std::thread monitoring_thread_;
std::atomic<bool> running_;

// Message queues
std::queue<LoRaMessage> outgoing_queue_;
std::queue<LoRaMessage> incoming_queue_;
std::queue<LoRaMessage> priority_queue_;
std::mutex outgoing_mutex_;
std::mutex incoming_mutex_;
std::mutex priority_mutex_;
std::condition_variable tx_cv_;
std::condition_variable rx_cv_;

// Message handlers
std::map<uint8_t, std::function<void(const LoRaMessage&)>> message_handlers_;
std::mutex handlers_mutex_;

// Frequency management
std::vector<uint32_t> frequency_list_;
std::atomic<size_t> current_frequency_index_;
FrequencyHoppingConfig hopping_config_;
std::mutex frequency_mutex_;

// Power management
std::atomic<int8_t> current_power_level_;
bool adaptive_power_enabled_;
std::map<uint16_t, int8_t> power_table_; // Per-drone power levels
std::mutex power_mutex_;

// Signal monitoring
std::atomic<bool> jamming_detected_;
SignalQuality signal_quality_2_4ghz_;
SignalQuality signal_quality_915mhz_;
std::mutex signal_mutex_;

// Mesh networking
bool mesh_enabled_;
std::map<uint16_t, MeshRouteInfo> mesh_routes_;
std::mutex mesh_mutex_;

// Security
std::atomic<bool> encryption_enabled_;
uint8_t encryption_key_[32];  // AES-256 key
uint8_t hmac_key_[32];        // HMAC key
std::atomic<uint32_t> sequence_number_;
std::mutex crypto_mutex_;

// Statistics
LoRaStatistics stats_;
std::mutex stats_mutex_;

// Worker thread methods
void transmission_worker();
void reception_worker();
void frequency_hopping_worker();
void monitoring_worker();

// Protocol implementation
bool transmit_via_sx1281(const CombatLoRaMessage& message);
bool transmit_via_sx1276(const CombatLoRaMessage& message);
bool receive_via_sx1281(CombatLoRaMessage& message, uint32_t timeout_ms);
bool receive_via_sx1276(CombatLoRaMessage& message, uint32_t timeout_ms);

// Message processing
bool encode_message(const LoRaMessage& app_message, CombatLoRaMessage& lora_message);
bool decode_message(const CombatLoRaMessage& lora_message, LoRaMessage& app_message);
bool encrypt_payload(const uint8_t* plaintext, size_t plaintext_len,
                     uint8_t* ciphertext, size_t& ciphertext_len);
bool decrypt_payload(const uint8_t* ciphertext, size_t ciphertext_len,
                     uint8_t* plaintext, size_t& plaintext_len);

// Frequency management helpers
bool select_best_frequency(LoRaFrequencyBand band, uint32_t& frequency);
bool test_frequency_quality(uint32_t frequency, LoRaFrequencyBand band);
bool detect_jamming(LoRaFrequencyBand band);
void perform_frequency_hop();

// Mesh routing helpers
bool forward_mesh_message(const LoRaMessage& message);
bool find_mesh_route(uint16_t destination_id, uint16_t& next_hop_id);
bool update_mesh_route_quality(uint16_t destination_id, double quality);
void clean_stale_mesh_routes();

// Signal quality helpers
void update_signal_quality(LoRaFrequencyBand band, int rssi, float snr);
double calculate_link_quality(int rssi, float snr, double packet_loss);
bool adapt_transmission_parameters();

// Utility methods
uint16_t calculate_crc16(const uint8_t* data, size_t length);
bool validate_message_integrity(const CombatLoRaMessage& message);
uint64_t get_precise_timestamp();
bool load_configuration();
bool initialize_encryption();
void log_event(const std::string& event, const std::string& details = "");

// Constants
static constexpr size_t MAX_PAYLOAD_SIZE = 200;
static constexpr uint32_t DEFAULT_TIMEOUT_MS = 1000;
static constexpr uint8_t MAX_HOP_COUNT = 10;
static constexpr double MIN_LINK_QUALITY = 0.3;
static constexpr auto FREQUENCY_HOP_INTERVAL = std::chrono::seconds(5);
static constexpr auto SIGNAL_MONITORING_INTERVAL = std::chrono::milliseconds(100);
};

} // namespace SwarmControl