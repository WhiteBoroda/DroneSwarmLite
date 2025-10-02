//
// CommunicationManager.h - Complete header with encryption support
//

#pragma once

#include "SwarmTypes.h"
#include "CryptoManager.h"
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <chrono>
#include <memory>

namespace SwarmControl {

// Message types for swarm communication
    enum class MessageType : uint8_t {
        TELEMETRY = 0x01,
        COMMAND = 0x02,
        STATUS = 0x03,
        FORMATION_UPDATE = 0x04,
        LEADER_ANNOUNCEMENT = 0x05,
        EMERGENCY = 0x06,
        HEARTBEAT = 0x07,
        MISSION_UPDATE = 0x08,
        UWB_SYNC = 0x09,
        VIDEO_CONTROL = 0x0A,
        FREQUENCY_HOP = 0x0B,
        MESH_DISCOVERY = 0x0C,
        ACK = 0xFE,
        NACK = 0xFF
    };

// Communication protocols
    enum class CommProtocol : uint8_t {
        LORA_P2P,
        LORA_MESH,
        ELRS_2_4GHZ,
        ELRS_915MHZ,
        EMERGENCY_BROADCAST
    };

// Communication status
    enum class CommunicationStatus : uint8_t {
        EXCELLENT = 0,
        GOOD = 1,
        POOR = 2,
        CRITICAL = 3,
        LOST = 4
    };

// Message structure
    struct SwarmMessage {
        MessageType type;
        DroneID source_id;
        DroneID destination_id; // 0xFFFF for broadcast
        uint16_t sequence_number;
        uint32_t timestamp_ms;
        std::vector<uint8_t> payload;
        uint8_t priority; // 0-255, 255 = highest
        uint8_t retry_count;
        bool encrypted; // Encryption flag

        SwarmMessage() : type(MessageType::HEARTBEAT), source_id(0), destination_id(0xFFFF),
                         sequence_number(0), timestamp_ms(0), priority(128), retry_count(0), encrypted(false) {}
    };
    struct ELRSConfig {
        enum FrequencyRange { ELRS_2_4GHZ, ELRS_915MHZ } frequency_range;
        int8_t power_level;
        uint16_t packet_rate;
        enum SwitchMode { ELRS_SWITCH_HYBRID, ELRS_SWITCH_WIDE } switch_mode;
    };

    struct ELRSFrame {
        struct {
            uint8_t frame_type;
            DroneID destination;
            DroneID sender;
            uint16_t sequence;
            uint16_t crc;
        } header;
        uint16_t channels[16]; // ELRS supports up to 16 channels
    };

// ELRS constants
    static const uint8_t ELRS_FRAME_RC_CHANNELS_PACKED = 0x16;
    static const uint8_t ELRS_PACKET_RATE_500HZ = 0;
    static const uint8_t ELRS_PACKET_RATE_100HZ = 4;
    static const uint8_t ELRS_MAX_CHANNELS = 16;
    static const size_t ELRS_MAX_FRAME_SIZE = 64;

// LoRa message structure for hardware communication
    struct LoRaMessage {
        uint32_t magic;           // 0xDEADBEEF - protocol signature
        uint8_t version;          // Protocol version
        uint8_t message_type;     // Message type
        DroneID sender_id;        // Sender ID
        DroneID target_id;        // Target ID (0 = broadcast)
        uint16_t sequence;        // Sequence number
        uint16_t payload_size;    // Payload size
        uint32_t checksum;        // CRC32 checksum
        uint8_t payload[512];     // Payload data

        LoRaMessage() {
            magic = 0xDEADBEEF;
            version = 1;
            message_type = 0;
            sender_id = 0;
            target_id = 0;
            sequence = 0;
            payload_size = 0;
            checksum = 0;
            memset(payload, 0, sizeof(payload));
        }
    };

// LoRa configuration
    struct LoRaConfig {
        uint32_t frequency;
        int8_t power;
        uint32_t bandwidth;
        uint8_t spreading_factor;
        uint8_t coding_rate;

        LoRaConfig() : frequency(868100000), power(10), bandwidth(125000),
                       spreading_factor(9), coding_rate(8) {}
    };

// Communication statistics
    struct CommStats {
        uint32_t messages_sent;
        uint32_t messages_received;
        uint32_t messages_failed;
        uint32_t retransmissions;
        double average_rssi;
        double packet_loss_rate;
        uint32_t frequency_hops;
        std::chrono::steady_clock::time_point last_successful_transmission;
    };

// Mesh neighbor info
    struct MeshNeighbor {
        DroneID drone_id;
        int8_t rssi;
        double packet_loss;
        uint32_t last_seen;
        bool is_reliable;
    };

// Mesh route info
    struct MeshRoute {
        DroneID destination;
        DroneID next_hop;
        uint8_t hop_count;
        double quality;
        uint32_t timestamp;
    };

    class CommunicationManager {
    public:
        explicit CommunicationManager(DroneID drone_id, const std::string& config_path);
        ~CommunicationManager();

        // Core lifecycle
        bool initialize();
        bool start();
        void stop();

        // Message transmission
        bool send_message(const SwarmMessage& message, CommProtocol protocol = CommProtocol::LORA_P2P);
        bool broadcast_message(const SwarmMessage& message);
        bool send_emergency_message(const SwarmMessage& message);

        // Message reception
        using MessageCallback = std::function<void(const SwarmMessage&)>;
        bool register_message_handler(MessageType type, MessageCallback callback);
        bool unregister_message_handler(MessageType type);

        // Protocol-specific methods
        bool send_via_lora(const SwarmMessage& message, const LoRaConfig& config);
        bool send_via_elrs_2_4ghz(const SwarmMessage& message);
        bool send_via_elrs_915mhz(const SwarmMessage& message);

        // Mesh networking
        bool join_mesh_network();
        bool leave_mesh_network();
        bool establish_mesh_route(DroneID destination);
        std::vector<DroneID> get_mesh_neighbors() const;
        bool forward_mesh_message(const SwarmMessage& message);

        // Ground station communication
        bool connect_to_ground_station();
        bool disconnect_from_ground_station();
        bool send_to_ground_station(const SwarmMessage& message);
        CommunicationStatus get_ground_station_status() const;

        // Adaptive communication
        bool adapt_transmission_power();
        bool perform_frequency_hop();
        bool scan_for_interference();
        bool select_best_frequency();
        uint32_t get_current_frequency() const;

        // Signal quality monitoring
        int8_t get_current_rssi() const;
        double get_packet_loss_rate() const;
        double get_link_quality() const;
        bool is_link_stable() const;

        // Queue management
        bool enqueue_outgoing_message(const SwarmMessage& message);
        SwarmMessage dequeue_incoming_message();
        size_t get_outgoing_queue_size() const;
        size_t get_incoming_queue_size() const;
        bool clear_message_queues();

        // Statistics
        CommStats get_communication_stats() const;
        void reset_statistics();



        //=============================================================================
        // NEW METHODS ADDED FROM IMPLEMENTATION
        //=============================================================================

        // Frequency management
        bool update_frequency_list(const std::vector<uint32_t>& new_frequencies);
        bool add_frequency(uint32_t frequency);
        bool remove_frequency(uint32_t frequency);
        bool change_frequency(uint32_t new_frequency);
        uint32_t select_best_frequency();
        bool enable_frequency_hopping(bool enable);
        bool set_frequency_hop_interval(uint32_t interval_ms);
        bool set_interference_threshold(double threshold_dbm);

        // Power management  
        bool set_power_level(int8_t power_dbm);
        bool set_min_power_level(int8_t min_power);
        bool set_max_power_level(int8_t max_power);
        bool enable_adaptive_power_control(bool enable);
        bool set_adaptive_power_step(int8_t step_dbm);
        bool set_rssi_threshold(double threshold_dbm);
        bool is_adaptive_power_enabled() const;

        // Communication timeouts
        bool set_communication_timeout(uint32_t timeout_ms);
        bool set_heartbeat_interval(uint32_t interval_ms);
        bool set_max_retries(uint8_t max_retries);
        uint32_t get_communication_timeout() const;
        uint32_t get_heartbeat_interval() const;
        uint8_t get_max_retries() const;

        // Mesh networking configuration
        bool enable_mesh_networking(bool enable);
        bool set_mesh_max_hops(uint8_t max_hops);
        bool set_mesh_discovery_interval(uint32_t interval_ms);
        bool is_mesh_enabled() const;
        uint8_t get_mesh_max_hops() const;

        // Anti-jamming and interference
        bool detect_interference();
        void broadcast_frequency_change(uint32_t new_frequency);

        //=============================================================================
        // ENCRYPTION METHODS (ADDED)
        //=============================================================================

        // Encryption setup
        bool initialize_encryption(std::shared_ptr<CryptoManager> crypto_manager);
        bool set_encryption_key(const std::vector<uint8_t>& key);
        bool enable_message_encryption(bool enable);
        bool is_encryption_enabled() const;

        // Message encryption/decryption
        bool encrypt_message(SwarmMessage& message);
        bool decrypt_message(SwarmMessage& message);
        bool encrypt_payload(std::vector<uint8_t>& payload, const std::vector<uint8_t>& key);
        bool decrypt_payload(std::vector<uint8_t>& payload, const std::vector<uint8_t>& key);

        // Key management
        bool rotate_encryption_key();
        bool distribute_key_to_swarm(const std::vector<uint8_t>& new_key);
        bool handle_key_rotation_request(const SwarmMessage& message);
        std::vector<uint8_t> generate_session_key();

        // Message authentication
        bool add_message_authentication(SwarmMessage& message);
        bool verify_message_authentication(const SwarmMessage& message);
        std::vector<uint8_t> calculate_message_hmac(const SwarmMessage& message, const std::vector<uint8_t>& key);

    private:
        // Core member variables
        DroneID drone_id_;
        std::string config_path_;
        std::atomic<bool> running_;
        uint16_t next_sequence_number_;

        // LoRa hardware state
        uint32_t current_frequency_;
        int8_t current_power_level_;
        size_t frequency_index_;
        LoRaConfig current_lora_config_;

        // Communication state
        std::atomic<bool> ground_station_connected_;
        std::atomic<int8_t> current_rssi_;
        std::atomic<double> packet_loss_rate_;
        std::atomic<double> link_quality_;
        int32_t network_time_offset_;

        // Configuration variables (loaded from config file)
        uint32_t communication_timeout_;
        uint32_t heartbeat_interval_;
        uint8_t max_retries_;
        bool adaptive_power_enabled_;
        int8_t adaptive_power_step_;
        double rssi_threshold_;
        bool frequency_hopping_enabled_;
        uint32_t frequency_hop_interval_;
        double interference_threshold_;
        bool mesh_enabled_;
        uint8_t mesh_max_hops_;
        uint32_t mesh_discovery_interval_;
        int8_t min_power_level_;
        int8_t max_power_level_;

        // Frequency management
        std::vector<uint32_t> frequency_list_;
        std::mutex freq_mutex_;
        std::chrono::steady_clock::time_point last_frequency_hop_;

        // Adaptive power control
        std::chrono::steady_clock::time_point last_adaptive_update_;

        // Mesh networking
        std::unordered_map<DroneID, MeshNeighbor> mesh_neighbors_;
        std::unordered_map<DroneID, MeshRoute> mesh_routes_;
        std::mutex mesh_mutex_;
        std::chrono::steady_clock::time_point last_mesh_discovery_;

        // Message queues and threading
        std::queue<SwarmMessage> outgoing_queue_;
        std::queue<SwarmMessage> priority_queue_;
        std::vector<SwarmMessage> incoming_queue_;
        std::mutex queue_mutex_;
        std::condition_variable queue_condition_;

        // Communication threads
        std::unique_ptr<std::thread> tx_thread_;
        std::unique_ptr<std::thread> rx_thread_;
        std::unique_ptr<std::thread> mesh_thread_;
        std::unique_ptr<std::thread> adaptive_thread_;

        // Statistics
        CommStats communication_stats_;

        // Message handling
        std::unordered_map<MessageType, MessageCallback> message_handlers_;
        std::unordered_set<uint16_t> seen_sequences_;

        //=============================================================================
        // ENCRYPTION MEMBER VARIABLES (ADDED)
        //=============================================================================

        // Encryption state
        std::shared_ptr<CryptoManager> crypto_manager_;
        bool encryption_enabled_;
        std::vector<uint8_t> current_encryption_key_;
        std::vector<uint8_t> hmac_key_;
        uint32_t key_rotation_counter_;
        std::chrono::steady_clock::time_point last_key_rotation_;
        std::mutex encryption_mutex_;

        // Session keys for different drones
        std::unordered_map<DroneID, std::vector<uint8_t>> drone_session_keys_;

        //=============================================================================
        // PRIVATE METHODS
        //=============================================================================

        // Configuration loading
        bool load_communication_config();

        // Hardware abstraction layer
        bool initialize_lora_radio();
        bool initialize_elrs_modules();
        bool set_lora_frequency(uint32_t frequency);
        bool set_lora_power(int8_t power_dbm);
        bool write_lora_register(uint8_t reg, uint8_t value);
        bool read_lora_register(uint8_t reg, uint8_t* value);

        // Message processing
        bool validate_message(const SwarmMessage& message) const;
        bool transmit_lora_message(const LoRaMessage* message);
        bool receive_lora_message(LoRaMessage* message);
        uint32_t calculate_checksum(const LoRaMessage* message);
        double test_frequency_quality(uint32_t frequency);
        double measure_noise_floor();

        // Threading loops
        void transmission_loop();
        void reception_loop();
        void mesh_maintenance_loop();
        void adaptive_communication_loop();

        // Utility
        void log_communication_event(const std::string& event, const std::string& details = "");
        uint32_t get_current_time_ms() const;

        bool initialize_elrs_2g4_module();
        bool initialize_elrs_915_module();
        void pack_message_into_elrs_channels(const SwarmMessage& message, uint16_t* channels);
        bool transmit_elrs_frame(const ELRSFrame& frame, const ELRSConfig& config);
        uint16_t calculate_elrs_crc(const ELRSFrame& frame);

        // LoRa register access (fixed circular dependency)
        uint8_t read_lora_register_direct(uint8_t address);

        // ELRS configuration methods (add to public section)
        bool enable_elrs(bool enabled);
        bool set_elrs_2g4_power(int8_t power_dbm);
        bool set_elrs_915_power(int8_t power_dbm);
        bool set_elrs_2g4_packet_rate(uint16_t rate_hz);
        bool set_elrs_915_packet_rate(uint16_t rate_hz);

        // Enhanced mesh networking methods (add to public section)
        bool set_mesh_max_hops(uint8_t max_hops);
        bool set_mesh_discovery_interval(uint32_t interval_ms);
        bool set_mesh_routing_algorithm(const std::string& algorithm);

        // Additional power and timing methods (add to public section)
        bool set_rssi_threshold(double threshold_dbm);
        bool enable_adaptive_power(bool enabled);

    };

} // namespace SwarmControl