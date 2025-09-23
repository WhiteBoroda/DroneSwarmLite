#pragma once

#include "SwarmTypes.h"
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <unordered_map>
#include <functional>

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

// Message structure
    struct SwarmMessage {
        MessageType type;
        DroneId source_id;
        DroneId destination_id; // 0xFFFF for broadcast
        uint16_t sequence_number;
        uint32_t timestamp_ms;
        std::vector<uint8_t> payload;
        uint8_t priority; // 0-255, 255 = highest
        uint8_t retry_count;

        SwarmMessage() : type(MessageType::HEARTBEAT), source_id(0), destination_id(0xFFFF),
                         sequence_number(0), timestamp_ms(0), priority(128), retry_count(0) {}
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
        Timestamp last_successful_transmission;
    };

    class CommunicationManager {
    public:
        explicit CommunicationManager(DroneId drone_id, const std::string& config_path);
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
        bool establish_mesh_route(DroneId destination);
        std::vector<DroneId> get_mesh_neighbors() const;
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

        // Synchronization and timing
        bool synchronize_network_time();
        uint32_t get_network_timestamp() const;
        bool set_time_reference(uint32_t reference_time);

        // Error handling and recovery
        bool handle_transmission_failure(const SwarmMessage& message);
        bool attempt_message_recovery();
        bool reset_communication_hardware();

        // Statistics and diagnostics
        CommStats get_communication_statistics() const;
        bool run_communication_diagnostics();
        bool test_link_quality(DroneId target_drone);

        // Configuration management
        bool reload_communication_config();
        bool update_lora_config(const LoRaConfig& config);
        bool update_power_limits(uint8_t max_power_dbm);

        // Hardware interface methods
        bool initialize_lora_radio();
        bool initialize_elrs_modules();
        bool configure_antennas();
        bool check_hardware_status();

        // Power management - hot reload support
        bool set_max_power_level(int8_t max_power_dbm);
        bool set_min_power_level(int8_t min_power_dbm);
        bool set_current_power_level(int8_t power_dbm);
        int8_t get_max_power_level() const;
        int8_t get_min_power_level() const;
        int8_t get_current_power_level() const;

        // Frequency management - hot reload support
        bool update_frequency_list(const std::vector<uint32_t>& new_frequencies);
        bool add_frequency(uint32_t frequency);
        bool remove_frequency(uint32_t frequency);
        std::vector<uint32_t> get_frequency_list() const;
        bool set_primary_frequency(uint32_t frequency);
        uint32_t get_current_frequency() const;

// Frequency hopping control - hot reload support
        bool enable_frequency_hopping(bool enable);
        bool set_frequency_hop_interval(uint32_t interval_ms);
        bool set_interference_threshold(double threshold_dbm);
        bool is_frequency_hopping_enabled() const;
        uint32_t get_frequency_hop_interval() const;

// Mesh networking control - hot reload support
        bool enable_mesh_networking(bool enable);
        bool set_mesh_max_hops(uint8_t max_hops);
        bool set_mesh_discovery_interval(uint32_t interval_ms);
        bool is_mesh_enabled() const;
        uint8_t get_mesh_max_hops() const;

// Communication timeouts - hot reload support
        bool set_communication_timeout(uint32_t timeout_ms);
        bool set_heartbeat_interval(uint32_t interval_ms);
        bool set_max_retries(uint8_t max_retries);
        uint32_t get_communication_timeout() const;
        uint32_t get_heartbeat_interval() const;
        uint8_t get_max_retries() const;

// Adaptive power control - hot reload support
        bool enable_adaptive_power_control(bool enable);
        bool set_adaptive_power_step(int8_t step_dbm);
        bool set_rssi_threshold(double threshold_dbm);
        bool is_adaptive_power_enabled() const;

    private:
        // Core identification
        DroneId drone_id_;
        std::string config_path_;

        // Hardware interfaces
        class LoRaRadio;
        class ELRSModule;

        std::unique_ptr<LoRaRadio> lora_radio_;
        std::unique_ptr<ELRSModule> elrs_2_4ghz_;
        std::unique_ptr<ELRSModule> elrs_915mhz_;

        // Threading
        std::thread tx_thread_;
        std::thread rx_thread_;
        std::thread mesh_maintenance_thread_;
        std::thread adaptive_comm_thread_;
        std::atomic<bool> running_;

        // Message queues
        std::queue<SwarmMessage> outgoing_queue_;
        std::queue<SwarmMessage> incoming_queue_;
        std::queue<SwarmMessage> priority_queue_;
        mutable std::mutex outgoing_mutex_;
        mutable std::mutex incoming_mutex_;
        mutable std::mutex priority_mutex_;


        std::condition_variable tx_cv_;
        std::condition_variable rx_cv_;

        // Message handling
        std::unordered_map<MessageType, MessageCallback> message_handlers_;
        mutable std::mutex handlers_mutex_;

        // Sequence tracking
        uint16_t next_sequence_number_;
        std::unordered_map<DroneId, uint16_t> last_received_sequence_;
        mutable std::mutex sequence_mutex_;

        // Mesh networking
        struct MeshNode {
            DroneId node_id;
            int8_t signal_strength;
            uint8_t hop_count;
            Timestamp last_seen;
            bool is_reachable;
        };

        std::unordered_map<DroneId, MeshNode> mesh_neighbors_;
        std::unordered_map<DroneId, std::vector<DroneId>> mesh_routes_;
        mutable std::mutex mesh_mutex_;
        mutable std::mutex freq_mutex_;

        // Communication parameters
        LoRaConfig current_lora_config_;
        uint32_t current_frequency_;
        uint8_t current_power_level_;
        std::vector<uint32_t> frequency_list_;
        size_t frequency_index_;

        // Ground station connection
        std::atomic<bool> ground_station_connected_;
        Timestamp last_ground_contact_;

        // Signal quality monitoring
        std::atomic<int8_t> current_rssi_;
        std::atomic<double> packet_loss_rate_;
        std::atomic<double> link_quality_;

        // Statistics
        CommStats communication_stats_;
        mutable std::mutex stats_mutex_;

        // Timing and synchronization
        uint32_t network_time_offset_;
        Timestamp last_time_sync_;
        mutable std::mutex time_sync_mutex_;

        // Private methods - Threading
        void transmission_loop();
        void reception_loop();
        void mesh_maintenance_loop();
        void adaptive_communication_loop();

        // Message processing
        bool process_outgoing_message(const SwarmMessage& message);
        bool process_incoming_message(const SwarmMessage& message);
        bool validate_message(const SwarmMessage& message) const;
        bool decrypt_message(SwarmMessage& message);
        bool encrypt_message(SwarmMessage& message);

        // Protocol implementation
        bool transmit_lora_message(const SwarmMessage& message);
        bool transmit_elrs_message(const SwarmMessage& message, CommProtocol protocol);
        bool receive_lora_message(SwarmMessage& message);
        bool receive_elrs_message(SwarmMessage& message);

        // Mesh networking helpers
        bool discover_mesh_neighbors();
        bool update_mesh_topology();
        bool find_best_route(DroneId destination, std::vector<DroneId>& route);
        bool clean_stale_mesh_entries();

        // Adaptive communication helpers
        bool measure_channel_quality();
        bool detect_interference();
        bool adjust_transmission_parameters();
        bool select_alternative_protocol();

        // Error handling
        bool handle_checksum_error(const SwarmMessage& message);
        bool handle_timeout_error(const SwarmMessage& message);
        bool handle_hardware_error();
        bool attempt_automatic_recovery();

        // Utility methods
        uint16_t calculate_checksum(const SwarmMessage& message) const;
        bool is_message_duplicate(const SwarmMessage& message) const;
        uint8_t calculate_signal_strength(int8_t rssi) const;
        bool should_forward_message(const SwarmMessage& message) const;

        // Configuration helpers
        bool load_communication_config();
        bool parse_frequency_list();
        bool configure_transmission_power();
        bool setup_mesh_parameters();

        // Hardware abstraction
        bool write_to_lora_register(uint8_t address, uint8_t value);
        uint8_t read_from_lora_register(uint8_t address);
        bool configure_lora_parameters(const LoRaConfig& config);
        bool check_lora_status();

        // Logging
        void log_communication_event(const std::string& event, const std::string& details = "");
        void log_message_transmission(const SwarmMessage& message, bool success);
        void log_signal_quality(int8_t rssi, double packet_loss);
    };

} // namespace SwarmControl//
// Created by yv on 22.09.2025.
//

#ifndef DRONESWARMLITE_COMMUNICATIONMANAGER_H
#define DRONESWARMLITE_COMMUNICATIONMANAGER_H

#endif //DRONESWARMLITE_COMMUNICATIONMANAGER_H
