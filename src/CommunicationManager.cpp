#include "../include/CommunicationManager.h"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <algorithm>
#include <random>

namespace SwarmControl {

    CommunicationManager::CommunicationManager(DroneID drone_id, const std::string& config_path)
            : drone_id_(drone_id)
            , config_path_(config_path)
            , running_(false)
            , next_sequence_number_(1)
            , current_frequency_(868100000) // Default to 868.1 MHz
            , current_power_level_(10) // Default 10 dBm
            , frequency_index_(0)
            , ground_station_connected_(false)
            , current_rssi_(-100)
            , packet_loss_rate_(0.0)
            , link_quality_(0.0)
            , network_time_offset_(0) {

        // Initialize frequency list from config
        frequency_list_ = {
                868100000, // 868.1 MHz
                868300000, // 868.3 MHz
                868500000, // 868.5 MHz
                915000000  // 915 MHz (if available)
        };

        log_communication_event("CommunicationManager initialized", "Drone ID: " + std::to_string(drone_id_));
    }

    CommunicationManager::~CommunicationManager() {
        if (running_.load()) {
            stop();
        }
    }

    bool CommunicationManager::initialize() {
        log_communication_event("Initializing communication systems...");

        try {
            // Load communication configuration
            if (!load_communication_config()) {
                log_communication_event("Failed to load communication config", "ERROR");
                return false;
            }

            // Initialize LoRa radio
            if (!initialize_lora_radio()) {
                log_communication_event("Failed to initialize LoRa radio", "ERROR");
                return false;
            }

            // Initialize ELRS modules
            if (!initialize_elrs_modules()) {
                log_communication_event("ELRS modules initialization failed", "WARNING");
                // Continue without ELRS - not critical
            }

            // Set initial LoRa configuration
            current_lora_config_.frequency = current_frequency_;
            current_lora_config_.power_level = current_power_level_;
            current_lora_config_.bandwidth = 125; // kHz
            current_lora_config_.spreading_factor = 7;
            current_lora_config_.coding_rate = 5;

            log_communication_event("Communication initialization completed successfully");
            return true;

        } catch (const std::exception& e) {
            log_communication_event("Exception during initialization: " + std::string(e.what()), "ERROR");
            return false;
        }
    }

    bool CommunicationManager::start() {
        if (running_.load()) {
            log_communication_event("Communication already running", "WARNING");
            return true;
        }

        log_communication_event("Starting communication threads...");

        try {
            running_.store(true);

            // Start main communication threads
            tx_thread_ = std::thread(&CommunicationManager::transmission_loop, this);
            rx_thread_ = std::thread(&CommunicationManager::reception_loop, this);
            mesh_maintenance_thread_ = std::thread(&CommunicationManager::mesh_maintenance_loop, this);
            adaptive_comm_thread_ = std::thread(&CommunicationManager::adaptive_communication_loop, this);

            log_communication_event("Communication started successfully");
            return true;

        } catch (const std::exception& e) {
            log_communication_event("Exception during start: " + std::string(e.what()), "ERROR");
            running_.store(false);
            return false;
        }
    }

    void CommunicationManager::stop() {
        log_communication_event("Stopping communication...");

        running_.store(false);

        // Wake up all threads
        tx_cv_.notify_all();
        rx_cv_.notify_all();

        // Wait for threads to finish
        if (tx_thread_.joinable()) {
            tx_thread_.join();
        }
        if (rx_thread_.joinable()) {
            rx_thread_.join();
        }
        if (mesh_maintenance_thread_.joinable()) {
            mesh_maintenance_thread_.join();
        }
        if (adaptive_comm_thread_.joinable()) {
            adaptive_comm_thread_.join();
        }

        log_communication_event("Communication stopped");
    }

// Message transmission methods
    bool CommunicationManager::send_message(const SwarmMessage& message, CommProtocol protocol) {
        if (!running_.load()) {
            return false;
        }

        // Add to outgoing queue
        return enqueue_outgoing_message(message);
    }

    bool CommunicationManager::broadcast_message(const SwarmMessage& message) {
        SwarmMessage broadcast_msg = message;
        broadcast_msg.destination_id = 0xFFFF; // Broadcast address
        broadcast_msg.source_id = drone_id_;
        broadcast_msg.sequence_number = next_sequence_number_++;
        broadcast_msg.timestamp_ms = get_network_timestamp();

        return send_message(broadcast_msg, CommProtocol::LORA_P2P);
    }

    bool CommunicationManager::send_emergency_message(const SwarmMessage& message) {
        SwarmMessage emergency_msg = message;
        emergency_msg.type = MessageType::EMERGENCY;
        emergency_msg.priority = 255; // Highest priority
        emergency_msg.source_id = drone_id_;
        emergency_msg.destination_id = 0xFFFF; // Broadcast

        // Send immediately on priority queue
        std::lock_guard<std::mutex> lock(priority_mutex_);
        priority_queue_.push(emergency_msg);
        tx_cv_.notify_one();

        return true;
    }

    bool CommunicationManager::enqueue_outgoing_message(const SwarmMessage& message) {
        std::lock_guard<std::mutex> lock(outgoing_mutex_);

        // Check queue size to prevent overflow
        if (outgoing_queue_.size() >= 100) {
            log_communication_event("Outgoing queue full, dropping oldest message", "WARNING");
            outgoing_queue_.pop();
        }

        outgoing_queue_.push(message);
        tx_cv_.notify_one();

        return true;
    }

    SwarmMessage CommunicationManager::dequeue_incoming_message() {
        std::lock_guard<std::mutex> lock(incoming_mutex_);

        if (incoming_queue_.empty()) {
            return SwarmMessage(); // Return default message
        }

        SwarmMessage message = incoming_queue_.front();
        incoming_queue_.pop();
        return message;
    }

// Protocol-specific transmission methods
    bool CommunicationManager::send_via_lora(const SwarmMessage& message, const LoRaConfig& config) {
        // TODO: Implement actual LoRa transmission
        // This would involve:
        // 1. Configure LoRa parameters
        // 2. Serialize message
        // 3. Transmit via LoRa hardware
        // 4. Wait for transmission complete

        log_communication_event("LoRa TX", "Freq: " + std::to_string(config.frequency) +
                                           ", Power: " + std::to_string(config.power_level) + "dBm");

        // Simulate transmission delay
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Update statistics
        std::lock_guard<std::mutex> lock(stats_mutex_);
        communication_stats_.messages_sent++;
        communication_stats_.last_successful_transmission = std::chrono::steady_clock::now();

        return true;
    }

    bool CommunicationManager::send_via_elrs_2_4ghz(const SwarmMessage& message) {
        // TODO: Implement ELRS 2.4GHz transmission
        log_communication_event("ELRS 2.4GHz TX", "Size: " + std::to_string(message.payload.size()));

        std::lock_guard<std::mutex> lock(stats_mutex_);
        communication_stats_.messages_sent++;

        return true;
    }

    bool CommunicationManager::send_via_elrs_915mhz(const SwarmMessage& message) {
        // TODO: Implement ELRS 915MHz transmission
        log_communication_event("ELRS 915MHz TX", "Size: " + std::to_string(message.payload.size()));

        std::lock_guard<std::mutex> lock(stats_mutex_);
        communication_stats_.messages_sent++;

        return true;
    }

// Adaptive communication methods
    bool CommunicationManager::adapt_transmission_power() {
        if (current_rssi_.load() < -90) {
            // Signal weak, increase power
            if (current_power_level_ < 20) {
                current_power_level_ += 2;
                current_lora_config_.power_level = current_power_level_;
                log_communication_event("Power increased", std::to_string(current_power_level_) + "dBm");
                return true;
            }
        } else if (current_rssi_.load() > -60) {
            // Signal strong, decrease power to save energy
            if (current_power_level_ > 0) {
                current_power_level_ -= 1;
                current_lora_config_.power_level = current_power_level_;
                log_communication_event("Power decreased", std::to_string(current_power_level_) + "dBm");
                return true;
            }
        }

        return false;
    }

    bool CommunicationManager::perform_frequency_hop() {
        // Move to next frequency in the list
        frequency_index_ = (frequency_index_ + 1) % frequency_list_.size();
        uint32_t new_frequency = frequency_list_[frequency_index_];

        if (new_frequency != current_frequency_) {
            current_frequency_ = new_frequency;
            current_lora_config_.frequency = current_frequency_;

            // TODO: Actually configure hardware to new frequency

            log_communication_event("Frequency hop", std::to_string(current_frequency_) + " Hz");

            std::lock_guard<std::mutex> lock(stats_mutex_);
            communication_stats_.frequency_hops++;

            return true;
        }

        return false;
    }

    bool CommunicationManager::scan_for_interference() {
        // TODO: Implement interference scanning
        // This would measure signal strength on each frequency

        bool interference_detected = false;

        // Simulate interference detection
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_real_distribution<> dis(0.0, 1.0);

        if (dis(gen) < 0.1) { // 10% chance of interference
            interference_detected = true;
            log_communication_event("Interference detected", "Frequency: " + std::to_string(current_frequency_));
        }

        return interference_detected;
    }

// Signal quality monitoring
    int8_t CommunicationManager::get_current_rssi() const {
        return current_rssi_.load();
    }

    double CommunicationManager::get_packet_loss_rate() const {
        return packet_loss_rate_.load();
    }

    double CommunicationManager::get_link_quality() const {
        return link_quality_.load();
    }

    bool CommunicationManager::is_link_stable() const {
        return (current_rssi_.load() > -90 && packet_loss_rate_.load() < 0.1);
    }

// Threading implementations - main loops
    void CommunicationManager::transmission_loop() {
        log_communication_event("TX loop started");

        while (running_.load()) {
            try {
                SwarmMessage message_to_send;
                bool have_message = false;

                // Check priority queue first
                {
                    std::lock_guard<std::mutex> lock(priority_mutex_);
                    if (!priority_queue_.empty()) {
                        message_to_send = priority_queue_.front();
                        priority_queue_.pop();
                        have_message = true;
                    }
                }

                // If no priority messages, check normal queue
                if (!have_message) {
                    std::lock_guard<std::mutex> lock(outgoing_mutex_);
                    if (!outgoing_queue_.empty()) {
                        message_to_send = outgoing_queue_.front();
                        outgoing_queue_.pop();
                        have_message = true;
                    }
                }

                if (have_message) {
                    process_outgoing_message(message_to_send);
                } else {
                    // Wait for new messages
                    std::unique_lock<std::mutex> lock(outgoing_mutex_);
                    tx_cv_.wait_for(lock, std::chrono::milliseconds(100));
                }

            } catch (const std::exception& e) {
                log_communication_event("Exception in TX loop: " + std::string(e.what()), "ERROR");
            }
        }

        log_communication_event("TX loop stopped");
    }

    void CommunicationManager::reception_loop() {
        log_communication_event("RX loop started");

        while (running_.load()) {
            try {
                // TODO: Implement actual message reception from hardware
                // For now, just simulate periodic reception

                std::this_thread::sleep_for(std::chrono::milliseconds(50));

                // Simulate receiving a message occasionally
                static std::random_device rd;
                static std::mt19937 gen(rd());
                static std::uniform_real_distribution<> dis(0.0, 1.0);

                if (dis(gen) < 0.1) { // 10% chance per loop iteration
                    SwarmMessage received_msg;
                    received_msg.type = MessageType::HEARTBEAT;
                    received_msg.source_id = 999; // Simulate message from another drone
                    received_msg.destination_id = drone_id_;
                    received_msg.timestamp_ms = get_network_timestamp();

                    process_incoming_message(received_msg);
                }

            } catch (const std::exception& e) {
                log_communication_event("Exception in RX loop: " + std::string(e.what()), "ERROR");
            }
        }

        log_communication_event("RX loop stopped");
    }

    void CommunicationManager::mesh_maintenance_loop() {
        log_communication_event("Mesh maintenance loop started");

        auto next_update = std::chrono::steady_clock::now();
        const auto update_period = std::chrono::seconds(10);

        while (running_.load()) {
            try {
                // Clean stale mesh entries
                clean_stale_mesh_entries();

                // Discover new neighbors
                discover_mesh_neighbors();

                // Update mesh topology
                update_mesh_topology();

            } catch (const std::exception& e) {
                log_communication_event("Exception in mesh maintenance: " + std::string(e.what()), "ERROR");
            }

            next_update += update_period;
            std::this_thread::sleep_until(next_update);
        }

        log_communication_event("Mesh maintenance loop stopped");
    }

    void CommunicationManager::adaptive_communication_loop() {
        log_communication_event("Adaptive communication loop started");

        auto next_update = std::chrono::steady_clock::now();
        const auto update_period = std::chrono::seconds(5);

        while (running_.load()) {
            try {
                // Monitor signal quality
                measure_channel_quality();

                // Adapt transmission power
                adapt_transmission_power();

                // Check for interference and hop if needed
                if (scan_for_interference()) {
                    perform_frequency_hop();
                }

                // Update link quality metrics
                double rssi = current_rssi_.load();
                double loss = packet_loss_rate_.load();
                link_quality_.store(std::max(0.0, (100.0 + rssi) / 100.0 * (1.0 - loss)));

            } catch (const std::exception& e) {
                log_communication_event("Exception in adaptive comm: " + std::string(e.what()), "ERROR");
            }

            next_update += update_period;
            std::this_thread::sleep_until(next_update);
        }

        log_communication_event("Adaptive communication loop stopped");
    }

// Message processing helpers
    bool CommunicationManager::process_outgoing_message(const SwarmMessage& message) {
        // Validate message
        if (!validate_message(message)) {
            log_communication_event("Invalid outgoing message", "WARNING");
            return false;
        }

        // Encrypt if needed
        SwarmMessage encrypted_message = message;
        encrypt_message(encrypted_message);

        // Send via appropriate protocol
        bool success = false;
        switch (message.type) {
            case MessageType::EMERGENCY:
                // Emergency messages use highest power and multiple protocols
                success = send_via_lora(encrypted_message, current_lora_config_);
                break;

            case MessageType::TELEMETRY:
            case MessageType::HEARTBEAT:
                // Regular telemetry via LoRa
                success = send_via_lora(encrypted_message, current_lora_config_);
                break;

            case MessageType::COMMAND:
            case MessageType::FORMATION_UPDATE:
                // Critical commands via LoRa with confirmation
                success = send_via_lora(encrypted_message, current_lora_config_);
                break;

            default:
                success = send_via_lora(encrypted_message, current_lora_config_);
                break;
        }

        if (success) {
            log_message_transmission(message, true);
        } else {
            log_message_transmission(message, false);
            std::lock_guard<std::mutex> lock(stats_mutex_);
            communication_stats_.messages_failed++;
        }

        return success;
    }

    bool CommunicationManager::process_incoming_message(const SwarmMessage& message) {
        // Validate and decrypt
        SwarmMessage decrypted_message = message;
        if (!validate_message(decrypted_message)) {
            return false;
        }

        decrypt_message(decrypted_message);

        // Update statistics
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            communication_stats_.messages_received++;
        }

        // Check for duplicates
        if (is_message_duplicate(decrypted_message)) {
            return false;
        }

        // Update sequence tracking
        {
            std::lock_guard<std::mutex> lock(sequence_mutex_);
            last_received_sequence_[decrypted_message.source_id] = decrypted_message.sequence_number;
        }

        // Handle message based on type
        {
            std::lock_guard<std::mutex> lock(handlers_mutex_);
            auto handler_it = message_handlers_.find(decrypted_message.type);
            if (handler_it != message_handlers_.end()) {
                // Call registered handler
                handler_it->second(decrypted_message);
            }
        }

        // Add to incoming queue for application processing
        {
            std::lock_guard<std::mutex> lock(incoming_mutex_);
            if (incoming_queue_.size() < 50) { // Prevent overflow
                incoming_queue_.push(decrypted_message);
            }
        }

        return true;
    }

// Helper implementations
    bool CommunicationManager::validate_message(const SwarmMessage& message) const {
        // Basic validation
        if (message.payload.size() > 250) { // Max LoRa payload
            return false;
        }

        if (message.source_id == 0 || message.source_id == 0xFFFF) {
            return false;
        }

        return true;
    }

    bool CommunicationManager::encrypt_message(SwarmMessage& message) {
        // TODO: Implement encryption
        // For now, just add a simple checksum
        uint16_t checksum = calculate_checksum(message);
        message.payload.push_back(checksum & 0xFF);
        message.payload.push_back((checksum >> 8) & 0xFF);

        return true;
    }

    bool CommunicationManager::decrypt_message(SwarmMessage& message) {
        // TODO: Implement decryption
        // For now, just validate checksum
        if (message.payload.size() < 2) {
            return false;
        }

        // Extract checksum
        uint16_t received_checksum = message.payload[message.payload.size()-2] |
                                     (message.payload[message.payload.size()-1] << 8);
        message.payload.resize(message.payload.size() - 2);

        uint16_t calculated_checksum = calculate_checksum(message);

        return (received_checksum == calculated_checksum);
    }

    uint16_t CommunicationManager::calculate_checksum(const SwarmMessage& message) const {
        uint16_t checksum = 0;
        checksum ^= static_cast<uint16_t>(message.type);
        checksum ^= message.source_id;
        checksum ^= message.destination_id;
        checksum ^= message.sequence_number;

        for (uint8_t byte : message.payload) {
            checksum ^= byte;
        }

        return checksum;
    }

    bool CommunicationManager::is_message_duplicate(const SwarmMessage& message) const {
        std::lock_guard<std::mutex> lock(sequence_mutex_);

        auto it = last_received_sequence_.find(message.source_id);
        if (it != last_received_sequence_.end()) {
            return (message.sequence_number <= it->second);
        }

        return false;
    }

// Configuration and initialization helpers
    bool CommunicationManager::load_communication_config() {
        try {
            YAML::Node config = YAML::LoadFile(config_path_);

            if (config["lora"]) {
                auto lora_config = config["lora"];

                if (lora_config["primary_frequencies"]) {
                    frequency_list_.clear();
                    for (const auto& freq : lora_config["primary_frequencies"]) {
                        frequency_list_.push_back(freq.as<uint32_t>());
                    }
                }

                if (lora_config["power_levels"]) {
                    auto power_config = lora_config["power_levels"];
                    // TODO: Parse power level configuration
                }
            }

            log_communication_event("Configuration loaded successfully");
            return true;

        } catch (const YAML::Exception& e) {
            log_communication_event("Failed to load config: " + std::string(e.what()), "ERROR");
            return false;
        }
    }

    bool CommunicationManager::initialize_lora_radio() {
        // TODO: Initialize actual LoRa hardware
        // This would involve:
        // 1. Initialize SPI communication
        // 2. Configure LoRa chip registers
        // 3. Set frequency, power, bandwidth, etc.
        // 4. Enable interrupts for RX/TX complete

        log_communication_event("LoRa radio initialized (stub)");
        return true;
    }

    bool CommunicationManager::initialize_elrs_modules() {
        // TODO: Initialize ELRS modules
        log_communication_event("ELRS modules initialized (stub)");
        return true;
    }

// Mesh networking stubs
    bool CommunicationManager::clean_stale_mesh_entries() {
        std::lock_guard<std::mutex> lock(mesh_mutex_);

        auto now = std::chrono::steady_clock::now();
        auto timeout = std::chrono::seconds(30);

        auto it = mesh_neighbors_.begin();
        while (it != mesh_neighbors_.end()) {
            if (now - it->second.last_seen > timeout) {
                log_communication_event("Removing stale mesh node", std::to_string(it->first));
                it = mesh_neighbors_.erase(it);
            } else {
                ++it;
            }
        }

        return true;
    }

    bool CommunicationManager::discover_mesh_neighbors() {
        // TODO: Implement mesh neighbor discovery
        return true;
    }

    bool CommunicationManager::update_mesh_topology() {
        // TODO: Implement mesh topology updates
        return true;
    }

    bool CommunicationManager::measure_channel_quality() {
        // TODO: Implement actual RSSI measurement
        // Simulate RSSI changes
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::normal_distribution<> rssi_dist(-75.0, 10.0);

        int8_t new_rssi = static_cast<int8_t>(std::clamp(rssi_dist(gen), -120.0, -30.0));
        current_rssi_.store(new_rssi);

        // Calculate packet loss based on RSSI
        double loss_rate = std::max(0.0, std::min(1.0, (-50.0 - new_rssi) / 40.0));
        packet_loss_rate_.store(loss_rate);

        return true;
    }

    uint32_t CommunicationManager::get_network_timestamp() const {
        auto now = std::chrono::steady_clock::now();
        auto duration = now.time_since_epoch();
        return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() + network_time_offset_;
    }

// Logging methods
    void CommunicationManager::log_communication_event(const std::string& event, const std::string& details) {
        std::cout << "[COMM] Drone" << drone_id_ << " " << event;
        if (!details.empty()) {
            std::cout << " (" << details << ")";
        }
        std::cout << std::endl;
    }

    void CommunicationManager::log_message_transmission(const SwarmMessage& message, bool success) {
        std::cout << "[COMM] " << (success ? "TX OK" : "TX FAIL")
                  << " Type:" << static_cast<int>(message.type)
                  << " To:" << message.destination_id
                  << " Seq:" << message.sequence_number << std::endl;
    }

} // namespace SwarmControl//
// Created by yv on 22.09.2025.
//
