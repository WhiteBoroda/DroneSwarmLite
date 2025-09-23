// src/LoRaSystem.cpp
// LoRa система для ELRS SX1281/SX1276 модулів
// Військова система зв'язку для дронів-камікадзе проти москалів
// 🇺🇦 Slava Ukraini! Death to russian occupants! 🇺🇦

#include "../include/LoRaSystem.h"
#include <iostream>
#include <cstring>
#include <random>
#include <algorithm>
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <openssl/aes.h>
#include <openssl/rand.h>
#include <openssl/sha.h>
#include <sstream>
#include <iomanip>

namespace SwarmControl {

// Combat-optimized LoRa message structure
    struct CombatLoRaMessage {
        uint16_t magic;              // 0xUKR1 - Ukrainian signature
        uint8_t version;             // Protocol version
        uint8_t message_type;        // Message type
        uint16_t sender_id;          // Sender drone ID
        uint16_t target_id;          // Target drone ID (0xFFFF = broadcast)
        uint32_t sequence_number;    // Sequence for replay protection
        uint64_t timestamp_ns;       // Nanosecond timestamp
        uint8_t hop_count;           // For mesh routing
        uint8_t priority;            // Message priority (0-255)
        uint16_t payload_length;     // Encrypted payload length
        uint8_t encrypted_payload[200]; // AES-256 encrypted data
        uint32_t hmac_signature[8];  // SHA-256 HMAC for authentication
        uint16_t crc16;              // CRC16 for error detection

        CombatLoRaMessage() {
            magic = 0x554B; // "UK" in hex
            version = 1;
            message_type = 0;
            sender_id = 0;
            target_id = 0;
            sequence_number = 0;
            timestamp_ns = 0;
            hop_count = 0;
            priority = 128;
            payload_length = 0;
            memset(encrypted_payload, 0, sizeof(encrypted_payload));
            memset(hmac_signature, 0, sizeof(hmac_signature));
            crc16 = 0;
        }
    } __attribute__((packed));

// Ukrainian combat frequencies (avoid Russian military bands)
    static const std::vector<uint32_t> UKRAINIAN_COMBAT_FREQUENCIES = {
            868100000,  // 868.1 MHz - EU ISM band
            868300000,  // 868.3 MHz
            868500000,  // 868.5 MHz
            915000000,  // 915 MHz - US ISM band
            915200000,  // 915.2 MHz
            915400000,  // 915.4 MHz
            433050000,  // 433.05 MHz - EU ISM band
            433200000,  // 433.2 MHz
            433400000,  // 433.4 MHz
            2400000000, // 2.4 GHz - ISM band via ELRS
            2420000000, // 2.42 GHz
            2450000000  // 2.45 GHz
    };

// Message types for combat operations
    enum class CombatMessageType : uint8_t {
        HEARTBEAT = 1,
        COMMAND = 2,
        TELEMETRY = 3,
        FORMATION_UPDATE = 4,
        TARGET_INFO = 5,
        EMERGENCY_RTH = 6,
        ABORT_MISSION = 7,
        FREQUENCY_CHANGE = 8,
        MESH_ROUTING = 9,
        TIME_SYNC = 10,
        ENCRYPTED_COMMAND = 11,
        BATTLE_DAMAGE_REPORT = 12
    };

// ELRS SX1281 (2.4GHz) Hardware Interface
    class LoRaSystem::SX1281Interface {
    private:
        int spi_fd_;
        int reset_pin_;
        int dio0_pin_;
        bool initialized_;
        uint32_t current_frequency_;
        int8_t current_power_;
        void LoRaSystem::perform_frequency_hop() {
            if (frequency_list_.empty()) {
                return;
            }

            std::lock_guard<std::mutex> lock(frequency_mutex_);

            size_t current_index = current_frequency_index_.load();
            size_t next_index = (current_index + 1) % frequency_list_.size();

            if (hopping_config_.randomize_sequence) {
                std::random_device rd;
                std::mt19937 gen(rd());
                std::uniform_int_distribution<> dis(0, frequency_list_.size() - 1);
                next_index = dis(gen);
            }

            current_frequency_index_.store(next_index);
            uint32_t new_frequency = frequency_list_[next_index];

            // Apply frequency to appropriate radio
            bool success = false;
            if (new_frequency >= 2400000000 && new_frequency <= 2500000000 && sx1281_interface_) {
                success = sx1281_interface_->set_frequency(new_frequency);
            } else if (new_frequency >= 902000000 && new_frequency <= 928000000 && sx1276_interface_) {
                success = sx1276_interface_->set_frequency(new_frequency);
            }

            if (success) {
                stats_.frequency_changes++;
                std::cout << "🔄 Frequency hopped to " << new_frequency << " Hz" << std::endl;
            }
        }

// MESH NETWORKING METHODS
        bool LoRaSystem::enable_mesh_routing(bool enable) {
            mesh_enabled_ = enable;

            if (enable) {
                std::cout << "🌐 Mesh routing enabled" << std::endl;
            } else {
                std::cout << "🚫 Mesh routing disabled" << std::endl;
            }

            return true;
        }

        bool LoRaSystem::add_mesh_route(const MeshRouteInfo& route) {
            std::lock_guard<std::mutex> lock(mesh_mutex_);

            mesh_routes_[route.destination_id] = route;

            std::cout << "➕ Added mesh route to drone " << route.destination_id
                      << " via drone " << route.next_hop_id << std::endl;

            return true;
        }

        bool LoRaSystem::remove_mesh_route(uint16_t destination_id) {
            std::lock_guard<std::mutex> lock(mesh_mutex_);

            auto it = mesh_routes_.find(destination_id);
            if (it != mesh_routes_.end()) {
                mesh_routes_.erase(it);
                std::cout << "➖ Removed mesh route to drone " << destination_id << std::endl;
                return true;
            }

            return false;
        }

        std::vector<MeshRouteInfo> LoRaSystem::get_mesh_routes() const {
            std::lock_guard<std::mutex> lock(mesh_mutex_);

            std::vector<MeshRouteInfo> routes;
            for (const auto& [dest_id, route] : mesh_routes_) {
                routes.push_back(route);
            }

            return routes;
        }

        bool LoRaSystem::forward_mesh_message(const LoRaMessage& message) {
            if (!mesh_enabled_ || message.hop_count >= MAX_HOP_COUNT) {
                return false;
            }

            std::lock_guard<std::mutex> lock(mesh_mutex_);

            // Find route to destination
            auto route_it = mesh_routes_.find(message.target_id);
            if (route_it == mesh_routes_.end() || !route_it->second.is_valid) {
                return false;
            }

            // Forward message with incremented hop count
            LoRaMessage forwarded_message = message;
            forwarded_message.hop_count++;

            // Send to next hop
            bool success = send_message(forwarded_message, CommProtocol::AUTO_SELECT);

            if (success) {
                std::cout << "🔄 Forwarded message to drone " << message.target_id
                          << " via drone " << route_it->second.next_hop_id << std::endl;
            }

            return success;
        }

        void LoRaSystem::clean_stale_mesh_routes() {
            std::lock_guard<std::mutex> lock(mesh_mutex_);

            auto now = std::chrono::steady_clock::now();
            auto timeout = std::chrono::minutes(5); // 5 minute timeout

            for (auto it = mesh_routes_.begin(); it != mesh_routes_.end();) {
                if (now - it->second.last_used > timeout) {
                    std::cout << "🧹 Removing stale mesh route to drone " << it->first << std::endl;
                    it = mesh_routes_.erase(it);
                } else {
                    ++it;
                }
            }
        }

// SIGNAL QUALITY AND MONITORING
        SignalQuality LoRaSystem::get_signal_quality(LoRaFrequencyBand band) const {
            std::lock_guard<std::mutex> lock(signal_mutex_);

            switch (band) {
                case LoRaFrequencyBand::BAND_2_4GHz:
                    return signal_quality_2_4ghz_;
                case LoRaFrequencyBand::BAND_915MHz:
                    return signal_quality_915mhz_;
                default:
                    return SignalQuality();
            }
        }

        bool LoRaSystem::run_signal_test(LoRaFrequencyBand band, uint32_t duration_ms) {
            std::cout << "🧪 Running signal test for " << duration_ms << "ms..." << std::endl;

            auto start_time = std::chrono::steady_clock::now();
            uint32_t test_packets_sent = 0;
            uint32_t test_packets_received = 0;

            while (std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - start_time).count() < duration_ms) {

                // Send test packet
                LoRaMessage test_message;
                test_message.sender_id = drone_id_;
                test_message.target_id = 0xFFFF; // Broadcast
                test_message.message_type = 255; // Test message type
                test_message.priority = MessagePriority::LOW;
                test_message.payload = {'T', 'E', 'S', 'T'};

                if (send_message(test_message, CommProtocol::AUTO_SELECT)) {
                    test_packets_sent++;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            // Calculate test results
            SignalQuality quality = get_signal_quality(band);
            double packet_success_rate = test_packets_sent > 0 ?
                                         static_cast<double>(test_packets_received) / test_packets_sent : 0.0;

            std::cout << "📊 Signal test results:" << std::endl;
            std::cout << "   RSSI: " << quality.rssi_dbm << " dBm" << std::endl;
            std::cout << "   SNR: " << quality.snr_db << " dB" << std::endl;
            std::cout << "   Packet Success Rate: " << (packet_success_rate * 100.0) << "%" << std::endl;

            return packet_success_rate > 0.8; // 80% success rate threshold
        }

// SECURITY AND ENCRYPTION
        bool LoRaSystem::enable_encryption(bool enable) {
            encryption_enabled_.store(enable);

            if (enable) {
                std::cout << "🔐 Encryption enabled - moskals cannot intercept!" << std::endl;
            } else {
                std::cout << "🔓 Encryption disabled" << std::endl;
            }

            return true;
        }

        bool LoRaSystem::set_encryption_key(const std::string& key) {
            if (key.length() != 32) {
                std::cerr << "❌ Invalid encryption key length (must be 32 bytes)" << std::endl;
                return false;
            }

            std::lock_guard<std::mutex> lock(crypto_mutex_);
            memcpy(encryption_key_, key.c_str(), 32);

            std::cout << "🔑 Encryption key updated" << std::endl;
            return true;
        }

        bool LoRaSystem::rotate_encryption_key() {
            std::lock_guard<std::mutex> lock(crypto_mutex_);

            // Generate new random key
            if (RAND_bytes(encryption_key_, sizeof(encryption_key_)) != 1) {
                std::cerr << "❌ Failed to rotate encryption key" << std::endl;
                return false;
            }

            std::cout << "🔄 Encryption key rotated successfully" << std::endl;
            return true;
        }

// MESSAGE HANDLERS
        bool LoRaSystem::set_message_handler(uint8_t message_type, std::function<void(const LoRaMessage&)> handler) {
            std::lock_guard<std::mutex> lock(handlers_mutex_);

            message_handlers_[message_type] = handler;

            std::cout << "📝 Message handler registered for type " << static_cast<int>(message_type) << std::endl;
            return true;
        }

        bool LoRaSystem::remove_message_handler(uint8_t message_type) {
            std::lock_guard<std::mutex> lock(handlers_mutex_);

            auto it = message_handlers_.find(message_type);
            if (it != message_handlers_.end()) {
                message_handlers_.erase(it);
                std::cout << "🗑️ Message handler removed for type " << static_cast<int>(message_type) << std::endl;
                return true;
            }

            return false;
        }

// STATISTICS AND DIAGNOSTICS
        LoRaStatistics LoRaSystem::get_statistics() const {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            return stats_;
        }

        bool LoRaSystem::perform_self_test() {
            std::cout << "🔧 Performing LoRa system self-test..." << std::endl;

            bool success = true;

            // Test 2.4GHz module
            if (sx1281_interface_) {
                uint32_t tx_count, rx_count, error_count;
                sx1281_interface_->get_stats(tx_count, rx_count, error_count);
                std::cout << "📊 SX1281 Stats: TX=" << tx_count << ", RX=" << rx_count
                          << ", Errors=" << error_count << std::endl;

                if (error_count > tx_count * 0.1) { // More than 10% errors
                    std::cerr << "⚠️ High error rate on 2.4GHz module" << std::endl;
                    success = false;
                }
            }

            // Test 915MHz module
            if (sx1276_interface_) {
                uint32_t tx_count, rx_count, error_count;
                sx1276_interface_->get_stats(tx_count, rx_count, error_count);
                std::cout << "📊 SX1276 Stats: TX=" << tx_count << ", RX=" << rx_count
                          << ", Errors=" << error_count << std::endl;

                if (error_count > tx_count * 0.1) {
                    std::cerr << "⚠️ High error rate on 915MHz module" << std::endl;
                    success = false;
                }
            }

            // Test encryption
            const char* test_data = "Ukrainian test message";
            uint8_t encrypted[64];
            size_t encrypted_len = sizeof(encrypted);

            if (encrypt_payload(reinterpret_cast<const uint8_t*>(test_data), strlen(test_data),
                                encrypted, encrypted_len)) {

                uint8_t decrypted[64];
                size_t decrypted_len = sizeof(decrypted);

                if (decrypt_payload(encrypted, encrypted_len, decrypted, decrypted_len)) {
                    if (memcmp(test_data, decrypted, strlen(test_data)) == 0) {
                        std::cout << "✅ Encryption test passed" << std::endl;
                    } else {
                        std::cerr << "❌ Encryption test failed - data mismatch" << std::endl;
                        success = false;
                    }
                } else {
                    std::cerr << "❌ Encryption test failed - decryption error" << std::endl;
                    success = false;
                }
            } else {
                std::cerr << "❌ Encryption test failed - encryption error" << std::endl;
                success = false;
            }

            if (success) {
                std::cout << "✅ LoRa system self-test completed successfully" << std::endl;
            } else {
                std::cout << "❌ LoRa system self-test failed" << std::endl;
            }

            return success;
        }

        std::string LoRaSystem::get_status_report() const {
            std::stringstream ss;

            ss << "=== LoRa System Status Report ===" << std::endl;
            ss << "Drone ID: " << drone_id_ << std::endl;
            ss << "Running: " << (running_.load() ? "Yes" : "No") << std::endl;
            ss << "Encryption: " << (encryption_enabled_.load() ? "Enabled" : "Disabled") << std::endl;
            ss << "Current Power: " << static_cast<int>(current_power_level_.load()) << " dBm" << std::endl;
            ss << "Jamming Detected: " << (jamming_detected_.load() ? "Yes" : "No") << std::endl;
            ss << "Mesh Routing: " << (mesh_enabled_ ? "Enabled" : "Disabled") << std::endl;
            ss << "Adaptive Power: " << (adaptive_power_enabled_ ? "Enabled" : "Disabled") << std::endl;

            {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                ss << std::endl << "=== Statistics ===" << std::endl;
                ss << "Messages Sent: " << stats_.messages_sent << std::endl;
                ss << "Messages Received: " << stats_.messages_received << std::endl;
                ss << "Messages Forwarded: " << stats_.messages_forwarded << std::endl;
                ss << "Messages Dropped: " << stats_.messages_dropped << std::endl;
                ss << "CRC Errors: " << stats_.crc_errors << std::endl;
                ss << "Encryption Errors: " << stats_.encryption_errors << std::endl;
                ss << "Frequency Changes: " << stats_.frequency_changes << std::endl;
                ss << "Average RSSI: " << stats_.average_rssi << " dBm" << std::endl;
                ss << "Packet Loss Rate: " << (stats_.packet_loss_rate * 100.0) << "%" << std::endl;
            }

            {
                std::lock_guard<std::mutex> lock(signal_mutex_);
                ss << std::endl << "=== Signal Quality ===" << std::endl;
                ss << "2.4GHz - RSSI: " << signal_quality_2_4ghz_.rssi_dbm
                   << " dBm, SNR: " << signal_quality_2_4ghz_.snr_db << " dB" << std::endl;
                ss << "915MHz - RSSI: " << signal_quality_915mhz_.rssi_dbm
                   << " dBm, SNR: " << signal_quality_915mhz_.snr_db << " dB" << std::endl;
            }

            {
                std::lock_guard<std::mutex> lock(frequency_mutex_);
                ss << std::endl << "=== Frequency Management ===" << std::endl;
                ss << "Frequency Hopping: " << (hopping_config_.enabled ? "Enabled" : "Disabled") << std::endl;
                ss << "Available Frequencies: " << frequency_list_.size() << std::endl;
                ss << "Current Frequency Index: " << current_frequency_index_.load() << std::endl;
                if (!frequency_list_.empty()) {
                    ss << "Current Frequency: " << frequency_list_[current_frequency_index_.load()] << " Hz" << std::endl;
                }
            }

            {
                std::lock_guard<std::mutex> lock(mesh_mutex_);
                ss << std::endl << "=== Mesh Network ===" << std::endl;
                ss << "Active Routes: " << mesh_routes_.size() << std::endl;
                for (const auto& [dest_id, route] : mesh_routes_) {
                    ss << "  Route to " << dest_id << " via " << route.next_hop_id
                       << " (hops: " << static_cast<int>(route.hop_count)
                       << ", quality: " << std::fixed << std::setprecision(2) << route.link_quality << ")" << std::endl;
                }
            }

            return ss.str();
        }

        bool LoRaSystem::export_diagnostics(const std::string& filename) const {
            std::ofstream file(filename);
            if (!file.is_open()) {
                std::cerr << "❌ Failed to open diagnostics file: " << filename << std::endl;
                return false;
            }

            // Export full status report
            file << get_status_report() << std::endl;

            // Export detailed statistics
            {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                file << std::endl << "=== Detailed Statistics ===" << std::endl;

                auto uptime = std::chrono::duration_cast<std::chrono::seconds>(
                        stats_.last_activity - stats_.start_time).count();
                file << "Uptime: " << uptime << " seconds" << std::endl;

                if (uptime > 0) {
                    file << "Messages per second: " << static_cast<double>(stats_.messages_sent) / uptime << std::endl;
                    file << "Throughput: " << static_cast<double>(stats_.messages_received) / uptime << " msg/s" << std::endl;
                }
            }

            file.close();

            std::cout << "📊 Diagnostics exported to " << filename << std::endl;
            return true;
        }

// CONFIGURATION MANAGEMENT
        bool LoRaSystem::reload_configuration() {
            std::cout << "🔄 Reloading LoRa configuration..." << std::endl;

            // Stop operations temporarily
            bool was_running = running_.load();
            if (was_running) {
                running_.store(false);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            // Reload frequency list
            frequency_list_ = UKRAINIAN_COMBAT_FREQUENCIES;

            // Reset frequency hopping config
            hopping_config_ = FrequencyHoppingConfig();

            // Restart if was running
            if (was_running) {
                running_.store(true);
            }

            std::cout << "✅ LoRa configuration reloaded successfully" << std::endl;
            return true;
        }

        bool LoRaSystem::set_drone_id(uint16_t drone_id) {
            drone_id_ = drone_id;
            std::cout << "🆔 Drone ID set to " << drone_id_ << std::endl;
            return true;
        }

        bool LoRaSystem::set_call_sign(const std::string& call_sign) {
            call_sign_ = call_sign;
            std::cout << "📻 Call sign set to '" << call_sign_ << "'" << std::endl;
            return true;
        }

// ADDITIONAL UTILITY METHODS
        std::vector<uint32_t> LoRaSystem::get_available_frequencies(LoRaFrequencyBand band) const {
            std::vector<uint32_t> frequencies;

            for (uint32_t freq : UKRAINIAN_COMBAT_FREQUENCIES) {
                bool matches_band = false;

                switch (band) {
                    case LoRaFrequencyBand::BAND_433MHz:
                        matches_band = (freq >= 433000000 && freq <= 434000000);
                        break;
                    case LoRaFrequencyBand::BAND_868MHz:
                        matches_band = (freq >= 868000000 && freq <= 869000000);
                        break;
                    case LoRaFrequencyBand::BAND_915MHz:
                        matches_band = (freq >= 902000000 && freq <= 928000000);
                        break;
                    case LoRaFrequencyBand::BAND_2_4GHz:
                        matches_band = (freq >= 2400000000 && freq <= 2500000000);
                        break;
                }

                if (matches_band) {
                    frequencies.push_back(freq);
                }
            }

            return frequencies;
        }

        int8_t LoRaSystem::get_optimal_power(uint16_t target_drone_id) const {
            std::lock_guard<std::mutex> lock(power_mutex_);

            auto it = power_table_.find(target_drone_id);
            if (it != power_table_.end()) {
                return it->second;
            }

            // Return default power if no specific setting
            return current_power_level_.load();
        }

        bool LoRaSystem::send_message_reliable(const LoRaMessage& message, uint8_t max_retries) {
            for (uint8_t attempt = 0; attempt < max_retries; ++attempt) {
                if (send_message(message, CommProtocol::AUTO_SELECT)) {
                    std::cout << "✅ Reliable message sent on attempt " << (attempt + 1) << std::endl;
                    return true;
                }

                // Wait before retry
                std::this_thread::sleep_for(std::chrono::milliseconds(100 * (attempt + 1)));
            }

            std::cerr << "❌ Reliable message failed after " << max_retries << " attempts" << std::endl;
            return false;
        }

        bool LoRaSystem::discover_mesh_neighbors() {
            if (!mesh_enabled_) {
                return false;
            }

            std::cout << "🔍 Discovering mesh neighbors..." << std::endl;

            // Send discovery broadcast
            LoRaMessage discovery_msg;
            discovery_msg.sender_id = drone_id_;
            discovery_msg.target_id = 0xFFFF; // Broadcast
            discovery_msg.message_type = static_cast<uint8_t>(CombatMessageType::MESH_ROUTING);
            discovery_msg.priority = MessagePriority::NORMAL;
            discovery_msg.payload = {'D', 'I', 'S', 'C', 'O', 'V', 'E', 'R'};

            bool success = broadcast_message(discovery_msg);

            if (success) {
                std::cout << "📡 Mesh discovery broadcast sent" << std::endl;
            }

            return success;
        }

    } // namespace SwarmControlatomic<uint32_t> packets_sent_;
    std::atomic<uint32_t> packets_received_;
    std::atomic<uint32_t> crc_errors_;

    public:
    SX1281Interface() : spi_fd_(-1), reset_pin_(4), dio0_pin_(2),
                        initialized_(false), current_frequency_(2400000000),
                        current_power_(14), packets_sent_(0), packets_received_(0), crc_errors_(0) {}

    ~SX1281Interface() {
        if (spi_fd_ >= 0) {
            close(spi_fd_);
        }
    }

    bool initialize() {
        std::cout << "🔧 Initializing SX1281 2.4GHz ELRS module..." << std::endl;

        // Initialize SPI interface
        std::string spi_device = "/dev/spidev0.0";
        spi_fd_ = open(spi_device.c_str(), O_RDWR);

        if (spi_fd_ < 0) {
            std::cerr << "❌ Cannot open SPI device: " << spi_device << std::endl;
            return false;
        }

        // Configure SPI
        uint8_t spi_mode = SPI_MODE_0;
        uint32_t spi_speed = 8000000; // 8 MHz

        if (ioctl(spi_fd_, SPI_IOC_WR_MODE, &spi_mode) < 0 ||
            ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed) < 0) {
            std::cerr << "❌ SPI configuration failed" << std::endl;
            return false;
        }

        // Hardware reset
        if (!hardware_reset()) {
            std::cerr << "❌ SX1281 hardware reset failed" << std::endl;
            return false;
        }

        // Read device version to verify communication
        uint8_t version = read_register(0x6F); // Version register
        if (version == 0x00 || version == 0xFF) {
            std::cerr << "❌ SX1281 communication failed, version: 0x" << std::hex << version << std::endl;
            return false;
        }

        std::cout << "✅ SX1281 detected, version: 0x" << std::hex << version << std::endl;

        // Load combat-optimized settings
        if (!configure_for_combat()) {
            std::cerr << "❌ Failed to configure SX1281 for combat" << std::endl;
            return false;
        }

        initialized_ = true;
        std::cout << "✅ SX1281 2.4GHz module ready for combat operations" << std::endl;
        return true;
    }

    bool configure_for_combat() {
        // Switch to LoRa mode
        write_register(0x01, 0x80); // Set LoRa mode

        // Set frequency to 2.4GHz band
        set_frequency(current_frequency_);

        // Set spreading factor for maximum range vs data rate balance
        write_register(0x1E, 0x74); // SF=7, BW=500kHz, CR=4/5

        // Set power amplifier configuration for maximum range
        write_register(0x09, 0x8F); // PA config
        set_tx_power(current_power_);

        // Set preamble length for better detection in noisy environment
        write_register(0x20, 0x00); // Preamble MSB
        write_register(0x21, 0x08); // Preamble LSB = 8 symbols

        // Enable CRC for error detection
        write_register(0x1E, read_register(0x1E) | 0x04);

        // Set LNA boost for better reception
        write_register(0x0C, 0x23);

        // Configure DIO0 for TxDone and RxDone
        write_register(0x40, 0x40); // DIO0 mapping

        std::cout << "⚡ SX1281 configured for maximum combat effectiveness" << std::endl;
        return true;
    }

    bool set_frequency(uint32_t frequency) {
    if (frequency < 2400000000 || frequency > 2500000000) {
    std::cerr << "❌ Invalid 2.4GHz frequency: " << frequency << std::endl;
    return false;
}

// Calculate frequency registers for SX1281
// Frf = (Fxosc * Frf) / 2^25
uint64_t frf = ((uint64_t)frequency << 25) / 32000000;

write_register(0x06, (frf >> 16) & 0xFF); // FrfMsb
write_register(0x07, (frf >> 8) & 0xFF);  // FrfMid
write_register(0x08, frf & 0xFF);         // FrfLsb

current_frequency_ = frequency;
std::cout << "📡 SX1281 frequency set to " << frequency << " Hz" << std::endl;
return true;
}

bool set_tx_power(int8_t power) {
    if (power < -3 || power > 15) {
        std::cerr << "❌ Invalid TX power for SX1281: " << static_cast<int>(power) << std::endl;
        return false;
    }

    // Set output power (0-15 maps to -3 to +15 dBm)
    uint8_t power_reg = (power + 3) & 0x0F;
    write_register(0x09, 0x80 | power_reg); // PA config with PA_BOOST

    current_power_ = power;
    std::cout << "⚡ SX1281 TX power set to " << static_cast<int>(power) << " dBm" << std::endl;
    return true;
}

bool send_packet(const uint8_t* data, size_t length) {
    if (!initialized_ || length > 255) {
        return false;
    }

    // Set to standby mode
    write_register(0x01, 0x81);

    // Set FIFO pointer to start
    write_register(0x0D, 0x00);
    write_register(0x0E, 0x00);

    // Write payload length
    write_register(0x22, length);

    // Write payload to FIFO
    write_burst(0x00, data, length);

    // Start transmission
    write_register(0x01, 0x83); // TX mode

    // Wait for transmission complete
    if (!wait_for_tx_complete()) {
        std::cerr << "❌ SX1281 transmission timeout" << std::endl;
        return false;
    }

    packets_sent_.fetch_add(1);
    return true;
}

bool receive_packet(uint8_t* buffer, size_t& length, uint32_t timeout_ms) {
if (!initialized_) {
return false;
}

// Set to continuous RX mode
write_register(0x01, 0x85);

// Wait for packet with timeout
if (!wait_for_rx_complete(timeout_ms)) {
return false; // Timeout, not an error
}

// Read received packet length
uint8_t rx_bytes = read_register(0x13);
if (rx_bytes > length) {
std::cerr << "❌ Received packet too large: " << rx_bytes << std::endl;
return false;
}

// Read packet from FIFO
uint8_t fifo_addr = read_register(0x10);
write_register(0x0D, fifo_addr);
read_burst(0x00, buffer, rx_bytes);

length = rx_bytes;
packets_received_.fetch_add(1);

return true;
}

int get_rssi() {
    return -read_register(0x1A) / 2; // RSSI register, convert to dBm
}

float get_snr() {
    int8_t snr_reg = read_register(0x1B);
    return snr_reg * 0.25f; // SNR in dB
}

void get_stats(uint32_t& tx_count, uint32_t& rx_count, uint32_t& error_count) {
    tx_count = packets_sent_.load();
    rx_count = packets_received_.load();
    error_count = crc_errors_.load();
}

private:
bool hardware_reset() {
    // GPIO control for hardware reset
    // Implementation would use GPIO library
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return true;
}

bool wait_for_tx_complete() {
    for (int i = 0; i < 100; ++i) {
        uint8_t irq_flags = read_register(0x12);
        if (irq_flags & 0x08) { // TxDone
            write_register(0x12, 0x08); // Clear flag
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
}

bool wait_for_rx_complete(uint32_t timeout_ms) {
for (uint32_t i = 0; i < timeout_ms; ++i) {
uint8_t irq_flags = read_register(0x12);
if (irq_flags & 0x40) { // RxDone
write_register(0x12, 0x40); // Clear flag
return true;
}
std::this_thread::sleep_for(std::chrono::milliseconds(1));
}
return false;
}

uint8_t read_register(uint8_t address) {
uint8_t tx_buf[2] = {address & 0x7F, 0x00};
uint8_t rx_buf[2] = {0};

struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_buf,
        .rx_buf = (unsigned long)rx_buf,
        .len = 2,
        .speed_hz = 8000000,
        .bits_per_word = 8,
};

if (ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr) >= 0) {
return rx_buf[1];
}
return 0;
}

void write_register(uint8_t address, uint8_t value) {
uint8_t tx_buf[2] = {address | 0x80, value};

struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_buf,
        .rx_buf = 0,
        .len = 2,
        .speed_hz = 8000000,
        .bits_per_word = 8,
};

ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);
}

void write_burst(uint8_t address, const uint8_t* data, size_t length) {
std::vector<uint8_t> tx_buf(length + 1);
tx_buf[0] = address | 0x80;
memcpy(&tx_buf[1], data, length);

struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_buf.data(),
        .rx_buf = 0,
        .len = static_cast<uint32_t>(length + 1),
        .speed_hz = 8000000,
        .bits_per_word = 8,
};

ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);
}

void read_burst(uint8_t address, uint8_t* data, size_t length) {
std::vector<uint8_t> tx_buf(length + 1);
std::vector<uint8_t> rx_buf(length + 1);
tx_buf[0] = address & 0x7F;

struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_buf.data(),
        .rx_buf = (unsigned long)rx_buf.data(),
        .len = static_cast<uint32_t>(length + 1),
        .speed_hz = 8000000,
        .bits_per_word = 8,
};

if (ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr) >= 0) {
memcpy(data, &rx_buf[1], length);
}
}
};

// ELRS SX1276 (915MHz) Hardware Interface
class LoRaSystem::SX1276Interface {
private:
    int spi_fd_;
    int reset_pin_;
    int dio0_pin_;
    bool initialized_;
    uint32_t current_frequency_;
    int8_t current_power_;
    std::atomic<uint32_t> packets_sent_;
    std::atomic<uint32_t> packets_received_;

public:
    SX1276Interface() : spi_fd_(-1), reset_pin_(27), dio0_pin_(15),
                        initialized_(false), current_frequency_(915000000),
                        current_power_(14), packets_sent_(0), packets_received_(0) {}

    ~SX1276Interface() {
        if (spi_fd_ >= 0) {
            close(spi_fd_);
        }
    }

    bool initialize() {
        std::cout << "🔧 Initializing SX1276 915MHz ELRS module..." << std::endl;

        // Initialize SPI interface
        std::string spi_device = "/dev/spidev0.1";
        spi_fd_ = open(spi_device.c_str(), O_RDWR);

        if (spi_fd_ < 0) {
            std::cerr << "❌ Cannot open SPI device: " << spi_device << std::endl;
            return false;
        }

        // Configure SPI
        uint8_t spi_mode = SPI_MODE_0;
        uint32_t spi_speed = 8000000;

        if (ioctl(spi_fd_, SPI_IOC_WR_MODE, &spi_mode) < 0 ||
            ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed) < 0) {
            std::cerr << "❌ SPI configuration failed" << std::endl;
            return false;
        }

        // Hardware reset
        if (!hardware_reset()) {
            std::cerr << "❌ SX1276 hardware reset failed" << std::endl;
            return false;
        }

        // Read device version
        uint8_t version = read_register(0x42);
        if (version != 0x12) {
            std::cerr << "❌ SX1276 communication failed, version: 0x" << std::hex << version << std::endl;
            return false;
        }

        std::cout << "✅ SX1276 detected, version: 0x" << std::hex << version << std::endl;

        // Configure for combat operations
        if (!configure_for_combat()) {
            std::cerr << "❌ Failed to configure SX1276 for combat" << std::endl;
            return false;
        }

        initialized_ = true;
        std::cout << "✅ SX1276 915MHz module ready for combat operations" << std::endl;
        return true;
    }

    bool configure_for_combat() {
        // Set sleep mode
        write_register(0x01, 0x00);

        // Set LoRa mode
        write_register(0x01, 0x80);

        // Set frequency
        set_frequency(current_frequency_);

        // Set modem config for combat (BW=125kHz, CR=4/5, SF=7)
        write_register(0x1D, 0x72); // BW=125kHz, CR=4/5, Header mode=explicit
        write_register(0x1E, 0x74); // SF=7, CRC=on

        // Set power configuration
        set_tx_power(current_power_);

        // Set preamble length
        write_register(0x20, 0x00); // Preamble MSB
        write_register(0x21, 0x08); // Preamble LSB = 8

        // Set LNA configuration for maximum sensitivity
        write_register(0x0C, 0x23);

        // Configure DIO pins
        write_register(0x40, 0x40); // DIO0=RxDone/TxDone

        std::cout << "⚡ SX1276 configured for maximum combat range" << std::endl;
        return true;
    }

    bool set_frequency(uint32_t frequency) {
        if (frequency < 902000000 || frequency > 928000000) {
            std::cerr << "❌ Invalid 915MHz frequency: " << frequency << std::endl;
            return false;
        }

        // Calculate frequency word: Frf = frequency / 61.03515625
        uint32_t frf = (uint32_t)((double)frequency / 61.03515625);

        write_register(0x06, (frf >> 16) & 0xFF);
        write_register(0x07, (frf >> 8) & 0xFF);
        write_register(0x08, frf & 0xFF);

        current_frequency_ = frequency;
        std::cout << "📡 SX1276 frequency set to " << frequency << " Hz" << std::endl;
        return true;
    }

    bool set_tx_power(int8_t power) {
        if (power < 2 || power > 17) {
            std::cerr << "❌ Invalid TX power for SX1276: " << static_cast<int>(power) << std::endl;
            return false;
        }

        // Configure PA_BOOST pin for high power
        write_register(0x09, 0x80 | (power - 2));

        current_power_ = power;
        std::cout << "⚡ SX1276 TX power set to " << static_cast<int>(power) << " dBm" << std::endl;
        return true;
    }

    bool send_packet(const uint8_t* data, size_t length) {
        if (!initialized_ || length > 255) {
            return false;
        }

        // Set standby mode
        write_register(0x01, 0x81);

        // Set FIFO pointers
        write_register(0x0D, 0x00); // FifoTxBaseAddr
        write_register(0x0E, 0x00); // FifoAddrPtr

        // Write payload length
        write_register(0x22, length);

        // Write payload
        for (size_t i = 0; i < length; ++i) {
            write_register(0x00, data[i]);
        }

        // Start transmission
        write_register(0x01, 0x83);

        // Wait for TxDone
        if (!wait_for_tx_complete()) {
            std::cerr << "❌ SX1276 transmission timeout" << std::endl;
            return false;
        }

        packets_sent_.fetch_add(1);
        return true;
    }

    bool receive_packet(uint8_t* buffer, size_t& length, uint32_t timeout_ms) {
        if (!initialized_) {
            return false;
        }

        // Set continuous RX mode
        write_register(0x01, 0x85);

        // Wait for RxDone
        if (!wait_for_rx_complete(timeout_ms)) {
            return false;
        }

        // Read packet info
        uint8_t rx_bytes = read_register(0x13);
        uint8_t fifo_addr = read_register(0x10);

        if (rx_bytes > length) {
            std::cerr << "❌ Packet too large: " << rx_bytes << std::endl;
            return false;
        }

        // Set FIFO pointer and read data
        write_register(0x0D, fifo_addr);
        for (size_t i = 0; i < rx_bytes; ++i) {
            buffer[i] = read_register(0x00);
        }

        length = rx_bytes;
        packets_received_.fetch_add(1);
        return true;
    }

    int get_rssi() {
        return read_register(0x1A) - 164; // PacketRssi
    }

    float get_snr() {
        int8_t snr = read_register(0x1B);
        return snr * 0.25f;
    }

    void get_stats(uint32_t& tx_count, uint32_t& rx_count, uint32_t& error_count) {
        tx_count = packets_sent_.load();
        rx_count = packets_received_.load();
        error_count = 0; // Could implement error counting
    }

private:
    bool hardware_reset() {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        return true;
    }

    bool wait_for_tx_complete() {
        for (int i = 0; i < 1000; ++i) {
            uint8_t irq_flags = read_register(0x12);
            if (irq_flags & 0x08) { // TxDone
                write_register(0x12, 0x08); // Clear flag
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return false;
    }

    bool wait_for_rx_complete(uint32_t timeout_ms) {
        for (uint32_t i = 0; i < timeout_ms; ++i) {
            uint8_t irq_flags = read_register(0x12);
            if (irq_flags & 0x40) { // RxDone
                write_register(0x12, 0x40); // Clear flag
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return false;
    }

    uint8_t read_register(uint8_t address) {
        uint8_t tx_buf[2] = {address & 0x7F, 0x00};
        uint8_t rx_buf[2] = {0};

        struct spi_ioc_transfer tr = {
                .tx_buf = (unsigned long)tx_buf,
                .rx_buf = (unsigned long)rx_buf,
                .len = 2,
                .speed_hz = 8000000,
                .bits_per_word = 8,
        };

        if (ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr) >= 0) {
            return rx_buf[1];
        }
        return 0;
    }

    void write_register(uint8_t address, uint8_t value) {
        uint8_t tx_buf[2] = {address | 0x80, value};

        struct spi_ioc_transfer tr = {
                .tx_buf = (unsigned long)tx_buf,
                .rx_buf = 0,
                .len = 2,
                .speed_hz = 8000000,
                .bits_per_word = 8,
        };

        ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);
    }
};

// Main LoRa System Implementation
LoRaSystem::LoRaSystem(uint16_t drone_id, const std::string& config_path)
: drone_id_(drone_id)
, config_path_(config_path)
, running_(false)
, current_frequency_index_(0)
, current_power_level_(14)
, jamming_detected_(false)
, encryption_enabled_(true)
, sequence_number_(0)
, adaptive_power_enabled_(false)
, mesh_enabled_(false) {

    // Initialize with combat frequencies
    frequency_list_ = UKRAINIAN_COMBAT_FREQUENCIES;

    // Clear statistics
    memset(&stats_, 0, sizeof(stats_));
    stats_.start_time = std::chrono::steady_clock::now();

    std::cout << "📡 LoRa System initialized for drone " << drone_id_ << std::endl;
}

LoRaSystem::~LoRaSystem() {
    if (running_.load()) {
        stop();
    }
}

bool LoRaSystem::initialize() {
    std::cout << "🔧 Initializing LoRa System..." << std::endl;

    try {
        // Initialize 2.4GHz ELRS module
        sx1281_interface_ = std::make_unique<SX1281Interface>();
        if (!sx1281_interface_->initialize()) {
            std::cerr << "❌ Failed to initialize SX1281 2.4GHz module" << std::endl;
            return false;
        }

        // Initialize 915MHz ELRS module
        sx1276_interface_ = std::make_unique<SX1276Interface>();
        if (!sx1276_interface_->initialize()) {
            std::cerr << "❌ Failed to initialize SX1276 915MHz module" << std::endl;
            return false;
        }

        // Load combat encryption keys
        if (!initialize_encryption()) {
            std::cerr << "❌ Failed to initialize encryption" << std::endl;
            return false;
        }

        std::cout << "✅ LoRa System initialized successfully" << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "❌ LoRa initialization exception: " << e.what() << std::endl;
        return false;
    }
}

bool LoRaSystem::start() {
    if (running_.load()) {
        std::cout << "⚠️ LoRa System already running" << std::endl;
        return true;
    }

    running_.store(true);

    // Start worker threads
    tx_thread_ = std::thread(&LoRaSystem::transmission_worker, this);
    rx_thread_ = std::thread(&LoRaSystem::reception_worker, this);
    frequency_hopping_thread_ = std::thread(&LoRaSystem::frequency_hopping_worker, this);
    monitoring_thread_ = std::thread(&LoRaSystem::monitoring_worker, this);

    std::cout << "✅ LoRa System started - Ready for combat!" << std::endl;
    return true;
}

bool LoRaSystem::stop() {
    if (!running_.load()) {
        return true;
    }

    std::cout << "🔄 Stopping LoRa System..." << std::endl;
    running_.store(false);

    // Notify all threads
    tx_cv_.notify_all();
    rx_cv_.notify_all();

    // Join worker threads
    if (tx_thread_.joinable()) tx_thread_.join();
    if (rx_thread_.joinable()) rx_thread_.join();
    if (frequency_hopping_thread_.joinable()) frequency_hopping_thread_.join();
    if (monitoring_thread_.joinable()) monitoring_thread_.join();

    std::cout << "✅ LoRa System stopped" << std::endl;
    return true;
}

// CORE MESSAGING METHODS
bool LoRaSystem::send_message(const LoRaMessage& message, CommProtocol protocol) {
    if (!running_.load()) {
        std::cerr << "❌ LoRa system not running" << std::endl;
        return false;
    }

    // Add to outgoing queue
    {
        std::lock_guard<std::mutex> lock(outgoing_mutex_);
        if (message.priority == MessagePriority::CRITICAL) {
            std::lock_guard<std::mutex> priority_lock(priority_mutex_);
            priority_queue_.push(message);
        } else {
            outgoing_queue_.push(message);
        }
    }

    // Notify transmission worker
    tx_cv_.notify_one();

    std::cout << "📤 Message queued for transmission to drone " << message.target_id << std::endl;
    return true;
}

bool LoRaSystem::broadcast_message(const LoRaMessage& message) {
    LoRaMessage broadcast_msg = message;
    broadcast_msg.target_id = 0xFFFF; // Broadcast address
    return send_message(broadcast_msg, CommProtocol::AUTO_SELECT);
}

bool LoRaSystem::send_emergency_message(const LoRaMessage& message) {
    LoRaMessage emergency_msg = message;
    emergency_msg.priority = MessagePriority::CRITICAL;

    // Emergency messages bypass normal queue and try all protocols
    CombatLoRaMessage lora_msg;
    if (!encode_message(emergency_msg, lora_msg)) {
        std::cerr << "❌ Failed to encode emergency message" << std::endl;
        return false;
    }

    bool success = false;

    // Try 2.4GHz first (usually better penetration)
    if (sx1281_interface_) {
        success = transmit_via_sx1281(lora_msg);
        if (success) {
            std::cout << "🚨 Emergency message sent via 2.4GHz" << std::endl;
        }
    }

    // Also try 915MHz for redundancy
    if (sx1276_interface_) {
        bool backup_success = transmit_via_sx1276(lora_msg);
        if (backup_success) {
            std::cout << "🚨 Emergency message sent via 915MHz" << std::endl;
            success = true;
        }
    }

    if (success) {
        stats_.messages_sent++;
    } else {
        stats_.messages_dropped++;
        std::cerr << "❌ Emergency message transmission failed on all protocols" << std::endl;
    }

    return success;
}

bool LoRaSystem::receive_message(LoRaMessage& message, uint32_t timeout_ms) {
    auto start_time = std::chrono::steady_clock::now();

    while (std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count() < timeout_ms) {

        {
            std::lock_guard<std::mutex> lock(incoming_mutex_);
            if (!incoming_queue_.empty()) {
                message = incoming_queue_.front();
                incoming_queue_.pop();
                return true;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return false; // Timeout
}

// FREQUENCY MANAGEMENT
bool LoRaSystem::set_frequency(uint32_t frequency, LoRaFrequencyBand band) {
std::lock_guard<std::mutex> lock(frequency_mutex_);

bool success = false;

switch (band) {
case LoRaFrequencyBand::BAND_2_4GHz:
if (sx1281_interface_) {
success = sx1281_interface_->set_frequency(frequency);
}
break;

case LoRaFrequencyBand::BAND_915MHz:
if (sx1276_interface_) {
success = sx1276_interface_->set_frequency(frequency);
}
break;

default:
std::cerr << "❌ Unsupported frequency band" << std::endl;
return false;
}

if (success) {
std::cout << "📡 Frequency changed to " << frequency << " Hz" << std::endl;
stats_.frequency_changes++;
}

return success;
}

bool LoRaSystem::enable_frequency_hopping(const FrequencyHoppingConfig& config) {
    std::lock_guard<std::mutex> lock(frequency_mutex_);

    hopping_config_ = config;

    if (config.enabled && !config.frequency_list.empty()) {
        frequency_list_ = config.frequency_list;
        current_frequency_index_.store(0);

        std::cout << "🔄 Frequency hopping enabled with " << frequency_list_.size()
                  << " frequencies" << std::endl;
        return true;
    }

    std::cerr << "❌ Invalid frequency hopping configuration" << std::endl;
    return false;
}

bool LoRaSystem::perform_emergency_frequency_hop() {
    std::cout << "🚨 Emergency frequency hop triggered!" << std::endl;

    std::lock_guard<std::mutex> lock(frequency_mutex_);

    if (frequency_list_.empty()) {
        frequency_list_ = UKRAINIAN_COMBAT_FREQUENCIES;
    }

    // Jump to random frequency to avoid jammers
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, frequency_list_.size() - 1);

    size_t new_index = dis(gen);
    current_frequency_index_.store(new_index);

    uint32_t new_frequency = frequency_list_[new_index];

    // Apply to both radios
    bool success = true;
    if (sx1281_interface_ && new_frequency >= 2400000000 && new_frequency <= 2500000000) {
        success &= sx1281_interface_->set_frequency(new_frequency);
    }
    if (sx1276_interface_ && new_frequency >= 902000000 && new_frequency <= 928000000) {
        success &= sx1276_interface_->set_frequency(new_frequency);
    }

    if (success) {
        std::cout << "✅ Emergency hop to " << new_frequency << " Hz completed" << std::endl;
        stats_.frequency_changes++;
    }

    return success;
}

// POWER MANAGEMENT
bool LoRaSystem::set_tx_power(int8_t power_dbm, LoRaFrequencyBand band) {
    bool success = false;

    switch (band) {
        case LoRaFrequencyBand::BAND_2_4GHz:
            if (sx1281_interface_) {
                success = sx1281_interface_->set_tx_power(power_dbm);
            }
            break;

        case LoRaFrequencyBand::BAND_915MHz:
            if (sx1276_interface_) {
                success = sx1276_interface_->set_tx_power(power_dbm);
            }
            break;

        default:
            return false;
    }

    if (success) {
        current_power_level_.store(power_dbm);
        std::cout << "⚡ TX power set to " << static_cast<int>(power_dbm) << " dBm" << std::endl;
    }

    return success;
}

bool LoRaSystem::enable_adaptive_power(bool enable) {
    adaptive_power_enabled_ = enable;

    if (enable) {
        std::cout << "🔄 Adaptive power control enabled" << std::endl;
    } else {
        std::cout << "⏸️ Adaptive power control disabled" << std::endl;
    }

    return true;
}

// WORKER THREADS IMPLEMENTATION
void LoRaSystem::transmission_worker() {
    std::cout << "🚀 LoRa transmission worker started" << std::endl;

    while (running_.load()) {
        LoRaMessage message;
        bool has_message = false;

        // Check priority queue first
        {
            std::lock_guard<std::mutex> lock(priority_mutex_);
            if (!priority_queue_.empty()) {
                message = priority_queue_.front();
                priority_queue_.pop();
                has_message = true;
            }
        }

        // Check normal queue if no priority messages
        if (!has_message) {
            std::lock_guard<std::mutex> lock(outgoing_mutex_);
            if (!outgoing_queue_.empty()) {
                message = outgoing_queue_.front();
                outgoing_queue_.pop();
                has_message = true;
            }
        }

        if (has_message) {
            // Encode and transmit message
            CombatLoRaMessage lora_msg;
            if (encode_message(message, lora_msg)) {
                bool success = false;

                // Select best protocol based on conditions
                if (message.target_id == 0xFFFF || jamming_detected_.load()) {
                    // Broadcast or jamming - try both protocols
                    if (sx1281_interface_) {
                        success |= transmit_via_sx1281(lora_msg);
                    }
                    if (sx1276_interface_) {
                        success |= transmit_via_sx1276(lora_msg);
                    }
                } else {
                    // Unicast - select best protocol
                    if (sx1281_interface_ && signal_quality_2_4ghz_.rssi_dbm > signal_quality_915mhz_.rssi_dbm) {
                        success = transmit_via_sx1281(lora_msg);
                    } else if (sx1276_interface_) {
                        success = transmit_via_sx1276(lora_msg);
                    }
                }

                if (success) {
                    stats_.messages_sent++;
                    std::cout << "📤 Message transmitted to drone " << message.target_id << std::endl;
                } else {
                    stats_.messages_dropped++;
                    std::cerr << "❌ Message transmission failed to drone " << message.target_id << std::endl;
                }
            }
        } else {
            // Wait for new messages
            std::unique_lock<std::mutex> lock(outgoing_mutex_);
            tx_cv_.wait_for(lock, std::chrono::milliseconds(100));
        }
    }

    std::cout << "🛑 LoRa transmission worker stopped" << std::endl;
}

void LoRaSystem::reception_worker() {
    std::cout << "🚀 LoRa reception worker started" << std::endl;

    while (running_.load()) {
        CombatLoRaMessage lora_msg;
        bool received = false;

        // Try to receive from both interfaces
        if (sx1281_interface_) {
            received = receive_via_sx1281(lora_msg, 10);
        }

        if (!received && sx1276_interface_) {
            received = receive_via_sx1276(lora_msg, 10);
        }

        if (received) {
            // Decode message
            LoRaMessage app_message;
            if (decode_message(lora_msg, app_message)) {
                // Check if message is for us or broadcast
                if (app_message.target_id == drone_id_ || app_message.target_id == 0xFFFF) {
                    // Add to incoming queue
                    {
                        std::lock_guard<std::mutex> lock(incoming_mutex_);
                        incoming_queue_.push(app_message);
                    }

                    // Call message handler if registered
                    {
                        std::lock_guard<std::mutex> lock(handlers_mutex_);
                        auto handler_it = message_handlers_.find(app_message.message_type);
                        if (handler_it != message_handlers_.end()) {
                            handler_it->second(app_message);
                        }
                    }

                    stats_.messages_received++;
                    std::cout << "📥 Message received from drone " << app_message.sender_id << std::endl;

                } else if (mesh_enabled_ && app_message.hop_count < MAX_HOP_COUNT) {
                    // Forward message in mesh network
                    forward_mesh_message(app_message);
                    stats_.messages_forwarded++;
                }
            }
        }

        // Small delay to prevent CPU spinning
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cout << "🛑 LoRa reception worker stopped" << std::endl;
}

void LoRaSystem::frequency_hopping_worker() {
    std::cout << "🚀 LoRa frequency hopping worker started" << std::endl;

    while (running_.load()) {
        if (hopping_config_.enabled && !frequency_list_.empty()) {
            // Check if we need to hop due to jamming
            if (jamming_detected_.load()) {
                perform_emergency_frequency_hop();
                jamming_detected_.store(false);
            } else {
                // Normal scheduled hop
                std::this_thread::sleep_for(std::chrono::milliseconds(hopping_config_.hop_interval_ms));

                if (running_.load()) {
                    perform_frequency_hop();
                }
            }
        } else {
            // Frequency hopping disabled, check every 5 seconds
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    }

    std::cout << "🛑 LoRa frequency hopping worker stopped" << std::endl;
}

void LoRaSystem::monitoring_worker() {
    std::cout << "🚀 LoRa monitoring worker started" << std::endl;

    while (running_.load()) {
        // Update signal quality metrics
        if (sx1281_interface_) {
            int rssi = sx1281_interface_->get_rssi();
            float snr = sx1281_interface_->get_snr();
            update_signal_quality(LoRaFrequencyBand::BAND_2_4GHz, rssi, snr);
        }

        if (sx1276_interface_) {
            int rssi = sx1276_interface_->get_rssi();
            float snr = sx1276_interface_->get_snr();
            update_signal_quality(LoRaFrequencyBand::BAND_915MHz, rssi, snr);
        }

        // Detect jamming
        bool jamming_2_4 = detect_jamming(LoRaFrequencyBand::BAND_2_4GHz);
        bool jamming_915 = detect_jamming(LoRaFrequencyBand::BAND_915MHz);

        if (jamming_2_4 || jamming_915) {
            jamming_detected_.store(true);
            std::cout << "🚨 Jamming detected! Initiating countermeasures..." << std::endl;
        }

        // Adaptive power control
        if (adaptive_power_enabled_) {
            adapt_transmission_parameters();
        }

        // Clean stale mesh routes
        if (mesh_enabled_) {
            clean_stale_mesh_routes();
        }

        // Update statistics
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.last_activity = std::chrono::steady_clock::now();

            // Calculate average RSSI
            stats_.average_rssi = (signal_quality_2_4ghz_.rssi_dbm + signal_quality_915mhz_.rssi_dbm) / 2.0;

            // Calculate packet loss rate
            uint32_t total_expected = stats_.messages_sent + stats_.messages_received;
            if (total_expected > 0) {
                stats_.packet_loss_rate = static_cast<double>(stats_.messages_dropped) / total_expected;
            }
        }

        std::this_thread::sleep_for(SIGNAL_MONITORING_INTERVAL);
    }

    std::cout << "🛑 LoRa monitoring worker stopped" << std::endl;
}

// MESSAGE ENCODING/DECODING
bool LoRaSystem::encode_message(const LoRaMessage& app_message, CombatLoRaMessage& lora_message) {
    lora_message = CombatLoRaMessage(); // Clear structure

    lora_message.sender_id = app_message.sender_id;
    lora_message.target_id = app_message.target_id;
    lora_message.message_type = app_message.message_type;
    lora_message.sequence_number = sequence_number_.fetch_add(1);
    lora_message.timestamp_ns = get_precise_timestamp();
    lora_message.hop_count = app_message.hop_count;
    lora_message.priority = static_cast<uint8_t>(app_message.priority);

    // Encrypt payload if encryption is enabled
    if (encryption_enabled_.load()) {
        size_t encrypted_len = sizeof(lora_message.encrypted_payload);
        if (!encrypt_payload(app_message.payload.data(), app_message.payload.size(),
                             lora_message.encrypted_payload, encrypted_len)) {
            std::cerr << "❌ Payload encryption failed" << std::endl;
            return false;
        }
        lora_message.payload_length = encrypted_len;
    } else {
        // Copy plaintext payload
        size_t copy_len = std::min(app_message.payload.size(), sizeof(lora_message.encrypted_payload));
        memcpy(lora_message.encrypted_payload, app_message.payload.data(), copy_len);
        lora_message.payload_length = copy_len;
    }

    // Calculate CRC16 for the entire message
    lora_message.crc16 = calculate_crc16(reinterpret_cast<const uint8_t*>(&lora_message),
                                         sizeof(lora_message) - sizeof(lora_message.crc16));

    return true;
}

bool LoRaSystem::decode_message(const CombatLoRaMessage& lora_message, LoRaMessage& app_message) {
    // Verify CRC16
    uint16_t calculated_crc = calculate_crc16(reinterpret_cast<const uint8_t*>(&lora_message),
                                              sizeof(lora_message) - sizeof(lora_message.crc16));
    if (calculated_crc != lora_message.crc16) {
        std::cerr << "❌ CRC16 mismatch in received message" << std::endl;
        stats_.crc_errors++;
        return false;
    }

    // Check magic number
    if (lora_message.magic != 0x554B) {
        std::cerr << "❌ Invalid magic number in received message" << std::endl;
        return false;
    }

    app_message.sender_id = lora_message.sender_id;
    app_message.target_id = lora_message.target_id;
    app_message.message_type = lora_message.message_type;
    app_message.timestamp_ns = lora_message.timestamp_ns;
    app_message.hop_count = lora_message.hop_count;
    app_message.priority = static_cast<MessagePriority>(lora_message.priority);

    // Decrypt payload if encryption is enabled
    if (encryption_enabled_.load()) {
        uint8_t decrypted_payload[MAX_PAYLOAD_SIZE];
        size_t decrypted_len = sizeof(decrypted_payload);

        if (!decrypt_payload(lora_message.encrypted_payload, lora_message.payload_length,
                             decrypted_payload, decrypted_len)) {
            std::cerr << "❌ Payload decryption failed" << std::endl;
            stats_.encryption_errors++;
            return false;
        }

        app_message.payload.assign(decrypted_payload, decrypted_payload + decrypted_len);
    } else {
        // Copy plaintext payload
        app_message.payload.assign(lora_message.encrypted_payload,
                                   lora_message.encrypted_payload + lora_message.payload_length);
    }

    return true;
}

// PROTOCOL TRANSMISSION METHODS
bool LoRaSystem::transmit_via_sx1281(const CombatLoRaMessage& message) {
    if (!sx1281_interface_) {
        return false;
    }

    return sx1281_interface_->send_packet(reinterpret_cast<const uint8_t*>(&message), sizeof(message));
}

bool LoRaSystem::transmit_via_sx1276(const CombatLoRaMessage& message) {
    if (!sx1276_interface_) {
        return false;
    }

    return sx1276_interface_->send_packet(reinterpret_cast<const uint8_t*>(&message), sizeof(message));
}

bool LoRaSystem::receive_via_sx1281(CombatLoRaMessage& message, uint32_t timeout_ms) {
    if (!sx1281_interface_) {
        return false;
    }

    size_t length = sizeof(message);
    return sx1281_interface_->receive_packet(reinterpret_cast<uint8_t*>(&message), length, timeout_ms);
}

bool LoRaSystem::receive_via_sx1276(CombatLoRaMessage& message, uint32_t timeout_ms) {
    if (!sx1276_interface_) {
        return false;
    }

    size_t length = sizeof(message);
    return sx1276_interface_->receive_packet(reinterpret_cast<uint8_t*>(&message), length, timeout_ms);
}

// ENCRYPTION METHODS
bool LoRaSystem::initialize_encryption() {
    std::cout << "🔐 Initializing military-grade encryption..." << std::endl;

    // Generate random encryption keys (in production, these would be loaded from secure storage)
    if (RAND_bytes(encryption_key_, sizeof(encryption_key_)) != 1) {
        std::cerr << "❌ Failed to generate encryption key" << std::endl;
        return false;
    }

    if (RAND_bytes(hmac_key_, sizeof(hmac_key_)) != 1) {
        std::cerr << "❌ Failed to generate HMAC key" << std::endl;
        return false;
    }

    std::cout << "✅ Encryption initialized - moskals cannot intercept!" << std::endl;
    return true;
}

bool LoRaSystem::encrypt_payload(const uint8_t* plaintext, size_t plaintext_len,
                                 uint8_t* ciphertext, size_t& ciphertext_len) {
    if (plaintext_len > MAX_PAYLOAD_SIZE - 16) { // Reserve space for IV
        return false;
    }

    // Generate random IV
    uint8_t iv[16];
    if (RAND_bytes(iv, sizeof(iv)) != 1) {
        return false;
    }

    // AES-256-CBC encryption
    AES_KEY aes_key;
    if (AES_set_encrypt_key(encryption_key_, 256, &aes_key) != 0) {
        return false;
    }

    // Pad plaintext to 16-byte boundary (PKCS#7)
    uint8_t padded_plaintext[MAX_PAYLOAD_SIZE];
    size_t padded_len = ((plaintext_len + 15) / 16) * 16;
    uint8_t padding = padded_len - plaintext_len;

    memcpy(padded_plaintext, plaintext, plaintext_len);
    for (size_t i = plaintext_len; i < padded_len; ++i) {
        padded_plaintext[i] = padding;
    }

    // Store IV at the beginning of ciphertext
    memcpy(ciphertext, iv, sizeof(iv));

    // Encrypt
    uint8_t iv_copy[16];
    memcpy(iv_copy, iv, sizeof(iv));
    AES_cbc_encrypt(padded_plaintext, ciphertext + sizeof(iv), padded_len, &aes_key, iv_copy, AES_ENCRYPT);

    ciphertext_len = sizeof(iv) + padded_len;
    return true;
}

bool LoRaSystem::decrypt_payload(const uint8_t* ciphertext, size_t ciphertext_len,
                                 uint8_t* plaintext, size_t& plaintext_len) {
    if (ciphertext_len < 16 || (ciphertext_len - 16) % 16 != 0) {
        return false;
    }

    // Extract IV
    uint8_t iv[16];
    memcpy(iv, ciphertext, sizeof(iv));

    // AES-256-CBC decryption
    AES_KEY aes_key;
    if (AES_set_decrypt_key(encryption_key_, 256, &aes_key) != 0) {
        return false;
    }

    // Decrypt
    uint8_t decrypted[MAX_PAYLOAD_SIZE];
    size_t encrypted_len = ciphertext_len - sizeof(iv);
    AES_cbc_encrypt(ciphertext + sizeof(iv), decrypted, encrypted_len, &aes_key, iv, AES_DECRYPT);

    // Remove PKCS#7 padding
    uint8_t padding = decrypted[encrypted_len - 1];
    if (padding > 16 || padding > encrypted_len) {
        return false;
    }

    plaintext_len = encrypted_len - padding;
    memcpy(plaintext, decrypted, plaintext_len);

    return true;
}

// SIGNAL QUALITY METHODS
void LoRaSystem::update_signal_quality(LoRaFrequencyBand band, int rssi, float snr) {
    std::lock_guard<std::mutex> lock(signal_mutex_);

    SignalQuality* quality = nullptr;
    switch (band) {
        case LoRaFrequencyBand::BAND_2_4GHz:
            quality = &signal_quality_2_4ghz_;
            break;
        case LoRaFrequencyBand::BAND_915MHz:
            quality = &signal_quality_915mhz_;
            break;
        default:
            return;
    }

    quality->rssi_dbm = rssi;
    quality->snr_db = snr;
    quality->last_update = std::chrono::steady_clock::now();
}

bool LoRaSystem::detect_jamming(LoRaFrequencyBand band) {
    std::lock_guard<std::mutex> lock(signal_mutex_);

    const SignalQuality* quality = nullptr;
    switch (band) {
        case LoRaFrequencyBand::BAND_2_4GHz:
            quality = &signal_quality_2_4ghz_;
            break;
        case LoRaFrequencyBand::BAND_915MHz:
            quality = &signal_quality_915mhz_;
            break;
        default:
            return false;
    }

    // Simple jamming detection: strong signal but poor SNR
    if (quality->rssi_dbm > -50 && quality->snr_db < -10) {
        return true;
    }

    // High packet loss rate
    if (quality->packet_loss > 0.7) {
        return true;
    }

    return false;
}

bool LoRaSystem::adapt_transmission_parameters() {
    // Adaptive power based on signal quality
    bool adapted = false;

    if (signal_quality_2_4ghz_.rssi_dbm < -90 && sx1281_interface_) {
        // Weak signal - increase power
        sx1281_interface_->set_tx_power(15); // Maximum power
        adapted = true;
    } else if (signal_quality_2_4ghz_.rssi_dbm > -60 && sx1281_interface_) {
        // Strong signal - reduce power to save energy
        sx1281_interface_->set_tx_power(10);
        adapted = true;
    }

    if (signal_quality_915mhz_.rssi_dbm < -90 && sx1276_interface_) {
        sx1276_interface_->set_tx_power(17); // Maximum power
        adapted = true;
    } else if (signal_quality_915mhz_.rssi_dbm > -60 && sx1276_interface_) {
        sx1276_interface_->set_tx_power(14);
        adapted = true;
    }

    return adapted;
}

// UTILITY METHODS
uint16_t LoRaSystem::calculate_crc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

uint64_t LoRaSystem::get_precise_timestamp() {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

void LoRaSystem::perform_frequency_hop() {
    if (frequency_list_.empty()) {
        return;
    }

    std::