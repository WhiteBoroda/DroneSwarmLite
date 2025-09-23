// include/UWBCryptoSync.h
// Заголовочний файл для криптографії та синхронізації часу UWB системи
// Військова безпека для дронів-камікадзе проти москалів
// 🇺🇦 Slava Ukraini! Death to russian occupants! 🇺🇦

#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <atomic>
#include <mutex>
#include <functional>
#include <memory>

namespace SwarmControl {

// Forward declarations
    using DroneID = uint16_t;

// Military-grade encryption for UWB packets
    class UWBCrypto {
    private:
        uint8_t aes_key_[32];        // AES-256 key
        uint8_t hmac_key_[32];       // HMAC-SHA256 key
        uint64_t packet_counter_;    // Replay attack protection
        bool initialized_;

        // Private methods
        bool derive_keys_from_secret(const std::string& secret);
        uint64_t get_secure_random_uint64();

    public:
        UWBCrypto();
        ~UWBCrypto() = default;

        // Initialization
        bool initialize(const std::string& shared_secret);

        // Encryption/Decryption
        bool encrypt_packet(const uint8_t* plaintext, size_t plaintext_len,
                            uint8_t* ciphertext, size_t& ciphertext_len,
                            uint64_t& packet_counter);

        bool decrypt_packet(const uint8_t* ciphertext, size_t ciphertext_len,
                            uint8_t* plaintext, size_t& plaintext_len,
                            uint64_t packet_counter);

        // Status
        bool is_initialized() const { return initialized_; }
        uint64_t get_packet_counter() const { return packet_counter_; }
    };

// High-precision time synchronization for swarm coordination
    class UWBTimeSync {
    private:
        std::atomic<bool> is_master_;
        std::atomic<double> clock_offset_ns_;
        std::atomic<uint64_t> last_sync_time_;
        std::vector<double> offset_history_;
        std::mutex sync_mutex_;

        // Clock quality metrics
        std::atomic<double> clock_drift_rate_;
        std::atomic<double> sync_accuracy_;

        // Private methods
        uint64_t get_precise_timestamp_ns();
        double calculate_clock_offset(uint64_t t1, uint64_t t2, uint64_t t3, uint64_t t4);
        void apply_clock_offset(double new_offset);

    public:
        UWBTimeSync();
        ~UWBTimeSync() = default;

        // Initialization
        bool initialize_as_master();
        bool initialize_as_slave();

        // Synchronization
        bool synchronize_with_master(DroneID master_drone_id,
                                     std::function<bool(DroneID, const uint8_t*, size_t)> send_packet,
                                     std::function<bool(uint8_t*, size_t&, DroneID&)> receive_packet);

        bool handle_sync_request(DroneID requesting_drone,
                                 const uint8_t* packet_data, size_t packet_len,
                                 std::function<bool(DroneID, const uint8_t*, size_t)> send_response);

        // Time services
        uint64_t get_synchronized_timestamp_ns();
        double get_sync_accuracy_ns() const;
        bool is_synchronized() const;
        bool is_master() const { return is_master_.load(); }

        // Statistics
        double get_clock_drift_rate() const { return clock_drift_rate_.load(); }
        double get_clock_offset() const { return clock_offset_ns_.load(); }
    };

// Sync packet structure for time synchronization protocol
    struct SyncPacket {
        uint8_t msg_type;       // 1=SYNC_REQ, 2=SYNC_RESP, 3=DELAY_REQ, 4=DELAY_RESP
        uint64_t t1_ns;         // Time when sync request sent
        uint64_t t2_ns;         // Time when sync request received
        uint64_t t3_ns;         // Time when sync response sent
        uint64_t t4_ns;         // Time when sync response received
        double clock_accuracy;   // Master clock accuracy
    } __attribute__((packed));

// Sync message types
    enum class SyncMessageType : uint8_t {
        SYNC_REQUEST = 1,
        SYNC_RESPONSE = 2,
        DELAY_REQUEST = 3,
        DELAY_RESPONSE = 4
    };

} // namespace SwarmControl