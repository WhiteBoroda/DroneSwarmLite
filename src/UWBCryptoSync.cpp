// src/UWBCryptoSync.cpp
// Криптографія та синхронізація часу для UWB системи
// Військова безпека для дронів-камікадзе проти москалів
// 🇺🇦 Slava Ukraini! Death to russian occupants! 🇺🇦

#include "../include/UWBCryptoSync.h"
#include "../include/UWBManager.h"
#include <openssl/aes.h>
#include <openssl/rand.h>
#include <openssl/sha.h>
#include <openssl/hmac.h>
#include <openssl/evp.h>
#include <openssl/kdf.h>
#include <chrono>
#include <random>
#include <iostream>
#include <cstring>

namespace SwarmControl {

// UWBCrypto Implementation
    UWBCrypto::UWBCrypto() : packet_counter_(0), initialized_(false) {
        memset(aes_key_, 0, sizeof(aes_key_));
        memset(hmac_key_, 0, sizeof(hmac_key_));
    }

// UWBTimeSync Implementation
    UWBTimeSync::UWBTimeSync() : is_master_(false), clock_offset_ns_(0.0),
                                 last_sync_time_(0), clock_drift_rate_(0.0), sync_accuracy_(999.0) {
    }

    bool UWBCrypto::initialize(const std::string& shared_secret) {
        std::cout << "🔐 Initializing military-grade UWB encryption..." << std::endl;

        // Derive encryption keys from shared secret using PBKDF2
        if (!derive_keys_from_secret(shared_secret)) {
            std::cerr << "❌ Key derivation failed" << std::endl;
            return false;
        }

        // Initialize packet counter with random value
        packet_counter_ = get_secure_random_uint64();

        initialized_ = true;
        std::cout << "✅ UWB encryption initialized - moskals cannot intercept!" << std::endl;
        return true;
    }

    bool encrypt_packet(const uint8_t* plaintext, size_t plaintext_len,
                        uint8_t* ciphertext, size_t& ciphertext_len,
                        uint64_t& packet_counter) {
        if (!initialized_ || plaintext_len > 48) { // Max payload for UWB packet
            return false;
        }

        packet_counter = ++packet_counter_;

        // Create IV from packet counter and random bytes
        uint8_t iv[16];
        memcpy(iv, &packet_counter, 8);
        RAND_bytes(iv + 8, 8);

        // AES-256-CBC encryption
        AES_KEY aes_key;
        AES_set_encrypt_key(aes_key_, 256, &aes_key);

        uint8_t encrypted_data[64];
        uint8_t padded_plaintext[48];

        // PKCS7 padding
        memcpy(padded_plaintext, plaintext, plaintext_len);
        uint8_t padding_len = 16 - (plaintext_len % 16);
        if (padding_len == 0) padding_len = 16;

        for (size_t i = plaintext_len; i < plaintext_len + padding_len; ++i) {
            padded_plaintext[i] = padding_len;
        }

        size_t padded_len = plaintext_len + padding_len;

        // Encrypt
        for (size_t i = 0; i < padded_len; i += 16) {
            AES_cbc_encrypt(padded_plaintext + i, encrypted_data + i, 16, &aes_key, iv, AES_ENCRYPT);
        }

        // Calculate HMAC for authentication
        uint8_t hmac[32];
        unsigned int hmac_len = 32;
        HMAC(EVP_sha256(), hmac_key_, 32, encrypted_data, padded_len, hmac, &hmac_len);

        // Pack: [encrypted_data][hmac_first_8_bytes][iv_last_8_bytes]
        memcpy(ciphertext, encrypted_data, padded_len);
        memcpy(ciphertext + padded_len, hmac, 8);
        memcpy(ciphertext + padded_len + 8, iv + 8, 8);

        ciphertext_len = padded_len + 16;
        return true;
    }

    bool decrypt_packet(const uint8_t* ciphertext, size_t ciphertext_len,
                        uint8_t* plaintext, size_t& plaintext_len,
                        uint64_t packet_counter) {
        if (!initialized_ || ciphertext_len < 32 || ciphertext_len > 64) {
            return false;
        }

        // Replay attack protection
        if (packet_counter <= packet_counter_) {
            std::cerr << "⚠️ Potential replay attack detected!" << std::endl;
            return false;
        }

        size_t encrypted_len = ciphertext_len - 16;
        const uint8_t* encrypted_data = ciphertext;
        const uint8_t* received_hmac = ciphertext + encrypted_len;
        const uint8_t* iv_suffix = ciphertext + encrypted_len + 8;

        // Reconstruct IV
        uint8_t iv[16];
        memcpy(iv, &packet_counter, 8);
        memcpy(iv + 8, iv_suffix, 8);

        // Verify HMAC
        uint8_t calculated_hmac[32];
        unsigned int hmac_len = 32;
        HMAC(EVP_sha256(), hmac_key_, 32, encrypted_data, encrypted_len, calculated_hmac, &hmac_len);

        if (memcmp(received_hmac, calculated_hmac, 8) != 0) {
            std::cerr << "⚠️ HMAC verification failed - packet corrupted or forged!" << std::endl;
            return false;
        }

        // Decrypt
        AES_KEY aes_key;
        AES_set_decrypt_key(aes_key_, 256, &aes_key);

        uint8_t decrypted_data[64];
        for (size_t i = 0; i < encrypted_len; i += 16) {
            AES_cbc_encrypt(encrypted_data + i, decrypted_data + i, 16, &aes_key, iv, AES_DECRYPT);
        }

        // Remove PKCS7 padding
        uint8_t padding_len = decrypted_data[encrypted_len - 1];
        if (padding_len > 16 || padding_len > encrypted_len) {
            std::cerr << "⚠️ Invalid padding in decrypted packet" << std::endl;
            return false;
        }

        plaintext_len = encrypted_len - padding_len;
        memcpy(plaintext, decrypted_data, plaintext_len);

        // Update counter only after successful decryption
        if (packet_counter > packet_counter_) {
            packet_counter_ = packet_counter;
        }

        return true;
    }

    private:
    bool derive_keys_from_secret(const std::string& secret) {
        // Use PBKDF2 with high iteration count for military security
        const int iterations = 100000;
        const uint8_t salt[] = "UkrainianDroneSwarm2024SlavaUkraini";

        // Derive AES key
        if (PKCS5_PBKDF2_HMAC(secret.c_str(), secret.length(),
                              salt, sizeof(salt) - 1,
                              iterations, EVP_sha256(),
                              32, aes_key_) != 1) {
            return false;
        }

        // Derive HMAC key with different salt
        const uint8_t hmac_salt[] = "DeathToMoscalsUkraineWillWin2024";
        if (PKCS5_PBKDF2_HMAC(secret.c_str(), secret.length(),
                              hmac_salt, sizeof(hmac_salt) - 1,
                              iterations, EVP_sha256(),
                              32, hmac_key_) != 1) {
            return false;
        }

        return true;
    }

    uint64_t get_secure_random_uint64() {
        uint64_t result;
        RAND_bytes(reinterpret_cast<uint8_t*>(&result), sizeof(result));
        return result;
    }
};

bool UWBTimeSync::initialize_as_master() {
    std::cout << "⏰ Initializing as master clock for swarm..." << std::endl;

    is_master_.store(true);
    clock_offset_ns_.store(0.0);
    last_sync_time_.store(get_precise_timestamp_ns());
    sync_accuracy_.store(0.0);

    std::cout << "✅ Master clock initialized - coordinating swarm time" << std::endl;
    return true;
}

bool initialize_as_slave() {
    std::cout << "⏰ Initializing as slave clock..." << std::endl;

    is_master_.store(false);
    clock_offset_ns_.store(0.0);
    sync_accuracy_.store(999.0);

    std::cout << "✅ Slave clock initialized - awaiting sync" << std::endl;
    return true;
}

// Precision Time Protocol (PTP) style synchronization
bool synchronize_with_master(DroneID master_drone_id,
std::function<bool(DroneID, const uint8_t*, size_t)> send_packet,
std::function<bool(uint8_t*, size_t&, DroneID&)> receive_packet) {

if (is_master_.load()) {
return true; // Master doesn't sync with anyone
}

std::cout << "🔄 Synchronizing with master drone " << master_drone_id << "..." << std::endl;

// Four-way handshake for precise time sync
struct SyncPacket {
    uint8_t msg_type;       // 1=SYNC_REQ, 2=SYNC_RESP, 3=DELAY_REQ, 4=DELAY_RESP
    uint64_t t1_ns;         // Time when sync request sent
    uint64_t t2_ns;         // Time when sync request received
    uint64_t t3_ns;         // Time when sync response sent
    uint64_t t4_ns;         // Time when sync response received
    double clock_accuracy;   // Master clock accuracy
} __attribute__((packed));

SyncPacket sync_req = {};
sync_req.msg_type = 1; // SYNC_REQ
sync_req.t1_ns = get_precise_timestamp_ns();

// Step 1: Send sync request
if (!send_packet(master_drone_id, reinterpret_cast<uint8_t*>(&sync_req), sizeof(sync_req))) {
std::cerr << "❌ Failed to send sync request" << std::endl;
return false;
}

// Step 2: Wait for sync response
uint8_t response_buffer[256];
size_t response_len = sizeof(response_buffer);
DroneID sender_id;

auto timeout_start = std::chrono::steady_clock::now();
while (std::chrono::duration_cast<std::chrono::milliseconds>(
std::chrono::steady_clock::now() - timeout_start).count() < 100) {

if (receive_packet(response_buffer, response_len, sender_id) &&
sender_id == master_drone_id && response_len == sizeof(SyncPacket)) {

SyncPacket* sync_resp = reinterpret_cast<SyncPacket*>(response_buffer);
if (sync_resp->msg_type == 2) { // SYNC_RESP
sync_resp->t4_ns = get_precise_timestamp_ns();

// Calculate clock offset using IEEE 1588 algorithm
double offset = calculate_clock_offset(sync_resp->t1_ns, sync_resp->t2_ns,
                                       sync_resp->t3_ns, sync_resp->t4_ns);

// Apply offset with smoothing
apply_clock_offset(offset);

sync_accuracy_.store(sync_resp->clock_accuracy + 10.0); // Add network uncertainty
last_sync_time_.store(get_precise_timestamp_ns());

std::cout << "✅ Time synchronized: offset=" << offset << "ns, accuracy=±"
<< sync_accuracy_.load() << "ns" << std::endl;
return true;
}
}

std::this_thread::sleep_for(std::chrono::microseconds(100));
}

std::cerr << "❌ Sync timeout with master drone" << std::endl;
return false;
}

// Handle incoming sync requests (for master clock)
bool handle_sync_request(DroneID requesting_drone,
                         const uint8_t* packet_data, size_t packet_len,
                         std::function<bool(DroneID, const uint8_t*, size_t)> send_response) {

if (!is_master_.load() || packet_len != sizeof(SyncPacket)) {
return false;
}

const SyncPacket* sync_req = reinterpret_cast<const SyncPacket*>(packet_data);
if (sync_req->msg_type != 1) { // Not a SYNC_REQ
return false;
}

SyncPacket sync_resp = {};
sync_resp.msg_type = 2; // SYNC_RESP
sync_resp.t1_ns = sync_req->t1_ns;          // Copy from request
sync_resp.t2_ns = get_precise_timestamp_ns(); // Time we received request
sync_resp.t3_ns = get_precise_timestamp_ns(); // Time we send response
sync_resp.clock_accuracy = sync_accuracy_.load();

return send_response(requesting_drone, reinterpret_cast<uint8_t*>(&sync_resp), sizeof(sync_resp));
}

uint64_t get_synchronized_timestamp_ns() {
    uint64_t local_time = get_precise_timestamp_ns();

    if (is_master_.load()) {
        return local_time;
    }

    // Apply clock offset for slave
    double offset = clock_offset_ns_.load();

    // Apply drift compensation
    uint64_t time_since_sync = local_time - last_sync_time_.load();
    double drift_correction = clock_drift_rate_.load() * time_since_sync;

    return local_time + static_cast<uint64_t>(offset + drift_correction);
}

double get_sync_accuracy_ns() const {
    return sync_accuracy_.load();
}

bool is_synchronized() const {
    if (is_master_.load()) return true;

    uint64_t time_since_sync = get_precise_timestamp_ns() - last_sync_time_.load();
    return (time_since_sync < 5000000000ULL); // 5 seconds max
}

private:
uint64_t get_precise_timestamp_ns() {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

double calculate_clock_offset(uint64_t t1, uint64_t t2, uint64_t t3, uint64_t t4) {
    // IEEE 1588 Precision Time Protocol offset calculation
    // offset = ((t2 - t1) + (t3 - t4)) / 2
    double delay = static_cast<double>((t4 - t1) - (t3 - t2)) / 2.0;
    double offset = static_cast<double>((t2 - t1) + (t3 - t4)) / 2.0;

    return offset;
}

void apply_clock_offset(double new_offset) {
    std::lock_guard<std::mutex> lock(sync_mutex_);

    // Store offset history for drift calculation
    offset_history_.push_back(new_offset);
    if (offset_history_.size() > 10) {
        offset_history_.erase(offset_history_.begin());
    }

    // Calculate drift rate if we have enough history
    if (offset_history_.size() >= 3) {
        double drift_sum = 0.0;
        for (size_t i = 1; i < offset_history_.size(); ++i) {
            drift_sum += offset_history_[i] - offset_history_[i-1];
        }
        clock_drift_rate_.store(drift_sum / (offset_history_.size() - 1));
    }

    // Apply offset with exponential smoothing to avoid sudden jumps
    double current_offset = clock_offset_ns_.load();
    double smoothed_offset = current_offset * 0.8 + new_offset * 0.2;
    clock_offset_ns_.store(smoothed_offset);
}
};

// Integration into UWBManager
bool UWBManager::initialize_crypto(const std::string& shared_secret) {
    crypto_engine_ = std::make_unique<UWBCrypto>();
    return crypto_engine_->initialize(shared_secret);
}

bool UWBManager::initialize_time_sync(bool is_master) {
    time_sync_ = std::make_unique<UWBTimeSync>();

    if (is_master) {
        return time_sync_->initialize_as_master();
    } else {
        return time_sync_->initialize_as_slave();
    }
}

bool UWBManager::send_encrypted_packet(DroneID target_drone, const uint8_t* data, size_t data_len) {
    if (!crypto_engine_ || !uwb_hardware_) {
        return false;
    }

    UWBPacket packet;
    packet.sender_id = static_cast<uint16_t>(drone_id_);
    packet.target_id = static_cast<uint16_t>(target_drone);
    packet.timestamp_ns = time_sync_ ? time_sync_->get_synchronized_timestamp_ns() :
                          std::chrono::high_resolution_clock::now().time_since_epoch().count();
    packet.packet_type = static_cast<uint8_t>(UWBPacketType::RANGING_REQUEST);

    size_t encrypted_len;
    uint64_t packet_counter;

    if (!crypto_engine_->encrypt_packet(data, data_len, packet.encrypted_data,
                                        encrypted_len, packet_counter)) {
        std::cerr << "❌ Packet encryption failed" << std::endl;
        return false;
    }

    packet.sequence = static_cast<uint32_t>(packet_counter);
    packet.checksum = packet.calculate_checksum();

    return uwb_hardware_->send_ranging_packet(packet);
}

} // namespace SwarmControl