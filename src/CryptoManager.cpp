// src/CryptoManager.cpp
// Криптографический менеджер для боевой системы управления роем
// 🇺🇦 Slava Ukraini! 🇺🇦

#include "../include/CryptoManager.h"
#include "../include/SwarmTypes.h"
#include <openssl/aes.h>
#include <openssl/evp.h>
#include <openssl/rand.h>
#include <openssl/kdf.h>
#include <openssl/sha.h>
#include <openssl/hmac.h>
#include <openssl/hkdf.h>
#include <cstring>
#include <iostream>
#include <algorithm>
#include <chrono>

namespace SwarmSystem {

// Authenticated encryption message structure
    struct AuthenticatedMessage {
        uint8_t nonce[12];              // 96-bit nonce for GCM
        uint32_t sender_id;             // Sender drone ID
        uint32_t sequence_counter;      // Per-sender sequence counter
        uint32_t timestamp;             // Message timestamp (seconds)
        uint8_t ciphertext[512];        // Encrypted payload
        uint8_t auth_tag[16];           // GCM authentication tag
        uint16_t ciphertext_length;     // Length of encrypted data
        uint8_t version;                // Protocol version
        uint8_t message_type;           // Message type identifier

        AuthenticatedMessage() {
            memset(nonce, 0, sizeof(nonce));
            sender_id = 0;
            sequence_counter = 0;
            timestamp = 0;
            memset(ciphertext, 0, sizeof(ciphertext));
            memset(auth_tag, 0, sizeof(auth_tag));
            ciphertext_length = 0;
            version = 2; // Version 2: AES-GCM with per-drone counters
            message_type = 0;
        }
    } __attribute__((packed));

    CryptoManager::CryptoManager(DroneID my_drone_id)
            : my_drone_id_(my_drone_id)
            , master_key_derived_(false)
            , session_key_derived_(false) {

        memset(master_key_, 0, sizeof(master_key_));
        memset(current_session_key_, 0, sizeof(current_session_key_));

        // Initialize per-drone sequence counters
        my_sequence_counter_ = 0;

        std::cout << "🔐 CryptoManager initialized for drone " << my_drone_id_ << std::endl;
    }

    CryptoManager::~CryptoManager() {
        // Secure cleanup of cryptographic material
        OPENSSL_cleanse(master_key_, sizeof(master_key_));
        OPENSSL_cleanse(current_session_key_, sizeof(current_session_key_));

        std::cout << "🔐 CryptoManager destroyed - keys securely cleared" << std::endl;
    }

    bool CryptoManager::Initialize(const std::string& shared_secret) {
        std::cout << "🔑 Initializing cryptographic system..." << std::endl;

        try {
            // Derive master key from shared secret using HKDF
            if (!DeriveMasterKey(shared_secret)) {
                std::cerr << "❌ Failed to derive master key" << std::endl;
                return false;
            }

            // Generate initial session key
            if (!DeriveSessionKey()) {
                std::cerr << "❌ Failed to derive session key" << std::endl;
                return false;
            }

            // Initialize sequence counters with secure random values
            if (!InitializeSequenceCounters()) {
                std::cerr << "❌ Failed to initialize sequence counters" << std::endl;
                return false;
            }

            // Test encryption/decryption functionality
            if (!RunSelfTest()) {
                std::cerr << "❌ Cryptographic self-test failed!" << std::endl;
                return false;
            }

            std::cout << "✅ Cryptographic system initialized successfully" << std::endl;
            std::cout << "🛡️ Using AES-256-GCM with HKDF key derivation" << std::endl;
            std::cout << "🔒 Per-drone replay protection enabled" << std::endl;

            return true;

        } catch (const std::exception& e) {
            std::cerr << "❌ Exception during crypto initialization: " << e.what() << std::endl;
            return false;
        }
    }

    bool CryptoManager::EncryptMessage(const std::vector<uint8_t>& plaintext,
                                       DroneID destination_id,
                                       uint8_t message_type,
                                       std::vector<uint8_t>& encrypted_output) {
        if (!session_key_derived_) {
            std::cerr << "❌ Cannot encrypt: session key not derived" << std::endl;
            return false;
        }

        if (plaintext.size() > MAX_PLAINTEXT_SIZE) {
            std::cerr << "❌ Plaintext too large: " << plaintext.size() << " > " << MAX_PLAINTEXT_SIZE << std::endl;
            return false;
        }

        try {
            AuthenticatedMessage auth_msg;

            // Generate secure random nonce
            if (!GenerateSecureRandom(auth_msg.nonce, sizeof(auth_msg.nonce))) {
                std::cerr << "❌ Failed to generate nonce" << std::endl;
                return false;
            }

            // Set message metadata
            auth_msg.sender_id = static_cast<uint32_t>(my_drone_id_);
            auth_msg.sequence_counter = GetNextSequenceCounter();
            auth_msg.timestamp = GetCurrentTimestamp();
            auth_msg.message_type = message_type;

            // Encrypt with AES-256-GCM
            EVP_CIPHER_CTX* ctx = EVP_CIPHER_CTX_new();
            if (!ctx) {
                std::cerr << "❌ Failed to create cipher context" << std::endl;
                return false;
            }

            // Initialize GCM encryption
            if (EVP_EncryptInit_ex(ctx, EVP_aes_256_gcm(), nullptr, nullptr, nullptr) != 1) {
                EVP_CIPHER_CTX_free(ctx);
                std::cerr << "❌ Failed to initialize GCM" << std::endl;
                return false;
            }

            // Set nonce length (12 bytes for GCM)
            if (EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_GCM_SET_IVLEN, 12, nullptr) != 1) {
                EVP_CIPHER_CTX_free(ctx);
                std::cerr << "❌ Failed to set GCM IV length" << std::endl;
                return false;
            }

            // Set key and nonce
            if (EVP_EncryptInit_ex(ctx, nullptr, nullptr, current_session_key_, auth_msg.nonce) != 1) {
                EVP_CIPHER_CTX_free(ctx);
                std::cerr << "❌ Failed to set key and nonce" << std::endl;
                return false;
            }

            // Add associated data (everything except ciphertext and auth_tag)
            uint8_t* aad_ptr = reinterpret_cast<uint8_t*>(&auth_msg);
            size_t aad_len = offsetof(AuthenticatedMessage, ciphertext);

            int len;
            if (EVP_EncryptUpdate(ctx, nullptr, &len, aad_ptr, aad_len) != 1) {
                EVP_CIPHER_CTX_free(ctx);
                std::cerr << "❌ Failed to add associated data" << std::endl;
                return false;
            }

            // Encrypt plaintext
            int ciphertext_len;
            if (EVP_EncryptUpdate(ctx, auth_msg.ciphertext, &ciphertext_len,
                                  plaintext.data(), plaintext.size()) != 1) {
                EVP_CIPHER_CTX_free(ctx);
                std::cerr << "❌ Failed to encrypt plaintext" << std::endl;
                return false;
            }

            // Finalize encryption
            int final_len;
            if (EVP_EncryptFinal_ex(ctx, auth_msg.ciphertext + ciphertext_len, &final_len) != 1) {
                EVP_CIPHER_CTX_free(ctx);
                std::cerr << "❌ Failed to finalize encryption" << std::endl;
                return false;
            }

            auth_msg.ciphertext_length = ciphertext_len + final_len;

            // Get authentication tag
            if (EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_GCM_GET_TAG, 16, auth_msg.auth_tag) != 1) {
                EVP_CIPHER_CTX_free(ctx);
                std::cerr << "❌ Failed to get authentication tag" << std::endl;
                return false;
            }

            EVP_CIPHER_CTX_free(ctx);

            // Copy to output buffer
            encrypted_output.resize(sizeof(AuthenticatedMessage));
            memcpy(encrypted_output.data(), &auth_msg, sizeof(AuthenticatedMessage));

            // Update statistics
            messages_encrypted_++;

            return true;

        } catch (const std::exception& e) {
            std::cerr << "❌ Exception during encryption: " << e.what() << std::endl;
            return false;
        }
    }

    bool CryptoManager::DecryptMessage(const std::vector<uint8_t>& encrypted_data,
                                       DroneID& sender_id,
                                       uint8_t& message_type,
                                       std::vector<uint8_t>& plaintext_output) {
        if (!session_key_derived_) {
            std::cerr << "❌ Cannot decrypt: session key not derived" << std::endl;
            return false;
        }

        if (encrypted_data.size() < sizeof(AuthenticatedMessage)) {
            std::cerr << "❌ Encrypted data too small" << std::endl;
            return false;
        }

        try {
            AuthenticatedMessage auth_msg;
            memcpy(&auth_msg, encrypted_data.data(), sizeof(AuthenticatedMessage));

            // Validate message structure
            if (auth_msg.version != 2) {
                std::cerr << "❌ Unsupported protocol version: " << static_cast<int>(auth_msg.version) << std::endl;
                return false;
            }

            if (auth_msg.ciphertext_length > sizeof(auth_msg.ciphertext)) {
                std::cerr << "❌ Invalid ciphertext length: " << auth_msg.ciphertext_length << std::endl;
                return false;
            }

            sender_id = static_cast<DroneID>(auth_msg.sender_id);
            message_type = auth_msg.message_type;

            // Validate sequence counter (replay protection)
            if (!ValidateSequenceCounter(sender_id, auth_msg.sequence_counter)) {
                std::cerr << "❌ Replay attack detected from drone " << sender_id
                          << " (counter: " << auth_msg.sequence_counter << ")" << std::endl;
                replay_attacks_detected_++;
                return false;
            }

            // Validate timestamp (prevent old message replay)
            uint32_t current_time = GetCurrentTimestamp();
            if (auth_msg.timestamp > current_time + MAX_CLOCK_SKEW_SECONDS ||
                auth_msg.timestamp < current_time - MAX_MESSAGE_AGE_SECONDS) {
                std::cerr << "❌ Message timestamp out of acceptable range" << std::endl;
                return false;
            }

            // Decrypt with AES-256-GCM
            EVP_CIPHER_CTX* ctx = EVP_CIPHER_CTX_new();
            if (!ctx) {
                std::cerr << "❌ Failed to create cipher context for decryption" << std::endl;
                return false;
            }

            // Initialize GCM decryption
            if (EVP_DecryptInit_ex(ctx, EVP_aes_256_gcm(), nullptr, nullptr, nullptr) != 1) {
                EVP_CIPHER_CTX_free(ctx);
                std::cerr << "❌ Failed to initialize GCM decryption" << std::endl;
                return false;
            }

            // Set nonce length
            if (EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_GCM_SET_IVLEN, 12, nullptr) != 1) {
                EVP_CIPHER_CTX_free(ctx);
                std::cerr << "❌ Failed to set GCM IV length for decryption" << std::endl;
                return false;
            }

            // Set key and nonce
            if (EVP_DecryptInit_ex(ctx, nullptr, nullptr, current_session_key_, auth_msg.nonce) != 1) {
                EVP_CIPHER_CTX_free(ctx);
                std::cerr << "❌ Failed to set key and nonce for decryption" << std::endl;
                return false;
            }

            // Set expected authentication tag
            if (EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_GCM_SET_TAG, 16, auth_msg.auth_tag) != 1) {
                EVP_CIPHER_CTX_free(ctx);
                std::cerr << "❌ Failed to set authentication tag" << std::endl;
                return false;
            }

            // Add associated data
            uint8_t* aad_ptr = reinterpret_cast<uint8_t*>(&auth_msg);
            size_t aad_len = offsetof(AuthenticatedMessage, ciphertext);

            int len;
            if (EVP_DecryptUpdate(ctx, nullptr, &len, aad_ptr, aad_len) != 1) {
                EVP_CIPHER_CTX_free(ctx);
                std::cerr << "❌ Failed to process associated data" << std::endl;
                return false;
            }

            // Decrypt ciphertext
            uint8_t plaintext_buffer[MAX_PLAINTEXT_SIZE];
            int plaintext_len;
            if (EVP_DecryptUpdate(ctx, plaintext_buffer, &plaintext_len,
                                  auth_msg.ciphertext, auth_msg.ciphertext_length) != 1) {
                EVP_CIPHER_CTX_free(ctx);
                std::cerr << "❌ Failed to decrypt ciphertext" << std::endl;
                return false;
            }

            // Finalize decryption and verify authentication tag
            int final_len;
            int result = EVP_DecryptFinal_ex(ctx, plaintext_buffer + plaintext_len, &final_len);
            EVP_CIPHER_CTX_free(ctx);

            if (result != 1) {
                std::cerr << "❌ Authentication verification failed - message tampered or corrupted!" << std::endl;
                authentication_failures_++;
                return false;
            }

            // Update sequence counter for sender
            UpdateSequenceCounter(sender_id, auth_msg.sequence_counter);

            // Copy plaintext to output
            int total_plaintext_len = plaintext_len + final_len;
            plaintext_output.resize(total_plaintext_len);
            memcpy(plaintext_output.data(), plaintext_buffer, total_plaintext_len);

            // Update statistics
            messages_decrypted_++;

            return true;

        } catch (const std::exception& e) {
            std::cerr << "❌ Exception during decryption: " << e.what() << std::endl;
            return false;
        }
    }

    bool CryptoManager::RotateSessionKey() {
        std::cout << "🔄 Rotating session key..." << std::endl;

        // Derive new session key from master key with current timestamp as salt
        uint32_t timestamp = GetCurrentTimestamp();
        uint8_t salt[8];
        memcpy(salt, &timestamp, sizeof(timestamp));

        // Add drone ID to salt for uniqueness
        memcpy(salt + 4, &my_drone_id_, sizeof(my_drone_id_));

        if (HKDF(current_session_key_, sizeof(current_session_key_),
                 EVP_sha256(),
                 master_key_, sizeof(master_key_),
                 salt, sizeof(salt),
                 reinterpret_cast<const uint8_t*>("SESSION_KEY_ROTATION"), 19) != 1) {
            std::cerr << "❌ Failed to derive new session key" << std::endl;
            return false;
        }

        // Reset sequence counters after key rotation
        my_sequence_counter_ = 0;
        received_sequence_counters_.clear();

        std::cout << "✅ Session key rotated successfully" << std::endl;
        return true;
    }

    CryptoStatistics CryptoManager::GetStatistics() const {
        CryptoStatistics stats;
        stats.messages_encrypted = messages_encrypted_;
        stats.messages_decrypted = messages_decrypted_;
        stats.authentication_failures = authentication_failures_;
        stats.replay_attacks_detected = replay_attacks_detected_;
        stats.key_rotations = key_rotations_;
        stats.session_key_active = session_key_derived_;

        return stats;
    }

    void CryptoManager::PrintStatistics() const {
        auto stats = GetStatistics();

        std::cout << "\n🔐 === CRYPTO STATISTICS ===" << std::endl;
        std::cout << "📨 Messages encrypted: " << stats.messages_encrypted << std::endl;
        std::cout << "📩 Messages decrypted: " << stats.messages_decrypted << std::endl;
        std::cout << "❌ Authentication failures: " << stats.authentication_failures << std::endl;
        std::cout << "🔄 Replay attacks blocked: " << stats.replay_attacks_detected << std::endl;
        std::cout << "🔑 Key rotations: " << stats.key_rotations << std::endl;
        std::cout << "🟢 Session key status: " << (stats.session_key_active ? "ACTIVE" : "INACTIVE") << std::endl;
        std::cout << "==========================\n" << std::endl;
    }

// Private helper methods
    bool CryptoManager::DeriveMasterKey(const std::string& shared_secret) {
        // Use HKDF to derive master key from shared secret
        // Salt includes "UKRAINE" for additional uniqueness
        const uint8_t ukraine_salt[] = "UKRAINE_2024_SLAVA_UKRAINI";

        if (HKDF(master_key_, sizeof(master_key_),
                 EVP_sha256(),
                 reinterpret_cast<const uint8_t*>(shared_secret.c_str()), shared_secret.length(),
                 ukraine_salt, sizeof(ukraine_salt) - 1, // -1 to exclude null terminator
                 reinterpret_cast<const uint8_t*>("SWARM_MASTER_KEY"), 16) != 1) {
            return false;
        }

        master_key_derived_ = true;
        return true;
    }

    bool CryptoManager::DeriveSessionKey() {
        if (!master_key_derived_) {
            return false;
        }

        // Derive session key from master key with drone ID and timestamp
        uint32_t timestamp = GetCurrentTimestamp();
        uint8_t salt[8];
        memcpy(salt, &my_drone_id_, sizeof(my_drone_id_));
        memcpy(salt + 4, &timestamp, sizeof(timestamp));

        if (HKDF(current_session_key_, sizeof(current_session_key_),
                 EVP_sha256(),
                 master_key_, sizeof(master_key_),
                 salt, sizeof(salt),
                 reinterpret_cast<const uint8_t*>("SESSION_KEY_INITIAL"), 19) != 1) {
            return false;
        }

        session_key_derived_ = true;
        return true;
    }

    bool CryptoManager::InitializeSequenceCounters() {
        // Initialize sequence counter with secure random value to prevent predictability
        if (!GenerateSecureRandom(reinterpret_cast<uint8_t*>(&my_sequence_counter_), sizeof(my_sequence_counter_))) {
            return false;
        }

        // Ensure counter starts from a reasonable range (not too close to wrap-around)
        my_sequence_counter_ = my_sequence_counter_ % 1000000000; // Keep it under 1 billion

        return true;
    }

    bool CryptoManager::ValidateSequenceCounter(DroneID sender_id, uint32_t received_counter) {
        auto it = received_sequence_counters_.find(sender_id);

        if (it == received_sequence_counters_.end()) {
            // First message from this drone - accept and store
            return true;
        }

        uint32_t last_counter = it->second;

        // Check for replay (counter must be strictly increasing)
        if (received_counter <= last_counter) {
            return false; // Replay attack
        }

        // Check for excessively large counter jump (potential attack)
        const uint32_t MAX_COUNTER_JUMP = 1000000; // Allow up to 1M message gap
        if (received_counter > last_counter + MAX_COUNTER_JUMP) {
            std::cerr << "⚠️ Large counter jump from drone " << sender_id
                      << ": " << last_counter << " -> " << received_counter << std::endl;
            // Accept but log suspicious activity
        }

        return true;
    }

    void CryptoManager::UpdateSequenceCounter(DroneID sender_id, uint32_t counter) {
        received_sequence_counters_[sender_id] = counter;

        // Clean up old entries if we have too many
        if (received_sequence_counters_.size() > MAX_TRACKED_DRONES) {
            // Remove oldest entries (simplified cleanup)
            auto oldest = received_sequence_counters_.begin();
            received_sequence_counters_.erase(oldest);
        }
    }

    uint32_t CryptoManager::GetNextSequenceCounter() {
        return ++my_sequence_counter_;
    }

    uint32_t CryptoManager::GetCurrentTimestamp() const {
        return std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
    }

    bool CryptoManager::GenerateSecureRandom(uint8_t* buffer, size_t size) {
        return RAND_bytes(buffer, size) == 1;
    }

    bool CryptoManager::RunSelfTest() {
        std::cout << "🧪 Running cryptographic self-test..." << std::endl;

        try {
            // Test encryption/decryption roundtrip
            std::vector<uint8_t> test_plaintext = {'T', 'e', 's', 't', ' ', 'm', 'e', 's', 's', 'a', 'g', 'e'};
            std::vector<uint8_t> encrypted_data;
            std::vector<uint8_t> decrypted_data;

            // Encrypt test message
            if (!EncryptMessage(test_plaintext, 999, 0x42, encrypted_data)) {
                std::cerr << "❌ Self-test: encryption failed" << std::endl;
                return false;
            }

            // Decrypt test message
            DroneId sender_id;
            uint8_t message_type;
            if (!DecryptMessage(encrypted_data, sender_id, message_type, decrypted_data)) {
                std::cerr << "❌ Self-test: decryption failed" << std::endl;
                return false;
            }

            // Verify roundtrip
            if (sender_id != my_drone_id_ || message_type != 0x42 ||
                decrypted_data != test_plaintext) {
                std::cerr << "❌ Self-test: roundtrip verification failed" << std::endl;
                return false;
            }

            // Test replay protection
            DroneID dummy_sender;
            uint8_t dummy_type;
            std::vector<uint8_t> dummy_output;
            if (DecryptMessage(encrypted_data, dummy_sender, dummy_type, dummy_output)) {
                std::cerr << "❌ Self-test: replay protection failed" << std::endl;
                return false;
            }

            std::cout << "✅ Cryptographic self-test passed" << std::endl;
            return true;

        } catch (const std::exception& e) {
            std::cerr << "❌ Exception in self-test: " << e.what() << std::endl;
            return false;
        }
    }

} // namespace SwarmSystem