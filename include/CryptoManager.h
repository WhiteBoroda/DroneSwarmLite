// include/CryptoManager.h
// Криптографический менеджер для боевой системы управления роем
// 🇺🇦 Slava Ukraini! 🇺🇦

#pragma once

#include "SwarmTypes.h"
#include <vector>
#include <string>
#include <unordered_map>
#include <memory>
#include <cstdint>

namespace SwarmSystem {

// Cryptographic statistics for monitoring
    struct CryptoStatistics {
        uint32_t messages_encrypted;        // Total messages encrypted
        uint32_t messages_decrypted;        // Total messages decrypted
        uint32_t authentication_failures;   // Failed authentications
        uint32_t replay_attacks_detected;   // Blocked replay attacks
        uint32_t key_rotations;            // Number of key rotations
        bool session_key_active;           // Current session key status

        CryptoStatistics() : messages_encrypted(0), messages_decrypted(0),
                             authentication_failures(0), replay_attacks_detected(0),
                             key_rotations(0), session_key_active(false) {}
    };

// Cryptographic configuration parameters
    struct CryptoConfig {
        uint32_t key_rotation_interval;    // seconds - how often to rotate session keys
        uint32_t max_message_age;          // seconds - maximum age of acceptable messages
        uint32_t max_clock_skew;           // seconds - maximum acceptable clock difference
        bool enable_compression;           // compress plaintext before encryption

        CryptoConfig() : key_rotation_interval(3600),  // 1 hour
                         max_message_age(300),           // 5 minutes
                         max_clock_skew(60),             // 1 minute
                         enable_compression(false) {}
    };

// Main cryptographic manager class
    class CryptoManager {
    public:
        explicit CryptoManager(DroneId my_drone_id);
        ~CryptoManager();

        // Initialization and lifecycle
        bool Initialize(const std::string& shared_secret);
        bool IsInitialized() const { return session_key_derived_; }

        // Message encryption/decryption
        bool EncryptMessage(const std::vector<uint8_t>& plaintext,
                            DroneId destination_id,
                            uint8_t message_type,
                            std::vector<uint8_t>& encrypted_output);

        bool DecryptMessage(const std::vector<uint8_t>& encrypted_data,
                            DroneId& sender_id,
                            uint8_t& message_type,
                            std::vector<uint8_t>& plaintext_output);

        // Key management
        bool RotateSessionKey();
        bool UpdateSharedSecret(const std::string& new_shared_secret);

        // Security monitoring
        CryptoStatistics GetStatistics() const;
        void PrintStatistics() const;
        bool HasReplayAttacks() const { return replay_attacks_detected_ > 0; }
        bool HasAuthenticationFailures() const { return authentication_failures_ > 0; }

        // Configuration
        void SetConfig(const CryptoConfig& config) { crypto_config_ = config; }
        CryptoConfig GetConfig() const { return crypto_config_; }

        // Utility methods for specific message types
        bool EncryptTelemetry(const TelemetryData& telemetry, std::vector<uint8_t>& encrypted_output);
        bool DecryptTelemetry(const std::vector<uint8_t>& encrypted_data, TelemetryData& telemetry_output);

        bool EncryptCommand(const DistributedCommand& command, std::vector<uint8_t>& encrypted_output);
        bool DecryptCommand(const std::vector<uint8_t>& encrypted_data, DistributedCommand& command_output);

        // Emergency procedures
        void ZeroizeKeys();              // Emergency key destruction
        bool RegenerateKeys();           // Generate new keys after compromise

    private:
        DroneId my_drone_id_;

        // Cryptographic keys
        uint8_t master_key_[32];         // AES-256 master key derived from shared secret
        uint8_t current_session_key_[32]; // Current session key
        bool master_key_derived_;
        bool session_key_derived_;

        // Sequence counters for replay protection
        uint32_t my_sequence_counter_;    // My outgoing message counter
        std::unordered_map<DroneId, uint32_t> received_sequence_counters_; // Per-drone incoming counters

        // Configuration
        CryptoConfig crypto_config_;

        // Statistics
        mutable uint32_t messages_encrypted_;
        mutable uint32_t messages_decrypted_;
        mutable uint32_t authentication_failures_;
        mutable uint32_t replay_attacks_detected_;
        mutable uint32_t key_rotations_;

        // Constants
        static constexpr size_t MAX_PLAINTEXT_SIZE = 400;  // Maximum plaintext size
        static constexpr uint32_t MAX_MESSAGE_AGE_SECONDS = 300;  // 5 minutes
        static constexpr uint32_t MAX_CLOCK_SKEW_SECONDS = 60;    // 1 minute
        static constexpr size_t MAX_TRACKED_DRONES = 1000;        // Maximum drones to track

        // Private helper methods
        bool DeriveMasterKey(const std::string& shared_secret);
        bool DeriveSessionKey();
        bool InitializeSequenceCounters();

        // Sequence counter management
        bool ValidateSequenceCounter(DroneId sender_id, uint32_t received_counter);
        void UpdateSequenceCounter(DroneId sender_id, uint32_t counter);
        uint32_t GetNextSequenceCounter();

        // Utility functions
        uint32_t GetCurrentTimestamp() const;
        bool GenerateSecureRandom(uint8_t* buffer, size_t size);
        bool RunSelfTest();

        // Prevent copying
        CryptoManager(const CryptoManager&) = delete;
        CryptoManager& operator=(const CryptoManager&) = delete;
    };

// Factory function for creating crypto managers
    std::shared_ptr<CryptoManager> CreateCryptoManager(DroneId drone_id, const std::string& shared_secret);

// Utility functions for cryptographic operations
    namespace CryptoUtils {
        // Message serialization helpers
        std::vector<uint8_t> SerializeTelemetry(const TelemetryData& telemetry);
        bool DeserializeTelemetry(const std::vector<uint8_t>& data, TelemetryData& telemetry);

        std::vector<uint8_t> SerializeCommand(const DistributedCommand& command);
        bool DeserializeCommand(const std::vector<uint8_t>& data, DistributedCommand& command);

        // Key derivation utilities
        std::string GenerateSecurePassphrase(size_t length = 32);
        bool ValidatePassphraseStrength(const std::string& passphrase);

        // Secure memory operations
        void SecureZeroize(void* ptr, size_t size);
        bool SecureCompare(const void* a, const void* b, size_t size);

        // Combat-specific utilities
        std::string GenerateCombatCallsign(DroneId drone_id);  // Generate unique callsigns
        bool ValidateCombatMessage(const std::vector<uint8_t>& message); // Additional validation
    }

// Exception class for cryptographic errors
    class CryptographicException : public std::exception {
    public:
        explicit CryptographicException(const std::string& message) : message_(message) {}
        const char* what() const noexcept override { return message_.c_str(); }

    private:
        std::string message_;
    };

} // namespace SwarmSystem