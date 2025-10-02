#ifndef PAIRING_CRYPTO_H
#define PAIRING_CRYPTO_H

#include <Arduino.h>
#include <mbedtls/ecdh.h>
#include <mbedtls/entropy.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/sha256.h>
#include <mbedtls/aes.h>
#include <mbedtls/gcm.h>

//=============================================================================
// ✅ КРИПТОГРАФІЧНІ КОНСТАНТИ
//=============================================================================

namespace PairingCrypto {
    // Використовуємо NIST P-256 (secp256r1) - баланс безпеки та продуктивності
    constexpr auto ECDH_CURVE = MBEDTLS_ECP_DP_SECP256R1;

    // Розміри ключів
    constexpr size_t PUBLIC_KEY_SIZE = 65;    // 1 + 2*32 (uncompressed format)
    constexpr size_t PRIVATE_KEY_SIZE = 32;   // 256 bits
    constexpr size_t SHARED_SECRET_SIZE = 32; // 256 bits
    constexpr size_t SWARM_KEY_SIZE = 32;     // AES-256
    constexpr size_t NONCE_SIZE = 12;         // GCM nonce
    constexpr size_t AUTH_TAG_SIZE = 16;      // GCM auth tag

    // Pre-shared секрет для верифікації (можна змінити)
    const char* const PRE_SHARED_PASSWORD = "SLAVA_UKRAINI_2024_HEROIAM_SLAVA";
}

//=============================================================================
// ✅ СТРУКТУРИ ДЛЯ КРИПТОГРАФІЧНИХ ПОВІДОМЛЕНЬ
//=============================================================================

// Публічний ключ від координатора
struct CoordinatorPublicKey {
    uint8_t message_type;                          // 0xE1
    uint64_t coordinator_mac;                      // MAC координатора
    uint8_t public_key[PairingCrypto::PUBLIC_KEY_SIZE];  // ECDH public key
    uint32_t timestamp;
    uint8_t signature[64];                         // ECDSA signature (для верифікації)
    uint32_t checksum;
} __attribute__((packed));

// Публічний ключ від дрона
struct DronePublicKey {
    uint8_t message_type;                          // 0xE2
    uint64_t drone_mac;                            // MAC дрона
    uint8_t public_key[PairingCrypto::PUBLIC_KEY_SIZE];  // ECDH public key
    uint32_t timestamp;
    uint8_t psk_hash[32];                          // SHA-256(pre-shared password)
    uint32_t checksum;
} __attribute__((packed));

// Зашифроване ID assignment
struct EncryptedIDAssignment {
    uint8_t message_type;                          // 0xE3
    uint64_t coordinator_mac;
    uint64_t target_mac;
    uint8_t nonce[PairingCrypto::NONCE_SIZE];     // GCM nonce
    uint8_t encrypted_data[32];                    // Зашифрований: [ID(2) + timestamp(4) + padding]
    uint8_t auth_tag[PairingCrypto::AUTH_TAG_SIZE]; // GCM tag
    uint32_t checksum;
} __attribute__((packed));

// Swarm Master Key (broadcast після pairing)
struct SwarmMasterKey {
    uint8_t message_type;                          // 0xE4
    uint64_t coordinator_mac;
    uint8_t nonce[PairingCrypto::NONCE_SIZE];
    uint8_t encrypted_key[PairingCrypto::SWARM_KEY_SIZE + 16]; // Key + tag
    uint32_t key_version;                          // Версія ключа для rotation
    uint32_t checksum;
} __attribute__((packed));

//=============================================================================
// ✅ КЛАС PAIRING CRYPTO MANAGER
//=============================================================================

class PairingCryptoManager {
private:
    // mbed TLS contexts
    mbedtls_ecdh_context ecdh_ctx_;
    mbedtls_entropy_context entropy_;
    mbedtls_ctr_drbg_context drbg_;

    // Ключі
    uint8_t my_public_key_[PairingCrypto::PUBLIC_KEY_SIZE];
    uint8_t my_private_key_[PairingCrypto::PRIVATE_KEY_SIZE];
    uint8_t shared_secret_[PairingCrypto::SHARED_SECRET_SIZE];
    uint8_t swarm_master_key_[PairingCrypto::SWARM_KEY_SIZE];

    // Стан
    bool keys_generated_;
    bool shared_secret_derived_;
    bool swarm_key_received_;
    uint64_t my_mac_;

    // Per-drone публічні ключі (для координатора)
    struct DroneKeyInfo {
        uint64_t mac;
        uint8_t public_key[PairingCrypto::PUBLIC_KEY_SIZE];
        uint8_t shared_secret[PairingCrypto::SHARED_SECRET_SIZE];
        bool secret_derived;
    };
    std::vector<DroneKeyInfo> drone_keys_;  // Тільки для координатора

public:
    PairingCryptoManager(uint64_t my_mac);
    ~PairingCryptoManager();

    //=========================================================================
    // ✅ ІНІЦІАЛІЗАЦІЯ
    //=========================================================================

    bool Initialize();
    bool GenerateECDHKeyPair();

    //=========================================================================
    // ✅ KEY EXCHANGE (COORDINATOR)
    //=========================================================================

    // Coordinator broadcast свій public key
    bool BroadcastCoordinatorPublicKey();

    // Coordinator отримує public keys від дронів
    bool ReceiveDronePublicKey(const DronePublicKey* key_msg);

    // Coordinator обчислює shared secret з кожним дроном
    bool DeriveSharedSecretWithDrone(uint64_t drone_mac);

    // Coordinator генерує та broadcast swarm master key
    bool GenerateAndBroadcastSwarmKey();

    //=========================================================================
    // ✅ KEY EXCHANGE (DRONE)
    //=========================================================================

    // Drone отримує public key координатора
    bool ReceiveCoordinatorPublicKey(const CoordinatorPublicKey* key_msg);

    // Drone відправляє свій public key
    bool SendMyPublicKey();

    // Drone обчислює shared secret з координатором
    bool DeriveSharedSecretWithCoordinator(const uint8_t* coordinator_public_key);

    // Drone отримує swarm master key
    bool ReceiveSwarmMasterKey(const SwarmMasterKey* key_msg);

    //=========================================================================
    // ✅ ШИФРУВАННЯ/ДЕШИФРУВАННЯ
    //=========================================================================

    // Шифрування ID assignment для конкретного дрона
    bool EncryptIDAssignment(
            uint64_t target_drone_mac,
            uint16_t drone_id,
            EncryptedIDAssignment& encrypted_msg
    );

    // Дешифрування ID assignment
    bool DecryptIDAssignment(
            const EncryptedIDAssignment* encrypted_msg,
            uint16_t& drone_id
    );

    // AES-GCM шифрування з shared secret
    bool EncryptWithSharedSecret(
            const uint8_t* plaintext, size_t plaintext_len,
            const uint8_t* shared_secret,
            uint8_t* nonce, uint8_t* ciphertext, uint8_t* tag
    );

    // AES-GCM дешифрування з shared secret
    bool DecryptWithSharedSecret(
            const uint8_t* ciphertext, size_t ciphertext_len,
            const uint8_t* shared_secret,
            const uint8_t* nonce, const uint8_t* tag,
            uint8_t* plaintext
    );

    //=========================================================================
    // ✅ ВЕРИФІКАЦІЯ ТА UTILITIES
    //=========================================================================

    // Верифікація pre-shared password
    bool VerifyPreSharedPassword(const uint8_t* psk_hash);

    // Генерація PSK hash
    void GeneratePreSharedHash(uint8_t* output);

    // Getters
    const uint8_t* GetMyPublicKey() const { return my_public_key_; }
    const uint8_t* GetSwarmMasterKey() const { return swarm_master_key_; }
    bool IsSwarmKeyReceived() const { return swarm_key_received_; }

    //=========================================================================
    // ✅ ЗБЕРЕЖЕННЯ КЛЮЧІВ В EEPROM
    //=========================================================================

    bool SaveSwarmKeyToEEPROM();
    bool LoadSwarmKeyFromEEPROM();
    void ClearSwarmKeyFromEEPROM();

    //=========================================================================
    // ✅ ДІАГНОСТИКА
    //=========================================================================

    void PrintCryptoStatus();
    void PrintKeyInfo(const char* label, const uint8_t* key, size_t len);
};

//=============================================================================
// ✅ ГЛОБАЛЬНІ HELPER ФУНКЦІЇ
//=============================================================================

// Генерація secure random bytes
bool GenerateSecureRandom(uint8_t* output, size_t length);

// SHA-256 hash
void ComputeSHA256(const uint8_t* input, size_t length, uint8_t* output);

// Верифікація integrity
bool VerifyMessageIntegrity(const void* message, size_t size, uint32_t expected_checksum);
uint32_t CalculateMessageChecksum(const void* message, size_t size);

#endif // PAIRING_CRYPTO_H