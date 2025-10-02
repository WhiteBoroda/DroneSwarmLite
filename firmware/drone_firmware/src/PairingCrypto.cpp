#include "../include/PairingCrypto.h"
#include <esp_system.h>

//=============================================================================
// ✅ CONSTRUCTOR & DESTRUCTOR
//=============================================================================

PairingCryptoManager::PairingCryptoManager(uint64_t my_mac)
        : keys_generated_(false)
        , shared_secret_derived_(false)
        , swarm_key_received_(false)
        , my_mac_(my_mac) {

    memset(my_public_key_, 0, sizeof(my_public_key_));
    memset(my_private_key_, 0, sizeof(my_private_key_));
    memset(shared_secret_, 0, sizeof(shared_secret_));
    memset(swarm_master_key_, 0, sizeof(swarm_master_key_));

    // Ініціалізація mbed TLS contexts
    mbedtls_ecdh_init(&ecdh_ctx_);
    mbedtls_entropy_init(&entropy_);
    mbedtls_ctr_drbg_init(&drbg_);

    Serial.println("🔐 PairingCryptoManager створено");
}

PairingCryptoManager::~PairingCryptoManager() {
    // Secure cleanup
    mbedtls_ecdh_free(&ecdh_ctx_);
    mbedtls_entropy_free(&entropy_);
    mbedtls_ctr_drbg_free(&drbg_);

    // Zero out sensitive data
    memset(my_private_key_, 0, sizeof(my_private_key_));
    memset(shared_secret_, 0, sizeof(shared_secret_));
    memset(swarm_master_key_, 0, sizeof(swarm_master_key_));

    Serial.println("🔐 PairingCryptoManager знищено (ключі очищено)");
}

//=============================================================================
// ✅ ІНІЦІАЛІЗАЦІЯ
//=============================================================================

bool PairingCryptoManager::Initialize() {
    Serial.println("🔑 Ініціалізація криптографії для pairing...");

    // Seed random number generator
    const char* pers = "swarm_pairing_ecdh";
    int ret = mbedtls_ctr_drbg_seed(
            &drbg_,
            mbedtls_entropy_func,
            &entropy_,
            (const unsigned char*)pers,
            strlen(pers)
    );

    if (ret != 0) {
        Serial.printf("❌ DRBG seed failed: -0x%04x\n", -ret);
        return false;
    }

    // Генеруємо ECDH key pair
    if (!GenerateECDHKeyPair()) {
        Serial.println("❌ Key pair generation failed!");
        return false;
    }

    Serial.println("✅ Crypto initialized");
    return true;
}

bool PairingCryptoManager::GenerateECDHKeyPair() {
    Serial.println("🔑 Генерація ECDH key pair (NIST P-256)...");

    // Setup ECDH context з NIST P-256 curve
    int ret = mbedtls_ecp_group_load(&ecdh_ctx_.grp, PairingCrypto::ECDH_CURVE);
    if (ret != 0) {
        Serial.printf("❌ Group load failed: -0x%04x\n", -ret);
        return false;
    }

    // Генеруємо ключі
    ret = mbedtls_ecdh_gen_public(
            &ecdh_ctx_.grp,
            &ecdh_ctx_.d,
            &ecdh_ctx_.Q,
            mbedtls_ctr_drbg_random,
            &drbg_
    );

    if (ret != 0) {
        Serial.printf("❌ Key generation failed: -0x%04x\n", -ret);
        return false;
    }

    // Export public key в uncompressed format
    size_t olen = 0;
    ret = mbedtls_ecp_point_write_binary(
            &ecdh_ctx_.grp,
            &ecdh_ctx_.Q,
            MBEDTLS_ECP_PF_UNCOMPRESSED,
            &olen,
            my_public_key_,
            sizeof(my_public_key_)
    );

    if (ret != 0 || olen != PairingCrypto::PUBLIC_KEY_SIZE) {
        Serial.printf("❌ Public key export failed: -0x%04x\n", -ret);
        return false;
    }

    // Export private key
    ret = mbedtls_mpi_write_binary(&ecdh_ctx_.d, my_private_key_, sizeof(my_private_key_));
    if (ret != 0) {
        Serial.printf("❌ Private key export failed: -0x%04x\n", -ret);
        return false;
    }

    keys_generated_ = true;

    Serial.println("✅ ECDH keys generated");
    PrintKeyInfo("Public Key", my_public_key_, 65);

    return true;
}

//=============================================================================
// ✅ COORDINATOR: BROADCAST PUBLIC KEY
//=============================================================================

bool PairingCryptoManager::BroadcastCoordinatorPublicKey() {
    if (!keys_generated_) {
        Serial.println("❌ Keys not generated yet!");
        return false;
    }

    Serial.println("📡 Broadcasting coordinator public key...");

    CoordinatorPublicKey msg;
    msg.message_type = 0xE1;
    msg.coordinator_mac = my_mac_;
    memcpy(msg.public_key, my_public_key_, sizeof(msg.public_key));
    msg.timestamp = millis();

    // TODO: ECDSA signature (for now just checksum)
    memset(msg.signature, 0, sizeof(msg.signature));

    msg.checksum = CalculateMessageChecksum(&msg, sizeof(msg) - sizeof(msg.checksum));

    // Відправка через LoRa (3 рази для надійності)
    for (int i = 0; i < 3; i++) {
        LoRa.beginPacket();
        LoRa.write((uint8_t*)&msg, sizeof(msg));
        LoRa.endPacket();
        delay(100);
    }

    LoRa.receive();

    Serial.println("✅ Public key broadcast complete");
    return true;
}

//=============================================================================
// ✅ COORDINATOR: RECEIVE DRONE PUBLIC KEY
//=============================================================================

bool PairingCryptoManager::ReceiveDronePublicKey(const DronePublicKey* key_msg) {
    // Верифікація checksum
    if (!VerifyMessageIntegrity(key_msg, sizeof(*key_msg), key_msg->checksum)) {
        Serial.println("⚠️ Invalid checksum in drone public key");
        return false;
    }

    // Верифікація pre-shared password
    if (!VerifyPreSharedPassword(key_msg->psk_hash)) {
        Serial.println("🚨 INVALID PRE-SHARED PASSWORD! Potential attacker!");
        return false;
    }

    Serial.printf("✅ Received valid public key from MAC=%012llX\n", key_msg->drone_mac);

    // Зберігаємо public key дрона
    DroneKeyInfo info;
    info.mac = key_msg->drone_mac;
    memcpy(info.public_key, key_msg->public_key, sizeof(info.public_key));
    info.secret_derived = false;

    drone_keys_.push_back(info);

    return true;
}

//=============================================================================
// ✅ COORDINATOR: DERIVE SHARED SECRET З ДРОНОМ
//=============================================================================

bool PairingCryptoManager::DeriveSharedSecretWithDrone(uint64_t drone_mac) {
    // Знаходимо дрон
    DroneKeyInfo* drone_info = nullptr;
    for (auto& info : drone_keys_) {
        if (info.mac == drone_mac) {
            drone_info = &info;
            break;
        }
    }

    if (!drone_info) {
        Serial.println("❌ Drone not found!");
        return false;
    }

    Serial.printf("🔑 Deriving shared secret with drone MAC=%012llX\n", drone_mac);

    // Import drone's public key
    mbedtls_ecp_point Q_peer;
    mbedtls_ecp_point_init(&Q_peer);

    int ret = mbedtls_ecp_point_read_binary(
            &ecdh_ctx_.grp,
            &Q_peer,
            drone_info->public_key,
            PairingCrypto::PUBLIC_KEY_SIZE
    );

    if (ret != 0) {
        Serial.printf("❌ Failed to import peer key: -0x%04x\n", -ret);
        mbedtls_ecp_point_free(&Q_peer);
        return false;
    }

    // Compute shared secret
    mbedtls_mpi z;
    mbedtls_mpi_init(&z);

    ret = mbedtls_ecdh_compute_shared(
            &ecdh_ctx_.grp,
            &z,
            &Q_peer,
            &ecdh_ctx_.d,
            mbedtls_ctr_drbg_random,
            &drbg_
    );

    if (ret != 0) {
        Serial.printf("❌ ECDH compute failed: -0x%04x\n", -ret);
        mbedtls_mpi_free(&z);
        mbedtls_ecp_point_free(&Q_peer);
        return false;
    }

    // Export shared secret
    ret = mbedtls_mpi_write_binary(&z, drone_info->shared_secret, PairingCrypto::SHARED_SECRET_SIZE);

    mbedtls_mpi_free(&z);
    mbedtls_ecp_point_free(&Q_peer);

    if (ret != 0) {
        Serial.printf("❌ Secret export failed: -0x%04x\n", -ret);
        return false;
    }

    drone_info->secret_derived = true;

    Serial.println("✅ Shared secret derived");
    PrintKeyInfo("Shared Secret", drone_info->shared_secret, 32);

    return true;
}

//=============================================================================
// ✅ COORDINATOR: GENERATE & BROADCAST SWARM MASTER KEY
//=============================================================================

bool PairingCryptoManager::GenerateAndBroadcastSwarmKey() {
    Serial.println("🔑 Generating swarm master key...");

    // Генеруємо 256-bit swarm master key
    if (!GenerateSecureRandom(swarm_master_key_, sizeof(swarm_master_key_))) {
        Serial.println("❌ Random generation failed!");
        return false;
    }

    swarm_key_received_ = true;

    Serial.println("✅ Swarm key generated");
    PrintKeyInfo("Swarm Master Key", swarm_master_key_, 32);

    // Зберігаємо в EEPROM
    SaveSwarmKeyToEEPROM();

    // Broadcast зашифрованого ключа (TODO: implement encrypted broadcast)
    // Поки що відправляємо plain для тестування
    Serial.println("📡 Broadcasting swarm key...");

    return true;
}

//=============================================================================
// ✅ ENCRYPTION/DECRYPTION
//=============================================================================

bool PairingCryptoManager::EncryptIDAssignment(
        uint64_t target_drone_mac,
        uint16_t drone_id,
        EncryptedIDAssignment& encrypted_msg
) {
    // Знаходимо shared secret з цим дроном
    const uint8_t* shared_secret = nullptr;
    for (const auto& info : drone_keys_) {
        if (info.mac == target_drone_mac && info.secret_derived) {
            shared_secret = info.shared_secret;
            break;
        }
    }

    if (!shared_secret) {
        Serial.println("❌ No shared secret with this drone!");
        return false;
    }

    // Готуємо plaintext: [ID(2) + timestamp(4) + padding]
    uint8_t plaintext[16];
    memset(plaintext, 0, sizeof(plaintext));
    memcpy(plaintext, &drone_id, sizeof(drone_id));
    uint32_t timestamp = millis();
    memcpy(plaintext + 2, &timestamp, sizeof(timestamp));

    // Генеруємо nonce
    GenerateSecureRandom(encrypted_msg.nonce, sizeof(encrypted_msg.nonce));

    // Шифруємо
    if (!EncryptWithSharedSecret(
            plaintext, sizeof(plaintext),
            shared_secret,
            encrypted_msg.nonce,
            encrypted_msg.encrypted_data,
            encrypted_msg.auth_tag
    )) {
        return false;
    }

    encrypted_msg.message_type = 0xE3;
    encrypted_msg.coordinator_mac = my_mac_;
    encrypted_msg.target_mac = target_drone_mac;
    encrypted_msg.checksum = CalculateMessageChecksum(&encrypted_msg,
                                                      sizeof(encrypted_msg) - sizeof(encrypted_msg.checksum));

    return true;
}

bool PairingCryptoManager::DecryptIDAssignment(
        const EncryptedIDAssignment* encrypted_msg,
        uint16_t& drone_id
) {
    if (!shared_secret_derived_) {
        Serial.println("❌ No shared secret derived!");
        return false;
    }

    uint8_t plaintext[16];

    if (!DecryptWithSharedSecret(
            encrypted_msg->encrypted_data, 16,
            shared_secret_,
            encrypted_msg->nonce,
            encrypted_msg->auth_tag,
            plaintext
    )) {
        return false;
    }

    memcpy(&drone_id, plaintext, sizeof(drone_id));

    Serial.printf("✅ Decrypted ID: %04d\n", drone_id);
    return true;
}

//=============================================================================
// ✅ AES-GCM ENCRYPTION/DECRYPTION
//=============================================================================

bool PairingCryptoManager::EncryptWithSharedSecret(
        const uint8_t* plaintext, size_t plaintext_len,
        const uint8_t* shared_secret,
        uint8_t* nonce, uint8_t* ciphertext, uint8_t* tag
) {
    mbedtls_gcm_context gcm;
    mbedtls_gcm_init(&gcm);

    int ret = mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, shared_secret, 256);
    if (ret != 0) {
        mbedtls_gcm_free(&gcm);
        return false;
    }

    ret = mbedtls_gcm_crypt_and_tag(
            &gcm,
            MBEDTLS_GCM_ENCRYPT,
            plaintext_len,
            nonce, PairingCrypto::NONCE_SIZE,
            nullptr, 0,  // No additional data
            plaintext,
            ciphertext,
            PairingCrypto::AUTH_TAG_SIZE,
            tag
    );

    mbedtls_gcm_free(&gcm);

    return (ret == 0);
}

bool PairingCryptoManager::DecryptWithSharedSecret(
        const uint8_t* ciphertext, size_t ciphertext_len,
        const uint8_t* shared_secret,
        const uint8_t* nonce, const uint8_t* tag,
        uint8_t* plaintext
) {
    mbedtls_gcm_context gcm;
    mbedtls_gcm_init(&gcm);

    int ret = mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, shared_secret, 256);
    if (ret != 0) {
        mbedtls_gcm_free(&gcm);
        return false;
    }

    ret = mbedtls_gcm_auth_decrypt(
            &gcm,
            ciphertext_len,
            nonce, PairingCrypto::NONCE_SIZE,
            nullptr, 0,
            tag, PairingCrypto::AUTH_TAG_SIZE,
            ciphertext,
            plaintext
    );

    mbedtls_gcm_free(&gcm);

    if (ret != 0) {
        Serial.printf("⚠️ Decryption failed: -0x%04x\n", -ret);
    }

    return (ret == 0);
}

//=============================================================================
// ✅ PRE-SHARED PASSWORD VERIFICATION
//=============================================================================

bool PairingCryptoManager::VerifyPreSharedPassword(const uint8_t* psk_hash) {
    uint8_t expected_hash[32];
    GeneratePreSharedHash(expected_hash);

    return (memcmp(psk_hash, expected_hash, 32) == 0);
}

void PairingCryptoManager::GeneratePreSharedHash(uint8_t* output) {
    ComputeSHA256(
            (const uint8_t*)PairingCrypto::PRE_SHARED_PASSWORD,
            strlen(PairingCrypto::PRE_SHARED_PASSWORD),
            output
    );
}

//=============================================================================
// ✅ EEPROM STORAGE
//=============================================================================

bool PairingCryptoManager::SaveSwarmKeyToEEPROM() {
    EEPROM.begin(512);

    // Offset 200 для swarm key
    EEPROM.put(200, 0xCAFEBABE);  // Magic
    EEPROM.write(204, swarm_master_key_, sizeof(swarm_master_key_));

    return EEPROM.commit();
}

bool PairingCryptoManager::LoadSwarmKeyFromEEPROM() {
    EEPROM.begin(512);

    uint32_t magic = 0;
    EEPROM.get(200, magic);

    if (magic != 0xCAFEBABE) {
        return false;
    }

    EEPROM.readBytes(204, swarm_master_key_, sizeof(swarm_master_key_));
    swarm_key_received_ = true;

    return true;
}

void PairingCryptoManager::ClearSwarmKeyFromEEPROM() {
    EEPROM.begin(512);
    for (int i = 200; i < 240; i++) {
        EEPROM.write(i, 0xFF);
    }
    EEPROM.commit();
}

//=============================================================================
// ✅ UTILITIES
//=============================================================================

bool GenerateSecureRandom(uint8_t* output, size_t length) {
    for (size_t i = 0; i < length; i++) {
        output[i] = esp_random() & 0xFF;
    }
    return true;
}

void ComputeSHA256(const uint8_t* input, size_t length, uint8_t* output) {
    mbedtls_sha256_context ctx;
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts_ret(&ctx, 0);  // 0 = SHA-256 (not SHA-224)
    mbedtls_sha256_update_ret(&ctx, input, length);
    mbedtls_sha256_finish_ret(&ctx, output);
    mbedtls_sha256_free(&ctx);
}

uint32_t CalculateMessageChecksum(const void* message, size_t size) {
    const uint8_t* data = (const uint8_t*)message;
    uint32_t sum = 0;
    for (size_t i = 0; i < size; i++) {
        sum += data[i];
        sum = (sum << 1) | (sum >> 31);  // Rotate left
    }
    return sum;
}

bool VerifyMessageIntegrity(const void* message, size_t size, uint32_t expected_checksum) {
    return (CalculateMessageChecksum(message, size) == expected_checksum);
}

void PairingCryptoManager::PrintKeyInfo(const char* label, const uint8_t* key, size_t len) {
    Serial.printf("%s (%zu bytes): ", label, len);
    for (size_t i = 0; i < min(len, (size_t)16); i++) {
        Serial.printf("%02X", key[i]);
    }
    if (len > 16) Serial.print("...");
    Serial.println();
}

void PairingCryptoManager::PrintCryptoStatus() {
    Serial.println("\n╔════════════════════════════════════╗");
    Serial.println("║   PAIRING CRYPTO STATUS          ║");
    Serial.println("╠════════════════════════════════════╣");
    Serial.printf("║ Keys Generated:     %-14s ║\n", keys_generated_ ? "YES" : "NO");
    Serial.printf("║ Shared Secret:      %-14s ║\n", shared_secret_derived_ ? "YES" : "NO");
    Serial.printf("║ Swarm Key Received: %-14s ║\n", swarm_key_received_ ? "YES" : "NO");
    Serial.printf("║ Drone Keys Stored:  %-14zu ║\n", drone_keys_.size());
    Serial.println("╚════════════════════════════════════╝\n");
}