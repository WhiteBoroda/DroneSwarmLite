//=============================================================================
// firmware/drone_firmware/src/SwarmPairing.cpp
// 🤝 Swarm Auto-Discovery and Pairing System Implementation
// 🇺🇦 SLAVA UKRAINI! 🇺🇦
//=============================================================================

#include "../include/SwarmPairing.h"
#include <LoRa.h>
#include <EEPROM.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

// Глобальний pointer для interrupt handler
SwarmPairingManager* g_pairing_manager = nullptr;

//=============================================================================
// ✅ CONSTRUCTOR & DESTRUCTOR
//=============================================================================

SwarmPairingManager::SwarmPairingManager()
        : my_mac_address_(0)
        , my_drone_id_(0)
        , is_coordinator_(false)
        , pairing_complete_(false)
        , coordinator_mac_(0)
        , beacons_sent_(0)
        , beacons_received_(0)
        , discovery_start_time_(0)
        , pairing_button_pressed_(false)
        , pairing_button_press_time_(0) {

    my_mac_address_ = GetESP32MacAddress();
    pinMode(HardwarePins::PAIRING_BUTTON, INPUT_PULLUP);
    pinMode(HardwarePins::PAIRING_LED, OUTPUT);
    digitalWrite(HardwarePins::PAIRING_LED, LOW);

    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  🤝 SWARM PAIRING MANAGER СТВОРЕНО   ║");
    Serial.println("╚════════════════════════════════════════╝");
    Serial.printf("📱 Мій MAC: %012llX\n\n", my_mac_address_);
    Serial.println("🔘 Кнопка PAIRING готова (Pin 26)\n");

    g_pairing_manager = this;
}

SwarmPairingManager::~SwarmPairingManager() {
    g_pairing_manager = nullptr;
}

//=============================================================================
// ✅ ГОЛОВНА ФУНКЦІЯ AUTO-DISCOVERY
//=============================================================================

bool SwarmPairingManager::EnterAutoDiscoveryMode() {
    Serial.println("\n╔══════════════════════════════════════╗");
    Serial.println("║  🔍 AUTO-DISCOVERY MODE АКТИВНИЙ   ║");
    Serial.println("╚══════════════════════════════════════╝\n");

    pinMode(PairingConfig::LED_DISCOVERY_PIN, OUTPUT);

    // Ініціалізуємо LoRa для discovery
    if (!InitializeLoRaForDiscovery()) {
        Serial.println("❌ Помилка ініціалізації LoRa!");
        return false;
    }

    if (!WaitForPairingButton(300000)) { // Чекаємо 5 хв
        Serial.println("❌ Pairing скасовано (timeout кнопки)");
        return false;
    }

    // ✅ ЗАТРИМКА ДЛЯ СИНХРОНІЗАЦІЇ
    // Після натискання кнопки на ПЕРШОМУ дроні, оператор має час увімкнути інші
    if (is_coordinator_) {
        Serial.println("\n⏰ COORDINATOR: Чекаю 2 хв для включення інших дронів...");
        Serial.println("   Включай інші дрони та натискай на них кнопку!");

        for (int i = 120; i > 0; i--) {
            Serial.printf("   ⏱️  %d секунд...\n", i);

            // LED пульсує (показує що чекаємо)
            int brightness = (i % 2) ? 255 : 50;
            analogWrite(HardwarePins::PAIRING_LED, brightness);

            delay(1000);
            esp_task_wdt_reset();
        }

        Serial.println("✅ Час вийшов - ПОЧИНАЄМО DISCOVERY!\n");
        digitalWrite(HardwarePins::PAIRING_LED, HIGH);

    } else {
        // Follower - невелика затримка
        Serial.println("\n⏰ FOLLOWER: Чекаю 5 секунд перед discovery...");
        delay(5000);
    }

    // ФАЗА 1: Discovery (пошук сусідів)
    if (!RunDiscoveryPhase()) {
        Serial.println("❌ Discovery phase failed!");
        return false;
    }
    // ✅ ФАЗА 1.5: KEY EXCHANGE
    if (!RunKeyExchangePhase()) {
        Serial.println("❌ Key exchange failed!");
        return false;
    }

    // ФАЗА 2: Вибір координатора
    if (!ElectCoordinator()) {
        Serial.println("❌ Coordinator election failed!");
        return false;
    }

    // ФАЗА 3: Pairing (роздача/отримання ID)
    if (is_coordinator_) {
        return AssignEncryptedDroneIDs();
    } else {
        return WaitForEncryptedIDAssignment();
    }
}

bool SwarmPairingManager::WaitForPairingButton(uint32_t timeout_ms) {
    Serial.println("\n╔══════════════════════════════════════╗");
    Serial.println("║  🔘 ОЧІКУВАННЯ КНОПКИ START PAIRING ║");
    Serial.println("╚══════════════════════════════════════╝\n");

    if (timeout_ms == 0) {
        Serial.println("⏳ Чекаю БЕЗКІНЕЧНО...");
        Serial.println("   Натисни кнопку PAIRING для старту");
    } else {
        Serial.printf("⏳ Чекаю %u секунд...\n", timeout_ms / 1000);
        Serial.println("   Натисни кнопку PAIRING для старту");
    }

    Serial.println("\n💡 КНОПКА:");
    Serial.println("   - Короткий натиск (< 2 сек) = JOIN режим (follower)");
    Serial.println("   - Довгий натиск (> 2 сек) = COORDINATOR режим\n");

    unsigned long start_time = millis();
    unsigned long last_blink = 0;
    bool led_state = false;

    while (true) {
        // Timeout перевірка
        if (timeout_ms > 0 && (millis() - start_time > timeout_ms)) {
            Serial.println("\n⏰ TIMEOUT! Кнопка не натиснута");
            digitalWrite(HardwarePins::PAIRING_LED, LOW);
            return false;
        }

        // LED мигає (індикація очікування)
        if (millis() - last_blink > 500) {
            led_state = !led_state;
            digitalWrite(HardwarePins::PAIRING_LED, led_state);
            last_blink = millis();
        }

        // Перевірка кнопки (з debouncing)
        if (IsPairingButtonPressed()) {
            unsigned long press_start = millis();

            // Чекаємо поки кнопка натиснута
            while (digitalRead(HardwarePins::PAIRING_BUTTON) == LOW) {
                delay(10);

                // LED швидко мигає під час натискання
                if ((millis() - press_start) % 100 < 50) {
                    digitalWrite(HardwarePins::PAIRING_LED, HIGH);
                } else {
                    digitalWrite(HardwarePins::PAIRING_LED, LOW);
                }
            }

            unsigned long press_duration = millis() - press_start;

            // Визначаємо режим
            if (press_duration > LONG_PRESS_MS) {
                // ДОВГИЙ натиск = COORDINATOR
                Serial.println("\n👑 ДОВГИЙ НАТИСК - COORDINATOR MODE");
                is_coordinator_ = true;

                // LED постійно світиться (coordinator)
                digitalWrite(HardwarePins::PAIRING_LED, HIGH);
                delay(1000);

            } else {
                // КОРОТКИЙ натиск = FOLLOWER
                Serial.println("\n🚁 КОРОТКИЙ НАТИСК - FOLLOWER MODE");
                is_coordinator_ = false;

                // LED швидко мигає 3 рази (follower)
                for (int i = 0; i < 3; i++) {
                    digitalWrite(HardwarePins::PAIRING_LED, HIGH);
                    delay(100);
                    digitalWrite(HardwarePins::PAIRING_LED, LOW);
                    delay(100);
                }
            }

            Serial.println("✅ Кнопка натиснута - ПОЧИНАЄМО PAIRING!\n");
            return true;
        }

        delay(10);
        esp_task_wdt_reset();
    }
}

bool SwarmPairingManager::IsPairingButtonPressed() {
    static bool last_button_state = HIGH;
    static unsigned long last_debounce_time = 0;

    bool current_state = digitalRead(HardwarePins::PAIRING_BUTTON);

    // Якщо стан змінився
    if (current_state != last_button_state) {
        last_debounce_time = millis();
    }

    // Якщо пройшов debounce час
    if ((millis() - last_debounce_time) > BUTTON_DEBOUNCE_MS) {
        // Якщо кнопка натиснута (LOW бо INPUT_PULLUP)
        if (current_state == LOW && !pairing_button_pressed_) {
            pairing_button_pressed_ = true;
            pairing_button_press_time_ = millis();
            last_button_state = current_state;
            return true;
        }

        // Якщо кнопка відпущена
        if (current_state == HIGH) {
            pairing_button_pressed_ = false;
        }
    }

    last_button_state = current_state;
    return false;
}

//=============================================================================
// ✅ ФАЗА 1: DISCOVERY (Пошук сусідів)
//=============================================================================

bool SwarmPairingManager::RunDiscoveryPhase() {
    Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    Serial.println("🔎 ФАЗА 1: ПОШУК СУСІДІВ");
    Serial.printf("⏱️  Тривалість: %d секунд\n",
                  PairingConfig::DISCOVERY_DURATION_MS / 1000);
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

    discovery_start_time_ = millis();
    unsigned long last_beacon = 0;
    unsigned long last_status_print = 0;

    while (millis() - discovery_start_time_ < PairingConfig::DISCOVERY_DURATION_MS) {
        // Broadcast beacon кожні 500ms
        if (millis() - last_beacon >= PairingConfig::BEACON_INTERVAL_MS) {
            BroadcastDiscoveryBeacon();
            last_beacon = millis();

            // LED: швидке мигання (активний пошук)
            SetDiscoveryLED(true);
        }

        // Статус кожні 3 секунди
        if (millis() - last_status_print >= 3000) {
            PrintDiscoveryStatus();
            last_status_print = millis();
        }

        // Невелика затримка для обробки вхідних повідомлень
        delay(10);
        esp_task_wdt_reset();
    }

    Serial.println("\n✅ ФАЗА 1 ЗАВЕРШЕНА");
    Serial.printf("📊 Знайдено дронів: %d\n", discovered_drones_.size());
    Serial.printf("📡 Beacons відправлено: %d\n", beacons_sent_);
    Serial.printf("📥 Beacons отримано: %d\n\n", beacons_received_);

    return true;
}
bool SwarmPairingManager::RunKeyExchangePhase() {
    Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    Serial.println("🔐 PHASE 1.5: KEY EXCHANGE");
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

    // Ініціалізація crypto
    crypto_manager_ = new PairingCryptoManager(my_mac_address_);
    if (!crypto_manager_->Initialize()) {
        return false;
    }

    if (is_coordinator_) {
        // Coordinator broadcast свій public key
        delay(1000);  // Даємо час дронам підготуватись
        crypto_manager_->BroadcastCoordinatorPublicKey();

        // Чекаємо public keys від всіх дронів
        Serial.printf("👂 Waiting for public keys from %d drones...\n", discovered_drones_.size());

        unsigned long wait_start = millis();
        size_t keys_received = 0;

        while (millis() - wait_start < 10000) {  // 10 сек timeout
            int packetSize = LoRa.parsePacket();

            if (packetSize == sizeof(DronePublicKey)) {
                DronePublicKey drone_key;
                LoRa.readBytes((uint8_t * ) & drone_key, sizeof(drone_key));

                // Перевірка типу повідомлення
                if (drone_key.message_type == 0xE2) {
                    // ✅ Зберігаємо public key
                    if (crypto_manager_->ReceiveDronePublicKey(&drone_key)) {
                        keys_received++;
                        Serial.printf("✅ [%d/%d] Received key from drone %012llX\n",
                                      keys_received, discovered_drones_.size(), drone_key.drone_mac);

                        // Blink LED
                        digitalWrite(PairingConfig::LED_DISCOVERY_PIN, HIGH);
                        delay(50);
                        digitalWrite(PairingConfig::LED_DISCOVERY_PIN, LOW);
                    }
                }
            }

            delay(100);
        }

        if (keys_received == 0) {
            Serial.println("❌ No drone public keys received!");
            return false;
        }

        Serial.printf("✅ Received %d/%d public keys\n\n", keys_received, discovered_drones_.size());

        // Обчислюємо shared secrets з кожним дроном
        Serial.println("🔐 Computing shared secrets with drones...");

        size_t secrets_computed = 0;
        for (const auto &drone: discovered_drones_) {
            if (crypto_manager_->DeriveSharedSecretWithDrone(drone.mac_address)) {
                secrets_computed++;
                Serial.printf("✅ [%d/%d] Shared secret with %012llX\n",
                              secrets_computed, discovered_drones_.size(), drone.mac_address);
            } else {
                Serial.printf("❌ Failed with %012llX\n", drone.mac_address);
            }
        }

        if (secrets_computed == 0) {
            Serial.println("❌ Failed to compute any shared secrets!");
            return false;
        }

        Serial.printf("\n✅ KEY EXCHANGE: %d/%d secrets\n\n", secrets_computed, discovered_drones_.size());


    } else {
        // Drone чекає public key координатора
        Serial.println("👂 Waiting for coordinator public key...");

        CoordinatorPublicKey coord_key;
        bool received = false;
        unsigned long wait_start = millis();

        while (millis() - wait_start < 10000 && !received) {
            int packetSize = LoRa.parsePacket();

            if (packetSize == sizeof(CoordinatorPublicKey)) {
                LoRa.readBytes((uint8_t * ) & coord_key, sizeof(coord_key));

                if (coord_key.message_type == 0xE1) {
                    // ✅ Зберігаємо coordinator public key
                    if (crypto_manager_->ReceiveCoordinatorPublicKey(&coord_key)) {
                        received = true;
                        Serial.printf("✅ Received coordinator key from %012llX\n",
                                      coord_key.coordinator_mac);

                        // Blink LED
                        digitalWrite(PairingConfig::LED_DISCOVERY_PIN, HIGH);
                        delay(100);
                        digitalWrite(PairingConfig::LED_DISCOVERY_PIN, LOW);
                    }
                }
            }

            delay(100);
        }

        if (!received) {
            Serial.println("❌ Coordinator public key timeout!");
            return false;
        }

        // Затримка для рандомізації відповідей (уникаємо колізій)
        delay(random(100, 500));

        // Відправляємо свій public key
        Serial.println("📡 Sending my public key to coordinator...");
        if (!crypto_manager_->SendMyPublicKey()) {
            Serial.println("❌ Failed to send public key!");
            return false;
        }

        // ✅ РОЗКОМЕНТУВАТИ: Обчислюємо shared secret
        Serial.println("🔐 Computing shared secret with coordinator...");
        if (!crypto_manager_->DeriveSharedSecretWithCoordinator(coord_key.public_key)) {
            Serial.println("❌ Failed to derive shared secret!");
            return false;
        }

        Serial.println("✅ Shared secret computed successfully!");
    }
}

bool SwarmPairingManager::AssignEncryptedDroneIDs() {
    Serial.println("📋 PHASE 3: ENCRYPTED ID ASSIGNMENT\n");

    uint16_t next_id = PairingConfig::SWARM_ID_START;

    // Собі
    my_drone_id_ = next_id++;
    SavePairingInfo(my_drone_id_, coordinator_mac_);

    // Іншим (ЗАШИФРОВАНО)
    for (auto& drone : discovered_drones_) {
        uint16_t assigned_id = next_id++;

        // ✅ Шифруємо ID з shared secret
        EncryptedIDAssignment encrypted_msg;
        if (!crypto_manager_->EncryptIDAssignment(
                drone.mac_address,
                assigned_id,
                encrypted_msg
        )) {
            Serial.printf("❌ Encryption failed for drone %012llX\n", drone.mac_address);
            continue;
        }

        // Відправляємо зашифроване повідомлення
        Serial.printf("🔒 Sending encrypted ID=%04d to MAC=%012llX\n",
                      assigned_id, drone.mac_address);

        for (int retry = 0; retry < 3; retry++) {
            LoRa.beginPacket();
            LoRa.write((uint8_t*)&encrypted_msg, sizeof(encrypted_msg));
            LoRa.endPacket();
            delay(100);
        }

        drone.assigned_id = assigned_id;
        drone.is_paired = true;
    }

    // Генеруємо та broadcast swarm master key
    crypto_manager_->GenerateAndBroadcastSwarmKey();

    return true;
}

//=============================================================================
// ✅ BROADCAST DISCOVERY BEACON
//=============================================================================

void SwarmPairingManager::BroadcastDiscoveryBeacon() {
    DiscoveryBeacon beacon;
    beacon.message_type = static_cast<uint8_t>(DiscoveryMessageType::BEACON);
    beacon.mac_address = my_mac_address_;
    beacon.battery_percent = 85;  // TODO: отримати реальний рівень
    beacon.uptime_seconds = millis() / 1000;

    // Checksum: XOR всіх полів
    beacon.checksum = beacon.mac_address ^ beacon.battery_percent ^ beacon.uptime_seconds;

    // Відправка через LoRa
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&beacon, sizeof(beacon));
    LoRa.endPacket();

    beacons_sent_++;

    // Повертаємось в режим прийому
    LoRa.receive();
}

//=============================================================================
// ✅ ОБРОБКА ВХІДНИХ ПОВІДОМЛЕНЬ
//=============================================================================

void SwarmPairingManager::HandleIncomingMessage(uint8_t* data, int length, int rssi) {
    if (length < 1) return;

    DiscoveryMessageType msg_type = static_cast<DiscoveryMessageType>(data[0]);

    switch (msg_type) {
        case DiscoveryMessageType::BEACON:
            if (length >= sizeof(DiscoveryBeacon)) {
                HandleDiscoveryBeacon((DiscoveryBeacon*)data, rssi);
            }
            break;

        case DiscoveryMessageType::ID_ASSIGNMENT:
            if (length >= sizeof(IDAssignmentMessage)) {
                HandleIDAssignment((IDAssignmentMessage*)data);
            }
            break;

        case DiscoveryMessageType::PAIRING_COMPLETE:
            if (length >= sizeof(PairingCompleteMessage)) {
                HandlePairingComplete((PairingCompleteMessage*)data);
            }
            break;

        case DiscoveryMessageType::ID_CONFIRMATION:
            if (length >= sizeof(IDConfirmationMessage)) {
                HandleIDConfirmation((IDConfirmationMessage*)data);
            }
            break;

        default:
            Serial.printf("⚠️ Невідомий тип повідомлення: 0x%02X\n", data[0]);
            break;
    }
}

void SwarmPairingManager::HandleDiscoveryBeacon(const DiscoveryBeacon* beacon, int rssi) {
    // Перевірка checksum
    uint32_t expected = beacon->mac_address ^ beacon->battery_percent ^ beacon->uptime_seconds;
    if (beacon->checksum != expected) {
        return;
    }

    // Ігноруємо свої власні beacons
    if (beacon->mac_address == my_mac_address_) {
        return;
    }

    beacons_received_++;

    // Шукаємо чи вже є цей дрон
    bool found = false;
    for (auto& drone : discovered_drones_) {
        if (drone.mac_address == beacon->mac_address) {
            // Оновлюємо існуючий
            drone.rssi = rssi;
            drone.battery_percent = beacon->battery_percent;
            drone.last_seen = millis();
            found = true;
            break;
        }
    }

    if (!found) {
        // Додаємо новий
        DiscoveredDrone new_drone;
        new_drone.mac_address = beacon->mac_address;
        new_drone.rssi = rssi;
        new_drone.battery_percent = beacon->battery_percent;
        new_drone.last_seen = millis();
        new_drone.is_paired = false;

        discovered_drones_.push_back(new_drone);

        Serial.printf("✨ Новий дрон: MAC=%012llX, RSSI=%d, Battery=%d%%\n",
                      new_drone.mac_address, new_drone.rssi, new_drone.battery_percent);
    }
}

//=============================================================================
// ✅ ФАЗА 2: ВИБІР КООРДИНАТОРА
//=============================================================================

bool SwarmPairingManager::ElectCoordinator() {
    Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    Serial.println("🗳️  ФАЗА 2: ВИБІР КООРДИНАТОРА");
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

    uint64_t best_mac = GetBestCoordinatorMac();

    if (best_mac == my_mac_address_) {
        Serial.println("👑 Я ОБРАНИЙ КООРДИНАТОРОМ!");
        is_coordinator_ = true;
        coordinator_mac_ = my_mac_address_;
    } else {
        Serial.printf("👥 Координатор: MAC=%012llX\n", best_mac);
        is_coordinator_ = false;
        coordinator_mac_ = best_mac;
    }

    Serial.println();
    return true;
}

uint64_t SwarmPairingManager::GetBestCoordinatorMac() {
    uint64_t best_mac = my_mac_address_;
    double best_score = CalculateCoordinatorScore(my_mac_address_, 85, 0);

    Serial.printf("  Я:  MAC=%012llX, Score=%.2f\n", my_mac_address_, best_score);

    for (const auto& drone : discovered_drones_) {
        double score = CalculateCoordinatorScore(
                drone.mac_address,
                drone.battery_percent,
                drone.rssi
        );

        Serial.printf("  Дрон: MAC=%012llX, Score=%.2f\n", drone.mac_address, score);

        if (score > best_score || (score == best_score && drone.mac_address < best_mac)) {
            best_score = score;
            best_mac = drone.mac_address;
        }
    }

    return best_mac;
}

double SwarmPairingManager::CalculateCoordinatorScore(
        uint64_t mac, uint8_t battery, int8_t rssi) {

    // Battery: 0-100 → 50 балів max
    double battery_score = (battery / 100.0) * 50.0;

    // RSSI: -120 до -30 → 30 балів max
    double rssi_normalized = (rssi + 120.0) / 90.0;
    if (rssi_normalized < 0) rssi_normalized = 0;
    if (rssi_normalized > 1) rssi_normalized = 1;
    double rssi_score = rssi_normalized * 30.0;

    // MAC (для детермінізму): менший MAC = більше балів
    double mac_score = 20.0 - ((mac % 1000) / 50.0);

    return battery_score + rssi_score + mac_score;
}

//=============================================================================
// ✅ ФАЗА 3: COORDINATOR - РОЗДАЧА ID
//=============================================================================

bool SwarmPairingManager::AssignDroneIDs() {
    Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    Serial.println("📋 ФАЗА 3: РОЗДАЧА ID (КООРДИНАТОР)");
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

    // Сортуємо по MAC для детермінізму
    std::sort(discovered_drones_.begin(), discovered_drones_.end(),
              [](const DiscoveredDrone& a, const DiscoveredDrone& b) {
                  return a.mac_address < b.mac_address;
              });

    // Роздаємо ID
    uint16_t next_id = PairingConfig::SWARM_ID_START;

    // Спочатку собі
    my_drone_id_ = next_id++;
    SavePairingInfo(my_drone_id_, coordinator_mac_);
    Serial.printf("✅ Мій ID: %04d (КООРДИНАТОР)\n\n", my_drone_id_);

    // Потім іншим
    for (auto& drone : discovered_drones_) {
        uint16_t assigned_id = next_id++;

        Serial.printf("📤 Відправка ID=%04d → MAC=%012llX\n",
                      assigned_id, drone.mac_address);

        // Відправляємо з retry
        bool confirmed = false;
        for (int retry = 0; retry < PairingConfig::MAX_RETRIES && !confirmed; retry++) {
            SendIDAssignment(drone.mac_address, assigned_id);

            // Чекаємо підтвердження (TODO: implement confirmation check)
            delay(PairingConfig::RETRY_DELAY_MS);
            esp_task_wdt_reset();
        }

        drone.assigned_id = assigned_id;
        drone.is_paired = true;
        Serial.println("  ✅ Відправлено\n");
    }

    // Broadcast "Pairing Complete"
    BroadcastPairingComplete();

    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    Serial.println("🎉 PAIRING ЗАВЕРШЕНИЙ!");
    Serial.printf("📊 Всього дронів у рої: %d\n", discovered_drones_.size() + 1);
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

    pairing_complete_ = true;
    return true;
}

void SwarmPairingManager::SendIDAssignment(uint64_t target_mac, uint16_t assigned_id) {
    IDAssignmentMessage msg;
    msg.message_type = static_cast<uint8_t>(DiscoveryMessageType::ID_ASSIGNMENT);
    msg.coordinator_mac = my_mac_address_;
    msg.target_mac = target_mac;
    msg.assigned_id = assigned_id;
    msg.timestamp = millis();
    msg.checksum = msg.coordinator_mac ^ msg.target_mac ^ msg.assigned_id ^ msg.timestamp;

    // Відправляємо 3 рази для надійності
    for (int i = 0; i < 3; i++) {
        LoRa.beginPacket();
        LoRa.write((uint8_t*)&msg, sizeof(msg));
        LoRa.endPacket();
        delay(100);
    }

    LoRa.receive();
}

void SwarmPairingManager::BroadcastPairingComplete() {
    PairingCompleteMessage msg;
    msg.message_type = static_cast<uint8_t>(DiscoveryMessageType::PAIRING_COMPLETE);
    msg.coordinator_mac = my_mac_address_;
    msg.total_drones = discovered_drones_.size() + 1;
    msg.timestamp = millis();
    msg.checksum = msg.coordinator_mac ^ msg.total_drones ^ msg.timestamp;

    Serial.println("📢 BROADCAST: PAIRING COMPLETE");

    // Broadcast 5 разів
    for (int i = 0; i < 5; i++) {
        LoRa.beginPacket();
        LoRa.write((uint8_t*)&msg, sizeof(msg));
        LoRa.endPacket();
        delay(200);
    }

    LoRa.receive();
}

//=============================================================================
// ✅ ФАЗА 3: FOLLOWER - ОЧІКУВАННЯ ID
//=============================================================================

bool SwarmPairingManager::WaitForIDAssignment() {
    Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    Serial.println("⏳ ФАЗА 3: ОЧІКУВАННЯ ID");
    Serial.printf("  Координатор: MAC=%012llX\n", coordinator_mac_);
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

    unsigned long wait_start = millis();

    // LED: повільне мигання
    SetDiscoveryLED(false);

    while (millis() - wait_start < PairingConfig::ID_ASSIGNMENT_TIMEOUT_MS) {
        // Перевіряємо чи отримали ID
        if (my_drone_id_ != 0) {
            Serial.printf("\n✅ ОТРИМАВ ID: %04d\n", my_drone_id_);

            // Відправляємо підтвердження
            SendIDConfirmation(coordinator_mac_);

            pairing_complete_ = true;
            return true;
        }

        delay(100);
        esp_task_wdt_reset();
    }

    // Timeout
    Serial.println("\n⏰ TIMEOUT! Pairing не вдався");
    return false;
}

void SwarmPairingManager::HandleIDAssignment(const IDAssignmentMessage* msg) {
    // Перевірка checksum
    uint32_t expected = msg->coordinator_mac ^ msg->target_mac ^ msg->assigned_id ^ msg->timestamp;
    if (msg->checksum != expected) {
        Serial.println("⚠️ Невірний checksum ID assignment");
        return;
    }

    // Це для мене?
    if (msg->target_mac == my_mac_address_) {
        Serial.printf("\n📥 ОТРИМАНО ID ASSIGNMENT: %04d\n", msg->assigned_id);

        my_drone_id_ = msg->assigned_id;
        SavePairingInfo(my_drone_id_, msg->coordinator_mac);

        Serial.printf("💾 ID збережено в EEPROM\n");
    }
}

void SwarmPairingManager::SendIDConfirmation(uint64_t coordinator_mac) {
    IDConfirmationMessage msg;
    msg.message_type = static_cast<uint8_t>(DiscoveryMessageType::ID_CONFIRMATION);
    msg.my_mac = my_mac_address_;
    msg.my_id = my_drone_id_;
    msg.coordinator_mac = coordinator_mac;
    msg.checksum = msg.my_mac ^ msg.my_id ^ msg.coordinator_mac;

    // Відправляємо 3 рази
    for (int i = 0; i < 3; i++) {
        LoRa.beginPacket();
        LoRa.write((uint8_t*)&msg, sizeof(msg));
        LoRa.endPacket();
        delay(100);
    }

    LoRa.receive();
}

void SwarmPairingManager::HandlePairingComplete(const PairingCompleteMessage* msg) {
    Serial.println("\n🎉 PAIRING ЗАВЕРШЕНИЙ!");
    Serial.printf("📊 Всього дронів: %d\n", msg->total_drones);
    pairing_complete_ = true;
}

void SwarmPairingManager::HandleIDConfirmation(const IDConfirmationMessage* msg) {
    // Coordinator отримує підтвердження
    if (is_coordinator_) {
        Serial.printf("✅ Підтвердження від MAC=%012llX, ID=%04d\n",
                      msg->my_mac, msg->my_id);

        // Позначаємо як paired
        for (auto& drone : discovered_drones_) {
            if (drone.mac_address == msg->my_mac) {
                drone.is_paired = true;
                break;
            }
        }
    }
}

//=============================================================================
// ✅ ДІАГНОСТИКА
//=============================================================================

void SwarmPairingManager::PrintDiscoveryStatus() {
    unsigned long elapsed = (millis() - discovery_start_time_) / 1000;
    unsigned long remaining = (PairingConfig::DISCOVERY_DURATION_MS / 1000) - elapsed;

    Serial.println("\n┌─────────────────────────────────────┐");
    Serial.printf("│ Час: %lu/%lu сек   Залишилось: %lu│\n",
                  elapsed, PairingConfig::DISCOVERY_DURATION_MS / 1000, remaining);
    Serial.printf("│ Знайдено дронів: %-18d│\n", discovered_drones_.size());
    Serial.println("├─────────────────────────────────────┤");

    for (size_t i = 0; i < discovered_drones_.size(); i++) {
        auto& drone = discovered_drones_[i];
        Serial.printf("│ %2zu. MAC: ...%06llX             │\n", i+1, drone.mac_address & 0xFFFFFF);
        Serial.printf("│     RSSI: %-4d  Battery: %-3d%% │\n",
                      drone.rssi, drone.battery_percent);
    }

    Serial.println("└─────────────────────────────────────┘\n");
}

//=============================================================================
// ✅ FORMATION HELPERS
//=============================================================================

Position3D SwarmPairingManager::CalculateFormationPosition(
        uint16_t drone_id, FormationType type, int total_drones, float spacing) {

    Position3D pos;
    int index = drone_id - PairingConfig::SWARM_ID_START;

    switch (type) {
        case FormationType::LINE:
            pos.x = index * spacing;
            pos.y = 0;
            pos.z = 5.0;
            break;

        case FormationType::V_SHAPE:
            pos.x = abs(index - total_drones/2) * (spacing * 0.6f);
            pos.y = index * spacing * 0.8f;
            pos.z = 5.0;
            break;

        case FormationType::CIRCLE: {
            float angle = (2 * PI / total_drones) * index;
            float radius = spacing * 2.0f;
            pos.x = radius * cos(angle);
            pos.y = radius * sin(angle);
            pos.z = 5.0;
            break;
        }

        case FormationType::WEDGE:
            // Клин (трикутник)
            pos.x = abs(index - total_drones/2) * spacing;
            pos.y = -index * spacing * 0.5f;
            pos.z = 5.0;
            break;

        case FormationType::SQUARE: {
            // Квадрат
            int side_size = (int)ceil(sqrt(total_drones));
            int row = index / side_size;
            int col = index % side_size;
            pos.x = col * spacing;
            pos.y = row * spacing;
            pos.z = 5.0;
            break;
        }

        case FormationType::DIAMOND: {
            // Ромб
            int half = total_drones / 2;
            if (index <= half) {
                pos.x = index * (spacing * 0.7f);
                pos.y = (half - index) * spacing;
            } else {
                pos.x = (total_drones - index) * (spacing * 0.7f);
                pos.y = (index - half) * spacing;
            }
            pos.z = 5.0;
            break;
        }
    }

    return pos;
}

const char* SwarmPairingManager::GetFormationName(FormationType type) {
    switch (type) {
        case FormationType::LINE: return "LINE";
        case FormationType::V_SHAPE: return "V-SHAPE";
        case FormationType::CIRCLE: return "CIRCLE";
        case FormationType::WEDGE: return "WEDGE";
        case FormationType::SQUARE: return "SQUARE";
        case FormationType::DIAMOND: return "DIAMOND";
        default: return "UNKNOWN";
    }
}

//=============================================================================
// ✅ GLOBAL HELPER FUNCTIONS
//=============================================================================

uint64_t GetESP32MacAddress() {
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);

    uint64_t mac_uint64 = 0;
    for (int i = 0; i < 6; i++) {
        mac_uint64 |= ((uint64_t)mac[i]) << (8 * (5 - i));
    }

    return mac_uint64;
}

bool SwarmPairingManager::InitializeLoRaForDiscovery() {
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  📡 ІНІЦІАЛІЗАЦІЯ LoRa МОДУЛЯ        ║");
    Serial.println("╚════════════════════════════════════════╝\n");

    // ✅ КРОК 1: Ініціалізація SPI шини
    Serial.println("1️⃣ Ініціалізація SPI шини...");
    SPI.begin(HardwarePins::LORA_SCK,
              HardwarePins::LORA_MISO,
              HardwarePins::LORA_MOSI,
              HardwarePins::LORA_SS);
    Serial.println("   ✅ SPI готовий");

    // ✅ КРОК 2: Налаштування LoRa пінів
    Serial.println("2️⃣ Налаштування LoRa пінів...");
    Serial.printf("   CS:   GPIO %d\n", HardwarePins::LORA_SS);
    Serial.printf("   RST:  GPIO %d\n", HardwarePins::LORA_RST);
    Serial.printf("   DIO0: GPIO %d\n", HardwarePins::LORA_DIO0);

    LoRa.setPins(HardwarePins::LORA_SS,
                 HardwarePins::LORA_RST,
                 HardwarePins::LORA_DIO0);
    Serial.println("   ✅ Піни налаштовані");

    // ✅ КРОК 3: Запуск LoRa на частоті discovery
    Serial.println("3️⃣ Запуск LoRa модуля...");
    Serial.printf("   Частота: %.1f MHz\n", PairingConfig::DISCOVERY_FREQUENCY / 1e6);

    if (!LoRa.begin(PairingConfig::DISCOVERY_FREQUENCY)) {
        Serial.println("   ❌ ПОМИЛКА: LoRa.begin() failed!");
        Serial.println("\n🔍 ДІАГНОСТИКА:");
        Serial.println("   - Перевір підключення SPI");
        Serial.println("   - Перевір живлення LoRa модуля");
        Serial.println("   - Перевір що використовуєш SX1276/SX1278");
        return false;
    }
    Serial.println("   ✅ LoRa модуль запущено");

    // ✅ КРОК 4: Налаштування Spreading Factor
    Serial.println("4️⃣ Налаштування Spreading Factor...");
    Serial.printf("   SF: %d\n", CommConfig::LORA_SPREADING_FACTOR);
    LoRa.setSpreadingFactor(CommConfig::LORA_SPREADING_FACTOR);
    Serial.println("   ✅ SF налаштовано");

    // ✅ КРОК 5: Налаштування Bandwidth
    Serial.println("5️⃣ Налаштування Bandwidth...");
    Serial.printf("   BW: %d kHz\n", CommConfig::LORA_BANDWIDTH);
    LoRa.setSignalBandwidth(CommConfig::LORA_BANDWIDTH * 1000);  // kHz → Hz
    Serial.println("   ✅ BW налаштовано");

    // ✅ КРОК 6: Налаштування Coding Rate
    Serial.println("6️⃣ Налаштування Coding Rate...");
    Serial.printf("   CR: 4/%d\n", CommConfig::LORA_CODING_RATE);
    LoRa.setCodingRate4(CommConfig::LORA_CODING_RATE);
    Serial.println("   ✅ CR налаштовано");

    // ✅ КРОК 7: Налаштування TX Power
    Serial.println("7️⃣ Налаштування TX Power...");
    Serial.printf("   Power: %d dBm\n", CommConfig::LORA_TX_POWER);
    LoRa.setTxPower(CommConfig::LORA_TX_POWER);
    Serial.println("   ✅ TX Power налаштовано");

    // ✅ КРОК 8: Налаштування Sync Word (КРИТИЧНО!)
    Serial.println("8️⃣ Налаштування Sync Word...");
    Serial.println("   Sync Word: 0x34 (Приватна мережа 🇺🇦)");
    LoRa.setSyncWord(0x34);  // Приватна мережа Ukrainian forces
    Serial.println("   ✅ Sync Word налаштовано");
    Serial.println("   ⚠️  ВАЖЛИВО: Тільки дрони з Sync Word 0x34 будуть чути один одного!");

    // ✅ КРОК 9: Увімкнення CRC
    Serial.println("9️⃣ Увімкнення CRC...");
    LoRa.enableCrc();
    Serial.println("   ✅ CRC увімкнено");

    // ✅ КРОК 10: Налаштування Preamble Length
    Serial.println("🔟 Налаштування Preamble Length...");
    LoRa.setPreambleLength(8);  // 8 symbols - стандарт
    Serial.println("   ✅ Preamble = 8 symbols");

    // ✅ КРОК 11: Перехід в RX режим (слухати)
    Serial.println("1️⃣1️⃣ Перехід в RX режим...");
    LoRa.receive();
    Serial.println("   ✅ LoRa слухає ефір");

    // ✅ КРОК 12: Діагностика та тести
    Serial.println("\n📊 ДІАГНОСТИКА РЕЗУЛЬТАТІВ:");
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

    // Перевірка версії чіпа
    uint8_t version = LoRa.readRegister(0x42);  // REG_VERSION
    Serial.printf("Версія чіпа:     0x%02X ", version);
    if (version == 0x12) {
        Serial.println("(SX1276/78 ✅)");
    } else {
        Serial.printf("(Невідомий чіп! ⚠️)\n");
    }

    // Перевірка частоти
    long freq = LoRa.getSignalBandwidth();
    Serial.printf("Bandwidth:       %ld Hz\n", freq);

    // Перевірка Spreading Factor
    int sf = LoRa.getSpreadingFactor();
    Serial.printf("Spreading Factor: SF%d\n", sf);

    // Тестова передача (для перевірки)
    Serial.println("\n🧪 ТЕСТОВА ПЕРЕДАЧА:");
    LoRa.beginPacket();
    LoRa.write(0xAA);  // Тестовий байт
    LoRa.write(0xBB);
    if (LoRa.endPacket()) {
        Serial.println("   ✅ Тестовий пакет відправлено");
    } else {
        Serial.println("   ⚠️  Помилка відправки тестового пакету");
    }

    // Повернення в RX режим після тесту
    LoRa.receive();

    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  ✅ LoRa ГОТОВИЙ ДО PAIRING!         ║");
    Serial.println("╚════════════════════════════════════════╝\n");

    // Виводимо фінальну конфігурацію
    Serial.println("📋 ФІНАЛЬНА КОНФІГУРАЦІЯ:");
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    Serial.printf("Частота:         %.1f MHz\n", PairingConfig::DISCOVERY_FREQUENCY / 1e6);
    Serial.printf("Spreading Factor: SF%d\n", CommConfig::LORA_SPREADING_FACTOR);
    Serial.printf("Bandwidth:       %d kHz\n", CommConfig::LORA_BANDWIDTH);
    Serial.printf("Coding Rate:     4/%d\n", CommConfig::LORA_CODING_RATE);
    Serial.printf("TX Power:        %d dBm\n", CommConfig::LORA_TX_POWER);
    Serial.printf("Sync Word:       0x34 🇺🇦\n");
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

    // Обчислення очікуваних параметрів
    float time_on_air = calculateTimeOnAir(100);  // для пакету 100 байт
    float max_range = estimateRange(CommConfig::LORA_TX_POWER,
                                    CommConfig::LORA_SPREADING_FACTOR);

    Serial.println("📈 ОЧІКУВАНІ ПАРАМЕТРИ:");
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    Serial.printf("Time-on-Air (100B): ~%.0f ms\n", time_on_air);
    Serial.printf("Макс. дальність:    ~%.1f км\n", max_range);
    Serial.printf("Макс. throughput:   ~%.2f kbps\n", calculateThroughput());
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

    return true;
}

float SwarmPairingManager::calculateTimeOnAir(uint8_t payload_size) {
    // Формула для LoRa Time-on-Air
    // https://www.rfwireless-world.com/calculators/LoRaWAN-Airtime-calculator.html

    int SF = CommConfig::LORA_SPREADING_FACTOR;
    int BW = CommConfig::LORA_BANDWIDTH * 1000;  // kHz → Hz
    int CR = CommConfig::LORA_CODING_RATE;
    bool CRC = true;
    bool implicit_header = false;

    float T_sym = (1 << SF) / (float)BW * 1000.0;  // Symbol time (ms)

    int payload_symb_nb = 8 + max((int)ceil((8.0 * payload_size - 4.0 * SF + 28 + 16 * CRC - 20 * implicit_header) / (4.0 * SF)) * CR, 0);

    float T_payload = payload_symb_nb * T_sym;
    float T_preamble = (8 + 4.25) * T_sym;  // 8 symbols preamble + 4.25 sync

    return T_preamble + T_payload;
}

//=============================================================================
// ✅ HELPER: Оцінка дальності
//=============================================================================

float SwarmPairingManager::estimateRange(int8_t tx_power, uint8_t sf) {
    // Спрощена формула для оцінки дальності
    // Реальна дальність залежить від рельєфу, перешкод, антени

    // Link Budget = TX Power + RX Sensitivity - Margins
    float rx_sensitivity = -148.0 + (sf - 7) * 2.5;  // Приблизно для SF7-SF12
    float link_budget = tx_power - rx_sensitivity;

    // Free Space Path Loss: FSPL = 20*log10(d) + 20*log10(f) + 20*log10(4π/c)
    // Для 868 MHz: FSPL = 20*log10(d) + 91.5
    // d = 10^((Link_Budget - 91.5) / 20)

    float path_loss_exponent = 2.0;  // 2.0 - free space, 3.5 - urban
    float distance_km = pow(10, (link_budget - 91.5) / (10 * path_loss_exponent)) / 1000.0;

    return distance_km;
}

//=============================================================================
// ✅ HELPER: Обчислення throughput
//=============================================================================

float SwarmPairingManager::calculateThroughput() {
    // Throughput (kbps) = (Payload_size * 8) / Time_on_Air
    float toa = calculateTimeOnAir(255);  // Max payload
    return (255.0 * 8.0) / toa;  // kbps
}

//=============================================================================
// ✅ ДОДАТКОВА ФУНКЦІЯ: Тест LoRa зв'язку
//=============================================================================

bool SwarmPairingManager::TestLoRaConnection() {
    Serial.println("\n🧪 ТЕСТ LoRa ЗВ'ЯЗКУ:");
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

    // Тест 1: Відправка
    Serial.println("1️⃣ Тест TX (передача)...");
    uint8_t test_packet[] = {0xDE, 0xAD, 0xBE, 0xEF};
    LoRa.beginPacket();
    LoRa.write(test_packet, sizeof(test_packet));
    if (LoRa.endPacket()) {
        Serial.println("   ✅ TX працює");
    } else {
        Serial.println("   ❌ TX НЕ працює!");
        return false;
    }

    // Тест 2: RSSI
    Serial.println("2️⃣ Тест RSSI (якість сигналу)...");
    delay(100);
    int rssi = LoRa.rssi();
    Serial.printf("   RSSI: %d dBm ", rssi);
    if (rssi < -120) {
        Serial.println("(Немає сигналу ⚠️)");
    } else if (rssi < -100) {
        Serial.println("(Слабкий ✅)");
    } else {
        Serial.println("(Сильний ✅)");
    }

    // Тест 3: Прийом (loopback тест якщо є 2 дрони)
    Serial.println("3️⃣ Тест RX (прийом)...");
    Serial.println("   Чекаю 3 секунди на пакет від іншого дрона...");

    LoRa.receive();
    unsigned long start = millis();
    bool received = false;

    while (millis() - start < 3000) {
        int packetSize = LoRa.parsePacket();
        if (packetSize > 0) {
            Serial.printf("   ✅ Отримано пакет (%d bytes, RSSI: %d dBm)\n",
                          packetSize, LoRa.packetRssi());
            received = true;
            break;
        }
        delay(10);
    }

    if (!received) {
        Serial.println("   ⚠️  Пакетів не отримано (це нормально якщо поки немає інших дронів)");
    }

    Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    Serial.println("✅ ТЕСТ ЗАВЕРШЕНО\n");

    return true;
}

void OnDiscoveryMessageReceived(int packet_size) {
    if (!g_pairing_manager || packet_size < 1) return;

    uint8_t buffer[256];
    int i = 0;
    while (LoRa.available() && i < sizeof(buffer)) {
        buffer[i++] = LoRa.read();
    }

    int rssi = LoRa.packetRssi();
    g_pairing_manager->HandleIncomingMessage(buffer, i, rssi);
}

void SetDiscoveryLED(bool fast_blink) {
    static unsigned long last_toggle = 0;
    unsigned long interval = fast_blink ? 100 : 500;

    if (millis() - last_toggle >= interval) {
        digitalWrite(PairingConfig::LED_DISCOVERY_PIN, !digitalRead(PairingConfig::LED_DISCOVERY_PIN));
        last_toggle = millis();
    }
}

bool SavePairingInfo(uint16_t drone_id, uint64_t coordinator_mac) {
    EEPROM.begin(512);
    EEPROM.put(0, 0xDEADBEEF);  // Magic
    EEPROM.put(4, drone_id);
    EEPROM.put(8, coordinator_mac);
    EEPROM.commit();
    return true;
}

bool LoadPairingInfo(uint16_t& drone_id, uint64_t& coordinator_mac) {
    EEPROM.begin(512);

    uint32_t magic = 0;
    EEPROM.get(0, magic);

    if (magic != 0xDEADBEEF) {
        return false;
    }

    EEPROM.get(4, drone_id);
    EEPROM.get(8, coordinator_mac);

    return (drone_id != 0 && drone_id != 0xFFFF);
}

void ClearPairingInfo() {
    EEPROM.begin(512);
    for (int i = 0; i < 16; i++) {
        EEPROM.write(i, 0xFF);
    }
    EEPROM.commit();
}

//=============================================================================
// 🇺🇦 SLAVA UKRAINI! 🇺🇦
//=============================================================================