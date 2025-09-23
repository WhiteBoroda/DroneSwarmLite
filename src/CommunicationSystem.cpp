#include "../include/SwarmSystem.h"
#include <thread>
#include <chrono>
#include <random>
#include <algorithm>
#include <cstring>

// Додаткові інклюди для роботи з обладнанням
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

namespace SwarmSystem {

// Протокол повідомлень
    struct LoRaMessage {
        uint32_t magic;           // 0xDEADBEEF - магічне число
        uint8_t version;          // Версія протоколу
        uint8_t message_type;     // Тип повідомлення
        DroneID sender_id;        // ID відправника
        DroneID target_id;        // ID отримувача (0 = broadcast)
        uint16_t sequence;        // Номер послідовності
        uint16_t payload_size;    // Розмір корисних даних
        uint32_t checksum;        // Контрольна сума
        uint8_t payload[512];     // Корисні дані

        LoRaMessage() {
            magic = 0xDEADBEEF;
            version = 1;
            message_type = 0;
            sender_id = 0;
            target_id = 0;
            sequence = 0;
            payload_size = 0;
            checksum = 0;
            memset(payload, 0, sizeof(payload));
        }
    };

// Типи повідомлень
    enum class MessageType : uint8_t {
        HEARTBEAT = 1,
        COMMAND = 2,
        TELEMETRY = 3,
        FORMATION_UPDATE = 4,
        LEADER_ELECTION = 5,
        EMERGENCY = 6,
        VIDEO_CONTROL = 7,
        FREQUENCY_CHANGE = 8
    };

    CommunicationSystem::CommunicationSystem(std::shared_ptr<ConfigManager> config)
            : SwarmSubsystem(config), current_rssi_(-100), jamming_detected_(false) {
        comm_config_ = config_->GetCommConfig();
    }

    bool CommunicationSystem::Initialize() {
        if (!InitializeLoRa()) {
            std::cerr << "Помилка ініціалізації LoRa" << std::endl;
            return false;
        }

        if (comm_config_.mesh_enabled && !InitializeMesh()) {
            std::cerr << "Попередження: Mesh мережа не ініціалізована" << std::endl;
        }

        is_healthy_ = true;
        return true;
    }

    bool CommunicationSystem::InitializeLoRa() {
        try {
            // Ініціалізація ELRS Nano SX1281 (2.4GHz) та SX1276 (915MHz)
            // Тут має бути код ініціалізації конкретного обладнання

            // Налаштування першої частоти
            if (!comm_config_.lora_frequencies.empty()) {
                comm_config_.current_frequency = comm_config_.lora_frequencies[0];
            }

            // Встановлення початкової потужності
            AdjustPower(comm_config_.power_level);

            std::cout << "LoRa ініціалізовано на частоті "
                      << comm_config_.current_frequency << " Hz, "
                      << "потужність " << static_cast<int>(comm_config_.power_level) << " dBm"
                      << std::endl;

            return true;
        }
        catch (const std::exception& e) {
            std::cerr << "Помилка ініціалізації LoRa: " << e.what() << std::endl;
            return false;
        }
    }

    bool CommunicationSystem::InitializeMesh() {
        // Ініціалізація mesh мережі як fallback
        // Використовує інший ELRS модуль або окремий канал

        std::cout << "Mesh мережа ініціалізована як fallback" << std::endl;
        return true;
    }

    bool CommunicationSystem::Start() {
        if (!is_healthy_) {
            return false;
        }

        is_running_ = true;

        // Запуск потоків моніторингу
        std::thread signal_monitor_thread(&CommunicationSystem::MonitorSignalQuality, this);
        signal_monitor_thread.detach();

        std::thread frequency_hopping_thread(&CommunicationSystem::HandleFrequencyHopping, this);
        frequency_hopping_thread.detach();

        std::cout << "Система зв'язку запущена" << std::endl;
        return true;
    }

    bool CommunicationSystem::Stop() {
        is_running_ = false;
        is_healthy_ = false;

        std::cout << "Система зв'язку зупинена" << std::endl;
        return true;
    }

    void CommunicationSystem::Update() {
        if (!is_running_) return;

        // Основний цикл обробки повідомлень
        // Виконується в основному потоці

        // Перевірка стану зв'язку
        auto status = GetStatus();
        if (status == CommunicationStatus::LOST) {
            is_healthy_ = false;
        }
    }

    void CommunicationSystem::MonitorSignalQuality() {
        while (is_running_) {
            // Читання RSSI з обладнання
            int current_rssi = ReadRSSI(); // Фіктивна функція
            current_rssi_.store(current_rssi);

            // Перевірка на заглушування
            if (DetectJamming()) {
                jamming_detected_ = true;
                std::cout << "УВАГА: Виявлено заглушування сигналу!" << std::endl;
            } else {
                jamming_detected_ = false;
            }

            // Адаптивна зміна потужності
            if (current_rssi < comm_config_.rssi_threshold) {
                int new_power = std::min(30, comm_config_.power_level + 2);
                if (new_power != comm_config_.power_level) {
                    AdjustPower(new_power);
                    std::cout << "Збільшення потужності до " << new_power << " dBm" << std::endl;
                }
            } else if (current_rssi > comm_config_.rssi_threshold + 10) {
                int new_power = std::max(0, comm_config_.power_level - 1);
                if (new_power != comm_config_.power_level) {
                    AdjustPower(new_power);
                    std::cout << "Зменшення потужності до " << new_power << " dBm" << std::endl;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void CommunicationSystem::HandleFrequencyHopping() {
        auto hop_interval = config_->GetValue<int>("lora.frequency_hopping.hop_interval", 5000);

        while (is_running_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(hop_interval));

            if (jamming_detected_) {
                // Швидке переключення при заглушуванні
                uint32_t new_freq = SelectBestFrequency();
                if (new_freq != comm_config_.current_frequency) {
                    ChangeFrequency(new_freq);
                    std::cout << "Переключення частоти через заглушування: "
                              << new_freq << " Hz" << std::endl;
                }
            }
        }
    }

    bool CommunicationSystem::DetectJamming() {
        // Простий алгоритм виявлення заглушування
        int rssi = current_rssi_.load();

        // Якщо сигнал занадто сильний і немає корисних даних
        if (rssi > -50) {
            // Можливо заглушування
            return true;
        }

        // Якщо рівень шуму перевищує поріг
        double noise_floor = MeasureNoiseFloor(); // Фіктивна функція
        if (noise_floor > comm_config_.noise_threshold) {
            return true;
        }

        return false;
    }

    bool CommunicationSystem::SendMessage(DroneID target, const std::vector<uint8_t>& data) {
        if (!is_running_ || !is_healthy_) {
            return false;
        }

        LoRaMessage message;
        message.message_type = static_cast<uint8_t>(MessageType::COMMAND);
        message.sender_id = config_->GetValue<DroneID>("system.my_drone_id", 0);
        message.target_id = target;
        message.payload_size = std::min(data.size(), sizeof(message.payload));

        std::copy(data.begin(), data.begin() + message.payload_size, message.payload);

        // Розрахунок контрольної суми
        message.checksum = CalculateChecksum(&message);

        // Відправка через LoRa
        return TransmitLoRaMessage(&message);
    }

    bool CommunicationSystem::BroadcastMessage(const std::vector<uint8_t>& data) {
        return SendMessage(0, data); // 0 = broadcast
    }

    bool CommunicationSystem::ReceiveMessage(std::vector<uint8_t>& data, DroneID& sender) {
        LoRaMessage message;

        if (!ReceiveLoRaMessage(&message)) {
            return false;
        }

        // Перевірка контрольної суми
        if (message.checksum != CalculateChecksum(&message)) {
            std::cerr << "Помилка контрольної суми повідомлення" << std::endl;
            return false;
        }

        sender = message.sender_id;
        data.assign(message.payload, message.payload + message.payload_size);

        return true;
    }

    bool CommunicationSystem::ChangeFrequency(uint32_t new_frequency) {
        // Перевірка, чи частота в дозволеному списку
        auto& frequencies = comm_config_.lora_frequencies;
        if (std::find(frequencies.begin(), frequencies.end(), new_frequency) == frequencies.end()) {
            std::cerr << "Недозволена частота: " << new_frequency << std::endl;
            return false;
        }

        // Зміна частоти обладнання
        if (SetLoRaFrequency(new_frequency)) {
            comm_config_.current_frequency = new_frequency;

            // Повідомлення іншим дронам про зміну частоти
            std::vector<uint8_t> freq_data(sizeof(uint32_t));
            memcpy(freq_data.data(), &new_frequency, sizeof(uint32_t));

            LoRaMessage freq_message;
            freq_message.message_type = static_cast<uint8_t>(MessageType::FREQUENCY_CHANGE);
            freq_message.target_id = 0; // Broadcast
            freq_message.payload_size = freq_data.size();
            memcpy(freq_message.payload, freq_data.data(), freq_data.size());

            TransmitLoRaMessage(&freq_message);

            return true;
        }

        return false;
    }

    bool CommunicationSystem::AdjustPower(int8_t new_power) {
        // Обмеження потужності
        new_power = std::max(static_cast<int8_t>(0),
                             std::min(static_cast<int8_t>(30), new_power));

        if (SetLoRaPower(new_power)) {
            comm_config_.power_level = new_power;
            return true;
        }

        return false;
    }

    void CommunicationSystem::EnableMesh(bool enable) {
        comm_config_.mesh_enabled = enable;

        if (enable) {
            std::cout << "Mesh мережа увімкнена" << std::endl;
        } else {
            std::cout << "Mesh мережа вимкнена" << std::endl;
        }
    }

    CommunicationStatus CommunicationSystem::GetStatus() const {
        int rssi = current_rssi_.load();

        if (rssi > -70) return CommunicationStatus::EXCELLENT;
        if (rssi > -85) return CommunicationStatus::GOOD;
        if (rssi > -100) return CommunicationStatus::POOR;
        if (rssi > -120) return CommunicationStatus::CRITICAL;

        return CommunicationStatus::LOST;
    }

// Допоміжні функції (заглушки для реального обладнання)
    int CommunicationSystem::ReadRSSI() {
        // Тут має бути код читання RSSI з ELRS модуля
        // Поки що повертаємо симульовані дані
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_int_distribution<> dis(-120, -50);

        return dis(gen);
    }

    double CommunicationSystem::MeasureNoiseFloor() {
        // Вимірювання рівня шуму
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_real_distribution<> dis(-95.0, -85.0);

        return dis(gen);
    }

    uint32_t CommunicationSystem::SelectBestFrequency() {
        // Вибір найкращої частоти на основі вимірювань
        uint32_t best_freq = comm_config_.current_frequency;
        int best_rssi = -120;

        for (auto freq : comm_config_.lora_frequencies) {
            if (freq == comm_config_.current_frequency) continue;

            // Швидке тестування частоти
            int test_rssi = TestFrequency(freq);
            if (test_rssi > best_rssi) {
                best_rssi = test_rssi;
                best_freq = freq;
            }
        }

        return best_freq;
    }

    int CommunicationSystem::TestFrequency(uint32_t frequency) {
        // Швидкий тест частоти
        // Тут має бути код реального тестування
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_int_distribution<> dis(-110, -60);

        return dis(gen);
    }

    uint32_t CommunicationSystem::CalculateChecksum(const LoRaMessage* message) {
        // Простий CRC32
        uint32_t crc = 0xFFFFFFFF;
        const uint8_t* data = reinterpret_cast<const uint8_t*>(message);
        size_t size = sizeof(LoRaMessage) - sizeof(message->checksum);

        for (size_t i = 0; i < size; ++i) {
            crc ^= data[i];
            for (int j = 0; j < 8; ++j) {
                crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320 : 0);
            }
        }

        return ~crc;
    }

    bool CommunicationSystem::TransmitLoRaMessage(const LoRaMessage* message) {
        // Тут має бути код відправки через ELRS
        // Поки що заглушка
        std::cout << "TX: " << static_cast<int>(message->message_type)
                  << " від " << message->sender_id
                  << " до " << message->target_id << std::endl;
        return true;
    }

    bool CommunicationSystem::ReceiveLoRaMessage(LoRaMessage* message) {
        // Тут має бути код прийому через ELRS
        // Поки що заглушка
        return false;
    }

    bool CommunicationSystem::SetLoRaFrequency(uint32_t frequency) {
        // Тут має бути код зміни частоти ELRS модуля
        std::cout << "Зміна частоти на " << frequency << " Hz" << std::endl;
        return true;
    }

    bool CommunicationSystem::SetLoRaPower(int8_t power) {
        // Тут має бути код зміни потужності ELRS модуля
        std::cout << "Зміна потужності на " << static_cast<int>(power) << " dBm" << std::endl;
        return true;
    }

} // namespace SwarmSystem