#include "include/SwarmSystem.h"
#include <thread>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <random>

// Криптографічні бібліотеки
#include <openssl/aes.h>
#include <openssl/rand.h>
#include <openssl/evp.h>

namespace SwarmSystem {

// UWB пакет даних
    struct UWBPacket {
        DroneID sender_id;
        DroneID target_id;
        uint64_t timestamp_ns;    // Час відправки в наносекундах
        uint32_t sequence;        // Номер послідовності
        uint8_t packet_type;      // Тип пакету (ranging, position, etc.)
        uint8_t encrypted_data[64]; // Зашифровані дані
        uint32_t checksum;

        UWBPacket() {
            sender_id = 0;
            target_id = 0;
            timestamp_ns = 0;
            sequence = 0;
            packet_type = 0;
            memset(encrypted_data, 0, sizeof(encrypted_data));
            checksum = 0;
        }
    };

// Тип пакетів UWB
    enum class UWBPacketType : uint8_t {
        RANGING_REQUEST = 1,   // Запит вимірювання відстані
        RANGING_RESPONSE = 2,  // Відповідь з часовою міткою
        POSITION_UPDATE = 3,   // Оновлення позиції
        FORMATION_SYNC = 4     // Синхронізація формації
    };

// Дані вимірювання відстані
    struct RangingData {
        DroneID target_drone;
        double distance_m;
        double accuracy_m;
        std::chrono::high_resolution_clock::time_point timestamp;
        bool is_valid;

        RangingData() : target_drone(0), distance_m(0), accuracy_m(0), is_valid(false) {}
    };

// Криптографічний клас для захисту комунікації
    class CryptoManager {
    private:
        uint8_t aes_key_[32];        // AES-256 ключ
        uint8_t current_iv_[16];     // Поточний IV для AES
        uint32_t message_counter_;   // Лічильник для запобігання replay атак

        bool GenerateSecureRandom(uint8_t* buffer, size_t size);
        bool DeriveKey(const std::string& passphrase, const uint8_t* salt, size_t salt_len);

    public:
        CryptoManager();
        ~CryptoManager();

        bool Initialize(const std::string& shared_secret);
        bool EncryptData(const uint8_t* plaintext, size_t plaintext_len,
                         uint8_t* ciphertext, size_t* ciphertext_len);
        bool DecryptData(const uint8_t* ciphertext, size_t ciphertext_len,
                         uint8_t* plaintext, size_t* plaintext_len);

        uint32_t GetNextMessageCounter() { return ++message_counter_; }
        bool ValidateMessageCounter(uint32_t received_counter);
    };

    PositioningSystem::PositioningSystem(std::shared_ptr<ConfigManager> config)
            : SwarmSubsystem(config), position_accuracy_(0.1) {
        reference_position_ = Position3D(0, 0, 0); // Лідер завжди в нулі
    }

    bool PositioningSystem::Initialize() {
        if (!InitializeUWB()) {
            std::cerr << "Помилка ініціалізації UWB" << std::endl;
            return false;
        }

        // Ініціалізація криптографії
        crypto_manager_ = std::make_unique<CryptoManager>();
        std::string shared_secret = config_->GetValue<std::string>("security.shared_secret", "SlavaUkraini2024!");

        if (!crypto_manager_->Initialize(shared_secret)) {
            std::cerr << "Помилка ініціалізації шифрування" << std::endl;
            return false;
        }

        position_accuracy_ = config_->GetValue<double>("uwb.range_accuracy", 0.1);

        is_healthy_ = true;
        std::cout << "UWB система позиціонування ініціалізована" << std::endl;
        return true;
    }

    bool PositioningSystem::InitializeUWB() {
        try {
            // Ініціалізація UWB модуля на частоті 4.9 GHz
            uint32_t uwb_frequency = config_->GetValue<uint32_t>("uwb.frequency", 4900000000);
            int update_rate = config_->GetValue<int>("uwb.update_rate", 100);

            // Тут має бути код ініціалізації реального UWB обладнання
            // Наприклад, DWM1001 або аналогічного модуля

            std::cout << "UWB ініціалізовано на частоті " << uwb_frequency
                      << " Hz, швидкість оновлення " << update_rate << " Hz" << std::endl;

            return true;
        }
        catch (const std::exception& e) {
            std::cerr << "Помилка ініціалізації UWB: " << e.what() << std::endl;
            return false;
        }
    }

    bool PositioningSystem::Start() {
        if (!is_healthy_) {
            return false;
        }

        is_running_ = true;

        // Запуск потоку вимірювання відстаней
        std::thread ranging_thread(&PositioningSystem::RangingWorker, this);
        ranging_thread.detach();

        // Запуск потоку обчислення позицій
        std::thread position_thread(&PositioningSystem::PositionCalculationWorker, this);
        position_thread.detach();

        std::cout << "UWB система позиціонування запущена" << std::endl;
        return true;
    }

    bool PositioningSystem::Stop() {
        is_running_ = false;
        is_healthy_ = false;

        std::cout << "UWB система позиціонування зупинена" << std::endl;
        return true;
    }

    void PositioningSystem::Update() {
        if (!is_running_) return;

        // Оновлення відносних позицій
        UpdateRelativePositions();

        // Перевірка здоров'я системи
        CheckSystemHealth();
    }

    void PositioningSystem::RangingWorker() {
        auto update_interval = std::chrono::milliseconds(
                1000 / config_->GetValue<int>("uwb.update_rate", 100)
        );

        while (is_running_) {
            // Вимірювання відстаней до всіх дронів в межах досяжності
            MeasureDistancesToAllDrones();

            std::this_thread::sleep_for(update_interval);
        }
    }

    void PositioningSystem::PositionCalculationWorker() {
        while (is_running_) {
            // Обчислення позицій на основі вимірювань
            CalculatePositionsFromRanging();

            std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 20 Hz
        }
    }

    void PositioningSystem::MeasureDistancesToAllDrones() {
        DroneID my_id = config_->GetValue<DroneID>("system.my_drone_id", 0);

        // Отримуємо список активних дронів з менеджера лідерства
        // (це має бути ін'єкція залежності, але поки спрощуємо)
        std::vector<DroneID> active_drones = GetActiveDrones();

        for (DroneID target_id : active_drones) {
            if (target_id == my_id) continue;

            RangingData ranging_result = PerformRanging(target_id);
            if (ranging_result.is_valid) {
                std::lock_guard<std::mutex> lock(ranging_mutex_);
                ranging_data_[target_id] = ranging_result;
            }
        }
    }

    RangingData PositioningSystem::PerformRanging(DroneID target_id) {
        RangingData result;
        result.target_drone = target_id;
        result.timestamp = std::chrono::high_resolution_clock::now();

        try {
            // Відправка ranging запиту
            UWBPacket request_packet;
            request_packet.sender_id = config_->GetValue<DroneID>("system.my_drone_id", 0);
            request_packet.target_id = target_id;
            request_packet.packet_type = static_cast<uint8_t>(UWBPacketType::RANGING_REQUEST);
            request_packet.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    result.timestamp.time_since_epoch()).count();

            // Шифрування пакета
            if (!EncryptUWBPacket(&request_packet)) {
                result.is_valid = false;
                return result;
            }

            // Відправка через UWB
            if (!TransmitUWBPacket(&request_packet)) {
                result.is_valid = false;
                return result;
            }

            // Очікування відповіді (таймаут 10мс)
            UWBPacket response_packet;
            auto timeout = std::chrono::milliseconds(10);

            if (ReceiveUWBPacket(&response_packet, timeout)) {
                // Розшифровка та валідація
                if (DecryptUWBPacket(&response_packet) &&
                    response_packet.sender_id == target_id &&
                    response_packet.packet_type == static_cast<uint8_t>(UWBPacketType::RANGING_RESPONSE)) {

                    // Обчислення відстані на основі ToF (Time of Flight)
                    auto response_time = std::chrono::high_resolution_clock::now();
                    auto flight_time = response_time - result.timestamp;

                    // Швидкість світла ≈ 3×10^8 м/с
                    // Час польоту в обидва боки, тому ділимо на 2
                    double distance = std::chrono::duration<double>(flight_time).count() *
                                      299792458.0 / 2.0;

                    // Обмеження реалістичності
                    if (distance > 0.1 && distance < 1000.0) {
                        result.distance_m = distance;
                        result.accuracy_m = position_accuracy_;
                        result.is_valid = true;
                    }
                }
            }

        }
        catch (const std::exception& e) {
            std::cerr << "Помилка вимірювання відстані до дрона " << target_id
                      << ": " << e.what() << std::endl;
        }

        return result;
    }

    void PositioningSystem::CalculatePositionsFromRanging() {
        std::lock_guard<std::mutex> lock(ranging_mutex_);

        DroneID my_id = config_->GetValue<DroneID>("system.my_drone_id", 0);
        DroneID leader_id = GetCurrentLeader(); // Отримуємо від LeadershipManager

        if (my_id == leader_id) {
            // Лідер завжди в позиції (0, 0, 0)
            drone_positions_[my_id] = Position3D(0, 0, 0);
            return;
        }

        // Для ведомих дронів обчислюємо позицію відносно лідера та інших дронів
        if (ranging_data_.find(leader_id) != ranging_data_.end()) {
            double distance_to_leader = ranging_data_[leader_id].distance_m;

            // Якщо у нас є вимірювання до лідера та ще хоча б двох дронів,
            // можемо обчислити 3D позицію через тріангуляцію
            std::vector<std::pair<DroneID, RangingData>> valid_measurements;

            for (const auto& [drone_id, ranging] : ranging_data_) {
                if (ranging.is_valid && drone_id != my_id) {
                    valid_measurements.emplace_back(drone_id, ranging);
                }
            }

            if (valid_measurements.size() >= 3) {
                Position3D calculated_position = TriangulatePosition(valid_measurements);

                if (ValidatePosition(calculated_position)) {
                    drone_positions_[my_id] = calculated_position;
                }
            } else if (ranging_data_[leader_id].is_valid) {
                // Якщо недостатньо вимірювань для повної тріангуляції,
                // використовуємо попередню позицію та оновлюємо відстань до лідера
                Position3D current_pos = drone_positions_[my_id];
                double current_distance = current_pos.norm();

                if (std::abs(current_distance - distance_to_leader) > 1.0) {
                    // Корегуємо позицію пропорційно
                    if (current_distance > 0.1) {
                        drone_positions_[my_id] = current_pos * (distance_to_leader / current_distance);
                    }
                }
            }
        }
    }

    Position3D PositioningSystem::TriangulatePosition(
            const std::vector<std::pair<DroneID, RangingData>>& measurements) {

        // Спрощена тріангуляція для 3D простору
        // Використовуємо метод найменших квадратів

        if (measurements.size() < 3) {
            return Position3D(0, 0, 0);
        }

        // Беремо позиції відомих дронів
        std::vector<Position3D> known_positions;
        std::vector<double> distances;

        for (const auto& [drone_id, ranging] : measurements) {
            if (drone_positions_.find(drone_id) != drone_positions_.end()) {
                known_positions.push_back(drone_positions_[drone_id]);
                distances.push_back(ranging.distance_m);
            }
        }

        if (known_positions.size() < 3) {
            return Position3D(0, 0, 0);
        }

        // Метод Ньютона-Рафсона для розв'язання системи рівнянь
        Position3D estimated_pos(0, 0, 0);

        for (int iteration = 0; iteration < 10; ++iteration) {
            Eigen::Vector3d gradient(0, 0, 0);
            Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();

            for (size_t i = 0; i < known_positions.size() && i < distances.size(); ++i) {
                Position3D diff = estimated_pos - known_positions[i];
                double calc_distance = diff.norm();
                double error = calc_distance - distances[i];

                if (calc_distance > 1e-6) {
                    Position3D unit_diff = diff / calc_distance;
                    gradient += error * unit_diff;

                    // Спрощена матриця Гессе
                    hessian += unit_diff * unit_diff.transpose();
                }
            }

            if (hessian.determinant() > 1e-6) {
                Position3D delta = hessian.inverse() * gradient;
                estimated_pos -= delta;

                if (delta.norm() < 0.01) break; // Збіжність
            }
        }

        return estimated_pos;
    }

    bool PositioningSystem::ValidatePosition(const Position3D& pos) {
        // Перевірка реалістичності позиції
        double distance_from_origin = pos.norm();
        double max_formation_distance = config_->GetValue<double>("uwb.max_range", 50.0);

        return distance_from_origin <= max_formation_distance;
    }

    void PositioningSystem::UpdateRelativePositions() {
        DroneID leader_id = GetCurrentLeader();
        Position3D leader_pos = drone_positions_[leader_id];

        for (auto& [drone_id, position] : drone_positions_) {
            // Всі позиції відносно лідера
            relative_positions_[drone_id] = position - leader_pos;
        }
    }

    Position3D PositioningSystem::GetPosition(DroneID drone_id) const {
        std::lock_guard<std::mutex> lock(position_mutex_);

        auto it = drone_positions_.find(drone_id);
        if (it != drone_positions_.end()) {
            return it->second;
        }

        return Position3D(0, 0, 0);
    }

    Position3D PositioningSystem::GetRelativePosition(DroneID drone_id, DroneID reference_id) const {
        Position3D drone_pos = GetPosition(drone_id);
        Position3D ref_pos = GetPosition(reference_id);

        return drone_pos - ref_pos;
    }

    bool PositioningSystem::SetReferencePosition(DroneID reference_drone) {
        std::lock_guard<std::mutex> lock(position_mutex_);

        auto it = drone_positions_.find(reference_drone);
        if (it != drone_positions_.end()) {
            reference_position_ = it->second;

            // Перерахунок всіх позицій відносно нової референсної точки
            for (auto& [drone_id, position] : drone_positions_) {
                position -= reference_position_;
            }

            reference_position_ = Position3D(0, 0, 0);
            return true;
        }

        return false;
    }

    bool PositioningSystem::IsInFormation(const std::vector<DroneID>& swarm,
                                          const FormationPattern& pattern,
                                          double tolerance) const {
        if (swarm.size() != pattern.positions.size()) {
            return false;
        }

        for (size_t i = 0; i < swarm.size(); ++i) {
            Position3D current_pos = GetPosition(swarm[i]);
            Position3D target_pos = pattern.positions[i];

            double deviation = (current_pos - target_pos).norm();
            if (deviation > tolerance) {
                return false;
            }
        }

        return true;
    }

// Криптографічні функції (заглушки для повної реалізації)
    bool PositioningSystem::EncryptUWBPacket(UWBPacket* packet) {
        // Тут має бути реальне шифрування через crypto_manager_
        return true;
    }

    bool PositioningSystem::DecryptUWBPacket(UWBPacket* packet) {
        // Тут має бути реальне розшифрування через crypto_manager_
        return true;
    }

// Функції роботи з UWB обладнанням (заглушки)
    bool PositioningSystem::TransmitUWBPacket(const UWBPacket* packet) {
        // Відправка через реальний UWB модуль
        return true;
    }

    bool PositioningSystem::ReceiveUWBPacket(UWBPacket* packet, std::chrono::milliseconds timeout) {
        // Прийом через реальний UWB модуль з таймаутом
        return false; // Поки заглушка
    }

} // namespace SwarmSystem//
// Created by yv on 22.09.2025.
//
