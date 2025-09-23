#include "include/SwarmSystem.h"
#include <algorithm>
#include <thread>
#include <chrono>

namespace SwarmSystem {

// Повідомлення для управління лідерством
    struct LeadershipMessage {
        enum Type {
            HEARTBEAT = 1,
            ELECTION_REQUEST = 2,
            ELECTION_RESPONSE = 3,
            LEADER_TRANSFER = 4,
            LEADER_CONFIRMATION = 5,
            ATTACK_MODE_ENGAGED = 6
        };

        Type message_type;
        DroneID sender_id;
        DroneID target_leader_id;    // Пропонований лідер
        double leadership_score;     // Оцінка придатності
        uint64_t timestamp;

        // Дані для оцінки лідерства
        double battery_level;
        double signal_strength;
        double system_health_score;
        double position_accuracy;
        bool is_attack_capable;      // Чи може атакувати ціль

        LeadershipMessage() {
            message_type = HEARTBEAT;
            sender_id = 0;
            target_leader_id = 0;
            leadership_score = 0.0;
            timestamp = 0;
            battery_level = 0.0;
            signal_strength = 0.0;
            system_health_score = 0.0;
            position_accuracy = 0.0;
            is_attack_capable = true;
        }
    };

    LeadershipManager::LeadershipManager(std::shared_ptr<ConfigManager> config)
            : SwarmSubsystem(config), current_leader_(0) {

        last_leader_heartbeat_ = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch());
    }

    bool LeadershipManager::Initialize() {
        DroneID my_id = config_->GetValue<DroneID>("system.my_drone_id", 0);

        // Ініціалізація власного стану
        DroneState my_state;
        my_state.id = my_id;
        my_state.role = DroneRole::FOLLOWER;
        my_state.battery_level = 100.0;
        my_state.temperature = 25.0;
        my_state.comm_status = CommunicationStatus::GOOD;
        my_state.is_healthy = true;
        my_state.last_update = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch());

        drone_states_[my_id] = my_state;

        is_healthy_ = true;
        std::cout << "LeadershipManager ініціалізований для дрона " << my_id << std::endl;
        return true;
    }

    bool LeadershipManager::Start() {
        if (!is_healthy_) {
            return false;
        }

        is_running_ = true;

        // Запуск потоку моніторингу лідерства
        std::thread leadership_thread(&LeadershipManager::LeadershipWorker, this);
        leadership_thread.detach();

        // Запуск потоку heartbeat
        std::thread heartbeat_thread(&LeadershipManager::HeartbeatWorker, this);
        heartbeat_thread.detach();

        std::cout << "LeadershipManager запущений" << std::endl;
        return true;
    }

    bool LeadershipManager::Stop() {
        is_running_ = false;
        is_healthy_ = false;

        std::cout << "LeadershipManager зупинений" << std::endl;
        return true;
    }

    void LeadershipManager::Update() {
        if (!is_running_) return;

        // Перевірка стану поточного лідера
        CheckLeaderHealth();

        // Обновлення власного стану
        UpdateMyDroneState();

        // Очищення застарілих записів
        CleanupOldStates();
    }

    void LeadershipManager::LeadershipWorker() {
        auto check_interval = std::chrono::milliseconds(
                config_->GetValue<int>("leadership.heartbeat_interval", 500));

        while (is_running_) {
            if (current_leader_ == 0) {
                // Немає лідера - починаємо вибори
                InitiateLeaderElection();
            } else if (!IsLeaderHealthy()) {
                std::cout << "Лідер " << current_leader_ << " недоступний, починаємо нові вибори" << std::endl;
                InitiateLeaderElection();
            }

            std::this_thread::sleep_for(check_interval);
        }
    }

    void LeadershipManager::HeartbeatWorker() {
        DroneID my_id = config_->GetValue<DroneID>("system.my_drone_id", 0);
        auto heartbeat_interval = std::chrono::milliseconds(
                config_->GetValue<int>("leadership.heartbeat_interval", 500));

        while (is_running_) {
            if (current_leader_ == my_id) {
                // Я лідер - відправляю heartbeat
                SendLeaderHeartbeat();
            }

            std::this_thread::sleep_for(heartbeat_interval);
        }
    }

    void LeadershipManager::InitiateLeaderElection() {
        std::cout << "Ініціація виборів лідера" << std::endl;

        DroneID my_id = config_->GetValue<DroneID>("system.my_drone_id", 0);

        // Розсилаємо запит на вибори
        LeadershipMessage election_msg;
        election_msg.message_type = LeadershipMessage::ELECTION_REQUEST;
        election_msg.sender_id = my_id;
        election_msg.leadership_score = CalculateLeadershipScore(my_id);
        election_msg.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();

        // Заповнюємо дані для оцінки
        if (drone_states_.find(my_id) != drone_states_.end()) {
            const auto& my_state = drone_states_[my_id];
            election_msg.battery_level = my_state.battery_level;
            election_msg.signal_strength = GetSignalStrength(); // З CommunicationSystem
            election_msg.system_health_score = GetSystemHealth();
            election_msg.position_accuracy = GetPositionAccuracy(); // З PositioningSystem
            election_msg.is_attack_capable = true; // Поки всі дрони можуть атакувати
        }

        BroadcastLeadershipMessage(election_msg);

        // Чекаємо відповіді від інших дронів
        std::this_thread::sleep_for(std::chrono::milliseconds(
                config_->GetValue<int>("leadership.election_timeout", 3000)));

        // Вибираємо найкращого кандидата
        DroneID new_leader = ElectNewLeader();
        if (new_leader != 0) {
            SetLeader(new_leader);
        }
    }

    DroneID LeadershipManager::ElectNewLeader() {
        std::vector<std::pair<DroneID, double>> candidates;

        // Збираємо всіх здорових кандидатів
        for (const auto& [drone_id, state] : drone_states_) {
            if (state.is_healthy && ValidateLeaderCandidate(drone_id)) {
                double score = CalculateLeadershipScore(drone_id);
                candidates.emplace_back(drone_id, score);
            }
        }

        if (candidates.empty()) {
            std::cerr << "Немає придатних кандидатів для лідерства!" << std::endl;
            return 0;
        }

        // Сортуємо за оцінкою (найвища оцінка = найкращий лідер)
        std::sort(candidates.begin(), candidates.end(),
                  [](const auto& a, const auto& b) { return a.second > b.second; });

        DroneID best_candidate = candidates[0].first;
        double best_score = candidates[0].second;

        std::cout << "Вибраний новий лідер: дрон " << best_candidate
                  << " з оцінкою " << best_score << std::endl;

        return best_candidate;
    }

    double LeadershipManager::CalculateLeadershipScore(DroneID drone_id) const {
        auto it = drone_states_.find(drone_id);
        if (it == drone_states_.end()) {
            return 0.0;
        }

        const auto& state = it->second;

        // Вагові коефіцієнти для різних критеріїв
        const double BATTERY_WEIGHT = 0.3;
        const double SIGNAL_WEIGHT = 0.25;
        const double HEALTH_WEIGHT = 0.25;
        const double POSITION_WEIGHT = 0.2;

        double score = 0.0;

        // Рівень батареї (0-100)
        score += (state.battery_level / 100.0) * BATTERY_WEIGHT;

        // Якість сигналу (нормалізуємо RSSI від -120 до -50)
        double signal_quality = 0.0;
        if (state.comm_status != CommunicationStatus::LOST) {
            int rssi = GetDroneRSSI(drone_id); // Функція отримання RSSI
            signal_quality = std::max(0.0, std::min(1.0, (rssi + 120.0) / 70.0));
        }
        score += signal_quality * SIGNAL_WEIGHT;

        // Здоров'я системи
        double health_score = state.is_healthy ? 1.0 : 0.0;
        if (state.temperature > 80.0) health_score *= 0.5; // Перегрів
        score += health_score * HEALTH_WEIGHT;

        // Точність позиціонування (чим менше помилка, тим краще)
        double position_score = 1.0; // Заглушка, має братися з PositioningSystem
        score += position_score * POSITION_WEIGHT;

        // Бонус за стабільність (дрон давно в мережі)
        auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch());
        auto time_in_network = now - state.last_update;
        if (time_in_network.count() < 60000) { // Менше хвилини
            score *= 1.1; // 10% бонус за стабільність
        }

        return score;
    }

    bool LeadershipManager::ValidateLeaderCandidate(DroneID drone_id) const {
        auto it = drone_states_.find(drone_id);
        if (it == drone_states_.end()) {
            return false;
        }

        const auto& state = it->second;

        // Базові вимоги до лідера
        if (!state.is_healthy) return false;
        if (state.battery_level < config_->GetValue<double>("health.battery_critical", 20.0)) return false;
        if (state.comm_status == CommunicationStatus::LOST) return false;
        if (state.role == DroneRole::SELF_DESTRUCT) return false;

        return true;
    }

    bool LeadershipManager::SetLeader(DroneID drone_id) {
        if (!ValidateLeaderCandidate(drone_id)) {
            std::cerr << "Дрон " << drone_id << " не може бути лідером" << std::endl;
            return false;
        }

        DroneID old_leader = current_leader_;
        current_leader_ = drone_id;

        // Оновлюємо ролі
        if (drone_states_.find(old_leader) != drone_states_.end()) {
            drone_states_[old_leader].role = DroneRole::FOLLOWER;
        }

        if (drone_states_.find(drone_id) != drone_states_.end()) {
            drone_states_[drone_id].role = DroneRole::LEADER;
        }

        // Повідомляємо всім про нового лідера
        LeadershipMessage confirm_msg;
        confirm_msg.message_type = LeadershipMessage::LEADER_CONFIRMATION;
        confirm_msg.sender_id = config_->GetValue<DroneID>("system.my_drone_id", 0);
        confirm_msg.target_leader_id = drone_id;
        confirm_msg.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();

        BroadcastLeadershipMessage(confirm_msg);

        std::cout << "Новий лідер встановлений: дрон " << drone_id << std::endl;

        // Оновлюємо час останнього heartbeat
        last_leader_heartbeat_ = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch());

        return true;
    }

    bool LeadershipManager::ProcessHeartbeat(DroneID drone_id) {
        if (drone_id == current_leader_) {
            last_leader_heartbeat_ = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch());
            return true;
        }

        return false;
    }

    bool LeadershipManager::IsLeaderHealthy() const {
        if (current_leader_ == 0) return false;

        auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch());

        auto timeout = std::chrono::milliseconds(
                config_->GetValue<int>("leadership.leader_lost_timeout", 2000));

        return (now - last_leader_heartbeat_) < timeout;
    }

    bool LeadershipManager::UpdateDroneState(const DroneState& state) {
        std::lock_guard<std::mutex> lock(states_mutex_);

        drone_states_[state.id] = state;
        drone_states_[state.id].last_update = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch());

        return true;
    }

    DroneState LeadershipManager::GetDroneState(DroneID drone_id) const {
        std::lock_guard<std::mutex> lock(states_mutex_);

        auto it = drone_states_.find(drone_id);
        if (it != drone_states_.end()) {
            return it->second;
        }

        return DroneState(); // Повертаємо порожній стан
    }

    std::vector<DroneID> LeadershipManager::GetHealthyDrones() const {
        std::lock_guard<std::mutex> lock(states_mutex_);

        std::vector<DroneID> healthy_drones;

        for (const auto& [drone_id, state] : drone_states_) {
            if (state.is_healthy && state.comm_status != CommunicationStatus::LOST) {
                healthy_drones.push_back(drone_id);
            }
        }

        return healthy_drones;
    }

// Швидка передача лідерства для атаки (ID+1 алгоритм)
    bool LeadershipManager::TransferLeadershipForAttack(DroneID attacking_drone, DroneID manual_next_leader = 0) {
        if (attacking_drone != current_leader_) {
            std::cerr << "Тільки поточний лідер може передати лідерство для атаки" << std::endl;
            return false;
        }

        DroneID new_leader = 0;

        if (manual_next_leader != 0) {
            // Ручний вибір через OSD
            if (ValidateLeaderCandidate(manual_next_leader)) {
                new_leader = manual_next_leader;
                std::cout << "Ручний вибір нового лідера: " << new_leader << std::endl;
            } else {
                std::cerr << "Ручно вибраний дрон " << manual_next_leader << " не придатний" << std::endl;
            }
        }

        if (new_leader == 0) {
            // Автоматичний вибір: наступний ID після поточного лідера
            new_leader = FindNextAvailableLeader(attacking_drone);
        }

        if (new_leader == 0) {
            std::cerr << "Немає придатних кандидатів для передачі лідерства!" << std::endl;
            return false;
        }

        // ШВИДКА передача лідерства - мінімум повідомлень
        auto start_time = std::chrono::high_resolution_clock::now();

        // 1. Одразу повідомляємо всім про передачу (одне повідомлення)
        LeadershipMessage urgent_transfer;
        urgent_transfer.message_type = LeadershipMessage::URGENT_LEADER_TRANSFER;
        urgent_transfer.sender_id = attacking_drone;
        urgent_transfer.target_leader_id = new_leader;
        urgent_transfer.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();

        BroadcastLeadershipMessage(urgent_transfer);

        // 2. Локально змінюємо лідера негайно
        SetLeaderInstant(new_leader);

        // 3. Команда роя зависнути на поточних позиціях
        SendHoverCommand();

        // 4. Позначаємо дрон, що атакує, як відключений
        if (drone_states_.find(attacking_drone) != drone_states_.end()) {
            drone_states_[attacking_drone].role = DroneRole::LOST_COMMUNICATION;
            drone_states_[attacking_drone].comm_status = CommunicationStatus::LOST;
        }

        // 5. Активуємо систему доведення через OSD
        ActivateTargetingSystem(attacking_drone);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto transfer_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        std::cout << "ШВИДКА передача лідерства завершена за " << transfer_duration.count()
                  << " мс. Дрон " << attacking_drone << " → дрон " << new_leader << std::endl;

        return true;
    }

// Пошук наступного доступного лідера (ID+1 алгоритм)
    DroneID LeadershipManager::FindNextAvailableLeader(DroneID current_leader) {
        std::vector<DroneID> available_drones;

        // Збираємо всіх здорових дронів
        for (const auto& [drone_id, state] : drone_states_) {
            if (drone_id != current_leader &&
                state.is_healthy &&
                ValidateLeaderCandidate(drone_id)) {
                available_drones.push_back(drone_id);
            }
        }

        if (available_drones.empty()) {
            return 0;
        }

        // Сортуємо по ID
        std::sort(available_drones.begin(), available_drones.end());

        // Шукаємо найближчий більший ID
        for (DroneID candidate : available_drones) {
            if (candidate > current_leader) {
                return candidate;
            }
        }

        // Якщо немає більшого ID, беремо найменший (циклічно)
        return available_drones[0];
    }

// Миттєва зміна лідера без додаткових перевірок
    bool LeadershipManager::SetLeaderInstant(DroneID new_leader) {
        DroneID old_leader = current_leader_;
        current_leader_ = new_leader;

        // Оновлюємо ролі миттєво
        if (drone_states_.find(old_leader) != drone_states_.end()) {
            drone_states_[old_leader].role = DroneRole::FOLLOWER;
        }

        if (drone_states_.find(new_leader) != drone_states_.end()) {
            drone_states_[new_leader].role = DroneRole::LEADER;
        }

        // Оновлюємо час останнього heartbeat
        last_leader_heartbeat_ = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch());

        return true;
    }

// Команда роя зависнути на поточних позиціях
    void LeadershipManager::SendHoverCommand() {
        SwarmCommand hover_cmd;
        hover_cmd.command_type = SwarmCommand::HOVER_IN_PLACE;
        hover_cmd.sender_id = config_->GetValue<DroneID>("system.my_drone_id", 0);
        hover_cmd.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
        hover_cmd.priority = SwarmCommand::URGENT;

        BroadcastSwarmCommand(hover_cmd);

        std::cout << "Команда зависання надіслана всьому рою" << std::endl;
    }

// Активація системи доведення через OSD
    void LeadershipManager::ActivateTargetingSystem(DroneID attacking_drone) {
        // Відправляємо команду активації системи доведення
        OSDCommand targeting_cmd;
        targeting_cmd.command = OSDCommand::ACTIVATE_TARGETING;
        targeting_cmd.target_drone = attacking_drone;
        targeting_cmd.timestamp = std::chrono::high_resolution_clock::now();

        SendOSDCommand(targeting_cmd);

        std::cout << "Система доведення активована для дрона " << attacking_drone << std::endl;
    }

// Допоміжні функції (заглушки)
    void LeadershipManager::SendLeaderHeartbeat() {
        DroneID my_id = config_->GetValue<DroneID>("system.my_drone_id", 0);

        LeadershipMessage heartbeat;
        heartbeat.message_type = LeadershipMessage::HEARTBEAT;
        heartbeat.sender_id = my_id;
        heartbeat.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();

        BroadcastLeadershipMessage(heartbeat);
    }

    void LeadershipManager::BroadcastLeadershipMessage(const LeadershipMessage& msg) {
        // Тут має бути інтеграція з CommunicationSystem
        // Поки заглушка
        std::cout << "Broadcast leadership message type " << msg.message_type
                  << " from drone " << msg.sender_id << std::endl;
    }

    void LeadershipManager::CheckLeaderHealth() {
        if (current_leader_ != 0 && !IsLeaderHealthy()) {
            std::cout << "Лідер " << current_leader_ << " втратив зв'язок" << std::endl;
            current_leader_ = 0; // Скидаємо лідера
        }
    }

    void LeadershipManager::UpdateMyDroneState() {
        DroneID my_id = config_->GetValue<DroneID>("system.my_drone_id", 0);

        if (drone_states_.find(my_id) != drone_states_.end()) {
            auto& my_state = drone_states_[my_id];
            my_state.last_update = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch());

            // Тут має бути оновлення реальних даних з сенсорів
            // my_state.battery_level = GetBatteryLevel();
            // my_state.temperature = GetTemperature();
            // my_state.comm_status = GetCommunicationStatus();
        }
    }

    void LeadershipManager::CleanupOldStates() {
        auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch());

        auto timeout = std::chrono::milliseconds(30000); // 30 секунд

        std::lock_guard<std::mutex> lock(states_mutex_);

        for (auto it = drone_states_.begin(); it != drone_states_.end();) {
            if ((now - it->second.last_update) > timeout) {
                std::cout << "Видаляємо застарілий стан дрона " << it->first << std::endl;
                it = drone_states_.erase(it);
            } else {
                ++it;
            }
        }
    }

// Заглушки для інтеграції з іншими системами
    double LeadershipManager::GetSignalStrength() const {
        return -75.0; // dBm заглушка
    }

    double LeadershipManager::GetSystemHealth() const {
        return 0.95; // 95% здоров'я заглушка
    }

    double LeadershipManager::GetPositionAccuracy() const {
        return 0.1; // 10см точність заглушка
    }

    int LeadershipManager::GetDroneRSSI(DroneID drone_id) const {
        return -80; // dBm заглушка
    }

} // namespace SwarmSystem//
// Created by yv on 22.09.2025.
//
