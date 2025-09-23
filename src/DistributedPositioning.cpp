#include "../include/DistributedPositioning.h"
#include "../include/MeshProtocol.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace DistributedPositioning {

    DistributedPositionTracker::DistributedPositionTracker(
            DroneID my_id,
            std::shared_ptr<MeshNetwork::SwarmMeshProtocol> mesh)
            : my_id_(my_id), mesh_protocol_(mesh) {

        // Ініціалізація якоря за замовчуванням
        current_anchor_.type = AnchorType::NONE;
        current_anchor_.is_valid = false;

        std::cout << "🗺️ Ініціалізація розподіленого трекера позицій для дрона " << my_id_ << std::endl;
    }

    bool DistributedPositionTracker::UpdateMyPosition(const Position3D& measured_position) {
        // Валідація вимірювання
        if (!ValidatePositionUpdate(my_id_, measured_position)) {
            std::cout << "⚠️ Невалідне вимірювання позиції: ("
                      << measured_position.x << ", " << measured_position.y
                      << ", " << measured_position.z << ")" << std::endl;
            return false;
        }

        // Оновлюємо власну позицію
        relative_positions_[my_id_] = measured_position;

        // Додаємо до історії
        uint32_t current_time = GetCurrentTime();
        position_histories_[my_id_].AddPosition(measured_position, current_time);

        // Оновлюємо швидкість
        drone_velocities_[my_id_] = EstimateVelocity(my_id_);

        // Розсилаємо оновлення через mesh
        BroadcastMyPosition();

        // Перевіряємо чи потрібно оновити якір
        if (ShouldUpdateAnchor()) {
            UpdateAnchor();
        }

        std::cout << "📍 Позицію оновлено: ("
                  << measured_position.x << ", " << measured_position.y
                  << ", " << measured_position.z << ")" << std::endl;

        return true;
    }

    bool DistributedPositionTracker::ProcessPositionUpdate(DroneID drone_id, const Position3D& position) {
        if (drone_id == my_id_) {
            return true; // Ігноруємо власні повідомлення
        }

        // Валідація
        if (!ValidatePositionUpdate(drone_id, position)) {
            std::cout << "⚠️ Невалідне оновлення від дрона " << drone_id << std::endl;
            return false;
        }

        // Оновлюємо позицію
        relative_positions_[drone_id] = position;

        // Додаємо до історії
        uint32_t current_time = GetCurrentTime();
        position_histories_[drone_id].AddPosition(position, current_time);

        // Оновлюємо швидкість
        drone_velocities_[drone_id] = EstimateVelocity(drone_id);

        std::cout << "📍 Отримано позицію дрона " << drone_id << ": ("
                  << position.x << ", " << position.y << ", " << position.z << ")" << std::endl;

        return true;
    }

    bool DistributedPositionTracker::EstablishAnchor(AnchorType type) {
        AnchorPoint new_anchor;
        new_anchor.type = type;
        new_anchor.established_time = GetCurrentTime();
        new_anchor.last_update = new_anchor.established_time;

        switch (type) {
            case AnchorType::GEOMETRIC_CENTER:
                new_anchor.position = CalculateGeometricCenter();
                new_anchor.anchor_drone_id = 0; // Немає конкретного дрона
                new_anchor.stability_score = CalculateGeometricCenterStability();
                break;

            case AnchorType::MOST_STABLE_DRONE:
            {
                DroneID stable_drone = FindMostStableDrone();
                if (stable_drone != 0) {
                    new_anchor.position = relative_positions_[stable_drone];
                    new_anchor.anchor_drone_id = stable_drone;
                    new_anchor.stability_score = CalculateStabilityScore(stable_drone);
                } else {
                    // Fallback до геометричного центру
                    return EstablishAnchor(AnchorType::GEOMETRIC_CENTER);
                }
            }
                break;

            case AnchorType::CONSENSUS_ANCHOR:
                // Запускаємо процес консенсусу
                return ProposeNewAnchor(new_anchor);

            default:
                std::cout << "❌ Невідомий тип якоря: " << static_cast<int>(type) << std::endl;
                return false;
        }

        // Перевіряємо якість нового якоря
        if (new_anchor.stability_score > 0.5) {
            current_anchor_ = new_anchor;
            current_anchor_.is_valid = true;

            std::cout << "⚓ Встановлено новий якір типу " << static_cast<int>(type)
                      << " з стабільністю " << new_anchor.stability_score << std::endl;

            // Перераховуємо всі позиції відносно нового якоря
            RecalculateRelativePositions();

            return true;
        } else {
            std::cout << "❌ Якір недостатньо стабільний: " << new_anchor.stability_score << std::endl;
            return false;
        }
    }

    bool DistributedPositionTracker::UpdateAnchor() {
        if (!current_anchor_.is_valid) {
            return EstablishAnchor(AnchorType::GEOMETRIC_CENTER);
        }

        // Перевіряємо поточний якір
        if (!ValidateCurrentAnchor()) {
            std::cout << "⚠️ Поточний якір став невалідним - встановлюємо новий" << std::endl;

            // Спробуємо той же тип якоря
            return EstablishAnchor(current_anchor_.type);
        }

        // Оновлюємо якір залежно від типу
        switch (current_anchor_.type) {
            case AnchorType::GEOMETRIC_CENTER:
                current_anchor_.position = CalculateGeometricCenter();
                current_anchor_.stability_score = CalculateGeometricCenterStability();
                break;

            case AnchorType::MOST_STABLE_DRONE:
                if (current_anchor_.anchor_drone_id != 0) {
                    auto pos_it = relative_positions_.find(current_anchor_.anchor_drone_id);
                    if (pos_it != relative_positions_.end()) {
                        current_anchor_.position = pos_it->second;
                        current_anchor_.stability_score = CalculateStabilityScore(current_anchor_.anchor_drone_id);
                    } else {
                        // Дрон-якір втрачений
                        return EstablishAnchor(AnchorType::MOST_STABLE_DRONE);
                    }
                }
                break;

            default:
                break;
        }

        current_anchor_.last_update = GetCurrentTime();

        std::cout << "⚓ Якір оновлено, стабільність: " << current_anchor_.stability_score << std::endl;
        return true;
    }

    bool DistributedPositionTracker::ValidateCurrentAnchor() {
        if (!current_anchor_.is_valid) {
            return false;
        }

        // Перевірка за часом - якір не повинен бути занадто старим
        uint32_t current_time = GetCurrentTime();
        if (current_time - current_anchor_.last_update > 30000) { // 30 секунд
            std::cout << "⚠️ Якір застарів (" << (current_time - current_anchor_.last_update) << "мс)" << std::endl;
            return false;
        }

        // Перевірка стабільності
        if (current_anchor_.stability_score < 0.3) {
            std::cout << "⚠️ Якір нестабільний: " << current_anchor_.stability_score << std::endl;
            return false;
        }

        // Для якоря-дрона - перевіряємо чи дрон ще існує
        if (current_anchor_.type == AnchorType::MOST_STABLE_DRONE) {
            if (current_anchor_.anchor_drone_id != 0) {
                auto pos_it = relative_positions_.find(current_anchor_.anchor_drone_id);
                if (pos_it == relative_positions_.end()) {
                    std::cout << "⚠️ Дрон-якір " << current_anchor_.anchor_drone_id << " втрачений" << std::endl;
                    return false;
                }
            }
        }

        return true;
    }

    Position3D DistributedPositionTracker::GetMyRelativePosition() const {
        auto it = relative_positions_.find(my_id_);
        if (it != relative_positions_.end() && current_anchor_.is_valid) {
            return it->second - current_anchor_.position;
        }
        return Position3D(0, 0, 0);
    }

    Position3D DistributedPositionTracker::GetDroneRelativePosition(DroneID drone_id) const {
        auto it = relative_positions_.find(drone_id);
        if (it != relative_positions_.end() && current_anchor_.is_valid) {
            return it->second - current_anchor_.position;
        }
        return Position3D(0, 0, 0);
    }

    Position3D DistributedPositionTracker::GetSwarmCenter() const {
        return CalculateGeometricCenter();
    }

    bool DistributedPositionTracker::SynchronizeWithMesh() {
        if (!mesh_protocol_) {
            return false;
        }

        // Отримуємо позиції всіх дронів через mesh
        auto mesh_positions = mesh_protocol_->GetSwarmPositions();

        for (const auto& [drone_id, position] : mesh_positions) {
            if (drone_id != my_id_) {
                ProcessPositionUpdate(drone_id, position);
            }
        }

        std::cout << "🔄 Синхронізовано позиції " << mesh_positions.size() << " дронів через mesh" << std::endl;
        return true;
    }

    bool DistributedPositionTracker::BroadcastMyPosition() {
        if (!mesh_protocol_) {
            return false;
        }

        auto my_pos_it = relative_positions_.find(my_id_);
        if (my_pos_it != relative_positions_.end()) {
            return mesh_protocol_->UpdatePositionInSwarm(my_pos_it->second);
        }

        return false;
    }

    DroneID DistributedPositionTracker::FindMostStableDrone() {
        DroneID most_stable = 0;
        double best_stability = -1.0;

        for (const auto& [drone_id, position] : relative_positions_) {
            double stability = CalculateStabilityScore(drone_id);
            if (stability > best_stability) {
                best_stability = stability;
                most_stable = drone_id;
            }
        }

        std::cout << "🎯 Найстабільніший дрон: " << most_stable
                  << " (стабільність: " << best_stability << ")" << std::endl;

        return most_stable;
    }

    Position3D DistributedPositionTracker::CalculateGeometricCenter() {
        if (relative_positions_.empty()) {
            return Position3D(0, 0, 0);
        }

        Position3D center(0, 0, 0);
        for (const auto& [drone_id, position] : relative_positions_) {
            center.x += position.x;
            center.y += position.y;
            center.z += position.z;
        }

        size_t count = relative_positions_.size();
        center.x /= count;
        center.y /= count;
        center.z /= count;

        return center;
    }

    double DistributedPositionTracker::CalculateGeometricCenterStability() {
        if (relative_positions_.size() < 3) {
            return 0.1; // Недостатньо дронів для стабільного центру
        }

        // Обчислюємо дисперсію позицій відносно центру
        Position3D center = CalculateGeometricCenter();
        double variance = 0.0;

        for (const auto& [drone_id, position] : relative_positions_) {
            double dist_sq = pow(position.x - center.x, 2) +
                             pow(position.y - center.y, 2) +
                             pow(position.z - center.z, 2);
            variance += dist_sq;
        }

        variance /= relative_positions_.size();

        // Перетворюємо дисперсію в оцінку стабільності (0-1)
        double stability = 1.0 / (1.0 + variance / 100.0); // Нормалізація

        return std::min(1.0, std::max(0.0, stability));
    }

    double DistributedPositionTracker::CalculateStabilityScore(DroneID drone_id) {
        auto history_it = position_histories_.find(drone_id);
        if (history_it == position_histories_.end()) {
            return 0.0;
        }

        return history_it->second.GetStabilityScore();
    }

    Velocity3D DistributedPositionTracker::EstimateVelocity(DroneID drone_id) {
        auto history_it = position_histories_.find(drone_id);
        if (history_it == position_histories_.end() ||
            history_it->second.positions.size() < 2) {
            return Velocity3D(0, 0, 0);
        }

        const auto& positions = history_it->second.positions;
        const auto& timestamps = history_it->second.timestamps;

        // Беремо останні дві позиції
        size_t size = positions.size();
        const Position3D& pos1 = positions[size - 2];
        const Position3D& pos2 = positions[size - 1];
        uint32_t time1 = timestamps[size - 2];
        uint32_t time2 = timestamps[size - 1];

        double dt = (time2 - time1) / 1000.0; // Переводимо в секунди
        if (dt <= 0.0) {
            return Velocity3D(0, 0, 0);
        }

        Velocity3D velocity;
        velocity.x = (pos2.x - pos1.x) / dt;
        velocity.y = (pos2.y - pos1.y) / dt;
        velocity.z = (pos2.z - pos1.z) / dt;

        return velocity;
    }

    bool DistributedPositionTracker::ValidatePositionUpdate(DroneID drone_id, const Position3D& position) {
        // Перевірка на розумні межі
        const double MAX_COORDINATE = 10000.0; // 10 км
        if (abs(position.x) > MAX_COORDINATE ||
            abs(position.y) > MAX_COORDINATE ||
            abs(position.z) > MAX_COORDINATE) {
            return false;
        }

        // Перевірка на різкі стрибки позиції
        auto prev_pos_it = relative_positions_.find(drone_id);
        if (prev_pos_it != relative_positions_.end()) {
            const Position3D& prev_pos = prev_pos_it->second;
            double distance = sqrt(pow(position.x - prev_pos.x, 2) +
                                   pow(position.y - prev_pos.y, 2) +
                                   pow(position.z - prev_pos.z, 2));

            const double MAX_POSITION_JUMP = 100.0; // 100 метрів максимум
            if (distance > MAX_POSITION_JUMP) {
                std::cout << "⚠️ Занадто великий стрибок позиції дрона " << drone_id
                          << ": " << distance << "м" << std::endl;
                return false;
            }
        }

        return true;
    }

    void DistributedPositionTracker::RecalculateRelativePositions() {
        if (!current_anchor_.is_valid) {
            return;
        }

        std::cout << "🔄 Перерахунок позицій відносно нового якоря" << std::endl;

        // Всі позиції вже збережені як абсолютні
        // Відносні позиції обчислюються в GetDroneRelativePosition()
        // Тут просто логування

        for (const auto& [drone_id, position] : relative_positions_) {
            Position3D relative = position - current_anchor_.position;
            std::cout << "  Дрон " << drone_id << ": ("
                      << relative.x << ", " << relative.y << ", " << relative.z << ")" << std::endl;
        }
    }

    bool DistributedPositionTracker::ProposeNewAnchor(const AnchorPoint& proposed_anchor) {
        // Тут має бути інтеграція з ConsensusPositioning
        // Поки що спрощена версія
        std::cout << "🗳️ Пропозиція нового якоря типу " << static_cast<int>(proposed_anchor.type) << std::endl;

        // Симуляція консенсусу - приймаємо якщо стабільність > 0.7
        if (proposed_anchor.stability_score > 0.7) {
            current_anchor_ = proposed_anchor;
            current_anchor_.is_valid = true;
            RecalculateRelativePositions();
            return true;
        }

        return false;
    }

    bool DistributedPositionTracker::ShouldUpdateAnchor() {
        if (!current_anchor_.is_valid) {
            return true;
        }

        uint32_t current_time = GetCurrentTime();

        // Оновлюємо якір кожні 5 секунд для геометричного центру
        if (current_anchor_.type == AnchorType::GEOMETRIC_CENTER) {
            return (current_time - current_anchor_.last_update) > 5000;
        }

        // Для інших типів - менше частих оновлень
        return (current_time - current_anchor_.last_update) > 10000;
    }

    uint32_t DistributedPositionTracker::GetCurrentTime() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
    }

    double DistributedPositionTracker::GetPositioningAccuracy() const {
        if (!current_anchor_.is_valid) {
            return 0.0;
        }

        return current_anchor_.position_accuracy > 0 ?
               current_anchor_.position_accuracy : 1.0; // 1 метр за замовчуванням
    }

    bool DistributedPositionTracker::IsPositioningReliable() const {
        return current_anchor_.is_valid &&
               current_anchor_.stability_score > 0.5 &&
               relative_positions_.size() >= 2;
    }

} // namespace DistributedPositioning//
// Created by yv on 22.09.2025.
//
