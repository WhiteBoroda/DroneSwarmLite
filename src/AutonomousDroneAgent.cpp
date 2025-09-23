#include "../include/AutonomousDroneAgent.h"
#include "../include/SwarmTypes.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace SwarmSystem {

    AutonomousDroneAgent::AutonomousDroneAgent(DroneID id)
            : my_id_(id), mission_state_(MissionState::IDLE), current_waypoint_index_(0) {

        // Ініціалізація стану
        current_state_.id = my_id_;
        current_state_.role = DroneRole::AUTONOMOUS;
        current_state_.is_autonomous = true;
        current_state_.can_lead = true;
        current_state_.is_healthy = true;

        std::cout << "🤖 Створено автономного агента для дрона " << my_id_ << std::endl;
    }

    bool AutonomousDroneAgent::Initialize() {
        try {
            // Ініціалізація підсистем автономії
            obstacle_avoidance_ = std::make_shared<ObstacleAvoidance>();
            navigator_ = std::make_shared<DeadReckoningNavigator>();
            local_map_ = std::make_shared<LocalEnvironmentMap>();

            // Встановлення початкової позиції
            Position3D initial_pos(0, 0, 0);
            if (!navigator_->Initialize(initial_pos)) {
                std::cerr << "❌ Помилка ініціалізації навігатора" << std::endl;
                return false;
            }

            mission_state_ = MissionState::IDLE;

            std::cout << "✅ Автономний агент дрона " << my_id_ << " ініціалізований" << std::endl;
            return true;

        } catch (const std::exception& e) {
            std::cerr << "❌ Виняток при ініціалізації агента: " << e.what() << std::endl;
            return false;
        }
    }

    bool AutonomousDroneAgent::ProcessDistributedCommand(const DistributedCommand& command) {
        std::cout << "📋 Обробка розподіленої команди типу " << command.command_type
                  << " від дрона " << command.originator_id << std::endl;

        // Зберігаємо команду як поточну місію
        current_mission_ = command;

        // Плануємо виконання залежно від типу команди
        switch (command.command_type) {
            case DistributedCommand::MOVE_TO_WAYPOINT:
                return PlanWaypointMission(command.target_position);

            case DistributedCommand::SEARCH_PATTERN:
                return PlanSearchMission(command.area_center, command.search_pattern_width,
                                         command.search_pattern_height);

            case DistributedCommand::LOITER_AREA:
                return PlanLoiterMission(command.area_center, command.area_radius);

            case DistributedCommand::ATTACK_TARGET:
                return PlanAttackMission(command.target_position);

            case DistributedCommand::CHANGE_FORMATION:
                return PlanFormationChange(command.formation_type);

            case DistributedCommand::EMERGENCY_STOP:
                return InitiateEmergencyStop();

            case DistributedCommand::AUTONOMOUS_MODE:
                return EnableFullAutonomy();

            default:
                std::cout << "⚠️ Невідомий тип команди: " << command.command_type << std::endl;
                return false;
        }
    }

    bool AutonomousDroneAgent::ExecuteCurrentMission() {
        if (mission_state_ == MissionState::IDLE) {
            return true; // Немає активної місії
        }

        // Оновлюємо стан навігації та карти
        UpdateNavigationState();

        // Виконуємо поточну місію
        switch (current_mission_.command_type) {
            case DistributedCommand::MOVE_TO_WAYPOINT:
                return ExecuteWaypointMission();

            case DistributedCommand::SEARCH_PATTERN:
                return ExecuteSearchMission();

            case DistributedCommand::LOITER_AREA:
                return ExecuteLoiterMission();

            case DistributedCommand::ATTACK_TARGET:
                return ExecuteAttackMission();

            default:
                return ExecuteDefaultBehavior();
        }
    }

    bool AutonomousDroneAgent::PlanTrajectoryToTarget(const Position3D& target) {
        if (!navigator_) {
            std::cerr << "❌ Навігатор не ініціалізований" << std::endl;
            return false;
        }

        Position3D current_pos = navigator_->GetEstimatedPosition();

        // Перевіряємо чи ціль досяжна
        double distance = SwarmUtils::CalculateDistance(current_pos, target);
        if (distance > 10000.0) { // Максимум 10км
            std::cout << "⚠️ Ціль занадто далеко: " << distance << " метрів" << std::endl;
            return false;
        }

        // Планування безпечної траєкторії з уникненням перешкод
        if (obstacle_avoidance_) {
            planned_waypoints_ = obstacle_avoidance_->PlanSafePath(current_pos, target);
        } else {
            // Простий прямий шлях якщо немає системи уникнення
            planned_waypoints_.clear();
            planned_waypoints_.push_back(current_pos);
            planned_waypoints_.push_back(target);
        }

        current_waypoint_index_ = 0;

        std::cout << "🗺️ Заплановано траєкторію з " << planned_waypoints_.size()
                  << " точок до цілі (" << target.x() << ", " << target.y() << ", " << target.z() << ")" << std::endl;

        return !planned_waypoints_.empty();
    }

    bool AutonomousDroneAgent::AdaptToEnvironment() {
        if (!local_map_ || !obstacle_avoidance_) {
            return true; // Немає системи адаптації
        }

        // Оновлюємо локальну карту з сенсорів
        local_map_->UpdateFromSensors();

        // Перевіряємо чи поточна траєкторія безпечна
        if (!planned_waypoints_.empty() && current_waypoint_index_ < planned_waypoints_.size()) {
            Position3D current_pos = navigator_->GetEstimatedPosition();
            Position3D next_waypoint = planned_waypoints_[current_waypoint_index_];

            if (!local_map_->IsPathClear(current_pos, next_waypoint)) {
                std::cout << "⚠️ Перешкода на шляху - перепланування траєкторії" << std::endl;

                // Перепланування з урахуванням нових перешкод
                Position3D final_target = planned_waypoints_.back();
                return PlanTrajectoryToTarget(final_target);
            }
        }

        return true;
    }

    bool AutonomousDroneAgent::HandleCommunicationLoss() {
        std::cout << "📡 Втрата зв'язку - перехід в автономний режим" << std::endl;

        mission_state_ = MissionState::SELF_SUFFICIENT;
        current_state_.comm_status = CommunicationStatus::LOST;

        // Продовжуємо поточну місію автономно
        return ContinueMissionAutonomously();
    }

    bool AutonomousDroneAgent::ContinueMissionAutonomously() {
        std::cout << "🤖 Автономне продовження місії" << std::endl;

        // Якщо немає поточної місії - переходимо в режим патрулювання
        if (current_mission_.command_type == DistributedCommand::MOVE_TO_WAYPOINT &&
            planned_waypoints_.empty()) {

            return InitiateAutonomousPatrol();
        }

        // Продовжуємо виконання поточної місії
        mission_state_ = MissionState::EXECUTING_COMMAND;
        return true;
    }

    bool AutonomousDroneAgent::CoordinateWithNearbyDrones() {
        std::cout << "🤝 Координація з сусідніми дронами" << std::endl;

        // Знаходимо ближчих дронів (тут має бути інтеграція з mesh-мережею)
        std::vector<DroneID> nearby_drones = GetNearbyDrones();

        if (nearby_drones.empty()) {
            std::cout << "👤 Немає ближчих дронів - працюю самостійно" << std::endl;
            return true;
        }

        // Обмінюємося статусом з ближчими дронами
        for (DroneID drone_id : nearby_drones) {
            SendStatusToDrone(drone_id);
        }

        // Якщо є дрони з кращими характеристиками - слідуємо за ними
        DroneID best_leader = FindBestLocalLeader(nearby_drones);
        if (best_leader != 0 && best_leader != my_id_) {
            std::cout << "👑 Слідую за локальним лідером " << best_leader << std::endl;
            return FollowLocalLeader(best_leader);
        }

        return true;
    }

// Приватні методи реалізації

    bool AutonomousDroneAgent::PlanWaypointMission(const Position3D& target) {
        std::cout << "📍 Планування місії до точки (" << target.x() << ", " << target.y() << ", " << target.z() << ")" << std::endl;

        if (!PlanTrajectoryToTarget(target)) {
            return false;
        }

        mission_state_ = MissionState::EXECUTING_COMMAND;
        return true;
    }

    bool AutonomousDroneAgent::PlanSearchMission(const Position3D& center, double width, double height) {
        std::cout << "🔍 Планування пошукової місії в зоні " << width << "x" << height << " навколо ("
                  << center.x() << ", " << center.y() << ")" << std::endl;

        // Створюємо зигзагоподібний пошуковий патерн
        planned_waypoints_.clear();

        int strips = static_cast<int>(width / 50.0); // Смуги по 50 метрів
        double strip_width = width / strips;

        for (int i = 0; i < strips; ++i) {
            double x_offset = -width/2 + i * strip_width;

            if (i % 2 == 0) {
                // Прямий прохід
                planned_waypoints_.emplace_back(center.x() + x_offset, center.y() - height/2, center.z());
                planned_waypoints_.emplace_back(center.x() + x_offset, center.y() + height/2, center.z());
            } else {
                // Зворотний прохід
                planned_waypoints_.emplace_back(center.x() + x_offset, center.y() + height/2, center.z());
                planned_waypoints_.emplace_back(center.x() + x_offset, center.y() - height/2, center.z());
            }
        }

        current_waypoint_index_ = 0;
        mission_state_ = MissionState::EXECUTING_COMMAND;

        std::cout << "📋 Заплановано " << planned_waypoints_.size() << " точок для пошуку" << std::endl;
        return true;
    }

    bool AutonomousDroneAgent::PlanLoiterMission(const Position3D& center, double radius) {
        std::cout << "🔄 Планування патрулювання навколо (" << center.x() << ", " << center.y()
                  << ") радіус " << radius << "м" << std::endl;

        // Створюємо кругову траєкторію
        planned_waypoints_.clear();

        int points = 16; // 16 точок по колу
        for (int i = 0; i < points; ++i) {
            double angle = (2.0 * SwarmConstants::PI * i) / points;
            double x = center.x() + radius * cos(angle);
            double y = center.y() + radius * sin(angle);
            planned_waypoints_.emplace_back(x, y, center.z());
        }

        current_waypoint_index_ = 0;
        mission_state_ = MissionState::EXECUTING_COMMAND;

        std::cout << "📋 Заплановано кругове патрулювання з " << points << " точок" << std::endl;
        return true;
    }

    bool AutonomousDroneAgent::PlanAttackMission(const Position3D& target) {
        std::cout << "⚔️ Планування атаки на ціль (" << target.x() << ", " << target.y() << ", " << target.z() << ")" << std::endl;

        // Атака можлива тільки з підтвердженням оператора
        // В автономному режимі переходимо в патрулювання навколо цілі
        return PlanLoiterMission(target, 100.0); // 100м навколо цілі
    }

    bool AutonomousDroneAgent::PlanFormationChange(FormationType new_formation) {
        std::cout << "📐 Зміна формації на " << SwarmUtils::FormationTypeToString(new_formation) << std::endl;

        // Тут має бути інтеграція з DynamicFormationManager
        // Поки що просто змінюємо стан
        mission_state_ = MissionState::EXECUTING_COMMAND;
        return true;
    }

    bool AutonomousDroneAgent::InitiateEmergencyStop() {
        std::cout << "🚨 АВАРІЙНА ЗУПИНКА!" << std::endl;

        // Очищаємо всі місії
        planned_waypoints_.clear();
        current_waypoint_index_ = 0;

        // Переходимо в режим зависання
        mission_state_ = MissionState::EMERGENCY_MODE;
        current_state_.role = DroneRole::SELF_DESTRUCT;

        return true;
    }

    bool AutonomousDroneAgent::EnableFullAutonomy() {
        std::cout << "🤖 Повна автономія увімкнена" << std::endl;

        mission_state_ = MissionState::SELF_SUFFICIENT;
        current_state_.is_autonomous = true;
        current_state_.role = DroneRole::AUTONOMOUS;

        // Якщо немає поточної місії - починаємо патрулювання
        if (planned_waypoints_.empty()) {
            return InitiateAutonomousPatrol();
        }

        return true;
    }

    bool AutonomousDroneAgent::ExecuteWaypointMission() {
        if (planned_waypoints_.empty() || current_waypoint_index_ >= planned_waypoints_.size()) {
            std::cout << "✅ Місія з waypoint завершена" << std::endl;
            mission_state_ = MissionState::IDLE;
            return true;
        }

        Position3D current_pos = navigator_->GetEstimatedPosition();
        Position3D target_waypoint = planned_waypoints_[current_waypoint_index_];

        double distance = SwarmUtils::CalculateDistance(current_pos, target_waypoint);

        if (distance < 5.0) { // Досягли точки з точністю 5м
            current_waypoint_index_++;
            std::cout << "✅ Досягнуто waypoint " << current_waypoint_index_
                      << " з " << planned_waypoints_.size() << std::endl;

            if (current_waypoint_index_ >= planned_waypoints_.size()) {
                mission_state_ = MissionState::IDLE;
                std::cout << "🎯 Місія повністю завершена" << std::endl;
            }
        } else {
            // Рухаємося до поточної точки
            SendMovementCommand(target_waypoint);
        }

        return true;
    }

    bool AutonomousDroneAgent::ExecuteSearchMission() {
        // Аналогічно до waypoint, але з додатковими перевірками на виявлення цілей
        bool waypoint_result = ExecuteWaypointMission();

        // Тут має бути інтеграція з системами виявлення цілей
        CheckForTargets();

        return waypoint_result;
    }

    bool AutonomousDroneAgent::ExecuteLoiterMission() {
        if (planned_waypoints_.empty()) {
            std::cout << "❌ Немає точок для патрулювання" << std::endl;
            return false;
        }

        // Циклічне патрулювання
        if (current_waypoint_index_ >= planned_waypoints_.size()) {
            current_waypoint_index_ = 0; // Повертаємося на початок
            std::cout << "🔄 Новий цикл патрулювання розпочато" << std::endl;
        }

        return ExecuteWaypointMission();
    }

    bool AutonomousDroneAgent::ExecuteAttackMission() {
        std::cout << "⚔️ Виконання атакувальної місії (обмежено в автономному режимі)" << std::endl;

        // В автономному режимі замість атаки - патрулювання навколо цілі
        return ExecuteLoiterMission();
    }

    bool AutonomousDroneAgent::ExecuteDefaultBehavior() {
        std::cout << "🤖 Виконання стандартної поведінки - патрулювання" << std::endl;

        if (planned_waypoints_.empty()) {
            return InitiateAutonomousPatrol();
        }

        return ExecuteLoiterMission();
    }

    bool AutonomousDroneAgent::InitiateAutonomousPatrol() {
        std::cout << "🛡️ Ініціація автономного патрулювання" << std::endl;

        Position3D current_pos = navigator_->GetEstimatedPosition();

        // Створюємо патрульну зону навколо поточної позиції
        return PlanLoiterMission(current_pos, 200.0); // 200м радіус
    }

    void AutonomousDroneAgent::UpdateNavigationState() {
        if (!navigator_) {
            return;
        }

        // Оновлюємо позицію на основі сенсорів
        navigator_->UpdatePosition(0.05); // 50мс оновлення

        // Оновлюємо стан дрона
        current_state_.position = navigator_->GetEstimatedPosition();
        current_state_.velocity = EstimateCurrentVelocity();
        current_state_.last_update = std::chrono::duration_cast<Timestamp>(
                std::chrono::system_clock::now().time_since_epoch());
    }

    void AutonomousDroneAgent::SendMovementCommand(const Position3D& target) {
        // Тут має бути інтеграція з політним контролером
        std::cout << "🚁 Рух до позиції (" << target.x() << ", " << target.y() << ", " << target.z() << ")" << std::endl;
    }

    void AutonomousDroneAgent::CheckForTargets() {
        // Тут має бути інтеграція з системами виявлення цілей
        // Поки що заглушка
    }

    Velocity3D AutonomousDroneAgent::EstimateCurrentVelocity() {
        // Спрощена оцінка швидкості
        return Velocity3D(0, 0, 0);
    }

// Заглушки для інтеграції з mesh-мережею
    std::vector<DroneID> AutonomousDroneAgent::GetNearbyDrones() {
        // Тут має бути інтеграція з mesh-протоколом
        return std::vector<DroneID>();
    }

    void AutonomousDroneAgent::SendStatusToDrone(DroneID drone_id) {
        std::cout << "📡 Відправка статусу дрону " << drone_id << std::endl;
    }

    DroneID AutonomousDroneAgent::FindBestLocalLeader(const std::vector<DroneID>& candidates) {
        // Поки що повертаємо першого доступного
        return candidates.empty() ? 0 : candidates[0];
    }

    bool AutonomousDroneAgent::FollowLocalLeader(DroneID leader_id) {
        std::cout << "👥 Слідування за лідером " << leader_id << std::endl;
        // Тут має бути логіка слідування
        return true;
    }

} // namespace SwarmSystem