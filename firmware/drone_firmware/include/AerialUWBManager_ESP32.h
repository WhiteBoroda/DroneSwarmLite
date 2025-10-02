#pragma once
#include "UWBManager_ESP32.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace AerialUWB_ESP32 {

// Aerial UWB modes for drones
    enum class AerialMode_ESP32 {
        STATIC_ANCHOR = 0,      // Ваш текущий режим
        STATIC_TAG,             // Ваш текущий режим
        DYNAMIC_ANCHOR_TAG,     // НОВЫЙ - переключение
        MOBILE_ANCHOR,          // НОВЫЙ - мобильный реперный точки
        FORMATION_LEADER,       // НОВЫЙ - лидер строя
        FORMATION_FOLLOWER      // НОВЫЙ - ведомый в строю
    };

// High-speed UWB configuration
    struct AerialUWBConfig_ESP32 {
        uint16_t fast_ranging_rate_hz;
        float max_velocity_ms;
        bool doppler_compensation_enabled;
        bool motion_prediction_enabled;
        uint32_t anchor_switch_interval_ms;
        float min_anchor_stability;
        uint16_t ranging_timeout_us;
        uint16_t position_prediction_ms;
        bool kalman_filter_enabled;
        bool formation_sync_enabled;
        uint16_t fast_ranging_rate_hz = 200;    // Быстрые измерения для скорости
        float max_velocity_ms = 27.78f;         // 100 км/ч максимум
        bool dynamic_switching_enabled = true;  // Динамическое переключение ролей
        uint32_t anchor_switch_interval_ms = 30000; // Интервал смены якорей
        bool motion_prediction_enabled = true;  // Предсказание движения
    };

// Motion-compensated measurement
    struct MotionCompensatedMeasurement_ESP32 {
        uint8_t target_id;
        float distance_m;
        float relative_velocity_ms;
        float doppler_shift_hz;
        uint64_t timestamp_us;
        float accuracy_m;
        bool motion_compensated;
    };

// Fast ranging frame structure
    struct FastRangingFrame_ESP32 {
        uint8_t frame_control;
        uint16_t sequence_number;
        uint8_t sender_id;
        uint8_t target_id;
        UWBMessageType_ESP32 message_type;
        uint64_t timestamp_tx;
        uint64_t timestamp_rx;
        Position3D_ESP32 sender_position;
        Position3D_ESP32 sender_velocity;
        uint16_t ranging_timeout_us;
        uint16_t expected_response_delay_us;
        uint16_t crc;
    } __attribute__((packed));

// Formation sync frame
    struct FormationSyncFrame_ESP32 {
        uint8_t frame_control;
        uint16_t sequence_number;
        uint8_t sender_id;
        uint8_t target_id;
        UWBMessageType_ESP32 message_type;
        uint64_t timestamp_tx;
        Position3D_ESP32 current_position;
        Position3D_ESP32 current_velocity;
        Position3D_ESP32 formation_center;
        Position3D_ESP32 formation_velocity;
        float formation_accuracy;
        uint16_t crc;
    } __attribute__((packed));

    class AerialUWBManager_ESP32 : public UWBManager_ESP32 {
    private:
        // Aerial-specific state
        AerialMode_ESP32 current_aerial_mode_;
        AerialConfig_ESP32 aerial_config_;
        Position3D_ESP32 current_velocity_;
        uint8_t formation_leader_id_;
        Position3D_ESP32 current_velocity_;
        Position3D_ESP32 current_acceleration_;
        Position3D_ESP32 formation_center_;
        Position3D_ESP32 formation_velocity_;
        TaskHandle_t aerial_management_task_handle_;
        TaskHandle_t high_speed_ranging_task_handle_;
        MotionCompensatedMeasurement_ESP32 motion_measurements_[16];
        uint8_t motion_measurement_count_;


        // Motion tracking
        float motion_state_[9]; // [x,y,z,vx,vy,vz,ax,ay,az]
        uint64_t last_filter_update_us_;
        uint64_t anchor_start_time_;
        uint64_t last_role_switch_;
        uint8_t active_anchor_count_;

        // Task handles
        TaskHandle_t aerial_management_task_handle_;
        TaskHandle_t high_speed_ranging_task_handle_;

    public:
        AerialUWBManager_ESP32(uint8_t drone_id)
                : UWBManager_ESP32(drone_id)  // ← ВЫЗЫВАЕМ КОНСТРУКТОР
                , current_aerial_mode_(AerialMode_ESP32::DYNAMIC_ANCHOR_TAG)
                , formation_leader_id_(0)
                , aerial_management_task_handle_(nullptr)
                , high_speed_ranging_task_handle_(nullptr)
                , motion_measurement_count_(0) {

            UWB_DEBUG("Aerial UWB Manager initialized");
        }

        // Aerial system control
        bool initializeAerialSystem();
        bool startAerialOperations();
        void stopAerialOperations();

        // Role switching
        bool switchToAnchorMode();
        bool switchToTagMode();
        bool shouldSwitchRole();
        AerialMode_ESP32 getCurrentAerialMode() const { return current_aerial_mode_; }
        bool canServeAsAnchor();

        // High-speed positioning
        bool performHighSpeedRanging(uint8_t target_id);
        bool calculateMotionCompensatedDistance(const FastRangingFrame_ESP32& request,
                                                const FastRangingFrame_ESP32& response,
                                                MotionCompensatedMeasurement_ESP32& measurement);

        // Motion prediction
        bool updateMotionFilter(const Position3D_ESP32& measured_position);
        Position3D_ESP32 predictPositionAtTime(uint64_t future_time_us);
        float calculateCurrentSpeed();

        // Motion tracking
        Position3D_ESP32 predictPositionAtTime(uint64_t future_time_us);
        float calculateCurrentSpeed();
        bool updateMotionFilter(const Position3D_ESP32& measured_position);

        // Formation support
        bool synchronizeWithFormation();
        bool broadcastFormationPosition();
        bool setFormationLeader(uint8_t leader_id);

    private:
        // Internal methods
        static void aerialManagementTask(void* parameter);
        static void highSpeedRangingTask(void* parameter);
        bool configureForAerialOperations();
        void initializeMotionFilter(const Position3D_ESP32& initial_position);
        void predictMotionState(double dt);
        void updateMotionState(const Position3D_ESP32& measured_position);
    };

} // namespace AerialUWB_ESP32