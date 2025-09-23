#pragma once

#include "SwarmTypes.h"
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

namespace SwarmControl {

// Flight modes
    enum class FlightMode : uint8_t {
        MANUAL,
        STABILIZE,
        ALTITUDE_HOLD,
        POSITION_HOLD,
        AUTO,
        RTL,  // Return to Launch
        LAND,
        LOITER,
        GUIDED,
        FORMATION,
        TERMINAL_GUIDANCE,
        EMERGENCY
    };

// Hardware status
    struct FlightControllerStatus {
        bool armed;
        bool ready_to_fly;
        FlightMode current_mode;
        bool gps_available;  // Will be false in our case
        bool sensors_healthy;
        bool battery_ok;
        bool rc_signal_ok;
        double cpu_load;
        uint32_t flight_time_ms;
    };

// PID controller parameters
    struct PIDParams {
        double kp, ki, kd;
        double max_output;
        double integral_limit;

        PIDParams() : kp(1.0), ki(0.0), kd(0.0), max_output(100.0), integral_limit(10.0) {}
        PIDParams(double p, double i, double d, double max_out = 100.0, double int_lim = 10.0)
                : kp(p), ki(i), kd(d), max_output(max_out), integral_limit(int_lim) {}
    };

// Motor outputs (PWM values for 4 motors)
    struct MotorOutputs {
        uint16_t motor1, motor2, motor3, motor4;  // PWM values 1000-2000

        MotorOutputs() : motor1(1000), motor2(1000), motor3(1000), motor4(1000) {}
        MotorOutputs(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
                : motor1(m1), motor2(m2), motor3(m3), motor4(m4) {}
    };

// Sensor data structure
    struct SensorData {
        // IMU data
        struct {
            double ax, ay, az;  // m/s² - accelerometer
            double gx, gy, gz;  // rad/s - gyroscope
            double mx, my, mz;  // μT - magnetometer
        } imu;

        // Barometer
        double pressure;     // Pa
        double altitude;     // meters (relative to start)

        // Battery
        double voltage;      // V
        double current;      // A
        uint8_t remaining;   // percentage

        // Additional sensors
        double temperature;  // °C

        Timestamp timestamp;
    };

    class FlightController {
    public:
        explicit FlightController(DroneId drone_id, const std::string& config_path);
        ~FlightController();

        // Core lifecycle
        bool initialize();
        bool start();
        void stop();
        void emergency_shutdown();

        // Arming and safety
        bool arm();
        bool disarm();
        bool is_armed() const;
        bool is_ready_to_fly() const;
        bool run_pre_flight_checks();

        // Flight modes
        bool set_flight_mode(FlightMode mode);
        FlightMode get_flight_mode() const;
        bool is_mode_available(FlightMode mode) const;

        // Basic flight commands
        bool takeoff(double target_altitude);
        bool land();
        bool emergency_land();
        bool hover();
        bool set_throttle(double throttle_percent); // 0-100%

        // Position control (no GPS - relative positioning only)
        bool set_target_position(const Position3D& target);
        bool set_target_velocity(const Velocity3D& velocity);
        bool set_target_attitude(const Attitude& attitude);

        Position3D get_current_position() const;
        Velocity3D get_current_velocity() const;
        Attitude get_current_attitude() const;

        // Formation flying support
        bool set_formation_offset(const Position3D& offset);
        bool enable_formation_mode();
        bool disable_formation_mode();
        bool follow_leader(const Position3D& leader_pos, const Velocity3D& leader_vel);

        // Wind compensation and environmental adaptation
        bool enable_wind_compensation();
        bool disable_wind_compensation();
        bool set_wind_estimate(const Velocity3D& wind_velocity);
        Velocity3D get_wind_estimate() const;

        // Low-level control
        bool set_motor_outputs(const MotorOutputs& outputs);
        MotorOutputs get_motor_outputs() const;
        bool set_servo_output(uint8_t servo_id, uint16_t pwm_value);

        // PID tuning and control
        bool set_pid_params(const std::string& controller, const PIDParams& params);
        PIDParams get_pid_params(const std::string& controller) const;
        bool reset_pid_controllers();

        // Sensor access
        SensorData get_sensor_data() const;
        bool calibrate_sensors();
        bool is_sensor_healthy(const std::string& sensor_name) const;

        // Status and diagnostics
        FlightControllerStatus get_status() const;
        std::vector<std::string> get_system_warnings() const;
        std::vector<std::string> get_system_errors() const;

        // Configuration and parameters
        bool load_flight_parameters();
        bool save_flight_parameters();
        bool set_parameter(const std::string& name, double value);
        double get_parameter(const std::string& name) const;

        // Logging and telemetry
        bool enable_flight_logging();
        bool disable_flight_logging();
        bool export_flight_log(const std::string& filename);

        // Emergency procedures
        bool execute_failsafe();
        bool set_failsafe_action(const std::string& trigger, const std::string& action);

    private:
        DroneId drone_id_;
        std::string config_path_;

        // Flight state
        std::atomic<bool> armed_;
        std::atomic<FlightMode> current_mode_;
        std::atomic<bool> ready_to_fly_;

        // Position and attitude tracking
        mutable std::mutex position_mutex_;
        Position3D current_position_;
        Position3D target_position_;
        Velocity3D current_velocity_;
        Velocity3D target_velocity_;
        Attitude current_attitude_;
        Attitude target_attitude_;

        // Formation flying
        Position3D formation_offset_;
        std::atomic<bool> formation_mode_enabled_;

        // Wind compensation
        std::atomic<bool> wind_compensation_enabled_;
        Velocity3D wind_estimate_;
        mutable std::mutex wind_mutex_;

        // PID Controllers
        class PIDController {
        public:
            PIDController(const PIDParams& params);
            double update(double error, double dt);
            void reset();
            void set_params(const PIDParams& params);
            PIDParams get_params() const;

        private:
            PIDParams params_;
            double integral_;
            double last_error_;
            bool first_run_;
            mutable std::mutex pid_mutex_;
        };

        std::unique_ptr<PIDController> pid_roll_;
        std::unique_ptr<PIDController> pid_pitch_;
        std::unique_ptr<PIDController> pid_yaw_;
        std::unique_ptr<PIDController> pid_altitude_;
        std::unique_ptr<PIDController> pid_pos_x_;
        std::unique_ptr<PIDController> pid_pos_y_;

        // Hardware interfaces
        class MambaF722Interface;
        std::unique_ptr<MambaF722Interface> flight_hardware_;

        // Sensor data
        mutable std::mutex sensor_mutex_;
        SensorData latest_sensor_data_;
        std::atomic<Timestamp> last_sensor_update_;

        // Motor outputs
        mutable std::mutex motor_mutex_;
        MotorOutputs current_motor_outputs_;

        // Threading
        std::thread control_loop_thread_;
        std::thread sensor_thread_;
        std::thread safety_thread_;
        std::atomic<bool> running_;

        std::condition_variable control_loop_cv_;
        std::mutex control_loop_mutex_;

        // Control loop timing
        static constexpr Duration CONTROL_LOOP_PERIOD = std::chrono::milliseconds(4); // 250Hz
        static constexpr Duration SENSOR_UPDATE_PERIOD = std::chrono::milliseconds(2); // 500Hz
        static constexpr Duration SAFETY_CHECK_PERIOD = std::chrono::milliseconds(100); // 10Hz

        // Status tracking
        FlightControllerStatus current_status_;
        mutable std::mutex status_mutex_;

        std::vector<std::string> system_warnings_;
        std::vector<std::string> system_errors_;
        mutable std::mutex warnings_mutex_;

        // Configuration parameters
        std::unordered_map<std::string, double> flight_parameters_;
        mutable std::mutex parameters_mutex_;

        // Logging
        std::atomic<bool> logging_enabled_;
        std::string log_filename_;

        // Private methods - Main loops
        void control_loop();
        void sensor_update_loop();
        void safety_monitor_loop();

        // Flight control algorithms
        bool update_attitude_control();
        bool update_position_control();
        bool update_altitude_control();
        bool calculate_motor_mixing();

        // Sensor processing
        bool read_imu_data();
        bool read_barometer_data();
        bool read_battery_data();
        bool update_state_estimation();
        bool detect_sensor_failures();

        // Safety and monitoring
        bool check_flight_safety();
        bool monitor_battery_levels();
        bool check_hardware_health();
        bool validate_control_commands();

        // Hardware abstraction
        bool initialize_flight_hardware();
        bool configure_pwm_outputs();
        bool setup_sensor_interfaces();
        bool calibrate_flight_hardware();

        // Formation flying helpers
        Position3D calculate_formation_target(const Position3D& leader_pos);
        bool apply_formation_corrections();

        // Wind compensation algorithms
        Velocity3D estimate_wind_velocity();
        bool apply_wind_compensation();

        // Control mixing and limits
        bool apply_control_limits(MotorOutputs& outputs);
        bool check_control_authority();
        double constrain_value(double value, double min_val, double max_val);

        // Parameter management
        bool load_default_parameters();
        bool validate_parameter_ranges();
        bool apply_parameter_changes();

        // Utility methods
        double wrap_angle(double angle) const; // Wrap angle to [-π, π]
        double degrees_to_radians(double degrees) const;
        double radians_to_degrees(double radians) const;

        // Logging helpers
        void log_flight_data();
        void log_control_outputs();
        void log_sensor_data();

        // Error handling
        void handle_sensor_error(const std::string& sensor);
        void handle_control_error(const std::string& controller);
        void trigger_failsafe(const std::string& reason);
    };

} // namespace SwarmControl//
// Created by yv on 22.09.2025.
//

#ifndef DRONESWARMLITE_FLIGHTCONTROLLER_H
#define DRONESWARMLITE_FLIGHTCONTROLLER_H

#endif //DRONESWARMLITE_FLIGHTCONTROLLER_H
