//=============================================================================
// src/HighSpeedRanging.cpp
// High-speed UWB ranging algorithms with motion compensation
// Optimized for drones moving up to 100 km/h
// 🇺🇦 Slava Ukraini! 🇺🇦
//=============================================================================

#include "../include/HighSpeedRanging.h"
#include "../include/AerialUWBManager.h"
#include <cmath>
#include <iostream>

namespace AerialUWB {

//=============================================================================
// ✅ FAST SS-TWR (Single-Sided Two-Way Ranging)
//=============================================================================

    bool HighSpeedRanging::FastSS_TWR(
            AerialUWBManager* uwb_manager,
            DroneID target_drone,
            MotionCompensatedMeasurement& result)
    {
        if (!uwb_manager) {
            return false;
        }

        // Perform standard ranging
        if (!uwb_manager->range_to_drone(target_drone)) {
            return false;
        }

        // Get raw measurement
        auto measurements = uwb_manager->get_latest_measurements(1);
        if (measurements.empty()) {
            return false;
        }

        const auto& measurement = measurements[0];

        // Get motion state
        Position3D sender_pos = uwb_manager->get_estimated_position();
        Position3D sender_vel = uwb_manager->GetCurrentVelocity();

        // Apply simple motion compensation for SS-TWR
        // Distance correction = velocity * (propagation_time / 2)
        const double SPEED_OF_LIGHT = 299792458.0;  // m/s
        double propagation_time = measurement.distance / SPEED_OF_LIGHT;

        double velocity_magnitude = std::sqrt(
                sender_vel.x * sender_vel.x +
                sender_vel.y * sender_vel.y +
                sender_vel.z * sender_vel.z
        );

        double motion_correction = velocity_magnitude * (propagation_time / 2.0);

        // Fill result
        result.target_drone = target_drone;
        result.raw_distance = measurement.distance;
        result.corrected_distance = measurement.distance + motion_correction;
        result.measurement_time = measurement.timestamp;
        result.sender_position = sender_pos;
        result.sender_velocity = sender_vel;
        result.doppler_correction = 0.0;  // Not calculated in SS-TWR
        result.clock_drift_correction = 0.0;

        return true;
    }

//=============================================================================
// ✅ MOTION-COMPENSATED DS-TWR (Double-Sided Two-Way Ranging)
//=============================================================================

    bool HighSpeedRanging::MotionCompensatedDS_TWR(
            AerialUWBManager* uwb_manager,
            DroneID target_drone,
            MotionCompensatedMeasurement& result)
    {
        if (!uwb_manager) {
            return false;
        }

        // Get current state
        Position3D sender_pos = uwb_manager->get_estimated_position();
        Position3D sender_vel = uwb_manager->GetCurrentVelocity();

        uint64_t t1 = 0, t2 = 0, t3 = 0, t4 = 0;  // Timestamps

        // PHASE 1: Send ranging request
        // t1 = TX time at sender
        if (!uwb_manager->range_to_drone(target_drone)) {
            return false;
        }

        auto measurements = uwb_manager->get_latest_measurements(1);
        if (measurements.empty()) {
            return false;
        }

        const auto& measurement = measurements[0];

        // Calculate timestamps (simplified - in real implementation these come from UWB chip)
        const double SPEED_OF_LIGHT = 299792458.0;
        double round_trip_time = (measurement.distance * 2.0) / SPEED_OF_LIGHT;

        t1 = measurement.timestamp;
        t4 = measurement.timestamp + static_cast<uint64_t>(round_trip_time * 1e6);  // Convert to microseconds

        // Estimate remote timestamps (in real implementation, received from target)
        t2 = t1 + static_cast<uint64_t>((measurement.distance / SPEED_OF_LIGHT) * 1e6);
        t3 = t2 + 1000;  // Assume 1ms response delay

        // Calculate round-trip times
        uint64_t round_trip_initiator = t4 - t1;
        uint64_t round_trip_responder = t3 - t2;

        // DS-TWR formula: ToF = (RTT1 + RTT2) / 4
        double time_of_flight = (round_trip_initiator + round_trip_responder) / 4.0e6;  // Convert to seconds
        double distance = time_of_flight * SPEED_OF_LIGHT;

        // Apply clock drift correction
        double drift_correction = calculateClockDriftCorrection(t1, t4);
        distance *= (1.0 + drift_correction);

        // Get target motion state (if available)
        Position3D target_pos{0, 0, 0};
        Position3D target_vel{0, 0, 0};

        auto anchors = uwb_manager->GetActiveAnchors();
        for (const auto& anchor : anchors) {
            if (anchor.drone_id == target_drone) {
                target_pos = anchor.position;
                target_vel = anchor.velocity;
                break;
            }
        }

        // Apply relative motion compensation
        double motion_correction;
        if (compensateForRelativeMotion(sender_pos, sender_vel,
                                        target_pos, target_vel,
                                        motion_correction)) {
            distance += motion_correction;
        }

        // Calculate Doppler correction
        double rel_vx = sender_vel.x - target_vel.x;
        double rel_vy = sender_vel.y - target_vel.y;
        double rel_vz = sender_vel.z - target_vel.z;
        double relative_velocity = std::sqrt(rel_vx*rel_vx + rel_vy*rel_vy + rel_vz*rel_vz);

        double doppler_factor = 1.0 + (relative_velocity / SPEED_OF_LIGHT);
        double doppler_corrected_distance = distance / doppler_factor;

        // Fill result
        result.target_drone = target_drone;
        result.raw_distance = measurement.distance;
        result.corrected_distance = doppler_corrected_distance;
        result.measurement_time = measurement.timestamp;
        result.sender_position = sender_pos;
        result.sender_velocity = sender_vel;
        result.target_position = target_pos;
        result.target_velocity = target_vel;
        result.doppler_correction = doppler_corrected_distance - distance;
        result.clock_drift_correction = drift_correction;

        return true;
    }

//=============================================================================
// ✅ PARALLEL RANGING
//=============================================================================

    bool HighSpeedRanging::ParallelRanging(
            AerialUWBManager* uwb_manager,
            const std::vector<DroneID>& target_drones,
            std::vector<MotionCompensatedMeasurement>& results)
    {
        if (!uwb_manager || target_drones.empty()) {
            return false;
        }

        results.clear();
        results.reserve(target_drones.size());

        bool any_success = false;

        // Perform ranging to all targets sequentially
        // (True parallel would require multiple UWB transceivers)
        for (DroneID target : target_drones) {
            MotionCompensatedMeasurement measurement;

            if (MotionCompensatedDS_TWR(uwb_manager, target, measurement)) {
                results.push_back(measurement);
                any_success = true;
            }
        }

        return any_success;
    }

//=============================================================================
// ✅ FORMATION-OPTIMIZED RANGING
//=============================================================================

    bool HighSpeedRanging::FormationRanging(
            AerialUWBManager* uwb_manager,
            const std::vector<DroneID>& formation_drones,
            std::vector<MotionCompensatedMeasurement>& results)
    {
        if (!uwb_manager || formation_drones.empty()) {
            return false;
        }

        // In formation flight, drones are typically close together
        // Use faster SS-TWR for nearby drones, DS-TWR for distant ones

        results.clear();
        results.reserve(formation_drones.size());

        Position3D my_pos = uwb_manager->get_estimated_position();

        for (DroneID target : formation_drones) {
            // Estimate distance to target
            Position3D target_pos{0, 0, 0};

            auto anchors = uwb_manager->GetActiveAnchors();
            for (const auto& anchor : anchors) {
                if (anchor.drone_id == target) {
                    target_pos = anchor.position;
                    break;
                }
            }

            double estimated_distance = std::sqrt(
                    std::pow(target_pos.x - my_pos.x, 2) +
                    std::pow(target_pos.y - my_pos.y, 2) +
                    std::pow(target_pos.z - my_pos.z, 2)
            );

            MotionCompensatedMeasurement measurement;
            bool success;

            // Use SS-TWR for close drones (<50m), DS-TWR for distant
            if (estimated_distance < 50.0) {
                success = FastSS_TWR(uwb_manager, target, measurement);
            } else {
                success = MotionCompensatedDS_TWR(uwb_manager, target, measurement);
            }

            if (success) {
                results.push_back(measurement);
            }
        }

        return !results.empty();
    }

//=============================================================================
// ✅ HELPER FUNCTIONS
//=============================================================================

    double HighSpeedRanging::calculateClockDriftCorrection(
            uint64_t tx_time,
            uint64_t rx_time)
    {
        // Estimate clock drift based on timing
        // Real implementation would use actual clock calibration data

        double elapsed_us = static_cast<double>(rx_time - tx_time);

        // Typical crystal oscillator drift: ±20 ppm
        const double MAX_DRIFT_PPM = 20.0;

        // Calculate maximum possible drift
        double max_drift_correction = (MAX_DRIFT_PPM / 1e6) * elapsed_us;

        // For now, assume average drift (could be improved with calibration)
        double drift_ppm = 10.0;  // Conservative estimate

        return (drift_ppm / 1e6);
    }

    bool HighSpeedRanging::compensateForRelativeMotion(
            const Position3D& sender_pos,
            const Position3D& sender_vel,
            const Position3D& target_pos,
            const Position3D& target_vel,
            double& distance_correction)
    {
        // Calculate relative velocity vector
        double rel_vx = target_vel.x - sender_vel.x;
        double rel_vy = target_vel.y - sender_vel.y;
        double rel_vz = target_vel.z - sender_vel.z;

        // Calculate position difference vector
        double dx = target_pos.x - sender_pos.x;
        double dy = target_pos.y - sender_pos.y;
        double dz = target_pos.z - sender_pos.z;

        double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

        if (distance < 0.1) {
            distance_correction = 0.0;
            return false;  // Too close to calculate meaningful correction
        }

        // Unit vector from sender to target
        double ux = dx / distance;
        double uy = dy / distance;
        double uz = dz / distance;

        // Project relative velocity onto line-of-sight
        double radial_velocity = rel_vx * ux + rel_vy * uy + rel_vz * uz;

        // Correction factor based on signal propagation time
        const double SPEED_OF_LIGHT = 299792458.0;
        double propagation_time = distance / SPEED_OF_LIGHT;

        // Distance changed during propagation
        distance_correction = radial_velocity * propagation_time;

        return true;
    }

//=============================================================================
// ✅ HIGH-SPEED KALMAN FILTER IMPLEMENTATION
//=============================================================================

    HighSpeedKalmanFilter::HighSpeedKalmanFilter() : initialized(false) {
        // Initialize state
        for (int i = 0; i < STATE_SIZE; ++i) {
            state_x[i] = 0.0;
            for (int j = 0; j < STATE_SIZE; ++j) {
                state_covariance_p[i][j] = (i == j) ? 1.0 : 0.0;
            }
        }

        // Initialize process noise (Q matrix)
        for (int i = 0; i < STATE_SIZE; ++i) {
            for (int j = 0; j < STATE_SIZE; ++j) {
                process_noise_q[i][j] = 0.0;
            }
        }

        // Position process noise
        process_noise_q[0][0] = 0.1;  // x position noise
        process_noise_q[1][1] = 0.1;  // y position noise
        process_noise_q[2][2] = 0.1;  // z position noise

        // Velocity process noise
        process_noise_q[3][3] = 0.5;  // x velocity noise
        process_noise_q[4][4] = 0.5;  // y velocity noise
        process_noise_q[5][5] = 0.5;  // z velocity noise

        // Acceleration process noise
        process_noise_q[6][6] = 1.0;  // x acceleration noise
        process_noise_q[7][7] = 1.0;  // y acceleration noise
        process_noise_q[8][8] = 1.0;  // z acceleration noise

        // Initialize measurement noise (R matrix)
        for (int i = 0; i < MEASUREMENT_SIZE; ++i) {
            measurement_noise_r[i] = 1.0;  // 1m standard deviation
        }
    }

    void HighSpeedKalmanFilter::initialize(
            const Position3D& initial_position,
            const Position3D& initial_velocity)
    {
        // Set initial state
        state_x[0] = initial_position.x;
        state_x[1] = initial_position.y;
        state_x[2] = initial_position.z;
        state_x[3] = initial_velocity.x;
        state_x[4] = initial_velocity.y;
        state_x[5] = initial_velocity.z;
        state_x[6] = 0.0;  // acceleration x
        state_x[7] = 0.0;  // acceleration y
        state_x[8] = 0.0;  // acceleration z

        // Initialize covariance with moderate uncertainty
        for (int i = 0; i < STATE_SIZE; ++i) {
            for (int j = 0; j < STATE_SIZE; ++j) {
                state_covariance_p[i][j] = (i == j) ? 10.0 : 0.0;
            }
        }

        last_update = std::chrono::steady_clock::now();
        initialized = true;

        std::cout << "✅ Kalman filter initialized at ("
                  << initial_position.x << ", "
                  << initial_position.y << ", "
                  << initial_position.z << ")" << std::endl;
    }

    void HighSpeedKalmanFilter::predict(double dt_seconds) {
        if (!initialized || dt_seconds <= 0) {
            return;
        }

        // State transition matrix F (constant acceleration model)
        double F[STATE_SIZE][STATE_SIZE] = {0};

        // Identity
        for (int i = 0; i < STATE_SIZE; ++i) {
            F[i][i] = 1.0;
        }

        // Position-velocity coupling
        F[0][3] = dt_seconds;  // x = x + vx*dt
        F[1][4] = dt_seconds;  // y = y + vy*dt
        F[2][5] = dt_seconds;  // z = z + vz*dt

        // Position-acceleration coupling
        F[0][6] = 0.5 * dt_seconds * dt_seconds;  // x = x + 0.5*ax*dt^2
        F[1][7] = 0.5 * dt_seconds * dt_seconds;  // y = y + 0.5*ay*dt^2
        F[2][8] = 0.5 * dt_seconds * dt_seconds;  // z = z + 0.5*az*dt^2

        // Velocity-acceleration coupling
        F[3][6] = dt_seconds;  // vx = vx + ax*dt
        F[4][7] = dt_seconds;  // vy = vy + ay*dt
        F[5][8] = dt_seconds;  // vz = vz + az*dt

        // Predict state: x = F * x
        double predicted_state[STATE_SIZE] = {0};
        for (int i = 0; i < STATE_SIZE; ++i) {
            for (int j = 0; j < STATE_SIZE; ++j) {
                predicted_state[i] += F[i][j] * state_x[j];
            }
        }

        // Copy predicted state
        for (int i = 0; i < STATE_SIZE; ++i) {
            state_x[i] = predicted_state[i];
        }

        // Predict covariance: P = F * P * F^T + Q
        double temp[STATE_SIZE][STATE_SIZE] = {0};
        double predicted_cov[STATE_SIZE][STATE_SIZE] = {0};

        // temp = F * P
        for (int i = 0; i < STATE_SIZE; ++i) {
            for (int j = 0; j < STATE_SIZE; ++j) {
                for (int k = 0; k < STATE_SIZE; ++k) {
                    temp[i][j] += F[i][k] * state_covariance_p[k][j];
                }
            }
        }

        // predicted_cov = temp * F^T
        for (int i = 0; i < STATE_SIZE; ++i) {
            for (int j = 0; j < STATE_SIZE; ++j) {
                for (int k = 0; k < STATE_SIZE; ++k) {
                    predicted_cov[i][j] += temp[i][k] * F[j][k];  // F^T means swap indices
                }
            }
        }

        // Add process noise: P = P + Q
        for (int i = 0; i < STATE_SIZE; ++i) {
            for (int j = 0; j < STATE_SIZE; ++j) {
                state_covariance_p[i][j] = predicted_cov[i][j] + process_noise_q[i][j];
            }
        }
    }

    void HighSpeedKalmanFilter::update(
            const Position3D& measured_position,
            double measurement_accuracy)
    {
        if (!initialized) {
            initialize(measured_position, Position3D{0, 0, 0});
            return;
        }

        // Calculate time since last update
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_update).count();

        if (dt > 0.001) {  // Only predict if enough time passed
            predict(dt);
        }

        // Measurement matrix H (we only measure position)
        double H[MEASUREMENT_SIZE][STATE_SIZE] = {0};
        H[0][0] = 1.0;  // Measure x
        H[1][1] = 1.0;  // Measure y
        H[2][2] = 1.0;  // Measure z

        // Measurement vector
        double z[MEASUREMENT_SIZE] = {
                measured_position.x,
                measured_position.y,
                measured_position.z
        };

        // Update measurement noise based on accuracy
        for (int i = 0; i < MEASUREMENT_SIZE; ++i) {
            measurement_noise_r[i] = measurement_accuracy * measurement_accuracy;
        }

        // Innovation: y = z - H*x
        double innovation[MEASUREMENT_SIZE];
        for (int i = 0; i < MEASUREMENT_SIZE; ++i) {
            innovation[i] = z[i];
            for (int j = 0; j < STATE_SIZE; ++j) {
                innovation[i] -= H[i][j] * state_x[j];
            }
        }

        // Innovation covariance: S = H*P*H^T + R
        double S[MEASUREMENT_SIZE][MEASUREMENT_SIZE] = {0};
        double temp[MEASUREMENT_SIZE][STATE_SIZE] = {0};

        // temp = H * P
        for (int i = 0; i < MEASUREMENT_SIZE; ++i) {
            for (int j = 0; j < STATE_SIZE; ++j) {
                for (int k = 0; k < STATE_SIZE; ++k) {
                    temp[i][j] += H[i][k] * state_covariance_p[k][j];
                }
            }
        }

        // S = temp * H^T + R
        for (int i = 0; i < MEASUREMENT_SIZE; ++i) {
            for (int j = 0; j < MEASUREMENT_SIZE; ++j) {
                for (int k = 0; k < STATE_SIZE; ++k) {
                    S[i][j] += temp[i][k] * H[j][k];  // H^T
                }
                if (i == j) {
                    S[i][j] += measurement_noise_r[i];
                }
            }
        }

        // Kalman gain: K = P*H^T * S^-1
        // For 3x3 matrix, use simple inversion
        double S_inv[MEASUREMENT_SIZE][MEASUREMENT_SIZE];
        if (!invert3x3Matrix(S, S_inv)) {
            return;  // Singular matrix, skip update
        }

        double K[STATE_SIZE][MEASUREMENT_SIZE] = {0};
        double temp2[STATE_SIZE][MEASUREMENT_SIZE] = {0};

        // temp2 = P * H^T
        for (int i = 0; i < STATE_SIZE; ++i) {
            for (int j = 0; j < MEASUREMENT_SIZE; ++j) {
                for (int k = 0; k < STATE_SIZE; ++k) {
                    temp2[i][j] += state_covariance_p[i][k] * H[j][k];  // H^T
                }
            }
        }

        // K = temp2 * S_inv
        for (int i = 0; i < STATE_SIZE; ++i) {
            for (int j = 0; j < MEASUREMENT_SIZE; ++j) {
                for (int k = 0; k < MEASUREMENT_SIZE; ++k) {
                    K[i][j] += temp2[i][k] * S_inv[k][j];
                }
            }
        }

        // Update state: x = x + K*innovation
        for (int i = 0; i < STATE_SIZE; ++i) {
            for (int j = 0; j < MEASUREMENT_SIZE; ++j) {
                state_x[i] += K[i][j] * innovation[j];
            }
        }

        // Update covariance: P = (I - K*H) * P
        double I_KH[STATE_SIZE][STATE_SIZE];
        for (int i = 0; i < STATE_SIZE; ++i) {
            for (int j = 0; j < STATE_SIZE; ++j) {
                I_KH[i][j] = (i == j) ? 1.0 : 0.0;
                for (int k = 0; k < MEASUREMENT_SIZE; ++k) {
                    I_KH[i][j] -= K[i][k] * H[k][j];
                }
            }
        }

        double new_P[STATE_SIZE][STATE_SIZE] = {0};
        for (int i = 0; i < STATE_SIZE; ++i) {
            for (int j = 0; j < STATE_SIZE; ++j) {
                for (int k = 0; k < STATE_SIZE; ++k) {
                    new_P[i][j] += I_KH[i][k] * state_covariance_p[k][j];
                }
            }
        }

        // Copy new covariance
        for (int i = 0; i < STATE_SIZE; ++i) {
            for (int j = 0; j < STATE_SIZE; ++j) {
                state_covariance_p[i][j] = new_P[i][j];
            }
        }

        last_update = now;
    }

    Position3D HighSpeedKalmanFilter::getPredictedPosition(double future_dt_seconds) const {
        if (!initialized) {
            return Position3D{0, 0, 0};
        }

        // Predict position based on current velocity and acceleration
        Position3D predicted;
        predicted.x = state_x[0] + state_x[3] * future_dt_seconds +
                      0.5 * state_x[6] * future_dt_seconds * future_dt_seconds;
        predicted.y = state_x[1] + state_x[4] * future_dt_seconds +
                      0.5 * state_x[7] * future_dt_seconds * future_dt_seconds;
        predicted.z = state_x[2] + state_x[5] * future_dt_seconds +
                      0.5 * state_x[8] * future_dt_seconds * future_dt_seconds;

        return predicted;
    }

    Position3D HighSpeedKalmanFilter::getCurrentVelocity() const {
        if (!initialized) {
            return Position3D{0, 0, 0};
        }

        return Position3D{state_x[3], state_x[4], state_x[5]};
    }

    Position3D HighSpeedKalmanFilter::getCurrentAcceleration() const {
        if (!initialized) {
            return Position3D{0, 0, 0};
        }

        return Position3D{state_x[6], state_x[7], state_x[8]};
    }

    double HighSpeedKalmanFilter::getPositionAccuracy() const {
        if (!initialized) {
            return 999.0;
        }

        // Calculate position uncertainty from covariance diagonal
        double pos_variance = state_covariance_p[0][0] +
                              state_covariance_p[1][1] +
                              state_covariance_p[2][2];

        return std::sqrt(pos_variance / 3.0);  // RMS error
    }

    void HighSpeedKalmanFilter::reset() {
        initialized = false;

        for (int i = 0; i < STATE_SIZE; ++i) {
            state_x[i] = 0.0;
            for (int j = 0; j < STATE_SIZE; ++j) {
                state_covariance_p[i][j] = (i == j) ? 1.0 : 0.0;
            }
        }
    }

    bool HighSpeedKalmanFilter::invert3x3Matrix(
            const double input[3][3],
            double output[3][3]) const
    {
        // Calculate determinant
        double det = input[0][0] * (input[1][1] * input[2][2] - input[1][2] * input[2][1])
                     - input[0][1] * (input[1][0] * input[2][2] - input[1][2] * input[2][0])
                     + input[0][2] * (input[1][0] * input[2][1] - input[1][1] * input[2][0]);

        if (std::abs(det) < 1e-10) {
            return false;  // Singular matrix
        }

        double inv_det = 1.0 / det;

        // Calculate inverse
        output[0][0] = (input[1][1] * input[2][2] - input[1][2] * input[2][1]) * inv_det;
        output[0][1] = (input[0][2] * input[2][1] - input[0][1] * input[2][2]) * inv_det;
        output[0][2] = (input[0][1] * input[1][2] - input[0][2] * input[1][1]) * inv_det;

        output[1][0] = (input[1][2] * input[2][0] - input[1][0] * input[2][2]) * inv_det;
        output[1][1] = (input[0][0] * input[2][2] - input[0][2] * input[2][0]) * inv_det;
        output[1][2] = (input[0][2] * input[1][0] - input[0][0] * input[1][2]) * inv_det;

        output[2][0] = (input[1][0] * input[2][1] - input[1][1] * input[2][0]) * inv_det;
        output[2][1] = (input[0][1] * input[2][0] - input[0][0] * input[2][1]) * inv_det;
        output[2][2] = (input[0][0] * input[1][1] - input[0][1] * input[1][0]) * inv_det;

        return true;
    }

} // namespace AerialUWB