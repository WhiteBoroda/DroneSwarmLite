#pragma once

#include "SwarmTypes.h"
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>

namespace SwarmControl {

// Video quality settings
    enum class VideoQuality : uint8_t {
        LOW = 0,      // 480p, low bitrate
        MEDIUM = 1,   // 720p, medium bitrate
        HIGH = 2,     // 1080p, high bitrate
        ULTRA = 3     // Max resolution, max bitrate
    };

// Video transmission modes
    enum class TransmissionMode : uint8_t {
        CONTINUOUS,   // Always transmitting
        ON_DEMAND,    // Transmit when requested
        FORMATION_LEAD, // Only leader transmits
        ROTATION,     // Rotating between drones
        EMERGENCY     // Emergency broadcast mode
    };

// Camera control settings
    struct CameraSettings {
        uint16_t resolution_width;
        uint16_t resolution_height;
        uint8_t framerate;         // FPS
        uint8_t brightness;        // 0-255
        uint8_t contrast;          // 0-255
        uint8_t saturation;        // 0-255
        bool auto_exposure;
        uint16_t exposure_time;    // microseconds if manual
        bool auto_white_balance;
        uint16_t white_balance;    // Kelvin if manual

        CameraSettings() : resolution_width(1280), resolution_height(720), framerate(60),
                           brightness(128), contrast(128), saturation(128),
                           auto_exposure(true), exposure_time(10000),
                           auto_white_balance(true), white_balance(5000) {}
    };

// Video stream statistics
    struct VideoStreamStats {
        uint32_t frames_transmitted;
        uint32_t frames_dropped;
        double current_bitrate;     // Mbps
        double average_bitrate;     // Mbps
        uint8_t current_quality;    // 0-100%
        int8_t signal_strength;     // dBm
        double latency_ms;
        uint32_t transmission_errors;
        Timestamp stream_start_time;
        Duration total_stream_time;
    };

// VTX (Video Transmitter) hardware control
    struct VTXSettings {
        VideoChannel channel;
        uint16_t frequency_mhz;    // 5.8GHz band frequency
        uint16_t power_mw;         // 25-2500mW
        uint8_t band;              // A, B, E, F, R, L
        uint8_t channel_number;    // 1-8
        bool pit_mode;             // Pit mode (low power for close range)

        VTXSettings() : channel(VideoChannel::BAND_F_1), frequency_mhz(5740),
                        power_mw(200), band('F'), channel_number(1), pit_mode(false) {}
    };

    class VideoTransmitter {
    public:
        explicit VideoTransmitter(DroneId drone_id, const std::string& config_path);
        ~VideoTransmitter();

        // Core lifecycle
        bool initialize();
        bool start();
        void stop();
        void reset();

        // Stream control
        bool start_video_stream();
        bool stop_video_stream();
        bool pause_video_stream();
        bool resume_video_stream();
        bool is_streaming() const;

        // Channel and frequency management
        bool set_channel(VideoChannel channel);
        VideoChannel get_current_channel() const;
        bool set_frequency(uint16_t frequency_mhz);
        uint16_t get_current_frequency() const;
        bool scan_for_clear_channel();
        std::vector<VideoChannel> get_available_channels() const;

        // Power management
        bool set_transmission_power(uint16_t power_mw);
        uint16_t get_transmission_power() const;
        bool enable_adaptive_power();
        bool disable_adaptive_power();
        bool is_adaptive_power_enabled() const;

        // Video quality control
        bool set_video_quality(VideoQuality quality);
        VideoQuality get_video_quality() const;
        bool set_bitrate(uint32_t bitrate_kbps);
        uint32_t get_current_bitrate() const;
        bool enable_dynamic_quality();
        bool disable_dynamic_quality();

        // Camera control
        bool set_camera_settings(const CameraSettings& settings);
        CameraSettings get_camera_settings() const;
        bool auto_adjust_camera();
        bool capture_snapshot(const std::string& filename);

        // Stream switching and coordination
        bool enable_transmission();
        bool disable_transmission();
        bool is_transmission_enabled() const;
        bool set_transmission_mode(TransmissionMode mode);
        TransmissionMode get_transmission_mode() const;

        // Signal quality monitoring
        int8_t get_signal_strength() const;
        double get_video_latency() const;
        bool is_signal_quality_good() const;
        VideoStreamStats get_stream_statistics() const;

        // Interference handling
        bool detect_interference();
        bool handle_interference();
        bool switch_to_backup_channel();
        std::vector<uint16_t> scan_interference_levels() const;

        // Formation video management
        bool coordinate_with_swarm(const std::vector<DroneId>& swarm_members);
        bool request_video_priority();
        bool release_video_priority();
        bool is_primary_video_source() const;

        // Emergency video features
        bool enable_emergency_broadcast();
        bool disable_emergency_broadcast();
        bool boost_signal_power();
        bool set_emergency_channel();

        // Configuration and diagnostics
        bool reload_video_config();
        bool run_video_diagnostics();
        bool test_video_transmission();
        std::vector<std::string> get_system_status() const;

    private:
        DroneId drone_id_;
        std::string config_path_;

        // Hardware interfaces
        class CaddxRatelProCamera;    // Camera interface
        class VTX58GInterface;        // 5.8G VTX interface

        std::unique_ptr<CaddxRatelProCamera> camera_;
        std::unique_ptr<VTX58GInterface> vtx_hardware_;

        // Video stream state
        std::atomic<bool> streaming_active_;
        std::atomic<bool> transmission_enabled_;
        std::atomic<VideoQuality> current_quality_;
        std::atomic<TransmissionMode> transmission_mode_;

        // Channel and power management
        mutable std::mutex vtx_mutex_;
        VTXSettings current_vtx_settings_;
        std::atomic<bool> adaptive_power_enabled_;
        std::atomic<bool> dynamic_quality_enabled_;

        // Camera settings
        mutable std::mutex camera_mutex_;
        CameraSettings current_camera_settings_;

        // Signal monitoring
        std::atomic<int8_t> current_rssi_;
        std::atomic<double> current_latency_;
        std::deque<int8_t> rssi_history_;
        mutable std::mutex rssi_mutex_;

        // Statistics tracking
        VideoStreamStats stream_stats_;
        mutable std::mutex stats_mutex_;

        // Threading
        std::thread video_processing_thread_;
        std::thread signal_monitoring_thread_;
        std::thread power_management_thread_;
        std::atomic<bool> running_;

        std::condition_variable video_cv_;
        std::mutex video_mutex_;

        // Video frame buffer
        struct VideoFrame {
            std::vector<uint8_t> data;
            Timestamp timestamp;
            uint32_t frame_number;
            size_t size_bytes;
        };

        std::queue<VideoFrame> frame_buffer_;
        mutable std::mutex buffer_mutex_;
        static constexpr size_t MAX_FRAME_BUFFER = 30; // ~0.5 second at 60fps

        // Channel management
        std::vector<VideoChannel> available_channels_;
        std::vector<uint16_t> interference_levels_;
        mutable std::mutex channel_mutex_;

        // Power adaptation
        struct PowerLevel {
            uint16_t power_mw;
            int8_t min_rssi_threshold;
            double max_range_meters;
        };

        std::vector<PowerLevel> power_levels_;
        size_t current_power_level_index_;

        // Timing constants
        static constexpr Duration VIDEO_PROCESS_PERIOD = std::chrono::milliseconds(16); // ~60fps
        static constexpr Duration SIGNAL_MONITOR_PERIOD = std::chrono::milliseconds(100); // 10Hz
        static constexpr Duration POWER_ADAPT_PERIOD = std::chrono::seconds(1); // 1Hz

        // Private methods - Main loops
        void video_processing_loop();
        void signal_monitoring_loop();
        void power_management_loop();

        // Video processing
        bool capture_video_frame(VideoFrame& frame);
        bool encode_video_frame(VideoFrame& frame);
        bool transmit_video_frame(const VideoFrame& frame);
        bool process_frame_buffer();

        // Hardware control
        bool initialize_camera_hardware();
        bool initialize_vtx_hardware();
        bool configure_camera_parameters();
        bool configure_vtx_parameters();

        // Channel and frequency management
        uint16_t channel_to_frequency(VideoChannel channel) const;
        VideoChannel frequency_to_channel(uint16_t frequency) const;
        bool is_channel_clear(VideoChannel channel) const;
        bool measure_channel_interference(VideoChannel channel, uint16_t& interference_level);

        // Power management algorithms
        bool adapt_transmission_power();
        bool calculate_optimal_power();
        bool monitor_power_consumption();
        uint16_t get_minimum_required_power() const;

        // Quality adaptation
        bool adapt_video_quality();
        VideoQuality calculate_optimal_quality() const;
        bool adjust_bitrate_for_conditions();
        uint32_t calculate_target_bitrate() const;

        // Signal quality assessment
        bool assess_signal_quality();
        bool detect_signal_degradation();
        bool predict_signal_loss();
        double calculate_signal_quality_score() const;

        // Interference handling
        bool analyze_interference_pattern();
        VideoChannel find_best_alternative_channel() const;
        bool implement_interference_mitigation();
        bool coordinate_channel_selection();

        // Formation video coordination
        bool negotiate_video_priority();
        bool handle_video_handover();
        bool synchronize_video_parameters();
        bool manage_bandwidth_allocation();

        // Emergency procedures
        bool activate_emergency_mode();
        bool maximize_transmission_range();
        bool override_power_limits();
        bool send_emergency_video_signal();

        // Camera control helpers
        bool auto_adjust_exposure();
        bool auto_adjust_white_balance();
        bool optimize_camera_settings();
        bool handle_low_light_conditions();

        // Statistics and monitoring
        void update_stream_statistics();
        void calculate_performance_metrics();
        void log_video_events();

        // Error handling
        bool handle_transmission_error();
        bool recover_from_video_failure();
        bool reinitialize_video_hardware();
        void report_video_error(const std::string& error);

        // Configuration helpers
        bool load_video_configuration();
        bool validate_video_parameters();
        bool save_video_settings();
        bool apply_configuration_changes();

        // Utility methods
        bool is_power_level_valid(uint16_t power_mw) const;
        bool is_frequency_valid(uint16_t frequency_mhz) const;
        double calculate_estimated_range() const;
        std::string channel_to_string(VideoChannel channel) const;

        // Logging
        void log_video_event(const std::string& event, const std::string& details = "");
        void log_channel_change(VideoChannel old_channel, VideoChannel new_channel);
        void log_power_change(uint16_t old_power, uint16_t new_power);
    };

} // namespace SwarmControl//
// Created by yv on 22.09.2025.
//

#ifndef DRONESWARMLITE_VIDEOTRANSMITTER_H
#define DRONESWARMLITE_VIDEOTRANSMITTER_H

#endif //DRONESWARMLITE_VIDEOTRANSMITTER_H
