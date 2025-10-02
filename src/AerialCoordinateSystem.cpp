//=============================================================================
// src/AerialCoordinateSystem.cpp
// 3D aerial coordinate system without ground references
// Formation-based positioning for drone swarms
// 🇺🇦 Slava Ukraini! 🇺🇦
//=============================================================================

#include "../include/AerialCoordinateSystem.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>

namespace AerialUWB {

//=============================================================================
// ✅ CONSTRUCTOR & DESTRUCTOR
//=============================================================================

    AerialCoordinateSystem::AerialCoordinateSystem()
            : primary_anchor_(0)
            , coordinate_system_valid_(false)
            , formation_quality_(0.0)
            , last_update_time_(0)
    {
        formation_center_ = {0.0, 0.0, 0.0};
        formation_velocity_ = {0.0, 0.0, 0.0};
    }

    AerialCoordinateSystem::~AerialCoordinateSystem() = default;

//=============================================================================
// ✅ INITIALIZATION AND CONTROL
//=============================================================================

    bool AerialCoordinateSystem::Initialize(const std::vector<AerialAnchorPoint>& initial_anchors) {
        std::cout << "🧭 Initializing aerial coordinate system..." << std::endl;

        if (initial_anchors.size() < 3) {
            std::cerr << "❌ Need at least 3 anchors to establish coordinate system" << std::endl;
            return false;
        }

        std::lock_guard<std::mutex> lock(anchors_mutex_);

        // Store anchors
        anchors_.clear();
        for (const auto& anchor : initial_anchors) {
            anchors_[anchor.drone_id] = anchor;
        }

        // Calculate initial formation center
        CalculateFormationCenter();

        // Select primary anchor (most stable)
        SelectPrimaryAnchor();

        // Calculate coordinate system quality
        formation_quality_ = CalculateGeometryQuality();

        coordinate_system_valid_ = (formation_quality_ > 0.5);
        last_update_time_ = GetCurrentTimestamp();

        std::cout << "✅ Aerial coordinate system initialized" << std::endl;
        std::cout << "   Anchors: " << anchors_.size() << std::endl;
        std::cout << "   Formation center: ("
                  << formation_center_.x << ", "
                  << formation_center_.y << ", "
                  << formation_center_.z << ")" << std::endl;
        std::cout << "   Geometry quality: " << formation_quality_ << std::endl;

        return coordinate_system_valid_;
    }

    bool AerialCoordinateSystem::UpdateAnchors(const std::vector<AerialAnchorPoint>& updated_anchors) {
        std::lock_guard<std::mutex> lock(anchors_mutex_);

        // Update existing anchors or add new ones
        for (const auto& anchor : updated_anchors) {
            anchors_[anchor.drone_id] = anchor;
        }

        // Recalculate formation parameters
        CalculateFormationCenter();
        formation_quality_ = CalculateGeometryQuality();
        coordinate_system_valid_ = (formation_quality_ > 0.5);
        last_update_time_ = GetCurrentTimestamp();

        // Check if primary anchor is still valid
        auto primary_it = anchors_.find(primary_anchor_);
        if (primary_it == anchors_.end() || !primary_it->second.is_active) {
            SelectPrimaryAnchor();
        }

        return coordinate_system_valid_;
    }

    void AerialCoordinateSystem::RemoveAnchor(DroneID drone_id) {
        std::lock_guard<std::mutex> lock(anchors_mutex_);

        anchors_.erase(drone_id);

        if (primary_anchor_ == drone_id) {
            SelectPrimaryAnchor();
        }

        CalculateFormationCenter();
        formation_quality_ = CalculateGeometryQuality();
        coordinate_system_valid_ = (anchors_.size() >= 3 && formation_quality_ > 0.5);
    }

//=============================================================================
// ✅ COORDINATE TRANSFORMATIONS
//=============================================================================

    bool AerialCoordinateSystem::TransformToFormationFrame(
            const Position3D& absolute_position,
            Position3D& formation_relative_position) const
    {
        if (!coordinate_system_valid_) {
            return false;
        }

        // Simple translation: relative = absolute - formation_center
        formation_relative_position.x = absolute_position.x - formation_center_.x;
        formation_relative_position.y = absolute_position.y - formation_center_.y;
        formation_relative_position.z = absolute_position.z - formation_center_.z;

        return true;
    }

    bool AerialCoordinateSystem::TransformToAbsoluteFrame(
            const Position3D& formation_relative_position,
            Position3D& absolute_position) const
    {
        if (!coordinate_system_valid_) {
            return false;
        }

        // Simple translation: absolute = relative + formation_center
        absolute_position.x = formation_relative_position.x + formation_center_.x;
        absolute_position.y = formation_relative_position.y + formation_center_.y;
        absolute_position.z = formation_relative_position.z + formation_center_.z;

        return true;
    }

    Position3D AerialCoordinateSystem::GetFormationCenter() const {
        return formation_center_;
    }

    Position3D AerialCoordinateSystem::GetFormationVelocity() const {
        return formation_velocity_;
    }

//=============================================================================
// ✅ ANCHOR SELECTION AND QUALITY
//=============================================================================

    bool AerialCoordinateSystem::SelectPrimaryAnchor() {
        std::lock_guard<std::mutex> lock(anchors_mutex_);

        if (anchors_.empty()) {
            primary_anchor_ = 0;
            return false;
        }

        // Select anchor with highest priority and quality
        DroneID best_anchor = 0;
        double best_score = -1.0;

        for (const auto& [drone_id, anchor] : anchors_) {
            if (!anchor.is_active) {
                continue;
            }

            // Calculate selection score
            double score = 0.0;

            // 1. Quality (40% weight)
            score += 0.4 * anchor.quality;

            // 2. Priority (30% weight)
            score += 0.3 * (anchor.anchor_priority / 100.0);

            // 3. Time synchronization (20% weight)
            if (anchor.time_synchronized) {
                score += 0.2;
            }

            // 4. Stability (10% weight) - prefer slower-moving anchors
            double velocity_mag = std::sqrt(
                    anchor.velocity.x * anchor.velocity.x +
                    anchor.velocity.y * anchor.velocity.y +
                    anchor.velocity.z * anchor.velocity.z
            );
            score += 0.1 * (1.0 - std::min(velocity_mag / 10.0, 1.0));

            if (score > best_score) {
                best_score = score;
                best_anchor = drone_id;
            }
        }

        if (best_anchor != 0) {
            primary_anchor_ = best_anchor;
            std::cout << "📍 Primary anchor selected: " << best_anchor
                      << " (score: " << best_score << ")" << std::endl;
            return true;
        }

        return false;
    }

    std::vector<DroneID> AerialCoordinateSystem::SelectOptimalAnchorSet(
            size_t target_count) const
    {
        std::lock_guard<std::mutex> lock(anchors_mutex_);

        std::vector<DroneID> selected_anchors;

        if (target_count == 0 || anchors_.empty()) {
            return selected_anchors;
        }

        // Create list of candidate anchors
        struct AnchorCandidate {
            DroneID id;
            double score;
        };

        std::vector<AnchorCandidate> candidates;

        for (const auto& [drone_id, anchor] : anchors_) {
            if (!anchor.is_active) {
                continue;
            }

            double score = anchor.quality * 0.6 + (anchor.anchor_priority / 100.0) * 0.4;
            candidates.push_back({drone_id, score});
        }

        // Sort by score (descending)
        std::sort(candidates.begin(), candidates.end(),
                  [](const AnchorCandidate& a, const AnchorCandidate& b) {
                      return a.score > b.score;
                  });

        // Select top anchors up to target_count
        size_t count = std::min(target_count, candidates.size());
        for (size_t i = 0; i < count; ++i) {
            selected_anchors.push_back(candidates[i].id);
        }

        return selected_anchors;
    }

    double AerialCoordinateSystem::CalculateGeometryQuality() const {
        // Calculate geometric dilution of precision (GDOP) equivalent

        if (anchors_.size() < 3) {
            return 0.0;
        }

        // Get anchor positions
        std::vector<Position3D> positions;
        for (const auto& [id, anchor] : anchors_) {
            if (anchor.is_active) {
                positions.push_back(anchor.position);
            }
        }

        if (positions.size() < 3) {
            return 0.0;
        }

        // Calculate average distance from formation center
        double avg_distance = 0.0;
        for (const auto& pos : positions) {
            double dx = pos.x - formation_center_.x;
            double dy = pos.y - formation_center_.y;
            double dz = pos.z - formation_center_.z;
            avg_distance += std::sqrt(dx*dx + dy*dy + dz*dz);
        }
        avg_distance /= positions.size();

        // Calculate spread/variance
        double variance = 0.0;
        for (const auto& pos : positions) {
            double dx = pos.x - formation_center_.x;
            double dy = pos.y - formation_center_.y;
            double dz = pos.z - formation_center_.z;
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            variance += (dist - avg_distance) * (dist - avg_distance);
        }
        variance /= positions.size();
        double std_dev = std::sqrt(variance);

        // Calculate angular distribution quality
        double angular_quality = CalculateAngularDistribution(positions);

        // Combine metrics
        double spread_quality = std::min(std_dev / avg_distance, 1.0);  // Better when more spread out
        double distance_quality = std::min(avg_distance / 100.0, 1.0);  // Better with larger distances

        double quality = 0.5 * angular_quality + 0.3 * spread_quality + 0.2 * distance_quality;

        return std::min(quality, 1.0);
    }

    double AerialCoordinateSystem::CalculateAngularDistribution(
            const std::vector<Position3D>& positions) const
    {
        if (positions.size() < 3) {
            return 0.0;
        }

        // Calculate angles between anchors as seen from formation center
        std::vector<double> angles;

        for (size_t i = 0; i < positions.size(); ++i) {
            for (size_t j = i + 1; j < positions.size(); ++j) {
                // Vectors from center to each anchor
                double v1x = positions[i].x - formation_center_.x;
                double v1y = positions[i].y - formation_center_.y;
                double v1z = positions[i].z - formation_center_.z;

                double v2x = positions[j].x - formation_center_.x;
                double v2y = positions[j].y - formation_center_.y;
                double v2z = positions[j].z - formation_center_.z;

                double mag1 = std::sqrt(v1x*v1x + v1y*v1y + v1z*v1z);
                double mag2 = std::sqrt(v2x*v2x + v2y*v2y + v2z*v2z);

                if (mag1 > 0.1 && mag2 > 0.1) {
                    // Dot product
                    double dot = v1x*v2x + v1y*v2y + v1z*v2z;
                    double cos_angle = dot / (mag1 * mag2);

                    // Clamp to [-1, 1] to avoid numerical issues
                    cos_angle = std::max(-1.0, std::min(1.0, cos_angle));

                    double angle = std::acos(cos_angle);
                    angles.push_back(angle);
                }
            }
        }

        if (angles.empty()) {
            return 0.0;
        }

        // Calculate variance of angles (ideal is uniform distribution)
        double mean_angle = 0.0;
        for (double angle : angles) {
            mean_angle += angle;
        }
        mean_angle /= angles.size();

        double variance = 0.0;
        for (double angle : angles) {
            variance += (angle - mean_angle) * (angle - mean_angle);
        }
        variance /= angles.size();

        // Lower variance is better (more uniform distribution)
        double uniformity = 1.0 - std::min(variance / (M_PI * M_PI / 4.0), 1.0);

        return uniformity;
    }

//=============================================================================
// ✅ FORMATION MANAGEMENT
//=============================================================================

    bool AerialCoordinateSystem::CalculateFormationCenter() {
        if (anchors_.empty()) {
            formation_center_ = {0.0, 0.0, 0.0};
            formation_velocity_ = {0.0, 0.0, 0.0};
            return false;
        }

        // Calculate weighted center (using quality as weight)
        double total_weight = 0.0;
        double weighted_x = 0.0;
        double weighted_y = 0.0;
        double weighted_z = 0.0;
        double weighted_vx = 0.0;
        double weighted_vy = 0.0;
        double weighted_vz = 0.0;

        for (const auto& [id, anchor] : anchors_) {
            if (!anchor.is_active) {
                continue;
            }

            double weight = anchor.quality;
            total_weight += weight;

            weighted_x += anchor.position.x * weight;
            weighted_y += anchor.position.y * weight;
            weighted_z += anchor.position.z * weight;

            weighted_vx += anchor.velocity.x * weight;
            weighted_vy += anchor.velocity.y * weight;
            weighted_vz += anchor.velocity.z * weight;
        }

        if (total_weight > 0.0) {
            formation_center_.x = weighted_x / total_weight;
            formation_center_.y = weighted_y / total_weight;
            formation_center_.z = weighted_z / total_weight;

            formation_velocity_.x = weighted_vx / total_weight;
            formation_velocity_.y = weighted_vy / total_weight;
            formation_velocity_.z = weighted_vz / total_weight;

            return true;
        }

        return false;
    }

    bool AerialCoordinateSystem::UpdateFormationState(
            const Position3D& new_center,
            const Position3D& new_velocity)
    {
        formation_center_ = new_center;
        formation_velocity_ = new_velocity;
        last_update_time_ = GetCurrentTimestamp();

        return true;
    }

    FormationGeometry AerialCoordinateSystem::AnalyzeFormationGeometry() const {
        FormationGeometry geometry;
        geometry.valid = false;

        std::lock_guard<std::mutex> lock(anchors_mutex_);

        if (anchors_.size() < 3) {
            return geometry;
        }

        // Calculate formation dimensions
        double min_x = 1e9, max_x = -1e9;
        double min_y = 1e9, max_y = -1e9;
        double min_z = 1e9, max_z = -1e9;

        for (const auto& [id, anchor] : anchors_) {
            if (!anchor.is_active) continue;

            min_x = std::min(min_x, anchor.position.x);
            max_x = std::max(max_x, anchor.position.x);
            min_y = std::min(min_y, anchor.position.y);
            max_y = std::max(max_y, anchor.position.y);
            min_z = std::min(min_z, anchor.position.z);
            max_z = std::max(max_z, anchor.position.z);
        }

        geometry.bounding_box_size_x = max_x - min_x;
        geometry.bounding_box_size_y = max_y - min_y;
        geometry.bounding_box_size_z = max_z - min_z;

        // Calculate average spacing
        double total_distance = 0.0;
        size_t pair_count = 0;

        for (auto it1 = anchors_.begin(); it1 != anchors_.end(); ++it1) {
            if (!it1->second.is_active) continue;

            for (auto it2 = std::next(it1); it2 != anchors_.end(); ++it2) {
                if (!it2->second.is_active) continue;

                double dx = it2->second.position.x - it1->second.position.x;
                double dy = it2->second.position.y - it1->second.position.y;
                double dz = it2->second.position.z - it1->second.position.z;
                double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

                total_distance += dist;
                pair_count++;
            }
        }

        if (pair_count > 0) {
            geometry.average_anchor_spacing = total_distance / pair_count;
            geometry.valid = true;
        }

        geometry.formation_center = formation_center_;
        geometry.formation_velocity = formation_velocity_;
        geometry.geometry_quality = formation_quality_;

        return geometry;
    }

//=============================================================================
// ✅ STATUS AND DIAGNOSTICS
//=============================================================================

    bool AerialCoordinateSystem::IsValid() const {
        return coordinate_system_valid_;
    }

    double AerialCoordinateSystem::GetQuality() const {
        return formation_quality_;
    }

    size_t AerialCoordinateSystem::GetAnchorCount() const {
        std::lock_guard<std::mutex> lock(anchors_mutex_);

        size_t count = 0;
        for (const auto& [id, anchor] : anchors_) {
            if (anchor.is_active) {
                count++;
            }
        }

        return count;
    }

    DroneID AerialCoordinateSystem::GetPrimaryAnchor() const {
        return primary_anchor_;
    }

    std::vector<AerialAnchorPoint> AerialCoordinateSystem::GetAllAnchors() const {
        std::lock_guard<std::mutex> lock(anchors_mutex_);

        std::vector<AerialAnchorPoint> anchor_list;
        for (const auto& [id, anchor] : anchors_) {
            anchor_list.push_back(anchor);
        }

        return anchor_list;
    }

    void AerialCoordinateSystem::PrintStatus() const {
        std::cout << "\n╔══════════════════════════════════════════════════════════╗" << std::endl;
        std::cout << "║         AERIAL COORDINATE SYSTEM STATUS                 ║" << std::endl;
        std::cout << "╚══════════════════════════════════════════════════════════╝" << std::endl;

        std::cout << "Valid: " << (coordinate_system_valid_ ? "YES ✅" : "NO ❌") << std::endl;
        std::cout << "Quality: " << std::fixed << std::setprecision(2)
                  << formation_quality_ * 100.0 << "%" << std::endl;
        std::cout << "Active anchors: " << GetAnchorCount() << std::endl;
        std::cout << "Primary anchor: " << primary_anchor_ << std::endl;

        std::cout << "\nFormation center: ("
                  << std::setprecision(2)
                  << formation_center_.x << ", "
                  << formation_center_.y << ", "
                  << formation_center_.z << ") m" << std::endl;

        std::cout << "Formation velocity: ("
                  << std::setprecision(2)
                  << formation_velocity_.x << ", "
                  << formation_velocity_.y << ", "
                  << formation_velocity_.z << ") m/s" << std::endl;

        double speed = std::sqrt(
                formation_velocity_.x * formation_velocity_.x +
                formation_velocity_.y * formation_velocity_.y +
                formation_velocity_.z * formation_velocity_.z
        );
        std::cout << "Formation speed: " << speed << " m/s ("
                  << speed * 3.6 << " km/h)" << std::endl;

        // Print anchor details
        std::lock_guard<std::mutex> lock(anchors_mutex_);

        std::cout << "\n┌─────────────────────────────────────────────────────┐" << std::endl;
        std::cout << "│ Anchor Details:                                     │" << std::endl;
        std::cout << "└─────────────────────────────────────────────────────┘" << std::endl;

        for (const auto& [id, anchor] : anchors_) {
            std::cout << "  Drone " << id
                      << (id == primary_anchor_ ? " [PRIMARY]" : "")
                      << (anchor.is_active ? " 🟢" : " 🔴") << std::endl;
            std::cout << "    Position: ("
                      << anchor.position.x << ", "
                      << anchor.position.y << ", "
                      << anchor.position.z << ")" << std::endl;
            std::cout << "    Quality: " << anchor.quality * 100.0 << "%" << std::endl;
            std::cout << "    RSSI: " << anchor.rssi_dbm << " dBm" << std::endl;
        }

        std::cout << std::endl;
    }

//=============================================================================
// ✅ HELPER FUNCTIONS
//=============================================================================

    uint64_t AerialCoordinateSystem::GetCurrentTimestamp() const {
        auto now = std::chrono::steady_clock::now();
        auto duration = now.time_since_epoch();
        return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
    }

} // namespace AerialUWB