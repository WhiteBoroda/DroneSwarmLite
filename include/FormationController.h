class FormationController {
public:
    bool SetDefaultFormation(const std::string& formation_type) { return true; }
    bool SetFormationSpacing(double spacing) { return true; }
    bool SetFormationLeader(DroneID leader_id) { return true; }
    bool SetFollowDistance(double distance) { return true; }
    bool EnableCollisionAvoidance(bool enabled) { return true; }
};