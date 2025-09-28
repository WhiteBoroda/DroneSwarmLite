class SafetyManager {
public:
    bool EnableGeofence(bool enabled) { return true; }
    bool SetMaxAltitude(double altitude) { return true; }
    bool SetMaxSpeed(double speed) { return true; }
    bool EnableEmergencyLand(bool enabled) { return true; }
    bool SetFailsafeMode(const std::string& mode) { return true; }
};