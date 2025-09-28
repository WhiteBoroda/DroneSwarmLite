class PowerManager {
public:
    bool SetLowBatteryThreshold(double threshold) { return true; }
    bool SetCriticalBatteryThreshold(double threshold) { return true; }
    bool EnableAutoRTL(bool enabled) { return true; }
    bool EnablePowerSaving(bool enabled) { return true; }
};