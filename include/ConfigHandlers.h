#pragma once
#include "ConfigWatcher.h"
#include "CommunicationManager.h"
#include "UWBManager.h"
#include "MeshProtocol.h"

class CommunicationConfigHandler : public ConfigSectionHandler {
private:
    SwarmControl::CommunicationManager* comm_manager_;

public:
    CommunicationConfigHandler(SwarmControl::CommunicationManager* comm_mgr)
            : comm_manager_(comm_mgr) {}

    std::string GetSectionName() const override {
        return "communication";
    }

    bool HandleConfigChange(const std::string& key,
                            const std::string& old_value,
                            const std::string& new_value) override {
        if (!comm_manager_) return false;

        try {
            if (key == "max_power_dbm") {
                int8_t power = std::stoi(new_value);
                return comm_manager_->set_max_power_level(power);
            } else if (key == "frequency_hopping_enabled") {
                bool enabled = (new_value == "true");
                return comm_manager_->enable_frequency_hopping(enabled);
            } else if (key == "encryption_enabled") {
                bool enabled = (new_value == "true");
                return comm_manager_->enable_message_encryption(enabled);
            } else if (key == "communication_timeout_ms") {
                uint32_t timeout = std::stoul(new_value);
                return comm_manager_->set_communication_timeout(timeout);
            } else if (key == "heartbeat_interval_ms") {
                uint32_t interval = std::stoul(new_value);
                return comm_manager_->set_heartbeat_interval(interval);
            }

            std::cout << "✅ Communication config updated: " << key << " = " << new_value << std::endl;
            return true;

        } catch (const std::exception& e) {
            std::cerr << "❌ Communication config error: " << e.what() << std::endl;
            return false;
        }
    }
};

class UWBConfigHandler : public ConfigSectionHandler {
private:
    UWBManager* uwb_manager_;

public:
    UWBConfigHandler(UWBManager* uwb_mgr) : uwb_manager_(uwb_mgr) {}

    std::string GetSectionName() const override {
        return "uwb";
    }

    bool HandleConfigChange(const std::string& key,
                            const std::string& old_value,
                            const std::string& new_value) override {
        if (!uwb_manager_) return false;

        try {
            if (key == "tx_power_dbm") {
                int8_t power = std::stoi(new_value);
                return uwb_manager_->SetTxPower(power);
            } else if (key == "update_rate_hz") {
                uint16_t rate = std::stoul(new_value);
                return uwb_manager_->SetUpdateRate(rate);
            } else if (key == "ranging_mode") {
                UWBRangingMode mode;
                if (new_value == "DS_TWR") mode = UWBRangingMode::DS_TWR;
                else if (new_value == "SS_TWR") mode = UWBRangingMode::SS_TWR;
                else if (new_value == "TDOA") mode = UWBRangingMode::TDOA;
                else mode = UWBRangingMode::TWR;
                return uwb_manager_->SetRangingMode(mode);
            } else if (key == "rx_gain") {
                uint8_t gain = std::stoul(new_value);
                return uwb_manager_->SetRxGain(gain);
            } else if (key == "measurement_interval_ms") {
                uint32_t interval = std::stoul(new_value);
                return uwb_manager_->SetMeasurementInterval(interval);
            }

            std::cout << "✅ UWB config updated: " << key << " = " << new_value << std::endl;
            return true;

        } catch (const std::exception& e) {
            std::cerr << "❌ UWB config error: " << e.what() << std::endl;
            return false;
        }
    }
};

class MeshConfigHandler : public ConfigSectionHandler {
private:
    MeshNetwork::SwarmMeshProtocol* mesh_protocol_;

public:
    MeshConfigHandler(MeshNetwork::SwarmMeshProtocol* mesh_proto)
            : mesh_protocol_(mesh_proto) {}

    std::string GetSectionName() const override {
        return "mesh";
    }

    bool HandleConfigChange(const std::string& key,
                            const std::string& old_value,
                            const std::string& new_value) override {
        if (!mesh_protocol_) return false;

        try {
            if (key == "enabled") {
                bool enabled = (new_value == "true");
                return mesh_protocol_->SetMeshEnabled(enabled);
            } else if (key == "max_hops") {
                uint8_t hops = std::stoul(new_value);
                return mesh_protocol_->SetMaxHops(hops);
            } else if (key == "discovery_interval_ms") {
                uint32_t interval = std::stoul(new_value);
                return mesh_protocol_->SetDiscoveryInterval(interval);
            } else if (key == "routing_algorithm") {
                return mesh_protocol_->SetRoutingAlgorithm(new_value);
            }

            std::cout << "✅ Mesh config updated: " << key << " = " << new_value << std::endl;
            return true;

        } catch (const std::exception& e) {
            std::cerr << "❌ Mesh config error: " << e.what() << std::endl;
            return false;
        }
    }
};