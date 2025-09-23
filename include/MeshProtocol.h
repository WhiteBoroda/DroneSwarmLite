// include/MeshProtocol.h
// Mesh-протокол для распределенной сети дронов
// 🇺🇦 Slava Ukraini! 🇺🇦

#pragma once

#include "SwarmTypes.h"
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <vector>
#include <functional>
#include <memory>
#include <mutex>
#include <random>

namespace MeshNetwork {

// Message types for mesh network
    enum class MeshMessageType : uint8_t {
        HELLO = 0x01,               // Neighbor discovery
        HELLO_ACK = 0x02,           // Neighbor acknowledgment
        ROUTE_REQUEST = 0x03,       // Route discovery
        ROUTE_REPLY = 0x04,         // Route response
        DATA = 0x05,                // Data message
        ACK = 0x06,                 // Acknowledgment
        HEARTBEAT = 0x07,           // Keep-alive
        EMERGENCY = 0x08,           // Emergency broadcast
        COMMAND_DISTRIBUTE = 0x09,  // Distributed command
        TELEMETRY = 0x0A,           // Telemetry data
        POSITION_UPDATE = 0x0B,     // Position synchronization
        ANCHOR_ANNOUNCE = 0x0C      // Anchor point announcement
    };

// Mesh message header
    struct MeshHeader {
        uint32_t magic;             // 0x4D455348 ("MESH")
        uint8_t version;            // Protocol version
        MeshMessageType type;       // Message type
        DroneID source_id;          // Original sender
        DroneID destination_id;     // Final destination (0x0000 = broadcast)
        DroneID next_hop;           // Next hop in route
        DroneID previous_hop;       // Previous hop (for ACK)
        uint8_t hop_count;          // Current hop count
        uint8_t max_hops;           // Maximum allowed hops
        uint32_t sequence_number;   // Message sequence number
        uint32_t timestamp;         // Creation time (ms)
        uint16_t payload_size;      // Payload size in bytes
        uint8_t priority;           // 0=highest, 255=lowest
        uint8_t flags;              // Message flags
        uint16_t checksum;          // Header checksum
    } __attribute__((packed));

// Message flags
    namespace MeshFlags {
        constexpr uint8_t ACK_REQUIRED = 0x01;
        constexpr uint8_t ENCRYPTED = 0x02;
        constexpr uint8_t FRAGMENTED = 0x04;
        constexpr uint8_t URGENT = 0x08;
        constexpr uint8_t BROADCAST = 0x10;
    }

// Complete mesh message
    struct MeshMessage {
        MeshHeader header;
        std::vector<uint8_t> payload;

        MeshMessage() {
            memset(&header, 0, sizeof(header));
            header.magic = 0x4D455348;  // "MESH"
            header.version = 1;
            header.max_hops = 10;
            header.destination_id = BROADCAST_ID;
        }
    };

// Route table entry
    struct RouteEntry {
        DroneID destination;
        DroneID next_hop;
        uint8_t hop_count;
        double route_quality;       // 0.0-1.0
        Timestamp last_used;
        Timestamp expires_at;
        bool is_valid;

        RouteEntry() : destination(INVALID_DRONE_ID), next_hop(INVALID_DRONE_ID),
                       hop_count(255), route_quality(0.0), is_valid(false) {}
    };

// Neighbor information
    struct NeighborInfo {
        DroneID neighbor_id;
        int8_t rssi;                // Signal strength
        double packet_loss;         // 0.0-1.0
        Timestamp last_hello;
        Timestamp last_data;
        uint32_t messages_sent;
        uint32_t messages_received;
        bool is_reliable;
        double link_quality;        // Calculated quality score

        NeighborInfo() : neighbor_id(INVALID_DRONE_ID), rssi(-100), packet_loss(1.0),
                         messages_sent(0), messages_received(0), is_reliable(false),
                         link_quality(0.0) {}
    };

// Statistics and monitoring
    struct MeshStatistics {
        uint32_t messages_sent;
        uint32_t messages_received;
        uint32_t messages_forwarded;
        uint32_t messages_dropped;
        uint32_t route_discoveries;
        uint32_t hello_messages_sent;
        uint32_t hello_messages_received;
        double average_hop_count;
        double network_reliability;
        Timestamp start_time;

        MeshStatistics() {
            memset(this, 0, sizeof(MeshStatistics));
            start_time = std::chrono::steady_clock::now();
        }
    };

// Message handler callback
    using MessageHandler = std::function<void(const MeshMessage&, DroneID from_neighbor)>;

// Main mesh protocol class
    class SwarmMeshProtocol {
    public:
        explicit SwarmMeshProtocol(DroneID my_id);
        ~SwarmMeshProtocol();

        // Lifecycle management
        bool Initialize();
        bool Start();
        void Stop();
        void Update();  // Call periodically in main loop

        // Message sending
        bool SendMessage(const MeshMessage& message);
        bool SendMessageTo(DroneID destination, MeshMessageType type,
                           const std::vector<uint8_t>& payload, uint8_t priority = 128);
        bool BroadcastMessage(MeshMessageType type, const std::vector<uint8_t>& payload,
                              uint8_t priority = 128);
        bool SendEmergencyMessage(const std::vector<uint8_t>& payload);

        // Message handling
        bool RegisterMessageHandler(MeshMessageType type, MessageHandler handler);
        void UnregisterMessageHandler(MeshMessageType type);

        // Network topology
        std::vector<DroneID> GetNeighbors() const;
        std::vector<DroneID> GetReachableDrones() const;
        bool IsNeighbor(DroneID drone_id) const;
        bool IsReachable(DroneID drone_id) const;
        double GetLinkQuality(DroneID neighbor_id) const;

        // Route management
        bool FindRoute(DroneID destination);
        std::vector<DroneID> GetRouteTo(DroneID destination) const;
        void InvalidateRoute(DroneID destination);
        void ClearRoutes();

        // Network information
        size_t GetNeighborCount() const;
        size_t GetRouteCount() const;
        MeshStatistics GetStatistics() const;
        std::vector<NeighborInfo> GetNeighborInfo() const;

        // Configuration
        void SetMaxHops(uint8_t max_hops);
        void SetHelloInterval(Duration interval);
        void SetRouteTimeout(Duration timeout);
        void SetNeighborTimeout(Duration timeout);

    private:
        DroneID my_id_;
        bool running_;

        // Network topology
        std::unordered_map<DroneID, NeighborInfo> neighbors_;
        std::unordered_map<DroneID, RouteEntry> routing_table_;
        mutable std::mutex neighbors_mutex_;
        mutable std::mutex routing_mutex_;

        // Message handling
        std::unordered_map<MeshMessageType, MessageHandler> message_handlers_;
        std::mutex handlers_mutex_;

        // Message queues
        std::queue<MeshMessage> outgoing_queue_;
        std::queue<MeshMessage> forwarding_queue_;
        std::mutex outgoing_mutex_;
        std::mutex forwarding_mutex_;

        // Duplicate detection
        std::unordered_set<uint32_t> seen_sequences_;
        std::mutex sequences_mutex_;

        // Pending acknowledgments
        struct PendingAck {
            MeshMessage message;
            Timestamp sent_time;
            uint8_t retry_count;
            DroneID expected_from;
        };
        std::unordered_map<uint32_t, PendingAck> pending_acks_;
        std::mutex acks_mutex_;

        // Statistics
        MeshStatistics stats_;
        mutable std::mutex stats_mutex_;

        // Configuration
        uint8_t max_hops_;
        Duration hello_interval_;
        Duration route_timeout_;
        Duration neighbor_timeout_;

        // Timing
        Timestamp last_hello_time_;
        Timestamp last_route_cleanup_;
        Timestamp last_neighbor_discovery_;

        // Sequence number generation
        std::mt19937 sequence_generator_;

        // Core protocol methods
        void ProcessIncomingMessage(const MeshMessage& message, DroneID from_neighbor);
        void ProcessOutgoingQueue();
        void ProcessForwardingQueue();

        // Message type handlers
        void HandleHelloMessage(const MeshMessage& message, DroneID from_neighbor);
        void HandleHelloAck(const MeshMessage& message, DroneID from_neighbor);
        void HandleRouteRequest(const MeshMessage& message, DroneID from_neighbor);
        void HandleRouteReply(const MeshMessage& message, DroneID from_neighbor);
        void HandleDataMessage(const MeshMessage& message, DroneID from_neighbor);
        void HandleAckMessage(const MeshMessage& message, DroneID from_neighbor);
        void HandleEmergencyMessage(const MeshMessage& message, DroneID from_neighbor);

        // Network maintenance
        void SendHelloMessage();
        void SendHelloAck(DroneID to_neighbor);
        void UpdateNeighborInfo(DroneID neighbor_id, int8_t rssi);
        void RemoveDeadNeighbors();
        void CleanupExpiredRoutes();
        void RetransmitPendingAcks();

        // Route discovery
        void InitiateRouteDiscovery(DroneID destination);
        void ForwardRouteRequest(const MeshMessage& message);
        void SendRouteReply(DroneID destination, const std::vector<DroneID>& route,
                            DroneID reply_to);

        // Message forwarding
        bool ForwardMessage(const MeshMessage& message);
        DroneID SelectBestNextHop(DroneID destination);
        bool ShouldForwardMessage(const MeshMessage& message);

        // Quality metrics
        void UpdateLinkQuality(DroneID neighbor_id);
        double CalculateRouteQuality(DroneID destination, const std::vector<DroneID>& route);
        double CalculateNeighborQuality(const NeighborInfo& neighbor);

        // Utility methods
        uint32_t GenerateSequenceNumber();
        bool IsSequenceSeen(uint32_t sequence);
        void MarkSequenceSeen(uint32_t sequence);
        uint16_t CalculateChecksum(const MeshHeader& header);
        bool ValidateMessage(const MeshMessage& message);

        // Low-level transmission (implemented by hardware layer)
        bool TransmitMessage(const MeshMessage& message);
        void BroadcastToNeighbors(const MeshMessage& message);

        // Time utilities
        uint32_t millis();  // Get current time in milliseconds
    };

// Factory function for creating mesh protocol instances
    std::shared_ptr<SwarmMeshProtocol> CreateMeshProtocol(DroneID drone_id);

// Utility functions
    namespace MeshUtils {
        std::string MessageTypeToString(MeshMessageType type);
        std::vector<uint8_t> SerializeDistributedCommand(const DistributedCommand& command);
        DistributedCommand DeserializeDistributedCommand(const std::vector<uint8_t>& data);
        std::vector<uint8_t> SerializeTelemetry(const TelemetryData& telemetry);
        TelemetryData DeserializeTelemetry(const std::vector<uint8_t>& data);
    }

} // namespace MeshNetwork