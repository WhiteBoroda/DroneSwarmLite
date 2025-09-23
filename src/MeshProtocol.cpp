#include "../include/MeshProtocol.h"
#include "../include/SwarmManager.h"
#include <algorithm>
#include <random>
#include <cstring>
#include <iostream>

namespace MeshNetwork {

    MeshProtocol::MeshProtocol(DroneID my_id) : my_id_(my_id) {
        last_hello_time_ = 0;
        last_route_cleanup_ = 0;
        last_neighbor_discovery_ = 0;

        // Очищення статистики
        memset(&stats_, 0, sizeof(stats_));

        // Ініціалізація генератора послідовних номерів
        std::random_device rd;
        sequence_generator_.seed(rd());
    }

    bool MeshProtocol::Initialize() {
        std::cout << "📡 Ініціалізація Mesh-протоколу для дрона " << my_id_ << std::endl;

        // Очищення всіх таблиць
        neighbors_.clear();
        routing_table_.clear();
        seen_sequences_.clear();
        pending_acks_.clear();

        // Очищення черг
        while (!outgoing_queue_.empty()) outgoing_queue_.pop();
        while (!forwarding_queue_.empty()) forwarding_queue_.pop();

        std::cout << "✅ Mesh-протокол ініціалізовано" << std::endl;
        return true;
    }

    bool MeshProtocol::Start() {
        std::cout << "🚀 Запуск Mesh-протоколу..." << std::endl;

        // Відправляємо початковий HELLO
        SendHelloMessage();

        std::cout << "✅ Mesh-протокол запущено" << std::endl;
        return true;
    }

    void MeshProtocol::Update() {
        uint32_t current_time = millis();

        // Відправка періодичних HELLO (кожні 5 секунд)
        if (current_time - last_hello_time_ > 5000) {
            SendHelloMessage();
            last_hello_time_ = current_time;
        }

        // Очищення застарілих маршрутів (кожні 30 секунд)
        if (current_time - last_route_cleanup_ > 30000) {
            CleanupExpiredRoutes();
            last_route_cleanup_ = current_time;
        }

        // Видалення мертвих сусідів (кожні 10 секунд)
        if (current_time - last_neighbor_discovery_ > 10000) {
            RemoveDeadNeighbors();
            last_neighbor_discovery_ = current_time;
        }

        // Обробка черг повідомлень
        ProcessOutgoingQueue();
        ProcessForwardingQueue();
    }

    bool MeshProtocol::Stop() {
        std::cout << "⏹️ Зупинка Mesh-протоколу..." << std::endl;

        // Відправляємо повідомлення про від'єднання
        SendGoodbyeMessage();

        // Очищення всіх структур даних
        neighbors_.clear();
        routing_table_.clear();
        seen_sequences_.clear();
        pending_acks_.clear();

        std::cout << "✅ Mesh-протокол зупинено" << std::endl;
        return true;
    }

    bool MeshProtocol::SendMessage(DroneID destination, const std::vector<uint8_t>& data, uint8_t priority) {
        if (data.size() > 512) {
            std::cerr << "❌ Повідомлення занадто велике: " << data.size() << " байт" << std::endl;
            return false;
        }

        // Створюємо mesh-повідомлення
        MeshMessage message;
        message.header.message_type = MESH_DATA;
        message.header.source_id = my_id_;
        message.header.destination_id = destination;
        message.header.sequence_number = GenerateSequenceNumber();
        message.header.timestamp = millis();
        message.header.priority = priority;
        message.header.flags = FLAG_ACK_REQUIRED;
        message.header.payload_size = data.size();
        message.header.hop_count = 0;
        message.header.max_hops = 10;

        // Копіюємо дані
        std::copy(data.begin(), data.end(), message.payload);

        // Знаходимо маршрут
        if (destination == 0) {
            // Broadcast
            return BroadcastMessage(data, priority);
        } else {
            // Unicast - знаходимо маршрут
            auto route_it = routing_table_.find(destination);
            if (route_it != routing_table_.end()) {
                message.header.next_hop = route_it->second.next_hop;
                return TransmitMessage(message);
            } else {
                // Маршрут не знайдено - запускаємо пошук
                std::cout << "🔍 Пошук маршруту до дрона " << destination << std::endl;
                SendRouteRequest(destination);

                // Додаємо повідомлення в чергу на пізніше
                outgoing_queue_.push(message);
                return true;
            }
        }
    }

    bool MeshProtocol::BroadcastMessage(const std::vector<uint8_t>& data, uint8_t priority) {
        MeshMessage message;
        message.header.message_type = MESH_DATA;
        message.header.source_id = my_id_;
        message.header.destination_id = 0; // Broadcast
        message.header.sequence_number = GenerateSequenceNumber();
        message.header.timestamp = millis();
        message.header.priority = priority;
        message.header.flags = FLAG_BROADCAST;
        message.header.payload_size = data.size();
        message.header.hop_count = 0;
        message.header.max_hops = 5; // Менше hop'ів для broadcast

        std::copy(data.begin(), data.end(), message.payload);

        // Відправляємо всім сусідам
        bool success = false;
        for (const auto& [neighbor_id, neighbor_info] : neighbors_) {
            if (neighbor_info.is_reliable) {
                message.header.next_hop = neighbor_id;
                if (TransmitMessage(message)) {
                    success = true;
                }
            }
        }

        stats_.packets_sent++;
        return success;
    }

    bool MeshProtocol::SendEmergencyMessage(const std::vector<uint8_t>& data) {
        MeshMessage message;
        message.header.message_type = MESH_EMERGENCY;
        message.header.source_id = my_id_;
        message.header.destination_id = 0; // Broadcast
        message.header.sequence_number = GenerateSequenceNumber();
        message.header.timestamp = millis();
        message.header.priority = 0; // Найвищий пріоритет
        message.header.flags = FLAG_EMERGENCY | FLAG_BROADCAST;
        message.header.payload_size = data.size();
        message.header.hop_count = 0;
        message.header.max_hops = 15; // Більше hop'ів для аварійних

        std::copy(data.begin(), data.end(), message.payload);

        // Відправляємо ВСІМ сусідам незалежно від надійності
        bool success = false;
        for (const auto& [neighbor_id, neighbor_info] : neighbors_) {
            message.header.next_hop = neighbor_id;
            if (TransmitMessage(message)) {
                success = true;
            }
        }

        std::cout << "🚨 Аварійне повідомлення розіслане через mesh" << std::endl;
        return success;
    }

    void MeshProtocol::ProcessIncomingMessage(const MeshMessage& message) {
        stats_.packets_received++;

        // Перевірка на дублікати
        if (IsSequenceSeen(message.header.sequence_number)) {
            std::cout << "⚠️ Дублікат повідомлення від " << message.header.source_id << std::endl;
            return;
        }
        MarkSequenceSeen(message.header.sequence_number);

        // Оновлюємо інформацію про сусіда
        UpdateNeighborFromMessage(message);

        // Обробляємо залежно від типу
        switch (message.header.message_type) {
            case MESH_HELLO:
                ProcessHelloMessage(message);
                break;

            case MESH_ROUTE_REQUEST:
                ProcessRouteRequest(message);
                break;

            case MESH_ROUTE_REPLY:
                ProcessRouteReply(message);
                break;

            case MESH_DATA:
                ProcessDataMessage(message);
                break;

            case MESH_ACK:
                ProcessAckMessage(message);
                break;

            case MESH_ROUTE_ERROR:
                ProcessRouteError(message);
                break;

            case MESH_HEARTBEAT:
                ProcessHeartbeat(message);
                break;

            case MESH_EMERGENCY:
                ProcessEmergencyMessage(message);
                break;

            default:
                std::cout << "⚠️ Невідомий тип mesh-повідомлення: "
                          << static_cast<int>(message.header.message_type) << std::endl;
                break;
        }
    }

    void MeshProtocol::ProcessHelloMessage(const MeshMessage& message) {
        std::cout << "👋 HELLO від дрона " << message.header.source_id << std::endl;

        // Оновлюємо або додаємо сусіда
        NeighborInfo neighbor;
        neighbor.id = message.header.source_id;
        neighbor.last_seen = millis();
        neighbor.rssi = -60; // Тут має бути реальний RSSI
        neighbor.is_reliable = true;
        neighbor.hop_count = 1; // Прямий сусід

        neighbors_[message.header.source_id] = neighbor;

        // Відправляємо HELLO у відповідь якщо це новий сусід
        if (neighbors_.find(message.header.source_id) == neighbors_.end()) {
            SendHelloMessage();
        }
    }

    void MeshProtocol::ProcessRouteRequest(const MeshMessage& message) {
        DroneID destination = *reinterpret_cast<const DroneID*>(message.payload);

        std::cout << "🔍 ROUTE_REQUEST від " << message.header.source_id
                  << " до " << destination << std::endl;

        if (destination == my_id_) {
            // Це запит до мене - відправляємо відповідь
            SendRouteReply(destination, message.header.source_id);
        } else {
            // Форвардимо запит далі (якщо TTL дозволяє)
            if (ShouldForwardMessage(message)) {
                ForwardMessage(message);
            }
        }
    }

    void MeshProtocol::ProcessRouteReply(const MeshMessage& message) {
        std::cout << "✅ ROUTE_REPLY від " << message.header.source_id << std::endl;

        // Додаємо або оновлюємо маршрут
        RouteEntry route;
        route.destination = message.header.source_id;
        route.next_hop = message.header.previous_hop;
        route.hop_count = message.header.hop_count;
        route.expiry_time = millis() + 300000; // 5 хвилин
        route.link_quality = CalculateLinkQuality(neighbors_[route.next_hop]);

        routing_table_[route.destination] = route;

        std::cout << "📊 Маршрут до " << route.destination
                  << " через " << route.next_hop
                  << " (" << static_cast<int>(route.hop_count) << " hops)" << std::endl;
    }

    void MeshProtocol::ProcessDataMessage(const MeshMessage& message) {
        if (message.header.destination_id == my_id_ || message.header.destination_id == 0) {
            // Повідомлення для мене
            std::cout << "📨 Отримано дані від " << message.header.source_id
                      << " (" << message.header.payload_size << " байт)" << std::endl;

            // Відправляємо ACK якщо потрібно
            if (message.header.flags & FLAG_ACK_REQUIRED) {
                SendAckMessage(message.header.source_id, message.header.sequence_number);
            }

            // Передаємо дані в додаток
            std::vector<uint8_t> data(message.payload,
                                      message.payload + message.header.payload_size);
            OnDataReceived(message.header.source_id, data);

        } else {
            // Повідомлення не для мене - форвардимо
            if (ShouldForwardMessage(message)) {
                ForwardMessage(message);
            }
        }
    }

    void MeshProtocol::ProcessEmergencyMessage(const MeshMessage& message) {
        std::cout << "🚨 АВАРІЙНЕ повідомлення від " << message.header.source_id << std::endl;

        // Аварійні повідомлення обробляємо з найвищим пріоритетом
        std::vector<uint8_t> data(message.payload,
                                  message.payload + message.header.payload_size);
        OnEmergencyReceived(message.header.source_id, data);

        // Форвардимо аварійне повідомлення далі
        if (message.header.hop_count < message.header.max_hops) {
            ForwardMessage(message);
        }
    }

    void MeshProtocol::SendHelloMessage() {
        MeshMessage hello;
        hello.header.message_type = MESH_HELLO;
        hello.header.source_id = my_id_;
        hello.header.destination_id = 0; // Broadcast
        hello.header.sequence_number = GenerateSequenceNumber();
        hello.header.timestamp = millis();
        hello.header.flags = FLAG_BROADCAST;
        hello.header.hop_count = 0;
        hello.header.max_hops = 1; // HELLO тільки прямим сусідам

        // В payload можемо додати додаткову інформацію
        hello.header.payload_size = 0;

        // Broadcast до всіх
        BroadcastToNeighbors(hello);

        std::cout << "👋 HELLO надіслано сусідам" << std::endl;
    }

    void MeshProtocol::SendRouteRequest(DroneID destination) {
        MeshMessage request;
        request.header.message_type = MESH_ROUTE_REQUEST;
        request.header.source_id = my_id_;
        request.header.destination_id = 0; // Broadcast
        request.header.sequence_number = GenerateSequenceNumber();
        request.header.timestamp = millis();
        request.header.flags = FLAG_ROUTE_DISCOVERY;
        request.header.hop_count = 0;
        request.header.max_hops = 10;

        // В payload вказуємо кого шукаємо
        *reinterpret_cast<DroneID*>(request.payload) = destination;
        request.header.payload_size = sizeof(DroneID);

        BroadcastToNeighbors(request);

        stats_.route_discoveries++;
        std::cout << "🔍 ROUTE_REQUEST для " << destination << " надіслано" << std::endl;
    }

    void MeshProtocol::SendRouteReply(DroneID destination, DroneID requester) {
        MeshMessage reply;
        reply.header.message_type = MESH_ROUTE_REPLY;
        reply.header.source_id = my_id_;
        reply.header.destination_id = requester;
        reply.header.sequence_number = GenerateSequenceNumber();
        reply.header.timestamp = millis();
        reply.header.hop_count = 0;
        reply.header.max_hops = 10;

        // Знаходимо маршрут назад до requester
        auto route_it = routing_table_.find(requester);
        if (route_it != routing_table_.end()) {
            reply.header.next_hop = route_it->second.next_hop;
            TransmitMessage(reply);

            std::cout << "✅ ROUTE_REPLY для " << requester << " надіслано" << std::endl;
        }
    }

    bool MeshProtocol::ShouldForwardMessage(const MeshMessage& message) {
        // Перевірка TTL
        if (message.header.hop_count >= message.header.max_hops) {
            return false;
        }

        // Не форвардимо власні повідомлення
        if (message.header.source_id == my_id_) {
            return false;
        }

        // Перевірка на дублікати
        if (IsSequenceSeen(message.header.sequence_number)) {
            return false;
        }

        return true;
    }

    void MeshProtocol::ForwardMessage(const MeshMessage& original_message) {
        MeshMessage forwarded = original_message;
        forwarded.header.hop_count++;
        forwarded.header.previous_hop = my_id_;

        // Знаходимо найкращий наступний hop
        if (forwarded.header.destination_id != 0) {
            // Unicast - знаходимо конкретний маршрут
            DroneID next_hop = SelectBestNextHop(forwarded.header.destination_id);
            if (next_hop != 0) {
                forwarded.header.next_hop = next_hop;
                TransmitMessage(forwarded);
                stats_.packets_forwarded++;
            }
        } else {
            // Broadcast - форвардимо всім сусідам крім того, від кого отримали
            for (const auto& [neighbor_id, neighbor_info] : neighbors_) {
                if (neighbor_id != original_message.header.previous_hop &&
                    neighbor_info.is_reliable) {
                    forwarded.header.next_hop = neighbor_id;
                    TransmitMessage(forwarded);
                }
            }
            stats_.packets_forwarded++;
        }

        std::cout << "🔄 Повідомлення переслано (hop "
                  << static_cast<int>(forwarded.header.hop_count) << ")" << std::endl;
    }

    DroneID MeshProtocol::SelectBestNextHop(DroneID destination) {
        auto route_it = routing_table_.find(destination);
        if (route_it != routing_table_.end() &&
            route_it->second.expiry_time > millis()) {
            return route_it->second.next_hop;
        }

        // Немає конкретного маршруту - вибираємо найкращого сусіда
        DroneID best_neighbor = 0;
        double best_quality = -1.0;

        for (const auto& [neighbor_id, neighbor_info] : neighbors_) {
            if (neighbor_info.is_reliable) {
                double quality = CalculateLinkQuality(neighbor_info);
                if (quality > best_quality) {
                    best_quality = quality;
                    best_neighbor = neighbor_id;
                }
            }
        }

        return best_neighbor;
    }

    double MeshProtocol::CalculateLinkQuality(const NeighborInfo& neighbor) {
        // Комбінуємо різні фактори якості зв'язку
        double rssi_factor = (neighbor.rssi + 120.0) / 70.0; // Нормалізуємо -120 до -50
        double age_factor = 1.0 - std::min(1.0, (millis() - neighbor.last_seen) / 30000.0);
        double reliability_factor = neighbor.is_reliable ? 1.0 : 0.1;
        double loss_factor = 1.0 - (neighbor.packet_loss / 100.0);

        return (rssi_factor * 0.4 + age_factor * 0.2 +
                reliability_factor * 0.2 + loss_factor * 0.2);
    }

    uint32_t MeshProtocol::GenerateSequenceNumber() {
        static uint32_t sequence = 0;
        return ++sequence;
    }

    bool MeshProtocol::IsSequenceSeen(uint32_t sequence) {
        return seen_sequences_.find(sequence) != seen_sequences_.end();
    }

    void MeshProtocol::MarkSequenceSeen(uint32_t sequence) {
        seen_sequences_.insert(sequence);

        // Обмежуємо розмір кешу
        if (seen_sequences_.size() > 10000) {
            // Видаляємо найстаріші записи (спрощена версія)
            seen_sequences_.clear();
        }
    }

// Допоміжні функції (заглушки для інтеграції)
    bool MeshProtocol::TransmitMessage(const MeshMessage& message) {
        // Тут має бути реальна передача через LoRa
        std::cout << "📡 TX mesh: " << static_cast<int>(message.header.message_type)
                  << " від " << message.header.source_id
                  << " до " << message.header.next_hop << std::endl;
        return true;
    }

    void MeshProtocol::BroadcastToNeighbors(const MeshMessage& message) {
        for (const auto& [neighbor_id, neighbor_info] : neighbors_) {
            if (neighbor_info.is_reliable) {
                MeshMessage msg_copy = message;
                msg_copy.header.next_hop = neighbor_id;
                TransmitMessage(msg_copy);
            }
        }
    }

    uint32_t MeshProtocol::millis() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
    }

} // namespace MeshNetwork