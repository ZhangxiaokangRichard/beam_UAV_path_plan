/**
 * @file mqtt_client.cpp
 * @brief ros_mqtt_bridge 的 MQTT 通用客户端实现。
 *
 * 本类只负责 MQTT 连接生命周期、订阅、发布和断线重连。
 * MQTT 网络线程不直接操作 ROS Publisher，收到消息后通过回调交给上层；
 * 目标桥接节点再将消息放入队列，由 ROS 主线程完成 JSON 解析和 ROS 发布。
 */

#include "ros_mqtt_bridge/mqtt_client.h"

#include <mosquitto.h>

#include <cstring>

namespace ros_mqtt_bridge {

MqttClient::MqttClient(const std::string& client_id, const std::string& host, int port,
                       int keepalive, int qos, MessageHandler handler)
    : host_(host), port_(port), keepalive_(keepalive), qos_(qos), handler_(std::move(handler))
{
    // libmosquitto 需要进程级初始化；同一进程内的多个客户端共享该运行时。
    mosquitto_lib_init();
    mosq_ = mosquitto_new(client_id.c_str(), true, this);
    if (mosq_) {
        // userdata 指向当前 MqttClient，用于在 C 风格回调中恢复 C++ 对象上下文。
        mosquitto_connect_callback_set(mosq_, &MqttClient::onConnect);
        mosquitto_disconnect_callback_set(mosq_, &MqttClient::onDisconnect);
        mosquitto_message_callback_set(mosq_, &MqttClient::onMessage);
    }
}

MqttClient::~MqttClient()
{
    // 先设置关闭标志，避免主动断开时触发自动重连。
    shutting_down_ = true;
    if (mosq_) {
        mosquitto_loop_stop(mosq_, true);
        mosquitto_disconnect(mosq_);
        mosquitto_destroy(mosq_);
    }
    mosquitto_lib_cleanup();
}

void MqttClient::addSubscription(const std::string& topic, int qos)
{
    if (topic.empty()) return;
    subscriptions_.push_back({topic, qos < 0 ? qos_ : qos});
}

bool MqttClient::start(const std::string& subscription_topic)
{
    if (!mosq_) return false;
    // 兼容旧的单话题接口：传入 topic 时清空并以该话题作为唯一订阅；
    // 若已通过 addSubscription() 添加多话题，则保留这些订阅。
    if (!subscription_topic.empty()) {
        subscriptions_.clear();
        subscriptions_.push_back({subscription_topic, qos_});
    }
    // 使用异步连接，避免 Broker 暂不可用时阻塞 ROS 节点启动。
    if (mosquitto_connect_async(mosq_, host_.c_str(), port_, keepalive_) != MOSQ_ERR_SUCCESS)
        return false;
    return mosquitto_loop_start(mosq_) == MOSQ_ERR_SUCCESS;
}

bool MqttClient::publish(const std::string& topic, const std::string& payload, bool retain)
{
    // disconnected 时直接返回失败，由业务层保留待发送数据并在重连后重试。
    if (!mosq_ || !connected_) return false;
    return mosquitto_publish(mosq_, nullptr, topic.c_str(), static_cast<int>(payload.size()),
                             payload.data(), qos_, retain) == MOSQ_ERR_SUCCESS;
}

void MqttClient::onConnect(mosquitto* mosq, void* userdata, int result)
{
    auto* client = static_cast<MqttClient*>(userdata);
    client->connected_ = result == MOSQ_ERR_SUCCESS;
    // 每次重连都重新订阅；订阅状态不能假定会跨 TCP 连接保留。
    if (client->connected_) {
        for (const auto& sub : client->subscriptions_) {
            if (!sub.topic.empty())
                mosquitto_subscribe(mosq, nullptr, sub.topic.c_str(), sub.qos);
        }
    }
}

void MqttClient::onDisconnect(mosquitto*, void* userdata, int)
{
    auto* client = static_cast<MqttClient*>(userdata);
    client->connected_ = false;
    // 网络异常断开时交给 libmosquitto 异步重连；析构阶段则禁止重连。
    if (!client->shutting_down_ && client->mosq_)
        mosquitto_reconnect_async(client->mosq_);
}

void MqttClient::onMessage(mosquitto*, void* userdata, const ::mosquitto_message* message)
{
    auto* client = static_cast<MqttClient*>(userdata);
    // MQTT payload 不是以 '\0' 结尾的 C 字符串，必须使用 payloadlen 构造 std::string。
    if (!message || !message->topic || !message->payload || message->payloadlen < 0) return;
    client->handler_(message->topic,
                     std::string(static_cast<const char*>(message->payload), message->payloadlen));
}

}  // namespace ros_mqtt_bridge