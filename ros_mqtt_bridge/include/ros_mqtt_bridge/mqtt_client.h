#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

struct mosquitto;
struct mosquitto_message;

namespace ros_mqtt_bridge {

class MqttClient {
public:
    using MessageHandler = std::function<void(const std::string&, const std::string&)>;

    MqttClient(const std::string& client_id, const std::string& host, int port,
               int keepalive, int qos, MessageHandler handler);
    ~MqttClient();

    // 单话题订阅（兼容旧接口）：调用后清空其它订阅，以该话题为唯一订阅。
    bool start(const std::string& subscription_topic = "");

    // 多话题订阅：连接成功 / 断线重连时自动订阅全部。须在 start() 之前调用。
    void addSubscription(const std::string& topic, int qos = -1);

    bool publish(const std::string& topic, const std::string& payload, bool retain = false);
    bool connected() const { return connected_; }

private:
    static void onConnect(mosquitto* mosq, void* userdata, int result);
    static void onDisconnect(mosquitto* mosq, void* userdata, int result);
    static void onMessage(mosquitto* mosq, void* userdata, const ::mosquitto_message* message);

    mosquitto* mosq_ = nullptr;
    std::string host_;
    int port_ = 1883;
    int keepalive_ = 30;
    int qos_ = 1;
    struct Subscription { std::string topic; int qos; };
    std::vector<Subscription> subscriptions_;
    MessageHandler handler_;
    bool connected_ = false;
    bool shutting_down_ = false;
};

}  // namespace ros_mqtt_bridge