#pragma once

#include <functional>
#include <memory>
#include <string>

struct mosquitto;
struct mosquitto_message;

namespace ros_mqtt_bridge {

class MqttClient {
public:
    using MessageHandler = std::function<void(const std::string&, const std::string&)>;

    MqttClient(const std::string& client_id, const std::string& host, int port,
               int keepalive, int qos, MessageHandler handler);
    ~MqttClient();

    bool start(const std::string& subscription_topic = "");
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
    std::string subscription_topic_;
    MessageHandler handler_;
    bool connected_ = false;
    bool shutting_down_ = false;
};

}  // namespace ros_mqtt_bridge