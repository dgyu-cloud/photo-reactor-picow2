#include <cstdlib>
#include <iostream>

#include <MQTTClient.h>

namespace {
// MQTT broker settings.
constexpr char mqtt_broker_uri[] = "tcp://192.168.10.2";
constexpr char mqtt_client_id[] = "pc-demo-01";
constexpr char mqtt_topic[] = "labauto/pp/photoreactor/t1";

// Use nullptr for both fields if your broker allows anonymous clients.
constexpr const char* mqtt_username = nullptr;
constexpr const char* mqtt_password = nullptr;

// Edit this JSON and run the program to publish it once.
constexpr char json_payload[] = R"json({
  "is_running": true,
  "temp_pv": 52,
  
  "temp_sp": 60,
  "o2_conc":0.5,
  "peltier_pwm": 10,
  
  
})json";

constexpr int qos = 1;
constexpr unsigned long acknowledgement_timeout_ms = 5000;

int publish(MQTTClient client) {
    MQTTClient_deliveryToken token{};
    const int result = MQTTClient_publish(client, mqtt_topic, sizeof(json_payload) - 1,
                                          json_payload, qos, 0, &token);
    if (result != MQTTCLIENT_SUCCESS) {
        std::cerr << "MQTT publish failed (error " << result << ").\n";
        return EXIT_FAILURE;
    }

    const int acknowledgement = MQTTClient_waitForCompletion(
        client, token, acknowledgement_timeout_ms);
    if (acknowledgement != MQTTCLIENT_SUCCESS) {
        std::cerr << "No MQTT broker acknowledgement (error " << acknowledgement << ").\n";
        return EXIT_FAILURE;
    }

    std::cout << "Broker acknowledged the JSON sent to " << mqtt_topic << ".\n";
    return EXIT_SUCCESS;
}
}  // namespace

int main() {
    MQTTClient client{};
    const int creation = MQTTClient_create(&client, mqtt_broker_uri, mqtt_client_id,
                                           MQTTCLIENT_PERSISTENCE_NONE, nullptr);
    if (creation != MQTTCLIENT_SUCCESS) {
        std::cerr << "Could not create MQTT client (error " << creation << ").\n";
        return EXIT_FAILURE;
    }

    MQTTClient_connectOptions options = MQTTClient_connectOptions_initializer;
    options.username = mqtt_username;
    options.password = mqtt_password;
    options.connectTimeout = 5;

    const int connection = MQTTClient_connect(client, &options);
    int exit_code = EXIT_FAILURE;
    if (connection == MQTTCLIENT_SUCCESS) {
        exit_code = publish(client);
        MQTTClient_disconnect(client, 1000);
    } else {
        std::cerr << "Could not connect to " << mqtt_broker_uri
                  << " (error " << connection << ").\n";
    }

    MQTTClient_destroy(&client);
    return exit_code;
}
