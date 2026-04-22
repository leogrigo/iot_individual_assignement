#include "comm_edge.hpp"
#include "../../include/secrets.hpp"

#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_wifi.h>

// namespace for internal linkage of helper functions and state
namespace {
    constexpr uint16_t MQTT_PORT     = 1883;

    constexpr char MQTT_CLIENT_ID[]  = "esp32-node-01";
    constexpr char MQTT_TOPIC_EDGE[] = "iot/node01/aggregate";

    WiFiClient wifiClient;
    PubSubClient mqttClient(wifiClient);

    void configure_wifi_modem_sleep() {
        WiFi.setSleep(true);
        const esp_err_t err = esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
        if (err != ESP_OK) {
            Serial.printf("[EDGE] Failed to enable WiFi modem sleep, err=%d\n", err);
        }
    }

    // Helper function to ensure WiFi connection
    bool ensure_wifi_connected(uint32_t timeout_ms = 10000) {
        if (WiFi.status() == WL_CONNECTED) {
            return true;
        }

        WiFi.disconnect(false, false);
        WiFi.mode(WIFI_STA);
        configure_wifi_modem_sleep();
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

        const uint32_t start_ms = millis();
        while (WiFi.status() != WL_CONNECTED && (millis() - start_ms) < timeout_ms) {
            vTaskDelay(pdMS_TO_TICKS(250));
        }

        return WiFi.status() == WL_CONNECTED;
    }

    // Helper function to ensure MQTT connection
    bool ensure_mqtt_connected(uint32_t timeout_ms = 5000) {
        if (mqttClient.connected()) {
            return true;
        }

        if (WiFi.status() != WL_CONNECTED) {
            return false;
        }

        const uint32_t start_ms = millis();

        while (!mqttClient.connected() && (millis() - start_ms) < timeout_ms) {
            if (mqttClient.connect(MQTT_CLIENT_ID)) {
                return true;
            }

            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        return mqttClient.connected();
    }

    // Helper function to build JSON payload from AggregatedValue
    bool build_payload(const AggregatedValue& agg, char* out, size_t out_size) {
        if (!out || out_size == 0) {
            return false;
        }

        // JSON payload format:
        const int written = snprintf(
            out,
            out_size,
            "{\"node_id\":\"%s\",\"window_id\":%lu,\"aggregate\":%.3f,\"duration_ms\":%.1f,\"sample_count\":%lu}",
            MQTT_CLIENT_ID,
            static_cast<unsigned long>(agg.window_id),
            agg.mean,
            agg.duration_ms,
            static_cast<unsigned long>(agg.sample_count)
        );

        return (written > 0 && static_cast<size_t>(written) < out_size);
    }
}


// Public API
void edge_comm_init() {
    WiFi.mode(WIFI_STA);
    configure_wifi_modem_sleep();
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    Serial.println("[EDGE] MQTT client initialized");
}

void edge_comm_loop() {
    if (WiFi.status() != WL_CONNECTED) {
        (void)ensure_wifi_connected();
    }

    if (WiFi.status() == WL_CONNECTED && !mqttClient.connected()) {
        (void)ensure_mqtt_connected();
    }

    if (mqttClient.connected()) {
        mqttClient.loop();
    }
}

bool edge_comm_send(const AggregatedValue& agg) {
    if (!ensure_wifi_connected()) {
        Serial.println("[EDGE] WiFi not connected");
        return false;
    }

    if (!ensure_mqtt_connected()) {
        Serial.print("[EDGE] MQTT not connected, state=");
        Serial.println(mqttClient.state());
        return false;
    }

    char payload[160];
    if (!build_payload(agg, payload, sizeof(payload))) {
        Serial.println("[EDGE] Payload build failed");
        return false;
    }

    // Serial.printf("[METRIC][VDT] payload_size=%lu\n", sizeof(payload));
    bool ok = mqttClient.publish(MQTT_TOPIC_EDGE, payload);
    if (!ok) {
        Serial.print("[EDGE] Publish failed, state=");
        Serial.println(mqttClient.state());
    } else {
        Serial.printf("[EDGE] Published to MQTT: %s\n", payload);
    }

    return ok;
}
