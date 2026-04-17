#include "comm_cloud.hpp"
#include "../../include/secrets.hpp"

#include <Arduino.h>
#include <SPI.h>
#include <cstring>
#include <LoRaWan_APP.h>


// namespace for internal linkage of helper functions and state
namespace {
    constexpr uint8_t LORAWAN_APP_PORT = 1;
    constexpr uint8_t PAYLOAD_SIZE = sizeof(uint16_t) + sizeof(float);

    uint8_t g_payload[PAYLOAD_SIZE]; // buffer for payload construction
    uint8_t g_payload_len = 0;

    bool g_initialized = false; // flag to track initialization status
    bool g_payload_pending = false; // flag to track if there's a payload waiting to be sent

    // Helper function to build payload from AggregatedValue
    bool build_payload(const AggregatedValue& agg, uint8_t* out, uint8_t& out_len) {
        if (!out) return false;

        uint16_t window_id = agg.window_id;

        // window_id (2 byte)
        out[0] = (window_id >> 8) & 0xFF;
        out[1] = window_id & 0xFF;

        // average (4 byte float)
        std::memcpy(&out[2], &agg.mean, sizeof(float));

        out_len = 6;
        return true;
    }

    // Helper function to check if device has joined the network
    bool joined_state_reached() {
        return IsLoRaMacNetworkJoined;
    }

    // Helper function to queue a send operation if possible
    void queue_send_if_possible() {
        if (!g_payload_pending) {
            // No payload to send
            return;
        }

        if (!joined_state_reached()) {
            // Not joined yet
            return;
        }

        if (deviceState == DEVICE_STATE_SLEEP || deviceState == DEVICE_STATE_CYCLE) {
            // Transition to SEND state to trigger sending in the next loop iteration
            deviceState = DEVICE_STATE_SEND;
        }
    }
}

// OTAA keys
uint8_t devEui[8] = {0}; // device identifier
uint8_t appEui[8] = {0}; // JoinEUI
uint8_t appKey[16] = {0}; // application key

uint8_t nwkSKey[16] = {0};
uint8_t appSKey[16] = {0};
uint32_t devAddr = 0;

// LoRaWAN config
bool overTheAirActivation = true;
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;
bool loraWanAdr = true;
bool isTxConfirmed = false;
uint8_t appPort = LORAWAN_APP_PORT;
uint8_t confirmedNbTrials = 1;
DeviceClass_t loraWanClass = CLASS_A;

uint16_t userChannelsMask[6] = {0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

uint32_t appTxDutyCycle = 15000;

// Downlink callback (bytes received from the network)
void downLinkDataHandle(McpsIndication_t* mcpsIndication) {
    if (mcpsIndication == nullptr) {
        return;
    }
    Serial.print("[CLOUD] Downlink received, bytes=");
    Serial.println(mcpsIndication->BufferSize);
}

// Public API
void cloud_comm_init() {
    if (g_initialized) {
        return;
    }

    // Copy OTAA keys from secrets
    std::memcpy(appEui, TTN_JOIN_EUI, sizeof(appEui));
    std::memcpy(devEui, TTN_DEV_EUI, sizeof(devEui));
    std::memcpy(appKey, TTN_APP_KEY, sizeof(appKey));

    // Init radio/SPI
    SPI.begin(SCK, MISO, MOSI, SS);
    Mcu.begin();

    deviceState = DEVICE_STATE_INIT;
    g_initialized = true;
}

void cloud_comm_loop() {
    if (!g_initialized) {
        return;
    }

    switch (deviceState) {
        case DEVICE_STATE_INIT:
            Serial.println("[CLOUD] DEVICE_STATE_INIT: initializing LoRaWAN stack");
            LoRaWAN.init(loraWanClass, loraWanRegion);
            break;

        case DEVICE_STATE_JOIN:
            Serial.println("[CLOUD] DEVICE_STATE_JOIN: joining LoRaWAN network");
            LoRaWAN.join();
            break;

        case DEVICE_STATE_SEND:
            if (g_payload_pending) {
                std::memcpy(appData, g_payload, g_payload_len);
                appDataSize = g_payload_len;
                appPort = LORAWAN_APP_PORT;

                Serial.print("[CLOUD] DEVICE_STATE_SEND: Sending uplink, bytes=");
                Serial.println(appDataSize);

                LoRaWAN.send();
                g_payload_pending = false;
            }

            deviceState = DEVICE_STATE_CYCLE;
            break;

        case DEVICE_STATE_CYCLE:
            txDutyCycleTime = appTxDutyCycle;
            LoRaWAN.cycle(txDutyCycleTime);
            deviceState = DEVICE_STATE_SLEEP;
            break;

        case DEVICE_STATE_SLEEP:
            LoRaWAN.sleep(loraWanClass);
            queue_send_if_possible();
            break;

        default:
            deviceState = DEVICE_STATE_INIT;
            break;
    }
}

bool cloud_comm_is_ready() {
    return g_initialized && joined_state_reached() && !g_payload_pending;
}

bool cloud_comm_send(const AggregatedValue& agg) {
    if (!g_initialized) {
        Serial.println("[CLOUD] Not initialized");
        return false;
    }

    if (g_payload_pending) {
        Serial.println("[CLOUD] Previous payload still pending");
        return false;
    }

    if (!build_payload(agg, g_payload, g_payload_len)) {
        Serial.println("[CLOUD] Payload build failed");
        return false;
    }

    g_payload_pending = true;
    queue_send_if_possible();

    Serial.print("[CLOUD] Payload queued, bytes=");
    Serial.println(g_payload_len);

    return true;
}