#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "../../include/common.hpp"

// Initialize WiFi and MQTT connections
void edge_comm_init();

// Mantains WiFi and MQTT connections
void edge_comm_loop();

// Sends the aggregated value to the edge server over MQTT
bool edge_comm_send(const AggregatedValue& agg);
