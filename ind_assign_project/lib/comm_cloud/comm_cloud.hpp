#pragma once

#include "utils.hpp"

// Initialize LoRaWAN stack and radio
void cloud_comm_init();

// Mantains LoRaWAN state machine and handles transmissions
void cloud_comm_loop();

// Returns true if the cloud communication module is ready
bool cloud_comm_is_ready();

// Prepares and sends the aggregated value to the cloud via LoRaWAN
bool cloud_comm_send(const AggregatedValue& agg);


// STATE MACHINE OVERVIEW

//                     |
//                     v
//           +----------------------+
//           |  DEVICE_STATE_INIT   |
//           +----------------------+
//           | Set by:              |
//           | - cloud_comm_init()  |
//           | - fallback default   |
//           +----------------------+
//                     |
//                     | cloud_comm_loop()
//                     | calls LoRaWAN.init(...)
//                     | (library continues flow)
//                     v
//           +----------------------+
//           |  DEVICE_STATE_JOIN   |
//           +----------------------+
//           | Entered by:          |
//           | - Heltec library     |
//           |   after init         |
//           +----------------------+
//                     |
//                     | cloud_comm_loop()
//                     | calls LoRaWAN.join()
//                     v
//     +-------------------------------------------+
//     | JOIN procedure handled by Heltec library |
//     | - TX join-request                        |
//     | - open RX windows                        |
//     | - wait join-accept                       |
//     +-------------------------------------------+
//            |                            |
//            | join success               | join failed / no coverage
//            |                            | / RX timeout
//            v                            |
//   +----------------------+              |
//   |  joined = true       |              |
//   |  (library flag)      |              |
//   +----------------------+              |
//            |                            |
//            | next loop / cycle          |
//            |                            |
//            v                            |
//    +----------------------+             |
//    | DEVICE_STATE_SLEEP   |<------------+
//    +----------------------+
//    | Entered by:          |
//    | - our code after     |
//    |   DEVICE_STATE_CYCLE |
//    | - library scheduling |
//    +----------------------+
//            |
//            | cloud_comm_loop()
//            | calls LoRaWAN.sleep(...)
//            |
//            | queue_send_if_possible()
//            | checks:
//            |   - payload pending?
//            |   - joined?
//            |   - state is SLEEP/CYCLE?
//            |
//            +------------------------------+
//                                           |
//                                           | if YES
//                                           v
//                              +----------------------+
//                              |  DEVICE_STATE_SEND   |
//                              +----------------------+
//                              | Set by:              |
//                              | - our helper         |
//                              |   queue_send_if_     |
//                              |   possible()         |
//                              +----------------------+
//                                           |
//                                           | cloud_comm_loop()
//                                           | copies:
//                                           |   g_payload -> appData
//                                           |   g_payload_len -> appDataSize
//                                           | then calls LoRaWAN.send()
//                                           v
//                              +----------------------+
//                              |  DEVICE_STATE_CYCLE  |
//                              +----------------------+
//                              | Set by:              |
//                              | - our code after     |
//                              |   LoRaWAN.send()     |
//                              +----------------------+
//                                           |
//                                           | cloud_comm_loop()
//                                           | sets txDutyCycleTime
//                                           | calls LoRaWAN.cycle(...)
//                                           v
//                              +----------------------+
//                              |  DEVICE_STATE_SLEEP  |
//                              +----------------------+
//                                           |
//                                           +----> waits for next
//                                                 payload / next cycle
