/**
 * SPDX-FileCopyrightText: 2025 Maximiliano Ramirez <maximiliano.ramirezbravo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <Arduino.h>

// Configuration macros

// AT command new line sequence
#define AT_NL "\r\n"

// AT common responses
#define AT_OK    "OK"
#define AT_ERROR "ERROR"

// Modem TX buffer size
#ifndef SIM7600_MODEM_TX_BUFFER_SIZE_B
#define SIM7600_MODEM_TX_BUFFER_SIZE_B 256
#endif

// Modem RX buffer size
#ifndef SIM7600_MODEM_RX_BUFFER_SIZE_B
#define SIM7600_MODEM_RX_BUFFER_SIZE_B 256
#endif

// Modem initialization default timeout [ms]
#define SIM7600_MODEM_DEFAULT_INIT_TIMEOUT_MS 30000

// Modem operation default timeout [ms]
#define SIM7600_MODEM_DEFAULT_TIMEOUT_MS 1000

// Modem wait for network operations timeout [ms]
#define SIM7600_MODEM_NETWORK_TIMEOUT_MS 30000

// Modem min and max GPS antenna voltage [mV]
#define SIM7600_MODEM_MIN_GPS_ANTENNA_VOLTAGE_MV 1700
#define SIM7600_MODEM_MAX_GPS_ANTENNA_VOLTAGE_MV 3050

// Modem default NTP synchronization timeout [ms]
#define SIM7600_MODEM_NTP_SYNC_TIMEOUT_MS 10000

// Modem default loop duration [ms]
#define SIM7600_MODEM_DEFAULT_LOOP_DURATION_MS 100

// TCP max clients
#define SIM7600_TCP_MAX_CLIENTS 10

// TCP max response time [ms], including some margin over data timeout for AT commands
#define SIM7600_TCP_MAX_RESPONSE_TIME_MS 130000

// MQTT max clients
#define SIM7600_MQTT_MAX_CLIENTS 2

// MQTT max client length
#define SIM7600_MQTT_MAX_CLIENT_ID_LENGTH 128

// MQTT max server/username/password address length
#define SIM7600_MQTT_MAX_SERVER_USER_PASS_LENGTH 256

// MQTT max topic length
#define SIM7600_MQTT_MAX_TOPIC_LENGTH 1024

// MQTT max payload length
#define SIM7600_MQTT_MAX_PAYLOAD_LENGTH 10240

// MQTT sending/receiving data operation timeout [s]
#define SIM7600_MQTT_DATA_TIMEOUT_S 120

// MQTT min and max disconnect timeout [s]
#define SIM7600_MQTT_MIN_DISCONNECT_TIMEOUT_S 60
#define SIM7600_MQTT_MAX_DISCONNECT_TIMEOUT_S 120

// MQTT default keep-alive period [s]
#define SIM7600_MQTT_DEFAULT_KEEP_ALIVE_S 60

// MQTT max response time [ms], including some margin over data timeout for AT commands
#define SIM7600_MQTT_MAX_RESPONSE_TIME_MS 130000

// MQTT max and max keep-alive period [s]
#define SIM7600_MQTT_MIN_KEEP_ALIVE_S 1
#define SIM7600_MQTT_MAX_KEEP_ALIVE_S 64800

// MQTT default ports
#define SIM7600_MQTT_DEFAULT_PORT     1883
#define SIM7600_MQTT_DEFAULT_TLS_PORT 8883

// Check for std::function availability
#ifdef __has_include

#if __has_include(<functional>)
#define SIM7600_HAS_STD_FUNCTION 1
#else
#define SIM7600_HAS_STD_FUNCTION 0
#endif
#else
#if defined(ESP32) || defined(ESP8266)
#define SIM7600_HAS_STD_FUNCTION 1
#else
#define SIM7600_HAS_STD_FUNCTION 0
#endif
#endif

namespace SIM7600 {

// Status codes
#define SIM7600_STATUS_LIST        \
  X(Success)                       \
  X(Timeout)                       \
  X(Error)                         \
  X(BufferOverflow)                \
  X(ClientNotUsable)               \
  X(Disconnected)                  \
  X(EmptyLine)                     \
  X(GPSNotFixed)                   \
  X(InvalidAntennaVoltage)         \
  X(InvalidAPN)                    \
  X(InvalidBuffer)                 \
  X(InvalidCallback)               \
  X(InvalidClientName)             \
  X(InvalidHost)                   \
  X(InvalidModem)                  \
  X(InvalidParameter)              \
  X(InvalidPass)                   \
  X(InvalidPayload)                \
  X(InvalidPIN)                    \
  X(InvalidResponse)               \
  X(InvalidTimeout)                \
  X(InvalidTopic)                  \
  X(InvalidUser)                   \
  X(NoSerialPort)                  \
  X(NotConnected)                  \
  X(NoUserProvided)                \
  X(PINRequired)                   \
  X(SendBufferFullOrPeerCongested) \
  X(SerialSentLessThanExpected)    \
  X(TCPNoDataAvailable)            \
  X(TCPSentLessThanExpected)

#define X(name) name,
enum class Status { SIM7600_STATUS_LIST };
#undef X

/**
 * @brief Convert Status enum to string.
 * @param status Status enum value.
 * @return const char*: String representation of the status.
 */
const char* statusToString(Status status);

} // namespace SIM7600