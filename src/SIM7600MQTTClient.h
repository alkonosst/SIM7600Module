/**
 * SPDX-FileCopyrightText: 2025 Maximiliano Ramirez <maximiliano.ramirezbravo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include "SIM7600ClientManager.h"
#include "SIM7600Common.h"
#include "SIM7600Modem.h"

namespace SIM7600 {

/// @brief MQTT protocol version.
enum class MQTTVersion { V3_1 = 3, V3_1_1 = 4 };

/// @brief MQTT QoS levels
enum class MQTTQoS { AtMostOnce = 0, AtLeastOnce = 1, ExactlyOnce = 2 };

/// @brief MQTT connection lost cause
enum class MQTTConnLostCause { SocketClosedPassively = 1, SocketReset, NetworkClosed };

class MQTTClient : public ClientManager<MQTTClient, SIM7600_MQTT_MAX_CLIENTS> {
  public:
  // Callback function types. Use std::function if available.
#if SIM7600_HAS_STD_FUNCTION
  using ConnectionLostCB =
    std::function<void(MQTTClient* const client, const MQTTConnLostCause cause)>;
  using PingFailedCB = std::function<void(MQTTClient* const client)>;
  using RxStartCB    = std::function<void(MQTTClient* const client, const size_t topic_length,
    const size_t payload_length)>;
  using RxTopicCB =
    std::function<void(MQTTClient* const client, const char* topic, const size_t topic_length)>;
  using RxPayloadCB = std::function<void(MQTTClient* const client, uint8_t* const payload,
    const size_t payload_length)>;
  using RxEndCB     = std::function<void(MQTTClient* const client)>;
#else
  using ConnectionLostCB = void (*)(MQTTClient* const client, const MQTTConnLostCause cause);
  using PingFailedCB     = void (*)(MQTTClient* const client);
  using RxStartCB        = void (*)(MQTTClient* const client, const size_t topic_length,
    const size_t payload_length);
  using RxTopicCB        = void (*)(MQTTClient* const client, const char* topic,
    const size_t topic_length);
  using RxPayloadCB      = void (*)(MQTTClient* const client, uint8_t* const payload,
    const size_t payload_length);
  using RxEndCB          = void (*)(MQTTClient* const client);
#endif

  /**
   * @brief Create a new MQTTClient instance. You need to set the modem later using setModem().
   */
  MQTTClient();

  /**
   * @brief Create a new MQTTClient instance with the given modem.
   * @param modem Modem instance.
   */
  MQTTClient(Modem* modem);

  /**
   * @brief Destroy the MQTTClient instance.
   */
  ~MQTTClient();

  /**
   * @brief Set the Modem instance to be used by the MQTTClient.
   * @param modem Modem instance.
   * @return Status::Success on success, error code otherwise.
   */
  Status setModem(Modem* const modem);

  /**
   * @brief Set the connection lost callback.
   * @param callback Callback function to be called when the MQTT connection is lost.
   * @return Status::Success on success, error code otherwise.
   */
  Status setConnectionLostCallback(ConnectionLostCB callback);

  /**
   * @brief Set the ping failed callback.
   * @param callback Callback function to be called when the MQTT ping fails.
   * @return Status::Success on success, error code otherwise.
   */
  Status setPingFailedCallback(PingFailedCB callback);

  /**
   * @brief Set the Received Start callback.
   * @param callback Callback function to be called when a new MQTT message starts to be received.
   * @return Status::Success on success, error code otherwise.
   */
  Status setRxStartCallback(RxStartCB callback);

  /**
   * @brief Set the Received Topic callback.
   * @param callback Callback function to be called when a new MQTT message topic is received.
   * @return Status::Success on success, error code otherwise.
   */
  Status setRxTopicCallback(RxTopicCB callback);

  /**
   * @brief Set the Received Payload callback.
   * @param callback Callback function to be called when a new MQTT message payload is received.
   * @return Status::Success on success, error code otherwise.
   */
  Status setRxPayloadCallback(RxPayloadCB callback);

  /**
   * @brief Set the Received End callback.
   * @param callback Callback function to be called when a new MQTT message has been fully received.
   * @return Status::Success on success, error code otherwise.
   */
  Status setRxEndCallback(RxEndCB callback);

  /**
   * @brief Acquire an MQTT client with the given parameters. You must acquire a client before
   * connecting.
   * @param client_id Client identifier for the SIM7600. Must be unique among other MQTT clients.
   * @param use_tls Whether to use TLS for the connection.
   * @param version MQTT protocol version.
   * @return Status::Success on success, error code otherwise.
   */
  Status acquireClient(const char* client_id, const bool use_tls = false,
    const MQTTVersion version = MQTTVersion::V3_1_1);

  /**
   * @brief Release the MQTT client.
   * @return Status::Success on success, error code otherwise.
   */
  Status releaseClient();

  /**
   * @brief Set the Last Will and Testament topic and message.
   * @param topic Topic string.
   * @return Status::Success on success, error code otherwise.
   */
  Status setLastWillMessage(const char* topic, const char* message, const MQTTQoS qos);

  /**
   * @brief Connect to the MQTT broker.
   * @param host Broker hostname.
   * @param port Broker port number.
   * @param username Username for authentication (optional).
   * @param password Password for authentication (optional).
   * @param keep_alive_s Keep alive interval in seconds. Value must be between
   * SIM7600_MQTT_MIN_KEEP_ALIVE_S and SIM7600_MQTT_MAX_KEEP_ALIVE_S.
   * @param clean_session Whether to start a clean session.
   * @return Status::Success on success, error code otherwise.
   */
  Status connect(const char* host, const uint16_t port = SIM7600_MQTT_DEFAULT_PORT,
    const char* username = nullptr, const char* password = nullptr,
    const uint16_t keep_alive_s = SIM7600_MQTT_DEFAULT_KEEP_ALIVE_S,
    const bool clean_session    = true);

  /**
   * @brief Disconnect from the MQTT broker.
   * @param timeout_s Timeout in seconds for the disconnect operation. Value must be between
   * SIM7600_MQTT_MIN_DISCONNECT_TIMEOUT_S and SIM7600_MQTT_MAX_DISCONNECT_TIMEOUT_S.
   * @return Status::Success on success, error code otherwise.
   */
  Status disconnect(const uint8_t timeout_s = SIM7600_MQTT_MAX_DISCONNECT_TIMEOUT_S);

  /**
   * @brief Check if the client is connected to the MQTT broker.
   * @param connected Reference to store the connected status.
   * @return Status::Success on success, error code otherwise.
   */
  Status isConnected(bool& connected);

  /**
   * @brief Publish a message to a topic.
   * @param topic Topic string.
   * @param payload Payload data buffer.
   * @param payload_length Length of the payload data.
   * @param qos Quality of Service level.
   * @param retained Whether the message should be retained by the broker.
   * @return Status::Success on success, error code otherwise.
   */
  Status publish(const char* topic, const uint8_t* payload, const size_t payload_length,
    const MQTTQoS qos, const bool retained = false);

  /**
   * @brief Subscribe to a topic.
   * @param topic Topic string.
   * @param qos Quality of Service level.
   * @return Status::Success on success, error code otherwise.
   */
  Status subscribe(const char* topic, const MQTTQoS qos);

  /**
   * @brief Unsubscribe from a topic.
   * @param topic Topic string.
   * @return Status::Success on success, error code otherwise.
   */
  Status unsubscribe(const char* topic);

  private:
  Modem* _modem;
  char* _tx_buf;
  char* _rx_buf;

  ConnectionLostCB _cb_connection_lost;
  PingFailedCB _cb_ping_failed;
  RxStartCB _cb_rx_start;
  RxTopicCB _cb_rx_topic;
  RxPayloadCB _cb_rx_payload;
  RxEndCB _cb_rx_end;

  static bool _handleMQTTURCs(const char* line);
};

} // namespace SIM7600