/**
 * SPDX-FileCopyrightText: 2025 Maximiliano Ramirez <maximiliano.ramirezbravo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */

#include "SIM7600MQTTClient.h"
#include "SIM7600Log.h"

static const char* tag = "SIM7600MQTTClient";

namespace SIM7600 {

MQTTClient::MQTTClient()
    : _modem(nullptr)
    , _tx_buf(nullptr)
    , _rx_buf(nullptr)
    , _cb_connection_lost(nullptr)
    , _cb_ping_failed(nullptr)
    , _cb_rx_start(nullptr)
    , _cb_rx_topic(nullptr)
    , _cb_rx_payload(nullptr)
    , _cb_rx_end(nullptr) {
  if (!isUsable()) {
    SIM7600_LOGE(tag, "Maximum number of MQTT clients reached");
  }
}

MQTTClient::MQTTClient(Modem* modem)
    : _modem(modem)
    , _tx_buf(nullptr)
    , _rx_buf(nullptr)
    , _cb_connection_lost(nullptr)
    , _cb_ping_failed(nullptr)
    , _cb_rx_start(nullptr)
    , _cb_rx_topic(nullptr)
    , _cb_rx_payload(nullptr)
    , _cb_rx_end(nullptr) {
  if (!isUsable()) {
    SIM7600_LOGE(tag, "Maximum number of MQTT clients reached");
    return;
  }

  if (_modem == nullptr) return;

  _tx_buf = _modem->_tx_buf;
  _rx_buf = _modem->_rx_buf;

  _modem->_registerURCHandler(_handleMQTTURCs);
}

MQTTClient::~MQTTClient() {
  disconnect();
  releaseClient();
}

Status MQTTClient::setModem(Modem* const modem) {
  if (!_client_usable) return Status::ClientNotUsable;
  if (modem == nullptr) return Status::InvalidModem;

  SIM7600_LOGI(tag, "Setting modem for client %u", _client_id);

  _modem  = modem;
  _tx_buf = _modem->_tx_buf;
  _rx_buf = _modem->_rx_buf;
  _modem->_registerURCHandler(_handleMQTTURCs);

  return Status::Success;
}

Status MQTTClient::setConnectionLostCallback(ConnectionLostCB callback) {
  if (callback == nullptr) return Status::InvalidCallback;
  _cb_connection_lost = callback;
  return Status::Success;
}

Status MQTTClient::setPingFailedCallback(PingFailedCB callback) {
  if (callback == nullptr) return Status::InvalidCallback;
  _cb_ping_failed = callback;
  return Status::Success;
}

Status MQTTClient::setRxStartCallback(RxStartCB callback) {
  if (callback == nullptr) return Status::InvalidCallback;
  _cb_rx_start = callback;
  return Status::Success;
}

Status MQTTClient::setRxTopicCallback(RxTopicCB callback) {
  if (callback == nullptr) return Status::InvalidCallback;
  _cb_rx_topic = callback;
  return Status::Success;
}

Status MQTTClient::setRxPayloadCallback(RxPayloadCB callback) {
  if (callback == nullptr) return Status::InvalidCallback;
  _cb_rx_payload = callback;
  return Status::Success;
}

Status MQTTClient::setRxEndCallback(RxEndCB callback) {
  if (callback == nullptr) return Status::InvalidCallback;
  _cb_rx_end = callback;
  return Status::Success;
}

Status MQTTClient::acquireClient(const char* client_name, const bool use_tls,
  const MQTTVersion version) {
  if (!_client_usable) return Status::ClientNotUsable;
  if (_modem == nullptr) return Status::InvalidModem;
  if (client_name == nullptr) return Status::InvalidClientName;

  if (strlen(client_name) == 0 || strlen(client_name) > SIM7600_MQTT_MAX_CLIENT_ID_LENGTH) {
    return Status::InvalidClientName;
  }

  SIM7600_LOGD(tag, "MQTTClient %u: Acquiring client", _client_id);

  Status status = _modem->sendATCmd("AT+CMQTTACCQ=%u,\"%s\",%u,%u",
    _client_id,
    client_name,
    use_tls ? 1 : 0,
    static_cast<uint8_t>(version));
  if (status != Status::Success) return status;

  return _modem->waitForResponse(AT_OK);
}

Status MQTTClient::releaseClient() {
  if (!_client_usable) return Status::ClientNotUsable;
  if (_modem == nullptr) return Status::InvalidModem;

  SIM7600_LOGD(tag, "MQTTClient %u: Releasing client", _client_id);

  Status status = _modem->sendATCmd("AT+CMQTTREL=%u", _client_id);
  if (status != Status::Success) return status;

  return _modem->waitForResponse(AT_OK, SIM7600_MQTT_MAX_RESPONSE_TIME_MS);
}

Status MQTTClient::setLastWillMessage(const char* topic, const char* message, const MQTTQoS qos) {
  if (!_client_usable) return Status::ClientNotUsable;
  if (_modem == nullptr) return Status::InvalidModem;
  if (topic == nullptr) return Status::InvalidTopic;
  if (message == nullptr) return Status::InvalidPayload;

  size_t topic_len = strlen(topic);

  if (topic_len == 0 || topic_len > SIM7600_MQTT_MAX_TOPIC_LENGTH) {
    return Status::InvalidTopic;
  }

  size_t msg_len = strlen(message);

  if (msg_len == 0 || msg_len > SIM7600_MQTT_MAX_TOPIC_LENGTH) {
    return Status::InvalidPayload;
  }

  SIM7600_LOGD(tag, "MQTTClient %u: Setting Will topic", _client_id);

  Status status = _modem->sendATCmd("AT+CMQTTWILLTOPIC=%u,%u", _client_id, topic_len);
  if (status != Status::Success) return status;

  // Send data after '>' prompt
  size_t bytes_sent = 0;
  status =
    _modem->_waitPromptAndSendData(reinterpret_cast<const uint8_t*>(topic), topic_len, bytes_sent);
  if (status != Status::Success) return status;

  status = _modem->waitForResponse(AT_OK);
  if (status != Status::Success) return status;

  SIM7600_LOGD(tag, "MQTTClient %u: Setting Will message", _client_id);

  status =
    _modem->sendATCmd("AT+CMQTTWILLMSG=%u,%u,%u", _client_id, msg_len, static_cast<uint8_t>(qos));
  if (status != Status::Success) return status;

  // Send data after '>' prompt
  bytes_sent = 0;
  status =
    _modem->_waitPromptAndSendData(reinterpret_cast<const uint8_t*>(message), msg_len, bytes_sent);
  if (status != Status::Success) return status;

  return _modem->waitForResponse(AT_OK);
}

Status MQTTClient::connect(const char* host, const uint16_t port, const char* username,
  const char* password, const uint16_t keep_alive_s, const bool clean_session) {
  if (!_client_usable) return Status::ClientNotUsable;
  if (_modem == nullptr) return Status::InvalidModem;
  if (host == nullptr) return Status::InvalidHost;

  size_t host_len = strlen(host);

  if (host_len == 0) return Status::InvalidHost;

  // 12 extra bytes for "tcp://" and ":00000" (max port size)
  // Check for the valid server string total size
  size_t server_addr_len = host_len + 12;
  if (server_addr_len > SIM7600_MQTT_MAX_SERVER_USER_PASS_LENGTH) {
    SIM7600_LOGE(tag, "MQTTClient %u: Invalid host", _client_id);
    return Status::InvalidHost;
  }

  // Check for username and password length if provided
  if (username != nullptr) {
    size_t user_len = strlen(username);

    if (user_len == 0 || user_len > SIM7600_MQTT_MAX_SERVER_USER_PASS_LENGTH) {
      SIM7600_LOGE(tag, "MQTTClient %u: Invalid username", _client_id);
      return Status::InvalidUser;
    }
  }

  if (password != nullptr) {
    size_t pass_len = strlen(password);

    if (pass_len == 0 || pass_len > SIM7600_MQTT_MAX_SERVER_USER_PASS_LENGTH) {
      SIM7600_LOGE(tag, "MQTTClient %u: Invalid password", _client_id);
      return Status::InvalidPass;
    }
  }

  // If password is provided, username must be provided too
  if (password != nullptr && username == nullptr) {
    SIM7600_LOGE(tag, "MQTTClient %u: Password provided without username", _client_id);
    return Status::NoUserProvided;
  }

  SIM7600_LOGI(tag, "MQTTClient %u: Connecting to %s:%u", _client_id, host, port);

  // Check if client is connected and disconnect first
  bool connected = false;
  Status status  = isConnected(connected);
  if (status != Status::Success) return status;

  if (connected) {
    SIM7600_LOGW(tag, "MQTTClient %u: Already connected, disconnecting first", _client_id);
    status = disconnect();
    if (status != Status::Success) return status;
  }

  char user_buf[64];
  char pass_buf[64];

  if (username == nullptr) {
    user_buf[0] = '\0';
  } else {
    snprintf(user_buf, sizeof(user_buf), ",\"%s\"", username);
  }

  if (password == nullptr) {
    pass_buf[0] = '\0';
  } else {
    snprintf(pass_buf, sizeof(pass_buf), ",\"%s\"", password);
  }

  status = _modem->sendATCmd("AT+CMQTTCONNECT=%u,\"tcp://%s:%u\",%u,%u%s%s",
    _client_id,
    host,
    port,
    keep_alive_s,
    clean_session,
    user_buf,
    pass_buf);
  if (status != Status::Success) return status;

  // Wait for OK
  status = _modem->waitForResponse(AT_OK);
  if (status != Status::Success) return status;

  // Wait for +CMQTTCONNECT response
  snprintf(_tx_buf, SIM7600_MODEM_TX_BUFFER_SIZE_B, "+CMQTTCONNECT: %u", _client_id);
  status = _modem->waitForResponse(_tx_buf, SIM7600_MQTT_MAX_RESPONSE_TIME_MS);
  if (status != Status::Success) return status;

  // Expected response: +CMQTTCONNECT: <client_id>,<result> / where result: 0=success,
  // others=failure
  uint8_t client_id;
  uint8_t result;
  status = _modem->parseLine(_rx_buf, 2, "+CMQTTCONNECT: %hhu,%hhu", &client_id, &result);
  if (status != Status::Success) return status;

  if (result != 0) {
    SIM7600_LOGE(tag, "MQTTClient %u: Failed to connect, result: %u", _client_id, result);
    return Status::Error;
  }

  SIM7600_LOGI(tag, "MQTTClient %u: Connected successfully", _client_id);
  return Status::Success;
}

Status MQTTClient::disconnect(const uint8_t timeout_s) {
  if (!_client_usable) return Status::ClientNotUsable;
  if (_modem == nullptr) return Status::InvalidModem;

  // Timeout can be between min and max values
  if (timeout_s < SIM7600_MQTT_MIN_DISCONNECT_TIMEOUT_S ||
      timeout_s > SIM7600_MQTT_MAX_DISCONNECT_TIMEOUT_S) {
    SIM7600_LOGE(tag, "MQTTClient %u: Invalid disconnect timeout", _client_id);
    return Status::InvalidTimeout;
  }

  SIM7600_LOGI(tag, "MQTTClient %u: Disconnecting", _client_id);

  // Check if client is connected
  bool connected = false;
  Status status  = isConnected(connected);
  if (status != Status::Success) return status;

  if (!connected) {
    SIM7600_LOGW(tag, "MQTTClient %u: Already disconnected", _client_id);
    return Status::Success;
  }

  status = _modem->sendATCmd("AT+CMQTTDISC=%u,%u", _client_id, timeout_s);
  if (status != Status::Success) return status;

  // If success, modem responds with OK and +CMQTT... but order is not guaranteed
  snprintf(_tx_buf, SIM7600_MODEM_TX_BUFFER_SIZE_B, "+CMQTTDISC: %u", _client_id);
  const char* expected_responses[] = {_tx_buf, AT_OK};

  uint8_t found_index = 0;
  status =
    _modem->waitForResponses(expected_responses, 2, found_index, SIM7600_MQTT_MAX_RESPONSE_TIME_MS);
  if (status != Status::Success) return status;

  bool got_mqtt_first = (found_index == 0);

  // Expected response: +CMQTTDISC: <client_id>,<err> / where err: 0=success,
  // others=failure
  uint8_t client_id;
  uint8_t err;
  uint8_t parsed;

  if (got_mqtt_first) {
    status = _modem->parseLine(_rx_buf, 2, "+CMQTTDISC: %hhu,%hhu", &client_id, &err);
    if (status != Status::Success) return status;

    // Need to wait for OK, but could receive ERROR
    status = _modem->waitForResponse(AT_OK);
    if (status != Status::Success) return status;

  } else {
    // Got OK, now wait for +CMQTTDISC:
    status = _modem->waitForResponse(_tx_buf, SIM7600_MQTT_MAX_RESPONSE_TIME_MS);
    if (status != Status::Success) return status;

    status = _modem->parseLine(_rx_buf, 2, "+CMQTTDISC: %hhu,%hhu", &client_id, &err);
    if (status != Status::Success) return status;
  }

  if (err != 0) {
    SIM7600_LOGE(tag, "MQTTClient %u: Failed to disconnect, err: %u", _client_id, err);
    return Status::Error;
  }

  SIM7600_LOGI(tag, "MQTTClient %u: Disconnected successfully", _client_id);
  return Status::Success;
}

Status MQTTClient::isConnected(bool& connected) {
  if (!_client_usable) return Status::ClientNotUsable;
  if (_modem == nullptr) return Status::InvalidModem;

  connected = false;

  SIM7600_LOGD(tag, "MQTTClient %u: Checking connection status", _client_id);

  Status status = _modem->sendATCmd("AT+CMQTTDISC?");
  if (status != Status::Success) return status;

  // Wait for +CMQTTDISC response
  snprintf(_tx_buf, SIM7600_MODEM_TX_BUFFER_SIZE_B, "+CMQTTDISC: %u", _client_id);
  status = _modem->waitForResponse(_tx_buf);
  if (status != Status::Success) return status;

  // Expected response: +CMQTTDISC: <client_id>,<state> / where state: 0=disconnected,
  // 1=connected
  uint8_t client_id;
  uint8_t disconnected;
  status = _modem->parseLine(_rx_buf, 2, "+CMQTTDISC: %hhu,%hhu", &client_id, &disconnected);
  if (status != Status::Success) return status;

  connected = (disconnected == 0);
  return _modem->waitForResponse(AT_OK);
}

Status MQTTClient::publish(const char* topic, const uint8_t* payload, const size_t payload_length,
  const MQTTQoS qos = MQTTQoS::AtLeastOnce, const bool retained) {
  if (!_client_usable) return Status::ClientNotUsable;
  if (_modem == nullptr) return Status::InvalidModem;
  if (topic == nullptr) return Status::InvalidTopic;
  if (payload == nullptr) return Status::InvalidPayload;

  size_t topic_len = strlen(topic);

  if (topic_len == 0 || topic_len > SIM7600_MQTT_MAX_TOPIC_LENGTH) {
    return Status::InvalidTopic;
  }

  if (payload_length == 0 || payload_length > SIM7600_MQTT_MAX_PAYLOAD_LENGTH) {
    return Status::InvalidPayload;
  }

  SIM7600_LOGD(tag, "MQTTClient %u: Publishing message to topic %s", _client_id, topic);

  // Set topic
  Status status = _modem->sendATCmd("AT+CMQTTTOPIC=%u,%u", _client_id, topic_len);
  if (status != Status::Success) return status;

  size_t bytes_sent = 0;

  // Send data after '>' prompt
  status =
    _modem->_waitPromptAndSendData(reinterpret_cast<const uint8_t*>(topic), topic_len, bytes_sent);
  if (status != Status::Success) return status;

  status = _modem->waitForResponse(AT_OK);
  if (status != Status::Success) return status;

  // Set payload
  status = _modem->sendATCmd("AT+CMQTTPAYLOAD=%u,%u", _client_id, payload_length);
  if (status != Status::Success) return status;

  // Send data after '>' prompt
  status = _modem->_waitPromptAndSendData(reinterpret_cast<const uint8_t*>(payload),
    payload_length,
    bytes_sent);
  if (status != Status::Success) return status;

  status = _modem->waitForResponse(AT_OK);
  if (status != Status::Success) return status;

  // Publish message
  status = _modem->sendATCmd("AT+CMQTTPUB=%u,%u,%u,%u",
    _client_id,
    static_cast<uint8_t>(qos),
    SIM7600_MQTT_DATA_TIMEOUT_S,
    retained ? 1 : 0);
  if (status != Status::Success) return status;

  // Expected +CMQTTPUB: <client_id>,<err> / where err: 0=success, others=failure
  snprintf(_tx_buf, SIM7600_MODEM_TX_BUFFER_SIZE_B, "+CMQTTPUB: %u,", _client_id);
  uint8_t err;
  status = _modem->_waitAsyncMQTTResponse(_tx_buf, err);
  if (status != Status::Success) return status;

  if (err != 0) {
    SIM7600_LOGE(tag, "MQTTClient %u: Failed to publish message, err: %u", _client_id, err);
    return Status::Error;
  }

  SIM7600_LOGD(tag, "MQTTClient %u: Published message successfully", _client_id);
  return Status::Success;
}

Status MQTTClient::subscribe(const char* topic, const MQTTQoS qos) {
  if (!_client_usable) return Status::ClientNotUsable;
  if (_modem == nullptr) return Status::InvalidModem;
  if (topic == nullptr) return Status::InvalidTopic;

  size_t topic_len = strlen(topic);

  if (topic_len == 0 || topic_len > SIM7600_MQTT_MAX_TOPIC_LENGTH) {
    return Status::InvalidTopic;
  }

  SIM7600_LOGD(tag, "MQTTClient %u: Subscribing to topic %s", _client_id, topic);

  // Set topic
  Status status =
    _modem->sendATCmd("AT+CMQTTSUB=%u,%u,%u", _client_id, topic_len, static_cast<uint8_t>(qos));
  if (status != Status::Success) return status;

  // Send data after '>' prompt
  size_t bytes_sent = 0;
  status =
    _modem->_waitPromptAndSendData(reinterpret_cast<const uint8_t*>(topic), topic_len, bytes_sent);
  if (status != Status::Success) return status;

  // Wait for +CMQTTSUB response
  status = _modem->waitForResponse("+CMQTTSUB:", SIM7600_MQTT_MAX_RESPONSE_TIME_MS);
  if (status != Status::Success) return status;

  // Expected +CMQTTSUB: <client_id>,<err> / where err: 0=success, others=failure
  uint8_t client_id;
  uint8_t err;
  status = _modem->parseLine(_rx_buf, 2, "+CMQTTSUB: %hhu,%hhu", &client_id, &err);
  if (status != Status::Success) return status;

  if (err == 0) {
    SIM7600_LOGD(tag, "MQTTClient %u: Subscribed to topic successfully", _client_id);
    return Status::Success;
  }

  return _modem->waitForResponse(AT_ERROR);
}

Status MQTTClient::unsubscribe(const char* topic) {
  if (!_client_usable) return Status::ClientNotUsable;
  if (_modem == nullptr) return Status::InvalidModem;
  if (topic == nullptr) return Status::InvalidTopic;

  size_t topic_len = strlen(topic);

  if (topic_len == 0 || topic_len > SIM7600_MQTT_MAX_TOPIC_LENGTH) {
    return Status::InvalidTopic;
  }

  SIM7600_LOGD(tag, "MQTTClient %u: Unsubscribing from topic %s", _client_id, topic);

  // Set unsubscribe topic
  Status status = _modem->sendATCmd("AT+CMQTTUNSUB=%u,%u,0", _client_id, topic_len);
  if (status != Status::Success) return status;

  // Send data after '>' prompt
  size_t bytes_sent = 0;
  status =
    _modem->_waitPromptAndSendData(reinterpret_cast<const uint8_t*>(topic), topic_len, bytes_sent);
  if (status != Status::Success) return status;

  // Wait for +CMQTTUNSUB response
  status = _modem->waitForResponse("+CMQTTUNSUB:", SIM7600_MQTT_MAX_RESPONSE_TIME_MS);
  if (status != Status::Success) return status;

  // Expected +CMQTTUNSUB: <client_id>,<err> / where err: 0=success, others=failure
  uint8_t client_id;
  uint8_t err;
  status = _modem->parseLine(_rx_buf, 2, "+CMQTTUNSUB: %hhu,%hhu", &client_id, &err);
  if (status != Status::Success) return status;

  if (err == 0) {
    SIM7600_LOGD(tag, "MQTTClient %u: Unsubscribed from topic successfully", _client_id);
    return Status::Success;
  }

  return _modem->waitForResponse(AT_ERROR);
}

bool MQTTClient::_handleMQTTURCs(const char* line) {
  if (line == nullptr) return false;

  // Connection lost
  // Expected: +CMQTTCONNLOST: <client_id>,<cause>
  if (strstr(line, "+CMQTTCONNLOST:") != nullptr) {
    uint8_t client_id;
    uint8_t cause;
    uint8_t parsed = sscanf(line, "+CMQTTCONNLOST: %hhu,%hhu", &client_id, &cause);
    if (parsed != 2) return false;

    SIM7600_LOGD(tag, "MQTT URC Client %u: Connection lost", client_id);

    MQTTClient* instance = _instances[client_id];
    if (instance == nullptr) return false;

    if (instance->_cb_connection_lost != nullptr) {
      instance->_cb_connection_lost(instance, static_cast<MQTTConnLostCause>(cause));
    }

    return true;
  }

  // Ping failed
  // Expected: +CMQTTPING: <client_id>,<err>
  if (strstr(line, "+CMQTTPING:") != nullptr) {
    uint8_t client_id;
    uint8_t err;
    uint8_t parsed = sscanf(line, "+CMQTTPING: %hhu,%hhu", &client_id, &err);
    if (parsed != 2) return false;

    SIM7600_LOGD(tag, "MQTT URC Client %u: Ping failed", client_id);

    MQTTClient* instance = _instances[client_id];
    if (instance == nullptr) return false;

    if (instance->_cb_ping_failed != nullptr) {
      instance->_cb_ping_failed(instance);
    }

    return true;
  }

  // RX start
  // Expected: +CMQTTRXSTART: <client_id>,<topic_length>,<payload_length>
  else if (strstr(line, "+CMQTTRXSTART:") != nullptr) {
    uint8_t client_id;
    size_t topic_length;
    size_t payload_length;
    uint8_t parsed =
      sscanf(line, "+CMQTTRXSTART: %hhu,%zu,%zu", &client_id, &topic_length, &payload_length);
    if (parsed != 3) return false;

    SIM7600_LOGD(tag, "MQTT URC Client %u: RX start", client_id);

    MQTTClient* instance = _instances[client_id];
    if (instance == nullptr) return false;

    if (instance->_cb_rx_start != nullptr) {
      instance->_cb_rx_start(instance, topic_length, payload_length);
    }

    return true;
  }

  // RX topic
  // Expected: +CMQTTRXTOPIC: <client_id>,<topic_length>\r\n<topic>
  else if (strstr(line, "+CMQTTRXTOPIC:") != nullptr) {
    uint8_t client_id;
    size_t topic_length;
    uint8_t parsed = sscanf(line, "+CMQTTRXTOPIC: %hhu,%zu", &client_id, &topic_length);
    if (parsed != 2) return false;

    SIM7600_LOGD(tag, "MQTT URC Client %u: RX topic", client_id);

    MQTTClient* instance = _instances[client_id];
    if (instance == nullptr) return false;

    // The topic is in the next line
    size_t bytes_read = 0;
    Status status     = instance->_modem->readBytes(reinterpret_cast<uint8_t*>(instance->_rx_buf),
      SIM7600_MODEM_RX_BUFFER_SIZE_B,
      topic_length,
      bytes_read);
    if (status != Status::Success) return false;

    // Limit topic length to buffer size
    if (topic_length >= SIM7600_MODEM_RX_BUFFER_SIZE_B) {
      topic_length = SIM7600_MODEM_RX_BUFFER_SIZE_B - 1;
    }

    // Null terminate the topic
    instance->_rx_buf[topic_length] = '\0';

    if (instance->_cb_rx_topic != nullptr) {
      instance->_cb_rx_topic(instance, instance->_rx_buf, topic_length);
    }

    return true;
  }

  // RX payload
  // Expected: +CMQTTRXPAYLOAD: <client_id>,<payload_length>\r\n<payload>
  else if (strstr(line, "+CMQTTRXPAYLOAD:") != nullptr) {
    uint8_t client_id;
    size_t payload_length;
    uint8_t parsed = sscanf(line, "+CMQTTRXPAYLOAD: %hhu,%zu", &client_id, &payload_length);
    if (parsed != 2) return false;

    SIM7600_LOGD(tag, "MQTT URC Client %u: RX payload", client_id);

    MQTTClient* instance = _instances[client_id];
    if (instance == nullptr) return false;

    // The payload is in the next line;
    size_t bytes_read = 0;
    Status status     = instance->_modem->readBytes(reinterpret_cast<uint8_t*>(instance->_rx_buf),
      SIM7600_MODEM_RX_BUFFER_SIZE_B,
      payload_length,
      bytes_read);
    if (status != Status::Success) return false;

    // Limit payload length to buffer size
    if (payload_length >= SIM7600_MODEM_RX_BUFFER_SIZE_B) {
      payload_length = SIM7600_MODEM_RX_BUFFER_SIZE_B - 1;
    }

    // Null terminate the payload
    instance->_rx_buf[payload_length] = '\0';

    if (instance->_cb_rx_payload != nullptr) {
      instance->_cb_rx_payload(instance,
        reinterpret_cast<uint8_t*>(instance->_rx_buf),
        payload_length);
    }

    return true;
  }

  // RX end
  // Expected: +CMQTTRXEND: <client_id>
  else if (strstr(line, "+CMQTTRXEND:") != nullptr) {
    uint8_t client_id;
    uint8_t parsed = sscanf(line, "+CMQTTRXEND: %hhu", &client_id);
    if (parsed != 1) return false;

    SIM7600_LOGD(tag, "MQTT URC Client %u: RX end", client_id);

    MQTTClient* instance = _instances[client_id];
    if (instance == nullptr) return false;

    if (instance->_cb_rx_end != nullptr) {
      instance->_cb_rx_end(instance);
    }

    return true;
  }

  return false;
}

} // namespace SIM7600
