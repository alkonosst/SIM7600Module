#include "TB_SIM7600Client.h"

static const char* tag = "TB_SIM7600Client";

using namespace SIM7600;

TB_SIM7600Client::TB_SIM7600Client(SIM7600::Modem& modem)
    : IMQTT_Client()
    , _modem(modem)
    , _client(&_modem)
    , _server(nullptr)
    , _port(SIM7600_MQTT_DEFAULT_PORT)
    , _cb_rx_topic(nullptr)
    , _cb_rx_payload(nullptr)
    , m_received_data_callback()
    , m_connected_callback() {
  memset(_topic_buffer, 0, sizeof(_topic_buffer));

  // Set up internal MQTT client callbacks
  _client.setRxTopicCallback(
    [this](MQTTClient* const client, const char* topic, const size_t topic_length) {
      this->_onRxTopic(client, topic, topic_length);
    });

  _client.setRxPayloadCallback(
    [this](MQTTClient* const client, uint8_t* const payload, const size_t payload_length) {
      this->_onRxPayload(client, payload, payload_length);
    });
}

Modem& TB_SIM7600Client::getModem() { return _modem; }

MQTTClient& TB_SIM7600Client::getClient() { return _client; }

void TB_SIM7600Client::set_data_callback(
  Callback<void, char*, uint8_t*, unsigned int>::function callback) {
  m_received_data_callback.Set_Callback(callback);
}

void TB_SIM7600Client::set_connect_callback(Callback<void>::function callback) {
  m_connected_callback.Set_Callback(callback);
}

bool TB_SIM7600Client::set_buffer_size(uint16_t receive_buffer_size, uint16_t send_buffer_size) {
  return true;
}

uint16_t TB_SIM7600Client::get_receive_buffer_size() { return SIM7600_MODEM_RX_BUFFER_SIZE_B; }

uint16_t TB_SIM7600Client::get_send_buffer_size() { return SIM7600_MODEM_TX_BUFFER_SIZE_B; }

void TB_SIM7600Client::set_server(char const* domain, uint16_t port) {
  _server = domain;
  _port   = port;
}

bool TB_SIM7600Client::connect(char const* client_id, char const* user_name, char const* password) {
  ESP_LOGD(tag, "Connecting to %s:%u", _server, _port);

  Status status = _client.connect(_server, _port, user_name, password);

  if (status != Status::Success) {
    ESP_LOGE(tag, "Failed to connect to broker: %s", _modem.statusToString(status));
    return false;
  }

  m_connected_callback.Call_Callback();
  return true;
}

void TB_SIM7600Client::disconnect() {
  ESP_LOGE(tag, "Disconnecting from broker");

  Status status = _client.disconnect();

  if (status != Status::Success) {
    ESP_LOGE(tag, "Failed to disconnect from broker: %s", _modem.statusToString(status));
  }
}

bool TB_SIM7600Client::connected() {
  bool is_connected = false;
  Status status     = _client.isConnected(is_connected);

  if (status != Status::Success) {
    ESP_LOGE(tag, "Failed to get connection status: %s", _modem.statusToString(status));
    return false;
  }

  return is_connected;
}

bool TB_SIM7600Client::publish(char const* topic, uint8_t const* payload, size_t const& length) {
  ESP_LOGD(tag, "Publishing to topic \"%s\"", topic);

  Status status = _client.publish(topic, payload, length, MQTTQoS::AtMostOnce, false);

  if (status != Status::Success) {
    ESP_LOGE(tag, "Failed to publish to \"%s\": %s", topic, _modem.statusToString(status));
    return false;
  }

  return true;
}

bool TB_SIM7600Client::subscribe(char const* topic) {
  ESP_LOGD(tag, "Subscribing to topic \"%s\"", topic);

  Status status = _client.subscribe(topic, MQTTQoS::AtMostOnce);

  if (status != Status::Success) {
    ESP_LOGE(tag, "Failed to subscribe to \"%s\": %s", topic, _modem.statusToString(status));
    return false;
  }

  return true;
}

bool TB_SIM7600Client::unsubscribe(char const* topic) {
  ESP_LOGD(tag, "Unsubscribing from topic \"%s\"", topic);

  Status status = _client.unsubscribe(topic);

  if (status != Status::Success) {
    ESP_LOGE(tag, "Failed to unsubscribe from \"%s\": %s", topic, _modem.statusToString(status));
    return false;
  }

  return true;
}

bool TB_SIM7600Client::loop() {
  _modem.loop();
  return true;
}

void TB_SIM7600Client::_onRxTopic(MQTTClient* const client, const char* topic,
  const size_t topic_length) {
  snprintf(_topic_buffer, sizeof(_topic_buffer), "%s", topic);
}

void TB_SIM7600Client::_onRxPayload(MQTTClient* const client, uint8_t* const payload,
  const size_t payload_length) {
  m_received_data_callback.Call_Callback(_topic_buffer, payload, payload_length);
}
