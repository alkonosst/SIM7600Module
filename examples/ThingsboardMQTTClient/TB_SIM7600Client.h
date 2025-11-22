#pragma once

#include "IMQTT_Client.h"
#include "SIM7600MQTTClient.h"

class TB_SIM7600Client : public IMQTT_Client {
  public:
  explicit TB_SIM7600Client(SIM7600::Modem& modem);

  SIM7600::Modem& getModem();
  SIM7600::MQTTClient& getClient();

  // Overrides from IMQTT_Client
  void set_data_callback(Callback<void, char*, uint8_t*, unsigned int>::function callback) override;
  void set_connect_callback(Callback<void>::function callback) override;

  bool set_buffer_size(uint16_t receive_buffer_size, uint16_t send_buffer_size) override;
  uint16_t get_receive_buffer_size() override;
  uint16_t get_send_buffer_size() override;

  void set_server(char const* domain, uint16_t port) override;
  bool connect(char const* client_id, char const* user_name, char const* password) override;
  void disconnect() override;
  bool connected() override;

  bool publish(char const* topic, uint8_t const* payload, size_t const& length) override;
  bool subscribe(char const* topic) override;
  bool unsubscribe(char const* topic) override;

  bool loop() override;

  private:
  SIM7600::Modem& _modem;
  SIM7600::MQTTClient _client;

  const char* _server;
  uint16_t _port;

  char _topic_buffer[64];

  SIM7600::MQTTClient::RxTopicCB _cb_rx_topic;
  SIM7600::MQTTClient::RxPayloadCB _cb_rx_payload;

  void _onRxTopic(SIM7600::MQTTClient* const client, const char* topic, const size_t topic_length);

  void _onRxPayload(SIM7600::MQTTClient* const client, uint8_t* const payload,
    const size_t payload_length);

  // Callback that will be called as soon as the mqtt client receives any data
  Callback<void, char*, uint8_t*, unsigned int> m_received_data_callback;

  // Callback that will be called as soon as the mqtt client has connected
  Callback<void> m_connected_callback;
};