/**
 * SPDX-FileCopyrightText: 2025 Maximiliano Ramirez <maximiliano.ramirezbravo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */

#include "SIM7600TCPClient.h"
#include "SIM7600Log.h"

static const char* tag = "SIM7600TCPClient";

namespace SIM7600 {

TCPClient::TCPClient()
    : _modem(nullptr)
    , _connected(false)
    , _available_bytes(0)
    , _peek_byte(0)
    , _peek_available(false)
    , _tx_buf(nullptr)
    , _rx_buf(nullptr)
    , _cb_data(nullptr)
    , _cb_connection_closed(nullptr) {
  if (!isUsable()) {
    SIM7600_LOGE(tag, "Maximum number of TCP clients reached");
  }
}

TCPClient::TCPClient(Modem* modem)
    : _modem(modem)
    , _connected(false)
    , _available_bytes(0)
    , _peek_byte(0)
    , _peek_available(false)
    , _tx_buf(nullptr)
    , _rx_buf(nullptr)
    , _cb_data(nullptr)
    , _cb_connection_closed(nullptr) {

  if (!isUsable()) {
    SIM7600_LOGE(tag, "Maximum number of TCP clients reached");
    return;
  }

  if (modem == nullptr) return;

  _tx_buf = _modem->_tx_buf;
  _rx_buf = _modem->_rx_buf;

  _modem->_registerURCHandler(_handleTCPURCs);
}

TCPClient::~TCPClient() { disconnect(); }

Status TCPClient::setModem(Modem* const modem) {
  if (modem == nullptr) return Status::InvalidModem;

  SIM7600_LOGI(tag, "Setting modem for client %u", _client_id);

  _modem  = modem;
  _tx_buf = _modem->_tx_buf;
  _rx_buf = _modem->_rx_buf;
  _modem->_registerURCHandler(_handleTCPURCs);

  return Status::Success;
}

Status TCPClient::setDataReceivedCallback(DataReceivedCB callback) {
  if (callback == nullptr) return Status::InvalidCallback;
  _cb_data = callback;
  return Status::Success;
}

Status TCPClient::setConnectionClosedCallback(ConnectionClosedCB callback) {
  if (callback == nullptr) return Status::InvalidCallback;
  _cb_connection_closed = callback;
  return Status::Success;
}

Status TCPClient::connectToHost(const IPAddress ip, const uint16_t port) {
  if (!_client_usable) return Status::ClientNotUsable;
  if (_modem == nullptr) return Status::InvalidModem;

  char ip_str[16];
  snprintf(ip_str, sizeof(ip_str), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
  return connectToHost(ip_str, port);
}

Status TCPClient::connectToHost(const char* host, const uint16_t port) {
  if (!_client_usable) return Status::ClientNotUsable;
  if (_modem == nullptr) return Status::InvalidModem;
  if (host == nullptr) return Status::InvalidHost;

  // Check if already connected and disconnect before establishing a new connection
  bool connected = false;
  Status status  = isConnected(connected);
  if (status != Status::Success) return status;

  if (connected) {
    SIM7600_LOGW(tag, "TCPClient %u: Already connected, disconnecting first", _client_id);
    status = disconnect();
    if (status != Status::Success) return status;
  }

  SIM7600_LOGI(tag, "TCPClient %u: Connecting to %s:%u", _client_id, host, port);

  // Reset internal states
  _connected       = false;
  _available_bytes = 0;
  _peek_byte       = 0;
  _peek_available  = false;

  status = _modem->sendATCmd("AT+CIPOPEN=%u,\"TCP\",\"%s\",%u", _client_id, host, port);
  if (status != Status::Success) return status;

  status = _modem->waitForResponse(AT_OK);
  if (status != Status::Success) return status;

  // Wait for +CIPOPEN response
  snprintf(_tx_buf, SIM7600_MODEM_RX_BUFFER_SIZE_B, "+CIPOPEN: %u", _client_id);
  status = _modem->waitForResponse(_tx_buf, SIM7600_TCP_MAX_RESPONSE_TIME_MS);
  if (status != Status::Success) return status;

  // Expected response: +CIPOPEN: <link_num>,<result> / where result: 0=success, other=failure
  uint8_t link_num, result;
  status = _modem->parseLine(_rx_buf, 2, "+CIPOPEN: %hhu,%hhu", &link_num, &result);
  if (status != Status::Success) return status;

  if (result != 0) {
    SIM7600_LOGE(tag, "TCPClient %u: Failed to connect, result: %u", _client_id, result);
    return Status::Error;
  }

  _connected = true;

  SIM7600_LOGI(tag, "TCPClient %u: Connected successfully", _client_id);
  return Status::Success;
}

Status TCPClient::disconnect() {
  if (!_client_usable) return Status::ClientNotUsable;
  if (_modem == nullptr) return Status::InvalidModem;

  SIM7600_LOGI(tag, "TCPClient %u: Disconnecting", _client_id);

  if (!_connected) return Status::Success;

  Status status = _modem->sendATCmd("AT+CIPCLOSE=%u", _client_id);
  if (status != Status::Success) return status;

  status = _modem->waitForResponse(AT_OK);
  if (status != Status::Success) return status;

  // Wait for +CIPCLOSE response
  snprintf(_tx_buf, SIM7600_MODEM_RX_BUFFER_SIZE_B, "+CIPCLOSE: %u", _client_id);
  status = _modem->waitForResponse(_tx_buf);
  if (status != Status::Success) return status;

  // Expected response: +CIPCLOSE: <link_num>,<result> / where result: 0=success, other=failure
  uint8_t link_num, result;
  status = _modem->parseLine(_rx_buf, 2, "+CIPCLOSE: %hhu,%hhu", &link_num, &result);
  if (status != Status::Success) return status;

  if (result != 0) {
    SIM7600_LOGE(tag, "TCPClient %u: Failed to disconnect, result: %u", _client_id, result);
    return Status::Error;
  }

  // Reset internal states
  _connected       = false;
  _available_bytes = 0;
  _peek_byte       = 0;
  _peek_available  = false;

  SIM7600_LOGI(tag, "TCPClient %u: Disconnected successfully", _client_id);
  return Status::Success;
}

Status TCPClient::isConnected(bool& connected) {
  if (!_client_usable) return Status::ClientNotUsable;
  if (_modem == nullptr) return Status::InvalidModem;

  SIM7600_LOGD(tag, "TCPClient %u: Checking connection status", _client_id);

  // Check connection status
  Status status = _modem->sendATCmdAndWaitResp("AT+CIPCLOSE?", "+CIPCLOSE:");
  if (status != Status::Success) return status;

  // Expected response: +CIPCLOSE: <link0_state>,<link1_state>,...,<link9_state>
  // Where status: 0=disconnected, 1=connected
  uint8_t link_states[SIM7600_TCP_MAX_CLIENTS] = {0};

  status = _modem->parseLine(_rx_buf,
    SIM7600_TCP_MAX_CLIENTS,
    "+CIPCLOSE: %hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu",
    &link_states[0],
    &link_states[1],
    &link_states[2],
    &link_states[3],
    &link_states[4],
    &link_states[5],
    &link_states[6],
    &link_states[7],
    &link_states[8],
    &link_states[9]);
  if (status != Status::Success) return status;

  connected  = (link_states[_client_id] == 1);
  _connected = connected;

  return _modem->waitForResponse(AT_OK);
}

Status TCPClient::sendData(const uint8_t* buffer, const size_t buffer_size, size_t& bytes_sent) {
  if (!_client_usable) return Status::ClientNotUsable;
  if (_modem == nullptr) return Status::InvalidModem;
  if (buffer == nullptr || buffer_size == 0) return Status::InvalidBuffer;
  if (!_connected) return Status::NotConnected;

  SIM7600_LOGD(tag, "TCPClient %u: Sending %u bytes", _client_id, buffer_size);

  Status status = _modem->sendATCmd("AT+CIPSEND=%u,%u", _client_id, buffer_size);
  if (status != Status::Success) return status;

  // Send data after '>' prompt
  status = _modem->_waitPromptAndSendData(buffer, buffer_size, bytes_sent);
  if (status != Status::Success) return status;

  // Wait for +CIPSEND response
  snprintf(_tx_buf, SIM7600_MODEM_RX_BUFFER_SIZE_B, "+CIPSEND: %u", _client_id);
  status = _modem->waitForResponse(_tx_buf, SIM7600_TCP_MAX_RESPONSE_TIME_MS);
  if (status != Status::Success) return status;

  // Expected response: +CIPSEND: <link_num>,<requested>,<sent>
  uint8_t link_num;
  int32_t requested, sent;
  status = _modem->parseLine(_rx_buf, 3, "+CIPSEND: %hhu,%d,%d", &link_num, &requested, &sent);
  if (status != Status::Success) return status;

  // sent = -1 : disconnected
  // sent = 0 : buffer full or peer congested
  if (sent == -1) {
    SIM7600_LOGE(tag, "TCPClient %u: Disconnected while sending data", _client_id);
    return Status::Disconnected;
  }

  if (sent == 0) {
    SIM7600_LOGE(tag, "TCPClient %u: Send buffer full or peer congested", _client_id);
    return Status::SendBufferFullOrPeerCongested;
  }

  if (sent != buffer_size) {
    SIM7600_LOGW(tag,
      "TCPClient %u: Sent less bytes than expected: %d/%u",
      _client_id,
      sent,
      buffer_size);
    return Status::TCPSentLessThanExpected;
  }

  SIM7600_LOGD(tag, "TCPClient %u: Sent %d bytes", _client_id, sent);
  return Status::Success;
}

Status TCPClient::readData(uint8_t* buffer, const size_t buffer_size, size_t& bytes_read,
  const uint32_t timeout_ms) {
  if (!_client_usable) return Status::ClientNotUsable;
  if (_modem == nullptr) return Status::InvalidModem;
  if (buffer == nullptr || buffer_size == 0) return Status::InvalidBuffer;

  bytes_read = 0;

  SIM7600_LOGD(tag,
    "TCPClient %u: Reading up to %u bytes of data",
    _client_id,
    buffer_size,
    _client_id);

  Status status = _modem->sendATCmd("AT+CIPRXGET=2,%u,%u", _client_id, buffer_size);
  if (status != Status::Success) return status;

  // Wait for "+CIPRXGET" or "+IP ERROR: No data" response
  const char* no_data_response = "+IP ERROR: No data";
  snprintf(_tx_buf, SIM7600_MODEM_RX_BUFFER_SIZE_B, "+CIPRXGET: 2,%u", _client_id);

  const char* expected_responses[] = {
    _tx_buf,
    no_data_response,
  };

  uint8_t found_index = 0;
  status              = _modem->waitForResponses(expected_responses, 2, found_index);
  if (status != Status::Success) return status;

  // Handle "No data" response
  if (found_index == 1) {
    // Wait for ERROR
    status = _modem->waitForResponse(AT_ERROR);
    if (status != Status::Success) return status;

    SIM7600_LOGW(tag, "TCPClient %u: No data available", _client_id);
    return Status::TCPNoDataAvailable;
  }

  // Expected response: +CIPRXGET: <mode>,<link_num>,<read_len>,<rest_len>
  uint8_t link_num;
  size_t read_len, rest_len;
  status =
    _modem->parseLine(_rx_buf, 3, "+CIPRXGET: 2,%hhu,%zu,%zu", &link_num, &read_len, &rest_len);
  if (status != Status::Success) return status;

  // No data available
  if (read_len == 0) {
    bytes_read = 0;
    return _modem->waitForResponse(AT_OK);
  }

  // Adjust read_len if it exceeds buffer_size
  if (read_len > buffer_size) {
    read_len = buffer_size;
    SIM7600_LOGW(tag,
      "TCPClient %u: Read length exceeds buffer size, adjusting to %u bytes",
      _client_id,
      read_len);
  }

  status = _modem->readBytes(buffer, buffer_size, read_len, bytes_read, timeout_ms);
  if (status != Status::Success) return status;

  _available_bytes = rest_len;

  SIM7600_LOGD(tag,
    "TCPClient %u: Read %u bytes, available: %u",
    _client_id,
    bytes_read,
    _available_bytes);
  return _modem->waitForResponse(AT_OK);
}

Status TCPClient::getAvailableBytes(size_t& available_bytes) {
  if (!_client_usable) return Status::ClientNotUsable;
  if (_modem == nullptr) return Status::InvalidModem;

  available_bytes = 0;

  SIM7600_LOGD(tag, "TCPClient %u: Getting available bytes", _client_id);

  Status status = _modem->sendATCmd("AT+CIPRXGET=4,%u", _client_id);
  if (status != Status::Success) return status;

  // Wait for +CIPRXGET response
  snprintf(_tx_buf, SIM7600_MODEM_RX_BUFFER_SIZE_B, "+CIPRXGET: 4,%u", _client_id);
  status = _modem->waitForResponse(_tx_buf);
  if (status != Status::Success) return status;

  // Expected response: +CIPRXGET: <mode>,<link_num>,<rest_len>
  uint8_t link_num;
  size_t rest_len;
  status = _modem->parseLine(_rx_buf, 2, "+CIPRXGET: 4,%hhu,%zu", &link_num, &rest_len);
  if (status != Status::Success) return status;

  available_bytes  = rest_len;
  _available_bytes = rest_len;

  SIM7600_LOGD(tag, "TCPClient %u: Available bytes: %u", _client_id, available_bytes);
  return _modem->waitForResponse(AT_OK);
}

int TCPClient::connect(IPAddress ip, uint16_t port) {
  Status status = connectToHost(ip, port);
  return (status == Status::Success);
}

int TCPClient::connect(const char* host, uint16_t port) {
  Status status = connectToHost(host, port);
  return (status == Status::Success);
}

size_t TCPClient::write(uint8_t c) {
  if (!_client_usable || _modem == nullptr) return 0;

  size_t bytes_written = 0;
  sendData(&c, 1, bytes_written);
  return bytes_written;
}

size_t TCPClient::write(const uint8_t* buf, size_t size) {
  if (!_client_usable || _modem == nullptr) return 0;

  size_t bytes_written = 0;
  sendData(buf, size, bytes_written);
  return bytes_written;
}

int TCPClient::available() {
  if (!_client_usable || _modem == nullptr) return -1;

  // Return cached value plus peeked byte
  int total = static_cast<int>(_available_bytes);
  if (_peek_available) total++;
  return total;
}

int TCPClient::read() {
  if (!_client_usable || _modem == nullptr) return -1;

  // If we have a peeked byte, consume and return it
  if (_peek_available) {
    _peek_available = false;
    return static_cast<int>(_peek_byte);
  }

  uint8_t byte;
  size_t bytes_read = 0;
  Status status     = readData(&byte, 1, bytes_read);
  if (status != Status::Success || bytes_read == 0) return -1;
  return static_cast<int>(byte);
}

int TCPClient::read(uint8_t* buf, size_t size) {
  if (!_client_usable || _modem == nullptr) return -1;
  if (buf == nullptr || size == 0) return -1;

  // If we have a peeked byte, consume and add it to the buffer
  size_t offset = 0;

  if (_peek_available) {
    buf[0]          = _peek_byte;
    _peek_available = false;
    offset          = 1;

    // If the requested size is 1, we have already provided the peeked byte
    if (size == 1) return 1;
  }

  size_t bytes_read = 0;
  Status status     = readData(buf + offset, size - offset, bytes_read);

  // Return the number of bytes read plus any peeked byte
  if (status != Status::Success) {
    return (offset > 0) ? static_cast<int>(offset) : -1;
  }

  return static_cast<int>(bytes_read + offset);
}

int TCPClient::peek() {
  if (!_client_usable || _modem == nullptr) return -1;
  if (!_connected) return -1;

  // If we have a peeked byte, return it
  if (_peek_available) {
    return static_cast<int>(_peek_byte);
  }

  // Read one byte and store it in cache
  size_t bytes_read = 0;
  Status status     = readData(&_peek_byte, 1, bytes_read);
  if (status != Status::Success || bytes_read == 0) return -1;

  _peek_available = true;
  return static_cast<int>(_peek_byte);
}

void TCPClient::flush() {
  if (!_client_usable || _modem == nullptr) return;
  _modem->_serial->flush();
}

void TCPClient::stop() { disconnect(); }

uint8_t TCPClient::connected() { return _connected ? 1 : 0; }

TCPClient::operator bool() { return _connected; }

bool TCPClient::_handleTCPURCs(const char* line) {
  if (line == nullptr) return false;

  // Data available to read
  // Expected: +CIPRXGET: 1,<link_num>
  if (strstr(line, "+CIPRXGET: 1,") != nullptr) {
    uint8_t link_num;
    uint8_t parsed = sscanf(line, "+CIPRXGET: 1,%hhu", &link_num);
    if (parsed != 1) return false;

    SIM7600_LOGD(tag, "TCP URC Client %u: Data available", link_num);

    TCPClient* instance = _instances[link_num];
    if (instance == nullptr) return false;

    if (instance->_cb_data != nullptr) {
      instance->_cb_data(instance);
    }

    return true;
  }

  // Connection closed
  // Expected: +IPCLOSE: <link_num>,<close_reason>
  if (strstr(line, "+IPCLOSE: ") != nullptr) {
    uint8_t link_num;
    uint8_t close_reason;
    uint8_t parsed = sscanf(line, "+IPCLOSE: %hhu,%hhu", &link_num, &close_reason);
    if (parsed != 2) return false;

    SIM7600_LOGD(tag, "TCP URC Client %u: Connection closed, reason %u", link_num, close_reason);

    TCPClient* instance = _instances[link_num];
    if (instance == nullptr) return false;

    instance->_connected       = false;
    instance->_available_bytes = 0;
    instance->_peek_byte       = 0;
    instance->_peek_available  = false;

    if (instance->_cb_connection_closed != nullptr) {
      instance->_cb_connection_closed(instance, static_cast<TCPCloseReason>(close_reason));
    }

    return true;
  }

  return false;
}

} // namespace SIM7600